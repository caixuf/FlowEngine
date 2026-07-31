/**
 * perception_node.cpp — 感知节点插件 (FlowCoro 协程版)
 *
 * 从 perception_node.c 迁移而来，采用 CoroutineTask 协程框架：
 *   - co_await sleep_us(period_us) 替代 usleep 定频轮询（可被 stop 取消）
 *   - 保留 on_vehicle_state 持久回调更新 ego 状态
 *   - DBSCAN 聚类逻辑原样搬入 run()
 *
 * 输入 topics: vehicle/state, sensor/lidar (NOA Phase 2.1: 真实传感器链路)
 * 输出 topics: perception/obstacles
 *
 * 算法:
 *   - DBSCAN 点云聚类 (dbscan_cluster.c) — eps=2m, min_pts=4
 *   - RANSAC 地面移除
 *   - 基于真值的障碍物聚类仿真
 *
 * 采用 CoroutineTask（线程池 resume）：节点做重计算（DBSCAN 点云聚类），同步 resume 会阻塞
 * 消息总线分发线程导致 drops，故改用线程池 resume。
 * flowcoro 核心库为 header-only（INTERFACE），子项目已 include 其头文件目录，
 * 故只需 FLOWCORO_INTEGRATION 定义 + -fcoroutines，无需额外链接 flowcoro 库。
 */

#include "node_plugin.h"
#include "dbscan_cluster.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "serializer.h"   /* NOA Phase 2.1: msg_cast 解析 sensor/lidar LidarFrame */
#include "coroutine_task.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#include "clock_service.h"

#include <cjson/cJSON.h>
#include <memory>

namespace {

/* ── 节点本地状态 ───────────────────────────────────────────── */

struct PerceptionContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};

    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* 仿真状态 (通过 vehicle/state topic 更新) */
    double  ego_x{0}, ego_y{0}, ego_heading{0};
    double  ego_speed{0};
    int     n_obs{0};
    /* 障碍物世界系位置/速度，从 flowsim vehicle/state 的 ox/oy/ov/ovy 字段接收。
     * 下游 planning/control/safety 经 ego_heading 旋回车体系做几何判断；
     * 速度方向直接用世界系（对向迎面 obs_vx<0 即可识别）。 */
    double  obs_x[128]{}, obs_y[128]{}, obs_vx[128]{}, obs_vy[128]{};
    /* 障碍物类型与尺寸（来自 vehicle/state 的 ot/ol/ow 字段）。
     * 类型必须逐个透传，不能硬编码 —— 行人与车的尺寸差 9 倍，
     * 下游的让行逻辑、碰撞判定、识别率分层全依赖它。 */
    uint8_t obs_type[128]{};
    double  obs_length[128]{}, obs_width[128]{};

    /* 发布帧计数 */
    uint32_t frame_id{0};

    /* 道路几何缓存（从 road/geometry 订阅，用于 lane_id 赋值） */
    int    lane_count{2};
    double lane_width{3.5};
    volatile int has_road_geometry{0};

    /* DBSCAN */
    DbscanCluster dbscan{};

    /* 上一帧有效结果（DBSCAN 超时时复用） */
    ObstacleList  last_obs_list{};
    int           has_last_obs{0};
    uint32_t      overrun_count{0};

    /* 配置参数 */
    double dbscan_eps{2.0};
    int    dbscan_min_pts{4};
    int    lidar_rate_hz{20};
    double lidar_fov_deg{120.0};
    double lidar_max_range_m{120.0};
    double obs_noise_std_m{0.08};
    int    enable_simple_occlusion{1};

    /* NOA Phase 2.1: 感知输入模式
     *   ground_truth (默认): 从 vehicle/state 读真值 ego+obstacles，向后兼容
     *   sensor: 额外消费 sensor/lidar 的 LidarFrame，用传感器测量的 ego 位置替代
     *           vehicle/state 真值定位（建立 sensor/lidar → perception 数据链路，
     *           见 NOA_SCENARIO_PLAN §2.3）。障碍物仍由 vehicle/state 经 FOV/噪声/
     *           遮挡滤波提供——sensor_model 目前发布的是定位级 LidarFrame（单点），
     *           障碍物级点云发布为后续工作。 */
    int mode{0};  /* 0 = ground_truth (default) — obstacles from vehicle/state JSON */
    double lid_x{0}, lid_y{0};
    volatile int has_lidar{0};

    /* 协程任务 */
    std::unique_ptr<class PerceptionTask> task;
};

PerceptionContext g;

static double rand_uniform_signed(double span) {
    return (((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * span;
}

static int obstacle_in_fov(double rx, double ry, double max_range_m, double fov_deg) {
    const double range = hypot(rx, ry);
    if (range > max_range_m || range < 0.05) return 0;
    const double half_fov_rad = (fov_deg * 0.5) * M_PI / 180.0;
    const double ang = atan2(ry, rx);
    return fabs(ang) <= half_fov_rad;
}

/* ── vehicle/state 订阅 ──────────────────────────────────────── */

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "hdg")) && cJSON_IsNumber(j))
        g.ego_heading = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "spd")) && cJSON_IsNumber(j))
        g.ego_speed = j->valuedouble;
    /* ground_truth 模式：从 vehicle/state JSON 读取障碍物（oxN/oyN/ovN/ovyN） */
    cJSON* n_obs_j = cJSON_GetObjectItemCaseSensitive(root, "n_obs");
    if (cJSON_IsNumber(n_obs_j)) {
        int no = (int)n_obs_j->valuedouble;
        if (no > 128) no = 128;
        for (int i = 0; i < no; i++) {
            char key[16];
            snprintf(key, sizeof(key), "ox%d", i);
            cJSON* jx = cJSON_GetObjectItemCaseSensitive(root, key);
            snprintf(key, sizeof(key), "oy%d", i);
            cJSON* jy = cJSON_GetObjectItemCaseSensitive(root, key);
            snprintf(key, sizeof(key), "ov%d", i);
            cJSON* jvx = cJSON_GetObjectItemCaseSensitive(root, key);
            snprintf(key, sizeof(key), "ovy%d", i);
            cJSON* jvy = cJSON_GetObjectItemCaseSensitive(root, key);
            /* ot/ol/ow：flowsim 一直在发（flowsim_node.cpp 的 ot%d/ol%d/ow%d），
             * 但此前无人解析 —— ground_truth 分支把所有障碍物硬编码成
             * OBJ_TYPE_VEHICLE，于是场景里的行人在整条链路上从不存在：
             * VRU 真值统计恒为 0 → 识别率分母为 0 → 报告打印"感知 100%"，
             * 而 safety_control 的行人让行逻辑一次都没被触发过。 */
            snprintf(key, sizeof(key), "ot%d", i);
            cJSON* jt = cJSON_GetObjectItemCaseSensitive(root, key);
            snprintf(key, sizeof(key), "ol%d", i);
            cJSON* jl = cJSON_GetObjectItemCaseSensitive(root, key);
            snprintf(key, sizeof(key), "ow%d", i);
            cJSON* jw = cJSON_GetObjectItemCaseSensitive(root, key);
            if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy)) {
                g.obs_x[i] = jx->valuedouble;
                g.obs_y[i] = jy->valuedouble;
                g.obs_vx[i]= cJSON_IsNumber(jvx) ? jvx->valuedouble : 0.0;
                g.obs_vy[i]= cJSON_IsNumber(jvy) ? jvy->valuedouble : 0.0;
                /* 默认按车处理（尺寸也用车的默认值），仅在明确标注时改写 */
                g.obs_type[i]   = OBJ_TYPE_VEHICLE;
                g.obs_length[i] = 4.6;
                g.obs_width[i]  = 2.0;
                if (cJSON_IsString(jt) && jt->valuestring) {
                    const char* t = jt->valuestring;
                    if (strcmp(t, "pedestrian") == 0)   g.obs_type[i] = OBJ_TYPE_PEDESTRIAN;
                    else if (strcmp(t, "cyclist") == 0) g.obs_type[i] = OBJ_TYPE_CYCLIST;
                }
                if (cJSON_IsNumber(jl)) g.obs_length[i] = jl->valuedouble;
                if (cJSON_IsNumber(jw)) g.obs_width[i]  = jw->valuedouble;
            }
        }
        g.n_obs = no;
    } else {
        g.n_obs = 0;
    }
    cJSON_Delete(root);
}

/* ── sensor/lidar 订阅（NOA Phase 2.1: sensor 模式） ──────────
 * 解析 sensor_model 发布的 LidarFrame 二进制消息，取其 (x,y) 作为传感器测量
 * 的 ego 位置。sensor 模式下用它替代 vehicle/state 的真值定位，建立真实的
 * sensor/lidar → perception 数据依赖。 */
static void on_sensor_lidar(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    /* 直接走 _msg_cast_impl：C++ 编译器会优先匹配 serializer.h 的 msg_cast 模板
     * (要求 T::TYPE_ID 形式)，故按 fusion_node.cpp 的写法显式调用底层实现。 */
    const LidarFrame* f = (const LidarFrame*)_msg_cast_impl(msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
    if (!f) return;
    g.lid_x = (double)f->x;
    g.lid_y = (double)f->y;
    g.has_lidar = 1;
}

/* ── road/geometry 订阅 — 获取车道参数（用于 lane_id 赋值） ──── */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "lane_count")) && cJSON_IsNumber(j))
        g.lane_count = (int)j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "lane_width")) && cJSON_IsNumber(j))
        g.lane_width = j->valuedouble;
    g.has_road_geometry = 1;
    cJSON_Delete(root);
}

/* ── 协程任务 ────────────────────────────────────────────────── */

class PerceptionTask : public CoroutineTask {
public:
    PerceptionTask(MessageBus* bus, Transport* transport, int lidar_rate_hz)
        : CoroutineTask(bus), transport_(transport),
          period_us_(1000000L / (lidar_rate_hz > 0 ? lidar_rate_hz : 20)) {}

protected:
    Task run() override {
        LOG_INFO("perception", "FlowCoro perception started (%ld us period)", period_us_);

        while (!should_stop()) {
            /* 替代 usleep：sleep_us 自动注入 cancel_token_，stop() 可立即唤醒 */
            co_await sleep_us(period_us_);
            if (should_stop()) break;

            /* ALGORITHM_REFACTOR_PLAN §4: sensor 模式无 lidar 时跳过 */
            if (g.mode == 1 && !g.has_lidar) {
                g.frame_id++;
                continue;
            }

            /* ── DBSCAN ── */
            {
                Point3D pts[512];
                int np = 0;
                double ch = cos(-g.ego_heading), sh = sin(-g.ego_heading);

                /* NOA Phase 2.1: sensor 模式下用 sensor/lidar 测量的 ego 位置作为
                 * 障碍物相对坐标的参考原点（含传感器噪声），ground_truth 模式仍用
                 * vehicle/state 真值定位。 */
                double ego_ref_x = g.ego_x, ego_ref_y = g.ego_y;
                if (g.mode == 1 && g.has_lidar) {
                    ego_ref_x = g.lid_x;
                    ego_ref_y = g.lid_y;
                }

                /* 地面环 */
                for (int ring = 0; ring < 2 && np < 512; ring++) {
                    float r = 6.0f + (float)ring * 4.0f;
                    for (int k = 0; k < 12 && np < 512; k++) {
                        float a = (float)k / 12.0f * 2.0f * (float)M_PI;
                        pts[np].x = cosf(a) * r; pts[np].y = sinf(a) * r;
                        pts[np].z = 0.05f; pts[np].intensity = 0.3f; np++;
                    }
                }
                /* 传感器可见障碍物（FOV/量程/简化遮挡） */
                double vis_rx[128], vis_ry[128], vis_r[128], vis_a[128];
                int vis_idx[128];
                int vis_count = 0;
                for (int oi = 0; oi < g.n_obs && vis_count < 128; oi++) {
                    double dx = g.obs_x[oi] - ego_ref_x;
                    double dy = g.obs_y[oi] - ego_ref_y;
                    double rx = dx * ch - dy * sh;
                    double ry = dx * sh + dy * ch;
                    if (!obstacle_in_fov(rx, ry, g.lidar_max_range_m, g.lidar_fov_deg)) {
                        continue;
                    }
                    vis_rx[vis_count] = rx;
                    vis_ry[vis_count] = ry;
                    vis_r[vis_count] = hypot(rx, ry);
                    vis_a[vis_count] = atan2(ry, rx);
                    vis_idx[vis_count] = oi;
                    vis_count++;
                }

                int vis_keep[128];
                for (int i = 0; i < vis_count; i++) vis_keep[i] = 1;
                if (g.enable_simple_occlusion) {
                    const double occ_beam = 5.0 * M_PI / 180.0;
                    for (int i = 0; i < vis_count; i++) {
                        if (!vis_keep[i]) continue;
                        for (int j = 0; j < vis_count; j++) {
                            if (i == j || !vis_keep[j]) continue;
                            if (fabs(vis_a[i] - vis_a[j]) < occ_beam && vis_r[j] + 2.0 < vis_r[i]) {
                                vis_keep[i] = 0;
                                break;
                            }
                        }
                    }
                }

                /* 障碍物表面点 */
                for (int vi = 0; vi < vis_count && np < 512; vi++) {
                    if (!vis_keep[vi]) continue;
                    (void)vis_idx[vi];
                    double rx = vis_rx[vi];
                    double ry = vis_ry[vi];
                    for (int k = 0; k < 8 && np < 512; k++) {
                        pts[np].x = (float)rx + ((float)(k % 3) - 1.0f) * 0.8f + (float)rand_uniform_signed(g.obs_noise_std_m);
                        pts[np].y = (float)ry + ((float)(k / 3) - 1.0f) * 1.6f + (float)rand_uniform_signed(g.obs_noise_std_m);
                        pts[np].z = 0.6f + (float)(k % 4) * 0.4f;
                        pts[np].intensity = 0.7f; np++;
                    }
                }

                /* ── DBSCAN 时间预算保护 ── */
                uint64_t t_dbscan_start = clock_now_us();

                int n_clusters = dbscan_run(&g.dbscan, pts, np);

                uint64_t t_dbscan_end = clock_now_us();
                long dbscan_us = (long)(t_dbscan_end - t_dbscan_start);
                long budget_warn_us = (long)(period_us_ * 8 / 10);

                if (dbscan_us > period_us_) {
                    g.overrun_count++;
                    LOG_WARN("perception",
                             "DBSCAN overrun #%u: %ldus > period %ldus (pts=%d) — reusing last frame",
                             g.overrun_count, dbscan_us, period_us_, np);
                    g.frame_id++;
                    continue;
                } else if (dbscan_us > budget_warn_us) {
                    LOG_WARN("perception",
                             "DBSCAN budget warning: %ldus > 80%% of period %ldus (pts=%d)",
                             dbscan_us, period_us_, np);
                }

                ObstacleList obs_list;
                memset(&obs_list, 0, sizeof(obs_list));
                obs_list.frame_id = g.frame_id;

                /* ground_truth 模式：直接使用 vehicle/state 中的障碍物数据 */
                if (g.mode == 0 && g.n_obs > 0) {
                    int lc = g.has_road_geometry ? g.lane_count : 2;
                    double lw = g.has_road_geometry ? g.lane_width : 3.5;
                    for (int i = 0; i < g.n_obs && obs_list.count < 128; i++) {
                        Obstacle* ob = &obs_list.obstacles[obs_list.count++];
                        ob->id = (uint32_t)(g.frame_id * 100 + (uint32_t)i);
                        /* 世界坐标 → 车体坐标（Obstacle 约定车体系） */
                        double dx = g.obs_x[i] - g.ego_x;
                        double dy = g.obs_y[i] - g.ego_y;
                        double ch = cos(g.ego_heading), sh = sin(g.ego_heading);
                        ob->x = (float)(dx * ch + dy * sh);
                        ob->y = (float)(-dx * sh + dy * ch);
                        ob->vx = (float)(g.obs_vx[i] * ch + g.obs_vy[i] * sh);
                        ob->vy = (float)(-g.obs_vx[i] * sh + g.obs_vy[i] * ch);
                        ob->type = (ObstacleType)g.obs_type[i];
                        ob->width = (float)g.obs_width[i];
                        ob->length = (float)g.obs_length[i];
                        ob->confidence = 1.0f;
                        /* lane_id：从世界系 y 计算 */
                        double offset = (-g.obs_y[i]) / lw + (lc - 1) * 0.5;
                        int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
                        if (idx < 0) idx = 0;
                        if (idx >= lc) idx = lc - 1;
                        ob->lane_id = (int8_t)idx;
                    }
                } else {
                    /* sensor 模式（DBSCAN） */
                    for (int ci = 0; ci < n_clusters && obs_list.count < 128; ci++) {
                    const ClusterBounds* cb = dbscan_get_cluster(&g.dbscan, ci);
                    if (!cb || cb->point_count < 3) continue;
                    Obstacle* ob = &obs_list.obstacles[obs_list.count++];
                    ob->id = (uint32_t)(g.frame_id * 100 + (uint32_t)ci);
                    ob->x = cb->cx; ob->y = cb->cy;
                    ob->width = cb->width; ob->length = cb->length;
                    ob->confidence = cb->confidence;
                    /* lane_id：车体系 cx/cy → 世界系 y → 车道索引 */
                    {
                        double ch = cos(g.ego_heading), sh = sin(g.ego_heading);
                        double wy = g.ego_y + cb->cx * sh + cb->cy * ch;
                        int lc = g.has_road_geometry ? g.lane_count : 2;
                        double lw = g.has_road_geometry ? g.lane_width : 3.5;
                        /* lane_idx_from_y 公式：最左 = 0, 最右 = N-1 */
                        double offset = (-wy) / lw + (lc - 1) * 0.5;
                        int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
                        if (idx < 0) idx = 0;
                        if (idx >= lc) idx = lc - 1;
                        ob->lane_id = (int8_t)idx;
                    }
                    switch (cb->cls) {
                        case CLS_VEHICLE:    ob->type = OBJ_TYPE_VEHICLE;    break;
                        case CLS_PEDESTRIAN: ob->type = OBJ_TYPE_PEDESTRIAN; break;
                        default:             ob->type = OBJ_TYPE_UNKNOWN;    break;
                    }
                    /* 速度由 object_tracker 节点通过 KF 跟踪提供。
                     * ALGORITHM_REFACTOR_PLAN §4: 移除 ground truth 速度匹配，
                     * 速度在 object_tracker 的 KF 中跨帧关联得到。 */
                    }
                }  /* closes sensor mode else block */
                g.last_obs_list = obs_list;
                g.has_last_obs  = 1;

                uint8_t obs_buf[4368];  /* ObstacleList 序列化大小 = 16 + 128*34 */
                size_t obs_len = 0;
                if (ObstacleList_serialize(&obs_list, obs_buf, &obs_len) == 0 && obs_len > 0) {
                    transport_publish(transport_, "perception/obstacles", obs_buf, (uint32_t)obs_len);
                }
            }

            g.frame_id++;
        }

        LOG_INFO("perception", "FlowCoro perception stopped (%u frames)", g.frame_id);
    }

private:
    Transport* transport_;
    long       period_us_;
};

/* ── 协程宿主线程 ─────────────────────────────────────────────── */

void* perception_thread(void*) {
    pthread_setname_np(pthread_self(), "perception");
    try {
        flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1 }};
        g_node_exec = &ex;
        CoroutineTask& ct = *g.task;
        ex.spawn(ct.run(), "perception");
        node_pump(ex, [] { return (bool)g.should_stop; });
        ex.shutdown();
        g_node_exec = nullptr;
    } catch (...) {
        LOG_ERROR("perception", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "vehicle/state", "sensor/lidar", "road/geometry", nullptr };
static const char* s_outputs[] = { "perception/obstacles", nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾 */

static int perception_init(MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler,
                           const char* params_json) {
    (void)scheduler;

    /* 清零并重新初始化（atomic 不可拷贝，逐字段赋值） */
    g.ego_x = g.ego_y = g.ego_heading = g.ego_speed = 0.0;
    g.n_obs = 0;
    g.frame_id = 0;
    g.has_last_obs = 0;
    g.overrun_count = 0;
    g.dbscan_eps = 2.0;
    g.dbscan_min_pts = 4;
    g.lidar_rate_hz = 20;
    g.lidar_fov_deg = 120.0;
    g.lidar_max_range_m = 60.0;
    g.obs_noise_std_m = 0.08;
    g.enable_simple_occlusion = 1;
    g.mode         = 0;       /* 默认 ground_truth 模式 — 从 vehicle/state 读障碍物 */
    g.has_lidar    = 0;
    g.lid_x = g.lid_y = 0.0;
    g.transport    = transport;
    g.discovery    = discovery;
    g.should_stop  = false;

    /* 解析参数 */
    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "dbscan_eps")) && cJSON_IsNumber(j))
                g.dbscan_eps = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lidar_rate_hz")) && cJSON_IsNumber(j))
                g.lidar_rate_hz = (int)j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lidar_fov_deg")) && cJSON_IsNumber(j))
                g.lidar_fov_deg = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lidar_max_range_m")) && cJSON_IsNumber(j))
                g.lidar_max_range_m = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "obs_noise_std_m")) && cJSON_IsNumber(j))
                g.obs_noise_std_m = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "enable_simple_occlusion")) && cJSON_IsNumber(j))
                g.enable_simple_occlusion = (int)j->valuedouble;
            /* NOA Phase 2.1: mode = "sensor" | "ground_truth" (默认) */
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "mode")) && cJSON_IsString(j) &&
                j->valuestring && strcmp(j->valuestring, "sensor") == 0) {
                g.mode = 1;
            }
            cJSON_Delete(p);
        }
    }

    /* Fixed seed for reproducibility — flowsim_node drives deterministic time */
    srand(42u);
    dbscan_init(&g.dbscan, (float)g.dbscan_eps, g.dbscan_min_pts);
    dbscan_set_ransac(&g.dbscan, 100, 0.2f, 0.3f);

    transport_subscribe(transport, "vehicle/state", on_vehicle_state, nullptr);
    /* sensor 模式额外消费 sensor/lidar（ground_truth 模式下订阅无害，仅更新 has_lidar） */
    transport_subscribe(transport, "sensor/lidar", on_sensor_lidar, nullptr);
    /* 订阅 road/geometry 获取车道参数（用于 Obstacle.lane_id 赋值） */
    transport_subscribe(transport, "road/geometry", on_road_geometry, nullptr);

    discovery_advertise(discovery, "vehicle/state",         0x1C0E5A7Eu, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, "sensor/lidar",          LIDARFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/geometry",         0x80AD5C12u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, "perception/obstacles",  OBSTACLELIST_TYPE_ID, CAP_PUBLISHER, 20.0);

    transport_advertise(transport, "perception/obstacles", OBSTACLELIST_TYPE_ID);

    g.task = std::make_unique<PerceptionTask>(bus, transport, g.lidar_rate_hz);

    LOG_INFO("perception", "initialized (FlowCoro, mode=%s, DBSCAN eps=%.1f, LiDAR %dHz FOV=%.0fdeg range=%.0fm noise=%.2f occ=%d)",
             g.mode == 1 ? "sensor" : "ground_truth",
             g.dbscan_eps, g.lidar_rate_hz, g.lidar_fov_deg, g.lidar_max_range_m,
             g.obs_noise_std_m, g.enable_simple_occlusion);
    return 0;
}

static int perception_start(void) {
    if (!g.task) return -1;
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, perception_thread, nullptr) != 0) {
        LOG_WARN("perception", "pthread_create failed: %s", strerror(errno));
        return -1;
    }
    g.running = true;
    LOG_INFO("perception", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void perception_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->set_stop();
}

static void perception_cleanup(void) {
    perception_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
    LOG_INFO("perception", "cleanup done");
}

static int perception_health(void) { return 0; }

/* ── 导出入口 ────────────────────────────────────────────────── */

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "perception",
    "1.0.0",
    "LiDAR/GPS/Camera simulation + DBSCAN clustering [FlowCoro]",
    s_inputs,
    s_outputs,
    perception_init,
    perception_start,
    perception_stop,
    perception_cleanup,
    perception_health,
    nullptr,  /* taskbase: 旧路径自管线程 */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
