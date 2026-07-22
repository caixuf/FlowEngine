/**
 * perception_node.cpp — 感知节点插件 (FlowCoro 协程版)
 *
 * 从 perception_node.c 迁移而来，采用 FlowCoroTask 协程框架：
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
 * 采用 FlowCoroTask（线程池 resume）：节点做重计算（DBSCAN 点云聚类），同步 resume 会阻塞
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
    double  obs_x[16]{}, obs_y[16]{}, obs_vx[16]{};

    /* 发布帧计数 */
    uint32_t frame_id{0};

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
    double lidar_max_range_m{60.0};
    double obs_noise_std_m{0.08};
    int    enable_simple_occlusion{1};

    /* NOA Phase 2.1: 感知输入模式
     *   ground_truth (默认): 从 vehicle/state 读真值 ego+obstacles，向后兼容
     *   sensor: 额外消费 sensor/lidar 的 LidarFrame，用传感器测量的 ego 位置替代
     *           vehicle/state 真值定位（建立 sensor/lidar → perception 数据链路，
     *           见 NOA_SCENARIO_PLAN §2.3）。障碍物仍由 vehicle/state 经 FOV/噪声/
     *           遮挡滤波提供——sensor_model 目前发布的是定位级 LidarFrame（单点），
     *           障碍物级点云发布为后续工作。 */
    int mode{0};  /* 0 = ground_truth, 1 = sensor */
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
    if (!msg || !msg->data) return;
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
    int n = 0;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "n_obs")) && cJSON_IsNumber(j))
        n = (int)j->valuedouble;
    if (n < 1 || n > 16) n = 3;
    g.n_obs = n;
    for (int i = 0; i < n; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ox%d", i);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, key)) && cJSON_IsNumber(j))
            g.obs_x[i] = j->valuedouble;
        snprintf(key, sizeof(key), "oy%d", i);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, key)) && cJSON_IsNumber(j))
            g.obs_y[i] = j->valuedouble;
        snprintf(key, sizeof(key), "ov%d", i);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, key)) && cJSON_IsNumber(j))
            g.obs_vx[i] = j->valuedouble;
    }
    cJSON_Delete(root);
}

/* ── sensor/lidar 订阅（NOA Phase 2.1: sensor 模式） ──────────
 * 解析 sensor_model 发布的 LidarFrame 二进制消息，取其 (x,y) 作为传感器测量
 * 的 ego 位置。sensor 模式下用它替代 vehicle/state 的真值定位，建立真实的
 * sensor/lidar → perception 数据依赖。 */
static void on_sensor_lidar(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    /* 直接走 _msg_cast_impl：C++ 编译器会优先匹配 serializer.h 的 msg_cast 模板
     * (要求 T::TYPE_ID 形式)，故按 fusion_node.cpp 的写法显式调用底层实现。 */
    const LidarFrame* f = (const LidarFrame*)_msg_cast_impl(msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
    if (!f) return;
    g.lid_x = (double)f->x;
    g.lid_y = (double)f->y;
    g.has_lidar = 1;
}

/* ── 协程任务 ────────────────────────────────────────────────── */

class PerceptionTask : public FlowCoroTask {
public:
    PerceptionTask(MessageBus* bus, Transport* transport, int lidar_rate_hz)
        : FlowCoroTask(bus), transport_(transport),
          period_us_(1000000L / (lidar_rate_hz > 0 ? lidar_rate_hz : 20)) {}

protected:
    Task run() override {
        LOG_INFO("perception", "FlowCoro perception started (%ld us period)", period_us_);

        while (!should_stop()) {
            /* 替代 usleep：sleep_us 自动注入 cancel_token_，stop() 可立即唤醒 */
            co_await sleep_us(period_us_);
            if (should_stop()) break;

            /* ── DBSCAN ── */
            {
                Point3D pts[256];
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
                for (int ring = 0; ring < 2 && np < 256; ring++) {
                    float r = 6.0f + (float)ring * 4.0f;
                    for (int k = 0; k < 12 && np < 256; k++) {
                        float a = (float)k / 12.0f * 2.0f * (float)M_PI;
                        pts[np].x = cosf(a) * r; pts[np].y = sinf(a) * r;
                        pts[np].z = 0.05f; pts[np].intensity = 0.3f; np++;
                    }
                }
                /* 传感器可见障碍物（FOV/量程/简化遮挡） */
                double vis_rx[16], vis_ry[16], vis_r[16], vis_a[16];
                int vis_idx[16];
                int vis_count = 0;
                for (int oi = 0; oi < g.n_obs && vis_count < 16; oi++) {
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

                int vis_keep[16];
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
                for (int vi = 0; vi < vis_count && np < 256; vi++) {
                    if (!vis_keep[vi]) continue;
                    (void)vis_idx[vi];
                    double rx = vis_rx[vi];
                    double ry = vis_ry[vi];
                    for (int k = 0; k < 8 && np < 256; k++) {
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
                for (int ci = 0; ci < n_clusters && obs_list.count < 8; ci++) {
                    const ClusterBounds* cb = dbscan_get_cluster(&g.dbscan, ci);
                    if (!cb || cb->point_count < 3) continue;
                    Obstacle* ob = &obs_list.obstacles[obs_list.count++];
                    ob->id = (uint32_t)(g.frame_id * 100 + (uint32_t)ci);
                    ob->x = cb->cx; ob->y = cb->cy;
                    ob->width = cb->width; ob->length = cb->length;
                    ob->confidence = cb->confidence;
                    switch (cb->cls) {
                        case CLS_VEHICLE:    ob->type = OBJ_TYPE_VEHICLE;    break;
                        case CLS_PEDESTRIAN: ob->type = OBJ_TYPE_PEDESTRIAN; break;
                        default:             ob->type = OBJ_TYPE_UNKNOWN;    break;
                    }
                }
                g.last_obs_list = obs_list;
                g.has_last_obs  = 1;

                uint8_t obs_buf[280];
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
        g.task->execute();
    } catch (...) {
        LOG_ERROR("perception", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "vehicle/state", "sensor/lidar", nullptr };
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
    g.mode         = 0;       /* NOA Phase 2.1: 默认 ground_truth，向后兼容 */
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

    /* Fixed seed for reproducibility — sim_world drives deterministic time */
    srand(42u);
    dbscan_init(&g.dbscan, (float)g.dbscan_eps, g.dbscan_min_pts);
    dbscan_set_ransac(&g.dbscan, 100, 0.2f, 0.3f);

    transport_subscribe(transport, "vehicle/state", on_vehicle_state, nullptr);
    /* sensor 模式额外消费 sensor/lidar（ground_truth 模式下订阅无害，仅更新 has_lidar） */
    transport_subscribe(transport, "sensor/lidar", on_sensor_lidar, nullptr);

    discovery_advertise(discovery, "vehicle/state",         0x1C0E5A7Eu, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, "sensor/lidar",          LIDARFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "perception/obstacles",  0x0B5A010Eu, CAP_PUBLISHER, 20.0);

    transport_advertise(transport, "perception/obstacles", 0x0B5A010Eu);

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
    if (g.task) g.task->stop();
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
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
