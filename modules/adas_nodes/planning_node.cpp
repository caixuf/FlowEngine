/**
 * planning_node.cpp — Frenet 轨迹规划节点插件 (FlowCoro 协程版)
 *
 * 从 planning_node.c 迁移而来，采用 FlowCoroTask 协程框架：
 *   - co_await sleep_us(50000) 替代 usleep 20Hz 轮询（可被 stop 取消）
 *   - 保留 on_fusion / on_vehicle_state / on_road_geometry 持久回调
 *   - 驾驶模式状态机 + 路线变道 + Frenet 规划逻辑原样搬入 run()
 *
 * 订阅 fusion/localization → 发布 planning/trajectory
 *
 * NodePlugin 接口，编译为 libplanning_node.so。
 *
 * 采用 FlowCoroTask（线程池 resume）：节点做重计算（Frenet 轨迹规划），同步 resume 会阻塞
 * 消息总线分发线程导致 drops，故改用线程池 resume。
 * flowcoro 核心库为 header-only（INTERFACE），子项目已 include 其头文件目录，
 * 故只需 FLOWCORO_INTEGRATION 定义 + -fcoroutines，无需额外链接 flowcoro 库。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "scenario_loader.h"
#include "road_geometry.h"
#include "traffic_light.h"
/* adas_msgs_gen.h previously included for Localization_deserialize — removed in cJSON-only cleanup */
#include "coroutine_task.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#ifdef HAVE_FRENET
#include "frenet_bridge.h"
#endif

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include <memory>
#include <atomic>

namespace {

/* ── 节点本地状态 ───────────────────────────────────────────── */

struct PlanningContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler*        scheduler{nullptr};

    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* 反射式状态机：跟踪生命周期 */
    ReflectiveStateMachine sm{};

    /* 驾驶模式状态机（NA/ACC/CP/NP/NOA）：系统级功能仲裁, 由感知/定位/
     * 路况条件驱动升降级, 是 NOA 相对 LCC 的核心能力入口。 */
    ReflectiveStateMachine mode_sm{};
    uint64_t mode_last_check_us{0};
    uint64_t last_fusion_us{0};      /* fusion 消息到达的单调时间戳（模式降级用） */
    int      highway_ready{0};       /* ego_v 持续高于阈值一段时间 -> 视为高速工况 */
    double   highway_speed_timer{0.0};

    /* 从场景文件加载的导航路线（可选, 仅当 params 提供 scenario_file 时有效） */
    char              scenario_file[256]{};
    ScenarioRouteStep route[SCENARIO_MAX_ROUTE_STEPS];
    int               route_count{0};
    int               route_next_idx{0};    /* 下一条待触发的路线指令 */
    int               route_target_lane{0}; /* 当前路线要求的目标车道: 0=无, -1=y<0一侧, +1=y>0一侧 */
    double            route_target_speed{0.0};/* 当前路线步骤要求的目标速度（0 = 无改变） */
    /* NOA Phase 3.2/3.3: 当前路线步骤类型 + branch_select 选中的 connecting_road id。
     * - route_type 持续下发到 control/monitor，供下游区分普通变道/分支选路/汇入。
     * - current_branch_id 用于 branch_select 后参考路径切换的几何参考（fallback 模式
     *   下无 esmini，仅作日志/可视化标识；HAVE_FRENET 模式下未来用于切换参考路径）。 */
    RouteStepType     route_type{ROUTE_LANE_CHANGE};
    int               current_branch_id{-1};

    /* NOA Phase 6 merge 闭环：从 scene/frame entities 采样主线来车，算前后 gap
     * 决策并入时机。entities 是世界坐标无 segment_id，用"前方同向行驶 + 主路 y 范围"
     * 近似筛选主线车辆。
     *
     * merge_state: 0=未在 merge, 1=等待 gap(跟车巡航), 2=gap 充足已下发并入
     * merge_gap_front/rear: 主路前后最近同向车的纵向间距(m)，1e9 表示无车
     * merge_min_gap: 触发并入所需最小前 gap(m)，默认 4s 时距对应距离
     * merge_hold_lane: gap 不足时暂保持的车道（不把 route_target_lane 下发给 control） */
    int    merge_state{0};
    double merge_gap_front{1e9};
    double merge_gap_rear{1e9};
    double merge_min_gap{30.0};
    int    merge_hold_lane{0};
    volatile int has_scene_frame{0};

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    FrenetHandle* frenet{nullptr};
#else
    void*         frenet{nullptr};  /* unused stub */
#endif
    int           plan_count{0};
    double        target_speed{0.0};

    /* 从 fusion/localization 解析的最新状态 */
    double ego_x{0}, ego_y{0}, ego_v{0}, ego_heading{0};
    volatile int has_fusion{0};

    /* 从 vehicle/state 解析的障碍物位置（世界坐标） */
    double obs_x[4]{}, obs_y[4]{}, obs_vx[4]{}, obs_vy[4]{};  /* Phase 3: +vy */
    volatile int has_vstate{0};

    /* 配置参数 */
    double cfg_target_speed{15.0};
    double cfg_max_speed{20.0};
    double cfg_max_accel{4.0};
    double cfg_ref_path_length{5000.0};
    double ref_path_start_x{0.0};
    double cfg_highway_speed_mps{13.0}; /* CP->NP 升级所需的持续速度阈值 (m/s) */

    /* 道路几何（可选弯道，来自场景文件 "road"；全零 = 直道，行为不变） */
    double curve_start_x{0};
    double curve_length_m{0};
    double curve_offset_m{0};

    /* 红绿灯状态缓存（从 road/traffic_lights topic 获取，sim_world 发布）。
     * 缓存前方最近的红/黄灯，用于在 Frenet 障碍物数组中注入虚拟停止线墙。
     * 红灯/黄灯时注入一面跨车道宽度的静止"墙"，绿灯时不注入——
     * safety_control 现有的 TTC/brake 逻辑直接对虚拟墙生效，无需改安全层。 */
#define TL_CACHE_MAX 4
    double tl_x[TL_CACHE_MAX];         /* 停止线 x（世界坐标） */
    double tl_y_lane[TL_CACHE_MAX];    /* 灯所在车道 y */
    int    tl_state[TL_CACHE_MAX];     /* 0=green 1=yellow 2=red */
    int    tl_count{0};
    volatile int has_traffic_lights{0};

    int tid{0};  /* scheduler task id */

    /* 协程任务 */
    std::unique_ptr<class PlanningTask> task;
};

PlanningContext g;

/* 障碍物过滤与传递给 Frenet 规划器的空间范围 */
#define OBS_MIN_DX_M      (-10.0)   /* 忽略已经落后 ego 超过此距离的障碍物 */
#define OBS_MAX_DX_M      120.0     /* 忽略前方超过此距离的障碍物 */
#define OBS_MAX_ABS_Y_M     6.0     /* 忽略横向距离超出道路范围的障碍物 */
#define OBSTACLE_WIDTH_M    2.0     /* 默认障碍物宽度 (m) */
#define OBSTACLE_LENGTH_M   4.6     /* 默认障碍物长度 (m) */

/* 驾驶模式仲裁常量 */
#define MODE_CHECK_INTERVAL_US   1000000ULL  /* 每 1s 检查一次模式升降级条件 */
#define FUSION_STALE_TIMEOUT_US  1500000ULL  /* 定位超过此时长未更新 -> 判定条件丢失 */
#define HIGHWAY_SPEED_HOLD_S     3.0          /* 速度需持续高于阈值这么久才算"高速工况" */

/**
 * 驾驶模式转移守卫：把真实感知/定位/路况条件接到状态机上，
 * 而不是无条件放行——这是把"演示用状态机"变成"真正的模式仲裁器"的关键。
 *
 *   NA  -> ACC : 定位与车辆状态均已上线
 *   ACC -> CP  : 定位持续有效（车道居中所需的姿态/位置可信）
 *   CP  -> NP  : 达到并保持高速工况（导航加速辅助的 ODD 之一）
 *   NP  -> NOA : 已加载导航路线（等效 HD 地图/路线规划可用）
 * 其余转移（降级/故障/退出）默认放行。
 */
static bool mode_transition_guard(void* task, StateId from, EventId event, StateId to) {
    (void)task; (void)from;
    StateId to_mode = SM_MODE_OF(to);
    if (event == SM_EVT_CONDITIONS_MET) {
        return g.has_fusion && g.has_vstate;
    }
    if (event == SM_EVT_MODE_UPGRADE) {
        switch (to_mode) {
            case SM_MODE_CP:  return g.has_fusion != 0;
            case SM_MODE_NP:
                /* 已加载导航路线时放行 NP（路线步骤本身定义驾驶策略，
                 * 不应被 highway_ready 卡住——城市段也有路线指令如减速/
                 * 过红绿灯/匝道汇入）。无路线时仍保持高速工况守卫。 */
                if (g.route_count > 0) return true;
                return g.highway_ready != 0;
            case SM_MODE_NOA: return g.route_count > 0;
            default: return true;
        }
    }
    return true;
}

/** 每个控制周期尝试驱动模式向上一级演进（受 guard 约束，条件不满足时静默保持）。 */
static void try_mode_progress(void) {
    StateId cur  = statem_current(&g.mode_sm);
    StateId mode = SM_MODE_OF(cur);
    EventId ev   = (mode == SM_MODE_NA) ? SM_EVT_CONDITIONS_MET : SM_EVT_MODE_UPGRADE;

    if (statem_send_event(&g.mode_sm, ev, NULL)) {
        char buf[32];
        statem_format_hierarchical(statem_current(&g.mode_sm), buf, sizeof(buf));
        LOG_INFO("planning", "driving mode -> %s", buf);
    }
}

/** 定位/车辆状态长时间未更新 -> 条件丢失，模式退回 NA（安全兜底）。 */
static void check_mode_downgrade(uint64_t now_us) {
    if (SM_MODE_OF(statem_current(&g.mode_sm)) == SM_MODE_NA) return;
    if (g.last_fusion_us != 0 && now_us - g.last_fusion_us > FUSION_STALE_TIMEOUT_US) {
        if (statem_send_event(&g.mode_sm, SM_EVT_CONDITIONS_LOST, NULL)) {
            LOG_WARN("planning", "fusion stale (%.1fs) -> driving mode downgraded to NA",
                     (double)(now_us - g.last_fusion_us) / 1e6);
        }
    }
}

static void update_reference_path(double start_x) {
#ifdef HAVE_FRENET
    double wx[101], wy[101];
    const int ref_n = 101;
    for (int i = 0; i < ref_n; i++) {
        wx[i] = start_x + (double)i * (g.cfg_ref_path_length / (double)(ref_n - 1));
        /* 弯道时参考路径跟随道路中心线；curve_length_m<=0 时 road_center_y()
         * 恒为 0，与既有直线参考路径完全一致。 */
        wy[i] = -1.75 + road_center_y(wx[i], g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    }
    frenet_set_reference_path(g.frenet, wx, wy, ref_n);
    g.ref_path_start_x = start_x;
#else
    (void)start_x;
#endif
}

/* ── fusion/localization 订阅回调 ───────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* cJSON parsing (fusion/localization now publishes cJSON) */
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* item;
        item = cJSON_GetObjectItem(root, "x");
        if (cJSON_IsNumber(item))       g.ego_x = item->valuedouble;
        item = cJSON_GetObjectItem(root, "y");
        if (cJSON_IsNumber(item))       g.ego_y = item->valuedouble;
        item = cJSON_GetObjectItem(root, "v");
        if (cJSON_IsNumber(item))       g.ego_v = item->valuedouble;
        item = cJSON_GetObjectItem(root, "heading");
        if (cJSON_IsNumber(item))       g.ego_heading = item->valuedouble;
        cJSON_Delete(root);
    }
    g.has_fusion = 1;
    g.last_fusion_us = clock_now_us();
}

/* ── vehicle/state 订阅 — 解析障碍物位置（世界坐标） ─────────── */

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        for (int i = 0; i < 4; i++) {
            char key[16];
            snprintf(key, sizeof(key), "ox%d", i);
            cJSON* item = cJSON_GetObjectItem(root, key);
            if (item) g.obs_x[i] = item->valuedouble;
            snprintf(key, sizeof(key), "oy%d", i);
            item = cJSON_GetObjectItem(root, key);
            if (item) g.obs_y[i] = item->valuedouble;
            snprintf(key, sizeof(key), "ov%d", i);
            item = cJSON_GetObjectItem(root, key);
            if (item) g.obs_vx[i] = item->valuedouble;
            snprintf(key, sizeof(key), "ovy%d", i);
            item = cJSON_GetObjectItem(root, key);
            if (item) g.obs_vy[i] = item->valuedouble;
        }
        cJSON_Delete(root);
    }
    g.has_vstate = 1;
}

/* ── road/geometry 订阅回调（Phase 2 统一道路几何） ─────────── */
/* 从 sim_world 发布的 road/geometry topic 获取弯道参数，
 * 替代此前从 scenario_load() 读取弯道的冗余方式。
 * NOA route steps 仍从场景文件读取（planning 独有需求）。 */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* item;
        if ((item = cJSON_GetObjectItem(root, "curve_start_x")))  g.curve_start_x = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "curve_length_m"))) g.curve_length_m = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "curve_offset_m"))) g.curve_offset_m = item->valuedouble;
        cJSON_Delete(root);
    }
}

/* ── scene/frame 订阅回调（NOA Phase 6 merge 闭环） ─────────── */
/* 从 flowsim_node 发布的 scene/frame 提取主线来车，缓存前后 gap 供 merge 决策。
 *
 * entities 是世界坐标(x/y/vx/vy)，无 segment_id 字段。主线筛选启发式：
 *   - 类型为 car/suv/truck（排除 ego/pedestrian/tl/etc_gate/stop_line）
 *   - 同向行驶：vx > 2 m/s（主路车流方向，与 ego 一致）
 *   - 主路 y 范围：|y - ego_y| < 6m（粗略排除对向车道和匝道车）
 *
 * gap 计算：相对 ego 的纵向 dx = ent.x - ego_x
 *   - 前方车：dx > 0，取最近一辆的 dx 为 merge_gap_front
 *   - 后方车：dx < 0，取最近一辆的 |dx| 为 merge_gap_rear
 * 同时考虑相对速度：前车比 ego 慢则 gap 会缩小，用 TTC 加权 min_gap。 */
static void on_scene_frame(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    if (g.route_type != ROUTE_MERGE) {
        /* 非 merge 状态无需每帧解析 entities（省 CPU） */
        g.has_scene_frame = 1;
        return;
    }
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* entities = cJSON_GetObjectItem(root, "entities");
    if (!entities || !cJSON_IsArray(entities)) { cJSON_Delete(root); return; }

    double gap_front = 1e9, gap_rear = 1e9;
    double ego_x = g.ego_x, ego_y = g.ego_y;
    const double MAINLINE_Y_TOL = 6.0;   /* 主路 y 容差(m) */
    const double SAMEDIR_VX_MIN = 2.0;   /* 同向最低速度(m/s) */

    cJSON* ent;
    cJSON_ArrayForEach(ent, entities) {
        cJSON* jtype = cJSON_GetObjectItem(ent, "type");
        if (!jtype || !cJSON_IsString(jtype)) continue;
        const char* t = jtype->valuestring;
        if (strcmp(t, "car") != 0 && strcmp(t, "suv") != 0 && strcmp(t, "truck") != 0) continue;

        cJSON* jx = cJSON_GetObjectItem(ent, "x");
        cJSON* jy = cJSON_GetObjectItem(ent, "y");
        cJSON* jvx = cJSON_GetObjectItem(ent, "vx");
        if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jy) || !cJSON_IsNumber(jvx)) continue;

        double ex = jx->valuedouble, ey = jy->valuedouble, evx = jvx->valuedouble;
        if (evx < SAMEDIR_VX_MIN) continue;                 /* 仅同向来车 */
        if (fabs(ey - ego_y) > MAINLINE_Y_TOL) continue;    /* 仅主路 y 范围 */

        double dx = ex - ego_x;
        if (dx > 0.0 && dx < gap_front) gap_front = dx;
        else if (dx < 0.0 && -dx < gap_rear) gap_rear = -dx;
    }
    cJSON_Delete(root);

    g.merge_gap_front = gap_front;
    g.merge_gap_rear  = gap_rear;
    g.has_scene_frame = 1;
}

/* ── road/traffic_lights 订阅回调（Phase 2 红绿灯） ────────── */
/* 从 sim_world 发布的 road/traffic_lights topic 获取红绿灯状态。
 * JSON 格式: {"lights":[{"id":0,"x":100.0,"y_lane":-1.75,
 *                         "state":"red","remain_s":12.3},...]}
 * 解析每个灯的 x(停止线位置)、y_lane(车道)、state(green/yellow/red)。
 * 缓存到 g.tl_x/tl_y_lane/tl_state，供 Frenet 障碍物注入逻辑读取。 */
static void on_traffic_lights(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* lights = cJSON_GetObjectItem(root, "lights");
        if (lights && cJSON_IsArray(lights)) {
            int n = 0;
            cJSON* light;
            cJSON_ArrayForEach(light, lights) {
                if (n >= TL_CACHE_MAX) break;
                cJSON* item;
                g.tl_x[n] = 0.0;
                g.tl_y_lane[n] = -1.75;
                g.tl_state[n] = 0; /* default green */
                if ((item = cJSON_GetObjectItem(light, "x")))       g.tl_x[n] = item->valuedouble;
                if ((item = cJSON_GetObjectItem(light, "y_lane")))  g.tl_y_lane[n] = item->valuedouble;
                if ((item = cJSON_GetObjectItem(light, "state")) && cJSON_IsString(item)) {
                    if (strcmp(item->valuestring, "red") == 0)    g.tl_state[n] = 2;
                    else if (strcmp(item->valuestring, "yellow") == 0) g.tl_state[n] = 1;
                }
                n++;
            }
            g.tl_count = n;
            g.has_traffic_lights = (n > 0) ? 1 : 0;
        } else {
            g.tl_count = 0;
            g.has_traffic_lights = 0;
        }
        cJSON_Delete(root);
    } else {
        g.tl_count = 0;
        g.has_traffic_lights = 0;
    }
}

/* ── 协程任务 ────────────────────────────────────────────────── */

class PlanningTask : public FlowCoroTask {
public:
    PlanningTask(MessageBus* bus, Transport* transport)
        : FlowCoroTask(bus), transport_(transport) {}

protected:
    Task run() override {
        pthread_setname_np(pthread_self(), "planning");

        /* 参考路径: 长直线（左车道中心 y=-1.75）。运行时间较长时，ego 会开出
         * 初始路径范围；接近末端时向前滑动 reference path，避免 Frenet 插值越界
         * 导致 planning 线程挂掉。 */
        update_reference_path(0.0);

        while (!should_stop()) {
            /* 替代 usleep：sleep_us 自动注入 cancel_token_，stop() 可立即唤醒 */
            co_await sleep_us(50000);  /* 20Hz 检查 */
            if (should_stop() || !g.has_fusion) continue;

            /* ── 驾驶模式仲裁：周期性检查条件，尝试升级；定位丢失时立即降级 ── */
            uint64_t now_us = clock_now_us();
            g.highway_ready = (g.ego_v >= g.cfg_highway_speed_mps) &&
                               (g.highway_speed_timer >= HIGHWAY_SPEED_HOLD_S);
            if (g.ego_v >= g.cfg_highway_speed_mps) g.highway_speed_timer += 0.05;
            else                                    g.highway_speed_timer = 0.0;

            check_mode_downgrade(now_us);
            if (now_us - g.mode_last_check_us >= MODE_CHECK_INTERVAL_US) {
                try_mode_progress();
                g.mode_last_check_us = now_us;
            }

            /* ── 路线驱动的主动变道（仅 NOA 生效）：不依赖障碍物，由导航路线
             * 提前决定车道，是 NOA 区别于被动跟车/超车（LCC/NP）的核心行为。
             * route_target_lane 一经触发即持续下发，直到下一条路线指令覆盖它，
             * 供 control 节点在其既有安全变道状态机上执行（rear/front gap 检查）。
             * 若路线步骤指定了 target_speed，则同时调整巡航速度（用于出口匝道减速等）。
             *
             * NOA Phase 3.2/3.3: 按 step->type 分发：
             *   lane_change   — 同路段变道（原行为），设 target_lane + target_speed
             *   branch_select — 路口分叉选路，记录 branch_id，触发参考路径刷新，
             *                   target_lane 缺省 -1（靠右准备出匝道，符合 json_to_xodr
             *                   默认右匝道方向）
             *   merge         — 加速车道汇入，强制 target_speed（加速目标）+ target_lane
             *                   （汇入后车道方向），control 节点据此做 gap 检查 + 加速 */
            if (SM_MODE_OF(statem_current(&g.mode_sm)) == SM_MODE_NOA &&
                g.route_next_idx < g.route_count &&
                g.ego_x >= g.route[g.route_next_idx].trigger_x) {
                const ScenarioRouteStep* step = &g.route[g.route_next_idx];
                g.route_type = step->type;

                switch (step->type) {
                case ROUTE_BRANCH_SELECT: {
                    /* 分叉选路：选中 branch_id 指定的 connecting_road。
                     * target_lane 缺省 -1（右匝道方向）；若场景显式指定则用场景值。 */
                    int lane = (step->target_lane != 0) ? step->target_lane : -1;
                    g.route_target_lane = lane;
                    g.current_branch_id = step->branch_id;
                    if (step->target_speed > 0.0) g.route_target_speed = step->target_speed;
                    else                          g.route_target_speed = 0.0;
                    /* 触发参考路径刷新：branch_select 后 ego 进入新的 connecting_road，
                     * 参考路径需重新基于新路段几何构建（fallback 模式下沿用 curve_*
                     * 单段几何，HAVE_FRENET 模式下应切到 branch_id 对应道路采样）。 */
                    update_reference_path(g.ego_x - 5.0);
                    LOG_INFO("planning",
                             "NOA branch_select #%d @x=%.0f -> branch_id=%d lane=%d (%s)",
                             g.route_next_idx, g.ego_x, step->branch_id, lane,
                             step->label[0] ? step->label : "-");
                    break;
                }
                case ROUTE_MERGE: {
                    /* 加速车道汇入：强制 target_speed（加速到主路速度）+ target_lane
                     * （汇入后车道方向）。control 节点在 merge 状态下做 gap 检查 +
                     * 加速 + 横向汇入。target_lane 缺省 +1（汇入左侧主路）。
                     *
                     * NOA Phase 6 merge 闭环：进入 merge 后先置 state=1（等 gap），
                     * 主循环根据 scene/frame 的主线来车 gap 决策何时下发并入。
                     * gap 不足期间 route_target_lane 暂存到 merge_hold_lane 并置 0
                     * （不下发变道），control 保持当前车道跟车巡航。 */
                    int lane = (step->target_lane != 0) ? step->target_lane : 1;
                    g.route_target_lane = lane;
                    g.merge_hold_lane = lane;       /* 暂存，gap 充足时恢复 */
                    g.merge_state = 1;              /* 进入等 gap 状态 */
                    g.current_branch_id = -1;
                    if (step->target_speed > 0.0) {
                        g.route_target_speed = step->target_speed;
                        LOG_INFO("planning",
                                 "NOA merge #%d @x=%.0f -> lane=%d accel to %.1f m/s wait_gap (%s)",
                                 g.route_next_idx, g.ego_x, lane, step->target_speed,
                                 step->label[0] ? step->label : "-");
                    } else {
                        /* merge 必须有 target_speed（加速目标），缺省用 cfg_max_speed */
                        g.route_target_speed = g.cfg_max_speed;
                        LOG_INFO("planning",
                                 "NOA merge #%d @x=%.0f -> lane=%d accel to %.1f m/s wait_gap (default, %s)",
                                 g.route_next_idx, g.ego_x, lane, g.cfg_max_speed,
                                 step->label[0] ? step->label : "-");
                    }
                    break;
                }
                case ROUTE_LANE_CHANGE:
                default: {
                    /* 普通变道：原行为，设 target_lane + target_speed */
                    g.route_target_lane = step->target_lane;
                    g.current_branch_id = -1;
                    if (step->target_speed > 0.0) {
                        g.route_target_speed = step->target_speed;
                        LOG_INFO("planning",
                                 "NOA route step #%d triggered @x=%.0f -> lane=%d speed=%.1f (%s)",
                                 g.route_next_idx, g.ego_x, step->target_lane, step->target_speed,
                                 step->label[0] ? step->label : "-");
                    } else {
                        g.route_target_speed = 0.0;
                        LOG_INFO("planning",
                                 "NOA route step #%d triggered @x=%.0f -> lane=%d (%s)",
                                 g.route_next_idx, g.ego_x, step->target_lane,
                                 step->label[0] ? step->label : "-");
                    }
                    break;
                }
                }
                g.route_next_idx++;
            }

            if (g.ego_x > g.ref_path_start_x + g.cfg_ref_path_length * 0.8) {
                double new_start = g.ego_x - 50.0;
                if (new_start < 0.0) new_start = 0.0;
                update_reference_path(new_start);
                LOG_INFO("planning", "reference path shifted to x=%.0f..%.0f",
                         g.ref_path_start_x, g.ref_path_start_x + g.cfg_ref_path_length);
            }

            /* NOA 路线步骤可能指定目标速度（如出口匝道减速），覆盖纯粹的巡航速度。
             * 叠加关系：route_target_speed > 0 时作为最终目标，否则用默认 target_speed。 */
            double command_speed = (g.route_target_speed > 0.0) ? g.route_target_speed : g.target_speed;

            /* ── NOA Phase 6 merge 闭环：基于 scene/frame 主线来车 gap 决策并入 ──
             * merge_state==1（等 gap）：用 on_scene_frame 缓存的前后 gap 判断
             *   - 前 gap ≥ min_gap 且 后 gap ≥ min_gap*0.6 → state=2，恢复 route_target_lane 下发并入
             *   - 否则：route_target_lane 置 0（不下发变道），command_speed 跟前车（gap_front<60m 时按 TTC 限速）
             * merge_state==2（已下发并入）：保持 route_target_lane，不再干预
             * 无 scene/frame 数据时（has_scene_frame==0）保守放行，避免阻塞 demo。 */
            if (g.merge_state == 1 && g.route_type == ROUTE_MERGE) {
                double gf = g.merge_gap_front, gr = g.merge_gap_rear;
                double min_front = g.merge_min_gap;
                double min_rear  = g.merge_min_gap * 0.6;
                if (!g.has_scene_frame || (gf >= min_front && gr >= min_rear)) {
                    /* gap 充足或无数据 → 放行并入 */
                    g.route_target_lane = g.merge_hold_lane;
                    g.merge_state = 2;
                    LOG_INFO("planning",
                             "NOA merge gap OK front=%.0f rear=%.0f -> commit lane=%d",
                             gf, gr, g.merge_hold_lane);
                } else {
                    /* gap 不足：跟车巡航，不下发变道 */
                    g.route_target_lane = 0;
                    /* 前 gap < 60m 时按 3s TTC 限速跟随前车 */
                    if (gf < 60.0 && gf > 0.0) {
                        double follow_speed = (gf - 5.0) / 3.0;  /* (gap-5m)/3s */
                        if (follow_speed < 0.0) follow_speed = 0.0;
                        if (follow_speed > command_speed) follow_speed = command_speed;
                        command_speed = follow_speed;
                    }
                    if (g.plan_count % 25 == 0) {
                        LOG_INFO("planning",
                                 "NOA merge wait gap front=%.0f rear=%.0f (need f>=%.0f r>=%.0f) follow=%.1f",
                                 gf, gr, min_front, min_rear, command_speed);
                    }
                }
            }

            /* ── Phase 5: 会车让行 + 窄路减速 ────────────────
             * 在 Frenet 规划前调整 command_speed,让规划器在约束下生成轨迹。
             * 会车: 对向有来车时降速让行; 窄路: 两侧间距不足时降速。 */
            if (g.has_vstate) {
                int oncoming = 0;
                double min_clearance_left  = 1e9;  /* 左侧最近障碍横向距离 */
                double min_clearance_right = 1e9;  /* 右侧最近障碍横向距离 */

                for (int i = 0; i < 4; i++) {
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx < -10.0 || dx > 80.0) continue;
                    double dy = g.obs_y[i] - g.ego_y;  /* 相对 ego 的横向偏移 */

                    /* 会车检测: 对向车道 (dy > 2.0m,即 y≈+1.75 方向),
                     * 迎面驶来 (vx < -2 m/s), 前方 60m 内 */
                    if (dy > 2.0 && dx > 0.0 && dx < 60.0 &&
                        g.obs_vx[i] < -2.0) {
                        oncoming = 1;
                    }

                    /* 窄路检测: 统计左右两侧最近障碍物横向距离 */
                    if (dx > 0.0 && dx < 20.0) {  /* 20m 前瞻窗 */
                        if (dy < 0.0 && fabs(dy) < min_clearance_right)
                            min_clearance_right = fabs(dy);
                        if (dy > 0.0 && dy < min_clearance_left)
                            min_clearance_left = dy;
                    }
                }

                /* 会车让行: 降速到 40% 巡航速度 (≈5m/s),让对向车先通过 */
                if (oncoming) {
                    double yield_speed = g.target_speed * 0.4;
                    if (yield_speed < command_speed)
                        command_speed = yield_speed;
                }

                /* 窄路减速: 两侧间距 < 1.5m 时限制速度 */
                double narrow_width = min_clearance_left + min_clearance_right;
                if (narrow_width < 1e8 && narrow_width < 1.5) {
                    /* 间距越窄速度越低: 1.5m→100%, 0.5m→33% */
                    double ratio = (narrow_width - 0.3) / 1.2;
                    if (ratio < 0.1) ratio = 0.1;
                    if (ratio > 1.0) ratio = 1.0;
                    double narrow_speed = g.target_speed * ratio;
                    if (narrow_speed < command_speed)
                        command_speed = narrow_speed;
                }
            }

            /* 向 Frenet 规划器注入障碍物（世界坐标），触发自动避障/变道 */
#ifdef HAVE_FRENET
            if (g.has_vstate) {
                /* 障碍物数组扩容到 8：4 个 vehicle/state 障碍物 + 最多 4 个
                 * 红绿灯虚拟停止线墙。红绿灯墙在红灯/黄灯时注入，绿灯时不注入。
                 * Phase 3: 添加 vx/vy 数组，传给 Frenet 做位置外推。 */
                double ox[8], oy[8], ow[8], ol[8], ovx[8], ovy[8];
                int n_obs = 0;
                for (int i = 0; i < 4 && n_obs < 8; i++) {
                    /* 只传入前方和侧方的有效障碍物（排除行人 y>4） */
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx < OBS_MIN_DX_M || dx > OBS_MAX_DX_M) continue;
                    if (fabs(g.obs_y[i]) > OBS_MAX_ABS_Y_M) continue;
                    ox[n_obs]  = g.obs_x[i];
                    oy[n_obs]  = g.obs_y[i];
                    ow[n_obs]  = OBSTACLE_WIDTH_M;
                    ol[n_obs]  = OBSTACLE_LENGTH_M;
                    ovx[n_obs] = g.obs_vx[i];
                    ovy[n_obs] = g.obs_vy[i];
                    n_obs++;
                }

                /* 红绿灯虚拟停止线墙注入：
                 * 遍历缓存的灯，找前方最近的红/黄灯。若 ego 与停止线距离在
                 * 刹停可行范围内，注入一面跨车道宽度的静止"墙"——
                 * safety_control 现有 TTC/brake 逻辑直接对虚拟墙生效，免费复用。
                 *
                 * 刹停距离估算: d_brake = v² / (2*a) + 余量，a≈4 m/s²。
                 * 黄灯判据: 仅当能安全刹停时注入（dx > min_stop_dist）；
                 *           太近无法安全刹停时不注入（让车通过，避免急刹追尾）。 */
                if (g.has_traffic_lights && n_obs < 8) {
                    double v = g.ego_v;
                    if (v < 0.0) v = 0.0;
                    /* 刹停距离 = v²/(2*4) + 3m 安全余量；最小 5m 保证近距也能停 */
                    double brake_dist = v * v / 8.0 + 3.0;
                    if (brake_dist < 5.0) brake_dist = 5.0;
                    /* 黄灯最小安全刹停距离：速度太低时用 3m */
                    double min_yellow_stop = (v > 2.0) ? 3.0 : 0.0;

                    for (int ti = 0; ti < g.tl_count && n_obs < 8; ti++) {
                        if (g.tl_state[ti] == 0) continue;  /* 绿灯，不注入 */
                        double dx_tl = g.tl_x[ti] - g.ego_x;
                        if (dx_tl <= 0.0) continue;  /* 已过停止线 */
                        if (dx_tl > 60.0) continue;  /* 太远，不注入 */

                        if (g.tl_state[ti] == 2) {
                            /* 红灯：在刹停距离内注入墙 */
                            if (dx_tl > brake_dist + 5.0) continue;  /* 还很远，不急 */
                        } else {
                            /* 黄灯：仅当能安全刹停时注入（太近则通过） */
                            if (dx_tl < min_yellow_stop) continue;
                            if (dx_tl > brake_dist + 5.0) continue;
                        }

                        /* 注入虚拟墙：位置在停止线前 1m，宽度覆盖全路（8m > 6m OBS_MAX_ABS_Y），
                         * 长度给薄墙 0.5m。vx=0 静止。 */
                        ox[n_obs]  = g.tl_x[ti] - 1.0;  /* 停止线前 1m */
                        oy[n_obs]  = 0.0;               /* 路中心 */
                        ow[n_obs]  = 8.0;                /* 跨双车道 */
                        ol[n_obs]  = 0.5;                /* 薄墙 */
                        ovx[n_obs] = 0.0;                /* 静止虚拟墙 */
                        ovy[n_obs] = 0.0;
                        n_obs++;
                    }
                }

                /* Phase 3: 传入速度数据,Frenet bridge 做 2s 位置外推 */
                frenet_set_obstacles_v(g.frenet, ox, oy, ow, ol, ovx, ovy, n_obs);
            }
#endif

            /* 规划轨迹 */
            double s_out[50], d_out[50], spd_out[50];
            int n_wp = 0;

#ifdef HAVE_FRENET
            {
                /* 弯道参考路径在 y=-1.75 + road_center_y，所以 Frenet d 应是
                 * ego_y - (-1.75 + road_center_y(ego_x)) = ego_y + 1.75 - road_center_y(ego_x) */
                double rc_y = road_center_y(g.ego_x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
                double ego_d = g.ego_y + 1.75 - rc_y;  /* 相对 y=-1.75 的偏移（弯道修正） */
                n_wp = frenet_plan(g.frenet,
                    g.ego_x, ego_d, g.ego_v,
                    command_speed,
                    s_out, d_out, spd_out, 50);
            }
#else
            /* Fallback: 生成简单的车道保持 + 恒速轨迹 */
            {
                double horizon = 50.0;   /* 前方 50m */
                int n = 10;              /* 10 个路径点 */
                for (int i = 0; i < n; i++) {
                    s_out[i] = g.ego_x + horizon * (double)i / (double)(n - 1);
                    d_out[i] = 0.0;     /* 保持参考线（y=-1.75） */
                    spd_out[i] = command_speed;
                }
                n_wp = n;
            }
#endif

            char traj[1024];
            if (n_wp > 0) {
                command_speed = spd_out[0];
                /* Debuggability: report the ACTUAL planner mode instead of always
                 * claiming "frenet". When built without Eigen (HAVE_FRENET undefined),
                 * this path is the simple lane-keep fallback (d=0.0 always, no lane
                 * changes) — that must be visible in the trajectory JSON / dashboard,
                 * not just a one-line startup log that scrolls away. */
#ifdef HAVE_FRENET
                const char* traj_type = "frenet";
#else
                const char* traj_type = "lane_keep_fallback";
#endif
                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "type", traj_type);
                cJSON_AddNumberToObject(root, "plan", g.plan_count);
                cJSON_AddNumberToObject(root, "wp", n_wp);
                cJSON_AddNumberToObject(root, "target_speed", command_speed);
                cJSON* path = cJSON_AddArrayToObject(root, "path");
                for (int i = 0; i < n_wp; i++) {
                    if (i % 3 != 0 && i > 0 && i < n_wp - 1) continue;
                    cJSON* pt = cJSON_CreateArray();
                    cJSON_AddItemToArray(pt, cJSON_CreateNumber(s_out[i]));
                    cJSON_AddItemToArray(pt, cJSON_CreateNumber(d_out[i]));
                    cJSON_AddItemToArray(pt, cJSON_CreateNumber(spd_out[i]));
                    cJSON_AddItemToArray(path, pt);
                }
                char* json_part = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                snprintf(traj, sizeof(traj), "%s", json_part);
                cJSON_free(json_part);
            } else {
                double failsafe = command_speed;
                if (failsafe > g.ego_v + 1.0) failsafe = g.ego_v + 1.0;
                if (failsafe > g.cfg_max_speed) failsafe = g.cfg_max_speed;
                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "type", "failsafe");
                cJSON_AddNumberToObject(root, "target_speed", failsafe);
                cJSON_AddNumberToObject(root, "plan", g.plan_count);
                cJSON_AddNumberToObject(root, "lane_keep_d", 0.0);
                char* json_part = cJSON_PrintUnformatted(root);
                cJSON_Delete(root);
                snprintf(traj, sizeof(traj), "%s", json_part);
                cJSON_free(json_part);
            }

            /* 后向兼容: PID 也读取 speed= 字段。附加 mode/route_lane 供 control 消费
             * (NOA 主动变道) 及仪表盘展示驾驶模式，采用与 speed= 相同的宽松追加格式,
             * 不影响既有基于 strstr 的解析。
             *
             * NOA Phase 3.2/3.3: 追加 route_type= 字段（lane_change/branch_select/merge）
             * 供 control/monitor 区分当前 NOA 行为，branch_select 还附带 branch_id=。 */
            char mode_buf[32];
            statem_format_hierarchical(statem_current(&g.mode_sm), mode_buf, sizeof(mode_buf));
            const char* route_type_str = "lane_change";
            switch (g.route_type) {
                case ROUTE_BRANCH_SELECT: route_type_str = "branch_select"; break;
                case ROUTE_MERGE:         route_type_str = "merge";         break;
                default:                  route_type_str = "lane_change";   break;
            }
            char traj_final[1200];
            if (g.route_type == ROUTE_BRANCH_SELECT) {
                snprintf(traj_final, sizeof(traj_final),
                         "%s speed=%.1f mode=%s route_lane=%d route_type=%s branch_id=%d",
                         traj, command_speed, mode_buf, g.route_target_lane,
                         route_type_str, g.current_branch_id);
            } else {
                snprintf(traj_final, sizeof(traj_final),
                         "%s speed=%.1f mode=%s route_lane=%d route_type=%s",
                         traj, command_speed, mode_buf, g.route_target_lane,
                         route_type_str);
            }

            transport_publish(transport_, "planning/trajectory",
                              (const uint8_t*)traj_final,
                              (uint32_t)strlen(traj_final) + 1);
            g.plan_count++;

            if (g.plan_count % 25 == 1) {
                LOG_INFO("planning", "#%d ego@(%.0f,%.1f) v=%.1f → target=%.1f wp=%d",
                         g.plan_count, g.ego_x, g.ego_y, g.ego_v,
                         command_speed, n_wp);
            }
#ifndef HAVE_FRENET
            /* Repeat this loudly and periodically (not just once at init) so it
             * doesn't get lost in scrollback during a long-running demo — this is
             * exactly the kind of "why won't it overtake" question that should be
             * answerable from logs alone. */
            if (g.plan_count % 200 == 1) {
                LOG_WARN("planning", "#%d running WITHOUT Frenet planner — "
                         "lane-keep-only fallback, ego will NEVER change lanes. "
                         "Install libeigen3-dev and rebuild modules/adas_nodes.",
                         g.plan_count);
            }
#endif
        }

        LOG_INFO("planning", "stopped (%d trajectories, state=%s)",
                 g.plan_count, statem_state_name(&g.sm, g.sm.current));
        statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
        statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    }

private:
    Transport* transport_;
};

/* ── 协程宿主线程 ─────────────────────────────────────────────── */

void* planning_thread(void*) {
    try {
        g.task->execute();
    } catch (...) {
        LOG_ERROR("planning", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "vehicle/state", "road/geometry", "road/traffic_lights", "scene/frame", nullptr };
static const char* s_outputs[] = { "planning/trajectory", nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾 */

static int planning_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    /* 清零并重新初始化（atomic/unique_ptr 不可拷贝，逐字段赋值） */
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.should_stop  = false;
    g.has_fusion   = 0;
    g.has_vstate   = 0;

    g.mode_last_check_us = 0;
    g.last_fusion_us     = 0;
    g.highway_ready      = 0;
    g.highway_speed_timer = 0.0;

    g.route_count        = 0;
    g.route_next_idx     = 0;
    g.route_target_lane  = 0;
    g.route_target_speed = 0.0;
    /* NOA Phase 3.2/3.3: 路线步骤类型默认 lane_change，branch_id 复位 */
    g.route_type         = ROUTE_LANE_CHANGE;
    g.current_branch_id  = -1;

    g.plan_count  = 0;
    g.ego_x = g.ego_y = g.ego_v = g.ego_heading = 0.0;
    for (int i = 0; i < 4; i++) { g.obs_x[i] = g.obs_y[i] = g.obs_vx[i] = g.obs_vy[i] = 0.0; }

    g.curve_start_x   = 0.0;
    g.curve_length_m  = 0.0;
    g.curve_offset_m  = 0.0;
    g.ref_path_start_x = 0.0;

    /* 默认参数 */
    g.cfg_target_speed      = 15.0;
    g.cfg_max_speed         = 20.0;
    g.cfg_max_accel         = 4.0;
    g.cfg_ref_path_length   = 5000.0;
    g.cfg_highway_speed_mps = 13.0;  /* 未提供 highway_speed_mps 参数时的兜底默认值，
                                        需低于当前场景实际巡航速度才能触发 NP 升级；
                                        pipeline.json 会显式覆盖为更贴近实际场景的值。 */
    g.scenario_file[0] = '\0';

    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* item;
            if ((item = cJSON_GetObjectItem(root, "target_speed")))
                g.cfg_target_speed = item->valuedouble;
            if ((item = cJSON_GetObjectItem(root, "max_speed")))
                g.cfg_max_speed = item->valuedouble;
            if ((item = cJSON_GetObjectItem(root, "max_accel")))
                g.cfg_max_accel = item->valuedouble;
            if ((item = cJSON_GetObjectItem(root, "ref_path_length_m")))
                g.cfg_ref_path_length = item->valuedouble;
            if ((item = cJSON_GetObjectItem(root, "highway_speed_mps")))
                g.cfg_highway_speed_mps = item->valuedouble;
            if ((item = cJSON_GetObjectItem(root, "scenario_file")) && cJSON_IsString(item)) {
                size_t len = strlen(item->valuestring);
                if (len >= sizeof(g.scenario_file)) len = sizeof(g.scenario_file) - 1;
                memcpy(g.scenario_file, item->valuestring, len);
                g.scenario_file[len] = '\0';
            }
            cJSON_Delete(root);
        }
    }

    g.target_speed = g.cfg_target_speed;
    g.route_target_speed = 0.0;

    /* 从场景文件加载导航路线（可选）：NOA 主动变道所需的"路线/地图"数据来源。
     * Phase 2: 弯道几何不再从此处读取，改由 road/geometry topic 获取（sim_world 发布）。
     * 此处仅加载 NOA route steps。 */
    if (g.scenario_file[0] != '\0') {
        ScenarioConfig* sc = scenario_load(g.scenario_file);
        if (sc) {
            g.route_count = sc->route_count;
            /* sc 由 calloc 分配，未用槽位已清零；这里按实际 route_count 精确
             * 拷贝，避免依赖 calloc 的清零语义。 */
            memcpy(g.route, sc->route, sizeof(ScenarioRouteStep) * (size_t)g.route_count);
            scenario_free(sc);
            LOG_INFO("planning", "loaded %d NOA route step(s) from '%s'",
                     g.route_count, g.scenario_file);
        } else {
            LOG_WARN("planning", "scenario_file '%s' not loadable — NOA route disabled",
                     g.scenario_file);
        }
    }

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    g.frenet = frenet_create(g.cfg_max_speed, g.cfg_max_accel);
    if (!g.frenet) {
        LOG_ERROR("planning", "frenet_create failed");
        return -1;
    }
#else
    g.frenet = nullptr;
    LOG_INFO("planning", "built without Frenet — using lane-keep fallback");
#endif

    transport_subscribe(transport, "fusion/localization", on_fusion, nullptr);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, nullptr);
    transport_subscribe(transport, "road/geometry", on_road_geometry, nullptr);
    transport_subscribe(transport, "road/traffic_lights", on_traffic_lights, nullptr);
    transport_subscribe(transport, "scene/frame", on_scene_frame, nullptr);
    transport_advertise(transport, "planning/trajectory", 0x3A7B1C2Du);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/geometry", 0x80AD5C12u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/traffic_lights", 0x7E5C0FFEu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "scene/frame",         0x5CF12A60u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_PUBLISHER, 10.0);

    /* 初始化反射式状态机 */
    statem_init(&g.sm, nullptr, SM_STATE_INITIALIZED, "planning");
    statem_send_event(&g.sm, SM_EVENT_START, nullptr);

    /* 初始化驾驶模式状态机（NA/ACC/CP/NP/NOA）：真实条件驱动升降级 */
    statem_init(&g.mode_sm, SM_TABLE_MODE_SWITCHING, SM_MODE_NA, "driving_mode");
    statem_set_guard(&g.mode_sm, mode_transition_guard);

    g.task = std::make_unique<PlanningTask>(bus, transport);

    LOG_INFO("planning", "initialized (FlowCoro, target=%.0f m/s, max=%.0f m/s)",
             g.cfg_target_speed, g.cfg_max_speed);
    return 0;
}

static int planning_start(void) {
    if (!g.task) return -1;
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, planning_thread, nullptr) != 0) {
        LOG_WARN("planning", "pthread_create failed: %s", strerror(errno));
        return -1;
    }
    g.running = true;
    LOG_INFO("planning", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void planning_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->stop();
}

static void planning_cleanup(void) {
    planning_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
#ifdef HAVE_FRENET
    if (g.frenet) { frenet_destroy(g.frenet); g.frenet = nullptr; }
#else
    g.frenet = nullptr;
#endif
    statem_cleanup(&g.sm);
    statem_cleanup(&g.mode_sm);
    LOG_INFO("planning", "cleanup done");
}

static int planning_health(void) { return 0; }

/* ── 导出入口 ────────────────────────────────────────────────── */

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "planning",
    "1.0.0",
    "Frenet Optimal Trajectory Planner [FlowCoro]",
    s_inputs,
    s_outputs,
    planning_init,
    planning_start,
    planning_stop,
    planning_cleanup,
    planning_health,
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
