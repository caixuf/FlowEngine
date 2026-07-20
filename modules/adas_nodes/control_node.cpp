/**
 * control_node.cpp — PID 纵向/横向控制节点插件 (FlowCoro 协程版)
 *
 * 从 control_node.c 迁移而来，采用 CoroutineTask 协程框架：
 *   - co_await sleep_us(50000) 替代 usleep 20Hz 轮询（可被 stop 取消）
 *   - 保留 on_fusion / on_trajectory / on_vehicle_state / on_road_geometry 回调
 *   - PID + ACC + 变道状态机逻辑原样搬入 run()
 *   - was_blocked_sm 从函数 static 改为 Task 成员（语义等价）
 *
 * 采用 CoroutineTask（同步 resume），而非 FlowCoroTask（线程池 resume）：
 * control 是延迟敏感的闭环控制，周期精度直接影响横向稳定性。FlowCoroTask
 * 的线程池 resume 会引入调度抖动，导致 20Hz 周期不一致，prev_steer 低通
 * 滤波时间间隔波动，steer 产生小幅振荡（左摇右晃）。CoroutineTask 同步
 * resume 周期精确，且 PID+Stanley 计算量小（远小于 fusion 的 EKF+序列化），
 * 同步 resume 阻塞总线时间可忽略。与 safety_control 一致。
 *
 * 订阅 fusion/localization, planning/trajectory → 发布 control/raw_cmd
 *
 * NodePlugin 接口，编译为 libcontrol_node.so。
 */

#include "node_plugin.h"
#include "param_registry.h"
#include "state_machine.h"
#include "road_geometry.h"
#include "adas_msgs_gen.h"       /* ControlRaw_serialize, CONTROLRAW_TYPE_ID */
#include "coroutine_task.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

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
#include <vector>

namespace {

/* ── 节点本地状态 ───────────────────────────────────────────── */

/* NOA Phase 6: 障碍物槽位扩到 8。
 * 0-3: vehicle/state topic 提供的 ego-relative 障碍物（fusion 发布）
 * 4-7: scene/frame entities 提供的世界坐标车辆（flowsim 发布），由
 *      on_scene_frame 填充前方/侧方最近的车辆，补全 24-NPC 场景下 vehicle/state
 *      4 槽位截断导致的感知盲区——这是 P0 pipeline 冻结的根因之一：ego 被慢车
 *      阻挡但该慢车未进入 vehicle/state 的 4 个槽位，control 的被动超车逻辑
 *      因 best_gap=1e9 而不触发变道，ego 减速至 0 后管道静默。 */
#define MAX_OBS 8

/* 横向级联 PD 常量 */
#define MAX_PSI_DES_RAD    0.349   /* 最大期望航向角 ≈ ±20° */
#define STEER_FILTER_NEW   0.8     /* 低通滤波新值权重 (0.8: -3dB@2.8Hz, 减少相位滞后防蛇行) */
#define STEER_FILTER_PREV  0.2     /* 低通滤波旧值权重 */
#define LC_STABILIZE_S     1.0     /* 变道完成后保持变道增益的稳定期 (s) */
#define LC_COMPLETE_THRESH 0.15    /* 变道完成横向偏差阈值 (m) — 收紧防振荡 */
/* 轴距 (m)：真车默认 2.7，RC 小车在 pipeline_car.json 里通过 params.wheelbase
 * 覆盖为 0.25-0.4。这个宏只作为 g.wheelbase 的初值，运行时由配置注入。 */
#define CONTROL_WHEELBASE_DEFAULT_M 2.7
/* 控制环周期: 20Hz → 50ms。所有计时器累加步长使用此常量, 与实际循环频率保持一致。 */
#define CONTROL_DT_S       0.05

/* 车道判定迟滞: 已提交车道保持不变, 直到 ego_y 越过中心线 ±此死区才切换,
 * 避免车骑在车道线 (y≈0) 附近时目标车道每帧翻转造成的横向抖振。 */
#define LANE_HYSTERESIS_M   0.5
/* 死锁恢复: 近乎静止且横向卡在车道线附近持续超过此秒数, 强制收敛到最近车道中心。 */
#define STUCK_SPEED_MPS     0.5    /* 判定"近乎静止"的速度阈值 */
#define STUCK_LATERAL_M     0.6    /* 判定"卡在车道线附近"的 |ego_y| 阈值 */
#define STUCK_RECOVER_S     3.0    /* 触发恢复所需持续时间 (秒) */
/* 全域速度死锁: 不依赖横向位置, 只要速度持续为0超过此秒数就给小油门。
 * 覆盖 STUCK (|y|<0.6) 和 ROAD_GUARD (|y|>2.1) 之间的盲区 0.6<|y|<2.1。 */
#define SPEED_ZERO_RECOVER_S  5.0  /* 全域速度死锁触发阈值 (秒) */

/* ── 控制节点状态机定义 ─────────────────────────────────────── */
/* 自定义事件 (从 SM_EVENT_USER_BASE=16 开始) */
#define CTL_EVENT_OBSTACLE_BLOCKED  16
#define CTL_EVENT_OBSTACLE_CLEARED  17
#define CTL_EVENT_LANE_CHANGE_START 18
#define CTL_EVENT_LANE_CHANGE_DONE  19
#define CTL_EVENT_SPEED_LIMITED     20

/* 自定义状态 */
#define CTL_STATE_CRUISING    SM_STATE_RUNNING     /* 复用 RUNNING = 正常巡航 */
/* blocked/lane_change 通过 TransitionRule 的 description 区分 */

static const TransitionRule g_ctl_transitions[] = {
    { SM_STATE_INITIALIZED, SM_EVENT_START,             SM_STATE_RUNNING,    "INIT→RUNNING",          false },
    { SM_STATE_RUNNING,     CTL_EVENT_OBSTACLE_BLOCKED, SM_STATE_RUNNING,    "RUNNING: obstacle_blocked", false },
    { SM_STATE_RUNNING,     CTL_EVENT_OBSTACLE_CLEARED, SM_STATE_RUNNING,    "RUNNING: obstacle_cleared", false },
    { SM_STATE_RUNNING,     CTL_EVENT_LANE_CHANGE_START,SM_STATE_RUNNING,    "RUNNING: lane_change_start",false },
    { SM_STATE_RUNNING,     CTL_EVENT_LANE_CHANGE_DONE, SM_STATE_RUNNING,    "RUNNING: lane_change_done", false },
    { SM_STATE_RUNNING,     SM_EVENT_STOP,              SM_STATE_STOPPING,   "RUNNING→STOPPING",       false },
    { SM_STATE_STOPPING,    SM_EVENT_DONE,              SM_STATE_STOPPED,    "STOPPING→STOPPED",       false },
    { SM_STATE_RUNNING,     SM_EVENT_ERROR,             SM_STATE_ERROR,      "RUNNING→ERROR",          false },
    { SM_STATE_ERROR,       SM_EVENT_RESTART,           SM_STATE_INITIALIZED,"ERROR→INIT",             false },
    TRANSITION_TABLE_END
};

struct ControlContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler*        scheduler{nullptr};

    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* PID 状态 */
    double kp{0}, ki{0}, kd{0};
    double integral{0};
    double prev_error{0};
    /* 横向级联 PD 状态 */
    double lat_kp{0};          /* lateral error → desired heading (rad/m) */
    double lat_kd_heading{0};  /* heading error → steer (阻尼) */
    double yaw_damping{0};     /* yaw_rate → steer 阻尼, 抑制极限环振荡 */
    double ego_heading{0};     /* 从 fusion 获取的航向角 (rad) */
    double ego_yaw_rate{0};    /* 从 fusion 获取的偏航角速度 (rad/s) */
    double prev_steer{0};

    /* 从 topic 解析的值 */
    double current_speed{0};
    double target_speed{0};
    double ego_x{0}, ego_y{0};
    double lane_d{0};          /* 从 trajectory 解析的横向偏移（Frenet d） */
    char   driving_mode[32]{}; /* 从 planning 广播的驾驶模式（如 "NOA:READY"），仅用于日志/透传 */
    /* NOA 导航路线要求的目标车道索引。
     * 语义：-1=无目标（保持当前车道），0..N-1=目标车道索引。
     * 旧约定（{-1=左, 0=无, +1=右}）已被多车道模型取代，planning 下发时
     * 会做兼容映射（详见 planning_node.cpp 的 route_target_lane 注释）。 */
    int    route_lane{-1};

    /* 障碍物数据 (从 vehicle/state 解析) */
    double obs_x[MAX_OBS]{}, obs_y[MAX_OBS]{}, obs_vx[MAX_OBS]{};
    int    obs_valid[MAX_OBS]{};
    char   obs_type[MAX_OBS][16]{}; /* e.g. "car", "pedestrian" */
    int    ped_index{-1};              /* index of pedestrian obs, -1 if none */

    volatile int has_fusion{0};
    volatile int has_planning{0};
    uint64_t last_fusion_us{0};    /* monotonic timestamp of last fusion message */
    uint64_t last_planning_us{0};  /* monotonic timestamp of last planning message */

    /* 变道状态机 */
    int    lc_state{0};     /* 0=正常 1=左变道中 2=左车道巡航 3=右回正 */
    int    lc_attempted{0};
    double lc_timer{0};
    double lc_wait{0};
    double lc_cooldown{0};
    double lc_origin_y{0};
    double lc_target_y{0};
    int    lc_target_idx{-1};   /* 变道目标车道索引（变道发起时记录，完成时 commit 到 committed_lane_side） */
    double lane_width{3.5};
    int    lane_count{2};       /* 当前 ego 所在路段可行驶车道数（从 road/geometry 实时订阅） */
    double blocked_timeout_s{0};

    /* NOA 超车调优参数（原硬编码，现可配置 + pipeline.json 即时生效） */
    double lc_stable_wait_s{0};           /* 变道后稳定巡航多久允许再次评估变道 (原硬编码 8.0) */
    double lc_cooldown_after_stable_s{0}; /* 稳定期结束后的冷却 (原硬编码 3.0) */
    double lc_cooldown_after_return_s{0}; /* 回到原车道后的冷却 (原硬编码 4.0) */
    double min_overtake_gap_base{0};      /* 触发超车所需最小本车道前车间距基准 (原硬编码 18.0) */
    double min_overtake_gap_cap{0};       /* min_overtake_gap 上限, 高速时不再无脑扩大 (原硬编码 60.0) */
    double min_overtake_gap_speed_mult{0};/* min_overtake_gap 相对速度乘数 (原硬编码 current_speed * 2.0，高速下会导致 gap 过大无法触发超车) */
    double steer_min_clamp{0};            /* 高速最小转向钳位 (原硬编码 0.012，高速下变道耗时过长) */

    /* LDW 车道偏离预警 */
    double ldw_threshold{0.5};            /* 横向偏离阈值 (m)，|cte| 超此值发警告 */
    double ldw_min_speed{1.0};            /* LDW 生效最低速度 (m/s)，低于此速不告警（停车/起步不算偏离） */
    double ldw_cooldown{2.0};            /* 告警冷却期 (s)，避免刷屏 */
    double ldw_last_warn_time{0};        /* 上次告警时间 (s) */

    /* 车道迟滞 + 死锁恢复状态。
     * 字段名保留 committed_lane_side 以减少 diff，但语义已改为
     * committed_lane_idx: -1=未初始化, 0..N-1=车道索引（0=最左, N-1=最右）。
     * 旧约定 {-1=左, 0=未初始化, +1=右} 在 N 车道模型下不够用，已废弃。 */
    int    committed_lane_side{-1};
    double stuck_timer{0};          /* 近乎静止且卡在车道线附近的累计时间 (秒) */
    double speed_zero_timer{0};     /* 全域速度死锁: 无论 y 位置, 速度持续为0的累计时间 (秒) */

    uint32_t cycle{0};

    /* 状态机（反射式生命周期跟踪） */
    ReflectiveStateMachine sm{};

    /* 配置参数 */
    double cfg_kp{0}, cfg_ki{0}, cfg_kd{0};
    double cfg_cruise_speed{0};
    double wheelbase{CONTROL_WHEELBASE_DEFAULT_M};  /* 轴距 (m)：真车 2.7，RC 小车 0.25-0.4 */

    /* 道路几何（Phase 2: 从 road/geometry topic 获取，全零 = 直道） */
    double curve_start_x{0};
    double curve_length_m{0};
    double curve_offset_m{0};

    /* ego route-following 参考路径（从 road/ref_path topic 获取）。
     * 当 ref_path 非空时，Stanley 横向控制用最近点的 (y, h, kappa) 替代 curve_*
     * 单段直线参考，让 ego 能跟随多 edge + fork 路网（如匝道分叉）。
     * 空 ref_path 时回退到 curve_*（兼容旧场景/旧 flowsim）。 */
    struct RefPt { double x, y, h, kappa, rs; };
    std::vector<RefPt> ref_path;
    uint64_t last_ref_path_us{0};
    pthread_mutex_t ref_path_mtx = PTHREAD_MUTEX_INITIALIZER;

    /* NOA Phase 3.4: 弯道曲率前馈权重提升参数。
     * 当道路曲率半径 R ≤ curve_ff_boost_radius_m 时，前馈权重 × curve_ff_boost_factor，
     * 让 Stanley 控制器在急弯（如匝道 R=45m）预先打方向，而非等 CTE 累积后反应。
     * 默认 R≤60m 触发 ×1.5 提升，可经 params 配置覆盖。 */
    double curve_ff_boost_radius_m{60.0};
    double curve_ff_boost_factor{1.5};

    /* 协程任务 */
    std::unique_ptr<class ControlTask> task;
};

ControlContext g;

static double steer_limit_for_speed(double speed_mps, double max_lateral_accel_mps2) {
    double speed = speed_mps;
    if (speed < 2.0) speed = 2.0;
    double limit = atan(max_lateral_accel_mps2 * g.wheelbase / (speed * speed));
    if (limit < g.steer_min_clamp) limit = g.steer_min_clamp;
    if (limit > 0.24) limit = 0.24;
    return limit;
}

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* cJSON parsing (fusion/localization now publishes cJSON) */
    {
        cJSON* root = cJSON_Parse((const char*)msg->data);
        if (root) {
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(root, "v");
            if (cJSON_IsNumber(j)) g.current_speed = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(root, "x");
            if (cJSON_IsNumber(j)) g.ego_x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(root, "y");
            if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(root, "heading");
            if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(root, "yaw_rate");
            if (cJSON_IsNumber(j)) g.ego_yaw_rate = j->valuedouble;
            cJSON_Delete(root);
        }
        g.has_fusion = 1;
        g.last_fusion_us = clock_now_us();
    }
}

static void on_trajectory(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;

    /* 解析 JSON 部分（cJSON 会忽略尾部附加文本） */
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "target_speed");
        if (cJSON_IsNumber(j)) g.target_speed = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "lane_keep_d");
        if (cJSON_IsNumber(j)) g.lane_d = j->valuedouble;
        /* 解析路径数组 first element: [s,d,spd] */
        cJSON* path = cJSON_GetObjectItemCaseSensitive(root, "path");
        if (cJSON_IsArray(path) && cJSON_GetArraySize(path) > 0) {
            cJSON* first = cJSON_GetArrayItem(path, 0);
            if (cJSON_IsArray(first) && cJSON_GetArraySize(first) >= 2) {
                cJSON* d_item = cJSON_GetArrayItem(first, 1);
                if (cJSON_IsNumber(d_item)) g.lane_d = d_item->valuedouble;
            }
        }
        cJSON_Delete(root);
    } else {
        /* cJSON parse failed, try text fallback for target_speed */
        const char* r = strstr(d, "speed=");
        if (r) sscanf(r + 6, "%lf", &g.target_speed);
    }

    /* 尾部附加文本字段：mode=, route_lane= */
    {
        const char* p = strstr(d, "mode=");
        if (p) {
            char buf[32] = {0};
            sscanf(p + 5, "%31s", buf);
            snprintf(g.driving_mode, sizeof(g.driving_mode), "%s", buf);
        }
    }
    {
        const char* p = strstr(d, "route_lane=");
        if (p) {
            int v = -1;
            sscanf(p + 11, "%d", &v);
            /* 兼容旧约定：planning 仍可能下发 0=无目标（旧 2 车道场景）。
             * 新约定下 -1=无目标，0=第 0 车道（最左）。仅在 lane_count==2 且
             * |v|<=1 时把旧 0 映射为 -1；多车道场景下 0 是合法车道索引，原样保留。 */
            if (g.lane_count == 2 && v >= -1 && v <= 1 && v == 0) v = -1;
            g.route_lane = v;
        }
    }

    g.has_planning = 1;
    g.last_planning_us = clock_now_us();
}

/* ── vehicle/state 订阅 — 解析障碍物位置 ─────────────────────── */
static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    g.ped_index = -1;
    if (root) {
        for (int i = 0; i < MAX_OBS; i++) {
            char key[16];
            snprintf(key, sizeof(key), "ox%d", i);
            cJSON* j = cJSON_GetObjectItemCaseSensitive(root, key);
            if (cJSON_IsNumber(j)) {
                g.obs_valid[i] = 1;
                g.obs_x[i] = j->valuedouble;
            } else { g.obs_valid[i] = 0; continue; }
            snprintf(key, sizeof(key), "oy%d", i);
            j = cJSON_GetObjectItemCaseSensitive(root, key);
            if (cJSON_IsNumber(j)) g.obs_y[i] = j->valuedouble;
            snprintf(key, sizeof(key), "ov%d", i);
            j = cJSON_GetObjectItemCaseSensitive(root, key);
            if (cJSON_IsNumber(j)) g.obs_vx[i] = j->valuedouble;
            snprintf(key, sizeof(key), "ot%d", i);
            j = cJSON_GetObjectItemCaseSensitive(root, key);
            if (cJSON_IsString(j) && j->valuestring) {
                size_t tlen = strlen(j->valuestring);
                if (tlen >= sizeof(g.obs_type[i])) tlen = sizeof(g.obs_type[i]) - 1;
                memcpy(g.obs_type[i], j->valuestring, tlen);
                g.obs_type[i][tlen] = '\0';
                if (strcmp(g.obs_type[i], "pedestrian") == 0 && g.ped_index < 0)
                    g.ped_index = i;
            }
        }
        cJSON_Delete(root);
    }
}

static double lane_lead_gap(double lane_y, double same_lane_tol) {
    double best_gap = 1e9;
    for (int i = 0; i < MAX_OBS; i++) {
        if (!g.obs_valid[i] || g.obs_vx[i] < 0) continue;
        if (fabs(g.obs_y[i] - lane_y) > same_lane_tol) continue;
        double dx = g.obs_x[i] - g.ego_x;
        double gap = dx - 4.6;
        if (dx > 0 && gap < best_gap) best_gap = gap;
    }
    return best_gap;
}

/* ── scene/frame 订阅（NOA Phase 6 超车感知补全） ──────────────
 * 从 flowsim 的 scene/frame entities 取世界坐标车辆，填入 obs[4..7]。
 * 筛选：前方 dx∈[-10,120]m、横向 |dy|<6m、同向 vx>0 的 car/suv/truck。
 * 填充策略：按 dx 升序取前 4 辆填入 4-7 槽位，每帧覆盖（entities 是最新快照）。
 * 这补全了 vehicle/state 4 槽位截断导致的盲区，让 lane_lead_gap 等函数
 * 能看到前方慢车，触发被动超车状态机。 */
static int _ent_cmp_dx(const void* a, const void* b) {
    double da = ((const double*)a)[0], db = ((const double*)b)[0];
    return (da > db) - (da < db);
}

static void on_scene_frame(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* entities = cJSON_GetObjectItem(root, "entities");
    if (!entities || !cJSON_IsArray(entities)) { cJSON_Delete(root); return; }

    /* 收集候选车辆 [dx, x, y, vx, type_idx]，type_idx: 0=car 1=suv 2=truck */
    double cand[32][5];
    int ncand = 0;
    cJSON* ent;
    cJSON_ArrayForEach(ent, entities) {
        if (ncand >= 32) break;
        cJSON* jtype = cJSON_GetObjectItem(ent, "type");
        if (!jtype || !cJSON_IsString(jtype)) continue;
        const char* t = jtype->valuestring;
        int tidx;
        if (strcmp(t, "car") == 0) tidx = 0;
        else if (strcmp(t, "suv") == 0) tidx = 1;
        else if (strcmp(t, "truck") == 0) tidx = 2;
        else continue;

        cJSON* jx = cJSON_GetObjectItem(ent, "x");
        cJSON* jy = cJSON_GetObjectItem(ent, "y");
        cJSON* jvx = cJSON_GetObjectItem(ent, "vx");
        if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jy) || !cJSON_IsNumber(jvx)) continue;

        double ex = jx->valuedouble, ey = jy->valuedouble, evx = jvx->valuedouble;
        double dx = ex - g.ego_x, dy = ey - g.ego_y;
        if (dx < -10.0 || dx > 120.0) continue;       /* 前方 120m 内 */
        if (fabs(dy) > 6.0) continue;                 /* 道路范围内 */
        if (evx < 0.0) continue;                       /* 同向车 */

        cand[ncand][0] = dx;
        cand[ncand][1] = ex;
        cand[ncand][2] = ey;
        cand[ncand][3] = evx;
        cand[ncand][4] = (double)tidx;
        ncand++;
    }
    cJSON_Delete(root);

    /* 先清空 4-7 槽位（每帧重填），0-3 保留 vehicle/state 数据 */
    for (int i = 4; i < MAX_OBS; i++) g.obs_valid[i] = 0;

    if (ncand == 0) return;
    qsort(cand, ncand, sizeof(cand[0]), _ent_cmp_dx);  /* 按 dx 升序 */

    static const char* type_names[] = { "car", "suv", "truck" };
    int fill = 4;
    for (int i = 0; i < ncand && fill < MAX_OBS; i++) {
        g.obs_x[fill]    = cand[i][1];
        g.obs_y[fill]    = cand[i][2];
        g.obs_vx[fill]   = cand[i][3];
        g.obs_valid[fill] = 1;
        int tidx = (int)cand[i][4];
        if (tidx < 0 || tidx > 2) tidx = 0;
        snprintf(g.obs_type[fill], sizeof(g.obs_type[fill]), "%s", type_names[tidx]);
        fill++;
    }
}

static double lane_lead_speed(double lane_y, double same_lane_tol) {
    double best_dx = 1e9;
    double speed = 1e9;
    for (int i = 0; i < MAX_OBS; i++) {
        if (!g.obs_valid[i] || g.obs_vx[i] < 0) continue;
        if (fabs(g.obs_y[i] - lane_y) > same_lane_tol) continue;
        double dx = g.obs_x[i] - g.ego_x;
        if (dx > 0 && dx < best_dx) {
            best_dx = dx;
            speed = g.obs_vx[i];
        }
    }
    return speed;
}

/* ── road/geometry 订阅回调（Phase 2 统一道路几何） ─────────── */
/* 从 sim_world 发布的 road/geometry topic 获取弯道参数 + 车道宽度，
 * 替代此前各自 scenario_load() 的冗余方式。 */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "curve_start_x");
        if (cJSON_IsNumber(j)) g.curve_start_x = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "curve_length_m");
        if (cJSON_IsNumber(j)) g.curve_length_m = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "curve_offset_m");
        if (cJSON_IsNumber(j)) g.curve_offset_m = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "lane_width");
        if (cJSON_IsNumber(j)) g.lane_width = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "lane_count");
        if (cJSON_IsNumber(j) && j->valuedouble >= 1.0) g.lane_count = (int)j->valuedouble;
        cJSON_Delete(root);
    }
}

/**
 * on_ref_path — ego route-following 参考路径订阅回调。
 *
 * flowsim 每帧采样 ego 前方 100m 内 N 个参考点 [(x,y,h,kappa,rs)]，control_node
 * 缓存到本地，Stanley 横向控制用最近点替代 curve_* 算 cte/heading/kappa。
 *
 * 解析在订阅回调里完成（轻量 cJSON 操作），缓存到 g.ref_path 加 mutex 保护，
 * 控制循环直接读缓存无需重新解析。
 */
static void on_ref_path(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    cJSON* pts = cJSON_GetObjectItemCaseSensitive(root, "points");
    if (cJSON_IsArray(pts)) {
        std::vector<ControlContext::RefPt> new_pts;
        new_pts.reserve(cJSON_GetArraySize(pts));
        cJSON* pt = nullptr;
        cJSON_ArrayForEach(pt, pts) {
            ControlContext::RefPt r{};
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(pt, "x");     if (cJSON_IsNumber(j)) r.x = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(pt, "y");     if (cJSON_IsNumber(j)) r.y = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(pt, "h");     if (cJSON_IsNumber(j)) r.h = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(pt, "kappa");  if (cJSON_IsNumber(j)) r.kappa = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(pt, "rs");     if (cJSON_IsNumber(j)) r.rs = j->valuedouble;
            new_pts.push_back(r);
        }
        pthread_mutex_lock(&g.ref_path_mtx);
        g.ref_path.swap(new_pts);
        g.last_ref_path_us = clock_now_us();
        pthread_mutex_unlock(&g.ref_path_mtx);
    }
    cJSON_Delete(root);
}

/**
 * query_ref_at — 查找 ref_path 中离 (ego_x, ego_y) 最近的参考点。
 *
 * 用于 Stanley 横向控制替代 curve_* 单段直线参考：
 *   - 返回该点的 (y, h, kappa) 让 ego 跟随多 edge + fork 路网（如匝道分叉）
 *   - ref_path 为空或陈旧 (>500ms 未更新) → 返回 false，调用方回退 curve_*
 *   - 最近点距离 > 5m（ego 已离开参考路径覆盖范围）→ 返回 false 避免误用
 *
 * 注意：ref_path 是 ego 前方 100m 的离散采样（5m 步长），最近点本身略偏 ego
 * 前方——这对横向控制是合理的（轻微前瞻）。
 */
static bool query_ref_at(double ego_x, double ego_y,
                         double& out_y, double& out_h, double& out_kappa) {
    if (g.ref_path.empty()) return false;
    uint64_t now_us = clock_now_us();
    if (g.last_ref_path_us == 0 ||
        (now_us > g.last_ref_path_us &&
         now_us - g.last_ref_path_us > 500000ULL)) return false;

    pthread_mutex_lock(&g.ref_path_mtx);
    if (g.ref_path.empty()) {
        pthread_mutex_unlock(&g.ref_path_mtx);
        return false;
    }
    double best_d2 = 1e18;
    const ControlContext::RefPt* best = nullptr;
    for (const auto& p : g.ref_path) {
        double dx = p.x - ego_x;
        double dy = p.y - ego_y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = &p; }
    }
    if (!best || best_d2 > 25.0 /* >5m, ego 偏离参考 */) {
        pthread_mutex_unlock(&g.ref_path_mtx);
        return false;
    }
    out_y     = best->y;
    out_h     = best->h;
    out_kappa = best->kappa;
    pthread_mutex_unlock(&g.ref_path_mtx);
    return true;
}

static int lane_rear_safe(double target_lane_y, double same_lane_tol) {
    for (int i = 0; i < MAX_OBS; i++) {
        if (!g.obs_valid[i]) continue;
        if (fabs(g.obs_y[i] - target_lane_y) > same_lane_tol) continue;
        double dx = g.obs_x[i] - g.ego_x;
        if (dx >= 0.0) continue;
        double closing_speed = g.obs_vx[i] - g.current_speed;
        double required_rear_gap = 12.0 + fmax(0.0, closing_speed) * 2.0;
        if (-dx < required_rear_gap) return 0;
    }
    return 1;
}

static int lane_front_allows_merge(double target_lane_y, double same_lane_tol, int* need_accel) {
    double target_gap = lane_lead_gap(target_lane_y, same_lane_tol);
    double target_speed = lane_lead_speed(target_lane_y, same_lane_tol);
    *need_accel = 0;
    if (target_gap > 40.0) return 1;
    if (target_gap > 18.0 && target_speed > g.current_speed + 1.5) {
        *need_accel = 1;
        return 1;
    }
    return 0;
}

static int lane_has_pedestrian_risk(double target_lane_y, double same_lane_tol) {
    int pi = g.ped_index;
    if (pi < 0 || pi >= MAX_OBS || !g.obs_valid[pi]) return 0;
    double dx = g.obs_x[pi] - g.ego_x;
    if (dx < -8.0 || dx > 90.0) return 0;
    /* 路边行人 (|y|>5.0) 不会横穿到目标车道，不构成风险；
     * 仅当行人实际处于路面范围内 (|y|<=5.0) 且靠近目标车道时才视为风险 */
    if (fabs(g.obs_y[pi]) > 5.0) return 0;
    return fabs(g.obs_y[pi] - target_lane_y) <= same_lane_tol + 0.8;
}

/* ── 协程任务 ────────────────────────────────────────────────── */

class ControlTask : public CoroutineTask {
public:
    ControlTask(MessageBus* bus, Transport* transport)
        : CoroutineTask(bus), transport_(transport) {}

protected:
    Task run() override {
        pthread_setname_np(pthread_self(), "control");

        const double same_lane_tol = 2.0;
        const double time_headway  = 1.4;
        const double min_gap       = 5.0;

        while (!should_stop()) {
            /* 替代 usleep：sleep_us 自动注入 cancel_token_，stop() 可立即唤醒 */
            co_await sleep_us(50000);  /* 20Hz */
            if (should_stop()) break;

            g.cycle++;

            /* Reset stale data flags: if no message received for >1000ms, clear flag */
            uint64_t now_us = clock_now_us();
            if (g.has_fusion   && now_us - g.last_fusion_us   > 1000000ULL) g.has_fusion   = 0;
            if (g.has_planning && now_us - g.last_planning_us > 1000000ULL) g.has_planning = 0;

            /* 数据陈旧时不跳过输出——发布安全减速指令，保持下游流水线畅通 */
            if (!g.has_fusion || !g.has_planning) {
                /* Phase 5: 弯道跟随。原始 fallback 把 target 钉死在 0.0，遇到弯道
                 * （road_center_y != 0）会让车辆直线冲出行车道。现改为：先按
                 * 道路中心线算一个 Stanley 风格转向指令，再做轻刹车保持车流连续。
                 * 该指令沿用主控制器相同的 lat_kp / lat_kd_heading / 一阶低通，
                 * 保证 fallback 与主控制输出在弯道中行为一致。 */
                double fb_road_c = 0.0, fb_road_heading = 0.0, fb_kappa_unused = 0.0;
                if (g.has_fusion) {
                    if (!query_ref_at(g.ego_x, g.ego_y, fb_road_c, fb_road_heading, fb_kappa_unused)) {
                        /* ref_path 不可用 → 回退到 curve_* 单段直线参考 */
                        fb_road_c = road_center_y(g.ego_x, g.curve_start_x,
                                                  g.curve_length_m, g.curve_offset_m);
                        fb_road_heading = road_center_heading(g.ego_x, g.curve_start_x,
                                                              g.curve_length_m, g.curve_offset_m);
                    }
                }
                /* 关键：目标必须是 ego 所在车道中心，而非道路中心线 road_c。
                 * 早期版本用 road_c 作目标，导致 ego 从 y=-1.75 被拉向 y=0，
                 * 越过中心线后 committed_lane_side 翻转、lc_state 被强制设为 2 并进入
                 * ROAD_GUARD，最终冲出右侧路沿 (road departure)。
                 *
                 * N 车道模型：用 lane_idx_from_y 算出 ego 当前所在车道 idx，
                 * 再用 lane_center_y 算该车道中心 y。EKF 未收敛（|ego_y - road_c| < 1.0m）
                 * 时 target=ego_y（横向不动），避免被拉向任意一侧。 */
                double fb_target_y = fb_road_c;
                if (g.has_fusion) {
                    if (fabs(g.ego_y - fb_road_c) > 1.0) {
                        int fb_idx = lane_idx_from_y(g.ego_y, g.lane_count, g.lane_width, fb_road_c);
                        fb_target_y = lane_center_y(fb_idx, g.lane_count, g.lane_width, fb_road_c);
                    } else {
                        fb_target_y = g.ego_y;  /* EKF 未收敛, 横向保持不动 */
                    }
                }
                double fb_lat_error = fb_target_y - g.ego_y;
                double fb_cte_term  = atan2(g.lat_kp * fb_lat_error, fmax(g.current_speed, 3.0));
                double fb_heading_term = g.lat_kd_heading * (g.ego_heading - fb_road_heading);
                double fb_steer = fb_cte_term - fb_heading_term;
                double fb_steer_limit = steer_limit_for_speed(g.current_speed, 1.4);
                if (fb_steer >  fb_steer_limit) fb_steer =  fb_steer_limit;
                if (fb_steer < -fb_steer_limit) fb_steer = -fb_steer_limit;
                fb_steer = STEER_FILTER_NEW * fb_steer + STEER_FILTER_PREV * g.prev_steer;
                g.prev_steer = fb_steer;

                ControlRaw raw;
                raw.seq      = g.cycle;
                raw.throttle = 0.0f;
                raw.brake    = 0.25f;  /* 温和减速，防止无人加速撞前车 */
                raw.steering = (float)fb_steer;
                raw.speed    = (float)g.current_speed;
                raw.target   = (float)fb_target_y;  /* 跟随 ego 所在车道中心 */
                raw.error    = (float)fb_lat_error;
                memset(raw.mode, 0, sizeof(raw.mode));
                snprintf(raw.mode, sizeof(raw.mode), "DATA_TIMEOUT");

                uint8_t raw_buf[64];
                size_t  raw_len = sizeof(raw_buf);
                ControlRaw_serialize(&raw, raw_buf, &raw_len);
                transport_publish(transport_, "control/raw_cmd",
                                  raw_buf, (uint32_t)raw_len);

                char cmd_text[256];
                snprintf(cmd_text, sizeof(cmd_text),
                         "throttle=0.00 brake=0.25 steer=%.4f "
                         "speed=%.1f target=%.1f error=%.1f mode=DATA_TIMEOUT",
                         fb_steer, g.current_speed, fb_target_y, fb_lat_error);
                transport_publish(transport_, "control/raw_cmd/text",
                                  (const uint8_t*)cmd_text, (uint32_t)strlen(cmd_text) + 1);

                if (g.cycle % 20 == 1) {
                    LOG_WARN("control", "#%d DATA_TIMEOUT — lane-following fallback "
                             "(spd=%.1f, steer=%.4f, target_y=%.2f, err=%.2f)",
                             g.cycle, g.current_speed, fb_steer, fb_target_y, fb_lat_error);
                }
                continue;
            }

            if (g.lc_cooldown > 0.0) g.lc_cooldown -= CONTROL_DT_S;

            /* 道路中心线在当前 ego_x 处的横向偏移（弯道禁用时恒为 0，下面所有
             * "相对道路中心"的判断与之前的绝对 y 判断完全等价）。
             *
             * A4 ego route-following: 优先用 ref_path（flowsim 每帧采样 ego 前方
             * 100m 路网中心线）替代 curve_* 单段直线参考。ref_path 不可用时回退
             * curve_*，保证旧场景/旧 flowsim 兼容。
             * road_heading / ref_kappa 缓存给下方 Stanley 块复用，避免重复查询。 */
            double road_c = 0.0;
            double ref_road_heading = 0.0;
            double ref_kappa = 0.0;
            bool   ref_path_ok = query_ref_at(g.ego_x, g.ego_y,
                                              road_c, ref_road_heading, ref_kappa);
            if (!ref_path_ok) {
                road_c = road_center_y(g.ego_x, g.curve_start_x,
                                       g.curve_length_m, g.curve_offset_m);
                ref_road_heading = road_center_heading(g.ego_x, g.curve_start_x,
                                                       g.curve_length_m, g.curve_offset_m);
                ref_kappa = road_center_curvature(g.ego_x, g.curve_start_x,
                                                  g.curve_length_m, g.curve_offset_m);
            }
            /* N 车道模型下的"半路宽"——ego 允许的横向范围。
             * 旧实现用 lane_width - 1.0 等价于"半车道宽 - 1m"，是 2 车道假设下的
             * ROAD_GUARD 触发阈值。N 车道模型下应改为"半路宽 - 1m"。 */
            double half_road = g.lane_count * g.lane_width * 0.5;
            double road_center_limit = half_road - 1.0;
            double half_lane = g.lane_width * 0.5;

            /* ── 车道判定加迟滞: 使用"已提交车道 idx", 只有 ego_y 明确越过当前
             *    车道中心 ±LANE_HYSTERESIS_M 才重算 idx, 避免 y≈车道线处每帧翻转。
             *
             *    N 车道模型：committed_lane_side（实为 committed_lane_idx）
             *    用 lane_idx_from_y 量化 ego_y 到最近车道中心 idx。
             *    EKF 未收敛时（|ego_y - road_c| < 1.0m）保持 idx=-1（未初始化），
             *    cruise_lane_y 退化为 ego_y（横向不动），避免被拉向任意一侧。 */
            if (g.committed_lane_side < 0 && g.ego_x > 0.5) {
                /* 仅在 ego_y 明确偏离道路中心 (EKF 收敛后) 才初始化车道 idx。
                 * fusion_node EKF 初始 y=0, 场景 ego 实际从 y=-1.75 出发；
                 * 收敛前 |ego_y - road_c| < 1.0m，此时不初始化。 */
                if (fabs(g.ego_y - road_c) > 1.0) {
                    g.committed_lane_side = lane_idx_from_y(g.ego_y, g.lane_count,
                                                            g.lane_width, road_c);
                }
                /* 否则保持 -1（未初始化），下方 cruise_lane_y 退化为 ego_y */
            } else if (g.committed_lane_side >= 0) {
                /* 已初始化：用迟滞判定是否切换到相邻车道。
                 * 当前车道中心 y 与 ego_y 的偏差超过 LANE_HYSTERESIS_M 才重算 idx。 */
                int cur_idx = g.committed_lane_side;
                double cur_center = lane_center_y(cur_idx, g.lane_count, g.lane_width, road_c);
                if (fabs(g.ego_y - cur_center) > half_lane) {
                    /* 已越过当前车道边界 → 量化到新车道 */
                    int new_idx = lane_idx_from_y(g.ego_y, g.lane_count, g.lane_width, road_c);
                    if (new_idx != cur_idx) g.committed_lane_side = new_idx;
                }
            }
            /* committed_lane_side<0 (EKF 未收敛) 时, 目标=当前 ego_y, 横向不动,
             * 避免在车道侧未确定前把 ego 拉向任意一侧。 */
            double cruise_lane_y = (g.committed_lane_side < 0)
                                   ? g.ego_y
                                   : lane_center_y(g.committed_lane_side, g.lane_count,
                                                   g.lane_width, road_c);
            /* 相邻车道 y：N 车道下有 N-1 个邻车道，这里默认选"右侧邻车道"（idx+1），
             * 用于被动超车评估。NOA 主动变道由 route_lane 显式指定 idx，
             * 不依赖 adjacent_lane_y 的镜像假设。
             * 最右车道（idx==lane_count-1）无右邻，回退到左邻（idx-1）。
             * lane_lead_gap 等函数仍接收绝对 y，无需改签名。 */
            int adj_idx = g.committed_lane_side + 1;
            if (adj_idx >= g.lane_count) adj_idx = g.committed_lane_side - 1;
            if (adj_idx < 0) adj_idx = 0;  /* 单车道场景 */
            double adjacent_lane_y = (g.committed_lane_side < 0)
                                     ? g.ego_y
                                     : lane_center_y(adj_idx, g.lane_count, g.lane_width, road_c);
            if (fabs(g.ego_y - road_c) > road_center_limit - 0.4) {
                /* 接近路沿 → 强制收敛到最近车道，触发 ROAD_GUARD-style 恢复 */
                g.committed_lane_side = lane_idx_from_y(g.ego_y, g.lane_count,
                                                        g.lane_width, road_c);
                cruise_lane_y = lane_center_y(g.committed_lane_side, g.lane_count,
                                               g.lane_width, road_c);
                adj_idx = g.committed_lane_side + 1;
                if (adj_idx >= g.lane_count) adj_idx = g.committed_lane_side - 1;
                if (adj_idx < 0) adj_idx = 0;
                adjacent_lane_y = lane_center_y(adj_idx, g.lane_count, g.lane_width, road_c);
                g.lc_state = 2;
                g.lc_timer = 0.0;
            }

            /* ── 死锁恢复: 车长时间近乎静止且横向卡在车道线附近 (骑线不动) 时,
             *    强制收敛到最近车道中心并复位变道状态机, 打破 chatter/死锁 ── */
            if (g.current_speed < STUCK_SPEED_MPS && fabs(g.ego_y - road_c) < STUCK_LATERAL_M) {
                g.stuck_timer += CONTROL_DT_S;
            } else {
                g.stuck_timer = 0.0;
            }
            if (g.stuck_timer > STUCK_RECOVER_S) {
                g.committed_lane_side = lane_idx_from_y(g.ego_y, g.lane_count,
                                                        g.lane_width, road_c);
                cruise_lane_y = lane_center_y(g.committed_lane_side, g.lane_count,
                                               g.lane_width, road_c);
                adj_idx = g.committed_lane_side + 1;
                if (adj_idx >= g.lane_count) adj_idx = g.committed_lane_side - 1;
                if (adj_idx < 0) adj_idx = 0;
                adjacent_lane_y = lane_center_y(adj_idx, g.lane_count, g.lane_width, road_c);
                g.lc_state     = 0;
                g.lc_attempted = 0;
                g.lc_cooldown  = 0.0;
                g.lc_timer     = 0.0;
                g.stuck_timer  = 0.0;
                g.speed_zero_timer = 0.0;
                LOG_WARN("control", ">>> STUCK RECOVERY: converge to lane y=%.2f (ego@(%.1f,%.1f))",
                         cruise_lane_y, g.ego_x, g.ego_y);
            }

            /* ── 全域速度死锁恢复: 覆盖 0.6<|y|<2.1 盲区
             *    ROAD_GUARD (|y|>2.1) 自带低速油门; STUCK (|y|<0.6) 由上方处理。
             *    此处捕获中间盲区: 无论 y 值, 只要速度持续为0就计时, 到阈值给小油门打破死锁。 ── */
            if (g.current_speed < STUCK_SPEED_MPS) {
                g.speed_zero_timer += CONTROL_DT_S;
            } else {
                g.speed_zero_timer = 0.0;
            }
            /* ── ACC & 变道: 计算本车道前车间距 ── */
            double best_gap = lane_lead_gap(cruise_lane_y, same_lane_tol);
            double adjacent_gap = lane_lead_gap(adjacent_lane_y, same_lane_tol);
            double lead_speed = lane_lead_speed(cruise_lane_y, same_lane_tol);
            double safe_gap = min_gap + g.current_speed * time_headway;
            double boost_target = fmax(g.target_speed, g.cfg_cruise_speed);
            double acc_target = boost_target;
            int blocked = 0;
            int overtake_worthwhile = 0;
            int overtake_need_accel = 0;
            if (best_gap < safe_gap && best_gap < 80.0) {
                /* 跟车目标速度: gap=0 时匹配前车速度, gap≥safe_gap 时恢复巡航。
                 * 用 lead_speed 作下界替代 ratio*boost_target，避免 ego 减速过头
                 * 导致 gap 反复振荡→速度归零→管道冻住。 */
                double ratio = best_gap / safe_gap;
                if (ratio > 1.0) ratio = 1.0;
                if (ratio < 0.0) ratio = 0.0;
                acc_target = lead_speed + (boost_target - lead_speed) * ratio;
                if (acc_target > boost_target) acc_target = boost_target;
                if (acc_target < boost_target * 0.7) blocked = 1;
            }
            /* 状态机: 跟踪障碍物阻塞状态变化（was_blocked_sm 从函数 static 改为成员） */
            if (blocked && !was_blocked_sm_)
                statem_send_event(&g.sm, CTL_EVENT_OBSTACLE_BLOCKED, NULL);
            else if (!blocked && was_blocked_sm_)
                statem_send_event(&g.sm, CTL_EVENT_OBSTACLE_CLEARED, NULL);
            was_blocked_sm_ = blocked;

            /* 用相对速度 (ego - 前车) 代替绝对速度, 避免高速下 gap 门槛过大而永远无法触发超车。
             * 若前车比 ego 快 (rel_speed<0), 说明并非"追尾慢车", 只需 base gap 即可, 不额外放大门槛。 */
            double rel_speed = g.current_speed - lead_speed;
            if (rel_speed < 0.0) rel_speed = 0.0;
            double min_overtake_gap = g.min_overtake_gap_base + rel_speed * g.min_overtake_gap_speed_mult;
            if (min_overtake_gap > g.min_overtake_gap_cap) min_overtake_gap = g.min_overtake_gap_cap;
            if ((g.lc_state == 0) && (g.lc_cooldown <= 0.0) &&
                (!g.lc_attempted) &&
                best_gap > min_overtake_gap && best_gap < g.min_overtake_gap_cap) {
                int lead_is_slow = lead_speed < boost_target - 2.0;
                int front_allows_merge = lane_front_allows_merge(adjacent_lane_y, same_lane_tol, &overtake_need_accel);
                if (lead_is_slow && !lane_has_pedestrian_risk(adjacent_lane_y, same_lane_tol) &&
                    lane_rear_safe(adjacent_lane_y, same_lane_tol)) {
                    /* 只在目标车道间距足够时立即发起变道（overtake_worthwhile=1）；
                     * 间距不足时让 lc_timer 计时，超时后（blocked_timeout_s）重新评估，
                     * 届时如果目标车道已清空仍能变道。 */
                    if (front_allows_merge) {
                        overtake_worthwhile = 1;
                    }
                    blocked = 1;
                }
            }

            /* ── NOA: 导航路线驱动的主动变道 ──
             * planning 节点仅在 NOA 模式下才会下发 route_lane≥0（见 planning_node.cpp
             * 的 "route_lane=" 字段）；这里与被动超车共用同一套安全检查
             * （rear/front gap、行人风险）和执行状态机，区别只是触发原因不是
             * "前车太慢"而是"导航路线要求换道"（如提前变道以便驶出）。
             *
             * N 车道模型：route_lane 与 committed_lane_side（实为 idx）都是 0..N-1
             * 索引，可以直接比较。-1=无目标。NOA 变道目标 y 用 lane_center_y 算出，
             * 替代旧的 adjacent_lane_y 镜像假设。 */
            int route_triggered = 0;
            double route_target_y = cruise_lane_y;  /* NOA 触发时被覆盖 */
            if (!blocked && g.route_lane >= 0 && g.route_lane != g.committed_lane_side &&
                g.lc_state == 0 && g.lc_cooldown <= 0.0) {
                /* 计算 NOA 目标车道中心 y */
                route_target_y = lane_center_y(g.route_lane, g.lane_count, g.lane_width, road_c);
                if (!lane_has_pedestrian_risk(route_target_y, same_lane_tol) &&
                    lane_rear_safe(route_target_y, same_lane_tol)) {
                    overtake_worthwhile = 1;
                    blocked = 1;
                    route_triggered = 1;
                }
            }

            /* ── 变道等待期: 不减速，维持当前速度准备变道 ── */
            if (blocked && overtake_worthwhile && g.lc_state == 0) {
                if (acc_target < g.current_speed) acc_target = g.current_speed + 0.5;
                if (overtake_need_accel && acc_target < g.current_speed + 2.0) acc_target = g.current_speed + 2.0;
            }

            /* ── 变道中: 临时提高目标速度，加速完成变道 ── */
            if (g.lc_state == 1) {
                boost_target = g.cfg_cruise_speed;
                acc_target = boost_target;
            }

            if (acc_target > g.cfg_cruise_speed) acc_target = g.cfg_cruise_speed;
            if (g.current_speed > g.cfg_cruise_speed + 1.0) acc_target = g.cfg_cruise_speed - 1.0;

            /* ── 自适应变道状态机 ──
             * effective_target_y 钳到路宽 [road_c - half_road, road_c + half_road]，
             * 不再钳到单车道边界（旧 2 车道假设）。N 车道下 ego 可以到达任何车道。 */
            double effective_target_y = (g.lc_state != 0) ? g.lc_target_y : cruise_lane_y;
            if (fabs(g.ego_y - road_c) > road_center_limit - 0.4) {
                /* 接近路沿 → 收敛到最近车道中心（不再硬钳到 ±half_lane） */
                int emerg_idx = lane_idx_from_y(g.ego_y, g.lane_count, g.lane_width, road_c);
                effective_target_y = lane_center_y(emerg_idx, g.lane_count, g.lane_width, road_c);
                if (acc_target > 6.0) acc_target = 6.0;
            }
            if (effective_target_y > road_c + half_road) effective_target_y = road_c + half_road;
            if (effective_target_y < road_c - half_road) effective_target_y = road_c - half_road;

            if (blocked && g.lc_state == 0) {
                g.lc_timer += CONTROL_DT_S;
                if (overtake_worthwhile || g.lc_timer > g.blocked_timeout_s) {
                    int need_accel = 0;
                    /* NOA 路线触发时用 route_target_y；被动超车用 adjacent_lane_y */
                    double lc_target_y_candidate = route_triggered ? route_target_y : adjacent_lane_y;
                    /* 防止逆行：多车道（lane_count>2，即顺行侧≥2 条）场景下，
                     * 禁止 ego 跨过道路中心线 road_c 进入对向车道。
                     *
                     * 背景：flowsim_node 发布的 lane_count 是 esmini 的"双向合计"
                     * 车道数（如 4 车道 = 2 顺行 + 2 对向），而 lane_center_y 用
                     * 中心对称模型把 N 个 idx 对称布置在 road_c 两侧——idx ≥ N/2
                     * 的车道实际上是对向车道（y>road_c）。control_node 此前没有
                     * "不可跨越中心线"约束，导致 ego 被动超车或 NOA 路线变道时
                     * 可能选到 idx 2/3（y>road_c，对向最内/最外），看起来像逆行。
                     *
                     * 2 车道场景（lane_count==2，1 顺 + 1 对）保留旧行为：ego 可以
                     * 借对向车道超车（前方安全时），因为顺行侧只有 1 条车道，不借
                     * 对向就无法超车。多车道场景下顺行侧已有 ≥2 条车道，应只在顺行
                     * 侧内变道，不跨中心线。
                     *
                     * 检查：lane_count>2 且候选 y 在 road_c 对侧（与 ego 不同侧）
                     * 时拒绝变道，进入冷却避免反复尝试。 */
                    int ego_on_negative_side = (g.ego_y < road_c);
                    int candidate_on_opposite_side =
                        (ego_on_negative_side && lc_target_y_candidate > road_c + 0.1) ||
                        (!ego_on_negative_side && lc_target_y_candidate < road_c - 0.1);
                    if (g.lane_count > 2 && candidate_on_opposite_side) {
                        LOG_WARN("control",
                                 ">>> LANE CHANGE REJECTED (oncoming): target_y=%.2f crosses road_c=%.2f "
                                 "into opposite side (lane_count=%d, ego_y=%.2f) — staying on same-direction side",
                                 lc_target_y_candidate, road_c, g.lane_count, g.ego_y);
                        g.lc_timer = g.blocked_timeout_s;  /* 冷却，避免反复尝试 */
                        route_triggered = 0;
                        overtake_worthwhile = 0;
                        blocked = 0;
                    } else {
                    int front_allows_merge = lane_front_allows_merge(lc_target_y_candidate, same_lane_tol, &need_accel);
                    if (front_allows_merge &&
                        !lane_has_pedestrian_risk(lc_target_y_candidate, same_lane_tol) &&
                        lane_rear_safe(lc_target_y_candidate, same_lane_tol)) {
                        g.lc_origin_y = cruise_lane_y;
                        g.lc_target_y = lc_target_y_candidate;
                        g.lc_target_idx = route_triggered ? g.route_lane : adj_idx;
                        effective_target_y = g.lc_target_y;
                        if (need_accel && acc_target < g.current_speed + 2.0) acc_target = g.current_speed + 2.0;
                        g.lc_state = 1; g.lc_attempted = 1; g.lc_timer = 0;
                        statem_send_event(&g.sm, CTL_EVENT_LANE_CHANGE_START, NULL);
                        LOG_INFO("control", ">>> LANE CHANGE %s%s (cur_gap=%.1f adj_gap=%.1f lead_v=%.1f ego@(%.1f,%.1f) target_y=%.1f mode=%s)",
                                 route_triggered ? "NOA_ROUTE" : (overtake_worthwhile ? "OVERTAKE" : "BLOCKED"),
                                 need_accel ? "+ACCEL" : "+CRUISE",
                                 best_gap, adjacent_gap, lead_speed, g.ego_x, g.ego_y, effective_target_y,
                                 g.driving_mode[0] ? g.driving_mode : "?");
                    } else {
                        LOG_INFO("control", ">>> LANE CHANGE BLOCKED by obstacle in target lane");
                        g.lc_timer = g.blocked_timeout_s;
                    }
                    }  /* end else (not oncoming) */
                }
            } else if (!blocked && g.lc_state == 0) {
                g.lc_timer = 0;
            }

            /* 超车后稳定巡航 + 主动评估回原车道。
             * 不强制回原车道（避免回切与慢车重叠），但主动检查原始车道前方是否已清空，
             * 若清空则安全返回，比被动等待 lc_stable_wait_s 秒 + 冷却是更自然的驾驶行为。
             * NOA 路线驱动时（route_triggered 历史），回原车道可能由下一条路线步骤触发，
             * 此处只处理无路线步骤时的自主回切。
             *
             * N 车道模型：original_lane_idx 用变道发起时的 lc_target_idx 推回，
             * 不再用 2.0*road_c - cruise_lane_y 镜像（只在 2 车道对称时正确）。 */
            int original_lane_idx = (g.lc_target_idx >= 0) ? g.lc_target_idx : g.committed_lane_side;
            double original_lane_y = lane_center_y(original_lane_idx, g.lane_count, g.lane_width, road_c);
            if (g.lc_state == 2) {
                g.lc_wait += CONTROL_DT_S;
                if (g.lc_wait > g.lc_stable_wait_s && g.lc_cooldown <= 0.0) {
                    /* NOA 模式中（driving_mode 以 "NOA" 开头），路线步骤会接管变道决策，
                     * 主动回切逻辑不应干预，避免在出口路段提前返回原车道。 */
                    int in_noa_mode = (strncmp(g.driving_mode, "NOA", 3) == 0);
                    if (!in_noa_mode && (g.route_lane < 0 || g.route_lane == g.committed_lane_side)) {
                        /* 无 NOA 路线约束：评估是否可安全返回原始车道。
                         * 条件：原始车道前车间距 > 安全间距的 1.5 倍（说明已清空），
                         * 且后方无风险、无行人风险。 */
                        double orig_gap = lane_lead_gap(original_lane_y, same_lane_tol);
                        double orig_safe = min_gap + g.current_speed * time_headway;
                        int    can_return = (orig_gap > orig_safe * 1.5) &&
                                            !lane_has_pedestrian_risk(original_lane_y, same_lane_tol) &&
                                            lane_rear_safe(original_lane_y, same_lane_tol);
                        if (can_return) {
                            /* 发起回切：设置 lc_state=3 (return) */
                            g.lc_origin_y = cruise_lane_y;
                            g.lc_target_y = original_lane_y;
                            g.lc_target_idx = original_lane_idx;
                            effective_target_y = g.lc_target_y;
                            g.lc_state = 3;
                            g.lc_wait = 0.0;
                            LOG_INFO("control", ">>> RETURN to original lane (orig_gap=%.1f safe=%.1f)", orig_gap, orig_safe);
                        } else {
                            /* 还不安全：重置允许后续超车评估但不回切 */
                            g.lc_state     = 0;
                            g.lc_attempted = 0;
                            g.lc_cooldown  = g.lc_cooldown_after_stable_s;
                            g.lc_wait      = 0.0;
                        }
                    } else {
                        /* NOA 路线要求保留在当前车道（如正在出口车道上），不回切 */
                        g.lc_state     = 0;
                        g.lc_attempted = 0;
                        g.lc_cooldown  = g.lc_cooldown_after_stable_s;
                        g.lc_wait      = 0.0;
                    }
                }
            }

            /* 检测变道完成 (横向偏差 < LC_COMPLETE_THRESH, 收紧防完成后振荡)。
             * N 车道模型：直接 commit lc_target_idx（变道目标 idx）到 committed_lane_side。 */
            if (g.lc_state == 1 && fabs(g.ego_y - effective_target_y) < LC_COMPLETE_THRESH) {
                g.committed_lane_side = (g.lc_target_idx >= 0) ? g.lc_target_idx
                                                                : lane_idx_from_y(g.ego_y, g.lane_count, g.lane_width, road_c);
                g.lc_state = 2; g.lc_wait = 0;
                statem_send_event(&g.sm, CTL_EVENT_LANE_CHANGE_DONE, NULL);
                LOG_INFO("control", ">>> lane change complete");
            }
            if (g.lc_state == 3 && fabs(g.ego_y - effective_target_y) < LC_COMPLETE_THRESH) {
                g.lc_state = 0;
                g.lc_attempted = 0;
                g.lc_cooldown = g.lc_cooldown_after_return_s;
                g.lc_timer = 0.0;
                LOG_INFO("control", ">>> returned to original lane");
            }

            /* ── 纵向 PID 计算 (目标为 ACC 限速后的值) ── */
            double error = acc_target - g.current_speed;
            g.integral += error * 0.05;
            if (g.integral > 500)  g.integral = 500;
            if (g.integral < -200) g.integral = -200;

            double derivative = (error - g.prev_error) / 0.05;
            double output = g.kp * error + g.ki * g.integral + g.kd * derivative;

            double throttle = 0, brake = 0;
            const char* mode;
            if (output > 0) {
                throttle = output / 5000.0;
                if (throttle > 1.0) throttle = 1.0;
                brake = 0;
                mode = (error < 1.0) ? "HOLD" : "ACCEL";
            } else {
                throttle = 0;
                brake = (-output) / 8000.0;
                if (brake > 1.0) brake = 1.0;
                mode = "BRAKE";
            }

            /* Anti-windup: don't accumulate integral when output is saturated */
            if (g.integral > 0 && throttle >= 1.0 && error > 0)
                g.integral -= error * 0.05;  /* unwind positive accumulation */
            if (g.integral < 0 && brake >= 1.0 && error < 0)
                g.integral -= error * 0.05;  /* unwind negative accumulation */

            if (g.current_speed > g.cfg_cruise_speed + 1.0) {
                throttle = 0.0;
                double overspeed_brake = (g.current_speed - g.cfg_cruise_speed - 1.0) / 5.0;
                if (overspeed_brake > brake) brake = overspeed_brake;
                if (brake > 1.0) brake = 1.0;
                g.integral = 0.0;
                mode = "SPEED_LIMIT";
            }

            /* 仅在盲区 (不在 ROAD_GUARD 区域) 激活; ROAD_GUARD 会在后续覆写 throttle/brake。 */
            if (g.speed_zero_timer > SPEED_ZERO_RECOVER_S &&
                fabs(g.ego_y - road_c) <= road_center_limit - 0.4) {
                throttle = 0.15;
                brake    = 0.0;
                mode     = "SPEED_ZERO_RECOVERY";
                g.lc_state     = 0;
                g.lc_attempted = 0;
                g.lc_cooldown  = 0.0;
                g.speed_zero_timer = 0.0;
                LOG_WARN("control", ">>> SPEED_ZERO RECOVERY: throttle bump at y=%.2f (ego@(%.1f,%.1f))",
                         g.ego_y, g.ego_x, g.ego_y);
            }

            /* ── 横向级联 PD：lat_error → psi_des → steer（阻尼消振） ── */
            double steer = 0.0;
            double lat_error = effective_target_y - g.ego_y;
            if (fabs(g.ego_y - road_c) > road_center_limit - 0.4) {
                double steer_limit = steer_limit_for_speed(g.current_speed, 2.4);
                steer = (lat_error > 0.0) ? steer_limit : -steer_limit;
                /* 低速时给少许油门使自行车模型能横向移动回车道中心，
                 * 避免 speed=0 时永久卡在路边缘的死锁。 */
                if (g.current_speed < 2.5) {
                    throttle = 0.18;
                    brake = 0.0;
                    g.speed_zero_timer = 0.0;  /* ROAD_GUARD 自己处理低速, 重置全域计时器 */
                } else {
                    throttle = 0.0;
                    if (brake < 0.65) brake = 0.65;
                }
                g.prev_steer = steer;
                mode = "ROAD_GUARD";
            } else {
                /* ── Stanley 式横向控制（高速自适应阻尼） ──
                 * cross-track 项: atan2(k*e, v) 随速度自然衰减 → 高速小幅打方向;
                 * heading 项: lat_kd_heading 阻尼抑制航向偏差, 避免极限环振荡。
                 * 弯道时以道路中心线切线航向为参考 (road_center_heading)。
                 *
                 * 高速变道时 (>12m/s) 动态调节：
                 *   1) heading 阻尼增益基于配置 lat_kd_heading 缩放 (×0.9), 避免过阻尼
                 *      但仍保持足够阻尼防止蛇行 (之前的硬编码 1.2 低于配置值 1.35, 阻尼不足)
                 *   2) lateral accel 限幅从 2.8 → 2.2 (m/s²) 减少横向冲击
                 *   3) 低通滤波权重从 0.8/0.2 → 0.5/0.5 更激进平滑 */
                double lc_lat_kd = g.lat_kd_heading;
                double lc_lat_accel_max = 1.4;
                double filter_new = STEER_FILTER_NEW;
                /* 变道中或变道后稳定期内使用变道增益 */
                int lc_active = (g.lc_state == 1) ||
                                (g.lc_state == 2 && g.lc_wait < LC_STABILIZE_S);
                if (lc_active) {
                    if (g.current_speed > 12.0) {
                        lc_lat_kd = g.lat_kd_heading * 0.9;   /* 基于配置缩放, 可调 */
                        lc_lat_accel_max = 2.2;               /* 减少横向冲击 */
                    } else {
                        lc_lat_kd = g.lat_kd_heading * 1.2;   /* 低速可稍强 */
                        lc_lat_accel_max = 2.8;
                    }
                    filter_new = 0.5;                          /* 更激进滤波 */
                }
                /* A4: 用上方 query_ref_at 已缓存的 ref_road_heading / ref_kappa
                 * （ref_path 不可用时已是 curve_* 回退值），不再重复算。 */
                /* ref_path heading 合理性检查: esmini 在 junction/fork 处可能返回
                 * 与 ego 当前航向偏差极大的切线方向（如匝道 h≈5 rad 而 ego h≈0）。
                 * 这种 ref_h 会让 heading_term 爆炸, 反向打满 steer 把 ego 拉向岔路。
                 * 检测: 将 (ref_h - ego_heading) 归一化到 [-π,π], 若绝对值 > 0.5 rad
                 * (≈29°) 则视为无效参考, 用 ego_heading 替代 (heading_term=0)。
                 * 阈值比 π/2 更严: esmini 在路口前 100m 采样的 fork 切线常差 60°+,
                 * 但直道/缓弯 ref_h 与 ego 航向差 < 15°, 0.5 rad 能区分两者。
                 * 真正的急弯跟随由下方 ff_term (曲率前馈) 处理, 不依赖 heading_term。 */
                double ref_h_eff = ref_road_heading;
                {
                    double dh = ref_h_eff - g.ego_heading;
                    while (dh >  M_PI) dh -= 2.0 * M_PI;
                    while (dh < -M_PI) dh += 2.0 * M_PI;
                    if (fabs(dh) > 0.5) {
                        ref_h_eff = g.ego_heading;  /* ref_h 与 ego 航向差 > 29°, 视为无效 */
                    }
                }
                double cte_term     = atan2(g.lat_kp * lat_error, fmax(g.current_speed, 3.0));
                double heading_term = lc_lat_kd * (g.ego_heading - ref_h_eff);
                /* yaw_rate 阻尼项：抑制 1.6Hz 极限环振荡（左摇右晃）。
                 * 偏航角速度反映瞬时转向趋势，反向阻尼消除高频摆动。 */
                double yaw_damp_term = g.yaw_damping * g.ego_yaw_rate;

                /* NOA Phase 3.4: 曲率前馈项 δ_ff = wheelbase * κ。
                 * steady-state 转向角，让控制器预先打方向而非等 CTE 累积。
                 * 急弯（R ≤ curve_ff_boost_radius_m）时权重 ×curve_ff_boost_factor，
                 * 匝道 R=45m 回头弯尤其需要——纯反馈在急弯入口总是滞后。
                 * 直道 / 弯道端点外 κ=0，前馈为 0，与原行为完全一致。 */
                double kappa = ref_kappa;
                double ff_weight = 1.0;
                if (fabs(kappa) > 1e-9) {
                    double R = 1.0 / fabs(kappa);  /* 曲率半径 (m) */
                    if (R <= g.curve_ff_boost_radius_m) {
                        ff_weight = g.curve_ff_boost_factor;
                    }
                }
                double ff_term = g.wheelbase * kappa * ff_weight;

                steer = cte_term - heading_term - yaw_damp_term + ff_term;
                double steer_limit = steer_limit_for_speed(g.current_speed, lc_lat_accel_max);
                if (steer >  steer_limit) steer =  steer_limit;
                if (steer < -steer_limit) steer = -steer_limit;
                /* 一阶低通滤波（高速变道时更激进） */
                steer = filter_new * steer + (1.0 - filter_new) * g.prev_steer;
                g.prev_steer = steer;
            }

            /* ── 发布控制指令 (二进制序列化 ControlRaw) ── */
            ControlRaw raw;
            raw.seq      = g.cycle;
            raw.throttle = (float)throttle;
            raw.brake    = (float)brake;
            raw.steering = (float)steer;
            raw.speed    = (float)g.current_speed;
            raw.target   = (float)acc_target;
            raw.error    = (float)error;
            memset(raw.mode, 0, sizeof(raw.mode));
            snprintf(raw.mode, sizeof(raw.mode) - 1, "%s", mode);

            uint8_t raw_buf[64];
            size_t  raw_len = sizeof(raw_buf);
            ControlRaw_serialize(&raw, raw_buf, &raw_len);
            transport_publish(transport_, "control/raw_cmd",
                              raw_buf, (uint32_t)raw_len);

            /* Also publish text format for backward compat (monitor/logging) */
            char cmd_text[256];
            snprintf(cmd_text, sizeof(cmd_text),
                     "throttle=%.2f brake=%.2f steer=%.4f "
                     "speed=%.1f target=%.1f error=%.1f mode=%s",
                     throttle, brake, steer,
                     g.current_speed, acc_target, error, mode);
            transport_publish(transport_, "control/raw_cmd/text",
                              (const uint8_t*)cmd_text, (uint32_t)strlen(cmd_text) + 1);

            /* 发布 CTE（横向误差）供 LDW/监控/数据记录用 */
            {
                cJSON* cte_root = cJSON_CreateObject();
                cJSON_AddNumberToObject(cte_root, "cte", lat_error);
                cJSON_AddNumberToObject(cte_root, "speed", g.current_speed);
                cJSON_AddNumberToObject(cte_root, "seq", g.cycle);
                char* cte_s = cJSON_PrintUnformatted(cte_root);
                transport_publish(transport_, "control/cte",
                                  (const uint8_t*)cte_s, (uint32_t)strlen(cte_s) + 1);
                free(cte_s);
                cJSON_Delete(cte_root);
            }

            /* LDW 车道偏离预警：|cte| 超阈值且速度足够高时告警（带冷却期防刷屏） */
            if (g.current_speed > g.ldw_min_speed && fabs(lat_error) > g.ldw_threshold) {
                double now_s = (double)clock_now_us() * 1e-6;
                if (now_s - g.ldw_last_warn_time > g.ldw_cooldown) {
                    g.ldw_last_warn_time = now_s;
                    const char* side = lat_error > 0 ? "left" : "right";
                    LOG_WARN("control", "LDW: lane departure! cte=%.3fm (threshold=%.3fm) speed=%.1f side=%s",
                             lat_error, g.ldw_threshold, g.current_speed, side);
                    cJSON* ldw_root = cJSON_CreateObject();
                    cJSON_AddNumberToObject(ldw_root, "warn", 1);
                    cJSON_AddNumberToObject(ldw_root, "cte", lat_error);
                    cJSON_AddNumberToObject(ldw_root, "threshold", g.ldw_threshold);
                    cJSON_AddStringToObject(ldw_root, "side", side);
                    char* ldw_s = cJSON_PrintUnformatted(ldw_root);
                    transport_publish(transport_, "control/ldw",
                                      (const uint8_t*)ldw_s, (uint32_t)strlen(ldw_s) + 1);
                    free(ldw_s);
                    cJSON_Delete(ldw_root);
                }
            }

            g.prev_error = error;

            if (g.cycle % 20 == 1) {
                LOG_INFO("control", "#%d spd=%.1f→%.1f err=%.1f thr=%.2f brk=%.2f st=%.4f d=%.2f target_y=%.2f lc=%d %s",
                         g.cycle, g.current_speed, g.target_speed,
                         error, throttle, brake, steer, g.lane_d, effective_target_y, g.lc_state, mode);
            }
        }

        LOG_INFO("control", "stopped (%u cycles, final speed=%.1f m/s)",
                 g.cycle, g.current_speed);
        statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
        statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
        LOG_INFO("control", "state machine: %s", statem_state_name(&g.sm, g.sm.current));
    }

private:
    Transport* transport_;
    int was_blocked_sm_{0};  /* 原 control_thread 内的 static int, 跟踪阻塞状态变化 */
};

/* ── 协程宿主线程 ─────────────────────────────────────────────── */

void* control_thread(void*) {
    try {
        g.task->execute();
    } catch (...) {
        LOG_ERROR("control", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", "vehicle/state", "road/geometry", "road/ref_path", "scene/frame", nullptr };
static const char* s_outputs[] = { "control/raw_cmd", nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾 */

static int control_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    /* 清零并重新初始化（atomic/unique_ptr 不可拷贝，逐字段赋值） */
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.should_stop  = false;

    g.kp = g.ki = g.kd = 0.0;
    g.integral = 0.0;
    g.prev_error = 0.0;
    g.lat_kp = 0.0;
    g.lat_kd_heading = 0.0;
    g.ego_heading = 0.0;
    g.prev_steer = 0.0;

    g.current_speed = 0.0;
    g.target_speed = 0.0;
    g.ego_x = g.ego_y = 0.0;
    g.lane_d = 0.0;
    g.driving_mode[0] = '\0';
    g.route_lane = -1;  /* N 车道模型：-1=无目标 */

    for (int i = 0; i < MAX_OBS; i++) {
        g.obs_x[i] = g.obs_y[i] = g.obs_vx[i] = 0.0;
        g.obs_valid[i] = 0;
        g.obs_type[i][0] = '\0';
    }
    g.ped_index = -1;

    g.has_fusion = 0;
    g.has_planning = 0;
    g.last_fusion_us = 0;
    g.last_planning_us = 0;

    g.lc_state = 0;
    g.lc_attempted = 0;
    g.lc_timer = 0.0;
    g.lc_wait = 0.0;
    g.lc_cooldown = 0.0;
    g.lc_origin_y = 0.0;
    g.lc_target_y = 0.0;
    g.lc_target_idx = -1;
    g.lane_width = 3.5;
    g.lane_count = 2;  /* 默认 2 车道，flowsim 发布 road/geometry 时会按 ego road_id 实时更新 */
    g.blocked_timeout_s = 0.0;

    g.committed_lane_side = -1;  /* N 车道模型：-1=未初始化 */
    g.stuck_timer = 0.0;
    g.speed_zero_timer = 0.0;

    g.cycle = 0;

    g.curve_start_x = 0.0;
    g.curve_length_m = 0.0;
    g.curve_offset_m = 0.0;
    /* NOA Phase 3.4: 弯道前馈权重提升默认参数 */
    g.curve_ff_boost_radius_m = 60.0;
    g.curve_ff_boost_factor   = 1.5;

    /* 默认 PID 参数 */
    g.cfg_kp = 800.0; g.cfg_ki = 50.0; g.cfg_kd = 100.0;
    g.cfg_cruise_speed = 12.0;
    g.wheelbase = CONTROL_WHEELBASE_DEFAULT_M;
    g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
    g.lat_kp          = 0.5;   /* lateral error → desired heading (rad/m), 与 sim 内置一致 */
    g.lat_kd_heading  = 2.0;   /* heading error → steer, 阻尼增益 */
    g.yaw_damping     = 0.15;  /* yaw_rate → steer 阻尼, 抑制 1.6Hz 极限环振荡 */
    g.lane_width = 3.5;
    g.blocked_timeout_s = 1.2;   /* 原 2.0s，缩短阻塞判定等待，更快评估超车可行性 */
    g.lc_stable_wait_s           = 4.0;  /* 原硬编码 8.0，缩短变道后稳定巡航期 */
    g.lc_cooldown_after_stable_s = 1.5;  /* 原硬编码 3.0 */
    g.lc_cooldown_after_return_s = 2.0;  /* 原硬编码 4.0 */
    g.min_overtake_gap_base      = 14.0; /* 原硬编码 18.0 */
    g.min_overtake_gap_cap       = 90.0; /* 原 min_overtake_gap 计算中硬编码的上限 60.0（与 best_gap<90.0 的独立筛选阈值无关），
                                           * 提高上限避免高速场景 min_overtake_gap 被过度收窄导致无法触发超车 */
    g.min_overtake_gap_speed_mult = 0.7; /* 原硬编码 current_speed * 2.0（绝对速度），改为相对速度乘数，避免高速下 gap 永远无法满足 */
    g.steer_min_clamp             = 0.016; /* 原硬编码 0.012，提高最小转向钳位以缩短高速变道耗时 */

    /* 注册参数到 param_registry (类型安全，可运行时热重载) */
    param_register_float("control.pid_kp", g.cfg_kp, 0.0, 5000.0, "PID proportional gain");
    param_register_float("control.pid_ki", g.cfg_ki, 0.0, 1000.0, "PID integral gain");
    param_register_float("control.pid_kd", g.cfg_kd, 0.0, 2000.0, "PID derivative gain");
    param_register_float("control.cruise_speed", g.cfg_cruise_speed, 1.0, 50.0, "Target cruise speed m/s");
    param_register_float("control.lane_width", g.lane_width, 2.5, 5.0, "Lane width meters");
    param_register_float("control.lat_kp", g.lat_kp, 0.0, 2.0, "Lateral P gain");
    param_register_float("control.lat_kd_heading", g.lat_kd_heading, 0.0, 5.0, "Heading damping gain");
    param_register_float("control.yaw_damping", g.yaw_damping, 0.0, 2.0, "Yaw rate damping gain (suppress limit-cycle oscillation)");
    param_register_float("control.blocked_timeout_s", g.blocked_timeout_s, 0.5, 30.0, "Blocked timeout seconds");
    param_register_float("control.lc_stable_wait_s", g.lc_stable_wait_s, 1.0, 30.0, "Post lane-change stable cruise wait seconds");
    param_register_float("control.lc_cooldown_after_stable_s", g.lc_cooldown_after_stable_s, 0.0, 30.0, "Cooldown after stable cruise period");
    param_register_float("control.lc_cooldown_after_return_s", g.lc_cooldown_after_return_s, 0.0, 30.0, "Cooldown after returning to original lane");
    param_register_float("control.min_overtake_gap_base", g.min_overtake_gap_base, 1.0, 100.0, "Base min lead gap to trigger overtake (m)");
    param_register_float("control.min_overtake_gap_cap", g.min_overtake_gap_cap, 1.0, 200.0, "Max clip for min overtake gap at high speed (m)");
    param_register_float("control.min_overtake_gap_speed_mult", g.min_overtake_gap_speed_mult, 0.0, 5.0, "Relative-speed multiplier for min overtake gap formula");
    param_register_float("control.steer_min_clamp", g.steer_min_clamp, 0.001, 0.1, "Minimum steer limit clamp at high speed (rad)");

    /* 运行时从 param_registry 读取 (支持 flowctl param set 热重载) */
    g.kp = param_get_float("control.pid_kp");
    g.ki = param_get_float("control.pid_ki");
    g.kd = param_get_float("control.pid_kd");
    g.cfg_cruise_speed = param_get_float("control.cruise_speed");
    g.lane_width       = param_get_float("control.lane_width");
    g.lat_kp           = param_get_float("control.lat_kp");
    g.lat_kd_heading   = param_get_float("control.lat_kd_heading");
    g.blocked_timeout_s = param_get_float("control.blocked_timeout_s");
    g.lc_stable_wait_s           = param_get_float("control.lc_stable_wait_s");
    g.lc_cooldown_after_stable_s = param_get_float("control.lc_cooldown_after_stable_s");
    g.lc_cooldown_after_return_s = param_get_float("control.lc_cooldown_after_return_s");
    g.min_overtake_gap_base      = param_get_float("control.min_overtake_gap_base");
    g.min_overtake_gap_cap       = param_get_float("control.min_overtake_gap_cap");
    g.min_overtake_gap_speed_mult = param_get_float("control.min_overtake_gap_speed_mult");
    g.steer_min_clamp             = param_get_float("control.steer_min_clamp");
    g.yaw_damping                 = param_get_float("control.yaw_damping");

    /* 初始化反射式状态机 */
    statem_init(&g.sm, g_ctl_transitions, SM_STATE_INITIALIZED, "control");
    statem_send_event(&g.sm, SM_EVENT_START, nullptr);  /* INITIALIZED → RUNNING */

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(p, "pid_kp");
            if (cJSON_IsNumber(j)) g.cfg_kp = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "pid_ki");
            if (cJSON_IsNumber(j)) g.cfg_ki = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "pid_kd");
            if (cJSON_IsNumber(j)) g.cfg_kd = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "target_speed");
            if (cJSON_IsNumber(j)) g.cfg_cruise_speed = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lat_kp");
            if (cJSON_IsNumber(j)) g.lat_kp = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lat_kd_heading");
            if (cJSON_IsNumber(j)) g.lat_kd_heading = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "yaw_damping");
            if (cJSON_IsNumber(j)) g.yaw_damping = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lane_change_blocked_timeout_s");
            if (cJSON_IsNumber(j)) g.blocked_timeout_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lc_stable_wait_s");
            if (cJSON_IsNumber(j)) g.lc_stable_wait_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lc_cooldown_after_stable_s");
            if (cJSON_IsNumber(j)) g.lc_cooldown_after_stable_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "lc_cooldown_after_return_s");
            if (cJSON_IsNumber(j)) g.lc_cooldown_after_return_s = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "min_overtake_gap_base");
            if (cJSON_IsNumber(j)) g.min_overtake_gap_base = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "min_overtake_gap_cap");
            if (cJSON_IsNumber(j)) g.min_overtake_gap_cap = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "min_overtake_gap_speed_mult");
            if (cJSON_IsNumber(j)) g.min_overtake_gap_speed_mult = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "steer_min_clamp");
            if (cJSON_IsNumber(j)) g.steer_min_clamp = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "wheelbase");
            if (cJSON_IsNumber(j)) g.wheelbase = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "ldw_threshold");
            if (cJSON_IsNumber(j)) g.ldw_threshold = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "ldw_min_speed");
            if (cJSON_IsNumber(j)) g.ldw_min_speed = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "ldw_cooldown");
            if (cJSON_IsNumber(j)) g.ldw_cooldown = j->valuedouble;
            /* NOA Phase 3.4: 弯道前馈权重提升参数 */
            j = cJSON_GetObjectItemCaseSensitive(p, "curve_ff_boost_radius_m");
            if (cJSON_IsNumber(j)) g.curve_ff_boost_radius_m = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "curve_ff_boost_factor");
            if (cJSON_IsNumber(j)) g.curve_ff_boost_factor = j->valuedouble;
            cJSON_Delete(p);
            g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
        }
    }

    /* Phase 2: 道路几何从 road/geometry topic 获取（sim_world 发布），
     * 不再独立 scenario_load。 */

    transport_subscribe(transport, "fusion/localization", on_fusion, nullptr);
    transport_subscribe(transport, "planning/trajectory", on_trajectory, nullptr);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, nullptr);
    transport_subscribe(transport, "road/geometry", on_road_geometry, nullptr);
    transport_subscribe(transport, "road/ref_path", on_ref_path, nullptr);
    transport_subscribe(transport, "scene/frame", on_scene_frame, nullptr);
    transport_advertise(transport, "control/raw_cmd", CONTROLRAW_TYPE_ID);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/geometry", 0x80AD5C12u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/ref_path", 0x7E5A3C11u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "scene/frame",         0x5CF12A60u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/raw_cmd", CONTROLRAW_TYPE_ID,
                        CAP_PUBLISHER, 100.0);

    g.task = std::make_unique<ControlTask>(bus, transport);

    LOG_INFO("control", "initialized (FlowCoro, PID: kp=%.0f ki=%.0f kd=%.0f)",
             g.kp, g.ki, g.kd);
    return 0;
}

static int control_start(void) {
    if (!g.task) return -1;
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, control_thread, nullptr) != 0) {
        LOG_WARN("control", "pthread_create failed: %s", strerror(errno));
        return -1;
    }
    g.running = true;
    LOG_INFO("control", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void control_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->stop();
}

static void control_cleanup(void) {
    control_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
    statem_cleanup(&g.sm);
    LOG_INFO("control", "cleanup done");
}

static int control_health(void) { return 0; }

/* ── 导出入口 ────────────────────────────────────────────────── */

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "control",
    "1.0.0",
    "PID longitudinal controller + ACC [FlowCoro]",
    s_inputs,
    s_outputs,
    control_init,
    control_start,
    control_stop,
    control_cleanup,
    control_health,
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
