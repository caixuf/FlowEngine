/**
 * behavior_planner_node.cpp — 行为规划节点 (FlowCoro 协程版)
 *
 * 从 planning_node 中拆出独立的行为决策层。职责：
 *   - 消费感知语义输出（perception/obstacles, perception/lanes）
 *   - 运行行为状态机（巡航/跟车/变道/停车/让行）
 *   - 发布 planning/behavior 指令，下游 planning_node 据此生成轨迹
 *
 * 架构意义：行为规划与轨迹规划解耦。行为规划看的是"前方 3s 的语义场景"，
 * 轨迹规划看的是"接下来 2s 的路径形状"。两者频率可以不同，也可以独立
 * 做降级（行为规划退化→保持当前车道巡航，轨迹规划退化→直道恒速）。
 *
 * 输入: perception/obstacles, fusion/localization, road/geometry
 * 输出: planning/behavior
 *
 * NodePlugin 接口，编译为 libbehavior_planner_node.so。
 */
#include "node_plugin.h"
#include "topic_registry.h"
#include "coroutine_task.h"
#include "adas_msgs_gen.h"
#include "road_geometry.h"
#include "traffic_light.h"
#include "state_machine.h"
#include "param_registry.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

namespace {

/* ── 行为状态机状态/事件 ID ───────────────────────────── */
/* 使用 SM_EVENT_USER_BASE+ 区域，避免与内置事件冲突 */

enum BehState {
    BEH_ST_CRUISE       = 200,
    BEH_ST_FOLLOW       = 201,
    BEH_ST_LEFT_CHANGE  = 202,
    BEH_ST_RIGHT_CHANGE = 203,
    BEH_ST_STOP         = 204,
    BEH_ST_YIELD        = 205,
    BEH_ST_EMERGENCY    = 206,
    BEH_ST_U_TURN       = 207,
};

enum BehEvent {
    BEH_EV_BLOCKED        = 200,  /* 本车道有前车且暂无变道条件 */
    BEH_EV_LOST_LEAD      = 201,  /* 前车消失或间距充足 */
    BEH_EV_OVERTAKE_LEFT  = 202,  /* 条件满足→向左变道 */
    BEH_EV_OVERTAKE_RIGHT = 203,  /* 条件满足→向右变道 */
    BEH_EV_COMPLETED      = 204,  /* 变道完成 */
    BEH_EV_TIMEOUT        = 205,  /* 变道超时回退 */
    BEH_EV_UTURN_TRIGGER  = 206,  /* 到达路端 → 开始掉头 */
};

/* ── 行为状态机转移表 ───────────────────────────────── */
static const TransitionRule BEH_TRANSITIONS[] = {
    /* 巡航 → 跟车 */
    { BEH_ST_CRUISE, BEH_EV_BLOCKED,        BEH_ST_FOLLOW,       "CRUISE + BLOCKED -> FOLLOW",       false },
    /* 巡航 → 变道（有变道条件时直接跳巡航→变道） */
    { BEH_ST_CRUISE, BEH_EV_OVERTAKE_LEFT,  BEH_ST_LEFT_CHANGE,  "CRUISE + OVERTAKE_LEFT -> LEFT",   false },
    { BEH_ST_CRUISE, BEH_EV_OVERTAKE_RIGHT, BEH_ST_RIGHT_CHANGE, "CRUISE + OVERTAKE_RIGHT -> RIGHT", false },
    /* 跟车 → 巡航（前车消失） */
    { BEH_ST_FOLLOW, BEH_EV_LOST_LEAD,      BEH_ST_CRUISE,       "FOLLOW + LOST_LEAD -> CRUISE",     false },
    /* 跟车 → 变道 */
    { BEH_ST_FOLLOW, BEH_EV_OVERTAKE_LEFT,  BEH_ST_LEFT_CHANGE,  "FOLLOW + OVERTAKE_LEFT -> LEFT",   false },
    { BEH_ST_FOLLOW, BEH_EV_OVERTAKE_RIGHT, BEH_ST_RIGHT_CHANGE, "FOLLOW + OVERTAKE_RIGHT -> RIGHT", false },
    /* 变道 → 巡航（完成或超时回退） */
    { BEH_ST_LEFT_CHANGE,  BEH_EV_COMPLETED, BEH_ST_CRUISE,      "LEFT + COMPLETED -> CRUISE",       false },
    { BEH_ST_LEFT_CHANGE,  BEH_EV_TIMEOUT,   BEH_ST_CRUISE,      "LEFT + TIMEOUT -> CRUISE",         false },
    { BEH_ST_RIGHT_CHANGE, BEH_EV_COMPLETED, BEH_ST_CRUISE,      "RIGHT + COMPLETED -> CRUISE",      false },
    { BEH_ST_RIGHT_CHANGE, BEH_EV_TIMEOUT,   BEH_ST_CRUISE,      "RIGHT + TIMEOUT -> CRUISE",        false },
    /* 巡航 → 掉头（到达路端） */
    { BEH_ST_CRUISE, BEH_EV_UTURN_TRIGGER, BEH_ST_U_TURN, "CRUISE + UTURN_TRIGGER -> U_TURN", false },
    /* 跟停/停车/让行 → 掉头：路端是硬约束。旧实现只有 CRUISE 能触发掉头，
     * 前车堵在路端 → FOLLOW blocked 卡死 → 永不掉头 → 撞"墙"（2026-08-03
     * 实测：best_gap=2.9 lead=0 FOLLOW v=0 卡死在路端）。 */
    { BEH_ST_FOLLOW, BEH_EV_UTURN_TRIGGER, BEH_ST_U_TURN, "FOLLOW + UTURN_TRIGGER -> U_TURN", false },
    { BEH_ST_STOP,   BEH_EV_UTURN_TRIGGER, BEH_ST_U_TURN, "STOP + UTURN_TRIGGER -> U_TURN",   false },
    { BEH_ST_YIELD,  BEH_EV_UTURN_TRIGGER, BEH_ST_U_TURN, "YIELD + UTURN_TRIGGER -> U_TURN",  false },
    /* 掉头 → 巡航（完成或超时） */
    { BEH_ST_U_TURN, BEH_EV_COMPLETED, BEH_ST_CRUISE, "U_TURN + COMPLETED -> CRUISE", false },
    { BEH_ST_U_TURN, BEH_EV_TIMEOUT,   BEH_ST_CRUISE, "U_TURN + TIMEOUT -> CRUISE",   false },
    TRANSITION_TABLE_END,
};

/* ── 节点本地状态 ───────────────────────────────────────────── */

#define BEH_MAX_OBS 128
/* TL_CACHE_MAX 来自 traffic_light.h（共享红绿灯解析），不在此重复定义 */

struct BehaviorContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler*        scheduler{nullptr};

    /* 发布帧计数 */
    uint32_t seq{0};

    /* ego 状态（从 fusion/localization 或 vehicle/state JSON 解析）。
     * vehicle/state 是 flowsim 真值，优先级高于 fusion/localization（EKF 估计）。
     * 当 vehicle/state 近期到达（<200ms）时，on_fusion 不覆盖 ego 状态，
     * 避免 EKF 发散时用错误值覆盖真值。 */
    double ego_x{0}, ego_y{0}, ego_v{0}, ego_heading{0};
    volatile int has_fusion{0};
    uint64_t last_fusion_us{0};
    uint64_t last_vstate_us{0};

    /* 障碍物缓存（从 perception/obstacles 反序列化，世界坐标） */
    double obs_x[BEH_MAX_OBS]{}, obs_y[BEH_MAX_OBS]{};
    double obs_vx[BEH_MAX_OBS]{}, obs_vy[BEH_MAX_OBS]{};
    int8_t obs_lane_id[BEH_MAX_OBS]{};
    uint8_t obs_type[BEH_MAX_OBS]{};
    uint32_t obs_id[BEH_MAX_OBS]{};
    int obs_count{0};
    volatile int has_obs{0};

    /* 道路几何（从 road/geometry JSON 解析） */
    int    lane_count{2};
    double lane_width{3.5};
    volatile int has_road_geometry{0};

    /* 红绿灯缓存（road/traffic_lights 解析走 traffic_light.h 的共享
     * traffic_lights_parse()，此处只存消费视图，TL_CACHE_MAX 即该头
     * 定义的 16）。归位（超车后切回内侧道）前要确认目标车道前方没有
     * 红灯，否则切回去立刻停在灯前——2026-07-31 实跑：RIGHT 超车后
     * CRUISE 归位回 lane2，距 x=350 红灯仅 ~15m，当场刹停。 */
    double tl_x[TL_CACHE_MAX]{}, tl_y_lane[TL_CACHE_MAX]{};
    int    tl_state[TL_CACHE_MAX]{};  /* 0=green 1=yellow 2=red */
    int    tl_count{0};
    volatile int has_traffic_lights{0};

    /* 行进方向：flowsim road/ref_path.reverse（掉头返程=1）。返程时几何镜像，
     * 前进向的变道/归位逻辑（merge back、overtake 的左右/gap）全部失配，会误发
     * OVERTAKE_LEFT 把 ego 从对向内道拽到对向外道（y=5.25）绕圈。返程一律纯
     * 车道保持（抑制所有变道事件），纵向仍正常 CRUISE/FOLLOW。 */
    volatile int on_return{0};

    /* ── 行为状态机状态 ── */
    int    state{0};        /* BehaviorCommand enum (镜像 sm.current) */
    double state_timer{0};  /* 当前状态持续秒数 */
    double cooldown{0};     /* 变道冷却秒数 */

    /* 框架反射式状态机 */
    ReflectiveStateMachine sm{};

    /* 当前所在车道（由 committed_lane_idx 跟踪） */
    int committed_lane_idx{0};

    /* 变道目标 */
    int    target_lane_idx{-1};
    double target_speed{10.0};      /* 当前目标速度（FOLLOW会降低，CRUISE应恢复） */
    double cfg_cruise_speed{15.0};  /* 巡航速度基准（永不被FOLLOW降低，CRUISE恢复用） */
    uint32_t follow_obs_id{0};

    /* 超车参数 */
    double min_overtake_gap_base{25.0};
    double min_overtake_gap_cap{90.0};
    double min_overtake_gap_speed_mult{2.0};

    /* ── ACC 常量时距（CTG）跟车律参数 ────────────────────────
     * 期望间距 desired_gap = acc_standoff + acc_time_headway * ego_v
     * 目标速度 v_target    = lead_speed + acc_k_gap * (gap - desired_gap)
     *
     * 原实现 `new_target_speed = lead_speed` 是纯速度跟随，**间距开环**：
     * 从 12 m/s 减到前车的 7 m/s 需要时间，这期间 ego 一直在接近；到达
     * 同速时 gap 已被吃掉一大截，随后冻结在那个偶然值，没有任何项把它
     * 推回安全距离。前车再轻微减速就追尾 —— min_forward_gap 在 -0.69m
     * 和 4.66m 之间随机漂移正是这个开环的表现，撞不撞取决于运气。
     *
     * CTG 律的关键性质：gap < desired 时 v_target **低于**前车速度，
     * 主动拉开距离；稳态收敛到 gap == desired_gap，是闭环的。 */
    double acc_standoff{5.0};       /* 静止安全余量 (m) */
    double acc_time_headway{1.5};   /* 时距 (s)，与 safety_control 的 1.3 留余量 */
    double acc_k_gap{0.4};          /* 间距误差增益 (1/s) */
    double acc_gap_err_clamp{8.0};  /* 间距误差对目标速度的修正上限 (m/s)，
                                     * 防止远距离时目标速度被抬到超速 */

    /* ── 变道/跟车决策阈值（全部可热调） ── */
    double blocked_range_mult{3.5};     /* blocked 检测距离 = max(min_m, desired_gap * mult)
                                         * 8.0 时 15m/s 下 blocked_range≈220m——前车还在
                                         * 119m 外就判定"堵车"并变道（实测启动 2s 即变道，
                                         * 之后长期滞留外侧道错过红绿灯）。3.5 时 ≈96m，
                                         * 配合 min_gap≈41m，变道发生在合理接近窗口。 */
    double blocked_range_min{30.0};     /* blocked 检测最小距离 (m) */
    double follow_hysteresis{1.3};      /* FOLLOW→CRUISE 退出滞环倍数（进入紧退出松） */
    double lane_change_timeout_s{8.0};  /* 变道超时 (s)，超时回退 CRUISE */
    double lane_change_cooldown_s{3.0}; /* 变道完成冷却 (s) */
    double lane_change_cooldown_timeout_s{5.0}; /* 变道超时后冷却 (s) */
    double lc_gap_mult{1.5};            /* 目标车道前车间距阈值 = min_gap * mult */
    double rear_safe_min_m{15.0};       /* 后向安全最小距离 (m) */
    double rear_safe_time_s{3.0};       /* 后向安全时距 (s) */
    double same_lane_tol_offset{0.6};   /* 车道归属横向容差偏移 (m)，半车道宽 + offset */

    /* ── 掉头触发参数 ── */
    double uturn_approach_dist_m{120.0};  /* 距路端此距离触发掉头 (m)，含刹车距离。
                                           * 60m 不够：ref_path_end_x 是 ego 前方 100m
                                           * 动态采样点，触发条件在未封顶时永不满足，
                                           * 实际触发点 = 路端封顶后 -60m → 车 20 m/s
                                           * 只给 60m 刹车（需 ~21m 减到 8）+ 施工占位
                                           * → 冲到施工面前才触发 → 卡死（2026-08-03
                                           * 实测触发点 2950，施工前缘 2970）。
                                           * 120m：触发提前 → 减速从容 → 施工前完成。 */
    double uturn_max_trigger_speed{8.0};  /* 掉头触发速度上限 (m/s)：高于此先减速
                                           * 再触发，避免 64 点轨迹被 Phase 1 刹车截断 */
    double uturn_timeout_s{15.0};         /* 掉头超时 (s)，超时回退 CRUISE */
    double ref_path_end_x{1e9};           /* 参考路径终点 x（从 road/ref_path 解析，1e9=未初始化） */
    double road_end_x{1e9};               /* 路端 x（flowsim 权威 route 总长，掉头触发固定参考） */

    /* TaskBase 包装器（由 EXPORT_COROUTINE_TASK 宏创建） */
    struct behavior_Wrapper* task_wrapper{nullptr};

    /* 调试计数：每 50 帧打印一次全景状态 */
    int dbg_count{0};
};

BehaviorContext g;

/* ── 状态机 guard ── */
static bool beh_guard(void* task, StateId from, EventId event, StateId to) {
    (void)task; (void)from; (void)event; (void)to;
    /* 所有转移都已在事件选择阶段验证条件，guard 仅作最终安全检查 */
    return true;
}

/* ── 状态 → BehaviorCommand 枚举 ── */
static int beh_state_to_cmd(StateId s) {
    if (s == BEH_ST_FOLLOW)       return BEH_FOLLOW;
    if (s == BEH_ST_LEFT_CHANGE)  return BEH_LEFT_CHANGE;
    if (s == BEH_ST_RIGHT_CHANGE) return BEH_RIGHT_CHANGE;
    if (s == BEH_ST_STOP)         return BEH_STOP;
    if (s == BEH_ST_YIELD)        return BEH_YIELD;
    if (s == BEH_ST_EMERGENCY)    return BEH_EMERGENCY;
    if (s == BEH_ST_U_TURN)       return BEH_U_TURN;
    return BEH_CRUISE;
}

/* ── StateId → 可读名称（框架 statem_state_name 不识自定义 ID） ── */
static const char* beh_state_str(StateId s) {
    switch (s) {
        case BEH_ST_CRUISE:       return "CRUISE";
        case BEH_ST_FOLLOW:       return "FOLLOW";
        case BEH_ST_LEFT_CHANGE:  return "LEFT_CHANGE";
        case BEH_ST_RIGHT_CHANGE: return "RIGHT_CHANGE";
        case BEH_ST_STOP:         return "STOP";
        case BEH_ST_YIELD:        return "YIELD";
        case BEH_ST_EMERGENCY:    return "EMERGENCY";
        case BEH_ST_U_TURN:       return "U_TURN";
        default:                  return "?";
    }
}

/* ── EventId → 可读名称（框架 statem_event_name 不识自定义 ID） ── */
static const char* beh_event_str(EventId ev) {
    switch (ev) {
        case BEH_EV_BLOCKED:        return "BLOCKED";
        case BEH_EV_LOST_LEAD:      return "LOST_LEAD";
        case BEH_EV_OVERTAKE_LEFT:  return "OVERTAKE_LEFT";
        case BEH_EV_OVERTAKE_RIGHT: return "OVERTAKE_RIGHT";
        case BEH_EV_COMPLETED:      return "COMPLETED";
        case BEH_EV_TIMEOUT:        return "TIMEOUT";
        case BEH_EV_UTURN_TRIGGER:  return "UTURN_TRIGGER";
        default:                    return "?";
    }
}

/* ── debug hook ── */
static void beh_debug_hook(void* task, StateId from, EventId event,
                           StateId to, const char* rule_desc, bool accepted) {
    (void)task; (void)from; (void)event; (void)to;
    if (accepted && rule_desc) {
        LOG_INFO("behavior", "[BEH] %s", rule_desc);
    }
}

/* ── fusion/localization 订阅 ────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    /* vehicle/state 近期到达时不覆盖：它是 flowsim 真值，优先于 EKF 估计。
     * EKF 发散时会发布错误的 ego_y/v（如 y=0.01 v=20.94），若覆盖 vehicle/state
     * 的正确值（y=-1.75 v=15.1），会导致车道判定错误 → find_lead 找错车道
     * → 漏检正前方前车 → 不减速 → 追尾。 */
    uint64_t now = clock_now_us();
    bool vstate_recent = (g.last_vstate_us != 0 && now - g.last_vstate_us < 200000ULL);
    if (!vstate_recent) {
        cJSON* j;
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
            g.ego_x = j->valuedouble;
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
            g.ego_y = j->valuedouble;
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "v")) && cJSON_IsNumber(j))
            g.ego_v = j->valuedouble;
        if ((j = cJSON_GetObjectItemCaseSensitive(root, "heading")) && cJSON_IsNumber(j))
            g.ego_heading = j->valuedouble;
    }
    g.last_fusion_us = now;
    g.has_fusion = 1;
    cJSON_Delete(root);
}
/* ── vehicle/state 订阅 ── 用 flowsim 真值覆盖 ego 位置 */
static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "spd")) && cJSON_IsNumber(j))
        g.ego_v = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "hdg")) && cJSON_IsNumber(j))
        g.ego_heading = j->valuedouble;
    g.last_vstate_us = clock_now_us();
    g.has_fusion = 1;
    cJSON_Delete(root);
}


/* ── perception/tracked_objects 订阅（JSON，带 tracking） ──
 * 仅当 objects 数组非空时覆盖 obs 缓存；空数组或字段缺失时保留 on_raw_obstacles
 * 填充的 raw 数据作为回退，避免 tracker 偶发空帧清空障碍物导致 FOLLOW 丢失。 */
static void on_tracked_objects(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    cJSON* objects = cJSON_GetObjectItemCaseSensitive(root, "objects");
    if (objects && cJSON_IsArray(objects)) {
        int n = cJSON_GetArraySize(objects);
        if (n > 0) {
            int lc = g.has_road_geometry ? g.lane_count : 2;
            double lw = g.has_road_geometry ? g.lane_width : 3.5;
            double ch = cos(g.ego_heading), sh = sin(g.ego_heading);
            if (n > BEH_MAX_OBS) n = BEH_MAX_OBS;
            g.obs_count = 0;

            for (int i = 0; i < n; i++) {
                cJSON* obj = cJSON_GetArrayItem(objects, i);
                if (!obj) continue;
                cJSON* j;

                /* 车体系坐标 */
                double ox = 0.0, oy = 0.0;
                if ((j = cJSON_GetObjectItemCaseSensitive(obj, "x")) && cJSON_IsNumber(j)) ox = j->valuedouble;
                if ((j = cJSON_GetObjectItemCaseSensitive(obj, "y")) && cJSON_IsNumber(j)) oy = j->valuedouble;

                /* 车体 → 世界 */
                g.obs_x[i] = g.ego_x + ox * ch - oy * sh;
                g.obs_y[i] = g.ego_y + ox * sh + oy * ch;

                /* 速度（车体 → 世界） */
                double vx = 0.0, vy = 0.0;
                if ((j = cJSON_GetObjectItemCaseSensitive(obj, "vx")) && cJSON_IsNumber(j)) vx = j->valuedouble;
                if ((j = cJSON_GetObjectItemCaseSensitive(obj, "vy")) && cJSON_IsNumber(j)) vy = j->valuedouble;
                g.obs_vx[i] = vx * ch - vy * sh;
                g.obs_vy[i] = vx * sh + vy * ch;

                /* 类型 */
                if ((j = cJSON_GetObjectItemCaseSensitive(obj, "type")) && cJSON_IsString(j)) {
                    const char* t = j->valuestring;
                    if (strcmp(t, "VEHICLE") == 0)   g.obs_type[i] = 1;
                    else if (strcmp(t, "PEDESTRIAN") == 0) g.obs_type[i] = 2;
                    else if (strcmp(t, "CYCLIST") == 0)    g.obs_type[i] = 3;
                    else g.obs_type[i] = 0;
                }

                /* ID */
                g.obs_id[i] = 0;
                if ((j = cJSON_GetObjectItemCaseSensitive(obj, "id")) && cJSON_IsNumber(j))
                    g.obs_id[i] = (uint32_t)j->valuedouble;

                /* lane_id：从世界系 y 计算 */
                {
                    double wy = g.obs_y[i];
                    double offset = (-wy) / lw + (lc - 1) * 0.5;
                    int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
                    if (idx < 0) idx = 0;
                    if (idx >= lc) idx = lc - 1;
                    g.obs_lane_id[i] = (int8_t)idx;
                }
                g.obs_count++;
            }
            g.has_obs = 1;
        }
    }
    cJSON_Delete(root);
}

/* ── perception/obstacles 直连回退（每帧无条件填充 raw 数据） ──
 * 回调顺序保证：perception_node 先发布 raw obstacles → object_tracker 订阅后
 * 发布 tracked_objects。因此 on_raw_obstacles 先执行，on_tracked_objects 后
 * 执行。若 tracker 正常产出 tracked 数据，会在本回调之后覆盖 obs 缓存；若 tracker
 * 停更/未就绪，本回调填充的 raw 数据保持最新。
 *
 * 之前的 if(g.obs_count == 0) 守卫存在致命问题：tracker 第一帧发布后停止产出，
 * g.obs_count 停留在非零值，raw 数据永远无法刷新，障碍物坐标冻结在第一帧。
 * 随着 ego 前进，前车 dx 变为负数（落在后方），best_gap 永远 1e9 → 永不进入
 * FOLLOW，表现为"全速冲向前车直到追尾"。 */
static void on_raw_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */

    ObstacleList obs_list;
    if (ObstacleList_deserialize(&obs_list, (const uint8_t*)msg->data, msg->data_size) != 0)
        return;
    if (obs_list.count == 0) return;

    int lc = g.has_road_geometry ? g.lane_count : 2;
    double lw = g.has_road_geometry ? g.lane_width : 3.5;
    double ch = cos(g.ego_heading), sh = sin(g.ego_heading);
    int n = obs_list.count < BEH_MAX_OBS ? (int)obs_list.count : BEH_MAX_OBS;
    g.obs_count = 0;
    for (int i = 0; i < n; i++) {
        const Obstacle* o = &obs_list.obstacles[i];
        /* 车体系 → 世界系（位置和速度都需要旋转） */
        g.obs_x[i] = g.ego_x + (double)o->x * ch - (double)o->y * sh;
        g.obs_y[i] = g.ego_y + (double)o->x * sh + (double)o->y * ch;
        g.obs_vx[i] = (double)o->vx * ch - (double)o->vy * sh;
        g.obs_vy[i] = (double)o->vx * sh + (double)o->vy * ch;
        g.obs_type[i] = (uint8_t)o->type;

        /* DBG: 346257217 100 3452702473462112233452152603452112153344270252351232234347244231347211251 body=world 345235220346240207345217230346215242357274214347241256350256244344274240346204237346225260346215256345215217350256256 */
        if (i < 3 && g.seq % 100 == 0) {
            LOG_WARN("behavior", "[DBG_OBS] #%d body(x=%.1f y=%.1f vx=%.1f vy=%.1f) "
                     "ego(x=%.1f y=%.1f h=%.3f) world(x=%.1f y=%.1f vx=%.1f) "
                     "type=%d id=%u count=%d",
                     i, (double)o->x, (double)o->y, (double)o->vx, (double)o->vy,
                     g.ego_x, g.ego_y, g.ego_heading,
                     g.obs_x[i], g.obs_y[i], g.obs_vx[i],
                     o->type, o->id, n);
        }
        g.obs_id[i] = o->id;
        /* lane_id */
        {
            double wy = g.obs_y[i];
            double offset = (-wy) / lw + (lc - 1) * 0.5;
            int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
            if (idx < 0) idx = 0;
            if (idx >= lc) idx = lc - 1;
            g.obs_lane_id[i] = (int8_t)idx;
        }
        g.obs_count++;
    }
    g.has_obs = 1;
}

/* ── road/geometry 订阅 ────────────────────────────────── */

static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* item;
    if ((item = cJSON_GetObjectItem(root, "lane_count"))) g.lane_count = item->valueint;
    if ((item = cJSON_GetObjectItem(root, "lane_width"))) g.lane_width = item->valuedouble;
    g.has_road_geometry = 1;
    cJSON_Delete(root);
}

/* ── road/traffic_lights 订阅 — 缓存红绿灯位置/状态，供归位决策 ──
 * JSON 解析统一走 traffic_light.h 的 traffic_lights_parse()（共享），
 * 本节点只把缓存拷进自己的 tl_* 消费视图。 */
static void on_traffic_lights(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    TrafficLightCache c;
    traffic_lights_parse((const char*)msg->data, &c);
    g.tl_count = c.count;
    g.has_traffic_lights = c.valid;
    for (int i = 0; i < c.count; i++) {
        g.tl_x[i]      = c.x[i];
        g.tl_y_lane[i] = c.y_lane[i];
        g.tl_state[i]  = c.state[i];
    }
}

/* ── road/ref_path 订阅 — 消费 flowsim 权威行进方向标志 reverse ──
 * reverse=true（掉头返程）时，本节点切纯车道保持，抑制所有变道决策。
 * 同时解析最后一个点的 x 坐标作为路端参考，用于掉头触发判定。 */
static void on_ref_path(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* jr = cJSON_GetObjectItemCaseSensitive(root, "reverse");
    if (cJSON_IsBool(jr)) g.on_return = cJSON_IsTrue(jr) ? 1 : 0;
    /* 路端位置（flowsim 权威，route 总长）：掉头触发固定参考，替代
     * 采样末点封顶检测（低速起步误判 → 启动即掉头逆行，2026-08-03） */
    cJSON* jend = cJSON_GetObjectItemCaseSensitive(root, "road_end_x");
    if (cJSON_IsNumber(jend) && jend->valuedouble > 1.0) {
        g.road_end_x = jend->valuedouble;
    }
    /* 解析参考路径终点 x：路端判定用 */
    cJSON* pts = cJSON_GetObjectItemCaseSensitive(root, "points");
    if (cJSON_IsArray(pts)) {
        int n = cJSON_GetArraySize(pts);
        if (n > 0) {
            cJSON* last = cJSON_GetArrayItem(pts, n - 1);
            if (last) {
                cJSON* jx = cJSON_GetObjectItemCaseSensitive(last, "x");
                if (cJSON_IsNumber(jx)) g.ref_path_end_x = jx->valuedouble;
                /* 封顶检测已废弃（2026-08-03）：低速起步误判 → 启动即掉头。
                 * 路端判定改用 flowsim 权威 road_end_x 字段（见触发处）。 */
            }
        }
    }
    cJSON_Delete(root);
}

/* 归位目标车道（idx-1）前方 stop_range 内是否有非绿灯？有则不归位。 */
static bool lane_ahead_stop_light(int lane_idx, int lc, double lw) {
    if (!g.has_traffic_lights || g.tl_count <= 0) return false;
    double lane_c = lane_center_y(lane_idx, lc, lw, 0.0, 0.0);
    double v = g.ego_v; if (v < 0.0) v = 0.0;
    /* 刹车距离 base = v²/8 + 3（≈4m/s² 减速度 + 3m 余量），与 planning_node
     * 红灯 override（brake_dist = v*v/8 + 3，floor 5m）同源，此处再加 20m
     * 余量、下限 60m：距红灯 60m 内都算"马上要停"，归位回去毫无意义。
     * 若改 base 公式，planning_node.cpp 的红灯 override 需同步。 */
    double stop_range = v * v / 8.0 + 3.0 + 20.0;
    if (stop_range < 60.0) stop_range = 60.0;
    for (int i = 0; i < g.tl_count; i++) {
        if (g.tl_state[i] == 0) continue;  /* 绿灯 */
        if (fabs(g.tl_y_lane[i] - lane_c) > lw * 0.5) continue;  /* 只查目标车道 */
        double dx = g.tl_x[i] - g.ego_x;
        if (dx <= 0.0 || dx > stop_range) continue;
        return true;
    }
    return false;
}

/* ── 协程任务 ──────────────────────────────────────────── */

class BehaviorTask : public CoroutineTask {
public:
    BehaviorTask(MessageBus* bus) : CoroutineTask(bus) {}

    void set_params(Transport* transport) {
        transport_ = transport;
    }

protected:
    Task run() override {
        LOG_INFO("behavior", "FlowCoro behavior planner started (20Hz)");

        while (!should_stop()) {
            uint64_t t_start = clock_now_us();

            /* 等待感知数据就绪 */
            if (!g.has_fusion || !g.has_obs) {
                g.seq++;
                co_await sleep_us(50000);  /* 未就绪也必须让出，否则忙自旋饿死全管道 */
                continue;
            }

            /* 参数热重载（三处之三）：漏了这步，注册了也改不动，只能重启。 */
            g.cfg_cruise_speed          = param_get_float("behavior.cruise_speed");
            g.acc_standoff              = param_get_float("behavior.acc_standoff");
            g.acc_time_headway          = param_get_float("behavior.acc_time_headway");
            g.acc_k_gap                 = param_get_float("behavior.acc_k_gap");
            g.acc_gap_err_clamp         = param_get_float("behavior.acc_gap_err_clamp");
            g.blocked_range_mult        = param_get_float("behavior.blocked_range_mult");
            g.blocked_range_min         = param_get_float("behavior.blocked_range_min");
            g.follow_hysteresis         = param_get_float("behavior.follow_hysteresis");
            g.lane_change_timeout_s     = param_get_float("behavior.lane_change_timeout_s");
            g.lane_change_cooldown_s    = param_get_float("behavior.lane_change_cooldown_s");
            g.lc_gap_mult               = param_get_float("behavior.lc_gap_mult");
            g.rear_safe_min_m           = param_get_float("behavior.rear_safe_min_m");
            g.rear_safe_time_s          = param_get_float("behavior.rear_safe_time_s");
            g.same_lane_tol_offset      = param_get_float("behavior.same_lane_tol_offset");
            g.uturn_approach_dist_m     = param_get_float("behavior.uturn_approach_dist_m");
            g.uturn_max_trigger_speed   = param_get_float("behavior.uturn_max_trigger_speed");
            g.uturn_timeout_s           = param_get_float("behavior.uturn_timeout_s");

            int lc = g.has_road_geometry ? g.lane_count : 2;
            double lw = g.has_road_geometry ? g.lane_width : 3.5;
            if (lc < 1) lc = 2;
            if (lw < 1.0) lw = 3.5;

            /* ── 状态计时 ── */
            g.state_timer += 0.05;
            if (g.cooldown > 0.0) g.cooldown -= 0.05;

            /* ── 当前车道索引：每帧从 ego_y 重算（变道进行中由变道完成检测接管） ──
             *
             * 关键修复：变道进行中（LEFT_CHANGE/RIGHT_CHANGE），
             * 不能用 ego_y 四舍五入重算 committed_lane_idx——否则：
             *   1. 第810行检测到接近目标车道设 committed_lane_idx=target
             *   2. 下一帧第499行立刻用 ego_y 重算覆盖回去（变道中超调时可能还在两车道之间）
             *   3. 第729行 committed_lane_idx==target_lane_idx 永远不成立 → TIMEOUT
             *
             * 修复：变道进行中先检测"是否已进入目标车道"（阈值取半车道0.875m，比0.15m合理），
             * 若已进入则锁定 committed_lane_idx=target，不再重算；否则用重算值但不锁定。*/
            int recalc_idx;
            {
                double offset = (-g.ego_y) / lw + (lc - 1) * 0.5;
                int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
                if (idx < 0) idx = 0;
                if (idx >= lc) idx = lc - 1;
                recalc_idx = idx;
            }

            StateId cur_sm = statem_current(&g.sm);
            bool in_lane_change = (cur_sm == BEH_ST_LEFT_CHANGE || cur_sm == BEH_ST_RIGHT_CHANGE);
            bool in_uturn = (cur_sm == BEH_ST_U_TURN);
            if (in_uturn) {
                /* 掉头中：不重算 committed_lane，保持原值。
                 * 掉头期间 ego 会跨过全部车道，重算会导致 committed_lane 剧烈抖动。 */
            } else if (g.on_return) {
                /* 掉头返程：几何镜像，锁定 committed_lane 为 ego 当前所在车道，
                 * 忽略残留 target（前进向变道逻辑发的 stale 指令）。纯车道保持。 */
                g.committed_lane_idx = recalc_idx;
            } else if (in_lane_change && g.target_lane_idx >= 0) {
                double target_lane_y = lane_center_y(g.target_lane_idx, lc, lw, 0.0, 0.0);
                double dist_to_target = fabs(g.ego_y - target_lane_y);
                /* 进入目标车道中心半个车道宽度内(1.75/2≈0.875m)即判定变道完成 */
                if (dist_to_target < lw * 0.3) {
                    g.committed_lane_idx = g.target_lane_idx;
                } else {
                    /* 变道中但未到位：用重算值但不锁定（供前车检测使用） */
                    g.committed_lane_idx = recalc_idx;
                }
            } else {
                g.committed_lane_idx = recalc_idx;
            }

            int current_idx = g.committed_lane_idx;
            if (current_idx < 0) current_idx = 0;

            /* ── 找本车道前车 ──
             * 用横向距离而非车道号精确相等来筛。理由：离散车道号在车道线
             * 附近会抖动，`obs_lane_id[i] != current_idx` 这种精确匹配会让
             * 前车在抖动的一帧里整个消失，ACC 随之被踢出。真实 ACC 用的是
             * 横向距离阈值 —— 它对索引抖动天然免疫，也能正确处理"前车正在
             * 跨线切入本车道"这种索引尚未更新的情形。
             * 阈值取半车道 + 0.6m 余量，略宽于车道以捕捉切入车。 */
            double lane_center_cur = lane_center_y(current_idx, lc, lw, 0.0, 0.0);
            double lead_lat_tol = lw * 0.5 + 0.6;
            double best_gap = 1e9;
            double lead_speed = g.target_speed;
            uint32_t lead_id = 0;
            for (int i = 0; i < g.obs_count; i++) {
                if (g.obs_vx[i] < 0) continue;
                if (fabs(g.obs_y[i] - lane_center_cur) > lead_lat_tol) continue;
                double dx = g.obs_x[i] - g.ego_x;
                if (dx > 0 && dx < best_gap) {
                    best_gap = dx;
                    lead_speed = g.obs_vx[i];
                    lead_id = g.obs_id[i];
                }
            }

            /* ── ACC 期望间距 + CTG 跟车目标速度 ──
             * 在 blocked 判定之前算，因为 blocked 现在以 desired_gap 为尺度，
             * 不再用裸 80m 魔法数。 */
            double desired_gap = g.acc_standoff + g.acc_time_headway * g.ego_v;
            double follow_speed = lead_speed;
            if (best_gap < 1e8) {
                double gap_err = best_gap - desired_gap;
                if (gap_err >  g.acc_gap_err_clamp) gap_err =  g.acc_gap_err_clamp;
                if (gap_err < -g.acc_gap_err_clamp) gap_err = -g.acc_gap_err_clamp;
                follow_speed = lead_speed + g.acc_k_gap * gap_err;
                if (follow_speed < 0.0) follow_speed = 0.0;
                /* 上限用巡航速度基准 cfg_cruise_speed，不用 g.target_speed ——
                 * 后者在 FOLLOW 中已被降低，用它做上限会死锁：
                 * target_speed 降→follow_speed 被压更低→target_speed 再降→…→0 */
                if (follow_speed > g.cfg_cruise_speed) follow_speed = g.cfg_cruise_speed;
            }

            /* ── 超车判定 ──
             * blocked 的语义是"本车道前方有车影响通行"，用 desired_gap 的倍数
             * 表达而非裸 80m：mult× 时距间距在 12 m/s 下约 desired_gap*mult，
             * 会随车速自动伸缩（高速时更早察觉、低速时不误触发）。 */
            double blocked_range = fmax(g.blocked_range_min, desired_gap * g.blocked_range_mult);
            /* 滞环：已在 FOLLOW 时用 hysteresis× 的退出距离，避免前车在阈值附近
             * 徘徊时 BLOCKED/LOST_LEAD 每帧翻转（实测曾 150ms 一次进出，
             * 跟车律因此从未连续作用）。进入紧、退出松。 */
            bool in_follow = (statem_current(&g.sm) == BEH_ST_FOLLOW);
            bool blocked = (best_gap < (in_follow ? blocked_range * g.follow_hysteresis : blocked_range));
            double rel_speed = g.ego_v - lead_speed;
            if (rel_speed < 0.0) rel_speed = 0.0;
            double min_gap = g.min_overtake_gap_base + rel_speed * g.min_overtake_gap_speed_mult;
            if (min_gap > g.min_overtake_gap_cap) min_gap = g.min_overtake_gap_cap;
            bool worthwhile = blocked && (best_gap > min_gap) && !g.on_return;

            /* ── D: 行为决策 debug 日志（每 10 帧，复盘跟车/追尾过程） ── */
            if (g.seq % 10 == 0) {
                LOG_INFO("behavior",
                    "[BEH] seq=%u best_gap=%.1f lead_speed=%.1f follow_speed=%.1f "
                    "desired_gap=%.1f blocked=%d worthwhile=%d committed_lane=%d "
                    "target_lane=%d rel_speed=%.1f obs=%d state=%s",
                    g.seq, best_gap, lead_speed, follow_speed,
                    desired_gap, (int)blocked, (int)worthwhile, g.committed_lane_idx,
                    g.target_lane_idx, rel_speed, g.obs_count,
                    beh_state_str(statem_current(&g.sm)));
            }

            /* ── 左右车道评估 ──
             * 关键约束：禁止变道到对向车道！
             *
             * 双向道路以 y=0（道路中心）为界，同向车道的 y 与 ego_y 同号，
             * 对向车道 y 与 ego_y 异号。中国靠右行驶（heading=0 朝 +X）：
             *   - lane 2 (y=-1.75)、lane 3 (y=-5.25)：同向（y<0）
             *   - lane 0 (y=+5.25)、lane 1 (y=+1.75)：对向（y>0），禁止变入
             *
             * 之前只检查 current_idx>0 / current_idx<lc-1，导致 lane 2→lane 1
             * 被判定为合法变道，ego 冲入对向车道逆行。 */
            int adj_idx = -1;
            double adj_speed = g.target_speed;

            /* 道路中心 y（双向道路用0，单向road/geometry可传side_offset） */
            double road_center_y_pos = 0.0;

            /* 左：lane_idx 减小 → y 增大方向 */
            double left_gap = 1e9;
            double left_lead_v = g.target_speed;
            bool left_rear_safe = false;
            bool left_same_side = false;  /* 是否与 ego 在道路中心同侧 */
            if (current_idx > 0) {
                int tl = current_idx - 1;
                double tl_y = lane_center_y(tl, lc, lw, 0.0, 0.0);
                left_same_side = (tl_y - road_center_y_pos) * (g.ego_y - road_center_y_pos) > 0.0;

                if (left_same_side) {
                    double lat_tol = lw * 0.5 + g.same_lane_tol_offset;
                    for (int i = 0; i < g.obs_count; i++) {
                        if (g.obs_vx[i] < 0) continue;
                        if (fabs(g.obs_y[i] - tl_y) > lat_tol) continue;
                        double dx = g.obs_x[i] - g.ego_x;
                        if (dx > 0 && dx < left_gap) { left_gap = dx; left_lead_v = g.obs_vx[i]; }
                    }
                    left_rear_safe = true;
                    for (int i = 0; i < g.obs_count; i++) {
                        if (fabs(g.obs_y[i] - tl_y) > lat_tol) continue;
                        double dx = g.obs_x[i] - g.ego_x;
                        if (dx < 0) {
                            double rd = -dx;
                            double rrs = g.obs_vx[i] - g.ego_v;
                            double min_rd = (rrs > 0.0) ? fmax(g.rear_safe_min_m, rrs * g.rear_safe_time_s) : g.rear_safe_min_m;
                            if (rd < min_rd) left_rear_safe = false;
                        }
                    }
                }
            }

            /* 右：lane_idx 增大 → y 减小方向 */
            double right_gap = 1e9;
            double right_lead_v = g.target_speed;
            bool right_rear_safe = false;
            bool right_same_side = false;
            if (current_idx < lc - 1) {
                int tl = current_idx + 1;
                double tl_y = lane_center_y(tl, lc, lw, 0.0, 0.0);
                right_same_side = (tl_y - road_center_y_pos) * (g.ego_y - road_center_y_pos) > 0.0;

                if (right_same_side) {
                    double lat_tol = lw * 0.5 + g.same_lane_tol_offset;
                    for (int i = 0; i < g.obs_count; i++) {
                        if (g.obs_vx[i] < 0) continue;
                        if (fabs(g.obs_y[i] - tl_y) > lat_tol) continue;
                        double dx = g.obs_x[i] - g.ego_x;
                        if (dx > 0 && dx < right_gap) { right_gap = dx; right_lead_v = g.obs_vx[i]; }
                    }
                    right_rear_safe = true;
                    for (int i = 0; i < g.obs_count; i++) {
                        if (fabs(g.obs_y[i] - tl_y) > lat_tol) continue;
                        double dx = g.obs_x[i] - g.ego_x;
                        if (dx < 0) {
                            double rd = -dx;
                            double rrs = g.obs_vx[i] - g.ego_v;
                            /* 右侧后方安全距离增强（超车后切回场景）：
                             * 左道超车后切回右道时，被超的车虽在后方且更慢，
                             * 但 ego 切回后立即减速至车流速度，后车会追上来。
                             * 后车减速安全距离：直接用 min_gap 做阈值
                             * （与前向 min_gap*lc_gap_mult 对称），
                             * 确保 merge-back 有足够的双向安全裕度。 */
                            double rear_min_gap = fmax(min_gap, g.rear_safe_min_m);
                            double min_rd = (rrs > 0.0)
                                ? fmax(rear_min_gap, rrs * g.rear_safe_time_s)
                                : rear_min_gap;
                            if (rd < min_rd) right_rear_safe = false;
                        }
                    }
                }
            }

            bool left_ok  = left_same_side && left_rear_safe && (left_gap > min_gap * g.lc_gap_mult);
            bool right_ok = right_same_side && right_rear_safe && (right_gap > min_gap * g.lc_gap_mult);

            if (left_ok && right_ok) {
                adj_idx = (left_gap >= right_gap) ? current_idx - 1 : current_idx + 1;
                adj_speed = (left_gap >= right_gap) ? left_lead_v : right_lead_v;
            } else if (left_ok) {
                adj_idx = current_idx - 1;
                adj_speed = left_lead_v;
            } else if (right_ok) {
                adj_idx = current_idx + 1;
                adj_speed = right_lead_v;
            }

            /* ── 事件计算与状态转移 ──
             * 基于当前条件计算该发什么事件给状态机。
             * 转移规则由 BEH_TRANSITIONS 表 + beh_guard 决定。 */
            EventId ev = SM_EVENT_NONE;
            char reason[128] = "";
            double old_timer = g.state_timer;
            int new_target_lane = g.target_lane_idx;
            double new_target_speed = g.target_speed;
            uint32_t new_follow_id = g.follow_obs_id;

            {
                StateId cur = statem_current(&g.sm);
                /* ── 掉头触发（优先级最高：路端是硬约束，任何状态生效）──
                 * 前进 trip：ego_x 接近 ref_path 终点 → 掉头到对向车道
                 * 返程 trip：ego_x 接近路起点（x≈0）→ 掉头回前进车道
                 * 旧实现只在 CRUISE 检测：前车堵在路端 → FOLLOW blocked
                 * 卡死永不掉头（2026-08-03 实测撞"墙"）。FOLLOW/STOP/YIELD
                 * 转移已补（见转移表）。速度门槛：>8m/s 先减速再触发，
                 * 避免 64 点轨迹被 Phase 1 刹车截断。 */
                bool uturn_trigger = false;
                if (cur != BEH_ST_U_TURN) {
                    /* 掉头触发参考点 = min(路端, 触发区内前方最近静止障碍)。
                     * 施工区在路端时必须在施工前完成掉头（2026-08-03 实测：
                     * straight_road 施工 x=2985 前缘 2970，旧实现只按路端
                     * 3000 触发 → 车开到 2965 被施工挡住 → 掉头 Phase 2
                     * 撞施工 → safety 刹停 → 卡死）。
                     * 障碍识别不依赖类型枚举（OBJ_TYPE_* 跨节点不一致），
                     * 用运动学特征：静止 + 在前方。只在接近路端（2×触发
                     * 距离内）时生效，防止远处排队车辆误触发。 */
                    /* 接近路端判定：用 flowsim 权威路端（road_end_x，route
                     * 总长）—— 旧"采样末点封顶检测"在低速起步时误判封顶
                     * → 起点就触发掉头（实测启动即 U_TURN 逆行，2026-08-03）。
                     * 路端固定，触发区 = [road_end - 2×approach, road_end]。 */
                    bool at_road_end = (g.road_end_x < 1e8 &&
                                        g.ego_x > g.road_end_x - g.uturn_approach_dist_m * 2.0);
                    double uturn_ref_x = g.road_end_x;
                    /* 障碍缩短触发点（施工前掉头）只在已接近路端时生效：
                     * 起步阶段前方慢车/未起步 NPC 会被误当"路端障碍"，
                     * 提前触发掉头（实测启动即 U_TURN）。 */
                    if (at_road_end &&
                        g.ego_x > g.road_end_x - g.uturn_approach_dist_m * 2.0) {
                        for (int i = 0; i < g.obs_count; ++i) {
                            if (std::fabs(g.obs_vx[i]) < 0.5 &&
                                std::fabs(g.obs_vy[i]) < 0.5 &&
                                g.obs_x[i] > g.ego_x && g.obs_x[i] < uturn_ref_x) {
                                uturn_ref_x = g.obs_x[i];
                            }
                        }
                    }
                    if (!g.on_return && at_road_end && uturn_ref_x < 1e8 &&
                        g.ego_x > uturn_ref_x - g.uturn_approach_dist_m &&
                        g.cooldown <= 0.0) {
                        if (g.ego_v > g.uturn_max_trigger_speed) {
                            new_target_lane = -1;
                            new_target_speed = g.uturn_max_trigger_speed;
                            snprintf(reason, sizeof(reason),
                                     "uturn approach: decelerate %.1f→%.1f m/s before trigger (x=%.1f end=%.1f)",
                                     g.ego_v, g.uturn_max_trigger_speed, g.ego_x, g.ref_path_end_x);
                        } else {
                            uturn_trigger = true;
                        }
                    } else if (g.on_return &&
                               g.ego_x < g.uturn_approach_dist_m &&
                               g.cooldown <= 0.0) {
                        if (g.ego_v > g.uturn_max_trigger_speed) {
                            new_target_lane = -1;
                            new_target_speed = g.uturn_max_trigger_speed;
                            snprintf(reason, sizeof(reason),
                                     "uturn approach(return): decelerate %.1f→%.1f m/s",
                                     g.ego_v, g.uturn_max_trigger_speed);
                        } else {
                            uturn_trigger = true;
                        }
                    }
                }
                if (uturn_trigger) {
                    ev = BEH_EV_UTURN_TRIGGER;
                    new_target_lane = -1;
                    new_target_speed = g.cfg_cruise_speed;
                    snprintf(reason, sizeof(reason),
                             "uturn trigger: ego_x=%.1f ref_end=%.1f on_return=%d → U_TURN",
                             g.ego_x, g.ref_path_end_x, g.on_return);
                } else if (cur == BEH_ST_CRUISE) {
                    if (worthwhile && adj_idx >= 0 &&
                        !lane_ahead_stop_light(adj_idx, lc, lw)) {
                        ev = (adj_idx < current_idx) ? BEH_EV_OVERTAKE_LEFT : BEH_EV_OVERTAKE_RIGHT;
                        new_target_lane = adj_idx;
                        new_target_speed = fmax(adj_speed, g.ego_v);
                        snprintf(reason, sizeof(reason),
                                 "blocked gap=%.1f lead=%.1fm/s → %s lane%d (left_gap=%.1f left_safe=%d right_gap=%.1f right_safe=%d)",
                                 best_gap, lead_speed,
                                 (ev == BEH_EV_OVERTAKE_LEFT) ? "LEFT_CHANGE" : "RIGHT_CHANGE",
                                 adj_idx, left_gap, left_rear_safe, right_gap, right_rear_safe);
                    } else if (blocked) {
                        ev = BEH_EV_BLOCKED;
                        new_target_speed = follow_speed;
                        new_follow_id = lead_id;
                        snprintf(reason, sizeof(reason),
                                 "blocked gap=%.1f/%.1f lead=%.1fm/s → FOLLOW id=%u v=%.1f (no adj lane: left_ok=%d right_ok=%d cooldown=%.1f)",
                                 best_gap, desired_gap, lead_speed, lead_id, follow_speed,
                                 left_ok, right_ok, g.cooldown);
                    } else if (!g.on_return && left_ok && current_idx > 0 && g.cooldown <= 0.0 &&
                               !lane_ahead_stop_light(current_idx - 1, lc, lw) &&
                               (left_gap >= 1e8 ||
                                left_lead_v >= g.cfg_cruise_speed * 0.7)) {
                        /* 超车完成归位：巡航且未被堵时，若内侧道（idx-1，same_side
                         * 已保证非对向）前方 gap 充足，变回内侧道巡航。
                         * 加"目标车道可用"条件：要么空旷（无前车），要么前车速度
                         * ≥ 0.7×巡航——否则切回去立刻又被慢车堵 → 再变道再归位，
                         * 2026-07-31 用户反馈的"纠结变道"（变过去变回来再变过去
                         * 再超）120s 必现 3 个来回。目标车道慢时留在快车道巡航。
                         * 之前没有归位机制——超车后长期滞留外侧道，而红绿灯
                         * 只管辖 y_lane=-1.75 的内侧道，导致红灯不停车。
                         * 归位用 LEFT_CHANGE 转移，速度恢复巡航基准。
                         *
                         * 2026-07-31：加 lane_ahead_stop_light 条件——归位前
                         * 确认目标车道前方 60m 内没有红灯/黄灯。否则切回 lane2
                         * 立刻停在灯前（实跑：距 x=350 红灯 ~15m 切回即刹停），
                         * 归位变成无效变道。目标车道有灯时留在外侧道继续巡航。 */
                        ev = BEH_EV_OVERTAKE_LEFT;
                        new_target_lane = current_idx - 1;
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason),
                                 "merge back: outer lane%d -> lane%d (left_gap=%.1f) → LEFT_CHANGE",
                                 current_idx, current_idx - 1, left_gap);
                    } else {
                        /* 正常巡航：恢复到巡航速度基准。
                         * 如果从 FOLLOW 退回 CRUISE，target_speed 可能被降低到
                         * 3.8 等，不恢复就会一直低速行驶 + planning 不消费
                         * behavior 的 target_speed（只在 FOLLOW 时消费）。 */
                        new_target_speed = g.cfg_cruise_speed;
                    }
                } else if (cur == BEH_ST_FOLLOW) {
                    if (!blocked) {
                        ev = BEH_EV_LOST_LEAD;
                        new_follow_id = 0;
                        /* 退出 FOLLOW 时恢复巡航速度，否则 target_speed 残留
                         * FOLLOW 的低值，CRUISE 空窗期 planning 不消费
                         * behavior 速度 → 车速卡在低值不加速。 */
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason),
                                 "lead lost (gap=%.1f > %.1f) → CRUISE", best_gap, blocked_range);
                    } else if (worthwhile && adj_idx >= 0 && g.cooldown <= 0.0 &&
                               !lane_ahead_stop_light(adj_idx, lc, lw)) {
                        ev = (adj_idx < current_idx) ? BEH_EV_OVERTAKE_LEFT : BEH_EV_OVERTAKE_RIGHT;
                        new_target_lane = adj_idx;
                        new_target_speed = fmax(adj_speed, g.ego_v);
                        snprintf(reason, sizeof(reason),
                                 "follow blocked gap=%.1f → %s lane%d (left_gap=%.1f right_gap=%.1f)",
                                 best_gap,
                                 (ev == BEH_EV_OVERTAKE_LEFT) ? "LEFT_CHANGE" : "RIGHT_CHANGE",
                                 adj_idx, left_gap, right_gap);
                    } else {
                        /* FOLLOW 稳态：每帧重算 CTG 目标速度。
                         * 这条分支是跟车期间的常驻路径 —— 原来这里也是
                         * `= lead_speed`，所以即使入口帧算对了间距，稳态下
                         * 又退回纯速度跟随，gap 依然无人闭环。 */
                        new_target_speed = follow_speed;
                        new_follow_id = lead_id;
                    }
                } else if (cur == BEH_ST_LEFT_CHANGE || cur == BEH_ST_RIGHT_CHANGE) {
                    /* 变道进行中：target_speed 保持进入时的超车速度
                     * (fmax(adj_speed, ego_v))，不跟车限速。
                     *
                     * P5 修复（变道中每帧 target=lead_speed 防追尾）2026-07-31 删除——
                     * 实测它是"减速变道"根因且会锁死：变道转移首帧 blocked=1、
                     * lead_speed=0（前车停着等红灯）→ target 被设成 0；后续
                     * blocked=0 的帧无分支重置 → 锁死，planning command_speed=0
                     * → 全刹（demo 实测 spd=10.7→0.0 brk=1.00），超车失去意义。
                     * 防追尾改由下层兜底：planning 常规 TTC（gap<~45m 起按
                     * (gap-5)/4 限速）+ safety_control 近场 TTC（gap 逼近 20m
                     * 内全刹）。变道横向位移 ~3s 内完成，纵向闭合由 TTC 分层
                     * 接管，不需要 behavior 在变道中强制跟车。 */
                    if (g.on_return) {
                        /* 掉头返程：立即中止任何进行中的变道，回 CRUISE 保持当前车道。 */
                        ev = BEH_EV_COMPLETED;
                        new_target_lane = -1;
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason), "return trip → abort lane change, hold lane");
                    } else if (g.committed_lane_idx == g.target_lane_idx) {
                        ev = BEH_EV_COMPLETED;
                        new_target_lane = -1;
                        g.cooldown = g.lane_change_cooldown_s;
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason), "lane change complete → CRUISE (cooldown=%.1fs)", g.lane_change_cooldown_s);
                    } else if (g.state_timer > g.lane_change_timeout_s) {
                        ev = BEH_EV_TIMEOUT;
                        new_target_lane = -1;
                        g.cooldown = g.lane_change_cooldown_timeout_s;
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason), "timeout %.1fs → CRUISE fallback (cooldown=%.1fs)", g.state_timer, g.lane_change_cooldown_timeout_s);
                        LOG_WARN("behavior", "lane change timeout (state=%s, target_lane=%d, current=%d, timer=%.1f)",
                                 statem_state_name(&g.sm, cur), g.target_lane_idx, g.committed_lane_idx, g.state_timer);
                    }
                    /* 变道进行中且未超时：不覆盖 target_speed，保持进入时的超车速度 */
                } else if (cur == BEH_ST_U_TURN) {
                    /* 掉头进行中：等待掉头完成。
                     * 掉头完成判定（2026-08 根治，激活此前不可达的 COMPLETED 转移）：
                     *   flowsim 的 ref_path.reverse（on_return）在车进入对向车道时翻
                     *   true（u_turn_active = lane_id>0），且车头已朝返程方向
                     *   （heading≈±π，Phase 4 对齐收严到 ±0.10 后此条件在完成边缘
                     *   触发）——heading 条件同时防 Phase 3 倒车短暂跨线误判。
                     * 完成后 → CRUISE → planning overtake_state 复位、正常规划器
                     * 接管。旧实现只能 15s 超时退出（COMPLETED 转移是死代码，
                     * 注释声称 planning 会通知但从不回写）。 */
                    double hn = g.ego_heading;
                    while (hn > M_PI) hn -= 2.0 * M_PI;
                    while (hn < -M_PI) hn += 2.0 * M_PI;
                    if (g.on_return && fabs(fabs(hn) - M_PI) < 0.15) {
                        ev = BEH_EV_COMPLETED;
                        new_target_lane = -1;
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason),
                                 "uturn completed (on_return=1 h=%.2f) → CRUISE", g.ego_heading);
                        LOG_WARN("behavior", "uturn COMPLETED (h=%.2f) → CRUISE", g.ego_heading);
                    } else if (g.state_timer > g.uturn_timeout_s) {
                        ev = BEH_EV_TIMEOUT;
                        new_target_lane = -1;
                        /* 掉头失败冷却：TIMEOUT 后一段时间内不再触发掉头，
                         * 防止 planning 轨迹/车位置异常时反复进入掉头死循环
                         * （实测 2026-08-03：失败后每帧重触发，卡死 3 分钟）。 */
                        g.cooldown = g.lane_change_cooldown_timeout_s * 6.0;  /* 30s */
                        new_target_speed = g.cfg_cruise_speed;
                        snprintf(reason, sizeof(reason), "uturn timeout %.1fs → CRUISE fallback (cooldown=%.1fs)", g.state_timer, g.cooldown);
                        LOG_WARN("behavior", "uturn timeout (timer=%.1f) → cooldown=%.1fs", g.state_timer, g.cooldown);
                    }
                    /* U_TURN 稳态：不覆盖 target_speed */
                }
            }

            /* 发送事件到状态机 */
            if (ev != SM_EVENT_NONE) {
                if (statem_send_event(&g.sm, ev, nullptr)) {
                    g.state_timer = 0.0;  /* 转移成功 → 计时归零 */
                } else {
                    /* 被 guard 拒绝：保留原状态，reason 不用 */
                    reason[0] = '\0';
                }
            }

            /* ── P5 修复：变道超时后立即检查前车，避免 1-2 cycle 的 CRUISE 空窗期 ──
             *
             * 问题：LEFT_CHANGE 超时 → CRUISE 的转移在本周期完成，但 CRUISE 分支
             * 的 blocked 检查在上方已跳过（进入分支时 cur 还是 LEFT_CHANGE）。
             * 下一周期（50ms 后）才会检测到前车并转 FOLLOW。这 50ms 空窗期内
             * planning 按 CRUISE 下发 target_speed=15（巡航），control 加速冲向
             * 15 m/s，而前方 NPC 仅 7 m/s → 追尾（min_forward_gap=-4.56m）。
             *
             * 修复：状态机转移后，若新状态为 CRUISE 且当前帧已检测到 blocked
             * （lead 搜索在上方 line 349-362 每周期都跑），立即发 BLOCKED 转
             * FOLLOW，并设 target_speed=lead_speed。同一周期完成 CRUISE→FOLLOW，
             * planning 直接收到 FOLLOW + lead_speed，无空窗期。 */
            if (ev == BEH_EV_TIMEOUT || ev == BEH_EV_COMPLETED) {
                StateId new_st = statem_current(&g.sm);
                if (new_st == BEH_ST_CRUISE && blocked) {
                    if (statem_send_event(&g.sm, BEH_EV_BLOCKED, nullptr)) {
                        new_target_speed = lead_speed;
                        new_follow_id = lead_id;
                        snprintf(reason, sizeof(reason),
                                 "post-timeout blocked gap=%.1f lead=%.1fm/s → FOLLOW id=%u (immediate)",
                                 best_gap, lead_speed, lead_id);
                    }
                }
            }

            /* 同步 state 镜像 */
            g.state = beh_state_to_cmd(statem_current(&g.sm));
            /* 应用新速度/车道/跟车目标（状态机转移时计算的新值） */
            g.target_speed = new_target_speed;
            g.target_lane_idx = new_target_lane;
            g.follow_obs_id = new_follow_id;
            /* 未发生转移时保持计时递增 */
            if (g.state_timer == 0.0 && old_timer > 0.0) {
                /* statem_send_event 成功时会置 0，静置 */
            } else if (ev == SM_EVENT_NONE) {
                g.state_timer = old_timer;
            }

            /* ── 发布 Behavior ── */
            Behavior beh;
            memset(&beh, 0, sizeof(beh));
            beh.seq = g.seq;
            beh.stamp_us = clock_now_us();
            beh.command = (BehaviorCommand)g.state;
            beh.target_lane_idx = (int8_t)g.target_lane_idx;
            beh.target_speed = (float)g.target_speed;
            beh.follow_obs_id = g.follow_obs_id;

            uint8_t buf[128];
            size_t len = 0;
            if (Behavior_serialize(&beh, buf, &len) == 0 && len > 0) {
                transport_publish(transport_, TOPIC_PLANNING_BEHAVIOR, buf, (uint32_t)len);
            }

            /* ── FOLLOW/变道高频调试日志 + 实时metrics JSON（每 10 帧 ≈ 0.5s） ──
             * metrics JSON (behavior/state) 与日志同步输出，供 quick_verify.py
             * 等工具实时读取，包含跟车/变道全链路关键变量。 */
            if (g.seq % 10 == 0) {
                StateId cur = statem_current(&g.sm);
                if (cur == BEH_ST_FOLLOW || cur == BEH_ST_LEFT_CHANGE || cur == BEH_ST_RIGHT_CHANGE || blocked) {
                    LOG_INFO("behavior",
                             "[BEH-DBG] %s ego_v=%.2f tgt=%.2f best_gap=%.1f lead_v=%.2f "
                             "desired_gap=%.1f follow_v=%.2f blocked=%d worth=%d "
                             "L(ok=%d gap=%.0f safe=%d same=%d) R(ok=%d gap=%.0f safe=%d same=%d) "
                             "adj=%d lane=%d y=%.2f",
                             beh_state_str(cur), g.ego_v, g.target_speed,
                             best_gap, lead_speed,
                             desired_gap, follow_speed,
                             blocked, worthwhile,
                             left_ok, left_gap, left_rear_safe, left_same_side,
                             right_ok, right_gap, right_rear_safe, right_same_side,
                             adj_idx, g.committed_lane_idx, g.ego_y);
                }

                /* 发布 monitor JSON（behavior/state topic，每 0.5s） */
                {
                    uint64_t elapsed_ms = (clock_now_us() - g.sm.entered_at_us) / 1000;
                    cJSON* root = cJSON_CreateObject();
                    cJSON_AddStringToObject(root, "state", beh_state_str(cur));
                    cJSON_AddNumberToObject(root, "committed_lane", g.committed_lane_idx);
                    cJSON_AddNumberToObject(root, "target_lane", g.target_lane_idx);
                    cJSON_AddNumberToObject(root, "speed", g.ego_v);
                    cJSON_AddNumberToObject(root, "target_speed", g.target_speed);
                    cJSON_AddNumberToObject(root, "cooldown", g.cooldown);
                    cJSON_AddNumberToObject(root, "state_timer", g.state_timer);
                    cJSON_AddNumberToObject(root, "elapsed_ms", (double)elapsed_ms);
                    cJSON_AddNumberToObject(root, "obs_count", g.obs_count);
                    cJSON_AddNumberToObject(root, "ego_x", g.ego_x);
                    cJSON_AddNumberToObject(root, "ego_y", g.ego_y);
                    cJSON_AddNumberToObject(root, "ego_heading", g.ego_heading);
                    cJSON_AddNumberToObject(root, "lane_count", lc);
                    cJSON_AddNumberToObject(root, "lane_width", lw);
                    /* 横向变道调试：目标车道中心y + 到目标的距离 */
                    if (g.target_lane_idx >= 0) {
                        double tgt_y = lane_center_y(g.target_lane_idx, lc, lw, 0.0, 0.0);
                        cJSON_AddNumberToObject(root, "target_lane_y", tgt_y);
                        cJSON_AddNumberToObject(root, "dist_to_target_lane", fabs(g.ego_y - tgt_y));
                    }
                    double cur_lane_y = lane_center_y(g.committed_lane_idx, lc, lw, 0.0, 0.0);
                    cJSON_AddNumberToObject(root, "current_lane_y", cur_lane_y);
                    cJSON_AddNumberToObject(root, "cte", g.ego_y - cur_lane_y);
                    /* 跟车关键变量 */
                    cJSON_AddNumberToObject(root, "best_gap", best_gap < 1e8 ? best_gap : -1.0);
                    cJSON_AddNumberToObject(root, "lead_speed", lead_speed);
                    cJSON_AddNumberToObject(root, "desired_gap", desired_gap);
                    cJSON_AddNumberToObject(root, "follow_speed", follow_speed);
                    cJSON_AddBoolToObject(root, "blocked", blocked);
                    cJSON_AddBoolToObject(root, "worthwhile", worthwhile);
                    /* 变道评估 */
                    cJSON_AddNumberToObject(root, "left_gap", left_gap < 1e8 ? left_gap : -1.0);
                    cJSON_AddNumberToObject(root, "right_gap", right_gap < 1e8 ? right_gap : -1.0);
                    cJSON_AddBoolToObject(root, "left_ok", left_ok);
                    cJSON_AddBoolToObject(root, "right_ok", right_ok);
                    cJSON_AddBoolToObject(root, "left_rear_safe", left_rear_safe);
                    cJSON_AddBoolToObject(root, "right_rear_safe", right_rear_safe);
                    cJSON_AddNumberToObject(root, "adj_idx", adj_idx);
                    cJSON_AddNumberToObject(root, "adj_speed", adj_speed);
                    /* 转移历史：最近 3 条（仅在 50 帧时输出，减少带宽） */
                    if (g.seq % 50 == 0) {
                        cJSON* hist = cJSON_CreateArray();
                        uint32_t n = g.sm.history_count > 3 ? 3 : g.sm.history_count;
                        uint32_t start = (g.sm.history_head + SM_HISTORY_DEPTH - g.sm.history_count) % SM_HISTORY_DEPTH;
                        for (uint32_t hi = 0; hi < n; hi++) {
                            uint32_t idx = (start + hi) % SM_HISTORY_DEPTH;
                            const TransitionRecord* rec = &g.sm.history[idx];
                            cJSON* he = cJSON_CreateObject();
                            cJSON_AddStringToObject(he, "from", beh_state_str(rec->from));
                            cJSON_AddStringToObject(he, "to", beh_state_str(rec->to));
                            cJSON_AddNumberToObject(he, "t_us", (double)rec->timestamp_us);
                            cJSON_AddItemToArray(hist, he);
                        }
                        cJSON_AddItemToObject(root, "history", hist);
                    }
                    char* js = cJSON_PrintUnformatted(root);
                    if (js) {
                        transport_publish(transport_, "behavior/state",
                                          (const uint8_t*)js, (uint32_t)strlen(js) + 1);
                        free(js);
                    }
                    cJSON_Delete(root);
                }
            }

            /* ── 周期状态日志（每 50 帧 ≈ 2.5s） ── */
            if (g.seq % 50 == 0) {
                StateId cur = statem_current(&g.sm);
                uint64_t elapsed_ms = (clock_now_us() - g.sm.entered_at_us) / 1000;
                EventId evs[8];
                int ne = statem_allowed_events(&g.sm, evs, 8);
                char ev_buf[128] = "";
                for (int i = 0; i < ne && i < 8; i++) {
                    if (i > 0) strcat(ev_buf, ",");
                    strcat(ev_buf, beh_event_str(evs[i]));
                }
                LOG_INFO("behavior", "[SM] state=%s allowed=[%s] elapsed=%lums obs=%d lane=%d/%d v=%.1f",
                         beh_state_str(cur), ev_buf, (unsigned long)elapsed_ms,
                         g.obs_count, g.committed_lane_idx, lc, g.ego_v);
            }

            g.seq++;

            /* 自适应 sleep：维持稳定 20Hz，减去本帧工作时间 */
            uint64_t t_frame_us = clock_now_us() - t_start;
            uint64_t sleep_us_val = (t_frame_us < 50000) ? (50000 - t_frame_us) : 0;
            if (should_stop()) break;
            co_await sleep_us(sleep_us_val);
        }

        LOG_INFO("behavior", "FlowCoro behavior planner stopped (%u frames)", g.seq);
        co_return;
    }

private:
    Transport* transport_;
};

/* ── TaskBase 包装器（宏生成） — 必须在 behavior_init 前展开 ─────── */
EXPORT_COROUTINE_TASK(BehaviorTask, behavior)

/* ── NodePlugin 实现 ───────────────────────────────────── */

static const char* s_inputs[]  = {
    TOPIC_FUSION_LOCALIZATION,
    TOPIC_PERCEPTION_TRACKED_OBJECTS,
    TOPIC_PERCEPTION_OBSTACLES,
    TOPIC_ROAD_GEOMETRY,
    TOPIC_ROAD_TRAFFIC_LIGHTS,
    nullptr
};
static const char* s_outputs[] = {
    TOPIC_PLANNING_BEHAVIOR,
    nullptr
};

extern NodePlugin s_plugin;

static int behavior_init(MessageBus* bus, Transport* transport,
                          DiscoveryManager* discovery, Scheduler* scheduler,
                          const char* params_json) {
    g.scheduler = scheduler;

    g.ego_x = g.ego_y = g.ego_v = g.ego_heading = 0.0;
    g.has_fusion = 0;
    g.has_obs = 0;
    g.has_road_geometry = 0;
    g.obs_count = 0;
    g.seq = 0;
    g.state = BEH_CRUISE;
    g.state_timer = 0.0;
    g.cooldown = 0.0;
    g.committed_lane_idx = 0;
    g.target_lane_idx = -1;
    g.target_speed = 10.0;
    g.cfg_cruise_speed = 15.0;  /* 默认巡航速度，可被 config 覆盖 */
    g.follow_obs_id = 0;
    g.lane_count = 2;
    g.lane_width = 3.5;

    /* 初始化框架状态机 */
    statem_init(&g.sm, BEH_TRANSITIONS, BEH_ST_CRUISE, "behavior");
    g.sm.guard = beh_guard;
    g.sm.debug_hook = beh_debug_hook;
    g.sm.trace_enabled = true;

    g.transport = transport;
    g.discovery = discovery;

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "target_speed")) && cJSON_IsNumber(j)) {
                g.target_speed = j->valuedouble;
                g.cfg_cruise_speed = j->valuedouble;  /* config 设置 target_speed 时同步巡航基准 */
            }
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "min_overtake_gap_base")) && cJSON_IsNumber(j))
                g.min_overtake_gap_base = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "min_overtake_gap_cap")) && cJSON_IsNumber(j))
                g.min_overtake_gap_cap = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "acc_standoff")) && cJSON_IsNumber(j))
                g.acc_standoff = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "acc_time_headway")) && cJSON_IsNumber(j))
                g.acc_time_headway = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "acc_k_gap")) && cJSON_IsNumber(j))
                g.acc_k_gap = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "acc_gap_err_clamp")) && cJSON_IsNumber(j))
                g.acc_gap_err_clamp = j->valuedouble;
            cJSON_Delete(p);
        }
    }

    /* ── 参数注册（默认值用 g.<字段>，即上面解析后的值，不用硬编码字面量，
     *    否则会把 params_json 解析到的值盖掉）。逐帧 param_get_float 重读
     *    见 BehaviorTask::run()，三处都通才能 `flowctl param set` 生效。 */
    param_register_float("behavior.cruise_speed",          g.cfg_cruise_speed,     1.0, 50.0,
                         "巡航目标速度 m/s（速度决策唯一来源，control 不再有 cruise_speed）");
    param_register_float("behavior.acc_standoff",      g.acc_standoff,      0.5, 20.0,
                         "ACC 静止安全余量 (m)");
    param_register_float("behavior.acc_time_headway",  g.acc_time_headway,  0.5, 4.0,
                         "ACC 时距 (s)：desired_gap = standoff + headway*v");
    param_register_float("behavior.acc_k_gap",         g.acc_k_gap,         0.0, 2.0,
                         "ACC 间距误差增益 (1/s)");
    param_register_float("behavior.acc_gap_err_clamp", g.acc_gap_err_clamp, 1.0, 30.0,
                         "ACC 间距误差对目标速度的修正上限 (m/s)");
    param_register_float("behavior.blocked_range_mult",     g.blocked_range_mult,     1.0, 10.0,
                         "blocked 检测距离倍数: max(min_m, desired_gap*mult)");
    param_register_float("behavior.blocked_range_min",      g.blocked_range_min,      5.0, 100.0,
                         "blocked 检测最小距离 (m)");
    param_register_float("behavior.follow_hysteresis",      g.follow_hysteresis,      1.0, 3.0,
                         "FOLLOW→CRUISE 退出滞环倍数（进入紧退出松）");
    param_register_float("behavior.lane_change_timeout_s",  g.lane_change_timeout_s,  3.0, 20.0,
                         "变道超时时间 (s)，超时回退 CRUISE");
    param_register_float("behavior.lane_change_cooldown_s", g.lane_change_cooldown_s, 1.0, 10.0,
                         "变道完成后冷却 (s)，期间不变道");
    param_register_float("behavior.lc_gap_mult",            g.lc_gap_mult,            1.0, 5.0,
                         "目标车道前车间距阈值倍数 = min_gap*mult");
    param_register_float("behavior.rear_safe_min_m",        g.rear_safe_min_m,        5.0, 50.0,
                         "后向安全最小距离 (m)");
    param_register_float("behavior.rear_safe_time_s",       g.rear_safe_time_s,       1.0, 8.0,
                         "后向安全时距 (s)：min_rd = max(min_m, rrs*time_s)");
    param_register_float("behavior.same_lane_tol_offset",   g.same_lane_tol_offset,   0.1, 2.0,
                         "车道归属横向容差偏移 (m)：半车道宽+offset");
    param_register_float("behavior.uturn_approach_dist_m",  g.uturn_approach_dist_m,  20.0, 200.0,
                         "掉头触发距离阈值 (m)：ego_x 距路端此距离时触发掉头");
    param_register_float("behavior.uturn_max_trigger_speed", g.uturn_max_trigger_speed, 3.0, 12.0,
                         "掉头触发速度上限 (m/s)：高于此先减速再触发，防 64 点轨迹截断");
    param_register_float("behavior.uturn_timeout_s",        g.uturn_timeout_s,        5.0, 30.0,
                         "掉头超时 (s)：超时回退 CRUISE");

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION,         on_fusion,             nullptr);
    transport_subscribe(transport, TOPIC_VEHICLE_STATE, on_vehicle_state, nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_TRACKED_OBJECTS,  on_tracked_objects,    nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_OBSTACLES,        on_raw_obstacles,      nullptr);
    transport_subscribe(transport, TOPIC_ROAD_GEOMETRY,               on_road_geometry,      nullptr);
    transport_subscribe(transport, TOPIC_ROAD_TRAFFIC_LIGHTS,         on_traffic_lights,     nullptr);
    transport_subscribe(transport, TOPIC_ROAD_REF_PATH,               on_ref_path,           nullptr);

    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION,       0u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, TOPIC_PERCEPTION_TRACKED_OBJECTS, 0u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, TOPIC_ROAD_GEOMETRY,             0x80AD5C12u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, TOPIC_ROAD_TRAFFIC_LIGHTS,       0x7E5C0FFEu, CAP_SUBSCRIBER,  0);  /* 归位决策 */
    discovery_advertise(discovery, TOPIC_PLANNING_BEHAVIOR,    BEHAVIOR_TYPE_ID, CAP_PUBLISHER, 20.0);
    discovery_advertise(discovery, "behavior/state",            0u, CAP_PUBLISHER, 0.4);  /* 每 2.5s */

    transport_advertise(transport, TOPIC_PLANNING_BEHAVIOR, BEHAVIOR_TYPE_ID);
    transport_advertise(transport, "behavior/state", 0u);  /* JSON text, no type check */

    /* 创建 TaskBase 包装器（托管模式） */
    TaskConfig tcfg = {};
    snprintf(tcfg.name, sizeof(tcfg.name), "behavior");
    tcfg.priority = TASK_PRIORITY_NORMAL;
    g.task_wrapper = behavior_create(&tcfg, bus);
    if (!g.task_wrapper) {
        LOG_ERROR("behavior", "behavior_create failed");
        return -1;
    }
    g.task_wrapper->impl->set_params(transport);
    s_plugin.taskbase = behavior_get_base(g.task_wrapper);

    LOG_INFO("behavior", "initialized (FlowCoro, target_speed=%.1f m/s, managed mode)", g.target_speed);
    return 0;
}

static int behavior_start(void) {
    if (!g.task_wrapper) return -1;
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) LOG_WARN("behavior", "node_start_managed failed: %d", rc);
    node_announce_self(g.transport, &s_plugin);
    LOG_INFO("behavior", "started (managed mode)");
    return 0;
}

static void behavior_stop(void) {
    if (g.task_wrapper) {
        behavior_stop(&g.task_wrapper->base);
    }
}

static void behavior_cleanup(void) {
    if (g.task_wrapper) {
        behavior_destroy(g.task_wrapper);
        g.task_wrapper = nullptr;
    }
    s_plugin.taskbase = nullptr;
    LOG_INFO("behavior", "cleanup done");
}

static int behavior_health(void) { return 0; }

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "behavior_planner",
    "1.0.0",
    "Behavior planner: cruise/follow/lane_change decisions [FlowCoro]",
    s_inputs,
    s_outputs,
    behavior_init,
    behavior_start,
    behavior_stop,
    behavior_cleanup,
    behavior_health,
    nullptr,  /* taskbase: 在 init() 中通过 behavior_create 设置 */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
