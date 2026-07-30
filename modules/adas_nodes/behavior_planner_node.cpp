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
#include <pthread.h>
#include <unistd.h>

#include <memory>
#include <atomic>

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
};

enum BehEvent {
    BEH_EV_BLOCKED        = 200,  /* 本车道有前车且暂无变道条件 */
    BEH_EV_LOST_LEAD      = 201,  /* 前车消失或间距充足 */
    BEH_EV_OVERTAKE_LEFT  = 202,  /* 条件满足→向左变道 */
    BEH_EV_OVERTAKE_RIGHT = 203,  /* 条件满足→向右变道 */
    BEH_EV_COMPLETED      = 204,  /* 变道完成 */
    BEH_EV_TIMEOUT        = 205,  /* 变道超时回退 */
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
    TRANSITION_TABLE_END,
};

/* ── 节点本地状态 ───────────────────────────────────────────── */

#define BEH_MAX_OBS 128

struct BehaviorContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};

    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* 发布帧计数 */
    uint32_t seq{0};

    /* ego 状态（从 fusion/localization JSON 解析） */
    double ego_x{0}, ego_y{0}, ego_v{0}, ego_heading{0};
    volatile int has_fusion{0};

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
    double target_speed{10.0};
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
    double blocked_range_mult{4.0};     /* blocked 检测距离 = max(min_m, desired_gap * mult) */
    double blocked_range_min{30.0};     /* blocked 检测最小距离 (m) */
    double follow_hysteresis{1.3};      /* FOLLOW→CRUISE 退出滞环倍数（进入紧退出松） */
    double lane_change_timeout_s{8.0};  /* 变道超时 (s)，超时回退 CRUISE */
    double lane_change_cooldown_s{3.0}; /* 变道完成冷却 (s) */
    double lane_change_cooldown_timeout_s{5.0}; /* 变道超时后冷却 (s) */
    double lc_gap_mult{1.5};            /* 目标车道前车间距阈值 = min_gap * mult */
    double rear_safe_min_m{15.0};       /* 后向安全最小距离 (m) */
    double rear_safe_time_s{3.0};       /* 后向安全时距 (s) */
    double same_lane_tol_offset{0.6};   /* 车道归属横向容差偏移 (m)，半车道宽 + offset */

    /* 协程 */
    std::unique_ptr<class BehaviorTask> task;

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
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "v")) && cJSON_IsNumber(j))
        g.ego_v = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "heading")) && cJSON_IsNumber(j))
        g.ego_heading = j->valuedouble;
    g.has_fusion = 1;
    cJSON_Delete(root);
}

/* ── perception/tracked_objects 订阅（JSON，带 tracking） ──
 * 仅当 objects 数组非空时覆盖 obs 缓存；空数组或字段缺失时保留 on_raw_obstacles
 * 填充的 raw 数据作为回退，避免 tracker 偶发空帧清空障碍物导致 FOLLOW 丢失。 */
static void on_tracked_objects(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
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
    if (!msg || !msg->data) return;

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
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* item;
    if ((item = cJSON_GetObjectItem(root, "lane_count"))) g.lane_count = item->valueint;
    if ((item = cJSON_GetObjectItem(root, "lane_width"))) g.lane_width = item->valuedouble;
    g.has_road_geometry = 1;
    cJSON_Delete(root);
}

/* ── 协程任务 ──────────────────────────────────────────── */

class BehaviorTask : public CoroutineTask {
public:
    BehaviorTask(MessageBus* bus, Transport* transport)
        : CoroutineTask(bus), transport_(transport) {}

protected:
    Task run() override {
        LOG_INFO("behavior", "FlowCoro behavior planner started (20Hz)");

        while (!should_stop()) {
            co_await sleep_us(50000);  /* 20Hz */
            if (should_stop()) break;

            /* 等待感知数据就绪 */
            if (!g.has_fusion || !g.has_obs) {
                g.seq++;
                continue;
            }

            /* 参数热重载（三处之三）：漏了这步，注册了也改不动，只能重启。 */
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

            int lc = g.has_road_geometry ? g.lane_count : 2;
            double lw = g.has_road_geometry ? g.lane_width : 3.5;
            if (lc < 1) lc = 2;
            if (lw < 1.0) lw = 3.5;

            /* ── 状态计时 ── */
            g.state_timer += 0.05;
            if (g.cooldown > 0.0) g.cooldown -= 0.05;

            /* ── 当前车道索引：每帧从 ego_y 重算（带滞环） ──
             *
             * 原实现只在 `committed_lane_idx == 0 && seq < 10` 时算一次，
             * 之后仅在变道完成时更新。ego 横向一漂，committed_lane_idx 就
             * 与实际车道脱节，而前车筛选按它做精确匹配 → 前车周期性"消失"
             * → best_gap=1e9 → blocked=false → LOST_LEAD 退出 FOLLOW。
             * 实测 FOLLOW 进出只隔 150ms，ACC 从未连续作用过，ego 全速接近
             * 前车直到 gap=-3.15m。**追尾的主因是这个，不是 ACC 律本身。**
             *
             * 滞环：只有偏离当前车道中心超过 (半车道 + LANE_HYST_M) 才换
             * 索引，避免在车道线上抖动时索引来回跳。变道进行中不重算
             * （由变道完成逻辑接管），否则会与 target_lane_idx 打架。 */
            // 逐帧重算车道索引，不依赖滞环。原滞环在 fusion 偶发错误帧时
            // 会把 committed_lane_idx 锁死在错误值，导致永远看不到前车。
            {
                double offset = (-g.ego_y) / lw + (lc - 1) * 0.5;
                int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
                if (idx < 0) idx = 0;
                if (idx >= lc) idx = lc - 1;
                g.committed_lane_idx = idx;
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
                if (follow_speed > g.target_speed) follow_speed = g.target_speed;
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
            bool worthwhile = blocked && (best_gap > min_gap);

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
                if (cur == BEH_ST_CRUISE) {
                    if (worthwhile && adj_idx >= 0) {
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
                    }
                } else if (cur == BEH_ST_FOLLOW) {
                    if (!blocked) {
                        ev = BEH_EV_LOST_LEAD;
                        new_follow_id = 0;
                        snprintf(reason, sizeof(reason),
                                 "lead lost (gap=%.1f > %.1f) → CRUISE", best_gap, blocked_range);
                    } else if (worthwhile && adj_idx >= 0 && g.cooldown <= 0.0) {
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
                    if (g.committed_lane_idx == g.target_lane_idx) {
                        ev = BEH_EV_COMPLETED;
                        new_target_lane = -1;
                        g.cooldown = g.lane_change_cooldown_s;
                        snprintf(reason, sizeof(reason), "lane change complete → CRUISE (cooldown=%.1fs)", g.lane_change_cooldown_s);
                    } else if (g.state_timer > g.lane_change_timeout_s) {
                        ev = BEH_EV_TIMEOUT;
                        new_target_lane = -1;
                        g.cooldown = g.lane_change_cooldown_timeout_s;
                        snprintf(reason, sizeof(reason), "timeout %.1fs → CRUISE fallback (cooldown=%.1fs)", g.state_timer, g.lane_change_cooldown_timeout_s);
                        LOG_WARN("behavior", "lane change timeout (state=%s, target_lane=%d, current=%d, timer=%.1f)",
                                 statem_state_name(&g.sm, cur), g.target_lane_idx, g.committed_lane_idx, g.state_timer);
                    } else if (blocked) {
                        /* P5 修复：变道进行中但仍在前车后方 → 保持跟车速度防追尾。
                         *
                         * 问题：原实现 LEFT_CHANGE/RIGHT_CHANGE 期间不更新 target_speed，
                         * 保持进入时的 fmax(adj_speed, ego_v)≈ego_v（14.4 m/s）。但变道
                         * 需要 3-5s 才能完成（planning 横向位移），期间 ego 仍在原车道，
                         * 前方 NPC 仅 7 m/s → 5s 内位移差 35m，必然追尾。
                         *
                         * 修复：变道未完成（committed_lane_idx != target_lane_idx）时，
                         * 若本车道仍有前车（blocked），target_speed = lead_speed（跟车速度）。
                         * 变道完成进入新车道后才由 COMPLETED 转移到 CRUISE 释放速度。 */
                        new_target_speed = lead_speed;
                        new_follow_id = lead_id;
                    }
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

            /* ── 车道追踪：检测变道完成 ── */
            if ((g.state == BEH_LEFT_CHANGE || g.state == BEH_RIGHT_CHANGE) && g.target_lane_idx >= 0) {
                double target_lane_y = lane_center_y(g.target_lane_idx, lc, lw, 0.0, 0.0);
                if (fabs(g.ego_y - target_lane_y) < 0.15) {
                    g.committed_lane_idx = g.target_lane_idx;
                }
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
                    cJSON_AddNumberToObject(root, "elapsed_ms", (double)elapsed_ms);
                    cJSON_AddNumberToObject(root, "obs_count", g.obs_count);
                    cJSON_AddNumberToObject(root, "ego_x", g.ego_x);
                    cJSON_AddNumberToObject(root, "ego_y", g.ego_y);
                    cJSON_AddNumberToObject(root, "ego_heading", g.ego_heading);
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
        }

        LOG_INFO("behavior", "FlowCoro behavior planner stopped (%u frames)", g.seq);
        co_return;
    }

private:
    Transport* transport_;
};

/* ── 协程宿主线程 ───────────────────────────────────────── */

void* behavior_thread(void*) {
    pthread_setname_np(pthread_self(), "behavior");
    try {
        flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1 }};
        g_node_exec = &ex;
        CoroutineTask& ct = *g.task;
        ex.spawn(ct.run(), "behavior");
        node_pump(ex, [] { return (bool)g.should_stop; });
        ex.shutdown();
        g_node_exec = nullptr;
    } catch (...) {
        LOG_ERROR("behavior", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ───────────────────────────────────── */

static const char* s_inputs[]  = {
    TOPIC_FUSION_LOCALIZATION,
    TOPIC_PERCEPTION_TRACKED_OBJECTS,
    TOPIC_PERCEPTION_OBSTACLES,
    TOPIC_ROAD_GEOMETRY,
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
    (void)scheduler;

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
    g.should_stop = false;

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "target_speed")) && cJSON_IsNumber(j))
                g.target_speed = j->valuedouble;
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

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION,         on_fusion,             nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_TRACKED_OBJECTS,  on_tracked_objects,    nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_OBSTACLES,        on_raw_obstacles,      nullptr);
    transport_subscribe(transport, TOPIC_ROAD_GEOMETRY,               on_road_geometry,      nullptr);

    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION,       0u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, TOPIC_PERCEPTION_TRACKED_OBJECTS, 0u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, TOPIC_ROAD_GEOMETRY,             0x80AD5C12u, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, TOPIC_PLANNING_BEHAVIOR,    BEHAVIOR_TYPE_ID, CAP_PUBLISHER, 20.0);
    discovery_advertise(discovery, "behavior/state",            0u, CAP_PUBLISHER, 0.4);  /* 每 2.5s */

    transport_advertise(transport, TOPIC_PLANNING_BEHAVIOR, BEHAVIOR_TYPE_ID);
    transport_advertise(transport, "behavior/state", 0u);  /* JSON text, no type check */

    g.task = std::make_unique<BehaviorTask>(bus, transport);

    LOG_INFO("behavior", "initialized (FlowCoro, target_speed=%.1f m/s)", g.target_speed);
    return 0;
}

static int behavior_start(void) {
    if (!g.task) return -1;
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, behavior_thread, nullptr) != 0) {
        LOG_WARN("behavior", "pthread_create failed: %s", strerror(errno));
        return -1;
    }
    g.running = true;
    LOG_INFO("behavior", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void behavior_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->set_stop();
}

static void behavior_cleanup(void) {
    behavior_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
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
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
