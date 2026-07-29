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
    double min_overtake_gap_base{10.0};
    double min_overtake_gap_cap{90.0};
    double min_overtake_gap_speed_mult{0.7};

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

/* ── perception/tracked_objects 订阅（JSON，带 tracking） ── */
static void on_tracked_objects(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    g.obs_count = 0;

    int lc = g.has_road_geometry ? g.lane_count : 2;
    double lw = g.has_road_geometry ? g.lane_width : 3.5;
    double ch = cos(g.ego_heading), sh = sin(g.ego_heading);

    cJSON* objects = cJSON_GetObjectItemCaseSensitive(root, "objects");
    if (objects && cJSON_IsArray(objects)) {
        int n = cJSON_GetArraySize(objects);
        if (n > BEH_MAX_OBS) n = BEH_MAX_OBS;
        g.obs_count = n;

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
        }
    }
    g.has_obs = 1;
    cJSON_Delete(root);
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

            int lc = g.has_road_geometry ? g.lane_count : 2;
            double lw = g.has_road_geometry ? g.lane_width : 3.5;
            if (lc < 1) lc = 2;
            if (lw < 1.0) lw = 3.5;

            /* ── 状态计时 ── */
            g.state_timer += 0.05;
            if (g.cooldown > 0.0) g.cooldown -= 0.05;

            /* ── 初始化车道索引（首次收到融合数据时从 ego_y 推定） ── */
            if (g.committed_lane_idx == 0 && g.seq < 10) {
                double offset = (-g.ego_y) / lw + (lc - 1) * 0.5;
                int idx = (int)(offset >= 0.0 ? offset + 0.5 : offset - 0.5);
                if (idx < 0) idx = 0;
                if (idx >= lc) idx = lc - 1;
                g.committed_lane_idx = idx;
            }

            int current_idx = g.committed_lane_idx;
            if (current_idx < 0) current_idx = 0;

            /* ── 找本车道前车 ── */
            double best_gap = 1e9;
            double lead_speed = g.target_speed;
            uint32_t lead_id = 0;
            for (int i = 0; i < g.obs_count; i++) {
                if (g.obs_vx[i] < 0) continue;
                if (g.obs_lane_id[i] != current_idx) continue;
                double dx = g.obs_x[i] - g.ego_x;
                if (dx > 0 && dx < best_gap) {
                    best_gap = dx;
                    lead_speed = g.obs_vx[i];
                    lead_id = g.obs_id[i];
                }
            }

            /* ── 超车判定 ── */
            bool blocked = (best_gap < 80.0);
            double rel_speed = g.ego_v - lead_speed;
            if (rel_speed < 0.0) rel_speed = 0.0;
            double min_gap = g.min_overtake_gap_base + rel_speed * g.min_overtake_gap_speed_mult;
            if (min_gap > g.min_overtake_gap_cap) min_gap = g.min_overtake_gap_cap;
            bool worthwhile = blocked && (best_gap < min_gap);

            /* ── 左右车道评估 ── */
            int adj_idx = -1;
            double adj_speed = g.target_speed;

            /* 左 */
            double left_gap = 1e9;
            double left_lead_v = g.target_speed;
            bool left_rear_safe = false;
            if (current_idx > 0) {
                int tl = current_idx - 1;
                for (int i = 0; i < g.obs_count; i++) {
                    if (g.obs_vx[i] < 0) continue;
                    if (g.obs_lane_id[i] != tl) continue;
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx > 0 && dx < left_gap) { left_gap = dx; left_lead_v = g.obs_vx[i]; }
                }
                left_rear_safe = true;
                for (int i = 0; i < g.obs_count; i++) {
                    if (g.obs_vx[i] < 0) continue;
                    if (g.obs_lane_id[i] != tl) continue;
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx < 0) {
                        double rd = -dx;
                        double rrs = g.obs_vx[i] - g.ego_v;
                        double min_rd = (rrs > 0.0) ? fmax(15.0, rrs * 3.0) : 15.0;
                        if (rd < min_rd) left_rear_safe = false;
                    }
                }
            }

            /* 右 */
            double right_gap = 1e9;
            double right_lead_v = g.target_speed;
            bool right_rear_safe = false;
            if (current_idx < lc - 1) {
                int tl = current_idx + 1;
                for (int i = 0; i < g.obs_count; i++) {
                    if (g.obs_vx[i] < 0) continue;
                    if (g.obs_lane_id[i] != tl) continue;
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx > 0 && dx < right_gap) { right_gap = dx; right_lead_v = g.obs_vx[i]; }
                }
                right_rear_safe = true;
                for (int i = 0; i < g.obs_count; i++) {
                    if (g.obs_vx[i] < 0) continue;
                    if (g.obs_lane_id[i] != tl) continue;
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx < 0) {
                        double rd = -dx;
                        double rrs = g.obs_vx[i] - g.ego_v;
                        double min_rd = (rrs > 0.0) ? fmax(15.0, rrs * 3.0) : 15.0;
                        if (rd < min_rd) right_rear_safe = false;
                    }
                }
            }

            bool left_ok  = (current_idx > 0) && left_rear_safe && (left_gap > min_gap * 1.5);
            bool right_ok = (current_idx < lc - 1) && right_rear_safe && (right_gap > min_gap * 1.5);

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
                        new_target_speed = lead_speed;
                        new_follow_id = lead_id;
                        snprintf(reason, sizeof(reason),
                                 "blocked gap=%.1f lead=%.1fm/s → FOLLOW id=%u (no adj lane: left_ok=%d right_ok=%d cooldown=%.1f)",
                                 best_gap, lead_speed, lead_id, left_ok, right_ok, g.cooldown);
                    }
                } else if (cur == BEH_ST_FOLLOW) {
                    if (!blocked) {
                        ev = BEH_EV_LOST_LEAD;
                        new_follow_id = 0;
                        snprintf(reason, sizeof(reason),
                                 "lead lost (gap=%.1f > 80m) → CRUISE", best_gap);
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
                        new_target_speed = lead_speed;
                        new_follow_id = lead_id;
                    }
                } else if (cur == BEH_ST_LEFT_CHANGE || cur == BEH_ST_RIGHT_CHANGE) {
                    if (g.committed_lane_idx == g.target_lane_idx) {
                        ev = BEH_EV_COMPLETED;
                        new_target_lane = -1;
                        g.cooldown = 3.0;
                        snprintf(reason, sizeof(reason), "lane change complete → CRUISE (cooldown=3.0s)");
                    } else if (g.state_timer > 5.0) {
                        ev = BEH_EV_TIMEOUT;
                        new_target_lane = -1;
                        g.cooldown = 5.0;
                        snprintf(reason, sizeof(reason), "timeout %.1fs → CRUISE fallback (cooldown=5.0s)", g.state_timer);
                        LOG_WARN("behavior", "lane change timeout (state=%s, target_lane=%d, current=%d, timer=%.1f)",
                                 statem_state_name(&g.sm, cur), g.target_lane_idx, g.committed_lane_idx, g.state_timer);
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

            /* 同步 state 镜像 */
            g.state = beh_state_to_cmd(statem_current(&g.sm));
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

                /* 发布 monitor JSON（behavior/state topic） */
                {
                    cJSON* root = cJSON_CreateObject();
                    cJSON_AddStringToObject(root, "state", beh_state_str(cur));
                    cJSON_AddNumberToObject(root, "committed_lane", g.committed_lane_idx);
                    cJSON_AddNumberToObject(root, "target_lane", g.target_lane_idx);
                    cJSON_AddNumberToObject(root, "speed", g.ego_v);
                    cJSON_AddNumberToObject(root, "target_speed", g.target_speed);
                    cJSON_AddNumberToObject(root, "cooldown", g.cooldown);
                    cJSON_AddNumberToObject(root, "elapsed_ms", (double)elapsed_ms);
                    /* 转移历史：最近 3 条 */
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
                    char* js = cJSON_PrintUnformatted(root);
                    if (js) {
                        transport_publish(transport_, "behavior/state",
                                          (const uint8_t*)js, (uint32_t)strlen(js) + 1);
                        free(js);
                    }
                    cJSON_Delete(root);
                }
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
            cJSON_Delete(p);
        }
    }

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION,      on_fusion,                nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_TRACKED_OBJECTS, on_tracked_objects,      nullptr);
    transport_subscribe(transport, TOPIC_ROAD_GEOMETRY,            on_road_geometry,         nullptr);

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
