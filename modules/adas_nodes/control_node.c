/**
 * control_node.c — PID 纵向/横向控制节点插件
 *
 * 订阅 fusion/localization, planning/trajectory
 * PID 计算油门/刹车/转向 → 发布 control/raw_cmd
 *
 * 与 sim_world_node 配合: control 产生指令, sim_world 执行物理模拟。
 *
 * NodePlugin 接口，编译为 libcontrol_node.so。
 */

#include "node_plugin.h"
#include "logger.h"
#include "param_registry.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

/* ── 节点本地状态 ───────────────────────────────────────────── */

#define MAX_OBS 4

/* 横向级联 PD 常量 */
#define MAX_PSI_DES_RAD    0.349   /* 最大期望航向角 ≈ ±20° */
#define STEER_FILTER_NEW   0.7     /* 低通滤波新值权重 */
#define STEER_FILTER_PREV  0.3     /* 低通滤波旧值权重 */
#define CONTROL_WHEELBASE_M 2.7

/* 车道判定迟滞: 已提交车道保持不变, 直到 ego_y 越过中心线 ±此死区才切换,
 * 避免车骑在车道线 (y≈0) 附近时目标车道每帧翻转造成的横向抖振。 */
#define LANE_HYSTERESIS_M   0.5
/* 死锁恢复: 近乎静止且横向卡在车道线附近持续超过此秒数, 强制收敛到最近车道中心。 */
#define STUCK_SPEED_MPS     0.5    /* 判定"近乎静止"的速度阈值 */
#define STUCK_LATERAL_M     0.6    /* 判定"卡在车道线附近"的 |ego_y| 阈值 */
#define STUCK_RECOVER_S     3.0    /* 触发恢复所需持续时间 (秒) */

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

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* PID 状态 */
    double kp, ki, kd;
    double integral;
    double prev_error;
    /* 横向级联 PD 状态 */
    double lat_kp;          /* lateral error → desired heading (rad/m) */
    double lat_kd_heading;  /* heading error → steer (阻尼) */
    double ego_heading;     /* 从 fusion 获取的航向角 (rad) */
    double prev_steer;

    /* 从 topic 解析的值 */
    double current_speed;
    double target_speed;
    double ego_x, ego_y;
    double lane_d;          /* 从 trajectory 解析的横向偏移（Frenet d） */

    /* 障碍物数据 (从 vehicle/state 解析) */
    double obs_x[MAX_OBS], obs_y[MAX_OBS], obs_vx[MAX_OBS];
    int    obs_valid[MAX_OBS];
    char   obs_type[MAX_OBS][16]; /* e.g. "car", "pedestrian" */
    int    ped_index;              /* index of pedestrian obs, -1 if none */

    volatile int has_fusion;
    volatile int has_planning;
    uint64_t last_fusion_us;    /* monotonic timestamp of last fusion message */
    uint64_t last_planning_us;  /* monotonic timestamp of last planning message */

    /* 变道状态机 */
    int    lc_state;     /* 0=正常 1=左变道中 2=左车道巡航 3=右回正 */
    int    lc_attempted;
    double lc_timer;
    double lc_wait;
    double lc_cooldown;
    double lc_origin_y;
    double lc_target_y;
    double lane_width;
    double blocked_timeout_s;

    /* 车道迟滞 + 死锁恢复状态 */
    int    committed_lane_side;  /* 0=未初始化 -1=左车道(负y) +1=右车道(正y) */
    double stuck_timer;          /* 近乎静止且卡在车道线附近的累计时间 (秒) */

    uint32_t cycle;

    /* 状态机（反射式生命周期跟踪） */
    ReflectiveStateMachine sm;

    /* 配置参数 */
    double cfg_kp, cfg_ki, cfg_kd;
    double cfg_cruise_speed;
} g;

static double steer_limit_for_speed(double speed_mps, double max_lateral_accel_mps2) {
    double speed = speed_mps;
    if (speed < 2.0) speed = 2.0;
    double limit = atan(max_lateral_accel_mps2 * CONTROL_WHEELBASE_M / (speed * speed));
    if (limit < 0.012) limit = 0.012;
    if (limit > 0.24) limit = 0.24;
    return limit;
}

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* Try binary deserialization (serializer path) */
    {
        Localization loc;
        if (Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.current_speed = loc.v;
            g.ego_x         = loc.x;
            g.ego_y         = loc.y;
            g.ego_heading   = loc.heading;
            g.has_fusion    = 1;
            struct timespec ts_fusion; clock_gettime(CLOCK_MONOTONIC, &ts_fusion);
            g.last_fusion_us = (uint64_t)ts_fusion.tv_sec * 1000000ULL + (uint64_t)ts_fusion.tv_nsec / 1000ULL;
            return;
        }
    }

    /* Fallback: text JSON parsing */
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"v\":")))
        sscanf(p + 4, "%lf", &g.current_speed);
    else if ((p = strstr(d, "speed=")))
        sscanf(p + 6, "%lf", &g.current_speed);
    if ((p = strstr(d, "\"x\":")))       sscanf(p + 4,  "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))       sscanf(p + 4,  "%lf", &g.ego_y);
    if ((p = strstr(d, "\"heading\":"))) sscanf(p + 10, "%lf", &g.ego_heading);
    g.has_fusion = 1;
    struct timespec ts_fusion; clock_gettime(CLOCK_MONOTONIC, &ts_fusion);
    g.last_fusion_us = (uint64_t)ts_fusion.tv_sec * 1000000ULL + (uint64_t)ts_fusion.tv_nsec / 1000ULL;
}

static void on_trajectory(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;

    if (strstr(d, "speed="))
        sscanf(strstr(d, "speed=") + 6, "%lf", &g.target_speed);
    const char* target_speed_json = strstr(d, "\"target_speed\":");
    if (target_speed_json)
        sscanf(target_speed_json + 15, "%lf", &g.target_speed);

    /* 解析第一个路径点的 d 值（横向偏移），格式: [s,d,spd] */
    const char* bracket = strchr(d, '[');
    if (bracket) {
        double s, d_lat, spd;
        if (sscanf(bracket, "[%lf,%lf,%lf]", &s, &d_lat, &spd) >= 2)
            g.lane_d = d_lat;
    }
    /* 无路径点时，读取 failsafe 的 lane_keep_d 作为降级 */
    if (!bracket) {
        const char* p = strstr(d, "\"lane_keep_d\":");
        if (p) {
            double keep_d = 0.0;
            if (sscanf(p + 14, "%lf", &keep_d) >= 1)
                g.lane_d = keep_d;
        }
    }

    g.has_planning = 1;
    struct timespec ts_plan; clock_gettime(CLOCK_MONOTONIC, &ts_plan);
    g.last_planning_us = (uint64_t)ts_plan.tv_sec * 1000000ULL + (uint64_t)ts_plan.tv_nsec / 1000ULL;
}

/* ── vehicle/state 订阅 — 解析障碍物位置 ─────────────────────── */
static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    g.ped_index = -1;
    for (int i = 0; i < MAX_OBS; i++) {
        char key[16];
        int klen = snprintf(key, sizeof(key), "\"ox%d\":", i);
        const char* p = strstr(d, key);
        if (p) {
            g.obs_valid[i] = 1;
            sscanf(p + klen, "%lf", &g.obs_x[i]);
        } else { g.obs_valid[i] = 0; continue; }
        klen = snprintf(key, sizeof(key), "\"oy%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_y[i]);
        klen = snprintf(key, sizeof(key), "\"ov%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_vx[i]);
        /* parse obstacle type to detect pedestrian dynamically */
        klen = snprintf(key, sizeof(key), "\"ot%d\":\"", i);
        if ((p = strstr(d, key))) {
            p += klen;
            const char* end = strchr(p, '"');
            if (end) {
                size_t tlen = (size_t)(end - p);
                if (tlen >= sizeof(g.obs_type[i])) tlen = sizeof(g.obs_type[i]) - 1;
                memcpy(g.obs_type[i], p, tlen);
                g.obs_type[i][tlen] = '\0';
                if (strcmp(g.obs_type[i], "pedestrian") == 0 && g.ped_index < 0)
                    g.ped_index = i;
            }
        }
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
    if (fabs(g.obs_y[pi] - target_lane_y) <= same_lane_tol + 0.8) return 1;
    return fabs(g.obs_y[pi]) < 9.0 && fabs(target_lane_y) <= g.lane_width * 0.5 + 0.2;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* control_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "control");

    const double same_lane_tol = 2.0;
    const double time_headway  = 1.4;
    const double min_gap       = 5.0;
    const double ref_y         = -1.75;  /* Frenet 参考路径在左车道中心 */

    while (!g.should_stop) {
        usleep(50000);  /* 20Hz */
        if (g.should_stop) break;

        g.cycle++;

        /* Reset stale data flags: if no message received for >1000ms, clear flag */
        struct timespec now_ts; clock_gettime(CLOCK_MONOTONIC, &now_ts);
        uint64_t now_us = (uint64_t)now_ts.tv_sec * 1000000ULL + (uint64_t)now_ts.tv_nsec / 1000ULL;
        if (g.has_fusion   && now_us - g.last_fusion_us   > 1000000ULL) g.has_fusion   = 0;
        if (g.has_planning && now_us - g.last_planning_us > 1000000ULL) g.has_planning = 0;

        /* 数据陈旧时不跳过输出——发布安全减速指令，保持下游流水线畅通 */
        if (!g.has_fusion || !g.has_planning) {
            ControlRaw raw;
            raw.seq      = g.cycle;
            raw.throttle = 0.0f;
            raw.brake    = 0.25f;  /* 温和减速，防止无人加速撞前车 */
            raw.steering = (float)g.prev_steer;
            raw.speed    = (float)g.current_speed;
            raw.target   = 0.0f;
            raw.error    = 0.0f;
            memset(raw.mode, 0, sizeof(raw.mode));
            snprintf(raw.mode, sizeof(raw.mode), "DATA_TIMEOUT");

            uint8_t raw_buf[64];
            size_t  raw_len = sizeof(raw_buf);
            ControlRaw_serialize(&raw, raw_buf, &raw_len);
            transport_publish(g.transport, "control/raw_cmd",
                              raw_buf, (uint32_t)raw_len);

            char cmd_text[256];
            snprintf(cmd_text, sizeof(cmd_text),
                     "throttle=0.00 brake=0.25 steer=%.4f "
                     "speed=%.1f target=0.0 error=0.0 mode=DATA_TIMEOUT",
                     g.prev_steer, g.current_speed);
            transport_publish(g.transport, "control/raw_cmd/text",
                              (const uint8_t*)cmd_text, (uint32_t)strlen(cmd_text) + 1);

            if (g.cycle % 20 == 1) {
                LOG_WARN("control", "#%d DATA_TIMEOUT — publishing safe brake (spd=%.1f, steer=%.4f)",
                         g.cycle, g.current_speed, g.prev_steer);
            }
            continue;
        }

        if (g.lc_cooldown > 0.0) g.lc_cooldown -= 0.05;

        double road_center_limit = g.lane_width - 1.0;
        double half_lane = g.lane_width * 0.5;

        /* ── 车道判定加迟滞: 使用"已提交车道", 只有 ego_y 明确越过中心线
         *    ±LANE_HYSTERESIS_M 才切换, 避免 y≈0 处目标车道每帧翻转的抖振 ── */
        if (g.committed_lane_side == 0) {
            g.committed_lane_side = (g.ego_y < 0.0) ? -1 : 1;
        } else if (g.committed_lane_side < 0 && g.ego_y > LANE_HYSTERESIS_M) {
            g.committed_lane_side = 1;
        } else if (g.committed_lane_side > 0 && g.ego_y < -LANE_HYSTERESIS_M) {
            g.committed_lane_side = -1;
        }
        double cruise_lane_y = (g.committed_lane_side < 0) ? -half_lane : half_lane;
        double adjacent_lane_y = -cruise_lane_y;
        if (fabs(g.ego_y) > road_center_limit - 0.4) {
            g.committed_lane_side = (g.ego_y < 0.0) ? -1 : 1;
            cruise_lane_y = (g.committed_lane_side < 0) ? -half_lane : half_lane;
            adjacent_lane_y = -cruise_lane_y;
            g.lc_state = 2;
            g.lc_timer = 0.0;
        }

        /* ── 死锁恢复: 车长时间近乎静止且横向卡在车道线附近 (骑线不动) 时,
         *    强制收敛到最近车道中心并复位变道状态机, 打破 chatter/死锁 ── */
        if (g.current_speed < STUCK_SPEED_MPS && fabs(g.ego_y) < STUCK_LATERAL_M) {
            g.stuck_timer += 0.05;
        } else {
            g.stuck_timer = 0.0;
        }
        if (g.stuck_timer > STUCK_RECOVER_S) {
            g.committed_lane_side = (g.ego_y < 0.0) ? -1 : 1;
            cruise_lane_y = (g.committed_lane_side < 0) ? -half_lane : half_lane;
            adjacent_lane_y = -cruise_lane_y;
            g.lc_state     = 0;
            g.lc_attempted = 0;
            g.lc_cooldown  = 0.0;
            g.lc_timer     = 0.0;
            g.stuck_timer  = 0.0;
            LOG_WARN("control", ">>> STUCK RECOVERY: converge to lane y=%.2f (ego@(%.1f,%.1f))",
                     cruise_lane_y, g.ego_x, g.ego_y);
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
            double ratio = best_gap / safe_gap;
            if (ratio < 0.2) ratio = 0.2;
            acc_target = boost_target * ratio;
            if (acc_target < boost_target * 0.7) blocked = 1;
        }
        /* 状态机: 跟踪障碍物阻塞状态变化 */
        static int was_blocked_sm = 0;
        if (blocked && !was_blocked_sm)
            statem_send_event(&g.sm, CTL_EVENT_OBSTACLE_BLOCKED, NULL);
        else if (!blocked && was_blocked_sm)
            statem_send_event(&g.sm, CTL_EVENT_OBSTACLE_CLEARED, NULL);
        was_blocked_sm = blocked;

        double min_overtake_gap = 22.0 + g.current_speed * 2.6;
        if (min_overtake_gap > 70.0) min_overtake_gap = 70.0;
        if ((g.lc_state == 0) && (g.lc_cooldown <= 0.0) &&
            (!g.lc_attempted) &&
            best_gap > min_overtake_gap && best_gap < 90.0) {
            int lead_is_slow = lead_speed < boost_target - 2.0;
            int front_allows_merge = lane_front_allows_merge(adjacent_lane_y, same_lane_tol, &overtake_need_accel);
            if (lead_is_slow && !lane_has_pedestrian_risk(adjacent_lane_y, same_lane_tol) &&
                lane_rear_safe(adjacent_lane_y, same_lane_tol)) {
                if (!front_allows_merge) overtake_need_accel = 1;
                overtake_worthwhile = 1;
                blocked = 1;
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

        /* ── 自适应变道状态机 ── */
        double effective_target_y = (g.lc_state != 0) ? g.lc_target_y : cruise_lane_y;
        if (fabs(g.ego_y) > road_center_limit - 0.4) {
            effective_target_y = (g.ego_y < 0.0) ? -g.lane_width * 0.5 : g.lane_width * 0.5;
            if (acc_target > 6.0) acc_target = 6.0;
        }
        if (effective_target_y > g.lane_width * 0.5) effective_target_y = g.lane_width * 0.5;
        if (effective_target_y < -g.lane_width * 0.5) effective_target_y = -g.lane_width * 0.5;

        if (blocked && g.lc_state == 0) {
            g.lc_timer += 0.05;
            if (overtake_worthwhile || g.lc_timer > g.blocked_timeout_s) {
                int need_accel = 0;
                int front_allows_merge = lane_front_allows_merge(adjacent_lane_y, same_lane_tol, &need_accel);
                if (!lane_has_pedestrian_risk(adjacent_lane_y, same_lane_tol) &&
                    lane_rear_safe(adjacent_lane_y, same_lane_tol)) {
                    if (!front_allows_merge) need_accel = 1;
                    g.lc_origin_y = cruise_lane_y;
                    g.lc_target_y = adjacent_lane_y;
                    effective_target_y = g.lc_target_y;
                    if (need_accel && acc_target < g.current_speed + 2.0) acc_target = g.current_speed + 2.0;
                    g.lc_state = 1; g.lc_attempted = 1; g.lc_timer = 0;
                    statem_send_event(&g.sm, CTL_EVENT_LANE_CHANGE_START, NULL);
                    LOG_INFO("control", ">>> LANE CHANGE %s%s (cur_gap=%.1f adj_gap=%.1f lead_v=%.1f ego@(%.1f,%.1f) target_y=%.1f)",
                             overtake_worthwhile ? "OVERTAKE" : "BLOCKED",
                             need_accel ? "+ACCEL" : "+CRUISE",
                             best_gap, adjacent_gap, lead_speed, g.ego_x, g.ego_y, effective_target_y);
                } else {
                    LOG_INFO("control", ">>> LANE CHANGE BLOCKED by obstacle in target lane");
                    g.lc_timer = g.blocked_timeout_s;
                }
            }
        } else if (!blocked && g.lc_state == 0) {
            g.lc_timer = 0;
        }

        /* 超车后先稳定巡航，不强制回原车道，避免回切与慢车重叠。
         * 通过重置 lc_attempted 允许后续再次发起变道。 */
        if (g.lc_state == 2) {
            g.lc_wait += 0.05;
            if (g.lc_wait > 8.0 && g.lc_cooldown <= 0.0) {
                g.lc_attempted = 0;
                g.lc_cooldown = 3.0;
                g.lc_wait = 0.0;
            }
        }

        /* 检测变道完成 (横向偏差 < 0.3m) */
        if (g.lc_state == 1 && fabs(g.ego_y - effective_target_y) < 0.3) {
            g.lc_state = 2; g.lc_wait = 0;
            statem_send_event(&g.sm, CTL_EVENT_LANE_CHANGE_DONE, NULL);
            LOG_INFO("control", ">>> lane change complete");
        }
        if (g.lc_state == 3 && fabs(g.ego_y - effective_target_y) < 0.3) {
            g.lc_state = 0;
            g.lc_attempted = 0;
            g.lc_cooldown = 4.0;
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

        /* ── 横向级联 PD：lat_error → psi_des → steer（阻尼消振） ── */
        double steer = 0.0;
        double lat_error = effective_target_y - g.ego_y;
        if (fabs(g.ego_y) > road_center_limit - 0.4) {
            double steer_limit = steer_limit_for_speed(g.current_speed, 2.4);
            steer = (lat_error > 0.0) ? steer_limit : -steer_limit;
            /* 低速时给少许油门使自行车模型能横向移动回车道中心，
             * 避免 speed=0 时永久卡在路边缘的死锁。 */
            if (g.current_speed < 2.5) {
                throttle = 0.18;
                brake = 0.0;
            } else {
                throttle = 0.0;
                if (brake < 0.65) brake = 0.65;
            }
            g.prev_steer = steer;
            mode = "ROAD_GUARD";
        } else {
            /* ── Stanley 式横向控制（收敛，不自激） ──
             * cross-track 项: atan2(k*e, v) 随速度自然衰减 → 高速小幅打方向;
             * heading 项: lat_kd_heading 阻尼抑制航向偏差, 避免极限环振荡。 */
            double cte_term     = atan2(g.lat_kp * lat_error, fmax(g.current_speed, 3.0));
            double heading_term = g.lat_kd_heading * g.ego_heading;
            steer = cte_term - heading_term;
            double steer_limit = steer_limit_for_speed(g.current_speed, 1.4);
            if (steer >  steer_limit) steer =  steer_limit;
            if (steer < -steer_limit) steer = -steer_limit;
            /* 一阶低通滤波，防止跳变 */
            steer = STEER_FILTER_NEW * steer + STEER_FILTER_PREV * g.prev_steer;
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
        transport_publish(g.transport, "control/raw_cmd",
                          raw_buf, (uint32_t)raw_len);

        /* Also publish text format for backward compat (monitor/logging) */
        char cmd_text[256];
        snprintf(cmd_text, sizeof(cmd_text),
                 "throttle=%.2f brake=%.2f steer=%.4f "
                 "speed=%.1f target=%.1f error=%.1f mode=%s",
                 throttle, brake, steer,
                 g.current_speed, acc_target, error, mode);
        transport_publish(g.transport, "control/raw_cmd/text",
                          (const uint8_t*)cmd_text, (uint32_t)strlen(cmd_text) + 1);

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
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", "vehicle/state", NULL };
static const char* s_outputs[] = { "control/raw_cmd", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int control_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    (void)bus; (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.should_stop  = 0;
    g.ped_index    = -1;

    /* 默认 PID 参数 */
    g.cfg_kp = 800.0; g.cfg_ki = 50.0; g.cfg_kd = 100.0;
    g.cfg_cruise_speed = 12.0;
    g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
    g.lat_kp          = 0.5;   /* lateral error → desired heading (rad/m), 与 sim 内置一致 */
    g.lat_kd_heading  = 2.0;   /* heading error → steer, 阻尼增益 */
    g.lane_width = 3.5;
    g.blocked_timeout_s = 2.0;

    /* 注册参数到 param_registry (类型安全，可运行时热重载) */
    param_register_float("control.pid_kp", g.cfg_kp, 0.0, 5000.0, "PID proportional gain");
    param_register_float("control.pid_ki", g.cfg_ki, 0.0, 1000.0, "PID integral gain");
    param_register_float("control.pid_kd", g.cfg_kd, 0.0, 2000.0, "PID derivative gain");
    param_register_float("control.cruise_speed", g.cfg_cruise_speed, 1.0, 50.0, "Target cruise speed m/s");
    param_register_float("control.lane_width", g.lane_width, 2.5, 5.0, "Lane width meters");
    param_register_float("control.lat_kp", g.lat_kp, 0.0, 2.0, "Lateral P gain");
    param_register_float("control.lat_kd_heading", g.lat_kd_heading, 0.0, 5.0, "Heading damping gain");
    param_register_float("control.blocked_timeout_s", g.blocked_timeout_s, 0.5, 30.0, "Blocked timeout seconds");

    /* 运行时从 param_registry 读取 (支持 flowctl param set 热重载) */
    g.kp = param_get_float("control.pid_kp");
    g.ki = param_get_float("control.pid_ki");
    g.kd = param_get_float("control.pid_kd");
    g.cfg_cruise_speed = param_get_float("control.cruise_speed");
    g.lane_width       = param_get_float("control.lane_width");
    g.lat_kp           = param_get_float("control.lat_kp");
    g.lat_kd_heading   = param_get_float("control.lat_kd_heading");
    g.blocked_timeout_s = param_get_float("control.blocked_timeout_s");

    /* 初始化反射式状态机 */
    statem_init(&g.sm, g_ctl_transitions, SM_STATE_INITIALIZED, "control");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);  /* INITIALIZED → RUNNING */

    if (params_json) {
        /* Fallback: override with JSON params if provided (backward compat) */
        const char* p;
        if ((p = strstr(params_json, "\"pid_kp\":")))          sscanf(p + 9, "%lf", &g.cfg_kp);
        if ((p = strstr(params_json, "\"pid_ki\":")))          sscanf(p + 9, "%lf", &g.cfg_ki);
        if ((p = strstr(params_json, "\"pid_kd\":")))          sscanf(p + 9, "%lf", &g.cfg_kd);
        if ((p = strstr(params_json, "\"target_speed\":")))    sscanf(p + 15, "%lf", &g.cfg_cruise_speed);
        if ((p = strstr(params_json, "\"lat_kp\":")))          sscanf(p + 9, "%lf", &g.lat_kp);
        if ((p = strstr(params_json, "\"lat_kd_heading\":")))  sscanf(p + 17, "%lf", &g.lat_kd_heading);
        if ((p = strstr(params_json, "\"lane_change_blocked_timeout_s\":"))) sscanf(p + 32, "%lf", &g.blocked_timeout_s);
        g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_trajectory, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);
    transport_advertise(transport, "control/raw_cmd", 0x2D95C6D3u);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/raw_cmd", 0x2D95C6D3u,
                        CAP_PUBLISHER, 100.0);

    LOG_INFO("control", "initialized (PID: kp=%.0f ki=%.0f kd=%.0f)",
             g.kp, g.ki, g.kd);
    return 0;
}

static int control_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, control_thread, NULL) != 0) return -1;
    LOG_INFO("control", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void control_stop(void)        { g.should_stop = 1; }
static void control_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    LOG_INFO("control", "cleanup done");
}
static int  control_health(void)      { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "control",
    .version       = "1.0.0",
    .description   = "PID longitudinal controller + ACC",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = control_init,
    .start         = control_start,
    .stop          = control_stop,
    .cleanup       = control_cleanup,
    .health        = control_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
