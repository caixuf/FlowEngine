/**
 * control_node.cpp — PID 纵向/横向控制节点插件 (FlowCoro 协程版)
 *
 * 从 control_node.c 迁移而来，采用 CoroutineTask 协程框架：
 *   - co_await sleep_us(50000) 替代 usleep 20Hz 轮询（可被 stop 取消）
 *   - 保留 on_fusion / on_trajectory 回调（on_ref_path 已移除，自推参考路径已断）
 *   - PID + Stanley 横向控制逻辑原样搬入 run()
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
#include "topic_registry.h"
#include "adas_msgs_gen.h"       /* ControlRaw_serialize, CONTROLRAW_TYPE_ID */
#include "degrade_ladder.h"
#include "ltv_mpc.h"
#include "coroutine_task.h"
#include "logger.h"
#include "clock_service.h"
#include "maneuver_tracker.h"      /* 通用机动跟踪器（header-only，替代掉头 5 层 gate） */
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
#include <vector>

namespace {

/* ── 节点本地状态 ───────────────────────────────────────────── */

/* 横向级联 PD 常量 */
#define MAX_PSI_DES_RAD    0.349   /* 最大期望航向角 ≈ ±20° */
/* 低通滤波新值权重：0.5 = -3dB@1.2Hz (20Hz 采样)。
 * 旧值 0.8 (-3dB@2.8Hz) 高频抑制不足, Stanley 控制器在 cte_term 与 heading_term
 * 互相反向时产生 ~1.6Hz 极限环振荡（左摇右晃）。降到 0.5 增加阻尼, 让 steer
 * 平滑过渡, 牺牲少量相位裕度换取稳定性。yaw_damping 配合抑制高频。 */
#define STEER_FILTER_NEW   0.5     /* 低通滤波新值权重 */
#define STEER_FILTER_PREV  0.5     /* 低通滤波旧值权重 */
/* 轴距 (m)：真车默认 2.7，RC 小车在 pipeline_car.json 里通过 params.wheelbase
 * 覆盖为 0.25-0.4。这个宏只作为 g.wheelbase 的初值，运行时由配置注入。 */
#define CONTROL_WHEELBASE_DEFAULT_M 2.7
/* 控制环周期: 20Hz → 50ms。所有计时器累加步长使用此常量, 与实际循环频率保持一致。 */
#define CONTROL_DT_S       0.05

/* 死锁恢复: 车长时间近乎静止时，给一点前向油门打破静摩擦。 */
#define STUCK_SPEED_MPS     0.5    /* 判定"近乎静止"的速度阈值 */
/* 全域速度死锁: 只要速度持续为0超过此秒数就给小油门。 */
#define SPEED_ZERO_RECOVER_S  5.0  /* 全域速度死锁触发阈值 (秒) */
/* ROAD_GUARD 触发阈值: 距道路中心超过此值强制回正。车道判定已移到 planning，
 * 此阈值仅为安全冗余，保持 control 独立于 lane_width/lane_count 配置。 */
#define ROAD_GUARD_THRESHOLD_M 3.0

/* ── 控制节点状态机定义 ─────────────────────────────────────── */

static const TransitionRule g_ctl_transitions[] = {
    { SM_STATE_INITIALIZED, SM_EVENT_START,             SM_STATE_RUNNING,    "INIT→RUNNING",          false },
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

    /* TaskBase 包装器（由 EXPORT_COROUTINE_TASK 宏创建） */
    struct control_Wrapper* task_wrapper{nullptr};

    /* PID 状态 */
    double kp{0}, ki{0}, kd{0};
    double integral{0};
    double prev_error{0};
    uint64_t last_ctrl_us{0};        /* 上次控制调度的单调时间戳（40Hz 限速用） */
    /* 横向级联 PD 状态 */
    double lat_kp{0};          /* lateral error → desired heading (rad/m) */
    double lat_kd_heading{0};  /* heading error → steer (阻尼) */
    double yaw_damping{0};     /* yaw_rate → steer 阻尼, 抑制极限环振荡 */
    double ego_heading{0};     /* 从 fusion 获取的航向角 (rad) */
    double ego_yaw_rate{0};    /* 从 fusion 获取的偏航角速度 (rad/s) */
    double prev_steer{0};

    /* ── 真车级横向控制（替代 Stanley 极限环补偿）── */
    double lat_lookahead_gain{0.8};   /* 前视距离系数 (s) */
    double k_v_lat{0.22};             /* 横向速度阻尼增益（自标定最优值，0.2-0.3 推荐区间）*/

    /* A10 横向速度规划 PD 增益（可通过 pipeline.json 热重载） */
    double k_vy{0.35};               /* v_y_des 位置增益：v_y_des = k_vy*lat_error - k_d*v_lat */
    double k_vy_damp{0.6};           /* v_y_des 速度阻尼增益 */

    /* 从 topic 解析的值 */
    double current_speed{0};
    double target_speed{0};
    int    has_target_speed{0};  /* trajectory 回调是否已设置 target_speed */
    double ego_x{0}, ego_y{0};
    double lane_d{0};          /* 从 trajectory 解析的横向偏移（Frenet d） */
    double target_path_y{0};   /* 同一前视点的全局 y，避免最近点 rc_y + 前视点 lane_d 混拼 */
    double target_path_x{0};   /* 前视点全局 x（机动控制律横向误差投影用） */
    double target_path_heading{0};
    double target_path_kappa{0};
    double target_road_center_y{0};
    double road_center_y{0};   /* 当前帧目标道路中心 y（来自 trajectory 第一个点，供 fallback 使用） */
    char   driving_mode[32]{}; /* 从 planning 广播的驾驶模式（如 "NOA:READY"），仅用于日志/透传 */
    int8_t beh_command{0};     /* 最新 planning/behavior 指令（BehaviorCommand enum：LEFT_CHANGE=2…），用于转向灯 */
    int8_t gear{GEAR_DRIVE};   /* 当前档位（机动期 = ManeuverTracker 输出镜像；巡航 = DRIVE） */
    bool   gear_pending{false}; /* 换挡待决：想换挡但带速，本帧刹停（tracker 输出镜像） */
    bool   maneuver_mode{false}; /* 机动轨迹（掉头/倒车）：轨迹含倒车点或曲率超巡航转向域。
                                  * 驱动 safety/MPC/label gate（+MANEUVER、ROAD_GUARD 豁免、
                                  * 左转向灯、steer 限幅 0.60）。档位/横向由 ManeuverTracker 负责。 */
    maneuver::ManeuverTracker       mv_tracker;   /* 通用机动跟随器（弧长推进 + 挡位 + 横向） */
    maneuver::ManeuverTrackerParams mv_params;    /* 热重载调参（control.mv_*） */

    volatile int has_fusion{0};
    volatile int has_planning{0};
    uint64_t last_fusion_us{0};    /* monotonic timestamp of last fusion message */
    uint64_t last_vstate_us{0};    /* monotonic timestamp of last vehicle/state message */
    uint64_t last_planning_us{0};  /* monotonic timestamp of last planning message */

    /* LDW 车道偏离预警 */
    double ldw_threshold{0.5};            /* 横向偏离阈值 (m)，|cte| 超此值发警告 */
    double ldw_min_speed{1.0};            /* LDW 生效最低速度 (m/s)，低于此速不告警（停车/起步不算偏离） */
    double ldw_cooldown{2.0};            /* 告警冷却期 (s)，避免刷屏 */
    double ldw_last_warn_time{0};        /* 上次告警时间 (s) */

    /* 死锁恢复状态 */
    double stuck_timer{0};          /* 近乎静止的累计时间 (秒) */
    double speed_zero_timer{0};     /* 全域速度死锁: 无论 y 位置, 速度持续为0的累计时间 (秒) */
    double mrm_stall_us{0};         /* MRM 停稳累计时长 (us)，停稳 3s 自动恢复降级 */

    uint32_t cycle{0};

    /* 状态机（反射式生命周期跟踪） */
    ReflectiveStateMachine sm{};

    /* 配置参数 */
    double cfg_kp{0}, cfg_ki{0}, cfg_kd{0};
    double cfg_cruise_speed{0};
    double wheelbase{CONTROL_WHEELBASE_DEFAULT_M};  /* 轴距 (m)：真车 2.7，RC 小车 0.25-0.4 */

    /* ego route-following 参考路径：来自 planning/trajectory，on_trajectory
     * 回调将其存为 ref_path。Stanley 横向控制用最近点的 (y, h, kappa) 替代
     * curve_* 单段直线参考，让 ego 能跟随多 edge + fork 路网（如匝道分叉）。
     * 不再独立订阅 road/geometry 或 road/ref_path。 */
    struct RefPt { double x, y, h, kappa, rs, l; };  /* l = Frenet 横向偏移 */
    std::vector<RefPt> ref_path;
    uint64_t last_ref_path_us{0};
    pthread_mutex_t ref_path_mtx = PTHREAD_MUTEX_INITIALIZER;

    /* NOA Phase 3.4: 弯道曲率前馈权重提升参数。
     * 当道路曲率半径 R ≤ curve_ff_boost_radius_m 时，前馈权重 × curve_ff_boost_factor，
     * 让 Stanley 控制器在急弯（如匝道 R=45m）预先打方向，而非等 CTE 累积后反应。
     * 默认 R≤60m 触发 ×1.5 提升，可经 params 配置覆盖。 */
    double curve_ff_boost_radius_m{60.0};
    double curve_ff_boost_factor{1.5};

    /* LTV MPC 控制器（§10 替代已删除的 mpc_controller + LQR） */
    LtvMpcSolver* ltv_mpc{nullptr};
    int use_ltv_mpc{0};         /* 是否启用 LTV MPC */
    LtvMpcConfig ltv_mpc_cfg;   /* MPC 配置 */

    };

ControlContext g;

static double steer_limit_for_speed(double speed_mps, double max_lateral_accel_mps2) {
    double speed = speed_mps;
    if (speed < 2.0) speed = 2.0;
    double limit = atan(max_lateral_accel_mps2 * g.wheelbase / (speed * speed));
    if (limit < 0.016) limit = 0.016;
    if (limit > 0.16) limit = 0.16;
    return limit;
}

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */

    /* cJSON parsing (fusion/localization now publishes cJSON) */
    {
        cJSON* root = cJSON_Parse((const char*)msg->data);
        if (root) {
            /* vehicle/state 近期到达时不覆盖（同 behavior/planning 的修复） */
            uint64_t now = clock_now_us();
            bool vstate_recent = (g.last_vstate_us != 0 && now - g.last_vstate_us < 200000ULL);
            if (!vstate_recent) {
                cJSON* j;
                j = cJSON_GetObjectItemCaseSensitive(root, "v");
                if (cJSON_IsNumber(j)) {
                    double fv = j->valuedouble;
                    /* 速度异常值过滤：EKF 偶尔输出速度尖峰（>50 m/s），
                     * 直接使用会触发 ROAD_GUARD 刹车。丢弃明显异常值，
                     * 保持上一帧速度（control 20Hz，丢失一帧速度无影响）。 */
                    if (fv >= 0.0 && fv <= 50.0)
                        g.current_speed = fv;
                }
                j = cJSON_GetObjectItemCaseSensitive(root, "x");
                if (cJSON_IsNumber(j)) g.ego_x = j->valuedouble;
                j = cJSON_GetObjectItemCaseSensitive(root, "y");
                if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
                j = cJSON_GetObjectItemCaseSensitive(root, "heading");
                if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
                j = cJSON_GetObjectItemCaseSensitive(root, "yaw_rate");
                if (cJSON_IsNumber(j)) g.ego_yaw_rate = j->valuedouble;
            }
            cJSON_Delete(root);
        }
        g.has_fusion = 1;
        g.last_fusion_us = clock_now_us();
    }
}

/* ── vehicle/state 订阅 — 用 flowsim 真值覆盖 ego 位置 ── */
static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    j = cJSON_GetObjectItemCaseSensitive(root, "x");
    if (cJSON_IsNumber(j)) g.ego_x = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "y");
    if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "spd");
    if (cJSON_IsNumber(j)) g.current_speed = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "hdg");
    if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
    /* 车参同步：场景 ego 块（单一事实源）→ flowsim 广播 → 此处覆盖。
     * 广播优先于 pipeline params（params.wheelbase 仅在没有广播时生效，
     * 兼容旧 pipeline_car.json RC 小车配置；新配置应写场景 ego 块）。 */
    j = cJSON_GetObjectItemCaseSensitive(root, "wheelbase");
    if (cJSON_IsNumber(j) && j->valuedouble > 0.0) {
        g.wheelbase = j->valuedouble;
    }
    g.last_vstate_us = clock_now_us();
    g.has_fusion = 1;
    g.last_fusion_us = clock_now_us();
    cJSON_Delete(root);
}

/* planning/behavior 订阅：读 Behavior.command（BehaviorCommand enum），
 * 用于转向灯指令。behavior_planner_node 每 50ms 发布一次类型化消息，
 * 比 behavior/state 监控 JSON（0.5s）响应快、无字符串耦合、无双线程
 * 撕裂读（char 缓冲 strcmp → int8 enum 原子读）。 */
static void on_planning_behavior(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size == 0) return;
    Behavior beh;
    if (Behavior_deserialize(&beh, (const uint8_t*)msg->data, msg->data_size) != 0) return;
    g.beh_command = beh.command;
}

static void on_trajectory(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size == 0) return;

    Trajectory traj;
    memset(&traj, 0, sizeof(traj));
    if (Trajectory_deserialize(&traj, (const uint8_t*)msg->data, msg->data_size) != 0) {
        return;
    }

    uint32_t n_pts = traj.point_count;
    if (n_pts == 0 || n_pts > 64) return;

    /* §5: 退化轨迹检测 — valid==0 或全零 (x,y) 时静默丢弃并触发降级 */
    if (!traj.valid) {
        LOG_WARN("control", "trajectory valid=0 — skipping, triggering L1 degrade");
        degrade_set_level(DEGRADE_L1, DEGRADE_REASON_PLANNING_TO);
        return;
    }
    /* 全零检查：n_pts >= 1 时都查。旧代码只查 n_pts > 1，导致单点
     * (0,0) 轨迹绕过检查 → Stanley 拿到 ref_path=[(0,0)] → 巨大 CTE
     * → 疯狂转向。 */
    {
        int all_zero = 1;
        for (uint32_t i = 0; i < n_pts; i++) {
            if (fabs((double)traj.points[i].x) > 0.001 ||
                fabs((double)traj.points[i].y) > 0.001) {
                all_zero = 0;
                break;
            }
        }
        if (all_zero) {
            LOG_WARN("control", "trajectory all-zero (x/y) — skipping, triggering L1 degrade");
            degrade_set_level(DEGRADE_L1, DEGRADE_REASON_PLANNING_TO);
            return;
        }
    }

    /* ── 机动分类器（留在 control）：这是机动轨迹吗？──
     * whole-trajectory any(v<0) 或 max_kappa>0.12。只决定"是否调用
     * ManeuverTracker"，不再推导 gear —— gear 由 tracker 的 s+exec v 决定，
     * Phase 2 前进段误判倒挡的旧 bug（曾致 y=11.13 出路面）结构上消失。 */
    bool has_reverse = false;
    double max_kappa = 0.0;
    for (uint32_t i = 0; i < n_pts; i++) {
        double k = fabs((double)traj.points[i].kappa);
        if (k > max_kappa) max_kappa = k;
        if ((double)traj.points[i].v < -0.1) has_reverse = true;
    }
    bool mv = has_reverse || (max_kappa > 0.12);
    if (mv != g.maneuver_mode) {
        LOG_WARN("control", "[MANEUVER] %d->%d has_rev=%d max_kappa=%.3f n_pts=%u",
                 (int)g.maneuver_mode, (int)mv, (int)has_reverse, max_kappa, n_pts);
    }
    if (mv && !g.maneuver_mode) {
        g.mv_tracker.init(traj.points, (int)n_pts, g.wheelbase);
        g.mv_tracker.setParams(g.mv_params);
    } else if (mv) {
        g.mv_tracker.setTrajectory(traj.points, (int)n_pts, g.wheelbase);
        g.mv_tracker.setParams(g.mv_params);
    } else if (g.maneuver_mode) {
        g.gear = GEAR_DRIVE;        /* 机动结束：复位巡航挡位基线 */
        g.gear_pending = false;
    }
    g.maneuver_mode = mv;

    /* 目标速度：巡航 = 轨迹末点（跟随减速/加速意图）；
     * 机动 = ManeuverTracker 每帧刷新（见 tick）。 */
    if (!mv) {
        g.target_speed = (double)traj.points[n_pts - 1].v;
    }
    g.has_target_speed = 1;

    /* lane_d / target_path_*：巡航 = 0.5s t_rel 前视点（原样保留，向后兼容）；
     * 机动 = tracker 前视点（tracker 横向反馈用的同一几何）。 */
    uint32_t d_idx = 0;
    if (mv) {
        const maneuver::TrackPoint& lp = g.mv_tracker.lookaheadPoint();
        g.lane_d = lp.l;
        g.target_path_y = lp.y;
        g.target_path_x = lp.x;
        g.target_path_heading = lp.heading;
        g.target_path_kappa = lp.kappa;
    } else {
        for (uint32_t i = 0; i < n_pts; i++) {
            if ((double)traj.points[i].t_rel_us >= 500000.0) {  /* 0.5s */
                d_idx = i;
                break;
            }
        }
        if (d_idx >= n_pts) d_idx = n_pts - 1;
        g.lane_d = (double)traj.points[d_idx].l;
        g.target_path_y = (double)traj.points[d_idx].y;
        g.target_path_x = (double)traj.points[d_idx].x;
        g.target_path_heading = (double)traj.points[d_idx].heading;
        g.target_path_kappa = (double)traj.points[d_idx].kappa;
    }
    g.target_road_center_y = g.target_path_y - g.lane_d * cos(g.target_path_heading);
    if (g.cycle > 900 && g.cycle < 1000) {
        LOG_WARN("control", "[DBG traj] cycle=%d pts=%d v_last=%.2f d_la=%.2f valid=%d",
                 g.cycle, n_pts, (double)traj.points[n_pts - 1].v,
                 (double)traj.points[d_idx].l, traj.valid);
    }

    /* 存储路径点供 Stanley 横向控制使用 */
    pthread_mutex_lock(&g.ref_path_mtx);
    g.ref_path.clear();
    for (uint32_t i = 0; i < n_pts; i++) {
        ControlContext::RefPt rp;
        rp.x = (double)traj.points[i].x;
        rp.y = (double)traj.points[i].y;
        rp.h = (double)traj.points[i].heading;
        rp.kappa = (double)traj.points[i].kappa;
        rp.rs = (double)traj.points[i].s;
        rp.l = (double)traj.points[i].l;
        g.ref_path.push_back(rp);
    }
    pthread_mutex_unlock(&g.ref_path_mtx);

    g.last_ref_path_us = clock_now_us();  /* query_ref_at 陈旧检查依赖此时间戳 */
    g.has_planning = 1;
    g.last_planning_us = clock_now_us();
}

/* on_ref_path 已移除：规划层通过 planning/trajectory 下发轨迹，
 * control 不独立订阅 road/ref_path。ref_path 仅由 on_trajectory 回调填充。 */

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
    /* 返回道路中心 y（= 轨迹点 y - 横向偏移 d 沿法线投影）。
     * 轨迹点 y = road_center_y + d * cos(theta)，
     * 所以 road_center_y = y - d * cos(theta) = y - l * cos(h)。
     *
     * 旧实现直接返回 best->y（含偏移），导致：
     *   cruise_lane_y = (road_center_y + d) + lane_d = 双重计算偏移
     *   → lat_error 随 ego 偏移正反馈发散 → 车飘到 y=-135m */
    out_y     = best->y - best->l * cos(best->h);
    out_h     = best->h;
    out_kappa = best->kappa;
    pthread_mutex_unlock(&g.ref_path_mtx);
    return true;
}

/* ── 协程任务 ────────────────────────────────────────────────── */

class ControlTask : public CoroutineTask {
public:
    ControlTask(MessageBus* bus) : CoroutineTask(bus) {}

    void set_params(Transport* transport) {
        transport_ = transport;
    }

protected:
    Task run() override {
        /* BusQueueBridge 常驻订阅，替代 select_for 每次循环反复注册/退订 */
        BusQueueBridge ctrl_bridge(bus(),
            {TOPIC_FUSION_LOCALIZATION, TOPIC_PLANNING_TRAJECTORY});
        while (!should_stop()) {
            /* recv_any_for: 等待 fusion 或 planning 消息（消息驱动），
             * 50ms 超时兜底保持 DATA_TIMEOUT fallback 及时性。 */
            (void)co_await ctrl_bridge.recv_any_for(50000);
            if (should_stop()) break;

            /* 40Hz rate limit */
            uint64_t _ctrl_now = clock_now_us();
            if (_ctrl_now - g.last_ctrl_us < 25000) continue;
            g.last_ctrl_us = _ctrl_now;

            g.cycle++;

            /* §11.2: heartbeat 上报 — monitor_node 的 degrade_supervisor_tick 据此检测超时 */
            degrade_supervisor_record_heartbeat("control_node", clock_now_us() / 1000);

            /* 热重载：每帧从 param_registry 重新读取参数，支持 flowctl param set 运行时修改 */
            g.kp = param_get_float("control.pid_kp");
            g.ki = param_get_float("control.pid_ki");
            g.kd = param_get_float("control.pid_kd");
            g.cfg_cruise_speed = param_get_float("control.cruise_speed");
            g.lat_kp           = param_get_float("control.lat_kp");
            g.lat_kd_heading   = param_get_float("control.lat_kd_heading");
            g.yaw_damping      = param_get_float("control.yaw_damping");
            g.lat_lookahead_gain = param_get_float("control.lat_lookahead_gain");
            g.k_v_lat          = param_get_float("control.k_v_lat");
            g.k_vy             = param_get_float("control.k_vy");
            g.k_vy_damp        = param_get_float("control.k_vy_damp");
            /* ManeuverTracker 调参热重载（control.mv_*） */
            g.mv_params.heading_gate_rad = param_get_float("control.mv_heading_gate_rad");
            g.mv_params.diverge_guard_rad= param_get_float("control.mv_diverge_guard_rad");
            g.mv_params.lookahead_m      = param_get_float("control.mv_lookahead_m");
            g.mv_params.speed_scan_m     = param_get_float("control.mv_speed_scan_m");
            g.mv_params.speed_floor_mps  = param_get_float("control.mv_speed_floor_mps");
            g.mv_params.gear_v_threshold = param_get_float("control.mv_gear_v_threshold");
            g.mv_params.gear_pending_speed=param_get_float("control.mv_gear_pending_speed");
            g.mv_params.lat_gain_dh      = param_get_float("control.mv_lat_gain_dh");
            g.mv_params.lat_gain_elat    = param_get_float("control.mv_lat_gain_elat");
            g.mv_params.max_steer        = param_get_float("control.mv_max_steer");
            g.mv_tracker.setParams(g.mv_params);
            /* Reset stale data flags: if no message received for >1000ms, clear flag */
            uint64_t now_us = clock_now_us();
            if (g.has_fusion   && now_us - g.last_fusion_us   > 1000000ULL) g.has_fusion   = 0;
            if (g.has_planning && now_us - g.last_planning_us > 1000000ULL) g.has_planning = 0;

            /* 数据陈旧时不跳过输出——发布安全减速指令，保持下游流水线畅通 */
            if (!g.has_fusion || !g.has_planning) {
                /* DATA_TIMEOUT fallback: 用最后缓存的 trajectory 路径点做横向保持。
                 * 控制层不做独立车道判定——committed_lane_side 已移到 planning。 */
                double fb_road_c = 0.0, fb_road_heading = 0.0, fb_kappa_unused = 0.0;
                if (g.has_fusion) {
                    if (!query_ref_at(g.ego_x, g.ego_y, fb_road_c, fb_road_heading, fb_kappa_unused)) {
                        /* ref_path 不可用 → 保持当前位置，不自推车道参考 */
                        fb_road_c = g.ego_y;
                        fb_road_heading = g.ego_heading;
                    }
                }
                /* 无 planning 时保持当前车道（road_center + lane_d），非自推目标车道。
                 * fb_road_c 是道路中心（0），lane_d 是上次轨迹的横向偏移（如 -5.25）。
                 * 旧实现用 fb_road_c 做目标 → 车被拉向道路中心而非当前车道。 */
                double fb_target_y = (g.has_fusion && g.ref_path.size() > 0)
                                     ? fb_road_c + g.lane_d
                                     : g.ego_y;
                if (g.has_fusion && g.ref_path.size() > 0) {
                    g.road_center_y = fb_road_c;
                }
                double fb_lat_error = fb_target_y - g.ego_y;
                double fb_cte_term  = atan2(g.lat_kp * fb_lat_error, fmax(g.current_speed, 3.0));
                /* fallback heading_term: 同主循环一样加 wrap + junction 守卫 */
                double fb_ref_h_eff = fb_road_heading;
                {
                    double dh = fb_ref_h_eff - g.ego_heading;
                    while (dh >  M_PI) dh -= 2.0 * M_PI;
                    while (dh < -M_PI) dh += 2.0 * M_PI;
                    if (fabs(dh) > 0.5) {
                        fb_ref_h_eff = g.ego_heading;
                    }
                }
                double fb_heading_term = g.lat_kd_heading * (g.ego_heading - fb_ref_h_eff);
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
                /* 无条件发布：与正常路径一致，depth+drop_oldest 兜底而非跳过 */
                transport_publish(transport_, TOPIC_CONTROL_RAW_CMD,
                                  raw_buf, (uint32_t)raw_len);

                char cmd_text[256];
                snprintf(cmd_text, sizeof(cmd_text),
                         "throttle=0.00 brake=0.25 steer=%.4f "
                         "speed=%.1f target=%.1f error=%.1f mode=DATA_TIMEOUT",
                         fb_steer, g.current_speed, fb_target_y, fb_lat_error);
                transport_publish(transport_, TOPIC_CONTROL_RAW_CMD_TEXT,
                                  (const uint8_t*)cmd_text, (uint32_t)strlen(cmd_text) + 1);

                if (g.cycle % 20 == 1) {
                    LOG_WARN("control", "#%d DATA_TIMEOUT — lane-following fallback "
                             "(spd=%.1f, steer=%.4f, target_y=%.2f, err=%.2f)",
                             g.cycle, g.current_speed, fb_steer, fb_target_y, fb_lat_error);
                }
                continue;
            }

            /* 道路几何参数：仅来自 planning/trajectory 的 ref_path 参考点。
             * 控制层不再独立计算 road_center_y，不做车道判定。
             *
             * 优先用 query_ref_at 返回的**本地**参考（离 ego 最近的轨迹点）：
             * road_c/heading/kappa 都在 ego 当前位置，弯道处 lat_error 才是
             * 真横向误差。旧实现一律覆盖成轨迹 0.5s 前视点（target_path_*）——
             * 前视点在弯道上比 ego 当前位置高 → lat_error 虚高 → 车持续往
             * 弯内侧漂，S 弯里稳态偏出 ~3m（贴着道路中心而非目标车道）。
             * 只有本地查询失败（轨迹稀疏/ego 偏离>5m）才回退前视点。 */
            double ref_road_heading = 0.0;
            double ref_kappa = 0.0;
            double road_c = g.road_center_y;  /* 默认保持上一帧值 */
            bool ref_ok = query_ref_at(g.ego_x, g.ego_y,
                                       road_c, ref_road_heading, ref_kappa);
            if (!ref_ok) {
                /* ref_path 不可用：回退到轨迹前视点几何，否则 heading/kappa 归零 */
                ref_road_heading = 0.0;
                ref_kappa = 0.0;
                if (g.has_planning) {
                    road_c = g.target_road_center_y;
                    ref_road_heading = g.target_path_heading;
                    ref_kappa = g.target_path_kappa;
                }
            }
            g.road_center_y = road_c;

            /* 车道保持目标：本地道路中心 + 轨迹 lane_d。
             * 控制层不做独立车道判定——committed_lane_side 已移到 planning。
             *
             * 目标用本地 road_c（query_ref_at 最近点），不用轨迹前视点
             * target_path_y：弯道上前视点 y 高于 ego 当前位置，直用会让
             * lat_error 虚高 → 车往弯内侧漂（S 弯稳态偏 ~3m）。本地
             * road_c + lane_d·cos(h) 就是 ego 所在弧位处的车道中心。 */
            double cruise_lane_y = g.has_planning
                ? (g.road_center_y + g.lane_d * cos(ref_road_heading))
                : g.ego_y;
            /* 出路沿恢复：不管有没有 planning，只要 |ego_y| >> 道路范围就强制回车道。
             * 出路沿后 Frenet 仍可能输出轨迹（投影到参考线外推），让 has_planning=1，
             * 旧 recovery 不触发。必须无条件拦截。
             * 2026-08-05 去硬编码：旧实现 -1.75 写死"前进车道"，掉头返程（西行）
             * 出路沿时把车拽向对向侧（方向盲）。改为恢复 ego 所在侧的第一条车道
             * （road_center ± 半车道宽），前进/返程自洽。control 刻意不订阅
             * lane_width（保持独立），1.75 = 标准 3.5m 车道半宽。 */
            if (fabs(g.ego_y - g.road_center_y) > 15.0) {
                const double side = (g.ego_y >= g.road_center_y) ? 1.0 : -1.0;
                cruise_lane_y = g.road_center_y + side * 1.75;
                g.integral = 0;  /* 出路沿时清零积分，防止恢复后积分饱和 */
            }

            /* ── 死锁恢复: 车长时间近乎静止时，给一点前向油门打破静摩擦 ── */
            if (fabs(g.current_speed) < STUCK_SPEED_MPS) {
                g.stuck_timer += CONTROL_DT_S;
            } else {
                g.stuck_timer = 0.0;
            }

            /* ── 全域速度死锁恢复: 无论 y 位置, 速度持续为0超过阈值就给小油门 ── */
            if (fabs(g.current_speed) < STUCK_SPEED_MPS) {
                g.speed_zero_timer += CONTROL_DT_S;
            } else {
                g.speed_zero_timer = 0.0;
            }

            /* ── 纵向控制：Apollo 原则 —— control 是纯轨迹跟随器 ──
             * 速度唯一来源是 planning 轨迹末点 v（g.target_speed）。
             * control 不做速度决策：没有 cfg_cruise_speed 覆盖、没有 boost、没有 overspeed 降档。
             *
             * 无轨迹时（has_planning=0）acc_target=0 → 匀减速停车。
             * 这里的 has_planning 检查在上方 DATA_TIMEOUT 分支已 continue，
             * 所以走到这里时 has_planning 必为 1。但保险起见仍做检查。 */
            /* ── 机动执行：ManeuverTracker 每帧推进 s + 算 steer/target/gear ──
             * 放在 acc_target 之前：让下方纵向 PID（含 reverse mirror）和 MRM
             * 叠加读到新鲜的 target_speed / gear。tracker 只做跟随，纵向 PID 留在
             * control（复用已验证的 reverse mirror + anti-windup + SHIFT_STOP）。 */
            maneuver::ManeuverResult mv_res;
            if (g.maneuver_mode && g.mv_tracker.hasTrajectory()) {
                mv_res = g.mv_tracker.tick(g.ego_x, g.ego_y, g.ego_heading,
                                           g.current_speed, CONTROL_DT_S);
                g.gear         = (mv_res.gear == -1) ? GEAR_REVERSE : GEAR_DRIVE;
                g.gear_pending = mv_res.gear_pending;
                g.target_speed = mv_res.target_speed;   /* 覆盖 on_trajectory 的巡航末点 */
                g.prev_steer   = mv_res.steer;          /* 平滑机动→巡航 */
                /* 刷新参考几何跟随 s（planning 未必每帧都发新轨迹） */
                const maneuver::TrackPoint& lp = g.mv_tracker.lookaheadPoint();
                g.lane_d = lp.l;
                g.target_path_y = lp.y;
                g.target_path_x = lp.x;
                g.target_path_heading = lp.heading;
                g.target_path_kappa = lp.kappa;
                g.target_road_center_y = g.target_path_y - g.lane_d * std::cos(g.target_path_heading);
            }

            double acc_target;
            if (g.has_planning && g.has_target_speed) {
                acc_target = g.target_speed;  /* 纯轨迹跟随 */
            } else {
                acc_target = 0.0;  /* 无轨迹 → 停车 */
            }
            /* 停车指令时清 PID 积分，抗 windup */
            if (acc_target < 0.01 && g.integral > 0) {
                g.integral = 0;
            }

            /* 横向目标：直接使用 trajectory 提供的 lane_d（planning 负责车道决定）。
             * 无变道场景下, cruise_lane_y = road_center_y + lane_d 即为目标车道中心。 */
            double effective_target_y = cruise_lane_y;

            /* §11.2 MRM 前置：降级必须在 PID 之前生效。
             * 职责分明：降级速度上限的唯一权威 = degrade_ladder 的 l1_speed_limit
             * （L2=3.0 爬行 / L3=0 停车）。control 不得自行硬编码"L2→停车"——
             * 那会把瞬时心跳抖动（WSL 过载 500ms）放大成永久趴窝。
             *
             * 自动恢复：L2/L3 均支持 3s 停稳后清降级。L2 原只限速不恢复，
             * 导致碰撞后 safety_control 设 L2 → 车卡在 0 永不解锁
             * （2026-08-03 事故链：冷启动碰撞 → L2 → 永久 MRM）。 */
            {
                DegradeState* ds = degrade_global_state();
                bool stalled = (fabs(g.current_speed) < 0.5);
                if (ds->degrade_level >= DEGRADE_L3) {
                    g.target_speed = 0.0;
                    acc_target = 0.0;
                    g.integral = 0;
                    if (stalled) {
                        g.mrm_stall_us += CONTROL_DT_S * 1e6;
                        if (g.mrm_stall_us > 3000000.0) {
                            degrade_clear();
                            g.mrm_stall_us = 0;
                            g.integral = 0;
                            LOG_WARN("control",
                                     "MRM auto-recover(L3): stalled 3s at spd=%.1f — degrade cleared",
                                     g.current_speed);
                        }
                    } else {
                        g.mrm_stall_us = 0;
                    }
                } else if (ds->degrade_level >= DEGRADE_L2) {
                    double lim = ds->l1_speed_limit > 0.0 ? ds->l1_speed_limit : 3.0;
                    if (acc_target > lim) acc_target = lim;   /* 爬行，不停车 */
                    /* L2 自动恢复：停稳 3s 后清降级。与 L3 同理——
                     * 碰撞后 safety_control 设 L2，车已停但降级永不解锁。 */
                    if (stalled) {
                        g.mrm_stall_us += CONTROL_DT_S * 1e6;
                        if (g.mrm_stall_us > 3000000.0) {
                            degrade_clear();
                            g.mrm_stall_us = 0;
                            g.integral = 0;
                            LOG_WARN("control",
                                     "MRM auto-recover(L2): stalled 3s at spd=%.1f — degrade cleared",
                                     g.current_speed);
                        }
                    } else {
                        g.mrm_stall_us = 0;
                    }
                } else {
                    g.mrm_stall_us = 0;
                }
            }

            /* ── target 加速限幅（2026-08-05，planning 重生后）──
             * ST 图每 0.5s 重规划 + behavior 状态切换会让 target_speed 突跳
             * （实测 Δ17-20 m/s），PID 直接追 → 油门/刹车突变 → 车一顿一顿。
             * 不对称限幅：正向(加速)限 2 m/s²（平滑起步，防冲），负向(减速)
             * 不限（红灯/障碍的急刹是安全必需，不能平滑掉）。
             * g.target_slew_v 跨帧保持，仅限 planning 轨迹目标（acc_target）。 */
            {
                static double target_slew_v = 0.0;   /* 上一帧平滑后的目标 */
                static int    target_slew_init = 0;
                if (!target_slew_init) {
                    target_slew_v = acc_target;
                    target_slew_init = 1;
                }
                double dv = acc_target - target_slew_v;
                const double MAX_ACCEL = 2.0;        /* m/s² 正向限幅 */
                if (dv > MAX_ACCEL * CONTROL_DT_S)
                    dv = MAX_ACCEL * CONTROL_DT_S;
                /* 负向不限（急刹必需） */
                target_slew_v += dv;
                acc_target = target_slew_v;
            }

            double error = acc_target - g.current_speed;
            double lat_error = effective_target_y - g.ego_y;
            /* 横向误差投影到参考线左法向：n̂=(−sinθ,cosθ)，y 差分量 = cosθ。
             * lat_error 是世界系 y 差，Stanley/MPC/psi_des 全链假设"正误差=左打舵"，
             * 该假设仅在车头朝 +x（θ≈0）成立；掉头返程 θ≈π 时符号语义翻转，
             * 控制器会稳定在镜像平衡点——目标 y=+1.75 却收敛到 y=-1.2 对向
             * 车道（2026-08-03 demo13 实测逆行）。投影后对两个方向都自洽。 */
            double lat_err_n = lat_error * cos(ref_road_heading);
            if (g.cycle % 100 == 0 || fabs(lat_error) > 0.5) {
                LOG_WARN("control", "[DBG_LAT] cyc=%d lane_d=%.2f rc_y=%.2f tgt_y=%.2f ego_y=%.2f lat_err=%.2f spd=%.1f gear=%d",
                         g.cycle, g.lane_d, g.road_center_y, effective_target_y, g.ego_y,
                         lat_error, g.current_speed, (int)g.gear);
            }
            double throttle = 0, brake = 0, steer = 0;
            const char* mode = "NONE";

            /* PID 纵向 — 倒车时镜像速度符号，让 PID 始终在正域工作。
             * 倒车：target=-3.0, current=-1.0 → pid_target=3.0, pid_current=1.0
             * → error=2.0 → output>0 → throttle=+ → 最后取反为负油门。 */
            double pid_target = acc_target;
            double pid_current = g.current_speed;
            bool is_reverse = (g.gear == GEAR_REVERSE);
            if (is_reverse) {
                pid_target = -acc_target;
                pid_current = -g.current_speed;
            }
            double pid_error = pid_target - pid_current;

            g.integral += pid_error * 0.05;
            if (g.integral > 500)  g.integral = 500;
            if (g.integral < -200) g.integral = -200;

            double derivative = (pid_error - g.prev_error) / 0.05;
            double output = g.kp * pid_error + g.ki * g.integral + g.kd * derivative;

            if (output > 0) {
                throttle = output / 5000.0;
                if (throttle > 1.0) throttle = 1.0;
                brake = 0;
                mode = (pid_error < 1.0) ? "HOLD" : (is_reverse ? "REV_ACCEL" : "ACCEL");
            } else {
                throttle = 0;
                brake = (-output) / 8000.0;
                if (brake > 1.0) brake = 1.0;
                mode = is_reverse ? "REV_BRAKE" : "BRAKE";
            }
            /* 倒车：油门取反（flowsim physics 用负油门触发倒车） */
            if (is_reverse) throttle = -throttle;

            /* 机动换挡刹停：gear_pending = 带速想换挡（掉头 Phase 2→3 D→R
             * 交界）。巡航 PID 是减速工况标定的，对 2.4m/s 只给 0.23 brake
             * （decel≈1.2m/s²，需 1.9s/2.3m），车还没停就冲过倒车段、最近点
             * 跳到 Phase 4 正向 → gear 翻回 D → 加速冲出路面（2026-08-03
             * 实测 y=8.5）。换挡必须物理刹停，直接给全刹——这就是掉头
             * 三把方向的"停"字。 */
            if (g.maneuver_mode && g.gear_pending) {
                throttle = 0.0;
                brake = 1.0;
                mode = "SHIFT_STOP";
                g.integral = 0;  /* 全刹时清积分，防换挡后残余积分反向推车 */
            }

            /* Anti-windup：error 从正翻负时（加速→减速切换），积分饱和是追尾主因。
             * 加速阶段积分可累积到 +500（I=50×+500=+25000），此时减速指令 P=800×(-8)=-6400，
             * 总量 +18600 → 油门全开撞上去。
             * 修复：error 翻负且 |error|>2 时直接清零正积分；正常饱和时慢速泄放。 */
            if (pid_error < -2.0 && g.integral > 0) {
                g.integral = 0;  /* 从加速切到减速，残余正积分是催命符，立刻清零 */
            } else {
                if (g.integral > 0 && throttle >= 1.0 && pid_error > 0)
                    g.integral -= pid_error * 0.05;
                if (g.integral > 0 && brake >= 1.0 && pid_error < 0)
                    g.integral += pid_error * 0.05;
            }
            /* 2026-08-05 掉头卡死修复（负积分对称清除）：Phase 1 刹停到 0 时
             * error<0 使积分转负，目标翻正（驻停/慢转 target>0）后负积分残留把车
             * 钉在原地 —— 掉头返程实测 spd=0 target=0.5 brk=0.13 卡死 30s timeout。
             * 机动模式下车近停且 target 为正时清负积分，让车能起步继续轨迹。 */
            if (g.maneuver_mode && g.integral < 0 &&
                pid_error > 0.2 && fabs(g.current_speed) < 1.0) {
                g.integral = 0;
            }
            g.prev_error = pid_error;

            /* ── LTV MPC 横向控制 ──
             * 机动模式（掉头/倒车）跳过：MPC 线性化假设小转角误差动力学，
             * 满舵弧 + 倒挡在其模型域外，直接走 Stanley + kappa 前馈。 */
            bool mpc_used = false;
            if (g.use_ltv_mpc && !g.maneuver_mode && g.has_planning && g.ref_path.size() > 1) {
                if (!g.ltv_mpc) {
                    g.ltv_mpc = ltv_mpc_create(&g.ltv_mpc_cfg);
                }
                if (g.ltv_mpc) {
                    ltv_mpc_update_config(g.ltv_mpc, &g.ltv_mpc_cfg);
                    double e_y = -lat_err_n;
                    double heading_error = g.ego_heading - ref_road_heading;
                    while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
                    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
                    double e_psi = -heading_error;
                    ltv_mpc_set_state(g.ltv_mpc, e_y, e_psi, g.prev_steer, g.current_speed);
                    double v_ref[LTV_MPC_MAX_HORIZON];
                    double kappa_ref[LTV_MPC_MAX_HORIZON];
                    int n_ref = (int)g.ref_path.size() < LTV_MPC_MAX_HORIZON ?
                                 (int)g.ref_path.size() : LTV_MPC_MAX_HORIZON;
                    for (int i = 0; i < n_ref; i++) {
                        v_ref[i] = g.target_speed;
                        kappa_ref[i] = g.ref_path[i].kappa;
                    }
                    ltv_mpc_set_reference(g.ltv_mpc, v_ref, kappa_ref, n_ref);
                    double mpc_steer_delta = 0.0;
                    int rc = ltv_mpc_solve(g.ltv_mpc, &mpc_steer_delta);
                    if (rc == LTV_MPC_OK) {
                        steer = g.prev_steer + mpc_steer_delta;
                        g.prev_steer = steer;
                        mpc_used = true;
                    }
                }
            }

            /* ── Stanley 横向控制（LTV MPC 未启用或求解失败时回退）── */
            if (!mpc_used && g.maneuver_mode) {
                /* ── 机动横向：ManeuverTracker.tick() 已在 acc_target 前算好 ──
                 * 几何跟踪（kappa 前馈 + 车体系 e_lat/dh 反馈，倒挡反号）
                 * 与 ±0.60 限幅都在 tracker 内部。这里只取结果跳过巡航 Stanley。 */
                steer = mv_res.steer;
                mpc_used = true;
            }
            if (!mpc_used) {
                steer = 0.0;
                {
                    /* 倒车时用绝对值做横向控制：速度符号翻转会导致
                     * v_lat_actual 和 cte_term 符号反转，steer 反向。 */
                    double abs_speed = fabs(g.current_speed);
                    double speed_eff = fmax(abs_speed, 3.0);
                    double v_lat_actual = abs_speed *
                        sin(g.ego_heading - ref_road_heading);
                    /* v_y_des = k_vy * e_lat - k_vy_damp * v_lat（参考系左法向，巡航模式） */
                    double v_y_des = g.k_vy * lat_err_n - g.k_vy_damp * v_lat_actual;
                    double psi_des = ref_road_heading;
                    {
                        double vy_ratio = v_y_des / speed_eff;
                        if (vy_ratio > 0.5) vy_ratio = 0.5;
                        if (vy_ratio < -0.5) vy_ratio = -0.5;
                        psi_des = ref_road_heading + asin(vy_ratio);
                    }
                    double delta_ff = atan(g.wheelbase * v_y_des /
                                            (speed_eff * speed_eff + 1e-6));
                    /* heading_term 跟踪 ψ_des（clamp 防大角度突变） */
                    double ref_h_eff = psi_des;
                    {
                        double dh = ref_h_eff - g.ego_heading;
                        while (dh >  M_PI) dh -= 2.0 * M_PI;
                        while (dh < -M_PI) dh += 2.0 * M_PI;
                        if (fabs(dh) > 0.5) ref_h_eff = g.ego_heading;
                    }
                    double cte_term     = atan2(g.lat_kp * lat_err_n, speed_eff);
                    double heading_term;
                    {
                        /* wrap：返程 heading 在 ±π 边界抖动时，ego_h=3.1 与
                         * ref_h_eff=-3.1 等价，裸差 6.2 会打满舵。 */
                        double dh_t = g.ego_heading - ref_h_eff;
                        while (dh_t >  M_PI) dh_t -= 2.0 * M_PI;
                        while (dh_t < -M_PI) dh_t += 2.0 * M_PI;
                        heading_term = g.lat_kd_heading * dh_t;
                    }
                    double yaw_damp_term = g.yaw_damping * g.ego_yaw_rate;
                    double kappa = ref_kappa;
                    double ff_weight = 1.0;
                    if (fabs(kappa) > 1e-9) {
                        double R = 1.0 / fabs(kappa);
                        if (R <= g.curve_ff_boost_radius_m) ff_weight = g.curve_ff_boost_factor;
                    }
                    double ff_term = g.wheelbase * kappa * ff_weight;

                    steer = cte_term - heading_term - yaw_damp_term + ff_term + delta_ff;
                    /* 机动模式放开限幅到满舵 0.60rad：巡航限幅 0.16rad 的转弯
                     * 半径 ≥16.7m，无法执行规划的掉头弧（0.45rad，R≈5.6m）。
                     * flowsim 物理层看到 |steer|>0.28 会同步 steer_override。 */
                    double steer_limit = g.maneuver_mode
                        ? 0.60 : steer_limit_for_speed(abs_speed, 1.4);
                    if (steer >  steer_limit) steer =  steer_limit;
                    if (steer < -steer_limit) steer = -steer_limit;
                    steer = STEER_FILTER_NEW * steer + (1.0 - STEER_FILTER_NEW) * g.prev_steer;
                    if (fabs(steer) < 0.005) steer = 0.0;
                    g.prev_steer = steer;
                }
            }

            /* §11.2 降级阶梯（MRM 目标已在前置块处理，此处仅标记模式） */
            {
                DegradeState* ds = degrade_global_state();
                if (ds->degrade_level >= DEGRADE_L2) {
                    /* 机动期必须保留 +MANEUVER 标签，否则 safety 认不出机动 →
                     * 近场 TTC 全刹 → 再设 L2 → MRM 自锁环（2026-08-03）。 */
                    mode = g.maneuver_mode ? "MRM+MANEUVER" : "MRM";
                }
                /* L1: 禁变道——planning 的 overtake_state 会被忽略，control 只巡航 */

                /* §n: Req/Reply — 每 ~5s 查询一次 safety 状态（非阻塞同步请求） */
                if (g.cycle % 100 == 1) {
                    Message reply;
                    memset(&reply, 0, sizeof(reply));
                    int rc = message_bus_request(bus(), "safety/status", "control_node",
                                                 nullptr, 0, &reply, 100);
                    if (rc == 0 && reply.data_size > 0) {
                        LOG_DEBUG("control", "safety/status: %.*s",
                                  (int)reply.data_size, (const char*)reply.data);
                    }
                }
            }

            /* ── Safety overrides ── */

            /* 目标车道中心 = 道路中心 + 横向偏移在世界系的投影。
             * lane_d 是 trajectory 前视点的 Frenet 横向 offset（参考线法向），
             * 映射到世界 y 需乘 cos(ref_road_heading)：前进(h=0) → +lane_d；
             * 掉头返程(h=π) → -lane_d。与第 313 行 road_c 的推导
             * (target_path_y - lane_d*cos(h)) 互逆，返程时符号自洽——否则
             * target_lane_center 落到对侧车道、y_from_target 虚高 >3m 误触
             * ROAD_GUARD，把在正确车道正常行驶的 ego 强行限速打转。 */
            double target_lane_center = road_c + g.lane_d * cos(ref_road_heading);

            /* 接近路沿增强拉回：偏离目标车道中心 >4.5 时限制 steer 幅度，
             * 避免大 steer 冲出路沿。旧实现用 |ego_y - road_c|，对 4 车道
             * 外车道（y=±5.25）永远 >4.5 → steer 被永久限幅。 */
            double y_from_target = fabs(g.ego_y - target_lane_center);
            if (y_from_target > 4.5 && !g.maneuver_mode) {
                const double near_edge_limit = 0.165;
                if (steer >  near_edge_limit) steer =  near_edge_limit;
                if (steer < -near_edge_limit) steer = -near_edge_limit;
                g.prev_steer = steer;
            }

            /* 超速保护已移除：planning 负责不超速（command_speed ≤ cfg_max_speed），
             * safety_control 层有 TTC 限幅。control 是纯轨迹跟随器，不做速度决策。
             * 原超速限幅用 cfg_cruise_speed 做阈值，但 control 不应有自己的巡航速度。 */

            /* 全域速度死锁恢复 */
            if (g.speed_zero_timer > SPEED_ZERO_RECOVER_S &&
                y_from_target <= ROAD_GUARD_THRESHOLD_M &&
                g.has_planning && g.target_speed > 1.0) {
                throttle = 0.15;
                brake    = 0.0;
                mode     = "SPEED_ZERO_RECOVERY";
                g.speed_zero_timer = 0.0;
                LOG_WARN("control", ">>> SPEED_ZERO RECOVERY: throttle bump at y=%.2f (ego@(%.1f,%.1f)) tgt=%.1f",
                         g.ego_y, g.ego_x, g.ego_y, g.target_speed);
            }

            /* ROAD_GUARD：车辆偏离目标车道中心过远时强制回正。
             * road_c 现在是道路中心 y（不含横向偏移），目标车道中心 = road_c + lane_d。
             * 旧实现检查 |ego_y - road_c|，对 4 车道外车道（y=±5.25）永远触发。
             * 机动期豁免：掉头本来就要横穿整条路，y_from_target 必然 >3m，
             * 不豁免则 ROAD_GUARD 抢走执行权（钳 steer=0.16 + 乱设油门），
             * 掉头永不完成（2026-08-03 demo6 实测车被拽出路面到 y=-11）。 */
            if (y_from_target > ROAD_GUARD_THRESHOLD_M && !g.maneuver_mode) {
                double steer_limit = steer_limit_for_speed(fabs(g.current_speed), 2.4);
                /* 用参考系投影误差 lat_err_n：正误差=左打对前进/返程都成立。
                 * 旧实现用世界系 lat_error，假设车头恒朝 +x，掉头返程时打反
                 * → 车持续北漂出路面越漂越远（2026-08-03 demo11：y 23→33）。 */
                steer = (lat_err_n > 0.0) ? steer_limit : -steer_limit;
                if (fabs(g.current_speed) < 2.5) {
                    throttle = 0.18;
                    brake = 0.0;
                    g.speed_zero_timer = 0.0;
                } else {
                    throttle = 0.0;
                    if (brake < 0.65) brake = 0.65;
                }
                g.prev_steer = steer;
                mode = "ROAD_GUARD";
            }

            /* 转向灯 / 双闪指令 */
            uint8_t turn_signal = 0;
            bool    hazard      = false;
            /* 变道打灯：LEFT_CHANGE/RIGHT_CHANGE → 左/右转向灯。
             * （原实现 turn_signal 恒 0，变道不打灯——2026-07-31 用户反馈。
             * 命令取自 planning/behavior 类型化消息的 command enum。）
             * 掉头机动（maneuver_mode）：第一把左转 → 左灯（用户规范
             * "左转向灯+方向盘接近打死第一把，然后倒车调整"），全程保持。 */
            if (g.maneuver_mode) {
                turn_signal = 1;  /* 左灯（掉头） */
            } else if (g.beh_command == BEH_LEFT_CHANGE) {
                turn_signal = 1;  /* 左灯 */
            } else if (g.beh_command == BEH_RIGHT_CHANGE) {
                turn_signal = 2;  /* 右灯 */
            }
            /* 紧急制动时开双闪（ROAD_GUARD / collision recovery） */
            if (strcmp(mode, "ROAD_GUARD") == 0 && brake > 0.6) {
                hazard = true;
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
            raw.cte      = (float)lat_error;
            raw.turn_signal = turn_signal;
            raw.hazard   = hazard;
            raw.gear     = g.gear;
            memset(raw.mode, 0, sizeof(raw.mode));
            /* 机动期（掉头/倒车）打 MANEUVER 标签：safety_control 据此放行
             * 满舵/豁免巡航级 TTC（Phase 1 直行减速段 steer 还小、挡位还是 D，
             * safety 无法从 steer/gear 推断机动 → 施工区被当前车全刹 →
             * v=0 掉头死锁）。mode 是 control→safety 的唯一带外信号通道。 */
            snprintf(raw.mode, sizeof(raw.mode) - 1, "%s%s",
                     mode, g.maneuver_mode ? "+MANEUVER" : "");

            uint8_t raw_buf[64];
            size_t  raw_len = sizeof(raw_buf);
            ControlRaw_serialize(&raw, raw_buf, &raw_len);
            /* 无条件发布：QoS depth+drop_oldest 兜底，确保最新指令必达。
             * 原反压跳过（topic_is_full 时丢弃本帧）配合 control/raw_cmd 的
             * depth=1 QoS，会在 dispatch 瞬时抖动时持续跳过 → safety 收不到
             * raw_cmd → 1s 超时 L3 → MRM 永久停车（2026-07-31 断流事故）。 */
            transport_publish(transport_, TOPIC_CONTROL_RAW_CMD,
                              raw_buf, (uint32_t)raw_len);

            /* Also publish text format for backward compat (monitor/logging) */
            char cmd_text[256];
            snprintf(cmd_text, sizeof(cmd_text),
                     "throttle=%.2f brake=%.2f steer=%.4f "
                     "speed=%.1f target=%.1f error=%.1f mode=%s "
                     "turn_signal=%d hazard=%d gear=%d",
                     throttle, brake, steer,
                     g.current_speed, acc_target, error, mode,
                     (int)turn_signal, (int)hazard, (int)g.gear);
            transport_publish(transport_, TOPIC_CONTROL_RAW_CMD_TEXT,
                              (const uint8_t*)cmd_text, (uint32_t)strlen(cmd_text) + 1);

            /* 发布 CTE（横向误差）供 LDW/监控/数据记录用 */
            {
                cJSON* cte_root = cJSON_CreateObject();
                cJSON_AddNumberToObject(cte_root, "cte", lat_error);
                cJSON_AddNumberToObject(cte_root, "speed", g.current_speed);
                cJSON_AddNumberToObject(cte_root, "seq", g.cycle);
                char* cte_s = cJSON_PrintUnformatted(cte_root);
                transport_publish(transport_, TOPIC_CONTROL_CTE,
                                  (const uint8_t*)cte_s, (uint32_t)strlen(cte_s) + 1);
                free(cte_s);
                cJSON_Delete(cte_root);
            }

            /* 发布 control/debug（全链路横向调试，每10帧一次≈2Hz） */
            if (g.cycle % 10 == 0) {
                cJSON* dbg = cJSON_CreateObject();
                cJSON_AddNumberToObject(dbg, "seq", g.cycle);
                cJSON_AddNumberToObject(dbg, "ego_x", g.ego_x);
                cJSON_AddNumberToObject(dbg, "ego_y", g.ego_y);
                cJSON_AddNumberToObject(dbg, "ego_heading", g.ego_heading);
                cJSON_AddNumberToObject(dbg, "speed", g.current_speed);
                cJSON_AddNumberToObject(dbg, "target_speed", acc_target);
                cJSON_AddNumberToObject(dbg, "lane_d", g.lane_d);
                cJSON_AddNumberToObject(dbg, "road_center_y", g.road_center_y);
                cJSON_AddNumberToObject(dbg, "target_y", effective_target_y);
                cJSON_AddNumberToObject(dbg, "lat_error", lat_error);
                cJSON_AddNumberToObject(dbg, "steer", steer);
                cJSON_AddNumberToObject(dbg, "throttle", throttle);
                cJSON_AddNumberToObject(dbg, "brake", brake);
                cJSON_AddStringToObject(dbg, "mode", mode);
                cJSON_AddBoolToObject(dbg, "hazard", hazard);
                cJSON_AddBoolToObject(dbg, "mpc_used", mpc_used);
                cJSON_AddNumberToObject(dbg, "ref_road_heading", ref_road_heading);
                cJSON_AddNumberToObject(dbg, "y_from_target", y_from_target);
                cJSON_AddNumberToObject(dbg, "has_planning", g.has_planning ? 1.0 : 0.0);
                double lookahead_dist = fmax(5.0, g.current_speed * g.lat_lookahead_gain);
                cJSON_AddNumberToObject(dbg, "lookahead_dist", lookahead_dist);
                char* dbg_s = cJSON_PrintUnformatted(dbg);
                transport_publish(transport_, TOPIC_CONTROL_DEBUG,
                                  (const uint8_t*)dbg_s, (uint32_t)strlen(dbg_s) + 1);
                free(dbg_s);
                cJSON_Delete(dbg);
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
                    transport_publish(transport_, TOPIC_CONTROL_LDW,
                                      (const uint8_t*)ldw_s, (uint32_t)strlen(ldw_s) + 1);
                    free(ldw_s);
                    cJSON_Delete(ldw_root);
                }
            }

            g.prev_error = error;

            if (g.cycle % 20 == 1) {
                uint64_t _lat_now = clock_now_us();
                uint64_t _plan_lat = (g.last_planning_us > 0) ? (_lat_now - g.last_planning_us) : 0;
                uint64_t _fusion_lat = (g.last_fusion_us > 0) ? (_lat_now - g.last_fusion_us) : 0;
                LOG_INFO("control", "#%d spd=%.1f→%.1f err=%.1f thr=%.2f brk=%.2f st=%.4f d=%.2f target_y=%.2f %s lat(plan=%lums fusion=%lums)",
                         g.cycle, g.current_speed, g.target_speed,
                         error, throttle, brake, steer, g.lane_d, effective_target_y, mode,
                         (unsigned long)(_plan_lat / 1000), (unsigned long)(_fusion_lat / 1000));
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
};

/* ── TaskBase 包装器（宏生成） — 必须在 control_init 前展开 ─────── */
EXPORT_COROUTINE_TASK(ControlTask, control)

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { TOPIC_FUSION_LOCALIZATION, TOPIC_PLANNING_TRAJECTORY, TOPIC_PLANNING_BEHAVIOR, nullptr };
static const char* s_outputs[] = { TOPIC_CONTROL_RAW_CMD, nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾 */

static int control_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    /* 清零并重新初始化 */
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;

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
    g.target_path_y = 0.0;
    g.target_path_heading = 0.0;
    g.target_path_kappa = 0.0;
    g.target_road_center_y = 0.0;
    g.driving_mode[0] = '\0';

    g.has_fusion = 0;
    g.has_planning = 0;
    g.last_fusion_us = 0;
    g.last_planning_us = 0;

    g.stuck_timer = 0.0;
    g.speed_zero_timer = 0.0;

    g.cycle = 0;

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
    g.yaw_damping     = 0.28;  /* yaw_rate → steer 阻尼（自标定最优值）*/
    g.lat_lookahead_gain = 0.8;  /* 前视距离 = max(5m, speed*0.8s)，Apollo 标准 */
    g.k_v_lat          = 0.22;   /* 横向速度阻尼增益（自标定最优值）*/
    g.k_vy             = 0.35;   /* v_y_des 位置增益（止血保守值，原 0.7） */
    g.k_vy_damp        = 0.6;    /* v_y_des 速度阻尼增益 */
    /* A-2 修复：先解析 JSON 配置（pipeline_car.json 等通过 params_json 传入），
     * 把 JSON 中的值刷入 g.* 字段；随后 param_register_* 用这些（可能被 JSON
     * 覆盖过的）值作为代码默认值注册。若 bootstrap 已把同名参数预加载进
     * registry，param_register 不会覆盖其 current_value（见 param_registry.c A-1）。
     * 不使用 param_set_float 回写 JSON 值——那会在参数尚未注册（无 min/max 元信息）
     * 时引入脆弱的范围校验失败。 */
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
            j = cJSON_GetObjectItemCaseSensitive(p, "lat_lookahead_gain");
            if (cJSON_IsNumber(j)) g.lat_lookahead_gain = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "k_v_lat");
            if (cJSON_IsNumber(j)) g.k_v_lat = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "k_vy");
            if (cJSON_IsNumber(j)) g.k_vy = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "k_vy_damp");
            if (cJSON_IsNumber(j)) g.k_vy_damp = j->valuedouble;
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
            j = cJSON_GetObjectItemCaseSensitive(p, "ltv_mpc_enable");
            if (cJSON_IsNumber(j)) g.use_ltv_mpc = (j->valuedouble > 0.5) ? 1 : 0;
            j = cJSON_GetObjectItemCaseSensitive(p, "ltv_q_y");
            if (cJSON_IsNumber(j)) g.ltv_mpc_cfg.q_y = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "ltv_q_psi");
            if (cJSON_IsNumber(j)) g.ltv_mpc_cfg.q_psi = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "ltv_r_ddelta");
            if (cJSON_IsNumber(j)) g.ltv_mpc_cfg.r_ddelta = j->valuedouble;
            
            cJSON_Delete(p);
            g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
        }
    }

    /* 注册参数到 param_registry (类型安全，可运行时热重载)。
     * 注意：此时 g.* 已被 JSON 覆盖（若 JSON 提供了对应字段），故注册的代码默认值
     * 就是 JSON 值；若 registry 中已存在同名参数（bootstrap 预加载），register
     * 不会覆盖其 current_value，仅刷新 min/max/desc。 */
    param_register_float("control.pid_kp", g.cfg_kp, 0.0, 5000.0, "PID proportional gain");
    param_register_float("control.pid_ki", g.cfg_ki, 0.0, 1000.0, "PID integral gain");
    param_register_float("control.pid_kd", g.cfg_kd, 0.0, 2000.0, "PID derivative gain");
    param_register_float("control.cruise_speed", g.cfg_cruise_speed, 1.0, 50.0, "Target cruise speed m/s");
    param_register_float("control.lat_kp", g.lat_kp, 0.0, 2.0, "Lateral P gain");
    param_register_float("control.lat_kd_heading", g.lat_kd_heading, 0.0, 5.0, "Heading damping gain");
    param_register_float("control.yaw_damping", g.yaw_damping, 0.0, 2.0, "Yaw rate damping gain (suppress limit-cycle oscillation)");
    param_register_float("control.lat_lookahead_gain", g.lat_lookahead_gain, 0.1, 3.0, "Lookahead time gain (s): lookahead=max(5m, speed*gain), Apollo-style target trajectory");
    param_register_float("control.k_v_lat", g.k_v_lat, 0.0, 2.0, "LQR-style lateral velocity damping gain (anti-overshoot, replaces yaw_damping patch)");
    param_register_float("control.k_vy", g.k_vy, 0.0, 2.0, "v_y_des position gain: v_y_des = k_vy*lat_error - k_vy_damp*v_lat");
    param_register_float("control.k_vy_damp", g.k_vy_damp, 0.0, 2.0, "v_y_des velocity damping gain (D term of lateral PD)");
    /* ManeuverTracker 通用机动跟随器调参（默认值 = Python 原型字面量） */
    param_register_float("control.mv_heading_gate_rad",  g.mv_params.heading_gate_rad,  0.0, 3.14, "Maneuver D/R boundary heading gate (rad)");
    param_register_float("control.mv_diverge_guard_rad", g.mv_params.diverge_guard_rad, 0.0, 3.14, "Maneuver exec/car heading diverge guard (rad)");
    param_register_float("control.mv_lookahead_m",       g.mv_params.lookahead_m,        0.5, 8.0,  "Maneuver lateral lookahead arc (m)");
    param_register_float("control.mv_speed_scan_m",      g.mv_params.speed_scan_m,       1.0, 20.0, "Maneuver target-speed min|v| scan window (m)");
    param_register_float("control.mv_speed_floor_mps",   g.mv_params.speed_floor_mps,    0.5, 5.0,  "Maneuver target-speed floor (m/s)");
    param_register_float("control.mv_gear_v_threshold",  g.mv_params.gear_v_threshold,   0.1, 1.0,  "Maneuver gear-intent |v| threshold");
    param_register_float("control.mv_gear_pending_speed",g.mv_params.gear_pending_speed, 0.1, 3.0,  "Maneuver gear-change brake speed threshold");
    param_register_float("control.mv_lat_gain_dh",       g.mv_params.lat_gain_dh,        0.0, 3.0,  "Maneuver heading-error feedback gain");
    param_register_float("control.mv_lat_gain_elat",     g.mv_params.lat_gain_elat,      0.0, 2.0,  "Maneuver body-frame lateral-error gain");
    param_register_float("control.mv_max_steer",         g.mv_params.max_steer,          0.1, 0.8,  "Maneuver max steer (rad)");

    /* 运行时从 param_registry 读取 (支持 flowctl param set 热重载)。
     * 全新初始化时此值等于上方注册的默认值（即 JSON 值或代码默认值）；
     * 若 registry 已被 bootstrap 预加载，此处取到的是预加载值。 */
    g.kp = param_get_float("control.pid_kp");
    g.ki = param_get_float("control.pid_ki");
    g.kd = param_get_float("control.pid_kd");
    g.cfg_cruise_speed = param_get_float("control.cruise_speed");
    g.lat_kp           = param_get_float("control.lat_kp");
    g.lat_kd_heading   = param_get_float("control.lat_kd_heading");
    g.yaw_damping      = param_get_float("control.yaw_damping");
    g.lat_lookahead_gain = param_get_float("control.lat_lookahead_gain");
    g.k_v_lat          = param_get_float("control.k_v_lat");
    g.k_vy             = param_get_float("control.k_vy");
    g.k_vy_damp        = param_get_float("control.k_vy_damp");

    /* 初始化反射式状态机 */
    statem_init(&g.sm, g_ctl_transitions, SM_STATE_INITIALIZED, "control");
    statem_send_event(&g.sm, SM_EVENT_START, nullptr);  /* INITIALIZED → RUNNING */

    /* 订阅：只订阅 fusion/localization 和 planning/trajectory。
     * road/geometry 和 road/ref_path 不再独立于 planning 订阅。 */

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION, on_fusion, nullptr);
    transport_subscribe(transport, TOPIC_VEHICLE_STATE, on_vehicle_state, nullptr);
    transport_subscribe(transport, TOPIC_PLANNING_TRAJECTORY, on_trajectory, nullptr);
    transport_subscribe(transport, TOPIC_PLANNING_BEHAVIOR, on_planning_behavior, nullptr);  /* 转向灯意图 */
    transport_advertise(transport, TOPIC_CONTROL_RAW_CMD, CONTROLRAW_TYPE_ID);
    transport_advertise(transport, TOPIC_CONTROL_DEBUG, 0u);  /* JSON text */
    transport_advertise(transport, TOPIC_CONTROL_CTE, 0u);    /* JSON text */
    transport_advertise(transport, TOPIC_CONTROL_LDW, 0u);    /* JSON text */

    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PLANNING_TRAJECTORY, 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PLANNING_BEHAVIOR, 0u,
                        CAP_SUBSCRIBER, 0);  /* 转向灯意图 */
    discovery_advertise(discovery, TOPIC_CONTROL_RAW_CMD, CONTROLRAW_TYPE_ID,
                        CAP_PUBLISHER, 100.0);
    discovery_advertise(discovery, TOPIC_CONTROL_DEBUG, 0u, CAP_PUBLISHER, 2.0);

    /* 创建 TaskBase 包装器（托管模式） */
    TaskConfig tcfg = {};
    snprintf(tcfg.name, sizeof(tcfg.name), "control");
    tcfg.priority = TASK_PRIORITY_NORMAL;
    g.task_wrapper = control_create(&tcfg, bus);
    if (!g.task_wrapper) {
        LOG_ERROR("control", "control_create failed");
        return -1;
    }
    g.task_wrapper->impl->set_params(transport);
    s_plugin.taskbase = control_get_base(g.task_wrapper);

    /* LTV MPC 初始化 */
    g.ltv_mpc_cfg = ltv_mpc_default_config();
    g.ltv_mpc_cfg.wheelbase = g.wheelbase;
    g.ltv_mpc_cfg.horizon = 20;
    g.ltv_mpc_cfg.dt = 0.05;
    g.use_ltv_mpc = 0;  /* 默认禁用，通过 JSON 参数启用 */

    LOG_INFO("control", "initialized (FlowCoro, PID: kp=%.0f ki=%.0f kd=%.0f)",
             g.kp, g.ki, g.kd);
    return 0;
}

static int control_start(void) {
    if (!g.task_wrapper) return -1;
    /* 托管模式：注册到调度器 + 派生工作线程 + 设置 choreo trigger */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("control", "node_start_managed failed: %d", rc);
    }
    node_announce_self(g.transport, &s_plugin);
    LOG_INFO("control", "started (managed mode)");
    return 0;
}

static void control_stop(void) {
    if (g.task_wrapper) {
        control_stop(&g.task_wrapper->base);
    }
}

static void control_cleanup(void) {
    if (g.task_wrapper) {
        control_destroy(g.task_wrapper);
        g.task_wrapper = nullptr;
    }
    s_plugin.taskbase = nullptr;
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
    nullptr,  /* taskbase: 在 init() 中通过 control_create 设置 */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }