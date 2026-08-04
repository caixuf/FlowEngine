/**
 * planning_node.cpp — Frenet 轨迹规划节点插件 (FlowCoro 协程版)
 *
 * 从 planning_node.c 迁移而来，采用 CoroutineTask 协程框架：
 *   - co_await sleep_us(50000) 替代 usleep 20Hz 轮询（可被 stop 取消）
 *   - 保留 on_fusion / on_vehicle_state / on_road_geometry 持久回调
 *   - 驾驶模式状态机 + 路线变道 + Frenet 规划逻辑原样搬入 run()
 *
 * 订阅 fusion/localization → 发布 planning/trajectory
 *
 * NodePlugin 接口，编译为 libplanning_node.so。
 *
 * 采用 CoroutineTask（线程池 resume）：节点做重计算（Frenet 轨迹规划），同步 resume 会阻塞
 * 消息总线分发线程导致 drops，故改用线程池 resume。
 * flowcoro 核心库为 header-only（INTERFACE），子项目已 include 其头文件目录，
 * 故只需 FLOWCORO_INTEGRATION 定义 + -fcoroutines，无需额外链接 flowcoro 库。
 */
#include "node_plugin.h"
#include "topic_registry.h"
#include "state_machine.h"
#include "scenario_loader.h"
#include "road_geometry.h"
#include "traffic_light.h"
#include "adas_msgs_gen.h"   /* ObstacleList_deserialize */
#include "piecewise_jerk_qp.h"

#include "coroutine_task.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"
#include "clock_service.h"
#include "degrade_ladder.h"
#include <cjson/cJSON.h>

#ifdef HAVE_FRENET
#include "frenet_bridge.h"
#endif

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

namespace {

/* ── 节点本地状态 ───────────────────────────────────────────── */

struct PlanningContext {
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler*        scheduler{nullptr};

    /* 反射式状态机：跟踪生命周期 */
    ReflectiveStateMachine sm{};

    /* 驾驶模式状态机（NA/ACC/CP/NP/NOA）：系统级功能仲裁, 由感知/定位/
     * 路况条件驱动升降级, 是 NOA 相对 LCC 的核心能力入口。 */
    ReflectiveStateMachine mode_sm{};
    uint64_t mode_last_check_us{0};
    uint64_t last_fusion_us{0};      /* fusion 消息到达的单调时间戳（模式降级用） */
    uint64_t last_plan_us{0};        /* 上次规划调度的单调时间戳（20Hz 限速用） */
    int      highway_ready{0};       /* ego_v 持续高于阈值一段时间 -> 视为高速工况 */
    double   highway_speed_timer{0.0};

    /* 导航状态（由 navigation_node 发布 navigation/path 驱动） */
    int               route_count{0};       /* 导航总步骤数（来自 route_status） */
    int               route_next_idx{0};    /* 下一条待触发步骤索引（来自 route_status/route_step） */
    volatile int      has_navigation{0};
    uint64_t          last_nav_us{0};
    /* 当前路线要求的目标车道索引。
     * 语义：-1=无目标（保持当前车道），0..N-1=目标车道索引（0=最左, N-1=最右）。
     * 旧约定（{-1=左, 0=无, +1=右}）已废弃，但 scenario JSON 中的旧值仍可通过
     * route_step_target_lane_to_idx() 兼容映射。 */
    int               route_target_lane{-1};
    double            route_target_speed{-1.0};/* 当前路线步骤要求的目标速度（-1 = 未设置/无改变，0 = 停车） */
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
    int    merge_hold_lane{-1};  /* N 车道模型：-1=无, 0..N-1=暂存的目标车道 idx */
    volatile int has_scene_frame{0};

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    FrenetHandle* frenet{nullptr};
#else
    void*         frenet{nullptr};  /* unused stub */
#endif
    int           plan_count{0};
    double        target_speed{0.0};

    /* 从 fusion/localization 或 vehicle/state 解析的最新状态。
     * vehicle/state（flowsim 真值）优先于 fusion/localization（EKF 估计），
     * 当 vehicle/state 近期到达时 on_fusion 不覆盖，避免 EKF 发散污染 ego 状态。 */
    double ego_x{0}, ego_y{0}, ego_v{0}, ego_heading{0};
    /* 车参单一事实源 = 场景 ego 块（flowsim 随 vehicle/state 广播，on_vehicle_state 同步）。
     * 旧硬编码 2.8 与 physics 2.7 差 3.7% 导致掉头轨迹漂移（2026-08 修复）。 */
    double wheelbase{2.7};
    int    ego_lane_id{0};   /* 当前车道 ID（负=前进车道，正=对向车道，取自 vehicle/state）*/
    int    ref_path_update_pending{0}; /* 1=亟需在下一主循环 tick 更新参考路径（车道方向变化）*/
    volatile int has_fusion{0};
    uint64_t last_vstate_us{0};

    /* 从 vehicle/state 解析的障碍物位置（世界坐标） */
    /* 容量与 perception 发布的 ObstacleList.obstacles[128] 对齐（adas_msgs_gen.h
     * 生成的 ObstacleList.h 中 obstacles[128]）。直接用编译期常量避免运行时切片越界。 */
    static constexpr int kMaxObs = 128;
    double obs_x[kMaxObs]{}, obs_y[kMaxObs]{}, obs_vx[kMaxObs]{}, obs_vy[kMaxObs]{};  /* Phase 3: +vy */
    int8_t obs_lane_id[kMaxObs]{};  /* 感知计算的车道归属（从 perception/obstacles 提取） */
    uint8_t obs_type[kMaxObs]{};    /* 障碍物类型：OBJ_TYPE_VEHICLE / PEDESTRIAN / CYCLIST */
    float   obs_confidence[kMaxObs]{}; /* 置信度 */

    /* 真值障碍物缓存（vehicle/state 的 oidN/oxN/oyN/ovN，flowsim 上帝视角）。
     * 仅用于 TTC 安全兜底——感知链（perception/obstacles）漏检/停更时，
     * 本车道前车物理 gap 仍有依据可限速。主决策（变道/跟车）仍走感知链。 */
    static constexpr int kMaxTruthObs = 64;
    double truth_obs_x[kMaxTruthObs]{}, truth_obs_y[kMaxTruthObs]{};
    double truth_obs_vx[kMaxTruthObs]{};
    int    truth_obs_count{0};
    volatile int has_vstate{0};

    /* 配置参数 */
    double cfg_target_speed{15.0};
    double cfg_max_speed{20.0};
    double cfg_max_accel{4.0};
    double cfg_ref_path_length{5000.0};
    double        ref_path_start_x{0.0};
    double        map_ref_x[128]{};
    double        map_ref_y[128]{};
    double        map_ref_s[128]{};
    int           map_ref_count{0};
    uint64_t      last_map_ref_us{0};
    double cfg_highway_speed_mps{13.0}; /* CP->NP 升级所需的持续速度阈值 (m/s) */

    /* 道路几何（可选弯道，来自场景文件 "road"；全零 = 直道，行为不变） */
    double curve_start_x{0};
    double curve_length_m{0};
    double curve_offset_m{0};
    int    lane_count{2};       /* 从 road/geometry 订阅获取 */
    double lane_width{3.5};     /* 从 road/geometry 订阅获取 */

    /* 变道圆弧曲率（固定方向盘 = 固定 kappa，2026-08）：
     * 变道轨迹生成时计算，回填后填给轨迹点，control kappa 前馈用 */
    double lane_change_kappa{0.0};
    int    lane_change_kappa_active{0};

    /* 红绿灯状态缓存（从 road/traffic_lights topic 获取，flowsim_node 发布）。
     * 解析统一走 include/traffic_light.h 的 traffic_lights_parse()（TL_CACHE_MAX
     * 即该头定义的 16），此处只存消费视图。
     * 缓存前方最近的红/黄灯，用于在 Frenet 障碍物数组中注入虚拟停止线墙。
     * 红灯/黄灯时注入一面跨车道宽度的静止"墙"，绿灯时不注入——
     * safety_control 现有的 TTC/brake 逻辑直接对虚拟墙生效，无需改安全层。 */
    double tl_x[TL_CACHE_MAX];         /* 停止线 x（世界坐标） */
    double tl_y_lane[TL_CACHE_MAX];    /* 灯所在车道 y */
    int    tl_state[TL_CACHE_MAX];     /* 0=green 1=yellow 2=red */
    int    tl_count{0};
    volatile int has_traffic_lights{0};

    int tid{0};  /* scheduler task id */

    /* §9 轨迹拼接：上帧轨迹缓存 */
    Trajectory prev_traj;           /* 上帧发布的轨迹 */
    uint64_t prev_traj_stamp_us;    /* 上帧轨迹发布时间戳 */
    int stitch_skip_count;          /* 连续性跳过次数统计（>5%→FAIL） */
    int stitch_total_count;

    /* ── 被动超车变道状态（P1 替代 control 中被删除的 lc_state）。
     * 规划层自己决策变道时机，通过 trajectory 输出偏移，控制层只跟轨迹。
     * 状态机已拆到 behavior_planner_node，此处只消费 Behavior 指令。 */
    int    overtake_state{0};       /* 0=巡航, 1=左变道, 2=右变道（缓存 behavior.command） */
    double target_lane_offset{0.0}; /* 当前目标车道偏移 (m, 相对于道路中心) */
    int    committed_lane_idx{0};  /* 当前所在车道索引（从 behavior_planner 的 Behavior.target_lane_idx 推导） */
    Behavior current_behavior{};    /* 最新 behavior 指令 */
    volatile int has_behavior{0};

    /* UTurn 轨迹缓存：避免每帧重新生成 64 点轨迹（含 LOG 轰炸 + 前向积分）。
     * 首次进入 U-turn 状态时生成一次，后续帧复用。当 ego 偏离首点 > 2m 时重新生成。 */
    TrajectoryPoint uturn_cache[64];
    int    uturn_cache_n{0};
    double uturn_cache_ego_x{0.0}, uturn_cache_ego_y{0.0};  /* 生成时的 ego 位置 */

    /* TaskBase 包装器（由 EXPORT_COROUTINE_TASK 宏创建） */
    struct planning_Wrapper* task_wrapper{nullptr};
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
 *   NP  -> NOA : navigation/path 已上线（等效 HD 地图/路线规划可用）
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
                /* 导航链路在线时放行 NP（路线步骤本身定义驾驶策略，
                 * 不应被 highway_ready 卡住——城市段也有路线指令如减速/
                 * 过红绿灯/匝道汇入）。导航未上线时仍保持高速工况守卫。 */
                if (g.has_navigation) return true;
                return g.highway_ready != 0;
            case SM_MODE_NOA: return g.has_navigation != 0;
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

static bool has_fresh_map_ref(void) {
    return g.map_ref_count >= 2 && g.last_map_ref_us != 0 &&
           clock_now_us() - g.last_map_ref_us < 500000ULL;
}

static double lane_center_offset(int lane_idx, int n_lanes, double lane_w) {
    return lane_center_y(lane_idx, n_lanes, lane_w, 0.0, 0.0);
}

static bool project_to_reference_path(double x, double y,
                                      double& out_s, double& out_d,
                                      double* out_ref_x = nullptr,
                                      double* out_ref_y = nullptr,
                                      double* out_heading = nullptr) {
    if (!has_fresh_map_ref()) return false;

    double best_dist2 = 1e300;
    bool found = false;
    for (int i = 0; i + 1 < g.map_ref_count; ++i) {
        double x0 = g.map_ref_x[i];
        double y0 = g.map_ref_y[i];
        double x1 = g.map_ref_x[i + 1];
        double y1 = g.map_ref_y[i + 1];
        double vx = x1 - x0;
        double vy = y1 - y0;
        double seg_len2 = vx * vx + vy * vy;
        if (seg_len2 <= 1e-9) continue;

        double t = ((x - x0) * vx + (y - y0) * vy) / seg_len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        double px = x0 + t * vx;
        double py = y0 + t * vy;
        double dx = x - px;
        double dy = y - py;
        double dist2 = dx * dx + dy * dy;
        if (dist2 >= best_dist2) continue;

        double theta = atan2(vy, vx);
        double seg_len = sqrt(seg_len2);
        out_s = g.map_ref_s[i] + t * seg_len;
        out_d = -dx * sin(theta) + dy * cos(theta);
        if (out_ref_x) *out_ref_x = px;
        if (out_ref_y) *out_ref_y = py;
        if (out_heading) *out_heading = theta;
        best_dist2 = dist2;
        found = true;
    }
    return found;
}

static void update_reference_path(double start_x, bool opposite = false) {
#ifdef HAVE_FRENET
    if (has_fresh_map_ref()) {
        frenet_set_reference_path(g.frenet, g.map_ref_x, g.map_ref_y, g.map_ref_count);
        g.ref_path_start_x = g.ego_x;
        return;
    }
    double wx[101], wy[101];
    const int ref_n = 101;
    double step = g.cfg_ref_path_length / (double)(ref_n - 1);
    /* 对向车道（lane_id > 0）：参考路径从右向左生成，heading = π，
     * 匹配 U-turn 后的反向行驶方向 */
    for (int i = 0; i < ref_n; i++) {
        wx[i] = opposite ? start_x - (double)i * step : start_x + (double)i * step;
        wy[i] = road_center_y(wx[i], g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    }
    frenet_set_reference_path(g.frenet, wx, wy, ref_n);
    g.ref_path_start_x = opposite ? start_x - g.cfg_ref_path_length : start_x;
#else
    (void)start_x;
    (void)opposite;
#endif
}

/**
 * Frenet→Cartesian 转换：将 (s, d) 沿参考路径映射回全局坐标。
 *
 * 参考路径由 update_reference_path() 构建：等间距沿 x 轴采样，
 * wy[i] = road_center_y(wx[i])。对每个 Frenet 输出点 (s, d)：
 *   1. 找到 s 对应的参考线段，线性插值得 (ref_x, ref_y)
 *   2. 由相邻参考点差分算 heading (theta)
 *   3. x = ref_x - d * sin(theta), y = ref_y + d * cos(theta)
 *   4. 由相邻点 heading 差分算 kappa
 *
 * @return true 成功，false 无效 s（超出参考路径范围或退化）
 */
static bool frenet_to_cartesian(double s, double d,
                                double& out_x, double& out_y,
                                double& out_heading, double& out_kappa) {
    if (has_fresh_map_ref()) {
        double total_s = g.map_ref_s[g.map_ref_count - 1];
        if (total_s <= 1e-6) return false;
        if (s < 0.0) s = 0.0;
        if (s > total_s) s = total_s;
        int idx = 0;
        while (idx + 1 < g.map_ref_count && g.map_ref_s[idx + 1] < s) idx++;
        if (idx >= g.map_ref_count - 1) idx = g.map_ref_count - 2;
        double s0 = g.map_ref_s[idx];
        double s1 = g.map_ref_s[idx + 1];
        double frac = (s1 > s0) ? ((s - s0) / (s1 - s0)) : 0.0;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;
        double rx0 = g.map_ref_x[idx];
        double ry0 = g.map_ref_y[idx];
        double rx1 = g.map_ref_x[idx + 1];
        double ry1 = g.map_ref_y[idx + 1];
        double ref_x = rx0 + frac * (rx1 - rx0);
        double ref_y = ry0 + frac * (ry1 - ry0);
        double dx = rx1 - rx0;
        double dy = ry1 - ry0;
        double theta = atan2(dy, dx);
        out_x = ref_x - d * sin(theta);
        out_y = ref_y + d * cos(theta);
        out_heading = theta;
        /* kappa = 参考线切线转角差 / 段长（map_ref 是 ~5m 采样的 route centerline，
         * 弯道处切线连续变化）。旧实现恒 0 → control 的 kappa 前馈 ff_term 丢失，
         * 弯道全靠 heading 反馈追，横向误差大/抖动。 */
        out_kappa = 0.0;
        if (idx > 0) {
            double h_prev = atan2(ry0 - g.map_ref_y[idx - 1],
                                  rx0 - g.map_ref_x[idx - 1]);
            double h_cur = atan2(ry1 - ry0, rx1 - rx0);
            double dh = h_cur - h_prev;
            while (dh >  M_PI) dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            double ds = (s1 - s0) > 1e-6 ? (s1 - s0) : 1e-6;
            out_kappa = dh / ds;
        }
        return true;
    }
    const int ref_n = 101;
    double step = g.cfg_ref_path_length / (double)(ref_n - 1);
    double ref_start = g.ref_path_start_x;

    /* s 超出 [0, cfg_ref_path_length] → 夹紧到端点 */
    if (s < 0.0) s = 0.0;
    if (s > g.cfg_ref_path_length) s = g.cfg_ref_path_length;

    int idx = (int)(s / step);
    if (idx < 0) idx = 0;
    if (idx >= ref_n - 1) idx = ref_n - 2;

    double frac = (s - (double)idx * step) / step;
    if (frac > 1.0) frac = 1.0;
    if (frac < 0.0) frac = 0.0;

    /* 参考点坐标 */
    double rx0 = ref_start + (double)idx * step;
    double rx1 = ref_start + (double)(idx + 1) * step;
    double ry0 = road_center_y(rx0, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    double ry1 = road_center_y(rx1, g.curve_start_x, g.curve_length_m, g.curve_offset_m);

    double ref_x = rx0 + frac * (rx1 - rx0);
    double ref_y = ry0 + frac * (ry1 - ry0);

    /* heading = 参考线切线方向 */
    double dx = rx1 - rx0;
    double dy = ry1 - ry0;
    double theta = atan2(dy, dx);

    /* kappa = 参考线曲率（road_center_curvature 已实现了 smoothstep 二阶导） */
    double kappa = 0.0;
    if (frac < 0.5) {
        kappa = road_center_curvature(rx0, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    } else {
        kappa = road_center_curvature(rx1, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
    }

    /* Frenet→Cartesian：沿法向偏移 d */
    out_x = ref_x - d * sin(theta);
    out_y = ref_y + d * cos(theta);
    out_heading = theta;
    out_kappa = kappa;
    return true;
}

/* 计算车道中心的 y 坐标 */
static double lane_center_y(int lane_idx, int n_lanes, double lane_w) {
    double road_c = 0.0;  /* 道路中心 y=0 */
    double side_offset = 0.0;
    return road_c + side_offset - (lane_idx - (n_lanes - 1) / 2.0) * lane_w;
}

/* ═══════════════════════════════════════════════════════════════
 * 实车级三把方向掉头轨迹生成器（UTurnPlanner）
 *
 * 中国掉头操作规范（驾考标准）：
 *   1. 先向左打死方向盘，尝试一把完成掉头
 *   2. 若一把过不去（车头逼近对向路沿），向右打方向盘倒车调整
 *   3. 再向左打前进，直至车头能进入对向车道
 *
 * 路端空间不足时的倒车腾挪（人类司机正常操作）：
 *   Phase 0: 若前向空间 < 最小掉头半径（~12m），先直线倒车腾出空间
 *   → 再进入 Phase 1-5 正常掉头流程
 *
 * 自适应相位切换（基于车辆状态，非固定时间）：
 *   Phase 0 (可选): 直线倒车腾挪 → 腾出 ≥12m 前向空间
 *     切换条件：前向空间 ≥12m OR 倒车 ≥5s（安全上限）
 *   Phase 1: 直线重刹 → 掉头安全低速 (v ≤ 3.5 m/s)
 *   Phase 2: 左打死前进 → 冲向对向路沿
 *     切换条件：heading 转过 ≥130° 且 y 已越过路中线 (y > 0.5m)
 *     OR 安全上限：heading 转过 ≥155°（防冲出路面）
 *   Phase 3: 右打死倒车 → 车尾摆回车道区域
 *     切换条件：y 回到对向车道半幅内 (|y - target_lane_y| < 1.5m)
 *     OR 安全上限：倒车 ≥3.0s（防无限倒车）
 *   Phase 4: 左打前进对齐 → 车头对准对向车道
 *     切换条件：heading 接近 ±π (|heading_normalized| < 0.25 rad)
 *     OR 安全上限：前进 ≥2.5s
 *   Phase 5: 巡航 → 剩余点以目标速度直行填充
 *
 * 轨迹点 v 为负表示倒车（control 据此设 GEAR_REVERSE）。
 * 自行车模型与 flowsim physics.cpp 完全一致：
 *   yaw_rate = (speed / wheelbase) * tan(steer)
 *   heading += yaw_rate * dt
 *   x += speed * cos(heading) * dt
 *   y += speed * sin(heading) * dt
 * ═══════════════════════════════════════════════════════════════ */
static int generate_uturn_trajectory(TrajectoryPoint* points, int max_points,
                                     double ego_x, double ego_y, double ego_heading,
                                     double ego_speed, double wheelbase,
                                     double forward_space_m) {
    const double dt = 0.05;               /* 20Hz 时间步长 */
    const double max_steer = 0.60;        /* 满舵角 (rad)，≈34°，匹配 physics steer_override */
    const double uturn_steer = 0.60;      /* 第一把前进转向角：满舵（用户规范"方向盘接近打死第一把"）。
                                           * 旧值 0.45（wheelbase=2.8 模型下 Python 扫参最优）转弯慢，
                                           * 64 点轨迹截断在 Phase 3 前；满舵 yaw_rate 提高 38%，
                                           * Phase 2 缩短到 ~2.6s，轨迹可覆盖全部相位。 */
    const double uturn_speed = 3.5;       /* 掉头目标低速 (m/s) */
    const double reverse_speed = -3.0;    /* 倒车速度 (m/s) */
    const double road_half_w = g.lane_width * g.lane_count * 0.5;  /* 半路宽 */
    const double half_wb = wheelbase * 0.5;  /* 半轴距：车辆中心偏移后轴距离 */
    /* 对向车道中心 y：前进车道在 y<0 侧（lane_id<0），对向车道在 y>0 侧 */
    const double opp_lane_center_y = g.lane_width * 0.5;  /* 对向最左车道中心 */
    const double min_uturn_space = 12.0;  /* 最小掉头前向空间 (m)：2×转弯半径 + 余量 */

    /* 安全上限（防止无限循环） */
    const double phase0_max_dur = 5.0;   /* 腾挪倒车最多 5s */
    const double phase2_max_dur = 4.0;   /* 左打死前进最多 4s */
    const double phase3_max_dur = 3.0;   /* 倒车最多 3s */
    const double phase4_max_dur = 2.5;   /* 对齐最多 2.5s */

    int n = 0;
    double x = ego_x, y = ego_y, h = ego_heading;
    double v = ego_speed;
    double steer = 0.0;
    uint32_t t_us = 0;

    /* 归一化 heading 到 [-π, π] */
    auto norm_h = [](double hd) -> double {
        while (hd > M_PI)  hd -= 2.0 * M_PI;
        while (hd < -M_PI) hd += 2.0 * M_PI;
        return hd;
    };

    double h_start = norm_h(h);  /* 记录起始 heading */

    /* 掉头目标车道方向（2026-08 返程掉头修复）：
     * 前进端掉头（进入时 heading≈0，前进车道）→ 掉到对向车道，Phase 4 对齐 ±π
     * 返程掉头（进入时 heading≈±π，对向车道）→ 掉回前进车道，Phase 4 对齐 0
     * 旧实现 Phase 4 硬编码对齐 |h|≈π —— 返程掉头永远对齐不到 → 超时失败
     * → 车滞留对向车道 → 正常规划器下发正向轨迹 → 逆行（2026-08-03 实测）。 */
    const double uturn_target_h =
        (std::fabs(h_start) > M_PI * 0.5) ? 0.0 : M_PI;
    /* 转向方向符号：前进端掉头（目标 ±π）左打为正；返程掉头（目标 0，
     * 车在对向车道）转向方向相反（右打）—— 旧实现恒左打，返程掉头
     * 转错方向（车转去更远）→ 掉头失败。 */
    const double steer_sign = (uturn_target_h < 0.5) ? -1.0 : 1.0;

    /* ═══ Phase 0 (可选): 倒车腾挪 — 前向空间不足时先倒车 ═══
     * 人类司机：路端空间不够时，先直线倒车腾出空间，再执行掉头。
     * 触发条件：forward_space_m > 0 且 < min_uturn_space。
     * 倒车到 forward_space ≥ min_uturn_space 或超时 5s。 */
    if (forward_space_m > 0.0 && forward_space_m < min_uturn_space) {
        double need_reverse = min_uturn_space - forward_space_m + 1.0;  /* +1m 安全余量 */
        double reversed = 0.0;
        steer = 0.0;  /* 直线倒车 */
        v = reverse_speed;
        double phase0_t = 0.0;
        LOG_INFO("planning", "[UTURN] forward_space=%.1fm < %.1fm → reverse to make room (need %.1fm)",
                 forward_space_m, min_uturn_space, need_reverse);
        while (n < max_points) {
            x += v * cos(h) * dt;
            y += v * sin(h) * dt;
            reversed += fabs(v) * dt;

            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;
            points[n].v = (float)v;  /* 负值 = 倒车档 */
            points[n].kappa = 0.0f;
            points[n].a = 0.0f;  points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            phase0_t += dt;
            t_us += (uint32_t)(dt * 1e6);

            bool space_ok = (reversed >= need_reverse);
            bool timeout  = (phase0_t >= phase0_max_dur);
            if (space_ok || timeout) break;
        }
        /* 倒车结束后刹停，再进入 Phase 1 */
        if (n < max_points && v < -0.1) {
            /* 加一个刹停点 */
            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;
            points[n].v = 0.0f;  /* 刹停 */
            points[n].kappa = 0.0f;
            points[n].a = -5.0f;  /* 中等刹车 */
            points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            t_us += (uint32_t)(dt * 1e6);
            v = 0.0;
        }
        LOG_INFO("planning", "[UTURN] Phase 0 done: reversed %.1fm, now at x=%.1f y=%.2f",
                 reversed, x, y);
    }

    /* ═══ Phase 1: 直线重刹 → 掉头低速 ═══ */
    {
        const double brake_decel = -8.0;  /* 急刹减速度 (m/s²) */
        while (v > uturn_speed + 0.1 && n < max_points) {
            v += brake_decel * dt;
            if (v < uturn_speed) v = uturn_speed;
            x += v * cos(h) * dt;
            y += v * sin(h) * dt;

            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;  points[n].v = (float)v;
            points[n].kappa = 0.0f;
            points[n].a = (float)brake_decel;
            points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            t_us += (uint32_t)(dt * 1e6);
        }
        /* 确保至少发布一个刹车点 */
        if (n == 0) {
            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;  points[n].v = (float)v;
            points[n].kappa = 0.0f;  points[n].a = 0.0f;
            points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            t_us += (uint32_t)(dt * 1e6);
        }
    }

    /* ═══ Phase 2: 左打死前进 → 冲向对向路沿 ═══
     * 自适应切换条件：
     *   - 正常：heading 转过 ≥130° 且 y 已越过路中线 (y > 0.5m)
     *   - 安全上限：heading 转过 ≥155° 或 y 逼近路沿 (y > road_half_w - 1.0)
     *   - 超时：phase2 超过 4s
     */
    {
        steer = steer_sign * uturn_steer;  /* 左打死/右打死（按目标方向） */
        v = uturn_speed;
        double phase2_t = 0.0;
        while (n < max_points) {
            double yaw_rate = (v / wheelbase) * tan(steer);
            h += yaw_rate * dt;
            /* 车辆中心参考点（与 physics.cpp step_bicycle 一致） */
            x += v * cos(h) * dt - half_wb * sin(h) * yaw_rate * dt;
            y += v * sin(h) * dt + half_wb * cos(h) * yaw_rate * dt;

            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;  points[n].v = (float)v;
            points[n].kappa = (float)(tan(steer) / wheelbase);
            points[n].a = 0.0f;  points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            phase2_t += dt;
            t_us += (uint32_t)(dt * 1e6);

            double dh = fabs(norm_h(h - h_start));
            bool heading_ok  = (dh >= 2.2689);              /* 130° in rad */
            /* 越过路中线方向按目标车道：前进端 y>0.5 / 返程 y<-0.5 */
            bool y_ok        = (uturn_target_h < 0.5) ? (y < -0.5) : (y > 0.5);
            bool safety_h    = (dh >= 2.7053);              /* 155° 安全上限 */
            bool safety_y    = (uturn_target_h < 0.5)
                                   ? (y < -(road_half_w - 1.0))
                                   : (y > road_half_w - 1.0);
            bool timeout     = (phase2_t >= phase2_max_dur);

            if ((heading_ok && y_ok) || safety_h || safety_y || timeout) break;
        }
    }

    /* ═══ Phase 2→3 交界: 刹停点 ═══
     * 掉头三把方向的前进→倒车必须经过 v=0（物理上 D→R 换挡要先刹停）。
     * 旧实现 Phase 2 直接跳 v=+3.5 → Phase 3 v=-3.0，control 在交界处
     * 靠 gear_pending 强置 target=0 刹停——但巡航 PID 对 2.4m/s 只给
     * 0.23 brake（decel≈1.2m/s²，需 1.9s/2.3m），车还没停就冲过倒车
     * 段、最近点跳到 Phase 4 正向段 → gear 翻回 D → 加速冲出路面
     * （2026-08-03 实测 y=8.5 路沿外）。补一个 v=0 刹停点，让
     * min|v| 前视把目标速度提前压下来 + gear 扫描提前看到倒车点。 */
    if (n < max_points && fabs(v) > 0.1) {
        points[n].t_rel_us = t_us;
        points[n].x = (float)x;  points[n].y = (float)y;
        points[n].heading = (float)h;
        points[n].v = 0.0f;  /* 刹停 → 换挡 */
        points[n].kappa = 0.0f;
        points[n].a = -5.0f;  /* 中等刹车 */
        points[n].jerk = 0.0f;  points[n].s = 0.0f;
        points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
        n++;
        t_us += (uint32_t)(dt * 1e6);
        v = 0.0;
    }

    /* ═══ Phase 3: 右打死倒车 → 车尾摆回车道区域 ═══
     * 倒车时方向盘右打 → 车尾右摆 → 车头左转（继续向对向车道方向转）。
     * 自适应切换条件：
     *   - 正常：y 回到对向车道半幅内 (|y - opp_lane_center_y| < 1.5m)
     *   - 安全上限：y 回到路中线以下 (y < 0.8m) 或 heading 接近 π
     *   - 超时：phase3 超过 3s
     */
    {
        steer = -steer_sign * max_steer;  /* 右打死/左打死（按目标方向，倒车相位反向） */
        v = reverse_speed;
        double phase3_t = 0.0;
        while (n < max_points) {
            double yaw_rate = (v / wheelbase) * tan(steer);
            h += yaw_rate * dt;
            /* 车辆中心参考点（与 physics.cpp step_bicycle 一致） */
            x += v * cos(h) * dt - half_wb * sin(h) * yaw_rate * dt;
            y += v * sin(h) * dt + half_wb * cos(h) * yaw_rate * dt;

            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;
            points[n].v = (float)v;  /* 负值 = 倒车档 */
            points[n].kappa = (float)(tan(steer) / wheelbase);
            points[n].a = 0.0f;  points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            phase3_t += dt;
            t_us += (uint32_t)(dt * 1e6);

            bool y_in_lane   = (fabs(y - opp_lane_center_y) < 1.5);  /* 在对向车道半幅内 */
            bool safety_y    = (y < 0.8);   /* 回到路中线以下，安全退出 */
            bool safety_h    = (fabs(norm_h(h - h_start)) >= M_PI - 0.35);  /* heading 接近 π */
            bool timeout     = (phase3_t >= phase3_max_dur);

            if (y_in_lane || safety_y || safety_h || timeout) break;
        }
    }

    /* ═══ Phase 4: 左打前进对齐 → 车头对准对向车道 ═══
     * 自适应切换条件：
     *   - 正常：heading 接近 ±π（|norm_h(h) - π| < 0.10 或 |norm_h(h) + π| < 0.10）
     *   - 安全上限：超时 2.5s
     * 容差 0.25→0.10：退出残差直接进 Phase 5 巡航轨迹 heading，control 以轨迹
     * 航向为横向目标 → 残差大 = 返程"斜着直行" + 触发 flowsim slew 原地转车身
     * （屁股横扫）。收严后掉头结束位姿更接近车道切线，slew 只清残余小角。
     */
    {
        steer = steer_sign * 0.15;  /* 较小转角精细对齐（方向按目标车道） */
        v = uturn_speed;
        double phase4_t = 0.0;
        while (n < max_points) {
            double yaw_rate = (v / wheelbase) * tan(steer);
            h += yaw_rate * dt;
            /* 车辆中心参考点（与 physics.cpp step_bicycle 一致） */
            x += v * cos(h) * dt - half_wb * sin(h) * yaw_rate * dt;
            y += v * sin(h) * dt + half_wb * cos(h) * yaw_rate * dt;

            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;  points[n].v = (float)v;
            points[n].kappa = (float)(tan(steer) / wheelbase);
            points[n].a = 0.0f;  points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            phase4_t += dt;
            t_us += (uint32_t)(dt * 1e6);

            double hn = norm_h(h);
            /* 对齐目标 = 目标车道方向（前进端掉头 ±π / 返程掉头 0） */
            bool heading_aligned = (fabs(norm_h(hn - uturn_target_h)) < 0.10);
            bool timeout         = (phase4_t >= phase4_max_dur);

            if (heading_aligned || timeout) break;
        }
    }

    /* ═══ Phase 5: 巡航 → 剩余点以目标速度直行填充 ═══
     * v 是车体纵向速度（负=倒挡），不是世界系 x 分量：h≈π 时
     * x += v·cos(h)·dt 已自动朝 -x 推进。旧实现 heading 反向时取
     * v=-cfg_target_speed 是双重取负——整段返程被标成"倒车 -20"
     * → control gear 推 REVERSE、末点 target=-20、"truncated in
     * reverse" 警告，掉头后狂倒车冲出路面（2026-08-03 demo6 y=-11）。 */
    {
        steer = 0.0;
        v = g.cfg_target_speed;
        while (n < max_points) {
            x += v * cos(h) * dt;
            y += v * sin(h) * dt;

            points[n].t_rel_us = t_us;
            points[n].x = (float)x;  points[n].y = (float)y;
            points[n].heading = (float)h;
            points[n].v = (float)v;
            points[n].kappa = 0.0f;
            points[n].a = 0.0f;  points[n].jerk = 0.0f;  points[n].s = 0.0f;
            points[n].l = (float)(y - road_center_y(x, g.curve_start_x, g.curve_length_m, g.curve_offset_m));
            n++;
            t_us += (uint32_t)(dt * 1e6);
        }
    }

    /* 截断兜底：64 点 (max_points) 在高速触发时可能截断在倒车段（v<0）。
     * control 的 target_speed 取轨迹末点 v，负目标在 DRIVE 档下 = 刹停到 0
     * → 掉头失败（实测 2026-08-03 spd 5.4→0.0）。强制末点为正速，
     * 倒车段由 gear 语义（最近点 v 符号）驱动，不靠末点 target。 */
    if (n > 0 && points[n - 1].v < -0.1) {
        points[n - 1].v = (float)uturn_speed;
        LOG_WARN("planning", "[UTURN] trajectory truncated in reverse segment — force end point v=%.1f (n=%d/%d)",
                 uturn_speed, n, max_points);
    }

    return n;
}

/* 段感知下采样：512 点细轨迹 → 消息容量 64 点。
 * 必须保留 v 符号翻转边界点（D/R 换挡点），否则倒车段被稀释后
 * control 的 gear 推导（最近点向前扫符号）会跳段错挡。
 * control 是最近点追踪，~0.5m 点距足够。 */
static int downsample_uturn(const TrajectoryPoint* src, int sn,
                            TrajectoryPoint* dst, int cap) {
    if (sn <= cap) {
        memcpy(dst, src, sizeof(TrajectoryPoint) * (size_t)sn);
        return sn;
    }
    static bool keep[512];
    memset(keep, 0, sizeof(keep));
    keep[0] = true; keep[sn - 1] = true;
    int forced = 2;
    for (int i = 1; i < sn; i++) {
        bool a = src[i - 1].v < -0.05f, b = src[i].v < -0.05f;
        if (a != b) {
            if (!keep[i - 1]) { keep[i - 1] = true; forced++; }
            if (!keep[i])     { keep[i] = true;     forced++; }
        }
    }
    int remain = cap - forced;
    double stride = (double)sn / (double)(remain > 0 ? remain + 1 : 1);
    for (int k = 1; k <= remain; k++) {
        int idx = (int)(k * stride);
        if (idx < 1) idx = 1;
        if (idx > sn - 1) idx = sn - 1;
        while (idx < sn && keep[idx]) idx++;
        if (idx < sn) keep[idx] = true;
    }
    int n = 0;
    for (int i = 0; i < sn && n < cap; i++)
        if (keep[i]) dst[n++] = src[i];
    return n;
}

/* ── fusion/localization 订阅回调 ───────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */

    /* cJSON parsing (fusion/localization now publishes cJSON) */
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        /* vehicle/state 近期到达时不覆盖（同 behavior_planner 的修复） */
        uint64_t now = clock_now_us();
        bool vstate_recent = (g.last_vstate_us != 0 && now - g.last_vstate_us < 200000ULL);
        if (!vstate_recent) {
            cJSON* item;
            item = cJSON_GetObjectItem(root, "x");
            if (cJSON_IsNumber(item))       g.ego_x = item->valuedouble;
            item = cJSON_GetObjectItem(root, "y");
            if (cJSON_IsNumber(item))       g.ego_y = item->valuedouble;
            item = cJSON_GetObjectItem(root, "v");
            if (cJSON_IsNumber(item))       g.ego_v = item->valuedouble;
            item = cJSON_GetObjectItem(root, "heading");
            if (cJSON_IsNumber(item))       g.ego_heading = item->valuedouble;
        }
        cJSON_Delete(root);
    }
    g.has_fusion = 1;
    g.last_fusion_us = clock_now_us();
}

/* ── vehicle/state 订阅 — 用 flowsim 真值覆盖 ego 位置 ── */
static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItem(root, "x")) && cJSON_IsNumber(j))
        g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "y")) && cJSON_IsNumber(j))
        g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "spd")) && cJSON_IsNumber(j))
        g.ego_v = j->valuedouble;
    if ((j = cJSON_GetObjectItem(root, "hdg")) && cJSON_IsNumber(j))
        g.ego_heading = j->valuedouble;
    /* 车参同步：flowsim 广播（场景 ego 块配置了 wheelbase 才发） */
    if ((j = cJSON_GetObjectItem(root, "wheelbase")) && cJSON_IsNumber(j) && j->valuedouble > 0.0) {
        g.wheelbase = j->valuedouble;
    }
    if ((j = cJSON_GetObjectItem(root, "lane_id")) && cJSON_IsNumber(j)) {
        int new_lane_id = (int)j->valuedouble;
        /* 检测车道方向变化：负→正（U-turn 到对向车道）或正→负（掉头回前进车道）
         * 参考路径必须立即更新，否则 Frenet 规划器用正向路径生成反向轨迹。 */
        if ((g.ego_lane_id < 0 && new_lane_id > 0) ||
            (g.ego_lane_id > 0 && new_lane_id < 0)) {
            g.ref_path_update_pending = 1;
            LOG_WARN("planning", "lane_id sign change: %d -> %d, ref_path update pending",
                     g.ego_lane_id, new_lane_id);
        }
        g.ego_lane_id = new_lane_id;
    }
    g.last_vstate_us = clock_now_us();
    g.has_fusion = 1;

    /* 真值障碍物缓存（vehicle/state 的 oidN/oxN/oyN/ovN/ovyN）。
     * 仅用于 TTC 安全兜底——感知链（perception/obstacles）漏检/停更时，
     * 本车道前车物理 gap 仍有依据可限速，杜绝"FOLLOW 状态巡航速度追尾"
     * （2026-07 实测 entity6 追尾：TTC 感知链 0 次触发）。 */
    j = cJSON_GetObjectItem(root, "n_obs");
    if (cJSON_IsNumber(j)) {
        int n = (int)j->valuedouble;
        if (n > g.kMaxTruthObs) n = g.kMaxTruthObs;
        g.truth_obs_count = 0;
        for (int i = 0; i < n; i++) {
            char key[20];
            snprintf(key, sizeof key, "ox%d", i);
            cJSON* jx = cJSON_GetObjectItem(root, key);
            snprintf(key, sizeof key, "oy%d", i);
            cJSON* jy = cJSON_GetObjectItem(root, key);
            snprintf(key, sizeof key, "ov%d", i);
            cJSON* jv = cJSON_GetObjectItem(root, key);
            if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jy)) continue;
            g.truth_obs_x[i] = jx->valuedouble;
            g.truth_obs_y[i] = jy->valuedouble;
            g.truth_obs_vx[i] = cJSON_IsNumber(jv) ? jv->valuedouble : 0.0;
            g.truth_obs_count++;
        }
    }
    cJSON_Delete(root);
}

/* ── perception/obstacles 订阅 — 解析障碍物（车体坐标系→世界坐标） ──── */

static void on_perception_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */

    ObstacleList list;
    if (ObstacleList_deserialize(&list, (const uint8_t*)msg->data, msg->data_size) != 0)
        return;

    double ch = cos(g.ego_heading), sh = sin(g.ego_heading);
    for (int i = 0; i < g.kMaxObs; i++) {
        if (i < (int)list.count) {
            const Obstacle* o = &list.obstacles[i];
            /* 车体坐标系 → 世界坐标 */
            g.obs_x[i]  = g.ego_x + o->x * ch - o->y * sh;
            g.obs_y[i]  = g.ego_y + o->x * sh + o->y * ch;
            g.obs_vx[i] = o->vx * ch - o->vy * sh;
            g.obs_vy[i] = o->vx * sh + o->vy * ch;
            g.obs_lane_id[i] = o->lane_id;  /* 感知已算好的车道归属 */
            g.obs_type[i] = (uint8_t)o->type;
            g.obs_confidence[i] = o->confidence;
        } else {
            g.obs_x[i] = g.obs_y[i] = g.obs_vx[i] = g.obs_vy[i] = 0.0;
            g.obs_lane_id[i] = -1;
            g.obs_type[i] = 0;
            g.obs_confidence[i] = 0.0f;
        }
    }
    g.has_vstate = 1;
}

/* ── road/geometry 订阅回调（Phase 2 统一道路几何） ─────────── */
/* 从 flowsim_node 发布的 road/geometry topic 获取弯道参数，
 * 替代此前从 scenario_load() 读取弯道的冗余方式。
 * NOA route steps 由 navigation_node 在 navigation/path 下发。 */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    const char* d = (const char*)msg->data;
    static int rg_count = 0;
    if (rg_count++ < 5 || rg_count % 50 == 0)
        LOG_INFO("planning", "[RG] recv road/geometry #%d size=%zu: %.80s", rg_count, (size_t)msg->data_size, d);
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* item;
        if ((item = cJSON_GetObjectItem(root, "curve_start_x")))  g.curve_start_x = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "curve_length_m"))) g.curve_length_m = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "curve_offset_m"))) g.curve_offset_m = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "lane_count"))) {
            int new_lc = item->valueint;
            if (new_lc != g.lane_count)
                LOG_WARN("planning", "[RG] lane_count UPDATE %d -> %d", g.lane_count, new_lc);
            g.lane_count = new_lc;
        }
        if ((item = cJSON_GetObjectItem(root, "lane_width")))     g.lane_width = item->valuedouble;
        cJSON_Delete(root);
    } else {
        LOG_WARN("planning", "[RG] cJSON_Parse FAILED");
    }
}

static void on_road_ref_path(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* points = cJSON_GetObjectItemCaseSensitive(root, "points");
    if (!cJSON_IsArray(points)) {
        cJSON_Delete(root);
        return;
    }
    int n = cJSON_GetArraySize(points);
    if (n > 128) n = 128;
    int out_n = 0;
    double accum_s = 0.0;
    double prev_x = 0.0, prev_y = 0.0;
    for (int i = 0; i < n; ++i) {
        cJSON* pt = cJSON_GetArrayItem(points, i);
        if (!cJSON_IsObject(pt)) continue;
        cJSON* jx = cJSON_GetObjectItemCaseSensitive(pt, "x");
        cJSON* jy = cJSON_GetObjectItemCaseSensitive(pt, "y");
        if (!cJSON_IsNumber(jx) || !cJSON_IsNumber(jy)) continue;
        double x = jx->valuedouble;
        double y = jy->valuedouble;
        if (out_n > 0) {
            double dx = x - prev_x;
            double dy = y - prev_y;
            accum_s += sqrt(dx * dx + dy * dy);
        }
        g.map_ref_x[out_n] = x;
        g.map_ref_y[out_n] = y;
        g.map_ref_s[out_n] = accum_s;
        prev_x = x;
        prev_y = y;
        out_n++;
    }
    g.map_ref_count = out_n;
    if (out_n >= 2) g.last_map_ref_us = clock_now_us();
    cJSON_Delete(root);
}

/* ── planning/behavior 订阅 — 接收行为规划指令 ──────────── */
static void on_planning_behavior(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
    Behavior beh;
    if (Behavior_deserialize(&beh, (const uint8_t*)msg->data, msg->data_size) != 0) {
        static int desfail_count = 0;
        if (desfail_count++ < 3)
            LOG_WARN("planning", "[DBG_BEH] deserialize FAILED size=%zu", (size_t)msg->data_size);
        return;
    }
    g.current_behavior = beh;
    int prev_overtake = g.overtake_state;
    g.overtake_state = (beh.command == BEH_LEFT_CHANGE) ? 1 :
                       (beh.command == BEH_RIGHT_CHANGE) ? 2 :
                       (beh.command == BEH_U_TURN) ? 3 : 0;
    /* 掉头退出（overtake 3→其他）时清掉头轨迹缓存，下次掉头重新生成 */
    if (prev_overtake == 3 && g.overtake_state != 3) {
        g.uturn_cache_n = 0;
        LOG_INFO("planning", "[UTURN] exit overtake_state=%d → cache cleared", g.overtake_state);
    }
    g.has_behavior = 1;
    static int beh_recv_count = 0;
    if (beh_recv_count++ % 50 == 0 || beh.command == BEH_RIGHT_CHANGE || beh.command == BEH_LEFT_CHANGE) {
        LOG_WARN("planning", "[DBG_BEH] recv cmd=%d tgt_lane=%d tgt_spd=%.1f has_beh=%d",
                 (int)beh.command, (int)beh.target_lane_idx, (double)beh.target_speed, g.has_behavior);
    }
}

/* ── navigation/path 订阅回调（单轨导航驱动） ─────────────── */
/* 消息格式：
 *   route_status: {"type":"route_status","route_count":N,"next_idx":k}
 *   route_step  : {"type":"route_step","step_type":"lane_change|branch_select|merge", ...}
 */
static void on_navigation_path(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;

    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    cJSON* jtype = cJSON_GetObjectItemCaseSensitive(root, "type");
    if (!cJSON_IsString(jtype) || !jtype->valuestring) {
        cJSON_Delete(root);
        return;
    }

    g.last_nav_us = clock_now_us();

    if (strcmp(jtype->valuestring, "route_status") == 0) {
        cJSON* jcount = cJSON_GetObjectItemCaseSensitive(root, "route_count");
        cJSON* jnext = cJSON_GetObjectItemCaseSensitive(root, "next_idx");
        if (cJSON_IsNumber(jcount)) g.route_count = (int)jcount->valuedouble;
        if (cJSON_IsNumber(jnext)) g.route_next_idx = (int)jnext->valuedouble;
        g.has_navigation = (g.route_count > 0) ? 1 : 0;
        cJSON_Delete(root);
        return;
    }

    if (strcmp(jtype->valuestring, "route_step") != 0) {
        cJSON_Delete(root);
        return;
    }

    RouteStepType step_type = ROUTE_LANE_CHANGE;
    int target_lane = -1;
    double target_speed = -1.0;
    int branch_id = -1;
    int step_index = -1;
    double ego_x = g.ego_x;

    cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "step_type");
    if (cJSON_IsString(j) && j->valuestring) {
        if (strcmp(j->valuestring, "branch_select") == 0) step_type = ROUTE_BRANCH_SELECT;
        else if (strcmp(j->valuestring, "merge") == 0)    step_type = ROUTE_MERGE;
        else if (strcmp(j->valuestring, "u_turn") == 0)   step_type = ROUTE_U_TURN;
    }
    j = cJSON_GetObjectItemCaseSensitive(root, "target_lane");
    if (cJSON_IsNumber(j)) target_lane = (int)j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "target_speed");
    if (cJSON_IsNumber(j)) target_speed = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "branch_id");
    if (cJSON_IsNumber(j)) branch_id = (int)j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "step_index");
    if (cJSON_IsNumber(j)) step_index = (int)j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "ego_x");
    if (cJSON_IsNumber(j)) ego_x = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "route_count");
    if (cJSON_IsNumber(j)) g.route_count = (int)j->valuedouble;
    g.has_navigation = (g.route_count > 0) ? 1 : 0;
    if (step_index >= 0) g.route_next_idx = step_index + 1;

    g.route_type = step_type;
    switch (step_type) {
        case ROUTE_BRANCH_SELECT: {
            int lane = (target_lane != 0) ? target_lane : -1;
            g.route_target_lane = lane;
            g.current_branch_id = branch_id;
            g.route_target_speed = (target_speed > 0.0) ? target_speed : -1.0;
            update_reference_path(g.ego_x - 5.0, g.ego_lane_id > 0);
            LOG_INFO("planning", "NAV branch_select #%d @x=%.0f -> branch_id=%d lane=%d",
                     step_index, ego_x, branch_id, lane);
            break;
        }
        case ROUTE_MERGE: {
            int lane = (target_lane != 0) ? target_lane : 1;
            g.route_target_lane = lane;
            g.merge_hold_lane = lane;
            g.merge_state = 1;
            g.current_branch_id = -1;
            g.route_target_speed = (target_speed > 0.0) ? target_speed : g.cfg_max_speed;
            LOG_INFO("planning", "NAV merge #%d @x=%.0f -> lane=%d speed=%.1f wait_gap",
                     step_index, ego_x, lane, g.route_target_speed);
            break;
        }
        case ROUTE_U_TURN: {
            /* 掉头触发已统一到 behavior_planner（路端检测 + 状态机 + 冷却 +
             * COMPLETED 完成握手）。navigation 的 route u_turn 步骤仅作播报，
             * 不再直连 overtake_state —— 旧实现与 behavior 触发并存，构成
             * 双重触发：NAV 置位被下一条 behavior 消息无条件覆盖，掉头在
             * 触发边缘抖动（实测 7ms 内退出，2026-08-03）。 */
            LOG_INFO("planning", "NAV u_turn #%d @x=%.0f ignored — u-turn unified to behavior (overtake=%d)",
                     step_index, ego_x, g.overtake_state);
            break;
        }
        case ROUTE_LANE_CHANGE:
        default:
            g.route_target_lane = target_lane;
            g.current_branch_id = -1;
            if (target_speed >= 0.0) g.route_target_speed = target_speed;
            else                     g.route_target_speed = -1.0;
            LOG_INFO("planning", "NAV lane_change #%d @x=%.0f -> lane=%d speed=%.1f",
                     step_index, ego_x, target_lane, g.route_target_speed);
            break;
    }

    cJSON_Delete(root);
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
    if (!msg) return;  /* data 是定长数组，永不为 NULL；空载由 data_size 判定 */
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
/* 从 flowsim_node 发布的 road/traffic_lights topic 获取红绿灯状态。
 * JSON 格式: {"lights":[{"id":0,"x":100.0,"y_lane":-1.75,
 *                         "state":"red","remain_s":12.3},...]}
 * 解析每个灯的 x(停止线位置)、y_lane(车道)、state(green/yellow/red)。
 * 缓存到 g.tl_x/tl_y_lane/tl_state，供 Frenet 障碍物注入逻辑读取。 */
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

/* ── 协程任务 ────────────────────────────────────────────────── */

class PlanningTask : public CoroutineTask {
public:
    PlanningTask(MessageBus* bus) : CoroutineTask(bus) {}

    void set_params(Transport* transport) {
        transport_ = transport;
    }

    /* 在 Trajectory 上按时间插值。
     * 返回距离 t_rel_us 最近的两个点的线性插值。 */
    static bool traj_interpolate(const Trajectory* traj, uint32_t t_rel_us,
                                  double* x, double* y, double* heading,
                                  double* v, double* a) {
        if (!traj || traj->point_count == 0) return false;
        uint32_t n = traj->point_count;

        /* 找到最后一个 t <= target 的点 */
        int idx = 0;
        for (uint32_t i = 0; i < n; i++) {
            if (traj->points[i].t_rel_us <= t_rel_us) idx = (int)i;
        }

        if (idx >= (int)n - 1) {
            /* 超出轨迹末端，用最后一个点 */
            *x = (double)traj->points[n-1].x;
            *y = (double)traj->points[n-1].y;
            *heading = (double)traj->points[n-1].heading;
            *v = (double)traj->points[n-1].v;
            *a = (double)traj->points[n-1].a;
            return true;
        }

        const TrajectoryPoint* p0 = &traj->points[idx];
        const TrajectoryPoint* p1 = &traj->points[idx+1];
        double dt = (double)(p1->t_rel_us - p0->t_rel_us);
        double frac = (dt > 0) ? (double)(t_rel_us - p0->t_rel_us) / dt : 0.0;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;

        *x = (double)p0->x + frac * ((double)p1->x - (double)p0->x);
        *y = (double)p0->y + frac * ((double)p1->y - (double)p0->y);
        *heading = (double)p0->heading + frac * ((double)p1->heading - (double)p0->heading);
        *v = (double)p0->v + frac * ((double)p1->v - (double)p0->v);
        *a = (double)p0->a + frac * ((double)p1->a - (double)p0->a);
        return true;
    }

protected:
    Task run() override {
        /* 参考路径: 长直线（左车道中心 y=-1.75）。运行时间较长时，ego 会开出
         * 初始路径范围；接近末端时向前滑动 reference path，避免 Frenet 插值越界
         * 导致 planning 线程挂掉。 */
        update_reference_path(0.0);

        while (!should_stop()) {
            /* select_for: 等待 fusion、障碍物或道路几何更新触发规划（消息驱动），
             * 50ms 超时兜底。替代 sleep_us 轮询，降低空等 CPU 占用。 */
            auto r = co_await select_for(bus(),
                {TOPIC_FUSION_LOCALIZATION, TOPIC_PERCEPTION_OBSTACLES,
                 TOPIC_ROAD_GEOMETRY, TOPIC_ROAD_REF_PATH}, 50000);
            (void)r;
            if (should_stop()) break;

            /* 20Hz rate limit */
            uint64_t _plan_now = clock_now_us();
            if (_plan_now - g.last_plan_us < 50000) continue;
            g.last_plan_us = _plan_now;

            /* §11.2: heartbeat 上报 — monitor_node 的 degrade_supervisor_tick 据此检测超时 */
            degrade_supervisor_record_heartbeat("planning_node", clock_now_us() / 1000);

            if (!g.has_fusion) continue;

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

            /* 路线触发由 navigation_node 完成；planning 只消费 navigation/path。 */

            /* 参考路径更新：车道方向变化（U-turn 完成）时立即重建参考路径
             *   负→正（对向车道）：ref_path 从右向左生成，匹配反向行驶方向
             *   正→负（回到前进车道）：ref_path 从左向右生成，恢复正常行驶 */
            if (g.ref_path_update_pending) {
                g.ref_path_update_pending = 0;
                bool opposite = (g.ego_lane_id > 0);
                double start_x = opposite ? g.ego_x + g.cfg_ref_path_length * 0.5
                                          : fmax(g.ego_x - 50.0, 0.0);
                update_reference_path(start_x, opposite);
                LOG_WARN("planning", "ref_path rebuilt for lane_id=%d opposite=%d start_x=%.0f..%.0f",
                         g.ego_lane_id, opposite ? 1 : 0,
                         opposite ? start_x - g.cfg_ref_path_length : start_x,
                         opposite ? start_x : start_x + g.cfg_ref_path_length);
            }

            /* 参考路径滑动：根据行驶方向（lane_id 符号）前移/后移
             *   前进车道（lane_id < 0）：ego_x 递增，路径向右滑动
             *   对向车道（lane_id > 0）：ego_x 递减，路径向左滑动
             * 对向车道的参考路径覆盖 [ref_path_start_x, ref_path_start_x + ref_path_length]，
             * "前方"方向是 x 递减（从高 x 到低 x），故 ego 应位于高 x 端（路径末端）。
             * 滑动条件：ego 接近路径低 x 端（< 20% 位置）时，将路径末端前移到 ego 附近。 */
            bool opposite = (g.ego_lane_id > 0);
            if (opposite) {
                if (g.ego_x < g.ref_path_start_x + g.cfg_ref_path_length * 0.2) {
                    /* 对向车道：路径覆盖 [ref_path_start_x, ref_path_start_x + ref_path_length]，
                     * "前方"是 x 递减方向，ego 应位于路径末端（高 x 端）。
                     * ego 到达低 x 端（<20%）时，把路径末端移到 ego 前方 50m。 */
                    double new_start = g.ego_x + 50.0;
                    update_reference_path(new_start, true);
                    LOG_INFO("planning", "ref path shifted (opposite) to x=%.0f..%.0f",
                             new_start - g.cfg_ref_path_length, new_start);
                }
            } else {
                if (g.ego_x > g.ref_path_start_x + g.cfg_ref_path_length * 0.8) {
                    double new_start = g.ego_x - 50.0;
                    if (new_start < 0.0) new_start = 0.0;
                    update_reference_path(new_start, false);
                    LOG_INFO("planning", "reference path shifted to x=%.0f..%.0f",
                             g.ref_path_start_x, g.ref_path_start_x + g.cfg_ref_path_length);
                }
            }

            /* ── 速度决策：behavior 是唯一来源（Apollo 原则）──
             * behavior_planner 拥有速度决策权：
             *   - CRUISE → cfg_cruise_speed（如 15.0）
             *   - FOLLOW → lead_speed 或 CTG 计算速度
             *   - STOP/YIELD/EMERGENCY → 0
             *   - LEFT_CHANGE/RIGHT_CHANGE → 变道速度
             *
             * planning 无条件消费 behavior 的 target_speed，不再只在 < command_speed 时覆盖。
             * 无 behavior 时回退到 scenario 默认 target_speed。
             * NOA route_target_speed 优先级最高（匝道限速/stop_at_red）。 */
            double command_speed;
            if (g.route_target_speed >= 0.0) {
                command_speed = g.route_target_speed;  /* NOA 路线步骤显式指定 */
            } else if (g.has_behavior && g.current_behavior.target_speed >= 0.0f) {
                command_speed = (double)g.current_behavior.target_speed;  /* behavior 决策 */
            } else {
                command_speed = g.target_speed;  /* scenario 默认 */
            }

        /* DBG: 打印 behavior 覆盖后的 command_speed */
        if (g.has_behavior && g.plan_count == 5) {
            LOG_WARN("planning", "[DBG_SPD] route_ts=%.1f g.ts=%.1f beh_cmd=%d beh_spd=%.1f cmd_spd=%.1f has_beh=%d",
                     g.route_target_speed >= 0.0 ? g.route_target_speed : -1.0,
                     g.target_speed,
                     g.has_behavior ? (int)g.current_behavior.command : -1,
                     g.has_behavior ? (double)g.current_behavior.target_speed : -1.0,
                     command_speed, g.has_behavior);
        }
            /* 安全夹紧：scenario JSON 的 target_speed 是"信任来源"，但安全上限不变量
             * 应在规划层就守住——任何来源的目标速度都不能超过 cfg_max_speed。
             *
             * 原 failsafe 分支（line 800）有此夹紧，主分支漏了。若 scenario 误写
             * target_speed: 50.0，规划层会原样下发，依赖 safety_control 的 TTC 限幅
             * 兜底——但 safety_control 假定上游 speed 已合理，不做绝对速度上限夹紧。
             * 这是安全分层架构里职责错位（上限夹紧应在最靠近数据源的层做）。 */
            if (command_speed > g.cfg_max_speed) {
                static int speed_clamp_warn = 0;
                if (speed_clamp_warn < 3) {
                    LOG_WARN("planning", "command_speed clamped %.2f → cfg_max_speed %.2f "
                             "(scenario target_speed/route_target_speed exceeds safety envelope)",
                             command_speed, g.cfg_max_speed);
                    speed_clamp_warn++;
                }
                command_speed = g.cfg_max_speed;
            }

            /* §5a: 掉头后重置 route_target_speed
             * flowsim_node 在路尾把 ego 瞬移到 s=0 (x≈5)，但 route_target_speed
             * 残留上次 route 步骤的值（如 enter_noa_route 的 12.0），导致
             * command_speed 被锁死在低值，车辆无法加速到巡航速度。
             * 检测条件：ego_x < 60（第一个 route 步骤 trigger_x），说明已回到起点。 */
            if (g.ego_x < 60.0 && g.route_target_speed >= 0.0) {
                static int teleport_warn = 0;
                if (teleport_warn < 3) {
                    LOG_WARN("planning", "teleport detected (ego_x=%.1f < 60), resetting route_target_speed %.1f → -1",
                             g.ego_x, g.route_target_speed);
                    teleport_warn++;
                }
                g.route_target_speed = -1.0;
            }

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
                    /* gap 不足：跟车巡航，不下发变道（N 车道模型：-1=无目标） */
                    g.route_target_lane = -1;
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

            /* ── 常规跟车 TTC 兜底 ─────────────────────────────
             * 与 NOA merge 的 (gap-5)/3 同一模式，但作用于常规巡航/跟车链：
             * 无论 behavior 是否正确下发 follow_speed（2026-07 实测追尾 id 11
             * 时 FOLLOW 状态速度保持 15.1 巡航值——感知抖动/状态翻转空窗），
             * 本车道前车物理 gap 逼近时按 3s TTC 强制限速，杜绝巡航速度追尾。
             * 用 planning 自己的感知障碍物（perception/obstacles），与
             * behavior 的 tracked_objects 独立，形成第二道防线。
             * 横向归属：按 ego 当前实际位置量化车道（变道中跨线时取本车道
             * 前车，保守方向——变道完成前仍视原车道前车为危险源）。 */
            if (g.has_vstate && g.lane_count >= 1) {
                double lane_w = g.lane_width;
                double lc_f = (-g.ego_y) / lane_w + (g.lane_count - 1) * 0.5;
                int cur_lane = (int)(lc_f >= 0.0 ? lc_f + 0.5 : lc_f - 0.5);
                if (cur_lane < 0) cur_lane = 0;
                if (cur_lane >= g.lane_count) cur_lane = g.lane_count - 1;
                double lane_center = lane_center_y(cur_lane, g.lane_count, lane_w);
                double min_gap = 1e9;
                int gap_src = 0;  /* 0=none 1=perception 2=truth */
                for (int i = 0; i < g.kMaxObs; i++) {
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx <= 0.0 || dx > 80.0) continue;
                    if (g.obs_vx[i] < -2.0) continue;  /* 对向车不走此限速 */
                    if (std::fabs(g.obs_y[i] - lane_center) > lane_w * 0.75) continue;
                    if (dx < min_gap) { min_gap = dx; gap_src = 1; }
                }
                /* 真值兜底：感知链未提供本车道前车（漏检/停更）时，
                 * 用 flowsim 真值障碍物计算 gap——安全兜底不依赖感知链。 */
                if (min_gap >= 1e8) {
                    for (int i = 0; i < g.truth_obs_count; i++) {
                        double dx = g.truth_obs_x[i] - g.ego_x;
                        if (dx <= 0.0 || dx > 80.0) continue;
                        if (g.truth_obs_vx[i] < -2.0) continue;
                        if (std::fabs(g.truth_obs_y[i] - lane_center) > lane_w * 0.75) continue;
                        if (dx < min_gap) { min_gap = dx; gap_src = 2; }
                    }
                }
                if (min_gap < 80.0 && min_gap > 0.0) {
                    /* 4s TTC：gap=65m 起开始压制（(65-5)/4=15≈巡航），
                     * 比 3s 版提前 15m 介入。2026-07 追尾复盘：3s 版在
                     * gap 23.5m 才压到 6.2 m/s，全刹制动距离 21m 加上
                     * 决策→执行延迟（~0.5s≈7.5m），物理上刹不住。 */
                    double ttc_follow = (min_gap - 5.0) / 4.0;
                    if (ttc_follow < 0.0) ttc_follow = 0.0;
                    if (ttc_follow < command_speed) {
                        if (g.plan_count % 10 == 0) {
                            LOG_WARN("planning", "TTC follow: gap=%.1f (src=%s) -> %.1f m/s (was %.1f)",
                                     min_gap, gap_src == 2 ? "truth" : "percept", ttc_follow,
                                     command_speed);
                        }
                        command_speed = ttc_follow;
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
                /* 会车横向判定上限：相邻车道范围内才算迎头，2+ 车道外的
                 * 对向车不是直接威胁。多车道高速上 ego 在 lane3、对向车在
                 * lane0（横向 10.5m）时，旧逻辑 |dy|>2.0 也触发让行，把
                 * 巡航压到 0.4× 刹停——2026-07-31 实跑。 */
                double oncoming_dy_max = (g.lane_width > 1.0 ? g.lane_width : 3.5) * 1.5;

                for (int i = 0; i < g.kMaxObs; i++) {
                    double dx = g.obs_x[i] - g.ego_x;
                    if (dx < -10.0 || dx > 80.0) continue;
                    double dy = g.obs_y[i] - g.ego_y;  /* 相对 ego 的横向偏移 */

                    /* 会车检测: 对向车道且横向相邻 (2.0 < |dy| ≤ 1.5×路宽),
                     * 迎面驶来 (vx < -2 m/s), 前方 60m 内 */
                    if (std::fabs(dy) > 2.0 && std::fabs(dy) <= oncoming_dy_max &&
                        dx > 0.0 && dx < 60.0 && g.obs_vx[i] < -2.0) {
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
                /* 障碍物数组扩容到 128：vehicle/state 障碍物 + 最多 4 个
             * 红绿灯虚拟停止线墙。红绿灯墙在红灯/黄灯时注入，绿灯时不注入。
             * Phase 3: 添加 vx/vy 数组，传给 Frenet 做位置外推。 */
            double ox[128], oy[128], ow[128], ol[128], ovx[128], ovy[128];
            int n_obs = 0;
            for (int i = 0; i < g.kMaxObs && n_obs < 128; i++) {
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
                 * Frenet 规划器在轨迹优化时会绕开/减速至墙前停车。
                 *
                 * 注意：虚拟墙仅存在于 Frenet 内部障碍物数组，不发布到
                 * perception/obstacles topic，故 safety_control 看不到虚拟墙。
                 * 红绿灯停车的执行链路是 planning(target_speed=0) → control(PID 跟踪)
                 * → safety(透传)。safety_control 作为最后闸门不直接感知红绿灯，
                 * 依赖 control 正确跟踪 planning 下发的 target_speed。
                 *
                 * 刹停距离估算: d_brake = v² / (2*a) + 余量，a≈4 m/s²。
                 * 黄灯判据: 仅当能安全刹停时注入（dx > min_stop_dist）；
                 *           太近无法安全刹停时不注入（让车通过，避免急刹追尾）。 */
                if (g.has_traffic_lights && n_obs < 128) {
                    double v = g.ego_v;
                    if (v < 0.0) v = 0.0;
                    /* 刹停距离 = v²/(2*4) + 3m 安全余量；最小 5m 保证近距也能停 */
                    double brake_dist = v * v / 8.0 + 3.0;
                    if (brake_dist < 5.0) brake_dist = 5.0;
                    /* 黄灯最小安全刹停距离：速度太低时用 3m */
                    double min_yellow_stop = (v > 2.0) ? 3.0 : 0.0;

                    for (int ti = 0; ti < g.tl_count && n_obs < 128; ti++) {
                        if (g.tl_state[ti] == 0) continue;  /* 绿灯，不注入 */
                        double dx_tl = g.tl_x[ti] - g.ego_x;
                        if (dx_tl <= 0.0) continue;  /* 已过停止线 */
                        if (dx_tl > 60.0) continue;  /* 太远，不注入 */

                        if (g.tl_state[ti] == 2) {
                            /* 红灯：在刹停距离内注入墙
                             * 加大余量到 brake_dist + 20m 补 fusion 滞后。 */
                            if (dx_tl > brake_dist + 20.0) continue;  /* 还很远，不急 */
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

            /* ── 消费 behavior_planner 的行为指令 ──
             * behavior_planner_node 运行状态机（巡航/跟车/变道），
             * 我们只根据 Behavior.target_lane_idx 算横向偏移。
             * 变道完成检测由 behavior_planner 通过 ego_y 位置做。 */
            {
                double lane_w = g.lane_width;
                int n_lanes = g.lane_count;
                double ego_lane_d = g.ego_y;
                double ego_ref_s = 0.0;
                (void)project_to_reference_path(g.ego_x, g.ego_y, ego_ref_s, ego_lane_d);

                if (g.lane_count < 1 || g.lane_count > 16 || g.lane_width < 1.0 || g.lane_width > 10.0) {
                    LOG_WARN("planning", "lane invariant: lane_count=%d lane_width=%.1f (unreasonable)",
                             g.lane_count, g.lane_width);
                }

                /* 更新 target_lane_offset
                 * 变道中：目标 = behavior 下发的目标车道中心
                 * 巡航/跟车：目标 = ego 当前最近车道中心（而非道路中心 y=0）。
                 *   变道 COMPLETED 后 ego 可能还未到新车道中心（如 y=-3.5），
                 *   若 target_lane_offset=0（道路中心），Frenet 会保持当前 y 不动，
                 *   ego 卡在两车道之间。改为跟踪最近车道中心，让 ego 继续归位。 */
                if (g.has_behavior && g.current_behavior.target_lane_idx >= 0 &&
                    g.current_behavior.target_lane_idx < n_lanes &&
                    (g.current_behavior.command == BEH_LEFT_CHANGE || g.current_behavior.command == BEH_RIGHT_CHANGE)) {
                    g.target_lane_offset = lane_center_offset(g.current_behavior.target_lane_idx, n_lanes, lane_w);
                } else {
                    /* 巡航/跟车：计算 ego 当前最近车道，目标其中心。
                     * 只允许本方向合法车道（行进坐标系下右半幅，idx ≥ n_lanes/2）：
                     * map_ref 已随行进方向对齐（返程反向采样），左侧半幅恒为对向
                     * 车道。不 clamp 则 ego 被扰动推过道路中心时"最近车道"跟着
                     * ego 翻到对向侧（正反馈），最终锁定逆行目标（2026-08-03
                     * demo12 实测：返程 ego 漂到 y=-8，目标锁 y=-5.25 东行道）。 */
                    double off = (-ego_lane_d) / lane_w + (n_lanes - 1) * 0.5;
                    int cur_lane = (int)(off >= 0.0 ? off + 0.5 : off - 0.5);
                    int own_side_min = n_lanes / 2;
                    if (cur_lane < own_side_min) cur_lane = own_side_min;
                    if (cur_lane >= n_lanes) cur_lane = n_lanes - 1;
                    g.target_lane_offset = lane_center_offset(cur_lane, n_lanes, lane_w);
                }
                if (g.plan_count % 200 == 0) {
                    LOG_WARN("planning", "[DBG_LC] pc=%d has_beh=%d cmd=%d tgt_lane=%d n_lanes=%d offset=%.2f ego_y=%.2f",
                            g.plan_count, g.has_behavior, (int)g.current_behavior.command,
                            (int)g.current_behavior.target_lane_idx, n_lanes, g.target_lane_offset, g.ego_y);
                }
            }

            /* 规划轨迹 */
            TrajectoryPoint points[64];
            int n_pts = 0;
            int n_wp = 0;  /* 规划路径点数量（Frenet 规划器输出；UTurn 时保持 0） */
            bool use_stitch = false;  /* 轨迹拼接标志 */
            double ego_ref_s = g.ego_x;
            double ego_ref_d = g.ego_y - road_center_y(g.ego_x, g.curve_start_x,
                                                       g.curve_length_m, g.curve_offset_m);
            memset(points, 0, sizeof(points));
            (void)project_to_reference_path(g.ego_x, g.ego_y, ego_ref_s, ego_ref_d);

            /* ── UTurnPlanner: 掉头时跳过 Frenet，直接生成自行车模型轨迹 ── */
            if (g.overtake_state == 3) {
                /* 车参单一事实源：场景 ego 块 → flowsim 广播 → 此处同步。
                 * 旧硬编码 2.8 与 physics 2.7 差 3.7% → 轨迹 yaw_rate 与实际
                 * 积分不一致 → 掉头轨迹与实车位姿漂移（2026-08 修复收口）。 */
                const double wheelbase = g.wheelbase;
                /* 掉头轨迹固定化：触发时生成一次，整个掉头期间复用。
                 * 旧实现每帧从当前 ego 状态重生成 → 车一旦偏离（v 归零/冲出
                 * 路面），新轨迹从偏离位置重新出发，目标永远跟着车跑 → 掉头
                 * 失败死循环（实测 y=11.05 路面外卡死 3 分钟）。固定轨迹让
                 * control 沿原始掉头弧执行，偏离会被拉回。 */
                bool cache_hit = (g.uturn_cache_n > 0) &&
                    (fabs(g.ego_x - g.uturn_cache_ego_x) < 40.0) &&
                    (fabs(g.ego_y - g.uturn_cache_ego_y) < 40.0);
                if (cache_hit) {
                    memcpy(points, g.uturn_cache, sizeof(TrajectoryPoint) * (size_t)g.uturn_cache_n);
                    n_pts = g.uturn_cache_n;
                    use_stitch = false;
                    goto publish_trajectory;
                }
                /* 计算前向可用空间：到路端/障碍物的距离，用于倒车腾挪决策 */
                double forward_space_m = 1e9;  /* 默认无限（无路端信息） */
                if (has_fresh_map_ref() && g.map_ref_count >= 2) {
                    /* 参考路径最后一点是路端（flowsim 前向采样在路端停止） */
                    double road_end_x = g.map_ref_x[g.map_ref_count - 1];
                    forward_space_m = road_end_x - g.ego_x;
                    if (forward_space_m < 0.0) forward_space_m = 0.0;
                }
                /* 前方静止障碍（施工区/停放车）同样占据掉头空间：只看路端会在
                 * 施工区前 1.3m 处误判 fwd=30m → 跳过 Phase 0 倒车腾挪直接
                 * 前进满舵 → 撞墙被 safety 拦停（2026-08-03 实测）。取
                 * min(路端, 本车道前方最近静止障碍距离-安全裕度)。 */
                for (int i = 0; i < g.kMaxObs; i++) {
                    /* 空槽 (0,0)：on_perception_obstacles 清零未用槽位 */
                    if (g.obs_x[i] == 0.0 && g.obs_y[i] == 0.0) continue;
                    if (fabs(g.obs_vx[i]) < 0.5 && fabs(g.obs_vy[i]) < 0.5 &&
                        g.obs_x[i] > g.ego_x &&
                        fabs(g.obs_y[i] - g.ego_y) < 3.0) {
                        double d = g.obs_x[i] - g.ego_x - 2.0; /* 2m 安全裕度 */
                        if (d < 0.0) d = 0.0;
                        if (d < forward_space_m) forward_space_m = d;
                    }
                }
                /* 512 点细生成 + 段感知下采样到 64：Phase 0 倒车 3.4s×20Hz
                 * 就吃掉 69 点，直接生成 64 点必截断满舵弧（kappa 全 0 →
                 * control 认不出机动，2026-08-03 demo5 实测）。 */
                static TrajectoryPoint uturn_big[512];
                int big_n = generate_uturn_trajectory(uturn_big, 512,
                                                  g.ego_x, g.ego_y, g.ego_heading,
                                                  g.ego_v, wheelbase,
                                                  forward_space_m);
                n_pts = downsample_uturn(uturn_big, big_n, points, 64);
                memcpy(g.uturn_cache, points, sizeof(TrajectoryPoint) * (size_t)n_pts);
                g.uturn_cache_n = n_pts;
                g.uturn_cache_ego_x = g.ego_x;
                g.uturn_cache_ego_y = g.ego_y;
                LOG_INFO("planning", "[UTURN] planner generated %d pts (ego_x=%.1f y=%.2f h=%.2f v=%.1f fwd=%.1fm) — cached, replay until exit",
                         n_pts, g.ego_x, g.ego_y, g.ego_heading, g.ego_v, forward_space_m);
                /* 跳过 stitch（掉头是全新轨迹，不与上帧拼接） */
                use_stitch = false;
                /* 跳过下方所有 Frenet 规划逻辑 */
                goto publish_trajectory;
            }

            double s_out[50], d_out[50], spd_out[50];

#ifdef HAVE_FRENET
            {
                /* map_ref 是 flowsim 发布的 route centerline 前视参考线时，
                 * Frenet 初始条件必须投影到该参考线本地弧长坐标系，不能再把全局
                 * ego_x 当 ego_s。否则右转/支路场景下 s 与参考线语义错位，control
                 * 只会忠实跟踪一条已经偏出路沿的坏轨迹。 */
                n_wp = frenet_plan(g.frenet,
                    ego_ref_s, ego_ref_d, g.ego_v,
                    command_speed,
                    s_out, d_out, spd_out, 50);
                if (n_wp < 3) {
                    /* Frenet 规划失败（参考路径未设/障碍物阻塞/求解失败）→ 简单路径兜底 */
                    static int frenet_fail_logged = 0;
                    if (!frenet_fail_logged) {
                        LOG_WARN("planning", "frenet_plan returned n_wp=%d (path_set=%d) — using fallback",
                                 n_wp, g.frenet ? 1 : 0);
                        frenet_fail_logged = 1;
                    }
                    double horizon = 50.0;
                    int n = 10;
                    for (int i = 0; i < n; i++) {
                        s_out[i] = ego_ref_s + horizon * (double)i / (double)(n - 1);
                        d_out[i] = ego_ref_d;  /* 保持当前横向位置 */
                        spd_out[i] = command_speed;
                    }
                    n_wp = n;
                }
            }
#else
            /* Fallback: 生成简单的车道保持 + 恒速轨迹 */
            {
                double horizon = 50.0;   /* 前方 50m */
                int n = 10;              /* 10 个路径点 */
                for (int i = 0; i < n; i++) {
                    s_out[i] = ego_ref_s + horizon * (double)i / (double)(n - 1);
                    d_out[i] = ego_ref_d;  /* 保持当前参考线横向位置 */
                    spd_out[i] = command_speed;
                }
                n_wp = n;
            }
#endif

            /* 变道时覆盖 d_out：Frenet 规划器只知收敛到参考线中心 (d=0)，
             * 不知目标车道偏移。behavior_planner 下发的 target_lane_idx 必须
             * 通过 d_out 显式插值到目标车道，否则车永远不变道。
             * 对 fallback 路径（d_out 恒为 ego_ref_d）同样适用。
             *
             * 渐变形态 = 圆弧 sagitta（2026-08 用户规范 + HTML 车辆实验室
             * 验证："左变道每一个方向盘固定就是一个圆弧，是圆的一部分，
             * 车头先进"）：d(s) = R·(1-cos(s/R))，固定曲率（固定方向盘）
             * 的圆的一段。车头沿弧线先转进目标车道，车身/屁股沿弧线跟随。
             * 与 tools/vehicle_lab.html 的操控行为一致。
             *
             * 数值稳健性（上次圆弧实现乱飞的修复）：
             *   - 变道纵向长度固定 L=50m，不依赖 target_ref_s 投影（投影
             *     在弯道/异常位置可能返回远点 → R 巨大 → cos 参数极小）。
             *   - D 取目标投影 d 与当前 d 之差（有界）。
             *   - 圆弧半径 R = (L²+D²)/(2|D|)，|D| 下限保护。 */
            if (g.has_behavior && n_wp > 2 &&
                (g.current_behavior.command == BEH_LEFT_CHANGE || g.current_behavior.command == BEH_RIGHT_CHANGE)) {
                double target_world_y = road_center_y(g.ego_x, g.curve_start_x,
                                                      g.curve_length_m, g.curve_offset_m)
                                      + g.target_lane_offset;
                double target_ref_s = 0, target_ref_d = 0;
                if (project_to_reference_path(g.ego_x, target_world_y, target_ref_s, target_ref_d)) {
                    const double D = target_ref_d - ego_ref_d;
                    const double L = 50.0;  /* 变道纵向长度固定（投影鲁棒） */
                    const double absD = std::fabs(D);
                    if (absD > 0.2 && absD < 8.0) {  /* 有效变道范围 */
                        const double R = (L * L + D * D) / (2.0 * absD);
                        for (int i = 0; i < n_wp; i++) {
                            const double t = (double)i / (double)(n_wp - 1);
                            const double s = t * L;
                            /* sagitta：d_out = ego_ref_d + D·frac，frac∈[0,1]。
                             * 圆弧凸高公式 frac = R/|D|·(1-cos(s/R))，恒正。
                             * 旧实现 st 里乘了 sign(D)——D 本身已带符号，右变道
                             * (D<0) 时 st<0 与 D 相乘得正 → 轨迹反向冲向 +y 侧，
                             * 车变道必冲出对侧路面（2026-08-03 demo 实测
                             * RIGHT_CHANGE 车冲到 y=+6.4 再反向回正）。 */
                            const double frac = (R / absD) *
                                              (1.0 - std::cos(s / R));
                            d_out[i] = ego_ref_d + D * frac;
                        }
                        /* 圆弧曲率（固定方向盘 = 固定 kappa）存入全局，
                         * 轨迹回填后填给变道段（control kappa 前馈用）。
                         * 符号与 D 一致：右变道(D<0)曲率负（向右弯）。 */
                        g.lane_change_kappa = (D >= 0.0) ? (1.0 / R) : -(1.0 / R);
                        g.lane_change_kappa_active = 1;
                    }
                }
            } else {
                g.lane_change_kappa_active = 0;
            }

            /* 行为规划下发的 command_speed 是硬约束，不是 Frenet 的优化目标。
             * Frenet 只决定横向路径（s, d），纵向速度全由 command_speed 确定。
             * 从当前速度线性过渡到目标速度，保证轨迹末点 100% 跟踪行为指令。 */
            if (n_wp > 2) {
                double v0 = g.ego_v;  /* 用车子实际速度，不是 Frenet 首点（可能已被设为目标速度） */
                for (int i = 0; i < n_wp; i++) {
                    double t = (double)i / (double)(n_wp - 1);
                    spd_out[i] = v0 * (1.0 - t) + command_speed * t;
                }
            }

            /* 车道保持：直接设目标车道中心，不再渐变混合。
             * 旧实现用线性渐变 d_out = d_out*(1-t) + target_lane_offset*t，
             * 0.5s 前视点只混合了 ~10% 的 target_lane_offset，Frenet 的
             * kd=1.0 代价又往 d=0（道路中心）拉 → 车逐渐偏离车道中心。
             * 修复：Frenet 设 kd=0（不再拉 d=0），此处巡航时直接锁定
             * target_lane_offset，控制层前视点拿到的就是精确车道中心。
             * 变道时 d_out 由上方 1668-1684 的变道逻辑覆盖，此处跳过。 */
            if (n_wp > 0) {
                bool in_lane_change = (g.has_behavior &&
                    (g.current_behavior.command == BEH_LEFT_CHANGE ||
                     g.current_behavior.command == BEH_RIGHT_CHANGE));
                if (!in_lane_change) {
                    for (int i = 0; i < n_wp; i++) {
                        d_out[i] = g.target_lane_offset;
                    }
                }
            }

            /* §9 轨迹拼接 */
            g.stitch_total_count++;
            if (g.prev_traj.point_count > 0 && g.prev_traj_stamp_us > 0) {
                /* 计算当前时刻在上一帧轨迹上的时间偏移 */
                uint64_t dt_us = clock_now_us() - g.prev_traj_stamp_us;
                if (dt_us < 500000) {  /* 只在上帧 0.5s 内有效 */
                    double pred_x, pred_y, pred_h, pred_v, pred_a;
                    if (traj_interpolate(&g.prev_traj, (uint32_t)dt_us,
                                          &pred_x, &pred_y, &pred_h, &pred_v, &pred_a)) {
                        /* 一致性检查 */
                        double pos_diff = hypot(pred_x - g.ego_x, pred_y - g.ego_y);
                        double spd_diff = fabs(pred_v - g.ego_v);
                        if (pos_diff < 2.0 && spd_diff < 3.0) {
                            use_stitch = true;
                        }
                    }
                }
            }

            /* ── 构建二进制 Trajectory 数组 ── */
            if (n_wp > 0) {
                /* 移除错误的 command_speed = max(command_speed, spd_out[0]) 逻辑。
                 * 原注释担心的"停车闭锁"(v=0→target=0→永远0)实际不成立：
                 *   - 红灯时 TL override 强制 command_speed=0
                 *   - 灯转绿后 TL override 不再触发（dx_tl<=0 或 state==green），
                 *     command_speed 自然回到巡航/跟车速度，不会被锁在0
                 *
                 * 而这行代码的实际危害是：FOLLOW 降速时，spd_out[0]≈当前车速(如15m/s)，
                 * 会把 behavior 下发的 lead_speed(如7m/s) 覆盖回去，导致 PID 永远追不到
                 * 目标速度，ego 以15m/s直冲7m/s前车→追尾。
                 *
                 * 加速度/减速度限制由 control 侧 PID 和 safety_control 负责，
                 * planning 不应在此处"帮倒忙"。 */
                if (command_speed > g.cfg_max_speed) command_speed = g.cfg_max_speed;
                if (command_speed < 0.0) command_speed = 0.0;

                /* ── 红绿灯速度强制 override ──
                 * Frenet 轨迹的第一个点速度 spd_out[0] ≈ 当前车速，control 节点
                 * 只读 target_speed 做速度 PID，不读 path 内的减速曲线。因此必须
                 * 在此显式置 0，否则 control 一直追当前车速，直接闯红灯。 */
                if (g.has_traffic_lights) {
                    double v = g.ego_v;
                    if (v < 0.0) v = 0.0;
                    double brake_dist = v * v / 8.0 + 3.0;
                    if (brake_dist < 5.0) brake_dist = 5.0;
                    for (int ti = 0; ti < g.tl_count; ti++) {
                        if (g.tl_state[ti] == 0) continue;  /* 绿灯 */
                        double dx_tl = g.tl_x[ti] - g.ego_x;
                        if (dx_tl <= 0.0 || dx_tl > 60.0) continue;
                        if (dx_tl > brake_dist + 20.0) continue;  /* 还很远（fusion 滞后补 20m 余量） */
                        /* 刹停范围内 → 强制停车
                         * 用 brake_dist + 20m 大余量补 fusion 定位滞后，
                         * 见 control_node.cpp boost_target 注释。 */
                        command_speed = 0.0;
                        /* 2026-07-31 修复：spd_out 在此前已按旧 command_speed 生成，
                         * 只改 command_speed 进不了轨迹（points[].v 读的是 spd_out），
                         * control 拿到的还是巡航速度 → 直接闯红灯（实测红灯前
                         * 只从 12 缓到 6.2 就冲过去）。必须同步重建 spd_out：
                         * 当前速度 → 0 的减速斜坡，轨迹末点归零 control 才刹停。 */
                        if (n_wp > 2) {
                            double v0 = g.ego_v;
                            if (v0 < 0.0) v0 = 0.0;
                            for (int i = 0; i < n_wp; i++) {
                                double t = (double)i / (double)(n_wp - 1);
                                spd_out[i] = v0 * (1.0 - t);  /* command_speed=0 → 斜坡到 0 */
                            }
                        }
                        break;
                    }
                }
                for (int i = 0; i < n_wp && n_pts < 64; i++) {
                    points[n_pts].t_rel_us = (uint32_t)(i * 100000);  /* 100ms per point (10Hz trajectory), in µs */
                    points[n_pts].x = 0.0f;   /* 占位，下面 Frenet→Cartesian 回填 */
                    points[n_pts].y = 0.0f;
                    points[n_pts].s = (float)s_out[i];
                    points[n_pts].l = (float)d_out[i];
                    points[n_pts].heading = 0.0f;  /* 占位 */
                    points[n_pts].kappa = 0.0f;    /* 占位 */
                    points[n_pts].v = (float)spd_out[i];
                    points[n_pts].a = 0.0f;
                    points[n_pts].jerk = 0.0f;
                    n_pts++;
                }
                /* ── Frenet→Cartesian 回填：用 (s, d) 沿参考路径映射回全局 x/y/heading/kappa ── */
                for (int i = 0; i < n_pts; i++) {
                    double cx, cy, ch, ck;
                    if (frenet_to_cartesian((double)points[i].s, (double)points[i].l,
                                             cx, cy, ch, ck)) {
                        points[i].x = (float)cx;
                        points[i].y = (float)cy;
                        points[i].heading = (float)ch;
                        points[i].kappa = (float)ck;
                    }
                }
                /* 变道轨迹 heading 修正（2026-08 根因修复）：
                 * frenet_to_cartesian 的 heading = 参考线切线，完全忽略 d 渐变
                 * → 变道轨迹 heading 恒 0 → control 的 psi_des 基准永远直行
                 * → 车头不转、横向硬拉 → "车屁股平移"（实测 lat_err 恒 1m、
                 * 车身偏转仅 2.5°）。
                 * 用相邻轨迹点世界坐标差分算实际路径切线（含 d 渐变）：
                 * 变道段车头朝目标车道偏转（斜插），收敛段回正。 */
                for (int i = 0; i + 1 < n_pts; i++) {
                    double dx = (double)points[i + 1].x - (double)points[i].x;
                    double dy = (double)points[i + 1].y - (double)points[i].y;
                    if (dx * dx + dy * dy > 1e-9) {
                        points[i].heading = (float)atan2(dy, dx);
                    }
                }
                if (n_pts > 1) points[n_pts - 1].heading = points[n_pts - 2].heading;
                /* 变道段 kappa = 圆弧固定曲率（固定方向盘，2026-08）：
                 * control 的 kappa 前馈（ff_term = wb·kappa）让车沿圆弧走，
                 * 而不是靠反馈慢慢追上。frenet_to_cartesian 的 kappa=0
                 * （参考线直线），圆弧曲率在这里显式填入。 */
                if (g.lane_change_kappa_active) {
                    for (int i = 0; i < n_pts; i++) {
                        points[i].kappa = (float)g.lane_change_kappa;
                    }
                }
            } else {
                /* 规划失败 → 停车（Apollo 原则：不能规划就停）
                 * 用 ego 当前位置 + v=0，control PID 会匀减速到 0。
                 * 不用 (0,0) ——否则 Stanley 疯狂转向。
                 * 不用 command_speed——planning 失败时不应继续按目标速度行驶。 */
                n_pts = 1;
                points[0].t_rel_us = 0;
                points[0].x = (float)g.ego_x;
                points[0].y = (float)g.ego_y;
                points[0].heading = (float)g.ego_heading;
                points[0].v = 0.0f;  /* 停车 */
            }

publish_trajectory:
            /* ── 发布二进制 Trajectory ── */
            Trajectory traj;
            memset(&traj, 0, sizeof(traj));
            traj.seq = g.plan_count;
            traj.stamp_us = clock_now_us();
            traj.ref_line_id = 0;  /* No reference line yet */
            traj.point_count = (uint16_t)n_pts;
            traj.planner_state = 2; /* QP */
            traj.is_stitched = use_stitch ? 1 : 0;

            for (int i = 0; i < n_pts && i < 64; i++) {
                traj.points[i] = points[i];
            }

            /* §8.5 可行性检查 */
            traj.valid = 1;  /* 默认有效 */
            if (n_wp > 1) {
                for (int i = 0; i < n_pts && i < 64; i++) {
                    /* kappa 超限检查 */
                    if (fabs((double)traj.points[i].kappa) > 0.25) {  /* ~14deg steer equivalent */
                        traj.valid = 0;
                        break;
                    }
                    /* 横向加速度检查 */
                    double a_lat = (double)traj.points[i].v * (double)traj.points[i].v * fabs((double)traj.points[i].kappa);
                    if (a_lat > 5.0) {  /* 5 m/s² max lateral accel */
                        traj.valid = 0;
                        break;
                    }
                    /* 速度合法性 */
                    if ((double)traj.points[i].v < -0.5 || (double)traj.points[i].v > g.cfg_max_speed * 1.1) {
                        traj.valid = 0;
                        break;
                    }
                }
            }
            /* #4: 全零退化轨迹检测 — 所有点 x==0 && y==0 时视为未初始化 */
            if (traj.valid && n_pts > 1) {
                int all_zero = 1;
                for (int i = 0; i < n_pts; i++) {
                    if (fabs((double)traj.points[i].x) > 0.001 ||
                        fabs((double)traj.points[i].y) > 0.001) {
                        all_zero = 0;
                        break;
                    }
                }
                if (all_zero) {
                    traj.valid = 0;
                    LOG_WARN("planning", "#%d all-zero trajectory (x/y all ==0) — marking invalid",
                             g.plan_count);
                }
            }
            if (!traj.valid) {
                LOG_WARN("planning", "#%d trajectory FAILED feasibility check — publishing invalid",
                         g.plan_count);
            }

            uint8_t buf[4096];
            size_t ser_len = sizeof(buf);
            int rc = Trajectory_serialize(&traj, buf, &ser_len);
            if (rc == 0) {
                transport_publish(transport_, TOPIC_PLANNING_TRAJECTORY, buf, (uint32_t)ser_len);
            }
            /* DEBUG: 临时打印发布的轨迹首点速度 */
            if (g.plan_count >= 760 && g.plan_count <= 810) {
                LOG_WARN("planning", "[DBG pub] #%d cmd_speed=%.2f pts[0].v=%.2f spd_out[0]=%.2f n_wp=%d beh_cmd=%d",
                         g.plan_count, command_speed, (double)points[0].v, spd_out[0], n_wp,
                         g.has_behavior ? (int)g.current_behavior.command : -1);
            }

            /* 发布 planning/debug（全链路横向调试，每10帧≈2Hz） */
            if (g.plan_count % 10 == 0) {
                double dbg_rc_y = road_center_y(g.ego_x, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
                double dbg_ref_s = g.ego_x;
                double dbg_ego_d = g.ego_y - dbg_rc_y;
                double dbg_ref_x = g.ego_x;
                double dbg_ref_y = dbg_rc_y;
                double dbg_ref_heading = 0.0;
                (void)project_to_reference_path(g.ego_x, g.ego_y, dbg_ref_s, dbg_ego_d,
                                                &dbg_ref_x, &dbg_ref_y, &dbg_ref_heading);
                dbg_rc_y = dbg_ref_y;
                cJSON* dbg = cJSON_CreateObject();
                char mode_buf[32] = {0};
                statem_format_hierarchical(statem_current(&g.mode_sm), mode_buf, sizeof(mode_buf));
                cJSON_AddNumberToObject(dbg, "seq", g.plan_count);
                cJSON_AddNumberToObject(dbg, "ego_x", g.ego_x);
                cJSON_AddNumberToObject(dbg, "ego_y", g.ego_y);
                cJSON_AddNumberToObject(dbg, "ego_v", g.ego_v);
                cJSON_AddStringToObject(dbg, "driver_mode", mode_buf[0] ? mode_buf : "NA:READY");
                cJSON_AddNumberToObject(dbg, "route_lane", g.route_target_lane);
                cJSON_AddNumberToObject(dbg, "route_count", g.route_count);
                cJSON_AddNumberToObject(dbg, "route_next_idx", g.route_next_idx);
                cJSON_AddNumberToObject(dbg, "road_center_y", dbg_rc_y);
                cJSON_AddNumberToObject(dbg, "ego_d", dbg_ego_d);
                cJSON_AddNumberToObject(dbg, "ref_s", dbg_ref_s);
                cJSON_AddNumberToObject(dbg, "ref_x", dbg_ref_x);
                cJSON_AddNumberToObject(dbg, "ref_y", dbg_ref_y);
                cJSON_AddNumberToObject(dbg, "ref_heading", dbg_ref_heading);
                cJSON_AddNumberToObject(dbg, "target_lane_offset", g.target_lane_offset);
                cJSON_AddNumberToObject(dbg, "command_speed", command_speed);
                cJSON_AddNumberToObject(dbg, "n_wp", n_wp);
                cJSON_AddNumberToObject(dbg, "traj_valid", traj.valid ? 1.0 : 0.0);
                cJSON_AddNumberToObject(dbg, "n_lanes", g.lane_count);
                cJSON_AddNumberToObject(dbg, "lane_width", g.lane_width);
                if (g.has_behavior) {
                    cJSON_AddNumberToObject(dbg, "beh_cmd", (double)g.current_behavior.command);
                    cJSON_AddNumberToObject(dbg, "beh_target_lane", (double)g.current_behavior.target_lane_idx);
                    cJSON_AddNumberToObject(dbg, "beh_target_speed", (double)g.current_behavior.target_speed);
                }
                /* 轨迹前视点和末点的 d 值（横向偏移），用于验证变道轨迹 */
                if (n_wp > 0) {
                    int la_idx = n_wp / 2;  /* 前视点（轨迹中点） */
                    cJSON_AddNumberToObject(dbg, "d_start", d_out[0]);
                    cJSON_AddNumberToObject(dbg, "d_lookahead", d_out[la_idx]);
                    cJSON_AddNumberToObject(dbg, "d_end", d_out[n_wp - 1]);
                    cJSON_AddNumberToObject(dbg, "v_end", spd_out[n_wp - 1]);
                }
                char* dbg_s = cJSON_PrintUnformatted(dbg);
                transport_publish(transport_, TOPIC_PLANNING_DEBUG,
                                  (const uint8_t*)dbg_s, (uint32_t)strlen(dbg_s) + 1);
                free(dbg_s);
                cJSON_Delete(dbg);
            }
            /* 缓存轨迹用于拼接 */
            g.prev_traj_stamp_us = clock_now_us();
            memcpy(&g.prev_traj, &traj, sizeof(traj));
            g.plan_count++;

            /* ── 发布参考线（经 QP 平滑） ── */
            {
                ReferenceLinePoint ref_pts[101];
                int n_ref = 101;
                double ref_start = g.ego_x;
                double ref_len = g.cfg_ref_path_length;

                /* 收集原始路径点 */
                double raw_x[101], raw_y[101];
                for (int i = 0; i < n_ref; i++) {
                    double px = ref_start + (double)i * (ref_len / (double)(n_ref - 1));
                    double py = road_center_y(px, g.curve_start_x, g.curve_length_m, g.curve_offset_m);
                    raw_x[i] = px;
                    raw_y[i] = py;
                }

                /* QP 平滑 */
                double sm_x[101], sm_y[101];
                double w_raw = 10.0;  /* 原始点信任度，越大越贴合原始线型 */
                pjqp_smooth_2d(sm_x, sm_y, raw_x, raw_y, w_raw, n_ref);

                /* 填充参考线点，计算 heading 和曲率 */
                for (int i = 0; i < n_ref; i++) {
                    ref_pts[i].s = (float)(sm_x[i] - ref_start);
                    ref_pts[i].x = (float)sm_x[i];
                    ref_pts[i].y = (float)sm_y[i];

                    /* heading = atan2(dy, dx) */
                    if (i > 0 && i < n_ref - 1) {
                        double dx = sm_x[i+1] - sm_x[i-1];
                        double dy = sm_y[i+1] - sm_y[i-1];
                        ref_pts[i].theta = (float)atan2(dy, dx);
                    } else if (i == 0) {
                        double dx = sm_x[1] - sm_x[0];
                        double dy = sm_y[1] - sm_y[0];
                        ref_pts[i].theta = (float)atan2(dy, dx);
                    } else {
                        double dx = sm_x[n_ref-1] - sm_x[n_ref-2];
                        double dy = sm_y[n_ref-1] - sm_y[n_ref-2];
                        ref_pts[i].theta = (float)atan2(dy, dx);
                    }

                    /* kappa = d(theta)/ds, 用三点中心差分 */
                    if (i > 0 && i < n_ref - 1) {
                        double ds = ref_pts[i+1].s - ref_pts[i-1].s;
                        double dtheta = (double)(ref_pts[i+1].theta - ref_pts[i-1].theta);
                        /* 角度归一化 */
                        while (dtheta >  M_PI) dtheta -= 2.0 * M_PI;
                        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
                        ref_pts[i].kappa = (float)(dtheta / ds);
                    } else {
                        ref_pts[i].kappa = 0.0f;
                    }

                    ref_pts[i].dkappa = 0.0f;
                    ref_pts[i].left_bound = 10.0f;
                    ref_pts[i].right_bound = 10.0f;
                    ref_pts[i].speed_limit = (float)g.cfg_max_speed;
                }
                uint8_t ref_buf[4096];
                uint16_t ref_count = (uint16_t)n_ref;
                uint32_t ref_len_bytes = 0;
                memcpy(ref_buf, &ref_count, sizeof(ref_count));
                memcpy(ref_buf + 2, ref_pts, n_ref * sizeof(ReferenceLinePoint));
                ref_len_bytes = 2 + n_ref * sizeof(ReferenceLinePoint);
                transport_publish(transport_, TOPIC_PLANNING_REFERENCE_LINE, ref_buf, ref_len_bytes);
            }

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

/* ── TaskBase 包装器（宏生成） — 必须在 planning_init 前展开 ─────── */
EXPORT_COROUTINE_TASK(PlanningTask, planning)

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = {
    TOPIC_FUSION_LOCALIZATION,
    TOPIC_PERCEPTION_OBSTACLES,
    TOPIC_ROAD_GEOMETRY,
    TOPIC_ROAD_REF_PATH,
    TOPIC_ROAD_TRAFFIC_LIGHTS,
    TOPIC_SCENE_FRAME,
    TOPIC_PLANNING_BEHAVIOR,
    TOPIC_NAVIGATION_PATH,
    nullptr
};
static const char* s_outputs[] = { TOPIC_PLANNING_TRAJECTORY, nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾 */

static int planning_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    /* 清零并重新初始化 */
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.has_fusion   = 0;
    g.has_vstate   = 0;

    g.mode_last_check_us = 0;
    g.last_fusion_us     = 0;
    g.highway_ready      = 0;
    g.highway_speed_timer = 0.0;

    g.route_count        = 0;
    g.route_next_idx     = 0;
    g.route_target_lane  = -1;  /* N 车道模型：-1=无目标 */
    g.route_target_speed = -1.0;  /* -1=未设置 */
    /* NOA Phase 3.2/3.3: 路线步骤类型默认 lane_change，branch_id 复位 */
    g.route_type         = ROUTE_LANE_CHANGE;
    g.current_branch_id  = -1;

    g.plan_count  = 0;

    /* §9 轨迹拼接初始化 */
    g.prev_traj.point_count = 0;
    g.prev_traj_stamp_us = 0;
    g.stitch_skip_count = 0;
    g.stitch_total_count = 0;

    /* 行为规划状态初始化 */
    memset(&g.current_behavior, 0, sizeof(g.current_behavior));
    g.has_behavior = 0;
    g.overtake_state = 0;
    g.committed_lane_idx = 0;
    g.target_lane_offset = 0.0;

    g.ego_x = g.ego_y = g.ego_v = g.ego_heading = 0.0;
    for (int i = 0; i < g.kMaxObs; i++) { 
        g.obs_x[i] = g.obs_y[i] = g.obs_vx[i] = g.obs_vy[i] = 0.0;
        g.obs_lane_id[i] = -1;
        g.obs_type[i] = 0;
        g.obs_confidence[i] = 0.0f;
    }

    g.curve_start_x   = 0.0;
    g.curve_length_m  = 0.0;
    g.curve_offset_m  = 0.0;
    g.ref_path_start_x = 0.0;
    g.map_ref_count = 0;
    g.last_map_ref_us = 0;

    /* 默认参数 */
    g.cfg_target_speed      = 15.0;
    g.cfg_max_speed         = 20.0;
    g.cfg_max_accel         = 4.0;
    g.cfg_ref_path_length   = 5000.0;
    g.cfg_highway_speed_mps = 13.0;  /* 未提供 highway_speed_mps 参数时的兜底默认值，
                                        需低于当前场景实际巡航速度才能触发 NP 升级；
                                        pipeline.json 会显式覆盖为更贴近实际场景的值。 */

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
            cJSON_Delete(root);
        }
    }

    g.target_speed = g.cfg_target_speed;
    g.route_target_speed = -1.0;  /* -1=未设置 */

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    g.frenet = frenet_create(g.cfg_max_speed, g.cfg_max_accel);
    if (!g.frenet) {
        LOG_ERROR("planning", "frenet_create failed");
        return -1;
    }
#else
    g.frenet = nullptr;
    LOG_WARN("planning", "════════════════════════════════════════════════════════");
    LOG_WARN("planning", "构建未启用 Frenet/Eigen — 规划已降级为「恒速车道保持」");
    LOG_WARN("planning", "  · 无横向轨迹优化：不做超车/绕障/变道路径规划");
    LOG_WARN("planning", "  · 障碍物仅由行为层减速处理，规划层不主动避让");
    LOG_WARN("planning", "  如需完整规划：装 Eigen 并以 -DHAVE_FRENET 重新构建");
    LOG_WARN("planning", "════════════════════════════════════════════════════════");
#endif

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION, on_fusion, nullptr);
    transport_subscribe(transport, TOPIC_VEHICLE_STATE, on_vehicle_state, nullptr);
    transport_subscribe(transport, TOPIC_PERCEPTION_OBSTACLES, on_perception_obstacles, nullptr);
    transport_subscribe(transport, TOPIC_ROAD_GEOMETRY, on_road_geometry, nullptr);
    transport_subscribe(transport, TOPIC_ROAD_REF_PATH, on_road_ref_path, nullptr);
    transport_subscribe(transport, TOPIC_ROAD_TRAFFIC_LIGHTS, on_traffic_lights, nullptr);
    transport_subscribe(transport, TOPIC_SCENE_FRAME, on_scene_frame, nullptr);
    transport_subscribe(transport, TOPIC_PLANNING_BEHAVIOR, on_planning_behavior, nullptr);
    transport_subscribe(transport, TOPIC_NAVIGATION_PATH, on_navigation_path, nullptr);
    transport_advertise(transport, TOPIC_PLANNING_TRAJECTORY, 0x3A7B1C2Du);
    transport_advertise(transport, TOPIC_PLANNING_DEBUG, 0u);  /* JSON text */

    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PERCEPTION_OBSTACLES, OBSTACLELIST_TYPE_ID,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_ROAD_GEOMETRY, 0x80AD5C12u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_ROAD_REF_PATH, 0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_ROAD_TRAFFIC_LIGHTS, 0x7E5C0FFEu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_SCENE_FRAME,         0x5CF12A60u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PLANNING_BEHAVIOR,   0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_NAVIGATION_PATH,     0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_PLANNING_TRAJECTORY, 0x3A7B1C2Du,
                        CAP_PUBLISHER, 10.0);
    discovery_advertise(discovery, TOPIC_PLANNING_DEBUG, 0u, CAP_PUBLISHER, 2.0);

    /* 初始化反射式状态机 */
    statem_init(&g.sm, nullptr, SM_STATE_INITIALIZED, "planning");
    statem_send_event(&g.sm, SM_EVENT_START, nullptr);

    /* 初始化驾驶模式状态机（NA/ACC/CP/NP/NOA）：真实条件驱动升降级 */
    statem_init(&g.mode_sm, SM_TABLE_MODE_SWITCHING, SM_MODE_NA, "driving_mode");
    statem_set_guard(&g.mode_sm, mode_transition_guard);

    /* 创建 TaskBase 包装器（托管模式） */
    TaskConfig tcfg = {};
    snprintf(tcfg.name, sizeof(tcfg.name), "planning");
    tcfg.priority = TASK_PRIORITY_NORMAL;
    g.task_wrapper = planning_create(&tcfg, bus);
    if (!g.task_wrapper) {
        LOG_ERROR("planning", "planning_create failed");
        return -1;
    }
    g.task_wrapper->impl->set_params(transport);
    s_plugin.taskbase = planning_get_base(g.task_wrapper);

    LOG_INFO("planning", "initialized (FlowCoro, target=%.0f m/s, max=%.0f m/s)",
             g.cfg_target_speed, g.cfg_max_speed);
    return 0;
}

static int planning_start(void) {
    if (!g.task_wrapper) return -1;
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) LOG_WARN("planning", "node_start_managed failed: %d", rc);
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    LOG_INFO("planning", "started (managed mode) [state=%s]", statem_state_name(&g.sm, g.sm.current));
    return 0;
}

static void planning_stop(void) {
    if (g.task_wrapper) {
        planning_stop(&g.task_wrapper->base);
    }
}

static void planning_cleanup(void) {
    if (g.task_wrapper) {
        planning_destroy(g.task_wrapper);
        g.task_wrapper = nullptr;
    }
    s_plugin.taskbase = nullptr;
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
    nullptr,  /* taskbase: 在 init() 中通过 planning_create 设置 */
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
