/**
 * npc_ai.h — NPC 驾驶行为（IDM 跟车 + 状态机 + 行人运动）
 *
 * 设计文档 §4：每个 NPC tick
 *   1. 找同车道最近前车
 *   2. 计算 IDM 期望速度
 *   3. 转成 throttle/brake
 *   4. 调 step_bicycle 积分
 *
 * 状态机覆盖：
 *   Cruise    → 匀速巡航（target_vx）
 *   Follow    → IDM 跟车
 *   Stop      → 停车（target_vx=0）
 *   StopForTL → 红灯前减速停车
 *   Yield     → 让行（汇入时减速）
 *
 * 行人 AI（来自 sim_world_node.c obstacles_tick）：
 *   - 横穿道路：到达对侧后等待 3s 反向
 *   - 路边行走：直线运动
 */

#ifndef FLOWSIM_NPC_AI_H
#define FLOWSIM_NPC_AI_H

#include "entity.h"

namespace flowsim {

struct FlowRoadNetwork;  // 前向声明，避免头文件循环依赖
class  Route;            // 中央有序 route（见 route.h）

/** NPC AI 配置参数 */
struct NpcAiConfig {
    double lane_width{3.5};         /**< 车道宽度，用于横向同车道判断 */
    double same_lane_tol{2.0};      /**< 横向同车道容差（m） */
    double look_ahead{80.0};        /**< 前车搜索距离 (m) */
    /* D4 IDM 参数调优：原 safe_gap_base=3 / time=1.0 / decel=4.0 太激进，
     * 5m/s 时 safe_gap=8m，10m/s 时 safe_gap=13m。现在 base=5 / time=1.5 / decel=3.0
     * → 5m/s 时 safe_gap=12.5m，10m/s 时 safe_gap=20m，跟车更柔和、有反应时间，
     * 不再"前车一动后车跟到死停"。 */
    double idm_safe_gap_base{5.0};  /**< IDM 静态安全间距 (m) */
    double idm_safe_gap_time{1.5};  /**< IDM 时间头way (s) → safe_gap = base + v*time */
    double accel_rate{1.5};         /**< 平稳加速率 (m/s²)，原 2.0 改 1.5 起步更柔 */
    double brake_rate{3.5};         /**< 平稳减速率 (m/s²)，原 4.0 改 3.5 避免硬刹 */
    double follow_decel_factor{3.0};/**< IDM 跟车减速强度系数，原 4.0 改 3.0 */
    double ped_boundary{7.8};       /**< 行人横穿边界（路宽，m） */
    double ped_wait_time{3.0};      /**< 行人到边后等待时间 (s) */

    /* ── CutIn 加塞状态机（横向 PID 跨实线变道，收费站排队场景）──
     * 设计文档 Phase 3.3：Cruise → 接近收费站 → 当前通道 != 目标 → CutIn
     *   → 横向 PID 跨实线变道，vx 减速 2 m/s
     *   → 到达目标通道 |offset - target_offset| < threshold → Cruise
     * 跨实线时 bypass lane_change_safe 检查（避免被前后车阻挡无法变道）。
     * PID 参数复用 control_node.cpp 调校经验：Kp=1.5（响应足够快），
     * Ki=0.05（小积分消除稳态误差），Kd=0.4（阻尼抑制超调）。 */
    double cutin_pid_kp{1.5};           /**< CutIn 横向 PID 比例系数 */
    double cutin_pid_ki{0.05};          /**< CutIn 横向 PID 积分系数 */
    double cutin_pid_kd{0.4};           /**< CutIn 横向 PID 微分系数 */
    double cutin_max_lateral_speed{3.0};/**< CutIn 横向最大速度（m/s），限幅防过冲 */
    double cutin_completion_threshold{0.15}; /**< CutIn 完成判定阈值：|err|<此值 → 回 Cruise */
    double cutin_longitudinal_decel{2.0};    /**< CutIn 时纵向速度减幅（m/s），放缓避免冲撞 */

    /* ── MOBIL 变道决策参数 ──
     * lane_change_safe 只检查"目标车道有无车"，不判断"变道是否有益"。
     * MOBIL 代价函数：gain = a'_c - a_c + p * (a'_n - a_n + a'_o - a_o)
     *   a_c  = 当前车道 IDM 加速度
     *   a'_c = 目标车道 IDM 加速度
     *   a_n  = 新跟随者（目标车道后车）当前加速度
     *   a'_n = 新跟随者变道后加速度
     *   a_o  = 旧跟随者（当前车道后车）当前加速度
     *   a'_o = 旧跟随者变道后加速度
     *   p    = 礼貌因子（0=纯利己，1=考虑他人）
     * 安全约束：a'_n > -b_safe（新跟随者不会被迫急刹） */
    double mobil_politeness{0.5};       /**< MOBIL 礼貌因子 [0,1] */
    double mobil_safe_brake{4.5};       /**< MOBIL 安全减速度阈值 (m/s²)，a'_n 不得低于此值 */
    double mobil_gain_threshold{0.5};   /**< MOBIL 增益阈值 (m/s²)，gain>此值才变道（原 0.2 太频繁） */
    double mobil_back_look{20.0};       /**< 后向搜索距离 (m)，加大让更多后车参与评估 */
    double mobil_lane_change_cooldown{8.0}; /**< 变道冷却时间 (s)，原 4s 太短导致频繁变道 */
};

/* ── 统一状态机：转移请求事件 ──
 * 外部系统通过 npc_request_state() 请求状态转移，不得直接写 npc.state。
 * 转移函数验证合法性后原子更新 state + 相关字段（PID 状态、计时器等）。 */
enum class NpcEvent : uint8_t {
    None,
    LeadFound,        /**< 同车道前方有车 → Follow */
    LeadLost,         /**< 前车消失 → Cruise */
    TL_Red,           /**< 前方红灯/黄灯 → StopForTL */
    TL_Green,         /**< 绿灯/通过路口 → Cruise */
    MobilChange,      /**< MOBIL 评估通过 → LaneChange */
    ChoreoCutIn,      /**< 编舞触发加塞 → CutIn */
    ChoreoOvertake,   /**< 编舞触发超车 → Cruise（保持传送后位置） */
    ScriptOverride,   /**< 场景脚本 override → CutIn/Cruise */
    Collision,        /**< 碰撞 → Stopped */
    Recycle,          /**< 路网末端回收 → Cruise（重置全部状态） */
};

/* 未设置的字段用此哨兵值（不要用 NAN，其依赖 <cmath> 头文件） */
static constexpr double NPC_REQ_UNSET = -1e30;

struct NpcTransitionRequest {
    NpcEvent event{NpcEvent::None};
    double target_offset{NPC_REQ_UNSET};  /**< 目标横向偏移 (LaneChange/CutIn) */
    double target_vx{NPC_REQ_UNSET};      /**< 目标速度 (choreo/script) */
    double vx{NPC_REQ_UNSET}, vy{NPC_REQ_UNSET};  /**< 速度覆盖 */
    double x{NPC_REQ_UNSET}, y{NPC_REQ_UNSET};    /**< 位置覆盖 */
    double heading{NPC_REQ_UNSET};        /**< 航向覆盖 */
};

/**
 * 统一状态转移入口（唯一合法修改 npc.state 的函数）。
 * @return true 转移被接受，false 被拒绝（调用者应 LOG_WARN）。
 */
bool npc_request_state(Entity& npc, const NpcTransitionRequest& req,
                       const NpcAiConfig& cfg);

/**
 * 单步 NPC 车辆 AI（每帧 tick）。
 * 内部按 npc.state 分发纵向+横向行为；状态内自行调用 npc_request_state
 * 完成自动转移（如找到前车 → Follow，CutIn 完成 → Cruise 等）。
 *
 * 更新 npc.throttle/brake/state/lead_id/follow_gap/位置。
 */
void step_npc_vehicle(Entity& npc, const EntityPool& pool,
                      double dt, const NpcAiConfig& cfg,
                      FlowRoadNetwork* roads = nullptr,
                      const Route* route = nullptr,
                      double ego_route_s = 0.0);

/**
 * 初始化 NPC 的 route 跟随状态（spawn 后调用一次）。
 * 依据已填好的 npc.road_id / npc.s 定位到 route 累计 s。
 * @param dir 行驶方向：<0 对向来车(route_dir=-1)，否则顺行(+1)。
 *            若 npc.road_id 不在 route 上，置 route_dir=0（该 NPC 走旧逻辑）。
 */
void npc_init_route(Entity& npc, const Route& route, int dir);

/**
 * 单步行人 AI。
 * 行为：按 vx/vy 匀速横穿，到达 |y| >= ped_boundary 后等待 ped_wait_time 反向。
 * 沿车道行走的行人（vy=0）保持直线运动。
 */
void step_npc_pedestrian(Entity& ped, double dt, const NpcAiConfig& cfg);

}  // namespace flowsim

#endif  // FLOWSIM_NPC_AI_H
