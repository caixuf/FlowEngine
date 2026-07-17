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

/** NPC AI 配置参数 */
struct NpcAiConfig {
    double lane_width{3.5};         /**< 车道宽度，用于横向同车道判断 */
    double same_lane_tol{2.0};      /**< 横向同车道容差（m） */
    double look_ahead{80.0};        /**< 前车搜索距离 (m) */
    double idm_safe_gap_base{3.0};  /**< IDM 静态安全间距 (m) */
    double idm_safe_gap_time{1.0};  /**< IDM 时间头way (s) → safe_gap = base + v*time */
    double accel_rate{2.0};         /**< 平稳加速率 (m/s²) */
    double brake_rate{4.0};         /**< 平稳减速率 (m/s²) */
    double follow_decel_factor{4.0};/**< IDM 跟车减速强度系数 */
    double ped_boundary{7.8};       /**< 行人横穿边界（路宽，m） */
    double ped_wait_time{3.0};      /**< 行人到边后等待时间 (s) */
};

/**
 * 单步 NPC 车辆 AI。
 * @param npc       NPC 实体（必须 is_npc_vehicle()）
 * @param pool      实体池（用于找前车）
 * @param dt        时间步长 (s)
 * @param cfg       AI 参数
 * @param roads     可选，路网（用于更新 Frenet 坐标；nullptr 跳过）
 *
 * 更新 npc.throttle/brake/steer/ai_state/lead_id/follow_gap，
 * 并调 step_bicycle 推进位置。steer 默认 0（直行），车道保持由
 * 外部横向控制负责（与 ego 一致）。
 *
 * 注：roads 为非 const 指针，因 world_to_frenet 会修改 esmini position
 * handle 的内部状态（路网本身不变，但 position 是可变的查询游标）。
 */
void step_npc_vehicle(Entity& npc, const EntityPool& pool,
                      double dt, const NpcAiConfig& cfg,
                      FlowRoadNetwork* roads = nullptr);

/**
 * 单步行人 AI。
 * 行为：按 vx/vy 匀速横穿，到达 |y| >= ped_boundary 后等待 ped_wait_time 反向。
 * 沿车道行走的行人（vy=0）保持直线运动。
 */
void step_npc_pedestrian(Entity& ped, double dt, const NpcAiConfig& cfg);

}  // namespace flowsim

#endif  // FLOWSIM_NPC_AI_H
