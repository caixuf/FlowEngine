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
 * @param npc          NPC 实体（必须 is_npc_vehicle()）
 * @param pool         实体池（用于找前车）
 * @param dt           时间步长 (s)
 * @param cfg          AI 参数
 * @param roads        可选，路网（Frenet↔World；nullptr 跳过）
 * @param route        可选，中央 route。提供且 npc.route_dir!=0 时走「沿车道
 *                     Frenet 推进」路径：NPC 严格贴道路几何行驶、过弯/爬匝道
 *                     自动跟随、到 route 末尾回收到 ego 附近。为空则退回旧的
 *                     世界系直线积分（steer=0），保证 esmini 缺失时不退化。
 * @param ego_route_s  ego 在 route 上的累计 s（回收 NPC 时用来放到 ego 附近）
 *
 * 更新 npc.throttle/brake/ai_state/lead_id/follow_gap/位置。
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
