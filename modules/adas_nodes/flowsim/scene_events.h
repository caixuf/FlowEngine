/**
 * scene_events.h — 场景事件调度（红绿灯/ETC/停止线）
 *
 * 设计文档 §5：
 *   - 红绿灯：按 green/yellow/red 周期循环，ego 接近红灯时 NPC 减速停车
 *   - ETC 闸门：ego 接近时关闭，到近距抬杆，通过后放下
 *   - 停止线：虚拟触发器，ego 越过时触发事件
 *
 * 事件实体在 EntityPool 里以 EntityType::TrafficLight/ETCGate/StopLine 表示，
 * 场景加载时创建。本模块负责推进它们的相位状态，并让 NPC AI 响应。
 */

#ifndef FLOWSIM_SCENE_EVENTS_H
#define FLOWSIM_SCENE_EVENTS_H

#include "entity.h"

/* 前向声明：Choreography 定义在 scenario_loader.h（C 头） */
struct Choreography;

namespace flowsim {

/** 红绿灯相位（存在 Entity::phase_state） */
enum class TLPhase : int {
    Green  = 0,
    Yellow = 1,
    Red    = 2,
};

/**
 * 推进所有红绿灯相位。
 * @param pool        实体池
 * @param sim_time_s  仿真时间（秒）
 *
 * 红绿灯参数存在 Entity 上：
 *   - phase_timer  ：当前相位剩余时间（会被本函数递减）
 *   - 用 Entity::length/width 字段复用存相位时长不合适，改用以下约定：
 *   - Entity::throttle = green 时长（s）
 *   - Entity::brake    = yellow 时长（s）
 *   - Entity::steer    = red 时长（s）
 *   - Entity::target_vx = 相位偏移（s，用于错峰）
 *
 * 这种字段复用是因为 Entity 是固定结构，不想为红绿灯单独加字段。
 * 场景加载时把 JSON 里的 green_s/yellow_s/red_s 写到这三个字段。
 */
void tick_traffic_lights(EntityPool& pool, double sim_time_s);

/**
 * 推进 ETC 闸门状态。
 * @param pool  实体池
 * @param ego   ego 实体
 * @param dt    时间步长
 *
 * ETC 状态机（用 ai_state）：
 *   - Stop（默认/远距）：闸门关闭
 *   - Yield（10-50m）：抬杆中
 *   - Cruise（<10m 或已通过）：闸门打开
 * 抬杆进度存在 Entity::phase_timer（0=关，1=全开）。
 */
void tick_etc_gates(EntityPool& pool, const Entity& ego, double dt);

/**
 * 检查 NPC 是否需要为前方红绿灯/ETC 减速。
 * 在 step_npc_vehicle 之前调用，会修改 npc.target_vx 和 npc.ai_state。
 *
 * @param pool  实体池
 * @param cfg   AI 配置（用 look_ahead）
 */
void check_npc_scene_events(EntityPool& pool, double look_ahead);

/**
 * 推进编舞循环：按 loop_period_s 周期重置 actor 到 ego 附近。
 * 复用 scene_events.cpp:44 的 fmod(sim_time+offset, T) 循环范式。
 *
 * 语义：每 loop_period_s 一轮，在 fmod(sim_time, T) ≈ beat.t 时，
 * 把该 actor 重置到 ego 前方 ds 米、横向偏 dl 米、给定 vx。
 *
 * 无 choreography 块时是 no-op（向后兼容）。
 *
 * @param pool        实体池
 * @param ego         ego 实体
 * @param sim_time_s  仿真时间（秒）
 * @param dt          时间步长（秒）
 * @param choreo      编舞配置（来自场景 JSON）
 */
void tick_choreography(EntityPool& pool, const Entity& ego,
                       double sim_time_s, double dt,
                       const Choreography* choreo);

}  // namespace flowsim

#endif  // FLOWSIM_SCENE_EVENTS_H
