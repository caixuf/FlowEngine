/**
 * physics.h — 车辆动力学（自行车模型）
 *
 * 从 sim_world_node.c 的 vehicle_tick() 提取的自行车模型积分：
 *   - 纵向：驱动力 - 制动力 - 空气阻力 → 加速度 → 速度
 *   - 横向：方向盘转角 → 航向变化 → x/y 位置更新
 *
 * 自行车模型假设：
 *   - 车辆是刚体，前后轮在同一平面
 *   - 前轮转向角度 = steer，后轮不转
 *   - 无侧滑（低速近似）
 *   - 适用于 <30 m/s 的乘用车仿真
 *
 * 参数默认值对应一辆中型轿车（mass=1500kg, wheelbase=2.7m）。
 */

#ifndef FLOWSIM_PHYSICS_H
#define FLOWSIM_PHYSICS_H

#include "entity.h"

namespace flowsim {

/**
 * 单步自行车模型积分。
 * @param e      实体（必须 is_vehicle()，会更新 x/y/heading/speed/vx/vy）
 * @param dt     时间步长 (s)
 * @param throttle 油门 [0,1]
 * @param brake    刹车 [0,1]
 * @param steer    方向盘转角 (rad)，正值右转
 *
 * 调用方负责设置 throttle/brake/steer（ego 从 control/cmd，NPC 从 AI）。
 * 本函数只做物理积分，不做决策。
 */
void step_bicycle(Entity& e, double dt, double throttle, double brake, double steer);

/**
 * 简易行人运动学：按 vx/vy 匀速移动，到达边界后反弹/停止。
 * 调用方负责设置 vx/vy 和边界逻辑（在 npc_ai 里处理）。
 * 本函数只做 x += vx*dt, y += vy*dt。
 */
void step_pedestrian(Entity& e, double dt);

/**
 * 按车辆类型设置默认参数。
 * car: 中型轿车，truck: 卡车（更重更长），suv: SUV（介于两者）。
 */
void apply_vehicle_defaults(Entity& e);

}  // namespace flowsim

#endif  // FLOWSIM_PHYSICS_H
