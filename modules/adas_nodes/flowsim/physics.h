/**
 * physics.h — 车辆动力学（自行车模型，运动学 + 动力学两版）
 *
 * 纵向（两版共用）：驱动力 - 制动力 - 空气阻力 → 加速度 → 速度。
 *
 * 横向两版：
 *   - step_bicycle（默认）——运动学：steer → 航向变化 → x/y。无侧滑假设，
 *     前后轮同平面刚体、后轮不转，适用于常规乘用车仿真。
 *   - step_bicycle_dynamic（opt-in）——线性轮胎二自由度：侧向速度 + 横摆角速度
 *     积分，低速退化运动学。边界见该函数注释与 docs/CALIBRATION_GUIDE.md。
 *
 * 参数默认值对应一辆中型轿车（mass=1500kg, wheelbase=2.7m）。
 */

#ifndef FLOWSIM_PHYSICS_H
#define FLOWSIM_PHYSICS_H

#include "entity.h"

namespace flowsim {

/**
 * 运动学自行车模型积分（当前默认）。
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
 * 动力学自行车模型积分（线性轮胎二自由度，opt-in）。
 *
 * 与运动学模型的区别：
 *   - 复用 step_bicycle 的纵向力（drive/brake/drag）与执行器滞后
 *   - 横向解侧向速度 v_y_body + 横摆角速度 yaw_rate，heading 由模型自主积分
 *     （与运动学一样都不做道路切线重置——flowsim_node 的 is_dynamic 分支对
 *     两模式一视同仁，只做 heading 归一化；区别在于本模型 v_y/r 是显式状态）
 *   - 滑移角 α_f=steer−atan2(v_y+a·r, v_x)、α_r=−atan2(v_y−b·r, v_x)
 *   - 线性轮胎 F_y=Cα·α；用到 apply_vehicle_defaults 设的 tire_stiffness_f/r、yaw_inertia
 *
 * 三条已知边界（详见 docs/CALIBRATION_GUIDE.md）：
 *   1. 线性轮胎——滑移角饱和于 ±0.12rad，超出不再增长（近似摩擦极限），
 *      故极限工况的过弯不精确，但保证有界不发散。
 *   2. 低速退化——v_x<5m/s 转调 step_bicycle（前向欧拉在高刚度低速发散 +
 *      低速轮胎滑移可忽略）。故起步/堵车段与运动学等价。
 *   3. 与现有 MPC 存在模型失配——控制器基于运动学假设整定，动力学下过弯
 *      横向误差可能变大。故默认关闭，仅作仿真保真度选项，验收标准是
 *      「稳定不发散 + invariant 通过」而非「控制精度优于运动学」。
 *
 * 启用方式：pipeline.json 中 flowsim 节点 params 加 "physics_model":"dynamic"
 *          （tire_stiffness_f/r、yaw_inertia 由 apply_vehicle_defaults 按车型预置）。
 */
void step_bicycle_dynamic(Entity& e, double dt, double throttle, double brake, double steer);

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
