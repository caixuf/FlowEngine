/**
 * physics.cpp — 自行车运动学模型积分实现
 *
 * 数值方法：前向欧拉积分，dt=0.05s（20Hz）足够稳定。
 * 力模型与 sim_world_node.c vehicle_tick() 一致，保证迁移后行为不退化：
 *   drive_force  = throttle * 5000 N
 *   brake_force  = brake    * 8000 N
 *   drag_force   = drag_coeff * speed²
 *   net_force    = drive - brake - drag
 *   accel        = net_force / mass
 *
 * 横向：自行车运动学
 *   d(heading)/dt = (speed / wheelbase) * tan(steer)
 *   dx/dt = speed * cos(heading)
 *   dy/dt = speed * sin(heading)
 */

#include "physics.h"

#include "logger.h"

#include <cmath>

namespace flowsim {

/* 动力学模型桩：单次日志计数，避免每帧刷屏 */
static bool s_dynamic_warned = false;

void step_bicycle(Entity& e, double dt, double throttle, double brake, double steer) {
    // 纵向力
    double drive_force = throttle * 5000.0;
    double brake_force = brake    * 8000.0;
    double drag_force  = e.drag_coeff * e.speed * e.speed;
    double net_force   = drive_force - brake_force - drag_force;
    double accel       = net_force / e.mass;

    e.speed += accel * dt;
    if (e.speed < 0.0) e.speed = 0.0;

    // ── 一阶执行器滞后 + 速率限幅 ──
    // steer_cmd 是指令，e.steer 是实际转角（有状态）。
    // τ≈0.15s 是典型 EPS 响应
    double steer_cmd = steer;
    if (steer_cmd >  0.25) steer_cmd =  0.25;
    if (steer_cmd < -0.25) steer_cmd = -0.25;

    double alpha = dt / (e.steer_tau + dt);          // τ=0.15
    double steer_next = e.steer + alpha * (steer_cmd - e.steer);

    double max_rate = e.steer_rate_max * dt;         // 0.6 rad/s
    double d = steer_next - e.steer;
    if (d >  max_rate) steer_next = e.steer + max_rate;
    if (d < -max_rate) steer_next = e.steer - max_rate;
    e.steer = steer_next;

    // ── 显式 yaw_rate ──
    e.yaw_rate = (e.speed / e.wheelbase) * std::tan(e.steer);
    e.heading += e.yaw_rate * dt;
    // 航向归一化到 [-π, π]
    while (e.heading >  M_PI) e.heading -= 2.0 * M_PI;
    while (e.heading < -M_PI) e.heading += 2.0 * M_PI;

    e.x += e.speed * dt * std::cos(e.heading);
    e.y += e.speed * dt * std::sin(e.heading);

    // 世界系速度（供 perception/sensor 使用）
    e.vx = e.speed * std::cos(e.heading);
    e.vy = e.speed * std::sin(e.heading);
}

void step_pedestrian(Entity& e, double dt) {
    e.x += e.vx * dt;
    e.y += e.vy * dt;
    e.speed = std::sqrt(e.vx * e.vx + e.vy * e.vy);
}

void step_bicycle_dynamic(Entity& e, double dt, double throttle, double brake, double steer) {
    if (!s_dynamic_warned) {
        s_dynamic_warned = true;
        LOG_WARN("flowsim", "physics_model=dynamic is not yet implemented "
                 "(tire slip / relaxation / Pacejka stub). "
                 "Falling back to kinematic step_bicycle.");
    }
    step_bicycle(e, dt, throttle, brake, steer);
}

void apply_vehicle_defaults(Entity& e) {
    /* 执行器滞后参数：所有车辆共用默认值 */
    e.steer_tau = 0.15;
    e.steer_rate_max = 0.6;

    switch (e.type) {
        case EntityType::Truck:
            e.length = 8.0;  e.width = 2.4;
            e.wheelbase = 5.0;
            e.mass = 8000.0;
            e.drag_coeff = 0.6;
            e.max_brake = 3.0;
            break;
        case EntityType::SUV:
            e.length = 4.8;  e.width = 2.0;
            e.wheelbase = 2.85;
            e.mass = 1800.0;
            e.drag_coeff = 0.45;
            e.max_brake = 4.0;
            break;
        case EntityType::Car:
        case EntityType::Ego:
        default:
            e.length = 4.6;  e.width = 2.0;
            e.wheelbase = 2.7;
            e.mass = 1500.0;
            e.drag_coeff = 0.4;
            e.max_brake = 4.0;
            break;
    }
}

}  // namespace flowsim
