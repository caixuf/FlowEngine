/**
 * physics.cpp — 自行车模型积分实现（运动学 + 动力学两版）
 *
 * 数值方法：前向欧拉积分，dt=0.05s（20Hz）。
 * 纵向力模型两版共用（与 sim_world_node.c vehicle_tick() 一致，行为不退化）：
 *   drive_force  = throttle * 5000 N
 *   brake_force  = brake    * 8000 N
 *   drag_force   = drag_coeff * speed²
 *   accel        = (drive - brake - drag) / mass
 *
 * 横向：
 *   运动学 step_bicycle —— d(heading)/dt = (speed/wheelbase)·tan(steer)
 *   动力学 step_bicycle_dynamic —— 线性轮胎二自由度（侧向速度 v_y + 横摆角速度 r）：
 *     α_f = steer − atan2(v_y + a·r, v_x)      前轴滑移角
 *     α_r =       − atan2(v_y − b·r, v_x)      后轴滑移角
 *     F_yf = Cαf·α_f,  F_yr = Cαr·α_r          线性轮胎侧向力
 *     v_y' = (F_yf + F_yr)/m − v_x·r
 *     r'   = (a·F_yf − b·F_yr)/Iz
 *   低速（v_x < LOW_SPEED_MS）退化运动学：前向欧拉在此区间对横向刚度不稳定，
 *   且低速轮胎滑移可忽略，运动学既够用又更稳。边界详见 docs/CALIBRATION_GUIDE.md。
 */

#include "physics.h"

#include <cmath>

namespace flowsim {

/* 动力学模型低于此纵向车速退化运动学（前向欧拉稳定域下界 + 低速滑移可忽略） */
static constexpr double LOW_SPEED_MS = 5.0;
/* 质心到前/后轴距离占轴距的比例（质心略偏前，等刚度下呈不足转向） */
static constexpr double CG_FRONT_FRAC = 0.45;
/* 滑移角饱和：线性轮胎仅在小滑移角有效，超出近似摩擦极限而饱和。
 * 既贴近真实轮胎（friction circle），又保证任意控制输入下前向欧拉有界不发散。 */
static constexpr double SLIP_ANGLE_MAX = 0.12;

static inline double clamp_slip(double a) {
    if (a >  SLIP_ANGLE_MAX) return  SLIP_ANGLE_MAX;
    if (a < -SLIP_ANGLE_MAX) return -SLIP_ANGLE_MAX;
    return a;
}

/* 纵向加速度：两模型共用（净力/质量） */
static double longitudinal_accel(const Entity& e, double throttle, double brake, double v) {
    double drive_force = throttle * 5000.0;
    double brake_force = brake    * 8000.0;
    double drag_force  = e.drag_coeff * v * v;
    return (drive_force - brake_force - drag_force) / e.mass;
}

/* 转向执行器：一阶滞后 (τ≈0.15s EPS) + 速率限幅，写回 e.steer。两模型共用 */
static void update_steer(Entity& e, double steer_cmd, double dt) {
    if (steer_cmd >  0.25) steer_cmd =  0.25;
    if (steer_cmd < -0.25) steer_cmd = -0.25;

    double alpha = dt / (e.steer_tau + dt);
    double steer_next = e.steer + alpha * (steer_cmd - e.steer);

    double max_rate = e.steer_rate_max * dt;
    double d = steer_next - e.steer;
    if (d >  max_rate) steer_next = e.steer + max_rate;
    if (d < -max_rate) steer_next = e.steer - max_rate;
    e.steer = steer_next;
}

static void normalize_heading(Entity& e) {
    while (e.heading >  M_PI) e.heading -= 2.0 * M_PI;
    while (e.heading < -M_PI) e.heading += 2.0 * M_PI;
}

void step_bicycle(Entity& e, double dt, double throttle, double brake, double steer) {
    e.speed += longitudinal_accel(e, throttle, brake, e.speed) * dt;
    if (e.speed < 0.0) e.speed = 0.0;

    update_steer(e, steer, dt);

    // ── 运动学 yaw_rate ──
    e.yaw_rate = (e.speed / e.wheelbase) * std::tan(e.steer);
    e.heading += e.yaw_rate * dt;
    normalize_heading(e);

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

/* 线性轮胎二自由度横向动力学积分：更新 v_y_body / yaw_rate / heading，
 * 并把车体系速度投影回世界系 x/y。调用前 e.v_x_body 应为当前纵向车速。 */
static void integrate_lateral_dynamics(Entity& e, double dt) {
    double a  = CG_FRONT_FRAC * e.wheelbase;          // 质心→前轴
    double b  = e.wheelbase - a;                      // 质心→后轴
    double vx = e.v_x_body;

    double alpha_f = clamp_slip(e.steer - std::atan2(e.v_y_body + a * e.yaw_rate, vx));
    double alpha_r = clamp_slip(       - std::atan2(e.v_y_body - b * e.yaw_rate, vx));

    e.F_yf = e.tire_stiffness_f * alpha_f;            // 线性轮胎 F_y = Cα·α（饱和后）
    e.F_yr = e.tire_stiffness_r * alpha_r;

    double vy_dot = (e.F_yf + e.F_yr) / e.mass - vx * e.yaw_rate;
    double r_dot  = (a * e.F_yf - b * e.F_yr) / e.yaw_inertia;

    e.v_y_body += vy_dot * dt;
    e.yaw_rate += r_dot  * dt;
    e.heading  += e.yaw_rate * dt;
    normalize_heading(e);

    // 车体系 (v_x, v_y) → 世界系
    double ch = std::cos(e.heading), sh = std::sin(e.heading);
    e.vx = e.v_x_body * ch - e.v_y_body * sh;
    e.vy = e.v_x_body * sh + e.v_y_body * ch;
    e.x += e.vx * dt;
    e.y += e.vy * dt;
    e.speed = std::sqrt(e.v_x_body * e.v_x_body + e.v_y_body * e.v_y_body);
}

void step_bicycle_dynamic(Entity& e, double dt, double throttle, double brake, double steer) {
    // 低速退化：运动学既够用又稳（前向欧拉对高横向刚度在低速发散）
    if (e.speed < LOW_SPEED_MS) {
        step_bicycle(e, dt, throttle, brake, steer);
        e.v_x_body = e.speed;                         // 供下一帧跨过阈值时无缝接续
        e.v_y_body = 0.0;
        return;
    }

    // 纵向车速积分（车体系纵向 = 标量车速）
    e.v_x_body = e.speed + longitudinal_accel(e, throttle, brake, e.speed) * dt;
    if (e.v_x_body < 0.0) e.v_x_body = 0.0;

    update_steer(e, steer, dt);
    integrate_lateral_dynamics(e, dt);               // 内部把 e.speed 更新为合速度大小
}

void apply_vehicle_defaults(Entity& e) {
    /* 执行器滞后参数：所有车辆共用默认值 */
    e.steer_tau = 0.15;
    e.steer_rate_max = 0.6;

    /* 动力学模型参数（physics_model=dynamic 时才被读取；运动学模式无视）：
     *   tire_stiffness_{f,r} —— 每轴等效侧偏刚度 (N/rad)，线性轮胎 F_y = Cα·α
     *   yaw_inertia          —— 绕 Z 轴转动惯量 Iz (kg·m²)
     * 量级取自乘用车/SUV/卡车典型标定值（见 docs/CALIBRATION_GUIDE.md）。 */
    switch (e.type) {
        case EntityType::Truck:
            e.length = 8.0;  e.width = 2.4;
            e.wheelbase = 5.0;
            e.mass = 8000.0;
            e.drag_coeff = 0.6;
            e.max_brake = 3.0;
            e.tire_stiffness_f = 180000.0;
            e.tire_stiffness_r = 180000.0;
            e.yaw_inertia = 25000.0;
            break;
        case EntityType::SUV:
            e.length = 4.8;  e.width = 2.0;
            e.wheelbase = 2.85;
            e.mass = 1800.0;
            e.drag_coeff = 0.45;
            e.max_brake = 4.0;
            e.tire_stiffness_f = 90000.0;
            e.tire_stiffness_r = 90000.0;
            e.yaw_inertia = 3200.0;
            break;
        case EntityType::Car:
        case EntityType::Ego:
        default:
            e.length = 4.6;  e.width = 2.0;
            e.wheelbase = 2.7;
            e.mass = 1500.0;
            e.drag_coeff = 0.4;
            e.max_brake = 4.0;
            e.tire_stiffness_f = 80000.0;
            e.tire_stiffness_r = 80000.0;
            e.yaw_inertia = 2250.0;
            break;
    }
}

}  // namespace flowsim
