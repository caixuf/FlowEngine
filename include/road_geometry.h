#ifndef ROAD_GEOMETRY_H
#define ROAD_GEOMETRY_H

/**
 * @file road_geometry.h
 * @brief 共享道路几何辅助函数（弯道/直道中心线）
 *
 * FlowEngine 的世界坐标系是全局 x(纵向)/y(横向)。默认情况下道路是一条
 * 沿 x 轴的直线，车道中心线 y=0（车辆/障碍物的 y 即为车道横向偏移）。
 *
 * 本文件引入一个可选的"弯道段"：在 [curve_start_x, curve_start_x +
 * curve_length_m] 区间内，道路中心线从 y=0 平滑过渡到 y=curve_offset_m
 * （S 形 smoothstep，一阶导数在两端为 0，保证航向连续、无突变）。
 *
 * curve_length_m <= 0 或 curve_offset_m == 0 时函数恒返回 0/0 —— 等价于
 * "禁用弯道"，与现有直道场景完全一致（向后兼容，默认零风险）。
 *
 * sim_world_node / planning_node / control_node 三方共享这份计算，
 * 保证仿真物理、Frenet 参考路径、车道居中控制对"道路在哪里"的理解一致。
 */

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 道路中心线在给定 x 处的横向偏移（m）。 */
static inline double road_center_y(double x,
                                    double curve_start_x,
                                    double curve_length_m,
                                    double curve_offset_m) {
    if (curve_length_m <= 0.0 || curve_offset_m == 0.0) return 0.0;
    if (x <= curve_start_x) return 0.0;
    double t = (x - curve_start_x) / curve_length_m;
    if (t >= 1.0) return curve_offset_m;
    /* smoothstep: 3t^2 - 2t^3，两端一阶导数为 0 → 航向平滑过渡 */
    return curve_offset_m * (3.0 * t * t - 2.0 * t * t * t);
}

/** 道路中心线在给定 x 处的切线航向角（rad），供横向控制做 heading 前馈。 */
static inline double road_center_heading(double x,
                                          double curve_start_x,
                                          double curve_length_m,
                                          double curve_offset_m) {
    if (curve_length_m <= 0.0 || curve_offset_m == 0.0) return 0.0;
    if (x <= curve_start_x || x >= curve_start_x + curve_length_m) return 0.0;
    double t = (x - curve_start_x) / curve_length_m;
    double dy_dt = curve_offset_m * (6.0 * t - 6.0 * t * t);
    double dy_dx = dy_dt / curve_length_m;
    return atan(dy_dx);
}

/**
 * 道路中心线在给定 x 处的曲率 κ = d²y/dx² / (1 + (dy/dx)²)^(3/2)。
 *
 * 用于横向控制的曲率前馈（Stanley/Pure Pursuit）：steady-state 转向角
 * δ_ff = wheelbase * κ。曲率带符号：κ>0 = 道路向 +y 方向弯曲（左弯），
 * κ<0 = 向 -y 方向弯曲（右弯）。
 *
 * smoothstep 的二阶导 d²y/dx² = offset * (6 - 12t) / len²，在 t=0.5 处为 0
 * （拐点），在两端极值 |6*offset/len²|。返回 0 表示直道或弯道端点外。
 *
 * NOA Phase 3.4: control_node 在 R ≤ 60m 时增大前馈权重，让控制器预先打
 * 方向盘而不是等 CTE 累积后再反应——匝道 R=45m 的回头弯尤其需要。
 */
static inline double road_center_curvature(double x,
                                            double curve_start_x,
                                            double curve_length_m,
                                            double curve_offset_m) {
    if (curve_length_m <= 0.0 || curve_offset_m == 0.0) return 0.0;
    if (x <= curve_start_x || x >= curve_start_x + curve_length_m) return 0.0;
    double t = (x - curve_start_x) / curve_length_m;
    double dy_dt  = curve_offset_m * (6.0 * t - 6.0 * t * t);
    double d2y_dt2 = curve_offset_m * (6.0 - 12.0 * t);
    double dy_dx   = dy_dt / curve_length_m;
    double d2y_dx2 = d2y_dt2 / (curve_length_m * curve_length_m);
    double denom = pow(1.0 + dy_dx * dy_dx, 1.5);
    if (denom < 1e-9) return 0.0;
    return d2y_dx2 / denom;
}

#ifdef __cplusplus
}
#endif

#endif /* ROAD_GEOMETRY_H */
