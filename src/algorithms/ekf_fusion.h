/**
 * ekf_fusion.h — Extended Kalman Filter for Multi-Sensor Fusion
 *
 * State vector (5D): [x, y, v, heading, yaw_rate]
 *   x, y      — world-frame position (m)
 *   v         — longitudinal velocity (m/s)
 *   heading   — yaw angle (rad, 0=east, CCW positive)
 *   yaw_rate  — turn rate (rad/s)
 *
 * Prediction:  bicycle kinematic model (constant steer & velocity)
 *   x'      = x  + v * cos(heading) * dt
 *   y'      = y  + v * sin(heading) * dt
 *   v'      = v  + a * dt  (a from process noise)
 *   ψ'      = ψ  + yaw_rate * dt
 *   ψ̇'      = ψ̇  (nearly constant turn rate)
 *
 * Updates (partial, sensor-dependent):
 *   LiDAR position:  z = [x_obj - x_ego,  y_obj - y_ego]  →  ego position
 *   GPS:             z = [lat→x, lon→y, v, heading]
 *
 * All matrix ops are hardcoded for 5x5 (or smaller) — no external deps.
 *
 * Usage:
 *   EkfFusion ekf;
 *   ekf_fusion_init(&ekf, dt);
 *   ekf_fusion_predict(&ekf);                        // dead-reckoning step
 *   ekf_fusion_update_lidar(&ekf, lx, ly);           // LiDAR position fix
 *   ekf_fusion_update_gps(&ekf, gx, gy, gv, gh);     // GPS fix
 *   ekf_fusion_get_state(&ekf, &x, &y, &v, &heading); // read fused state
 *
 * Reference:
 *   Thrun, Burgard, Fox. "Probabilistic Robotics". Ch.3, Ch.7.
 */

#ifndef EKF_FUSION_H
#define EKF_FUSION_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── EKF 状态维度 ──────────────────────────────────────────── */
#define EKF_STATE_DIM  5   /* [x, y, v, heading, yaw_rate] */

/* ── EKF 实例 ───────────────────────────────────────────────── */
typedef struct {
    /* 状态均值 */
    double x[EKF_STATE_DIM];

    /* 协方差矩阵 (5×5, row-major) */
    double P[EKF_STATE_DIM * EKF_STATE_DIM];

    /* 过程噪声协方差 Q (5×5) */
    double Q[EKF_STATE_DIM * EKF_STATE_DIM];

    /* 时间步长 (s) */
    double dt;

    /* 诊断 */
    int    predict_count;
    int    update_count;
    double last_innovation;  /* 最后一次更新的新息范数 */
    int    diverged;         /* 协方差是否发散 */
} EkfFusion;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 初始化 EKF 融合器。
 * @param ekf     实例
 * @param dt      预测步长 (s)，建议 0.05 (20Hz)
 * @param x0      初始状态 [x, y, v, heading, yaw_rate], 或 NULL 用默认值
 */
void ekf_fusion_init(EkfFusion* ekf, double dt, const double x0[EKF_STATE_DIM]);

/**
 * 预测步: 运动学模型向前推进一步。
 * 每次传感器更新前调用一次。
 */
void ekf_fusion_predict(EkfFusion* ekf);

/**
 * LiDAR 位置更新 — 观测自车相对于参考物的位置。
 * @param z_x  观测的 x 位置 (m)
 * @param z_y  观测的 y 位置 (m)
 * @param R    测量噪声协方差 (2×2, row-major) [var_x, 0, 0, var_y], 或 NULL 用默认
 */
void ekf_fusion_update_lidar(EkfFusion* ekf, double z_x, double z_y,
                              const double R[4]);

/**
 * GPS 速度+航向更新 — 观测速度和航向角。
 * @param z_v       观测速度 (m/s)
 * @param z_heading 观测航向 (rad)
 * @param R         测量噪声协方差 (2×2), 或 NULL 用默认
 *
 * 注意: GPS 位置 (lat/lon) 需要在外部先转换为局部坐标再调用 update_lidar。
 */
void ekf_fusion_update_gps(EkfFusion* ekf, double z_v, double z_heading,
                            const double R[4]);

/**
 * 全 GPS 更新 (位置+速度+航向) — 4维观测。
 * @param z_x,z_y   观测位置 (局部坐标, m)
 * @param z_v       观测速度 (m/s)
 * @param z_heading 观测航向 (rad)
 */
void ekf_fusion_update_gps_full(EkfFusion* ekf,
                                 double z_x, double z_y,
                                 double z_v, double z_heading);

/**
 * 读取融合后的状态。
 * 任意参数可传 NULL。
 */
void ekf_fusion_get_state(const EkfFusion* ekf,
                           double* x, double* y,
                           double* v, double* heading,
                           double* yaw_rate);

/**
 * 获取协方差对角线 (不确定性)。
 */
void ekf_fusion_get_covariance_diag(const EkfFusion* ekf, double diag[EKF_STATE_DIM]);

/**
 * 重置 EKF（状态保持，协方差恢复到初始值）。
 */
void ekf_fusion_reset(EkfFusion* ekf);

#ifdef __cplusplus
}
#endif

#endif /* EKF_FUSION_H */
