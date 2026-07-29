#ifndef LTV_MPC_H
#define LTV_MPC_H

/**
 * @file ltv_mpc.h
 * @brief LTV MPC 控制器 — 绕轨迹线性化的误差动力学 MPC
 *
 * 替代已删除的 mpc_controller + LQR 分支。
 * 使用 §8.0 带状 QP 求解器，确定性、零动态分配。
 *
 * 状态: [e_y, e_psi, delta] (横向误差、航向误差、前轮转角)
 * 控制: [ddelta] (转向速率)
 * 模型: 绕参考轨迹 (v(t), kappa(t)) 线性化的自行车模型
 *
 * 设计约束:
 *   - N=20, dt=0.05s → 1s 预测时域
 *   - 单次求解 < 1ms
 *   - 失败有确定返回码，不退化为 NaN/静默
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ─────────────────────────────────────────────────── */

#define LTV_MPC_MAX_HORIZON  20   /**< 最大预测步数 */
#define LTV_MPC_STATE_DIM    3    /**< [e_y, e_psi, delta] */
#define LTV_MPC_CONTROL_DIM  1    /**< [ddelta] */

/* ── 返回码 ────────────────────────────────────────────────── */

#define LTV_MPC_OK           0
#define LTV_MPC_ERR_N       -1   /**< N 超出上限 */
#define LTV_MPC_ERR_SINGULAR -2  /**< Hessian 奇异 */
#define LTV_MPC_ERR_ITER     -3  /**< 迭代耗尽 */
#define LTV_MPC_ERR_NAN      -4  /**< 解发散 */

/* ── 权重配置 ──────────────────────────────────────────────── */

typedef struct {
    double q_y;        /**< 横向误差权重 */
    double q_psi;      /**< 航向误差权重 */
    double q_delta;    /**< 转角状态权重（抑制大转角） */
    double r_ddelta;   /**< 转向速率权重（平顺性） */
    double qf_y;       /**< 终端横向误差权重 */
    double qf_psi;     /**< 终端航向误差权重 */
    int    horizon;    /**< 预测时域步数 (1..LTV_MPC_MAX_HORIZON) */
    double dt;         /**< 离散时间步长 (s) */
    double wheelbase;  /**< 轴距 (m) */
    double max_steer;  /**< 最大转角 (rad) */
    double max_dsteer; /**< 最大转向速率 (rad/s) */
} LtvMpcConfig;

/* ── 控制器状态机 ──────────────────────────────────────────── */

typedef enum {
    LTV_MPC_ACTIVE = 0,      /**< MPC 正常运行 */
    LTV_MPC_DEGRADED = 1,    /**< MPC 不收敛，用回退 */
    LTV_MPC_INIT = 2,        /**< 未初始化 */
} LtvMpcStatus;

/* ── MPC 求解器（不透明结构体） ────────────────────────────── */

typedef struct LtvMpcSolver LtvMpcSolver;

/**
 * 创建 MPC 求解器。
 * 返回 NULL 表示内存不足。
 */
LtvMpcSolver* ltv_mpc_create(const LtvMpcConfig* cfg);

/**
 * 销毁求解器。
 */
void ltv_mpc_destroy(LtvMpcSolver* solver);

/**
 * 在线更新权重（热重载）。
 */
void ltv_mpc_update_config(LtvMpcSolver* solver, const LtvMpcConfig* cfg);

/**
 * 设置参考轨迹。
 * 每步提供 v (参考速度) 和 kappa (参考曲率)。
 * @param v_ref    参考速度 [N] (m/s)
 * @param kappa_ref 参考曲率 [N] (1/m)
 * @param N        点数
 */
void ltv_mpc_set_reference(LtvMpcSolver* solver,
                            const double* v_ref, const double* kappa_ref, int N);

/**
 * 设置当前状态。
 * @param e_y     横向误差 (m)
 * @param e_psi   航向误差 (rad)
 * @param delta   当前前轮转角 (rad)
 * @param v       当前速度 (m/s)
 */
void ltv_mpc_set_state(LtvMpcSolver* solver,
                        double e_y, double e_psi, double delta, double v);

/**
 * MPC 求解一步。
 *
 * @param solver    求解器
 * @param steer_out 输出：最优转角增量 (rad)
 * @return          LTV_MPC_OK 或错误码
 */
int ltv_mpc_solve(LtvMpcSolver* solver, double* steer_out);

/**
 * 获取求解器状态。
 */
LtvMpcStatus ltv_mpc_status(const LtvMpcSolver* solver);

/**
 * 获取默认配置。
 */
LtvMpcConfig ltv_mpc_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* LTV_MPC_H */
