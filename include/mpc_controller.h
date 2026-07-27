#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

/**
 * @file mpc_controller.h
 * @brief 自行车模型 + 线性 MPC 控制器（5 维状态扩展版）
 *
 * 状态向量: [x, y, θ, v, δ]  —— δ 升级为状态，使转向角速率可被显式惩罚
 * 控制向量: [a, dδ]          —— dδ = δ 的速率 (rad/s)
 *
 * 运动学自行车模型（后轴中心）：
 *   x[t+1] = x[t] + v * cos(θ + β) * dt
 *   y[t+1] = y[t] + v * sin(θ + β) * dt
 *   θ[t+1] = θ[t] + v * sin(β) / L * dt
 *   v[t+1] = v[t] + a * dt
 *   δ[t+1] = δ[t] + dδ * dt          ← 新增：转向角作为状态
 *   其中 β = atan(tan(δ) * 0.5)
 *
 * 代价函数：
 *   J = Σ_{k=0}^{N-1} [ (x-x_ref)ᵀ Q (x-x_ref) + uᵀ R u ]
 *       + (x_N - x_ref_N)ᵀ Qf (x_N - x_ref_N)
 *   Q  = diag(q_x, q_y, q_theta, q_v, q_delta)
 *   R  = diag(r_a, r_ddelta)     ← r_ddelta 真正进入二阶项，抑制 dδ 振荡
 *   参考中 δ_ref = atan(kappa * L)（运动学前轮转向角）
 *
 * 约束：
 *   |δ| ≤ δ_max, |a| ≤ a_max, |dδ| ≤ dδ_max (rad/s), v_min ≤ v ≤ v_max
 *
 * 算法：iLQR（迭代线性化 + Riccati 反向传播 + 线搜索前向 rollout）
 *
 * 与上一版的差异（2026-07 重构）：
 *   - 旧版 [x,y,θ,v] + 控制 [a,δ]：r_ddelta 注册但从未被求解器使用 → 转向翻转零代价
 *   - 新版 [x,y,θ,v,δ] + 控制 [a,dδ]：dδ 直接作为控制变量，
 *     R[1][1]=r_ddelta 进入 Quu 二阶项，求解器真正感知转向速率代价。
 *     同时 δ 的绝对值约束通过状态约束实现，max_dsteer 限幅作用于 dδ。
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────── */

#define MPC_STATE_DIM     5       /**< 状态维数 [x, y, θ, v, δ] */
#define MPC_CONTROL_DIM   2       /**< 控制维数 [a, dδ] */
#define MPC_MAX_HORIZON   20      /**< 最大预测步数 */
#define MPC_MAX_REF_PTS   30      /**< 最大参考轨迹点数 */
#define MPC_MAX_ITER      50      /**< QP 求解最大迭代 */
#define MPC_DEFAULT_DT    0.05    /**< 默认离散时间步长 (s) = 50ms */

/* ── Configuration ──────────────────────────────────────────── */

typedef struct {
    int     horizon;          /**< 预测步数 (1..MPC_MAX_HORIZON) */
    double  dt;               /**< 离散时间步长 (s)，默认 0.05 */

    /* 状态权重 Q = diag(q_x, q_y, q_theta, q_v, q_delta) */
    double  q_x;              /**< 纵向位置误差权重 */
    double  q_y;              /**< 横向位置误差权重 (cross-track) */
    double  q_theta;          /**< 航向角误差权重 */
    double  q_v;              /**< 速度误差权重 */
    double  q_delta;          /**< 转向角误差权重（相对 δ_ref，原 r_delta 语义） */

    /* 终端权重 Qf = diag(qf_x, qf_y, qf_theta, qf_v) */
    double  qf_x;
    double  qf_y;
    double  qf_theta;
    double  qf_v;

    /* 控制权重 R = diag(r_a, r_ddelta) */
    double  r_a;              /**< 加速度代价（控制代价） */
    double  r_ddelta;         /**< 转向角速率代价（dδ 的二阶项，抑制振荡） */

    /* 控制约束 */
    double  max_accel;        /**< 最大加速度 (m/s²) */
    double  max_decel;        /**< 最大减速度 (m/s²) */
    double  max_steer;        /**< 最大转向角绝对值 (rad)，作用于状态 δ */
    double  max_dsteer;       /**< 最大转向角速率 (rad/s)，作用于控制 dδ */
    double  max_speed;        /**< 最高速度 (m/s) */
    double  min_speed;        /**< 最低速度 (m/s) */

    /* 车辆参数 */
    double  wheelbase;        /**< 轴距 (m) */

    /* 求解器参数 */
    double  convergence_tol;  /**< 收敛容差 (默认 1e-4) */
    double  line_search_c;    /**< 线搜索步长缩放 (默认 0.5) */
    int     max_iter;         /**< 最大迭代次数 */
} MpcConfig;

/** 参考轨迹点 */
typedef struct {
    double x;        /**< 世界坐标 x */
    double y;        /**< 世界坐标 y */
    double heading;  /**< 航向角 (rad) */
    double speed;    /**< 期望速度 (m/s) */
    double kappa;    /**< 曲率 (1/m)，用于计算 δ_ref = atan(kappa*L) */
} MpcRefPoint;

/** MPC 求解结果 */
typedef struct {
    double throttle;     /**< 油门 [0, 1] */
    double brake;        /**< 制动 [0, 1] */
    double steer;        /**< 转向角 (rad) —— 求解后第一帧的 δ 状态值 */
    double accel_cmd;    /**< 加速度指令 (m/s²) */
    double steer_rate;   /**< 转向角速率指令 (rad/s) —— 求解后第一帧 dδ */
    double predicted_traj[MPC_MAX_HORIZON][MPC_STATE_DIM]; /**< 预测轨迹 [step][x,y,θ,v,δ] */
    int    iterations;   /**< 实际迭代次数 */
    bool   converged;    /**< 是否收敛 */
    double cost;         /**< 最终代价 */
} MpcResult;

/* ── Opaque handle ──────────────────────────────────────────── */

typedef struct MpcController MpcController;

/* ── API ────────────────────────────────────────────────────── */

/**
 * 创建 MPC 控制器。
 * @param cfg  配置参数（深拷贝，调用后可释放）
 * @return 控制器指针，失败返回 NULL
 */
MpcController* mpc_create(const MpcConfig* cfg);

/** 销毁 MPC 控制器。 */
void mpc_destroy(MpcController* mpc);

/**
 * 设置参考轨迹（通常是 planning 输出的路径 + 速度曲线）。
 * @param ref_points  参考轨迹点数组
 * @param n_points    点数
 */
void mpc_set_reference(MpcController* mpc,
                       const MpcRefPoint* ref_points,
                       int n_points);

/**
 * 设置当前状态（从 fusion/localization 获取）。
 * @param x       ego 世界坐标 x
 * @param y       ego 世界坐标 y
 * @param heading ego 航向角 (rad)
 * @param speed   ego 速度 (m/s)
 */
void mpc_set_state(MpcController* mpc,
                   double x, double y,
                   double heading, double speed);

/**
 * 设置当前前轮转向角（作为状态初值 δ_0，用于转向速率约束）。
 * @param steer  当前转向角 (rad)
 */
void mpc_set_prev_steer(MpcController* mpc, double steer);

/**
 * 求解 MPC 优化问题。
 * 使用线性化 + QP 近似（迭代 LQR / iLQR 风格），
 * 在 MPC_MAX_ITER 次内收敛或返回最佳可行解。
 *
 * @param result  输出结果
 * @return 0 成功，-1 失败
 */
int mpc_solve(MpcController* mpc, MpcResult* result);

/** 获取默认配置（合理的默认值）。 */
MpcConfig mpc_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* MPC_CONTROLLER_H */
