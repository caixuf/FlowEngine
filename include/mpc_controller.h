#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

/**
 * @file mpc_controller.h
 * @brief 自行车模型 + 线性 MPC 控制器
 *
 * 基于运动学自行车模型 (Kinematic Bicycle Model)：
 *   x[t+1] = x[t] + v * cos(θ + δ) * dt
 *   y[t+1] = y[t] + v * sin(θ + δ) * dt
 *   θ[t+1] = θ[t] + v * sin(δ) / L * dt
 *   v[t+1] = v[t] + a * dt
 *
 * 状态向量: [x, y, θ, v]
 * 控制向量: [a, δ]  (加速度, 转向角)
 *
 * 代价函数:
 *   J = Σ (x - x_ref)ᵀ Q (x - x_ref) + uᵀ R u + (x_N - x_ref_N)ᵀ Qf (x_N - x_ref_N)
 *
 * 约束:
 *   |δ| ≤ δ_max, |a| ≤ a_max, v_min ≤ v ≤ v_max
 *
 * 与 PID 对比：
 *   - PID 无模型：仅对当前误差反应，无前瞻能力
 *   - MPC 有模型：预测 N 步未来，同时优化纵向和横向，天然处理约束
 *
 * 用法：
 *   MpcController mpc;
 *   mpc_init(&mpc, &cfg);
 *   mpc_set_reference(&mpc, ref_x, ref_y, ref_heading, ref_speed);
 *   mpc_set_state(&mpc, ego_x, ego_y, ego_heading, ego_speed);
 *   mpc_solve(&mpc, &throttle, &brake, &steer);
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────── */

#define MPC_MAX_HORIZON   20      /**< 最大预测步数 */
#define MPC_MAX_REF_PTS   30      /**< 最大参考轨迹点数 */
#define MPC_MAX_ITER      50      /**< QP 求解最大迭代 */
#define MPC_DEFAULT_DT    0.05    /**< 默认离散时间步长 (s) = 50ms */

/* ── Configuration ──────────────────────────────────────────── */

typedef struct {
    /** 预测步数 (1..MPC_MAX_HORIZON) */
    int     horizon;
    /** 离散时间步长 (s)，默认 0.05 (=50ms, 20Hz) */
    double  dt;

    /* 状态权重矩阵 Q = diag(q_x, q_y, q_theta, q_v) */
    double  q_x;        /**< 纵向位置误差权重 */
    double  q_y;        /**< 横向位置误差权重 (cross-track) */
    double  q_theta;    /**< 航向角误差权重 */
    double  q_v;        /**< 速度误差权重 */

    /* 终端权重 Qf = diag(qf_x, qf_y, qf_theta, qf_v) */
    double  qf_x;
    double  qf_y;
    double  qf_theta;
    double  qf_v;

    /* 控制权重矩阵 R = diag(r_a, r_delta) */
    double  r_a;        /**< 加速度变化惩罚 */
    double  r_delta;    /**< 转向角变化惩罚 */
    double  r_ddelta;   /**< 转向角速率变化惩罚 */

    /* 控制约束 */
    double  max_accel;      /**< 最大加速度 (m/s²) */
    double  max_decel;      /**< 最大减速度 (m/s²) */
    double  max_steer;      /**< 最大转向角绝对值 (rad) */
    double  max_dsteer;     /**< 最大转向角速率 (rad/s) */
    double  max_speed;      /**< 最高速度 (m/s) */
    double  min_speed;      /**< 最低速度 (m/s) */

    /* 车辆参数 */
    double  wheelbase;      /**< 轴距 (m) */

    /* 求解器参数 */
    double  convergence_tol; /**< 收敛容差 (默认 1e-4) */
    double  line_search_c;   /**< 线搜索步长缩放 (默认 0.5) */
    int     max_iter;        /**< 最大迭代次数 */
} MpcConfig;

/** 参考轨迹点 */
typedef struct {
    double x;       /**< 世界坐标 x */
    double y;       /**< 世界坐标 y */
    double heading; /**< 航向角 (rad) */
    double speed;   /**< 期望速度 (m/s) */
    double kappa;   /**< 曲率 (1/m) */
} MpcRefPoint;

/** MPC 求解结果 */
typedef struct {
    double throttle;     /**< 油门 [0, 1] */
    double brake;        /**< 制动 [0, 1] */
    double steer;        /**< 转向角 (rad) */
    double accel_cmd;    /**< 加速度指令 (m/s²) */
    double predicted_traj[MPC_MAX_HORIZON][4]; /**< 预测轨迹 [step][x,y,θ,v] */
    int    iterations;    /**< 实际迭代次数 */
    bool   converged;     /**< 是否收敛 */
    double cost;          /**< 最终代价 */
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

/**
 * 销毁 MPC 控制器。
 */
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
 * 设置当前前轮转向角（用于转向速率约束）。
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

/**
 * 获取默认配置（合理的默认值）。
 */
MpcConfig mpc_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* MPC_CONTROLLER_H */