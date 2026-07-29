#ifndef PIECEWISE_JERK_QP_H
#define PIECEWISE_JERK_QP_H

/**
 * @file piecewise_jerk_qp.h
 * @brief 带状 QP 求解器 — 应用于参考线平滑、Path QP、Speed QP
 *
 * 针对 piecewise-jerk 三对角/五对角 Hessian 优化的 banded LDLᵀ 求解，
 * 含 box 约束的 active-set 迭代。
 *
 * 三个典型应用：
 *   1. 参考线平滑：min Σ‖p_{i-1}−2p_i+p_{i+1}‖² + w Σ‖p_i−p_raw_i‖²
 *   2. Path QP (SL)：min w_l*l² + w_dl*l'² + w_ddl*l''² + w_dddl*l'''²
 *   3. Speed QP (ST)：min w_a*a² + w_j*j² + w_ref*(v−v_ref)²
 *
 * 数值方法：banded LDLᵀ + 投影 Newton active-set
 * 确定性：固定迭代上限，失败有确定返回码
 *
 * 设计约束：
 *   - 零动态分配（求解器在工作空间上操作）
 *   - 固定规模上限（N ≤ 200 点）
 *   - 单次求解亚毫秒级
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 常量 ─────────────────────────────────────────────────── */

#define PJQP_MAX_N 200  /**< 最大点数 */

/* ── 返回码 ────────────────────────────────────────────────── */

#define PJQP_OK            0   /**< 求解成功 */
#define PJQP_ERR_N         1   /**< N 超出上限 */
#define PJQP_ERR_SINGULAR  2   /**< Hessian 奇异（接触边界极多时） */
#define PJQP_ERR_ITER      3   /**< 迭代次数耗尽 */

/* ── 参考线平滑（无约束版本，只用软惩罚） ────────────────── */

/**
 * 参考线平滑 1D 求解。
 *
 * min  Σ_{i=1}^{N-2} (x_{i-1} − 2x_i + x_{i+1})²
 *      + w_raw Σ (x_i − x_raw_i)²
 *
 * Hessian = D₂ᵀD₂ + w_raw·I, 五对角（bandwidth=2）, 正定。
 * 单次 LDLᵀ 分解 + 回代，O(N·bw²)。
 *
 * @param x         输出：平滑后序列 [N]
 * @param x_raw     输入：原始序列 [N]
 * @param w_raw     原始点权重（越大越不光滑）
 * @param N         点数（3 ≤ N ≤ PJQP_MAX_N）
 * @return          0=成功，负数=错误码
 */
int pjqp_smooth_1d(double* x, const double* x_raw, double w_raw, int N);

/**
 * 参考线平滑 2D（x, y 独立求解）。
 *
 * @param x_out     输出：平滑后 X [N]
 * @param y_out     输出：平滑后 Y [N]
 * @param x_raw     输入：原始 X [N]
 * @param y_raw     输入：原始 Y [N]
 * @param w_raw     原始点权重
 * @param N         点数
 * @return          0=成功
 */
int pjqp_smooth_2d(double* x_out, double* y_out,
                    const double* x_raw, const double* y_raw,
                    double w_raw, int N);

/* ── 带状 LDLᵀ 求解器（低阶 API，可复用） ───────────────── */

/**
 * 对称五对角矩阵的带状 LDLᵀ 分解 + 求解。
 *
 * 矩阵 A ∈ R^{N×N}, 非零对角带 A[i][i], A[i][i±1], A[i][i±2]
 * 存储格式（LAPACK 风格）：L 和 D 压缩在 LD[5][N]
 *   LD[0][i] = main diagonal of D  (D[i])
 *   LD[1][i] = first sub-diagonal of L  (L[i][i-1])
 *   LD[2][i] = second sub-diagonal of L (L[i][i-2])
 *   LD[3][i] = first super-diagonal of L (L[i-1][i]) == LD[1][i-1]
 *   LD[4][i] = second super-diagonal of L (L[i-2][i]) == LD[2][i-1]
 *
 * @param LD        [inout] 5×N 带状存储，输入 A，输出 LDLᵀ 因子
 * @param N         矩阵维度
 * @return          0=成功，PJQP_ERR_SINGULAR=奇异
 */
int pjqp_banded_ldlt(double LD[5][PJQP_MAX_N], int N);

/**
 * LDLᵀ 回代求解 A·x = b。
 * 须在 pjqp_banded_ldlt 之后调用。
 *
 * @param x         输出解向量 [N]
 * @param b         右端项 [N]
 * @param LD        LDLᵀ 因子（来自 pjqp_banded_ldlt）
 * @param N         维度
 */
void pjqp_banded_solve(double* x, const double* b,
                        const double LD[5][PJQP_MAX_N], int N);

/* ── 带 box 约束的 path QP（SL 坐标系） ──────────────────── */

/**
 * Path QP 配置参数。
 */
typedef struct {
    /* Path QP weights */
    double w_l;        /**< 横向偏移权重 */
    double w_dl;       /**< 横向速度 l' 权重 */
    double w_ddl;      /**< 横向加速度 l'' 权重 */
    double w_dddl;     /**< 横向加加速度 l''' 权重 */
    double w_ref;      /**< 参考线跟踪权重 */
    /* Speed QP weights (ALGORITHM_REFACTOR_PLAN §8) */
    double w_a;        /**< 加速度权重 */
    double w_j;        /**< 加加速度权重 */
    double w_s_ref;    /**< 位置参考权重 */
    /* Common */
    double max_iter;   /**< 最大迭代次数（active-set） */
    double tol;        /**< 对偶可行容忍度 */
} PjqpConfig;

/**
 * 默认路径 QP 配置（平滑适中）。
 */
static const PjqpConfig PJQP_DEFAULT_PATH = {
    .w_l    = 0.0,
    .w_dl   = 1.0,
    .w_ddl  = 10.0,
    .w_dddl = 100.0,
    .w_ref  = 5.0,
    .w_a    = 1.0,
    .w_j    = 10.0,
    .w_s_ref = 0.0,
    .max_iter = 50,
    .tol    = 1e-6,
};

/**
 * 路径 QP 求解。
 *
 * min  Σ(w_l*l_i² + w_dl*l'_i² + w_ddl*l''_i² + w_dddl*l'''_i²)
 *      + w_ref Σ(l_i − l_ref_i)²
 *
 * s.t. l_low_i ≤ l_i ≤ l_up_i,  |l''_i| ≤ kappa_max
 *
 * 变量排列：每点 [l_i, l'_i, l''_i], i=0..N-1 → n=3N
 * Hessian 为块三对角（3×3 块）。
 *
 * @param l_out     输出：平滑后 l [N]
 * @param dl_out    输出：平滑后 l' [N]
 * @param ddl_out   输出：平滑后 l'' [N]
 * @param l_ref     参考线 l [N]
 * @param l_low     l 下界 [N]（可为 NULL）
 * @param l_up      l 上界 [N]（可为 NULL）
 * @param ddl_max   最大 l''（曲率约束）
 * @param cfg       配置参数
 * @param N         点数（3 ≤ N ≤ PJQP_MAX_N）
 * @return          0=成功
 */
int pjqp_path_solve(double* l_out, double* dl_out, double* ddl_out,
                     const double* l_ref,
                     const double* l_low, const double* l_up,
                     double ddl_max,
                     const PjqpConfig* cfg, int N);

/* ── Speed QP 求解（ST 坐标系，ALGORITHM_REFACTOR_PLAN §8） ── */

/**
 * Speed QP 求解（ST 坐标系）。
 *
 * min  w_a Σ(a_i²) + w_j Σ(j_i²) + w_ref Σ(v_i − v_ref_i)²
 *
 * s.t. v_min_i ≤ v_i ≤ v_max_i
 *      a_min ≤ a_i ≤ a_max
 *      s_i 单调递增 (s_{i+1} ≥ s_i + v_min*dt)
 *
 * 实现方式：将加速度和加加速度惩罚转为 v 的三对角系统，
 * 对 v 做 box 约束投影后，再递推 s 和 a。
 *
 * @param s_out     输出：位置 [N]
 * @param v_out     输出：速度 [N]
 * @param a_out     输出：加速度 [N]
 * @param v_ref     参考速度 [N]
 * @param v_max     速度上界 [N]
 * @param a_max     最大加速度
 * @param a_min     最小加速度（减速）
 * @param cfg       配置参数
 * @param dt        时间步长
 * @param N         点数
 * @return          0=成功
 */
int pjqp_speed_solve(double* s_out, double* v_out, double* a_out,
                      const double* v_ref, const double* v_max,
                      double a_max, double a_min,
                      const PjqpConfig* cfg, double dt, int N);

#ifdef __cplusplus
}
#endif

#endif /* PIECEWISE_JERK_QP_H */
