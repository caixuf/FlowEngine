/**
 * mpc_controller.c — 自行车模型 + iLQR MPC 求解器（5 维状态扩展版）
 *
 * 状态向量: [x, y, θ, v, δ]   (δ 升级为状态)
 * 控制向量: [a, dδ]           (dδ = δ 的速率)
 *
 * 算法：迭代线性化 MPC (iLQR 风格)
 *   1. 对当前状态线性化运动学模型
 *   2. 构建 LQR 代价 → 求解 Riccati 方程
 *   3. 线搜索 + 前向 rollout
 *   4. 重复直到收敛或达到最大迭代
 *
 * 数值稳定性：
 *   - 2×2 Hessian 直接求逆（Cholesky 失败时退化到对角近似）
 *   - 线搜索防止发散
 *   - 正则化保证 Hessian 正定
 *
 * 与旧版（4 维状态）的差异：
 *   - r_ddelta 真正进入 R[1][1] 的二阶项，iLQR 求解器显式感知转向速率代价
 *   - δ 绝对值通过 Q[4][4]=q_delta 的状态代价约束，约束更严格
 *   - max_dsteer 作用于 dδ（控制），max_steer 作用于 δ（状态）
 */

#include "mpc_controller.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

/* ── Internal state ─────────────────────────────────────────── */

struct MpcController {
    MpcConfig  cfg;

    /* 参考轨迹（插值表） */
    MpcRefPoint ref[MPC_MAX_REF_PTS];
    int         ref_n;

    /* 当前状态 */
    double  ego_x, ego_y, ego_heading, ego_speed;
    double  ego_delta;       /* 当前 δ（即 prev_steer） */

    /* 工作空间（避免重复分配） */
    double  X[MPC_MAX_HORIZON + 1][MPC_STATE_DIM];             /* 状态轨迹 [step][x,y,θ,v,δ] */
    double  U[MPC_MAX_HORIZON][MPC_CONTROL_DIM];               /* 控制轨迹 [step][a, dδ] */
    double  K[MPC_MAX_HORIZON][MPC_CONTROL_DIM][MPC_STATE_DIM]; /* 反馈增益 */
    double  k[MPC_MAX_HORIZON][MPC_CONTROL_DIM];               /* 前馈项 */
    double  P[MPC_STATE_DIM][MPC_STATE_DIM];                   /* 代价-to-go Hessian */
    double  p[MPC_STATE_DIM];                                  /* 代价-to-go 梯度 */
    double  A[MPC_STATE_DIM][MPC_STATE_DIM];                   /* 状态转移 Jacobian */
    double  B[MPC_STATE_DIM][MPC_CONTROL_DIM];                 /* 控制 Jacobian */
    double  Q[MPC_STATE_DIM][MPC_STATE_DIM];                   /* 运行代价 Hessian */
    double  R[MPC_CONTROL_DIM][MPC_CONTROL_DIM];               /* 控制代价 Hessian */
    double  dU[MPC_MAX_HORIZON][MPC_CONTROL_DIM];              /* 控制增量 */
};

/* ── Linear algebra helpers ─────────────────────────────────── */

static inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 2×2 matrix inverse */
static int mat2x2_inv(const double A[2][2], double Ainv[2][2]) {
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabs(det) < 1e-12) return -1;
    double inv_det = 1.0 / det;
    Ainv[0][0] =  A[1][1] * inv_det;
    Ainv[0][1] = -A[0][1] * inv_det;
    Ainv[1][0] = -A[1][0] * inv_det;
    Ainv[1][1] =  A[0][0] * inv_det;
    return 0;
}

/* 2×2 matrix multiply by 2×N: C[2][N] = A[2][2] * B[2][N] */
static void mat2x2_mul_2xN(const double A[2][2], const double B[2][MPC_STATE_DIM],
                            double C[2][MPC_STATE_DIM]) {
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < MPC_STATE_DIM; j++)
            C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j];
}

/* ── Kinematic bicycle model ────────────────────────────────── */

/**
 * 运动学自行车模型前向积分一步。
 * β = atan(tan(δ) * 0.5)  (后轴近似)
 */
static void bike_model_step(double x, double y, double theta, double v, double delta,
                            double a, double ddelta, double dt, double L,
                            double* nx, double* ny, double* ntheta, double* nv, double* ndelta) {
    double beta = atan(tan(delta) * 0.5);
    *nx     = x + v * cos(theta + beta) * dt;
    *ny     = y + v * sin(theta + beta) * dt;
    *ntheta = theta + v * sin(beta) / L * dt;
    *nv     = v + a * dt;
    *ndelta = delta + ddelta * dt;   /* δ 作为状态，由 dδ 积分 */
}

/**
 * 线性化 bicycle model：在 (θ, v, δ) 处计算 Jacobian A[5×5], B[5×2]
 *
 * 状态: [x, y, θ, v, δ]
 * 控制: [a, dδ]
 *
 * A 的非零项：
 *   A[0][0]=1, A[0][2]=-v*sin(θ+β)*dt, A[0][3]=cos(θ+β)*dt, A[0][4]=-v*sin(θ+β)*dbeta_ddelta*dt
 *   A[1][1]=1, A[1][2]= v*cos(θ+β)*dt, A[1][3]=sin(θ+β)*dt, A[1][4]= v*cos(θ+β)*dbeta_ddelta*dt
 *   A[2][2]=1, A[2][3]=sin(β)/L*dt,     A[2][4]=v*cos(β)*dbeta_ddelta/L*dt
 *   A[3][3]=1
 *   A[4][4]=1
 *
 * B 的非零项：
 *   B[3][0] = dt    (∂v'/∂a)
 *   B[4][1] = dt    (∂δ'/∂dδ)
 */
static void bike_model_linearize(double theta, double v, double delta,
                                  double dt, double L,
                                  double A[MPC_STATE_DIM][MPC_STATE_DIM],
                                  double B[MPC_STATE_DIM][MPC_CONTROL_DIM]) {
    double t = tan(delta);
    double beta = atan(t * 0.5);
    double cos_tb = cos(theta + beta);
    double sin_tb = sin(theta + beta);
    double cos_b  = cos(beta);
    double sin_b  = sin(beta);

    /* dβ/dδ = 0.5 * (1 + tan²δ) / (1 + 0.25 * tan²δ) */
    double dbeta_ddelta = 0.5 * (1.0 + t * t) / (1.0 + 0.25 * t * t);

    memset(A, 0, sizeof(double) * MPC_STATE_DIM * MPC_STATE_DIM);
    A[0][0] = 1.0;
    A[0][2] = -v * sin_tb * dt;
    A[0][3] =  cos_tb * dt;
    A[0][4] = -v * sin_tb * dbeta_ddelta * dt;
    A[1][1] = 1.0;
    A[1][2] =  v * cos_tb * dt;
    A[1][3] =  sin_tb * dt;
    A[1][4] =  v * cos_tb * dbeta_ddelta * dt;
    A[2][2] = 1.0;
    A[2][3] =  sin_b / L * dt;
    A[2][4] =  v * cos_b / L * dbeta_ddelta * dt;
    A[3][3] = 1.0;
    A[4][4] = 1.0;   /* δ' = δ + dδ*dt → ∂δ'/∂δ = 1 */

    memset(B, 0, sizeof(double) * MPC_STATE_DIM * MPC_CONTROL_DIM);
    B[3][0] = dt;   /* ∂v'/∂a */
    B[4][1] = dt;   /* ∂δ'/∂dδ */
}

/* ── Reference interpolation ────────────────────────────────── */

/**
 * 参考轨迹插值。参数 s ∈ [0, 1] 沿参考弧长比例。
 * 输出 (x, y, heading, speed, kappa)。
 * δ_ref 由调用方计算：δ_ref = atan(kappa * wheelbase)。
 */
static void ref_interpolate(const MpcController* mpc, double s, double* x, double* y,
                            double* heading, double* speed, double* kappa) {
    if (mpc->ref_n == 0) {
        *x = 0; *y = 0; *heading = 0; *speed = 0; *kappa = 0;
        return;
    }
    if (mpc->ref_n == 1 || s <= 0.0) {
        *x = mpc->ref[0].x; *y = mpc->ref[0].y;
        *heading = mpc->ref[0].heading;
        *speed = mpc->ref[0].speed; *kappa = mpc->ref[0].kappa;
        return;
    }
    int idx = (int)floor(s * (mpc->ref_n - 1));
    if (idx < 0) idx = 0;
    if (idx >= mpc->ref_n - 1) {
        idx = mpc->ref_n - 2;
        *x = mpc->ref[idx + 1].x; *y = mpc->ref[idx + 1].y;
        *heading = mpc->ref[idx + 1].heading;
        *speed = mpc->ref[idx + 1].speed; *kappa = mpc->ref[idx + 1].kappa;
        return;
    }
    double frac = s * (mpc->ref_n - 1) - idx;
    *x       = mpc->ref[idx].x       + frac * (mpc->ref[idx + 1].x       - mpc->ref[idx].x);
    *y       = mpc->ref[idx].y       + frac * (mpc->ref[idx + 1].y       - mpc->ref[idx].y);
    *heading = mpc->ref[idx].heading + frac * (mpc->ref[idx + 1].heading - mpc->ref[idx].heading);
    *speed   = mpc->ref[idx].speed   + frac * (mpc->ref[idx + 1].speed   - mpc->ref[idx].speed);
    *kappa   = mpc->ref[idx].kappa   + frac * (mpc->ref[idx + 1].kappa   - mpc->ref[idx].kappa);
}

/* ── Forward rollout ────────────────────────────────────────── */

/**
 * 前向 rollout：给定控制序列，积分状态并计算总代价。
 * 代价项：
 *   - 运行：q_x*ex² + q_y*ey² + q_theta*eh² + q_v*ev² + q_delta*eδ²
 *           + r_a*a² + r_ddelta*dδ²   ← r_ddelta 真正生效
 *   - 终端：qf_x*ex² + qf_y*ey² + qf_theta*eh² + qf_v*ev²
 *   - 软约束：速度越限、δ 越限的二次惩罚
 */
static double rollout(MpcController* mpc, const double U_seq[][MPC_CONTROL_DIM],
                      double X_seq[][MPC_STATE_DIM], double total_cost) {
    const MpcConfig* cfg = &mpc->cfg;
    double x = mpc->ego_x, y = mpc->ego_y;
    double theta = mpc->ego_heading, v = mpc->ego_speed;
    double delta = mpc->ego_delta;
    double cost = 0.0;
    (void)total_cost;

    X_seq[0][0] = x; X_seq[0][1] = y;
    X_seq[0][2] = theta; X_seq[0][3] = v;
    X_seq[0][4] = delta;

    for (int k = 0; k < cfg->horizon; k++) {
        double a      = U_seq[k][0];
        double ddelta = U_seq[k][1];

        /* 参考点 */
        double s = (double)k / (double)cfg->horizon;
        double rx, ry, rh, rv, rk;
        ref_interpolate(mpc, s, &rx, &ry, &rh, &rv, &rk);
        double delta_ref = atan(rk * cfg->wheelbase);

        /* 状态偏差 */
        double ex = x - rx, ey = y - ry;
        double eh = theta - rh;
        while (eh >  M_PI) eh -= 2.0 * M_PI;
        while (eh < -M_PI) eh += 2.0 * M_PI;
        double ev = v - rv;
        double edelta = delta - delta_ref;

        /* 运行代价（含 r_ddelta 的二阶项） */
        cost += cfg->q_x * ex * ex + cfg->q_y * ey * ey +
                cfg->q_theta * eh * eh + cfg->q_v * ev * ev +
                cfg->q_delta * edelta * edelta +
                cfg->r_a * a * a + cfg->r_ddelta * ddelta * ddelta;

        /* 前向积分 */
        bike_model_step(x, y, theta, v, delta, a, ddelta, cfg->dt, cfg->wheelbase,
                        &x, &y, &theta, &v, &delta);

        /* 速度软约束 */
        if (v < cfg->min_speed) cost += cfg->q_v * (cfg->min_speed - v) * (cfg->min_speed - v) * 10.0;
        if (v > cfg->max_speed) cost += cfg->q_v * (v - cfg->max_speed) * (v - cfg->max_speed) * 10.0;
        /* δ 软约束：超出 ±max_steer 时大权重拉回 */
        if (delta < -cfg->max_steer)
            cost += cfg->q_delta * (-cfg->max_steer - delta) * (-cfg->max_steer - delta) * 10.0;
        if (delta >  cfg->max_steer)
            cost += cfg->q_delta * (delta - cfg->max_steer) * (delta - cfg->max_steer) * 10.0;

        X_seq[k + 1][0] = x; X_seq[k + 1][1] = y;
        X_seq[k + 1][2] = theta; X_seq[k + 1][3] = v;
        X_seq[k + 1][4] = delta;
    }

    /* 终端代价（终端不约束 δ，故 qf 不含 δ） */
    {
        double s = 1.0;
        double rx, ry, rh, rv, rk;
        ref_interpolate(mpc, s, &rx, &ry, &rh, &rv, &rk);
        double ex = x - rx, ey = y - ry;
        double eh = theta - rh;
        while (eh >  M_PI) eh -= 2.0 * M_PI;
        while (eh < -M_PI) eh += 2.0 * M_PI;
        double ev = v - rv;
        cost += cfg->qf_x * ex * ex + cfg->qf_y * ey * ey +
                cfg->qf_theta * eh * eh + cfg->qf_v * ev * ev;
    }

    return cost;
}

/* ── iLQR backward pass ─────────────────────────────────────── */

/**
 * 反向传播：从终端到起点，计算反馈增益 K[k] 和前馈项 k[k]。
 *
 * 对每步 k：
 *   1. 在 (X[k], U[k]) 处线性化得 A, B
 *   2. 计算运行代价梯度 Qx = Q * dx，Qu = R * du
 *   3. BtP = Bᵀ * P (2×5)
 *   4. Quu = Bᵀ P B + R (2×2)，加正则化保证正定
 *   5. k[k] = -Quu⁻¹ * (Qu + Bᵀp)
 *   6. K[k] = -Quu⁻¹ * BtP
 *   7. 更新 P, p (Riccati)
 */
static void backward_pass(MpcController* mpc) {
    const MpcConfig* cfg = &mpc->cfg;
    int N = cfg->horizon;

    /* 终端代价 Hessian Qf (4×4，δ 终端不约束) */
    memset(mpc->P, 0, sizeof(mpc->P));
    mpc->P[0][0] = cfg->qf_x;
    mpc->P[1][1] = cfg->qf_y;
    mpc->P[2][2] = cfg->qf_theta;
    mpc->P[3][3] = cfg->qf_v;
    /* P[4][4] = 0: 终端 δ 不加权 */
    memset(mpc->p, 0, sizeof(mpc->p));

    /* 运行代价 Hessian Q (5×5) */
    memset(mpc->Q, 0, sizeof(mpc->Q));
    mpc->Q[0][0] = cfg->q_x;
    mpc->Q[1][1] = cfg->q_y;
    mpc->Q[2][2] = cfg->q_theta;
    mpc->Q[3][3] = cfg->q_v;
    mpc->Q[4][4] = cfg->q_delta;

    /* 控制代价 Hessian R (2×2) —— r_ddelta 真正进入二阶项 */
    memset(mpc->R, 0, sizeof(mpc->R));
    mpc->R[0][0] = cfg->r_a;
    mpc->R[1][1] = cfg->r_ddelta;

    for (int k = N - 1; k >= 0; k--) {
        /* 线性化在 (X[k], U[k]) 处 */
        double theta = mpc->X[k][2];
        double v     = mpc->X[k][3];
        double delta = mpc->X[k][4];   /* δ 状态 */

        bike_model_linearize(theta, v, delta, cfg->dt, cfg->wheelbase,
                             mpc->A, mpc->B);

        /* 参考 */
        double s = (double)k / (double)N;
        double rx, ry, rh, rv, rk;
        ref_interpolate(mpc, s, &rx, &ry, &rh, &rv, &rk);
        double delta_ref = atan(rk * cfg->wheelbase);

        /* 状态偏差 dx = X[k] - x_ref (5 维) */
        double dx[5] = {
            mpc->X[k][0] - rx,
            mpc->X[k][1] - ry,
            mpc->X[k][2] - rh,
            mpc->X[k][3] - rv,
            mpc->X[k][4] - delta_ref
        };
        while (dx[2] >  M_PI) dx[2] -= 2.0 * M_PI;
        while (dx[2] < -M_PI) dx[2] += 2.0 * M_PI;

        /* Qx = Q * dx (对角阵) */
        double Qx[5] = {
            mpc->Q[0][0] * dx[0],
            mpc->Q[1][1] * dx[1],
            mpc->Q[2][2] * dx[2],
            mpc->Q[3][3] * dx[3],
            mpc->Q[4][4] * dx[4]
        };

        /* Qu = R * u (对角阵) */
        double Qu[2] = {
            mpc->R[0][0] * mpc->U[k][0],
            mpc->R[1][1] * mpc->U[k][1]
        };

        /* BtP[i][j] = Σ_k B[k][i] * P[k][j], k=0..4 (2×5) */
        double BtP[2][5];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 5; j++) {
                BtP[i][j] = mpc->B[0][i] * mpc->P[0][j] +
                            mpc->B[1][i] * mpc->P[1][j] +
                            mpc->B[2][i] * mpc->P[2][j] +
                            mpc->B[3][i] * mpc->P[3][j] +
                            mpc->B[4][i] * mpc->P[4][j];
            }
        }

        /* Quu = Bᵀ P B + R (2×2) */
        double Quu[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                Quu[i][j] = BtP[i][0] * mpc->B[0][j] +
                            BtP[i][1] * mpc->B[1][j] +
                            BtP[i][2] * mpc->B[2][j] +
                            BtP[i][3] * mpc->B[3][j] +
                            BtP[i][4] * mpc->B[4][j];
            }
            Quu[i][i] += mpc->R[i][i];
        }

        /* 正则化 */
        double reg = 1e-6;
        Quu[0][0] += reg;
        Quu[1][1] += reg;

        /* Btp[i] = Σ_k B[k][i] * p[k], k=0..4 (2×1) */
        double Btp[2];
        Btp[0] = mpc->B[0][0] * mpc->p[0] + mpc->B[1][0] * mpc->p[1] +
                 mpc->B[2][0] * mpc->p[2] + mpc->B[3][0] * mpc->p[3] +
                 mpc->B[4][0] * mpc->p[4];
        Btp[1] = mpc->B[0][1] * mpc->p[0] + mpc->B[1][1] * mpc->p[1] +
                 mpc->B[2][1] * mpc->p[2] + mpc->B[3][1] * mpc->p[3] +
                 mpc->B[4][1] * mpc->p[4];

        /* Qu_total = Qu + Bᵀp */
        double Qu_total[2] = { Qu[0] + Btp[0], Qu[1] + Btp[1] };

        /* Quu_inv = Quu⁻¹ */
        double Quu_inv[2][2];
        if (mat2x2_inv(Quu, Quu_inv) != 0) {
            Quu_inv[0][0] = 1.0 / fmax(Quu[0][0], 1e-8);
            Quu_inv[0][1] = 0.0;
            Quu_inv[1][0] = 0.0;
            Quu_inv[1][1] = 1.0 / fmax(Quu[1][1], 1e-8);
        }

        /* k[k] = -Quu_inv * Qu_total (前馈) */
        mpc->k[k][0] = -(Quu_inv[0][0] * Qu_total[0] + Quu_inv[0][1] * Qu_total[1]);
        mpc->k[k][1] = -(Quu_inv[1][0] * Qu_total[0] + Quu_inv[1][1] * Qu_total[1]);

        /* K[k] = -Quu_inv * BtP (反馈, 2×5) */
        mat2x2_mul_2xN(Quu_inv, BtP, mpc->K[k]);
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 5; j++)
                mpc->K[k][i][j] = -mpc->K[k][i][j];

        /* ── 更新 P, p (Riccati) ── */

        /* AtP[i][j] = Σ_k A[k][i] * P[k][j], k=0..4 (5×5) */
        double AtP[5][5];
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                AtP[i][j] = mpc->A[0][i] * mpc->P[0][j] +
                            mpc->A[1][i] * mpc->P[1][j] +
                            mpc->A[2][i] * mpc->P[2][j] +
                            mpc->A[3][i] * mpc->P[3][j] +
                            mpc->A[4][i] * mpc->P[4][j];
            }
        }

        /* AtPA[i][j] = Σ_k AtP[i][k] * A[k][j], k=0..4 (5×5) */
        double AtPA[5][5];
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                AtPA[i][j] = AtP[i][0] * mpc->A[0][j] +
                             AtP[i][1] * mpc->A[1][j] +
                             AtP[i][2] * mpc->A[2][j] +
                             AtP[i][3] * mpc->A[3][j] +
                             AtP[i][4] * mpc->A[4][j];
            }
        }

        /* P_new = Q + AtPA + Kᵀ Quu K + Kᵀ BtP + BtPᵀ K */
        double P_new[5][5];
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
                P_new[i][j] = mpc->Q[i][j] + AtPA[i][j];

        /* Kᵀ Quu K (5×5) */
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                double sum = 0.0;
                for (int r = 0; r < 2; r++)
                    for (int c = 0; c < 2; c++)
                        sum += mpc->K[k][r][i] * Quu[r][c] * mpc->K[k][c][j];
                P_new[i][j] += sum;
            }
        }

        /* Kᵀ BtP + (BtP)ᵀ K (5×5) */
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                double sum = 0.0;
                for (int r = 0; r < 2; r++) {
                    sum += mpc->K[k][r][i] * BtP[r][j] +
                           BtP[r][i] * mpc->K[k][r][j];
                }
                P_new[i][j] += sum;
            }
        }

        memcpy(mpc->P, P_new, sizeof(mpc->P));

        /* Atp[i] = Σ_k A[k][i] * p[k], k=0..4 (5) */
        double Atp[5];
        for (int i = 0; i < 5; i++) {
            Atp[i] = mpc->A[0][i] * mpc->p[0] +
                     mpc->A[1][i] * mpc->p[1] +
                     mpc->A[2][i] * mpc->p[2] +
                     mpc->A[3][i] * mpc->p[3] +
                     mpc->A[4][i] * mpc->p[4];
        }

        /* p_new = Qx + Atp + Kᵀ(Qu_total + Quu k) + BtPᵀ k */
        double p_new[5];
        for (int i = 0; i < 5; i++) {
            double kn = Qx[i] + Atp[i];
            for (int r = 0; r < 2; r++) {
                kn += mpc->K[k][r][i] * (Qu_total[r] +
                        Quu[r][0] * mpc->k[k][0] + Quu[r][1] * mpc->k[k][1]);
                kn += BtP[r][i] * mpc->k[k][r];
            }
            p_new[i] = kn;
        }
        memcpy(mpc->p, p_new, sizeof(mpc->p));
    }
}

/* ── Forward pass (line search) ─────────────────────────────── */

/**
 * 前向 pass：线搜索一次，应用 K[k] 和 k[k] 更新控制序列。
 *
 * 控制约束：
 *   a      ∈ [-max_decel, max_accel]
 *   dδ     ∈ [-max_dsteer, max_dsteer]   (rad/s)
 * 状态约束：
 *   δ      ∈ [-max_steer, max_steer]      (rad)  —— 通过限幅 dδ 实现
 *   v      ∈ [min_speed, max_speed]
 */
static double forward_pass(MpcController* mpc, double alpha) {
    const MpcConfig* cfg = &mpc->cfg;

    /* 保存基准轨迹副本 */
    double X_save[MPC_MAX_HORIZON + 1][MPC_STATE_DIM];
    double U_save[MPC_MAX_HORIZON][MPC_CONTROL_DIM];
    memcpy(X_save, mpc->X, sizeof(X_save));
    memcpy(U_save, mpc->U, sizeof(U_save));

    double x  = mpc->ego_x, y  = mpc->ego_y;
    double th = mpc->ego_heading, v = mpc->ego_speed;
    double delta = mpc->ego_delta;   /* 当前 δ 状态，由 ego_delta 起步 */

    mpc->X[0][0] = x; mpc->X[0][1] = y;
    mpc->X[0][2] = th; mpc->X[0][3] = v;
    mpc->X[0][4] = delta;

    for (int k = 0; k < cfg->horizon; k++) {
        /* 状态偏差 dx = X[k] - X_save[k] (5 维) */
        double dx[5] = {
            mpc->X[k][0] - X_save[k][0],
            mpc->X[k][1] - X_save[k][1],
            mpc->X[k][2] - X_save[k][2],
            mpc->X[k][3] - X_save[k][3],
            mpc->X[k][4] - X_save[k][4]
        };
        while (dx[2] >  M_PI) dx[2] -= 2.0 * M_PI;
        while (dx[2] < -M_PI) dx[2] += 2.0 * M_PI;

        /* 控制增量 du = α * k[k] + K[k] * dx (2 维) */
        double du[2];
        du[0] = alpha * mpc->k[k][0] +
                mpc->K[k][0][0] * dx[0] + mpc->K[k][0][1] * dx[1] +
                mpc->K[k][0][2] * dx[2] + mpc->K[k][0][3] * dx[3] +
                mpc->K[k][0][4] * dx[4];
        du[1] = alpha * mpc->k[k][1] +
                mpc->K[k][1][0] * dx[0] + mpc->K[k][1][1] * dx[1] +
                mpc->K[k][1][2] * dx[2] + mpc->K[k][1][3] * dx[3] +
                mpc->K[k][1][4] * dx[4];

        /* 新控制 = 旧控制 + 增量 */
        double a_new      = U_save[k][0] + du[0];
        double ddelta_new = U_save[k][1] + du[1];

        /* 控制约束 */
        a_new      = clamp(a_new, -cfg->max_decel, cfg->max_accel);
        ddelta_new = clamp(ddelta_new, -cfg->max_dsteer, cfg->max_dsteer);

        /* δ 状态约束：限制下一帧 δ = δ_prev + dδ*dt 不超出 ±max_steer。
         * 若 dδ 会导致 δ 越限，按比例缩减 dδ。 */
        double delta_next = delta + ddelta_new * cfg->dt;
        if (delta_next >  cfg->max_steer) {
            delta_next = cfg->max_steer;
            ddelta_new = (delta_next - delta) / cfg->dt;
        } else if (delta_next < -cfg->max_steer) {
            delta_next = -cfg->max_steer;
            ddelta_new = (delta_next - delta) / cfg->dt;
        }
        delta = delta_next;

        mpc->U[k][0] = a_new;
        mpc->U[k][1] = ddelta_new;

        /* 前向积分 */
        bike_model_step(x, y, th, v, delta, a_new, ddelta_new, cfg->dt, cfg->wheelbase,
                        &x, &y, &th, &v, &delta);

        /* 速度约束 */
        v = clamp(v, cfg->min_speed, cfg->max_speed);

        mpc->X[k + 1][0] = x; mpc->X[k + 1][1] = y;
        mpc->X[k + 1][2] = th; mpc->X[k + 1][3] = v;
        mpc->X[k + 1][4] = delta;
    }

    /* 计算总代价 */
    return rollout(mpc, mpc->U, mpc->X, 0.0);
}

/* ── Public API ──────────────────────────────────────────────── */

MpcConfig mpc_default_config(void) {
    MpcConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.horizon         = 10;
    cfg.dt              = MPC_DEFAULT_DT;
    cfg.q_x             = 1.0;
    cfg.q_y             = 8.0;     /* cross-track 权重高 */
    cfg.q_theta         = 3.0;
    cfg.q_v             = 2.0;
    cfg.q_delta         = 2.0;     /* δ 状态权重（原 r_delta 默认值） */
    cfg.qf_x            = 2.0;
    cfg.qf_y            = 16.0;
    cfg.qf_theta        = 6.0;
    cfg.qf_v            = 4.0;
    cfg.r_a             = 0.1;     /* 允许快速加减速 */
    cfg.r_ddelta        = 5.0;     /* dδ 速率惩罚高，抑制振荡 */
    cfg.max_accel       = 3.0;
    cfg.max_decel       = 5.0;
    cfg.max_steer       = 0.35;    /* ~20° */
    cfg.max_dsteer      = 0.5;     /* rad/s */
    cfg.max_speed       = 30.0;
    cfg.min_speed       = -3.0;    /* 允许轻微倒车 */
    cfg.wheelbase       = 2.7;
    cfg.convergence_tol = 1e-4;
    cfg.line_search_c   = 0.5;
    cfg.max_iter        = MPC_MAX_ITER;
    return cfg;
}

MpcController* mpc_create(const MpcConfig* cfg) {
    MpcController* mpc = (MpcController*)calloc(1, sizeof(MpcController));
    if (!mpc) return NULL;
    if (cfg) {
        memcpy(&mpc->cfg, cfg, sizeof(MpcConfig));
    } else {
        mpc->cfg = mpc_default_config();
    }
    memset(mpc->U, 0, sizeof(mpc->U));
    memset(mpc->X, 0, sizeof(mpc->X));
    return mpc;
}

void mpc_destroy(MpcController* mpc) {
    if (mpc) free(mpc);
}

void mpc_set_reference(MpcController* mpc,
                       const MpcRefPoint* ref_points,
                       int n_points) {
    if (!mpc || !ref_points || n_points <= 0) return;
    int n = n_points < MPC_MAX_REF_PTS ? n_points : MPC_MAX_REF_PTS;
    memcpy(mpc->ref, ref_points, n * sizeof(MpcRefPoint));
    mpc->ref_n = n;
}

void mpc_set_state(MpcController* mpc,
                   double x, double y,
                   double heading, double speed) {
    if (!mpc) return;
    mpc->ego_x = x;
    mpc->ego_y = y;
    mpc->ego_heading = heading;
    mpc->ego_speed = speed;
}

void mpc_set_prev_steer(MpcController* mpc, double steer) {
    if (!mpc) return;
    mpc->ego_delta = steer;   /* δ 状态初值 */
}

int mpc_solve(MpcController* mpc, MpcResult* result) {
    if (!mpc || !result) return -1;
    const MpcConfig* cfg = &mpc->cfg;

    memset(result, 0, sizeof(MpcResult));

    /* 首次调用：热启动为 0 */
    bool cold_start = true;
    for (int k = 0; k < cfg->horizon; k++) {
        if (mpc->U[k][0] != 0.0 || mpc->U[k][1] != 0.0) {
            cold_start = false;
            break;
        }
    }
    if (cold_start) {
        memset(mpc->U, 0, sizeof(mpc->U));
    }

    /* 初始 rollout */
    double prev_cost = rollout(mpc, mpc->U, mpc->X, 0.0);

    int iter;
    for (iter = 0; iter < cfg->max_iter; iter++) {
        backward_pass(mpc);

        /* 线搜索 */
        double alpha = 1.0;
        double new_cost = prev_cost;
        int ls_iters = 0;
        for (; ls_iters < 10; ls_iters++) {
            new_cost = forward_pass(mpc, alpha);
            if (new_cost < prev_cost || alpha < 1e-8) break;
            alpha *= cfg->line_search_c;
        }

        /* 收敛检查 */
        if (fabs(prev_cost - new_cost) < cfg->convergence_tol) {
            prev_cost = new_cost;
            break;
        }
        prev_cost = new_cost;
    }

    /* 输出：第一帧控制量 a 和 dδ；δ 状态从 X[1] 取（已积分） */
    result->accel_cmd  = clamp(mpc->U[0][0], -cfg->max_decel, cfg->max_accel);
    result->steer_rate = clamp(mpc->U[0][1], -cfg->max_dsteer, cfg->max_dsteer);
    /* δ 输出取积分后第一帧状态（更接近真实执行值） */
    result->steer      = clamp(mpc->X[1][4], -cfg->max_steer, cfg->max_steer);

    /* 加速度 → throttle/brake */
    if (result->accel_cmd >= 0.0) {
        result->throttle = clamp(result->accel_cmd / cfg->max_accel, 0.0, 1.0);
        result->brake    = 0.0;
    } else {
        result->throttle = 0.0;
        result->brake    = clamp(-result->accel_cmd / cfg->max_decel, 0.0, 1.0);
    }

    result->iterations = iter;
    result->converged  = (iter < cfg->max_iter);
    result->cost       = prev_cost;

    /* 复制预测轨迹 */
    for (int k = 0; k < cfg->horizon; k++) {
        result->predicted_traj[k][0] = mpc->X[k + 1][0];
        result->predicted_traj[k][1] = mpc->X[k + 1][1];
        result->predicted_traj[k][2] = mpc->X[k + 1][2];
        result->predicted_traj[k][3] = mpc->X[k + 1][3];
        result->predicted_traj[k][4] = mpc->X[k + 1][4];
    }

    return 0;
}
