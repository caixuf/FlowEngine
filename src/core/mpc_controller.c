/**
 * mpc_controller.c — 自行车模型 + 线性 MPC 求解器实现
 *
 * 算法：迭代线性化 MPC (iLQR 风格)
 *   1. 对当前状态线性化运动学模型
 *   2. 构建 LQR 代价 → 求解 Riccati 方程
 *   3. 线搜索 + 前向 rollout
 *   4. 重复直到收敛或达到最大迭代
 *
 * 数值稳定性：
 *   - 使用 Cholesky 分解（若失败则退化到对角近似）
 *   - 线搜索防止发散
 *   - 正则化保证 Hessian 正定
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
    double  prev_steer;

    /* 工作空间（避免重复分配） */
    double  X[MPC_MAX_HORIZON + 1][4];   /* 状态轨迹 [step][x,y,θ,v] */
    double  U[MPC_MAX_HORIZON][2];       /* 控制轨迹 [step][a,δ] */
    double  K[MPC_MAX_HORIZON][2][4];    /* 反馈增益 */
    double  k[MPC_MAX_HORIZON][2];       /* 前馈项 */
    double  P[4][4];                     /* 代价-to-go Hessian */
    double  p[4];                        /* 代价-to-go 梯度 */
    double  A[4][4];                     /* 状态转移 Jacobian */
    double  B[4][2];                     /* 控制 Jacobian */
    double  Q[4][4];                     /* 运行代价 Hessian */
    double  R[2][2];                     /* 控制代价 Hessian */
    double  dU[MPC_MAX_HORIZON][2];      /* 控制增量 */

    /* 参考轨迹在当前状态下的插值索引 */
    double  ref_s[MPC_MAX_HORIZON + 1][4]; /* [step][x_ref, y_ref, θ_ref, v_ref] */
};

/* ── Linear algebra helpers ─────────────────────────────────── */

static inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 2×2 matrix inverse (Cholesky or direct) */
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

/* 2×2 matrix multiply by 2×4: C[2][4] = A[2][2] * B[2][4] */
static void mat2x2_mul_2x4(const double A[2][2], const double B[2][4], double C[2][4]) {
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++)
            C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j];
}

/* 2×4 multiply by 4×1: C[2] = A[2][4] * b[4] */
static void mat2x4_mul_4x1(const double A[2][4], const double b[4], double c[2]) {
    c[0] = A[0][0]*b[0] + A[0][1]*b[1] + A[0][2]*b[2] + A[0][3]*b[3];
    c[1] = A[1][0]*b[0] + A[1][1]*b[1] + A[1][2]*b[2] + A[1][3]*b[3];
}

/* ── Kinematic bicycle model ────────────────────────────────── */

static void bike_model_step(double x, double y, double theta, double v,
                            double a, double delta, double dt, double L,
                            double* nx, double* ny, double* ntheta, double* nv) {
    double beta = atan(tan(delta) * 0.5); /* 简化: 后轴近似 */
    /* 完整运动学自行车模型 (后轴中心) */
    *nx     = x + v * cos(theta + beta) * dt;
    *ny     = y + v * sin(theta + beta) * dt;
    *ntheta = theta + v * sin(beta) / L * dt;
    *nv     = v + a * dt;
}

/* 线性化 bicycle model: 计算 Jacobian A, B 在 (x, θ, v, δ) 处 */
static void bike_model_linearize(double theta, double v, double delta,
                                  double dt, double L,
                                  double A[4][4], double B[4][2]) {
    double beta = atan(tan(delta) * 0.5);
    double cos_tb = cos(theta + beta);
    double sin_tb = sin(theta + beta);

    /* A = ∂f/∂x */
    memset(A, 0, sizeof(double) * 16);
    A[0][0] = 1.0;                     /* ∂x'/∂x */
    A[0][2] = -v * sin_tb * dt;        /* ∂x'/∂θ */
    A[0][3] = cos_tb * dt;             /* ∂x'/∂v */
    A[1][1] = 1.0;                     /* ∂y'/∂y */
    A[1][2] =  v * cos_tb * dt;        /* ∂y'/∂θ */
    A[1][3] = sin_tb * dt;             /* ∂y'/∂v */
    A[2][2] = 1.0;                     /* ∂θ'/∂θ */
    A[2][3] = sin(beta) / L * dt;      /* ∂θ'/∂v */
    A[3][3] = 1.0;                     /* ∂v'/∂v */

    /* B = ∂f/∂u, u = [a, δ] */
    memset(B, 0, sizeof(double) * 8);
    B[0][0] = 0.0;                     /* ∂x'/∂a = 0 (二阶效应) */
    B[1][0] = 0.0;
    B[2][0] = 0.0;
    B[3][0] = dt;                      /* ∂v'/∂a */

    /* ∂/∂δ */
    double dbeta = 0.5 / (cos(delta) * cos(delta) + sin(delta) * sin(delta));
    /* ∂beta/∂δ = 0.5 / (1 + tan²δ * 0.25) * (1 + tan²δ) ... 简化: */
    double dbeta_ddelta = 0.5 / (1.0 + 0.25 * tan(delta) * tan(delta));
    dbeta_ddelta *= (1.0 + tan(delta) * tan(delta)); /* d/dδ tanδ = 1+tan²δ */

    B[0][1] = -v * sin_tb * dbeta_ddelta * dt; /* ∂x'/∂δ */
    B[1][1] =  v * cos_tb * dbeta_ddelta * dt; /* ∂y'/∂δ */
    B[2][1] =  v * cos(beta) / L * dbeta_ddelta * dt; /* ∂θ'/∂δ */
    B[3][1] = 0.0;                     /* ∂v'/∂δ */
}

/* ── Reference interpolation ────────────────────────────────── */

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
    /* 线性插值：找最近两点 */
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

static double rollout(MpcController* mpc, const double U_seq[][2],
                      double X_seq[][4], double total_cost) {
    const MpcConfig* cfg = &mpc->cfg;
    double x = mpc->ego_x, y = mpc->ego_y;
    double theta = mpc->ego_heading, v = mpc->ego_speed;
    double cost = 0.0;

    X_seq[0][0] = x; X_seq[0][1] = y;
    X_seq[0][2] = theta; X_seq[0][3] = v;

    for (int k = 0; k < cfg->horizon; k++) {
        double a = U_seq[k][0], delta = U_seq[k][1];

        /* 参考点 */
        double s = (double)k / (double)cfg->horizon;
        double rx, ry, rh, rv, rk;
        ref_interpolate(mpc, s, &rx, &ry, &rh, &rv, &rk);

        /* 运行代价 */
        double ex = x - rx, ey = y - ry;
        double eh = theta - rh;
        /* 归一化航向角差到 [-π, π] */
        while (eh >  M_PI) eh -= 2.0 * M_PI;
        while (eh < -M_PI) eh += 2.0 * M_PI;
        double ev = v - rv;

        cost += cfg->q_x * ex * ex + cfg->q_y * ey * ey +
                cfg->q_theta * eh * eh + cfg->q_v * ev * ev +
                cfg->r_a * a * a + cfg->r_delta * delta * delta;

        /* 前向积分 */
        bike_model_step(x, y, theta, v, a, delta, cfg->dt, cfg->wheelbase,
                        &x, &y, &theta, &v);

        /* 控制约束惩罚 */
        if (v < cfg->min_speed) cost += cfg->q_v * (cfg->min_speed - v) * (cfg->min_speed - v) * 10.0;
        if (v > cfg->max_speed) cost += cfg->q_v * (v - cfg->max_speed) * (v - cfg->max_speed) * 10.0;

        X_seq[k + 1][0] = x; X_seq[k + 1][1] = y;
        X_seq[k + 1][2] = theta; X_seq[k + 1][3] = v;
    }

    /* 终端代价 */
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

static void backward_pass(MpcController* mpc) {
    const MpcConfig* cfg = &mpc->cfg;
    int N = cfg->horizon;

    /* 终端代价 */
    memset(mpc->P, 0, sizeof(mpc->P));
    mpc->P[0][0] = cfg->qf_x;
    mpc->P[1][1] = cfg->qf_y;
    mpc->P[2][2] = cfg->qf_theta;
    mpc->P[3][3] = cfg->qf_v;
    memset(mpc->p, 0, sizeof(mpc->p));

    /* 运行代价 Hessian */
    memset(mpc->Q, 0, sizeof(mpc->Q));
    mpc->Q[0][0] = cfg->q_x;
    mpc->Q[1][1] = cfg->q_y;
    mpc->Q[2][2] = cfg->q_theta;
    mpc->Q[3][3] = cfg->q_v;

    /* 控制代价 Hessian */
    memset(mpc->R, 0, sizeof(mpc->R));
    mpc->R[0][0] = cfg->r_a;
    mpc->R[1][1] = cfg->r_delta;

    for (int k = N - 1; k >= 0; k--) {
        /* 线性化在 (X[k], U[k]) 处 */
        double theta = mpc->X[k][2];
        double v     = mpc->X[k][3];
        double delta = mpc->U[k][1];

        bike_model_linearize(theta, v, delta, cfg->dt, cfg->wheelbase,
                             mpc->A, mpc->B);

        /* 参考 */
        double s = (double)k / (double)N;
        double rx, ry, rh, rv, rk;
        ref_interpolate(mpc, s, &rx, &ry, &rh, &rv, &rk);

        /* 状态偏差 */
        double dx[4] = {
            mpc->X[k][0] - rx,
            mpc->X[k][1] - ry,
            mpc->X[k][2] - rh,
            mpc->X[k][3] - rv
        };
        /* 归一化航向角偏差 */
        while (dx[2] >  M_PI) dx[2] -= 2.0 * M_PI;
        while (dx[2] < -M_PI) dx[2] += 2.0 * M_PI;

        /* Qx = Q * dx (状态代价梯度) */
        double Qx[4] = {
            mpc->Q[0][0] * dx[0],
            mpc->Q[1][1] * dx[1],
            mpc->Q[2][2] * dx[2],
            mpc->Q[3][3] * dx[3]
        };

        /* Qu = R * u (控制代价梯度) */
        double Qu[2] = {
            mpc->R[0][0] * mpc->U[k][0],
            mpc->R[1][1] * mpc->U[k][1]
        };

        /* BᵀP: 2×4 */
        double BtP[2][4];
        /* BtP[i][j] = Σ_k B[k][i] * P[k][j] */
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 4; j++)
                BtP[i][j] = mpc->B[0][i] * mpc->P[0][j] +
                            mpc->B[1][i] * mpc->P[1][j] +
                            mpc->B[2][i] * mpc->P[2][j] +
                            mpc->B[3][i] * mpc->P[3][j];

        /* BᵀP B + R: 2×2 */
        double Quu[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                Quu[i][j] = BtP[i][0] * mpc->B[0][j] +
                            BtP[i][1] * mpc->B[1][j] +
                            BtP[i][2] * mpc->B[2][j] +
                            BtP[i][3] * mpc->B[3][j];
            }
            Quu[i][i] += mpc->R[i][i]; /* + R */
        }

        /* 正则化：保证 Quu 正定 */
        double reg = 1e-6;
        Quu[0][0] += reg;
        Quu[1][1] += reg;

        /* Bᵀp: 2×1 */
        double Btp[2];
        Btp[0] = mpc->B[0][0] * mpc->p[0] + mpc->B[1][0] * mpc->p[1] +
                 mpc->B[2][0] * mpc->p[2] + mpc->B[3][0] * mpc->p[3];
        Btp[1] = mpc->B[0][1] * mpc->p[0] + mpc->B[1][1] * mpc->p[1] +
                 mpc->B[2][1] * mpc->p[2] + mpc->B[3][1] * mpc->p[3];

        /* Qu_total = Qu + Btp */
        double Qu_total[2] = { Qu[0] + Btp[0], Qu[1] + Btp[1] };

        /* Quu_inv = (Quu)^(-1) */
        double Quu_inv[2][2];
        if (mat2x2_inv(Quu, Quu_inv) != 0) {
            /* 退化：用对角近似 */
            Quu_inv[0][0] = 1.0 / fmax(Quu[0][0], 1e-8);
            Quu_inv[0][1] = 0.0;
            Quu_inv[1][0] = 0.0;
            Quu_inv[1][1] = 1.0 / fmax(Quu[1][1], 1e-8);
        }

        /* k = -Quu_inv * Qu_total (前馈) */
        mpc->k[k][0] = -(Quu_inv[0][0] * Qu_total[0] + Quu_inv[0][1] * Qu_total[1]);
        mpc->k[k][1] = -(Quu_inv[1][0] * Qu_total[0] + Quu_inv[1][1] * Qu_total[1]);

        /* K = -Quu_inv * BtP (反馈) */
        mat2x2_mul_2x4(Quu_inv, BtP, mpc->K[k]);
        mpc->K[k][0][0] = -mpc->K[k][0][0]; mpc->K[k][0][1] = -mpc->K[k][0][1];
        mpc->K[k][0][2] = -mpc->K[k][0][2]; mpc->K[k][0][3] = -mpc->K[k][0][3];
        mpc->K[k][1][0] = -mpc->K[k][1][0]; mpc->K[k][1][1] = -mpc->K[k][1][1];
        mpc->K[k][1][2] = -mpc->K[k][1][2]; mpc->K[k][1][3] = -mpc->K[k][1][3];

        /* 更新 P, p (Riccati) */
        /* AᵀP */
        double AtP[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                AtP[i][j] = mpc->A[0][i] * mpc->P[0][j] +
                            mpc->A[1][i] * mpc->P[1][j] +
                            mpc->A[2][i] * mpc->P[2][j] +
                            mpc->A[3][i] * mpc->P[3][j];

        /* AᵀP A: 4×4 */
        double AtPA[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                AtPA[i][j] = AtP[i][0] * mpc->A[0][j] +
                             AtP[i][1] * mpc->A[1][j] +
                             AtP[i][2] * mpc->A[2][j] +
                             AtP[i][3] * mpc->A[3][j];

        /* P_new = Q + AᵀPA + KᵀQuuK + KᵀBtP + BtPᵀK */
        double P_new[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                P_new[i][j] = mpc->Q[i][j] + AtPA[i][j];

        /* KᵀQuuK */
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int r = 0; r < 2; r++)
                    for (int c = 0; c < 2; c++)
                        P_new[i][j] += mpc->K[k][r][i] * Quu[r][c] * mpc->K[k][c][j];

        /* KᵀBtP + BtPᵀK */
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int r = 0; r < 2; r++) {
                    P_new[i][j] += mpc->K[k][r][i] * BtP[r][j] +
                                   BtP[r][i] * mpc->K[k][r][j];
                }

        memcpy(mpc->P, P_new, sizeof(mpc->P));

        /* p_new = Qx + Aᵀp + KᵀQu_total + KᵀQuu k + BtPᵀk */
        /* Aᵀp */
        double Atp[4];
        for (int i = 0; i < 4; i++)
            Atp[i] = mpc->A[0][i] * mpc->p[0] + mpc->A[1][i] * mpc->p[1] +
                     mpc->A[2][i] * mpc->p[2] + mpc->A[3][i] * mpc->p[3];

        double p_new[4];
        for (int i = 0; i < 4; i++) {
            p_new[i] = Qx[i] + Atp[i];
            /* Kᵀ(Qu_total + Quu k) */
            for (int r = 0; r < 2; r++) {
                p_new[i] += mpc->K[k][r][i] * (Qu_total[r] +
                            Quu[r][0] * mpc->k[k][0] + Quu[r][1] * mpc->k[k][1]);
            }
            /* BtPᵀk */
            p_new[i] += BtP[0][i] * mpc->k[k][0] + BtP[1][i] * mpc->k[k][1];
        }
        memcpy(mpc->p, p_new, sizeof(mpc->p));
    }
}

/* ── Forward pass (line search) ─────────────────────────────── */

static double forward_pass(MpcController* mpc, double alpha) {
    const MpcConfig* cfg = &mpc->cfg;

    /* 存储基准轨迹的副本 */
    double X_save[MPC_MAX_HORIZON + 1][4];
    double U_save[MPC_MAX_HORIZON][2];
    memcpy(X_save, mpc->X, sizeof(X_save));
    memcpy(U_save, mpc->U, sizeof(U_save));

    double x  = mpc->ego_x, y  = mpc->ego_y;
    double th = mpc->ego_heading, v = mpc->ego_speed;
    double prev_delta = mpc->prev_steer;

    mpc->X[0][0] = x; mpc->X[0][1] = y;
    mpc->X[0][2] = th; mpc->X[0][3] = v;

    for (int k = 0; k < cfg->horizon; k++) {
        /* 状态偏差 */
        double dx[4] = {
            mpc->X[k][0] - X_save[k][0],
            mpc->X[k][1] - X_save[k][1],
            mpc->X[k][2] - X_save[k][2],
            mpc->X[k][3] - X_save[k][3]
        };
        while (dx[2] >  M_PI) dx[2] -= 2.0 * M_PI;
        while (dx[2] < -M_PI) dx[2] += 2.0 * M_PI;

        /* 控制增量 = α * k + K * dx */
        double du[2];
        du[0] = alpha * mpc->k[k][0] + mpc->K[k][0][0] * dx[0] +
                mpc->K[k][0][1] * dx[1] + mpc->K[k][0][2] * dx[2] +
                mpc->K[k][0][3] * dx[3];
        du[1] = alpha * mpc->k[k][1] + mpc->K[k][1][0] * dx[0] +
                mpc->K[k][1][1] * dx[1] + mpc->K[k][1][2] * dx[2] +
                mpc->K[k][1][3] * dx[3];

        /* 新控制 = 旧控制 + 增量 */
        double a_new     = U_save[k][0] + du[0];
        double delta_new = U_save[k][1] + du[1];

        /* 控制约束 */
        a_new = clamp(a_new, -cfg->max_decel, cfg->max_accel);
        delta_new = clamp(delta_new, -cfg->max_steer, cfg->max_steer);

        /* 转向速率约束 */
        double ddelta = delta_new - prev_delta;
        double max_dd = cfg->max_dsteer * cfg->dt;
        ddelta = clamp(ddelta, -max_dd, max_dd);
        delta_new = prev_delta + ddelta;
        prev_delta = delta_new;

        mpc->U[k][0] = a_new;
        mpc->U[k][1] = delta_new;

        /* 前向积分 */
        bike_model_step(x, y, th, v, a_new, delta_new, cfg->dt, cfg->wheelbase,
                        &x, &y, &th, &v);

        /* 速度约束 */
        v = clamp(v, cfg->min_speed, cfg->max_speed);

        mpc->X[k + 1][0] = x; mpc->X[k + 1][1] = y;
        mpc->X[k + 1][2] = th; mpc->X[k + 1][3] = v;
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
    cfg.q_y             = 8.0;    /* cross-track 权重高 */
    cfg.q_theta         = 3.0;    /* 航向角误差 */
    cfg.q_v             = 2.0;    /* 速度跟踪 */
    cfg.qf_x            = 2.0;
    cfg.qf_y            = 16.0;
    cfg.qf_theta        = 6.0;
    cfg.qf_v            = 4.0;
    cfg.r_a             = 0.1;    /* 加速度变化惩罚小 = 允许快速加减速 */
    cfg.r_delta         = 0.5;    /* 转向角惩罚中等 */
    cfg.r_ddelta        = 0.3;    /* 转向角速率惩罚 */
    cfg.max_accel       = 3.0;
    cfg.max_decel       = 5.0;
    cfg.max_steer       = 0.35;   /* ~20° */
    cfg.max_dsteer      = 0.5;    /* rad/s */
    cfg.max_speed       = 30.0;
    cfg.min_speed       = -3.0;   /* 允许轻微倒车 */
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
    /* 初始化控制轨迹为 0 */
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
    mpc->prev_steer = steer;
}

int mpc_solve(MpcController* mpc, MpcResult* result) {
    if (!mpc || !result) return -1;
    const MpcConfig* cfg = &mpc->cfg;

    memset(result, 0, sizeof(MpcResult));

    /* 初始化控制轨迹 */
    if (mpc->U[0][0] == 0.0 && mpc->U[0][1] == 0.0) {
        /* 首次调用：热启动为 0 */
        memset(mpc->U, 0, sizeof(mpc->U));
    }

    /* 初始 rollout */
    double prev_cost = rollout(mpc, mpc->U, mpc->X, 0.0);

    int iter;
    for (iter = 0; iter < cfg->max_iter; iter++) {
        /* 反向传播 */
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

    /* 输出结果 */
    result->accel_cmd = clamp(mpc->U[0][0], -cfg->max_decel, cfg->max_accel);
    result->steer     = clamp(mpc->U[0][1], -cfg->max_steer, cfg->max_steer);

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
    }

    return 0;
}