/**
 * ltv_mpc.c — LTV MPC 横向控制器实现
 *
 * 绕轨迹线性化的误差动力学，离散 Riccati 递归求解。
 * 3 状态 [e_y, e_psi, delta], 1 控制 [ddelta].
 * N=20, dt=0.05s。
 *
 * 数值方法：Riccati 递归（3×3 矩阵 + 标量求逆），O(N) 时间。
 * 零动态分配，失败有确定返回码。
 */

#include "ltv_mpc.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <float.h>

/* ── 3×3 矩阵运算（极小规模，内联展开） ────────────────── */

static inline void mat33_mul(const double A[3][3], const double B[3][3],
                              double C[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double s = 0;
            for (int k = 0; k < 3; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

static inline void mat33_mul_v3(const double A[3][3], const double v[3],
                                 double out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
}

static inline void mat33_add(double C[3][3], const double A[3][3],
                              const double B[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) C[i][j] = A[i][j] + B[i][j];
}

static inline void vec3_add(double c[3], const double a[3], const double b[3]) {
    c[0] = a[0] + b[0]; c[1] = a[1] + b[1]; c[2] = a[2] + b[2];
}

static inline void vec3_scale(double out[3], const double v[3], double s) {
    out[0] = v[0]*s; out[1] = v[1]*s; out[2] = v[2]*s;
}

/* ── 求解器结构体 ─────────────────────────────────────────── */

struct LtvMpcSolver {
    LtvMpcConfig cfg;

    /* 参考轨迹 */
    double v_ref[LTV_MPC_MAX_HORIZON];
    double kappa_ref[LTV_MPC_MAX_HORIZON];
    int    ref_n;

    /* 当前状态 */
    double e_y, e_psi, delta, v;

    /* Riccati 状态 */
    double P[3][3];  /* 代价-to-go Hessian */
    double p[3];     /* 代价-to-go 梯度 */
    double K[LTV_MPC_MAX_HORIZON][3];  /* 反馈增益 (1×3) */
    double kff[LTV_MPC_MAX_HORIZON];   /* 前馈项 (标量) */

    LtvMpcStatus status;
};

/* ── 创建与销毁 ───────────────────────────────────────────── */

LtvMpcSolver* ltv_mpc_create(const LtvMpcConfig* cfg) {
    LtvMpcSolver* s = (LtvMpcSolver*)calloc(1, sizeof(LtvMpcSolver));
    if (!s) return NULL;
    if (cfg) memcpy(&s->cfg, cfg, sizeof(LtvMpcConfig));
    else    s->cfg = ltv_mpc_default_config();
    s->status = LTV_MPC_INIT;
    s->ref_n = 0;
    return s;
}

void ltv_mpc_destroy(LtvMpcSolver* solver) {
    free(solver);
}

void ltv_mpc_update_config(LtvMpcSolver* solver, const LtvMpcConfig* cfg) {
    if (!solver || !cfg) return;
    memcpy(&solver->cfg, cfg, sizeof(LtvMpcConfig));
}

void ltv_mpc_set_reference(LtvMpcSolver* solver,
                            const double* v_ref, const double* kappa_ref, int N) {
    if (!solver) return;
    int n = N > LTV_MPC_MAX_HORIZON ? LTV_MPC_MAX_HORIZON : N;
    for (int i = 0; i < n; i++) {
        solver->v_ref[i] = v_ref[i];
        solver->kappa_ref[i] = kappa_ref[i];
    }
    solver->ref_n = n;
}

void ltv_mpc_set_state(LtvMpcSolver* solver,
                        double e_y, double e_psi, double delta, double v) {
    if (!solver) return;
    solver->e_y = e_y;
    solver->e_psi = e_psi;
    solver->delta = delta;
    solver->v = v;
}

LtvMpcConfig ltv_mpc_default_config(void) {
    LtvMpcConfig cfg;
    cfg.q_y       = 10.0;
    cfg.q_psi     = 20.0;
    cfg.q_delta   = 2.0;
    cfg.r_ddelta  = 0.5;
    cfg.qf_y      = 20.0;
    cfg.qf_psi    = 40.0;
    cfg.horizon   = 20;
    cfg.dt        = 0.05;
    cfg.wheelbase = 2.7;
    cfg.max_steer = 0.35;
    cfg.max_dsteer = 0.5;
    return cfg;
}

LtvMpcStatus ltv_mpc_status(const LtvMpcSolver* solver) {
    return solver ? solver->status : LTV_MPC_DEGRADED;
}

/* ══════════════════════════════════════════════════════════ */
/*  MPC 求解                                                     */
/* ══════════════════════════════════════════════════════════ */

int ltv_mpc_solve(LtvMpcSolver* solver, double* steer_out) {
    if (!solver || !steer_out) return LTV_MPC_ERR_N;
    const LtvMpcConfig* cfg = &solver->cfg;
    int N = cfg->horizon;
    if (N < 1 || N > LTV_MPC_MAX_HORIZON) { solver->status = LTV_MPC_DEGRADED; return LTV_MPC_ERR_N; }
    double dt = cfg->dt;
    double L  = cfg->wheelbase;
    double max_steer  = cfg->max_steer;
    double max_dsteer = cfg->max_dsteer;

    /* 终端代价 P_N = Qf */
    memset(solver->P, 0, sizeof(solver->P));
    solver->P[0][0] = cfg->qf_y;
    solver->P[1][1] = cfg->qf_psi;
    solver->P[2][2] = 0.0;  /* delta 终端不约束 */

    memset(solver->p, 0, sizeof(solver->p));

    /* 运行代价 Q 和 R */
    double Q[3][3] = {{0}};
    Q[0][0] = cfg->q_y;
    Q[1][1] = cfg->q_psi;
    Q[2][2] = cfg->q_delta;
    double R = cfg->r_ddelta;

    /* 构造扩展的 A, B, c（考虑曲率前馈） */
    double A[3][3], B[3], c[3];

    /* 后向 Riccati 递归 (k=N-1 → 0) */
    for (int k = N - 1; k >= 0; k--) {
        double vk = (k < solver->ref_n) ? solver->v_ref[k] : solver->v;
        double kk = (k < solver->ref_n) ? solver->kappa_ref[k] : 0.0;
        double v_safe = (vk < 0.01) ? 0.01 : vk;

        /* 线性化 A, B, c */
        /* e_y[k+1] = e_y[k] + dt*(v*e_psi + 0.5*v*delta) */
        /* e_psi[k+1] = e_psi[k] + dt*(0.5*v/L*delta - kappa*v) */
        /* delta[k+1] = delta[k] + dt*ddelta */
        memset(A, 0, sizeof(A));
        A[0][0] = 1.0;  A[0][1] = v_safe * dt;  A[0][2] = 0.5 * v_safe * dt;
        A[1][1] = 1.0;  A[1][2] = 0.5 * v_safe / L * dt;
        A[2][2] = 1.0;

        B[0] = 0.0;  B[1] = 0.0;  B[2] = dt;

        c[0] = 0.0;
        c[1] = -kk * v_safe * dt;
        c[2] = 0.0;

        /* 计算增益 */
        /* Quu_inv = 1.0 / (R + B' * P * B) */
        double BPB = B[0]*(solver->P[0][0]*B[0] + solver->P[0][1]*B[1] + solver->P[0][2]*B[2])
                   + B[1]*(solver->P[1][0]*B[0] + solver->P[1][1]*B[1] + solver->P[1][2]*B[2])
                   + B[2]*(solver->P[2][0]*B[0] + solver->P[2][1]*B[1] + solver->P[2][2]*B[2]);
        double Quu = R + BPB;
        if (fabs(Quu) < 1e-12) { solver->status = LTV_MPC_DEGRADED; return LTV_MPC_ERR_SINGULAR; }
        double Quu_inv = 1.0 / Quu;

        /* K[k] = -Quu_inv * B' * P * A  (1×3) */
        double BtP[3];
        BtP[0] = B[0]*solver->P[0][0] + B[1]*solver->P[1][0] + B[2]*solver->P[2][0];
        BtP[1] = B[0]*solver->P[0][1] + B[1]*solver->P[1][1] + B[2]*solver->P[2][1];
        BtP[2] = B[0]*solver->P[0][2] + B[1]*solver->P[1][2] + B[2]*solver->P[2][2];

        for (int j = 0; j < 3; j++) {
            solver->K[k][j] = -Quu_inv * (BtP[0]*A[0][j] + BtP[1]*A[1][j] + BtP[2]*A[2][j]);
        }

        /* kff[k] = -Quu_inv * B' * (P * c + p) */
        double Pc_p[3];
        for (int i = 0; i < 3; i++)
            Pc_p[i] = solver->P[i][0]*c[0] + solver->P[i][1]*c[1] + solver->P[i][2]*c[2]
                      + solver->p[i];
        solver->kff[k] = -Quu_inv * (B[0]*Pc_p[0] + B[1]*Pc_p[1] + B[2]*Pc_p[2]);

        /* A_cl = A + B*K  (闭环) */
        double A_cl[3][3];
        memcpy(A_cl, A, sizeof(A));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                A_cl[i][j] += B[i] * solver->K[k][j];

        /* 更新 P_k = Q + A_cl' * P_{k+1} * A_cl + K' * R * K */
        double AtPA[3][3], KtRK[3][3];
        memset(AtPA, 0, sizeof(AtPA));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int ii = 0; ii < 3; ii++)
                    for (int jj = 0; jj < 3; jj++)
                        AtPA[i][j] += A_cl[ii][i] * solver->P[ii][jj] * A_cl[jj][j];

        memset(KtRK, 0, sizeof(KtRK));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                KtRK[i][j] = solver->K[k][i] * R * solver->K[k][j];

        double P_new[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                P_new[i][j] = Q[i][j] + AtPA[i][j] + KtRK[i][j];
        memcpy(solver->P, P_new, sizeof(P_new));

        /* 更新 p_k = ... + A_cl' * (P_{k+1} * c + p_{k+1}) + K' * R * kff */
        double Pc_p_next[3];
        for (int i = 0; i < 3; i++)
            Pc_p_next[i] = solver->P[i][0]*c[0] + solver->P[i][1]*c[1] + solver->P[i][2]*c[2];

        double p_new[3];
        memset(p_new, 0, sizeof(p_new));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                p_new[i] += A_cl[j][i] * Pc_p_next[j];

        double KtR_kff[3];
        for (int i = 0; i < 3; i++)
            KtR_kff[i] = solver->K[k][i] * R * solver->kff[k];
        vec3_add(p_new, p_new, KtR_kff);
        memcpy(solver->p, p_new, sizeof(p_new));
    }

    /* 前向 rollout：计算最优控制序列 */
    double x[3] = { solver->e_y, solver->e_psi, solver->delta };
    double best_u = 0.0;

    for (int k = 0; k < N; k++) {
        /* 控制律: u[k] = K[k] * x + kff[k] */
        double u = solver->K[k][0]*x[0] + solver->K[k][1]*x[1] + solver->K[k][2]*x[2]
                   + solver->kff[k];

        /* 限幅 */
        if (u >  max_dsteer) u =  max_dsteer;
        if (u < -max_dsteer) u = -max_dsteer;

        if (k == 0) best_u = u;

        /* 前向积分 */
        double vk = (k < solver->ref_n) ? solver->v_ref[k] : solver->v;
        double kk = (k < solver->ref_n) ? solver->kappa_ref[k] : 0.0;
        double v_safe = (vk < 0.01) ? 0.01 : vk;

        double x_next[3];
        x_next[0] = x[0] + dt * (v_safe * x[1] + 0.5 * v_safe * x[2]);
        x_next[1] = x[1] + dt * (0.5 * v_safe / L * x[2] - kk * v_safe);
        x_next[2] = x[2] + dt * u;

        /* delta 限幅 */
        if (x_next[2] >  max_steer) x_next[2] =  max_steer;
        if (x_next[2] < -max_steer) x_next[2] = -max_steer;

        memcpy(x, x_next, sizeof(x));

        /* NaN 检查 */
        if (isnan(x[0]) || isnan(x[1]) || isnan(x[2]) || isnan(u)) {
            solver->status = LTV_MPC_DEGRADED;
            return LTV_MPC_ERR_NAN;
        }
    }

    *steer_out = best_u;
    solver->status = LTV_MPC_ACTIVE;
    return LTV_MPC_OK;
}
