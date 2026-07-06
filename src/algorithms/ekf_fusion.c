/**
 * ekf_fusion.c — Extended Kalman Filter implementation.
 *
 * 所有矩阵运算均内联展开以适配 5×5 的小维度，
 * 无需 BLAS/LAPACK 等外部库。
 */

#include "ekf_fusion.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── 默认噪声参数 ────────────────────────────────────────────── */

/* 过程噪声: 加速度方差 2.0 m²/s⁴, 偏航角加速度方差 0.5 rad²/s⁴ */
#define DEFAULT_Q_ACCEL_VAR      2.0
#define DEFAULT_Q_YAWRATE_VAR    0.01

/* LiDAR 位置测量噪声: 0.25 m² */
#define DEFAULT_R_LIDAR_VAR      0.25

/* GPS 速度噪声: 1.0 m²/s², 航向噪声: 0.02 rad² */
#define DEFAULT_R_GPS_V_VAR      1.0
#define DEFAULT_R_GPS_H_VAR      0.02

/* GPS 位置噪声 (用于 full update): 9.0 m² */
#define DEFAULT_R_GPS_POS_VAR    9.0

/* 初始协方差对角值 */
#define INIT_P_POS_VAR           100.0   /* 位置 */
#define INIT_P_VEL_VAR           25.0    /* 速度 */
#define INIT_P_HEAD_VAR          1.0     /* 航向 (rad²) */
#define INIT_P_YR_VAR            0.1     /* 偏航角速度 */

/* ══════════════════════════════════════════════════════════ */
/*  内部: 小型矩阵运算                                        */
/* ══════════════════════════════════════════════════════════ */

/* C = alpha*A + beta*B,  all n×n row-major */
static void mat_add_5(double* C, const double* A, const double* B,
                       double alpha, double beta) {
    for (int i = 0; i < 25; i++)
        C[i] = alpha * A[i] + beta * B[i];
}

/* C = A * B,  n×n row-major */
static void mat_mul_5(double* C, const double* A, const double* B) {
    double tmp[25] = {0};
    for (int i = 0; i < 5; i++)
        for (int k = 0; k < 5; k++)
            for (int j = 0; j < 5; j++)
                tmp[i*5 + j] += A[i*5 + k] * B[k*5 + j];
    memcpy(C, tmp, sizeof(tmp));
}

/* F = Jacobian of state transition, 5×5 */
static void compute_jacobian_F(double* F, const double x[5], double dt) {
    double v   = x[2];
    double psi = x[3];
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    /* Identity */
    memset(F, 0, 25 * sizeof(double));
    for (int i = 0; i < 5; i++) F[i*5 + i] = 1.0;

    F[0*5 + 2] = cos_psi * dt;           /* ∂x'/∂v */
    F[0*5 + 3] = -v * sin_psi * dt;      /* ∂x'/∂ψ */
    F[1*5 + 2] = sin_psi * dt;           /* ∂y'/∂v */
    F[1*5 + 3] =  v * cos_psi * dt;      /* ∂y'/∂ψ */
    F[3*5 + 4] = dt;                     /* ∂ψ'/∂ψ̇ */
}

/* H = Jacobian of measurement function; R = measurement noise covariance */
/* Variant: LiDAR position (2D measurement → partial 5D state) */
typedef struct {
    double H[2 * 5];  /* 2×5, row-major */
    double R[2 * 2];  /* 2×2 */
    double K[5 * 2];  /* 5×2 Kalman gain */
    double S[2 * 2];  /* 2×2 innovation covariance */
    double y[2];      /* innovation vector */
} EkfUpdate2D;

/* Variant: GPS velocity+heading (2D measurement) */
typedef struct {
    double H[2 * 5];
    double R[2 * 2];
    double K[5 * 2];
    double S[2 * 2];
    double y[2];
} EkfUpdateGps2D;

/* Variant: GPS full (4D: x, y, v, heading) */
typedef struct {
    double H[4 * 5];
    double R[4 * 4];
    double K[5 * 4];
    double S[4 * 4];
    double y[4];
} EkfUpdate4D;

/* ── 通用 EKF 更新: z = h(x), H = dh/dx, dim z = m ── */
static void ekf_update_generic(EkfFusion* ekf,
                                const double* y,    /* innovation: z - h(x),  size m */
                                const double* H,    /* jacobian, m×5 row-major */
                                const double* R,    /* noise cov, m×m row-major */
                                int m) {
    /* ── S = H*P*Hᵀ + R ── */
    /* 计算 P*Hᵀ → 5×m, 存在 K 里临时 */
    double PHT[5 * 4]; /* max m=4 */
    memset(PHT, 0, sizeof(PHT));
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < 5; k++)
                PHT[i*m + j] += ekf->P[i*5 + k] * H[j*5 + k]; /* H[j*5+k] = H[j][k] */

    double S[4 * 4] = {0};
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++) {
            for (int k = 0; k < 5; k++)
                S[i*m + j] += H[i*5 + k] * PHT[k*m + j];
            S[i*m + j] += R[i*m + j];
        }

    /* ── K = P*Hᵀ * S⁻¹ (5×m) ── */
    /* 对 m≤4 直接求 S⁻¹ (小矩阵解析求逆) */
    double Sinv[4 * 4] = {0};
    if (m == 1) {
        Sinv[0] = 1.0 / S[0];
    } else if (m == 2) {
        double det = S[0]*S[3] - S[1]*S[2];
        if (fabs(det) < 1e-12) return;
        Sinv[0] =  S[3] / det;
        Sinv[1] = -S[1] / det;
        Sinv[2] = -S[2] / det;
        Sinv[3] =  S[0] / det;
    } else if (m == 4) {
        /* 4×4 block inverse via cofactors (for small well-conditioned S) */
        /* 用 Gauss-Jordan 消元 (augmented matrix) */
        double A[4 * 8] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) A[i*8 + j] = S[i*4 + j];
            A[i*8 + 4 + i] = 1.0;
        }
        for (int col = 0; col < 4; col++) {
            /* pivot */
            double piv = A[col*8 + col];
            if (fabs(piv) < 1e-10) continue;
            for (int j = 0; j < 8; j++) A[col*8 + j] /= piv;
            for (int row = 0; row < 4; row++) {
                if (row == col) continue;
                double factor = A[row*8 + col];
                for (int j = 0; j < 8; j++) A[row*8 + j] -= factor * A[col*8 + j];
            }
        }
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                Sinv[i*4 + j] = A[i*8 + 4 + j];
    }

    /* K = PHT * Sinv */
    double K[5 * 4] = {0};
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < m; j++)
            for (int k = 0; k < m; k++)
                K[i*m + j] += PHT[i*m + k] * Sinv[k*m + j];

    /* ── x̂' = x̂ + K*y ── */
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < m; j++)
            ekf->x[i] += K[i*m + j] * y[j];

    /* ── P' = (I - K*H) * P ── */
    /* I - K*H */
    double IKH[5 * 5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double s = (i == j) ? 1.0 : 0.0;
            for (int k = 0; k < m; k++)
                s -= K[i*m + k] * H[k*5 + j];
            IKH[i*5 + j] = s;
        }
    }

    /* P' = IKH * P */
    double Pnew[25] = {0};
    for (int i = 0; i < 5; i++)
        for (int k = 0; k < 5; k++)
            for (int j = 0; j < 5; j++)
                Pnew[i*5 + j] += IKH[i*5 + k] * ekf->P[k*5 + j];
    memcpy(ekf->P, Pnew, sizeof(Pnew));

    /* innovation norm */
    double inno_norm = 0;
    for (int i = 0; i < m; i++) inno_norm += y[i] * y[i];
    ekf->last_innovation = sqrt(inno_norm);

    /* divergence check */
    double trace = 0;
    for (int i = 0; i < 5; i++) trace += ekf->P[i*5 + i];
    if (trace > 1e9) ekf->diverged = 1;

    ekf->update_count++;
}

/* ══════════════════════════════════════════════════════════ */
/*  Public API                                                */
/* ══════════════════════════════════════════════════════════ */

void ekf_fusion_init(EkfFusion* ekf, double dt, const double x0[EKF_STATE_DIM]) {
    memset(ekf, 0, sizeof(*ekf));
    ekf->dt = dt;

    /* 初始状态 */
    if (x0) {
        memcpy(ekf->x, x0, EKF_STATE_DIM * sizeof(double));
    } else {
        ekf->x[0] = 0.0;   /* x */
        ekf->x[1] = 0.0;   /* y */
        ekf->x[2] = 5.0;   /* v = 5 m/s */
        ekf->x[3] = 0.0;   /* heading = 0 rad (east) */
        ekf->x[4] = 0.0;   /* yaw_rate = 0 */
    }

    /* 初始协方差 (对角, 大值表示初始不确定性) */
    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[0*5 + 0] = INIT_P_POS_VAR;
    ekf->P[1*5 + 1] = INIT_P_POS_VAR;
    ekf->P[2*5 + 2] = INIT_P_VEL_VAR;
    ekf->P[3*5 + 3] = INIT_P_HEAD_VAR;
    ekf->P[4*5 + 4] = INIT_P_YR_VAR;

    /* 过程噪声 Q (对角) */
    memset(ekf->Q, 0, sizeof(ekf->Q));
    /* 加速度噪声通过 dt 影响位置和速度 */
    double dt2 = dt * dt;
    double dt3 = dt2 * dt / 2.0;
    double dt4 = dt2 * dt2 / 4.0;
    /* x 方差来自加速度噪声 */
    ekf->Q[0*5 + 0] = dt4 * DEFAULT_Q_ACCEL_VAR;
    ekf->Q[0*5 + 2] = dt3 * DEFAULT_Q_ACCEL_VAR;
    ekf->Q[2*5 + 0] = dt3 * DEFAULT_Q_ACCEL_VAR;
    ekf->Q[2*5 + 2] = dt2 * DEFAULT_Q_ACCEL_VAR;
    /* y 同上 */
    ekf->Q[1*5 + 1] = dt4 * DEFAULT_Q_ACCEL_VAR;
    ekf->Q[1*5 + 2] = dt3 * DEFAULT_Q_ACCEL_VAR;
    /* heading 来自 yaw rate 噪声 */
    ekf->Q[3*5 + 3] = dt2 * DEFAULT_Q_YAWRATE_VAR;
    ekf->Q[3*5 + 4] = dt  * DEFAULT_Q_YAWRATE_VAR;
    ekf->Q[4*5 + 3] = dt  * DEFAULT_Q_YAWRATE_VAR;
    ekf->Q[4*5 + 4] =       DEFAULT_Q_YAWRATE_VAR;
}

void ekf_fusion_predict(EkfFusion* ekf) {
    double  v     = ekf->x[2];
    double  psi   = ekf->x[3];
    double  yr    = ekf->x[4];
    double  dt    = ekf->dt;

    /* ── 运动学预测 ── */
    double x_pred[5];
    x_pred[0] = ekf->x[0] + v * cos(psi) * dt;
    x_pred[1] = ekf->x[1] + v * sin(psi) * dt;
    x_pred[2] = v;                        /* v 恒定 (加速度通过 Q 注入) */
    x_pred[3] = psi + yr * dt;
    x_pred[4] = yr;                       /* yaw_rate 恒定 */

    /* Jacobian F = ∂f/∂x */
    double F[25];
    compute_jacobian_F(F, ekf->x, dt);

    /* ── P' = F·P·Fᵀ + Q ── */
    /* P_temp = F * P */
    double FP[25] = {0};
    for (int i = 0; i < 5; i++)
        for (int k = 0; k < 5; k++)
            for (int j = 0; j < 5; j++)
                FP[i*5 + j] += F[i*5 + k] * ekf->P[k*5 + j];

    /* P_new = FP * Fᵀ + Q */
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < 5; i++)
        for (int k = 0; k < 5; k++)
            for (int j = 0; j < 5; j++)
                ekf->P[i*5 + j] += FP[i*5 + k] * F[j*5 + k]; /* F[j*5+k] = Fᵀ[k][j] */

    for (int i = 0; i < 25; i++) ekf->P[i] += ekf->Q[i];

    /* ── 更新状态 ── */
    memcpy(ekf->x, x_pred, sizeof(x_pred));
    ekf->predict_count++;

    /* 航向归一化到 [-π, π]，防止无限累积导致下游（Frenet 规划器等）误判 */
    while (ekf->x[3] >  M_PI) ekf->x[3] -= 2.0 * M_PI;
    while (ekf->x[3] < -M_PI) ekf->x[3] += 2.0 * M_PI;
}

void ekf_fusion_update_lidar(EkfFusion* ekf,
                              double z_x, double z_y,
                              const double R[4]) {
    /* h(x) = [x, y]ᵀ → H = [[1,0,0,0,0], [0,1,0,0,0]] */
    double H[2 * 5] = {
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
    };

    double y[2] = {
        z_x - ekf->x[0],
        z_y - ekf->x[1],
    };

    double R_local[4];
    if (R) {
        memcpy(R_local, R, sizeof(R_local));
    } else {
        R_local[0] = DEFAULT_R_LIDAR_VAR;  R_local[1] = 0;
        R_local[2] = 0;                     R_local[3] = DEFAULT_R_LIDAR_VAR;
    }

    ekf_update_generic(ekf, y, H, R_local, 2);
}

void ekf_fusion_update_gps(EkfFusion* ekf,
                            double z_v, double z_heading,
                            const double R[4]) {
    /* h(x) = [v, heading]ᵀ → H = [[0,0,1,0,0], [0,0,0,1,0]] */
    double H[2 * 5] = {
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
    };

    /* 角度归一化: 保证 innovation 在 [-π, π] */
    double head_err = z_heading - ekf->x[3];
    while (head_err >  M_PI) head_err -= 2.0 * M_PI;
    while (head_err < -M_PI) head_err += 2.0 * M_PI;

    double y[2] = {
        z_v - ekf->x[2],
        head_err,
    };

    double R_local[4];
    if (R) {
        memcpy(R_local, R, sizeof(R_local));
    } else {
        R_local[0] = DEFAULT_R_GPS_V_VAR;  R_local[1] = 0;
        R_local[2] = 0;                     R_local[3] = DEFAULT_R_GPS_H_VAR;
    }

    ekf_update_generic(ekf, y, H, R_local, 2);
}

void ekf_fusion_update_gps_full(EkfFusion* ekf,
                                 double z_x, double z_y,
                                 double z_v, double z_heading) {
    /* h(x) = [x, y, v, heading]ᵀ → H = 4×5 */
    double H[4 * 5] = {
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
    };

    double head_err = z_heading - ekf->x[3];
    while (head_err >  M_PI) head_err -= 2.0 * M_PI;
    while (head_err < -M_PI) head_err += 2.0 * M_PI;

    double y[4] = {
        z_x - ekf->x[0],
        z_y - ekf->x[1],
        z_v - ekf->x[2],
        head_err,
    };

    double R[16] = {
        DEFAULT_R_GPS_POS_VAR, 0, 0, 0,
        0, DEFAULT_R_GPS_POS_VAR, 0, 0,
        0, 0, DEFAULT_R_GPS_V_VAR,  0,
        0, 0, 0, DEFAULT_R_GPS_H_VAR,
    };

    ekf_update_generic(ekf, y, H, R, 4);
}

void ekf_fusion_get_state(const EkfFusion* ekf,
                           double* x, double* y,
                           double* v, double* heading,
                           double* yaw_rate) {
    if (x)        *x        = ekf->x[0];
    if (y)        *y        = ekf->x[1];
    if (v)        *v        = ekf->x[2];
    if (heading)  *heading  = ekf->x[3];
    if (yaw_rate) *yaw_rate = ekf->x[4];
}

void ekf_fusion_get_covariance_diag(const EkfFusion* ekf, double diag[EKF_STATE_DIM]) {
    for (int i = 0; i < EKF_STATE_DIM; i++)
        diag[i] = ekf->P[i*5 + i];
}

void ekf_fusion_reset(EkfFusion* ekf) {
    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[0*5 + 0] = INIT_P_POS_VAR;
    ekf->P[1*5 + 1] = INIT_P_POS_VAR;
    ekf->P[2*5 + 2] = INIT_P_VEL_VAR;
    ekf->P[3*5 + 3] = INIT_P_HEAD_VAR;
    ekf->P[4*5 + 4] = INIT_P_YR_VAR;
    ekf->diverged = 0;
}
