#include "ekf_slam.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



static void cov_mat_zero(EkfCovariance* P) {
    memset(P->data, 0, sizeof(P->data));
}

static void mat_mul(float* out, const float* A, const float* B, int rows, int cols, int inner) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            out[i * cols + j] = 0.0f;
            for (int k = 0; k < inner; k++) {
                out[i * cols + j] += A[i * inner + k] * B[k * cols + j];
            }
        }
    }
}

static void mat_add(float* out, const float* A, const float* B, int rows, int cols) {
    for (int i = 0; i < rows * cols; i++) {
        out[i] = A[i] + B[i];
    }
}

static void mat_transpose(float* out, const float* A, int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            out[j * rows + i] = A[i * cols + j];
        }
    }
}



void ekf_slam_init(EkfSlam* ekf, float initial_x, float initial_y, float initial_heading) {
    memset(ekf, 0, sizeof(*ekf));
    ekf->x.x = initial_x;
    ekf->x.y = initial_y;
    ekf->x.heading = initial_heading;
    ekf->x.v = 0.0f;
    ekf->x.omega = 0.0f;
    ekf->accel_bias = 0.0f;
    ekf->gyro_bias = 0.0f;
    
    cov_mat_zero(&ekf->P);
    ekf->P.data[0] = 0.1f;   
    ekf->P.data[6] = 0.1f;   
    ekf->P.data[12] = 0.001f;
    ekf->P.data[18] = 0.1f;  
    ekf->P.data[24] = 0.1f;  
    
    ekf->process_noise[0] = 0.001f; 
    ekf->process_noise[1] = 0.001f; 
    ekf->process_noise[2] = 0.001f; 
    ekf->process_noise[3] = 0.005f; 
    ekf->process_noise[4] = 0.005f; 
    
    ekf->measurement_noise = 0.5f;
    ekf->initialized = true;
    ekf->last_time_us = 0;
}

void ekf_slam_predict(EkfSlam* ekf, float accel_x, float gyro_z, uint64_t current_time_us) {
    if (!ekf->initialized) return;
    
    if (ekf->last_time_us == 0) {
        ekf->last_time_us = current_time_us;
        return;
    }
    
    float dt = (float)((double)(current_time_us - ekf->last_time_us) / 1000000.0);
    ekf->last_time_us = current_time_us;
    
    if (dt <= 0.0f || dt > 1.0f) return;
    
    float cos_h = cosf(ekf->x.heading);
    float sin_h = sinf(ekf->x.heading);
    
    ekf->x.x += ekf->x.v * cos_h * dt;
    ekf->x.y += ekf->x.v * sin_h * dt;
    ekf->x.heading += ekf->x.omega * dt;
    ekf->x.v += (accel_x - ekf->accel_bias) * dt;
    ekf->x.omega = gyro_z - ekf->gyro_bias;
    
    while (ekf->x.heading > (float)M_PI) ekf->x.heading -= 2.0f * (float)M_PI;
    while (ekf->x.heading < -(float)M_PI) ekf->x.heading += 2.0f * (float)M_PI;
    
    float F[EKF_COV_DIM] = {0};
    F[0] = 1.0f; F[1] = 0.0f; F[2] = 0.0f; F[3] = cos_h * dt; F[4] = 0.0f;
    F[5] = 0.0f; F[6] = 1.0f; F[7] = 0.0f; F[8] = sin_h * dt; F[9] = 0.0f;
    F[10] = 0.0f; F[11] = 0.0f; F[12] = 1.0f; F[13] = 0.0f; F[14] = dt;
    F[15] = 0.0f; F[16] = 0.0f; F[17] = 0.0f; F[18] = 1.0f; F[19] = 0.0f;
    F[20] = 0.0f; F[21] = 0.0f; F[22] = 0.0f; F[23] = 0.0f; F[24] = 1.0f;
    
    float F_trans[EKF_COV_DIM];
    mat_transpose(F_trans, F, EKF_STATE_DIM, EKF_STATE_DIM);
    
    float FP[EKF_COV_DIM];
    mat_mul(FP, F, ekf->P.data, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
    
    float FPF[EKF_COV_DIM];
    mat_mul(FPF, FP, F_trans, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
    
    float Q[EKF_COV_DIM] = {0};
    Q[0] = ekf->process_noise[0] * ekf->process_noise[0] * dt;
    Q[6] = ekf->process_noise[1] * ekf->process_noise[1] * dt;
    Q[12] = ekf->process_noise[2] * ekf->process_noise[2] * dt;
    Q[18] = ekf->process_noise[3] * ekf->process_noise[3] * dt;
    Q[24] = ekf->process_noise[4] * ekf->process_noise[4] * dt;
    
    mat_add(ekf->P.data, FPF, Q, EKF_STATE_DIM, EKF_STATE_DIM);
}

void ekf_slam_update(EkfSlam* ekf, float obs_x, float obs_y, float obs_heading) {
    if (!ekf->initialized) return;
    
    float H[15] = {
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f
    };
    
    float H_trans[15] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f
    };
    
    float PHt[15];
    mat_mul(PHt, ekf->P.data, H_trans, EKF_STATE_DIM, 3, EKF_STATE_DIM);
    
    float HPH[9];
    mat_mul(HPH, H, PHt, 3, 3, EKF_STATE_DIM);
    
    float R[9] = {0};
    float r = ekf->measurement_noise;
    R[0] = r * r; R[4] = r * r; R[8] = r * r * 0.1f;
    
    float S[9];
    mat_add(S, HPH, R, 3, 3);
    
    float S_inv[9];
    float det = S[0] * (S[4] * S[8] - S[5] * S[7]) - S[1] * (S[3] * S[8] - S[5] * S[6]) + S[2] * (S[3] * S[7] - S[4] * S[6]);
    if (fabsf(det) > 1e-10f) {
        float inv_det = 1.0f / det;
        S_inv[0] = (S[4] * S[8] - S[5] * S[7]) * inv_det;
        S_inv[1] = (S[2] * S[7] - S[1] * S[8]) * inv_det;
        S_inv[2] = (S[1] * S[5] - S[2] * S[4]) * inv_det;
        S_inv[3] = (S[5] * S[6] - S[3] * S[8]) * inv_det;
        S_inv[4] = (S[0] * S[8] - S[2] * S[6]) * inv_det;
        S_inv[5] = (S[2] * S[3] - S[0] * S[5]) * inv_det;
        S_inv[6] = (S[3] * S[7] - S[4] * S[6]) * inv_det;
        S_inv[7] = (S[1] * S[6] - S[0] * S[7]) * inv_det;
        S_inv[8] = (S[0] * S[4] - S[1] * S[3]) * inv_det;
    } else {
        for (int i = 0; i < 9; i++) S_inv[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    }
    
    float K[15] = {0};
    mat_mul(K, PHt, S_inv, EKF_STATE_DIM, 3, 3);
    
    float y[3] = {
        obs_x - ekf->x.x,
        obs_y - ekf->x.y,
        obs_heading - ekf->x.heading
    };
    
    float y_norm = y[2];
    while (y_norm > (float)M_PI) y_norm -= 2.0f * (float)M_PI;
    while (y_norm < -(float)M_PI) y_norm += 2.0f * (float)M_PI;
    y[2] = y_norm;
    
    float Ky[5];
    Ky[0] = K[0] * y[0] + K[1] * y[1] + K[2] * y[2];
    Ky[1] = K[3] * y[0] + K[4] * y[1] + K[5] * y[2];
    Ky[2] = K[6] * y[0] + K[7] * y[1] + K[8] * y[2];
    Ky[3] = K[9] * y[0] + K[10] * y[1] + K[11] * y[2];
    Ky[4] = K[12] * y[0] + K[13] * y[1] + K[14] * y[2];
    
    ekf->x.x += Ky[0];
    ekf->x.y += Ky[1];
    ekf->x.heading += Ky[2];
    ekf->x.v += Ky[3];
    ekf->x.omega += Ky[4];
    
    while (ekf->x.heading > (float)M_PI) ekf->x.heading -= 2.0f * (float)M_PI;
    while (ekf->x.heading < -(float)M_PI) ekf->x.heading += 2.0f * (float)M_PI;
    
    float KH[EKF_COV_DIM];
    mat_mul(KH, K, H, EKF_STATE_DIM, EKF_STATE_DIM, 3);
    
    float I_KH[EKF_COV_DIM] = {
        1.0f - KH[0], -KH[1], -KH[2], -KH[3], -KH[4],
        -KH[5], 1.0f - KH[6], -KH[7], -KH[8], -KH[9],
        -KH[10], -KH[11], 1.0f - KH[12], -KH[13], -KH[14],
        -KH[15], -KH[16], -KH[17], 1.0f - KH[18], -KH[19],
        -KH[20], -KH[21], -KH[22], -KH[23], 1.0f - KH[24]
    };
    
    float P_new[EKF_COV_DIM];
    mat_mul(P_new, I_KH, ekf->P.data, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);
    
    memcpy(ekf->P.data, P_new, sizeof(ekf->P.data));
}

void ekf_slam_get_pose(const EkfSlam* ekf, float* x, float* y, float* heading,
                       float* cov_xx, float* cov_yy, float* cov_hh) {
    if (!ekf->initialized) {
        *x = 0.0f; *y = 0.0f; *heading = 0.0f;
        *cov_xx = 1000.0f; *cov_yy = 1000.0f; *cov_hh = 100.0f;
        return;
    }
    *x = ekf->x.x;
    *y = ekf->x.y;
    *heading = ekf->x.heading;
    *cov_xx = ekf->P.data[0];
    *cov_yy = ekf->P.data[6];
    *cov_hh = ekf->P.data[12];
}
