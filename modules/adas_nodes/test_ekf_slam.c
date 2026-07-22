#include "ekf_slam.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    uint64_t timestamp_us;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MockImuData;

typedef struct {
    uint64_t timestamp_us;
    float speed;
    float steering_angle;
} MockWheelData;

static void generate_straight_line_motion(MockImuData* imu, MockWheelData* wheel, 
                                          int num_samples, float dt_us, float speed_mps) {
    for (int i = 0; i < num_samples; i++) {
        imu[i].timestamp_us = (uint64_t)(i * dt_us);
        imu[i].accel_x = 0.0f;
        imu[i].accel_y = 0.0f;
        imu[i].accel_z = -9.81f;
        imu[i].gyro_x = 0.0f;
        imu[i].gyro_y = 0.0f;
        imu[i].gyro_z = 0.0f;

        wheel[i].timestamp_us = (uint64_t)(i * dt_us);
        wheel[i].speed = speed_mps;
        wheel[i].steering_angle = 0.0f;
    }
}

static void generate_circular_motion(MockImuData* imu, MockWheelData* wheel,
                                     int num_samples, float dt_us, 
                                     float speed_mps, float radius_m) {
    float omega = speed_mps / radius_m;
    float centripetal_accel = speed_mps * speed_mps / radius_m;

    for (int i = 0; i < num_samples; i++) {
        imu[i].timestamp_us = (uint64_t)(i * dt_us);
        imu[i].accel_x = 0.0f;
        imu[i].accel_y = -centripetal_accel;
        imu[i].accel_z = -9.81f;
        imu[i].gyro_x = 0.0f;
        imu[i].gyro_y = 0.0f;
        imu[i].gyro_z = omega;

        wheel[i].timestamp_us = (uint64_t)(i * dt_us);
        wheel[i].speed = speed_mps;
        wheel[i].steering_angle = atanf(speed_mps / (radius_m * 10.0f));
    }
}

static void add_noise(MockImuData* imu, MockWheelData* wheel, int num_samples,
                      float accel_noise, float gyro_noise, float speed_noise) {
    for (int i = 0; i < num_samples; i++) {
        imu[i].accel_x += ((float)rand() / RAND_MAX - 0.5f) * accel_noise;
        imu[i].accel_y += ((float)rand() / RAND_MAX - 0.5f) * accel_noise;
        imu[i].accel_z += ((float)rand() / RAND_MAX - 0.5f) * accel_noise;
        imu[i].gyro_x += ((float)rand() / RAND_MAX - 0.5f) * gyro_noise;
        imu[i].gyro_y += ((float)rand() / RAND_MAX - 0.5f) * gyro_noise;
        imu[i].gyro_z += ((float)rand() / RAND_MAX - 0.5f) * gyro_noise;

        wheel[i].speed += ((float)rand() / RAND_MAX - 0.5f) * speed_noise;
    }
}

static float compute_rmse(float* estimates, float* ground_truth, int num_samples) {
    float sum_sq = 0.0f;
    for (int i = 0; i < num_samples; i++) {
        float diff = estimates[i] - ground_truth[i];
        sum_sq += diff * diff;
    }
    return sqrtf(sum_sq / (float)num_samples);
}

int main(int argc, char** argv) {
    srand(42);

    const int NUM_SAMPLES = 1000;
    const float DT_US = 10000.0f;
    const float SPEED_MPS = 20.0f;
    const float CIRCLE_RADIUS = 100.0f;

    MockImuData* imu_data = malloc(NUM_SAMPLES * sizeof(MockImuData));
    MockWheelData* wheel_data = malloc(NUM_SAMPLES * sizeof(MockWheelData));

    printf("=== EKF-SLAM Mock Test: GPS Loss Scenario ===\n\n");

    printf("Test 1: Straight Line Motion (Constant Speed %.1f m/s)\n", SPEED_MPS);
    printf("=================================================\n");
    
    generate_straight_line_motion(imu_data, wheel_data, NUM_SAMPLES, DT_US, SPEED_MPS);
    add_noise(imu_data, wheel_data, NUM_SAMPLES, 0.1f, 0.01f, 0.2f);

    EkfSlam ekf;
    ekf_slam_init(&ekf, 0.0f, 0.0f, 0.0f);
    ekf.x.v = SPEED_MPS;

    float* true_x = malloc(NUM_SAMPLES * sizeof(float));
    float* true_y = malloc(NUM_SAMPLES * sizeof(float));
    float* est_x = malloc(NUM_SAMPLES * sizeof(float));
    float* est_y = malloc(NUM_SAMPLES * sizeof(float));
    float* est_hdg = malloc(NUM_SAMPLES * sizeof(float));
    float* cov_x = malloc(NUM_SAMPLES * sizeof(float));
    float* cov_y = malloc(NUM_SAMPLES * sizeof(float));
    float* cov_hdg = malloc(NUM_SAMPLES * sizeof(float));

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ekf_slam_predict(&ekf, imu_data[i].accel_x, imu_data[i].gyro_z, imu_data[i].timestamp_us);
        
        float x, y, h, cvx, cvy, cvh;
        ekf_slam_get_pose(&ekf, &x, &y, &h, &cvx, &cvy, &cvh);
        
        est_x[i] = x;
        est_y[i] = y;
        est_hdg[i] = h;
        cov_x[i] = cvx;
        cov_y[i] = cvy;
        cov_hdg[i] = cvh;

        float t = (float)i * DT_US / 1e6f;
        true_x[i] = SPEED_MPS * t;
        true_y[i] = 0.0f;
    }

    float rmse_x = compute_rmse(est_x, true_x, NUM_SAMPLES);
    float rmse_y = compute_rmse(est_y, true_y, NUM_SAMPLES);
    
    printf("RMSE X: %.4f m\n", rmse_x);
    printf("RMSE Y: %.4f m\n", rmse_y);
    printf("Final X: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_x[NUM_SAMPLES-1], true_x[NUM_SAMPLES-1], 
           fabsf(est_x[NUM_SAMPLES-1] - true_x[NUM_SAMPLES-1]));
    printf("Final Y: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_y[NUM_SAMPLES-1], true_y[NUM_SAMPLES-1], 
           fabsf(est_y[NUM_SAMPLES-1] - true_y[NUM_SAMPLES-1]));
    printf("Final Covariance: [%.4f, %.4f, %.4f]\n", 
           cov_x[NUM_SAMPLES-1], cov_y[NUM_SAMPLES-1], cov_hdg[NUM_SAMPLES-1]);

    printf("\nTest 2: Circular Motion (R=%.0f m, v=%.1f m/s)\n", CIRCLE_RADIUS, SPEED_MPS);
    printf("=================================================\n");

    generate_circular_motion(imu_data, wheel_data, NUM_SAMPLES, DT_US, SPEED_MPS, CIRCLE_RADIUS);
    add_noise(imu_data, wheel_data, NUM_SAMPLES, 0.1f, 0.01f, 0.2f);

    ekf_slam_init(&ekf, 0.0f, 0.0f, 0.0f);
    ekf.x.v = SPEED_MPS;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ekf_slam_predict(&ekf, imu_data[i].accel_x, imu_data[i].gyro_z, imu_data[i].timestamp_us);
        
        float x, y, h, cvx, cvy, cvh;
        ekf_slam_get_pose(&ekf, &x, &y, &h, &cvx, &cvy, &cvh);
        
        est_x[i] = x;
        est_y[i] = y;
        est_hdg[i] = h;
        cov_x[i] = cvx;
        cov_y[i] = cvy;
        cov_hdg[i] = cvh;

        float t = (float)i * DT_US / 1e6f;
        float angle = SPEED_MPS / CIRCLE_RADIUS * t;
        true_x[i] = CIRCLE_RADIUS * sinf(angle);
        true_y[i] = CIRCLE_RADIUS * (1.0f - cosf(angle));
    }

    rmse_x = compute_rmse(est_x, true_x, NUM_SAMPLES);
    rmse_y = compute_rmse(est_y, true_y, NUM_SAMPLES);
    
    printf("RMSE X: %.4f m\n", rmse_x);
    printf("RMSE Y: %.4f m\n", rmse_y);
    printf("Final X: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_x[NUM_SAMPLES-1], true_x[NUM_SAMPLES-1], 
           fabsf(est_x[NUM_SAMPLES-1] - true_x[NUM_SAMPLES-1]));
    printf("Final Y: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_y[NUM_SAMPLES-1], true_y[NUM_SAMPLES-1], 
           fabsf(est_y[NUM_SAMPLES-1] - true_y[NUM_SAMPLES-1]));
    printf("Final Covariance: [%.4f, %.4f, %.4f]\n", 
           cov_x[NUM_SAMPLES-1], cov_y[NUM_SAMPLES-1], cov_hdg[NUM_SAMPLES-1]);

    printf("\nTest 3: GPS Recovery (Periodic Updates at 10Hz)\n");
    printf("=================================================\n");

    generate_straight_line_motion(imu_data, wheel_data, NUM_SAMPLES, DT_US, SPEED_MPS);
    add_noise(imu_data, wheel_data, NUM_SAMPLES, 0.2f, 0.02f, 0.5f);

    ekf_slam_init(&ekf, 0.0f, 0.0f, 0.0f);
    ekf.x.v = SPEED_MPS;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ekf_slam_predict(&ekf, imu_data[i].accel_x, imu_data[i].gyro_z, imu_data[i].timestamp_us);
        
        if (i % 10 == 0) {
            float t = (float)i * DT_US / 1e6f;
            float gps_noise_x = ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
            float gps_noise_y = ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
            ekf_slam_update(&ekf, SPEED_MPS * t + gps_noise_x, gps_noise_y, 0.0f);
        }
        
        float x, y, h, cvx, cvy, cvh;
        ekf_slam_get_pose(&ekf, &x, &y, &h, &cvx, &cvy, &cvh);
        
        est_x[i] = x;
        est_y[i] = y;
        est_hdg[i] = h;
        cov_x[i] = cvx;
        cov_y[i] = cvy;
        cov_hdg[i] = cvh;

        float t = (float)i * DT_US / 1e6f;
        true_x[i] = SPEED_MPS * t;
        true_y[i] = 0.0f;
    }

    rmse_x = compute_rmse(est_x, true_x, NUM_SAMPLES);
    rmse_y = compute_rmse(est_y, true_y, NUM_SAMPLES);
    
    printf("RMSE X: %.4f m\n", rmse_x);
    printf("RMSE Y: %.4f m\n", rmse_y);
    printf("Final X: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_x[NUM_SAMPLES-1], true_x[NUM_SAMPLES-1], 
           fabsf(est_x[NUM_SAMPLES-1] - true_x[NUM_SAMPLES-1]));
    printf("Final Y: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_y[NUM_SAMPLES-1], true_y[NUM_SAMPLES-1], 
           fabsf(est_y[NUM_SAMPLES-1] - true_y[NUM_SAMPLES-1]));
    printf("Final Covariance: [%.4f, %.4f, %.4f]\n", 
           cov_x[NUM_SAMPLES-1], cov_y[NUM_SAMPLES-1], cov_hdg[NUM_SAMPLES-1]);

    printf("\nTest 4: GPS Loss Recovery (30s gap, then recovery)\n");
    printf("=================================================\n");

    generate_straight_line_motion(imu_data, wheel_data, NUM_SAMPLES, DT_US, SPEED_MPS);
    add_noise(imu_data, wheel_data, NUM_SAMPLES, 0.2f, 0.02f, 0.5f);

    ekf_slam_init(&ekf, 0.0f, 0.0f, 0.0f);
    ekf.x.v = SPEED_MPS;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ekf_slam_predict(&ekf, imu_data[i].accel_x, imu_data[i].gyro_z, imu_data[i].timestamp_us);
        
        if (i < 100 || i > 400) {
            float t = (float)i * DT_US / 1e6f;
            float gps_noise_x = ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
            float gps_noise_y = ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
            ekf_slam_update(&ekf, SPEED_MPS * t + gps_noise_x, gps_noise_y, 0.0f);
        }
        
        float x, y, h, cvx, cvy, cvh;
        ekf_slam_get_pose(&ekf, &x, &y, &h, &cvx, &cvy, &cvh);
        
        est_x[i] = x;
        est_y[i] = y;
        est_hdg[i] = h;
        cov_x[i] = cvx;
        cov_y[i] = cvy;
        cov_hdg[i] = cvh;

        float t = (float)i * DT_US / 1e6f;
        true_x[i] = SPEED_MPS * t;
        true_y[i] = 0.0f;
    }

    rmse_x = compute_rmse(est_x, true_x, NUM_SAMPLES);
    rmse_y = compute_rmse(est_y, true_y, NUM_SAMPLES);
    
    printf("RMSE X: %.4f m\n", rmse_x);
    printf("RMSE Y: %.4f m\n", rmse_y);
    printf("Final X: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_x[NUM_SAMPLES-1], true_x[NUM_SAMPLES-1], 
           fabsf(est_x[NUM_SAMPLES-1] - true_x[NUM_SAMPLES-1]));
    printf("Final Y: est=%.2f, true=%.2f, drift=%.2f m\n", 
           est_y[NUM_SAMPLES-1], true_y[NUM_SAMPLES-1], 
           fabsf(est_y[NUM_SAMPLES-1] - true_y[NUM_SAMPLES-1]));
    printf("Final Covariance: [%.4f, %.4f, %.4f]\n", 
           cov_x[NUM_SAMPLES-1], cov_y[NUM_SAMPLES-1], cov_hdg[NUM_SAMPLES-1]);

    printf("\n=== Test Completed ===\n");

    free(imu_data);
    free(wheel_data);
    free(true_x);
    free(true_y);
    free(est_x);
    free(est_y);
    free(est_hdg);
    free(cov_x);
    free(cov_y);
    free(cov_hdg);

    return 0;
}
