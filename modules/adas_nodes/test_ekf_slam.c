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

static int g_failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); ++g_failures; } \
    else { printf("ok: %s\n", msg); } \
} while (0)

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

    printf("\nTest 5: Course-over-ground heading observation (航向收敛断言)\n");
    printf("=================================================\n");
    /* 隔离 slam_node 的 course-over-ground 航向观测语义：车实际以固定航向
     * TRUE_HDG 直线行驶，但 EKF 初始 heading=0（错 TRUE_HDG rad），gyro_z=0
     * （IMU 提供不了航向变化）。若像老 bug 那样把 EKF 自己的 heading 当观测
     * 喂回，残差恒 0、heading 永远卡在 0；喂真实 atan2(Δy,Δx) 航向观测，
     * heading 才会从 0 收敛到 TRUE_HDG。断言：残差下降、末端收敛、非恒 0。 */
    const float TRUE_HDG = 0.7853982f;  /* π/4 */
    const float V5 = 15.0f;
    ekf_slam_init(&ekf, 0.0f, 0.0f, 0.0f);
    ekf.x.v = V5;

    float chx = cosf(TRUE_HDG), chy = sinf(TRUE_HDG);
    float last_ox = 0.0f, last_oy = 0.0f;
    int have_last = 0;
    float initial_resid = -1.0f, final_resid = 0.0f, final_hdg = 0.0f;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint64_t ts = (uint64_t)(i * DT_US);
        ekf_slam_predict(&ekf, 0.0f, 0.0f, ts);  /* gyro_z=0：航向不来自 IMU */

        float t = (float)i * DT_US / 1e6f;
        float true_x5 = V5 * t * chx;
        float true_y5 = V5 * t * chy;
        /* lidar 观测位置带噪 */
        float ox = true_x5 + ((float)rand() / (float)RAND_MAX - 0.5f) * 0.1f;
        float oy = true_y5 + ((float)rand() / (float)RAND_MAX - 0.5f) * 0.1f;

        if (have_last) {
            float dx = ox - last_ox, dy = oy - last_oy;
            float disp = sqrtf(dx * dx + dy * dy);
            if (disp > 0.4f) {  /* 与 slam_node heading_obs_min_disp 一致 */
                float obs_heading = atan2f(dy, dx);
                ekf_slam_update(&ekf, ox, oy, obs_heading);
                last_ox = ox; last_oy = oy;
            } else {
                ekf_slam_update_pos(&ekf, ox, oy);
            }
        } else {
            ekf_slam_update_pos(&ekf, ox, oy);
            last_ox = ox; last_oy = oy; have_last = 1;
        }

        float x, y, h, cvx, cvy, cvh;
        ekf_slam_get_pose(&ekf, &x, &y, &h, &cvx, &cvy, &cvh);
        float resid = fabsf(h - TRUE_HDG);
        if (i == 50) initial_resid = resid;   /* 早期残差快照（此时已有若干观测） */
        final_resid = resid;
        final_hdg = h;
    }

    printf("initial_resid(@50)=%.4f final_resid=%.4f final_hdg=%.4f (true=%.4f)\n",
           initial_resid, final_resid, final_hdg, TRUE_HDG);
    CHECK(initial_resid > 0.0f, "captured initial heading residual");
    CHECK(final_resid < initial_resid, "heading residual decreased over time");
    CHECK(final_resid < 0.2f, "heading converged (residual < 0.2 rad)");
    CHECK(fabsf(final_hdg) > 0.3f, "heading estimate non-zero (not stuck at 0)");

    printf("\n=== Test Completed (%d failures) ===\n", g_failures);

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

    return g_failures == 0 ? 0 : 1;
}
