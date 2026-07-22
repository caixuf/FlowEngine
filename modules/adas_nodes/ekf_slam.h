#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EKF_STATE_DIM 5
#define EKF_COV_DIM   (EKF_STATE_DIM * EKF_STATE_DIM)

typedef struct {
    float x;      
    float y;      
    float heading;
    float v;      
    float omega;  
} EkfState;

typedef struct {
    float data[EKF_COV_DIM];
} EkfCovariance;

typedef struct {
    EkfState x;          
    EkfCovariance P;     
    float accel_bias;    
    float gyro_bias;     
    uint64_t last_time_us;
    bool initialized;    
    float process_noise[5]; 
    float measurement_noise;
} EkfSlam;

void ekf_slam_init(EkfSlam* ekf, float initial_x, float initial_y, float initial_heading);

void ekf_slam_predict(EkfSlam* ekf, float accel_x, float gyro_z, uint64_t current_time_us);

void ekf_slam_update(EkfSlam* ekf, float obs_x, float obs_y, float obs_heading);

void ekf_slam_get_pose(const EkfSlam* ekf, float* x, float* y, float* heading,
                       float* cov_xx, float* cov_yy, float* cov_hh);

#ifdef __cplusplus
}
#endif

#endif
