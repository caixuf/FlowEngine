# FlowEngine Algorithm Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                        FlowBoard Monitor                        │
│  http://localhost:8800  ← 实时拓扑 + 帧监控 + QoS + 图表        │
└─────────────────────────────────────────────────────────────────┘
                                ▲
                                │ discovery JSON + metrics
                                │
┌─────────────────────────────────────────────────────────────────┐
│                      FlowEngine Middleware                       │
│                                                                  │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌─────────────┐ │
│  │ Scheduler│   │  State   │   │Transport │   │     QoS     │ │
│  │ (Choreo) │   │ Machine  │   │  (AUTO)  │   │ (per-topic) │ │
│  └──────────┘   └──────────┘   └──────────┘   └─────────────┘ │
│                                                                  │
│  sensor/lidar ──→ sensor/camera ──→ fusion/objects              │
│                                          ↓                       │
│                                    planning/traj                 │
│                                          ↓                       │
│                                     control/cmd                  │
└─────────────────────────────────────────────────────────────────┘
         ▲               ▲               ▲               ▲
         │               │               │               │
    ┌────────┐     ┌────────┐     ┌────────┐     ┌────────┐
    │ OpenCV │     │ Eigen  │     │  OSQP  │     │  PID   │
    │ 4.10+  │     │ 3.4+   │     │ 0.6+   │     │ (ref)  │
    └────────┘     └────────┘     └────────┘     └────────┘
     Perception      Fusion       Planning        Control
```

## Component Details

### 1. OpenCV Perception Plugin

```cpp
// perception_opencv.cpp
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "algorithm_plugin.h"

struct OpenCVPerception {
    cv::dnn::Net yolo;           // YOLO/SSD detection model
    cv::Mat      camera_matrix;  // Intrinsic calibration
    cv::Mat      dist_coeffs;    // Distortion coefficients

    // Output: detected objects → publish to "perception/objects"
    std::vector<DetectedObject> objects;
};

// FlowEngine plugin API
static int opencv_process(void* handle, const void* input) {
    auto* p = (OpenCVPerception*)handle;
    // 1. Receive camera frame from bus
    cv::Mat frame = deserialize_image(input);

    // 2. Run YOLO inference
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, cv::Size(640,640));
    p->yolo.setInput(blob);
    std::vector<cv::Mat> outputs;
    p->yolo.forward(outputs);

    // 3. Parse detections → publish to bus
    parse_detections(outputs, p->objects);
    return 0;
}

// Dependencies: libopencv-dev
// Build: g++ -shared -fPIC perception_opencv.cpp -o libperception_opencv.so
//        $(pkg-config --cflags --libs opencv4)
```

### 2. Eigen Fusion Plugin

```cpp
// fusion_eigen.cpp
#include <Eigen/Dense>
#include "algorithm_plugin.h"

struct EigenFusion {
    // State vector: [x, y, vx, vy, ax, ay]
    Eigen::Matrix<double, 6, 1> state;
    Eigen::Matrix<double, 6, 6> covariance;

    // EKF predict step
    void predict(double dt) {
        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F(0,2) = dt; F(1,3) = dt;  // position += velocity * dt
        F(2,4) = dt; F(3,5) = dt;  // velocity += acceleration * dt

        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        state = F * state;
        covariance = F * covariance * F.transpose() + Q;
    }

    // EKF update from LiDAR measurement
    void update_lidar(double mx, double my) {
        Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
        H(0,0) = 1; H(1,1) = 1;  // measure position directly

        Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity() * 0.1;
        Eigen::Matrix<double, 2, 1> z; z << mx, my;
        Eigen::Matrix<double, 2, 1> y = z - H * state;

        Eigen::Matrix<double, 2, 2> S = H * covariance * H.transpose() + R;
        Eigen::Matrix<double, 6, 2> K = covariance * H.transpose() * S.inverse();

        state += K * y;
        covariance = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * covariance;
    }
};

// Dependencies: libeigen3-dev (header-only, no linking needed!)
// Build: g++ -shared -fPIC fusion_eigen.cpp -o libfusion_eigen.so
//        $(pkg-config --cflags eigen3)
```

### 3. OSQP MPC Controller Plugin

```cpp
// control_osqp.cpp
#include <osqp/osqp.h>
#include "algorithm_plugin.h"

struct OSQPController {
    OSQPWorkspace* workspace;
    OSQPSettings*  settings;
    OSQPData*      data;

    // MPC horizon parameters
    int    horizon;        // N = 20 steps
    double dt;             // 0.1s timestep
    double target_speed;   // 33 m/s

    bool solve_mpc(double current_state[6], double* control_output) {
        // Set up QP: min 0.5*x'Px + q'x  s.t. l <= Ax <= u
        // State cost + control cost + terminal cost
        // Constraints: acceleration limits, jerk limits, safety distance

        c_int exitflag = osqp_solve(workspace);
        if (exitflag == OSQP_SOLVED) {
            // Extract first control input: throttle, brake, steer
            control_output[0] = data->x[0]; // throttle
            control_output[1] = data->x[1]; // brake
            control_output[2] = data->x[2]; // steering
            return true;
        }
        return false; // QP failed → fallback to PID
    }
};

// Dependencies: libosqp-dev
// Build: g++ -shared -fPIC control_osqp.cpp -o libcontrol_osqp.so
//        $(pkg-config --cflags --libs osqp)
```

## Build Everything

```bash
# Install dependencies
sudo apt install libopencv-dev libeigen3-dev libosqp-dev

# Build all algorithm plugins
mkdir -p build/plugins
cd build/plugins

# OpenCV Perception
g++ -shared -fPIC -O2 \
    ../../src/plugins/perception_opencv.cpp \
    -o libperception_opencv.so \
    $(pkg-config --cflags --libs opencv4)

# Eigen Fusion (header-only, no linking)
g++ -shared -fPIC -O2 \
    ../../src/plugins/fusion_eigen.cpp \
    -o libfusion_eigen.so \
    $(pkg-config --cflags eigen3)

# OSQP Controller
g++ -shared -fPIC -O2 \
    ../../src/plugins/control_osqp.cpp \
    -o libcontrol_osqp.so \
    $(pkg-config --cflags --libs osqp)
```

## Launch Configuration

```json
{
  "scheduler": {"mode": "choreo"},
  "nodes": [
    {
      "name": "perception",
      "plugin": "build/plugins/libperception_opencv.so",
      "publish": [{"topic": "perception/objects", "type": "ObstacleList"}],
      "subscribe": [{"topic": "sensor/camera"}],
      "scheduling": {"priority": "critical", "cpu_affinity": [0,1], "max_frequency_hz": 30}
    },
    {
      "name": "fusion",
      "plugin": "build/plugins/libfusion_eigen.so",
      "publish": [{"topic": "fusion/state", "type": "EgoState"}],
      "subscribe": [{"topic": "sensor/lidar"}, {"topic": "sensor/gps"}],
      "scheduling": {"priority": "high", "cpu_affinity": [2,3], "max_frequency_hz": 100}
    },
    {
      "name": "control",
      "plugin": "build/plugins/libcontrol_osqp.so",
      "publish": [{"topic": "control/cmd", "type": "ControlCmd"}],
      "subscribe": [{"topic": "fusion/state"}, {"topic": "planning/traj"}],
      "scheduling": {"priority": "high", "cpuset": "0-1", "max_frequency_hz": 100}
    }
  ]
}
```

## Runtime Flow

```
Camera → OpenCV(YOLO) → ObstacleList ─┐
LiDAR  → PointCloud     → Objects   ─┤
GPS    → NMEA parse     → Position  ─┤
                                      ▼
                              Eigen EKF Fusion
                              (predict + update)
                                      │
                                      ▼
                              EgoState + Obstacles
                                      │
                              OSQP MPC Solver
                              (minimize cost, respect constraints)
                                      │
                                      ▼
                              ControlCmd (throttle, brake, steer)
                                      │
                              FlowBoard Monitor
                              (topology, latency, QoS, charts)
```

## Performance Budget (Target)

| Pipeline Stage | Latency Budget | CPU Cores |
|---------------|---------------|-----------|
| OpenCV YOLO   | < 30ms        | 2 cores   |
| Eigen EKF     | < 1ms         | 1 core    |
| OSQP MPC      | < 5ms (20-horizon) | 1 core |
| FlowEngine Bus| < 100µs       | shared    |
| **Total E2E** | **< 40ms**    | **4 cores** |
