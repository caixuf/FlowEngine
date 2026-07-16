# FlowEngine 算法栈

> **注意：本文档为参考设计。** 引用的 `perception_opencv.cpp`、`fusion_eigen.cpp`、
> `control_osqp.cpp` 等示例文件尚未实现，仅供学习算法集成模式。实际可运行的算法
> 栈见 `modules/adas_nodes/` 下的 DBSCAN、EKF、Frenet、PID 实现。

```
┌─────────────────────────────────────────────────────────────────┐
│                        FlowBoard Monitor                        │
│  http://localhost:8800  ← 实时拓扑 + 帧监控 + QoS + 图表        │
└─────────────────────────────────────────────────────────────────┘
                                ▲
                                │ discovery JSON + metrics
                                │
┌─────────────────────────────────────────────────────────────────┐
│                      FlowEngine 中间件                           │
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
     感知            融合           规划            控制
```

## 组件详情

### 1. OpenCV 感知插件

```cpp
// perception_opencv.cpp
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "algorithm_plugin.h"

struct OpenCVPerception {
    cv::dnn::Net yolo;           // YOLO/SSD 检测模型
    cv::Mat      camera_matrix;  // 内参标定
    cv::Mat      dist_coeffs;    // 畸变系数

    // 输出：检测到的物体 → 发布到 "perception/objects"
    std::vector<DetectedObject> objects;
};

// FlowEngine 插件 API
static int opencv_process(void* handle, const void* input) {
    auto* p = (OpenCVPerception*)handle;
    // 1. 从总线接收相机帧
    cv::Mat frame = deserialize_image(input);

    // 2. 运行 YOLO 推理
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1/255.0, cv::Size(640,640));
    p->yolo.setInput(blob);
    std::vector<cv::Mat> outputs;
    p->yolo.forward(outputs);

    // 3. 解析检测结果 → 发布到总线
    parse_detections(outputs, p->objects);
    return 0;
}

// 依赖：libopencv-dev
// 构建：g++ -shared -fPIC perception_opencv.cpp -o libperception_opencv.so
//        $(pkg-config --cflags --libs opencv4)
```

### 2. Eigen 融合插件

```cpp
// fusion_eigen.cpp
#include <Eigen/Dense>
#include "algorithm_plugin.h"

struct EigenFusion {
    // 状态向量：[x, y, vx, vy, ax, ay]
    Eigen::Matrix<double, 6, 1> state;
    Eigen::Matrix<double, 6, 6> covariance;

    // EKF 预测步骤
    void predict(double dt) {
        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F(0,2) = dt; F(1,3) = dt;  // 位置 += 速度 * dt
        F(2,4) = dt; F(3,5) = dt;  // 速度 += 加速度 * dt

        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
        state = F * state;
        covariance = F * covariance * F.transpose() + Q;
    }

    // EKF 更新（来自 LiDAR 量测）
    void update_lidar(double mx, double my) {
        Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
        H(0,0) = 1; H(1,1) = 1;  // 直接测量位置

        Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity() * 0.1;
        Eigen::Matrix<double, 2, 1> z; z << mx, my;
        Eigen::Matrix<double, 2, 1> y = z - H * state;

        Eigen::Matrix<double, 2, 2> S = H * covariance * H.transpose() + R;
        Eigen::Matrix<double, 6, 2> K = covariance * H.transpose() * S.inverse();

        state += K * y;
        covariance = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * covariance;
    }
};

// 依赖：libeigen3-dev（仅头文件，无需链接！）
// 构建：g++ -shared -fPIC fusion_eigen.cpp -o libfusion_eigen.so
//        $(pkg-config --cflags eigen3)
```

### 3. OSQP MPC 控制器插件

```cpp
// control_osqp.cpp
#include <osqp/osqp.h>
#include "algorithm_plugin.h"

struct OSQPController {
    OSQPWorkspace* workspace;
    OSQPSettings*  settings;
    OSQPData*      data;

    // MPC 预测时域参数
    int    horizon;        // N = 20 步
    double dt;             // 0.1s 时间步长
    double target_speed;   // 33 m/s

    bool solve_mpc(double current_state[6], double* control_output) {
        // 设置 QP：min 0.5*x'Px + q'x  s.t. l <= Ax <= u
        // 状态代价 + 控制代价 + 终端代价
        // 约束：加速度限制、jerk 限制、安全距离

        c_int exitflag = osqp_solve(workspace);
        if (exitflag == OSQP_SOLVED) {
            // 提取第一个控制输入：throttle、brake、steer
            control_output[0] = data->x[0]; // 油门
            control_output[1] = data->x[1]; // 刹车
            control_output[2] = data->x[2]; // 转向
            return true;
        }
        return false; // QP 求解失败 → 回退到 PID
    }
};

// 依赖：libosqp-dev
// 构建：g++ -shared -fPIC control_osqp.cpp -o libcontrol_osqp.so
//        $(pkg-config --cflags --libs osqp)
```

## 构建全部

```bash
# 安装依赖
sudo apt install libopencv-dev libeigen3-dev libosqp-dev

# 构建全部算法插件
mkdir -p build/plugins
cd build/plugins

# OpenCV 感知
g++ -shared -fPIC -O2 \
    ../../src/plugins/perception_opencv.cpp \
    -o libperception_opencv.so \
    $(pkg-config --cflags --libs opencv4)

# Eigen 融合（仅头文件，无需链接）
g++ -shared -fPIC -O2 \
    ../../src/plugins/fusion_eigen.cpp \
    -o libfusion_eigen.so \
    $(pkg-config --cflags eigen3)

# OSQP 控制器
g++ -shared -fPIC -O2 \
    ../../src/plugins/control_osqp.cpp \
    -o libcontrol_osqp.so \
    $(pkg-config --cflags --libs osqp)
```

## 启动配置

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

## 运行时数据流

```
Camera → OpenCV(YOLO) → ObstacleList ─┐
LiDAR  → PointCloud     → Objects   ─┤
GPS    → NMEA parse     → Position  ─┤
                                      ▼
                              Eigen EKF Fusion
                              （预测 + 更新）
                                      │
                                      ▼
                              EgoState + Obstacles
                                      │
                              OSQP MPC Solver
                              （最小化代价，满足约束）
                                      │
                                      ▼
                              ControlCmd（throttle、brake、steer）
                                      │
                              FlowBoard Monitor
                              （拓扑、延迟、QoS、图表）
```

## 性能预算（目标）

| Pipeline 阶段 | 延迟预算 | CPU 核数 |
|---------------|---------------|-----------|
| OpenCV YOLO   | < 30ms        | 2 核   |
| Eigen EKF     | < 1ms         | 1 核    |
| OSQP MPC      | < 5ms（20 步时域） | 1 核 |
| FlowEngine Bus| < 100µs       | 共享    |
| **端到端总计** | **< 40ms**    | **4 核** |
