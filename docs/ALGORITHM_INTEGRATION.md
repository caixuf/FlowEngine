# 算法集成指南

## 定位

```
FlowEngine = 中间件框架（调度 + 通信 + 状态机 + 监控）
第三方库  = 算法实现（感知数学 + 融合数学 + 规划数学 + 控制数学）

FlowEngine 不做算法，只做算法的"插座"。
```

## 架构

```
┌────────────────────────────────────────────────┐
│                  FlowEngine                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────┐ │
│  │ Scheduler│  │  State   │  │  Discovery   │ │
│  │ (Choreo) │  │ Machine  │  │  + Transport │ │
│  └──────────┘  └──────────┘  └──────────────┘ │
│       │              │               │          │
│       ▼              ▼               ▼          │
│  ┌──────────────────────────────────────────┐  │
│  │        Algorithm Plugin SDK              │  │
│  │  AlgorithmInterface (C ABI, dlopen)      │  │
│  └──────────────────────────────────────────┘  │
│       │         │         │         │          │
│       ▼         ▼         ▼         ▼          │
│  ┌────────┐┌────────┐┌────────┐┌────────┐    │
│  │Percept ││ Fusion ││Planning││Control │    │
│  │Plugin  ││Plugin  ││Plugin  ││Plugin  │    │
│  └────────┘└────────┘└────────┘└────────┘    │
└────────────────────────────────────────────────┘
         │         │         │         │
         ▼         ▼         ▼         ▼
   ┌─────────────────────────────────────────┐
   │        Third-Party Libraries             │
   │  OpenCV  │ Eigen  │ OMPL  │ Apollo      │
   │  TensorRT│ Ceres  │ GTSAM │ Control     │
   └─────────────────────────────────────────┘
```

## 接入步骤

### 1. 实现 AlgorithmInterface

```c
#include "algorithm_plugin.h"

static AlgorithmInterface my_interface = {
    .get_info      = my_get_info,
    .initialize    = my_init,
    .process       = my_process,
    .get_state_json = my_get_state,
    .set_param     = my_set_param,
    .destroy       = my_destroy,
};

AlgorithmInterface* get_algorithm_interface(void) { return &my_interface; }
const char* get_algorithm_version(void) { return "1.0.0"; }
```

### 2. 编译为 .so

```bash
gcc -shared -fPIC -I include my_algo.c -o libmy_algo.so \
    $(pkg-config --cflags --libs opencv4 eigen3)
```

### 3. Launch 配置加载

```json
{
  "nodes": [{
    "name": "perception_node",
    "plugin": "lib/libmy_algo.so",
    "scheduling": {"priority": "critical", "cpu_affinity": [0,1]},
    "subscribe": [{"topic": "sensor/camera"}],
    "publish": [{"topic": "perception/objects"}]
  }]
}
```

### 4. 状态机联动

```c
// 驾驶模式切换 → 自动激活/停用算法
algorithm_activate_for_mode(&sm, SM_MODE_CP, plugins);
// ACC 算法停用, LaneDetection + SteeringControl 激活
```

## 推荐第三方库映射

| 模块 | 推荐库 | 为什么 |
|------|--------|--------|
| **感知-2D检测** | OpenCV + ONNX Runtime | 轻量、跨平台、C API 友好 |
| **感知-3D检测** | TensorRT / OpenPCDet | GPU 加速、点云处理 |
| **融合-EKF** | Eigen | 头文件库、零依赖、C++ 模板 |
| **融合-因子图** | GTSAM / Ceres | 非线性优化、批量平滑 |
| **定位** | GTSAM / cartographer | 图优化、实时 SLAM |
| **预测** | 自研轻量 LSTM / Apollo Prediction | 轨迹预测 |
| **规划-路径** | OMPL / Apollo Planning | 采样规划、搜索 |
| **规划-速度** | 自研 ST 图 + DP/QP | 速度规划 |
| **控制-PID** | 自研（参考 example_pid_controller.c）| 最简单、好调试 |
| **控制-MPC** | OSQP / acados | 模型预测控制 |

## 参考实现

- `src/plugins/example_pid_controller.c` — PID 纵向控制器
- `src/plugins/fake_perception_task.c` — 模拟感知（用真实模型替换）
- `src/plugins/fake_control_task.c` — 模拟控制（用真实 PID 替换）

## 不推荐的做法

- ❌ 在 FlowEngine 内部写复杂的数学运算
- ❌ 把算法编译进核心库（应该作为独立 .so 插件）
- ❌ 用 C 写矩阵运算（用 Eigen/Ceres 等经过验证的库）

## 推荐的做法

- ✅ 算法作为独立 .so，通过 AlgorithmInterface 接入
- ✅ 状态机负责模式编排（NA→ACC→CP→NP→NOA）
- ✅ FlowEngine 负责调度、通信、监控
- ✅ 第三方库负责数学运算
