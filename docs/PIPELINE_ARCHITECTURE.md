# FlowEngine Pipeline 架构

FlowEngine 的 ADAS 演示 pipeline 由 12 个节点插件组成，通过 `flow_launcher config/pipeline.json` 配置驱动启动。

## 数据流

```
flowsim ─── vehicle/state ──→ sensor_model ─── sensor/lidar ──→ perception ─── perception/obstacles ──→ fusion
  │  │                             │         sensor/gps ──────────────┘                                       │
  │  │  ├── road/geometry          └── sensor/camera                                                              │
  │  │  ├── road/traffic_lights                                                                              fusion/localization
  │  │  ├── scene/frame                                                                                             │
  │  │  ├── sim/tick                       ┌─────────────────────────────────────────────────────────────────────┤
  │  │  └── sim/collision                  ▼                                                                     │
  │  └──────────── vehicle/state ────→ planning ─── planning/trajectory ──→ control ─── control/raw_cmd ──→ safety_control
  │                                          │                                   │                              │
  │                                          └── vehicle/state ─────────────────┘                      control/cmd
  │                                                                                                                  │
  │                                                                                                                  ▼
  └─────────────────────────────────────────────────────────────────────────────────────────────────────────── flowsim
                                                                                              (bicycle model + 碰撞)
                                                ↑
                                                └── inference (影子模式，仅对比监控，不接入控制)
```

## 节点清单

| 节点 | .so | 输入 topics | 输出 topics | 算法 |
|------|-----|------------|-------------|------|
| `flowsim` | `libflowsim_node.so` | `control/cmd` | `vehicle/state`, `road/geometry`, `road/traffic_lights`, `scene/frame`, `sim/tick`, `sim/collision` | Bicycle model + esmini RoadManager + OBB SAT 碰撞 + NPC AI 状态机 |
| `sensor_model` | `libsensor_model_node.so` | `vehicle/state` | `sensor/lidar`, `sensor/gps`, `sensor/camera` | 噪声注入 + FOV 裁剪 + NMEA GPS 回放 |
| `perception` | `libperception_node.so` | `vehicle/state`, `sensor/lidar` | `perception/obstacles` | DBSCAN (eps=2m, min_pts=4) + RANSAC 地面去除 |
| `fusion` | `libfusion_node.so` | `sensor/lidar`, `sensor/gps` | `fusion/localization`, `fusion/latency` | EKF 5D (x, y, v, heading, yaw_rate) |
| `planning` | `libplanning_node.so` | `fusion/localization`, `vehicle/state` | `planning/trajectory` | Frenet 最优轨迹规划（依赖 Eigen3） |
| `control` | `libcontrol_node.so` | `fusion/localization`, `planning/trajectory`, `vehicle/state` | `control/raw_cmd` | PID 纵向 + Stanley 横向 + ACC + 自适应变道 |
| `safety_control` | `libsafety_control_node.so` | `control/raw_cmd`, `vehicle/state` | `control/cmd` | FlowCoro 安全包络：TTC/横向交叉/行人 |
| `monitor` | `libmonitor_node.so` | `perception/obstacles`, `vehicle/state`, `fusion/latency` | — | 系统指标采集 + JSON 导出 + IPC 桥接 |
| `data_recorder` | `libdata_recorder_node.so` | `fusion/localization`, `planning/trajectory` | — | 特征/标签 JSONL 采样（Stage 0） |
| `inference` | `libinference_node.so` | `fusion/localization` | `inference/trajectory` | tiny-MLP 影子推理（Stage 2） |
| `learner` | `liblearner_node.so` | `inference/trajectory` 等 | `learner/loss` | 在线训练节点（车端学习闭环 Stage 3） |
| `model_ota` | `libmodel_ota_node.so` | `learner/loss` 等 | `model/ota_status` | 模型 OTA 升级协调 |

> **Learning Loop:** `data_recorder` → `inference` → `learner` → `model_ota` 是车端学习闭环节点。
> `inference` 运行在影子模式（shadow mode），只发布 `inference/trajectory` 供对比监控，
> **不**接入真实控制链路。详见 [LEARNING_LOOP.md](LEARNING_LOOP.md)。

## 控制闭环

```
control (PID) → control/raw_cmd → safety_control → control/cmd → flowsim (bicycle model) → vehicle/state
                                              ↑                                              │
                                              └──────── planning/trajectory ←── planning ←── fusion/localization ←── fusion ←── sensor/lidar,gps
```

- **频率**: flowsim 20Hz，planning 20Hz，control 20Hz
- **安全包络**: safety_control 限制 throttle ≤ 0.85，steer ≤ 0.22 rad（低速 0.18 rad）
- **ACC**: time headway 1.4s，最小 gap 5m
- **Stuck recovery**: 静止 >3s + 横向卡在线附近 → 强制收敛到最近车道中心
- **Road guard**: |ego_y| > 2.1m → brake + steer toward center

## 可视化链路

可视化由统一的 flowmond 守护进程（`build/bin/flowmond`）提供，同时启用 IPC 桥接（首选）
与文件桥接（回退）两条数据链路。

| 链路 | 数据路径 | 端口 |
|------|---------|------|
| IPC 桥接（首选） | monitor_node → `stats_bridge` / `dashboard_bridge` → `flowmond` | 8800 |
| 文件桥接（回退） | monitor_node → `/tmp/flow_topology.json` → `flowmond` | 8800 |
| ASCII 俯视（调试） | flowsim_node → `/tmp/flow_ascii_overhead.txt`（每秒覆盖） | — |
| Foxglove 3D | `foxglove_bridge.py` 读取 JSON 文件 | 8765 |

## 配置格式 (pipeline.json)

```json
{
  "scheduler": { "mode": "choreo", "tick_us": 1000 },
  "processes": [
    {
      "name": "flowsim",
      "library_path": "build/lib/libflowsim_node.so",
      "auto_start": true,
      "publish": [
        {"topic": "vehicle/state", "type": "VehicleState", "qos": {"depth": 1, "policy": "drop_oldest"}},
        {"topic": "road/geometry", "type": "RoadGeometry", "qos": {"depth": 1, "policy": "drop_oldest"}},
        {"topic": "road/traffic_lights", "qos": {"depth": 1, "policy": "drop_oldest"}},
        {"topic": "scene/frame", "qos": {"depth": 1, "policy": "drop_oldest"}},
        {"topic": "sim/tick", "qos": {"depth": 1, "policy": "drop_oldest"}},
        {"topic": "sim/collision", "qos": {"depth": 4, "policy": "drop_oldest"}}
      ],
      "subscribe": ["control/cmd"],
      "params": "{\"init_speed\":10.0,\"target_speed\":15.0,\"scenario_file\":\"scenarios/straight_road.json\"}"
    }
  ]
}
```

## 场景文件

| 场景 | 文件 | 要素 |
|------|------|------|
| 直路综合 | `scenarios/straight_road.json` | 4 车道直路 + 2 同向慢车 + 1 对向来车 + 1 行人 + 红绿灯 |

场景定义格式见 `include/scenario_loader.h`、`src/core/scenario_loader.c`。

## 关键参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| flowsim | `init_speed` | 10.0 | 初始速度 m/s |
| flowsim | `target_speed` | 15.0 | 目标巡航速度 m/s |
| flowsim | `scenario_file` | `scenarios/straight_road.json` | 场景文件路径 |
| control | `pid_kp/ki/kd` | 800/50/100 | PID 纵向控制器 |
| control | `lat_kp` | 0.5 | 横向误差增益 (rad/m) |
| control | `lat_kd_heading` | 1.35 | 航向阻尼增益 |
| control | `yaw_damping` | 0.15 | 横摆角速度阻尼增益 |
| control | `steer_min_clamp` | 0.030 | 高速最小转向钳位 (rad)，动态模型需比运动学模型更高 |
| control | `lc_lat_accel_max` | 3.5 (高速) / 4.5 (低速) | 变道最大侧向加速度 (m/s²)，动态模型下轮胎松弛导致有效 ay 偏低 |
| control | `lane_change_blocked_timeout_s` | 0.6 | 变道阻塞超时 (s) |
| control | `lc_stable_wait_s` | 4.0 | 变道后稳定巡航时间 |
| control | `lc_cooldown_after_stable_s` | 1.5 | 稳定后冷却 |
| control | `lc_cooldown_after_return_s` | 2.0 | 返回原车道后冷却 |
| control | `min_overtake_gap_base` | 14.0 | 触发超车最小间距基准 (m) |
| safety_control | `max_throttle` | 1.0 | 最大油门 |
| safety_control | `max_steer` | 0.22 | 最大转向角 (rad) |
| safety_control | `time_headway` | 1.8 | 安全时距 (s) |

## 验证

```bash
# 回归评估器
python3 tools/demo_evaluator.py --duration 45

# 烟测试
bash scripts/launcher_smoke.sh
```
