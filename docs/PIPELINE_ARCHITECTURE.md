# FlowEngine Pipeline 架构

FlowEngine 的 ADAS 演示 pipeline 由 8 个节点插件组成，通过 `flow_launcher config/pipeline.json` 配置驱动启动。

## 数据流

```
sim_world ─── vehicle/state ──→ sensor_model ─── sensor/lidar ──→ perception ─── perception/obstacles ──→ fusion
    │                                │         sensor/gps ──────────────┘                                        │
    │                                └── sensor/camera                                                                   │
    │                                                                                                            fusion/localization
    │                                                                                                                   │
    │                                           ┌───────────────────────────────────────────────────────────────────────┤
    │                                           ▼                                                                       │
    └────────────── vehicle/state ──────→ planning ─── planning/trajectory ──→ control ─── control/raw_cmd ──→ safety_control
                                                   │                                    │                              │
                                                   └── vehicle/state ───────────────────┘                       control/cmd
                                                                                                                    │
                                                                                                                    ▼
                                                                                                              sim_world
```

## 节点清单

| 节点 | .so | 输入 topics | 输出 topics | 算法 |
|------|-----|------------|-------------|------|
| `sim_world` | `libsim_world.so` | `control/cmd` | `vehicle/state`, `sim/tick`, `sim/collision` | Bicycle model + 障碍物运动学 + AABB 碰撞 |
| `sensor_model` | `libsensor_model.so` | `vehicle/state` | `sensor/lidar`, `sensor/gps`, `sensor/camera` | 噪声注入 + FOV 裁剪 + NMEA GPS 回放 |
| `perception` | `libperception_node.so`| `vehicle/state`, `sensor/lidar` | `perception/obstacles` | DBSCAN (eps=2m, min_pts=4) + RANSAC 地面去除 |
| `fusion` | `libfusion_node.so` | `sensor/lidar`, `sensor/gps` | `fusion/localization`, `fusion/latency` | EKF 5D (x, y, v, heading, yaw_rate) |
| `planning` | `libplanning_node.so` | `fusion/localization`, `vehicle/state` | `planning/trajectory` | Frenet 最优轨迹规划 |
| `control` | `libcontrol_node.so` | `fusion/localization`, `planning/trajectory`, `vehicle/state` | `control/raw_cmd` | PID 纵向 + Stanley 横向 + ACC + 自适应变道 |
| `safety_control` | `libsafety_control_node.so` | `control/raw_cmd`, `vehicle/state` | `control/cmd` | FlowCoro 安全包络：TTC/横向交叉/行人 |
| `monitor` | `libmonitor_node.so` | `perception/obstacles`, `vehicle/state`, `fusion/latency` | — | 系统指标采集 + JSON 导出 + IPC 桥接 |
| `data_recorder` | `libdata_recorder_node.so` | `fusion/localization`, `planning/trajectory` | — | 特征/标签 JSONL 采样（Stage 0） |
| `inference` | `libinference_node.so` | `fusion/localization` | `inference/trajectory` | tiny-MLP 影子推理（Stage 2） |

> **Learning Loop:** `data_recorder` 和 `inference` 是车端学习闭环节点。`inference` 运行在
> 影子模式（shadow mode），只发布 `inference/trajectory` 供对比监控，**不**接入真实控制链路。
> 详见 [LEARNING_LOOP.md](docs/LEARNING_LOOP.md)。

## 控制闭环

```
control (PID) → control/raw_cmd → safety_control → control/cmd → sim_world (bicycle model) → vehicle/state
                                              ↑                                              │
                                              └──────── planning/trajectory ←── planning ←── fusion/localization ←── fusion ←── sensor/lidar,gps
```

- **频率**: sim_world 20Hz，planning 20Hz，control 20Hz
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
| Foxglove 3D | `foxglove_bridge.py` 读取 JSON 文件 | 8765 |

## 配置格式 (pipeline.json)

```json
{
  "scheduler": { "mode": "choreo", "tick_us": 1000 },
  "processes": [
    {
      "name": "sim_world",
      "library_path": "build/lib/libsim_world.so",
      "auto_start": true,
      "publish": [{"topic": "vehicle/state", "type": "VehicleState", "qos": {"depth": 1}}],
      "subscribe": ["control/cmd"],
      "params": "{\"init_speed\":5.0,\"target_speed\":12.0,\"scenario_file\":\"scenarios/pedestrian_crossing.json\"}"
    }
  ]
}
```

## 场景文件

| 场景 | 文件 | 要素 |
|------|------|------|
| 行人横穿 | `scenarios/pedestrian_crossing.json` | 2 慢车 + 1 行人 + 1 邻道快车 |
| 高速超车 | `scenarios/highway_overtake.json` | 3 慢卡车 + 1 邻道快车 |

场景定义格式见 `include/scenario_loader.h`、`src/core/scenario_loader.c`。

## 关键参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| sim_world | `init_speed` | 5.0 | 初始速度 m/s |
| sim_world | `target_speed` | 12.0 | 目标巡航速度 m/s |
| control | `pid_kp/ki/kd` | 800/50/100 | PID 纵向控制器 |
| control | `lat_kp` | 0.32 | 横向误差增益 (rad/m) |
| control | `lat_kd_heading` | 1.35 | 航向阻尼增益 |
| control | `lane_change_blocked_timeout_s` | 2.0 | 变道阻塞超时 (s) |
| safety_control | `max_throttle` | 0.85 | 最大油门 |
| safety_control | `max_steer` | 0.22 | 最大转向角 (rad) |
| safety_control | `time_headway` | 1.8 | 安全时距 (s) |

## 验证

```bash
# 回归评估器
python3 tools/demo_evaluator.py --duration 45

# 烟测试
bash scripts/launcher_smoke.sh
```
