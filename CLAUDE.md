# FlowEngine — 项目概览

轻量级自动驾驶中间件，核心是一个 Pub/Sub 消息总线 + 调度器 + 传输层。

## 架构

```
sim_world → sensor_model → perception → fusion → planning → control → safety_control → monitor
     ↓            ↓             ↓           ↓          ↓          ↓            ↓            ↓
 vehicle/state  sensor/lidar perception/  fusion/  planning/  control/raw  control/cmd  dashboard
                sensor/gps  obstacles   localization trajectory  _cmd                     JSON
                     ↓             ↓           ↓          ↓          ↓            ↓
              ════════════════ Message Bus ════════════════════
                                    ↓
                             Transport (IPC/TCP) → Discovery → FlowRegistry
                                    ↓
                             flowmond (IPC stats bridge + HTTP/SSE) → DashBoard
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/e2e_demo.c` | 端到端演示：感知→融合→控制→监控 |
| `src/core/message_bus.c` | 进程内 Pub/Sub 总线 |
| `src/core/transport.c` | 统一传输抽象（local/IPC/TCP） |
| `src/core/scheduler.c` | 任务调度器（classic/choreo 模式） |
| `src/flowmond.c` | 监控守护进程（HTTP 仪表盘 + IPC 统计/仪表盘桥接 + 自动重连） |
| `src/core/monitor_server.c` | 内嵌 HTTP 服务器（多线程连接、/tools 静态资源、JSON 安全转义、过期缓存自动 fallback） |
| `src/core/stats_bridge.c` | 跨进程 topic 统计 IPC 桥接 |
| `src/core/dashboard_bridge.c` | 跨进程仪表盘 JSON IPC 桥接（分块传输协议） |
| `modules/adas_nodes/sensor_model_node.c` | 传感器模型（LiDAR/GPS/Camera，FOV/遮挡/噪声） |
| `modules/adas_nodes/safety_control_node.cpp` | FlowCoro 协程安全控制（TTC/横向交叉/行人防护） |
| `tools/flowboard.html` | 前端仪表盘（3D+2D+图表+D3 拓扑） |
| `tools/foxglove_bridge.py` | Foxglove Studio WebSocket 桥接 |
| `scripts/demo.sh` | 一键启动脚本 |

## 运行

```bash
bash scripts/demo.sh [duration]          # 启动演示
bash scripts/demo.sh --no-browser 15     # 不打开浏览器
```

仪表盘: `http://localhost:8800`
3D 桥接: `ws://localhost:8765`

## 最新 tag

`v0.1.0` — 创始版本，8 节点全链路稳定运行

---

# 可视化架构

## 当前可视化链路

### 主链路：flowmond (C HTTP 服务 + IPC 桥接)

```
pipeline 节点 (monitor_node)                  ← 10Hz 发布 dashboard JSON
  │  dashboard_bridge_publish() → IPC SHM → dashboard_bridge_subscriber()
  │  stats_bridge_publish()     → IPC SHM → stats_bridge_subscriber()
  ▼
flowmond :8800 (monitor_server.c — 多线程 HTTP/SSE)
  │  内嵌 HTTP 服务，直接读取 bus + discovery + IPC 聚合统计
  │  /tools/ 路由直接服务 D3.js / Three.js 静态文件（离线可用）
  │  自动检测 IPC 断连并重连（IPC_RECONNECT_STALE_SEC=5s）
  │  过期缓存自动 fallback 到本地 bus 数据
  ▼
浏览器 (flowboard.html — Three.js 3D + Canvas 2D + D3 拓扑)
```

### 辅助链路：文件桥接（foxglove MCAP 回放）

```
monitor_node ─→ 10Hz 写 /tmp/flow_topology.json ─→ foxglove_bridge.py :8765
```

## 关键说明

| 组件 | 端口 | 说明 |
|------|------|------|
| `flowmond` (C) | 8800 | 主仪表盘：HTTP/SSE + IPC 统计聚合 + 拓扑发现 + 自动重连 |
| `foxglove_bridge.py` | 8765 | Foxglove Studio 桥接（读取状态文件） |

## 重启恢复

杀掉 demo.sh 后重启，flowmond 的 IPC 桥接线程会检测数据断流（>5s），自动关闭旧通道并重连到新 pipeline 创建的共享内存。仪表盘在 pipeline 离线期间显示本地发现数据（不再卡在过期缓存），pipeline 恢复后自动切回实时数据。

## 详细文档

- [可视化架构](docs/VISUALIZATION_ARCHITECTURE.md) — API 端点、数据契约、鲁棒性设计
- [监控架构](docs/MONITORING_ARCHITECTURE.md) — flowmond 设计目标与实现
- [E2E 仿真设计](docs/E2E_SIMULATION_DESIGN.md) — offline 根因修复与 3D 仿真
