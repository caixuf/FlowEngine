# FlowEngine — 项目概览

轻量级自动驾驶中间件，核心是一个 Pub/Sub 消息总线 + 调度器 + 传输层。

## 架构

```
PerceptionTask → FusionTask → ControlTask → MonitorTask
     ↓              ↓             ↓             ↓
  sensor/lidar   sensor/gps   fusion/obj    control/cmd
     ↓              ↓             ↓             ↓
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
| `src/flowmond.c` | 监控守护进程（HTTP 仪表盘 + IPC 统计桥接） |
| `src/core/monitor_server.c` | 内嵌 HTTP 服务器（流量盘/API/SSE） |
| `src/core/stats_bridge.c` | 跨进程 topic 统计 IPC 桥接 |
| `tools/flowboard.html` | 前端仪表盘（3D+2D+图表+拓扑） |
| `tools/foxglove_bridge.py` | Foxglove Studio WebSocket 桥接 |
| `scripts/demo.sh` | 一键启动脚本 |

## 运行

```bash
bash scripts/demo.sh [duration]          # 启动演示
bash scripts/demo.sh --no-browser 15     # 不打开浏览器
```

仪表盘: `http://localhost:8800`
3D 桥接: `ws://localhost:8765`

---

# 可视化架构

## 当前可视化链路

### 主链路：flowmond (C HTTP 服务 + IPC 桥接)

```
pipeline 节点 (monitor_node)
  │  stats_bridge_publish() → IPC SHM → stats_bridge_subscriber()
  ▼
flowmond :8800 (monitor_server.c — HTTP/SSE)
  │  内嵌 HTTP 服务，直接读取 bus + discovery + IPC 聚合统计
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
| `flowmond` (C) | 8800 | 主仪表盘：HTTP/SSE + IPC 统计聚合 + 拓扑发现 |
| `foxglove_bridge.py` | 8765 | Foxglove Studio 桥接（读取状态文件） |

## 详细文档

- [可视化架构](docs/VISUALIZATION_ARCHITECTURE.md) — API 端点、数据契约、鲁棒性设计
- [监控架构](docs/MONITORING_ARCHITECTURE.md) — flowmond 设计目标与实现
- [E2E 仿真设计](docs/E2E_SIMULATION_DESIGN.md) — offline 根因修复与 3D 仿真
