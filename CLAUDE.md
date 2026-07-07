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
                  FlowBoard Server (HTTP/SSE) → DashBoard
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/e2e_demo.c` | 端到端演示：感知→融合→控制→监控 |
| `src/core/message_bus.c` | 进程内 Pub/Sub 总线 |
| `src/core/transport.c` | 统一传输抽象（local/IPC/TCP） |
| `src/core/scheduler.c` | 任务调度器（classic/choreo 模式） |
| `tools/flowboard_server.py` | 仪表盘 HTTP/SSE 服务器 |
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

## 当前唯一可用链路

```
flow_e2e ─→ 10Hz 写 /tmp/flow_topology.json ─→ flowboard_server.py :8800 ─→ 浏览器
```

> **核心原则**：所有数据通过**状态文件 JSON 桥接**，不依赖跨进程 topic 聚合。

## 关键说明

| 组件 | 状态 |
|------|------|
| `flowboard_server.py` (HTTP/SSE) | ✅ 当前默认，demo 脚本使用 |
| `flowboard.html` (Three.js 3D + Canvas 2D) | ✅ 由 flowboard_server 服务 |
| `foxglove_bridge.py` (WebSocket @8765) | ✅ MCAP 回放可视化 |
| `flowmond` (监控守护进程) | ⚠️ 创建独立 MessageBus，无法获取业务节点 topic 统计；跨进程 IPC/TCP bridge 未实现 |

## 详细文档

- [可视化架构](docs/VISUALIZATION_ARCHITECTURE.md) — API 端点、数据契约、鲁棒性设计
- [监控架构](docs/MONITORING_ARCHITECTURE.md) — flowmond 设计目标与当前局限
- [E2E 仿真设计](docs/E2E_SIMULATION_DESIGN.md) — offline 根因修复与 3D 仿真 |
