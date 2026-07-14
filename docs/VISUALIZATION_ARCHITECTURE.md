# FlowEngine 可视化架构

## 架构原则

```
业务节点写状态文件 → 轻量 HTTP 桥接 → 浏览器仪表盘 (零外部依赖)
```

FlowEngine 提供两种可视化链路，二者互补：

| 链路 | 采集方式 | 适用场景 | 状态 |
|------|----------|----------|------|
| **文件桥接** (`flowboard_server.py`) | 读取业务进程写出的状态 JSON (`/tmp/flow_topology.json`) | launch 仿真 / 演示 / 单机 | **当前默认，demo 脚本使用** |
| **监控守护进程** (`flowmond`) | 通过 UDP 发现 + 内置总线聚合多节点 | 分布式多节点部署 | 可选 / 演进中 |

> 重要：`flowmond` 拥有自己的进程内 Message Bus，**看不到** `flow_launcher`
> 进程内部的 topic 与仿真状态。因此 launch 演示必须使用文件桥接链路。
> 之前将 `flowboard_server.py` 标记为"已废弃"是错误的，已在本次修订中纠正。

## 组件（文件桥接链路，默认）

```
┌────────────────────┐   写状态文件    ┌──────────────────────────┐
│   flow_launcher config/pipeline.json │ ───────────────▶│  /tmp/flow_topology.json  │
│   (业务/仿真节点)   │  monitor 任务   │  (原子 tmp+rename, ~1Hz)  │
│                     │   10Hz 采集     └──────────────────────────┘
│  Message Bus        │                              │ 文件监听
│  Transport / Sched  │                              ▼
│  3D 仿真 (ego/障碍/  │              ┌──────────────────────────┐
│  LiDAR)             │              │  flowboard_server.py :8800 │
└────────────────────┘              │  (多线程 HTTP + SSE 桥接)  │
                                     │  /api/topology /api/stream │
                                     │  /api/health  freshness    │
                                     └──────────────────────────┘
                                                  │ HTTP SSE
                                                  ▼
                                     ┌──────────────────────────┐
                                     │   FlowBoard Dashboard     │
                                     │   (浏览器, Three.js 3D)   │
                                     │   LIVE / STALE / OFFLINE  │
                                     └──────────────────────────┘
```

## 使用方式（默认文件桥接）

```bash
# 1. 启动业务/仿真节点（写状态文件）
./build/bin/flow_launcher config/pipeline.json --duration 3600 &

# 2. 启动 HTTP 桥接（读状态文件, 多线程）
python3 tools/flowboard_server.py &

# 3. 浏览器打开仪表盘
open http://localhost:8800

# 或直接使用一键脚本
./scripts/demo.sh
```

## 使用方式（可选 flowmond 多节点）

```bash
# 分布式部署时, 由 flowmond 通过 UDP 发现聚合多个节点
./build/bin/flowmond --port 8800 &
./build/bin/flow_launcher config/pipeline.json --duration 3600 &        # 需要节点对外广播 topic 统计
open http://localhost:8800
```

## 鲁棒性设计（解决"离线"反复出现）

`flowboard_server.py` 与前端针对历史上反复出现的"离线"问题做了如下加固：

1. **多线程 HTTP 服务器**：改用 `ThreadingHTTPServer`。此前单线程服务器
   在一个 SSE 长连接（阻塞 ~300s）期间会饿死所有其它请求（`/api/topology`
   超时返回 0 字节），前端因此翻转为离线并静默显示假数据。
2. **数据新鲜度**：桥接层记录状态文件最后更新时间，超过 `STALE_AFTER_SEC`
   标记为 `stale`，并在 `/api/topology`、SSE 快照中返回 `stale` / `age_sec`。
3. **`/api/health` 端点**：返回 `status` / `source` / `age_sec`，便于探活。
4. **SSE 心跳**：每 15s 发送 `: keep-alive` 注释帧，避免代理断开；连接使用
   墙钟截止时间（300s）而非计数循环。
5. **前端三态**：`applyLiveStatus()` 依据 `source`/`stale`/`age_sec` 显示
   **LIVE / STALE / OFFLINE** 三态；数据陈旧时保留最后一帧真实数据，
   **不再静默切换到模拟数据**。
6. **后端并发死锁修复**：`recv_thread_fn` 不再在持有 `peers_mutex` 期间
   `usleep` 或 `message_bus_publish`，避免饿死写状态文件的 monitor 线程
   （曾导致状态文件停止更新 → 前端离线）。

## API 端点

```
GET /              → 实时仪表盘 HTML (SSE 自动更新)
GET /api/topology  → Bus 统计 + 拓扑 + 车辆/场景 JSON (含 source/stale/age_sec)
GET /api/stream    → SSE 事件流 (含心跳, source/stale 标记)
GET /api/health    → 探活: {status, source, age_sec}
```

## 数据流

```
flow_launcher config/pipeline.json (业务/仿真节点)
  │  monitor 任务 (10Hz)
  ├─ 采集 bus/transport/scheduler/latency/topics 统计
  ├─ 采集车辆闭环遥测 (PID: speed/target/throttle/brake)
  ├─ 采集 3D 场景 scene{ego, obstacles, lidar, lane}
  └─ 原子写出 /tmp/flow_topology.json (tmp + rename)
        │ 文件监听
        ▼
flowboard_server.py (HTTP 桥接, 多线程)
  ├─ file_watcher 读取并记录 g_last_update
  ├─ 新鲜度判定 (_is_stale) → source: live/stale/demo
  └─ HTTP / SSE 推送
        │
        ▼
FlowBoard (浏览器)
  ├─ fetch /api/topology  → 初始加载
  ├─ EventSource /api/stream → 实时更新
  ├─ applyLiveStatus → LIVE/STALE/OFFLINE 三态
  └─ Three.js 渲染 scene (ego 车 + 障碍物包围盒 + LiDAR 点云)
```

## `scene` 数据结构（真实 3D 仿真）

`monitor` 任务在 `metrics.scene` 下导出真实仿真场景，前端据此渲染真实
三维场景（而非随机点）。坐标系：**自车系, x 前为正, y 左为正 (米)**，
`heading` 为弧度。向后兼容——若 `scene` 缺失，前端回退到旧的占位渲染。

```json
{
  "scene": {
    "ego":       { "x": 45.1, "y": 0.0, "heading": 0.0, "speed": 10.1, "steer": 0.03 },
    "lane":      { "width": 3.5, "count": 2 },
    "obstacles": [
      { "id": 0, "type": "car",        "x": 21.7, "y": 0.0,  "vx": 6.0,  "len": 4.6, "wid": 2.0 },
      { "id": 1, "type": "car",        "x": 60.0, "y": -3.5, "vx": -9.0, "len": 4.6, "wid": 2.0 },
      { "id": 2, "type": "pedestrian", "x": 30.0, "y": 8.0,  "vx": 0.0,  "len": 0.6, "wid": 0.6 }
    ],
    "lidar":     [ [px, py, pz], ... ]
  }
}
```

- `obstacles` 中的坐标已转换到**自车系**（相对自车的前向/横向距离）。
- `lidar` 为对障碍物表面 + 地面环带做光线投射后的下采样点云（自车系）。
- 自车运动含横向自行车模型（转向 P 控制向目标车道），并具备 ACC
  （依据同车道前车间距动态限制目标速度）与变道演示。
