# FlowEngine 监控与数据采集架构

> **当前可用的监控/可视化链路：**
>
> **主链路（文件桥接）：**
> ```
> flow_launcher → 写 /tmp/flow_topology.json → flowboard_server.py (Python HTTP) → 浏览器
> ```
>
> **辅助链路（flowmond IPC 桥接）：**
> `stats_bridge` 跨进程 IPC 通道已实现（`src/core/stats_bridge.c`）。
> 业务进程通过 `stats_bridge_publish()` 发布 TopicStats，flowmond 通过
> `stats_bridge_subscriber_open()` 订阅聚合。仪表盘 JSON 通过 `dashboard_bridge`
> (`src/core/dashboard_bridge.c`) 跨进程传输。
> - `flowmond` 有独立的 `MessageBus`，不共享业务节点总线
> - 跨机 TCP bridge 尚未实现，当前仅支持同机 POSIX SHM
>
> ---

## 设计原则

```
监控进程 ≠ 业务进程
数据采集 ≠ 业务逻辑

监控是独立的一等公民，通过 IPC stats bridge 接收业务进程的统计快照。
```

## 跨进程监控架构

每个业务进程（如 `flow_launcher`）和监控守护进程（`flowmond`）各自拥有独立的
`MessageBus`，**进程间的 bus 实例不共享**。跨进程聚合通过
`stats_bridge` IPC 通道实现：

```
┌──────────────────────────────────────────────────────────────┐
│  Business Process (flow_launcher)                                  │
│                                                               │
│  PerceptionTask → FusionTask → ControlTask                   │
│       ↓                ↓             ↓                        │
│  ════════ MessageBus (e2e_bus) ═════════                     │
│                       ↓                                       │
│         stats_bridge_publish() every 5s                       │
│                       ↓                                       │
│       POSIX shm IPC channel "flow_stats_bridge"               │
└───────────────────────┬───────────────────────────────────────┘
                        │  StatsPacket (compact TopicStats[])
┌───────────────────────▼───────────────────────────────────────┐
│  flowmond                                                     │
│                                                               │
│  stats_bridge_subscriber_open()  ← IPC receive thread        │
│       ↓                                                       │
│  monitor_server_inject_remote_stats()                         │
│       ↓                                                       │
│  MonitorServer (HTTP :8800)  ← merges local + remote stats   │
│       ↓                                                       │
│  /api/stream SSE → FlowBoard (browser)                        │
└───────────────────────────────────────────────────────────────┘
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `include/stats_bridge.h` | StatsPacket 定义 + IPC bridge API |
| `src/core/stats_bridge.c` | 序列化 TopicStats → StatsPacket，IPC 发布/订阅 |
| `src/core/monitor_server.c` | HTTP 服务器，合并本地 + 远程 stats |
| `src/flowmond.c` | 订阅 IPC stats channel，注入 MonitorServer |
| `src/flow_launcher.c` | 发布 bus stats 到 IPC channel（后台线程，5秒周期）|

## 组件详解

### stats_bridge — IPC 统计数据桥接

业务进程发布端：

```c
IpcChannel* ch = stats_bridge_publisher_open();
stats_bridge_publish(ch, g_bus, "flow_launcher");  // 每 5 秒调用一次
```

监控进程订阅端（带自动重连）：

```c
/* flowmond 内部的重连循环 */
while (g_running) {
    IpcChannel* sub = stats_bridge_subscriber_open(on_remote_stats, NULL);
    if (sub) { ipc_channel_start(sub); /* 后台线程 */ break; }
    sleep(2);  /* 等待业务进程启动 */
}
```

`StatsPacket` 格式（< 2 KB，每条 IPC 消息携带最多 16 个 topic 的统计）：

```c
typedef struct {
    char            source_name[64];  /* e.g., "flow_launcher" */
    uint32_t        topic_count;
    uint64_t        bus_pub, bus_del, bus_drop;
    RemoteTopicStat topics[STATS_BRIDGE_MAX_TOPICS];  /* 16 topics */
} StatsPacket;
```

### flowmond — 监控守护进程

```
flowmond
├── Discovery Client       ← UDP 组播发现所有节点
├── Stats Bridge Subscriber← IPC channel "flow_stats_bridge"（自动重连）
├── MonitorServer          ← HTTP :8800，合并本地+远程 stats
│   ├── /api/stream (SSE)  ← 500ms 推送 local + remote topics
│   ├── /api/topology      ← 拓扑 JSON
│   └── /api/topics        ← 所有 topic 统计（标注 source 字段）
└── Alert Engine           ← 丢包/延迟告警
```

启动:
```bash
./build/bin/flowmond --port 8800          # 先启动
./build/bin/flow_launcher config/pipeline.json --duration 60                   # 再启动业务进程
# 约 5 秒后，flowmond dashboard 将显示来自 flow_launcher 的 topic 统计
```

### flowrec — 数据采集守护进程（计划中）

```
flowrec
├── Config Loader        ← 读取采集规则
├── Topic Matcher        ← 匹配需要录制的 topic
├── Bag Writer           ← 写入 bag 文件 (v2 格式)
├── Trigger Engine       ← 条件触发 (时间/事件/手动)
├── Rotation Manager     ← 文件轮转 (大小/时间)
└── Status Reporter      ← 采集状态上报给 flowmond
```

配置:
```yaml
# flowrec.yaml
collectors:
  - name: "sensor_raw"
    topics: ["sensor/lidar", "sensor/gps", "sensor/camera"]
    output: "/data/bags/sensor_%Y%m%d_%H%M%S.bag"
    max_size_mb: 1024
    rotation: "hourly"
    trigger: "always_on"

  - name: "event_triggered"
    topics: ["perception/objects", "control/cmd", "fusion/state"]
    output: "/data/bags/event_%Y%m%d_%H%M%S.bag"
    max_size_mb: 512
    trigger:
      type: "topic_value"
      topic: "control/cmd"
      field: "emergency_brake"
      condition: "== true"
    pre_buffer_sec: 5
    post_buffer_sec: 10
```

## flowmond 告警规则

```json
{
  "alerts": [
    {
      "name": "high_drop_rate",
      "condition": "topic.*.drop_rate > 10",
      "action": "log_error + webhook"
    },
    {
      "name": "node_offline",
      "condition": "node.*.alive == false for 30s",
      "action": "log_error + webhook"
    },
    {
      "name": "high_latency",
      "condition": "topic.control/cmd.p99_latency > 5000",
      "action": "log_warn"
    }
  ]
}
```

## 已知限制

| 限制 | 说明 |
|------|------|
| 单机 IPC only | `stats_bridge` 基于 POSIX shm，仅支持同机进程间通信；跨机聚合需要 TCP bridge（未实现）|
| 统计延迟 5s | 业务进程每 5 秒发布一次 stats 快照，非实时；本地总线数据是实时的 |
| 最多 16 个 topic | `StatsPacket` 单包限制；超过 16 个 topic 时截断（后续可分包）|
| flowrec 未实现 | 配置驱动数据采集仅在设计阶段 |

## 与 CyberRT cyber_monitor 对比

| 功能 | cyber_monitor | FlowEngine |
|------|-------------|------------|
| Topic 列表 | ✅ | ✅ flowmond + flowctl |
| 实时发布频率 | ✅ | ✅ 内嵌 HTTP SSE |
| 延迟统计 | ✅ | ✅ per-topic p50/p99 |
| 丢包统计 | ✅ | ✅ per-topic drop |
| 拓扑可视化 | ✅ | ✅ FlowBoard D3 力导向 |
| **跨进程统计聚合** | ✅ | ✅ stats_bridge IPC（同机）|
| **配置驱动采集** | ⚠️ 硬编码 | 🚧 flowrec YAML（计划中）|
| **触发条件录制** | ❌ | 🚧（计划中）|
| **独立监控进程** | ✅ | ✅ flowmond |
| **告警规则** | ⚠️ | ✅ rule engine |
| **WebSocket 推送** | ❌ | ✅ 实时 Dashboard |

## 快速开始

```bash
# 1. 构建
cmake -B build && cmake --build build --target flowmond flow_launcher

# 2. 启动监控守护进程（先启动，stats bridge 会自动等待业务进程）
./build/bin/flowmond --port 8800 &

# 3. 启动业务进程（flow_launcher 会自动开始发布 stats 到 IPC channel）
./build/bin/flow_launcher config/pipeline.json --duration 60

# 4. 约 5 秒后，浏览器查看（本地 + flow_launcher 的 topic 统计均可见）
open http://localhost:8800
```

