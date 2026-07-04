# FlowEngine 可视化架构

## 架构原则

```
一个监控守护进程 + 一个仪表盘前端 + 零外部依赖
```

## 组件

```
┌─────────────────────────────────────────────────────┐
│                   flowmond :8800                      │
│              (监控守护进程 — 只启动一次)               │
│                                                       │
│  UDP Discovery → 发现所有业务节点                      │
│  Message Bus   → 订阅 topic 统计                      │
│  HTTP Server   → /api/topics /api/stream /            │
│                                                       │
│  职责: 聚合所有节点的 topic 数据, 提供 HTTP API        │
└─────────────────────────────────────────────────────┘
         ▲                              │
         │ UDP multicast               │ HTTP SSE
         │                              ▼
┌────────────────────┐        ┌──────────────────────┐
│   flow_e2e         │        │   FlowBoard Dashboard │
│   (业务节点)        │        │   (浏览器打开)        │
│                     │        │                       │
│   不启动监控服务器   │        │   连接 flowmond:8800  │
│   只发布 topic 数据  │        │   实时 SSE 推送       │
└────────────────────┘        └──────────────────────┘
```

## 使用方式

```bash
# 1. 启动监控守护进程（只启动一次）
./build/bin/flowmond --port 8800 &

# 2. 启动业务节点（可以启动多个）
./build/bin/flow_e2e 3600 &

# 3. 浏览器打开仪表盘
open http://localhost:8800

# 4. CLI 查询
curl http://localhost:8800/api/topics
flowctl topic stats sensor/lidar
```

## 已废弃的组件

| 组件 | 原因 |
|------|------|
| `flowboard_server.py` | 被 `flowmond` 内置 HTTP 服务器替代 |
| `monitor_server` (嵌入 e2e) | 与 flowmond 职责重叠 |

## API 端点

```
GET /              → 实时仪表盘 HTML (SSE 自动更新)
GET /api/topology  → Bus 统计 + 发现拓扑 JSON
GET /api/topics    → Per-topic QoS 统计 (pub/del/drop/lat/freq)
GET /api/stream    → SSE 事件流 (500ms 推送)
```

## 数据流

```
flow_e2e (业务节点)
  │
  ├─ message_bus_publish("sensor/lidar", ...)  → 内部总线
  ├─ message_bus_publish("sensor/gps", ...)
  └─ discovery_advertise(...)                   → UDP 组播
        │
        ▼
flowmond (监控守护进程)
  │
  ├─ UDP 发现 → 知道有哪些节点
  ├─ 统计聚合 → pub/del/drop/latency per topic
  └─ HTTP SSE  → 推送到浏览器
        │
        ▼
FlowBoard (浏览器)
  │
  ├─ fetch /api/topology  → 初始加载
  └─ EventSource /api/stream → 实时更新
```
