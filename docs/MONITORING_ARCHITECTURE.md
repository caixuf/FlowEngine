# FlowEngine 监控与数据采集架构

## 设计原则

```
监控进程 ≠ 业务进程
数据采集 ≠ 业务逻辑

监控是独立的一等公民，通过 discovery 发现目标，通过 bus 订阅数据。
```

## 架构图

```
┌─────────────────────────────────────────────────────────────┐
│                     Production Cluster                       │
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────────┐ │
│  │perception│  │  fusion  │  │ control  │  │  planning  │ │
│  │  :8801   │  │  :8802   │  │  :8803   │  │  :8804     │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬───────┘ │
│       │              │              │              │         │
│       └──────────────┼──────────────┼──────────────┘         │
│                      │              │                         │
│              UDP Multicast Discovery (239.255.0.100:5500)    │
│                      │              │                         │
│       ┌──────────────┼──────────────┼──────────────┐         │
│       │              │              │              │         │
│  ┌────┴─────┐  ┌─────┴─────┐  ┌────┴─────┐  ┌────┴───────┐ │
│  │ flowmond │  │ flowrec   │  │ flowctl  │  │ FlowBoard  │ │
│  │ :8800    │  │ (daemon)  │  │ (cli)    │  │ (browser)  │ │
│  │ Monitor  │  │ Collector │  │ Query    │  │ Dashboard  │ │
│  └──────────┘  └──────────┘  └──────────┘  └──────────────┘ │
│                                                              │
│  flowmond: 实时监控 + Dashboard + Alert                      │
│  flowrec:  配置驱动数据采集 + Bag 录制                        │
│  flowctl:  命令行查询 + 参数管理                              │
│  FlowBoard: 浏览器可视化 (连接 flowmond)                     │
└─────────────────────────────────────────────────────────────┘
```

## 组件详解

### flowmond — 监控守护进程

```
flowmond
├── Discovery Client     ← UDP 组播发现所有节点
├── Topic Subscriber     ← 订阅 */stats 获取 topic 统计
├── Health Checker       ← 心跳超时检测 + 告警
├── Metrics Aggregator   ← 汇总所有节点的 topic 统计
├── Alert Manager        ← 规则引擎 (drop > N → alert)
├── HTTP Server :8800    ← Dashboard + JSON API
└── WebSocket Server     ← 实时推送到 FlowBoard
```

启动:
```bash
flowmond --port 8800 --config monitor.yaml
```

### flowrec — 数据采集守护进程

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
    pre_buffer_sec: 5    # 触发前 5 秒也保留
    post_buffer_sec: 10  # 触发后继续录 10 秒

  - name: "periodic_snapshot"
    topics: ["*"]          # 所有 topic
    output: "/data/bags/snapshot_%H.bag"
    max_size_mb: 2048
    rotation: "hourly"
    trigger:
      type: "cron"
      schedule: "*/10 * * * *"   # 每 10 分钟录 1 分钟
      duration_sec: 60
```

启动:
```bash
flowrec --config flowrec.yaml
```

## 配置驱动采集

flowrec 通过 JSON/YAML 配置决定采集策略，**不需要修改任何业务代码**:

```json
{
  "collectors": [
    {
      "name": "safety_critical",
      "topics": ["control/cmd", "fusion/state", "perception/objects"],
      "output_dir": "/data/recordings",
      "max_file_mb": 512,
      "trigger": {
        "type": "always_on"
      }
    },
    {
      "name": "debug_on_demand",
      "topics": ["*"],
      "output_dir": "/tmp/debug",
      "trigger": {
        "type": "manual",
        "api_endpoint": "/api/trigger_debug_capture"
      },
      "max_duration_sec": 60
    }
  ]
}
```

## flowmond 告警规则

```json
{
  "alerts": [
    {
      "name": "high_drop_rate",
      "condition": "topic.*.drop_rate > 10",
      "action": "log_error + webhook",
      "webhook_url": "http://alertmanager:9093/alert"
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

## 与 CyberRT cyber_monitor 对比

| 功能 | cyber_monitor | FlowEngine |
|------|-------------|------------|
| Topic 列表 | ✅ | ✅ flowmond + flowctl |
| 实时发布频率 | ✅ | ✅ 内嵌 HTTP SSE |
| 延迟统计 | ✅ | ✅ per-topic p50/p99 |
| 丢包统计 | ✅ | ✅ per-topic drop |
| 拓扑可视化 | ✅ | ✅ FlowBoard D3 力导向 |
| **配置驱动采集** | ⚠️ 硬编码 | ✅ flowrec YAML |
| **触发条件录制** | ❌ | ✅ topic_value / cron / manual |
| **独立监控进程** | ✅ | ✅ flowmond |
| **告警规则** | ⚠️ | ✅ rule engine |
| **WebSocket 推送** | ❌ | ✅ 实时 Dashboard |

## 快速开始

```bash
# 1. 启动业务节点
./build/bin/flow_e2e 3600 &

# 2. 启动监控守护进程
./build/bin/flowmond --port 8800 &

# 3. 启动数据采集
./build/bin/flowrec --config config/flowrec.yaml &

# 4. 浏览器查看
open http://localhost:8800

# 5. CLI 查询
flowctl list topics
flowctl topic stats sensor/lidar
```
