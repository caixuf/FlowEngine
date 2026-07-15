# FlowEngine

> Simulation-first middleware framework for autonomous driving & robotics — C11 kernel, C++20 coroutine shell, plugin architecture.
>
> **Scope:** FlowEngine is a *simulation-first, reproducible experiment platform*. It deliberately does **not**
> target real-vehicle deployment (no automotive mass production, no real ECU/CAN integration, no hard real-time
> or functional-safety certification). Everything — perception, fusion, planning, control, learning — is
> exercised, observed, tested, replayed and scored **entirely in simulation**.

[![CI](https://github.com/caixuf/FlowEngine/actions/workflows/ci.yml/badge.svg)](https://github.com/caixuf/FlowEngine/actions)
![License](https://img.shields.io/badge/license-MIT-blue)
![C](https://img.shields.io/badge/C-11-555555)
![C++](https://img.shields.io/badge/C++-20-659ad2)

---

## What is FlowEngine?

A from-scratch middleware framework inspired by Apollo CyberRT, providing the core abstractions in a lightweight,
embeddable package. Built to be **organizable, observable, testable, replayable and scoreable — all inside simulation**:

| Layer | Modules |
|-------|---------|
| **Communication** | Message Bus (pub/sub + req/reply + zero-copy), IPC (SHM), TCP Transport, Network Transport |
| **Execution** | Coroutine Scheduler (FIFO + CPU affinity + rate limit), Choreo DAG mode, Cancelable Coroutine Primitives (pub/sub · select · timer · req-reply, with timeout & graceful cancel) |
| **Introspection** | Reflective State Machine, UDP Service Discovery, Topology Tracking, SysMonitor |
| **Metadata** | FlowRegistry (tasks/topics/types/plugins/schemas), ParamRegistry (int/float/bool/string with hot-reload) |
| **Data** | Type-safe Serialization (IDL + codegen), Bag v2 Record/Replay, MCAP format, Data Fusion (EKF), Schema Validation |
| **QoS** | Per-topic QoS (depth + drop policy + deadline + reliability), Topic Stats (frequency, latency p50/p99, subscribers) |
| **Perception** | DBSCAN LiDAR clustering, Kalman tracking, EKF sensor fusion, NMEA GPS parser, nuScenes dataset loader |
| **Planning** | Frenet Optimal Trajectory (lane change / overtake), PID control (longitudinal + lateral) |
| **Safety** | FlowCoro coroutine-based safety envelope (TTC / lateral cross / pedestrian guard) |
| **Operations** | Unified Logger (ms timestamps), flowctl CLI, FlowBoard Dashboard (Three.js 3D + 2D), file-bridge or flowmond monitor server, Cross-process IPC Stats Bridge + Topic Bridge, CI/CD |
| **Learning** | In-sim learning loop: data recorder → offline trainer (scikit-learn MLP / PyTorch) → shadow-mode tiny-MLP inference + onboard SGD fine-tuning + model OTA with A-B comparison. See [docs/LEARNING_LOOP.md](docs/LEARNING_LOOP.md) |

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        FlowEngine Core (C11)                          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐ │
│  │ Message  │ │   IPC    │ │   Bag    │ │  Clock   │ │   State    │ │
│  │   Bus    │ │  (SHM)   │ │ (v2/MCAP)│ │ Service  │ │  Machine   │ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └────────────┘ │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐ │
│  │ Flow     │ │  Param   │ │ Discovery│ │ Serializer│ │   Task     │ │
│  │ Registry │ │ Registry │ │  (UDP)   │ │(IDL+FNV) │ │  Manager   │ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └────────────┘ │
├──────────────────────────────────────────────────────────────────────┤
│                     FlowEngine Shell (C++20)                          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐ │
│  │ Coroutine│ │Scheduler │ │  Fusion  │ │Transport │ │  Network   │ │
│  │  Tasks   │ │(Choreo)  │ │ (EKF)    │ │(TCP/IPC) │ │  Transport │ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └────────────┘ │
├──────────────────────────────────────────────────────────────────────┤
│                     ADAS Pipeline (dlopen plugins)                     │
│  sim_world → sensor_model → perception → fusion → planning → control │
│    → safety_control → inference → data_recorder → learner → model_ota│
│    → monitor                                                         │
└──────────────────────────────────────────────────────────────────────┘
                                                                         │
                      ════════════════════┼════════════════════
                                         │
                          ┌──────────────▼─────────────────────┐
                          │  文件桥接（默认）                     │
                          │  monitor_node → /tmp/flow_topology.json │
                          │  → flowboard_server.py :8800 → 浏览器    │
                          │  或                                     │
                          │  flowmond (IPC SHM) :8800 → 浏览器      │
                          └────────────────────────────────────────┘
```

**两种可视化链路互补：** 默认使用文件桥接（`flowboard_server.py` 读取 JSON 文件），
可选 `flowmond` 守护进程通过 IPC SHM 聚合多进程统计。详见 [docs/VISUALIZATION_ARCHITECTURE.md](docs/VISUALIZATION_ARCHITECTURE.md)。

---

## Quick Start

```bash
git clone https://github.com/caixuf/FlowEngine.git && cd FlowEngine

# One-click demo (build + run, default 15s)
bash scripts/demo.sh

# Or build manually
bash build.sh release
```

> **Entry point:** `flow_launcher config/pipeline.json` is the canonical,
> config-driven way to run a pipeline (each node is a `dlopen`-loaded `.so`
> plugin).

---

## Demo

```bash
bash scripts/demo.sh 30     # 30-second demo

# Other modes
bash scripts/demo.sh --multi      # 多进程模式（各节点独立 fork+exec）
bash scripts/demo.sh --record     # 录制 Bag 文件
bash scripts/demo.sh --no-browser # 不打开浏览器
```

```
  ╔══════════════════════════════════════════════════════════╗
  ║                                                          ║
  ║   ███████╗██╗      ██████╗ ██╗    ██╗                  ║
  ║   ██╔════╝██║     ██╔═══██╗██║    ██║                  ║
  ║   █████╗  ██║     ██║   ██║██║ █╗ ██║                  ║
  ║   ██╔══╝  ██║     ██║   ██║██║███╗██║                  ║
  ║   ██║     ███████╗╚██████╔╝╚███╔███╔╝                  ║
  ║   ╚═╝     ╚══════╝ ╚═════╝  ╚══╝╚══╝                   ║
  ║                                                          ║
  ║   E N G I N E                                           ║
  ║   Lightweight Middleware for Autonomous Driving          ║
  ║                                                          ║
  ╚══════════════════════════════════════════════════════════╝

  ┌─ SimWorld ─→  Perception ─→  Fusion  ─→  Planning ─→  Control ┐
  │  dynamics      DBSCAN          EKF          Frenet       PID     │
  └──────────────────────────────────────────────────────────────────┘

  ⏱ 15s  |  pub=133 del=239 lat=141µs speed=11.2m/s
```

![Dashboard](docs/dashboard.png)
> *FlowBoard real-time dashboard — topology graph, 3D scene, frame monitor, latency charts. Open `http://localhost:8800` during demo.*

**实时服务：**
| 服务 | 端口 | 说明 |
|------|------|------|
| FlowBoard Dashboard | `:8800` | 实时仪表盘（3D + 2D + D3 拓扑） |
| Foxglove 3D Bridge | `:8765` | Foxglove Studio WebSocket 桥接 |

---

## Pipeline

默认配置（`config/pipeline.json`）启动 **11 个插件节点**，默认场景为弯道巡航（`scenarios/curve_road.json`）：

| 节点 | 插件 (.so) | 频率 | 功能 |
|------|-----------|------|------|
| `sim_world` | `libsim_world.so` | 50Hz | 车辆动力学 + 障碍物模拟 + 场景加载 |
| `sensor_model` | `libsensor_model.so` | 20Hz | LiDAR/GPS/Camera 传感器模型（FOV/遮挡/噪声） |
| `perception` | `libperception_node.so` | 10Hz | DBSCAN 点云聚类 + 目标检测 |
| `fusion` | `libfusion_node.so` | 20Hz | EKF 传感器融合（定位 + 时间对齐） |
| `planning` | `libplanning_node.so` | 20Hz | Frenet 最优轨迹规划（变道/超车） |
| `control` | `libcontrol_node.so` | 50Hz | PID 纵横向控制 + Stanley 转向 |
| `safety_control` | `libsafety_control_node.so` | 协程 | FlowCoro 安全包围盒（TTC/横向/行人） |
| `inference` | `libinference_node.so` | 20Hz | tiny-MLP 影子推理（shadow mode，不执行） |
| `data_recorder` | `libdata_recorder_node.so` | 20Hz | 训练样本采集（模仿学习 JSONL） |
| `learner` | `liblearner_node.so` | 0.5Hz | 车端增量 SGD 微调（full/partial） |
| `model_ota` | `libmodel_ota_node.so` | 1Hz | 模型 OTA + 版本管理 + A-B 对比 |
| `monitor` | `libmonitor_node.so` | 10Hz | 系统监控 + 仪表盘 JSON 导出 |

---

## CLI

```bash
flowctl list tasks              # 注册任务列表
flowctl list topics             # 所有 topic 及统计
flowctl graph                   # ASCII 拓扑
flowctl state <task>            # 状态机状态
flowctl topic stats <topic>     # 单 topic 延迟/吞吐
flowctl bag info <file>         # Bag 元信息
flowctl schema <type>           # 类型定义
flowctl dashboard               # 启动 FlowBoard
flowctl version                 # 构建信息
flowctl param list              # 参数列表
flowctl param get <name>        # 获取参数
```

**其他工具：**

| 二进制 | 说明 |
|--------|------|
| `flow_launcher` | 配置驱动 pipeline 启动器（dlopen 加载插件） |
| `flowmond` | 监控守护进程（HTTP 仪表盘 + IPC 桥接 + 自动重连） |
| `flow_node_host` | 单节点插件宿主进程（用于多进程 fork+exec 模式） |
| `flow_mcap_replay` | MCAP 回放工具 |
| `flow_bag` | Bag 录制/回放 CLI |
| `flow_e2e` | 端到端演示二进制 |

---

## Visualization

**主链路（文件桥接，默认）：**

```bash
# Terminal 1: run the pipeline (writes /tmp/flow_topology.json)
./build/bin/flow_launcher config/pipeline.json --duration 3600

# Terminal 2: start the dashboard server
python3 tools/flowboard_server.py --port 8800 --json-file /tmp/flow_topology.json

# Open browser
open http://localhost:8800
```

**辅助链路（flowmond IPC 桥接）：**

```bash
# Terminal 1: monitor daemon
./build/bin/flowmond --port 8800 --html-path tools/flowboard/index.html

# Terminal 2: pipeline (publishes stats via IPC SHM)
./build/bin/flow_launcher config/pipeline.json --duration 60
```

Dashboard endpoints:

| Path | Description |
|------|-------------|
| `/` | Live FlowBoard UI (3D scene + topology + charts) |
| `/api/topics` | Per-topic stats (频率/延迟/订阅者) |
| `/api/topology` | Topology JSON (nodes + edges + metrics) |
| `/api/stream` | SSE real-time push (500 ms interval) |
| `/api/health` | Health check |

> **Bind address:** `flowboard_server.py` listens on `0.0.0.0` by default.
> `flowmond` listens on `127.0.0.1` (loopback) by default. For remote access,
> use `flowmond --bind 0.0.0.0` (or set `FLOWMOND_BIND_ADDR=0.0.0.0`).

---

## Docker

```bash
docker build -t flowengine .
docker run --rm flowengine          # Run e2e demo
docker run --rm flowengine demo 30  # 30-second demo
```

---

## Build from Source

| Requirement | Version |
|-------------|---------|
| GCC | 11+ (C++20 coroutines) |
| CMake | 3.16+ |
| libcjson | any (`apt install libcjson-dev`) |
| libeigen3 | 3.3+ (`apt install libeigen3-dev`) — **required for Frenet planner** (lane change / overtaking); without it `planning_node` silently falls back to lane-keep-only |
| Python | 3.8+ (codegen & dashboard) |
| libprotobuf-c (optional) | for protobuf support |

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Or one-click
bash build.sh release
```

### Install

```bash
sudo cmake --install build

# Verify
pkg-config --cflags --libs flowengine
flowctl version
```

Once installed, include the umbrella header:

```c
#include "flowengine.h"   /* FLOWENGINE_VERSION, NODE_PLUGIN_API_VERSION, bus/transport/... */
```

---

## Plugin System

FlowEngine uses a `dlopen`-based plugin architecture. Each pipeline node is a shared library (`.so`)
loaded at runtime by `flow_launcher`. Nodes communicate exclusively through the Message Bus —
no direct function calls between nodes.

```c
// C plugin — dlopen compatible ABI
#include "task_interface.h"

typedef struct { TaskBase base; int param; } MyTask;

static int my_execute(TaskBase* base) {
    while (!base->should_stop) {
        /* business logic */
        sleep(1);
    }
    return 0;
}

static TaskInterface vtable = { .execute = my_execute };

TaskBase* create_task(const TaskConfig* cfg) {
    MyTask* t = calloc(1, sizeof(MyTask));
    task_base_init(&t->base, &vtable, cfg);
    return &t->base;
}
```

---

## Configuration-Driven Launch

```json
{
  "scheduler": { "mode": "choreo" },
  "processes": [{
    "name": "perception",
    "library_path": "build/lib/libperception_node.so",
    "auto_start": true,
    "subscribe": ["vehicle/state"],
    "publish": [{ "topic": "perception/obstacles", "type": "ObstacleList" }],
    "params": "{\"frequency_hz\":10.0}"
  }]
}
```

```bash
./build/bin/flow_launcher config/pipeline.json
```

---

## Scenario Suite

13 JSON 场景定义，覆盖典型自动驾驶场景：

| 场景 | 描述 |
|------|------|
| `curve_road.json` | 弯道巡航（当前默认场景） |
| `highway_overtake.json` | 高速超车 |
| `pedestrian_crossing.json` | 行人横穿 |
| `congestion_follow.json` | 拥堵跟车 |
| `cutin.json` | 车辆切入 |
| `ghost_pedestrian.json` | 鬼探头行人 |
| `highway_exit.json` | 高速出口 |
| `highway_noa_route.json` | 高速 NOA 路线 |
| `intersection_unprotected.json` | 无保护路口 |
| `multi_pedestrian.json` | 多行人场景 |
| `obstacle_avoid.json` | 障碍物避让 |
| `roadwork_zone.json` | 施工区 |
| `suite.json` | 场景测试集（批量回归用） |

> 每个场景有对应的 `scenarios/baseline/*.json` 回归基线。

---

## Learning Loop

FlowEngine 实现了完整的车端学习闭环：

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  data_recorder│───▶│  离线训练      │───▶│  inference    │
│  (JSONL 采样)  │    │  train.py     │    │  (tiny-MLP)   │
│  20Hz         │    │  train_e2e/   │    │  shadow mode  │
└──────────────┘    │  (PyTorch)    │    └──────┬───────┘
                    └──────────────┘           │
                                              ▼
                    ┌──────────────┐    ┌──────────────┐
                    │  model_ota   │◀───│   learner    │
                    │  (A-B 对比)   │    │  (SGD微调)    │
                    └──────────────┘    └──────────────┘
```

- **Stage 0:** `data_recorder_node` — 采集人类/规则驾驶样本（JSONL）
- **Stage 1:** 离线训练 — `tools/train/train.py`（scikit-learn MLP）或 `tools/train_e2e/`（PyTorch 时序训练）
- **Stage 2:** `inference_node` — tiny-MLP 影子推理，与规则控制器并行评估
- **Stage 3:** `learner_node` — 车端增量 SGD 微调（全量/部分更新）
- **Stage 4:** `model_ota_node` — 模型版本管理 + A-B 效果对比 + 动态切换

详见 [docs/LEARNING_LOOP.md](docs/LEARNING_LOOP.md)。

---

## Regression Evaluator

```bash
# Run demo + auto-score: topology, collisions, road departure, stagnation, yaw wobble
python3 tools/demo_evaluator.py --duration 45

# Analyze last run without re-launching
python3 tools/demo_evaluator.py --no-run

# Full scenario suite vs baseline
python3 tools/scenario_regression.py --baseline
```

The evaluator samples `/tmp/flow_topology.json` during a demo run and checks:
topology edges, topic frequencies, collision events, road departure, vehicle
stagnation, lane-change count, yaw/steer oscillation, NPC teleport jumps, and
message drops.  Run it after any change to the pipeline chain.

---

## Tests & CI

| Job | Status | Description |
|-----|--------|-------------|
| Release | ✅ | gcc -O2, unit tests |
| Debug | ✅ | gcc -g, unit tests |
| ASAN | ✅ | Address Sanitizer |
| UBSAN | ✅ | Undefined Behavior Sanitizer |
| Stress | ✅ | 15s pipeline at full rate |
| Integration | ✅ | Multi-node pipeline + ctest |
| Coverage | ✅ | lcov report |
| Viz | ✅ | FlowBoard Python tests + server smoke test |
| Evaluator | ✅ | 45s regression evaluator (PR gate) |
| Scenario Regression | 🌙 | Full scenario suite vs baseline (nightly/manual) |
| Nightly Stability | 🌙 | Long-running (schedule only) |

> **TSAN 当前禁用** — 协程 + 无锁内存池的跨线程同步模式对 TSAN 产生大量假阳性，
> 待协程生命周期稳定后重新启用。

---

## Skills (深度教程)

| Skill | Topic |
|-------|-------|
| [01 — OOP in C](skills/01_oop_in_c.md) | C 语言面向对象编程 |
| [02 — Plugin System](skills/02_plugin_system.md) | dlopen 插件架构设计 |
| [03 — Message Bus](skills/03_message_bus.md) | 零拷贝 Pub/Sub 总线 |
| [04 — IPC Channel](skills/04_ipc_channel.md) | POSIX SHM 进程间通信 |
| [05 — Bag Recording](skills/05_bag_recording.md) | Bag v2 录制与回放 |
| [06 — Clock Service](skills/06_clock_service.md) | 时钟服务与时间管理 |
| [07 — Serializer](skills/07_serializer.md) | IDL 代码生成与序列化 |
| [08 — State Machine](skills/08_state_machine.md) | 反射式状态机 |
| [09 — Discovery](skills/09_discovery.md) | UDP 服务发现 |
| [10 — Fusion](skills/10_fusion.md) | EKF 传感器融合 |
| [11 — Coroutine](skills/11_coroutine.md) | C++20 协程调度 |
| [12 — Demo Evaluator](skills/12_demo_evaluator.md) | 回归评估器设计 |
| [13 — E2E Learning Loop](skills/13_e2e_learning_loop.md) | 车端学习闭环 |
| [14 — Dead Reckoning](skills/14_dead_reckoning.md) | 前端航位推算 |

---

## Documentation

| Doc | Topic |
|-----|-------|
| [Evolution Roadmap](docs/EVOLUTION_ROADMAP.md) | Future phases |
| [Project Review](docs/PROJECT_REVIEW.md) | Capability assessment |
| [Quick Start](docs/QUICK_START.md) | 30-min tutorial |
| [Technical Design](docs/TECHNICAL_DESIGN.md) | Architecture design |
| [API Quick Reference](docs/API_QUICK_REFERENCE.md) | C API reference |
| [Simulation Guide](docs/SIMULATION_GUIDE.md) | Simulation testing guide |
| [Visualization Architecture](docs/VISUALIZATION_ARCHITECTURE.md) | FlowBoard + flowmond |
| [Monitoring Architecture](docs/MONITORING_ARCHITECTURE.md) | flowmond + stats bridge |
| [Pipeline Architecture](docs/PIPELINE_ARCHITECTURE.md) | Pipeline design |
| [Algorithm Stack](docs/ALGORITHM_STACK.md) | Algorithm overview |
| [Algorithm Integration](docs/ALGORITHM_INTEGRATION.md) | Algorithm integration guide |
| [E2E Simulation Design](docs/E2E_SIMULATION_DESIGN.md) | End-to-end simulation design |
| [FlowBoard Contract](docs/FLOWBOARD_CONTRACT.md) | Dashboard data contract |
| [Implementation Guide](docs/IMPLEMENTATION_GUIDE.md) | Implementation guide |
| [Flow Registry Plan](docs/FLOW_REGISTRY_PLAN.md) | Flow Registry design |
| [Learning Loop](docs/LEARNING_LOOP.md) | In-sim learning loop |

---

## License

MIT
