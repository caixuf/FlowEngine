# FlowEngine

> Simulation-driven middleware framework for autonomous driving & robotics — C kernel + C++20 coroutine shell.
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

A from-scratch middleware framework providing the core abstractions of CyberRT in a lightweight, embeddable package.
It is built to be **organizable, observable, testable, replayable and scoreable — all inside simulation**:

| Layer | Modules |
|-------|---------|
| **Communication** | Message Bus (pub/sub + req/reply + zero-copy), IPC (SHM), TCP Transport |
| **Execution** | Coroutine Scheduler (FIFO + CPU affinity + rate limit), Choreo DAG mode, Cancelable Coroutine Primitives (pub/sub · select · timer · req-reply, with timeout & graceful cancel) |
| **Introspection** | Reflective State Machine, UDP Service Discovery, Topology Tracking |
| **Metadata** | FlowRegistry (tasks/topics/types/plugins/schemas), ParamRegistry (int/float/bool/string with hot-reload) |
| **Data** | Type-safe Serialization (IDL + codegen), Bag v2 Record/Replay, Data Fusion, Schema Validation |
| **QoS** | Per-topic QoS (depth + drop policy), Topic Stats (frequency, latency p50/p99, subscribers) |
| **Operations** | Unified Logger (ms timestamps), flowctl CLI, FlowBoard Dashboard, flowmond Monitor Daemon, Stats Bridge (cross-process IPC stats), CI/CD |
| **Learning** | In-sim learning loop: data recorder → offline trainer → shadow-mode tiny-MLP inference (evaluated against the rule-based controller, never actuated). See [docs/LEARNING_LOOP.md](docs/LEARNING_LOOP.md) |

## Quick Start

```bash
git clone https://github.com/caixuf/FlowEngine.git && cd FlowEngine

# Build & run end-to-end demo
bash scripts/demo.sh

# Or explore with the CLI
./build/bin/flowctl version
./build/bin/flowctl list tasks
./build/bin/flowctl schema LidarFrame
./build/bin/flowctl bag info data.bag
```

> **Entry points:** `flow_launcher config/pipeline.json` is the canonical,
> config-driven way to run a pipeline (each node is a `dlopen`-loaded `.so`
> plugin). `flow_e2e` is a single-binary demo that inlines every node — handy
> for quick debugging without compiling plugins, but not the recommended
> production entry point.

## Demo

```bash
bash scripts/demo.sh 15
```

```
  ╔══════════════════════════════════════════════════════════╗
  ║   ███████╗██╗      ██████╗ ██╗    ██╗                  ║
  ║   ██╔════╝██║     ██╔═══██╗██║    ██║                  ║
  ║   █████╗  ██║     ██║   ██║██║ █╗ ██║                  ║
  ║   ██╔══╝  ██║     ██║   ██║██║███╗██║                  ║
  ║   ██║     ███████╗╚██████╔╝╚███╔███╔╝                  ║
  ║   ╚═╝     ╚══════╝ ╚═════╝  ╚══╝╚══╝                   ║
  ╚══════════════════════════════════════════════════════════╝

  Perception (10Hz) ──→ Fusion (time-align) ──→ Control (decisions)
  ⏱ 5s | pub=133 del=239 lat=141µs p99=428µs | Drive Mode: ACC

  Tests: 7/7 passed ✅
```

![Dashboard](docs/dashboard.png)
> *FlowBoard real-time dashboard — topology, frame monitor, latency charts. Open `http://localhost:8800` during demo.*

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                     FlowEngine Core (C)                    │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────────┐ │
│  │ Message  │ │   Task   │ │  State   │ │  Discovery  │ │
│  │   Bus    │ │ Manager  │ │ Machine  │ │   (UDP)     │ │
│  └──────────┘ └──────────┘ └──────────┘ └─────────────┘ │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────────┐ │
│  │   IPC    │ │   Bag    │ │  Clock   │ │  Serializer │ │
│  │  (SHM)   │ │   (v2)   │ │ Service  │ │  (IDL+FNV)  │ │
│  └──────────┘ └──────────┘ └──────────┘ └─────────────┘ │
├──────────────────────────────────────────────────────────┤
│                   FlowEngine Shell (C++)                  │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌─────────────┐ │
│  │ Coroutine│ │ Scheduler│ │  Fusion  │ │  Transport  │ │
│  │  Tasks   │ │ (Choreo) │ │   Node   │ │  (TCP/IPC)  │ │
│  └──────────┘ └──────────┘ └──────────┘ └─────────────┘ │
└──────────────────────────────────────────────────────────┘
```

## CLI

```bash
flowctl list tasks              # Registered tasks
flowctl list topics             # All topics with stats
flowctl graph                   # ASCII topology
flowctl state <task>            # State machine status
flowctl topic stats <topic>     # Per-topic latency/throughput
flowctl bag info <file>         # Bag metadata
flowctl schema <type>           # Type information
flowctl dashboard               # Launch FlowBoard
flowctl version                 # Build info
```

## Monitoring (flowmond)

`flowmond` is a standalone monitor daemon — it aggregates stats from all running processes via a POSIX shared-memory IPC bridge and serves a live dashboard.

```bash
# Terminal 1: start the monitor daemon
./build/bin/flowmond --port 8800

# Terminal 2: run the business process (publishes stats every 5 s)
./build/bin/flow_e2e 60

# Open dashboard in browser (~5 s after both processes are up)
open http://localhost:8800
```

Dashboard endpoints:

| Path | Description |
|------|-------------|
| `/` | Live FlowBoard UI |
| `/api/topics` | Per-topic stats (local + remote processes) |
| `/api/topology` | Topology JSON |
| `/api/stream` | SSE real-time push (500 ms interval) |

> **Bind address:** the dashboard listens on `127.0.0.1` (loopback) by default so it
> is not exposed on every interface. For container or remote access, start it with
> `flowmond --bind 0.0.0.0` (or set `FLOWMOND_BIND_ADDR=0.0.0.0`).

## Docker (easiest)

```bash
docker build -t flowengine .
docker run --rm flowengine          # Run e2e demo
docker run --rm flowengine demo 30  # 30-second demo
```

## Install

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build

# Verify
pkg-config --cflags --libs flowengine
flowctl version
```

Once installed, include the umbrella header to pull in the core public API and
version/ABI macros:

```c
#include "flowengine.h"   /* FLOWENGINE_VERSION, NODE_PLUGIN_API_VERSION, bus/transport/... */
```

## Build from Source

| Requirement | Version |
|-------------|---------|
| GCC | 11+ (C++20 coroutines) |
| CMake | 3.16+ |
| libcjson | any (`apt install libcjson-dev`) |
| libeigen3 | 3.3+ (`apt install libeigen3-dev`) — **required for the Frenet planner** (lane change / overtaking); without it `planning_node` silently falls back to lane-keep-only |
| Python | 3.8+ (codegen & dashboard) |

```bash
bash build.sh release           # One-click build
cmake --build build -j$(nproc)  # Incremental
ctest --test-dir build          # Run 7 tests
```

## Tests & CI

| Job | Status |
|-----|--------|
| Release | ✅ gcc -O2, 7 tests |
| Debug | ✅ gcc -g, 7 tests |
| ASAN | ✅ Address Sanitizer |
| TSAN | ✅ Thread Sanitizer |
| UBSAN | ✅ Undefined Behavior Sanitizer |
| Coverage | ✅ lcov report |
| Stress | ✅ 30s e2e at 10Hz |

## Regression Evaluator

```bash
# Run demo + auto-score: topology, collisions, road departure, stagnation, yaw wobble
python3 tools/demo_evaluator.py --duration 45

# Analyze last run without re-launching
python3 tools/demo_evaluator.py --no-run
```

The evaluator samples `/tmp/flow_topology.json` during a demo run and checks:
topology edges, topic frequencies, collision events, road departure, vehicle
stagnation, lane-change count, yaw/steer oscillation, NPC teleport jumps, and
message drops.  Run it after any change to the pipeline chain.

## Plugin Example

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

## Configuration-Driven Launch

```json
{
  "scheduler": { "mode": "choreo" },
  "processes": [{
    "name": "perception",
    "library_path": "lib/libfake_perception_task.so",
    "scheduling": {
      "priority": "critical",
      "cpu_affinity": [0],
      "max_frequency_hz": 10.0
    }
  }]
}
```

```bash
./build/bin/flow_launcher config/pipeline.json
```

## Documentation

| Doc | Topic |
|-----|-------|
| [Evolution Roadmap](docs/EVOLUTION_ROADMAP.md) | Future phases |
| [Project Review](docs/PROJECT_REVIEW.md) | Capability assessment |
| [Quick Start](docs/QUICK_START.md) | 30-min tutorial |
| [Technical Design](docs/TECHNICAL_DESIGN.md) | Architecture |
| [API Quick Reference](docs/API_QUICK_REFERENCE.md) | C API reference |
| [Simulation Guide](docs/SIMULATION_GUIDE.md) | Simulation testing |
| [Visualization Architecture](docs/VISUALIZATION_ARCHITECTURE.md) | FlowBoard + flowmond |
| [Monitoring Architecture](docs/MONITORING_ARCHITECTURE.md) | flowmond + stats bridge |
| [Skills](skills/) | Deep dives (serializer, statem, discovery, fusion, bus, IPC, bag, clock, coroutine) |

## License

MIT
