# FlowEngine

> 面向自动驾驶与机器人的仿真优先中间件框架 —— C11 内核、C++20 协程外壳、插件化架构。
>
> **定位：** FlowEngine 是一个*仿真优先、可复现的实验平台*。核心能力——感知、融合、规划、控制、学习——
> 首先在**仿真内**被运行、观察、测试、回放与评分；通过纯逻辑单测、schema 迁移与统一时间戳语义，
> 逐步向真车部署演进。当前阶段不追车规量产认证，已提供 RC 小车硬件落地清单，
> 详见 [docs/RC_CAR_HARDWARE_CHECKLIST.md](docs/RC_CAR_HARDWARE_CHECKLIST.md)。

[![CI](https://github.com/caixuf/FlowEngine/actions/workflows/ci.yml/badge.svg)](https://github.com/caixuf/FlowEngine/actions)
![License](https://img.shields.io/badge/license-MIT-blue)
![C](https://img.shields.io/badge/C-11-555555)
![C++](https://img.shields.io/badge/C++-20-659ad2)

---

## FlowEngine 是什么

一个从零搭建的中间件框架，灵感来自 Apollo CyberRT，以轻量、可嵌入的包提供核心抽象。
目标是**可组织、可观察、可测试、可回放、可评估——全部在仿真内**：

| 层 | 模块 |
|-------|---------|
| **通信** | Message Bus（发布/订阅 + 请求/应答 + 零拷贝）、IPC（SHM）、TCP Transport、Network Transport |
| **执行** | Coroutine Scheduler（FIFO + CPU 亲和 + 限频）、Choreo DAG 模式、可取消协程原语（发布/订阅 · select · timer · req-reply，含超时与优雅取消） |
| **内省** | 反射式状态机、UDP 服务发现、拓扑追踪、SysMonitor |
| **元信息** | FlowRegistry（tasks/topics/types/plugins/schemas）、ParamRegistry（int/float/bool/string，支持热重载） |
| **数据** | 类型安全序列化（IDL + 代码生成）、Bag v2 录制/回放、MCAP 格式、数据融合（EKF）、Schema 校验；全链路 `timestamp_us` 统一为 `uint64`（GNSS 采集时刻优先、主机钟回退） |
| **QoS** | Per-topic QoS（深度 + 丢弃策略 + deadline + reliability）、Topic 统计（频率、延迟 p50/p99、订阅者） |
| **感知** | DBSCAN LiDAR 聚类、Kalman 跟踪、EKF 传感器融合、NMEA 0183 GPS 解析器（RMC/GGA，GNSS UTC → epoch μs 采集时刻）、IMU ASCII 行解析、nuScenes 数据集加载器 |
| **规控** | 行为 FSM（跟车/变道/让行/掉头）、Frenet 最优轨迹、ST 图 + DP 速度规划（红灯墙/动态障碍/曲率限速）、N 把方向掉头与泊车（ManeuverTracker）、Stanley 横向（可选 LTV MPC）+ PID/ACC 纵向 |
| **安全** | 基于 FlowCoro 协程的安全包络（TTC / 横向交叉 / 行人保护） |
| **运维** | 统一日志器（毫秒时间戳）、flowctl CLI、FlowBoard Dashboard（Three.js 3D + 2D）、flowmond 监控守护进程（IPC 桥接 + 文件桥接）、跨进程 IPC Stats Bridge + Topic Bridge、CI/CD |
| **学习** | 仿真内学习闭环：数据采集 → 离线训练（tiny-MLP / PyTorch）→ DAgger 自我对弈回灌 → PPO 强化学习（换老师路线）→ ONNX 导出 + 等价性门禁 → 影子评估 + promote 门禁 + 模型 OTA 与 A-B 对比。详见 [docs/LEARNING_LOOP.md](docs/LEARNING_LOOP.md) |

---

## 架构

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
│  flowsim → sensor_model → perception → object_tracker → fusion       │
│    → behavior_planner ⇄ navigation → planning → control              │
│    → safety_control → monitor                                        │
│  旁路: inference / data_recorder / learner / model_ota（学习闭环）     │
└──────────────────────────────────────────────────────────────────────┘
                                                                         │
                      ════════════════════┼════════════════════
                                         │
                          ┌──────────────▼─────────────────────┐
                          │  flowmond 监控守护进程 :8800          │
                          │  IPC 桥接（首选）：stats_bridge /      │
                          │    dashboard_bridge → IPC SHM          │
                          │  文件桥接（回退）：轮询                 │
                          │    /tmp/flow_topology.json             │
                          │  → 托管 flowboard/index.html 前端       │
                          └────────────────────────────────────────┘
```

**可视化链路：** 由统一的 C 监控守护进程 `flowmond` 提供 HTTP 仪表盘，
同时启用 IPC 桥接（首选）与文件桥接（回退）两条等价数据链路，按可用性自动回退。
详见 [docs/VISUALIZATION_ARCHITECTURE.md](docs/VISUALIZATION_ARCHITECTURE.md)。

---

## 快速开始

```bash
git clone https://github.com/caixuf/FlowEngine.git && cd FlowEngine

# 一键演示（构建 + 运行，默认 15s）
bash scripts/demo.sh

# 或手动构建
bash build.sh release
```

> **入口：** `flow_launcher config/pipeline.json` 是运行 pipeline 的标准、
> 配置驱动方式（每个节点都是 `dlopen` 加载的 `.so` 插件）。

### Windows（实验性原生支持）

Windows 原生路径优先支持 **单进程插件模式**（等价默认 demo）。`--multi`
依赖 POSIX `fork` + SHM IPC，Windows 下会明确拒绝；参数 AF_UNIX bridge、
训练/ops 的 fork 子进程 bridge 也会降级为不可用。flowcoro 节点使用
header-only `rt_executor` 路径；epoll/eventfd 版 `flowcoro_net` 不进入
Windows 默认构建，主仿真与 FlowBoard dashboard 不受影响。

```powershell
# 需要 CMake + Visual Studio Build Tools（可配 Ninja 使用）
powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 -Duration 15 -NoBrowser

# 或手动
cmake --preset windows-ninja
cmake --build --preset windows-ninja
cmake -S modules\adas_nodes -B build-win\modules\adas_nodes -DFLOWENGINE_BUILD="$PWD\build-win"
cmake --build build-win\modules\adas_nodes --config Release
build-win\bin\flow_launcher.exe config\pipeline.json --duration 15
```

**mingw-w64 交叉编译（Linux → Windows，CI 门禁用）**

除 MSVC 原生构建外，核心工程也可在 Linux 上用 mingw-w64 交叉编译出原生
Windows PE 二进制。这是 Windows 目标的**可复现回归门禁**（CI 无需 Windows
runner 即可拦截 `_WIN32` 路径 / 兼容层编译回归，见 `build-windows-mingw` job）：

```bash
sudo apt-get install -y gcc-mingw-w64-x86-64 g++-mingw-w64-x86-64 ninja-build

cmake -S . -B build-mingw -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-x86_64.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-mingw -j
file build-mingw/bin/flow_launcher.exe   # → PE32+ executable ... for MS Windows
```

> 交叉编译覆盖核心工程（`flow_launcher` / `flowmond` / `flowctl` / 总线 / IPC /
> bag 等）；`modules/adas_nodes` 的 `flowsim_node` 依赖 esmini RoadManager，需先
> 构建 esmini（或在 Windows 上用预构建库）。兼容层实现见
> `include/platform_compat.h`（`_WIN32` 段）与 `include/compat_win/`（POSIX 头平替）。

### 新手最短路径（复制即可跑）

```bash
# 1) 跑一次 demo（验证环境）
bash scripts/demo.sh --no-browser 15

# 2) 一键学习闭环：采集 -> 训练 -> 影子评估
python3 tools/learning_loop.py --collect 30 --eval-duration 30

# 3) 只重跑影子评估并自动晋级（promote gate）
python3 tools/learning_loop.py --eval-only <模型目录名> --eval-duration 30 --promote

# 4) 产部署包（用于非开发机）
bash scripts/deploy.sh --package

# 5) 解包后一键启动（包内脚本）
bash share/flowengine/scripts/quickstart.sh 15
```

---

## 演示

```bash
bash scripts/demo.sh 30     # 30 秒演示

# 其他模式
bash scripts/demo.sh --multi      # 多进程模式（各节点独立 fork+exec）
bash scripts/demo.sh --record     # 录制 Bag 文件
bash scripts/demo.sh --no-browser # 不打开浏览器
bash scripts/demo.sh --start-s 2700 --start-d -1.75  # 从 route 指定位置起跑

# Windows 原生单进程模式
powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 -Duration 30
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
  ║   面向自动驾驶的轻量级中间件                              ║
  ║                                                          ║
  ╚══════════════════════════════════════════════════════════╝

  ┌─ FlowSim ─→  Perception ─→  Fusion  ─→  Planning ─→  Control ┐
  │  dynamics      DBSCAN          EKF          Frenet       PID     │
  └──────────────────────────────────────────────────────────────────┘

  ⏱ 15s  |  pub=133 del=239 lat=141µs speed=11.2m/s
```

![Dashboard](docs/dashboard.png)
> *FlowBoard 实时仪表盘 —— 拓扑图、3D 场景、帧监控、延迟图表。演示期间打开 `http://localhost:8800`。*

**实时服务：**
| 服务 | 端口 | 说明 |
|------|------|------|
| FlowBoard Dashboard | `:8800` | 实时仪表盘（3D + 2D + D3 拓扑） |
| Foxglove 3D Bridge | `:8765` | Foxglove Studio WebSocket 桥接 |

---

## Pipeline

默认配置（`config/pipeline.json`）启动 **15 个插件节点**，默认场景为直路导航（`scenarios/straight_road.json`，3000m 双向 4 车道，含 NPC/行人/红绿灯/路尾掉头）：

| 节点 | 插件 (.so) | 频率 | 功能 |
|------|-----------|------|------|
| `flowsim` | `libflowsim_node.so` | 50Hz | 车辆动力学 + NPC（IDM）+ 场景加载 + 真值发布 |
| `sensor_model` | `libsensor_model.so` | 20Hz | LiDAR/GPS/Camera 传感器模型（FOV/遮挡/噪声） |
| `perception` | `libperception_node.so` | 10Hz | DBSCAN 点云聚类 + 目标检测 |
| `object_tracker` | `libobject_tracker.so` | 20Hz | 卡尔曼多目标跟踪 |
| `fusion` | `libfusion_node.so` | 20Hz | EKF 传感器融合（定位 + 时间对齐） |
| `behavior_planner` | `libbehavior_planner.so` | 10Hz | 8 状态 FSM 行为决策（跟车/变道/停车/让行/掉头） |
| `navigation` | `libnavigation_node.so` | 10Hz | 路由步骤 + 行进方向（消费 `ref_path.reverse`） |
| `planning` | `libplanning_node.so` | 20Hz | Frenet 轨迹 + ST 图 DP 速度规划 + N 把方向掉头 |
| `control` | `libcontrol_node.so` | 50Hz | Stanley 横向（可选 LTV MPC）+ PID/ACC 纵向 + ManeuverTracker 机动 |
| `safety_control` | `libsafety_control_node.so` | 协程 | FlowCoro 安全闸门（TTC / 横向交叉 / 行人保护 / MRM） |
| `inference` | `libinference_node.so` | 20Hz | tiny-MLP/ONNX 影子推理（shadow mode，不执行） |
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

## 可视化

可视化由统一的 C 监控守护进程 `flowmond` 提供，同时启用 IPC 桥接（首选）与
文件桥接（回退）两条数据链路。前端 `tools/flowboard/index.html` 由 flowmond 通过
`--html-path` 加载并托管。

3D 场景基于 vis/ 模块树（Layer + ViewRegistry 插件化，受 Qt 对象树启发）：
- 插件化 View：ground/road/viaduct/vehicle/npc/streetlight/barrier/obstacle 等独立 View，
  单个 View 抛错不影响整体渲染；
- Layer 树：env/road/agent/infra 4 个语义层递归 update；
- 车辆渲染：优先加载 glTF（SU7 海湾蓝金属漆），回退程序化几何体；
- 渲染管线：PMREM 环境贴图烘焙 + ShaderMaterial 渐变天空 + Fog 地平线融合 + PCFSoft 阴影 + ACESFilmic tone mapping；
- 高架 wrap：wrap 周期与高架实际 build 长度强制一致，消除 500m/1000m 接缝。

详见 [docs/VIS_MODULE_GUIDE.md](docs/VIS_MODULE_GUIDE.md) 与
[docs/TROUBLESHOOTING_3D_DASHBOARD.md](docs/TROUBLESHOOTING_3D_DASHBOARD.md)。

FlowBoard 界面按用途分三个工作区（顶部导航切换，记忆上次选择）：

| 工作区 | 内容 | 典型用途 |
|--------|------|----------|
| **Observe 观察** | 3D 场景 + 节点拓扑图 | 看车跑得对不对 |
| **Analyze 分析** | QoS 统计 / 消息矩阵 / 实时图表 | 查频率掉没掉、延迟高不高 |
| **Operate 操作** | 训练中心 + Ops 控制台 | 点按钮跑学习闭环、回灌 bag |

Ops 控制台（侧栏「Ops Console」或 Operate 工作区进入）支持**免命令行操作**：
- **Bag 回灌**：选 bag 文件 → 开始回灌 → 实时看日志 → 停止；
- **学习闭环**：一键「仅影子评估」或「采集→训练→评估」完整闭环；
- **训练中心**：查看模型列表 / shadow_eval 得分 / 一键晋级 runtime 模型。

```bash
# 终端 1：监控守护进程（加载前端，启用 IPC 桥接 + 文件桥接回退）
./build/bin/flowmond --html-path tools/flowboard/index.html

# 终端 2：运行 pipeline（写 /tmp/flow_topology.json + 发布 IPC 统计）
./build/bin/flow_launcher config/pipeline.json --duration 3600

# 打开浏览器
open http://localhost:8800
```

仪表盘端点：

| 路径 | 说明 |
|------|-------------|
| `/` | 实时 FlowBoard UI（3D 场景 + 拓扑 + 图表）|
| `/api/topics` | Per-topic 统计（频率/延迟/订阅者） |
| `/api/topology` | 拓扑 JSON（节点 + 边 + 指标）|
| `/api/stream` | SSE 实时推送（500 ms 间隔）|
| `/api/health` | 健康检查 |
| `/api/training/status` | 学习任务状态与模型列表（modelctl bridge） |
| `/api/training/start` | 启动训练任务（POST） |
| `/api/training/promote` | 晋级 tiny 模型到 C runtime（POST） |
| `/api/ops/status` | Ops Console 任务状态（bag 回灌 / learning_loop） |
| `/api/ops/run` | Ops Console 操作入口（POST） |

> **绑定地址：** `flowmond` 默认监听 `127.0.0.1`（回环）。如需远程访问，
> 使用 `flowmond --bind 0.0.0.0`（或设置 `FLOWMOND_BIND_ADDR=0.0.0.0`）。

---

## Docker

```bash
docker build -t flowengine .
docker run --rm flowengine          # 运行 e2e 演示
docker run --rm flowengine demo 30  # 30 秒演示
```

---

## 从源码构建

**平台支持：** Linux（Ubuntu，主力/CI）与 **macOS 原生**（Apple Silicon / Intel）。
两者共用同一套源码，平台差异由兼容层 `include/platform_compat.h` 与 CMake 的
`if(APPLE)` 分支收口，Linux 行为零变化。

| 依赖 | 版本 |
|-------------|---------|
| 编译器 | Linux: GCC 11+（`-fcoroutines`）／macOS: Apple clang（`-std=c++20` 原生协程，无需 `-fcoroutines`）|
| CMake | 3.16+ |
| libcjson | 任意版本（Linux `apt install libcjson-dev`；缺失时自动 FetchContent 源码构建，**macOS 无需手动安装**）|
| libeigen3 | 3.3+（Linux `apt install libeigen3-dev`；缺失时自动 git 拉取）—— **Frenet 规划器必需**（变道/超车）；缺失时 `planning_node` 会静默回退到仅车道保持 |
| Python | 3.8+（代码生成与仪表盘）|
| libprotobuf-c（可选）| 用于 protobuf 支持 |

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)   # macOS 用 -j$(sysctl -n hw.ncpu)

# 或一键构建（脚本自动探测核数与编译器，Linux/macOS 通用）
bash build.sh release
```

### macOS 快速开始

无需 Docker，原生跑通：

```bash
# 依赖仅需 cmake（cjson/eigen 会自动 FetchContent，无需 Homebrew）
brew install cmake              # 可选：若系统已装可跳过

bash scripts/demo.sh           # 一键构建 + 运行 + 打开浏览器
# 浏览器手动打开 http://localhost:8800（脚本也会用 `open` 自动打开）
```

> **macOS 已知限制**（均只影响非默认路径，默认单进程 dlopen demo 功能完全对齐）：
> - `--multi` 多进程模式的 robust-mutex 崩溃自愈降级为普通 mutex（macOS 无
>   `PTHREAD_MUTEX_ROBUST`；单机 demo 不触发该路径，共享状态另有 seq 号自愈）；
> - `benchmark` 二进制不构建（macOS 无名信号量 `sem_init` 已废弃）；
> - CAN/I2C 执行器走 dry-run（`<linux/can.h>`/`<linux/i2c-dev.h>` 为 Linux 专属）；
> - 无线程 CPU 亲和性绑定（macOS 无用户态线程 affinity API，`pin_cpu` 被静默忽略）。

### 安装

```bash
sudo cmake --install build

# 验证
pkg-config --cflags --libs flowengine
flowctl version
```

安装完成后，按需引入各公共头文件：

```c
#include "task_interface.h"  /* 任务接口（TaskBase / TaskInterface） */
#include "message_bus.h"     /* 消息总线 */
#include "transport.h"       /* 传输层（local/IPC/TCP） */
```

---

## 插件系统

FlowEngine 采用基于 `dlopen` 的插件架构。每个 pipeline 节点是一个共享库（`.so`），
由 `flow_launcher` 在运行时加载。节点之间仅通过 Message Bus 通信——
节点之间没有直接函数调用。

```c
// C 插件 —— dlopen 兼容 ABI
#include "task_interface.h"

typedef struct { TaskBase base; int param; } MyTask;

static int my_execute(TaskBase* base) {
    while (!base->should_stop) {
        /* 业务逻辑 */
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

## 配置驱动启动

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

## 场景套件

`scenarios/` 提供 13 个场景 + 1 个场景矩阵（`suite.json`，故障表中每类高频 bug 至少一个场景兜底）：

| 场景 | 覆盖 |
|------|------|
| `straight_road.json`（默认） | 直路导航：3000m 双向 4 车道，NPC/行人/红绿灯，路尾物理掉头到对向车道 |
| `curve_road.json` | S 弯（Hermite 平滑）：曲率限速 + MPC kappa 前馈 |
| `dense_npc.json` | 密集 NPC：跟车 / 变道决策 / 变道纠结回归 |
| `lane_change_traffic.json` | 密集车流 + NPC MOBIL 自主变道下的规控鲁棒性 |
| `multi_light.json` | 连续 3 盏错相红绿灯：红灯闯行 / 停稳不走闭锁 / 绿灯卡死 |
| `oncoming.json` | 对向会车：迎头误判 / 幽灵刹车回归 |
| `urban_challenge.json` | 城市综合：急刹 + 行人横穿 + 红绿灯 + cut-in |
| `auto_parking.json` / `right_turn_side_parking.json` | 自动泊车 / 右转支路 + 路侧车位 |
| `traffic_rules_exam.json` ~ `safe_driving_exam.json` | 驾考科目一~四：交规 / 侧方停车 / 道路驾驶 / 安全文明 |

场景矩阵回归见 `ci/evaluators/scenario_regression.py`（与基线对比），
单次运行在线评分见 `ci/evaluators/demo_evaluator.py`。

---

## 学习闭环

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
- **Stage 1:** 离线训练 — `tools/train_demo_model.py`（顶层入口，调度 `tools/train_e2e/{train,torch_train,temporal_train}.py`）；进阶：DAgger 自我对弈回灌（模型犯错帧回灌训练集）、PPO 强化学习换老师路线（`tools/train_e2e/rl_ppo.py`，4-seed 验证）
- **Stage 2:** `inference_node` — tiny-MLP/ONNX 影子推理（ONNX 导出带数值等价性门禁），与规则控制器并行评估
- **Stage 3:** `learner_node` — 车端增量 SGD 微调（全量/部分更新）
- **Stage 4:** `model_ota_node` — 模型版本管理 + A-B 效果对比 + 动态切换

详见 [docs/LEARNING_LOOP.md](docs/LEARNING_LOOP.md)。

**推荐命令（Phase 2 已打通）：**

```bash
# 一键：采集 -> 训练 -> 影子评估（结果写 runs/ 和 models/<name>/shadow_eval.json）
python3 tools/learning_loop.py --collect 30 --eval-duration 30

# 用已有模型只做影子评估（刷新 shadow_eval.json）
python3 tools/learning_loop.py --eval-only <name_or_dir> --eval-duration 30

# 评估达标后自动晋级到 runtime 模型
python3 tools/learning_loop.py --eval-only <name_or_dir> --eval-duration 30 --promote

# 查看当前 runtime 与 artifact
python3 tools/modelctl.py list
```

门禁规则（`modelctl promote`）：
1. 必须有 `shadow_eval.json`
2. `evaluator_result` 必须 `PASS`
3. `shadow_speed_mae <= 2.0`
4. `shadow_n >= 50`
5. 评估年龄 <= 7 天

---

## 回归评估器

```bash
# 运行演示 + 自动评分：拓扑、碰撞、冲出路面、停滞、yaw 抖动
python3 ci/evaluators/demo_evaluator.py --duration 45

# 分析上次运行（不重新启动）
python3 ci/evaluators/demo_evaluator.py --no-run

# 全场景套件与基线对比
python3 ci/evaluators/scenario_regression.py --baseline

# 秒级离线管道完整性（先跑这个再跑长测）
python3 tools/pipeline_check.py
```

评估器在演示运行期间采样 `/tmp/flow_topology.json` 并检查：
拓扑边、topic 频率、碰撞事件、冲出路面、车辆停滞、变道次数、yaw/steer 振荡、
NPC 瞬移跳变以及消息丢帧。对 pipeline 链路做任何改动后都应运行它。

---

## 测试与 CI

| 任务 | 状态 | 说明 |
|-----|--------|-------------|
| Release | ✅ | gcc -O2，单元测试 |
| Debug | ✅ | gcc -g，单元测试 |
| ASAN | ✅ | Address Sanitizer |
| UBSAN | ✅ | Undefined Behavior Sanitizer |
| Stress | ✅ | 15s 全速率 pipeline |
| Integration | ✅ | 多节点 pipeline + ctest |
| Coverage | ✅ | lcov 报告 |
| Viz | ✅ | flowmond 仪表盘冒烟测试 + FlowBoard 前端测试 |
| Evaluator | ✅ | 45s 回归评估器（PR 门禁）|
| Scenario Regression | 🌙 | 全场景套件与基线对比（nightly/手动）|
| Nightly Stability | 🌙 | 长时间运行（仅调度）|

> **TSAN 当前禁用** — 协程 + 无锁内存池的跨线程同步模式对 TSAN 产生大量假阳性，
> 待协程生命周期稳定后重新启用。

---

## Skills（深度教程）

| Skill | 主题 |
|-------|-------|
| [01 — OOP in C](docs/tutorials/01_oop_in_c.md) | C 语言面向对象编程 |
| [02 — Plugin System](docs/tutorials/02_plugin_system.md) | dlopen 插件架构设计 |
| [03 — Message Bus](docs/tutorials/03_message_bus.md) | 零拷贝 Pub/Sub 总线 |
| [04 — IPC Channel](docs/tutorials/04_ipc_channel.md) | POSIX SHM 进程间通信 |
| [05 — Bag Recording](docs/tutorials/05_bag_recording.md) | Bag v2 录制与回放 |
| [06 — Clock Service](docs/tutorials/06_clock_service.md) | 时钟服务与时间管理 |
| [07 — Serializer](docs/tutorials/07_serializer.md) | IDL 代码生成与序列化 |
| [08 — State Machine](docs/tutorials/08_state_machine.md) | 反射式状态机 |
| [09 — Discovery](docs/tutorials/09_discovery.md) | UDP 服务发现 |
| [10 — Fusion](docs/tutorials/10_fusion.md) | EKF 传感器融合 |
| [11 — Coroutine](docs/tutorials/11_coroutine.md) | C++20 协程调度 |
| [12 — Demo Evaluator](docs/tutorials/12_demo_evaluator.md) | 回归评估器设计 |
| [13 — E2E Learning Loop](docs/tutorials/13_e2e_learning_loop.md) | 车端学习闭环 |
| [14 — Dead Reckoning](docs/tutorials/14_dead_reckoning.md) | 前端航位推算 |
| [15 — SocketCAN Actuator](docs/tutorials/15_socketcan_actuator.md) | SocketCAN 执行器 |
| [16 — FlowSim Scenario Design](docs/tutorials/16_flowsim_scenario_design.md) | 仿真场景设计 |
| [17 — Vis Module Designer](docs/tutorials/17_vis_module_designer.md) | vis 模块设计（Layer + ViewRegistry 插件化）|

---

## 文档

> 📖 **完整文档索引见 [docs/README.md](docs/README.md)** —— 按主题分组的全部文档 + 17 篇教程入口。

| 文档 | 主题 |
|-----|-------|
| [Quick Start](docs/QUICK_START.md) | 30 分钟教程 |
| [Technical Design](docs/TECHNICAL_DESIGN.md) | 架构设计 |
| [API Quick Reference](docs/API_QUICK_REFERENCE.md) | C API 参考 |
| [Simulation Guide](docs/SIMULATION_GUIDE.md) | 仿真测试指南 |
| [Visualization Architecture](docs/VISUALIZATION_ARCHITECTURE.md) | flowmond + vis/ 模块树（Layer + ViewRegistry + Qt 对象树）|
| [Vis Module Guide](docs/VIS_MODULE_GUIDE.md) | vis/ 模块接口契约 + 设计 AI 提示词模板 |
| [Monitoring Architecture](docs/MONITORING_ARCHITECTURE.md) | flowmond + stats bridge |
| [Pipeline Architecture](docs/PIPELINE_ARCHITECTURE.md) | Pipeline 设计 |
| [Algorithm Stack](docs/ALGORITHM_STACK.md) | 算法总览（各模块真实算法 × 文件对照） |
| [Algorithm Integration](docs/ALGORITHM_INTEGRATION.md) | 算法集成指南 |
| [Planning Speed Upgrade](docs/PLANNING_SPEED_UPGRADE_DESIGN.md) | ST 图 + DP 速度规划设计 |
| [FlowBoard Contract](docs/FLOWBOARD_CONTRACT.md) | 仪表盘数据契约 |
| [FlowBoard Scene Contract](docs/FLOWBOARD_SCENE_CONTRACT.md) | scene 数据契约 |
| [FlowSim 仿真指南](docs/SIMULATION_GUIDE.md) | flowsim 仿真模式对照 |
| [Hardware Deployment](docs/HARDWARE_DEPLOYMENT.md) | 硬件部署 |
| [RC Car Hardware Checklist](docs/RC_CAR_HARDWARE_CHECKLIST.md) | RC 小车硬件落地清单 |
| [Learning Loop](docs/LEARNING_LOOP.md) | 仿真内学习闭环 |
| [Troubleshooting 3D Dashboard](docs/TROUBLESHOOTING_3D_DASHBOARD.md) | 3D 仪表盘故障排查 |

---

## 许可证

MIT
