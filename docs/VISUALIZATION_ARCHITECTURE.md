# FlowEngine 可视化架构

## 架构原则

```
业务节点 → flowmond（HTTP 仪表盘 + IPC 桥接 + 文件桥接）→ 浏览器仪表盘 (零外部依赖)
```

可视化由统一的 C 语言监控守护进程 **flowmond**（`src/flowmond.c`，编译产物
`build/bin/flowmond`）提供。flowmond 内置 HTTP 服务器并提供两条等价的数据采集链路，
按可用性自动回退：

| 数据来源 | 采集方式 | 适用场景 |
|----------|----------|----------|
| **IPC 桥接**（首选） | 通过 `stats_bridge` / `dashboard_bridge` IPC 通道订阅业务进程统计与仪表盘 JSON | 多进程部署，低延迟 |
| **文件桥接**（回退） | `dashboard_file_watcher_fn` 线程轮询 `/tmp/flow_topology.json`（由 monitor 节点周期性写入） | launch 仿真 / 演示 / 单机 / IPC 不可用时 |

> flowmond 同时启动两条链路：IPC 桥接为主，文件桥接为回退。launch 单进程演示时，
> 业务进程的 monitor 任务会写出 `/tmp/flow_topology.json`，flowmond 通过文件桥接读取，
> 保证演示场景始终有数据。

前端 `tools/flowboard/index.html`（HTML/JS 仪表盘）由 flowmond 通过 `--html-path` 加载并托管。
此外 `modules/adas_nodes/flowmond_node.cpp` 是 flowmond 的 `NodePlugin` 包装版，
可作为节点插件在 pipeline 内运行，功能等价。

## 组件

```
┌────────────────────┐   写状态文件    ┌──────────────────────────┐
│   flow_launcher config/pipeline.json │ ───────────────▶│  /tmp/flow_topology.json  │
│   (业务/仿真节点)   │  monitor 任务   │  (原子 tmp+rename, ~1Hz)  │
│                     │   10Hz 采集     └──────────────────────────┘
│  Message Bus        │           │ 文件桥接(轮询)        ▲ IPC 桥接
│  Transport / Sched  │           │                       │ (stats_bridge /
│  3D 仿真 (ego/障碍/  │           │                       │  dashboard_bridge)
│  LiDAR)             │           │                       │
└────────────────────┘           ▼                       │
                  ┌──────────────────────────┐           │
                  │  flowmond :8800          │◄──────────┘
                  │  (C 监控守护进程)         │
                  │  /api/topology /api/topics│
                  │  /api/stream /api/health │
                  │  + --html-path 前端托管   │
                  └──────────────────────────┘
                                                  │ HTTP SSE
                                                  ▼
                                     ┌──────────────────────────┐
                                     │   FlowBoard Dashboard     │
                                     │   (tools/flowboard/       │
                                     │    index.html, Three.js)  │
                                     │   LIVE / STALE / OFFLINE  │
                                     └──────────────────────────┘
```

## 使用方式

```bash
# 1. 启动监控守护进程（加载前端，同时启用 IPC 桥接 + 文件桥接回退）
./build/bin/flowmond --html-path tools/flowboard/index.html &

# 2. 启动业务/仿真节点（写状态文件 + 发布 IPC 统计）
./build/bin/flow_launcher config/pipeline.json --duration 3600 &

# 3. 浏览器打开仪表盘
open http://localhost:8800

# 或直接使用一键脚本（已改为调用 flowmond）
./scripts/demo.sh
```

## 鲁棒性设计（解决"离线"反复出现）

flowmond 与前端针对历史上反复出现的"离线"问题做了如下加固：

1. **多连接 HTTP 服务器**：flowmond 的 `monitor_server`（`src/core/monitor_server.c`）
   支持多线程并发连接，SSE 长连接不会饿死 `/api/topology` 等快照/重连请求，
   避免前端因拉取超时翻转为离线并静默显示假数据。
2. **数据新鲜度**：flowmond 记录状态最后更新时间，超过阈值标记为 `stale`，
   并在 `/api/topology`、SSE 快照中返回 `stale` / `age_sec`。
3. **`/api/health` 端点**：返回 `status` / `source` / `age_sec`，便于探活。
4. **SSE 心跳**：周期性发送 `: keep-alive` 注释帧，避免代理断开；连接使用
   墙钟截止时间而非计数循环。
5. **前端三态**：`applyLiveStatus()` 依据 `source`/`stale`/`age_sec` 显示
   **LIVE / STALE / OFFLINE** 三态；数据陈旧时保留最后一帧真实数据，
   **不再静默切换到模拟数据**。
6. **IPC 接收线程与文件桥接解耦**：`recv_thread_fn` 不在持有 `peers_mutex`
   期间 `usleep` 或 `message_bus_publish`，避免饿死轮询状态文件的
   `dashboard_file_watcher_fn` 线程（曾导致状态文件停止更新 → 前端离线）。

## API 端点

```
GET /              → 实时仪表盘 HTML (tools/flowboard/index.html, SSE 自动更新)
GET /api/topology  → Bus 统计 + 拓扑 + 车辆/场景 JSON (含 source/stale/age_sec)
GET /api/topics    → Per-topic 统计 (频率/延迟/订阅者)
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
        │ 文件桥接(轮询)            ▲ IPC 桥接
        ▼                           │ (stats_bridge / dashboard_bridge)
flowmond (C 监控守护进程, 多连接)
  ├─ dashboard_file_watcher_fn 轮询 /tmp/flow_topology.json
  ├─ 新鲜度判定 → source: live/stale/demo
  └─ HTTP / SSE 推送
        │
        ▼
FlowBoard (tools/flowboard/index.html, 浏览器)
  ├─ fetch /api/topology  → 初始加载
  ├─ EventSource /api/stream → 实时更新 (~10Hz)
  ├─ applyLiveStatus → LIVE/STALE/OFFLINE 三态
  ├─ deadreckon.js   → 统一航位推算引擎 (60fps 平滑)
  │   ├─ updateDeadReckon(x,z,speed,heading) ← SSE 数据到达时调用
  │   ├─ tickDeadReckon()                     ← 每帧 rAF 调用
  │   └─ getDeadReckonState()                 → scene3d.js / scene2d.js 读取
  ├─ scene3d.js (Three.js) → 3D 渲染
  └─ scene2d.js (Canvas 2D) → 2D 俯视图渲染
```

## `scene` 数据结构（真实 3D 仿真）

`monitor` 任务在 `metrics.scene` 下导出真实仿真场景，前端据此渲染真实
三维场景（而非随机点）。坐标系：**自车系, x 前为正, y 左为正 (米)**，
`heading` 为弧度。向后兼容——若 `scene` 缺失，前端回退到旧的占位渲染。

```json
{
  "scene": {
    "ego":       { "x": 45.1, "y": 0.0, "heading": 0.0, "speed": 10.1, "steer": 0.03 },
    "lane":      { "width": 3.5, "count": 2, "center": 0.0 },
    "road":      { "curve_start_x": 150.0, "curve_length_m": 260.0, "curve_offset_m": 9.0 },
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
- `road` 弯道几何由 `sim_world_node` 通过 `road/geometry` topic 统一发布，
  `control` / `planning` / `monitor` 三节点均**订阅**该 topic，不再各自独立加载场景文件。
  前端据此将道路分段 group 做 Z 向偏移渲染弯道。
- 自车运动含横向自行车模型（转向 P 控制向目标车道），并具备 ACC
  （依据同车道前车间距动态限制目标速度）与变道演示。

## 共享道路几何（Phase 2 — 已落地）

此前 `control` / `planning` / `monitor` 三个节点各自调用 `scenario_load()` 读取弯道参数，导致：

1. **数据不一致** — 修改场景文件后，三节点可能读到不同版本（缓存、路径差异）。
2. **字段名冲突** — vehicle/state JSON 中混入 `road_curve_sx/len/off`，污染车辆遥测。
3. **启动顺序依赖** — 后启动的节点可能错过初始弯道参数。

**修复方案：**

- 新增 `road/geometry` topic（Type ID `0x80AD5C12u`），JSON 格式：
  ```json
  {"curve_start_x":150.0,"curve_length_m":260.0,"curve_offset_m":9.0,
   "lane_width":3.5,"lane_count":2}
  ```
- `sim_world_node` 为**唯一发布者** — init 时立即发布一次，之后每 50 cycle（≈1Hz）重发。
- `control` / `planning` / `monitor` 移除 `scenario_load()` 中的弯道读取，改为订阅 `road/geometry`。
- `vehicle/state` 中移除弯道字段，字段名统一为 `curve_start_x` / `curve_length_m` / `curve_offset_m`。

**关键教训：** dlopen 单进程模式下所有节点共享同一个 `MessageBus`，订阅者总数受 `MSG_BUS_MAX_SUBSCRIBERS`（默认 32）限制。本轮重构中 monitor 的 `road/geometry` 订阅因总订阅数（33）溢出而静默失败，已上调至 128。

## 统一航位推算（Phase 3 — 已落地）

前端以 ~10Hz 接收 SSE 数据，但渲染以 60fps 运行。如果每帧直接渲染最新数据，车辆会在数据帧之间**冻结**，视觉体验极差。此前的航位推算状态分散在三个模块中，存在严重架构缺陷：

| 问题 | 位置 | 后果 |
|------|------|------|
| lerp 逻辑复制 | `scene3d.js _renderFrame()` + `scene2d.js _2dAnimLoop()` | 3D/2D 平滑参数不一致（0.15 vs 0.18） |
| heading 无角度环绕 | `scene3d.js` line 403 | 跨 ±π 时车辆整圈旋转 |
| 帧率相关 | 固定 lerp 因子 0.15 | 30fps 和 144fps 表现完全不同 |
| monkey-patch | `scene2d.js init2D()` | 污染 `updateAll()` 全局函数 |
| 职责分散 | 各模块直接修改 `_dr.last*` | 无单一 truth source，数据竞争风险 |

**修复方案 — 单一引擎 `deadreckon.js`：**

```text
数据到达 (SSE ~10Hz)
    │
    ▼
app.js sync2DTarget()
    ├─ updateDeadReckon(x, z, speed, heading)   ← 写入 ground-truth
    │   (dedup + first-sample snap)
    │
    └─ 每帧 rAF (3D 和 2D 各自调用)
        │
        ▼
    tickDeadReckon()
        ├─ 速度外推: targetX = lastX + speed * elapsed
        ├─ 帧率无关平滑: smooth += (target - smooth) * (1 - exp(-λ·dt))
        └─ 角度环绕: dh 归一化到 [-π, π]
        │
        ▼
    scene3d.js / scene2d.js 读取 _dr.smoothX / smoothZ / smoothHeading
```

**关键设计决策：**

1. **帧率无关平滑公式** — `α = 1 - exp(-λ·dt)`，λ=8（位置）、λ=6（heading）。30fps 和 144fps 下的收敛时间常数完全相同。
2. **dt 钳位 0.1s** — 浏览器 tab 后台 10s 后切回，不会把车弹飞 100m。
3. **单一馈入点** — `app.js sync2DTarget()` 是唯一调用 `updateDeadReckon()` 的地方。`scene3d.js update3D()` 不再直接修改 `_dr`，只负责障碍物/LiDAR 世界锚定。
4. **首帧 snap** — `init=false` 时 `_dr.smooth*` 直接设为 ground-truth，避免从 (0,0) lerp 入场。
5. **heading 最短路径** — `while (dh > π) dh -= 2π; while (dh < -π) dh += 2π;`

## 单一 `window.flowboard` 命名空间（Phase 4 — 已落地）

`flowboard.html` 此前在每个模块文件底部做 `window.X = X` 一连串 monkey-patch（30+ 个全局），导致：

| 问题 | 位置 | 后果 |
|------|------|------|
| 全局变量污染 | `utils.js` / `deadreckon.js` / `charts.js` / `scene2d.js` / `scene3d.js` / `app.js` | 与浏览器原生 / 第三方 lib 命名冲突、Node.js 加载时 `window is not defined` |
| `window.topoData` 跨模块隐式共享 | `charts.js` / `scene3d.js` / `scene2d.js` | setter 无任何封装，多处同时写、谁最后写谁赢 |
| `debug3d.html` 直接 `window._debugCam = _camOverrides` | `debug3d.html:316` | 跨文件耦合，IDE 无法追踪 |
| HTML `onclick="toggleCard(this)"` 依赖 `window.toggleCard` | `index.html` | 重构时易漏，inline handler 实际指向 undefined → 静默失败 |

**修复方案 — 单一 `window.flowboard` 命名空间 + ES Module 内部通信：**

```text
app.js 顶部
   │   import { init3DScene, resize3D, update3D, setTopoData as setTopoData3D, setDebugCam } from './scene3d.js';
   │   import { init2D, init2DFallback, draw2D, switchSceneView, _2d, setTopoData as setTopoData2D } from './scene2d.js';
   │   import { initCharts, updateCharts, onChartTopicChange, onChartRangeChange, setTopoData as setTopoDataChart } from './charts.js';
   │   import { safeCall, reportDiag, clearDiag } from './utils.js';
   │   import { updateDeadReckon, _dr, initDeadReckon, tickDeadReckon } from './deadreckon.js';
   │
   │   function setTopoData(d) {           ← fan-out 模式
   │     setTopoData3D(d);                  ← scene3d 内部 _topoData
   │     setTopoData2D(d);                  ← scene2d 内部 _topoData
   │     setTopoDataChart(d);               ← charts 内部 _topoData
   │   }
   │
   │   updateAll() 顶部 setTopoData(topoData);  ← 单一馈入
   │
   ▼
window.flowboard = {  ← 唯一全局导出（HTML onclick 全部走它）
    initAll, doConnect, doSimulate, doPause, clearDiag, clearFrames,
    toggleExportMenu, exportPNG, exportCSV, onFilterChange, resetView,
    openTrainingModal, closeTrainingModal, refreshTrainingStatus,
    startTraining, syncTrainingForm, switchSysView, switchSceneView,
    onChartTopicChange, onChartRangeChange, closeDetail,
    saveState, resize3D, _2d: _2dState, _topoData: () => topoData,
};
```

**关键设计决策：**

1. **`window.topoData` → 模块内 `let _topoData` + `setTopoData(d)` setter** — 写时序明确，杜绝 race。
2. **`window._debugCam` → `setDebugCam(v)`** — `debug3d.html` 也走 ES module 导出，不再写 `window.*`。
3. **HTML inline `onclick` 全部改 `flowboard.X()`** — 集中式重命名安全。
4. **Node.js `smoke.mjs`** — mock `global.window` / `document` 后 `import('./app.js')`，5 个模块加载链全通过；保证重构不会因某个 `window.X` 漏删而破坏 SSR / 测试。
5. **`var _2d` 显式 `export { _2d }`** — `var` 不能用 `export function`，需在文件底部 `export { _2d }` 单独列出（`scene2d.js:550`）。
6. **`onChartTopicChange` / `onChartRangeChange` 在 `charts.js` 内是 `export function`** — 不能再用 `export { onChartTopicChange }` 重复声明。
