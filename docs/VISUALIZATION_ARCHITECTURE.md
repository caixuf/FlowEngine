# FlowEngine 可视化架构

> 本文档记录 FlowEngine 可视化系统的完整架构：后端 flowmond 监控守护进程 +
> 前端 vis/ 模块树（SceneDirector + Layer 对象树 + ViewRegistry 插件注册 +
> SceneStore 数据契约 + DeadReckon 死推算）。
>
> 前端架构借鉴 Qt QObject 对象树 + 单向依赖思路：定好数据层契约后，建模层
> 像插件一样可热插拔，单个 View 抛错不传染兄弟。

---

## 总览

```
业务节点 → flowmond（HTTP + IPC 桥接 + 文件桥接）→ 浏览器（vis/ 模块树）
                                                         │
                                                         ▼
                              ┌──────────────────────────────────────┐
                              │  app.js (Shell)                      │
                              │  ├─ SSE 接收 → setTopoData           │
                              │  └─ sync2DTarget → DeadReckon        │
                              └──────────────┬───────────────────────┘
                                             │
                                             ▼
                              ┌──────────────────────────────────────┐
                              │  SceneDirector (导演)                 │
                              │  ├─ validateFrame (纯函数校验)       │
                              │  ├─ update() 写 SceneStore +         │
                              │  │   静态 View build（条件分支）     │
                              │  └─ tickAnimation(now) 推进死推算 +  │
                              │      Layer 树递归 update             │
                              └──────────────┬───────────────────────┘
                                             │
                                             ▼
                              ┌──────────────────────────────────────┐
                              │  Layer 树（Qt 对象树 + 错误隔离）    │
                              │  root                                │
                              │  ├── env   (ground, viaduct)         │
                              │  ├── road  (road, streetlight,      │
                              │  │         barrier, connector)       │
                              │  ├── agent (vehicle)                 │
                              │  └── infra (trafficLight, etcGate)   │
                              └──────────────┬───────────────────────┘
                                             │
                                             ▼
                              ┌──────────────────────────────────────┐
                              │  View 实例（ViewRegistry 注册）      │
                              │  每个 View 独立 try/catch，抛错     │
                              │  只 log + 跳过，不传染兄弟          │
                              └──────────────────────────────────────┘
```

**数据流（单向，不可逆向）**：

```
SSE tick (5-10Hz)
  → app.js sync2DTarget() → updateDeadReckon() → _dr（死推算状态）
  → app.js setTopoData() → _director.update(topoData)
    → validateFrame()（纯函数校验）
    → 写 SceneStore（ego/entities/roadNetwork）
    → 静态 View build（roadHash 变了才重建）
  → main.js rAF (60fps) → _director.tickAnimation(now)
    → tickDeadReckon()（推进 _dr.smooth*）
    → 写回 store.ego（平滑位置）
    → rootLayer.update(store, now)（递归所有动态 View）
```

---

## 第一部分：后端 flowmond

### 架构原则

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

### 组件

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

### 使用方式

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

### 鲁棒性设计（解决"离线"反复出现）

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

### API 端点

```
GET /              → 实时仪表盘 HTML (tools/flowboard/index.html, SSE 自动更新)
GET /api/topology  → Bus 统计 + 拓扑 + 车辆/场景 JSON (含 source/stale/age_sec)
GET /api/topics    → Per-topic 统计 (频率/延迟/订阅者)
GET /api/stream    → SSE 事件流 (含心跳, source/stale 标记)
GET /api/health    → 探活: {status, source, age_sec}
```

---

## 第二部分：前端 vis/ 模块树架构

> 这是本系统的核心架构。借鉴 Qt QObject 对象树 + 单向依赖思路，解决
> "一个模块坏了整个 3D 就坏了"的痛点。

### 目录结构

```
tools/flowboard/js/
├── app.js                    # Shell：SSE 接收 + fan-out 到 3D/2D/charts
├── scene2d.js                # 2D Canvas 渲染（俯视图）
├── models.js                 # glTF 车辆模型工厂（车灯/车轮节点约定）
├── charts.js                # D3 图表
├── utils.js                  # safeCall / reportDiag
└── vis/                      # 3D 可视化模块树（本架构核心）
    ├── main.js               # 3D 入口：Three.js scene 初始化 + rAF 循环
    ├── core/                 # 核心抽象（View 只能 import，不能修改）
    │   ├── Layer.js          # Qt 对象树节点（父子 + 递归 dispose + 错误隔离）
    │   ├── ViewRegistry.js   # View 插件注册 + 工厂实例化 + safeCall
    │   ├── AssetFactory.js   # 共享几何体/材质工厂（getBox/getCylinder/...）
    │   └── DeadReckon.js     # 死推算引擎（帧率无关指数平滑）
    ├── math/                 # 数学工具（坐标/曲线/几何合并）
    │   ├── Coord.js          # ENU↔THREE 坐标映射（worldToThree/headingToRotationY）
    │   ├── Curve.js          # 道路曲线采样（sampleEdgeNodes，内部做 ENU→THREE 交换）
    │   └── GeometryMerge.js  # mergeGeometries（静态批量合并）
    ├── store/
    │   └── SceneStore.js     # 单一数据源（ego/entities/roadNetwork/isViaduct）
    ├── director/
    │   ├── SceneDirector.js  # 导演：校验 + 写 store + 静态 build + tickAnimation
    │   └── FrameValidator.js # 纯函数 validateFrame（零 THREE 依赖，可单测）
    └── view/                 # View 模块（插件式，每个独立）
        ├── RoadView.js       # 路面 ribbon + 车道线（静态 build）
        ├── GroundView.js     # 草地地面（静态 build）
        ├── VehicleView.js    # 车辆（动态 update，含车灯闪烁/车轮滚动）
        ├── ConnectorView.js  # 路口连接段（静态 build）
        ├── TrafficLightView.js # 红绿灯（动态 update，emissive 切换）
        ├── ETCGateView.js    # ETC 门架（动态 update，boom 抬杆）
        ├── ViaductView.js    # 高架桥（静态 build，含环境）
        ├── StreetlightView.js # 路灯（InstancedMesh 静态 build）
        ├── BarrierView.js    # 护栏（InstancedMesh 静态 build）
        └── VehicleLights.js  # 纯函数：车灯位掩码 → 状态（零 THREE 依赖）
```

### SceneStore — 单一数据契约

`vis/store/SceneStore.js` 是唯一数据源，所有 View 只读不写。

```javascript
{
  roadNetwork: {              // road_network hash 变了才重建静态 View
    edges: [{ id, type, lanes, lane_width, length, nodes: [[x,y,z]...] }],
    junctions: [...],
  },
  roadHash: 'abc123',         // roadNetworkHash(rn) 结果，diff 用
  isViaduct: false,           // 高架场景标志（决定路灯/护栏是否布置）
  ego: {                      // 主车（tickAnimation 每帧覆盖 x/y/heading/speed）
    x, y, z,                  // THREE 坐标（已 worldToThree）
    heading, speed,           // 弧度 + m/s
    steer, brake, throttle,  // 控制量
    lights,                   // 车灯位掩码（见 VehicleLights.js）
    vx, vy,                   // 速度分量
    length, width,            // 尺寸
    ai_state,                 // AI 状态字符串
    _simX, _visualX, _wrapOffset, // 高架场景循环回绕用
  },
  entities: [                 // NPC + 红绿灯 + ETC 门架等（已过滤 ego）
    { id, type, x, y, z, heading, speed, ... }
  ],
}
```

**铁律**：
- View 只读 `store`，绝不反向调 SceneDirector 方法（单向依赖）
- View 不能直接读 `topoData` / `window.*` / DOM
- View 状态必须闭包在 factory 函数内，不能创建全局变量

### SceneDirector — 导演

`vis/director/SceneDirector.js` 负责三件事：

1. **数据校验** — `validateFrame(topoData)`（纯函数，零 THREE 依赖）校验
   schema，失败只 warn 一次（`_warnOnce`，避免 20Hz 刷屏），不抛错。
2. **写 store + 静态 View build** — `update(topoData)` 解包 frame，写
   SceneStore。roadNetwork hash 变了才重建静态 View（高架 vs 普通道路分支不同）。
3. **tickAnimation(now)** — 每帧推进死推算 + 把平滑结果写回 store.ego +
   递归调 Layer 树 update 所有动态 View。

```javascript
function tickAnimation(now) {
  tickDeadReckon();                    // 推进 _dr.smooth*
  if (store.ego && _dr.init) {
    store.ego.x = _dr.smoothX;         // 覆盖 x/y/heading/speed
    store.ego.y = _dr.smoothZ;
    store.ego.heading = _dr.smoothHeading;
    store.ego.speed = _dr.smoothSpeed;
  }
  rootLayer.update(store, now);       // 递归 agent/infra 层所有动态 View
}
```

### Layer 对象树 — Qt 风格

`vis/core/Layer.js` 实现 Qt QObject 风格的 View 容器节点，三大特性：

1. **父子关系** — `addChild` 自动设 parent + 从原 parent 移除（Qt reparent）；
   `findChild` / `findDescendant` 按名字查找。
2. **单向依赖** — 子只读 store + 持 parent 引用，绝不反向调父方法。
3. **递归 dispose** — 父 dispose 自动递归子层（深度优先，孙先销毁），
   view 若有 `dispose()` 则调，否则退化为 `clear()`。幂等。

SceneDirector 构造时建 4-Layer 树，所有 9 个 View 都挂到对应层：

```
root
├── env     (ground, viaduct)                      — 环境层
├── road    (road, streetlight, barrier, connector) — 道路层
├── agent   (vehicle)                              — 智能体层
└── infra   (trafficLight, etcGate)                — 路侧设施层
```

**关键 API**：
- `build(ctx)` / `update(store, now)` / `clear()` — 递归调用所有后代
- `dispose()` — 递归销毁（孙先，子后，幂等）
- `markFailed()` / `resetFailure()` — 失败标记跳过后续调用
- `getRootLayer()` / `getLayer(name)` — SceneDirector 暴露的调试 API

### ViewRegistry — 插件注册

`vis/core/ViewRegistry.js` 实现 View 工厂注册 + 实例化 + 错误隔离调用：

```javascript
ViewRegistry.register('road', createRoadView);     // 注册工厂
ViewRegistry.instantiateAll(scene);                 // 批量实例化（try/catch）
const roadView = ViewRegistry.get('road');           // 取实例
ViewRegistry.safeCall('road', 'build', rn);          // 安全调用（抛错只 log）
```

**与 Layer 的分工**：
- ViewRegistry 管"插件注册 + 工厂实例化"
- Layer 管"运行时父子树 + 递归调用 + dispose"
- 一个 View 被 registry 实例化后挂到某 Layer 下作为 child

### 错误隔离机制（核心）

这是"一个模块坏了整个 3D 就坏了"的解药：

1. **ViewRegistry.instantiateAll** — 工厂本身抛错只 log + 标记 `_failed`，
   不传染其他 View 的实例化。
2. **Layer.build/update/clear/dispose** — 每个 view/child 调用包 try/catch，
   单个抛错只 log + 跳过，兄弟继续渲染。
3. **ViewRegistry.safeCall** — SceneDirector 里静态 View 的 build 调用走
   safeCall，抛错只 log + 标记 `_failed`。
4. **update 抛错不标记 _failed** — 60fps 偶发错可能下次自愈，只 log 不永久跳过。
5. **build 抛错标记 _failed** — 静态 View build 失败后跳过后续 build/update，
   避免每帧刷屏；`resetFailures()` 可重试。

```javascript
// Layer.update 内部
for (const v of views) {
  if (typeof v.update !== 'function') continue;
  try { v.update(store, now); }
  catch (err) {
    console.error('[Layer ' + name + '] view.update threw:', err);
    // 不标记 _failed —— 60fps 偶发错可能下次自愈
  }
}
for (const c of children) {
  try { c.update(store, now); }
  catch (err) { console.error('[Layer ' + name + '] child ' + c.name + '.update threw:', err); }
}
```

### DeadReckon — 帧率无关死推算

`vis/core/DeadReckon.js` 解决 5-10Hz SSE 数据 → 60fps 渲染的平滑问题：

- **`updateDeadReckon(x, z, speed, heading)`** — SSE 数据到达时调，写 ground-truth
- **`tickDeadReckon()`** — 每帧 rAF 调，推进 `_dr.smooth*`
- **帧率无关平滑** — `α = 1 - exp(-λ·dt)`，λ_pos=8，λ_heading=6
- **dt 钳位 0.1s** — 浏览器 tab 后台 10s 后切回，不会把车弹飞 100m
- **heading 最短路径** — 跨 ±π 时归一化到 [-π, π]，不整圈旋转
- **首帧 snap** — `init=false` 时 smooth* 直接设为 ground-truth，避免从 (0,0) lerp

3D 和 2D 共享同一 `_dr` 全局单例，保证两个视图 ego 位置完全同步。

### 坐标映射 — ENU ↔ THREE

`vis/math/Coord.js` 的 `worldToThree(x, y, z)` 做 ENU→THREE 交换：
`world(x, y, z) → three(x, z, y)`

**重要约定**：
- `sampleEdgeNodes`（Curve.js）内部已做 ENU→THREE 交换，所以
  StreetlightView/BarrierView 直接用 `p.x`/`p.z`，**不调 worldToThree**（否则双交换）
- 其他 View（VehicleView/TrafficLightView/ETCGateView）消费 entity 坐标时调 `worldToThree`

`headingToRotationY(heading)` 把 ENU heading（弧度，逆时针正）转 THREE Y 旋转。

---

## 第三部分：View 接口契约

每个 View 模块导出一个 factory 函数：

```javascript
// vis/view/XxxView.js
import { getBox, getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { worldToThree, headingToRotationY } from '../math/Coord.js';

export function createXxxView(scene) {
  const pool = new Map();  // id → { group, ... } 闭包状态

  // 可选：静态布局 build（静态 View 用，road_network 变了才调）
  function build(rn) { /* InstancedMesh 沿路布置 */ }

  // 必选：每帧 update（动态 View 用，由 Layer 树递归调）
  function update(store, simTime) {
    // 1. 从 store 读数据（entities/ego/roadNetwork）
    // 2. diff：删除消失的实例
    // 3. 创建/更新
  }

  // 必选：清空所有实例（场景重建时调）
  function clear() {
    for (const [, entry] of pool) scene.remove(entry.group);
    pool.clear();
  }

  // 可选：销毁（Layer.dispose 递归调；若无则退化为 clear）
  function dispose() { clear(); /* 释放 geometry/material */ }

  return { build, update, clear, dispose };  // 按需导出
}
```

**View 分两类**：
- **静态布局 View**（road/ground/connector/streetlight/barrier/viaduct）— 只 `build`，
  roadNetwork hash 变了才重建。不挂 Layer 树的 update 路径（无 update 方法被跳过）。
- **动态更新 View**（vehicle/trafficLight/etcGate）— 每帧 `update`，
  挂到 Layer 树由 `tickAnimation → rootLayer.update` 递归调用。

---

## 第四部分：扩展指南 — 加新 View

### 加一个动态 View（每帧更新，如行人）

1. 写 `vis/view/PedestrianView.js`，导出 `createPedestrianView(scene)` 返回 `{ update, clear }`
2. 在 SceneDirector.js 顶部 `ViewRegistry.register('pedestrian', createPedestrianView)`
3. 在 Layer 挂载表加一行：`['agent', ['vehicle', 'pedestrian']]`
4. 完成。`tickAnimation` 会自动每帧调 `pedestrian.update(store, now)`，
   抛错只 log 不传染 vehicle。

### 加一个静态 View（沿路布置，如路牌）

1. 写 `vis/view/SignpostView.js`，导出 `createSignpostView(scene)` 返回 `{ build, clear }`
2. SceneDirector.js 注册 + 挂到 road Layer（`['road', [..., 'signpost']]`）
3. 在 `update()` 的普通道路分支加 `signpostView.build(rn)`（走 `ViewRegistry.safeCall`）
4. 完成。

### 换一辆车模型

`VehicleView.js` 的 `GLTF_TYPE_MAP` 控制车型映射：

```javascript
const GLTF_TYPE_MAP = {
  ego: 'su7',    // ego 用 SU7
  car: 'sedan',
  suv: 'suv',
  truck: 'truck',
};
```

1. 把新 glTF 文件（如 `porsche.glb`）丢到 `tools/flowboard/models/`
2. 改 `GLTF_TYPE_MAP.ego = 'porsche'`
3. 只要新模型符合节点命名约定（`wheel_FL/FR/RL/RR`、`brakelight_L/R`、
   `turnsignal_FL/...`），车灯闪烁、车轮滚动、刹车灯都会自动联动

---

## 第五部分：历史记录（Phase 2-4 增量重构，已落地）

### `scene` 数据结构（真实 3D 仿真）

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

### 共享道路几何（Phase 2 — 已落地）

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

**关键教训：** dlopen 单进程模式下所有节点共享同一个 `MessageBus`，订阅者总数受 `MSG_BUS_MAX_SUBSCRIBERS`（默认 32）限制。本轮重构中 monitor 的 `road/geometry` 订阅因总订阅数（33）溢出而静默失败，已上调至 128。

### 统一航位推算（Phase 3 — 已落地）

前端以 ~10Hz 接收 SSE 数据，但渲染以 60fps 运行。如果每帧直接渲染最新数据，车辆会在数据帧之间**冻结**。此前的航位推算状态分散在三个模块中。

**修复方案 — 单一引擎 `vis/core/DeadReckon.js`：**

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
    vis/main.js (3D) + scene2d.js (2D) 读 _dr.smooth*
```

**关键设计决策：**
1. **帧率无关平滑公式** — `α = 1 - exp(-λ·dt)`，λ=8（位置）、λ=6（heading）
2. **dt 钳位 0.1s** — 浏览器 tab 后台 10s 后切回，不会把车弹飞 100m
3. **单一馈入点** — `app.js sync2DTarget()` 是唯一调用 `updateDeadReckon()` 的地方
4. **首帧 snap** — `init=false` 时 `_dr.smooth*` 直接设为 ground-truth
5. **heading 最短路径** — `while (dh > π) dh -= 2π; while (dh < -π) dh += 2π;`

### 单一 `window.flowboard` 命名空间（Phase 4 — 已落地）

此前每个模块文件底部做 `window.X = X` 一连串 monkey-patch（30+ 个全局），导致命名冲突、Node.js 加载报错。

**修复方案 — 单一 `window.flowboard` 命名空间 + ES Module 内部通信：**

`app.js` 顶部统一 import 所有模块，fan-out `setTopoData(d)` 到 3D/2D/charts。
HTML inline `onclick` 全部改走 `window.flowboard.X()`，集中式重命名安全。

### vis/ 模块树重构（Phase 4+ — 已落地）

历史上 3D 渲染集中在 `scene3d.js` 一个 God Object（~2000 行），导致：
- 一个 View 坏了整个 3D 崩
- 难以定位 bug（所有逻辑耦合在一个文件）
- 无法单独测试某个 View 的逻辑

**重构方案 — 6 步小步快走**（详见 git log）：

1. SceneDirector 数据校验（`_warnOnce` + 8 项 schema）
2. DeadReckon 下沉到 `vis/core/DeadReckon.js`，`tickAnimation` 内聚到 SceneDirector
3. `Coord.worldToThree` 统一坐标映射
4. StreetlightView + BarrierView（InstancedMesh 沿路布置）
5. 纯函数抽离（VehicleLights + FrameValidator，可单测）
6. 文档 + 进度索引

**架构升级 — Layer 对象树 + ViewRegistry 插件化**：
- 引入 `Layer.js`（Qt 对象树 + 递归 dispose + 错误隔离）
- SceneDirector 改用 4-Layer 树驱动每帧 update
- 所有 9 个 View 挂到完整 Layer 树，`dispose()` 递归清理
- ViewRegistry 管插件注册，新增 View 只需 `register + addView`

**测试覆盖**（6 个测试文件，188 case，CI vis-js-tests job 守护）：
- `vis_director_validation.test.mjs` (46) — validateFrame 纯函数
- `vis_director_layer.test.mjs` (42) — SceneDirector × Layer 集成
- `vis_layer.test.mjs` (40) — Layer 对象树行为
- `vis_view_registry.test.mjs` (37) — ViewRegistry 插件注册
- `vis_vehicle_lights.test.mjs` (18) — 车灯位掩码
- `vis_geometry.test.mjs` (5) — 坐标映射 + 顶点法线
