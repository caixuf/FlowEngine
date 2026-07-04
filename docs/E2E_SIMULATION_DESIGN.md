# E2E 仿真 & 可视化设计（offline 根因修复 + 真实 3D 仿真）

本文档回答两个问题：

1. **为什么可视化面板会反复显示 `● offline` 并退化为假数据？** —— 根因是什么、怎么系统性修好。
2. **e2e demo 怎样从"数字跳动"升级为真正的 3D 仿真？** —— 场景模型、数据契约、渲染方案。

---

## 一、offline 问题：根因分析

### 1.1 现有数据链路

```
flow_e2e (业务进程)
  └─ monitor 任务每 1s 原子写入 /tmp/flow_topology.json
        │  (discovery + bus/latency/topic 统计 + vehicle 遥测)
        ▼
flowboard_server.py (HTTP 桥接)
  ├─ file_watcher 线程轮询 json → 归一化为 FlowBoard 模型
  ├─ GET /api/topology  一次性快照
  └─ GET /api/stream    SSE，每 1s 推一帧
        │
        ▼
flowboard.html (浏览器)
  ├─ doConnect(): fetch /api/topology → startSSE()
  └─ 连接失败重试 3 次 → setConnStatus('dead','● offline') → doSimulate()
```

### 1.2 真正的根因：HTTP 服务器是**单线程**的

`flowboard_server.py` 用的是 `http.server.HTTPServer`，它继承自
`socketserver.TCPServer`，**没有** `ThreadingMixIn`，因此一次只能处理一个请求。

而 `/api/stream` 的 SSE handler 会在一个循环里**阻塞长达 5 分钟**（每秒推一帧、
共 300 次）。于是：

- 浏览器 A 打开面板 → 建立 SSE 长连接 → 服务器唯一的工作线程被这条连接**独占**。
- 此时任何其它请求都排不上队：
  - 第二个标签页 / 刷新页面时的 `GET /api/topology`；
  - `EventSource` 因网络抖动 `onerror` 后自动重连发起的新连接；
  - 面板自身的 `tryReconnect()` / `doConnect()` 里的 `fetch`。
- 这些请求在 TCP backlog 里干等 → 浏览器侧超时 → `doConnect` 连续失败 3 次 →
  `setConnStatus('dead','● offline')` → `doSimulate()` 显示**假数据**。

复现（见 PR）：SSE 连接活跃时 `curl -m 3 /api/topology` 直接超时、返回 0 字节。

这就是"又出现了、鲁棒性太差"的本质——**架构缺陷**，不是偶发。任何一次页面刷新
或网络抖动都可能触发，且一旦触发面板就悄悄切到模拟数据，让人误以为链路还活着。

### 1.3 次要设计问题

| 问题 | 影响 |
|------|------|
| SSE 循环用"迭代次数"（300 次）当超时，而非墙钟时间 | 卡顿时长不可控 |
| 面板把"服务器可达但数据陈旧"和"服务器不可达"混为一谈 | 无法区分是 e2e 停了还是桥接挂了 |
| 失败后**静默**切换到 `doSimulate()` 假数据 | 掩盖真实故障，误导排查 |
| 文档（VISUALIZATION_ARCHITECTURE）写的是 `flowmond` 守护进程，实际脚本用的是 `flowboard_server.py` | 两套设计并存、互相矛盾 |

> 关于 `flowmond`：它创建自己的**进程内** `message_bus`，与 `flow_e2e` 不共享总线，
> 因此拿不到业务节点的 topic 统计。跨进程聚合需要 IPC/TCP 桥接，当前未实现。
> 所以**实际可用**的可视化链路是"状态文件 + flowboard_server.py"。本设计以此为准，
> 并在文档中消除矛盾。

---

## 二、offline 问题：修复方案

### 2.1 根因修复 —— 服务器多线程化

`HTTPServer` → `ThreadingHTTPServer`（Python 3.7+ 标准库自带）。每个连接一个线程，
SSE 长连接不再阻塞快照/重连请求。这是**一行级**的根因修复。

### 2.2 韧性增强

- **SSE 心跳 + 墙钟超时**：改用墙钟计时，并周期性发送 `: keep-alive` 注释帧，
  让代理/浏览器不会误判连接死亡。
- **数据新鲜度**：状态文件带 `timestamp`，服务器在 `/api/topology`、`/api/stream`
  中附带 `stale`（数据是否超过阈值未更新）字段，并新增 `GET /api/health`。
- **写-读竞态**：e2e 已经"写临时文件 + `rename`"原子落盘；服务器 `JSONDecodeError`
  时跳过本轮（沿用），避免读到半截文件。

### 2.3 前端三态化

面板连接状态从二态（live / offline）升级为**三态**：

| 状态 | 含义 | 处理 |
|------|------|------|
| `● live` | 服务器可达且数据新鲜 | 正常显示实时数据 |
| `● stale` | 服务器可达但数据超时未更新（e2e 已停止/卡住） | 显示最后一帧真实数据 + 陈旧提示，**不切假数据** |
| `● offline` | 服务器不可达 | 才允许退化到 `demo` 模拟，并**明确标注 DEMO** |

要点：**只有服务器真正不可达时才用模拟数据，且必须显式标注**，杜绝"假数据冒充实时"。
重试次数与退避也放宽，避免一次抖动就判死。

---

## 三、真实 3D 仿真方案

### 3.1 现状差距

当前"3D"是装饰性的：LiDAR 点是 `Math.random()` 生成的、没有真实障碍物、控制只有
纵向 PID（`steer=0.00` 写死），车辆沿直线跑。谈不上"仿真"。

### 3.2 目标

在**零新增依赖**前提下，把 e2e 变成一个可解释的闭环 3D 场景：

```
       ┌── 障碍物运动学（前车/对向车/行人）
场景 ──┤── 自车 bicycle 模型（纵向 + 横向转向 + 航向）
       └── LiDAR 光线投射（对障碍物/地面生成点云）
             │  (真实几何，非随机)
             ▼
      感知 → 融合 → 控制(ACC 跟车 + 车道保持)
             │
             ▼
   scene JSON  →  flowboard 3D 视图渲染真实障碍物 + 点云
```

### 3.3 场景与动力学

- **障碍物**：一个前车（同车道、变速），一辆对向车，一名过街行人。每个 tick 按简单
  运动学更新位置/速度。
- **自车**：在原有纵向质点模型上增加**横向 bicycle 模型**——用转向角更新航向
  `heading`，从而 `x/y` 走出真实轨迹（含一次温和变道）。
- **ACC 跟车**：控制器根据与前车的**纵向间距**动态调整 `target_speed`（间距近则减速/
  制动），让"BRAKE"是有物理来由的，而不是固定目标速度下的数值波动。
- **LiDAR 点云**：由障碍物包围盒 + 地面环带**投射**生成若干点（自车坐标系），
  下采样后导出，供 3D 视图真实显示。

### 3.4 数据契约（新增 `scene`）

e2e monitor 任务在状态 JSON 的 `metrics` 下新增 `scene`：

```jsonc
"scene": {
  "ego":       { "x": 0, "y": 0, "heading": 0.0, "speed": 8.3, "steer": 0.02 },
  "obstacles": [
    { "id": 0, "type": "car",        "x": 32.0, "y": 0.0,  "vx": 6.0, "len": 4.6, "wid": 2.0 },
    { "id": 1, "type": "car",        "x": 70.0, "y": -3.5, "vx": -8.0,"len": 4.6, "wid": 2.0 },
    { "id": 2, "type": "pedestrian", "x": 20.0, "y": 6.0,  "vx": 0.0, "len": 0.6, "wid": 0.6 }
  ],
  "lidar": [ [x,y,z], ... ],          // 自车坐标系，已下采样
  "lane":  { "width": 3.5, "count": 2 }
}
```

坐标系约定：`x` 纵向（前为正）、`y` 横向（左为正）、`heading` 弧度、单位米/秒。
`scene` 缺省时（老数据）前端回退到原有行为，向后兼容。

### 3.5 渲染

`flowboard.html` 的 Three.js 视图改为：自车按 `ego` 位姿摆放，`obstacles` 渲染为
彩色包围盒（车=蓝、对向=橙、行人=绿），`lidar` 用真实点坐标绘制点云；无 `scene`
时保留旧的随机点行为。2D BEV 兜底视图同理优先用真实数据。

---

## 四、实施顺序

1. 文档（本文件）。
2. offline 根因修复：`flowboard_server.py` 多线程化 + 新鲜度/health。
3. 前端三态化：`flowboard.html`。
4. e2e 真实场景：`e2e_demo.c` 障碍物/横向/ACC/LiDAR + 导出 `scene`。
5. 3D 视图渲染真实 `scene`。
6. 更新 `VISUALIZATION_ARCHITECTURE.md` / `SIMULATION_GUIDE.md`。
7. 构建 + 无头冒烟验证（并发请求不再超时、`scene` 正确落盘）。

## 五、验收标准

- SSE 连接活跃时，`/api/topology` 并发请求**不再超时**（根因已除）。
- e2e 停止后面板显示 `● stale` + 最后真实数据，**不会**静默变成假数据。
- 状态文件包含结构正确、随时间变化的 `scene`（障碍物移动、自车变道、点云非随机）。
- 面板 3D 视图显示真实障碍物与点云。
