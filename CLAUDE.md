# FlowEngine — 项目概览

轻量级自动驾驶中间件，核心是一个 Pub/Sub 消息总线 + 调度器 + 传输层。

## 架构

```
PerceptionTask → FusionTask → ControlTask → MonitorTask
     ↓              ↓             ↓             ↓
  sensor/lidar   sensor/gps   fusion/obj    control/cmd
     ↓              ↓             ↓             ↓
  ════════════════ Message Bus ════════════════════
                         ↓
                  Transport (IPC/TCP) → Discovery → FlowRegistry
                         ↓
                  FlowBoard Server (HTTP/SSE) → DashBoard
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/e2e_demo.c` | 端到端演示：感知→融合→控制→监控 |
| `src/core/message_bus.c` | 进程内 Pub/Sub 总线 |
| `src/core/transport.c` | 统一传输抽象（local/IPC/TCP） |
| `src/core/scheduler.c` | 任务调度器（classic/choreo 模式） |
| `tools/flowboard_server.py` | 仪表盘 HTTP/SSE 服务器 |
| `tools/flowboard.html` | 前端仪表盘（3D+2D+图表+拓扑） |
| `tools/foxglove_bridge.py` | Foxglove Studio WebSocket 桥接 |
| `scripts/demo.sh` | 一键启动脚本 |

## 运行

```bash
bash scripts/demo.sh [duration]          # 启动演示
bash scripts/demo.sh --no-browser 15     # 不打开浏览器
```

仪表盘: `http://localhost:8800`
3D 桥接: `ws://localhost:8765`

---

# 可视化进化路线图

## 现状

- 传感器: 代码生成假 LiDAR/GPS
- 感知: 无
- 融合: 仅时间对齐
- 规划: 硬编码 lane_target
- 控制: PID 纵向 + P 横向
- 地图: 手工 4 车道直线
- 可视化: 手工 Three.js 3D + Canvas 2D fallback

## 目标技术栈

```
┌──────────────────────────────────────────────────┐
│                    数据源                         │
│  CARLA 仿真器  or  nuScenes/Waymo 回放           │
└──────────────────────┬───────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────┐
│               算法引擎 (Autoware.Universe)         │
│  Perception → Planning → Control                 │
│  每个节点独立 ROS2 package，topic 输入输出         │
└──────────────────────┬───────────────────────────┘
                       ↓ (ROS2↔Flow bridge, ~300行)
┌──────────────────────────────────────────────────┐
│              FlowEngine Message Bus               │
│   sensor/*  fusion/*  planning/*  control/*       │
│   map/*     perception/*                          │
└──────┬───────────────────────────────┬────────────┘
       ↓                               ↓
┌──────────────────┐    ┌──────────────────────────┐
│  FlowBoard (Web)  │    │  Foxglove Studio (桌面)   │
│  ┌──────────────┐ │    │  MCAP 回放 + 时间轴       │
│  │streetscape.gl│ │    │  3D 全景 + 点云渲染       │
│  │ XvizLayer    │ │    │  (已有 bridge @8765)      │
│  ├──────────────┤ │    └──────────────────────────┘
│  │ D3.js Charts │ │
│  │ 拓扑图        │ │
│  └──────────────┘ │
└──────────────────┘
```

## Phase 计划

### Phase 0: 环境搭建 (当前)
- [ ] 安装 CARLA 0.9.15 (Docker 或源码)
- [ ] 安装 Autoware.Universe (Docker)
- [ ] 验证 CARLA-Autoware 联通
- **开源依赖**: CARLA, Autoware.Universe

### Phase 1: ROS2↔FlowEngine Bridge (2天)
- [ ] 写 `ros2_flow_bridge` 节点
  - ROS2 侧: 订阅 `/perception/*`, `/planning/*`, `/control/*`
  - FlowEngine 侧: 发布到对应 topic
  - 消息格式转换 (ROS2 msg → FlowEngine internal)
- **开源依赖**: rclcpp/rclpy
- **工作量**: ~300 行 C++/Python

### Phase 2: CARLA→Autoware 感知管线 (1周)
- [ ] CARLA 连 Autoware `lidar_centerpoint`
- [ ] 检测框经 bridge 进 FlowEngine
- [ ] 3D 场景渲染检测框
- **开源依赖**: Autoware Perception 模块, OpenPCDet (推理)

### Phase 3: streetscape.gl 替换手工 Three.js (1周)
- [ ] 安装 streetscape.gl + deck.gl (React)
- [ ] 写 XVIZ 格式转换层 (FlowEngine data → XVIZ frames)
- [ ] XvizLayer 渲染: 检测框, 车道线, 轨迹, 点云
- [ ] 保留 D3.js 图表和拓扑图
- **开源依赖**: [streetscape.gl](https://github.com/uber/streetscape.gl), [deck.gl](https://deck.gl/)
- **替换**: ~800 行手工 Three.js → ~50 行 XVIZ 转换 + 组件

### Phase 4: Autoware 规划控制接入 (1周)
- [ ] `behavior_path_planner` → 变道决策
- [ ] `trajectory_follower` → MPC 控制
- [ ] 过 bridge 驱动 CARLA 车辆
- **开源依赖**: Autoware Planning + Control 模块

### Phase 5: HD Map (Lanelet2) 渲染 (3天)
- [ ] Lanelet2 加载 CARLA Town 地图
- [ ] 经 bridge 发布到 `map/lanelet` topic
- [ ] streetscape.gl 渲染车道线 (从地图数据驱动)
- **开源依赖**: Lanelet2, streetscape.gl 内置 MapLayer

### Phase 6: Foxglove MCAP 完善 (1天)
- [ ] 补全 MCAP channel (检测框, 轨迹, 地图)
- [ ] Foxglove Studio 布局模板
- **开源依赖**: 已有 mcap_writer + foxglove_bridge

## 开源全家福

| 模块 | 开源项目 | 规模 | 链接 |
|------|---------|------|------|
| 全套 #1 | Apollo | ★★★★★ | github.com/ApolloAuto/apollo |
| 全套 #2 | Autoware.Universe | ★★★★★ | github.com/autowarefoundation/autoware.universe |
| 仿真器 | CARLA | ★★★★ | github.com/carla-simulator/carla |
| 数据集 | nuScenes | ★★★ | nuscenes.org |
| 数据集 | Waymo Open | ★★★★ | waymo.com/open |
| 感知 | OpenPCDet | ★★★★ | github.com/open-mmlab/OpenPCDet |
| 感知 | MMDetection3D | ★★★★★ | github.com/open-mmlab/mmdetection3d |
| 跟踪 | AB3DMOT | ★★ | github.com/xinshuoweng/AB3DMOT |
| 融合 | Apollo Fusion | ★★★ | apollo.auto |
| 地图 | Lanelet2 | ★★★ | github.com/fzi-forschungszentrum-informatik/Lanelet2 |
| 规划 | Apollo/ Autoware Planning | ★★★★ | apollo.auto / autoware.org |
| 控制 | Apollo/ Autoware Control (MPC) | ★★★ | apollo.auto / autoware.org |
| BEV 可视化 | streetscape.gl | ★★★ | github.com/uber/streetscape.gl |
| 3D 可视化 | Foxglove Studio | ★★★ | github.com/foxglove/studio |
| 交通流 | SUMO | ★★★ | github.com/eclipse-sumo/sumo |
