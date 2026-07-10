# FlowEngine — 项目概览

轻量级自动驾驶中间件，核心是一个 Pub/Sub 消息总线 + 调度器 + 传输层。

## 架构

```
sim_world → sensor_model → perception → fusion → planning → control → safety_control → monitor
     ↓            ↓             ↓           ↓          ↓          ↓            ↓            ↓
 vehicle/state  sensor/lidar perception/  fusion/  planning/  control/raw  control/cmd  dashboard
                sensor/gps  obstacles   localization trajectory  _cmd                     JSON
                     ↓             ↓           ↓          ↓          ↓            ↓
              ════════════════ Message Bus ════════════════════
                                    ↓
                             Transport (IPC/TCP) → Discovery → FlowRegistry
                                    ↓
                             flowmond (IPC stats bridge + HTTP/SSE) → DashBoard
```

## 关键文件

| 文件 | 作用 |
|------|------|
| `src/e2e_demo.c` | 端到端演示：感知→融合→控制→监控 |
| `src/core/message_bus.c` | 进程内 Pub/Sub 总线 |
| `src/core/transport.c` | 统一传输抽象（local/IPC/TCP） |
| `src/core/scheduler.c` | 任务调度器（classic/choreo 模式） |
| `src/flowmond.c` | 监控守护进程（HTTP 仪表盘 + IPC 统计/仪表盘桥接 + 自动重连） |
| `src/core/monitor_server.c` | 内嵌 HTTP 服务器（多线程连接、/tools 静态资源、JSON 安全转义、过期缓存自动 fallback） |
| `src/core/stats_bridge.c` | 跨进程 topic 统计 IPC 桥接 |
| `src/core/dashboard_bridge.c` | 跨进程仪表盘 JSON IPC 桥接（分块传输协议） |
| `modules/adas_nodes/sensor_model_node.c` | 传感器模型（LiDAR/GPS/Camera，FOV/遮挡/噪声） |
| `modules/adas_nodes/safety_control_node.cpp` | FlowCoro 协程安全控制（TTC/横向交叉/行人防护） |
| `tools/flowboard.html` | 前端仪表盘（3D+2D+图表+D3 拓扑） |
| `tools/foxglove_bridge.py` | Foxglove Studio WebSocket 桥接 |
| `tools/demo_evaluator.py` | 回归评估器：采样 JSON 并自动评分（碰撞/偏航/停滞/频率） |
| `scripts/demo.sh` | 一键启动脚本 |
| `src/flow_launcher.c` | 配置驱动启动器（读取 pipeline.json，dlopen 加载插件节点） |
| `src/flowctl.c` | CLI 工具（list/inspect/dashboard/param/bag 等子命令） |
| `src/core/flow_registry.c` | 统一元信息注册中心（Task/Topic/Type/Plugin/Schema） |
| `src/core/param_registry.c` | 参数系统（int/float/bool/string，范围校验，hot-reload） |
| `src/core/scenario_loader.c` | 场景 JSON 加载器（actor 定义 + ego 配置） |
| `tools/bag_check.c` | Bag 文件完整性检查器 |
| `scenarios/pedestrian_crossing.json` | 行人横穿场景 |
| `scenarios/highway_overtake.json` | 高速超车场景 |
| `modules/adas_nodes/data_recorder_node.c` | 训练样本采集（Learning Loop Stage 0） |
| `modules/adas_nodes/inference_node.c` | tiny-MLP 影子推理（Learning Loop Stage 2） |
| `modules/adas_nodes/tiny_mlp.h` | 纯 C 单隐层 MLP 推理内核 |
| `tools/train/train.py` | 离线训练脚本（scikit-learn MLP） |
| `docs/LEARNING_LOOP.md` | 车端学习闭环架构 |

> 深入教程见 `skills/` 目录（11 篇，覆盖 OOP in C、插件系统、消息总线、IPC、Bag、Clock、
> Serializer、State Machine、Discovery、Fusion、Coroutine）。

## 运行

```bash
bash scripts/demo.sh [duration]          # 启动演示
bash scripts/demo.sh --no-browser 15     # 不打开浏览器
```

仪表盘: `http://localhost:8800`
3D 桥接: `ws://localhost:8765`

## 验证

```bash
# 每次改动 pipeline 链路上的节点后，跑评估器
python3 tools/demo_evaluator.py --duration 45 --interval 0.5

# 仅分析当前数据，不重新启动 demo
python3 tools/demo_evaluator.py --no-run
```

评估器采样 `/tmp/flow_topology.json`，自动检查：拓扑完整性、topic 频率、碰撞、路沿偏离、停滞、变道次数、偏航抖动、NPC 瞬移。WARN 是已知问题可忽略，FAIL 必须修复。

## 常见故障模式

| 现象 | 根因 | 位置 |
|------|------|------|
| 车速降到 0 后永久卡死 | ROAD_GUARD 低俗恢复条件要求 `|y|>=road_center_limit`，但车可在任意 `2.1<|y|<2.5` 停下。改为只要 `speed<2.5` 就给小油门 | `control_node.c:534` |
| 变道冲出车道 | Stanley heading 阻尼硬编码 0.5，pipeline.json 的 `lat_kd_heading` 未生效 | `control_node.c:548` |
| NPC 瞬移 | 障碍物回收逻辑放入 100m 外（设计如此，非 bug） | `sim_world_node.c:204` |
| 感知降频 | DBSCAN 点云过多时聚类耗时超过 deadline | `perception_node.c` |

## 最新 tag

`v0.1.0` — 创始版本，8 节点全链路稳定运行

---

# 可视化架构

详见 [可视化架构](docs/VISUALIZATION_ARCHITECTURE.md)。

**主链路（文件桥接，默认）：**
```
monitor_node → 10Hz 写 /tmp/flow_topology.json → flowboard_server.py :8800 → 浏览器
```

**辅助链路（flowmond IPC 桥接）：**
```
monitor_node → stats_bridge / dashboard_bridge → IPC SHM → flowmond :8800 → 浏览器
```

| 组件 | 端口 | 说明 |
|------|------|------|
| `flowboard_server.py` | 8800 | 默认仪表盘：读取 JSON 文件，HTTP/SSE 推送 |
| `flowmond` (C) | 8800 | 可选：IPC 统计聚合 + 拓扑发现 + 自动重连 |
| `foxglove_bridge.py` | 8765 | Foxglove Studio 3D 桥接 |
