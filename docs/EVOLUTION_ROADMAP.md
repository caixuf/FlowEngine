# FlowEngine 进化路线图

> 日期：2026-07-04（更新 2026-07-15）
> 当前定位：**仿真驱动的自动驾驶中间件框架 + 可复现实验平台**
> 目标方向：从"功能原型"进化为"可组织、可观察、可测试、可回放、可评估的仿真框架"
>
> **定位说明（2026-07-10 重制）：** FlowEngine 明确**不做实车方向**——不追车规量产、
> 不接真实 ECU/CAN、不追硬实时/功能安全认证。全部能力（感知/融合/规划/控制/学习）
> 都在**仿真内闭环**验证。发展重心为两条主线：软件框架质量（主线 A）+ 仿真能力（主线 B）。
>
> **实现状态更新 (2026-07-10)：** Phase 1-3 已完成，Phase 4-7 大部分完成。
> FlowRegistry、ParamRegistry、flow_launcher、flowctl 主要命令、QoS 系统均已实现。
> 未完成项：跨进程 topic bridge、schema-aware bag 全功能、多进程仿真部署验证。
> 落地细化见 [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)。

---

> ## ⚠️ 现状更新（2026-07-13）
>
> 本路线图（2026-07-04 制定）大部分已落地，请勿再按原始 Phase 顺序推进。
> 已完成：统一命名（StartTool→FlowEngine）、`FlowRegistry`、配置驱动 `flow_launcher`、
> `flowctl`、Topic QoS + per-topic 统计、跨进程 IPC bridge、拓扑可视化（flowmond + flowboard）、
> 8 节点多进程 ADAS 全链路 + 3D dashboard。
>
> **进程/线程启动已统一**：新增 `flow_node_host`，让「多进程（fork+exec）」与
> 「单进程（dlopen 多线程）」复用同一份 NodePlugin `.so`。多进程模式不再依赖已废弃的
> `flow_launcher --multi` 多进程启动。
>
> **下一阶段优先级（取代下方旧 Phase 1–2）**：
> 1. 质量收敛：为传输/可视化桥接栈（transport / stats_bridge / dashboard_bridge）补单元测试
>    （✅ 已完成 `tests/test_bridges.c` → ctest `bridge_tests`：transport LOCAL 收发/统计/退订、
>    stats_bridge 序列化往返、dashboard_bridge 单/多分块重组）；
>    长稳/压力测试纳入 nightly（✅ 已完成，见 CI `stability` 标签 + nightly job）；
>    重新启用 TSAN（配 suppression）（待完成）。
> 2. 性能可信度：基于已有 per-topic P50/P99 统计建立端到端延迟/丢包回归基线。
> 3. 真实性：强化 fusion（watermark / 乱序 / 缺传感器策略）；打通真实数据集回放闭环；扩充场景库。
> 4. 数据资产：schema 版本兼容与 Bag schema。
>
> 下面的原始 Phase 描述保留作历史参考。

> ## ⚠️ 现状更新（2026-07-15）— Phase 4+5 已完成 ✅
>
> 本轮在 2026-07-14 的 Phase 2+3 基础上继续做架构性收敛，**三件一起做 + 不留技术债**：
>
> | 架构矛盾 | 修复方案 | 状态 |
> |---------|---------|------|
> | 消息总线无 Schema 强制校验（plugin 用 strstr+sscanf 裸解析） | 新增 `json_schema.h` 严格 JSON/DSL 校验 API（`json_get_*_strict` / `dsl_get_*_strict` / `dsl_validate`），4 个 plugin 文件全量替换为严格解析；新增 20 个 DSL 单测，48/48 通过 | ✅ 已落地 |
> | flowboard 前端用 `<script>` 直接做 monkey-patching (`window.X = X` × 30+) | 全部迁移到 `window.flowboard` 单一命名空间；`scene3d/scene2d/charts` 模块内部 state 化（`setTopoData` setter），HTML 所有 `onclick` 改走 `flowboard.X()`；`debug3d.html` 用 `setDebugCam` API；Node.js `smoke.mjs` 验证模块加载链 | ✅ 已落地 |
> | `DATA_TIMEOUT` fallback 把 `target` 钉死在 `0.0`，弯道中 `road_center_y` ≠ 0 → 车辆沿直线冲出行车道 | fallback 改为 Stanley 风格横向控制：以 `road_center_y(ego_x)` 为目标、`road_center_heading` 为参考、复用 `lat_kp`/`lat_kd_heading`/一阶低通（与主控制器完全一致），steer 受 `steer_limit_for_speed` 限幅；新增 9 个 Python 单测覆盖直道 / 弯道 / 双向偏移 / 限幅 / 滤波；`scenarios/curve_road.json` 跳一轮，最大车道偏差 1.74m（基线 1.67m） | ✅ 已落地 |
>
> **全量验证（2026-07-15）**
>
> | 套件 | 状态 |
> |------|------|
> | `tests/test_json_schema` (48/48) | ✅ |
> | `tests/test_modules` (50/50) | ✅ |
> | `tests/test_new_modules` (36/36) | ✅ |
> | `tests/test_bridges` (8/8) | ✅ |
> | `tools/tests/test_phase5_curve_fallback.py` (9/9) | ✅ **新增** |
> | `tools/flowboard/js/smoke.mjs` (模块加载链) | ✅ |
> | `python3 tools/demo_evaluator.py --scenario curve_road` (12s 跳) | ✅ |
>
> **下一阶段优先级（2026-07-15 更新）**
> 1. 重新启用 TSAN（配 suppression），覆盖 phase 2+3+4+5 后的并发面（transport / bridge / coroutine）。
> 2. Performance regression baseline: per-topic P50/P99 latency 端到端。
> 3. Schema-aware Bag（topic → msg schema 元信息 + bag info）补全。
> 4. 真实数据集回放闭环（nuScenes mini 端到端 + 影子驾驶对比）。
> 5. 扩充场景库（cutin/pedestrian/overtake/roadwork）+ NOA 高速匝道出入口。
> 6. **全场景 3D 可视化适配** — 13 个场景逐一调试 Three.js 前端渲染（道路几何/障碍物/自车姿态），解决 scene3d.js 中硬编码假设与新场景不匹配的问题，确保所有场景的 3D 桥接链路（文件桥接 + Foxglove WS）都能真实显示。

> ## ⚠️ 现状更新（2026-07-15）— 变道效果收敛 ✅
>
> 针对"变道效果差 / QoS 降级后 segfault / NOA 模式不激活"三类问题做了根因修复，
> 变道场景从 FAIL 转为稳定 PASS，无技术债遗留。
>
> | 问题现象 | 根因 | 修复 | 状态 |
> |---------|------|------|------|
> | QoS 降级后 ~2/5 次运行 Segfault（EXIT=139），sim_world 在 10-17s 提前停止 | `flow_launcher` 的 `run_dlopen_mode()` 中 `dlclose` 在 `message_bus_destroy` **之前**执行；MessageBus dispatch 线程在 bus 销毁前仍在调用节点注册的订阅回调，跳转到已 `dlclose` 卸载的 .so 代码段 → SIGSEGV | 将 `dlclose` 从 cleanup 循环移除，延迟到 `main()` 中 `message_bus_destroy` **之后**执行（dispatch 线程已 join，回调不会再被调用） | ✅ 10/10 运行无崩溃 |
> | `road/geometry` QOS_BLOCK 导致 sim_world 发布线程被阻塞 5s 级联，sim_world 提前停止 | `reliability: "reliable"` 自动升级为 `QOS_BLOCK` | `road/geometry` + `model_ota/active` 降级为 `best_effort`（消除最后两个 reliable topic） | ✅ sim_time 稳定 29.9s |
> | `highway_noa_route` 场景 FAIL：驾驶模式只到 ACC/CP，0 次变道，NOA 未激活 | `pipeline.json` 中 `highway_speed_mps: 20.0` 远高于 `target_speed: 12.0`，车辆永远达不到高速阈值，NP 无法升级，进而 NOA 无法激活 | `highway_speed_mps` 20.0 → 10.0（代码注释明确此值"需低于实际巡航速度"） | ✅ ACC→CP→NP→NOA 全激活，2 次变道 |
> | 变道蛇行振荡（前轮遗留） | `lc_lat_kd` 硬编码 + 滤波器参数不当 | `lc_lat_kd` 配置化（pipeline.json params），STEER_FILTER_NEW=0.8，LC_COMPLETE_THRESH=0.15 | ✅ 前轮已修，本轮保留 |
> | control/cmd staleness 级联阻塞（前轮遗留） | `control/raw_cmd` + `control/cmd` QOS_BLOCK 5s 阻塞 | 两者降级为 `best_effort` + `drop_oldest` | ✅ 前轮已修，本轮保留 |
>
> **全量验证（2026-07-15）**
>
> | 套件 | 结果 |
> |------|------|
> | `ctest -LE "benchmark\|manual\|stability\|integration"` | 9/9 PASS，0 失败 |
> | `highway_overtake` ×3 次复跑 | 3/3 PASS，每次 1 次变道、0 碰撞、avg_speed ≈12.0 m/s |
> | `highway_noa_route` ×3 次复跑 | 3/3 PASS，每次 2 次变道（匹配 2 个 route step）、NOA 激活、route_lane_active=True、0 碰撞 |
> | segfault 稳定性 | 6/6 次运行 0 崩溃（修复前 2/5 崩溃） |
>
> **关键架构要点（写入文档避免重蹈覆辙）**
> - **dlclose 必须晚于 message_bus_destroy**：MessageBus 的 dispatch 线程在 bus 销毁前会持续调用订阅回调，.so 代码段过早卸载会触发 use-after-unmap segfault。
> - **`reliability: "reliable"` 会自动升级为 `QOS_BLOCK`**：阻塞最多 5s，对高频发布节点（sim_world）是致命的级联阻塞源。仿真链路应统一用 `best_effort`。
> - **`highway_speed_mps` 必须 < `target_speed`**：NP 模式升级条件是 `ego_v >= highway_speed_mps` 持续 3s，阈值高于巡航速度会导致 NP 永远无法激活。

---

## 1. 总体方向

FlowEngine 现在已经有很多核心零件：任务系统、插件、消息总线、IPC、Bag、Clock、协程、状态机、Discovery、Fusion、Serializer。

下一步最重要的不是继续盲目增加模块，而是把已有模块组织成一个统一系统。

核心目标可以概括为：

> 让 FlowEngine 变成一个能启动、能发现、能观察、能调试、能回放的中间件框架。

因此，进化重点是：

- 工程健康度
- 统一命名
- 统一元信息/反射系统
- Launch 系统
- `flowctl` 命令行工具
- 通信 QoS 与 topic 统计
- Schema-aware Bag
- 拓扑可视化
- 参数系统
- 多进程真实验证

## 2. 推荐路线总览

| 阶段 | 目标 | 关键词 | 状态 |
|---|---|---|---|
| Phase 1 | 工程收敛 | 测试、脚本、命名、README | ✅ 已完成 |
| Phase 2 | 统一元信息 | `FlowRegistry`、反射、Meta | ✅ 已完成 |
| Phase 3 | Launch 系统 | 配置驱动启动、依赖、参数 | ✅ 已完成 |
| Phase 4 | 可观测性 | `flowctl`、状态、topic、拓扑 + JSON/DSL 严格校验 + flowboard 单一命名空间 | ✅ 已完成 |
| Phase 5 | 通信增强 | QoS、latency、drop policy、IPC bridge + DATA_TIMEOUT 弯道跟随 | ✅ 已完成 |
| Phase 6 | 数据闭环 | schema-aware bag、bag info、replay | 🔧 部分完成 |
| Phase 7 | 真实 ADAS 样例 | perception、fusion、control、monitor | ✅ 已完成 |

## 3. Phase 1：工程收敛

### 3.1 目标

让项目随时可以稳定执行：

- 一键构建
- 一键测试
- 一键运行 demo
- 一键清理
- 一键查看项目状态

### 3.2 需要解决的问题 ✅ 已解决

~~当前 `flow_task` 是交互式 demo，不适合直接作为 `ctest` 自动化测试。~~ 已移除 `flow_task_test` 和 `flow_adas_test`，ctest 全量自动通过。

### 3.3 建议任务

1. ✅ 修复 `ctest` 卡住问题 — 已移除交互式测试。
2. ✅ 给所有 `add_test()` 设置 `TIMEOUT` — 已完成。
3. ✅ 整理 scripts 目录 — 已清理过期脚本。
4. ✅ 统一构建、测试、demo 入口 — `demo.sh` 已统一。
5. 给 `flow_launcher` 增加 `--smoke` 模式纳入 CI（待完成）。

推荐 demo 参数：

```text
flow_launcher config/pipeline.json --duration 15
flow_bus --test
flow_coro --test
```

### 3.4 完成标准

- `cmake --build build -j$(nproc)` 稳定通过。
- `ctest --test-dir build --output-on-failure` 稳定通过。
- 所有 demo 测试都能自动退出。
- README 中的运行命令和实际工程一致。

## 4. Phase 2：统一命名和项目身份

### 4.1 目标

消除 StartTool 历史命名，让项目身份统一为 FlowEngine。

### 4.2 建议改名

| 当前命名 | 建议命名 |
|---|---|
| `starttool_core` | `flowengine_core` |
| `include/starttool` | `include/flowengine` |
| `lib/starttool/plugins` | `lib/flowengine/plugins` |
| 文档中的 StartTool | FlowEngine |

### 4.3 完成标准

- CMake target 命名统一。
- 安装路径统一。
- 文档统一。
- 包名、CPack 信息统一。

## 5. Phase 3：统一元信息/反射系统

### 5.1 为什么要做

当前项目已经有很多分散的元信息：

- `msg_schema`
- `serializer`
- `state_machine`
- `discovery`
- `task_manager`
- 插件 `create_task`

这些都属于“反射能力”的不同碎片。

下一步应该把它们收敛成统一注册中心。

### 5.2 推荐设计

```text
FlowRegistry
├── TaskRegistry
├── TopicRegistry
├── TypeRegistry
├── PluginRegistry
├── ParamRegistry
└── StateMachineRegistry
```

### 5.3 运行时应该能查询什么

框架应该能回答：

```text
现在有哪些 task？
有哪些 topic？
topic 对应什么类型？
这个 task 发布什么？
这个 task 订阅什么？
这个 task 有哪些参数？
这个 task 当前状态是什么？
这个插件提供了哪些能力？
```

### 5.4 推荐 Meta 类型

```text
TaskMeta
  name
  category
  plugin
  inputs
  outputs
  params
  lifecycle

TopicMeta
  name
  type_name
  type_id
  schema_version
  qos
  publisher_count
  subscriber_count

TypeMeta
  type_name
  type_id
  struct_size
  schema_hash
  fields
  serializer
  deserializer

ParamMeta
  name
  type
  default_value
  current_value
  min
  max
  description
  hot_reload

PluginMeta
  name
  path
  version
  provided_tasks
  provided_types
  capabilities
```

### 5.5 完成标准

- task、topic、type、plugin、param 可以统一注册。
- 可以导出 registry JSON。
- `launcher`、`flowctl`、topology viewer 都能使用同一套元信息。

## 6. Phase 4：Launch 系统

### 6.1 目标

让 FlowEngine 从“写代码启动系统”进化成“配置驱动启动系统”。

### 6.2 推荐配置格式

```json
{
  "nodes": [
    {
      "name": "perception_node",
      "plugin": "libperception_task.so",
      "params": {
        "rate_hz": 20
      },
      "publish": ["sensor/lidar", "sensor/obstacle"],
      "subscribe": [],
      "depends": []
    },
    {
      "name": "control_node",
      "plugin": "libcontrol_task.so",
      "params": {
        "max_speed": 30.0
      },
      "publish": ["control/cmd"],
      "subscribe": ["sensor/obstacle"],
      "depends": ["perception_node"]
    }
  ]
}
```

### 6.3 Launcher 职责

`launcher` 应该负责：

- 读取配置
- 加载插件
- 注册 task
- 注册 topic
- 注册参数
- 检查依赖
- 启动节点
- 监控状态
- 优雅退出

### 6.4 完成标准

- 一个 launch 文件可以启动完整 ADAS demo。
- 依赖检查失败时给出清晰错误。
- 启动顺序可解释。
- 退出时能按依赖反向停止。

## 7. Phase 5：`flowctl` 命令行工具

### 7.1 目标

让框架具备可操作、可观察、可调试的命令行入口。

没有 `flowctl`，框架内部再强，用户也很难看见。

### 7.2 推荐命令

```text
flowctl list tasks
flowctl list topics
flowctl list plugins
flowctl graph
flowctl state control_node
flowctl topic stats sensor/lidar
flowctl bag info demo.bag
flowctl schema ChassisState
flowctl param list
flowctl param get control_node.max_speed
flowctl param set control_node.max_speed 20
```

### 7.3 数据来源

`flowctl` 应该复用：

- `FlowRegistry`
- `DiscoveryManager`
- `TaskManager`
- `MessageBus` stats
- `StateMachine`
- `Bag`
- `Serializer`

### 7.4 完成标准

- 能查询 task/topic/plugin/schema。
- 能导出拓扑图。
- 能查看参数。
- 能查看 Bag 信息。

## 8. Phase 6：强化消息系统

### 8.1 目标

让 `MessageBus` 从可用 demo 进化为更像中间件通信层。

### 8.2 优先补强能力

1. Topic 级统计。
2. 队列策略。
3. 延迟统计。
4. QoS。
5. Topic introspection。
6. 跨进程 topic bridge。

### 8.3 Topic 统计建议

```text
topic name
publisher count
subscriber count
publish count
deliver count
drop count
avg latency
p50 latency
p99 latency
last timestamp
frequency_hz
```

### 8.4 队列策略建议

```text
drop_oldest
drop_latest
block
overwrite
```

### 8.5 完成标准

- `flowctl list topics` 能看到所有 topic。
- `flowctl topic stats <topic>` 能看到频率、延迟、丢包。
- 每个 topic 可以配置队列深度和溢出策略。

## 9. Phase 7：强化 IDL / Schema / Bag

### 9.1 目标

让消息变成“可描述、可回放、可兼容”的数据资产。

### 9.2 需要补强

- 字段级 schema
- schema hash
- schema version
- Bag 内保存 schema
- Bag info 工具
- Bag check 工具
- Bag play remap

### 9.3 目标效果

执行：

```text
flowctl bag info demo.bag
```

输出类似：

```text
topics:
  sensor/lidar
    type: LidarFrame
    schema_version: 1
    count: 10000
    frequency: 10Hz

  vehicle/chassis
    type: ChassisState
    schema_version: 2
    count: 10000
    frequency: 100Hz
```

### 9.4 完成标准

- Bag 文件中包含 topic、type、schema、timestamp。
- 可以查看 Bag 内容摘要。
- 可以按 topic 回放。
- 可以做 topic remap。

## 10. Phase 8：拓扑可视化

### 10.1 目标

把 discovery、registry、topic stats 显示出来，让系统结构可视化。

### 10.2 推荐展示内容

- 节点图
- topic 连线
- 发布频率
- 消息延迟
- 节点状态
- 心跳状态
- 错误状态
- topic 类型

### 10.3 目标效果

```text
perception_node ── sensor/lidar ──> fusion_node ── fusion/objects ──> control_node
```

每条边显示：

```text
10Hz
p99 latency 3.2ms
drop 0
```

### 10.4 完成标准

- topology viewer 能读取 JSON。
- 能展示节点、topic、方向和状态。
- 能显示频率、延迟、丢包。

## 11. Phase 9：参数系统

### 11.1 目标

让任务参数可以被描述、校验、查询、修改。

### 11.2 推荐设计

```text
ParamRegistry
  name
  type
  default_value
  current_value
  min
  max
  description
  readonly
  hot_reload
```

### 11.3 示例参数

```text
control_node.max_speed
control_node.enable_emergency_brake
perception_node.lidar_rate_hz
fusion.max_timestamp_delta_ms
```

### 11.4 推荐命令

```text
flowctl param list
flowctl param get control_node.max_speed
flowctl param set control_node.max_speed 20
```

### 11.5 完成标准

- 参数可以注册。
- 参数可以校验类型和范围。
- 参数可以查询。
- 支持部分参数热更新。

## 12. Phase 10：多进程仿真部署验证

### 12.1 目标

验证 FlowEngine 是否真的具备框架能力，而不是只在单进程 demo 中工作。
**注意：这里的"部署"指多进程仿真部署，不涉及任何实车 / ECU / CAN。**

### 12.2 推荐 demo

```text
process 1: perception_node
process 2: localization_node
process 3: fusion_node
process 4: control_node
process 5: monitor_node
```

它们通过：

```text
discovery + ipc_channel + message_bus bridge
```

连接起来。

### 12.3 推荐入口

```text
bash scripts/demo.sh
```

然后可以执行：

```text
flowctl graph
flowctl list topics
flowctl state
```

### 12.4 完成标准

- 多进程可以自动发现。
- topic 可以跨进程传递。
- monitor 可以看到拓扑。
- Bag 可以录制全链路数据。
- shutdown 可以优雅退出。

## 13. 优先级建议

### 近期最该做

1. 修 `ctest`。
2. 统一命名。
3. 更新 README。
4. 做 `FlowRegistry` 基础版。
5. 做 `flowctl list tasks/topics`。

### 中期最该做

1. Launch 配置驱动启动。
2. Topic stats。
3. 参数系统。
4. Bag info。
5. Topology viewer 接入真实数据。

### 长期最该做

1. QoS。
2. IPC bridge。
3. 多进程 ADAS demo。
4. ASAN/TSAN 长测。
5. Schema 兼容与版本管理。

## 14. 最终判断

FlowEngine 当前已经有很多零件，接下来应该进入“系统化整理期”。

最关键的四个进化点是：

```text
统一注册中心 + Launch + flowctl + Topology Viewer
```

这四件事完成后，FlowEngine 的质感会明显上一个台阶：

- 从功能集合变成框架。
- 从内部能跑变成外部可观察。
- 从手写启动变成配置启动。
- 从 demo 变成可持续演进的中间件。

最终目标不是“再加一个模块”，而是：

> 把已有模块统一成一个真正能启动、能发现、能观察、能调试、能回放的系统。
