# FlowEngine 进化路线图

> 日期：2026-07-04  
> 当前定位：自动驾驶/机器人中间件内核原型  
> 目标方向：从“功能原型”进化为“可组织、可观察、可测试、可部署的框架”

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

| 阶段 | 目标 | 关键词 |
|---|---|---|
| Phase 1 | 工程收敛 | 测试、脚本、命名、README |
| Phase 2 | 统一元信息 | `FlowRegistry`、反射、Meta |
| Phase 3 | Launch 系统 | 配置驱动启动、依赖、参数 |
| Phase 4 | 可观测性 | `flowctl`、状态、topic、拓扑 |
| Phase 5 | 通信增强 | QoS、latency、drop policy、IPC bridge |
| Phase 6 | 数据闭环 | schema-aware bag、bag info、replay |
| Phase 7 | 真实 ADAS 样例 | perception、fusion、control、monitor |

## 3. Phase 1：工程收敛

### 3.1 目标

让项目随时可以稳定执行：

- 一键构建
- 一键测试
- 一键运行 demo
- 一键清理
- 一键查看项目状态

### 3.2 需要解决的问题

当前 `flow_task` 是交互式 demo，不适合直接作为 `ctest` 自动化测试。它会导致测试卡住。

### 3.3 建议任务

1. 修复 `ctest` 卡住问题。
2. 给 demo 增加测试模式。
3. 给所有 `add_test()` 设置 `TIMEOUT`。
4. 整理 scripts 目录。
5. 统一构建、测试、demo 入口。

推荐脚本：

```text
scripts/build.sh
scripts/test.sh
scripts/demo.sh
scripts/clean.sh
scripts/check.sh
```

推荐 demo 参数：

```text
flow_task --test --duration 3
flow_bus --test
flow_adas --test
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
      "name": "fake_perception",
      "plugin": "libfake_perception_task.so",
      "params": {
        "rate_hz": 20
      },
      "publish": ["sensor/lidar", "sensor/obstacle"],
      "subscribe": [],
      "depends": []
    },
    {
      "name": "fake_control",
      "plugin": "libfake_control_task.so",
      "params": {
        "max_speed": 30.0
      },
      "publish": ["control/cmd"],
      "subscribe": ["sensor/obstacle"],
      "depends": ["fake_perception"]
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
flowctl state fake_control
flowctl topic stats sensor/lidar
flowctl bag info demo.bag
flowctl schema ChassisState
flowctl param list
flowctl param get fake_control.max_speed
flowctl param set fake_control.max_speed 20
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
fake_perception ── sensor/lidar ──> fusion_node ── fusion/objects ──> fake_control
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
fake_control.max_speed
fake_control.enable_emergency_brake
fake_perception.lidar_rate_hz
fusion.max_timestamp_delta_ms
```

### 11.4 推荐命令

```text
flowctl param list
flowctl param get fake_control.max_speed
flowctl param set fake_control.max_speed 20
```

### 11.5 完成标准

- 参数可以注册。
- 参数可以校验类型和范围。
- 参数可以查询。
- 支持部分参数热更新。

## 12. Phase 10：真实多进程验证

### 12.1 目标

验证 FlowEngine 是否真的具备框架能力，而不是只在单进程 demo 中工作。

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
bash scripts/fullstack_demo.sh
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
