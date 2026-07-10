# FlowEngine 落地实施指南（Implementation Guide）

> 配套文档：[EVOLUTION_ROADMAP.md](EVOLUTION_ROADMAP.md)（方向）· [PROJECT_REVIEW.md](PROJECT_REVIEW.md)（现状评估）
>
> **本文用途：** 把「发展计划（软件 + 仿真双主线）」拆成一个个**接口已定义、契约已写清**的小任务，
> 让后续实现者（包括能力较弱的模型）**只需补齐实现**即可推进，无需再做架构决策。
>
> **定位红线：** FlowEngine 不做实车。所有"部署 / 验证"均指**仿真内**（多进程仿真属于软件目标）。
> 不要引入车规 / ECU / CAN / 硬实时 / 功能安全相关代码。

## 怎么用这份指南

每个任务卡片都遵循同一结构：

- **目标** —— 做完后系统多了什么能力。
- **现状** —— 已有哪些零件可复用。
- **接口/契约** —— 已经写好的头文件或函数签名（你**只填实现**）。
- **待实现** —— 具体要写的文件 / 函数 / stub。
- **验收** —— 用哪条命令 / 测试证明做完了。

优先级顺序见文末「排期」。**先做 P0，再做 P1**；每做完一张卡片跑一次对应验收命令。

---

## 主线 A：软件框架质量

### A1. 工程健康度收尾

**目标：** CI 能自动跑 e2e 冒烟 + 长稳（ASAN/TSAN）轨道，单测覆盖 QoS/param/registry。

**现状：** `ctest` 全绿；`build-tsan/` 已存在；`flow_e2e` 支持带时长运行；`test_modules` 已覆盖 serializer/statem/scheduler/fusion。

**接口/契约：**
- `flow_e2e` 已可接收运行时长参数（见 `src/e2e_demo.c`）。需新增 `--smoke` 短时自检模式（几秒内退出、返回非零即失败）。
- 单测框架沿用 `test/`、`tests/` 现有风格（见 `tests/` 里的 `new_module_tests`）。

**待实现：**
1. `src/e2e_demo.c`：解析 `--smoke`，跑 ~3s 最小链路后自检关键 topic 频率>0 并退出（返回非零表示失败）。
2. `CMakeLists.txt`：新增 `add_test(NAME e2e_smoke_cli COMMAND flow_e2e --smoke)` 并设 `TIMEOUT 30`。
3. `tests/`：给 `message_bus` QoS（depth + drop policy）、`param_registry` 范围校验、`flow_registry_export_json()` 各加 1~2 个断言用例。
4. CI：新增可选 job 调用 `build-tsan` 跑 1 小时压力（可用现有 `benchmark` / `flow_bus --test` 循环）。

**验收：** `ctest --test-dir build --output-on-failure` 全绿，且包含 `e2e_smoke_cli`。

---

### A2. 统一元信息与内省收口

**目标：** flowctl / dashboard / 评估器**只从一个数据源**取元信息：`flow_registry_export_json()`。

**现状：** `include/flow_registry.h` 已定义 Task/Topic/Type/Plugin/Schema/Param 全部 Meta 与
`flow_registry_export_json()`；`flowctl` 已有 `list/graph/schema/param/bag info`（见 `src/flowctl.c`）。

**接口/契约：** 无需新增接口 —— `flow_registry.h` 已是唯一契约。工作是**收敛调用点**。

**待实现：**
1. 审计 `src/flowctl.c`：确保每个 `list_*` 子命令都走 `flow_registry_list_*()` 而非各模块私有表。
2. 让 monitor / dashboard JSON 也复用 `flow_registry_export_json()`（对比 `src/core/monitor_server.c` 当前拼 JSON 的方式）。
3. 补 `flowctl list types` 与 `flowctl topic stats <topic>`（若缺）——数据源同上。

**验收：** `flowctl list tasks|topics|types|plugins|params` 全部有输出，且与 dashboard 显示一致。

---

### A3. 跨进程 Topic 桥接（多进程仿真部署）

**目标：** 把单进程 demo 拆成多进程仍能互通，验证"框架"而非"单进程 demo"。

**现状：** `include/transport.h` 已有自动路由（local/IPC/TCP）；`include/ipc_channel.h` 有 SHM 通道。

**接口/契约：** ✅ 已写好 —— [`include/topic_bridge.h`](../include/topic_bridge.h)。
定义了 `topic_bridge_create / add_topic / start / stop / destroy / get_stats`，方向 PUB/SUB/BIDIR。

**待实现：**
1. 新建 `src/core/topic_bridge.c` 实现 `topic_bridge.h` 全部函数：
   - PUB 侧：`message_bus_subscribe` 本地 topic → `serializer_serialize` → `ipc_channel` 写。
   - SUB 侧：`ipc_channel` 读 → `serializer_deserialize` → `message_bus_publish` 本地。
   - 维护 `TopicBridgeStats`（forwarded/dropped/bytes/last_ts_us）。
2. `CMakeLists.txt`：把 `topic_bridge.c` 加入 `flowengine_core` 源列表。
3. 新增 `examples/` 或 `tests/` 双进程 demo：进程 A 发 `sensor/lidar`，进程 B 收到并计数。

**验收：** 双进程 demo 中 SUB 侧收到 PUB 侧的消息数 > 0；`topic_bridge_get_stats` 的 `forwarded` 递增。

---

### A4. 数据资产化（Schema + Bag）

**目标：** 每次仿真可录制成可回放、可比对的数据资产。

**现状（大部分已完成）：** `include/bag.h` 已支持写入 `type_id|schema_ver|endian`，
`bag_reader_info / get_topics / get_type_info / play_filtered`；`flowctl bag info` 已实现（`src/flowctl.c`）。

**接口/契约：** 见 `include/bag.h`。剩余是**补齐字段级 schema 与 remap**。

**待实现：**
1. `bag.h` + `src/core/bag.c`：在 bag 头写入 field-level schema 摘要（复用 `msg_schema.h` 的 `struct_size` 与
   `serializer.h` 的 `schema_hash`），并让 `bag_reader_get_type_info` 回填 schema hash/version。
2. Topic remap：给 `bag_reader_play_filtered` 增加"源 topic → 目标 topic"映射参数，或新增
   `bag_reader_play_remap(r, bus, speed, const char* from[], const char* to[], int n)`。
3. `flowctl bag info` 输出补 `frequency`（用 count / duration 估算）。

**验收：** `flowctl bag info demo.bag` 输出每 topic 的 type / schema_version / count / frequency；
remap 回放后目标 topic 在 bus 上出现。

---

## 主线 B：仿真能力

### B1. 仿真世界保真度

**目标：** 更真实的车辆动力学 + 传感器模型 + 多 NPC 交互行为。

**现状：** `modules/adas_nodes/sim_world_node.c`（车辆/ NPC 更新）、`sensor_model_node.c`（FOV/遮挡/噪声）、
`include/scenario_loader.h`（actor/ego JSON 契约，已支持 car/pedestrian/truck）。

**接口/契约：** `scenario_loader.h` 的 `ScenarioActor/ScenarioEgo/ScenarioCriteria` 已是稳定契约。
**新增场景无需改代码**，只加 JSON（见 `scenarios/*.json`）。改保真度才需动 C。

**待实现（按需，从易到难）：**
1. **纯数据（最简单，弱模型友好）**：继续往 `scenarios/` 加场景 JSON，并登记进 `scenarios/suite.json`。
2. 传感器模型：在 `sensor_model_node.c` 内细化噪声/遮挡（如按距离衰减、雨雾降 SNR）——保持既有 topic 契约不变。
3. NPC 行为：在 `sim_world_node.c` 给 actor 增加简单跟车/让行策略（读 `ScenarioActor` 扩展字段）。
   如需新字段，先在 `scenario_loader.h` 的 struct 里加，再在 `scenario_load()` 解析。

**验收：** 新场景能被 `flow_launcher` 加载并跑完；`demo_evaluator.py --scenario <新场景>` 有评分输出。

---

### B2. 仿真即测试（回归评估体系）✅ 框架已就绪

**目标：** 一条命令跑通「多场景批量仿真 → 自动评分 → 回归对比报告」。

**现状（本次已交付）：**
- `tools/demo_evaluator.py`：新增 `--scenario <path>`（临时覆盖 `sim_world.scenario_file`）与
  `--json-out <path>`（机器可读结果）。
- `tools/scenario_regression.py`：读取 `scenarios/suite.json`，逐场景跑分、聚合矩阵、与 baseline 对比。
- `scenarios/suite.json`：场景清单 + `baseline_tolerances`（数值回归阈值）。

**接口/契约（扩展点，弱模型只碰这两处）：**
- 加场景：编辑 `scenarios/suite.json` 的 `scenarios[]`（无需改代码）。
- 调回归阈值：编辑 `scenarios/suite.json` 的 `baseline_tolerances`（`min_ratio` / `max_abs_increase`）。
- 改评分口径：`demo_evaluator.py::score()`；改回归判定：`scenario_regression.py::compare_summary()`。

**待实现（收尾）：**
1. 首次录制基线：`python3 tools/scenario_regression.py --update-baseline`（产物落 `scenarios/baseline/`）。
2. 把 `python3 tools/scenario_regression.py --baseline` 接入 CI（可选、耗时任务）。

**验收：**
- `python3 tools/scenario_regression.py --dry-run` 列出全部场景。
- `python3 tools/scenario_regression.py` 产出 Regression Matrix 报告；全 PASS 时退出码 0。

---

### B3. 三层仿真体系打通

**目标：** Layer 1（Bag 回放）/ Layer 2（2D 模拟器）/ Layer 3（内置 3D 场景）共用同一数据契约，
同一场景可在三层间切换。

**现状：** 数据契约见 [FLOWBOARD_CONTRACT.md](FLOWBOARD_CONTRACT.md)（`/tmp/flow_topology.json` + `scene`）；
可视化见 `tools/flowboard.html`、`tools/foxglove_bridge.py`；仿真设计见 [E2E_SIMULATION_DESIGN.md](E2E_SIMULATION_DESIGN.md)。

**接口/契约：** `/tmp/flow_topology.json` 的 `scene` 字段是唯一契约。三层都读/写它。

**待实现：**
1. 确认 Bag 回放路径也能产出等价 `scene`（复用 `bag_reader_play` + monitor 写 JSON）。
2. 文档化"同一 `scenarios/*.json` 如何分别喂给三层"，补到 [SIMULATION_GUIDE.md](SIMULATION_GUIDE.md)。

**验收：** 同一场景在 2D 与 3D 面板显示一致的 ego / actor 位置。

---

### B4. 车端学习闭环（在仿真中闭合）

**目标：** 采集 → 离线训练 → 影子推理 → 评估，全在仿真内闭环，**影子模式绝不接管控制**。

**现状：** `modules/adas_nodes/data_recorder_node.c`（Stage 0 采样）、`tools/train/train.py`（离线训练）、
`modules/adas_nodes/inference_node.c` + `tiny_mlp.h`（Stage 2 影子推理）；架构见 [LEARNING_LOOP.md](LEARNING_LOOP.md)。

**接口/契约：** `tiny_mlp.h` 的推理内核与 `train.py` 导出的权重格式是两端契约（保持一致）。

**待实现：**
1. 跑通链路脚本：录样本 → `train.py` 产出权重 → `inference_node` 加载 → 与规则控制器输出对比。
2. 在 `demo_evaluator.py` 的 `summary` 里增加"影子推理 vs 规则控制"偏差指标（只读、不干预控制）。
3. 用 `scenario_regression.py` 跨场景评估影子模型，纳入报告。

**验收：** 一条脚本从采集跑到"影子偏差指标"入回归报告；控制链路输出不受影子推理影响。

---

## 排期（去实车后的新顺序）

| 优先级 | 卡片 | 一句话 |
|---|---|---|
| P0 | A1 | e2e `--smoke` 入 CI + 单测补覆盖 |
| P0 | B2（收尾） | 录基线 + 回归入 CI（框架已就绪） |
| P1 | A2 | 内省收口到 `flow_registry_export_json()` 单一数据源 |
| P1 | B1（纯数据） | 加场景 JSON + 登记 suite |
| P2 | A4 | schema-aware bag remap / frequency |
| P2 | B1（保真度） | 传感器/动力学/NPC 行为细化 |
| P3 | A3 | 实现 `topic_bridge.c` + 双进程仿真 demo |
| P3 | B3 | 三层仿真数据契约打通 |
| P3 | B4 | 学习闭环全链路 + 影子偏差入报告 |

## 总体验收（计划级）

- [ ] 一条命令跑通「多场景批量仿真 → 自动评分 → 回归对比报告」——`tools/scenario_regression.py`（框架已就绪，待录基线）。
- [ ] 任一算法改动都能在纯仿真下验证，无任何实车依赖。
- [ ] task/topic/type/param/schema/bag/state 均可经 `flowctl` 内省。
- [ ] 文档与代码定位一致，对外统一表述为「仿真驱动的自动驾驶中间件框架」。
