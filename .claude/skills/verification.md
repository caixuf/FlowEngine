---
name: verification
description: FlowEngine 验证与可观测性架构：分层验证阶梯（pipeline_check→demo_evaluator→scenario_regression→param_regression→quick_verify/auto_tune）、debug topic→monitor→topology 数据流、liveness/require 门禁、trace_incident 事故追溯、flowctl 热调参。
---

# Verification — FlowEngine 验证与可观测性架构

本项目的核心架构思想（上百个 commit 沉淀）：**每个节点都可观测，验证全部
走 Python 黑盒评分**。改算法、调参、排障都建立在这套机制上，而不是肉眼看
3D。

## 核心思想

1. **可观测性优先**：每个算法节点把中间变量以低频率 JSON topic 发出去
   （`planning/debug`、`control/debug`、`behavior/state`、`control/cte`、
   `control/ldw`），monitor_node 聚合写进唯一公共数据源 `/tmp/flow_topology.json`
   ——仪表盘、evaluator、trace_incident、flowctl **全部消费同一份数据**。
2. **黑盒评分**：验证工具不碰 C 内部函数，只采样公共 JSON + 日志，把 20+
   维度变成数值判据（碰撞计数、TTC、CTE、频率、丢帧、NPC 瞬移……）。
   肉眼看仪表盘只能发现"撞了/停了"；黑盒评分连偶发/慢漂移都抓。
3. **分层验证**：从秒级到分钟级分层，各司其职（见下）。
4. **门禁有效性自检**：门禁必须能抓住已知故障，抓不住 = 它的 PASS 不可信。

## 分层验证阶梯（改完代码按此走）

| 层 | 工具 | 耗时 | 阶段 | 抓什么 |
|---|---|---|---|---|
| L0 | `python3 tools/pipeline_check.py` | ~1s 不起 demo | 改完**立刻**跑 | 管道断没断：拓扑边、topic 频率、数据完整性、感知识别率、registry |
| L1 | `python3 ci/evaluators/demo_evaluator.py --duration 45` | 45s 真跑 | 改 pipeline 节点后、CI 门禁 | 行为退化：碰撞/停滞/偏航/红灯违章/变道/TOPIC 频率/拓扑 |
| L1.5 | `python3 ci/evaluators/scenario_regression.py --baseline` | N×45s | CI 场景矩阵 | 全场景 PASS + 数值 vs baseline（改前先 `--update-baseline`） |
| L2 | `python3 ci/evaluators/test_param_regression.py` | 20s+ | 调参前后 A/B | CTE/yaw/steer RMS、flip_rate、avg_speed 退化%（≥15 WARN ≥30 FAIL） |
| L3 | `python3 tools/quick_verify.py` | 常驻 demo | 人工交互探索 | `set/get/eval/reset/baseline` 热调参 + 即时评分 |
| L3 | `python3 tools/auto_tune_mpc.py` | 分钟级 | 自动扫参/自标定 | reward 评分 + 固化最优（Stanley 热应用 / MPC OTA） |

**改前存基线，改后对比**：`test_param_regression --save-baseline`、
`scenario_regression --update-baseline`，改完跑对比，量化"是改善还是退化"。

## 整链路调试数据流

```
每个算法节点 ──transport_publish(JSON text, 2~20Hz)──► monitor_node 订阅缓存
  planning/debug(2Hz)  control/debug(2Hz)  behavior/state(2Hz)
  control/cte(20Hz)    control/ldw(事件)
monitor 每 20Hz 聚合 ──写──► /tmp/flow_topology.json（原子 rename）
  │ nodes[].topics[].role    metrics.scene{ego,lane,entities,obstacles}
  │ metrics.topics[].freq    metrics.behavior / control_debug / planning_debug
  │ samples[]（200 帧 10s 环形缓冲 = 离线时序分析的唯一素材）
  ├─► flowmond :8800（HTTP/SSE 仪表盘 + IPC bridge 跨进程）
  └─► trace_incident / pipeline_check / demo_evaluator / flowctl
```

**关键观测点**（某层出错去哪看）：
- 感知：`metrics.scene.obstacles`（真值）vs `metrics.behavior.obs_count`（感知）
- 行为：`behavior/state` JSON（state/best_gap/blocked/变道评估）+ `[BEH]`/`[SM]` 日志
- 规划：`planning/debug`（ego_d/target_lane_offset/traj_valid/command_speed）
- 控制：`control/debug`（steer/lat_error/target_y/mode）+ `control/cte`(20Hz)
- 管道断/降频：`metrics.topics[].freq`；`flowctl topic stats <topic>`
- 事故复盘：`python3 tools/trace_incident.py`（逐层 dump + 碰撞前 5s 时间线）

## 事故追溯 trace_incident

```
python3 tools/trace_incident.py                # 最近一次碰撞/出路沿
python3 tools/trace_incident.py --at 17.6      # 指定时间戳
```
自动识别事故 → 逐层 dump（感知/行为/规划/控制 + 横向链路一致性检查）→
规则引擎给结论（感知盲开 / 行为没进 FOLLOW / 规划不减速 / 横向链路断裂…）。

## 门禁有效性（防"门禁空跑"）——这是本架构最深的洞察

历史上门禁被静默绕过至少六次（分母 0 满分、被检查量恒 0 上界永满足、
判据写进 warnings、场景不含该情形、指标与事实错配）。根因是**门禁不知道
自己有没有真的测到东西**。防御：

- **liveness gate**：`speed/x/y/heading/steer` 整 run 恒为单值 → `DEAD SIGNAL`
  FAIL。恒为初值 = 上游链路断了，依赖它的所有判据都是空转。
- **require()**：判据显式声明前置条件，不满足记 `INCONCLUSIVE` 并**计入
  failures**——"无法判定 ≠ 通过"，绝不 `continue` 掉。
- **scenario actor 计数**：场景里有行人却没有 vru 样本 → 感知漏整个类别，
  FAIL（不因"该层没数据"而跳过）。
- **指标交叉一致性**：碰撞却 critical_event_count=0 → METRIC MISMATCH FAIL。
- **门禁自测**：`ci/evaluators/test_evaluator_gate.py` 注入 12 种历史故障，
  断言评估器必须 FAIL——门禁先证伪自己，再判别人。

## 热调参（快速迭代的核心）

```
flowctl param list                      # 运行中进程的实时参数
flowctl param get control.mpc_r_ddelta
flowctl param set control.mpc_r_ddelta 2.0   # AF_UNIX socket，下一帧生效
```
**禁止**为试一个值去改常量重编译。整段 run 的聚合指标 A/B 用
`auto_tune_mpc.py` / `test_param_regression.py`（要可比就得从 x=0 起跑干净 run）。

## 快速上手

```bash
# 改完代码 → 秒级确认管道没断
python3 tools/pipeline_check.py
# → 45s 行为回归
python3 ci/evaluators/demo_evaluator.py --duration 45 --interval 0.5
# → 事故复盘（若发生）
python3 tools/trace_incident.py
# 调参：quick_verify 交互 → param_regression 确认不退化
```

## 参考

- 数据源：`modules/adas_nodes/monitor_node.c`（`export_dashboard_json` ~563-1120）
- 仪表盘：`src/flowmond.c` + `src/core/monitor_server.c`（HTTP/SSE）
- 各层 debug topic：`planning_node.cpp:1350`、`control_node.cpp:864/851/895`、
  `behavior_planner_node.cpp:943`
- 验证工具：`tools/pipeline_check.py`、`ci/evaluators/demo_evaluator.py`、
  `ci/evaluators/scenario_regression.py`、`ci/evaluators/test_param_regression.py`、
  `tools/quick_verify.py`、`tools/auto_tune_mpc.py`、`tools/trace_incident.py`
- 门禁自测：`ci/evaluators/test_evaluator_gate.py`
- 教程：`docs/tutorials/12_demo_evaluator.md`（评估器设计 + 4 个深层故障模式）
