# FlowEngine Skills — 入口路由

> 本目录是 **AI 工作流 skill**（Claude Code 自动加载，可直接 `/skill-name` 调用）。
> 收到开发任务后，先查下表选择对应 skill。

## Skill 索引

| Skill | 文件 | 何时使用 |
|-------|------|----------|
| **workflow** | `workflow.md` | 任何开发任务的完整流程：设计→执行→测试→迭代→清理→文档 |
| **verify-e2e** | `verify-e2e.md` | 改动 pipeline 链路节点或调参后，跑 demo_evaluator 端到端回归验证 |
| **verification** | `verification.md` | 理解/使用本项目的验证与可观测性架构：分层验证阶梯、debug topic→monitor→topology 数据流、liveness/require 门禁、trace_incident、flowctl 热调参 |
| **debugging** | `debugging.md` | 行为异常排查（转向灯反/该停不停/该走不走/刹停到 0/改代码现象不变）：分层探针 + 值传播验证 + 状态锁死 + 缓存层检查；先看 verification.md 理解有哪些现成观测手段 |
| **py-sim-first** | `py-sim-first.md` | 算法模块升级的 Python 仿真先行流程。任何控制/规划/感知/行为决策类改动，**必须先 Python 仿真验证再移植到 C++**。包含 6 场景（curve/emergency/stop_go/obstacle/merge/cutin）+ 参数扫描 + 掉头仿真 + 移植 checklist |

## 改动后必跑流程

```
写代码 → /verify → /code-review → /simplify → commit → 更新文档
```

> 编码规范（cJSON / clock_service / 参数解析 / 错误码 / 日志 / 场景格式）见 CLAUDE.md，此处不重复。

## 教程文档

深度教程在 [`docs/tutorials/`](../../docs/tutorials/)（17 篇：OOP in C、插件系统、消息总线、IPC、Bag、Clock、Serializer、State Machine、Discovery、Fusion、Coroutine、Demo Evaluator、E2E Learning Loop、Dead Reckoning、SocketCAN Actuator、FlowSim 场景设计、vis 模块设计）。

| 教程 | 何时使用 |
|------|----------|
| [`docs/tutorials/12_demo_evaluator.md`](../../docs/tutorials/12_demo_evaluator.md) | 改动 pipeline 链路节点后跑回归；含 7 种深层故障模式（EKF 收敛、ref_path 航向腐败、NPC 投影陷阱等） |
| [`docs/tutorials/16_flowsim_scenario_design.md`](../../docs/tutorials/16_flowsim_scenario_design.md) | 编写或修改 `scenarios/*.json`；多 edge + junction 路网设计、NPC 放置规范 |
| [`docs/tutorials/17_vis_module_designer.md`](../../docs/tutorials/17_vis_module_designer.md) | 设计并生成 vis/ 新 View 模块（路灯/护栏/行人/标志等）。用户说"加个 XX 模块"时触发 |
