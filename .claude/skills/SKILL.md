# FlowEngine Skills — 入口路由

> AI 使用本文件发现项目所有 skill。收到开发任务后，先查本表选择对应 skill。

## Skill 索引

| Skill | 文件 | 何时使用 |
|-------|------|----------|
| **workflow** | `workflow.md` | 任何开发任务的完整流程：设计→执行→测试→迭代→清理→文档 |
| **verify** | `verify.md` | 改动 pipeline 链路节点后，跑 demo_evaluator 端到端验证 |
| **verification** | `verification.md` | 理解/使用本项目的验证与可观测性架构：分层验证阶梯（pipeline_check→demo_evaluator→scenario_regression→param_regression→quick_verify/auto_tune）、整链路调试数据流（debug topic→monitor→topology JSON→仪表盘/evaluator/trace_incident）、liveness/require 门禁有效性、trace_incident 事故追溯、flowctl 热调参 |
| **debugging** | `debugging.md` | 行为异常排查（转向灯反/该停不停/该走不走/刹停到 0/改代码现象不变）：分层探针 + 值传播验证 + 状态锁死 + 缓存层检查；先看 verification.md 理解有哪些现成观测手段 |


## 编码规范（在 CLAUDE.md 和 workflow skill 中）

| 规范 | 说明 |
|------|------|
| JSON | 必须用 `cjson/cJSON.h`，禁止 `snprintf` 手拼 / `strstr`+`sscanf` 手写解析 |
| 时间 | 必须用 `clock_service.h` → `clock_now_us()`，禁止裸 `clock_gettime()` |
| 参数 | 必须用 `cJSON_Parse(params_json)`，禁止 `strstr` |
| 错误码 | 用 `error_codes.h` → `ERR_*` 替代 `return -1` |
| 日志 | 用 `LOG_INFO/WARN/ERROR(module, fmt, ...)` |
| 场景 | 新场景一律用 `road_network` + `s/l/segment_id` 格式 |

## 改动后必跑流程

```
写代码 → /verify → /code-review → /simplify → commit → 更新文档
```

## 教程文档

`skills/` 目录下 17 篇教程（OOP in C、插件系统、消息总线、IPC、Bag、Clock、Serializer、State Machine、Discovery、Fusion、Coroutine、Demo Evaluator、E2E Learning Loop、Dead Reckoning、SocketCAN Actuator、FlowSim 场景设计、vis 模块设计）。

| Skill | 何时使用 |
|-------|----------|
| `12_demo_evaluator.md` | 改动 pipeline 链路节点后跑回归；含 7 种深层故障模式（EKF 收敛、ref_path 航向腐败、NPC 投影陷阱等） |
| `16_flowsim_scenario_design.md` | 编写或修改 `scenarios/*.json`；多 edge + junction 路网设计、NPC 放置规范 |
| `17_vis_module_designer.md` | 设计并生成 vis/ 新 View 模块（路灯/护栏/行人/标志等）。用户说"加个 XX 模块"时触发 |
