# FlowEngine 文档索引

FlowEngine 全部文档的统一入口。按「我想做什么」分组，找不到时用页内搜索（Ctrl+F）关键词。

> 顶层 [README.md](../README.md) 是项目总览；[CLAUDE.md](../CLAUDE.md) 是给 AI/贡献者的编码规范与故障速查；
> `.claude/skills/` 是开发流程 skill（设计→执行→测试→迭代→清理→文档）。

## 🚀 快速上手

| 文档 | 说明 |
|------|------|
| [QUICK_START.md](QUICK_START.md) | 30 分钟快速入门：构建、跑 demo、看仪表盘 |
| [SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) | 仿真测试指南 + 场景矩阵回归 |
| [CODE_WIKI.md](CODE_WIKI.md) | 代码地图：核心模块 / 节点 / 关键函数逐一索引（找代码从这里开始） |
| [API_QUICK_REFERENCE.md](API_QUICK_REFERENCE.md) | 统一 API 速查（cJSON / clock_service / param / node_pump） |

## 🏗️ 架构与设计

| 文档 | 说明 |
|------|------|
| [TECHNICAL_DESIGN.md](TECHNICAL_DESIGN.md) | 技术设计总览 |
| [PIPELINE_ARCHITECTURE.md](PIPELINE_ARCHITECTURE.md) | Pipeline / 节点拓扑架构 |
| [MONITORING_ARCHITECTURE.md](MONITORING_ARCHITECTURE.md) | 监控与数据采集架构（flowmond / 桥接） |
| [VISUALIZATION_ARCHITECTURE.md](VISUALIZATION_ARCHITECTURE.md) | 3D 可视化架构（vis/ 模块） |
| [VIS_MODULE_GUIDE.md](VIS_MODULE_GUIDE.md) | vis/ 模块设计规范（View 接入指南） |

## 🧠 算法（规划 / 控制 / 感知）

| 文档 | 说明 |
|------|------|
| [ALGORITHM_STACK.md](ALGORITHM_STACK.md) | 算法栈总览（实际实现） |
| [ALGORITHM_INTEGRATION.md](ALGORITHM_INTEGRATION.md) | 算法集成指南 |
| [ALGORITHM_VERIFY_PATTERN.md](ALGORITHM_VERIFY_PATTERN.md) | 算法快速验证工作流（Python 仿真先行） |
| [PLANNING_SPEED_UPGRADE_DESIGN.md](PLANNING_SPEED_UPGRADE_DESIGN.md) | 速度规划升级设计（ST 图 + DP/QP） |
| [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md) | 控制参数标定指南 |

## 🎓 学习闭环

| 文档 | 说明 |
|------|------|
| [LEARNING_LOOP.md](LEARNING_LOOP.md) | 车端学习闭环（采集→训练→影子评估→OTA） |

## 🔌 数据契约与 Schema

| 文档 | 说明 |
|------|------|
| [FLOWBOARD_CONTRACT.md](FLOWBOARD_CONTRACT.md) | FlowBoard 数据契约 |
| [FLOWBOARD_SCENE_CONTRACT.md](FLOWBOARD_SCENE_CONTRACT.md) | FlowBoard 3D Scene 数据契约 |
| [SCHEMA_road_network.md](SCHEMA_road_network.md) | road_network JSON Schema |
| [SIM_DIGEST.md](SIM_DIGEST.md) | 仿真 digest / invariant 与调试可视化 |

## 🚗 硬件部署

| 文档 | 说明 |
|------|------|
| [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) | 真车硬件部署指南 |
| [RC_CAR_HARDWARE_CHECKLIST.md](RC_CAR_HARDWARE_CHECKLIST.md) | RC 小车硬件连接操作清单 |

## 🧪 场景与演示

| 文档 | 说明 |
|------|------|
| [DRIVING_SCHOOL_PLAN.md](DRIVING_SCHOOL_PLAN.md) | 驾校计划（科目一至科目四场景） |

## 🩺 故障排查

| 文档 | 说明 |
|------|------|
| [TROUBLESHOOTING_3D_DASHBOARD.md](TROUBLESHOOTING_3D_DASHBOARD.md) | 3D 仪表盘"加载失败"排查与修复 |

> 更多运行期故障速查见 [CLAUDE.md](../CLAUDE.md) 的「常见故障模式」表。

## 📚 教程（tutorials/）

从零理解 FlowEngine 各子系统的分步教程。

| # | 教程 | 主题 |
|---|------|------|
| 01 | [C 语言面向对象编程](tutorials/01_oop_in_c.md) | OOP in C |
| 02 | [插件化架构](tutorials/02_plugin_system.md) | dlopen 动态加载 |
| 03 | [消息总线与发布/订阅](tutorials/03_message_bus.md) | Message Bus |
| 04 | [IPC 跨进程通信](tutorials/04_ipc_channel.md) | 共享内存通道 |
| 05 | [数据录制与回放](tutorials/05_bag_recording.md) | Bag |
| 06 | [统一时钟服务](tutorials/06_clock_service.md) | Clock Service |
| 07 | [类型安全序列化层](tutorials/07_serializer.md) | Serializer |
| 08 | [反射式状态机](tutorials/08_state_machine.md) | State Machine |
| 09 | [服务发现与拓扑管理](tutorials/09_discovery.md) | Discovery |
| 10 | [数据融合框架](tutorials/10_fusion.md) | Fusion (EKF) |
| 11 | [C++20 协程通信原语](tutorials/11_coroutine.md) | Coroutine |
| 12 | [Demo Evaluator 回归评估器](tutorials/12_demo_evaluator.md) | Demo Evaluator |
| 13 | [E2E Learning Loop 训练闭环](tutorials/13_e2e_learning_loop.md) | E2E Learning Loop |
| 14 | [前端航位推算](tutorials/14_dead_reckoning.md) | Dead Reckoning |
| 15 | [SocketCAN 执行器节点](tutorials/15_socketcan_actuator.md) | SocketCAN Actuator |
| 16 | [FlowSim 场景设计](tutorials/16_flowsim_scenario_design.md) | 多 edge 路网 + NPC |
| 17 | [vis 模块设计](tutorials/17_vis_module_designer.md) | View 模块规范 |
