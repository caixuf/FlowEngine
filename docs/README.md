# FlowEngine 文档中心

## 技术深度文档 (skills/)

| # | 主题 | 内容 |
|---|------|------|
| 01 | [C语言OOP模式](../skills/01_oop_in_c.md) | struct继承 + vtable多态 |
| 02 | [插件系统](../skills/02_plugin_system.md) | dlopen/dlsym 动态加载 |
| 03 | [消息总线](../skills/03_message_bus.md) | Pub/Sub + Req/Reply + Zero-Copy |
| 04 | [IPC通道](../skills/04_ipc_channel.md) | POSIX共享内存跨进程通信 |
| 05 | [Bag录制回放](../skills/05_bag_recording.md) | 二进制录制 + 时间感知回放 |
| 06 | [时钟服务](../skills/06_clock_service.md) | 真实/仿真时钟统一 |
| **07** | **[序列化层](../skills/07_serializer.md)** 🆕 | FNV-1a 类型ID + IDL代码生成 + 字节序 |
| **08** | **[反射式状态机](../skills/08_state_machine.md)** 🆕 | 转移表反射 + guard运行时替换 + ADAS驾驶模式 |
| **09** | **[服务发现](../skills/09_discovery.md)** 🆕 | UDP组播 + 拓扑追踪 + 自动IPC |
| **10** | **[数据融合](../skills/10_fusion.md)** 🆕 | MessageBuffer + 时间对齐 + FusionNodeCpp |
| **11** | **[协程通信原语](../skills/11_coroutine.md)** 🆕 | C++20 协程等传感器/超时/多路选择 |

## 项目评估

- [项目完善度评估](PROJECT_REVIEW.md) - 当前完成度、模块成熟度、短板与下一步路线
- [项目进化路线图](EVOLUTION_ROADMAP.md) - 从中间件原型进化到仿真驱动框架的阶段规划
- [落地实施指南](IMPLEMENTATION_GUIDE.md) - 把发展计划拆成"接口已定义、只需补实现"的小任务卡片

## 可视化 (FlowBoard)

- [FlowBoard 数据契约](FLOWBOARD_CONTRACT.md) - 可视化链路数据格式的唯一事实来源(schema + live/stale/demo 语义 + 原子写约定)。测试见 `tools/tests/`。

## 学习路径

### 推荐路径

1. **阅读 [快速入门](QUICK_START.md)** (30分钟)
   - 理解核心概念
   - 编译运行演示
   - 了解插件架构

2. **通读 [skills/ 技术深度文档](../skills/)** (按需查阅)
   - 从 `01_oop_in_c.md` 到 `11_coroutine.md`，覆盖全部核心模块

3. **阅读项目评估与规划**
   - [项目完善度评估](PROJECT_REVIEW.md)
   - [项目进化路线图](EVOLUTION_ROADMAP.md)

### 进阶开发者路径（2+年经验）

- **直接阅读源码** - 从 `include/task_interface.h` 开始
- **运行 demo 观察实际行为** - `bash scripts/demo.sh`
- **参与项目贡献** - 查看 GitHub Issues

## 文档说明

### 文档结构

```
docs/
├── README.md              # 本文件，文档索引
├── QUICK_START.md         # 30分钟快速入门
├── TECHNICAL_DESIGN.md    # 技术设计文档
├── PROJECT_REVIEW.md      # 项目完善度评估
├── EVOLUTION_ROADMAP.md   # 项目进化路线图
├── ALGORITHM_INTEGRATION.md  # 算法集成指南
├── ALGORITHM_STACK.md        # 算法栈参考（设计稿）
├── FLOW_REGISTRY_PLAN.md     # FlowRegistry 设计规划（已实现）
├── FLOWBOARD_CONTRACT.md     # FlowBoard 数据契约
├── VISUALIZATION_ARCHITECTURE.md  # 可视化架构
├── MONITORING_ARCHITECTURE.md     # 监控架构
├── E2E_SIMULATION_DESIGN.md       # E2E 仿真设计
├── SIMULATION_GUIDE.md     # 仿真测试指南
└── API_QUICK_REFERENCE.md  # API 快速参考
```

另有：
- `skills/`（项目根目录）— 11 篇深入教程（OOP in C、插件系统、消息总线、IPC 等）
- `scenarios/` — JSON 场景定义（行人横穿、高速超车）
- `tools/demo_evaluator.py` — 回归评估器

### 阅读建议

**如果你是：**

- **C语言新手**（<1年）：建议先补充C语言基础，特别是指针和内存管理
- **C语言熟悉**（1年+）：从快速入门开始，然后查阅 skills/ 技术深度文档
- **系统编程老手**（3年+）：直接看源码，参考 API 文档即可
- **想快速了解**：只看快速入门和 README

## 参与贡献

### 文档贡献

如果你发现文档问题或想要改进：

1. Fork 项目
2. 修改 `docs/` 目录下的文档
3. 提交 Pull Request
4. 等待审核合并

### 改进建议

- 添加更多示例代码
- 补充常见问题解答
- 翻译为其他语言
- 录制视频教程

## 获取帮助

- **GitHub Issues**: 报告bug或请求功能
- **GitHub Discussions**: 技术讨论和问答
- **邮件联系**: <2024740941@qq.com>

---

**开始你的 FlowEngine 学习之旅吧！**
