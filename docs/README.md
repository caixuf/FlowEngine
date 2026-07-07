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

## 项目评估

- [项目完善度评估](PROJECT_REVIEW.md) - 当前完成度、模块成熟度、短板与下一步路线
- [项目进化路线图](EVOLUTION_ROADMAP.md) - 从中间件原型进化到可部署框架的阶段规划

## 学习路径

### 初学者路径（1年经验左右）

1. **[快速入门](QUICK_START.md)** (30分钟)
   - 理解核心概念
   - 编译运行演示
   - 创建第一个插件

2. **[完整学习指南](LEARNING_GUIDE.md)** (2-4周)
   - 深入理解架构设计
   - 掌握多线程编程
   - 学会C/C++混合编程

3. **[实战练习项目](PRACTICE_PROJECTS.md)** (1-3天/项目)
   - 系统监控器
   - HTTP服务器
   - 任务调度器
   - 数据库连接池

### 进阶开发者路径（2+年经验）

- **直接阅读源码** - 从 `include/task_interface.h` 开始
- **参与项目贡献** - 查看 GitHub Issues
- **架构设计讨论** - 加入技术交流群

## 文档说明

### 文档结构

```
docs/
├── README.md              # 本文件，文档索引
├── QUICK_START.md         # 30分钟快速入门
├── LEARNING_GUIDE.md      # 完整学习指南（2-4周）
├── PRACTICE_PROJECTS.md   # 实战练习项目
├── TECHNICAL_DESIGN.md    # 技术设计文档
├── PROJECT_REVIEW.md      # 项目完善度评估
├── EVOLUTION_ROADMAP.md   # 项目进化路线图
├── MONITORING_ARCHITECTURE.md # flowmond 监控守护进程 + stats_bridge IPC 桥接
├── ALGORITHM_INTEGRATION.md # 算法集成指南
├── FLOW_REGISTRY_PLAN.md  # FlowRegistry 设计规划
└── ...
```

### 阅读建议

**如果你是：**

- **C语言新手**（<1年）：建议先补充C语言基础，特别是指针和内存管理
- **C语言熟悉**（1年+）：从快速入门开始，然后完整学习指南
- **系统编程老手**（3年+）：直接看源码，参考API文档即可
- **想快速了解**：只看快速入门即可
- **想深入学习**：完整学习指南 + 实战项目)

## 学习目标检查

完成学习后，你应该能够：

- [ ] 解释插件化架构的优势
- [ ] 理解C语言面向对象编程
- [ ] 使用虚函数表实现多态
- [ ] 编写线程安全的代码
- [ ] 创建和集成自定义插件
- [ ] 调试多线程程序
- [ ] 设计可扩展的系统架构

## 参与贡献

### 文档贡献

如果你发现文档问题或想要改进：

1. Fork 项目)
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
