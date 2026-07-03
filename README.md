# FlowEngine - 轻量级任务调度与进程管理框架

## 项目概述

FlowEngine 是一个基于 C/C++ 实现的轻量级中间件框架，提供**统一进程启动**、**任务调度**、**消息总线**、**跨进程通信**、**数据录制回放**等核心能力，适合嵌入式系统、机器人、自动驾驶等对性能和实时性要求较高的场景。

> 原名 StartTool，随着功能不断扩展（消息总线、IPC 通道、Bag 录制、统一时钟等），重命名为 FlowEngine 以更好地反映其定位。

## 核心能力

| 模块 | 功能 |
|------|------|
| **进程管理** (Process Manager) | dlopen 动态加载插件，依赖排序，生命周期管理 |
| **任务接口** (Task Interface) | C 语言虚函数表，统一 initialize / execute / cleanup / health_check |
| **消息总线** (Message Bus) | 发布/订阅，消息驱动调度，替代 sleep 轮询 |
| **IPC 通道** (IPC Channel) | 基于共享内存的跨进程零拷贝通信 |
| **Bag 录制回放** (Bag) | 话题数据录制到文件，离线回放，配合统一时钟 |
| **统一时钟** (Clock Service) | 模拟时钟与真实时钟切换，支持加速/减速回放 |
| **配置管理** (Config Manager) | JSON 配置文件解析，支持多环境配置 |
| **统一日志** (Logger) | 分级日志，统一格式，可输出到文件 |

## 应用场景

### 典型痛点：多服务系统的协调管理

在多服务系统（如 GIS 系统、机器人、自动驾驶）中，经常面临：

- **启动混乱** — 多个服务需要按依赖顺序手动启动
- **配置分散** — 每个服务一套配置文件，难以统一管理
- **日志碎片** — 日志散落各处，问题排查效率低
- **依赖地狱** — 服务间依赖关系复杂，启动顺序难以维护
- **数据回放** — 无法离线复现线上问题

### FlowEngine 的解决方案

```bash
# 一条命令启动所有服务（自动按依赖顺序）
./launcher --config configs/development.json

[2025-07-22 10:30:01] INFO  FlowEngine 初始化完成
[2025-07-22 10:30:02] INFO  定位服务 (positioning_service) 启动成功
[2025-07-22 10:30:03] INFO  地图服务 (map_service) 启动成功
[2025-07-22 10:30:04] INFO  路径规划 (routing_service) 启动成功
[2025-07-22 10:30:05] INFO  所有 3 个服务启动完成，系统就绪!
```

## 项目结构

```
FlowEngine/
├── src/
│   ├── core/                    # 核心模块
│   │   ├── task_interface.c     # 任务接口（虚函数表机制）
│   │   ├── task_manager.c       # 任务生命周期管理
│   │   ├── process_manager.c    # 进程/插件管理
│   │   ├── message_bus.c        # 消息总线（发布/订阅）
│   │   ├── ipc_channel.c        # 跨进程共享内存通信
│   │   ├── bag.c                # 数据录制与回放
│   │   ├── clock_service.c      # 统一时钟服务
│   │   ├── config_manager.c     # JSON 配置管理
│   │   ├── logger.c             # 统一日志
│   │   └── msg_schema.c         # 消息格式定义
│   ├── plugins/                 # 示例插件
│   │   ├── example_task.c       # C 语言任务示例
│   │   ├── simple_cpp_task.cpp  # C++ 任务示例
│   │   ├── reactive_task.c      # 消息驱动任务示例
│   │   ├── network_service_task.cpp  # 网络服务示例
│   │   └── data_processor_task.cpp  # 数据处理示例
│   ├── launcher.c               # 主启动器
│   ├── task_demo.c              # 任务演示
│   ├── bus_demo.c               # 消息总线演示
│   ├── ipc_demo.c               # IPC 通道演示
│   └── bag_demo.c               # Bag 录制回放演示
├── include/                     # 公共头文件
├── config/                      # 配置文件模板
├── docs/                        # 详细技术文档
├── skills/                      # 技术要点文档
└── CMakeLists.txt
```

## 快速开始

### 环境要求

- **OS**：Linux (Ubuntu 18.04+ / CentOS 7+)
- **C 编译器**：GCC 7.0+ (C11) 或 Clang 8.0+
- **C++ 编译器**：G++ 7.0+ (C++20) 或 Clang++ 8.0+
- **构建工具**：CMake 3.10+
- **依赖**：pthread, dl, libcjson, rt

### 编译

```bash
git clone https://github.com/caixuf/startTool.git flowengine
cd flowengine

mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 运行演示

```bash
# C 任务演示（3 个并发任务，展示生命周期）
./build/bin/task_demo

# 消息总线演示（发布/订阅）
./build/bin/bus_demo

# IPC 跨进程通道演示
./build/bin/ipc_demo

# Bag 录制 & 回放演示（含统一时钟）
./build/bin/bag_demo

# C++ 任务演示
./build/bin/cpp_task_demo
```

## 开发插件

### C 语言插件

```c
#include "task_interface.h"

// 1. 定义服务结构体，第一个成员必须是 TaskBase
typedef struct {
    TaskBase base;
    int my_param;
} MyService;

// 2. 实现虚函数
static int my_init(TaskBase* base) {
    MyService* s = (MyService*)base;
    s->my_param = 42;
    return 0;
}

static int my_execute(TaskBase* base) {
    while (!base->should_stop) {
        // 业务逻辑
        sleep(1);
        base->stats.execution_count++;
    }
    return 0;
}

// 3. 注册虚函数表
static const TaskInterface my_vtable = {
    .initialize = my_init,
    .execute    = my_execute,
};

// 4. 导出工厂函数
TaskBase* create_task(const TaskConfig* config) {
    MyService* s = calloc(1, sizeof(MyService));
    task_base_init(&s->base, &my_vtable, config);
    return &s->base;
}
```

### 消息驱动插件

```c
// 任务不需要 sleep 轮询，由消息触发执行
static void my_on_message(TaskBase* base, const void* msg) {
    // 收到消息时被自动回调
    printf("收到消息，处理中...\n");
}

static int my_execute(TaskBase* base) {
    // 订阅话题后阻塞等待即可
    task_subscribe(base, g_bus, "sensor/data");
    pause();   // 由 on_message 回调驱动
    return 0;
}
```

## 技术文档

- [快速入门](docs/QUICK_START.md)
- [完整学习指南](docs/LEARNING_GUIDE.md)
- [任务系统详解](docs/TASK_SYSTEM_GUIDE.md)
- [技术设计文档](docs/TECHNICAL_DESIGN.md)
- [实战练习项目](docs/PRACTICE_PROJECTS.md)

## 技术要点 (skills/)

- [C 语言面向对象](skills/01_oop_in_c.md)
- [插件化架构 (dlopen)](skills/02_plugin_system.md)
- [消息总线与发布订阅](skills/03_message_bus.md)
- [IPC 跨进程通信](skills/04_ipc_channel.md)
- [数据录制与回放 (Bag)](skills/05_bag_recording.md)
- [统一时钟服务](skills/06_clock_service.md)

## License

MIT
