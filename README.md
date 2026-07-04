# FlowEngine - 轻量级任务调度与消息总线框架

## 项目概述

FlowEngine 是一个基于 C/C++ 实现的轻量级中间件框架，提供**统一进程启动**、**任务调度**、**消息总线**、**协程异步任务**、**跨进程通信**、**数据录制回放**等核心能力，适合嵌入式系统、机器人、自动驾驶等对性能和实时性要求较高的场景。

> 原名 StartTool，随着功能不断扩展（消息总线、IPC 通道、Bag 录制、协程集成、统一时钟等），重命名为 FlowEngine 以更好地反映其定位。

## 核心能力

| 模块 | 功能 |
|------|------|
| **进程管理** (Process Manager) | dlopen 动态加载插件，依赖排序，生命周期管理 |
| **任务接口** (Task Interface) | C 语言虚函数表，统一 initialize / execute / cleanup / health_check |
| **消息总线** (Message Bus) | 发布/订阅，消息驱动调度，替代 sleep 轮询 |
| **协程任务** (Coroutine Task) | C++20 协程 + flowcoro 无锁线程池，co_await 等待消息 |
| **IPC 通道** (IPC Channel) | 基于共享内存的跨进程零拷贝通信 |
| **Bag 录制回放** (Bag) | 话题数据录制到文件，离线回放，配合统一时钟 |
| **统一时钟** (Clock Service) | 模拟时钟与真实时钟切换，支持加速/减速回放 |
| **配置管理** (Config Manager) | JSON 配置文件解析，支持多环境配置 |
| **统一日志** (Logger) | 分级日志，线程安全，格式化输出，可输出到文件 |

## 应用场景

在多服务系统（如 GIS 系统、机器人、自动驾驶）中，FlowEngine 解决：

- **启动混乱** — 多个服务需要按依赖顺序手动启动
- **配置分散** — 每个服务一套配置文件，难以统一管理
- **日志碎片** — 日志散落各处，问题排查效率低
- **协程并发** — 传统回调难以维护，协程让异步代码像同步一样清晰
- **数据回放** — 无法离线复现线上问题

## 项目结构

```
FlowEngine/
├── src/
│   ├── core/                    # 核心模块（C）
│   │   ├── task_interface.c     # 任务接口（虚函数表机制）
│   │   ├── task_manager.c       # 任务生命周期管理
│   │   ├── process_manager.c    # 进程/插件管理
│   │   ├── message_bus.c        # 消息总线（发布/订阅）
│   │   ├── ipc_channel.c        # 跨进程共享内存通信
│   │   ├── bag.c                # 数据录制与回放
│   │   ├── clock_service.c      # 统一时钟服务
│   │   ├── config_manager.c     # JSON 配置管理
│   │   └── logger.c             # 统一日志（线程安全）
│   ├── plugins/                 # 插件示例
│   │   ├── example_task.c       # C 语言任务示例
│   │   ├── simple_cpp_task.cpp  # C++ 任务示例
│   │   ├── reactive_task.c      # 消息驱动任务示例
│   │   ├── network_service_task.cpp  # 网络服务示例
│   │   ├── data_processor_task.cpp   # 数据处理示例
│   │   ├── flowcoro_task.cpp    # C++20 协程任务示例（需 flowcoro）
│   │   ├── fake_perception_task.c  # 假感知节点（LiDAR/GPS/障碍物仿真）
│   │   └── fake_control_task.c     # 假控制节点（油门/制动/转向决策）
│   ├── launcher.c               # 主启动器
│   ├── task_demo.c              # flow_task 源文件（任务系统）
│   ├── bus_demo.c               # flow_bus 源文件（消息总线）
│   ├── simple_cpp_demo.cpp      # flow_cpp 源文件（C++ 任务）
│   ├── ipc_demo.c               # flow_ipc 源文件（IPC 通道）
│   ├── bag_demo.c               # flow_bag 源文件（Bag 录制回放）
│   ├── coro_bus_demo.cpp        # flow_coro 源文件（协程 + 总线）
│   └── adas_demo.c              # flow_adas 源文件（ADAS 完整链路）
├── include/                     # 公共头文件
│   ├── coroutine_task.h         # 协程任务基类（BusAwaitable / FlowCoroTask）
│   ├── task_interface.h         # 任务接口定义
│   └── ...
├── config/                      # 配置文件模板
├── docs/                        # 详细技术文档
├── skills/                      # 技术要点文档
├── build.sh                     # 一键编译脚本
└── CMakeLists.txt
```

## 快速开始

### 环境要求

| 工具 | 最低版本 | 说明 |
|------|---------|------|
| GCC / G++ | **11.0+** | 协程支持需要 GCC 11 以上 (`-fcoroutines`) |
| CMake | **3.16+** | FetchContent 功能 |
| libcjson | 任意 | JSON 配置解析 (`sudo apt install libcjson-dev`) |
| pthread / dl / rt | 系统自带 | 标准 POSIX 库 |

> **flowcoro** 由 CMake FetchContent 自动拉取，无需手动安装。

### 编译

```bash
git clone https://github.com/caixuf/FlowEngine.git
cd FlowEngine

# 使用一键脚本（推荐）
bash build.sh release

# 或手动
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

编译完成后产物位于 `build/bin/`（可执行文件）和 `build/lib/`（动态库插件）。

### 运行

```bash
# C 任务系统（3 个并发任务，展示生命周期）
./build/bin/flow_task

# 消息总线（发布/订阅）
./build/bin/flow_bus

# C++ 任务
./build/bin/flow_cpp

# IPC 跨进程通道（需两个终端）
./build/bin/flow_ipc pub   # 终端1：发布者
./build/bin/flow_ipc sub   # 终端2：订阅者

# Bag 录制 & 回放（含统一时钟）
./build/bin/flow_bag record
./build/bin/flow_bag play

# C++20 协程 + 消息总线
./build/bin/flow_coro

# ADAS 完整链路仿真（感知节点 + 控制节点）
./build/bin/flow_adas
```

## 开发插件

### C 语言插件

```c
#include "task_interface.h"

typedef struct {
    TaskBase base;   /* 必须放第一位 */
    int my_param;
} MyService;

static int my_init(TaskBase* base) {
    ((MyService*)base)->my_param = 42;
    return 0;
}

static int my_execute(TaskBase* base) {
    while (!base->should_stop) {
        /* 业务逻辑 */
        sleep(1);
        base->stats.execution_count++;
    }
    return 0;
}

static const TaskInterface my_vtable = {
    .initialize = my_init,
    .execute    = my_execute,
    .cleanup    = NULL,
    .on_message = NULL,
};

TaskBase* create_task(const TaskConfig* config) {
    MyService* s = calloc(1, sizeof(MyService));
    task_base_init(&s->base, &my_vtable, config);
    return &s->base;
}
```

### C++20 协程插件（需 flowcoro）

```cpp
#include "coroutine_task.h"

class MySensorTask : public FlowCoroTask {
public:
    MySensorTask(const TaskConfig* cfg, MessageBus* bus)
        : FlowCoroTask(cfg, bus) {}

    Task<void> run() override {
        while (true) {
            // 异步等待消息，协程在此挂起，不阻塞线程
            auto msg = co_await subscribe_once("sensor/lidar");
            process(msg);
        }
    }
};

EXPORT_COROUTINE_TASK(my_sensor, MySensorTask)
```

## 技术文档

- [项目完善度评估](docs/PROJECT_REVIEW.md)
- [项目进化路线图](docs/EVOLUTION_ROADMAP.md)
- [快速入门](docs/QUICK_START.md)
- [技术设计文档](docs/TECHNICAL_DESIGN.md)
- [完整学习指南](docs/LEARNING_GUIDE.md)
- [任务系统详解](docs/TASK_SYSTEM_GUIDE.md)
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
