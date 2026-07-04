# FlowEngine 技术设计文档

## 1. 项目概述

FlowEngine 是一个基于插件化架构的轻量级中间件框架（原名 StartTool），灵感来源于大疆的进程管理方案。它通过动态库加载机制（dlopen）实现统一的进程管理，并在此基础上提供消息总线、IPC、Bag 录制、C++20 协程任务等能力。

## 2. 核心设计理念

### 2.1 插件化架构
- 各业务任务编译为动态库（.so 文件）
- 通过统一接口规范交互，主框架与业务完全解耦
- 启动器作为容器，运行时动态加载与管理插件

### 2.2 C 语言面向对象
- 用 `struct + 函数指针表` 实现虚函数表（类似 C++ vtable）
- `TaskBase` 作为基类，派生任务将其置于 struct 首位实现"继承"
- 所有公共操作通过 `TaskInterface` 的函数指针多态分发

### 2.3 消息驱动
- 消息总线（Message Bus）替代 sleep 轮询，任务仅在有数据时被唤醒
- C++20 协程通过 `BusAwaitable` 将消息等待封装为 `co_await` 表达式
- flowcoro 无锁线程池负责跨线程安全地恢复挂起协程

## 3. 总体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        FlowEngine 核心                          │
├──────────────┬──────────────┬──────────────┬────────────────────┤
│ 进程管理器   │  任务管理器  │  消息总线    │  协程任务          │
│ Process      │  Task        │  Message     │  CoroutineTask /   │
│ Manager      │  Manager     │  Bus         │  FlowCoroTask      │
├──────────────┼──────────────┼──────────────┼────────────────────┤
│ IPC 通道     │  Bag 录制    │  统一时钟    │  配置/日志         │
│ IPC Channel  │  Bag         │  Clock       │  Config / Logger   │
└──────────────┴──────────────┴──────────────┴────────────────────┘
                          │
               dlopen / dlsym
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                         插件层                                  │
├──────────────┬──────────────┬──────────────┬────────────────────┤
│  C 任务.so   │ C++ 任务.so  │ 协程任务.so  │  网络/数据服务.so  │
│  (C vtable)  │  (C vtable)  │ (FlowCoro)   │   (C++ wrapper)   │
└──────────────┴──────────────┴──────────────┴────────────────────┘
```

## 4. 关键组件详解

### 4.1 任务接口（task_interface.h）

```c
typedef struct TaskInterface {
    int  (*initialize)   (TaskBase* task);
    int  (*execute)      (TaskBase* task);
    void (*cleanup)      (TaskBase* task);
    int  (*pause)        (TaskBase* task);
    int  (*resume)       (TaskBase* task);
    void (*handle_signal)(TaskBase* task, int sig);
    bool (*health_check) (TaskBase* task);
    int  (*get_status)   (TaskBase* task, char* buf, size_t size);
    void (*on_message)   (TaskBase* task, const void* msg);
} TaskInterface;
```

### 4.2 任务管理器（task_manager.c）

- 线程安全：状态变更、名称表均由 pthread_mutex 保护
- 依赖排序：拓扑排序保证有依赖的任务按序启动
- 事件回调：状态变更时通知注册的 `task_event_callback`
- 健康监控：定期调用 `vtable->health_check`，失败则重启

### 4.3 消息总线（message_bus.c）

- 发布 / 订阅模式：任意数量的发布者和订阅者
- 话题匹配：精确字符串匹配，最大话题长度 `MSG_BUS_MAX_TOPIC_LEN`
- 请求 / 应答：支持同步 RPC 模式（`bus_request` / `bus_reply`）
- 线程安全：内部通过 pthread_mutex 保护订阅表和消息队列

### 4.4 协程任务集成（coroutine_task.h）

FlowEngine 通过以下三层抽象将 C++20 协程与消息总线融合：

```
┌─────────────────────────────────────┐
│  FlowCoroTask / CoroutineTask       │  <- 协程任务基类
│  继承 TaskBase，实现 execute()      │
├─────────────────────────────────────┤
│  BusAwaitable                       │  <- co_await 等待器
│  订阅话题，挂起协程，消息到达时恢复 │
├─────────────────────────────────────┤
│  flowcoro::lockfree::ThreadPool     │  <- 无锁线程池
│  跨线程安全恢复，不阻塞调度线程     │
└─────────────────────────────────────┘
```

**flowcoro 依赖**（自动拉取）

| 属性 | 值 |
|------|----|
| 仓库 | https://github.com/caixuf/flowcoro |
| 版本 | v4.0.0 |
| 许可 | MIT |
| 引入方式 | CMake FetchContent |
| 使用部分 | 纯头文件（`include/flowcoro/`），不链接 `flowcoro_net` |

**BusAwaitable 工作流程**

```
co_await subscribe_once("topic")
    │
    ├─ await_ready()  → false（总是挂起）
    ├─ await_suspend() → 订阅话题，保存 coroutine_handle
    │
    │   消息总线回调线程
    │   on_message() 被触发
    │       └─ thread_pool.enqueue(handle.resume)  ← 异步投递
    │
    └─ 线程池线程执行 handle.resume()
           └─ await_resume() → 返回 Message 对象
```

**编译开关**

| 宏 | 说明 |
|----|------|
| `FLOWCORO_INTEGRATION` | 定义时启用线程池恢复；未定义时退化为调度线程内联恢复（向后兼容） |

### 4.5 日志系统（logger.c / logger.h）

- **线程安全**：每个 `Logger` 实例持有一把 `pthread_mutex_t`
- **格式化接口**：`logger_logf(logger, level, fmt, ...)` 类 printf 语法
- **全局默认**：`default_log_callback` 通过 `pthread_once` 初始化，可直接传给 `process_manager_create()`
- **文件输出**：`logger_create("app.log", LOG_LEVEL_INFO)` 打开追加模式；传 NULL 输出到 stderr

### 4.6 IPC 通道（ipc_channel.c）

- 基于 POSIX 共享内存（`shm_open` / `mmap`）
- 零拷贝传输：发布者写入共享区域，订阅者直接读取
- 适合同一主机多进程间大数据量高频传输

### 4.7 Bag 录制回放（bag.c）

- 话题数据序列化到二进制文件，保留时间戳
- 回放时配合统一时钟服务（`clock_service.c`）控制速率
- 支持加速 / 减速回放，用于离线调试

## 5. 构建系统

```
CMakeLists.txt
├── cmake_minimum_required(VERSION 3.16)
├── 全局编译标志：-std=c11 / -std=c++20 / -fcoroutines / -Wall
├── FetchContent：flowcoro（仅头文件，接口目标 flowcoro_headers）
├── 静态库：flowengine_core（所有 src/core/*.c）
├── 可执行文件：flow_e2e, flow_bus, flow_coro, flow_ipc,
│              flow_bag, flowmond, flowctl, launcher, benchmark
├── 共享库插件：example_task, example_process, cpp_example_task,
│              reactive_task, network_service_task,
│              data_processor_task, flowcoro_task
└── CTest 测试条目
```

## 6. 配置格式（JSON）

```json
{
  "log_file": "flowengine.log",
  "log_level": 1,
  "monitor_interval": 5,
  "enable_monitor": true,
  "processes": [
    {
      "name": "sensor_task",
      "library": "./lib/libsensor_task.so",
      "priority": 2,
      "dependencies": [],
      "config": { "topic": "sensor/lidar", "rate_hz": 10 }
    }
  ]
}
```

## 7. 性能考量

| 场景 | 策略 |
|------|------|
| 高频消息（>1 kHz） | 使用 IPC 共享内存，零拷贝 |
| 低延迟任务 | 绑定 CPU 核，使用 `SCHED_FIFO`（需 root） |
| 协程并发 | flowcoro 无锁线程池，避免 mutex 竞争 |
| 大量订阅者 | MessageBus 内部链表遍历，订阅者数量建议 < 100 |

## 8. 扩展指南

1. **新增 C 插件**：实现 `TaskInterface`，导出 `create_task` / `destroy_task`，加入 CMakeLists.txt
2. **新增协程插件**：继承 `FlowCoroTask`，覆写 `run()`，用 `EXPORT_COROUTINE_TASK` 宏导出
3. **新增话题**：直接调用 `message_bus_publish(bus, "my/topic", data, size, "sender")`
4. **自定义时钟**：调用 `clock_service_set_mode(CLOCK_MODE_SIMULATION)` 后控制速率

## 9. 已知限制

- MessageBus 单进程内使用；跨进程请用 IPC Channel
- 协程任务需要 GCC 11+ 或 Clang 14+（`-fcoroutines` / `-fcoroutines-ts`）
- flowcoro 线程池线程数默认为 `hardware_concurrency()`，生产环境可通过修改 `get_thread_pool()` 调整
