# FlowEngine 快速入门

> **目标：** 30 分钟内理解核心概念并运行第一个插件

## 步骤 1：编译项目

```bash
# 克隆仓库
git clone https://github.com/caixuf/FlowEngine.git
cd FlowEngine

# 安装系统依赖（Ubuntu / Debian）
sudo apt-get install -y build-essential cmake libcjson-dev

# 使用一键脚本编译（推荐）
bash build.sh release

# 可执行文件在 build/bin/，插件动态库在 build/lib/
```

> **GCC 11+ 是必要条件**，协程支持 (`-fcoroutines`) 从 GCC 11 开始提供。
> flowcoro 依赖库由 CMake FetchContent 自动下载，无需手动安装。

## 步骤 2：运行演示程序

```bash
# 端到端全链路演示（感知→融合→控制，15秒）
./build/bin/flow_e2e 15

# 消息总线演示（发布/订阅）
./build/bin/flow_bus

# IPC 跨进程通道演示（需两个终端）
./build/bin/flow_ipc pub   # 终端 1
./build/bin/flow_ipc sub   # 终端 2

# Bag 录制 & 回放演示
./build/bin/flow_bag record
./build/bin/flow_bag play

# C++20 协程 + 消息总线综合演示
./build/bin/flow_coro
```

## 步骤 3：理解核心概念

| 概念 | 说明 |
|------|------|
| **TaskBase** | 所有任务的公共基类（C struct） |
| **TaskInterface** | 函数指针表，定义任务必须实现的方法（C 的"虚函数表"） |
| **MessageBus** | 发布 / 订阅消息总线，替代 sleep 轮询 |
| **BusAwaitable** | C++20 awaitable，`co_await` 等待一条总线消息 |
| **FlowCoroTask** | 协程任务基类，结合 flowcoro 无锁线程池运行 |

## 步骤 4：查看最简单的 C 插件

参考 `src/plugins/example_task.c`：

```c
#include "task_interface.h"

typedef struct {
    TaskBase base;   /* 第一个成员必须是 TaskBase */
    int count;
} ExampleTask;

static int my_init(TaskBase* base) {
    ((ExampleTask*)base)->count = 0;
    return 0;
}

static int my_execute(TaskBase* base) {
    ExampleTask* t = (ExampleTask*)base;
    while (!base->should_stop) {
        printf("Hello #%d\n", ++t->count);
        sleep(1);
        base->stats.execution_count++;
    }
    return 0;
}

static void my_cleanup(TaskBase* base) { (void)base; }

static const TaskInterface my_vtable = {
    .initialize = my_init,
    .execute    = my_execute,
    .cleanup    = my_cleanup,
    .on_message = NULL,
};

TaskBase* create_task(const TaskConfig* config) {
    ExampleTask* t = calloc(1, sizeof(ExampleTask));
    task_base_init(&t->base, &my_vtable, config);
    return &t->base;
}

void destroy_task(TaskBase* base) {
    if (base) { task_base_destroy(base); free(base); }
}
```

在 `CMakeLists.txt` 中添加：

```cmake
add_library(my_plugin SHARED src/plugins/my_plugin.c)
target_link_libraries(my_plugin flowengine_core)
```

## 步骤 5：C++20 协程任务（进阶）

参考 `src/plugins/flowcoro_task.cpp`、`src/coro_bus_demo.cpp` 与
深入讲解 [`skills/11_coroutine.md`](../skills/11_coroutine.md)：

```cpp
#include "coroutine_task.h"

class MySensorTask : public FlowCoroTask {
protected:
    Task run() override {
        while (!should_stop()) {
            // 50ms 内没数据即视为丢帧；stop() 也会立刻唤醒协程
            auto r = co_await next_for("sensor/lidar", 50'000);
            if (r.cancelled()) break;          // 优雅停机，无需外发消息
            if (r.timed_out()) { watchdog(); continue; }
            process(*r);                       // r.message 为收到的消息
        }
    }
};

EXPORT_COROUTINE_TASK(my_sensor, MySensorTask)
```

协程在消息到达时由 flowcoro 无锁线程池自动恢复，无需回调地狱。
成员工厂 `next / next_for / select / select_for / sleep_ms / ask` 会自动注入
本任务的 `CancelToken`，因此 `stop()` 能直接唤醒悬挂在 `co_await` 上的协程并
以「已取消」语义返回，**无需再外发一条唤醒消息**。协程覆盖 pub/sub、select、
timer、req/reply 四类原语，并内置 `resume_count()` / `coro_latency()` 可观测统计。

## 关键技术点

### "继承"的实现

```c
typedef struct {
    TaskBase base;   /* 第一个成员 = 基类 */
    int my_field;
} DerivedTask;

/* 安全地从基类指针还原派生类指针 */
DerivedTask* d = (DerivedTask*)base_ptr;
```

### "多态"的实现

```c
/* 函数指针表 = 虚函数表 */
task->vtable->execute(task);   /* 调用具体实现 */
```

### 线程安全

FlowEngine 核心模块（logger、message_bus、task_manager）均通过 pthread mutex 保护共享状态；协程恢复路径通过 flowcoro 无锁队列分发，避免在消息回调线程中长时间持锁。

## 常见问题

**Q: 编译报 "coroutines not supported"？**
A: 升级到 GCC 11+，或安装 `g++-11`：`sudo apt install g++-11`

**Q: FetchContent 拉取 flowcoro 失败？**
A: 检查网络访问 GitHub 是否正常；或将 flowcoro 源码放入 `third_party/flowcoro/` 并在 CMakeLists.txt 中改用 `add_subdirectory`。

**Q: TaskBase 必须是第一个成员吗？**
A: 是的，这样才能在基类指针和派生类指针之间安全转换。

**Q: 如何调试插件？**
A: `gdb ./build/bin/flow_e2e`，设置 `LD_LIBRARY_PATH=./build/lib`。
