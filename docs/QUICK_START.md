# StartTool 快速入门示例

> **目标：** 30分钟内理解核心概念并运行第一个插件

## 快速体验

### 步骤1：编译运行演示程序

```bash
# 进入项目目录
cd /home/caixuf/MyCode/startTool

# 编译项目
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 运行C语言任务演示
./task_demo

# 运行C++任务演示  
./simple_cpp_demo
```

### 步骤2：理解核心概念

**最重要的3个概念：**

1. **TaskBase (任务基类)** - 所有任务的公共部分
2. **TaskInterface (任务接口)** - 定义任务必须实现的函数
3. **虚函数表** - C语言实现面向对象的关键技术

### 步骤3：查看一个最简单的插件

打开文件：`src/plugins/example_process.c`

**核心结构：**
```c
// 1. 定义你的任务结构（继承TaskBase）
typedef struct {
    TaskBase base;  // 必须放第一位！
    // 你的数据...
} MyTask;

// 2. 实现四个必需的函数
static int my_initialize(TaskBase* task) { /*初始化*/ }
static int my_execute(TaskBase* task) { /*主循环*/ }  
static void my_cleanup(TaskBase* task) { /*清理*/ }
static bool my_health_check(TaskBase* task) { /*健康检查*/ }

// 3. 创建虚函数表
static const TaskInterface my_vtable = {
    .initialize = my_initialize,
    .execute = my_execute,
    .cleanup = my_cleanup,
    .health_check = my_health_check
};

// 4. 导出创建函数
extern "C" TaskBase* create_task(const TaskConfig* config) {
    // 创建任务实例...
}
```

## 第一个练习：Hello World 任务

### 创建新文件：`hello_task.c`

```c
#include "task_interface.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// 步骤1：定义任务结构
typedef struct {
    TaskBase base;
    int count;
} HelloTask;

// 步骤2：实现初始化
static int hello_initialize(TaskBase* base) {
    HelloTask* task = (HelloTask*)base;
    task->count = 0;
    printf("[HelloTask] 初始化完成!\n");
    return 0;
}

// 步骤3：实现主循环
static int hello_execute(TaskBase* base) {
    HelloTask* task = (HelloTask*)base;
    
    while (!base->should_stop && task->count < 10) {
        printf("[HelloTask] Hello World! 第%d次\n", ++task->count);
        sleep(2);  // 每2秒打印一次
        base->stats.execution_count++;
    }
    
    printf("[HelloTask] 任务完成!\n");
    return 0;
}

// 步骤4：实现清理
static void hello_cleanup(TaskBase* base) {
    printf("[HelloTask] 清理资源\n");
}

// 步骤5：实现健康检查
static bool hello_health_check(TaskBase* base) {
    return base->state == TASK_STATE_RUNNING;
}

// 步骤6：创建虚函数表
static const TaskInterface hello_vtable = {
    .initialize = hello_initialize,
    .execute = hello_execute,
    .cleanup = hello_cleanup,
    .health_check = hello_health_check
};

// 步骤7：导出函数
extern "C" TaskBase* create_task(const TaskConfig* config) {
    HelloTask* task = malloc(sizeof(HelloTask));
    if (!task) return NULL;
    
    if (task_base_init(&task->base, &hello_vtable, config) != 0) {
        free(task);
        return NULL;
    }
    
    task->count = 0;
    return &task->base;
}

extern "C" void destroy_task(TaskBase* base) {
    if (base) {
        task_base_destroy(base);
        free(base);
    }
}
```

### 编译你的插件

在CMakeLists.txt中添加：
```cmake
add_library(hello_task SHARED src/plugins/hello_task.c)
target_link_libraries(hello_task starttool_core)
```

重新编译：
```bash
cd build
make
```

### 测试你的插件

创建简单的测试程序：
```c
// test_hello.c
#include "task_interface.h"

int main() {
    // 创建配置
    TaskConfig config = {
        .name = "HelloTask",
        .priority = TASK_PRIORITY_NORMAL
    };
    
    // 加载插件（简化版本，实际会用dlopen）
    TaskBase* task = create_task(&config);
    
    // 启动任务
    if (task_start(task) == 0) {
        printf("任务启动成功!\n");
        
        // 等待任务完成
        task_wait(task);
    }
    
    // 清理
    destroy_task(task);
    return 0;
}
```

## 理解执行流程

当你运行上面的HelloTask时，执行流程是：

```
1. create_task() 创建任务实例
   ↓
2. task_start() 启动任务
   ↓  
3. 创建新线程调用 task_thread_entry()
   ↓
4. task_thread_entry() 调用 hello_execute()
   ↓
5. hello_execute() 运行主循环
   ↓
6. 循环结束后调用 hello_cleanup()
   ↓
7. 任务结束
```

## 关键技术点解析

### 1. "继承"的实现
```c
typedef struct {
    TaskBase base;  // 第一个成员必须是基类
    // 子类数据...
} DerivedTask;

// 类型转换实现"继承"
DerivedTask* derived = (DerivedTask*)base_pointer;
```

### 2. "多态"的实现
```c
// 通过函数指针实现虚函数
task->vtable->execute(task);  // 调用具体实现

// 宏简化调用
#define TASK_CALL(task, method) \
    ((task)->vtable->method ? (task)->vtable->method(task) : -1)
```

### 3. 线程安全
```c
// 所有状态修改都要加锁
pthread_mutex_lock(&task->mutex);
task->state = TASK_STATE_RUNNING;
pthread_mutex_unlock(&task->mutex);
```

## 下一步学习建议

1. **修改HelloTask**：让它从配置文件读取打印次数
2. **学习现有插件**：分析 `simple_cpp_task.cpp` 的C++实现
3. **尝试通信**：让两个任务之间传递消息
4. **添加配置**：学习JSON配置解析
5. **深入源码**：理解 `task_manager.c` 的管理逻辑

## 常见新手问题

**Q: 为什么TaskBase必须是第一个成员？**
A: 这样可以安全地在基类指针和派生类指针之间转换，实现"继承"。

**Q: 虚函数表是什么？**  
A: 就是一个函数指针结构体，用来实现C语言的"多态"。

**Q: 为什么要用线程？**
A: 每个任务独立运行，不会相互阻塞。

**Q: 如何调试我的插件？**
A: 使用printf调试，或者用gdb：`gdb ./task_demo`

---

恭喜！你已经理解了StartTool的核心概念。继续学习完整的学习指南，深入掌握更多高级特性！
