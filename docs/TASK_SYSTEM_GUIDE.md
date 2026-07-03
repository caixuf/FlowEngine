# 任务接口系统使用指南

## 概述

我已经为你的启动器项目设计了一个基于面向对象思想的任务接口系统。这个系统在C语言中实现了类似于C++的继承和多态机制，让你可以通过父类指针来统一管理各种不同的任务。

## 核心概念

### 1. 任务基类 (TaskBase)
```c
typedef struct TaskBase {
    TaskConfig config;              // 任务配置
    TaskState state;                // 当前状态
    TaskStats stats;                // 统计信息
    pthread_t thread;               // 任务线程
    pthread_mutex_t mutex;          // 状态保护互斥锁
    bool should_stop;               // 停止标志
    uint32_t restart_count;         // 已重启次数
    const struct TaskInterface* vtable;  // 虚函数表指针
} TaskBase;
```

### 2. 虚函数接口 (TaskInterface)
```c
typedef struct TaskInterface {
    int (*initialize)(TaskBase* task);     // 纯虚函数 - 必须实现
    int (*execute)(TaskBase* task);        // 纯虚函数 - 必须实现  
    void (*cleanup)(TaskBase* task);       // 虚函数 - 可选重写
    int (*pause)(TaskBase* task);          // 虚函数 - 可选重写
    int (*resume)(TaskBase* task);         // 虚函数 - 可选重写
    void (*handle_signal)(TaskBase* task, int signal);  // 虚函数
    bool (*health_check)(TaskBase* task);  // 虚函数
    int (*get_status)(TaskBase* task, char* buffer, size_t size); // 虚函数
} TaskInterface;
```

### 3. 继承机制

在C语言中实现继承的关键是：**将基类结构体作为子类的第一个成员**

```c
// 示例任务 "继承" TaskBase
typedef struct ExampleTask {
    TaskBase base;              // "基类" - 必须是第一个成员
    
    // "子类"特有的成员变量
    int work_interval;          
    int counter;                
    char message[256];          
    bool use_random_delay;      
} ExampleTask;
```

### 4. 多态机制

通过虚函数表实现多态：

```c
// 通过基类指针调用子类实现的虚函数
TaskBase* task = get_some_task();
TASK_CALL(task, execute);  // 会调用具体子类的execute实现
```

## 使用步骤

### 步骤1: 定义你的任务结构体

```c
typedef struct MyTask {
    TaskBase base;          // 继承基类 - 必须是第一个成员
    
    // 你的任务特有数据
    int my_data;
    char my_string[256];
} MyTask;
```

### 步骤2: 实现虚函数

```c
// 实现initialize虚函数
static int my_task_initialize(TaskBase* base_task) {
    MyTask* task = TASK_CAST(MyTask, base_task);
    // 初始化逻辑
    task->my_data = 0;
    return 0;
}

// 实现execute虚函数(主循环)
static int my_task_execute(TaskBase* base_task) {
    MyTask* task = TASK_CAST(MyTask, base_task);
    
    while (!task_should_stop(base_task)) {
        // 你的任务逻辑
        task->my_data++;
        
        // 更新心跳
        task_update_heartbeat(base_task);
        
        // 工作间隔
        sleep(1);
    }
    return 0;
}

// 其他虚函数实现...
```

### 步骤3: 定义虚函数表

```c
static const TaskInterface my_task_vtable = {
    .initialize = my_task_initialize,
    .execute = my_task_execute,
    .cleanup = my_task_cleanup,        // 可选
    .health_check = my_task_health_check, // 可选
    // 其他函数使用默认实现
};
```

### 步骤4: 实现构造和析构函数

```c
MyTask* my_task_create(const TaskConfig* config) {
    MyTask* task = malloc(sizeof(MyTask));
    if (!task) return NULL;
    
    // 初始化基类
    if (task_base_init(&task->base, &my_task_vtable, config) != 0) {
        free(task);
        return NULL;
    }
    
    // 初始化子类成员
    task->my_data = 0;
    strcpy(task->my_string, "Hello");
    
    return task;
}

void my_task_destroy(MyTask* task) {
    if (!task) return;
    
    // 销毁基类
    task_base_destroy(&task->base);
    
    // 清理子类资源
    // ...
    
    free(task);
}
```

### 步骤5: 使用任务管理器

```c
// 创建任务管理器
TaskManager* manager = task_manager_create();

// 创建任务实例
TaskConfig config = {
    .name = "my_task",
    .description = "我的任务",
    .priority = TASK_PRIORITY_NORMAL,
    .max_restart_count = 3,
    .auto_restart = true
};

MyTask* my_task = my_task_create(&config);

// 注册到管理器 (向上转型为基类指针)
task_manager_register(manager, &my_task->base, "my_task");

// 启动任务 - 会创建线程执行execute函数
task_manager_start_task(manager, "my_task");

// 通过基类指针统一管理
TaskBase* base_task = task_manager_get_task(manager, "my_task");
TaskState state = task_get_state(base_task);  // 获取状态
```

## 关键特性

### 1. 统一的任务生命周期管理

```c
// 通过基类指针统一操作，实际调用的是子类实现
task_start(base_task);      // 启动任务线程
task_stop(base_task);       // 停止任务
task_restart(base_task);    // 重启任务
```

### 2. 线程安全的状态管理

```c
// 线程安全的状态检查
if (task_should_stop(base_task)) {
    // 任务应该退出
    break;
}

// 更新心跳
task_update_heartbeat(base_task);
```

### 3. 健康检查和自动重启

```c
// 在子类中重写健康检查
static bool my_task_health_check(TaskBase* base_task) {
    MyTask* task = TASK_CAST(MyTask, base_task);
    
    // 自定义健康检查逻辑
    if (task->my_data > MAX_VALUE) {
        return false;  // 不健康
    }
    
    return true;
}
```

### 4. 统计信息收集

```c
// 获取任务统计
const TaskStats* stats = task_get_stats(base_task);
printf("运行时间: %lu秒\n", stats->total_run_time);
printf("CPU使用率: %u%%\n", stats->cpu_usage);
```

## 编译和运行

```bash
# 编译任务演示程序
make flow_task

# 运行任务演示
make run_demo

# 或直接运行
cd build
./flow_task
```

## 演示程序命令

运行 flow_task 后，可以使用以下交互命令：

- `start task1` - 启动任务1
- `start task2` - 启动任务2  
- `stop task1` - 停止任务1
- `status task1` - 查看任务1详细状态
- `list` - 列出所有任务及状态
- `stats` - 查看管理器统计信息
- `health` - 执行健康检查
- `start_all` - 启动所有任务
- `stop_all` - 停止所有任务
- `quit` - 退出程序

## 设计优势

1. **面向对象设计**: 在C语言中实现了继承和多态
2. **类型安全**: 通过宏和类型转换确保安全性
3. **统一管理**: 通过基类指针统一管理不同类型的任务
4. **线程安全**: 内置互斥锁保护状态
5. **可扩展性**: 容易添加新的任务类型
6. **监控支持**: 内置健康检查、统计信息等
7. **自动重启**: 支持任务自动重启机制

这个设计完全符合你提到的需求：各个进程继承实现任务接口，然后利用父类指针指向子类对象来启动任务，`task_start()` 实际上就是启动线程执行任务的 `execute()` 方法。
