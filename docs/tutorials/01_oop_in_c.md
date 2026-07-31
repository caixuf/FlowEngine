# Skill 01 - C 语言面向对象编程

## 核心思想

C 语言没有原生的类和继承，但可以通过**结构体嵌套**和**函数指针**模拟面向对象的三大特性：封装、继承、多态。FlowEngine 的任务系统完整地运用了这套模式。

## 封装

把数据和操作数据的函数放在同一个结构体里（通过函数指针）：

```c
typedef struct {
    char name[64];
    int  port;
    // 操作数据的函数指针（方法）
    int  (*start)(struct MyService* self);
    void (*stop) (struct MyService* self);
} MyService;
```

## 继承（结构体嵌套）

将"父类"结构体作为子结构体的**第一个成员**，这样父子指针可以安全互转：

```c
// 父类
typedef struct TaskBase {
    TaskConfig config;
    TaskState  state;
    const struct TaskInterface* vtable;  // 虚函数表
    pthread_t  thread;
    bool       should_stop;
} TaskBase;

// 子类 —— base 必须是第一个成员
typedef struct {
    TaskBase base;   // "继承" TaskBase
    int      port;
    char*    db_url;
} NetworkService;

// 安全地向下转型
NetworkService* svc = (NetworkService*)base_ptr;
```

## 多态（虚函数表）

用函数指针结构体模拟 C++ 的 `vtable`：

```c
// 虚函数表定义
typedef struct TaskInterface {
    int  (*initialize)  (TaskBase* task);
    int  (*execute)     (TaskBase* task);
    void (*cleanup)     (TaskBase* task);
    bool (*health_check)(TaskBase* task);
    void (*on_message)  (TaskBase* task, const void* msg);
} TaskInterface;

// 每个"子类"提供自己的 vtable 实现
static const TaskInterface network_vtable = {
    .initialize   = network_init,
    .execute      = network_execute,
    .cleanup      = network_cleanup,
    .health_check = network_health_check,
};

// 调用虚函数（多态分发）
base->vtable->execute(base);

// 或使用项目中定义的宏
TASK_CALL(base, execute);
```

## 便利宏

```c
// 调用有返回值的虚函数（函数不存在时返回 -1）
#define TASK_CALL(task, method, ...) \
    ((task)->vtable && (task)->vtable->method ? \
     (task)->vtable->method((task), ##__VA_ARGS__) : -1)

// 调用无返回值的虚函数
#define TASK_CALL_VOID(task, method, ...) \
    do { \
        if ((task)->vtable && (task)->vtable->method) \
            (task)->vtable->method((task), ##__VA_ARGS__); \
    } while(0)

// 向下转型
#define TASK_CAST(type, task) ((type*)(task))
```

## 工厂模式（配合 dlopen）

每个插件 `.so` 导出统一的工厂函数，让框架可以在不知道具体类型的情况下创建对象：

```c
// 插件必须导出此符号
TaskBase* create_task(const TaskConfig* config) {
    NetworkService* svc = calloc(1, sizeof(NetworkService));
    task_base_init(&svc->base, &network_vtable, config);
    svc->port   = 8080;
    svc->db_url = NULL;
    return &svc->base;   // 返回父类指针
}
```

## 关键要点

1. **父类成员必须是第一个** — 这保证了父子指针值相同，转型安全。
2. **vtable 使用 `const` 静态变量** — 每个"类"只有一份 vtable，节省内存。
3. **可选虚函数设为 NULL** — 框架调用前先检查是否为 NULL，允许子类只实现必要方法。
4. **线程安全** — `TaskBase` 内置 `pthread_mutex_t`，子类无需重复加锁。

## 参考文件

- `include/task_interface.h` — TaskBase / TaskInterface 定义
- `src/core/task_interface.c` — 基类通用实现
- `src/plugins/example_task.c` — 最简 C 语言插件示例
- `src/plugins/reactive_task.c` — 消息驱动插件示例
