# Skill 03 - 消息总线与发布/订阅

## 核心思想

消息总线（Message Bus）提供**发布/订阅**（Pub/Sub）通信模型：发布者把消息发到某个**话题（topic）**，订阅了该话题的接收者自动收到通知。双方不直接依赖，实现**松耦合**。

## 架构

```
  发布者 A ──publish("sensor/gps", &msg)──▶ MessageBus
  发布者 B ──publish("sensor/imu", &msg)──▶     │
                                                 │ 路由
  订阅者 X ◀──on_message callback───────────────┤ ("sensor/*")
  订阅者 Y ◀──on_message callback───────────────┘ ("sensor/gps")
```

## 核心 API

```c
#include "message_bus.h"

/* 创建消息总线 */
MessageBus* bus = message_bus_create();

/* 订阅话题 */
message_bus_subscribe(bus, "sensor/gps", my_callback, user_data);

/* 发布消息 */
SensorMsg msg = { .lat = 39.9, .lon = 116.4 };
message_bus_publish(bus, "sensor/gps", &msg, sizeof(msg));

/* 销毁 */
message_bus_destroy(bus);
```

## 回调函数签名

```c
typedef void (*MessageCallback)(const char* topic,
                                const void* data,
                                size_t      size,
                                void*       user_data);

void my_callback(const char* topic,
                 const void* data,
                 size_t      size,
                 void*       user_data)
{
    const SensorMsg* msg = (const SensorMsg*)data;
    printf("收到 %s: lat=%.4f lon=%.4f\n", topic, msg->lat, msg->lon);
}
```

## 消息驱动任务（替代 sleep 轮询）

传统任务需要 `sleep` 轮询：

```c
// 低效：不管有没有数据，每秒检查一次
static int poll_execute(TaskBase* base) {
    while (!base->should_stop) {
        check_and_process();
        sleep(1);   // 浪费 CPU 或延迟高
    }
    return 0;
}
```

消息驱动任务由 `on_message` 回调触发，**零延迟、零空转**：

```c
static void reactive_on_message(TaskBase* base, const void* msg) {
    // 有消息才执行，无消息不占 CPU
    process(msg);
}

static int reactive_execute(TaskBase* base) {
    // 订阅并阻塞，由回调驱动
    task_subscribe(base, g_bus, "sensor/data");
    // execute 线程在此阻塞，直到 should_stop
    while (!base->should_stop) pause();
    return 0;
}

static const TaskInterface reactive_vtable = {
    .execute    = reactive_execute,
    .on_message = reactive_on_message,
    /* ... */
};
```

## 线程安全

消息总线内部使用互斥锁保护订阅列表，`publish` 和 `subscribe` 可在不同线程并发调用。回调函数在**发布者线程**中执行，若回调耗时较长建议异步处理（投递到队列）：

```c
void my_callback(const char* topic, const void* data, size_t size, void* ud) {
    // 不要在回调里做阻塞操作！
    // 将消息拷贝到队列，由工作线程处理
    enqueue(my_queue, data, size);
}
```

## 话题命名规范

推荐使用 `/` 分层命名，便于未来支持通配符订阅：

```
sensor/gps          GPS 数据
sensor/imu          IMU 数据
control/cmd         控制指令
system/heartbeat    心跳
log/error           错误日志
```

## 参考文件

- `include/message_bus.h` — API 定义
- `src/core/message_bus.c` — 实现
- `src/bus_demo.c` — 完整演示（多发布者/多订阅者）
- `src/plugins/reactive_task.c` — 消息驱动任务插件示例
