# Skill 06 - 统一时钟服务（Clock Service）

## 核心思想

系统中所有模块通过**同一个时钟接口**获取当前时间，而不是直接调用 `clock_gettime` / `gettimeofday`。这样只需在一处切换时钟模式，即可让整个系统在**真实时钟**和**模拟时钟**之间无缝切换。

## 为什么需要统一时钟

| 场景 | 问题 | 统一时钟的解决方案 |
|------|------|--------------------|
| Bag 回放 | 系统时间与录制时间不一致，算法产生错误结果 | 将模拟时钟拨到 bag 起始时间 |
| 加速/减速测试 | 真实时间无法控制速度 | 模拟时钟以任意倍速推进 |
| 单元测试 | 依赖真实时间的代码难以测试 | 固定时钟时间，使测试可重复 |
| 时间同步 | 多机系统时钟不一致 | 集中管理，统一分发 |

## 时钟模式

```c
typedef enum {
    CLOCK_MODE_REAL,   // 使用系统真实时钟（默认）
    CLOCK_MODE_SIM,    // 使用模拟时钟（手动推进）
} ClockMode;
```

## 核心 API

```c
#include "clock_service.h"

/* === 初始化 === */
clock_service_init();

/* === 切换模式 === */
clock_service_set_mode(CLOCK_MODE_REAL);   // 真实时钟
clock_service_set_mode(CLOCK_MODE_SIM);    // 模拟时钟

/* === 获取当前时间（纳秒时间戳）=== */
uint64_t now = clock_service_now();        // 所有模块都调这一个函数

/* === 模拟时钟操作 === */
clock_service_set_sim_time(timestamp_ns);  // 设置模拟时间
clock_service_advance(delta_ns);           // 推进指定纳秒数

/* === 清理 === */
clock_service_destroy();
```

## 使用示例

### 真实时钟（生产环境）

```c
clock_service_init();
// 默认就是真实时钟模式，无需额外操作

uint64_t t0 = clock_service_now();
do_work();
uint64_t elapsed = clock_service_now() - t0;
printf("耗时 %lu ms\n", elapsed / 1000000UL);
```

### 模拟时钟（Bag 回放）

```c
clock_service_init();
clock_service_set_mode(CLOCK_MODE_SIM);

BagReader* reader = bag_reader_open("data.bag");

// 将模拟时钟设为 bag 起始时间
clock_service_set_sim_time(bag_reader_start_time(reader));

// 回放时每条消息都推进时钟
BagRecord rec;
while (bag_reader_next(reader, &rec)) {
    clock_service_set_sim_time(rec.timestamp);  // 拨到消息时间
    message_bus_publish(bus, rec.topic, rec.data, rec.size);
}
```

### 加速回放

```c
// 以 5× 速度回放：每 200ms 真实时间 = 1000ms 模拟时间
float speed = 5.0f;
uint64_t prev_real = get_real_time_ns();
uint64_t sim_time  = bag_reader_start_time(reader);

BagRecord rec;
while (bag_reader_next(reader, &rec)) {
    uint64_t real_now   = get_real_time_ns();
    uint64_t real_delta = real_now - prev_real;
    prev_real = real_now;

    sim_time += (uint64_t)(real_delta * speed);
    clock_service_set_sim_time(sim_time);

    message_bus_publish(bus, rec.topic, rec.data, rec.size);
}
```

## 单元测试中的时钟控制

```c
void test_timeout_logic(void) {
    clock_service_set_mode(CLOCK_MODE_SIM);
    clock_service_set_sim_time(0);

    start_watchdog();   // 看门狗：超过 5s 没有心跳则报警

    // 推进 4s，不应触发超时
    clock_service_advance(4ULL * 1e9);
    assert(!is_timeout_triggered());

    // 再推进 2s，应触发超时
    clock_service_advance(2ULL * 1e9);
    assert(is_timeout_triggered());

    clock_service_set_mode(CLOCK_MODE_REAL);  // 恢复
}
```

## 实现要点

1. **线程安全** — 模拟时钟使用原子变量存储时间戳，读写无锁。
2. **精度** — 内部使用 `uint64_t` 纳秒时间戳，避免浮点精度损失。
3. **回调通知**（可选）— 模拟时钟推进时可通知注册的监听者（如定时器）。
4. **单调性** — 模拟时钟只允许向前推进，防止时间回退导致的逻辑错误。

## 参考文件

- `include/clock_service.h` — API 定义
- `src/core/clock_service.c` — 实现
- `src/flow_bag.c` — 统一时钟与 Bag 回放配合的完整示例
