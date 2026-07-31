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

## 核心 API（当前版本）

```c
#include "clock_service.h"

/* === 获取当前时间（微秒）=== */
uint64_t now = clock_now_us();              // CLOCK_MONOTONIC，仿真模式下可注入
uint64_t real = clock_now_realtime_us();    // CLOCK_REALTIME，始终真实墙钟

/* === 仿真模式控制 === */
clock_set_sim_mode(true);                   // 启用仿真时间
clock_set_sim_time(timestamp_us);           // 设置仿真时间
clock_advance_us(delta_us);                 // 推进 delta 微秒
bool sim = clock_is_sim_mode();             // 查询是否仿真模式

/* === 仿真步长 === */
clock_set_step_us(20000);                   // 设置仿真步长（如 20ms @ 50Hz）
uint64_t step = clock_get_step_us();        // 获取仿真步长
```

## 使用示例

### 正常模式（生产环境）

```c
#include "clock_service.h"

uint64_t t0 = clock_now_us();
do_work();
uint64_t elapsed_us = clock_now_us() - t0;
printf("耗时 %lu us\n", (unsigned long)elapsed_us);
```

### 仿真模式（SimWorld 初始化）

```c
// sim_world_node.c 中已使用:
#include "clock_service.h"

clock_set_step_us((uint64_t)(1e6 / frequency_hz));  // 如 50Hz → 20000us
// 主循环中每 tick:
uint64_t sim_time_us = cycle * step_us;
clock_set_sim_time(sim_time_us);
```

### Bag 回放

```c
#include "clock_service.h"

clock_set_sim_mode(true);
clock_set_sim_time(bag_start_time_us);

while (bag_next(reader, &rec)) {
    clock_set_sim_time(rec.timestamp_us);
    message_bus_publish(bus, rec.topic, rec.data, rec.size);
}

clock_set_sim_mode(false);  // 恢复真实时间
```

## 实现要点

1. **线程安全** — 使用 `pthread_mutex_t` 保护全局状态。
2. **单调性** — 正常模式使用 `CLOCK_MONOTONIC`，不受系统时间跳变影响。
3. **仿真模式** — `clock_set_sim_mode(true)` 后 `clock_now_us()` 返回注入的时间戳，不影响 `clock_now_realtime_us()`。
4. **零开销** — 正常模式下仅一次 mutex lock + clock_gettime，仿真模式下仅 mutex lock。

## 禁止行为

- ❌ **禁止** 在模块中 `static uint64_t monotonic_us(void) { clock_gettime(CLOCK_MONOTONIC, &ts); ... }` 重复造轮子
- ❌ **禁止** 直接调用 `clock_gettime(CLOCK_MONOTONIC, &ts)`
- ✅ **必须** 使用 `clock_now_us()`；需绝对时间（文件时间戳等）使用 `clock_now_realtime_us()`

## 参考文件

- `include/clock_service.h` — API 定义
- `src/core/clock_service.c` — 实现
- `modules/adas_nodes/sim_world_node.c` — 仿真模式参考示例
- `src/bag_demo.c` — Bag 回放参考示例
