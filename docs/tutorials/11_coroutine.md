# 11 — C++20 协程通信原语

## 问题

自动驾驶 / 机器人节点需要「等传感器数据、等应答、等超时、多路选择」。
传统回调写法把一条逻辑打散到多个函数，状态靠成员变量维护（回调地狱）：

```cpp
// 旧方式 — 逻辑被切碎，停机 / 超时 / 取消都要手工织进回调
void on_lidar(const Message* m, void* self) { /* 存状态，等 gps... */ }
void on_gps  (const Message* m, void* self) { /* 再拼起来... */ }
```

而且「协程正阻塞在 `co_await recv()` 时如何干净停机」是真实缺口：
若只置一个 `stop_flag`，悬挂的协程要靠外部再发一条消息才会醒来检查。

## 解决方案

**可取消 awaitable 家族**：把 MessageBus 的 pub/sub、select、timer、
req/reply 四类原语都包装成 C++20 awaitable，并统一接入一个
`CancelToken` + 共享 `AwaitCtl`（CAS 恢复守卫），使得：

- `stop()` 直接唤醒悬挂协程，**无需外发消息**即可干净退出；
- 消息 / 取消 / 超时三条恢复路径竞争同一挂起点，**至多恢复一次**；
- 超时、定时、请求应答均以「不占线程」的方式挂起。

```
co_await next(topic)             pub/sub  —— 等一条消息
co_await select({t1,t2})         select   —— 多路竞争，谁先到谁唤醒
co_await sleep_ms(n)             timer    —— 定时挂起（不占线程）
co_await ask(topic,...)          req/reply—— 请求并等待应答
        + _for(timeout) 变体     deadline —— 传感器丢帧 / 看门狗
```

## 快速开始

继承 `CoroutineTask`（纯 C++20）或 `FlowCoroTask`（接入 flowcoro 无锁线程池，
需 `-DFLOWCORO_INTEGRATION`），在 `run()` 里用成员工厂方法，它们会**自动注入
本任务的 CancelToken**，从而天然支持优雅停机：

```cpp
#include "coroutine_task.h"

class LidarTask : public FlowCoroTask {
protected:
    Task run() override {
        while (!should_stop()) {
            // 50ms 内没数据就当作丢帧（看门狗），stop() 也会立刻唤醒
            auto r = co_await next_for("sensor/lidar", 50'000);
            if (r.cancelled()) break;        // stop() 触发，干净退出
            if (r.timed_out()) { watchdog(); continue; }
            process(*r);                     // r.message 为收到的消息
        }
    }
};

EXPORT_COROUTINE_TASK(LidarTask, lidar_task)
```

多路选择 + 定时 + 请求应答：

```cpp
Task run() override {
    // 哪个 topic 先到就被哪个唤醒，其余订阅立即注销
    auto r = co_await select_for({"sensor/lidar", "sensor/gps"}, 20'000);

    co_await sleep_ms(5);                    // 定时挂起，不占线程

    uint32_t req = 42;
    auto rep = co_await ask("svc/echo", "lidar", &req, sizeof(req), /*timeout_ms*/100);
    if (rep.ok()) use(*rep);                 // 收到应答
    else if (rep.timed_out()) retry();
}
```

## 优雅停机（核心正确性保证）

`stop()` 做三件事，任意线程可调用：

1. 置 `stop_flag_`；
2. `CancelToken::request_cancel()` —— 唤醒所有悬挂 awaitable，它们以
   `AwaitStatus::Cancelled` 返回；
3. `notify()` 唤醒事件循环。

因此**悬挂在 `co_await` 上的协程无需外部再发消息**即可在下一个挂起点恢复、
检查到取消并 `break` 退出。停机路径由 `co_await` 结果的 `cancelled()` 显式表达。

## 可观测性

每个协程任务内置 `CoroStats`（复用 `scheduler.h` 的 `LatencyTracker`）：

```cpp
task.resume_count();     // 累计恢复次数
task.coro_latency();     // 挂起时长 LatencyStats（avg / p50 / p99 / min / max）
```

宿主 / flowctl 可读取这些统计以观测协程行为（每次挂起→恢复的时长分布）。

## 所有权与生命周期规则

C / C++ / 协程混用时务必遵守（详见 `include/coroutine_task.h` 顶部注释）：

1. **MessageBus 生命周期必须长于**引用它的 Task 及其协程帧。销毁顺序：
   先确保协程结束 / 取消且 `execute()` 返回，再 `message_bus_destroy(bus)`，
   最后析构 Task。
2. awaitable 在挂起时向 bus / CancelToken / TimerService 注册，在恢复或
   析构时反注册；协程帧销毁会依次析构挂起点 awaitable，从而在 bus 仍存活时
   完成反注册 —— 故规则 (1) 的顺序是必要条件。
3. 一次挂起至多恢复一次：三条恢复路径共享一个 `shared_ptr<AwaitCtl>`，
   即便 awaitable 已析构，仍在途的定时器 / 取消回调也不会悬垂访问。
4. 阻塞型原语（`ask` / `run_blocking`）只捕获值拷贝与共享控制块，绝不捕获
   `this`，被取消的协程销毁 awaitable 后迟到的结果不会访问已释放对象。

## API 速查

| 方法（`CoroutineTask` 成员） | 语义 | 返回 |
|------|------|------|
| `co_await next(topic)` | 等一条消息 | `Message` |
| `co_await next_for(topic, us)` | 带超时等一条消息 | `AwaitResult` |
| `co_await select({t...})` | 多路竞争等待 | `Message` |
| `co_await select_for({t...}, us)` | 带超时多路竞争 | `AwaitResult` |
| `co_await sleep_us/sleep_ms(n)` | 定时挂起 | `bool`（false=被取消） |
| `co_await ask(topic, sender, data, size, timeout_ms)` | 请求/应答 | `AwaitResult` |
| `stop()` | 请求停机（唤醒悬挂协程，无需外发消息） | — |
| `should_stop()` | 是否已请求停机 | `bool` |
| `resume_count()` / `coro_latency()` | 可观测统计 | — |

`AwaitResult`：`ok()` / `timed_out()` / `cancelled()`；`*r` / `r->` 访问 `message`。

自由函数（需显式传 `CancelToken*` 才可取消）：`subscribe_once`、
`subscribe_once_for`、`when_any_bus`、`when_any_bus_for`、`delay_ms`、`request`。

## 参考

- 实现：`include/coroutine_task.h`
- 示例：`src/coro_bus_demo.cpp`、`src/plugins/flowcoro_task.cpp`
- 正确性 / 压力测试：`test/coro_correctness_test.cpp`（不丢帧、when_any 只唤醒
  一次、优雅停机、超时、定时、请求应答、并发压力 + 可观测性）
