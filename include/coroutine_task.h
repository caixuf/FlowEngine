/**
 * @file coroutine_task.h
 * @brief C++20 协程任务基类 + MessageBus awaitable 适配器
 *
 * 提供两个核心组件：
 *
 * 1. BusAwaitable — 将 MessageBus 订阅包装成 C++20 awaitable，
 *    让协程可以 co_await 等待总线消息，而不占用调用线程。
 *
 * 2. CoroutineTask — 继承自 TaskBase（C 接口），execute() 内部
 *    启动协程事件循环，子类只需实现 run() 协程即可。
 *
 * 设计原则：
 *   - 不依赖外部协程框架（纯 C++20 标准库协程）
 *   - 保持 C 插件接口不变，协程支持作为可选 C++ 层叠加
 *   - 零额外系统线程：协程在调用 execute() 的线程中运行
 *
 * 使用示例：
 * @code
 *   class LidarTask : public CoroutineTask {
 *   protected:
 *       Task run() override {
 *           while (!should_stop()) {
 *               Message msg = co_await subscribe_once("sensor/lidar");
 *               process(msg);
 *           }
 *       }
 *   };
 * @endcode
 *
 * 若项目集成了 flowcoro，可将 BusAwaitable 的 resume 机制
 * 挂接到 flowcoro 的线程池调度器，实现跨线程协程恢复。
 */

#ifndef COROUTINE_TASK_H
#define COROUTINE_TASK_H

#if defined(__cplusplus) && __cplusplus >= 202002L

#include "task_interface.h"
#include "message_bus.h"

#include <coroutine>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <optional>
#include <string>
#include <stdexcept>
#include <cstring>

/* ─────────────────────────────────────────────────────────
 * 1. 基础 Task 协程类型
 * ───────────────────────────────────────────────────────── */

struct Task {
    struct promise_type {
        std::exception_ptr exception;

        Task get_return_object() {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_never  initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend()   noexcept { return {}; }
        void return_void() noexcept {}
        void unhandled_exception() { exception = std::current_exception(); }
    };

    explicit Task(std::coroutine_handle<promise_type> h) : handle_(h) {}

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    Task(Task&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    Task& operator=(Task&& o) noexcept {
        if (this != &o) {
            if (handle_) handle_.destroy();
            handle_ = o.handle_;
            o.handle_ = nullptr;
        }
        return *this;
    }

    ~Task() { if (handle_) handle_.destroy(); }

    bool done() const { return !handle_ || handle_.done(); }

    /** 重新抛出协程内部捕获的异常（如有） */
    void rethrow_if_exception() {
        if (handle_ && handle_.promise().exception) {
            std::rethrow_exception(handle_.promise().exception);
        }
    }

    std::coroutine_handle<promise_type> handle_;
};

/* ─────────────────────────────────────────────────────────
 * 2. BusAwaitable — co_await 等待一条总线消息
 * ───────────────────────────────────────────────────────── */

/**
 * co_await bus_awaitable("sensor/lidar") 会挂起当前协程，
 * 直到指定 topic 有消息到达后，由总线回调线程恢复协程。
 *
 * 线程安全：回调可能在总线分发线程调用，使用 mutex 保护状态。
 */
class BusAwaitable {
public:
    BusAwaitable(MessageBus* bus, const char* topic)
        : bus_(bus), topic_(topic) {}

    /* awaitable 协议 */
    bool await_ready() const noexcept { return false; /* 总是挂起 */ }

    void await_suspend(std::coroutine_handle<> handle) {
        handle_ = handle;
        /* 订阅一次性回调 */
        message_bus_subscribe(bus_, topic_.c_str(), &BusAwaitable::on_message, this);
    }

    Message await_resume() {
        message_bus_unsubscribe_ex(bus_, topic_.c_str(), &BusAwaitable::on_message, this);
        return received_msg_;
    }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<BusAwaitable*>(user_data);
        self->received_msg_ = *msg;
        /* 恢复协程（在回调线程中同步恢复） */
        auto h = self->handle_;
        if (h && !h.done()) h.resume();
    }

    MessageBus*                  bus_;
    std::string                  topic_;
    std::coroutine_handle<>      handle_;
    Message                      received_msg_{};
};

/**
 * 工厂函数：创建等待指定 topic 一条消息的 awaitable
 *
 * 使用：
 *   Message msg = co_await subscribe_once(bus, "sensor/lidar");
 */
inline BusAwaitable subscribe_once(MessageBus* bus, const char* topic) {
    return BusAwaitable{bus, topic};
}

/* ─────────────────────────────────────────────────────────
 * 3. CoroutineTask — C++ 协程任务基类
 *    继承自 TaskBase，兼容 C 插件接口
 * ───────────────────────────────────────────────────────── */

class CoroutineTask {
public:
    explicit CoroutineTask(MessageBus* bus = nullptr)
        : bus_(bus), stop_flag_(false) {}

    virtual ~CoroutineTask() = default;

    /** 子类实现具体协程逻辑 */
    virtual Task run() = 0;

    /**
     * 启动协程事件循环（阻塞直到协程完成或 stop() 被调用）
     *
     * 与 TaskBase::execute() 对应：在 task_execute() 线程中调用。
     */
    void execute() {
        stop_flag_ = false;
        task_ = std::make_unique<Task>(run());

        /* 简单轮询：等待协程完成或收到停止信号 */
        while (!task_->done() && !stop_flag_) {
            std::unique_lock<std::mutex> lk(coro_mutex_);
            coro_cv_.wait_for(lk, std::chrono::milliseconds(100));
        }

        task_->rethrow_if_exception();
    }

    /** 请求停止（可从任意线程调用） */
    void stop() {
        stop_flag_ = true;
        coro_cv_.notify_all();
    }

    bool should_stop() const { return stop_flag_; }

    MessageBus* bus() const { return bus_; }

    /** 唤醒事件循环（协程内部 await 恢复后调用，可选） */
    void notify() { coro_cv_.notify_all(); }

protected:
    MessageBus*                  bus_;
    std::atomic<bool>            stop_flag_;
    std::unique_ptr<Task>        task_;
    std::mutex                   coro_mutex_;
    std::condition_variable      coro_cv_;
};

/* ─────────────────────────────────────────────────────────
 * 4. C 包装宏：将 CoroutineTask 子类导出为 TaskBase 插件
 * ───────────────────────────────────────────────────────── */

/**
 * 在 .cpp 文件末尾调用此宏，自动生成 C 导出函数。
 *
 * 使用示例：
 * @code
 *   class MyCoroTask : public CoroutineTask { ... };
 *   EXPORT_COROUTINE_TASK(MyCoroTask, my_coro_task)
 * @endcode
 *
 * 生成：my_coro_task_create / my_coro_task_destroy / my_coro_task_get_base
 */
#define EXPORT_COROUTINE_TASK(ClassName, prefix)                              \
struct prefix##_Wrapper {                                                     \
    TaskBase       base;                                                      \
    ClassName*     impl;                                                      \
};                                                                            \
                                                                              \
static int prefix##_execute(TaskBase* b) {                                    \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    try { w->impl->execute(); return 0; }                                     \
    catch (...) { return -1; }                                                \
}                                                                             \
static void prefix##_cleanup(TaskBase* b) {                                   \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    w->impl->stop();                                                          \
}                                                                             \
static bool prefix##_health(TaskBase* b) {                                    \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    return !w->impl->should_stop();                                           \
}                                                                             \
static const TaskInterface prefix##_vtable = {                                \
    nullptr, prefix##_execute, prefix##_cleanup,                              \
    nullptr, nullptr, nullptr, prefix##_health, nullptr                       \
};                                                                            \
extern "C" {                                                                  \
prefix##_Wrapper* prefix##_create(const TaskConfig* cfg) {                    \
    auto* w = static_cast<prefix##_Wrapper*>(malloc(sizeof(prefix##_Wrapper))); \
    if (!w) return nullptr;                                                   \
    if (task_base_init(&w->base, &prefix##_vtable, cfg) != 0) {              \
        free(w); return nullptr;                                              \
    }                                                                         \
    w->impl = new ClassName();                                                \
    return w;                                                                 \
}                                                                             \
void prefix##_destroy(prefix##_Wrapper* w) {                                  \
    if (!w) return;                                                           \
    delete w->impl;                                                           \
    task_base_destroy(&w->base);                                              \
    free(w);                                                                  \
}                                                                             \
TaskBase* prefix##_get_base(prefix##_Wrapper* w) {                            \
    return w ? &w->base : nullptr;                                            \
}                                                                             \
}

#else
/* C++20 不可用时提供空头文件，避免编译错误 */
#warning "coroutine_task.h requires C++20 or later"
#endif /* C++20 */

#endif /* COROUTINE_TASK_H */
