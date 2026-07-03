/**
 * @file coroutine_task.h
 * @brief C++20 协程任务基类 + MessageBus awaitable 适配器
 *
 * 提供三个核心组件：
 *
 * 1. BusAwaitable — 将 MessageBus 订阅包装成 C++20 awaitable，
 *    让协程可以 co_await 等待总线消息，而不占用调用线程。
 *
 * 2. CoroutineTask — 继承自 TaskBase（C 接口），execute() 内部
 *    启动协程事件循环，子类只需实现 run() 协程即可。
 *
 * 3. FlowCoroTask（需要 FLOWCORO_INTEGRATION）— CoroutineTask 的子类，
 *    将协程的 resume 操作提交到 flowcoro 无锁线程池调度器，
 *    实现真正的跨线程协程恢复，解除消息总线分发线程的阻塞。
 *
 * 设计原则：
 *   - 默认不依赖外部协程框架（纯 C++20 标准库协程）
 *   - 通过编译宏 FLOWCORO_INTEGRATION 可选接入 flowcoro 线程池
 *     及 flowcoro::CoroTask 协程类型（惰性启动 + continuation 链式唤醒）
 *   - 保持 C 插件接口不变，协程支持作为可选 C++ 层叠加
 *   - FlowCoroTask：消息总线线程仅触发回调，不执行协程逻辑
 *
 * 使用示例（不依赖 flowcoro）：
 * @code
 *   class LidarTask : public CoroutineTask {
 *   protected:
 *       Task run() override {
 *           while (!should_stop()) {
 *               Message msg = co_await subscribe_once(bus(), "sensor/lidar");
 *               process(msg);
 *           }
 *       }
 *   };
 * @endcode
 *
 * 使用示例（集成 flowcoro 线程池调度器）：
 * @code
 *   // 编译时需要 -DFLOWCORO_INTEGRATION
 *   class LidarTask : public FlowCoroTask {
 *   protected:
 *       Task run() override {
 *           while (!should_stop()) {
 *               // resume 在 flowcoro 线程池中执行，不阻塞总线分发线程
 *               Message msg = co_await subscribe_once(bus(), "sensor/lidar");
 *               process(msg);
 *           }
 *       }
 *   };
 * @endcode
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
#include <atomic>
#include <thread>
#include <chrono>
#include <queue>
#include <vector>
#include <initializer_list>
#include <iostream>   /* required by flowcoro/thread_pool.h (std::cout/cerr) */

/* ─────────────────────────────────────────────────────────
 * 可选：flowcoro 无锁线程池 + CoroTask
 * 编译时加 -DFLOWCORO_INTEGRATION 启用
 * ───────────────────────────────────────────────────────── */
#ifdef FLOWCORO_INTEGRATION
#include <flowcoro/thread_pool.h>

/* flowcoro/logger.h 使用 PROJECT_SOURCE_DIR（CMake 生成的宏）作为
 * 若干静态函数的默认参数。以外部依赖形式引入时该宏未定义，
 * 在此提供默认值以保证头文件正常编译。                           */
#ifndef PROJECT_SOURCE_DIR
#  define PROJECT_SOURCE_DIR ""
#endif
/* coro_task.h 使用 CoroutineManager 但未自行包含其头文件，
 * 须在包含 coro_task.h 前显式引入。                              */
#include <flowcoro/coroutine_manager.h>
#include <flowcoro/coro_task.h>

namespace flowcoro_integration {

/**
 * 全局 flowcoro 线程池单例。
 * 线程数默认为 hardware_concurrency()，可在 main() 前通过
 * get_thread_pool() 的延迟初始化自动设置。
 */
inline lockfree::ThreadPool& get_thread_pool() {
    static lockfree::ThreadPool pool{std::thread::hardware_concurrency()};
    return pool;
}

} // namespace flowcoro_integration

/* ─────────────────────────────────────────────────────────
 * 1. 协程任务类型（FLOWCORO_INTEGRATION 模式）
 *
 * 直接使用 flowcoro::CoroTask，享用其完整特性：
 *   - initial_suspend = suspend_always（惰性启动，execute() 负责首次调度）
 *   - final_suspend   = FinalAwaiter（对称转移，支持 co_await task）
 *   - 内置异常捕获（promise.exception_），可通过
 *     handle.promise().exception_ 获取并重抛
 * ───────────────────────────────────────────────────────── */
using Task = flowcoro::CoroTask;

#else  /* !FLOWCORO_INTEGRATION */

/* ─────────────────────────────────────────────────────────
 * 1. 基础 Task 协程类型（纯 C++20 标准库，无外部依赖）
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

#endif /* FLOWCORO_INTEGRATION */

/* ─────────────────────────────────────────────────────────
 * 2. BusAwaitable — co_await 等待一条总线消息
 * ───────────────────────────────────────────────────────── */

/**
 * co_await subscribe_once(bus, "sensor/lidar") 会挂起当前协程，
 * 直到指定 topic 有消息到达后恢复协程。
 *
 * 线程安全：
 *   - 默认模式：回调在总线分发线程中直接 resume 协程（可能阻塞分发）。
 *   - FLOWCORO_INTEGRATION 模式：回调将 handle.resume() 提交到
 *     flowcoro 线程池，分发线程立即返回，协程在工作线程中运行。
 */
class BusAwaitable {
public:
    BusAwaitable(MessageBus* bus, const char* topic)
        : bus_(bus), topic_(topic) {}

    /* awaitable 协议 */
    bool await_ready() const noexcept { return false; /* 总是挂起 */ }

    void await_suspend(std::coroutine_handle<> handle) {
        handle_ = handle;
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
        auto h = self->handle_;
        if (!h || h.done()) return;

#ifdef FLOWCORO_INTEGRATION
        /* ── flowcoro 模式：将协程 resume 提交到线程池 ───────
         * 总线分发线程立刻返回，不被协程逻辑阻塞。
         * 协程在 flowcoro 工作线程上继续执行。               */
        flowcoro_integration::get_thread_pool().enqueue_void(
            [h]() mutable { if (!h.done()) h.resume(); });
#else
        /* ── 标准模式：在分发线程上同步 resume ─────────────── */
        h.resume();
#endif
    }

    MessageBus*              bus_;
    std::string              topic_;
    std::coroutine_handle<>  handle_;
    Message                  received_msg_{};
};

/**
 * 工厂函数：创建等待指定 topic 一条消息的 awaitable
 *
 * 用法：
 *   Message msg = co_await subscribe_once(bus, "sensor/lidar");
 */
inline BusAwaitable subscribe_once(MessageBus* bus, const char* topic) {
    return BusAwaitable{bus, topic};
}

/* ─────────────────────────────────────────────────────────
 * 2b. BusChannel — 持久订阅 + 消息缓冲通道
 * ───────────────────────────────────────────────────────── */

/**
 * BusChannel 在构造时订阅指定 topic，并将到达的消息缓冲在内部队列中。
 * 协程可以循环调用 co_await ch.recv() 来逐条消费消息，而不必每次
 * 重新订阅/取消订阅总线，大幅减少订阅开销。
 *
 * 对比 subscribe_once：
 *   - subscribe_once：每次 co_await 都要注册+注销一次回调，适合"等一条就走"。
 *   - BusChannel   ：一次订阅持续存活，消息批量缓冲，适合持续消费场景。
 *
 * 用法：
 *   BusChannel lidar_ch(bus, "sensor/lidar", 32);
 *   while (!should_stop()) {
 *       Message msg = co_await lidar_ch.recv();
 *       process(msg);
 *   }
 */
class BusChannel {
public:
    BusChannel(MessageBus* bus, const char* topic, size_t capacity = 32)
        : bus_(bus), topic_(topic), capacity_(capacity) {
        message_bus_subscribe(bus_, topic_.c_str(), &BusChannel::on_message, this);
    }

    ~BusChannel() {
        message_bus_unsubscribe_ex(bus_, topic_.c_str(), &BusChannel::on_message, this);
    }

    /* 禁止拷贝（订阅指针指向 this，不能移动） */
    BusChannel(const BusChannel&) = delete;
    BusChannel& operator=(const BusChannel&) = delete;

    /** 返回等待下一条消息的 awaitable，供 co_await 使用 */
    auto recv() { return RecvAwaitable{this}; }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<BusChannel*>(user_data);
        std::coroutine_handle<> waiter;
        {
            std::lock_guard<std::mutex> lk(self->mutex_);
            if (self->buffer_.size() < self->capacity_) {
                self->buffer_.push(*msg);
            }
            waiter = self->waiter_;
            self->waiter_ = nullptr;
        }
        if (!waiter || waiter.done()) return;

#ifdef FLOWCORO_INTEGRATION
        flowcoro_integration::get_thread_pool().enqueue_void(
            [waiter]() mutable { if (!waiter.done()) waiter.resume(); });
#else
        waiter.resume();
#endif
    }

    struct RecvAwaitable {
        BusChannel* ch;

        bool await_ready() {
            std::lock_guard<std::mutex> lk(ch->mutex_);
            return !ch->buffer_.empty();
        }

        void await_suspend(std::coroutine_handle<> h) {
            bool should_resume = false;
            {
                std::lock_guard<std::mutex> lk(ch->mutex_);
                /* 检查在 await_ready 到 await_suspend 之间是否有消息到达 */
                if (!ch->buffer_.empty()) {
                    should_resume = true;
                } else {
                    ch->waiter_ = h;
                }
            }
            if (should_resume) {
#ifdef FLOWCORO_INTEGRATION
                flowcoro_integration::get_thread_pool().enqueue_void(
                    [h]() mutable { if (!h.done()) h.resume(); });
#else
                h.resume();
#endif
            }
        }

        Message await_resume() {
            std::lock_guard<std::mutex> lk(ch->mutex_);
            Message msg = ch->buffer_.front();
            ch->buffer_.pop();
            return msg;
        }
    };

    MessageBus*              bus_;
    std::string              topic_;
    size_t                   capacity_;
    std::queue<Message>      buffer_;
    std::mutex               mutex_;
    std::coroutine_handle<>  waiter_{};
};

/* ─────────────────────────────────────────────────────────
 * 2c. WhenAnyBusAwaitable — 多 topic 竞争等待
 * ───────────────────────────────────────────────────────── */

/**
 * WhenAnyBusAwaitable 同时订阅多个 topic，哪个 topic 的消息先到，
 * 协程就被哪个 topic 唤醒，其余订阅立即注销。
 * 这是"多传感器竞争等待"的惯用模式，等价于 Go 的 select 或
 * C++ 提案中的 when_any。
 *
 * 用法：
 *   Message msg = co_await when_any_bus(bus, {"sensor/lidar", "sensor/gps"});
 *   // msg.topic 中可查看是哪个 topic 触发的
 */
class WhenAnyBusAwaitable {
public:
    WhenAnyBusAwaitable(MessageBus* bus, std::vector<std::string> topics)
        : bus_(bus), topics_(std::move(topics)) {}

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) {
        handle_ = h;
        fired_.store(false, std::memory_order_release);
        for (const auto& t : topics_) {
            message_bus_subscribe(bus_, t.c_str(), &WhenAnyBusAwaitable::on_message, this);
        }
    }

    Message await_resume() {
        /* 注销所有剩余订阅 */
        for (const auto& t : topics_) {
            message_bus_unsubscribe_ex(bus_, t.c_str(), &WhenAnyBusAwaitable::on_message, this);
        }
        return received_msg_;
    }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<WhenAnyBusAwaitable*>(user_data);
        /* 原子 CAS：只有第一个到达的 topic 能触发 resume */
        bool expected = false;
        if (!self->fired_.compare_exchange_strong(expected, true,
                                                  std::memory_order_acq_rel)) {
            return; /* 已被其他 topic 率先触发 */
        }
        self->received_msg_ = *msg;
        auto h = self->handle_;
        if (!h || h.done()) return;

#ifdef FLOWCORO_INTEGRATION
        flowcoro_integration::get_thread_pool().enqueue_void(
            [h]() mutable { if (!h.done()) h.resume(); });
#else
        h.resume();
#endif
    }

    MessageBus*              bus_;
    std::vector<std::string> topics_;
    std::coroutine_handle<>  handle_{};
    std::atomic<bool>        fired_{false};
    Message                  received_msg_{};
};

/**
 * 工厂函数：等待给定 topic 列表中最先到达的那条消息。
 *
 * 用法：
 *   Message msg = co_await when_any_bus(bus, {"sensor/lidar", "sensor/gps"});
 */
inline WhenAnyBusAwaitable when_any_bus(MessageBus* bus,
                                        std::initializer_list<const char*> topics) {
    return WhenAnyBusAwaitable{bus, std::vector<std::string>(topics.begin(), topics.end())};
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
     * 每 10ms 轮询一次，兼顾响应速度与 CPU 开销。
     *
     * FLOWCORO_INTEGRATION 模式：
     *   flowcoro::CoroTask 使用 suspend_always（惰性启动），
     *   execute() 负责在调用线程上执行第一次 resume，
     *   使协程跑到第一个 co_await 挂起点后返回。
     */
    virtual void execute() {
        stop_flag_ = false;
        task_ = std::make_unique<Task>(run());

#ifdef FLOWCORO_INTEGRATION
        /* flowcoro::CoroTask 惰性启动：在调用线程上触发首次 resume，
         * 协程跑到第一个 co_await 后挂起，控制权回到此处。 */
        if (task_->handle && !task_->handle.done()) {
            task_->handle.resume();
        }
#endif

        while (!task_->done() && !stop_flag_) {
            std::unique_lock<std::mutex> lk(coro_mutex_);
            coro_cv_.wait_for(lk, std::chrono::milliseconds(10));
        }

#ifdef FLOWCORO_INTEGRATION
        if (task_->handle && task_->handle.promise().exception_) {
            std::rethrow_exception(task_->handle.promise().exception_);
        }
#else
        task_->rethrow_if_exception();
#endif
    }

    /** 请求停止（可从任意线程调用） */
    void stop() {
        stop_flag_ = true;
        coro_cv_.notify_all();
    }

    bool should_stop() const { return stop_flag_; }

    MessageBus* bus() const { return bus_; }

    /** 唤醒事件循环（协程 resume 后调用，可加快响应） */
    void notify() { coro_cv_.notify_all(); }

protected:
    MessageBus*                  bus_;
    std::atomic<bool>            stop_flag_;
    std::unique_ptr<Task>        task_;
    std::mutex                   coro_mutex_;
    std::condition_variable      coro_cv_;
};

/* ─────────────────────────────────────────────────────────
 * 4. FlowCoroTask — 基于 flowcoro 线程池的协程任务基类
 *    仅在 FLOWCORO_INTEGRATION 启用时可用
 * ───────────────────────────────────────────────────────── */

#ifdef FLOWCORO_INTEGRATION
/**
 * FlowCoroTask 在 CoroutineTask 基础上：
 *   - Task 类型为 flowcoro::CoroTask（惰性启动，支持 continuation 链）。
 *   - execute() 将协程的首次 resume 提交到 flowcoro 线程池，
 *     协程体从一开始就在工作线程上执行，不阻塞调用线程。
 *   - BusAwaitable 将后续每次 resume 也提交到线程池，
 *     消息总线分发线程永不被协程逻辑阻塞。
 *   - execute() 以 5ms 轮询等待协程完成，CPU 负载极低。
 *
 * 使用方式与 CoroutineTask 完全相同，只需改继承类即可。
 */
class FlowCoroTask : public CoroutineTask {
public:
    explicit FlowCoroTask(MessageBus* bus = nullptr)
        : CoroutineTask(bus) {
        // 确保线程池在首次使用前已初始化
        (void)flowcoro_integration::get_thread_pool();
    }

    void execute() override {
        stop_flag_ = false;
        task_ = std::make_unique<Task>(run());

        /* flowcoro::CoroTask 惰性启动（suspend_always）：
         * 将首次 resume 提交到线程池，协程体立即在工作线程上开始执行，
         * 调用线程（通常是 task_manager 的启动线程）不被阻塞。 */
        if (task_->handle && !task_->handle.done()) {
            auto h = task_->handle;
            flowcoro_integration::get_thread_pool().enqueue_void(
                [h]() mutable { if (!h.done()) h.resume(); });
        }

        while (!task_->done() && !stop_flag_) {
            std::unique_lock<std::mutex> lk(coro_mutex_);
            coro_cv_.wait_for(lk, std::chrono::milliseconds(5));
        }

        if (task_->handle && task_->handle.promise().exception_) {
            std::rethrow_exception(task_->handle.promise().exception_);
        }
    }
};
#endif // FLOWCORO_INTEGRATION

/* ─────────────────────────────────────────────────────────
 * 5. C 包装宏：将 CoroutineTask/FlowCoroTask 子类导出为 TaskBase 插件
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
static void prefix##_stop(TaskBase* b) {                                      \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    w->impl->stop();                                                          \
}                                                                             \
static bool prefix##_health(TaskBase* b) {                                    \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    return !w->impl->should_stop();                                           \
}                                                                             \
static const TaskInterface prefix##_vtable = {                                \
    nullptr, prefix##_execute, prefix##_stop,                                 \
    nullptr, nullptr, nullptr, prefix##_health, nullptr, nullptr              \
};                                                                            \
extern "C" {                                                                  \
prefix##_Wrapper* prefix##_create(const TaskConfig* cfg, MessageBus* bus) {   \
    auto* w = static_cast<prefix##_Wrapper*>(malloc(sizeof(prefix##_Wrapper))); \
    if (!w) return nullptr;                                                   \
    if (task_base_init(&w->base, &prefix##_vtable, cfg) != 0) {              \
        free(w); return nullptr;                                              \
    }                                                                         \
    w->impl = new ClassName(bus);                                             \
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
