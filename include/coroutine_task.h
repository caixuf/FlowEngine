/**
 * @file coroutine_task.h
 * @brief C++20 协程任务基类 + MessageBus awaitable 适配器
 *
 * v2: flowcoro::rt 确定性执行模型
 *   - Task = flowcoro::rt::RtTask（惰性启动 + park final_suspend）
 *   - CoroutineTask 降为纯接口：只保留 run() / bus() / should_stop()
 *   - 每个 awaitable 捕获 exec_ = g_node_exec（TLS），跨线程走 exec_->post_ready()
 *   - 删除 CompletionNotifier / FlowCoroTask / CancelToken / schedule_resume
 *   - 取消不再靠唤醒，协程每轮循环查 rt::stop_requested()，配合超时周期性醒来
 *
 * 节点建线程 pattern：
 * @code
 *   flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1, .idle_sleep_us=200 }};
 *   g_node_exec = &ex;
 *   ex.spawn(g.task->run());
 *   while (!g.should_stop) ex.run();
 *   ex.shutdown();
 *   g_node_exec = nullptr;
 * @endcode
 */

#ifndef COROUTINE_TASK_H
#define COROUTINE_TASK_H

#if defined(__cplusplus) && __cplusplus >= 202002L

#include "task_interface.h"
#include "message_bus.h"
#include "scheduler.h"

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
#include <map>
#include <set>
#include <algorithm>
#include <vector>
#include <initializer_list>
#include <iostream>

#include <flowcoro/rt_executor.h>

/* ─────────────────────────────────────────────────────────
 * TLS: 本线程唯一的 RtExecutor 指针。
 * 每个 awaitable 在 await_suspend(executor 线程)中读一次存入 exec_。
 * 跨线程回调(bus/timer)使用已捕获的 self->exec_，不读 TLS（那些线程 TLS 是 null）。
 * ───────────────────────────────────────────────────────── */
namespace { thread_local flowcoro::rt::RtExecutor* g_node_exec = nullptr; }

/* ─────────────────────────────────────────────────────────
 * 1. Task 别名 — 所有 `Task run() override` 自动跟随
 * ───────────────────────────────────────────────────────── */
using Task = flowcoro::rt::RtTask;

/* ─────────────────────────────────────────────────────────
 * 1b. CoroStats — 协程可观测性
 * ───────────────────────────────────────────────────────── */
inline uint64_t coro_now_us() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class CoroStats {
public:
    void record_resume(uint64_t suspend_us) {
        resume_count_.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lk(mtx_);
        latency_tracker_record(&suspend_latency_, suspend_us);
    }
    uint64_t resume_count() const {
        return resume_count_.load(std::memory_order_relaxed);
    }
    LatencyStats suspend_latency() {
        std::lock_guard<std::mutex> lk(mtx_);
        return latency_tracker_stats(&suspend_latency_);
    }
    void reset() {
        resume_count_.store(0, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lk(mtx_);
        std::memset(&suspend_latency_, 0, sizeof(suspend_latency_));
    }
private:
    std::atomic<uint64_t> resume_count_{0};
    std::mutex            mtx_;
    LatencyTracker        suspend_latency_{};
};

/* ─────────────────────────────────────────────────────────
 * 1c. AwaitResult + AwaitCtl
 * ───────────────────────────────────────────────────────── */
enum class AwaitStatus {
    Ready = 0,
    Cancelled,
    Timeout,
};

struct AwaitResult {
    AwaitStatus status{AwaitStatus::Ready};
    Message     message{};
    bool ok()        const { return status == AwaitStatus::Ready; }
    bool cancelled() const { return status == AwaitStatus::Cancelled; }
    bool timed_out() const { return status == AwaitStatus::Timeout; }
    explicit operator bool() const { return ok(); }
    const Message& operator*()  const { return message; }
    const Message* operator->() const { return &message; }
};

struct AwaitCtl {
    std::atomic<bool> fired{false};
    AwaitStatus       status{AwaitStatus::Ready};
    bool try_fire(AwaitStatus s) {
        bool expected = false;
        if (fired.compare_exchange_strong(expected, true,
                                          std::memory_order_acq_rel)) {
            status = s;
            return true;
        }
        return false;
    }
};

/* ─────────────────────────────────────────────────────────
 * 1d. TimerService — 轻量定时服务
 * ───────────────────────────────────────────────────────── */
class TimerService {
public:
    static TimerService& instance() {
        static TimerService svc;
        return svc;
    }
    uint64_t add(uint64_t delay_us, std::function<void()> cb) {
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::microseconds(delay_us);
        uint64_t id;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            id = ++next_id_;
            heap_.push_back(Entry{deadline, id, std::move(cb)});
            std::push_heap(heap_.begin(), heap_.end(), Cmp{});
        }
        cv_.notify_all();
        return id;
    }
    void cancel(uint64_t id) {
        if (id == 0) return;
        std::lock_guard<std::mutex> lk(mtx_);
        cancelled_.insert(id);
    }
private:
    struct Entry {
        std::chrono::steady_clock::time_point deadline;
        uint64_t                              id;
        std::function<void()>                 cb;
    };
    struct Cmp {
        bool operator()(const Entry& a, const Entry& b) const {
            return a.deadline > b.deadline;
        }
    };
    TimerService() : thread_([this] { run(); }) {}
    ~TimerService() {
        { std::lock_guard<std::mutex> lk(mtx_); stop_ = true; }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }
    void run() {
        std::unique_lock<std::mutex> lk(mtx_);
        while (!stop_) {
            if (heap_.empty()) { cv_.wait(lk); continue; }
            auto next = heap_.front().deadline;
            if (cv_.wait_until(lk, next) == std::cv_status::timeout ||
                (!heap_.empty() && heap_.front().deadline <= std::chrono::steady_clock::now())) {
                if (heap_.empty()) continue;
                if (heap_.front().deadline > std::chrono::steady_clock::now()) continue;
                std::pop_heap(heap_.begin(), heap_.end(), Cmp{});
                Entry e = std::move(heap_.back());
                heap_.pop_back();
                if (cancelled_.erase(e.id) > 0) continue;
                lk.unlock();
                e.cb();
                lk.lock();
            }
        }
    }
    std::mutex              mtx_;
    std::condition_variable cv_;
    std::vector<Entry>      heap_;
    std::set<uint64_t>      cancelled_;
    uint64_t                next_id_{0};
    bool                    stop_{false};
    std::thread             thread_;
};

/* ─────────────────────────────────────────────────────────
 * 2. BusAwaitable — co_await 等待一条总线消息
 * ───────────────────────────────────────────────────────── */
template <bool WithResult>
class BusAwaitableT {
public:
    BusAwaitableT(MessageBus* bus, const char* topic, uint64_t timeout_us = 0)
        : bus_(bus), topic_(topic), timeout_us_(timeout_us),
          ctl_(std::make_shared<AwaitCtl>()) {}

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> handle) {
        handle_ = handle;
        exec_ = g_node_exec;  // TLS: 在 executor 线程的 resume 栈中读一次
        t_suspend_us_ = coro_now_us();

        if (timeout_us_ > 0) {
            auto ctl = ctl_;
            auto* ex = exec_;
            timer_id_ = TimerService::instance().add(timeout_us_, [ctl, ex, handle] {
                if (ctl->try_fire(AwaitStatus::Timeout)) ex->post_ready(handle);
            });
        }
        message_bus_subscribe(bus_, topic_.c_str(), &BusAwaitableT::on_message, this);
    }

    auto await_resume() {
        message_bus_unsubscribe_ex(bus_, topic_.c_str(), &BusAwaitableT::on_message, this);
        if (timer_id_) TimerService::instance().cancel(timer_id_);
        if constexpr (WithResult) {
            return AwaitResult{ctl_->status, received_msg_};
        } else {
            return received_msg_;
        }
    }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<BusAwaitableT*>(user_data);
        if (!self->ctl_->try_fire(AwaitStatus::Ready)) return;
        self->received_msg_ = *msg;
        self->exec_->post_ready(self->handle_);
    }

    MessageBus*                 bus_;
    std::string                 topic_;
    uint64_t                    timeout_us_;
    std::shared_ptr<AwaitCtl>   ctl_;
    uint64_t                    timer_id_{0};
    uint64_t                    t_suspend_us_{0};
    std::coroutine_handle<>     handle_;
    Message                     received_msg_{};
    flowcoro::rt::RtExecutor*   exec_ = nullptr;
};

using BusAwaitable = BusAwaitableT<false>;

inline BusAwaitableT<false> subscribe_once(MessageBus* bus, const char* topic) {
    return BusAwaitableT<false>{bus, topic};
}

inline BusAwaitableT<true> subscribe_once_for(MessageBus* bus, const char* topic,
                                              uint64_t timeout_us) {
    return BusAwaitableT<true>{bus, topic, timeout_us};
}

/* ─────────────────────────────────────────────────────────
 * 2b. BusChannel
 * ───────────────────────────────────────────────────────── */
class BusChannel {
public:
    BusChannel(MessageBus* bus, const char* topic, size_t capacity = 32)
        : bus_(bus), topic_(topic), capacity_(capacity) {
        message_bus_subscribe(bus_, topic_.c_str(), &BusChannel::on_message, this);
    }
    ~BusChannel() {
        message_bus_unsubscribe_ex(bus_, topic_.c_str(), &BusChannel::on_message, this);
    }
    BusChannel(const BusChannel&) = delete;
    BusChannel& operator=(const BusChannel&) = delete;

    auto recv() { return RecvAwaitable<false>{this, 0}; }
    auto recv_for(uint64_t timeout_us) { return RecvAwaitable<true>{this, timeout_us}; }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<BusChannel*>(user_data);
        std::coroutine_handle<>   waiter;
        std::shared_ptr<AwaitCtl> ctl;
        flowcoro::rt::RtExecutor* exec = nullptr;
        {
            std::lock_guard<std::mutex> lk(self->mutex_);
            if (self->buffer_.size() < self->capacity_) {
                self->buffer_.push(*msg);
            }
            waiter = self->waiter_;
            ctl    = self->waiter_ctl_;
            exec   = self->waiter_exec_;
            self->waiter_     = nullptr;
            self->waiter_ctl_ = nullptr;
            self->waiter_exec_ = nullptr;
        }
        if (!waiter || !ctl || !exec) return;
        if (ctl->try_fire(AwaitStatus::Ready)) exec->post_ready(waiter);
    }

    template <bool WithResult>
    struct RecvAwaitable {
        BusChannel* ch;
        uint64_t    timeout_us;
        std::shared_ptr<AwaitCtl> ctl{std::make_shared<AwaitCtl>()};
        uint64_t    timer_id{0};
        uint64_t    t_suspend_us{0};
        flowcoro::rt::RtExecutor* exec_ = nullptr;

        bool await_ready() {
            std::lock_guard<std::mutex> lk(ch->mutex_);
            return !ch->buffer_.empty();
        }
        void await_suspend(std::coroutine_handle<> h) {
            bool should_resume = false;
            t_suspend_us = coro_now_us();
            exec_ = g_node_exec;
            {
                std::lock_guard<std::mutex> lk(ch->mutex_);
                if (!ch->buffer_.empty()) {
                    should_resume = true;
                } else {
                    ch->waiter_      = h;
                    ch->waiter_ctl_  = ctl;
                    ch->waiter_exec_ = exec_;
                }
            }
            if (should_resume) {
                if (ctl->try_fire(AwaitStatus::Ready)) exec_->post_ready(h);
                return;
            }
            if (timeout_us > 0) {
                auto c = ctl;
                auto* ex = exec_;
                timer_id = TimerService::instance().add(timeout_us, [c, ex, h] {
                    if (c->try_fire(AwaitStatus::Timeout)) ex->post_ready(h);
                });
            }
        }
        auto await_resume() {
            if (timer_id) TimerService::instance().cancel(timer_id);
            Message msg{};
            AwaitStatus status = ctl->status;
            {
                std::lock_guard<std::mutex> lk(ch->mutex_);
                if (ch->waiter_ctl_ == ctl) {
                    ch->waiter_     = nullptr;
                    ch->waiter_ctl_ = nullptr;
                    ch->waiter_exec_ = nullptr;
                }
                if (!ch->buffer_.empty()) {
                    msg = ch->buffer_.front();
                    ch->buffer_.pop();
                    status = AwaitStatus::Ready;
                }
            }
            if constexpr (WithResult) {
                return AwaitResult{status, msg};
            } else {
                return msg;
            }
        }
    };

    MessageBus*               bus_;
    std::string               topic_;
    size_t                    capacity_;
    std::queue<Message>       buffer_;
    std::mutex                mutex_;
    std::coroutine_handle<>   waiter_{};
    std::shared_ptr<AwaitCtl> waiter_ctl_{};
    flowcoro::rt::RtExecutor* waiter_exec_{nullptr};
};

/* ─────────────────────────────────────────────────────────
 * 2c. WhenAnyBusAwaitable
 * ───────────────────────────────────────────────────────── */
template <bool WithResult>
class WhenAnyBusAwaitableT {
public:
    WhenAnyBusAwaitableT(MessageBus* bus, std::vector<std::string> topics,
                         uint64_t timeout_us = 0)
        : bus_(bus), topics_(std::move(topics)),
          timeout_us_(timeout_us), ctl_(std::make_shared<AwaitCtl>()) {}

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) {
        handle_ = h;
        exec_ = g_node_exec;
        t_suspend_us_ = coro_now_us();
        if (timeout_us_ > 0) {
            auto ctl = ctl_;
            auto* ex = exec_;
            timer_id_ = TimerService::instance().add(timeout_us_, [ctl, ex, h] {
                if (ctl->try_fire(AwaitStatus::Timeout)) ex->post_ready(h);
            });
        }
        for (const auto& t : topics_) {
            message_bus_subscribe(bus_, t.c_str(), &WhenAnyBusAwaitableT::on_message, this);
        }
    }

    auto await_resume() {
        for (const auto& t : topics_) {
            message_bus_unsubscribe_ex(bus_, t.c_str(), &WhenAnyBusAwaitableT::on_message, this);
        }
        if (timer_id_) TimerService::instance().cancel(timer_id_);
        if constexpr (WithResult) {
            return AwaitResult{ctl_->status, received_msg_};
        } else {
            return received_msg_;
        }
    }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<WhenAnyBusAwaitableT*>(user_data);
        if (!self->ctl_->try_fire(AwaitStatus::Ready)) return;
        self->received_msg_ = *msg;
        self->exec_->post_ready(self->handle_);
    }

    MessageBus*               bus_;
    std::vector<std::string>  topics_;
    uint64_t                  timeout_us_;
    std::shared_ptr<AwaitCtl> ctl_;
    uint64_t                  timer_id_{0};
    uint64_t                  t_suspend_us_{0};
    std::coroutine_handle<>   handle_{};
    Message                   received_msg_{};
    flowcoro::rt::RtExecutor* exec_ = nullptr;
};

using WhenAnyBusAwaitable = WhenAnyBusAwaitableT<false>;

inline WhenAnyBusAwaitableT<false> when_any_bus(MessageBus* bus,
                                                std::initializer_list<const char*> topics) {
    return WhenAnyBusAwaitableT<false>{
        bus, std::vector<std::string>(topics.begin(), topics.end())};
}

inline WhenAnyBusAwaitableT<true> when_any_bus_for(MessageBus* bus,
                                                   std::initializer_list<const char*> topics,
                                                   uint64_t timeout_us) {
    return WhenAnyBusAwaitableT<true>{
        bus, std::vector<std::string>(topics.begin(), topics.end()), timeout_us};
}

/* ─────────────────────────────────────────────────────────
 * 2d. DelayAwaitable
 * ───────────────────────────────────────────────────────── */
class DelayAwaitable {
public:
    explicit DelayAwaitable(uint64_t timeout_us) : timeout_us_(timeout_us),
          ctl_(std::make_shared<AwaitCtl>()) {}

    bool await_ready() const noexcept { return timeout_us_ == 0; }

    void await_suspend(std::coroutine_handle<> h) {
        exec_ = g_node_exec;
        t_suspend_us_ = coro_now_us();
        auto ctl = ctl_;
        auto* ex = exec_;
        timer_id_ = TimerService::instance().add(timeout_us_, [ctl, ex, h] {
            if (ctl->try_fire(AwaitStatus::Ready)) ex->post_ready(h);
        });
    }

    bool await_resume() {
        if (timer_id_) TimerService::instance().cancel(timer_id_);
        return ctl_->status == AwaitStatus::Ready;
    }

private:
    uint64_t                  timeout_us_;
    std::shared_ptr<AwaitCtl> ctl_;
    uint64_t                  timer_id_{0};
    uint64_t                  t_suspend_us_{0};
    flowcoro::rt::RtExecutor* exec_ = nullptr;
};

inline DelayAwaitable delay_us(uint64_t us) { return DelayAwaitable{us}; }
inline DelayAwaitable delay_ms(uint64_t ms) { return DelayAwaitable{ms * 1000ULL}; }

/* ─────────────────────────────────────────────────────────
 * 2e. run_blocking
 * ───────────────────────────────────────────────────────── */
inline void run_blocking(std::function<void()> fn) {
    std::thread(std::move(fn)).detach();
}

/* ─────────────────────────────────────────────────────────
 * 2f. RequestAwaitable
 * ───────────────────────────────────────────────────────── */
class RequestAwaitable {
public:
    RequestAwaitable(MessageBus* bus, const char* topic, const char* sender,
                     const void* data, uint32_t size, uint32_t timeout_ms)
        : bus_(bus), topic_(topic), sender_(sender ? sender : "coro"),
          timeout_ms_(timeout_ms),
          ctl_(std::make_shared<AwaitCtl>()),
          reply_(std::make_shared<Message>()) {
        if (data && size) {
            data_.assign(static_cast<const uint8_t*>(data),
                         static_cast<const uint8_t*>(data) + size);
        }
    }

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) {
        exec_ = g_node_exec;
        t_suspend_us_ = coro_now_us();
        auto ctl    = ctl_;
        auto reply  = reply_;
        auto bus    = bus_;
        auto topic  = topic_;
        auto sender = sender_;
        auto data   = data_;
        auto tmo    = timeout_ms_;
        auto* ex    = exec_;
        run_blocking([ctl, reply, bus, topic, sender, data, tmo, ex, h] {
            Message rep{};
            int rc = message_bus_request(bus, topic.c_str(), sender.c_str(),
                                         data.empty() ? nullptr : data.data(),
                                         static_cast<uint32_t>(data.size()),
                                         &rep, tmo);
            if (ctl->try_fire(rc == 0 ? AwaitStatus::Ready : AwaitStatus::Timeout)) {
                *reply = rep;
                ex->post_ready(h);
            }
        });
    }

    AwaitResult await_resume() {
        return AwaitResult{ctl_->status, *reply_};
    }

private:
    MessageBus*               bus_;
    std::string               topic_;
    std::string               sender_;
    std::vector<uint8_t>      data_;
    uint32_t                  timeout_ms_;
    std::shared_ptr<AwaitCtl> ctl_;
    std::shared_ptr<Message>  reply_;
    uint64_t                  t_suspend_us_{0};
    flowcoro::rt::RtExecutor* exec_ = nullptr;
};

inline RequestAwaitable request(MessageBus* bus, const char* topic, const char* sender,
                                const void* data, uint32_t size, uint32_t timeout_ms) {
    return RequestAwaitable{bus, topic, sender, data, size, timeout_ms};
}

/* ─────────────────────────────────────────────────────────
 * 3. CoroutineTask — 纯接口（降为最小基类）
 *    保留 bus() / should_stop() / set_stop()，删 execute() 和所有阻塞机器
 * ───────────────────────────────────────────────────────── */
class CoroutineTask {
public:
    explicit CoroutineTask(MessageBus* bus = nullptr)
        : bus_(bus), stop_flag_(false) {}

    virtual ~CoroutineTask() = default;

    /** 子类实现具体协程逻辑 */
    virtual Task run() = 0;

    MessageBus* bus() const { return bus_; }
    bool should_stop() const { return stop_flag_.load(std::memory_order_acquire); }
    void set_stop() { stop_flag_.store(true, std::memory_order_release); }

protected:
    MessageBus*       bus_;
    std::atomic<bool> stop_flag_;
};

/* ─────────────────────────────────────────────────────────
 * 4. C 包装宏：将 CoroutineTask 子类导出为 TaskBase 插件
 *
 * execute 改用 RtExecutor 循环，不再调用已删除的 CoroutineTask::execute()
 * ───────────────────────────────────────────────────────── */
#define EXPORT_COROUTINE_TASK(ClassName, prefix)                              \
struct prefix##_Wrapper {                                                     \
    TaskBase       base;                                                      \
    ClassName*     impl;                                                      \
};                                                                            \
                                                                              \
static int prefix##_execute(TaskBase* b) {                                    \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    try {                                                                     \
        flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1, .idle_sleep_us=200 }};    \
        g_node_exec = &ex;                                                    \
        ex.spawn(w->impl->run());                                             \
        while (!w->impl->should_stop()) ex.run();                             \
        ex.shutdown();                                                        \
        g_node_exec = nullptr;                                                \
        return 0;                                                             \
    } catch (...) { return -1; }                                              \
}                                                                             \
static void prefix##_stop(TaskBase* b) {                                      \
    auto* w = reinterpret_cast<prefix##_Wrapper*>(b);                        \
    w->impl->set_stop();                                                      \
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
#warning "coroutine_task.h requires C++20 or later"
#endif /* C++20 */

#endif /* COROUTINE_TASK_H */