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
 *
 * ─────────────────────────────────────────────────────────────────────
 * 所有权与生命周期规则（C / C++ / 协程混用，务必遵守）
 * ─────────────────────────────────────────────────────────────────────
 *   1. MessageBus 由外部拥有，其生命周期必须严格长于所有引用它的
 *      CoroutineTask 及其协程帧。销毁顺序：先确保协程结束/取消并
 *      execute() 返回，再 message_bus_destroy(bus)，最后析构 Task。
 *   2. 每个 awaitable 在 await_suspend 时向 bus 注册订阅（或向
 *      CancelToken/TimerService 注册回调），并在 await_resume 或其
 *      析构中反注册。协程帧销毁会依次析构挂起点的 awaitable，从而
 *      在 bus 仍存活时完成反注册——因此 (1) 的销毁顺序是必要条件。
 *   3. 一次挂起至多被恢复一次：消息 / 取消 / 超时三条路径共享同一个
 *      AwaitCtl，通过 CAS(try_fire) 决出唯一赢家；共享 shared_ptr 保证
 *      即便 awaitable 已析构，仍在途的定时器/取消回调也不会悬垂访问。
 *   4. 阻塞型原语（RequestAwaitable/run_blocking）只捕获值拷贝与共享
 *      控制块，绝不捕获 this；被取消的协程销毁 awaitable 后，迟到的
 *      阻塞结果不会访问已释放对象。
 *   5. stop() 通过 CancelToken 直接唤醒悬挂协程，无需外部再发消息；
 *      取消回调在持锁外调用，避免同步恢复模式下的重入死锁。
 */

#ifndef COROUTINE_TASK_H
#define COROUTINE_TASK_H

#if defined(__cplusplus) && __cplusplus >= 202002L

#include "task_interface.h"
#include "message_bus.h"
#include "scheduler.h"   /* LatencyTracker / LatencyStats — 协程可观测性 */

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
/* coro_task.h 自身使用了 CoroutineManager 但未包含其头文件，
 * 必须在包含 coro_task.h 之前显式引入，否则编译报错。         */
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
 * 1b. schedule_resume — 统一的协程恢复入口
 *
 * 将"在何处 resume 协程"的差异集中到一处：
 *   - FLOWCORO_INTEGRATION：投入无锁线程池，调用方（总线分发线程 /
 *     取消线程 / 定时器线程）立即返回，协程在工作线程上继续执行。
 *   - 标准模式：在调用线程上同步 resume。
 * 所有 awaitable 与 CancelToken 均复用此函数，避免复制粘贴分支。
 * ───────────────────────────────────────────────────────── */
inline void schedule_resume(std::coroutine_handle<> h) {
    if (!h || h.done()) return;
#ifdef FLOWCORO_INTEGRATION
    flowcoro_integration::get_thread_pool().enqueue_void(
        [h]() mutable { if (!h.done()) h.resume(); });
#else
    h.resume();
#endif
}

/* ─────────────────────────────────────────────────────────
 * 1b'. 协程可观测性 —— CoroStats
 *
 * 记录协程行为供 flowctl / 宿主观测：
 *   - resume_count：协程被恢复的累计次数。
 *   - suspend_latency：每次挂起→恢复的时长分布（复用 scheduler.h 的
 *     LatencyTracker，输出 avg/p50/p99）。
 *
 * 注意：latency_tracker_record 本身非线程安全，而 awaitable 可能从
 * 定时线程 / 总线分发线程 / 线程池 worker 恢复，故此处用互斥量保护。
 * ───────────────────────────────────────────────────────── */
inline uint64_t coro_now_us() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class CoroStats {
public:
    /** 记录一次恢复：suspend_us 为本次挂起持续的微秒数。线程安全。 */
    void record_resume(uint64_t suspend_us) {
        resume_count_.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lk(mtx_);
        latency_tracker_record(&suspend_latency_, suspend_us);
    }

    uint64_t resume_count() const {
        return resume_count_.load(std::memory_order_relaxed);
    }

    /** 挂起时长统计（avg/p50/p99/min/max）。线程安全。 */
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
 * 1c. CancelToken — 协程取消令牌
 *
 * 解决"悬挂协程无法在不外发消息的情况下被 stop() 唤醒退出"的缺陷。
 *
 * 语义：
 *   - CoroutineTask 持有一个 CancelToken。stop() 调用 request_cancel()。
 *   - 每个可取消的 awaitable 在挂起时通过 add_waiter() 注册一个唤醒回调，
 *     并在恢复时通过 remove_waiter() 注销（best-effort 清理，防止列表膨胀）。
 *   - request_cancel() 会置位 cancelled_ 并唤醒所有已注册的 waiter，
 *     使悬挂的协程立即恢复；协程通过 should_stop()/cancelled() 检查后干净退出。
 *
 * 双重恢复保护：
 *   awaitable 的消息路径与取消路径共享同一个 std::shared_ptr<atomic<bool>>
 *   "fired" 守卫（CAS），确保一次挂起最多恢复一次。守卫用 shared_ptr 持有，
 *   即便 awaitable 已析构，取消 waiter 仍可安全访问守卫，不产生悬垂引用。
 *
 * 线程安全：request_cancel() 在锁外调用 waiter，避免与协程内的再次
 * add_waiter()（同一线程同步 resume 场景）发生自死锁。
 * ───────────────────────────────────────────────────────── */
class CancelToken {
public:
    using WakeFn = std::function<void()>;

    /** 请求取消：置位标志并唤醒所有已注册的挂起 awaitable。可从任意线程调用。 */
    void request_cancel() {
        std::vector<WakeFn> to_wake;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            cancelled_.store(true, std::memory_order_release);
            to_wake.reserve(waiters_.size());
            for (auto& kv : waiters_) to_wake.push_back(kv.second);
            waiters_.clear();
        }
        /* 锁外唤醒，防止同步 resume 路径重入 add_waiter 造成死锁 */
        for (auto& fn : to_wake) fn();
    }

    /** 复位为未取消状态（任务重启时使用）。 */
    void reset() {
        std::lock_guard<std::mutex> lk(mtx_);
        cancelled_.store(false, std::memory_order_release);
        waiters_.clear();
    }

    bool cancelled() const { return cancelled_.load(std::memory_order_acquire); }

    /**
     * 注册唤醒回调。若已处于取消状态，立即在当前线程执行 wake 并返回 0
     * （0 表示"未挂号"，调用方无需 remove）。否则返回 >0 的注册 id。
     */
    uint64_t add_waiter(WakeFn wake) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!cancelled_.load(std::memory_order_acquire)) {
                uint64_t id = ++next_id_;
                waiters_.emplace(id, std::move(wake));
                return id;
            }
        }
        /* 已取消：锁外立即唤醒 */
        wake();
        return 0;
    }

    /** 注销唤醒回调（best-effort，id==0 表示无需注销）。 */
    void remove_waiter(uint64_t id) {
        if (id == 0) return;
        std::lock_guard<std::mutex> lk(mtx_);
        waiters_.erase(id);
    }

    /** 关联可观测统计（由持有本令牌的 CoroutineTask 设置）。 */
    void set_stats(CoroStats* stats) { stats_ = stats; }

    /** 转发一次恢复事件到统计（若未关联则忽略）。awaitable 在恢复时调用。 */
    void record_resume(uint64_t suspend_us) {
        if (stats_) stats_->record_resume(suspend_us);
    }

private:
    std::atomic<bool>                  cancelled_{false};
    std::mutex                         mtx_;
    std::map<uint64_t, WakeFn>         waiters_;
    uint64_t                           next_id_{0};
    CoroStats*                         stats_{nullptr};
};

/* ─────────────────────────────────────────────────────────
 * 1d. AwaitResult — 带状态的 co_await 结果
 *
 * 用于超时/取消感知的 awaitable（recv_for / when_any_for / request）。
 * 老式 awaitable 仍直接返回 Message，保持向后兼容。
 * ───────────────────────────────────────────────────────── */
enum class AwaitStatus {
    Ready = 0,   /**< 正常收到消息 */
    Cancelled,   /**< 被 CancelToken 取消 */
    Timeout,     /**< 等待超时 */
};

struct AwaitResult {
    AwaitStatus status{AwaitStatus::Ready};
    Message     message{};

    bool ok()        const { return status == AwaitStatus::Ready; }
    bool cancelled() const { return status == AwaitStatus::Cancelled; }
    bool timed_out() const { return status == AwaitStatus::Timeout; }

    /** if (auto r = co_await ...) { use r.message; } */
    explicit operator bool() const { return ok(); }
    const Message& operator*()  const { return message; }
    const Message* operator->() const { return &message; }
};

/* ─────────────────────────────────────────────────────────
 * 1d'. AwaitCtl — awaitable 的共享恢复控制块
 *
 * 消息 / 取消 / 超时三条恢复路径共享同一个 AwaitCtl：
 *   - fired  ：CAS 守卫，保证一次挂起最多恢复一次。
 *   - status ：由 CAS 胜者写入，await_resume 读取（happens-after resume）。
 * 用 shared_ptr 持有，即便 awaitable 已析构，仍在途的取消/超时回调
 * 也能安全访问，杜绝悬垂引用。
 * ───────────────────────────────────────────────────────── */
struct AwaitCtl {
    std::atomic<bool> fired{false};
    AwaitStatus       status{AwaitStatus::Ready};

    /** CAS 竞争恢复权；胜者写入状态并返回 true。 */
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
 * 1e. TimerService — 轻量定时服务（单后台线程 + 最小堆）
 *
 * 为协程层提供"不占线程的定时挂起"能力：
 *   - recv_for / when_any_for 的超时
 *   - delay(ms) 定时器 awaitable
 *
 * 单例，后台线程惰性启动，进程退出时静态析构停止线程。
 * 回调在定时线程上执行且在锁外调用，回调内部通过 schedule_resume
 * 将协程恢复投入线程池（FLOWCORO）或同步执行（标准模式）。
 * ───────────────────────────────────────────────────────── */
class TimerService {
public:
    static TimerService& instance() {
        static TimerService svc;
        return svc;
    }

    /**
     * 注册一个在 delay_us 微秒后触发的回调，返回定时器 id（>0）。
     * 回调若在触发前被 cancel() 注销则不会执行。
     */
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

    /** 注销定时器（best-effort，触发前有效）。 */
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
            return a.deadline > b.deadline; /* min-heap: 最早的在堆顶 */
        }
    };

    TimerService() : thread_([this] { run(); }) {}

    ~TimerService() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_ = true;
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    void run() {
        std::unique_lock<std::mutex> lk(mtx_);
        while (!stop_) {
            if (heap_.empty()) {
                cv_.wait(lk);
                continue;
            }
            auto next = heap_.front().deadline;
            if (cv_.wait_until(lk, next) == std::cv_status::timeout ||
                (!heap_.empty() && heap_.front().deadline <= std::chrono::steady_clock::now())) {
                if (heap_.empty()) continue;
                if (heap_.front().deadline > std::chrono::steady_clock::now()) continue;
                std::pop_heap(heap_.begin(), heap_.end(), Cmp{});
                Entry e = std::move(heap_.back());
                heap_.pop_back();
                bool skip = cancelled_.erase(e.id) > 0;
                if (skip) continue;
                /* 锁外执行回调，避免回调重入 add()/cancel() 造成死锁 */
                lk.unlock();
                e.cb();
                lk.lock();
            }
        }
    }

    std::mutex                  mtx_;
    std::condition_variable     cv_;
    std::vector<Entry>          heap_;
    std::set<uint64_t>          cancelled_;
    uint64_t                    next_id_{0};
    bool                        stop_{false};
    std::thread                 thread_;
};

/* ─────────────────────────────────────────────────────────
 * 2. BusAwaitable — co_await 等待一条总线消息
 * ───────────────────────────────────────────────────────── */

/**
 * co_await subscribe_once(bus, "sensor/lidar") 会挂起当前协程，
 * 直到指定 topic 有消息到达后恢复协程。
 *
 * 取消与超时：
 *   - 传入 CancelToken 后，stop() 可直接唤醒悬挂的协程，无需外发消息。
 *   - timeout_us > 0 时，超过时限自动唤醒并返回 Timeout 状态。
 *
 * 线程安全：
 *   - 消息 / 取消 / 超时三条恢复路径共享同一个 fired 守卫（CAS），
 *     确保一次挂起最多恢复一次。守卫用 shared_ptr 持有，
 *     即便 awaitable 已析构，取消/超时回调仍可安全访问，无悬垂引用。
 *   - FLOWCORO_INTEGRATION 模式：恢复经由线程池，分发线程不被协程阻塞。
 *
 * WithResult=false 时 await_resume 返回 Message（向后兼容 subscribe_once）；
 * WithResult=true  时返回 AwaitResult（携带 Ready/Cancelled/Timeout 状态）。
 */
template <bool WithResult>
class BusAwaitableT {
public:
    BusAwaitableT(MessageBus* bus, const char* topic,
                  CancelToken* cancel = nullptr, uint64_t timeout_us = 0)
        : bus_(bus), topic_(topic), cancel_(cancel), timeout_us_(timeout_us),
          ctl_(std::make_shared<AwaitCtl>()) {}

    /* awaitable 协议 */
    bool await_ready() const noexcept { return false; /* 总是挂起 */ }

    void await_suspend(std::coroutine_handle<> handle) {
        handle_ = handle;
        t_suspend_us_ = coro_now_us();

        /* 注册取消唤醒：stop() → request_cancel() → 唤醒本协程 */
        if (cancel_) {
            auto ctl = ctl_;
            cancel_id_ = cancel_->add_waiter([ctl, handle] {
                if (ctl->try_fire(AwaitStatus::Cancelled)) schedule_resume(handle);
            });
        }
        /* 注册超时唤醒 */
        if (timeout_us_ > 0) {
            auto ctl = ctl_;
            timer_id_ = TimerService::instance().add(timeout_us_, [ctl, handle] {
                if (ctl->try_fire(AwaitStatus::Timeout)) schedule_resume(handle);
            });
        }
        /* 订阅放在取消/超时注册之后，保证 on_message 触发 await_resume
         * 时 cancel_id_ 和 timer_id_ 已写入，不存在数据竞争。 */
        message_bus_subscribe(bus_, topic_.c_str(), &BusAwaitableT::on_message, this);
    }

    auto await_resume() {
        /* message_bus_unsubscribe_ex 持有 sub_mutex 直到返回，而 dispatch_message
         * 在整个回调期间也持有 sub_mutex，因此 unsubscribe_ex 返回后保证没有
         * 任何 on_message 调用仍在执行。 */
        message_bus_unsubscribe_ex(bus_, topic_.c_str(), &BusAwaitableT::on_message, this);
        if (cancel_) { cancel_->remove_waiter(cancel_id_); cancel_->record_resume(coro_now_us() - t_suspend_us_); }
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
        /* 消息 / 取消 / 超时共享 ctl_->fired，避免双重 resume（未定义行为）。 */
        if (!self->ctl_->try_fire(AwaitStatus::Ready)) {
            return; /* 已被取消/超时率先触发 */
        }
        self->received_msg_ = *msg;
        schedule_resume(self->handle_);
    }

    MessageBus*                 bus_;
    std::string                 topic_;
    CancelToken*                cancel_;
    uint64_t                    timeout_us_;
    std::shared_ptr<AwaitCtl>   ctl_;
    uint64_t                    cancel_id_{0};
    uint64_t                    timer_id_{0};
    uint64_t                    t_suspend_us_{0};
    std::coroutine_handle<>     handle_;
    Message                     received_msg_{};
};

using BusAwaitable = BusAwaitableT<false>;

/**
 * 工厂函数：创建等待指定 topic 一条消息的 awaitable
 *
 * 用法：
 *   Message msg = co_await subscribe_once(bus, "sensor/lidar");
 */
inline BusAwaitableT<false> subscribe_once(MessageBus* bus, const char* topic,
                                           CancelToken* cancel = nullptr) {
    return BusAwaitableT<false>{bus, topic, cancel};
}

/**
 * 带超时的一次性订阅：返回 AwaitResult，可区分 Ready/Cancelled/Timeout。
 *
 * 用法：
 *   auto r = co_await subscribe_once_for(bus, "sensor/lidar", 50000, token);
 *   if (r.ok())        process(r.message);
 *   else if (r.timed_out()) handle_timeout();
 */
inline BusAwaitableT<true> subscribe_once_for(MessageBus* bus, const char* topic,
                                              uint64_t timeout_us,
                                              CancelToken* cancel = nullptr) {
    return BusAwaitableT<true>{bus, topic, cancel, timeout_us};
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
    BusChannel(MessageBus* bus, const char* topic, size_t capacity = 32,
               CancelToken* cancel = nullptr)
        : bus_(bus), topic_(topic), capacity_(capacity), cancel_(cancel) {
        message_bus_subscribe(bus_, topic_.c_str(), &BusChannel::on_message, this);
    }

    ~BusChannel() {
        message_bus_unsubscribe_ex(bus_, topic_.c_str(), &BusChannel::on_message, this);
    }

    /* 禁止拷贝（订阅指针指向 this，不能移动） */
    BusChannel(const BusChannel&) = delete;
    BusChannel& operator=(const BusChannel&) = delete;

    /** 返回等待下一条消息的 awaitable（返回 Message，向后兼容）。 */
    auto recv() { return RecvAwaitable<false>{this, 0}; }

    /**
     * 带超时的接收：返回 AwaitResult，可区分 Ready/Cancelled/Timeout。
     *   auto r = co_await ch.recv_for(50000);
     *   if (r.timed_out()) { ... }
     */
    auto recv_for(uint64_t timeout_us) { return RecvAwaitable<true>{this, timeout_us}; }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<BusChannel*>(user_data);
        std::coroutine_handle<>   waiter;
        std::shared_ptr<AwaitCtl> ctl;
        {
            std::lock_guard<std::mutex> lk(self->mutex_);
            if (self->buffer_.size() < self->capacity_) {
                self->buffer_.push(*msg);
            }
            waiter = self->waiter_;
            ctl    = self->waiter_ctl_;
            self->waiter_     = nullptr;
            self->waiter_ctl_ = nullptr;
        }
        if (!waiter || !ctl) return;
        /* 与取消/超时共享 ctl：谁先 CAS 谁恢复协程 */
        if (ctl->try_fire(AwaitStatus::Ready)) schedule_resume(waiter);
    }

    template <bool WithResult>
    struct RecvAwaitable {
        BusChannel* ch;
        uint64_t    timeout_us;
        std::shared_ptr<AwaitCtl> ctl{std::make_shared<AwaitCtl>()};
        uint64_t    cancel_id{0};
        uint64_t    timer_id{0};
        uint64_t    t_suspend_us{0};

        bool await_ready() {
            std::lock_guard<std::mutex> lk(ch->mutex_);
            return !ch->buffer_.empty();
        }

        void await_suspend(std::coroutine_handle<> h) {
            bool should_resume = false;
            t_suspend_us = coro_now_us();
            {
                std::lock_guard<std::mutex> lk(ch->mutex_);
                /* 检查在 await_ready 到 await_suspend 之间是否有消息到达 */
                if (!ch->buffer_.empty()) {
                    should_resume = true;
                } else {
                    ch->waiter_     = h;
                    ch->waiter_ctl_ = ctl;
                }
            }
            if (should_resume) {
                if (ctl->try_fire(AwaitStatus::Ready)) schedule_resume(h);
                return;
            }
            /* 注册取消唤醒 */
            if (ch->cancel_) {
                auto c = ctl;
                cancel_id = ch->cancel_->add_waiter([c, h] {
                    if (c->try_fire(AwaitStatus::Cancelled)) schedule_resume(h);
                });
            }
            /* 注册超时唤醒 */
            if (timeout_us > 0) {
                auto c = ctl;
                timer_id = TimerService::instance().add(timeout_us, [c, h] {
                    if (c->try_fire(AwaitStatus::Timeout)) schedule_resume(h);
                });
            }
        }

        auto await_resume() {
            if (ch->cancel_) ch->cancel_->remove_waiter(cancel_id);
            if (timer_id)    TimerService::instance().cancel(timer_id);
            if (ch->cancel_ && t_suspend_us) ch->cancel_->record_resume(coro_now_us() - t_suspend_us);
            Message msg{};
            AwaitStatus status = ctl->status;
            {
                std::lock_guard<std::mutex> lk(ch->mutex_);
                /* 清理可能仍指向本 awaitable 的 waiter（取消/超时路径） */
                if (ch->waiter_ctl_ == ctl) {
                    ch->waiter_     = nullptr;
                    ch->waiter_ctl_ = nullptr;
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
    CancelToken*              cancel_;
    std::queue<Message>       buffer_;
    std::mutex                mutex_;
    std::coroutine_handle<>   waiter_{};
    std::shared_ptr<AwaitCtl> waiter_ctl_{};
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
template <bool WithResult>
class WhenAnyBusAwaitableT {
public:
    WhenAnyBusAwaitableT(MessageBus* bus, std::vector<std::string> topics,
                         CancelToken* cancel = nullptr, uint64_t timeout_us = 0)
        : bus_(bus), topics_(std::move(topics)), cancel_(cancel),
          timeout_us_(timeout_us), ctl_(std::make_shared<AwaitCtl>()) {}

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) {
        handle_ = h;
        t_suspend_us_ = coro_now_us();
        if (cancel_) {
            auto ctl = ctl_;
            cancel_id_ = cancel_->add_waiter([ctl, h] {
                if (ctl->try_fire(AwaitStatus::Cancelled)) schedule_resume(h);
            });
        }
        if (timeout_us_ > 0) {
            auto ctl = ctl_;
            timer_id_ = TimerService::instance().add(timeout_us_, [ctl, h] {
                if (ctl->try_fire(AwaitStatus::Timeout)) schedule_resume(h);
            });
        }
        /* 订阅放在取消/超时注册之后，保证 on_message 触发 await_resume
         * 时 cancel_id_ 和 timer_id_ 已写入，不存在数据竞争。 */
        for (const auto& t : topics_) {
            message_bus_subscribe(bus_, t.c_str(), &WhenAnyBusAwaitableT::on_message, this);
        }
    }

    auto await_resume() {
        /* 注销所有剩余订阅 */
        for (const auto& t : topics_) {
            message_bus_unsubscribe_ex(bus_, t.c_str(), &WhenAnyBusAwaitableT::on_message, this);
        }
        if (cancel_)   cancel_->remove_waiter(cancel_id_);
        if (timer_id_) TimerService::instance().cancel(timer_id_);
        if (cancel_)   cancel_->record_resume(coro_now_us() - t_suspend_us_);
        if constexpr (WithResult) {
            return AwaitResult{ctl_->status, received_msg_};
        } else {
            return received_msg_;
        }
    }

private:
    static void on_message(const Message* msg, void* user_data) {
        auto* self = static_cast<WhenAnyBusAwaitableT*>(user_data);
        /* 消息 / 取消 / 超时共享 ctl_->fired：只有第一个胜者触发 resume */
        if (!self->ctl_->try_fire(AwaitStatus::Ready)) {
            return; /* 已被其他 topic / 取消 / 超时率先触发 */
        }
        self->received_msg_ = *msg;
        schedule_resume(self->handle_);
    }

    MessageBus*               bus_;
    std::vector<std::string>  topics_;
    CancelToken*              cancel_;
    uint64_t                  timeout_us_;
    std::shared_ptr<AwaitCtl> ctl_;
    uint64_t                  cancel_id_{0};
    uint64_t                  timer_id_{0};
    uint64_t                  t_suspend_us_{0};
    std::coroutine_handle<>   handle_{};
    Message                   received_msg_{};
};

using WhenAnyBusAwaitable = WhenAnyBusAwaitableT<false>;

/**
 * 工厂函数：等待给定 topic 列表中最先到达的那条消息。
 *
 * 用法：
 *   Message msg = co_await when_any_bus(bus, {"sensor/lidar", "sensor/gps"});
 */
inline WhenAnyBusAwaitableT<false> when_any_bus(MessageBus* bus,
                                                std::initializer_list<const char*> topics,
                                                CancelToken* cancel = nullptr) {
    return WhenAnyBusAwaitableT<false>{
        bus, std::vector<std::string>(topics.begin(), topics.end()), cancel};
}

/**
 * 带超时的多 topic 竞争等待：返回 AwaitResult，可区分 Ready/Cancelled/Timeout。
 *
 * 用法：
 *   auto r = co_await when_any_bus_for(bus, {"sensor/lidar","sensor/gps"}, 50000, token);
 *   if (r.timed_out()) watchdog_fire();
 */
inline WhenAnyBusAwaitableT<true> when_any_bus_for(MessageBus* bus,
                                                   std::initializer_list<const char*> topics,
                                                   uint64_t timeout_us,
                                                   CancelToken* cancel = nullptr) {
    return WhenAnyBusAwaitableT<true>{
        bus, std::vector<std::string>(topics.begin(), topics.end()), cancel, timeout_us};
}

/* ─────────────────────────────────────────────────────────
 * 2d. DelayAwaitable — co_await delay_ms(ms) 定时挂起
 *
 * 不占用线程地挂起指定时长后自动恢复。若传入 CancelToken，
 * stop() 可提前唤醒；await_resume 返回 true 表示正常到时，
 * false 表示被取消。
 * ───────────────────────────────────────────────────────── */
class DelayAwaitable {
public:
    explicit DelayAwaitable(uint64_t timeout_us, CancelToken* cancel = nullptr)
        : timeout_us_(timeout_us), cancel_(cancel),
          ctl_(std::make_shared<AwaitCtl>()) {}

    bool await_ready() const noexcept { return timeout_us_ == 0; }

    void await_suspend(std::coroutine_handle<> h) {
        t_suspend_us_ = coro_now_us();
        if (cancel_) {
            auto ctl = ctl_;
            cancel_id_ = cancel_->add_waiter([ctl, h] {
                if (ctl->try_fire(AwaitStatus::Cancelled)) schedule_resume(h);
            });
        }
        auto ctl = ctl_;
        timer_id_ = TimerService::instance().add(timeout_us_, [ctl, h] {
            if (ctl->try_fire(AwaitStatus::Ready)) schedule_resume(h);
        });
    }

    /** @return true=正常到时，false=被取消 */
    bool await_resume() {
        if (cancel_)   cancel_->remove_waiter(cancel_id_);
        if (timer_id_) TimerService::instance().cancel(timer_id_);
        if (cancel_)   cancel_->record_resume(coro_now_us() - t_suspend_us_);
        return ctl_->status == AwaitStatus::Ready;
    }

private:
    uint64_t                  timeout_us_;
    CancelToken*              cancel_;
    std::shared_ptr<AwaitCtl> ctl_;
    uint64_t                  cancel_id_{0};
    uint64_t                  timer_id_{0};
    uint64_t                  t_suspend_us_{0};
};

/** co_await delay_us(500) — 挂起 500 微秒 */
inline DelayAwaitable delay_us(uint64_t us, CancelToken* cancel = nullptr) {
    return DelayAwaitable{us, cancel};
}
/** co_await delay_ms(10) — 挂起 10 毫秒 */
inline DelayAwaitable delay_ms(uint64_t ms, CancelToken* cancel = nullptr) {
    return DelayAwaitable{ms * 1000ULL, cancel};
}

/* ─────────────────────────────────────────────────────────
 * 2e. run_blocking — 在后台执行阻塞调用
 *
 * FLOWCORO：投入线程池；标准模式：分离线程。用于把同步阻塞的
 * message_bus_request 转成 awaitable，而不阻塞协程所在线程。
 * ───────────────────────────────────────────────────────── */
inline void run_blocking(std::function<void()> fn) {
#ifdef FLOWCORO_INTEGRATION
    flowcoro_integration::get_thread_pool().enqueue_void(std::move(fn));
#else
    std::thread(std::move(fn)).detach();
#endif
}

/* ─────────────────────────────────────────────────────────
 * 2f. RequestAwaitable — co_await request(...) 请求/应答
 *
 * 把 MessageBus 同步 req/reply 包装成 awaitable：在后台线程发起
 * 阻塞请求，收到回复（或超时）后恢复协程。返回 AwaitResult：
 *   - ok()        → message 为回复内容
 *   - timed_out() → 请求超时
 *   - cancelled() → 被 stop() 取消（后台请求仍会自行超时结束）
 *
 * 生命周期安全：后台线程仅捕获值拷贝与共享控制块/回复缓冲，
 * 不引用 awaitable 本身，即便协程已取消退出也无悬垂访问。
 * ───────────────────────────────────────────────────────── */
class RequestAwaitable {
public:
    RequestAwaitable(MessageBus* bus, const char* topic, const char* sender,
                     const void* data, uint32_t size, uint32_t timeout_ms,
                     CancelToken* cancel = nullptr)
        : bus_(bus), topic_(topic), sender_(sender ? sender : "coro"),
          timeout_ms_(timeout_ms), cancel_(cancel),
          ctl_(std::make_shared<AwaitCtl>()),
          reply_(std::make_shared<Message>()) {
        if (data && size) {
            data_.assign(static_cast<const uint8_t*>(data),
                         static_cast<const uint8_t*>(data) + size);
        }
    }

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) {
        t_suspend_us_ = coro_now_us();
        if (cancel_) {
            auto ctl = ctl_;
            cancel_id_ = cancel_->add_waiter([ctl, h] {
                if (ctl->try_fire(AwaitStatus::Cancelled)) schedule_resume(h);
            });
        }
        /* 捕获值拷贝 + 共享块，绝不捕获 this */
        auto ctl    = ctl_;
        auto reply  = reply_;
        auto bus    = bus_;
        auto topic  = topic_;
        auto sender = sender_;
        auto data   = data_;
        auto tmo    = timeout_ms_;
        run_blocking([ctl, reply, bus, topic, sender, data, tmo, h] {
            Message rep{};
            int rc = message_bus_request(bus, topic.c_str(), sender.c_str(),
                                         data.empty() ? nullptr : data.data(),
                                         static_cast<uint32_t>(data.size()),
                                         &rep, tmo);
            if (ctl->try_fire(rc == 0 ? AwaitStatus::Ready : AwaitStatus::Timeout)) {
                *reply = rep;
                schedule_resume(h);
            }
        });
    }

    AwaitResult await_resume() {
        if (cancel_) cancel_->remove_waiter(cancel_id_);
        if (cancel_) cancel_->record_resume(coro_now_us() - t_suspend_us_);
        return AwaitResult{ctl_->status, *reply_};
    }

private:
    MessageBus*               bus_;
    std::string               topic_;
    std::string               sender_;
    std::vector<uint8_t>      data_;
    uint32_t                  timeout_ms_;
    CancelToken*              cancel_;
    std::shared_ptr<AwaitCtl> ctl_;
    std::shared_ptr<Message>  reply_;
    uint64_t                  cancel_id_{0};
    uint64_t                  t_suspend_us_{0};
};

/**
 * 工厂函数：发起请求/应答。
 *   auto r = co_await request(bus, "service/plan", "ctrl", &req, sizeof(req), 2000, token);
 *   if (r.ok()) use(r.message);
 */
inline RequestAwaitable request(MessageBus* bus, const char* topic, const char* sender,
                                const void* data, uint32_t size, uint32_t timeout_ms,
                                CancelToken* cancel = nullptr) {
    return RequestAwaitable{bus, topic, sender, data, size, timeout_ms, cancel};
}

/* ─────────────────────────────────────────────────────────
 * 3. CoroutineTask — C++ 协程任务基类
 *    继承自 TaskBase，兼容 C 插件接口
 * ───────────────────────────────────────────────────────── */

class CoroutineTask {
public:
    explicit CoroutineTask(MessageBus* bus = nullptr)
        : bus_(bus), stop_flag_(false) {
        cancel_token_.set_stats(&coro_stats_);
    }

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
        cancel_token_.reset();
        coro_stats_.reset();
        task_ = std::make_unique<Task>(run());

#ifdef FLOWCORO_INTEGRATION
        /* flowcoro::CoroTask 惰性启动：在调用线程上触发首次 resume，
         * 协程跑到第一个 co_await 后挂起，控制权回到此处。 */
        if (task_->handle && !task_->handle.done()) {
            task_->handle.resume();
        }
#endif

        /* 事件循环：优先由 stop()/notify() 精确唤醒，50ms 兜底轮询防止
         * 协程自行完成（未调用 stop()）时长时间不被察觉。 */
        {
            std::unique_lock<std::mutex> lk(coro_mutex_);
            coro_cv_.wait_for(lk, std::chrono::milliseconds(50),
                              [this] { return task_->done() || stop_flag_.load(); });
            while (!task_->done() && !stop_flag_.load()) {
                coro_cv_.wait_for(lk, std::chrono::milliseconds(50),
                                  [this] { return task_->done() || stop_flag_.load(); });
            }
        }
#ifdef FLOWCORO_INTEGRATION
        if (task_->handle && task_->handle.promise().exception_) {
            std::rethrow_exception(task_->handle.promise().exception_);
        }
#else
        task_->rethrow_if_exception();
#endif
    }

    /** 请求停止（可从任意线程调用）：唤醒悬挂协程并通知事件循环 */
    void stop() {
        stop_flag_ = true;
        cancel_token_.request_cancel();
        coro_cv_.notify_all();
    }

    bool should_stop() const { return stop_flag_; }

    MessageBus* bus() const { return bus_; }

    /** 供 awaitable 使用的取消令牌（stop() 时被触发）。 */
    CancelToken* cancel_token() { return &cancel_token_; }

    /** 协程可观测统计：resume 次数与挂起时长分布（供 flowctl / 宿主读取）。 */
    CoroStats&   coro_stats() { return coro_stats_; }
    uint64_t     resume_count() { return coro_stats_.resume_count(); }
    LatencyStats coro_latency() { return coro_stats_.suspend_latency(); }

    /** 唤醒事件循环（协程 resume 后调用，可加快响应） */
    void notify() { coro_cv_.notify_all(); }

    /** 显式销毁协程帧（释放 BusChannel 等订阅资源）。
     *  必须在 message_bus_destroy(bus) 之前调用：协程帧析构会触发
     *  BusChannel 析构 → message_bus_unsubscribe_ex(bus, ...)，此时
     *  bus 仍需存活，否则会对已释放的 sub_mutex 加锁（use-after-free）。
     *  仅在 execute() 返回后调用；idempotent。 */
    void reset() { task_.reset(); }

    /* ── 便捷成员工厂：自动注入本任务的 CancelToken，使 stop() 可取消 ──
     * 子类在 run() 中优先使用这些方法（而非全局工厂），即可获得
     * "无需外发消息即可优雅停止"的能力。 */

    /** 可取消的一次性订阅。 */
    BusAwaitableT<false> next(const char* topic) {
        return BusAwaitableT<false>{bus_, topic, &cancel_token_};
    }
    /** 可取消 + 超时的一次性订阅。 */
    BusAwaitableT<true> next_for(const char* topic, uint64_t timeout_us) {
        return BusAwaitableT<true>{bus_, topic, &cancel_token_, timeout_us};
    }
    /** 可取消的多 topic 竞争等待。 */
    WhenAnyBusAwaitableT<false> select(std::initializer_list<const char*> topics) {
        return WhenAnyBusAwaitableT<false>{
            bus_, std::vector<std::string>(topics.begin(), topics.end()), &cancel_token_};
    }
    /** 可取消 + 超时的多 topic 竞争等待。 */
    WhenAnyBusAwaitableT<true> select_for(std::initializer_list<const char*> topics,
                                          uint64_t timeout_us) {
        return WhenAnyBusAwaitableT<true>{
            bus_, std::vector<std::string>(topics.begin(), topics.end()),
            &cancel_token_, timeout_us};
    }
    /** 可取消的定时挂起。 */
    DelayAwaitable sleep_us(uint64_t us) { return DelayAwaitable{us, &cancel_token_}; }
    DelayAwaitable sleep_ms(uint64_t ms) { return DelayAwaitable{ms * 1000ULL, &cancel_token_}; }
    /** 可取消的请求/应答。 */
    RequestAwaitable ask(const char* topic, const char* sender,
                         const void* data, uint32_t size, uint32_t timeout_ms) {
        return RequestAwaitable{bus_, topic, sender, data, size, timeout_ms, &cancel_token_};
    }

protected:
    MessageBus*                  bus_;
    std::atomic<bool>            stop_flag_;
    CancelToken                  cancel_token_;
    CoroStats                    coro_stats_;
    std::unique_ptr<Task>        task_;
    std::mutex                   coro_mutex_;
    std::condition_variable      coro_cv_;
};

/* ─────────────────────────────────────────────────────────
 * 4. FlowCoroTask — 基于 flowcoro 线程池的协程任务基类
 *    仅在 FLOWCORO_INTEGRATION 启用时可用
 * ───────────────────────────────────────────────────────── */

#ifdef FLOWCORO_INTEGRATION

/* CompletionNotifier — 用于安全等待协程真正结束的辅助协程。
 *
 * 问题：flowcoro::CoroTask 的 FinalAwaiter 在对称转移（symmetric transfer）
 * 前先将帧的 resume 指针清零（done() = true），再调用 await_suspend 返回续体。
 * 若 execute() 线程此时读取 task_->done() 发现为 true 并继续销毁任务，
 * 而线程池工作线程仍在 await_suspend 内部执行续体返回流程，就会出现数据竞争。
 *
 * 解决方案：在 execute() 调用前将 CompletionNotifier 句柄设为 task_ 的
 * continuation。协程完成时，FinalAwaiter 通过对称转移将控制权交给 notifier。
 * notifier 在 final_suspend 的 await_suspend 中原子地设置 frame_done_ 并唤醒 cv。
 *
 * 关键：frame_done_ 由 final_suspend::await_suspend 设置，此刻 notifier 帧已
 * 到达最终挂起点（resume 指针已清零，帧处于稳定状态，不再被工作线程访问）。
 * 因此 execute() 看到 frame_done_==true 后可直接销毁 notifier 帧，
 * 无需自旋等待 handle.done()——后者是对协程帧状态的非原子读，
 * 与工作线程的写操作构成数据竞争（UB，-O2 下编译器可能提升加载导致
 * 死循环或过早 destroy 仍在被写的帧，引发堆损坏）。
 */
struct CompletionNotifier {
    struct promise_type {
        std::atomic<bool>*       done_flag{nullptr};
        std::condition_variable* cv{nullptr};

        CompletionNotifier get_return_object() noexcept {
            return CompletionNotifier{
                std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_always initial_suspend() noexcept { return {}; }

        /* 自定义 final_suspend：帧到达最终挂起点后原子地发出完成信号。
         * 标准保证 await_suspend 执行时协程帧已处于稳定挂起状态，
         * 此处 store(release) 与 execute() 线程的 load(acquire) 建立
         * happens-before，使后续 handle.destroy() 安全。返回 noop_coroutine
         * 释放 worker 线程回线程池，不再访问 notifier 帧。 */
        struct FinalAwaiter {
            bool await_ready() noexcept { return false; }
            std::coroutine_handle<> await_suspend(
                std::coroutine_handle<promise_type> h) noexcept {
                auto& p = h.promise();
                if (p.done_flag) {
                    p.done_flag->store(true, std::memory_order_release);
                    p.cv->notify_all();
                }
                return std::noop_coroutine();
            }
            void await_resume() noexcept {}
        };
        FinalAwaiter final_suspend() noexcept { return {}; }

        void return_void() noexcept {
            /* 信号已移至 final_suspend，此处无需操作 */
        }
        void unhandled_exception() noexcept {}
    };

    std::coroutine_handle<promise_type> handle;

    explicit CompletionNotifier(std::coroutine_handle<promise_type> h) noexcept
        : handle(h) {}
    CompletionNotifier(const CompletionNotifier&)            = delete;
    CompletionNotifier& operator=(const CompletionNotifier&) = delete;
    CompletionNotifier(CompletionNotifier&& o) noexcept
        : handle(o.handle) { o.handle = nullptr; }

    ~CompletionNotifier() {
        /* execute() 已等待 frame_done_（由 final_suspend::await_suspend 设置），
         * 此时 notifier 帧处于稳定挂起状态，可直接 destroy，
         * 无需自旋 handle.done()（避免非原子读 UB）。 */
        if (handle) {
            handle.destroy();
        }
    }
};

/* 工厂函数：创建一个空体协程，挂起于 initial_suspend 等待被续体激活。
 * 激活后执行空 co_return，触发 promise_type::return_void() 发出完成信号。 */
inline CompletionNotifier make_completion_notifier(
    std::atomic<bool>* flag, std::condition_variable* cv)
{
    // 这里必须是协程（含 co_return），由编译器生成帧并挂起于 initial_suspend
    co_return;  // 空体：唯一目的是触发 return_void()
    // cppcheck-suppress unreachableCode
    (void)flag; (void)cv;  // 参数通过 promise 传递，此处仅用于满足语法
}

/**
 * FlowCoroTask 在 CoroutineTask 基础上：
 *   - Task 类型为 flowcoro::CoroTask（惰性启动，支持 continuation 链）。
 *   - execute() 将协程的首次 resume 提交到 flowcoro 线程池，
 *     协程体从一开始就在工作线程上执行，不阻塞调用线程。
 *   - BusAwaitable 将后续每次 resume 也提交到线程池，
 *     消息总线分发线程永不被协程逻辑阻塞。
 *   - execute() 通过 CompletionNotifier 无竞争地等待协程完成。
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
        stop_flag_   = false;
        frame_done_.store(false, std::memory_order_relaxed);
        cancel_token_.reset();
        coro_stats_.reset();
        task_ = std::make_unique<Task>(run());

        /* 将 CompletionNotifier 设为 task_ 的 continuation。
         * 协程完成时 FinalAwaiter 对称转移到 notifier，
         * notifier.return_void() 在工作线程上设置 frame_done_ 并唤醒 cv。
         * 这样 execute() 无需轮询 task_->done()（避免与帧写操作的竞争）。 */
        auto notifier = make_completion_notifier(&frame_done_, &coro_cv_);
        notifier.handle.promise().done_flag = &frame_done_;
        notifier.handle.promise().cv        = &coro_cv_;
        task_->handle.promise().continuation = notifier.handle;

        /* flowcoro::CoroTask 惰性启动（suspend_always）：
         * 将首次 resume 提交到线程池，协程体立即在工作线程上开始执行，
         * 调用线程（通常是 task_manager 的启动线程）不被阻塞。 */
        if (task_->handle && !task_->handle.done()) {
            auto h = task_->handle;
            flowcoro_integration::get_thread_pool().enqueue_void(
                [h]() mutable { if (!h.done()) h.resume(); });
        }

        /* 事件循环：stop()/notify() 精确唤醒 + 50ms 兜底轮询。
         * 使用 frame_done_（而非 task_->done()）等待协程完成，
         * 避免在工作线程仍执行 FinalAwaiter 时读取协程帧状态。 */
        {
            std::unique_lock<std::mutex> lk(coro_mutex_);
            coro_cv_.wait_for(lk, std::chrono::milliseconds(50),
                              [this] {
                                  return frame_done_.load(std::memory_order_acquire)
                                      || stop_flag_.load(std::memory_order_relaxed);
                              });
            while (!frame_done_.load(std::memory_order_acquire)
                   && !stop_flag_.load(std::memory_order_relaxed)) {
                coro_cv_.wait_for(lk, std::chrono::milliseconds(50),
                                  [this] {
                                      return frame_done_.load(std::memory_order_acquire)
                                          || stop_flag_.load(std::memory_order_relaxed);
                                  });
            }
            /* 二次等待：stop_flag_ 触发退出后，协程仍在工作线程上运行，
             * 必须等到 frame_done_ 为真（notifier 已被工作线程激活并完成），
             * 才能安全销毁 task_ 及其他共享状态。 */
            while (!frame_done_.load(std::memory_order_acquire)) {
                coro_cv_.wait_for(lk, std::chrono::milliseconds(50),
                                  [this] {
                                      return frame_done_.load(std::memory_order_acquire);
                                  });
            }
        }

        if (task_->handle && task_->handle.promise().exception_) {
            std::rethrow_exception(task_->handle.promise().exception_);
        }
    }

private:
    std::atomic<bool> frame_done_{false};
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
