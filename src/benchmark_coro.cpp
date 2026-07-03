/**
 * @file benchmark_coro.cpp
 * @brief FlowEngine 协程路径性能基准测试
 *
 * 对标 benchmark.c（普通路径），测量 flowcoro 协程集成层的实际性能：
 *
 *  Bench 1: BusChannel 端到端吞吐量（分批发布，有反压）
 *  Bench 2: subscribe_once vs BusChannel 单次端到端延迟对比
 *  Bench 3: when_any_bus 多 topic 竞争等待延迟
 *  Bench 4: 重回调场景吞吐量（每条消息模拟 200µs 处理，flowcoro 线程池并行）
 *  Bench 5: 多并发 FlowCoroTask 扩展性（1/2/4 任务消费同一 topic）
 *
 * 关键设计：每个任务都有 done_flag + done_cv，协程在退出前发出信号；
 * main 等到 done_flag 后再销毁 MessageBus，避免 use-after-free 崩溃。
 */

#include "coroutine_task.h"
#include "message_bus.h"

#include <cstdio>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <numeric>
#include <time.h>

/* ── 计时工具 ─────────────────────────────────────────────── */

static uint64_t now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ── 统计计算 ─────────────────────────────────────────────── */

struct Stats { uint64_t min_ns, max_ns, avg_ns, p50_ns, p99_ns; };

static Stats calc_stats(std::vector<uint64_t>& s) {
    std::sort(s.begin(), s.end());
    size_t n = s.size();
    uint64_t sum = std::accumulate(s.begin(), s.end(), uint64_t{0});
    return { s.front(), s.back(), sum/n, s[n/2], s[(size_t)((n-1)*0.99)] };
}

/* ── 格式化输出 ───────────────────────────────────────────── */

static void print_sep() {
    printf("─────────────────────────────────────────────────────────────────────\n");
}
static void print_bench_header(const char* title) {
    printf("\n"); print_sep(); printf("  %s\n", title); print_sep();
}
static void print_latency_row(const char* label, Stats s) {
    printf("  %-46s min=%6.1f  avg=%6.1f  p50=%6.1f  p99=%6.1f  max=%6.1f  (µs)\n",
           label,
           s.min_ns/1e3, s.avg_ns/1e3, s.p50_ns/1e3, s.p99_ns/1e3, s.max_ns/1e3);
}

/* ── 安全停止辅助函数 ─────────────────────────────────────────
 * 停止 task + 发一条唤醒消息（让协程从 co_await 醒来检查 should_stop()）
 * + 等待协程自身的 done_flag 后再 join 线程并销毁总线。
 * ─────────────────────────────────────────────────────────── */
static void safe_stop(FlowCoroTask& task, std::thread& t,
                      MessageBus* bus, const char* wakeup_topic,
                      std::atomic<bool>& done_flag,
                      std::mutex& done_mtx, std::condition_variable& done_cv,
                      int timeout_s = 10) {
    task.stop();
    if (wakeup_topic)
        message_bus_publish(bus, wakeup_topic, "bench", nullptr, 0);

    {
        std::unique_lock<std::mutex> lk(done_mtx);
        done_cv.wait_for(lk, std::chrono::seconds(timeout_s),
                         [&]{ return done_flag.load(std::memory_order_acquire); });
    }
    t.join();
}

/* ══════════════════════════════════════════════════════════
 * Bench 1: BusChannel 端到端吞吐量（分批有反压）
 * ══════════════════════════════════════════════════════════ */

class Bench1Task : public FlowCoroTask {
public:
    Bench1Task(MessageBus* bus, int target,
               std::atomic<int>& cnt,
               std::atomic<bool>& done_flag,
               std::mutex& mtx, std::condition_variable& cv)
        : FlowCoroTask(bus), target_(target), cnt_(cnt),
          done_flag_(done_flag), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        BusChannel ch(bus(), "b1/msg", 512);
        while (!should_stop()) {
            co_await ch.recv();
            int n = ++cnt_;
            if (n >= target_) break;
        }
        done_flag_.store(true, std::memory_order_release);
        { std::unique_lock<std::mutex> lk(mtx_); cv_.notify_all(); }
        stop();
    }

private:
    int target_;
    std::atomic<int>&  cnt_;
    std::atomic<bool>& done_flag_;
    std::mutex& mtx_;
    std::condition_variable& cv_;
};

static void bench_buschannel_throughput() {
    print_bench_header("Bench 1: BusChannel 端到端吞吐量 (FlowCoroTask + 分批有反压)");

    const int BATCH  = 100;
    const int ROUNDS = 50;
    const int TOTAL  = BATCH * ROUNDS;

    std::atomic<int>  cnt{0};
    std::atomic<bool> done_flag{false};
    std::mutex mtx; std::condition_variable cv;

    MessageBus* bus = message_bus_create("b1");
    Bench1Task task(bus, TOTAL, cnt, done_flag, mtx, cv);
    std::thread t([&]{ task.execute(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uint8_t payload[64]{};
    uint64_t t0 = now_ns();

    for (int r = 0; r < ROUNDS; ++r) {
        int target_r = (r + 1) * BATCH;
        for (int i = 0; i < BATCH; ++i)
            message_bus_publish(bus, "b1/msg", "bench", payload, sizeof(payload));
        while (cnt.load(std::memory_order_relaxed) < target_r)
            std::this_thread::yield();
    }

    uint64_t t1 = now_ns();
    safe_stop(task, t, bus, "b1/msg", done_flag, mtx, cv);

    double elapsed_s = (t1 - t0) / 1e9;
    printf("  总消息数:       %d\n", TOTAL);
    printf("  总耗时:         %.3f ms\n", elapsed_s * 1e3);
    printf("  端到端吞吐量:   %.0f 消息/秒\n", TOTAL / elapsed_s);
    printf("  平均端到端延迟: %.1f µs/消息\n", elapsed_s * 1e6 / TOTAL);

    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Bench 2: subscribe_once vs BusChannel 单次延迟对比
 * ══════════════════════════════════════════════════════════ */

class Bench2OnceTask : public FlowCoroTask {
public:
    Bench2OnceTask(MessageBus* bus, int n,
                   std::vector<uint64_t>& samples,
                   std::atomic<bool>& started,
                   std::atomic<bool>& done_flag,
                   std::mutex& mtx, std::condition_variable& cv)
        : FlowCoroTask(bus), n_(n), samples_(samples),
          started_(started), done_flag_(done_flag), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        started_.store(true, std::memory_order_release);
        for (int i = 0; i < n_ && !should_stop(); ++i) {
            Message msg = co_await subscribe_once(bus(), "b2/once");
            samples_[i] = now_ns() - msg.timestamp_us * 1000ULL;
        }
        done_flag_.store(true, std::memory_order_release);
        { std::unique_lock<std::mutex> lk(mtx_); cv_.notify_all(); }
        stop();
    }

private:
    int n_;
    std::vector<uint64_t>& samples_;
    std::atomic<bool>& started_;
    std::atomic<bool>& done_flag_;
    std::mutex& mtx_;
    std::condition_variable& cv_;
};

class Bench2ChanTask : public FlowCoroTask {
public:
    Bench2ChanTask(MessageBus* bus, int n,
                   std::vector<uint64_t>& samples,
                   std::atomic<bool>& started,
                   std::atomic<bool>& done_flag,
                   std::mutex& mtx, std::condition_variable& cv)
        : FlowCoroTask(bus), n_(n), samples_(samples),
          started_(started), done_flag_(done_flag), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        BusChannel ch(bus(), "b2/chan", 4);
        started_.store(true, std::memory_order_release);
        for (int i = 0; i < n_ && !should_stop(); ++i) {
            Message msg = co_await ch.recv();
            samples_[i] = now_ns() - msg.timestamp_us * 1000ULL;
        }
        done_flag_.store(true, std::memory_order_release);
        { std::unique_lock<std::mutex> lk(mtx_); cv_.notify_all(); }
        stop();
    }

private:
    int n_;
    std::vector<uint64_t>& samples_;
    std::atomic<bool>& started_;
    std::atomic<bool>& done_flag_;
    std::mutex& mtx_;
    std::condition_variable& cv_;
};

static void drive_latency(MessageBus* bus, const char* topic,
                           std::atomic<bool>& started, int n,
                           int interval_ms = 20) {
    while (!started.load(std::memory_order_acquire))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    uint8_t payload[8]{};
    for (int i = 0; i < n; ++i) {
        message_bus_publish(bus, topic, "bench", payload, sizeof(payload));
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
}

static void bench_subscribe_once_vs_channel() {
    print_bench_header("Bench 2: subscribe_once vs BusChannel 单次延迟对比");

    const int N = 200;

    /* ── subscribe_once ── */
    {
        std::vector<uint64_t> samples(N);
        std::atomic<bool> started{false}, done_flag{false};
        std::mutex mtx; std::condition_variable cv;

        MessageBus* bus = message_bus_create("b2a");
        Bench2OnceTask task(bus, N, samples, started, done_flag, mtx, cv);
        std::thread t([&]{ task.execute(); });

        drive_latency(bus, "b2/once", started, N);

        { std::unique_lock<std::mutex> lk(mtx);
          cv.wait_for(lk, std::chrono::seconds(30),
                      [&]{ return done_flag.load(std::memory_order_acquire); }); }
        safe_stop(task, t, bus, "b2/once", done_flag, mtx, cv);

        Stats s = calc_stats(samples);
        print_latency_row("subscribe_once (N=200)", s);
        message_bus_destroy(bus);
    }

    /* ── BusChannel ── */
    {
        std::vector<uint64_t> samples(N);
        std::atomic<bool> started{false}, done_flag{false};
        std::mutex mtx; std::condition_variable cv;

        MessageBus* bus = message_bus_create("b2b");
        Bench2ChanTask task(bus, N, samples, started, done_flag, mtx, cv);
        std::thread t([&]{ task.execute(); });

        drive_latency(bus, "b2/chan", started, N);

        { std::unique_lock<std::mutex> lk(mtx);
          cv.wait_for(lk, std::chrono::seconds(30),
                      [&]{ return done_flag.load(std::memory_order_acquire); }); }
        safe_stop(task, t, bus, "b2/chan", done_flag, mtx, cv);

        Stats s = calc_stats(samples);
        print_latency_row("BusChannel     (N=200)", s);
        message_bus_destroy(bus);
    }

    printf("  (延迟含 co_await 恢复时的线程池调度开销)\n");
}

/* ══════════════════════════════════════════════════════════
 * Bench 3: when_any_bus 多 topic 竞争等待延迟
 * ══════════════════════════════════════════════════════════ */

class Bench3Task : public FlowCoroTask {
public:
    Bench3Task(MessageBus* bus, int n,
               std::vector<uint64_t>& samples,
               std::atomic<bool>& started,
               std::atomic<bool>& done_flag,
               std::mutex& mtx, std::condition_variable& cv)
        : FlowCoroTask(bus), n_(n), samples_(samples),
          started_(started), done_flag_(done_flag), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        started_.store(true, std::memory_order_release);
        for (int i = 0; i < n_ && !should_stop(); ++i) {
            Message msg = co_await when_any_bus(bus(), {"b3/a", "b3/b"});
            samples_[i] = now_ns() - msg.timestamp_us * 1000ULL;
        }
        done_flag_.store(true, std::memory_order_release);
        { std::unique_lock<std::mutex> lk(mtx_); cv_.notify_all(); }
        stop();
    }

private:
    int n_;
    std::vector<uint64_t>& samples_;
    std::atomic<bool>& started_;
    std::atomic<bool>& done_flag_;
    std::mutex& mtx_;
    std::condition_variable& cv_;
};

static void bench_when_any_latency() {
    print_bench_header("Bench 3: when_any_bus 多 topic 竞争等待延迟");

    const int N = 200;
    std::vector<uint64_t> samples(N);
    std::atomic<bool> started{false}, done_flag{false};
    std::mutex mtx; std::condition_variable cv;

    MessageBus* bus = message_bus_create("b3");
    Bench3Task task(bus, N, samples, started, done_flag, mtx, cv);
    std::thread t([&]{ task.execute(); });

    while (!started.load(std::memory_order_acquire))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    uint8_t payload[8]{};
    for (int i = 0; i < N; ++i) {
        const char* topic = (i % 2 == 0) ? "b3/a" : "b3/b";
        message_bus_publish(bus, topic, "bench", payload, sizeof(payload));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    { std::unique_lock<std::mutex> lk(mtx);
      cv.wait_for(lk, std::chrono::seconds(30),
                  [&]{ return done_flag.load(std::memory_order_acquire); }); }
    safe_stop(task, t, bus, "b3/a", done_flag, mtx, cv);

    Stats s = calc_stats(samples);
    print_latency_row("when_any_bus (N=200, 2 topics)", s);
    printf("  等效最大 QPS: %.0f 次/秒\n", 1e9 / s.avg_ns);

    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Bench 4: 重回调场景吞吐量
 * ══════════════════════════════════════════════════════════ */

class Bench4Task : public FlowCoroTask {
public:
    Bench4Task(MessageBus* bus, int target,
               std::atomic<int>& cnt,
               std::atomic<bool>& done_flag,
               std::mutex& mtx, std::condition_variable& cv)
        : FlowCoroTask(bus), target_(target), cnt_(cnt),
          done_flag_(done_flag), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        BusChannel ch(bus(), "b4/heavy", 256);
        while (!should_stop()) {
            co_await ch.recv();
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            int n = ++cnt_;
            if (n >= target_) {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.notify_all();
                break;
            }
        }
        done_flag_.store(true, std::memory_order_release);
        { std::unique_lock<std::mutex> lk(mtx_); cv_.notify_all(); }
        stop();
    }

private:
    int target_;
    std::atomic<int>&  cnt_;
    std::atomic<bool>& done_flag_;
    std::mutex& mtx_;
    std::condition_variable& cv_;
};

static void bench_heavy_callback_throughput() {
    print_bench_header("Bench 4: 重回调场景吞吐量 (每条 200µs, flowcoro 线程池并行)");

    const int MSGS_PER_WORKER = 100;
    const int NUM_WORKERS     = (int)std::thread::hardware_concurrency();
    const int TOTAL           = MSGS_PER_WORKER * NUM_WORKERS;

    std::atomic<int>  cnt{0};
    std::mutex mtx; std::condition_variable cv;

    MessageBus* bus = message_bus_create("b4");

    std::vector<std::atomic<bool>> done_flags(NUM_WORKERS);
    for (auto& f : done_flags) f.store(false);

    std::vector<std::unique_ptr<Bench4Task>> tasks;
    std::vector<std::thread> threads;

    for (int i = 0; i < NUM_WORKERS; ++i) {
        tasks.push_back(std::make_unique<Bench4Task>(
            bus, TOTAL, cnt, done_flags[i], mtx, cv));
    }
    for (auto& task : tasks)
        threads.emplace_back([&t = *task]{ t.execute(); });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    uint8_t payload[8]{};
    uint64_t t0 = now_ns();

    const int BATCH = 50;
    for (int sent = 0; sent < TOTAL; ) {
        int batch = std::min(BATCH, TOTAL - sent);
        for (int i = 0; i < batch; ++i)
            message_bus_publish(bus, "b4/heavy", "bench", payload, sizeof(payload));
        sent += batch;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    { std::unique_lock<std::mutex> lk(mtx);
      cv.wait_for(lk, std::chrono::seconds(60),
                  [&]{ return cnt.load() >= TOTAL; }); }
    uint64_t t1 = now_ns();

    for (int i = 0; i < NUM_WORKERS; ++i) {
        tasks[i]->stop();
        message_bus_publish(bus, "b4/heavy", "bench", nullptr, 0);
    }
    for (int i = 0; i < NUM_WORKERS; ++i) {
        auto dl = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!done_flags[i].load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < dl)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (auto& th : threads) th.join();

    double elapsed_s = (t1 - t0) / 1e9;
    double throughput = cnt.load() / elapsed_s;
    printf("  线程池工作线程数: %d\n", NUM_WORKERS);
    printf("  总消息数:         %d (每 worker %d 条)\n", TOTAL, MSGS_PER_WORKER);
    printf("  总耗时:           %.3f ms\n", elapsed_s * 1e3);
    printf("  有效吞吐量:       %.0f 消息/秒\n", throughput);
    printf("  等效串行耗时:     %.1f ms (若无并行: %d×200µs=%.1f ms)\n",
           elapsed_s * 1e3, TOTAL, TOTAL * 0.2);

    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Bench 5: 多并发 FlowCoroTask 扩展性
 * ══════════════════════════════════════════════════════════ */

class Bench5Task : public FlowCoroTask {
public:
    Bench5Task(MessageBus* bus, int target,
               std::atomic<int>& cnt,
               std::atomic<bool>& done_flag,
               std::mutex& mtx, std::condition_variable& cv)
        : FlowCoroTask(bus), target_(target), cnt_(cnt),
          done_flag_(done_flag), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        BusChannel ch(bus(), "b5/fanout", 128);
        while (!should_stop()) {
            co_await ch.recv();
            int n = ++cnt_;
            if (n >= target_) {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.notify_all();
            }
        }
        done_flag_.store(true, std::memory_order_release);
        { std::unique_lock<std::mutex> lk(mtx_); cv_.notify_all(); }
        stop();
    }

private:
    int target_;
    std::atomic<int>&  cnt_;
    std::atomic<bool>& done_flag_;
    std::mutex& mtx_;
    std::condition_variable& cv_;
};

static void bench_concurrent_tasks() {
    print_bench_header("Bench 5: 多并发 FlowCoroTask 扩展性 (N 任务消费同一 topic)");

    const int MSGS_PER_WORKER = 2000;

    for (int num_workers : {1, 2, 4}) {
        int total = MSGS_PER_WORKER * num_workers;
        std::atomic<int>  cnt{0};
        std::mutex mtx; std::condition_variable cv;

        MessageBus* bus = message_bus_create("b5");

        std::vector<std::atomic<bool>> done_flags(num_workers);
        for (auto& f : done_flags) f.store(false);

        std::vector<std::unique_ptr<Bench5Task>> tasks;
        std::vector<std::thread> threads;

        for (int i = 0; i < num_workers; ++i) {
            tasks.push_back(std::make_unique<Bench5Task>(
                bus, total, cnt, done_flags[i], mtx, cv));
        }
        for (auto& task : tasks)
            threads.emplace_back([&t = *task]{ t.execute(); });

        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        uint8_t payload[8]{};
        uint64_t t0 = now_ns();

        const int BATCH = 100;
        for (int sent = 0; sent < total; ) {
            int batch = std::min(BATCH, total - sent);
            for (int i = 0; i < batch; ++i)
                message_bus_publish(bus, "b5/fanout", "bench", payload, sizeof(payload));
            sent += batch;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        { std::unique_lock<std::mutex> lk(mtx);
          cv.wait_for(lk, std::chrono::seconds(30),
                      [&]{ return cnt.load() >= total; }); }
        uint64_t t1 = now_ns();

        for (int i = 0; i < num_workers; ++i) {
            tasks[i]->stop();
            message_bus_publish(bus, "b5/fanout", "bench", nullptr, 0);
        }
        for (int i = 0; i < num_workers; ++i) {
            auto dl = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (!done_flags[i].load(std::memory_order_acquire) && std::chrono::steady_clock::now() < dl)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        for (auto& th : threads) th.join();

        double elapsed_s = (t1 - t0) / 1e9;
        printf("  workers=%d  总消息=%d  耗时=%.1f ms  吞吐=%.0f 消息/秒\n",
               num_workers, total, elapsed_s * 1e3, cnt.load() / elapsed_s);

        message_bus_destroy(bus);
    }
}

/* ── 主程序 ─────────────────────────────────────────────── */

int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║         FlowEngine 协程路径性能基准测试 (flowcoro)               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n\n");

    bench_buschannel_throughput();
    bench_subscribe_once_vs_channel();
    bench_when_any_latency();
    bench_heavy_callback_throughput();
    bench_concurrent_tasks();

    printf("\n");
    printf("─────────────────────────────────────────────────────────────────────\n");
    printf("  协程基准测试完成\n");
    printf("─────────────────────────────────────────────────────────────────────\n\n");
    return 0;
}
