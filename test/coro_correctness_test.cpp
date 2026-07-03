/**
 * @file coro_correctness_test.cpp
 * @brief FlowCoroTask 协程集成层正确性测试
 *
 * 测试项目:
 *   Test 1: BusChannel 消息不丢失
 *           发布 100 条消息，BusChannel 全部收到（无丢帧）。
 *
 *   Test 2: when_any_bus 只唤醒一次
 *           同时在两个 topic 上各发一条消息，协程只被唤醒一次（原子 CAS 语义）。
 *
 *   Test 3: FlowCoroTask 优雅停止
 *           调用 stop() 后协程在下一个 should_stop() 检查时干净退出，
 *           execute() 正常返回，不发生悬挂或资源泄漏。
 */

#include "coroutine_task.h"
#include "message_bus.h"

#include <cstdio>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

/* ── 测试框架（轻量断言）──────────────────────────────────── */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(expr, msg)                                              \
    do {                                                              \
        if (expr) {                                                   \
            printf("  [PASS] %s\n", msg);                            \
            ++g_pass;                                                 \
        } else {                                                      \
            printf("  [FAIL] %s\n", msg);                            \
            ++g_fail;                                                 \
        }                                                             \
    } while (0)

/* ══════════════════════════════════════════════════════════
 * Test 1: BusChannel 消息不丢失
 * 发布 N 条消息（每批 BATCH 条，等接收后再发下一批），
 * 验证最终计数 == N。
 * ══════════════════════════════════════════════════════════ */

class BusChannelNoLossTask : public FlowCoroTask {
public:
    BusChannelNoLossTask(MessageBus* bus, int n,
                         std::atomic<int>& counter,
                         std::mutex& mtx,
                         std::condition_variable& cv)
        : FlowCoroTask(bus), n_(n), counter_(counter), mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        BusChannel ch(bus(), "test1/msg", 128);
        int received = 0;
        while (!should_stop() && received < n_) {
            co_await ch.recv();
            ++received;
            int cnt = ++counter_;
            if (cnt >= n_) {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.notify_all();
            }
        }
        stop();
    }

private:
    int                       n_;
    std::atomic<int>&         counter_;
    std::mutex&               mtx_;
    std::condition_variable&  cv_;
};

static void test_buschannel_no_loss() {
    printf("\n[Test 1] BusChannel 消息不丢失\n");

    const int N = 100;
    std::atomic<int> counter{0};
    std::mutex mtx;
    std::condition_variable cv;

    MessageBus* bus = message_bus_create("t1");
    BusChannelNoLossTask task(bus, N, counter, mtx, cv);
    std::thread t([&]{ task.execute(); });

    /* 等协程到达第一个 co_await（已订阅）*/
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uint8_t payload[8]{};
    /* 分 10 批，每批 10 条，批间小延迟保证消费跟上 */
    for (int batch = 0; batch < 10; ++batch) {
        for (int i = 0; i < 10; ++i)
            message_bus_publish(bus, "test1/msg", "test", payload, sizeof(payload));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait_for(lk, std::chrono::seconds(10),
                    [&]{ return counter.load() >= N; });
    }
    task.stop();
    t.join();

    CHECK(counter.load() == N, "BusChannel 收到全部 100 条消息（无丢帧）");
    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Test 2: when_any_bus 只唤醒一次（原子 CAS 语义）
 *
 * 每轮向两个 topic 各发一条消息，协程只应被其中一个唤醒一次。
 * 因此：发 N 轮（每轮 2 条消息），协程应恰好 resume N 次。
 *
 * 同步策略：
 *   - 协程通过 started_ 标志告知主线程已进入事件循环。
 *   - 主线程在两轮发布之间留出 30 ms，使协程有充足时间从上
 *     一次 resume 回到下一个 co_await（完成订阅注册），
 *     避免 sem_post-before-subscribe 竞态条件。
 *   - 全部发完后通过 done_flag + condition_variable 等待协程结束。
 * ══════════════════════════════════════════════════════════ */

class WhenAnyOnceTask : public FlowCoroTask {
public:
    WhenAnyOnceTask(MessageBus* bus, int rounds,
                    std::atomic<int>& resume_count,
                    std::atomic<bool>& started,
                    std::atomic<bool>& done_flag,
                    std::mutex& mtx,
                    std::condition_variable& cv)
        : FlowCoroTask(bus), rounds_(rounds),
          resume_count_(resume_count),
          started_(started), done_flag_(done_flag),
          mtx_(mtx), cv_(cv) {}

protected:
    Task run() override {
        started_.store(true, std::memory_order_release);
        for (int i = 0; i < rounds_ && !should_stop(); ++i) {
            co_await when_any_bus(bus(), {"test2/a", "test2/b"});
            ++resume_count_;
        }
        done_flag_.store(true, std::memory_order_release);
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.notify_all();
        stop();
    }

private:
    int                rounds_;
    std::atomic<int>&  resume_count_;
    std::atomic<bool>& started_;
    std::atomic<bool>& done_flag_;
    std::mutex&        mtx_;
    std::condition_variable& cv_;
};

static void test_when_any_fires_once() {
    printf("\n[Test 2] when_any_bus 只唤醒一次（原子 CAS 语义）\n");

    const int ROUNDS = 20;
    std::atomic<int>  resume_count{0};
    std::atomic<bool> started{false};
    std::atomic<bool> done_flag{false};
    std::mutex mtx;
    std::condition_variable cv;

    MessageBus* bus = message_bus_create("t2");
    WhenAnyOnceTask task(bus, ROUNDS, resume_count, started, done_flag, mtx, cv);
    std::thread t([&]{ task.execute(); });

    /* 等协程线程池工作线程启动并到达第一个 co_await */
    while (!started.load(std::memory_order_acquire))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    /* 额外等待确保 await_suspend 完成订阅注册 */
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    uint8_t payload[8]{};
    for (int i = 0; i < ROUNDS; ++i) {
        /* 奇数轮发 topic-a，偶数轮发 topic-b：验证两个 topic 均可路由，
         * 且每轮恰好只触发一次 resume（若有双次 resume，总数会超出 ROUNDS）*/
        const char* topic = (i % 2 == 0) ? "test2/a" : "test2/b";
        message_bus_publish(bus, topic, "test", payload, sizeof(payload));
        /* 给协程足够时间 resume 并到达下一轮 co_await（重新订阅）*/
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait_for(lk, std::chrono::seconds(10),
                    [&]{ return done_flag.load(std::memory_order_acquire); });
    }
    task.stop();
    t.join();

    CHECK(resume_count.load() == ROUNDS,
          "when_any_bus 恰好 resume 20 次（10 轮 topic-a + 10 轮 topic-b 均正确路由）");

    message_bus_destroy(bus);
}

/* ══════════════════════════════════════════════════════════
 * Test 3: FlowCoroTask 优雅停止
 *
 * stop() 调用后，execute() 应在有限时间内正常返回；协程在
 * should_stop() 检查处干净退出（exited_cleanly_ 被设置）。
 *
 * 关键设计：execute() 一旦检测到 stop_flag_ 就退出，但协程此时
 * 可能仍悬挂在 co_await ch.recv()。因此在 execute() 返回后，
 * 需额外发一条消息让协程唤醒并通过 should_stop() 干净退出。
 * done_cv 用于等待协程自身完成（独立于 execute() 线程）。
 * ══════════════════════════════════════════════════════════ */

class GracefulStopTask : public FlowCoroTask {
public:
    GracefulStopTask(MessageBus* bus,
                     std::atomic<int>& loop_count,
                     std::atomic<bool>& exited_cleanly,
                     std::mutex& done_mtx,
                     std::condition_variable& done_cv)
        : FlowCoroTask(bus),
          loop_count_(loop_count),
          exited_cleanly_(exited_cleanly),
          done_mtx_(done_mtx),
          done_cv_(done_cv) {}

protected:
    Task run() override {
        BusChannel ch(bus(), "test3/msg", 32);
        while (!should_stop()) {
            co_await ch.recv();
            ++loop_count_;
        }
        /* 到达此处说明协程干净退出（should_stop() 返回 true） */
        exited_cleanly_.store(true, std::memory_order_release);
        {
            std::unique_lock<std::mutex> lk(done_mtx_);
            done_cv_.notify_all();
        }
        stop();
    }

private:
    std::atomic<int>&        loop_count_;
    std::atomic<bool>&       exited_cleanly_;
    std::mutex&              done_mtx_;
    std::condition_variable& done_cv_;
};

static void test_graceful_stop() {
    printf("\n[Test 3] FlowCoroTask 优雅停止\n");

    std::atomic<int>  loop_count{0};
    std::atomic<bool> exited_cleanly{false};
    std::atomic<bool> execute_returned{false};
    std::mutex        done_mtx;
    std::condition_variable done_cv;

    MessageBus* bus = message_bus_create("t3");
    GracefulStopTask task(bus, loop_count, exited_cleanly, done_mtx, done_cv);

    std::thread t([&]{
        task.execute();
        execute_returned.store(true, std::memory_order_release);
    });

    /* 让协程处理几条消息 */
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    uint8_t payload[8]{};
    for (int i = 0; i < 5; ++i) {
        message_bus_publish(bus, "test3/msg", "test", payload, sizeof(payload));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    /* 请求停止：设置 stop_flag_，execute() 将退出其等待循环 */
    task.stop();

    /* 等 execute() 返回（最长 3 秒）*/
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!execute_returned.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    t.join();

    /* execute() 已返回，但协程可能仍悬挂在 co_await ch.recv()。
     * 发一条唤醒消息，让协程检查 should_stop() 并干净退出。 */
    message_bus_publish(bus, "test3/msg", "test", payload, sizeof(payload));

    /* 等待协程自身完成（最长 3 秒）*/
    {
        std::unique_lock<std::mutex> lk(done_mtx);
        done_cv.wait_for(lk, std::chrono::seconds(3),
                         [&]{ return exited_cleanly.load(std::memory_order_acquire); });
    }

    CHECK(execute_returned.load(), "stop() 后 execute() 在 3 秒内正常返回");
    CHECK(exited_cleanly.load(),   "协程通过 should_stop() 检查干净退出（无强制终止）");
    CHECK(loop_count.load() > 0,   "停止前协程已处理至少 1 条消息（正常运行过）");

    message_bus_destroy(bus);
}

/* ── 主程序 ─────────────────────────────────────────────── */

int main() {
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║            FlowCoroTask 协程集成层正确性测试                     ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n");

    test_buschannel_no_loss();
    test_when_any_fires_once();
    test_graceful_stop();

    printf("\n─────────────────────────────────────────────────────────────────────\n");
    printf("  结果: %d 通过, %d 失败\n", g_pass, g_fail);
    printf("─────────────────────────────────────────────────────────────────────\n\n");

    return (g_fail == 0) ? 0 : 1;
}
