/**
 * @file coro_bus_demo.cpp
 * @brief flowcoro 协程 + 消息总线综合演示
 *
 * 演示 CoroutineTask 与 MessageBus 的完整集成，充分利用两个高级原语：
 *
 *   BusChannel   — 持久订阅 + 消息缓冲通道，适合持续消费单一 topic。
 *                  构造时一次订阅，每次 co_await ch.recv() 直接取下一条，
 *                  避免 subscribe_once 反复注册/注销的开销。
 *
 *   when_any_bus — 多 topic 竞争等待（select 语义）。
 *                  哪个 topic 先来消息，协程就被哪个唤醒，其余订阅自动注销。
 *
 * 场景：
 *   1. 创建消息总线 "adas_bus"
 *   2. LidarProcessTask：通过 BusChannel 持续接收 sensor/lidar，模拟点云处理。
 *   3. FusionTask      ：通过 when_any_bus 竞争等待 sensor/lidar 和 sensor/gps，
 *                        体现传感器融合中"哪个先来处理哪个"的事件驱动模型。
 *   4. 主线程以 10 Hz 发布激光雷达帧、5 Hz 发布 GPS 帧
 *   5. 所有协程 resume 在 flowcoro 线程池中执行，不阻塞总线分发线程
 */

#include "coroutine_task.h"
#include "message_bus.h"
#include "task_interface.h"
#include "task_manager.h"

#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cstdio>
#include <cstring>

/* ════════════════════════════════════════════════════════════
 * 全局停止标志
 * ════════════════════════════════════════════════════════════ */

static std::atomic<bool> g_stop{false};

static void signal_handler(int /*sig*/) { g_stop = true; }

/* ════════════════════════════════════════════════════════════
 * 协程任务定义
 * ════════════════════════════════════════════════════════════ */

/**
 * LidarProcessTask — 使用 BusChannel 持续接收激光雷达点云
 *
 * 与之前 subscribe_once 版本的区别：
 *   - 构造时一次订阅，析构时自动注销（RAII）。
 *   - 每次 co_await ch.recv() 从内部缓冲队列取消息，
 *     不再有循环订阅/注销的往返开销。
 *   - 消息积压时自动缓冲（默认 32 条），不丢帧。
 */
class LidarProcessTask : public CoroutineTask {
public:
    explicit LidarProcessTask(MessageBus* bus, int max_frames = -1)
        : CoroutineTask(bus), max_frames_(max_frames) {}

protected:
    Task run() override {
        std::cout << "[LidarTask] 协程启动 on 线程 "
                  << std::this_thread::get_id() << "\n";

        /* BusChannel 在此处构造，自动订阅 sensor/lidar */
        BusChannel lidar_ch(bus(), "sensor/lidar", 32);

        int frame = 0;
        while (!should_stop()) {
            /* co_await BusChannel::recv() — 有消息立即返回，否则挂起等待 */
            Message msg = co_await lidar_ch.recv();
            frame++;

            /* 模拟点云处理（在线程池工作线程中执行） */
            std::this_thread::sleep_for(std::chrono::microseconds(200));

            {
                std::ostringstream oss;
                oss << "[LidarTask] 帧#" << frame
                    << " seq=" << msg.msg_id
                    << " 处理线程=" << std::this_thread::get_id();
                std::cout << oss.str() << "\n";
            }

            if (max_frames_ > 0 && frame >= max_frames_) break;
        }

        std::cout << "[LidarTask] 退出，共处理 " << frame << " 帧\n";
        stop();
    }

private:
    int max_frames_;
};

/**
 * FusionTask — 使用 when_any_bus 实现多传感器竞争等待
 *
 * 与之前顺序等待版本的区别：
 *   - 之前：先等 lidar，再等 gps（串行，GPS 必须在 LiDAR 之后才处理）。
 *   - 现在：同时等待 lidar 和 gps，哪个先来就处理哪个（并行事件驱动）。
 *
 * 这正是自动驾驶中多传感器融合的真实模式：
 *   各传感器频率不同，融合算法不应被最慢的传感器阻塞。
 */
class FusionTask : public CoroutineTask {
public:
    explicit FusionTask(MessageBus* bus, int max_events = -1)
        : CoroutineTask(bus), max_events_(max_events) {}

protected:
    Task run() override {
        std::cout << "[FusionTask] 协程启动 on 线程 "
                  << std::this_thread::get_id() << "\n";

        int lidar_count = 0;
        int gps_count   = 0;
        int event       = 0;

        while (!should_stop()) {
            /* when_any_bus：同时订阅两个 topic，哪个先到唤醒协程 */
            Message msg = co_await when_any_bus(bus(), {"sensor/lidar", "sensor/gps"});
            event++;

            {
                std::ostringstream oss;
                oss << "[FusionTask] 事件#" << event
                    << " topic=" << msg.topic
                    << " seq=" << msg.msg_id
                    << " 线程=" << std::this_thread::get_id();
                std::cout << oss.str() << "\n";
            }

            if (std::string(msg.topic) == "sensor/lidar") {
                lidar_count++;
            } else {
                gps_count++;
            }

            if (max_events_ > 0 && event >= max_events_) break;
        }

        std::cout << "[FusionTask] 退出，共处理 LiDAR=" << lidar_count
                  << " GPS=" << gps_count << " 事件\n";
        stop();
    }

private:
    int max_events_;
};

/* ════════════════════════════════════════════════════════════
 * TaskBase 包装（让协程任务兼容 TaskManager 的线程管理）
 * ════════════════════════════════════════════════════════════ */

template <typename CoroTaskT>
struct CoroWrapper {
    TaskBase     base;
    CoroTaskT*   impl;
};

template <typename CoroTaskT>
static int coro_execute(TaskBase* b) {
    auto* w = reinterpret_cast<CoroWrapper<CoroTaskT>*>(b);
    try {
        flowcoro::rt::RtExecutor ex{{ .pin_cpu=-1, .idle_sleep_us=200 }};
        g_node_exec = &ex;
        ex.spawn(w->impl->run());
        while (!w->impl->should_stop()) ex.run();
        ex.shutdown();
        g_node_exec = nullptr;
        return 0;
    } catch (...) { return -1; }
}

template <typename CoroTaskT>
static void coro_stop(TaskBase* b) {
    auto* w = reinterpret_cast<CoroWrapper<CoroTaskT>*>(b);
    w->impl->set_stop();
}

template <typename CoroTaskT>
static bool coro_health(TaskBase* b) {
    auto* w = reinterpret_cast<CoroWrapper<CoroTaskT>*>(b);
    return !w->impl->should_stop();
}

/* ════════════════════════════════════════════════════════════
 * main
 * ════════════════════════════════════════════════════════════ */

int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    /* ── 打印 flowcoro 线程池信息 ─────────────────────────── */
    auto& pool = flowcoro_integration::get_thread_pool();
    std::cout << "╔══════════════════════════════════════════╗\n"
              << "║  flowcoro 协程 + 消息总线演示程序        ║\n"
              << "╚══════════════════════════════════════════╝\n"
              << "[Main] flowcoro 线程池工作线程数: "
              << pool.active_thread_count() << "\n\n";

    /* ── 创建消息总线 ─────────────────────────────────────── */
    MessageBus* bus = message_bus_create("adas_bus");
    if (!bus) {
        std::cerr << "创建消息总线失败\n";
        return 1;
    }
    std::cout << "[Main] 消息总线 'adas_bus' 创建成功\n";

    /* ── 创建协程任务 ─────────────────────────────────────── */
    constexpr int MAX_LIDAR_FRAMES  = 20;
    constexpr int MAX_FUSION_EVENTS = 15; /* when_any_bus 模式下每帧均计入 */

    auto lidar_task   = std::make_unique<LidarProcessTask>(bus, MAX_LIDAR_FRAMES);
    auto fusion_task  = std::make_unique<FusionTask>(bus, MAX_FUSION_EVENTS);

    /* ── TaskInterface vtable（模板实例化） ──────────────── */
    static const TaskInterface lidar_vtable = {
        nullptr,
        coro_execute<LidarProcessTask>,
        coro_stop<LidarProcessTask>,
        nullptr, nullptr, nullptr,
        coro_health<LidarProcessTask>,
        nullptr,
        nullptr   /* on_message */
    };
    static const TaskInterface fusion_vtable = {
        nullptr,
        coro_execute<FusionTask>,
        coro_stop<FusionTask>,
        nullptr, nullptr, nullptr,
        coro_health<FusionTask>,
        nullptr,
        nullptr   /* on_message */
    };

    CoroWrapper<LidarProcessTask>  lidar_wrapper{};
    CoroWrapper<FusionTask>        fusion_wrapper{};

    TaskConfig default_cfg{};
    default_cfg.auto_restart      = false;
    default_cfg.max_restart_count = 0;
    default_cfg.heartbeat_interval = 0;

    task_base_init(&lidar_wrapper.base,  &lidar_vtable,  &default_cfg);
    task_base_init(&fusion_wrapper.base, &fusion_vtable, &default_cfg);
    lidar_wrapper.impl  = lidar_task.get();
    fusion_wrapper.impl = fusion_task.get();

    /* ── 注册到 TaskManager ───────────────────────────────── */
    TaskManager* mgr = task_manager_create();
    task_manager_register(mgr, &lidar_wrapper.base,  "lidar_task");
    task_manager_register(mgr, &fusion_wrapper.base, "fusion_task");

    /* ── 启动协程任务（每个任务在独立线程中调用 execute()） */
    std::cout << "\n[Main] 启动协程任务...\n";
    task_manager_start_all(mgr);

    /* ── 发布者：主线程模拟传感器数据 ───────────────────── */
    std::cout << "[Main] 开始发布传感器数据（10 Hz LiDAR / 5 Hz GPS）\n\n";

    int lidar_seq = 0;
    int gps_seq   = 0;

    /* 运行直到两个协程都完成或用户中断 */
    for (int tick = 0; tick < 60 && !g_stop; ++tick) {
        /* 每两个 tick 发一帧 GPS。GPS tick 先发低频 topic，避免高频 LiDAR
         * 在 when_any_bus 演示中总是抢先唤醒 FusionTask。 */
        if (tick % 2 == 0) {
            Message msg{};
            msg.msg_id = static_cast<uint32_t>(++gps_seq);
            snprintf(msg.topic,  sizeof(msg.topic),  "sensor/gps");
            snprintf(msg.sender, sizeof(msg.sender), "gps_driver");
            msg.data_size = 0;
            message_bus_publish(bus, msg.topic, msg.sender, nullptr, 0);
        }

        /* 每 tick（100ms）发一帧激光雷达 */
        {
            Message msg{};
            msg.msg_id = static_cast<uint32_t>(++lidar_seq);
            snprintf(msg.topic,  sizeof(msg.topic),  "sensor/lidar");
            snprintf(msg.sender, sizeof(msg.sender), "lidar_driver");
            msg.data_size = 0;
            message_bus_publish(bus, msg.topic, msg.sender, nullptr, 0);
        }

        /* 检查协程是否已完成 */
        if (lidar_task->should_stop() && fusion_task->should_stop()) {
            std::cout << "[Main] 所有协程已完成，退出主循环\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /* ── 优雅停止 ─────────────────────────────────────────── */
    std::cout << "\n[Main] 停止所有任务...\n";
    lidar_task->set_stop();
    fusion_task->set_stop();
    task_manager_stop_all(mgr);

    /* ── 打印线程池状态 ───────────────────────────────────── */
    std::cout << "\n[Main] 最终线程池状态:\n";
    pool.print_status();

    /* ── 清理 ────────────────────────────────────────────── */
    task_manager_destroy(mgr);
    message_bus_destroy(bus);

    std::cout << "\n[Main] 演示结束。\n";
    return 0;
}
