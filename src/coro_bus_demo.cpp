/**
 * @file coro_bus_demo.cpp
 * @brief flowcoro 协程 + 消息总线综合演示
 *
 * 演示 FlowCoroTask 与 MessageBus 的完整集成：
 *
 *   1. 创建消息总线 "adas_bus"
 *   2. 注册两个 FlowCoroTask：
 *      - LidarProcessTask：co_await sensor/lidar，在线程池线程处理点云
 *      - FusionTask：轮流 co_await sensor/lidar 和 sensor/gps，融合数据
 *   3. 主线程以 10 Hz 发布激光雷达帧、5 Hz 发布 GPS 帧
 *   4. 所有协程 resume 均在 flowcoro 线程池中执行，
 *      消息总线分发线程不被阻塞
 *
 * 编译：CMakeLists.txt 中已定义 coro_bus_demo 目标并启用 FLOWCORO_INTEGRATION。
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
 * LidarProcessTask — 专用激光雷达点云处理协程
 *
 * 每收到一帧 sensor/lidar 消息，在 flowcoro 工作线程上处理点云数据。
 * 打印处理线程 ID，验证协程 resume 确实发生在线程池而非分发线程。
 */
class LidarProcessTask : public FlowCoroTask {
public:
    explicit LidarProcessTask(MessageBus* bus, int max_frames = -1)
        : FlowCoroTask(bus), max_frames_(max_frames) {}

protected:
    Task run() override {
        std::cout << "[LidarTask] 协程启动 on 线程 "
                  << std::this_thread::get_id() << "\n";

        int frame = 0;
        while (!should_stop()) {
            Message msg = co_await subscribe_once(bus(), "sensor/lidar");
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
    }

private:
    int max_frames_;
};

/**
 * FusionTask — 多传感器融合协程
 *
 * 交替等待 sensor/lidar 和 sensor/gps，模拟传感器融合处理。
 * 每轮融合后打印汇总信息。
 */
class FusionTask : public FlowCoroTask {
public:
    explicit FusionTask(MessageBus* bus, int max_rounds = -1)
        : FlowCoroTask(bus), max_rounds_(max_rounds) {}

protected:
    Task run() override {
        std::cout << "[FusionTask] 协程启动 on 线程 "
                  << std::this_thread::get_id() << "\n";

        int round = 0;
        while (!should_stop()) {
            /* 等待激光雷达 */
            Message lidar = co_await subscribe_once(bus(), "sensor/lidar");

            /* 等待 GPS */
            Message gps = co_await subscribe_once(bus(), "sensor/gps");

            round++;
            {
                std::ostringstream oss;
                oss << "[FusionTask] 融合 #" << round
                    << " LiDAR_seq=" << lidar.msg_id
                    << " GPS_seq=" << gps.msg_id
                    << " 线程=" << std::this_thread::get_id();
                std::cout << oss.str() << "\n";
            }

            if (max_rounds_ > 0 && round >= max_rounds_) break;
        }

        std::cout << "[FusionTask] 退出，共融合 " << round << " 轮\n";
    }

private:
    int max_rounds_;
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
    try { w->impl->execute(); return 0; }
    catch (...) { return -1; }
}

template <typename CoroTaskT>
static void coro_stop(TaskBase* b) {
    auto* w = reinterpret_cast<CoroWrapper<CoroTaskT>*>(b);
    w->impl->stop();
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
    constexpr int MAX_LIDAR_FRAMES = 20;
    constexpr int MAX_FUSION_ROUNDS = 8;

    auto lidar_task   = std::make_unique<LidarProcessTask>(bus, MAX_LIDAR_FRAMES);
    auto fusion_task  = std::make_unique<FusionTask>(bus, MAX_FUSION_ROUNDS);

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
        /* 每 tick（100ms）发一帧激光雷达 */
        {
            Message msg{};
            msg.msg_id = static_cast<uint32_t>(++lidar_seq);
            snprintf(msg.topic,  sizeof(msg.topic),  "sensor/lidar");
            snprintf(msg.sender, sizeof(msg.sender), "lidar_driver");
            msg.data_size = 0;
            message_bus_publish(bus, msg.topic, msg.sender, nullptr, 0);
        }

        /* 每两个 tick 发一帧 GPS */
        if (tick % 2 == 0) {
            Message msg{};
            msg.msg_id = static_cast<uint32_t>(++gps_seq);
            snprintf(msg.topic,  sizeof(msg.topic),  "sensor/gps");
            snprintf(msg.sender, sizeof(msg.sender), "gps_driver");
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
    lidar_task->stop();
    fusion_task->stop();
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
