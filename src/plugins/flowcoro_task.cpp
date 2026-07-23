/**
 * @file flowcoro_task.cpp
 * @brief flowcoro 协程任务插件示例
 *
 * 演示如何基于 CoroutineTask 实现一个消息驱动的协程任务插件：
 *   - 继承 CoroutineTask，只需实现 run() 协程
 *   - 使用 BusChannel 持久订阅消息总线，循环 co_await ch.recv() 消费消息
 *   - 消息到达时，resume 在 flowcoro 线程池工作线程执行，
 *     不阻塞消息总线的分发线程
 *   - 通过 EXPORT_COROUTINE_TASK 宏导出 C 接口，供 launcher 动态加载
 *
 * 与 subscribe_once 版本的区别：
 *   - BusChannel 在构造时订阅一次，析构时自动注销（RAII）。
 *   - 内部消息队列缓冲最多 32 条消息，不丢帧。
 *   - 无需在每次循环中重复注册/注销回调，降低分发线程压力。
 *
 * 编译时需要 -DFLOWCORO_INTEGRATION（CMakeLists.txt 已自动添加）。
 */

#include "coroutine_task.h"
#include "task_interface.h"

#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstring>

/* ═══════════════════════════════════════════════════════════
 * 1. 协程任务实现
 * ═══════════════════════════════════════════════════════════ */

/**
 * SensorFusionTask — 使用 BusChannel 持续消费传感器数据
 *
 * 功能：
 *   - 为 sensor/lidar 和 sensor/gps 各创建一个 BusChannel（持久订阅）
 *   - 用 when_any_bus 竞争等待两个通道，哪个先来就先处理
 *   - 支持优雅停止（should_stop()）
 */
class SensorFusionTask : public CoroutineTask {
public:
    explicit SensorFusionTask(MessageBus* bus) : CoroutineTask(bus) {}

protected:
    Task run() override {
        std::cout << "[SensorFusion] 协程启动，线程 ID: "
                  << std::this_thread::get_id() << "\n";

        /* BusChannel：持久订阅，RAII 生命周期 */
        BusChannel lidar_ch(bus(), "sensor/lidar", 32);
        BusChannel gps_ch  (bus(), "sensor/gps",   16);

        int lidar_frames = 0;
        int gps_frames   = 0;

        while (!should_stop()) {
            /* ── 竞争等待：哪个传感器先来就处理哪个 ─────────── */
            Message msg = co_await when_any_bus(bus(), {"sensor/lidar", "sensor/gps"});

            if (std::string(msg.topic) == "sensor/lidar") {
                lidar_frames++;
                std::ostringstream oss;
                oss << "[SensorFusion] LiDAR 帧 #" << lidar_frames
                    << " | 线程 ID: " << std::this_thread::get_id()
                    << " | seq: " << msg.msg_id;
                std::cout << oss.str() << "\n";
            } else {
                gps_frames++;
                std::ostringstream oss;
                oss << "[SensorFusion] GPS  帧 #" << gps_frames
                    << " | 线程 ID: " << std::this_thread::get_id()
                    << " | seq: " << msg.msg_id;
                std::cout << oss.str() << "\n";
            }

            /* ── 每 5 轮打印融合统计 ─────────────────────── */
            if ((lidar_frames + gps_frames) % 5 == 0) {
                std::cout << "[SensorFusion] 统计: LiDAR=" << lidar_frames
                          << " GPS=" << gps_frames << "\n";
            }
        }

        std::cout << "[SensorFusion] 协程退出，共处理 LiDAR="
                  << lidar_frames << " GPS=" << gps_frames << "\n";
    }
};

/* ═══════════════════════════════════════════════════════════
 * 2. C 接口导出（供 launcher/process_manager 动态加载）
 * ═══════════════════════════════════════════════════════════ */

EXPORT_COROUTINE_TASK(SensorFusionTask, sensor_fusion)
