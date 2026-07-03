/**
 * @file flowcoro_task.cpp
 * @brief flowcoro 协程任务插件示例
 *
 * 演示如何基于 FlowCoroTask 实现一个消息驱动的协程任务插件：
 *   - 继承 FlowCoroTask，只需实现 run() 协程
 *   - 使用 co_await subscribe_once() 等待消息总线消息
 *   - 消息到达时，resume 在 flowcoro 线程池工作线程执行，
 *     不阻塞消息总线的分发线程
 *   - 通过 EXPORT_COROUTINE_TASK 宏导出 C 接口，供 launcher 动态加载
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
 * SensorFusionTask — 模拟传感器融合协程任务
 *
 * 功能：
 *   - 同时等待 sensor/lidar 和 sensor/gps 消息
 *   - 每收到一帧激光雷达数据，打印线程 ID（验证在线程池中执行）
 *   - 每收到一帧 GPS 数据，打印坐标信息
 *   - 支持优雅停止（should_stop()）
 */
class SensorFusionTask : public FlowCoroTask {
public:
    explicit SensorFusionTask(MessageBus* bus) : FlowCoroTask(bus) {}

protected:
    Task run() override {
        std::cout << "[SensorFusion] 协程启动，线程 ID: "
                  << std::this_thread::get_id() << "\n";

        int lidar_frames = 0;
        int gps_frames   = 0;

        while (!should_stop()) {
            /* ── 等待激光雷达帧 ────────────────────────────── */
            Message lidar_msg = co_await subscribe_once(bus(), "sensor/lidar");
            lidar_frames++;

            {
                std::ostringstream oss;
                oss << "[SensorFusion] LiDAR 帧 #" << lidar_frames
                    << " | 线程 ID: " << std::this_thread::get_id()
                    << " | topic: " << lidar_msg.topic
                    << " | seq: " << lidar_msg.msg_id;
                std::cout << oss.str() << "\n";
            }

            /* ── 等待 GPS 定位 ──────────────────────────────── */
            Message gps_msg = co_await subscribe_once(bus(), "sensor/gps");
            gps_frames++;

            {
                std::ostringstream oss;
                oss << "[SensorFusion] GPS 帧 #" << gps_frames
                    << " | 线程 ID: " << std::this_thread::get_id()
                    << " | topic: " << gps_msg.topic;
                std::cout << oss.str() << "\n";
            }

            /* ── 每 5 轮打印融合统计 ─────────────────────── */
            if (lidar_frames % 5 == 0) {
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
