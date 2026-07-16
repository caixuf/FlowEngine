/**
 * fusion_node.cpp — EKF 传感器融合节点插件 (FlowCoro 协程版)
 *
 * 从 fusion_node.c 迁移而来，采用 CoroutineTask 协程框架替代 pthread + condvar：
 *   - co_await select_for({"sensor/lidar","sensor/gps"}, 100ms) 替代 pthread_cond_timedwait
 *   - 保留 MessageBuffer 时间对齐逻辑（message_buffer_find_nearest）
 *   - on_lidar/on_gps 回调仍 push 到 buf，但删除 cond_signal（协程订阅负责唤醒）
 *
 * 输入 topics: sensor/lidar, sensor/gps
 * 输出 topics: fusion/localization, fusion/latency
 *
 * 算法: EKF 5D 卡尔曼滤波 (ekf_fusion.c)
 *   状态向量: [x, y, v, heading, yaw_rate]
 *   LiDAR 观测: (x, y)
 *   GPS 观测:   (speed, heading)
 *
 * 采用 FlowCoroTask（线程池 resume），而非 CoroutineTask（同步 resume）：
 * EKF 计算 + 序列化 + transport_publish 单次 ~50-100μs，同步 resume 会在
 * 消息总线分发线程上执行完整协程体，阻塞后续消息分发，累积导致 drops。
 * FlowCoroTask 将 resume 交给 flowcoro 无锁线程池，总线分发线程只触发不阻塞。
 * 与 safety_control_node 不同——后者纯限幅 <10μs，同步 resume 无阻塞风险。
 */

#include "node_plugin.h"
#include "ekf_fusion.h"
#include "fusion.h"
#include "adas_msgs_gen.h"
#include "scheduler.h"
#include "transport.h"
#include "discovery.h"
#include "coroutine_task.h"
#undef LOG_TRACE
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_FATAL
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#include <memory>
#include <string>

namespace {

/* ── 节点状态 (进程本地，无跨节点耦合) ──────────────────────── */

struct FusionContext {
    /* 注入的基础设施 */
    Transport*        transport{nullptr};
    DiscoveryManager* discovery{nullptr};
    Scheduler*        scheduler{nullptr};

    /* 协程宿主线程 */
    pthread_t         thread{};
    bool              running{false};
    std::atomic<bool> should_stop{false};

    /* EKF */
    EkfFusion   ekf{};

    /* 消息缓冲区 */
    MessageBuffer* lidar_buf{nullptr};
    MessageBuffer* gps_buf{nullptr};

    /* 统计 */
    uint32_t fused_count{0};
    double   fused_x{0}, fused_y{0}, fused_v{0}, fused_heading{0}, fused_yaw_rate{0};

    /* 延迟统计 — 环形缓冲，128 个样本 */
    LatencyTracker  lat_tracker{};

    /* 协程任务 */
    std::unique_ptr<class FusionTask> task;
};

FusionContext g;

/* ── Topic 订阅回调 ──────────────────────────────────────────── */
/* 仅 push 到 MessageBuffer；协程的 select_for 内部订阅负责唤醒，无需 cond_signal。 */

static void on_lidar(const Message* msg, void* user_data) {
    (void)user_data;
    message_buffer_push(g.lidar_buf, msg);
}

static void on_gps(const Message* msg, void* user_data) {
    (void)user_data;
    message_buffer_push(g.gps_buf, msg);
}

/* ── 协程任务 ────────────────────────────────────────────────── */

class FusionTask : public FlowCoroTask {
public:
    FusionTask(MessageBus* bus, Transport* transport,
               MessageBuffer* lidar_buf, MessageBuffer* gps_buf,
               EkfFusion* ekf, LatencyTracker* lat_tracker)
        : FlowCoroTask(bus), transport_(transport),
          lidar_buf_(lidar_buf), gps_buf_(gps_buf),
          ekf_(ekf), lat_tracker_(lat_tracker) {}

protected:
    Task run() override {
        LOG_INFO("fusion", "FlowCoro fusion started (thread-pool resume)");

        while (!should_stop()) {
            /* 替代 pthread_cond_timedwait(100ms)：100ms 超时作 watchdog，
             * 防止 lidar 停发时协程卡死。select_for 自动注入 cancel_token_，
             * stop() 可立即唤醒。 */
            auto res = co_await select_for({"sensor/lidar", "sensor/gps"}, 100000);
            (void)res;  /* 唤醒即可，数据从 MessageBuffer 读取 */
            if (should_stop()) break;

            const Message* lidar_msg = message_buffer_latest(lidar_buf_);
            if (!lidar_msg) continue;

            uint64_t ref_ts = lidar_msg->timestamp_us;

            /* 时间对齐: 找最近 GPS (50ms 窗口) */
            const Message* gps_msg = message_buffer_find_nearest(gps_buf_, ref_ts, 50000);

            /* 类型安全访问 (C 宏版 msg_cast，与原 fusion_node.c 一致) */
            const LidarFrame* lidar = (const LidarFrame*)
                _msg_cast_impl(lidar_msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
            const GpsData* gps = gps_msg ? (const GpsData*)
                _msg_cast_impl(gps_msg, GPSDATA_TYPE_ID, sizeof(GpsData), "GpsData") : nullptr;
            if (!lidar) continue;

            /* ── EKF 预测 ── */
            ekf_fusion_predict(ekf_);

            /* ── LiDAR 位置更新 ── */
            ekf_fusion_update_lidar(ekf_, (double)lidar->x, (double)lidar->y, nullptr);

            /* ── GPS 速度/航向更新 ── */
            if (gps) {
                double heading_rad = (double)gps->heading_deg * M_PI / 180.0;
                ekf_fusion_update_gps(ekf_, (double)gps->speed_mps, heading_rad, nullptr);
            }

            /* 读取融合结果 */
            double x, y, v, h, yr, diag[5];
            ekf_fusion_get_state(ekf_, &x, &y, &v, &h, &yr);
            ekf_fusion_get_covariance_diag(ekf_, diag);

            g.fused_x = x; g.fused_y = y; g.fused_v = v;
            g.fused_heading = h; g.fused_yaw_rate = yr;
            g.fused_count++;

            /* EKF 发散恢复 */
            if (ekf_->diverged && g.fused_count % 10 == 0) {
                LOG_WARN("fusion", "EKF diverged (trace=%.0f) — resetting", diag[0]+diag[1]);
                ekf_fusion_reset(ekf_);
            }

            /* ── 发布融合结果 (二进制序列化) ── */
            Localization loc;
            loc.x          = (float)x;
            loc.y          = (float)y;
            loc.v          = (float)v;
            loc.heading    = (float)h;
            loc.yaw_rate   = (float)yr;
            loc.cov_xx     = (float)diag[0];
            loc.cov_yy     = (float)diag[1];
            loc.cov_vv     = (float)diag[2];
            loc.cov_hh     = (float)diag[3];
            loc.cov_yyaw   = (float)diag[4];
            loc.innovation = (float)ekf_->last_innovation;
            loc.diverged   = ekf_->diverged;
            loc.raw_pos_x  = (float)(lidar ? lidar->x : x);
            loc.raw_pos_y  = (float)(lidar ? lidar->y : y);
            loc.raw_speed  = (float)(gps ? (double)gps->speed_mps : v);

            uint8_t loc_buf[128];
            size_t  loc_len = sizeof(loc_buf);
            Localization_serialize(&loc, loc_buf, &loc_len);
            transport_publish(transport_, "fusion/localization",
                              loc_buf, (uint32_t)loc_len);

            if (g.fused_count % 50 == 0) {
                LOG_INFO("fusion", "#%u EKF:(%.1f,%.1f) v=%.1f hdg=%.1f° innov=%.2f",
                         g.fused_count, x, y, v, h * 180.0 / M_PI, ekf_->last_innovation);
            }

            /* ── 延迟跟踪 + 上报 (每 20 帧) ── */
            if (gps_msg && gps_msg->timestamp_us > 0) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t wall = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
                if (wall > gps_msg->timestamp_us)
                    latency_tracker_record(lat_tracker_, wall - gps_msg->timestamp_us);
            }
            if (g.fused_count % 20 == 0) {
                LatencyStats ls = latency_tracker_stats(lat_tracker_);
                LatencyReport lr;
                lr.avg_us = (uint32_t)ls.avg_us;
                lr.p50_us = (uint32_t)ls.p50_us;
                lr.p99_us = (uint32_t)ls.p99_us;

                uint8_t lr_buf[32];
                size_t  lr_len = sizeof(lr_buf);
                LatencyReport_serialize(&lr, lr_buf, &lr_len);
                transport_publish(transport_, "fusion/latency",
                                  lr_buf, (uint32_t)lr_len);
            }
        }

        LOG_INFO("fusion", "FlowCoro fusion stopped (%u fused frames)", g.fused_count);
    }

private:
    Transport*     transport_;
    MessageBuffer* lidar_buf_;
    MessageBuffer* gps_buf_;
    EkfFusion*     ekf_;
    LatencyTracker* lat_tracker_;
};

/* ── 协程宿主线程 ─────────────────────────────────────────────── */

void* fusion_thread(void*) {
    pthread_setname_np(pthread_self(), "fusion");
    try {
        g.task->execute();
    } catch (...) {
        LOG_ERROR("fusion", "FlowCoro task failed");
    }
    return nullptr;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "sensor/lidar", "sensor/gps", nullptr };
static const char* s_outputs[] = { "fusion/localization", "fusion/latency", nullptr };

extern NodePlugin s_plugin;  /* 前向声明：定义在文件末尾，供 init/start 引用 */

static int fusion_init(MessageBus* bus, Transport* transport,
                       DiscoveryManager* discovery, Scheduler* scheduler,
                       const char* params_json) {
    (void)params_json;

    g.transport  = transport;
    g.discovery  = discovery;
    g.scheduler  = scheduler;
    g.should_stop = false;
    g.running    = false;
    g.fused_count = 0;

    /* EKF: dt=0.05s, 初始状态 [0,0,5,0,0] */
    double x0[5] = {0.0, 0.0, 5.0, 0.0, 0.0};
    ekf_fusion_init(&g.ekf, 0.05, x0);

    /* 消息缓冲区 */
    g.lidar_buf = message_buffer_create("sensor/lidar", LIDARFRAME_TYPE_ID, 32, 5000000);
    g.gps_buf   = message_buffer_create("sensor/gps",   GPSDATA_TYPE_ID,    16, 5000000);
    if (!g.lidar_buf || !g.gps_buf) return -1;

    /* 延迟跟踪器 — 环形缓冲128样本 */
    memset(&g.lat_tracker, 0, sizeof(g.lat_tracker));

    /* 订阅输入 topics — 回调仅 push 到 buf，协程订阅负责唤醒 */
    transport_subscribe(transport, "sensor/lidar", on_lidar, nullptr);
    transport_subscribe(transport, "sensor/gps",   on_gps,   nullptr);

    /* Discovery 广告 */
    discovery_advertise(discovery, "sensor/lidar", LIDARFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "sensor/gps",   GPSDATA_TYPE_ID,   CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u, CAP_FUSION | CAP_PUBLISHER, 10.0);
    discovery_advertise(discovery, "fusion/latency",      0x1A7E9C01u, CAP_PUBLISHER, 2.0);

    /* 发布 topic 广告 */
    transport_advertise(transport, "fusion/localization", 0xF0ED10C0u);
    transport_advertise(transport, "fusion/latency",      0x1A7E9C01u);

    /* 构造协程任务 */
    g.task = std::make_unique<FusionTask>(bus, transport,
                                          g.lidar_buf, g.gps_buf,
                                          &g.ekf, &g.lat_tracker);

    LOG_INFO("fusion", "initialized (FlowCoro, EKF 5D, aligned LiDAR+GPS)");
    return 0;
}

static int fusion_start(void) {
    if (!g.task) return -1;
    g.should_stop = false;
    if (pthread_create(&g.thread, nullptr, fusion_thread, nullptr) != 0) {
        LOG_WARN("fusion", "failed to create thread");
        return -1;
    }
    g.running = true;
    LOG_INFO("fusion", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void fusion_stop(void) {
    g.should_stop = true;
    if (g.task) g.task->stop();  /* 触发 CancelToken 唤醒挂起的 select_for */
}

static void fusion_cleanup(void) {
    fusion_stop();
    if (g.running) {
        pthread_join(g.thread, nullptr);
        g.running = false;
    }
    g.task.reset();
    if (g.lidar_buf) { message_buffer_destroy(g.lidar_buf); g.lidar_buf = nullptr; }
    if (g.gps_buf)   { message_buffer_destroy(g.gps_buf);   g.gps_buf   = nullptr; }
    LOG_INFO("fusion", "cleanup done");
}

static int fusion_health(void) {
    /* 简单检查: 最近 5 秒是否有融合帧输出 */
    return (g.fused_count > 0) ? 0 : 1;
}

/* ── 导出入口 ────────────────────────────────────────────────── */

NodePlugin s_plugin = {
    NODE_PLUGIN_API_VERSION,
    "fusion",
    "1.0.0",
    "EKF 5D sensor fusion (LiDAR + GPS) [FlowCoro]",
    s_inputs,
    s_outputs,
    fusion_init,
    fusion_start,
    fusion_stop,
    fusion_cleanup,
    fusion_health,
};

} // namespace

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
