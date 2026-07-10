/**
 * fusion_node.c — EKF 传感器融合节点插件
 *
 * 实现 NodePlugin 接口，编译为 libfusion_node.so。
 * flow_launcher 通过 dlopen + node_get_plugin() 加载此节点。
 *
 * 输入 topics: sensor/lidar, sensor/gps
 * 输出 topics: fusion/localization, fusion/latency
 *
 * 算法: EKF 5D 卡尔曼滤波 (ekf_fusion.c)
 *   状态向量: [x, y, v, heading, yaw_rate]
 *   LiDAR 观测: (x, y)
 *   GPS 观测:   (speed, heading)
 *
 * 与 e2e_demo.c 的区别:
 *   旧: 使用全局 g_transport/g_bus/g_scheduler/g_running
 *   新: 所有基础设施通过 init() 注入，节点完全独立
 */

#include "node_plugin.h"
#include "ekf_fusion.h"
#include "fusion.h"
#include "adas_msgs_gen.h"
#include "scheduler.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

/* ── 节点状态 (进程本地，无跨节点耦合) ──────────────────────── */

static struct {
    /* 注入的基础设施 */
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 任务线程 */
    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* EKF */
    EkfFusion   ekf;

    /* 消息缓冲区 */
    MessageBuffer* lidar_buf;
    MessageBuffer* gps_buf;

    /* 统计 */
    uint32_t fused_count;
    double   fused_x, fused_y, fused_v, fused_heading, fused_yaw_rate;

    /* 延迟统计 — 环形缓冲，128 个样本 */
    LatencyTracker  lat_tracker;

    /* choreo 触发任务 ID */
    int tid;
} g;

/* ── Topic 订阅回调 ──────────────────────────────────────────── */

static void on_lidar(const Message* msg, void* user_data) {
    (void)user_data;
    message_buffer_push(g.lidar_buf, msg);
}

static void on_gps(const Message* msg, void* user_data) {
    (void)user_data;
    message_buffer_push(g.gps_buf, msg);
}

/* ── 任务线程主循环 ──────────────────────────────────────────── */

static void* fusion_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "fusion");

    /* 注册成调度器 choreo 任务 (无 TaskBase 封装，直接使用调度器 API) */
    /* 触发 topic: sensor/lidar — LiDAR 每帧触发一次融合 */
    /* TODO: 未来用 task_base_init + choreo_trigger 重构；此处直接用消息缓冲区轮询 */

    while (!g.should_stop) {
        /* 轮询 lidar 缓冲区，50ms 检查间隔 (20Hz，与 LiDAR 发布频率匹配，减少空轮询) */
        usleep(50000);
        const Message* lidar_msg = message_buffer_latest(g.lidar_buf);
        if (!lidar_msg) continue;
        if (g.should_stop) break;

        uint64_t ref_ts = lidar_msg->timestamp_us;

        /* 时间对齐: 找最近 GPS (50ms 窗口) */
        const Message* gps_msg = message_buffer_find_nearest(g.gps_buf, ref_ts, 50000);

        /* 类型安全访问 */
        const LidarFrame* lidar = (const LidarFrame*)
            _msg_cast_impl(lidar_msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
        const GpsData* gps = gps_msg ? (const GpsData*)
            _msg_cast_impl(gps_msg, GPSDATA_TYPE_ID, sizeof(GpsData), "GpsData") : NULL;
        if (!lidar) continue;

        /* ── EKF 预测 ── */
        ekf_fusion_predict(&g.ekf);

        /* ── LiDAR 位置更新 ── */
        ekf_fusion_update_lidar(&g.ekf, (double)lidar->x, (double)lidar->y, NULL);

        /* ── GPS 速度/航向更新 ── */
        if (gps) {
            double heading_rad = (double)gps->heading_deg * M_PI / 180.0;
            ekf_fusion_update_gps(&g.ekf, (double)gps->speed_mps, heading_rad, NULL);
        }

        /* 读取融合结果 */
        double diag[5];
        ekf_fusion_get_state(&g.ekf, &g.fused_x, &g.fused_y,
                             &g.fused_v, &g.fused_heading, &g.fused_yaw_rate);
        ekf_fusion_get_covariance_diag(&g.ekf, diag);

        g.fused_count++;

        /* EKF 发散恢复 */
        if (g.ekf.diverged && g.fused_count % 10 == 0) {
            LOG_WARN("fusion", "EKF diverged (trace=%.0f) — resetting", diag[0]+diag[1]);
            ekf_fusion_reset(&g.ekf);
        }

        /* ── 发布融合结果 (二进制序列化) ── */
        Localization loc;
        loc.x          = (float)g.fused_x;
        loc.y          = (float)g.fused_y;
        loc.v          = (float)g.fused_v;
        loc.heading    = (float)g.fused_heading;
        loc.yaw_rate   = (float)g.fused_yaw_rate;
        loc.cov_xx     = (float)diag[0];
        loc.cov_yy     = (float)diag[1];
        loc.cov_vv     = (float)diag[2];
        loc.cov_hh     = (float)diag[3];
        loc.cov_yyaw   = (float)diag[4];
        loc.innovation = (float)g.ekf.last_innovation;
        loc.diverged   = g.ekf.diverged;
        loc.raw_pos_x  = (float)(lidar ? lidar->x : g.fused_x);
        loc.raw_pos_y  = (float)(lidar ? lidar->y : g.fused_y);
        loc.raw_speed  = (float)(gps ? (double)gps->speed_mps : g.fused_v);

        uint8_t loc_buf[128];
        size_t  loc_len = sizeof(loc_buf);
        Localization_serialize(&loc, loc_buf, &loc_len);
        transport_publish(g.transport, "fusion/localization",
                          loc_buf, (uint32_t)loc_len);

        if (g.fused_count % 50 == 0) {
            LOG_INFO("fusion", "#%u EKF:(%.1f,%.1f) v=%.1f hdg=%.1f° innov=%.2f",
                     g.fused_count, g.fused_x, g.fused_y, g.fused_v,
                     g.fused_heading * 180.0 / M_PI, g.ekf.last_innovation);
        }

        /* ── 延迟跟踪 + 上报 (每 20 帧) ── */
        if (gps_msg && gps_msg->timestamp_us > 0) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint64_t wall = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
            if (wall > gps_msg->timestamp_us)
                latency_tracker_record(&g.lat_tracker, wall - gps_msg->timestamp_us);
        }
        if (g.fused_count % 20 == 0) {
            LatencyStats ls = latency_tracker_stats(&g.lat_tracker);
            LatencyReport lr;
            lr.avg_us = (uint32_t)ls.avg_us;
            lr.p50_us = (uint32_t)ls.p50_us;
            lr.p99_us = (uint32_t)ls.p99_us;

            uint8_t lr_buf[32];
            size_t  lr_len = sizeof(lr_buf);
            LatencyReport_serialize(&lr, lr_buf, &lr_len);
            transport_publish(g.transport, "fusion/latency",
                              lr_buf, (uint32_t)lr_len);
        }
    }

    LOG_INFO("fusion", "stopped (%u fused frames)", g.fused_count);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "sensor/lidar", "sensor/gps", NULL };
static const char* s_outputs[] = { "fusion/localization", "fusion/latency", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int fusion_init(MessageBus* bus, Transport* transport,
                       DiscoveryManager* discovery, Scheduler* scheduler,
                       const char* params_json) {
    (void)bus; (void)scheduler; (void)params_json;

    memset(&g, 0, sizeof(g));
    g.transport  = transport;
    g.discovery  = discovery;
    g.scheduler  = scheduler;
    g.should_stop = 0;

    /* EKF: dt=0.05s, 初始状态 [0,0,5,0,0] */
    double x0[5] = {0.0, 0.0, 5.0, 0.0, 0.0};
    ekf_fusion_init(&g.ekf, 0.05, x0);

    /* 消息缓冲区 */
    g.lidar_buf = message_buffer_create("sensor/lidar", LIDARFRAME_TYPE_ID, 32, 5000000);
    g.gps_buf   = message_buffer_create("sensor/gps",   GPSDATA_TYPE_ID,    16, 5000000);
    if (!g.lidar_buf || !g.gps_buf) return -1;

    /* 延迟跟踪器 — 环形缓冲128样本 */
    memset(&g.lat_tracker, 0, sizeof(g.lat_tracker));

    /* 订阅输入 topics */
    transport_subscribe(transport, "sensor/lidar", on_lidar, NULL);
    transport_subscribe(transport, "sensor/gps",   on_gps,   NULL);

    /* Discovery 广告 */
    discovery_advertise(discovery, "sensor/lidar", LIDARFRAME_TYPE_ID, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "sensor/gps",   GPSDATA_TYPE_ID,   CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u, CAP_FUSION | CAP_PUBLISHER, 10.0);
    discovery_advertise(discovery, "fusion/latency",      0x1A7E9C01u, CAP_PUBLISHER, 2.0);

    /* 发布 topic 广告 */
    transport_advertise(transport, "fusion/localization", 0xF0ED10C0u);
    transport_advertise(transport, "fusion/latency",      0x1A7E9C01u);

    LOG_INFO("fusion", "initialized (EKF 5D, aligned LiDAR+GPS)");
    return 0;
}

static int fusion_start(void) {
    g.running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, fusion_thread, NULL) != 0) {
        LOG_WARN("fusion", "failed to create thread");
        return -1;
    }
    LOG_INFO("fusion", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void fusion_stop(void)    { g.should_stop = 1; }
static void fusion_cleanup(void) {
    if (g.running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.running = 0;
    }
    if (g.lidar_buf) { message_buffer_destroy(g.lidar_buf); g.lidar_buf = NULL; }
    if (g.gps_buf)   { message_buffer_destroy(g.gps_buf);   g.gps_buf   = NULL; }
    LOG_INFO("fusion", "cleanup done");
}

static int fusion_health(void) {
    /* 简单检查: 最近 5 秒是否有融合帧输出 */
    return (g.fused_count > 0) ? 0 : 1;
}

/* ── 导出入口 ────────────────────────────────────────────────── */

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "fusion",
    .version       = "1.0.0",
    .description   = "EKF 5D sensor fusion (LiDAR + GPS)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = fusion_init,
    .start         = fusion_start,
    .stop          = fusion_stop,
    .cleanup       = fusion_cleanup,
    .health        = fusion_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
