/**
 * ekf_fusion_plugin.c — EKF 传感器融合插件 (TaskInterface)
 *
 * 用真正的扩展卡尔曼滤波器替换旧的字符串拼接融合。
 * 输入: sensor/lidar (位置), sensor/gps (速度/航向)
 * 输出: fusion/localization (融合状态)
 *
 * 编译为 .so: 由 CMakeLists.txt 处理
 */

#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "fusion.h"
#include "logger.h"
#include "adas_msgs_gen.h"
#include "../algorithms/ekf_fusion.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

typedef struct {
    TaskBase         base;
    int              tid;
    MessageBuffer*   lidar_buf;
    MessageBuffer*   gps_buf;
    uint32_t         fused_count;

    /* EKF */
    EkfFusion        ekf;

    /* 外部依赖 */
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;
} EkfFusionTask;

/* ── 传感器回调 ────────────────────────────────────────────── */

static void on_lidar(const Message* msg, void* u) {
    message_buffer_push(((EkfFusionTask*)u)->lidar_buf, msg);
}
static void on_gps(const Message* msg, void* u) {
    message_buffer_push(((EkfFusionTask*)u)->gps_buf, msg);
}

/* ── 初始化 ────────────────────────────────────────────────── */

static int plugin_ekf_fusion_init(TaskBase* base) {
    EkfFusionTask* ft = (EkfFusionTask*)base;

    ft->lidar_buf = message_buffer_create("sensor/lidar", LIDARFRAME_TYPE_ID, 32, 5000000);
    ft->gps_buf   = message_buffer_create("sensor/gps",   GPSDATA_TYPE_ID,    16, 5000000);

    transport_subscribe(ft->transport, "sensor/lidar", on_lidar, ft);
    transport_subscribe(ft->transport, "sensor/gps",   on_gps,  ft);

    discovery_advertise(ft->discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_FUSION | CAP_PUBLISHER, 20.0);
    transport_advertise(ft->transport, "fusion/localization", 0xF0ED10C0u);

    /* 初始化 EKF: dt=0.05s (20Hz), 初始状态 [x=0, y=0, v=5, heading=0, yaw_rate=0] */
    ekf_fusion_init(&ft->ekf, 0.05, NULL);

    scheduler_choreo_trigger_on(ft->scheduler, ft->tid, "sensor/lidar");

    LOG_INFO("ekf_fusion", "plugin loaded (EKF 5D state, 20Hz prediction)");
    return 0;
}

/* ── 执行循环 ──────────────────────────────────────────────── */

static int plugin_ekf_fusion_execute(TaskBase* base) {
    EkfFusionTask* ft = (EkfFusionTask*)base;

    while (!base->should_stop) {
        int ret = scheduler_choreo_wait(ft->scheduler, ft->tid, 500000);
        if (ret == -2) break;
        if (ret == -1) continue;

        const Message* lidar_msg = message_buffer_latest(ft->lidar_buf);
        if (!lidar_msg) continue;

        const LidarFrame* lidar = (const LidarFrame*)
            _msg_cast_impl(lidar_msg, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
        if (!lidar) continue;

        /* ── EKF 预测步 ── */
        ekf_fusion_predict(&ft->ekf);

        /* ── LiDAR 位置更新 ── */
        ekf_fusion_update_lidar(&ft->ekf, (double)lidar->x, (double)lidar->y, NULL);

        /* ── GPS 更新 (如果有) ── */
        const Message* gps_msg = message_buffer_find_nearest(
            ft->gps_buf, lidar_msg->timestamp_us, 50000); /* 50ms window */
        if (gps_msg) {
            const GpsData* gps = (const GpsData*)
                _msg_cast_impl(gps_msg, GPSDATA_TYPE_ID, sizeof(GpsData), "GpsData");
            if (gps) {
                double heading_rad = (double)gps->heading_deg * M_PI / 180.0;
                ekf_fusion_update_gps(&ft->ekf, (double)gps->speed_mps, heading_rad, NULL);
            }
        }

        /* ── 读取融合状态 ── */
        double x, y, v, heading, yr;
        ekf_fusion_get_state(&ft->ekf, &x, &y, &v, &heading, &yr);
        double diag[5];
        ekf_fusion_get_covariance_diag(&ft->ekf, diag);

        /* ── 发布结构化融合结果 ── */
        char out[512];
        snprintf(out, sizeof(out),
            "{"
            "\"x\":%.2f,\"y\":%.2f,\"v\":%.2f,\"heading\":%.3f,\"yaw_rate\":%.3f,"
            "\"cov\":[%.2f,%.2f,%.2f,%.3f,%.4f],"
            "\"innovation\":%.3f,\"diverged\":%d"
            "}",
            x, y, v, heading, yr,
            diag[0], diag[1], diag[2], diag[3], diag[4],
            ft->ekf.last_innovation, ft->ekf.diverged);

        Message omsg;
        msg_init_typed(&omsg, "fusion/localization", "ekf_fusion",
                       0xF0ED10C0u, 2, out, (uint32_t)strlen(out) + 1);
        omsg.timestamp_us = lidar_msg->timestamp_us;
        transport_publish(ft->transport, "fusion/localization", omsg.data, omsg.data_size);

        ft->fused_count++;

        /* 发散检测与恢复 */
        if (ft->ekf.diverged) {
            LOG_WARN("ekf_fusion", "EKF diverged — resetting covariance");
            ekf_fusion_reset(&ft->ekf);
        }

        if (ft->fused_count % 50 == 0) {
            LOG_DEBUG("ekf_fusion", "#%u pos=(%.1f,%.1f) v=%.1f ψ=%.2f° innov=%.2f",
                      ft->fused_count, x, y, v, heading * 180.0 / M_PI,
                      ft->ekf.last_innovation);
        }
    }

    LOG_INFO("ekf_fusion", "stopped (%u fused frames)", ft->fused_count);
    return 0;
}

static void plugin_ekf_fusion_cleanup(TaskBase* base) {
    EkfFusionTask* ft = (EkfFusionTask*)base;
    message_buffer_destroy(ft->lidar_buf);
    message_buffer_destroy(ft->gps_buf);
}

static TaskInterface g_ekf_fusion_vtable = {
    .initialize = plugin_ekf_fusion_init,
    .execute    = plugin_ekf_fusion_execute,
    .cleanup    = plugin_ekf_fusion_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ── dlopen 导出 ───────────────────────────────────────────── */

TaskBase* create_task(const TaskConfig* config) {
    EkfFusionTask* ft = (EkfFusionTask*)calloc(1, sizeof(EkfFusionTask));
    if (!ft) return NULL;
    task_base_init(&ft->base, &g_ekf_fusion_vtable, config);
    if (config->custom_config) {
        struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; }* deps =
            (void*)config->custom_config;
        ft->discovery = deps->d;
        ft->transport  = deps->t;
        ft->scheduler  = deps->s;
    }
    return &ft->base;
}

void destroy_task(TaskBase* base) {
    if (!base) return;
    task_base_destroy(base);
    free(base);
}
