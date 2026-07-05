/**
 * dbscan_perception_plugin.c — DBSCAN 聚类感知插件
 *
 * 用 DBSCAN 聚类 + 包围盒提取替换旧的假感知数据。
 * 输入: sensor/lidar (点云)
 * 输出: perception/obstacles (ObstacleList)
 *
 * 编译为 .so: 由 CMakeLists.txt 处理
 */

#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "logger.h"
#include "adas_msgs_gen.h"
#include "../algorithms/dbscan_cluster.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Camera frame type ID */
#define CAMERAFRAME_TYPE_ID  0x4A1B0C2Du

/* ── 插件结构 ───────────────────────────────────────────────── */

typedef struct {
    TaskBase         base;
    int              tid;
    uint32_t         frame_id;

    /* DBSCAN 聚类器 */
    DbscanCluster    db;

    /* 外部依赖 */
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;
} DbscanPerceptionTask;

/* ── 从 LidarFrame + 额外生成的点云构造 Point3D 数组 ───────── */

static int generate_point_cloud(const LidarFrame* lidar, Point3D* points, int max_points) {
    /* 以 LidarFrame 位置为中心生成模拟点云:
     * - 地面环: 多圈同心圆
     * - 物体表面: lidar->x 附近的密集点 (模拟前方障碍物) */
    int n = 0;

    float cx = lidar->x, cy = lidar->y;

    /* 地面环 (4 圈, 每圈 16 点) */
    for (int ring = 0; ring < 4 && n < max_points; ring++) {
        float r = 5.0f + (float)ring * 3.0f;
        for (int i = 0; i < 16 && n < max_points; i++) {
            float angle = (float)i / 16.0f * 2.0f * (float)M_PI;
            points[n].x = cx + cosf(angle) * r;
            points[n].y = cy + sinf(angle) * r;
            points[n].z = 0.05f;  /* 地面 */
            points[n].intensity = 0.3f;
            n++;
        }
    }

    /* 前方障碍物团块 (模拟车辆/行人) */
    /* 障碍物1: 正前方 35m, 宽1.8m, 长4.5m (车辆) */
    float obs1_x = cx + 35.0f, obs1_y = cy;
    for (int i = 0; i < 20 && n < max_points; i++) {
        points[n].x = obs1_x + ((float)(i % 4) - 1.5f) * 0.6f;
        points[n].y = obs1_y + ((float)(i / 4) - 2.5f) * 0.4f;
        points[n].z = 0.8f + (float)(i % 4) * 0.4f;
        points[n].intensity = 0.7f;
        n++;
    }

    /* 障碍物2: 左前方 55m (行人) */
    float obs2_x = cx + 55.0f, obs2_y = cy + 8.0f;
    for (int i = 0; i < 8 && n < max_points; i++) {
        points[n].x = obs2_x + ((float)(i % 3) - 1.0f) * 0.3f;
        points[n].y = obs2_y + ((float)(i / 3) - 1.0f) * 0.2f;
        points[n].z = 1.2f + (float)(i % 2) * 0.3f;
        points[n].intensity = 0.6f;
        n++;
    }

    return n;
}

/* ── 初始化 ────────────────────────────────────────────────── */

static int dbscan_perception_init(TaskBase* base) {
    DbscanPerceptionTask* pt = (DbscanPerceptionTask*)base;

    /* DBSCAN: eps=2.0m, min_pts=4, RANSAC ground removal */
    dbscan_init(&pt->db, 2.0f, 4);
    dbscan_set_ransac(&pt->db, 100, 0.2f, 0.3f);

    /* 广告 LiDAR + Camera + GPS (数据生产者) */
    discovery_advertise(pt->discovery, "sensor/lidar",  LIDARFRAME_TYPE_ID,
                        CAP_PUBLISHER, 20.0);
    discovery_advertise(pt->discovery, "sensor/camera", CAMERAFRAME_TYPE_ID,
                        CAP_PUBLISHER, 20.0);
    discovery_advertise(pt->discovery, "sensor/gps",    GPSDATA_TYPE_ID,
                        CAP_PUBLISHER, 10.0);

    transport_advertise(pt->transport, "sensor/lidar",  LIDARFRAME_TYPE_ID);
    transport_advertise(pt->transport, "sensor/camera", CAMERAFRAME_TYPE_ID);
    transport_advertise(pt->transport, "sensor/gps",    GPSDATA_TYPE_ID);

    /* 广告感知输出 */
    discovery_advertise(pt->discovery, "perception/obstacles", 0x0B5A010Eu,
                        CAP_PUBLISHER, 20.0);
    transport_advertise(pt->transport, "perception/obstacles", 0x0B5A010Eu);

    LOG_INFO("dbscan_perception", "plugin loaded (DBSCAN eps=%.1fm min_pts=%d ground=%.1fm)",
             pt->db.eps, pt->db.min_pts, pt->db.ground_z_thresh);
    return 0;
}

/* ── 执行循环 ──────────────────────────────────────────────── */

static int dbscan_perception_execute(TaskBase* base) {
    DbscanPerceptionTask* pt = (DbscanPerceptionTask*)base;
    RateControl* rc = scheduler_get_rate_control(pt->scheduler, pt->tid);

    while (!base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(1000); continue; }

        /* ── 生成传感器数据 ── */
        LidarFrame lidar = {
            .x = 50.0f - (float)pt->frame_id * 0.05f,
            .y = 0.0f, .z = 0.0f, .intensity = 0.85f,
            .point_count = 64000 + pt->frame_id,
            .frame_id = pt->frame_id
        };

        /* ── 发布 LiDAR ── */
        Message lmsg;
        msg_init_typed(&lmsg, "sensor/lidar", "perception",
                       LIDARFRAME_TYPE_ID, LIDARFRAME_SCHEMA_VERSION,
                       &lidar, sizeof(lidar));
        transport_publish(pt->transport, "sensor/lidar", lmsg.data, lmsg.data_size);

        /* ── DBSCAN: 生成点云 → 聚类 → 包围盒 ── */
        Point3D points[DBSCAN_MAX_POINTS];
        int n_points = generate_point_cloud(&lidar, points, DBSCAN_MAX_POINTS);

        int n_clusters = dbscan_run(&pt->db, points, n_points);

        /* ── 发布 ObstacleList ── */
        ObstacleList obs_list;
        memset(&obs_list, 0, sizeof(obs_list));
        obs_list.frame_id = pt->frame_id;
        obs_list.timestamp_us = 0; /* filled by transport */
        obs_list.count = 0;

        for (int i = 0; i < n_clusters && obs_list.count < 8; i++) { /* obstacles[8] */
            const ClusterBounds* cb = dbscan_get_cluster(&pt->db, i);
            if (!cb) continue;

            Obstacle* obs = &obs_list.obstacles[obs_list.count];
            obs->id         = (uint32_t)(pt->frame_id * 100 + (uint32_t)i);
            obs->x          = cb->cx - lidar.x;  /* 自车系 */
            obs->y          = cb->cy - lidar.y;
            obs->vx         = 0.0f;  /* 从 tracker 补充 */
            obs->vy         = 0.0f;
            obs->width      = cb->width;
            obs->length     = cb->length;
            obs->confidence = cb->confidence;

            switch (cb->cls) {
                case CLS_VEHICLE:    obs->type = OBJ_TYPE_VEHICLE;    break;
                case CLS_PEDESTRIAN: obs->type = OBJ_TYPE_PEDESTRIAN; break;
                case CLS_CYCLIST:    obs->type = OBJ_TYPE_CYCLIST;    break;
                default:             obs->type = OBJ_TYPE_UNKNOWN;    break;
            }
            obs_list.count++;
        }

        Message omsg;
        msg_init_typed(&omsg, "perception/obstacles", "perception",
                       0x0B5A010Eu, 1, &obs_list, sizeof(obs_list));
        transport_publish(pt->transport, "perception/obstacles", omsg.data, omsg.data_size);

        /* ── GPS @10Hz ── */
        if (pt->frame_id % 2 == 0) {
            GpsData gps = {
                .latitude  = 39.904 + (double)pt->frame_id * 0.00001,
                .longitude = 116.407 + (double)pt->frame_id * 0.00001,
                .speed_mps = 10.0f, .heading_deg = 0.0f, .accuracy_m = 0.5f
            };
            Message gmsg;
            msg_init_typed(&gmsg, "sensor/gps", "perception",
                           GPSDATA_TYPE_ID, GPSDATA_SCHEMA_VERSION,
                           &gps, sizeof(gps));
            transport_publish(pt->transport, "sensor/gps", gmsg.data, gmsg.data_size);
        }

        pt->frame_id++;

        if (pt->frame_id % 100 == 0) {
            LOG_INFO("dbscan_perception", "#%u lidar=%.1fm %d pts → %d clusters",
                     pt->frame_id, lidar.x, n_points, n_clusters);
        }
    }

    LOG_INFO("dbscan_perception", "stopped (%u frames)", pt->frame_id);
    return 0;
}

static void dbscan_perception_cleanup(TaskBase* base) {
    (void)base;
}

static TaskInterface g_dbscan_vtable = {
    .initialize = dbscan_perception_init,
    .execute    = dbscan_perception_execute,
    .cleanup    = dbscan_perception_cleanup,
    .health_check = NULL,
    .on_message = NULL,
};

/* ── dlopen 导出 ───────────────────────────────────────────── */

TaskBase* create_task(const TaskConfig* config) {
    DbscanPerceptionTask* pt = (DbscanPerceptionTask*)calloc(1, sizeof(DbscanPerceptionTask));
    if (!pt) return NULL;
    task_base_init(&pt->base, &g_dbscan_vtable, config);
    if (config->custom_config) {
        struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; }* deps =
            (void*)config->custom_config;
        pt->discovery = deps->d;
        pt->transport  = deps->t;
        pt->scheduler  = deps->s;
    }
    return &pt->base;
}

void destroy_task(TaskBase* base) {
    if (!base) return;
    task_base_destroy(base);
    free(base);
}
