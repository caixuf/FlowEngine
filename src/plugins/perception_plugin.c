/**
 * perception_plugin.c — 感知传感器模拟器插件
 *
 * 模拟 LiDAR(20Hz) + Camera(20Hz) + GPS(10Hz) 数据发布。
 * 可被 launcher dlopen 或直接链接到 e2e。
 */

#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "logger.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CAMERAFRAME_TYPE_ID  0x4A1B0C2Du

typedef struct {
    TaskBase        base;
    int             tid;
    uint32_t        frame_id;
    MessageBus*     bus;
    Transport*      transport;
    DiscoveryManager* discovery;
    Scheduler*      scheduler;
} PerceptionPlugin;

static int perception_init(TaskBase* base) {
    PerceptionPlugin* p = (PerceptionPlugin*)base;
    discovery_advertise(p->discovery, "sensor/lidar",  LIDARFRAME_TYPE_ID, CAP_PUBLISHER, 20.0);
    discovery_advertise(p->discovery, "sensor/camera", CAMERAFRAME_TYPE_ID, CAP_PUBLISHER, 20.0);
    discovery_advertise(p->discovery, "sensor/gps",    GPSDATA_TYPE_ID,     CAP_PUBLISHER, 10.0);
    transport_advertise(p->transport, "sensor/lidar",  LIDARFRAME_TYPE_ID);
    transport_advertise(p->transport, "sensor/camera", CAMERAFRAME_TYPE_ID);
    transport_advertise(p->transport, "sensor/gps",    GPSDATA_TYPE_ID);
    LOG_INFO("perception", "plugin loaded (20Hz LiDAR+Camera, 10Hz GPS)");
    return 0;
}

static int perception_execute(TaskBase* base) {
    PerceptionPlugin* p = (PerceptionPlugin*)base;
    RateControl* rc = scheduler_get_rate_control(p->scheduler, p->tid);

    while (!base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(1000); continue; }

        /* LiDAR @20Hz */
        LidarFrame lidar = { .x=50.0f-p->frame_id*0.05f, .y=0,.z=0,
                             .intensity=0.85f, .point_count=64000+p->frame_id, .frame_id=p->frame_id };
        Message lmsg; msg_init_typed(&lmsg, "sensor/lidar", "perception", LIDARFRAME_TYPE_ID, 1, &lidar, sizeof(lidar));
        transport_publish(p->transport, "sensor/lidar", lmsg.data, lmsg.data_size);

        /* Camera @20Hz */
        LidarFrame cam = { .x=48.0f-p->frame_id*0.05f, .y=12,.z=5,
                           .intensity=0.72f, .point_count=1280+p->frame_id%20, .frame_id=p->frame_id };
        Message cmsg; msg_init_typed(&cmsg, "sensor/camera", "perception", CAMERAFRAME_TYPE_ID, 1, &cam, sizeof(cam));
        transport_publish(p->transport, "sensor/camera", cmsg.data, cmsg.data_size);

        /* GPS @10Hz */
        if (p->frame_id % 2 == 0) {
            GpsData gps = { .latitude=39.904+p->frame_id*0.00001, .longitude=116.407+p->frame_id*0.00001,
                            .speed_mps=10.0f, .heading_deg=0, .accuracy_m=0.5f };
            Message gmsg; msg_init_typed(&gmsg, "sensor/gps", "perception", GPSDATA_TYPE_ID, 1, &gps, sizeof(gps));
            transport_publish(p->transport, "sensor/gps", gmsg.data, gmsg.data_size);
        }
        p->frame_id++;
    }
    LOG_INFO("perception", "stopped (%u frames)", p->frame_id);
    return 0;
}

static void perception_cleanup(TaskBase* base) { (void)base; }
static TaskInterface g_perception_vtable = { .initialize=perception_init, .execute=perception_execute, .cleanup=perception_cleanup };

TaskBase* create_task(const TaskConfig* config) {
    PerceptionPlugin* p = calloc(1, sizeof(PerceptionPlugin));
    task_base_init(&p->base, &g_perception_vtable, config);
    if (config->custom_config) {
        struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; }* deps = (void*)config->custom_config;
        p->discovery=deps->d; p->transport=deps->t; p->scheduler=deps->s;
    }
    return &p->base;
}
void destroy_task(TaskBase* base) { if(base){task_base_destroy(base);free(base);} }
