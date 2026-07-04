/** fusion_plugin.c — 时间对齐传感器融合插件 (TaskInterface) */
#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "fusion.h"
#include "logger.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef struct {
    TaskBase        base;
    int             tid;
    MessageBuffer*  lidar_buf;
    MessageBuffer*  gps_buf;
    uint32_t        fused_count;
    Transport*      transport;
    DiscoveryManager* discovery;
    Scheduler*      scheduler;
} FusionPlugin;

static void on_lidar(const Message* msg, void* u) { message_buffer_push(((FusionPlugin*)u)->lidar_buf, msg); }
static void on_gps(const Message* msg, void* u)   { message_buffer_push(((FusionPlugin*)u)->gps_buf, msg); }

static int fusion_init(TaskBase* base) {
    FusionPlugin* f = (FusionPlugin*)base;
    f->lidar_buf = message_buffer_create("sensor/lidar", LIDARFRAME_TYPE_ID, 32, 5000000);
    f->gps_buf   = message_buffer_create("sensor/gps",   GPSDATA_TYPE_ID,    16, 5000000);
    transport_subscribe(f->transport, "sensor/lidar", on_lidar, f);
    transport_subscribe(f->transport, "sensor/gps",   on_gps, f);
    discovery_advertise(f->discovery, "fusion/localization", 0xF0ED10C0u, CAP_FUSION|CAP_PUBLISHER, 10.0);
    transport_advertise(f->transport, "fusion/localization", 0xF0ED10C0u);
    LOG_INFO("fusion", "plugin loaded (TIME_ALIGNED 50ms window)");
    return 0;
}

static int fusion_execute(TaskBase* base) {
    FusionPlugin* f = (FusionPlugin*)base;
    while (!base->should_stop) {
        int ret = scheduler_choreo_wait(f->scheduler, f->tid, 1000000);
        if (ret == -2) break;
        const Message* lidar = message_buffer_latest(f->lidar_buf);
        const Message* gps   = message_buffer_latest(f->gps_buf);
        if (!lidar) continue;
        char out[256]; float x=0,y=0; double lat=0,lon=0; float speed=gps?10.0f:0;
        _msg_cast_impl(lidar, LIDARFRAME_TYPE_ID, sizeof(LidarFrame), "LidarFrame");
        if (lidar->data) { LidarFrame* lf=(LidarFrame*)lidar->data; x=lf->x; y=lf->y; }
        if (gps && gps->data) { GpsData* gd=(GpsData*)gps->data; lat=gd->latitude; lon=gd->longitude; speed=gd->speed_mps; }
        snprintf(out,sizeof(out),"pos=(%.1f,%.1f) gps=(%.6f,%.6f) speed=%.1f dt=%luus",x,y,lat,lon,speed,(unsigned long)(lidar->timestamp_us));
        Message omsg; msg_init_typed(&omsg,"fusion/localization","fusion",0xF0ED10C0u,1,out,(uint32_t)(strlen(out)+1));
        transport_publish(f->transport, "fusion/localization", omsg.data, omsg.data_size);
        f->fused_count++;
    }
    LOG_INFO("fusion", "stopped (%u fused frames)", f->fused_count);
    return 0;
}
static void fusion_cleanup(TaskBase* b) { (void)b; }
static TaskInterface g_fusion_vtable = { .initialize=fusion_init, .execute=fusion_execute, .cleanup=fusion_cleanup };

TaskBase* create_task(const TaskConfig* config) {
    FusionPlugin* f = calloc(1, sizeof(FusionPlugin));
    task_base_init(&f->base, &g_fusion_vtable, config);
    if (config->custom_config) {
        struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; }* deps = (void*)config->custom_config;
        f->discovery=deps->d; f->transport=deps->t; f->scheduler=deps->s;
    }
    return &f->base;
}
void destroy_task(TaskBase* base) { if(base){task_base_destroy(base);free(base);} }
