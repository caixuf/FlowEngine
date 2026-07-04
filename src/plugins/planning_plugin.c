/** planning_plugin.c — 轨迹规划插件 (TaskInterface)
 *  订阅 fusion/localization → 发布 planning/trajectory */
#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef struct {
    TaskBase        base; int tid; int plan_count;
    Transport*      transport; DiscoveryManager* discovery; Scheduler* scheduler;
} PlanningPlugin;

static void on_fusion(const Message* msg, void* u) {
    PlanningPlugin* p = (PlanningPlugin*)u;
    const char* data = (const char*)msg->data; if(!data)return;
    float x=0,speed=10.0f;
    if(strstr(data,"pos=(")) sscanf(data,"pos=(%f",&x);
    if(strstr(data,"speed=")) sscanf(data,"speed=%f",&speed);
    float ts = speed>15?15:speed;
    char traj[256];
    snprintf(traj,sizeof(traj),"traj=(%.1f,0.0) speed=%.1f lane=center",x+2.0f,ts);
    Message omsg; msg_init_typed(&omsg,"planning/trajectory","planning",0x3A7B1C2Du,1,traj,(uint32_t)(strlen(traj)+1));
    transport_publish(p->transport, "planning/trajectory", omsg.data, omsg.data_size);
    p->plan_count++;
}

static int planning_init(TaskBase* base) {
    PlanningPlugin* p = (PlanningPlugin*)base;
    transport_subscribe(p->transport, "fusion/localization", on_fusion, p);
    transport_advertise(p->transport, "planning/trajectory", 0x3A7B1C2Du);
    discovery_advertise(p->discovery, "planning/trajectory", 0x3A7B1C2Du, CAP_PUBLISHER, 10.0);
    discovery_advertise(p->discovery, "fusion/localization", 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    scheduler_choreo_trigger_on(p->scheduler, p->tid, "fusion/localization");
    LOG_INFO("planning", "plugin loaded (choreo, 10Hz trajectory)");
    return 0;
}
static int planning_execute(TaskBase* base) {
    PlanningPlugin* p = (PlanningPlugin*)base;
    RateControl* rc = scheduler_get_rate_control(p->scheduler, p->tid);
    while (!base->should_stop) { if(!rate_control_acquire(rc)){usleep(5000);continue;} task_update_heartbeat(base); }
    LOG_INFO("planning","stopped (%d trajectories)",p->plan_count); return 0;
}
static void planning_cleanup(TaskBase* b) { (void)b; }
static TaskInterface g_planning_vtable = { .initialize=planning_init, .execute=planning_execute, .cleanup=planning_cleanup };

TaskBase* create_task(const TaskConfig* config) {
    PlanningPlugin* p = calloc(1, sizeof(PlanningPlugin));
    task_base_init(&p->base, &g_planning_vtable, config);
    if (config->custom_config) {
        struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; }* deps = (void*)config->custom_config;
        p->transport=deps->t; p->discovery=deps->d; p->scheduler=deps->s;
    }
    return &p->base;
}
void destroy_task(TaskBase* base) { if(base){task_base_destroy(base);free(base);} }
