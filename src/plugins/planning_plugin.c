/** planning_plugin.c — 轨迹规划插件 (TaskInterface)
 *  订阅 fusion/localization → 发布 planning/trajectory */
#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "logger.h"
#include "json_schema.h"   /* Phase 4.4: dsl_get_double_strict 替换 strstr+sscanf */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>      /* isdigit — 解析 pos=(x,y) 中 x */
#include <unistd.h>

typedef struct {
    TaskBase        base; int tid; int plan_count;
    Transport*      transport; DiscoveryManager* discovery; Scheduler* scheduler;
} PlanningPlugin;

static void on_fusion(const Message* msg, void* u) {
    PlanningPlugin* p = (PlanningPlugin*)u;
    const char* data = (const char*)msg->data; if(!data)return;
    /* Phase 4.4: 用 dsl_get_double_strict + dsl_find_value 替换 strstr+sscanf。
     * - pos=(x,y) 是元组格式：用 dsl_find_value 定位，然后解析括号内数字。
     * - speed=5.0 走严格 DSL 提取，字段缺失时 speed 保持默认 10.0。 */
    float x = 0;
    const char* pos_val = dsl_find_value(data, "pos");
    if (pos_val && *pos_val == '(') {
        const char* num = pos_val + 1;
        if (*num == '-' || *num == '+') num++;
        if (isdigit((unsigned char)*num)) {
            x = strtof(num, NULL);
        }
    }
    float speed = 10.0f;
    double speed_d = 0;
    if (dsl_get_double_strict(data, "speed", &speed_d)) {
        speed = (float)speed_d;
    }
    float ts = 15.0f;  /* 固定巡航目标 15 m/s，不随当前速度变化 */
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
