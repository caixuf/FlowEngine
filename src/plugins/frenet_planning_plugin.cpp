/**
 * frenet_planning_plugin.cpp — Frenet 最优轨迹规划插件 (C++ TaskInterface)
 *
 * 包装开源 Frenet 最优轨迹规划器:
 *   https://github.com/fangedward/frenet-optimal-trajectory-planner (Apache-2.0)
 *
 * 参考论文: Werling et al. "Optimal Trajectory Generation for Dynamic
 *   Street Scenarios in a Frenet Frame." (2010)
 *
 * 输入:  fusion/localization (自车位置), perception/obstacles (障碍物)
 * 输出:  planning/trajectory (最优轨迹 waypoints)
 *
 * 编译为 .so: 由 CMakeLists.txt 处理
 */

#include "task_interface.h"
#include "message_bus.h"
#include "serializer.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <vector>

/* 开源 Frenet 规划器 */
#include "FrenetOptimalTrajectory.h"
#include "FrenetPath.h"
#include "py_cpp_struct.h"
#include "CubicSpline2D.h"

/* C wrapper from fot_wrapper.cpp */
extern "C" {
void run_fot(FrenetInitialConditions*, FrenetHyperparameters*, FrenetReturnValues*);
}

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── 默认参考路径: 直线 ────────────────────────────────────── */

static void make_default_reference_path(vector<double>& wx, vector<double>& wy) {
    for (int i = 0; i <= 100; i++) {
        wx.push_back(i * 2.0);
        wy.push_back(0.0);
    }
}

/* ── 插件结构 ───────────────────────────────────────────────── */

typedef struct {
    TaskBase         base;
    int              tid;
    int              plan_count;
    vector<double>   ref_x, ref_y;
    double           ego_x, ego_y, ego_speed, ego_heading;

    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;
} FrenetPlanningTask;

/* ── 回调 ───────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    FrenetPlanningTask* pt = (FrenetPlanningTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;
    double x = 0, y = 0, v = 10.0, heading = 0;
    if (strstr(data, "\"x\":"))
        sscanf(data, "{\"x\":%lf,\"y\":%lf,\"v\":%lf,\"heading\":%lf", &x, &y, &v, &heading);
    else if (strstr(data, "pos=(")) {
        sscanf(data, "pos=(%lf,%lf", &x, &y);
        if (strstr(data, "speed=")) sscanf(data, "speed=%lf", &v);
    }
    pt->ego_x = x; pt->ego_y = y;
    pt->ego_speed = v; pt->ego_heading = heading;
}

/* ── 生命周期 ──────────────────────────────────────────────── */

static int frenet_planning_init(TaskBase* base) {
    FrenetPlanningTask* pt = (FrenetPlanningTask*)base;
    make_default_reference_path(pt->ref_x, pt->ref_y);
    pt->ego_x = 0; pt->ego_y = 0; pt->ego_speed = 5.0; pt->ego_heading = 0;
    transport_subscribe(pt->transport, "fusion/localization", on_fusion, pt);
    transport_subscribe(pt->transport, "perception/obstacles", NULL, pt);
    transport_advertise(pt->transport, "planning/trajectory", 0x3A7B1C2Du);
    discovery_advertise(pt->discovery, "planning/trajectory", 0x3A7B1C2Du, CAP_PUBLISHER, 10.0);
    discovery_advertise(pt->discovery, "fusion/localization", 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    scheduler_choreo_trigger_on(pt->scheduler, pt->tid, "fusion/localization");
    LOG_INFO("frenet_planning", "plugin loaded (Werling 2010 Frenet Optimal Trajectory)");
    return 0;
}

static int frenet_planning_execute(TaskBase* base) {
    FrenetPlanningTask* pt = (FrenetPlanningTask*)base;
    RateControl* rc = scheduler_get_rate_control(pt->scheduler, pt->tid);

    while (!base->should_stop) {
        if (!rate_control_acquire(rc)) { usleep(5000); continue; }
        pt->plan_count++;

        /* 构建初始条件 */
        FrenetInitialConditions fot_ic;
        memset(&fot_ic, 0, sizeof(fot_ic));
        fot_ic.s0 = pt->ego_x;
        fot_ic.c_speed = pt->ego_speed;
        fot_ic.c_d = pt->ego_y;
        fot_ic.c_d_d = 0;
        fot_ic.c_d_dd = 0;
        fot_ic.target_speed = 15.0;
        fot_ic.wx = pt->ref_x.data();
        fot_ic.wy = pt->ref_y.data();
        fot_ic.nw = (int)pt->ref_x.size();
        fot_ic.no = 0;

        /* 超参数 */
        FrenetHyperparameters fot_hp;
        memset(&fot_hp, 0, sizeof(fot_hp));
        fot_hp.max_speed = 20.0;
        fot_hp.max_accel = 4.0;
        fot_hp.max_curvature = 0.3;
        fot_hp.max_road_width_l = 5.0;
        fot_hp.max_road_width_r = 5.0;
        fot_hp.d_road_w = 1.0;
        fot_hp.dt = 0.2;
        fot_hp.maxt = 8.0;
        fot_hp.mint = 2.0;
        fot_hp.d_t_s = 1.0;
        fot_hp.n_s_sample = 2.0;
        fot_hp.obstacle_clearance = 0.5;
        fot_hp.kd = 1.0;
        fot_hp.kv = 0.5;
        fot_hp.ka = 0.5;
        fot_hp.kj = 0.1;
        fot_hp.kt = 0.5;
        fot_hp.ko = 1.0;
        fot_hp.klat = 1.0;
        fot_hp.klon = 1.0;

        /* 运行规划器 */
        FrenetReturnValues fot_rv;
        memset(&fot_rv, 0, sizeof(fot_rv));
        run_fot(&fot_ic, &fot_hp, &fot_rv);

        /* 发布轨迹 */
        char traj[4096];
        if (fot_rv.success) {
            int off = snprintf(traj, sizeof(traj),
                "{\"type\":\"frenet\",\"plan\":%d,\"cost\":%.2f,\"wp\":[",
                pt->plan_count, fot_rv.costs[11]);
            int wp_count = 0;
            for (int i = 0; i < MAX_PATH_LENGTH && wp_count < 20; i++) {
                if (isnan(fot_rv.x_path[i])) break;
                if (i % 5 != 0 && i > 0) continue;
                off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                    "%s[%.1f,%.1f,%.1f]", wp_count > 0 ? "," : "",
                    fot_rv.x_path[i], fot_rv.y_path[i], fot_rv.speeds[i]);
                wp_count++;
            }
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "]}");
        } else {
            snprintf(traj, sizeof(traj), "{\"type\":\"failsafe\",\"v\":%.1f}", pt->ego_speed);
        }

        Message pmsg;
        msg_init_typed(&pmsg, "planning/trajectory", "frenet_planning",
                       0x3A7B1C2Du, 2, traj, (uint32_t)strlen(traj) + 1);
        transport_publish(pt->transport, "planning/trajectory", pmsg.data, pmsg.data_size);

        if (pt->plan_count % 50 == 0) {
            LOG_INFO("frenet_planning", "#%d cost=%.1f ego@(%.0f,%.1f) v=%.1f %s",
                     pt->plan_count, fot_rv.costs[11], pt->ego_x, pt->ego_y,
                     pt->ego_speed, fot_rv.success ? "ok" : "FAIL");
        }
        task_update_heartbeat(base);
    }
    LOG_INFO("frenet_planning", "stopped (%d plans)", pt->plan_count);
    return 0;
}

static void frenet_planning_cleanup(TaskBase* base) { (void)base; }

static TaskInterface g_frenet_vtable = {
    frenet_planning_init,       /* initialize */
    frenet_planning_execute,    /* execute */
    frenet_planning_cleanup,    /* cleanup */
    NULL,                       /* pause */
    NULL,                       /* resume */
    NULL,                       /* handle_signal */
    NULL,                       /* health_check */
    NULL,                       /* on_message */
    NULL,                       /* get_status */
};

/* ── dlopen 导出 ───────────────────────────────────────────── */

extern "C" {

TaskBase* create_task(const TaskConfig* config) {
    FrenetPlanningTask* pt = (FrenetPlanningTask*)calloc(1, sizeof(FrenetPlanningTask));
    if (!pt) return NULL;
    new (&pt->ref_x) vector<double>();
    new (&pt->ref_y) vector<double>();
    task_base_init(&pt->base, &g_frenet_vtable, config);
    if (config->custom_config) {
        /* Plugin dependency injection: layout {b, d, t, s} */
        typedef struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; } Deps;
        Deps* deps = (Deps*)config->custom_config;
        pt->discovery = deps->d;
        pt->transport = deps->t;
        pt->scheduler = deps->s;
    }
    return &pt->base;
}

void destroy_task(TaskBase* base) {
    if (!base) return;
    FrenetPlanningTask* pt = (FrenetPlanningTask*)base;
    pt->ref_x.~vector();
    pt->ref_y.~vector();
    task_base_destroy(base);
    free(base);
}

}  /* extern "C" */
