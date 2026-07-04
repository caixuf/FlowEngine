/**
 * pid_controller.c — PID 纵向控制器插件 (TaskInterface)
 *
 * 独立的 .so 插件，可以被 launcher dlopen 加载或被 e2e 直接链接。
 * 实现：PID 速度控制 + 车辆质点动力学 → 闭环仿真。
 *
 * 输入:  fusion/localization (当前速度)
 *        planning/trajectory  (目标速度)
 * 输出:  control/cmd (油门/刹车), vehicle/state (遥测)
 *
 * 编译为 .so:
 *   gcc -shared -fPIC -I include src/plugins/pid_controller.c
 *       -o build/lib/libpid_controller.so
 *
 * launcher 加载:
 *   { "name": "pid_control", "plugin": "lib/libpid_controller.so", ... }
 */

#include "task_interface.h"
#include "message_bus.h"
#include "discovery.h"
#include "transport.h"
#include "scheduler.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

/* ══════════════════════════════════════════════════════════ */
/* 车辆动力学模型 (质点)                                         */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    double x, y;           /* 位置 (m) */
    double speed;          /* 当前速度 (m/s) */
    double target_speed;   /* 目标速度 (m/s) */
    double throttle;       /* 油门 0..1 */
    double brake;          /* 刹车 0..1 */
    double heading;        /* 航向角 */
    double mass;           /* 质量 (kg) */
    double drag_coeff;     /* 空气阻力系数 */
} VehicleModel;

static void vehicle_tick(VehicleModel* v, double dt) {
    double drive_force  = v->throttle * 5000.0;
    double brake_force  = v->brake    * 8000.0;
    double drag_force   = v->drag_coeff * v->speed * v->speed;
    double net_force    = drive_force - brake_force - drag_force;
    double accel        = net_force / v->mass;

    v->speed += accel * dt;
    if (v->speed < 0) v->speed = 0;
    v->x += v->speed * dt * cos(v->heading);
    v->y += v->speed * dt * sin(v->heading);
}

/* ══════════════════════════════════════════════════════════ */
/* PID 控制器状态                                               */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    TaskBase    base;
    int         tid;

    /* PID 参数 */
    double      kp, ki, kd;
    double      integral;
    double      prev_error;

    /* 当前状态 */
    double      current_speed;
    double      target_speed;
    VehicleModel vehicle;

    /* 消息统计 */
    int         cycle_count;

    /* 外部依赖 (通过 init 注入) */
    MessageBus*       bus;
    DiscoveryManager* discovery;
    Transport*        transport;
    Scheduler*        scheduler;
} PidControlTask;

/* ── 融合数据回调 ────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    PidControlTask* pt = (PidControlTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;
    if (strstr(data, "speed="))
        sscanf(data, "speed=%lf", &pt->current_speed);
}

static void on_trajectory(const Message* msg, void* user_data) {
    PidControlTask* pt = (PidControlTask*)user_data;
    const char* data = (const char*)msg->data;
    if (!data) return;
    if (strstr(data, "speed="))
        sscanf(data, "speed=%lf", &pt->target_speed);
}

/* ── 初始化 ──────────────────────────────────────────────── */

static int pid_control_init(TaskBase* base) {
    PidControlTask* pt = (PidControlTask*)base;

    /* PID 参数: Kp=800, Ki=50, Kd=100 */
    pt->kp = 800.0; pt->ki = 50.0; pt->kd = 100.0;
    pt->integral = 0; pt->prev_error = 0;

    /* 车辆初始状态 */
    pt->vehicle = (VehicleModel){
        .x = 0, .y = 0, .speed = 5.0, .target_speed = 10.0,
        .throttle = 0, .brake = 0, .heading = 0,
        .mass = 1500.0, .drag_coeff = 0.3
    };
    pt->current_speed = pt->vehicle.speed;
    pt->target_speed  = pt->vehicle.target_speed;

    /* 订阅 */
    transport_subscribe(pt->transport, "fusion/localization", on_fusion, pt);
    transport_subscribe(pt->transport, "planning/trajectory", on_trajectory, pt);

    /* 广告 */
    transport_advertise(pt->transport, "control/cmd", 0x2D95C6D2u);
    discovery_advertise(pt->discovery, "control/cmd", 0x2D95C6D2u,
                        CAP_PUBLISHER, 100.0);
    discovery_advertise(pt->discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(pt->discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);

    /* Choreo: 被融合输出触发 */
    scheduler_choreo_trigger_on(pt->scheduler, pt->tid, "fusion/localization");

    LOG_INFO("pid_control", "initialized (PID kp=%.0f ki=%.0f kd=%.0f, target=%.1f m/s)",
             pt->kp, pt->ki, pt->kd, pt->target_speed);
    return 0;
}

/* ── 执行循环 (Choreo 触发, ~50ms/次) ───────────────────── */

static int pid_control_execute(TaskBase* base) {
    PidControlTask* pt = (PidControlTask*)base;

    while (!base->should_stop) {
        int ret = scheduler_choreo_wait(pt->scheduler, pt->tid, 500000);
        if (ret == -2) break;
        pt->cycle_count++;

        /* ── PID 计算 ── */
        double error  = pt->target_speed - pt->current_speed;
        pt->integral += error * 0.05;  /* dt ≈ 50ms */
        if (pt->integral > 500)  pt->integral = 500;
        if (pt->integral < -200) pt->integral = -200;

        double derivative = (error - pt->prev_error) / 0.05;
        double output = pt->kp * error + pt->ki * pt->integral + pt->kd * derivative;

        /* 油门/刹车拆分 */
        double throttle = 0, brake = 0;
        const char* mode;
        if (output > 0) {
            throttle = output / 5000.0;
            if (throttle > 1.0) throttle = 1.0;
            brake = 0;
            mode = (error < 0.5) ? "⏺ HOLD" : "🟢 ACCEL";
        } else {
            throttle = 0;
            brake = (-output) / 8000.0;
            if (brake > 1.0) brake = 1.0;
            mode = "🔴 BRAKE";
        }

        /* ── 更新车辆动力学 ── */
        pt->vehicle.throttle = throttle;
        pt->vehicle.brake    = brake;
        vehicle_tick(&pt->vehicle, 0.05);

        /* ── 发布控制指令 ── */
        char cmd[256];
        snprintf(cmd, sizeof(cmd),
                 "throttle=%.2f brake=%.2f steer=0.00 "
                 "speed=%.1f target=%.1f error=%.1f mode=%s",
                 throttle, brake, pt->vehicle.speed,
                 pt->target_speed, error, mode);
        Message cmsg;
        msg_init_typed(&cmsg, "control/cmd", "pid_control",
                       0x2D95C6D2u, 1, cmd, (uint32_t)(strlen(cmd) + 1));
        transport_publish(pt->transport, "control/cmd", cmsg.data, cmsg.data_size);

        /* 闭环: 车辆速度 → 感知 → 融合 → 回到这里 */
        pt->current_speed = pt->vehicle.speed;
        pt->prev_error    = error;

        LOG_DEBUG("pid_control", "#%d spd=%.1f→%.1f err=%.1f thr=%.2f %s",
                  pt->cycle_count, pt->vehicle.speed, pt->target_speed,
                  error, throttle, mode);
    }

    LOG_INFO("pid_control", "stopped (%d cycles, final speed=%.1f m/s)",
             pt->cycle_count, pt->vehicle.speed);
    return 0;
}

static void pid_control_cleanup(TaskBase* base) {
    (void)base;
}

/* ── 虚函数表 ────────────────────────────────────────────── */

static TaskInterface g_pid_vtable = {
    .initialize   = pid_control_init,
    .execute      = pid_control_execute,
    .cleanup      = pid_control_cleanup,
    .health_check = NULL,
    .on_message   = NULL,
};

/* ── 获取车辆状态 (供外部读取遥测) ──────────────────────── */

int pid_control_get_vehicle_state(PidControlTask* task,
                                  double* speed, double* target,
                                  double* throttle, double* brake,
                                  double* x, double* y) {
    if (!task) return -1;
    if (speed)    *speed    = task->vehicle.speed;
    if (target)   *target   = task->target_speed;
    if (throttle) *throttle = task->vehicle.throttle;
    if (brake)    *brake    = task->vehicle.brake;
    if (x)        *x        = task->vehicle.x;
    if (y)        *y        = task->vehicle.y;
    return 0;
}

/* ── dlopen 导出 ─────────────────────────────────────────── */

TaskBase* create_task(const TaskConfig* config) {
    PidControlTask* pt = (PidControlTask*)calloc(1, sizeof(PidControlTask));
    if (!pt) return NULL;

    task_base_init(&pt->base, &g_pid_vtable, config);

    /* 从 config.custom_config 读取外部依赖（void* 转结构体） */
    if (config->custom_config) {
        struct { MessageBus* b; DiscoveryManager* d; Transport* t; Scheduler* s; }* deps =
            (void*)config->custom_config;
        pt->bus       = deps->b;
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
