/**
 * control_node.c — PID 纵向/横向控制节点插件
 *
 * 订阅 fusion/localization, planning/trajectory
 * PID 计算油门/刹车/转向 → 发布 control/cmd
 *
 * 与 sim_world_node 配合: control 产生指令, sim_world 执行物理模拟。
 *
 * NodePlugin 接口，编译为 libcontrol_node.so。
 */

#include "node_plugin.h"
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* PID 状态 */
    double kp, ki, kd;
    double integral;
    double prev_error;

    /* 从 topic 解析的值 */
    double current_speed;
    double target_speed;
    double ego_x, ego_y;
    double lane_target;

    volatile int has_fusion;
    volatile int has_planning;

    /* 变道状态机 */
    int    lc_state;     /* 0=正常 1=左变道中 2=左车道巡航 3=右回正 */
    double lc_timer;
    double lc_wait;
    double lane_width;
    double blocked_timeout_s;

    uint32_t cycle;

    /* 配置参数 */
    double cfg_kp, cfg_ki, cfg_kd;
} g;

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    const char* p;

    if ((p = strstr(d, "\"v\":")))
        sscanf(p + 4, "%lf", &g.current_speed);
    else if ((p = strstr(d, "speed=")))
        sscanf(p + 6, "%lf", &g.current_speed);

    if ((p = strstr(d, "\"x\":")))  sscanf(p + 4, "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))  sscanf(p + 4, "%lf", &g.ego_y);

    g.has_fusion = 1;
}

static void on_trajectory(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;

    if (strstr(d, "speed="))
        sscanf(strstr(d, "speed=") + 6, "%lf", &g.target_speed);

    g.has_planning = 1;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* control_thread(void* arg) {
    (void)arg;

    while (!g.should_stop) {
        usleep(50000);  /* 20Hz */
        if (g.should_stop) break;

        g.cycle++;

        /* 等待初始数据 */
        if (!g.has_fusion) continue;
        if (!g.has_planning) continue;

        /* ── ACC: 目标速度来自规划 ── */
        double boost_target = g.target_speed;

        /* ── PID 计算 ── */
        double error = boost_target - g.current_speed;
        g.integral += error * 0.05;
        if (g.integral > 500)  g.integral = 500;
        if (g.integral < -200) g.integral = -200;

        double derivative = (error - g.prev_error) / 0.05;
        double output = g.kp * error + g.ki * g.integral + g.kd * derivative;

        double throttle = 0, brake = 0;
        const char* mode;
        if (output > 0) {
            throttle = output / 5000.0;
            if (throttle > 1.0) throttle = 1.0;
            brake = 0;
            mode = (error < 1.0) ? "HOLD" : "ACCEL";
        } else {
            throttle = 0;
            brake = (-output) / 8000.0;
            if (brake > 1.0) brake = 1.0;
            mode = "BRAKE";
        }

        /* ── 发布控制指令 ── */
        char cmd[256];
        snprintf(cmd, sizeof(cmd),
                 "throttle=%.2f brake=%.2f steer=%.2f "
                 "speed=%.1f target=%.1f error=%.1f mode=%s",
                 throttle, brake, 0.0,
                 g.current_speed, boost_target, error, mode);
        transport_publish(g.transport, "control/cmd",
                          (const uint8_t*)cmd, (uint32_t)strlen(cmd) + 1);

        g.prev_error = error;

        if (g.cycle % 20 == 1) {
            LOG_INFO("control", "#%d spd=%.1f→%.1f err=%.1f thr=%.2f brk=%.2f %s",
                     g.cycle, g.current_speed, g.target_speed,
                     error, throttle, brake, mode);
        }
    }

    LOG_INFO("control", "stopped (%u cycles, final speed=%.1f m/s)",
             g.cycle, g.current_speed);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", NULL };
static const char* s_outputs[] = { "control/cmd", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int control_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    (void)bus; (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.should_stop  = 0;

    /* 默认 PID 参数 */
    g.cfg_kp = 800.0; g.cfg_ki = 50.0; g.cfg_kd = 100.0;
    g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
    g.lane_width = 3.5;
    g.blocked_timeout_s = 2.0;

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"pid_kp\":")))
            sscanf(p + 9, "%lf", &g.cfg_kp);
        if ((p = strstr(params_json, "\"pid_ki\":")))
            sscanf(p + 9, "%lf", &g.cfg_ki);
        if ((p = strstr(params_json, "\"pid_kd\":")))
            sscanf(p + 9, "%lf", &g.cfg_kd);
        if ((p = strstr(params_json, "\"acc_time_headway\":")))
            sscanf(p + 19, "%lf", &g.lane_width);  /* 暂不独立使用 */
        if ((p = strstr(params_json, "\"lane_change_blocked_timeout_s\":")))
            sscanf(p + 32, "%lf", &g.blocked_timeout_s);
        g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_trajectory, NULL);
    transport_advertise(transport, "control/cmd", 0x2D95C6D2u);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/cmd", 0x2D95C6D2u,
                        CAP_PUBLISHER, 100.0);

    LOG_INFO("control", "initialized (PID: kp=%.0f ki=%.0f kd=%.0f)",
             g.kp, g.ki, g.kd);
    return 0;
}

static int control_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, control_thread, NULL) != 0) return -1;
    LOG_INFO("control", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void control_stop(void)        { g.should_stop = 1; }
static void control_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    LOG_INFO("control", "cleanup done");
}
static int  control_health(void)      { return 0; }

static NodePlugin s_plugin = {
    .name          = "control",
    .version       = "1.0.0",
    .description   = "PID longitudinal controller + ACC",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = control_init,
    .start         = control_start,
    .stop          = control_stop,
    .cleanup       = control_cleanup,
    .health        = control_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
