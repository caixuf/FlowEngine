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

#define MAX_OBS 3

/* 横向级联 PD 常量 */
#define MAX_PSI_DES_RAD    0.349   /* 最大期望航向角 ≈ ±20° */
#define STEER_FILTER_NEW   0.7     /* 低通滤波新值权重 */
#define STEER_FILTER_PREV  0.3     /* 低通滤波旧值权重 */

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
    /* 横向级联 PD 状态 */
    double lat_kp;          /* lateral error → desired heading (rad/m) */
    double lat_kd_heading;  /* heading error → steer (阻尼) */
    double ego_heading;     /* 从 fusion 获取的航向角 (rad) */
    double prev_steer;

    /* 从 topic 解析的值 */
    double current_speed;
    double target_speed;
    double ego_x, ego_y;
    double lane_d;          /* 从 trajectory 解析的横向偏移（Frenet d） */

    /* 障碍物数据 (从 vehicle/state 解析) */
    double obs_x[MAX_OBS], obs_y[MAX_OBS], obs_vx[MAX_OBS];
    int    obs_valid[MAX_OBS];

    volatile int has_fusion;
    volatile int has_planning;

    /* 变道状态机 */
    int    lc_state;     /* 0=正常 1=左变道中 2=左车道巡航 3=右回正 */
    double lc_timer;
    double lc_wait;
    double lc_origin_y;
    double lc_target_y;
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

    if ((p = strstr(d, "\"x\":")))       sscanf(p + 4,  "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))       sscanf(p + 4,  "%lf", &g.ego_y);
    if ((p = strstr(d, "\"heading\":"))) sscanf(p + 10, "%lf", &g.ego_heading);

    g.has_fusion = 1;
}

static void on_trajectory(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;

    if (strstr(d, "speed="))
        sscanf(strstr(d, "speed=") + 6, "%lf", &g.target_speed);

    /* 解析第一个路径点的 d 值（横向偏移），格式: [s,d,spd] */
    const char* bracket = strchr(d, '[');
    if (bracket) {
        double s, d_lat, spd;
        if (sscanf(bracket, "[%lf,%lf,%lf]", &s, &d_lat, &spd) >= 2)
            g.lane_d = d_lat;
    }
    /* 无路径点时，读取 failsafe 的 lane_keep_d 作为降级 */
    if (!bracket) {
        const char* p = strstr(d, "\"lane_keep_d\":");
        if (p) {
            double keep_d = 0.0;
            if (sscanf(p + 14, "%lf", &keep_d) >= 1)
                g.lane_d = keep_d;
        }
    }

    g.has_planning = 1;
}

/* ── vehicle/state 订阅 — 解析障碍物位置 ─────────────────────── */

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    for (int i = 0; i < MAX_OBS; i++) {
        char key[16];
        int klen = snprintf(key, sizeof(key), "\"ox%d\":", i);
        const char* p = strstr(d, key);
        if (p) {
            g.obs_valid[i] = 1;
            sscanf(p + klen, "%lf", &g.obs_x[i]);
        } else { g.obs_valid[i] = 0; continue; }
        klen = snprintf(key, sizeof(key), "\"oy%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_y[i]);
        klen = snprintf(key, sizeof(key), "\"ov%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_vx[i]);
    }
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* control_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "control");

    const double same_lane_tol = 2.0;
    const double time_headway  = 2.0;
    const double min_gap       = 6.0;
    const double ref_y         = -1.75;  /* Frenet 参考路径在左车道中心 */

    while (!g.should_stop) {
        usleep(50000);  /* 20Hz */
        if (g.should_stop) break;

        g.cycle++;

        /* 等待初始数据 */
        if (!g.has_fusion) continue;
        if (!g.has_planning) continue;

        /* ── ACC & 变道: 计算本车道前车间距 ── */
        double best_gap = 1e9;
        for (int i = 0; i < MAX_OBS; i++) {
            if (!g.obs_valid[i] || g.obs_vx[i] < 0) continue;
            if (fabs(g.obs_y[i] - g.ego_y) > same_lane_tol) continue;
            double dx = g.obs_x[i] - g.ego_x;
            double gap = dx - 4.6;  /* 粗略车头到后保险杠 */
            if (dx > 0 && gap < best_gap) best_gap = gap;
        }
        double safe_gap = min_gap + g.current_speed * time_headway;
        double boost_target = g.target_speed;
        double acc_target = boost_target;
        int blocked = 0;
        if (best_gap < safe_gap && best_gap < 80.0) {
            double ratio = best_gap / safe_gap;
            if (ratio < 0.2) ratio = 0.2;
            acc_target = boost_target * ratio;
            if (acc_target < boost_target * 0.7) blocked = 1;
        }

        /* ── 变道中: 临时提高目标速度，加速完成变道 ── */
        if (g.lc_state == 1) {
            boost_target = g.target_speed * 1.15;
            acc_target = boost_target;
        }

        /* ── 自适应变道状态机 ── */
        double cruise_lane_y = ref_y + g.lane_d;  /* Frenet d → 世界坐标 */
        double effective_lane_d = (g.lc_state != 0) ? g.lc_target_y : cruise_lane_y;

        if (blocked && g.lc_state == 0) {
            g.lc_timer += 0.05;
            if (g.lc_timer > g.blocked_timeout_s) {
                /* 检查左车道是否通畅 */
                int left_clear = 1;
                for (int i = 0; i < MAX_OBS; i++) {
                    if (!g.obs_valid[i]) continue;
                    double dy = fabs(g.obs_y[i] - (g.ego_y + g.lane_width));
                    if (dy < same_lane_tol) {
                        double dx = g.obs_x[i] - g.ego_x;
                        double rel_spd = g.current_speed - g.obs_vx[i];
                        if ((dx > -20.0 && dx < 80.0 && rel_spd > -3.0) ||
                            (dx < 0.0 && dx > -20.0 && rel_spd > 2.0)) {
                            left_clear = 0; break;
                        }
                    }
                }
                if (left_clear) {
                    g.lc_origin_y = cruise_lane_y;
                    g.lc_target_y = g.lc_origin_y + g.lane_width;
                    effective_lane_d = g.lc_target_y;
                    g.lc_state = 1; g.lc_timer = 0;
                    LOG_INFO("control", ">>> LANE CHANGE (gap=%.1f ego@(%.1f,%.1f) d=%.2f→%.2f)",
                             best_gap, g.ego_x, g.ego_y, g.lane_d, effective_lane_d);
                } else {
                    LOG_INFO("control", ">>> LANE CHANGE BLOCKED by obstacle in target lane");
                    g.lc_timer = g.blocked_timeout_s;
                }
            }
        } else if (!blocked && g.lc_state == 0) {
            g.lc_timer = 0;
        }

        /* 变道回正: 左车道巡航后回到原车道 */
        if (g.lc_state == 2) {
            g.lc_wait += 0.05;
            if (g.lc_wait > 8.0) {
                int right_clear = 1;
                double orig_y = g.lc_origin_y;
                for (int i = 0; i < MAX_OBS; i++) {
                    if (!g.obs_valid[i]) continue;
                    double dy = fabs(g.obs_y[i] - orig_y);
                    if (dy < same_lane_tol) {
                        double dx = g.obs_x[i] - g.ego_x;
                        double rel_spd = g.current_speed - g.obs_vx[i];
                        if ((dx > -20.0 && dx < 80.0 && rel_spd > -3.0) ||
                            (dx < 0.0 && dx > -20.0 && rel_spd > 2.0)) {
                            right_clear = 0; break;
                        }
                    }
                }
                if (right_clear) {
                    g.lc_target_y = orig_y;
                    effective_lane_d = g.lc_target_y;
                    g.lc_state = 3;
                    LOG_INFO("control", ">>> LANE CHANGE RETURN (orig_y=%.1f)", orig_y);
                }
            }
        }

        /* 检测变道完成 (横向偏差 < 0.3m) */
        if (g.lc_state == 1 && fabs(g.ego_y - effective_lane_d) < 0.3) {
            g.lc_state = 2; g.lc_wait = 0;
            LOG_INFO("control", ">>> lane change complete");
        }
        if (g.lc_state == 3 && fabs(g.ego_y - effective_lane_d) < 0.3) {
            g.lc_state = 0;
            LOG_INFO("control", ">>> returned to original lane");
        }

        /* ── 纵向 PID 计算 (目标为 ACC 限速后的值) ── */
        double error = acc_target - g.current_speed;
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

        /* ── 横向级联 PD：lat_error → psi_des → steer（阻尼消振） ── */
        double steer = 0.0;
        double lat_error = effective_lane_d - g.ego_y;
        /* P 层: 横向偏差 → 期望航向 */
        double psi_des = g.lat_kp * lat_error;
        if (psi_des >  MAX_PSI_DES_RAD) psi_des =  MAX_PSI_DES_RAD;
        if (psi_des < -MAX_PSI_DES_RAD) psi_des = -MAX_PSI_DES_RAD;
        /* D 层: 航向误差 → 转向（阻尼项，防止欠阻尼振荡） */
        steer = g.lat_kd_heading * (psi_des - g.ego_heading);
        if (steer >  0.25) steer =  0.25;
        if (steer < -0.25) steer = -0.25;
        /* 一阶低通滤波，防止跳变 */
        steer = STEER_FILTER_NEW * steer + STEER_FILTER_PREV * g.prev_steer;
        g.prev_steer = steer;

        /* ── 发布控制指令 ── */
        char cmd[256];
        snprintf(cmd, sizeof(cmd),
                 "throttle=%.2f brake=%.2f steer=%.4f "
                 "speed=%.1f target=%.1f error=%.1f mode=%s",
                 throttle, brake, steer,
                 g.current_speed, boost_target, error, mode);
        transport_publish(g.transport, "control/cmd",
                          (const uint8_t*)cmd, (uint32_t)strlen(cmd) + 1);

        g.prev_error = error;

        if (g.cycle % 20 == 1) {
            LOG_INFO("control", "#%d spd=%.1f→%.1f err=%.1f thr=%.2f brk=%.2f st=%.4f d=%.2f eff=%.2f lc=%d %s",
                     g.cycle, g.current_speed, g.target_speed,
                     error, throttle, brake, steer, g.lane_d, effective_lane_d, g.lc_state, mode);
        }
    }

    LOG_INFO("control", "stopped (%u cycles, final speed=%.1f m/s)",
             g.cycle, g.current_speed);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", "vehicle/state", NULL };
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
    g.lat_kp          = 0.5;   /* lateral error → desired heading (rad/m), 与 sim 内置一致 */
    g.lat_kd_heading  = 2.0;   /* heading error → steer, 阻尼增益 */
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
        if ((p = strstr(params_json, "\"lane_width\":")))
            sscanf(p + 13, "%lf", &g.lane_width);
        if ((p = strstr(params_json, "\"lat_kp\":")))
            sscanf(p + 9, "%lf", &g.lat_kp);
        if ((p = strstr(params_json, "\"lat_kd_heading\":")))
            sscanf(p + 17, "%lf", &g.lat_kd_heading);
        if ((p = strstr(params_json, "\"lane_change_blocked_timeout_s\":")))
            sscanf(p + 32, "%lf", &g.blocked_timeout_s);
        g.kp = g.cfg_kp; g.ki = g.cfg_ki; g.kd = g.cfg_kd;
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_trajectory, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);
    transport_advertise(transport, "control/cmd", 0x2D95C6D2u);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
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
