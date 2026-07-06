/**
 * planning_node.c — Frenet 轨迹规划节点插件
 *
 * 订阅 fusion/localization → 发布 planning/trajectory
 * 使用 Frenet 最优轨迹规划器生成平滑行驶轨迹。
 *
 * NodePlugin 接口，编译为 libplanning_node.so。
 */

#include "node_plugin.h"
#include "frenet_bridge.h"
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

    /* Frenet 规划器 */
    FrenetHandle* frenet;
    int           plan_count;
    double        target_speed;

    /* 从 fusion/localization 解析的最新状态 */
    double ego_x, ego_y, ego_v, ego_heading;
    volatile int has_fusion;

    /* 从 vehicle/state 解析的障碍物位置（世界坐标） */
    double obs_x[3], obs_y[3], obs_vx[3];
    volatile int has_vstate;

    /* 配置参数 */
    double cfg_target_speed;
    double cfg_max_speed;
    double cfg_max_accel;
    double cfg_ref_path_length;

    int tid;  /* scheduler task id */
} g;

/* 障碍物过滤与传递给 Frenet 规划器的空间范围 */
#define OBS_MIN_DX_M      -10.0   /* 忽略已经落后 ego 超过此距离的障碍物 */
#define OBS_MAX_DX_M      120.0   /* 忽略前方超过此距离的障碍物 */
#define OBS_MAX_ABS_Y_M     6.0   /* 忽略横向距离超出道路范围的障碍物 */
#define OBSTACLE_WIDTH_M    2.0   /* 默认障碍物宽度 (m) */
#define OBSTACLE_LENGTH_M   4.6   /* 默认障碍物长度 (m) */

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"x\":")))       sscanf(p + 4, "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))       sscanf(p + 4, "%lf", &g.ego_y);
    if ((p = strstr(d, "\"v\":")))       sscanf(p + 4, "%lf", &g.ego_v);
    if ((p = strstr(d, "\"heading\":"))) sscanf(p + 10, "%lf", &g.ego_heading);
    g.has_fusion = 1;
}

/* ── vehicle/state 订阅 — 解析障碍物位置（世界坐标） ─────────── */

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    for (int i = 0; i < 3; i++) {
        char key[16];
        int klen;
        klen = snprintf(key, sizeof(key), "\"ox%d\":", i);
        const char* p = strstr(d, key);
        if (p) sscanf(p + klen, "%lf", &g.obs_x[i]);
        klen = snprintf(key, sizeof(key), "\"oy%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_y[i]);
        klen = snprintf(key, sizeof(key), "\"ov%d\":", i);
        if ((p = strstr(d, key))) sscanf(p + klen, "%lf", &g.obs_vx[i]);
    }
    g.has_vstate = 1;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* planning_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "planning");

    /* 参考路径: 200m 直线（左车道中心 y=-1.75）*/
    double wx[101], wy[101];
    int ref_n = 101;
    for (int i = 0; i < ref_n; i++) {
        wx[i] = (double)i * (g.cfg_ref_path_length / (double)(ref_n - 1));
        wy[i] = -1.75;
    }
    frenet_set_reference_path(g.frenet, wx, wy, ref_n);

    while (!g.should_stop) {
        usleep(50000);  /* 20Hz 检查 */
        if (g.should_stop || !g.has_fusion) continue;

        /* 向 Frenet 规划器注入障碍物（世界坐标），触发自动避障/变道 */
        if (g.has_vstate) {
            double ox[3], oy[3], ow[3], ol[3];
            int n_obs = 0;
            for (int i = 0; i < 3; i++) {
                /* 只传入前方和侧方的有效障碍物（排除行人 y>4） */
                double dx = g.obs_x[i] - g.ego_x;
                if (dx < OBS_MIN_DX_M || dx > OBS_MAX_DX_M) continue;
                if (fabs(g.obs_y[i]) > OBS_MAX_ABS_Y_M) continue;
                ox[n_obs] = g.obs_x[i];
                oy[n_obs] = g.obs_y[i];
                ow[n_obs] = OBSTACLE_WIDTH_M;
                ol[n_obs] = OBSTACLE_LENGTH_M;
                n_obs++;
            }
            frenet_set_obstacles(g.frenet, ox, oy, ow, ol, n_obs);
        }

        /* Frenet 规划（参考路径在 y=-1.75, ego_d 是相对参考线的横向偏移） */
        double s_out[50], d_out[50], spd_out[50];
        double ego_d = g.ego_y + 1.75;  /* 相对 y=-1.75 的偏移 */
        int n_wp = frenet_plan(g.frenet,
            g.ego_x, ego_d, g.ego_v,
            g.target_speed,
            s_out, d_out, spd_out, 50);

        char traj[1024];
        int off;
        if (n_wp > 0) {
            off = snprintf(traj, sizeof(traj),
                "{\"type\":\"frenet\",\"plan\":%d,\"wp\":%d,",
                g.plan_count, n_wp);
            off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                "\"target_speed\":%.1f,", spd_out[0]);
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "\"path\":[");
            for (int i = 0; i < n_wp && off < (int)sizeof(traj) - 50; i++) {
                if (i % 3 != 0 && i > 0 && i < n_wp - 1) continue;
                off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                    "%s[%.1f,%.1f,%.1f]",
                    i > 0 ? "," : "", s_out[i], d_out[i], spd_out[i]);
            }
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "]}");
        } else {
            double failsafe = g.ego_v + 2.0;
            if (failsafe > g.cfg_max_speed) failsafe = g.cfg_max_speed;
            off = snprintf(traj, sizeof(traj),
                "{\"type\":\"failsafe\",\"target_speed\":%.1f,\"plan\":%d,"
                "\"lane_keep_d\":%.2f}",
                failsafe, g.plan_count, 0.0);  /* Frenet d=0 → 保持在参考线上 */
        }

        /* 后向兼容: PID 也读取 speed= 字段 */
        char traj_final[1100];
        snprintf(traj_final, sizeof(traj_final), "%s speed=%.1f",
                 traj, g.target_speed);

        transport_publish(g.transport, "planning/trajectory",
                          (const uint8_t*)traj_final,
                          (uint32_t)strlen(traj_final) + 1);
        g.plan_count++;

        if (g.plan_count % 25 == 1) {
            LOG_INFO("planning", "#%d ego@(%.0f,%.1f) v=%.1f → target=%.1f wp=%d",
                     g.plan_count, g.ego_x, g.ego_y, g.ego_v,
                     g.target_speed, n_wp);
        }
    }

    LOG_INFO("planning", "stopped (%d trajectories)", g.plan_count);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "fusion/localization", "vehicle/state", NULL };
static const char* s_outputs[] = { "planning/trajectory", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int planning_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.should_stop  = 0;
    g.has_fusion   = 0;

    /* 默认参数 */
    g.cfg_target_speed     = 15.0;
    g.cfg_max_speed        = 20.0;
    g.cfg_max_accel        = 4.0;
    g.cfg_ref_path_length  = 200.0;

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"target_speed\":")))
            sscanf(p + 15, "%lf", &g.cfg_target_speed);
        if ((p = strstr(params_json, "\"max_speed\":")))
            sscanf(p + 12, "%lf", &g.cfg_max_speed);
        if ((p = strstr(params_json, "\"max_accel\":")))
            sscanf(p + 12, "%lf", &g.cfg_max_accel);
        if ((p = strstr(params_json, "\"ref_path_length_m\":")))
            sscanf(p + 20, "%lf", &g.cfg_ref_path_length);
    }

    g.target_speed = g.cfg_target_speed;

    /* Frenet 规划器 */
    g.frenet = frenet_create(g.cfg_max_speed, g.cfg_max_accel);
    if (!g.frenet) {
        LOG_ERROR("planning", "frenet_create failed");
        return -1;
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);
    transport_advertise(transport, "planning/trajectory", 0x3A7B1C2Du);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_PUBLISHER, 10.0);

    LOG_INFO("planning", "initialized (Frenet, target=%.0f m/s, max=%.0f m/s)",
             g.cfg_target_speed, g.cfg_max_speed);
    return 0;
}

static int planning_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, planning_thread, NULL) != 0) return -1;
    LOG_INFO("planning", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void planning_stop(void)       { g.should_stop = 1; }
static void planning_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    if (g.frenet) { frenet_destroy(g.frenet); g.frenet = NULL; }
    LOG_INFO("planning", "cleanup done");
}
static int  planning_health(void)     { return g.frenet ? 0 : -1; }

static NodePlugin s_plugin = {
    .name          = "planning",
    .version       = "1.0.0",
    .description   = "Frenet Optimal Trajectory Planner",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = planning_init,
    .start         = planning_start,
    .stop          = planning_stop,
    .cleanup       = planning_cleanup,
    .health        = planning_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
