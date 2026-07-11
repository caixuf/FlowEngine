/**
 * planning_node.c — Frenet 轨迹规划节点插件
 *
 * 订阅 fusion/localization → 发布 planning/trajectory
 * 使用 Frenet 最优轨迹规划器生成平滑行驶轨迹。
 *
 * NodePlugin 接口，编译为 libplanning_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"
#include "logger.h"

#ifdef HAVE_FRENET
#include "frenet_bridge.h"
#endif

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

    /* 反射式状态机：跟踪生命周期 */
    ReflectiveStateMachine sm;

    /* Frenet 规划器 */
#ifdef HAVE_FRENET
    FrenetHandle* frenet;
#else
    void*         frenet;  /* unused stub */
#endif
    int           plan_count;
    double        target_speed;

    /* 从 fusion/localization 解析的最新状态 */
    double ego_x, ego_y, ego_v, ego_heading;
    volatile int has_fusion;

    /* 从 vehicle/state 解析的障碍物位置（世界坐标） */
    double obs_x[4], obs_y[4], obs_vx[4];
    volatile int has_vstate;

    /* 配置参数 */
    double cfg_target_speed;
    double cfg_max_speed;
    double cfg_max_accel;
    double cfg_ref_path_length;
    double ref_path_start_x;

    int tid;  /* scheduler task id */
} g;

/* 障碍物过滤与传递给 Frenet 规划器的空间范围 */
#define OBS_MIN_DX_M      -10.0   /* 忽略已经落后 ego 超过此距离的障碍物 */
#define OBS_MAX_DX_M      120.0   /* 忽略前方超过此距离的障碍物 */
#define OBS_MAX_ABS_Y_M     6.0   /* 忽略横向距离超出道路范围的障碍物 */
#define OBSTACLE_WIDTH_M    2.0   /* 默认障碍物宽度 (m) */
#define OBSTACLE_LENGTH_M   4.6   /* 默认障碍物长度 (m) */

static void update_reference_path(double start_x) {
#ifdef HAVE_FRENET
    double wx[101], wy[101];
    const int ref_n = 101;
    for (int i = 0; i < ref_n; i++) {
        wx[i] = start_x + (double)i * (g.cfg_ref_path_length / (double)(ref_n - 1));
        wy[i] = -1.75;
    }
    frenet_set_reference_path(g.frenet, wx, wy, ref_n);
    g.ref_path_start_x = start_x;
#else
    (void)start_x;
#endif
}

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* Try binary deserialization (serializer path) */
    {
        Localization loc;
        if (Localization_deserialize(&loc, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.ego_x       = loc.x;
            g.ego_y       = loc.y;
            g.ego_v       = loc.v;
            g.ego_heading = loc.heading;
            g.has_fusion  = 1;
            return;
        }
    }

    /* Fallback: text JSON parsing */
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
    for (int i = 0; i < 4; i++) {
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

    /* 参考路径: 长直线（左车道中心 y=-1.75）。运行时间较长时，ego 会开出
     * 初始路径范围；接近末端时向前滑动 reference path，避免 Frenet 插值越界
     * 导致 planning 线程挂掉。 */
    update_reference_path(0.0);

    while (!g.should_stop) {
        usleep(50000);  /* 20Hz 检查 */
        if (g.should_stop || !g.has_fusion) continue;

        if (g.ego_x > g.ref_path_start_x + g.cfg_ref_path_length * 0.8) {
            double new_start = g.ego_x - 50.0;
            if (new_start < 0.0) new_start = 0.0;
            update_reference_path(new_start);
            LOG_INFO("planning", "reference path shifted to x=%.0f..%.0f",
                     g.ref_path_start_x, g.ref_path_start_x + g.cfg_ref_path_length);
        }

        double command_speed = g.target_speed;

        /* 向 Frenet 规划器注入障碍物（世界坐标），触发自动避障/变道 */
#ifdef HAVE_FRENET
        if (g.has_vstate) {
            double ox[4], oy[4], ow[4], ol[4];
            int n_obs = 0;
            for (int i = 0; i < 4; i++) {
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
#endif

        /* 规划轨迹 */
        double s_out[50], d_out[50], spd_out[50];
        int n_wp = 0;

#ifdef HAVE_FRENET
        {
            double ego_d = g.ego_y + 1.75;  /* 相对 y=-1.75 的偏移 */
            n_wp = frenet_plan(g.frenet,
                g.ego_x, ego_d, g.ego_v,
                command_speed,
                s_out, d_out, spd_out, 50);
        }
#else
        /* Fallback: 生成简单的车道保持 + 恒速轨迹 */
        {
            double horizon = 50.0;   /* 前方 50m */
            int n = 10;              /* 10 个路径点 */
            for (int i = 0; i < n; i++) {
                s_out[i] = g.ego_x + horizon * (double)i / (double)(n - 1);
                d_out[i] = 0.0;     /* 保持参考线（y=-1.75） */
                spd_out[i] = command_speed;
            }
            n_wp = n;
        }
#endif

        char traj[1024];
        int off;
        if (n_wp > 0) {
            command_speed = spd_out[0];
            off = snprintf(traj, sizeof(traj),
                "{\"type\":\"frenet\",\"plan\":%d,\"wp\":%d,",
                g.plan_count, n_wp);
            off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                "\"target_speed\":%.1f,", command_speed);
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "\"path\":[");
            for (int i = 0; i < n_wp && off < (int)sizeof(traj) - 50; i++) {
                if (i % 3 != 0 && i > 0 && i < n_wp - 1) continue;
                off += snprintf(traj + off, sizeof(traj) - (size_t)off,
                    "%s[%.1f,%.1f,%.1f]",
                    i > 0 ? "," : "", s_out[i], d_out[i], spd_out[i]);
            }
            off += snprintf(traj + off, sizeof(traj) - (size_t)off, "]}");
        } else {
            double failsafe = command_speed;
            if (failsafe > g.ego_v + 1.0) failsafe = g.ego_v + 1.0;
            if (failsafe > g.cfg_max_speed) failsafe = g.cfg_max_speed;
            off = snprintf(traj, sizeof(traj),
                "{\"type\":\"failsafe\",\"target_speed\":%.1f,\"plan\":%d,"
                "\"lane_keep_d\":%.2f}",
                failsafe, g.plan_count, 0.0);  /* Frenet d=0 → 保持在参考线上 */
        }

        /* 后向兼容: PID 也读取 speed= 字段 */
        char traj_final[1100];
        snprintf(traj_final, sizeof(traj_final), "%s speed=%.1f",
                 traj, command_speed);

        transport_publish(g.transport, "planning/trajectory",
                          (const uint8_t*)traj_final,
                          (uint32_t)strlen(traj_final) + 1);
        g.plan_count++;

        if (g.plan_count % 25 == 1) {
            LOG_INFO("planning", "#%d ego@(%.0f,%.1f) v=%.1f → target=%.1f wp=%d",
                     g.plan_count, g.ego_x, g.ego_y, g.ego_v,
                     command_speed, n_wp);
        }
    }

    LOG_INFO("planning", "stopped (%d trajectories, state=%s)",
             g.plan_count, statem_state_name(&g.sm, g.sm.current));
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
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
    g.cfg_ref_path_length  = 5000.0;

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
#ifdef HAVE_FRENET
    g.frenet = frenet_create(g.cfg_max_speed, g.cfg_max_accel);
    if (!g.frenet) {
        LOG_ERROR("planning", "frenet_create failed");
        return -1;
    }
#else
    g.frenet = NULL;
    LOG_INFO("planning", "built without Frenet — using lane-keep fallback");
#endif

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);
    transport_advertise(transport, "planning/trajectory", 0x3A7B1C2Du);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_PUBLISHER, 10.0);

    /* 初始化反射式状态机 */
    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "planning");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("planning", "initialized (Frenet, target=%.0f m/s, max=%.0f m/s)",
             g.cfg_target_speed, g.cfg_max_speed);
    return 0;
}

static int planning_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, planning_thread, NULL) != 0) return -1;
    LOG_INFO("planning", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void planning_stop(void)       { g.should_stop = 1; }
static void planning_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
#ifdef HAVE_FRENET
    if (g.frenet) { frenet_destroy(g.frenet); g.frenet = NULL; }
#else
    g.frenet = NULL;
#endif
    LOG_INFO("planning", "cleanup done");
}
static int  planning_health(void)     { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
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
