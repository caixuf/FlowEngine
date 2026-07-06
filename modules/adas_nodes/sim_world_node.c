/**
 * sim_world_node.c — 仿真世界节点插件
 *
 * 模拟车辆动力学 + 障碍物运动学，作为传感器数据源。
 * 可订阅 control/cmd 接收 PID 控制指令，无指令时按默认参数运行。
 * 输出 vehicle/state JSON 供 perception_node 消费。
 *
 * NodePlugin 接口，编译为 libsim_world.so。
 */

#include "node_plugin.h"
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

/* ── 仿真常量 ────────────────────────────────────────────────── */

#define SAME_LANE_TOL_M    2.0
#define EGO_LEN_M          4.6
#define SIM_OBSTACLE_COUNT 3
#define MAX_SPEED          20.0
#define FREQUENCY_HZ       20.0
#define DT_SEC             (1.0 / FREQUENCY_HZ)

/* ── 仿真障碍物 ────────────────────────────────────────────────── */

typedef struct {
    int    id;
    char   type[16];
    double vx, vy;
    double x, y;
    double len, wid;
} SimObstacle;

/* ── 车辆状态 ────────────────────────────────────────────────── */

typedef struct {
    double x, y;
    double speed;
    double heading;
    double steer;
    double throttle, brake;
    double target_speed;
    double lane_target;
    double wheelbase;
    double mass;
    double drag_coeff;
} VehicleSim;

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* 仿真状态 */
    VehicleSim    vehicle;
    SimObstacle   obstacles[SIM_OBSTACLE_COUNT];
    uint32_t      cycle;
    int           has_control_input;  /* 是否收到过 control/cmd */

    /* 变道状态机 */
    double lc_start_y;
    double lc_target_y;
    double lc_elapsed;
    double lc_duration;
    int    lc_active;
    double lc_timer;
    int    lc_state;     /* 0=正常 1=左变道中 2=左车道巡航 3=右回正 */
    double lc_wait;

    /* 配置参数 */
    double init_speed;
    double target_speed;
    double lane_width;
    int    obstacle_count;
} g;

/* ── 障碍物初始化 ─────────────────────────────────────────────── */

static void init_obstacles(void) {
    /* 障碍物 0: 在 ego 同车道前方 35m, 慢车 7 m/s — 用于触发变道 */
    g.obstacles[0] = (SimObstacle){ 0, "car",          7.0,  0.0, 35.0, -1.75, 4.6, 2.0 };
    /* 障碍物 1: 在邻道(右车道 y=1.75), 120m 前, 稍慢 — 远到不阻碍变道 */
    g.obstacles[1] = (SimObstacle){ 1, "car",          9.0,  0.0, 120.0, 1.75, 4.6, 2.0 };
    /* 障碍物 2: 行人, 在路边往复行走 */
    g.obstacles[2] = (SimObstacle){ 2, "pedestrian",   0.0,  0.6, 55.0,  8.0,  0.6, 0.6 };
}

/* ── 障碍物运动学 ─────────────────────────────────────────────── */

static void obstacles_tick(void) {
    for (int i = 0; i < SIM_OBSTACLE_COUNT; i++) {
        SimObstacle* o = &g.obstacles[i];
        o->x += o->vx * DT_SEC;
        o->y += o->vy * DT_SEC;
        if (strcmp(o->type, "pedestrian") == 0) {
            if (o->y >  8.0) { o->y =  8.0; o->vy = -fabs(o->vy); }
            if (o->y < -8.0) { o->y = -8.0; o->vy =  fabs(o->vy); }
        }
        double rel = o->x - g.vehicle.x;
        if (o->vx >= 0) {
            if (rel < -40.0)  o->x = g.vehicle.x + 120.0 + (double)i * 5.0;
            if (rel >  220.0) o->x = g.vehicle.x + 100.0;
        } else {
            if (rel < -50.0) o->x = g.vehicle.x + 500.0;
        }
    }
}

/* ── 同车道前车间距 ───────────────────────────────────────────── */

static double lead_gap(void) {
    double best = 1e9;
    for (int i = 0; i < SIM_OBSTACLE_COUNT; i++) {
        SimObstacle* o = &g.obstacles[i];
        if (o->vx < 0) continue;
        if (fabs(o->y - g.vehicle.y) > SAME_LANE_TOL_M) continue;
        double center = o->x - g.vehicle.x;
        double gap = center - (o->len * 0.5 + EGO_LEN_M * 0.5);
        if (center > 0 && gap < best) best = gap;
    }
    return best;
}

/* ── 车辆运动学（bicycle model） ──────────────────────────────── */

static void vehicle_tick(void) {
    /* 驱动力 */
    double drive_force  = g.vehicle.throttle * 5000.0;
    double brake_force  = g.vehicle.brake    * 8000.0;
    double drag_force   = g.vehicle.drag_coeff * g.vehicle.speed * g.vehicle.speed;
    double net_force    = drive_force - brake_force - drag_force;
    double accel        = net_force / g.vehicle.mass;

    g.vehicle.speed += accel * DT_SEC;
    if (g.vehicle.speed < 0) g.vehicle.speed = 0;

    /* ── 横向: 收到外部 control/cmd 时完全交由外部 PID 控制 ── */
    if (g.has_control_input) {
        /* 外部 steer 已由 on_control_cmd 设置到 g.vehicle.steer,
         * 直接走自行车模型积分，不覆盖 steer。 */
    } else {
        /* ── 无外部控制时使用内置车道保持 + 平滑变道轨迹 ── */
        if (fabs(g.vehicle.lane_target - g.lc_target_y) > 0.1 && !g.lc_active) {
            g.lc_start_y  = g.vehicle.y;
            g.lc_target_y = g.vehicle.lane_target;
            g.lc_elapsed  = 0;
            g.lc_active   = 1;
        }
        if (g.lc_active && g.lc_elapsed >= g.lc_duration) {
            g.vehicle.y = g.lc_target_y;
            g.lc_active = 0;
        }

        double y_desired;
        if (g.lc_active) {
            g.lc_elapsed += DT_SEC;
            double t = g.lc_elapsed / g.lc_duration;
            if (t > 1.0) t = 1.0;
            double s = sin(t * M_PI / 2.0);
            y_desired = g.lc_start_y + (g.lc_target_y - g.lc_start_y) * s * s;
        } else {
            y_desired = g.vehicle.lane_target;
        }

        double y_err   = y_desired - g.vehicle.y;
        double psi_des = 0.5 * y_err;
        psi_des = fmax(-0.3, fmin(0.3, psi_des));
        double steer   = 2.0 * (psi_des - g.vehicle.heading);
        steer = fmax(-0.25, fmin(0.25, steer));
        g.vehicle.steer = steer;
    }

    g.vehicle.heading += (g.vehicle.speed / g.vehicle.wheelbase)
                         * tan(g.vehicle.steer) * DT_SEC;
    g.vehicle.x += g.vehicle.speed * DT_SEC * cos(g.vehicle.heading);
    g.vehicle.y += g.vehicle.speed * DT_SEC * sin(g.vehicle.heading);

    obstacles_tick();
}

/* ── control/cmd 订阅回调 ──────────────────────────────────── */

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "throttle="))) sscanf(p + 9, "%lf", &g.vehicle.throttle);
    if ((p = strstr(d, "brake=")))    sscanf(p + 6,  "%lf", &g.vehicle.brake);
    if ((p = strstr(d, "steer=")))    sscanf(p + 6,  "%lf", &g.vehicle.steer);
    if ((p = strstr(d, "target=")))   sscanf(p + 7,  "%lf", &g.vehicle.target_speed);
    g.has_control_input = 1;
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* sim_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "sim_world");

    while (!g.should_stop) {
        usleep((unsigned long)(DT_SEC * 1e6));
        if (g.should_stop) break;

        /* 未收到 control/cmd 时：使用内置简单巡航 + 车道保持 */
        if (!g.has_control_input) {
            double gap = lead_gap();
            double time_hw  = 2.0;
            double min_gap  = 6.0;
            double safe_gap = min_gap + g.vehicle.speed * time_hw;
            double target   = g.target_speed;

            if (gap < safe_gap && gap < 80.0) {
                double ratio = gap / safe_gap;
                if (ratio < 0.2) ratio = 0.2;
                target = g.target_speed * ratio;
            }

            double error = target - g.vehicle.speed;
            double output = 800.0 * error;
            if (output > 0) {
                g.vehicle.throttle = output / 5000.0;
                if (g.vehicle.throttle > 1.0) g.vehicle.throttle = 1.0;
                g.vehicle.brake = 0;
            } else {
                g.vehicle.throttle = 0;
                g.vehicle.brake = (-output) / 8000.0;
                if (g.vehicle.brake > 1.0) g.vehicle.brake = 1.0;
            }
        }

        vehicle_tick();

        /* ── 发布 vehicle/state ── */
        char vstate[512];
        snprintf(vstate, sizeof(vstate),
                 "{\"x\":%.2f,\"y\":%.2f,\"spd\":%.3f,\"hdg\":%.4f,"
                 "\"thr\":%.3f,\"brk\":%.3f,\"tgt\":%.2f,\"st\":%.4f,"
                 "\"ox0\":%.2f,\"oy0\":%.2f,\"ov0\":%.3f,"
                 "\"ox1\":%.2f,\"oy1\":%.2f,\"ov1\":%.3f,"
                 "\"ox2\":%.2f,\"oy2\":%.2f,\"ov2\":%.3f}",
                 g.vehicle.x, g.vehicle.y, g.vehicle.speed, g.vehicle.heading,
                 g.vehicle.throttle, g.vehicle.brake, g.vehicle.target_speed, g.vehicle.steer,
                 g.obstacles[0].x, g.obstacles[0].y, g.obstacles[0].vx,
                 g.obstacles[1].x, g.obstacles[1].y, g.obstacles[1].vx,
                 g.obstacles[2].x, g.obstacles[2].y, g.obstacles[2].vx);
        transport_publish(g.transport, "vehicle/state", (const uint8_t*)vstate,
                          (uint32_t)strlen(vstate) + 1);

        g.cycle++;
    }

    LOG_INFO("sim_world", "stopped (%u cycles, final speed=%.1f m/s)",
             g.cycle, g.vehicle.speed);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "control/cmd", NULL };
static const char* s_outputs[] = { "vehicle/state", "sim/world_state", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int sim_init(MessageBus* bus, Transport* transport,
                    DiscoveryManager* discovery, Scheduler* scheduler,
                    const char* params_json) {
    (void)bus; (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.should_stop  = 0;

    /* 默认参数 */
    g.init_speed    = 5.0;
    g.target_speed  = 10.0;
    g.lane_width    = 3.5;
    g.obstacle_count = 3;

    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"init_speed\":")))
            sscanf(p + 13, "%lf", &g.init_speed);
        if ((p = strstr(params_json, "\"target_speed\":")))
            sscanf(p + 14, "%lf", &g.target_speed);
        if ((p = strstr(params_json, "\"lane_width\":")))
            sscanf(p + 12, "%lf", &g.lane_width);
        if ((p = strstr(params_json, "\"obstacle_count\":")))
            sscanf(p + 17, "%d", &g.obstacle_count);
        if ((p = strstr(params_json, "\"lane_target\":")))
            sscanf(p + 13, "%lf", &g.vehicle.lane_target);
    }

    srand((unsigned)time(NULL));

    /* 初始化车辆 — 在左车道中心 (车道宽3.5m, 中心y=-1.75) */
    g.vehicle.x           = 0.0;
    g.vehicle.y           = -1.75;
    g.vehicle.speed       = g.init_speed;
    g.vehicle.heading     = 0.0;
    g.vehicle.steer       = 0.0;
    g.vehicle.throttle    = 0.0;
    g.vehicle.brake       = 0.0;
    g.vehicle.target_speed = g.target_speed;
    g.vehicle.lane_target  = -1.75;  /* 左车道中心（车道宽3.5m） */
    g.vehicle.wheelbase   = 2.7;
    g.vehicle.mass        = 1500.0;
    g.vehicle.drag_coeff  = 0.3;

    /* 变道状态 */
    g.lc_duration = 3.5;

    init_obstacles();

    /* 订阅 control/cmd */
    transport_subscribe(transport, "control/cmd", on_control_cmd, NULL);
    discovery_advertise(discovery, "control/cmd", 0x2D95C6D2u, CAP_SUBSCRIBER, 0);

    /* 发布 vehicle/state */
    transport_advertise(transport, "vehicle/state", 0x1C0E5A7Eu);
    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu, CAP_PUBLISHER, 20.0);

    LOG_INFO("sim_world", "initialized (init=%.1f m/s, target=%.1f m/s, %.0f Hz)",
             g.init_speed, g.target_speed, FREQUENCY_HZ);
    return 0;
}

static int sim_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, sim_thread, NULL) != 0) return -1;
    LOG_INFO("sim_world", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void sim_stop(void)          { g.should_stop = 1; }
static void sim_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    LOG_INFO("sim_world", "cleanup done");
}
static int  sim_health(void)        { return 0; }

static NodePlugin s_plugin = {
    .name          = "sim_world",
    .version       = "1.0.0",
    .description   = "Vehicle dynamics + obstacle simulation",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = sim_init,
    .start         = sim_start,
    .stop          = sim_stop,
    .cleanup       = sim_cleanup,
    .health        = sim_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
