/**
 * perception_node.c — 感知节点插件
 *
 * 实现 NodePlugin 接口，编译为 libperception_node.so。
 *
 * 输入 topics: vehicle/state (IPC 模式下同步仿真状态)
 * 输出 topics: perception/obstacles
 *
 * 算法:
 *   - DBSCAN 点云聚类 (dbscan_cluster.c) — eps=2m, min_pts=4
 *   - RANSAC 地面移除
 *   - 基于真值的障碍物聚类仿真
 */

#include "node_plugin.h"
#include "dbscan_cluster.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* 仿真状态 (通过 vehicle/state topic 更新) */
    double  ego_x, ego_y, ego_heading;
    double  ego_speed;
    int     n_obs;                          /* 当前场景实际 actor 数量 */
    double  obs_x[16], obs_y[16], obs_vx[16];

    /* 发布帧计数 */
    uint32_t frame_id;

    /* DBSCAN */
    DbscanCluster dbscan;

    /* 配置参数 */
    double dbscan_eps;
    int    dbscan_min_pts;
    int    lidar_rate_hz;
    double lidar_fov_deg;
    double lidar_max_range_m;
    double obs_noise_std_m;
    int    enable_simple_occlusion;
} g;

static double rand_uniform_signed(double span) {
    return (((double)rand() / (double)RAND_MAX) * 2.0 - 1.0) * span;
}

static int obstacle_in_fov(double rx, double ry, double max_range_m, double fov_deg) {
    const double range = hypot(rx, ry);
    if (range > max_range_m || range < 0.05) return 0;
    const double half_fov_rad = (fov_deg * 0.5) * M_PI / 180.0;
    const double ang = atan2(ry, rx);
    return fabs(ang) <= half_fov_rad;
}

/* ── vehicle/state 订阅 ──────────────────────────────────────── */

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"x\":")))   sscanf(p + 4, "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":")))   sscanf(p + 4, "%lf", &g.ego_y);
    if ((p = strstr(d, "\"hdg\":"))) sscanf(p + 6, "%lf", &g.ego_heading);
    if ((p = strstr(d, "\"spd\":"))) sscanf(p + 6, "%lf", &g.ego_speed);
    int n = 0;
    if ((p = strstr(d, "\"n_obs\":"))) sscanf(p + 8, "%d", &n);
    /* 3 was the original hardcoded obstacle count before P0.2; keep it as the
     * legacy fall-back so old vehicle/state messages without "n_obs" still work. */
    if (n < 1 || n > 16) n = 3;
    g.n_obs = n;
    for (int i = 0; i < n; i++) {
        char kx[16], ky[16], kvx[16];
        snprintf(kx,  sizeof(kx),  "\"ox%d\":", i);
        snprintf(ky,  sizeof(ky),  "\"oy%d\":", i);
        snprintf(kvx, sizeof(kvx), "\"ov%d\":", i);
        if ((p = strstr(d, kx)))  sscanf(p + strlen(kx),  "%lf", &g.obs_x[i]);
        if ((p = strstr(d, ky)))  sscanf(p + strlen(ky),  "%lf", &g.obs_y[i]);
        if ((p = strstr(d, kvx))) sscanf(p + strlen(kvx), "%lf", &g.obs_vx[i]);
    }
}

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* perception_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "perception");

    long period_us = 1000000L / (g.lidar_rate_hz > 0 ? g.lidar_rate_hz : 20);

    while (!g.should_stop) {
        usleep((unsigned long)period_us);
        if (g.should_stop) break;

        /* ── DBSCAN ── */
        {
            Point3D pts[256];
            int np = 0;
            double ch = cos(-g.ego_heading), sh = sin(-g.ego_heading);

            /* 地面环 */
            for (int ring = 0; ring < 2 && np < 256; ring++) {
                float r = 6.0f + (float)ring * 4.0f;
                for (int k = 0; k < 12 && np < 256; k++) {
                    float a = (float)k / 12.0f * 2.0f * (float)M_PI;
                    pts[np].x = cosf(a) * r; pts[np].y = sinf(a) * r;
                    pts[np].z = 0.05f; pts[np].intensity = 0.3f; np++;
                }
            }
            /* 传感器可见障碍物（FOV/量程/简化遮挡） */
            double vis_rx[16], vis_ry[16], vis_r[16], vis_a[16];
            int vis_idx[16];
            int vis_count = 0;
            for (int oi = 0; oi < g.n_obs && vis_count < 16; oi++) {
                double dx = g.obs_x[oi] - g.ego_x;
                double dy = g.obs_y[oi] - g.ego_y;
                double rx = dx * ch - dy * sh;
                double ry = dx * sh + dy * ch;
                if (!obstacle_in_fov(rx, ry, g.lidar_max_range_m, g.lidar_fov_deg)) {
                    continue;
                }
                vis_rx[vis_count] = rx;
                vis_ry[vis_count] = ry;
                vis_r[vis_count] = hypot(rx, ry);
                vis_a[vis_count] = atan2(ry, rx);
                vis_idx[vis_count] = oi;
                vis_count++;
            }

            int vis_keep[16];
            for (int i = 0; i < vis_count; i++) vis_keep[i] = 1;
            if (g.enable_simple_occlusion) {
                const double occ_beam = 5.0 * M_PI / 180.0;
                for (int i = 0; i < vis_count; i++) {
                    if (!vis_keep[i]) continue;
                    for (int j = 0; j < vis_count; j++) {
                        if (i == j || !vis_keep[j]) continue;
                        if (fabs(vis_a[i] - vis_a[j]) < occ_beam && vis_r[j] + 2.0 < vis_r[i]) {
                            vis_keep[i] = 0;
                            break;
                        }
                    }
                }
            }

            /* 障碍物表面点 */
            for (int vi = 0; vi < vis_count && np < 256; vi++) {
                if (!vis_keep[vi]) continue;
                (void)vis_idx[vi];
                double rx = vis_rx[vi];
                double ry = vis_ry[vi];
                for (int k = 0; k < 8 && np < 256; k++) {
                    pts[np].x = (float)rx + ((float)(k % 3) - 1.0f) * 0.8f + (float)rand_uniform_signed(g.obs_noise_std_m);
                    pts[np].y = (float)ry + ((float)(k / 3) - 1.0f) * 1.6f + (float)rand_uniform_signed(g.obs_noise_std_m);
                    pts[np].z = 0.6f + (float)(k % 4) * 0.4f;
                    pts[np].intensity = 0.7f; np++;
                }
            }

            int n_clusters = dbscan_run(&g.dbscan, pts, np);
            ObstacleList obs_list;
            memset(&obs_list, 0, sizeof(obs_list));
            obs_list.frame_id = g.frame_id;
            for (int ci = 0; ci < n_clusters && obs_list.count < 8; ci++) {
                const ClusterBounds* cb = dbscan_get_cluster(&g.dbscan, ci);
                if (!cb || cb->point_count < 3) continue;
                Obstacle* ob = &obs_list.obstacles[obs_list.count++];
                ob->id = (uint32_t)(g.frame_id * 100 + (uint32_t)ci);
                ob->x = cb->cx; ob->y = cb->cy;
                ob->width = cb->width; ob->length = cb->length;
                ob->confidence = cb->confidence;
                switch (cb->cls) {
                    case CLS_VEHICLE:    ob->type = OBJ_TYPE_VEHICLE;    break;
                    case CLS_PEDESTRIAN: ob->type = OBJ_TYPE_PEDESTRIAN; break;
                    default:             ob->type = OBJ_TYPE_UNKNOWN;    break;
                }
            }
            Message omsg;
            msg_init_typed(&omsg, "perception/obstacles", "perception",
                           0x0B5A010Eu, 1, &obs_list, sizeof(obs_list));
            transport_publish(g.transport, "perception/obstacles", omsg.data, omsg.data_size);
        }

        g.frame_id++;
    }

    LOG_INFO("perception", "stopped (%u frames)", g.frame_id);
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "vehicle/state", NULL };
static const char* s_outputs[] = { "perception/obstacles", NULL };

static NodePlugin s_plugin;  /* forward decl */
static int perception_init(MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler,
                           const char* params_json) {
    (void)bus; (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.should_stop  = 0;
    g.dbscan_eps   = 2.0;
    g.dbscan_min_pts = 4;
    g.lidar_rate_hz  = 20;
    g.lidar_fov_deg = 120.0;
    g.lidar_max_range_m = 60.0;
    g.obs_noise_std_m = 0.08;
    g.enable_simple_occlusion = 1;

    /* 解析参数 */
    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"dbscan_eps\":")))
            sscanf(p + 13, "%lf", &g.dbscan_eps);
        if ((p = strstr(params_json, "\"lidar_rate_hz\":")))
            sscanf(p + 16, "%d", &g.lidar_rate_hz);
        if ((p = strstr(params_json, "\"lidar_fov_deg\":")))
            sscanf(p + 16, "%lf", &g.lidar_fov_deg);
        if ((p = strstr(params_json, "\"lidar_max_range_m\":")))
            sscanf(p + 20, "%lf", &g.lidar_max_range_m);
        if ((p = strstr(params_json, "\"obs_noise_std_m\":")))
            sscanf(p + 18, "%lf", &g.obs_noise_std_m);
        if ((p = strstr(params_json, "\"enable_simple_occlusion\":")))
            sscanf(p + 26, "%d", &g.enable_simple_occlusion);
    }

    /* Fixed seed for reproducibility — sim_world drives deterministic time */
    srand(42u);
    dbscan_init(&g.dbscan, (float)g.dbscan_eps, g.dbscan_min_pts);
    dbscan_set_ransac(&g.dbscan, 100, 0.2f, 0.3f);

    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);

    discovery_advertise(discovery, "vehicle/state",         0x1C0E5A7Eu, CAP_SUBSCRIBER,  0);
    discovery_advertise(discovery, "perception/obstacles",  0x0B5A010Eu, CAP_PUBLISHER, 20.0);

    transport_advertise(transport, "perception/obstacles", 0x0B5A010Eu);

    LOG_INFO("perception", "initialized (DBSCAN eps=%.1f, LiDAR %dHz FOV=%.0fdeg range=%.0fm noise=%.2f occ=%d)",
             g.dbscan_eps, g.lidar_rate_hz, g.lidar_fov_deg, g.lidar_max_range_m,
             g.obs_noise_std_m, g.enable_simple_occlusion);
    return 0;
}

static int perception_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, perception_thread, NULL) != 0) return -1;
    LOG_INFO("perception", "started");
    node_announce_self(g.transport, &s_plugin);  /* start() 时广播: monitor 已订阅 */
    return 0;
}

static void perception_stop(void)    { g.should_stop = 1; }
static void perception_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    LOG_INFO("perception", "cleanup done");
}
static int  perception_health(void)  { return 0; }

static NodePlugin s_plugin = {
    .name          = "perception",
    .version       = "1.0.0",
    .description   = "LiDAR/GPS/Camera simulation + DBSCAN clustering",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = perception_init,
    .start         = perception_start,
    .stop          = perception_stop,
    .cleanup       = perception_cleanup,
    .health        = perception_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
