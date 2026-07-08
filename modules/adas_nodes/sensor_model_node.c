/**
 * sensor_model_node.c — 传感器模型节点插件
 *
 * 从 vehicle/state 真值生成带噪声/视场/简化遮挡约束的传感器输出。
 *
 * 输入 topics: vehicle/state
 * 输出 topics: sensor/lidar, sensor/gps, sensor/camera
 */

#include "node_plugin.h"
#include "adas_msgs_gen.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static struct {
    Transport* transport;
    DiscoveryManager* discovery;

    pthread_t thread;
    volatile int running;
    volatile int should_stop;

    double ego_x;
    double ego_y;
    double ego_heading;
    double ego_speed;

    int n_obs;
    double obs_x[16];
    double obs_y[16];

    uint32_t frame_id;

    int lidar_rate_hz;
    double lidar_fov_deg;
    double lidar_max_range_m;
    double obs_noise_std_m;
    int enable_simple_occlusion;
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

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    const char* d = (const char*)msg->data;
    const char* p;
    if ((p = strstr(d, "\"x\":"))) sscanf(p + 4, "%lf", &g.ego_x);
    if ((p = strstr(d, "\"y\":"))) sscanf(p + 4, "%lf", &g.ego_y);
    if ((p = strstr(d, "\"hdg\":"))) sscanf(p + 6, "%lf", &g.ego_heading);
    if ((p = strstr(d, "\"spd\":"))) sscanf(p + 6, "%lf", &g.ego_speed);

    int n = 0;
    if ((p = strstr(d, "\"n_obs\":"))) sscanf(p + 8, "%d", &n);
    if (n < 1 || n > 16) n = 3;
    g.n_obs = n;

    for (int i = 0; i < n; i++) {
        char kx[16], ky[16];
        snprintf(kx, sizeof(kx), "\"ox%d\":", i);
        snprintf(ky, sizeof(ky), "\"oy%d\":", i);
        if ((p = strstr(d, kx))) sscanf(p + (int)strlen(kx), "%lf", &g.obs_x[i]);
        if ((p = strstr(d, ky))) sscanf(p + (int)strlen(ky), "%lf", &g.obs_y[i]);
    }
}

static uint32_t estimate_visible_point_count(void) {
    double vis_r[16], vis_a[16];
    int vis_count = 0;

    const double ch = cos(-g.ego_heading);
    const double sh = sin(-g.ego_heading);

    for (int i = 0; i < g.n_obs && vis_count < 16; i++) {
        const double dx = g.obs_x[i] - g.ego_x;
        const double dy = g.obs_y[i] - g.ego_y;
        const double rx = dx * ch - dy * sh;
        const double ry = dx * sh + dy * ch;
        if (!obstacle_in_fov(rx, ry, g.lidar_max_range_m, g.lidar_fov_deg)) {
            continue;
        }
        vis_r[vis_count] = hypot(rx, ry);
        vis_a[vis_count] = atan2(ry, rx);
        vis_count++;
    }

    int visible_after_occ = vis_count;
    if (g.enable_simple_occlusion) {
        int keep[16];
        const double occ_beam = 5.0 * M_PI / 180.0;
        for (int i = 0; i < vis_count; i++) keep[i] = 1;
        for (int i = 0; i < vis_count; i++) {
            if (!keep[i]) continue;
            for (int j = 0; j < vis_count; j++) {
                if (i == j || !keep[j]) continue;
                if (fabs(vis_a[i] - vis_a[j]) < occ_beam && vis_r[j] + 2.0 < vis_r[i]) {
                    keep[i] = 0;
                    break;
                }
            }
        }
        visible_after_occ = 0;
        for (int i = 0; i < vis_count; i++) visible_after_occ += keep[i];
    }

    return (uint32_t)(62000 + visible_after_occ * 450 + (g.frame_id % 1024));
}

static void* sensor_model_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "sensor_model");

    long period_us = 1000000L / (g.lidar_rate_hz > 0 ? g.lidar_rate_hz : 20);

    while (!g.should_stop) {
        usleep((unsigned long)period_us);
        if (g.should_stop) break;

        double noise_x = rand_uniform_signed(g.obs_noise_std_m);
        double noise_y = rand_uniform_signed(g.obs_noise_std_m);

        LidarFrame lidar = {
            .x = (float)(g.ego_x + noise_x),
            .y = (float)(g.ego_y + noise_y),
            .z = 0.0f,
            .intensity = 0.85f,
            .point_count = estimate_visible_point_count(),
            .frame_id = g.frame_id,
        };
        Message lmsg;
        msg_init_typed(&lmsg, "sensor/lidar", "sensor_model",
                       LIDARFRAME_TYPE_ID, LIDARFRAME_SCHEMA_VERSION,
                       &lidar, sizeof(lidar));
        transport_publish(g.transport, "sensor/lidar", lmsg.data, lmsg.data_size);

        if (g.frame_id % 2 == 0) {
            double noise_s = rand_uniform_signed(0.25);
            double noise_h = rand_uniform_signed(0.25);
            GpsData gps = {
                .latitude = 39.904 + g.ego_x * 0.00001,
                .longitude = 116.407 + g.ego_y * 0.00001,
                .speed_mps = (float)(g.ego_speed + noise_s),
                .heading_deg = (float)(g.ego_heading * 180.0 / M_PI + noise_h),
                .accuracy_m = 0.5f,
            };
            Message gmsg;
            msg_init_typed(&gmsg, "sensor/gps", "sensor_model",
                           GPSDATA_TYPE_ID, GPSDATA_SCHEMA_VERSION,
                           &gps, sizeof(gps));
            transport_publish(g.transport, "sensor/gps", gmsg.data, gmsg.data_size);
        }

        /* camera channel is reserved for future richer payloads; keep heartbeat-style frame. */
        if (g.frame_id % 3 == 0) {
            LidarFrame cam = lidar;
            cam.intensity = 0.5f;
            Message cmsg;
            msg_init_typed(&cmsg, "sensor/camera", "sensor_model",
                           LIDARFRAME_TYPE_ID, LIDARFRAME_SCHEMA_VERSION,
                           &cam, sizeof(cam));
            transport_publish(g.transport, "sensor/camera", cmsg.data, cmsg.data_size);
        }

        g.frame_id++;
    }

    LOG_INFO("sensor_model", "stopped (%u frames)", g.frame_id);
    return NULL;
}

static const char* s_inputs[] = {"vehicle/state", NULL};
static const char* s_outputs[] = {"sensor/lidar", "sensor/gps", "sensor/camera", NULL};

static NodePlugin s_plugin;

static int sensor_model_init(MessageBus* bus, Transport* transport,
                             DiscoveryManager* discovery, Scheduler* scheduler,
                             const char* params_json) {
    (void)bus;
    (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.should_stop = 0;

    g.lidar_rate_hz = 20;
    g.lidar_fov_deg = 120.0;
    g.lidar_max_range_m = 60.0;
    g.obs_noise_std_m = 0.08;
    g.enable_simple_occlusion = 1;

    if (params_json) {
        const char* p;
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

    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);

    discovery_advertise(discovery, "vehicle/state", 0x1C0E5A7Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "sensor/lidar", LIDARFRAME_TYPE_ID, CAP_PUBLISHER, 20.0);
    discovery_advertise(discovery, "sensor/gps", GPSDATA_TYPE_ID, CAP_PUBLISHER, 10.0);
    discovery_advertise(discovery, "sensor/camera", LIDARFRAME_TYPE_ID, CAP_PUBLISHER, 6.0);

    transport_advertise(transport, "sensor/lidar", LIDARFRAME_TYPE_ID);
    transport_advertise(transport, "sensor/gps", GPSDATA_TYPE_ID);
    transport_advertise(transport, "sensor/camera", LIDARFRAME_TYPE_ID);

    LOG_INFO("sensor_model", "initialized (LiDAR %dHz FOV=%.0fdeg range=%.0fm noise=%.2f occ=%d)",
             g.lidar_rate_hz, g.lidar_fov_deg, g.lidar_max_range_m,
             g.obs_noise_std_m, g.enable_simple_occlusion);
    return 0;
}

static int sensor_model_start(void) {
    g.running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, sensor_model_thread, NULL) != 0) return -1;
    LOG_INFO("sensor_model", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void sensor_model_stop(void) { g.should_stop = 1; }

static void sensor_model_cleanup(void) {
    if (g.running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.running = 0;
    }
    LOG_INFO("sensor_model", "cleanup done");
}

static int sensor_model_health(void) { return 0; }

static NodePlugin s_plugin = {
    .name = "sensor_model",
    .version = "1.0.0",
    .description = "Noisy/FOV/occlusion sensor model",
    .input_topics = s_inputs,
    .output_topics = s_outputs,
    .init = sensor_model_init,
    .start = sensor_model_start,
    .stop = sensor_model_stop,
    .cleanup = sensor_model_cleanup,
    .health = sensor_model_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
