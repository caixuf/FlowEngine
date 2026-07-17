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
#include "nmea_parser.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <cjson/cJSON.h>

static struct {
    Transport* transport;
    DiscoveryManager* discovery;
    Scheduler* scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 sensor_model_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase taskbase;

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

    /* 真实 GPS 传感器格式接入：可选 NMEA 0183 日志回放 */
    char     gps_nmea_file[256];
    GpsData* nmea_frames;   /* 从 NMEA 日志解析出的 GPS 帧序列 */
    int      nmea_count;    /* 帧数量 */
    int      nmea_idx;      /* 回放游标 */
} g;

#define SENSOR_NMEA_MAX_FRAMES 8192

/* 从 NMEA 0183 日志文件加载 GPS 帧序列。返回加载帧数（失败返回 0）。 */
static int load_nmea_gps_log(const char* path, GpsData** out_frames) {
    FILE* f = fopen(path, "r");
    if (!f) {
        LOG_WARN("sensor_model", "cannot open NMEA file: %s", path);
        return 0;
    }
    GpsData* frames = (GpsData*)calloc(SENSOR_NMEA_MAX_FRAMES, sizeof(GpsData));
    if (!frames) { fclose(f); return 0; }

    NmeaParser parser;
    nmea_parser_init(&parser);

    int count = 0;
    char line[256];
    while (count < SENSOR_NMEA_MAX_FRAMES && fgets(line, sizeof(line), f)) {
        GpsData gps;
        if (nmea_parse_line(&parser, line, &gps) == NMEA_OK) {
            frames[count++] = gps;
        }
    }
    if (count >= SENSOR_NMEA_MAX_FRAMES && !feof(f)) {
        LOG_WARN("sensor_model",
                 "NMEA file truncated at %d frames (SENSOR_NMEA_MAX_FRAMES): %s",
                 count, path);
    }
    fclose(f);

    if (count == 0) {
        LOG_WARN("sensor_model", "NMEA file had no valid GPS fixes: %s", path);
        free(frames);
        return 0;
    }
    *out_frames = frames;
    return count;
}

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

    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "hdg")) && cJSON_IsNumber(j))
        g.ego_heading = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "spd")) && cJSON_IsNumber(j))
        g.ego_speed = j->valuedouble;

    int n = 0;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "n_obs")) && cJSON_IsNumber(j))
        n = (int)j->valuedouble;
    if (n < 1 || n > 16) n = 3;
    g.n_obs = n;

    for (int i = 0; i < n; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ox%d", i);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, key)) && cJSON_IsNumber(j))
            g.obs_x[i] = j->valuedouble;
        snprintf(key, sizeof(key), "oy%d", i);
        if ((j = cJSON_GetObjectItemCaseSensitive(root, key)) && cJSON_IsNumber(j))
            g.obs_y[i] = j->valuedouble;
    }
    cJSON_Delete(root);
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

static int sensor_model_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "sensor_model");

    long period_us = 1000000L / (g.lidar_rate_hz > 0 ? g.lidar_rate_hz : 20);

    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

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
            GpsData gps;
            if (g.nmea_count > 0) {
                /* 真实 NMEA 0183 日志回放：循环播放解析出的 GPS 帧 */
                gps = g.nmea_frames[g.nmea_idx];
                g.nmea_idx = (g.nmea_idx + 1) % g.nmea_count;
            } else {
                double noise_s = rand_uniform_signed(0.25);
                double noise_h = rand_uniform_signed(0.25);
                gps = (GpsData){
                    .latitude = 39.904 + g.ego_x * 0.00001,
                    .longitude = 116.407 + g.ego_y * 0.00001,
                    .speed_mps = (float)(g.ego_speed + noise_s),
                    .heading_deg = (float)(g.ego_heading * 180.0 / M_PI + noise_h),
                    .accuracy_m = 0.5f,
                };
            }
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
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface sensor_model_vtable = {
    .execute = sensor_model_execute,
};

static const char* s_inputs[] = {"vehicle/state", NULL};
static const char* s_outputs[] = {"sensor/lidar", "sensor/gps", "sensor/camera", NULL};

static NodePlugin s_plugin;

static int sensor_model_init(MessageBus* bus, Transport* transport,
                             DiscoveryManager* discovery, Scheduler* scheduler,
                             const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;

    g.lidar_rate_hz = 20;
    g.lidar_fov_deg = 120.0;
    g.lidar_max_range_m = 60.0;
    g.obs_noise_std_m = 0.08;
    g.enable_simple_occlusion = 1;

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lidar_rate_hz")) && cJSON_IsNumber(j))
                g.lidar_rate_hz = (int)j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lidar_fov_deg")) && cJSON_IsNumber(j))
                g.lidar_fov_deg = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "lidar_max_range_m")) && cJSON_IsNumber(j))
                g.lidar_max_range_m = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "obs_noise_std_m")) && cJSON_IsNumber(j))
                g.obs_noise_std_m = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "enable_simple_occlusion")) && cJSON_IsNumber(j))
                g.enable_simple_occlusion = (int)j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "gps_nmea_file")) && cJSON_IsString(j))
                strncpy(g.gps_nmea_file, j->valuestring, sizeof(g.gps_nmea_file) - 1);
            cJSON_Delete(p);
        }
    }

    /* 可选：接入真实 GPS 传感器格式 (NMEA 0183 日志回放) */
    if (g.gps_nmea_file[0]) {
        g.nmea_count = load_nmea_gps_log(g.gps_nmea_file, &g.nmea_frames);
        g.nmea_idx = 0;
        if (g.nmea_count > 0) {
            LOG_INFO("sensor_model", "GPS source: NMEA 0183 replay '%s' (%d fixes)",
                     g.gps_nmea_file, g.nmea_count);
        }
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

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "sensor_model");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = (double)g.lidar_rate_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &sensor_model_vtable, &cfg) != 0) {
        LOG_WARN("sensor_model", "task_base_init failed");
        return -1;
    }

    LOG_INFO("sensor_model", "initialized (LiDAR %dHz FOV=%.0fdeg range=%.0fm noise=%.2f occ=%d)",
             g.lidar_rate_hz, g.lidar_fov_deg, g.lidar_max_range_m,
             g.obs_noise_std_m, g.enable_simple_occlusion);
    return 0;
}

static int sensor_model_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * sensor_model_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("sensor_model", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("sensor_model", "started (managed)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void sensor_model_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（sensor_model_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void sensor_model_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源（互斥锁等）。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    if (g.nmea_frames) {
        free(g.nmea_frames);
        g.nmea_frames = NULL;
        g.nmea_count = 0;
    }
    LOG_INFO("sensor_model", "cleanup done");
}

static int sensor_model_health(void) { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
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
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子，指向嵌入的 TaskBase */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
