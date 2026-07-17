/**
 * object_tracker_node.c — Multi-Object Kalman Tracker Node Plugin
 *
 * Subscribes to raw ObstacleList (binary), assigns persistent IDs across frames
 * via Kalman filter + Hungarian association, classifies static/dynamic targets,
 * and outputs TrackedObjectList (cJSON).
 *
 * Input:  perception/obstacles  — binary ObstacleList (type_id=0x308f5f71)
 * Output: perception/tracked_objects — cJSON TrackedObjectList
 *
 * Algorithm:
 *   1. ktracker_predict() — propagate all tracks
 *   2. Parse incoming cJSON → KTrackDetection[]
 *   3. ktracker_associate_and_update() — Hungarian matching
 *   4. Build cJSON output from CONFIRMED tracks
 */

#include "node_plugin.h"
#include "kalman_tracker.h"
#include "adas_msgs_gen.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

/* ── Constants ────────────────────────────────────────────────── */
#define TRACKER_TOPIC_IN       "perception/obstacles"
#define TRACKER_TOPIC_OUT      "perception/tracked_objects"
#define TRACKER_DEFAULT_HZ     10.0f
#define STATIC_SPEED_THRESHOLD 0.5f   /**< m/s below which target is candidate-static */
#define STATIC_FRAMES_MIN      20u    /**< consecutive low-speed frames to declare static */

/* ── Per-track static classification state ───────────────────── */
typedef struct {
    int  track_id;     /**< KalmanTrack::id this entry belongs to; -1 = unused */
    int  counter;      /**< consecutive frames with |v| < STATIC_SPEED_THRESHOLD */
} TrackStaticEntry;

/* ── Module state ─────────────────────────────────────────────── */
static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 tracker_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase    taskbase;

    KalmanTracker  kt;
    float          rate_hz;
    uint32_t       frame_id;

    /* ── Incoming detection buffer (written by callback, read by thread) ── */
    double obs_x[KTRACKER_MAX_DETS];
    double obs_y[KTRACKER_MAX_DETS];
    double obs_vx[KTRACKER_MAX_DETS];
    double obs_vy[KTRACKER_MAX_DETS];
    double obs_width[KTRACKER_MAX_DETS];
    double obs_length[KTRACKER_MAX_DETS];
    double obs_confidence[KTRACKER_MAX_DETS];
    int    obs_cls[KTRACKER_MAX_DETS];
    int    obs_count;
    volatile int has_new_data;

    pthread_mutex_t mutex;

    /* ── Static classification lookup (track-id-keyed) ── */
    TrackStaticEntry static_cache[KTRACKER_MAX_TRACKS];
    int              n_static_cache;
} g;

/* ── Subscription callback: perception/obstacles (binary ObstacleList) ── */
static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    ObstacleList obs_list;
    if (ObstacleList_deserialize(&obs_list, (const uint8_t*)msg->data, msg->data_size) != 0) {
        LOG_WARN("tracker", "ObstacleList deserialize failed (size=%u)", msg->data_size);
        return;
    }

    pthread_mutex_lock(&g.mutex);

    int n = (int)obs_list.count;
    if (n > KTRACKER_MAX_DETS) n = KTRACKER_MAX_DETS;
    g.obs_count = n;

    for (int i = 0; i < n; i++) {
        const Obstacle* o = &obs_list.obstacles[i];
        g.obs_x[i]         = (double)o->x;
        g.obs_y[i]         = (double)o->y;
        g.obs_vx[i]        = (double)o->vx;
        g.obs_vy[i]        = (double)o->vy;
        g.obs_width[i]     = (double)o->width;
        g.obs_length[i]    = (double)o->length;
        g.obs_cls[i]       = (int)o->type;
        g.obs_confidence[i]= (double)o->confidence;
    }

    g.has_new_data = 1;
    pthread_mutex_unlock(&g.mutex);
}

/* ── Helper: map numerical class → string label ──────────────── */
static const char* type_to_string(int cls) {
    switch (cls) {
        case 1: return "VEHICLE";
        case 2: return "PEDESTRIAN";
        case 3: return "CYCLIST";
        default: return "UNKNOWN";
    }
}

/* ── Static cache helpers ────────────────────────────────────── */
static int static_cache_find_or_create(int track_id) {
    for (int i = 0; i < g.n_static_cache; i++) {
        if (g.static_cache[i].track_id == track_id)
            return i;
    }
    /* Create new entry */
    if (g.n_static_cache < KTRACKER_MAX_TRACKS) {
        int idx = g.n_static_cache++;
        g.static_cache[idx].track_id = track_id;
        g.static_cache[idx].counter  = 0;
        return idx;
    }
    return -1; /* should never happen */
}

static void static_cache_prune(void) {
    int write = 0;
    for (int i = 0; i < g.n_static_cache; i++) {
        int found = 0;
        for (int j = 0; j < g.kt.n_tracks; j++) {
            if (g.kt.tracks[j].id == g.static_cache[i].track_id) {
                found = 1;
                break;
            }
        }
        if (found) {
            if (write != i)
                g.static_cache[write] = g.static_cache[i];
            write++;
        }
    }
    g.n_static_cache = write;
}

/* ── Managed-mode main processing loop ──────────────────────── */
static int tracker_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "obj_tracker");

    const long period_us = (long)(1000000.0 / g.rate_hz);

    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

        /* ── Snapshot incoming detections under lock ── */
        pthread_mutex_lock(&g.mutex);
        const int   has_data = g.has_new_data;
        const int   n_dets   = g.obs_count;
        KTrackDetection dets[KTRACKER_MAX_DETS];
        if (has_data && n_dets > 0) {
            for (int i = 0; i < n_dets; i++) {
                dets[i].x          = (float)g.obs_x[i];
                dets[i].y          = (float)g.obs_y[i];
                dets[i].vx         = (float)g.obs_vx[i];
                dets[i].vy         = (float)g.obs_vy[i];
                dets[i].width      = (float)g.obs_width[i];
                dets[i].length     = (float)g.obs_length[i];
                dets[i].cls        = g.obs_cls[i];
                dets[i].confidence = (float)g.obs_confidence[i];
            }
        }
        g.has_new_data = 0;
        pthread_mutex_unlock(&g.mutex);

        /* ── Step a: Predict ── */
        ktracker_predict(&g.kt);

        /* ── Step b+c: Associate & update ── */
        if (has_data) {
            ktracker_associate_and_update(&g.kt, dets, n_dets);
        }

        /* ── Step d: Build cJSON output ── */
        cJSON* root      = cJSON_CreateObject();
        cJSON* j_objects = cJSON_AddArrayToObject(root, "objects");

        cJSON_AddNumberToObject(root, "frame_id",     (double)g.frame_id);
        cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());

        for (int i = 0; i < g.kt.n_tracks; i++) {
            const KTrack* trk = &g.kt.tracks[i];
            if (trk->state != TRACK_CONFIRMED) continue;

            const float vx    = (float)trk->x[2];
            const float vy    = (float)trk->x[3];
            const float speed = sqrtf(vx * vx + vy * vy);

            /* ── Static / dynamic classification ── */
            int sidx = static_cache_find_or_create(trk->id);
            if (sidx >= 0) {
                if (speed < STATIC_SPEED_THRESHOLD) {
                    g.static_cache[sidx].counter++;
                } else {
                    g.static_cache[sidx].counter = 0;
                }
            }
            const int is_static = (sidx >= 0 &&
                (unsigned int)g.static_cache[sidx].counter >= STATIC_FRAMES_MIN) ? 1 : 0;

            cJSON* obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(obj, "id",         (double)trk->id);
            cJSON_AddNumberToObject(obj, "track_age",  (double)trk->age);
            cJSON_AddStringToObject(obj, "type",       type_to_string(trk->cls));
            cJSON_AddNumberToObject(obj, "x",          trk->x[0]);
            cJSON_AddNumberToObject(obj, "y",          trk->x[1]);
            cJSON_AddNumberToObject(obj, "z",          0.0);
            cJSON_AddNumberToObject(obj, "vx",         (double)vx);
            cJSON_AddNumberToObject(obj, "vy",         (double)vy);
            cJSON_AddNumberToObject(obj, "vz",         0.0);
            cJSON_AddNumberToObject(obj, "ax",         0.0);
            cJSON_AddNumberToObject(obj, "ay",         0.0);
            cJSON_AddNumberToObject(obj, "heading",    0.0);
            cJSON_AddNumberToObject(obj, "width",      (double)trk->width);
            cJSON_AddNumberToObject(obj, "length",     (double)trk->length);
            cJSON_AddNumberToObject(obj, "height",     1.5);
            cJSON_AddNumberToObject(obj, "confidence", (double)trk->confidence);
            cJSON_AddNumberToObject(obj, "lane_id",    -1);
            cJSON_AddBoolToObject(obj,  "is_static",   is_static);
            cJSON_AddBoolToObject(obj,  "is_on_road",  1);

            cJSON_AddItemToArray(j_objects, obj);
        }

        /* ── Prune stale static cache entries ── */
        static_cache_prune();

        /* ── Publish ── */
        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            transport_publish(g.transport, TRACKER_TOPIC_OUT,
                              json_str, (uint32_t)strlen(json_str) + 1);
            free(json_str);
        }
        cJSON_Delete(root);

        g.frame_id++;
    }

    LOG_INFO("tracker", "stopped (%u frames)", g.frame_id);
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface tracker_vtable = {
    .execute = tracker_execute,
};

/* ── NodePlugin lifecycle ────────────────────────────────────── */
static const char* s_inputs[]  = { TRACKER_TOPIC_IN, NULL };
static const char* s_outputs[] = { TRACKER_TOPIC_OUT, NULL };

static NodePlugin s_plugin;

static int tracker_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;

    g.rate_hz = TRACKER_DEFAULT_HZ;

    /* ── Parse parameters ── */
    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "rate_hz")) && cJSON_IsNumber(j))
                g.rate_hz = (float)j->valuedouble;
            cJSON_Delete(p);
        }
    }

    /* ── Initialize Kalman tracker ── */
    ktracker_init(&g.kt, (double)(1.0 / g.rate_hz));

    /* ── Subscriptions ── */
    transport_subscribe(transport, TRACKER_TOPIC_IN, on_obstacles, NULL);
    transport_advertise(transport, TRACKER_TOPIC_OUT, 0x0B5A010Eu);

    /* ── Discovery ── */
    discovery_advertise(discovery, TRACKER_TOPIC_IN,  0x0B5A010Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TRACKER_TOPIC_OUT, 0xDA7A7A11u, CAP_PUBLISHER, g.rate_hz);

    /* ── Mutex ── */
    pthread_mutex_init(&g.mutex, NULL);

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "object_tracker");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = (double)g.rate_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &tracker_vtable, &cfg) != 0) {
        LOG_WARN("tracker", "task_base_init failed");
        return -1;
    }

    LOG_INFO("tracker", "initialized (%.1f Hz, dt=%.3f s)", g.rate_hz, g.kt.dt);
    return 0;
}

static int tracker_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * tracker_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("tracker", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("tracker", "started (managed)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void tracker_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（tracker_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void tracker_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源。节点互斥锁 g.mutex 与 taskbase 无关，
     * 单独销毁。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    pthread_mutex_destroy(&g.mutex);
    LOG_INFO("tracker", "cleanup done");
}

static int tracker_health(void) {
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "object_tracker",
    .version       = "1.0.0",
    .description   = "Multi-object Kalman tracker with static/dynamic classification",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = tracker_init,
    .start         = tracker_start,
    .stop          = tracker_stop,
    .cleanup       = tracker_cleanup,
    .health        = tracker_health,
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子，指向嵌入的 TaskBase */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
