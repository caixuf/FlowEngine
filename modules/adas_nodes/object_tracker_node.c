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
#define TRACKER_TOPIC_EGO      "vehicle/state"
#define TRACKER_TOPIC_OUT      "perception/tracked_objects"
#define TRACKER_DEFAULT_HZ     20.0f
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

    /* ── Incoming detection buffer (body-frame raw, written by callback, read by thread) ── */
    double obs_bx[KTRACKER_MAX_DETS];      /* body-frame x (forward) */
    double obs_by[KTRACKER_MAX_DETS];      /* body-frame y (left) */
    double obs_bvx[KTRACKER_MAX_DETS];     /* body-frame vx (absolute, projected) */
    double obs_bvy[KTRACKER_MAX_DETS];     /* body-frame vy (absolute, projected) */
    double obs_width[KTRACKER_MAX_DETS];
    double obs_length[KTRACKER_MAX_DETS];
    double obs_confidence[KTRACKER_MAX_DETS];
    int    obs_cls[KTRACKER_MAX_DETS];
    int    obs_lane_id[KTRACKER_MAX_DETS];
    int    obs_count;
    volatile int has_new_data;

    pthread_mutex_t mutex;

    /* ── Static classification lookup (track-id-keyed) ── */
    TrackStaticEntry static_cache[KTRACKER_MAX_TRACKS];
    int              n_static_cache;

    /* ── Ego pose (world frame, for coordinate transform) ── */
    double ego_x, ego_y, ego_heading, ego_v;
    volatile int has_ego;
} g;

/* ── Subscription callback: vehicle/state (ego pose in world frame, JSON) ── */
static void on_ego_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))   g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))   g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "hdg")) && cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "spd")) && cJSON_IsNumber(j)) g.ego_v = j->valuedouble;
    if (!g.has_ego) {
        LOG_INFO("tracker", "Got ego from vehicle/state: x=%.1f y=%.1f hdg=%.2f v=%.1f",
                 g.ego_x, g.ego_y, g.ego_heading, g.ego_v);
    }
    g.has_ego = 1;
    cJSON_Delete(root);
}

/* ── Subscription callback: perception/obstacles (binary ObstacleList, body frame) ──
 * Stores raw body-frame detections. Coordinate transform to world frame is done
 * in the main loop using the latest ego pose, ensuring temporal consistency. */
static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;

    ObstacleList obs_list;
    int deser_rc = ObstacleList_deserialize(&obs_list, (const uint8_t*)msg->data, msg->data_size);
    if (deser_rc != 0) {
        LOG_WARN("tracker", "ObstacleList deserialize failed rc=%d size=%u", deser_rc, msg->data_size);
        return;
    }

    pthread_mutex_lock(&g.mutex);

    int n = (int)obs_list.count;
    if (n > KTRACKER_MAX_DETS) n = KTRACKER_MAX_DETS;
    g.obs_count = n;

    for (int i = 0; i < n; i++) {
        const Obstacle* o = &obs_list.obstacles[i];
        g.obs_bx[i]        = (double)o->x;
        g.obs_by[i]        = (double)o->y;
        g.obs_bvx[i]       = (double)o->vx;
        g.obs_bvy[i]       = (double)o->vy;
        g.obs_width[i]     = (double)o->width;
        g.obs_length[i]    = (double)o->length;
        g.obs_cls[i]       = (int)o->type;
        g.obs_confidence[i]= (double)o->confidence;
        g.obs_lane_id[i]   = (int)o->lane_id;
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

        /* ── Snapshot incoming detections + ego pose under lock ── */
        pthread_mutex_lock(&g.mutex);
        const int   has_data = g.has_new_data;
        const int   n_dets   = g.obs_count;
        /* Snapshot ego pose at the same moment as detections */
        const double ego_x_snap  = g.ego_x;
        const double ego_y_snap  = g.ego_y;
        const double ego_h_snap  = g.ego_heading;
        KTrackDetection dets[KTRACKER_MAX_DETS];
        if (has_data && n_dets > 0 && g.has_ego) {
            const double ch = cos(ego_h_snap);
            const double sh = sin(ego_h_snap);
            for (int i = 0; i < n_dets; i++) {
                /* body-frame (x forward, y left) → world frame */
                const double bx  = g.obs_bx[i];
                const double by  = g.obs_by[i];
                const double bvx = g.obs_bvx[i];
                const double bvy = g.obs_bvy[i];
                dets[i].x          = (float)(ego_x_snap + bx * ch - by * sh);
                dets[i].y          = (float)(ego_y_snap + bx * sh + by * ch);
                dets[i].vx         = (float)(bvx * ch - bvy * sh);
                dets[i].vy         = (float)(bvx * sh + bvy * ch);
                dets[i].width      = (float)g.obs_width[i];
                dets[i].length     = (float)g.obs_length[i];
                dets[i].cls        = g.obs_cls[i];
                dets[i].confidence = (float)g.obs_confidence[i];
            }
        }
        g.has_new_data = 0;
        pthread_mutex_unlock(&g.mutex);

        /* Skip if no ego pose yet */
        const int has_ego = g.has_ego;

        /* ── Step a: Predict ── */
        ktracker_predict(&g.kt);

        /* ── Step b+c: Associate & update (only if we have ego + detections) ── */
        if (has_data && has_ego) {
            ktracker_associate_and_update(&g.kt, dets, n_dets);
        }

        /* ── 日志：每10帧打一次track状态 ── */
        if (g.frame_id % 10 == 0) {
            int n_total = g.kt.n_tracks;
            int n_confirmed = 0, n_tentative = 0, n_coasting = 0;
            for (int i = 0; i < n_total; i++) {
                switch (g.kt.tracks[i].state) {
                    case TRACK_CONFIRMED: n_confirmed++; break;
                    case TRACK_TENTATIVE: n_tentative++; break;
                    case TRACK_COASTING:  n_coasting++;  break;
                    default: break;
                }
            }
            LOG_INFO("tracker", "frame=%u has_data=%d n_dets=%d tracks(total=%d conf=%d tent=%d coast=%d)",
                     g.frame_id, has_data, n_dets, n_total, n_confirmed, n_tentative, n_coasting);

            /* 有障碍物但没确认track时，额外打印hits_streak信息 */
            if (has_data && n_dets > 0 && n_confirmed == 0) {
                for (int i = 0; i < n_total && i < 5; i++) {
                    const KTrack* t = &g.kt.tracks[i];
                    LOG_INFO("tracker", "  track[%d]: id=%d state=%d hits=%d streak=%d misses=%d age=%d "
                             "pos(%.1f,%.1f) vel(%.1f,%.1f)",
                             i, t->id, t->state, t->hits, t->hits_streak, t->misses, t->age,
                             t->x[0], t->x[1], t->x[2], t->x[3]);
                }
            }
        }

        /* ── Snapshot ego pose for world→body transform (no lock needed, double is atomic enough for tracking) ── */
        const double ego_h = g.ego_heading;
        const double ego_xx = g.ego_x;
        const double ego_yy = g.ego_y;
        const double ch_out = cos(ego_h);
        const double sh_out = sin(ego_h);

        /* ── Step d: Build cJSON output ── */
        cJSON* root      = cJSON_CreateObject();
        cJSON* j_objects = cJSON_AddArrayToObject(root, "objects");

        cJSON_AddNumberToObject(root, "frame_id",     (double)g.frame_id);
        cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());

        for (int i = 0; i < g.kt.n_tracks; i++) {
            const KTrack* trk = &g.kt.tracks[i];
            if (trk->state != TRACK_CONFIRMED) continue;

            /* world-frame absolute state from Kalman filter */
            const double wx = trk->x[0];
            const double wy = trk->x[1];
            const double wvx = trk->x[2];
            const double wvy = trk->x[3];
            const float speed = sqrtf(wvx * wvx + wvy * wvy);

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

            /* world → body (Ego-centered, x-forward, y-left) */
            const double dx = wx - ego_xx;
            const double dy = wy - ego_yy;
            const double bx = dx * ch_out + dy * sh_out;
            const double by = -dx * sh_out + dy * ch_out;
            /* absolute velocity (inertial) projected onto body axes */
            const double bvx = wvx * ch_out + wvy * sh_out;
            const double bvy = -wvx * sh_out + wvy * ch_out;

            cJSON* obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(obj, "id",         (double)trk->id);
            cJSON_AddNumberToObject(obj, "track_age",  (double)trk->age);
            cJSON_AddStringToObject(obj, "type",       type_to_string(trk->cls));
            cJSON_AddNumberToObject(obj, "x",          bx);
            cJSON_AddNumberToObject(obj, "y",          by);
            cJSON_AddNumberToObject(obj, "z",          0.0);
            cJSON_AddNumberToObject(obj, "vx",         bvx);
            cJSON_AddNumberToObject(obj, "vy",         bvy);
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

        /* ── 日志：输出障碍物计数 ── */
        {
            int out_count = cJSON_GetArraySize(j_objects);
            if (out_count > 0 || g.frame_id % 10 == 0) {
                LOG_INFO("tracker", "publishing %d tracked objects (frame=%u)", out_count, g.frame_id);
            }
        }

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
static const char* s_inputs[]  = { TRACKER_TOPIC_IN, TRACKER_TOPIC_EGO, NULL };
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
    transport_subscribe(transport, TRACKER_TOPIC_EGO, on_ego_state, NULL);
    transport_advertise(transport, TRACKER_TOPIC_OUT, 0xDA7A7A11u);

    /* ── Discovery ── */
    discovery_advertise(discovery, TRACKER_TOPIC_IN,  0x0B5A010Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TRACKER_TOPIC_EGO, 0u, CAP_SUBSCRIBER, 0);
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
