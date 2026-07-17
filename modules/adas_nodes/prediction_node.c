/**
 * prediction_node.c — Multi-Trajectory Motion Prediction Node Plugin
 *
 * For each tracked object generates 1-3 possible future trajectories with
 * mode probabilities. Sandbox implementation (no deep learning).
 *
 * Input:  perception/tracked_objects  — cJSON TrackedObjectList
 *         fusion/localization          — cJSON ego state (latched)
 * Output: prediction/tracks           — cJSON PredictionList
 *
 * Trajectory modes:
 *   1. Constant Velocity (0.7 prob) — continue at current velocity
 *   2. Lane Keeping      (0.2 prob) — maintain x velocity, y stays constant
 *   3. Lane Change        (0.1 prob) — move laterally up to one lane width (3.5 m)
 */

#include "node_plugin.h"
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
#define PRED_TOPIC_IN_TRACKS  "perception/tracked_objects"
#define PRED_TOPIC_IN_LOC     "fusion/localization"
#define PRED_TOPIC_OUT        "prediction/tracks"
#define PRED_DEFAULT_HZ       10.0f

#define PRED_N_WAYPOINTS       10u    /**< waypoints per trajectory */
#define PRED_WAYPOINT_DT_S     0.5f   /**< spacing between waypoints (s) */
#define PRED_HORIZON_S         5.0f   /**< total prediction horizon (s) */

#define PRED_CV_PROB           0.7f   /**< constant velocity probability */
#define PRED_LK_PROB           0.2f   /**< lane keeping probability */
#define PRED_LC_PROB           0.1f   /**< lane change probability */

#define PRED_BASE_CONFIDENCE   0.7f   /**< base confidence for prediction */
#define PRED_CONF_AGE_MAX      50.0f  /**< track age at which confidence saturates */

#define PRED_LANE_WIDTH_M      3.5f   /**< maximum lateral displacement for lane change (m) */

/* ── Module state ─────────────────────────────────────────────── */
static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    pthread_t    thread;
    volatile int running;
    volatile int should_stop;

    float     rate_hz;
    uint32_t  frame_id;

    /* ── Latch: tracked objects (written by callback, read by thread) ── */
    char*         tracks_json;   /**< heap-copied input JSON string */
    volatile int  has_tracks;
    pthread_mutex_t mutex;

    /* ── Latch: ego state (latched but not used by sandbox algorithm) ── */
    double ego_x, ego_y, ego_v, ego_heading;
    volatile int has_ego;
} g;

/* ── Callback: perception/tracked_objects ────────────────────── */
static void on_tracked_objects(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* Deep-copy the JSON string for thread-safe access */
    char* copy = strdup((const char*)msg->data);
    if (!copy) return;

    pthread_mutex_lock(&g.mutex);
    if (g.tracks_json) free(g.tracks_json);
    g.tracks_json = copy;
    g.has_tracks  = 1;
    pthread_mutex_unlock(&g.mutex);
}

/* ── Callback: fusion/localization ───────────────────────────── */
static void on_localization(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    cJSON* j;
    double x = 0, y = 0, v = 0, heading = 0;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "v")) && cJSON_IsNumber(j))
        v = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "heading")) && cJSON_IsNumber(j))
        heading = j->valuedouble;
    cJSON_Delete(root);

    pthread_mutex_lock(&g.mutex);
    g.ego_x       = x;
    g.ego_y       = y;
    g.ego_v       = v;
    g.ego_heading = heading;
    g.has_ego     = 1;
    pthread_mutex_unlock(&g.mutex);
}

/* ── Helper: signum ──────────────────────────────────────────── */
static inline float signf(float v) {
    return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f);
}

/* ── Helper: square ──────────────────────────────────────────── */
static inline float sqrf(float v) { return v * v; }

/* ── Build a waypoints array (cJSON) for a single trajectory ───
 *
 *  @param x0, y0   Starting position
 *  @param vx, vy   Velocity components (m/s)
 *  @param mode     0=CV, 1=Lane Keeping, 2=Lane Change
 *  @return cJSON array of [x, y, v] triples; caller owns the reference.
 */
static cJSON* build_waypoints(float x0, float y0, float vx, float vy, int mode) {
    cJSON* wps = cJSON_CreateArray();
    if (!wps) return NULL;

    const float dt     = PRED_WAYPOINT_DT_S;
    const float v_mag  = sqrtf(sqrf(vx) + sqrf(vy));
    const float sy     = signf(vy);
    const float abs_vy = fabsf(vy);

    for (unsigned int step = 0; step < PRED_N_WAYPOINTS; step++) {
        const float t  = (float)(step + 1) * dt;  /* t ∈ [0.5, 1.0, …, 5.0] */
        float xt, yt, vt;

        switch (mode) {
            case 0: /* Constant Velocity */
                xt = x0 + vx * t;
                yt = y0 + vy * t;
                vt = v_mag;
                break;

            case 1: /* Lane Keeping — y stays constant */
                xt = x0 + vx * t;
                yt = y0;
                vt = fabsf(vx);
                break;

            case 2: /* Lane Change — lateral displacement capped at one lane width */
                xt = x0 + vx * t;
                yt = y0 + sy * fminf(abs_vy * t, PRED_LANE_WIDTH_M);
                /* Effective speed: once vy saturates at lane width, use only vx */
                if (abs_vy * t < PRED_LANE_WIDTH_M && abs_vy > 1e-6f) {
                    vt = sqrtf(sqrf(vx) + sqrf(vy));
                } else {
                    vt = fabsf(vx);
                }
                break;

            default:
                xt = x0; yt = y0; vt = 0;
                break;
        }

        cJSON* wp = cJSON_CreateArray();
        cJSON_AddItemToArray(wp, cJSON_CreateNumber((double)xt));
        cJSON_AddItemToArray(wp, cJSON_CreateNumber((double)yt));
        cJSON_AddItemToArray(wp, cJSON_CreateNumber((double)vt));
        cJSON_AddItemToArray(wps, wp);
    }

    return wps;
}

/* ── Main processing thread ──────────────────────────────────── */
static void* prediction_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "prediction");

    const long period_us = (long)(1000000.0 / g.rate_hz);

    while (!g.should_stop) {
        usleep((unsigned long)period_us);
        if (g.should_stop) break;

        /* ── Snapshot the latest tracks JSON under lock ── */
        pthread_mutex_lock(&g.mutex);
        const int has_data = g.has_tracks;
        char* tracks_copy = NULL;
        if (has_data && g.tracks_json) {
            tracks_copy = strdup(g.tracks_json);
        }
        pthread_mutex_unlock(&g.mutex);

        if (!has_data || !tracks_copy) {
            free(tracks_copy);
            continue;
        }

        /* ── Parse tracked objects ── */
        cJSON* root = cJSON_Parse(tracks_copy);
        free(tracks_copy);
        if (!root) continue;

        cJSON* j_objects = cJSON_GetObjectItemCaseSensitive(root, "objects");
        if (!cJSON_IsArray(j_objects)) {
            cJSON_Delete(root);
            continue;
        }

        const int n_objects = cJSON_GetArraySize(j_objects);

        /* ── Build output ── */
        cJSON* out_root   = cJSON_CreateObject();
        cJSON* j_tracks   = cJSON_AddArrayToObject(out_root, "tracks");

        cJSON_AddNumberToObject(out_root, "frame_id",     (double)g.frame_id);
        cJSON_AddNumberToObject(out_root, "timestamp_us", (double)clock_now_us());

        for (int i = 0; i < n_objects; i++) {
            cJSON* obj = cJSON_GetArrayItem(j_objects, i);
            if (!obj) continue;

            /* Parse required fields */
            cJSON* j;

            /* Skip static objects */
            j = cJSON_GetObjectItemCaseSensitive(obj, "is_static");
            if (cJSON_IsBool(j) && cJSON_IsTrue(j)) continue;

            double obj_x = 0, obj_y = 0, obj_vx = 0, obj_vy = 0;
            double track_age = 0, obj_confidence = 0;
            int    object_id = 0;

            j = cJSON_GetObjectItemCaseSensitive(obj, "id");
            if (cJSON_IsNumber(j)) object_id = (int)j->valuedouble;

            j = cJSON_GetObjectItemCaseSensitive(obj, "x");
            if (cJSON_IsNumber(j)) obj_x = j->valuedouble;

            j = cJSON_GetObjectItemCaseSensitive(obj, "y");
            if (cJSON_IsNumber(j)) obj_y = j->valuedouble;

            j = cJSON_GetObjectItemCaseSensitive(obj, "vx");
            if (cJSON_IsNumber(j)) obj_vx = j->valuedouble;

            j = cJSON_GetObjectItemCaseSensitive(obj, "vy");
            if (cJSON_IsNumber(j)) obj_vy = j->valuedouble;

            j = cJSON_GetObjectItemCaseSensitive(obj, "track_age");
            if (cJSON_IsNumber(j)) track_age = j->valuedouble;

            j = cJSON_GetObjectItemCaseSensitive(obj, "confidence");
            if (cJSON_IsNumber(j)) obj_confidence = j->valuedouble;

            /* Compute prediction confidence:
             * base * track_age_factor (saturates at CONF_AGE_MAX) */
            const float age_factor = (float)fmin(track_age / PRED_CONF_AGE_MAX, 1.0);
            const float pred_conf  = PRED_BASE_CONFIDENCE * age_factor;

            /* Skip very low-confidence predictions */
            if (pred_conf < 0.1f) continue;

            const float fx0 = (float)obj_x;
            const float fy0 = (float)obj_y;
            const float fvx = (float)obj_vx;
            const float fvy = (float)obj_vy;

            /* ── Build prediction track entry ── */
            cJSON* track_entry = cJSON_CreateObject();
            cJSON_AddNumberToObject(track_entry, "object_id",   (double)object_id);
            cJSON_AddNumberToObject(track_entry, "confidence",  (double)pred_conf);
            cJSON_AddNumberToObject(track_entry, "horizon_s",   (double)PRED_HORIZON_S);

            cJSON* j_trajs = cJSON_AddArrayToObject(track_entry, "trajectories");

            /* Mode 1: Constant Velocity (prob = 0.7) */
            {
                cJSON* traj = cJSON_CreateObject();
                cJSON_AddNumberToObject(traj, "prob", (double)PRED_CV_PROB);
                cJSON* wps = build_waypoints(fx0, fy0, fvx, fvy, 0);
                if (wps) cJSON_AddItemToObject(traj, "waypoints", wps);
                cJSON_AddItemToArray(j_trajs, traj);
            }

            /* Mode 2: Lane Keeping (prob = 0.2) */
            {
                cJSON* traj = cJSON_CreateObject();
                cJSON_AddNumberToObject(traj, "prob", (double)PRED_LK_PROB);
                cJSON* wps = build_waypoints(fx0, fy0, fvx, fvy, 1);
                if (wps) cJSON_AddItemToObject(traj, "waypoints", wps);
                cJSON_AddItemToArray(j_trajs, traj);
            }

            /* Mode 3: Lane Change (prob = 0.1) */
            {
                cJSON* traj = cJSON_CreateObject();
                cJSON_AddNumberToObject(traj, "prob", (double)PRED_LC_PROB);
                cJSON* wps = build_waypoints(fx0, fy0, fvx, fvy, 2);
                if (wps) cJSON_AddItemToObject(traj, "waypoints", wps);
                cJSON_AddItemToArray(j_trajs, traj);
            }

            cJSON_AddItemToArray(j_tracks, track_entry);
        }

        /* ── Publish ── */
        char* json_str = cJSON_PrintUnformatted(out_root);
        if (json_str) {
            transport_publish(g.transport, PRED_TOPIC_OUT,
                              json_str, (uint32_t)strlen(json_str) + 1);
            free(json_str);
        }

        cJSON_Delete(out_root);
        cJSON_Delete(root);

        g.frame_id++;
    }

    LOG_INFO("prediction", "stopped (%u frames)", g.frame_id);
    return NULL;
}

/* ── NodePlugin lifecycle ────────────────────────────────────── */
static const char* s_inputs[]  = { PRED_TOPIC_IN_TRACKS, PRED_TOPIC_IN_LOC, NULL };
static const char* s_outputs[] = { PRED_TOPIC_OUT, NULL };

static NodePlugin s_plugin;

static int prediction_init(MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler,
                           const char* params_json) {
    (void)bus;
    (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.should_stop = 0;

    g.rate_hz = PRED_DEFAULT_HZ;

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

    /* ── Subscriptions ── */
    transport_subscribe(transport, PRED_TOPIC_IN_TRACKS, on_tracked_objects, NULL);
    transport_subscribe(transport, PRED_TOPIC_IN_LOC,    on_localization,    NULL);
    transport_advertise(transport, PRED_TOPIC_OUT, 0x3A7B1C2Du);

    /* ── Discovery ── */
    discovery_advertise(discovery, PRED_TOPIC_IN_TRACKS, 0xDA7A7A11u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, PRED_TOPIC_IN_LOC,    0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, PRED_TOPIC_OUT,       0xB8A3C4D5u, CAP_PUBLISHER,  g.rate_hz);

    /* ── Mutex ── */
    pthread_mutex_init(&g.mutex, NULL);

    LOG_INFO("prediction", "initialized (%.1f Hz)", g.rate_hz);
    return 0;
}

static int prediction_start(void) {
    g.running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, prediction_thread, NULL) != 0) {
        LOG_WARN("prediction", "pthread_create failed: %s", strerror(errno));
        g.running = 0;
        return -1;
    }
    LOG_INFO("prediction", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void prediction_stop(void) {
    g.should_stop = 1;
}

static void prediction_cleanup(void) {
    if (g.running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.running = 0;
    }
    pthread_mutex_destroy(&g.mutex);
    if (g.tracks_json) {
        free(g.tracks_json);
        g.tracks_json = NULL;
    }
    LOG_INFO("prediction", "cleanup done");
}

static int prediction_health(void) {
    return 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "prediction",
    .version       = "1.0.0",
    .description   = "Multi-trajectory motion prediction (CV/Lane Keep/Lane Change)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = prediction_init,
    .start         = prediction_start,
    .stop          = prediction_stop,
    .cleanup       = prediction_cleanup,
    .health        = prediction_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
