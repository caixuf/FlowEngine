/**
 * traffic_light_recognition_node.c — Traffic light recognition node plugin (sandbox)
 *
 * Sandbox: forwards sim_world's traffic light state from road/traffic_lights
 * into the perception pipeline as perception/traffic_lights, adding
 * source="simulation" and confidence=1.0 per light.
 * Real version (HAVE_CV): would do camera-based traffic light detection
 * using color blob analysis / deep learning.
 *
 * Input topics:  road/traffic_lights   — JSON (from sim_world)
 *                sensor/camera         — reserved for real vision (no-op in sandbox)
 * Output topics: perception/traffic_lights — JSON with added perception metadata
 *
 * Sandbox algorithm:
 *   - Parse incoming road/traffic_lights JSON with cJSON_Parse
 *   - Republish to perception/traffic_lights with:
 *     * "source": "simulation" at root level
 *     * "confidence": 1.0 added to each light
 *     * "frame_id" and "timestamp_us" stamped by this node
 *   - Cache last known state, publish at 10 Hz even when no new input arrives
 *
 * NodePlugin interface, compiled to libtraffic_light_recognition_node.so.
 */

#include "node_plugin.h"
#include "clock_service.h"
#include "transport.h"
#include "discovery.h"
#include "logger.h"
#include <cjson/cJSON.h>

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

/* ── Type IDs for discovery/advertising ──────────────────── */
#define PERCEPTION_TRAFFIC_LIGHTS_TYPE_ID  0x5452464Cu  /* "TRFL" (Traffic Light) */
#define ROAD_TRAFFIC_LIGHTS_TYPE_ID        0x7E5C0FFEu

/* ── Node local state ────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    /* Cached traffic light state (from road/traffic_lights) */
    char   cached_json[2048];
    volatile int has_cached;

    uint32_t frame_id;
    double   frequency_hz;
} g;

/* ── road/traffic_lights callback ────────────────────────── */
static void on_road_traffic_lights(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    size_t copy = msg->data_size < sizeof(g.cached_json) - 1
                  ? msg->data_size : sizeof(g.cached_json) - 1;
    memcpy(g.cached_json, msg->data, copy);
    g.cached_json[copy] = '\0';
    g.has_cached = 1;
}

/* ── sensor/camera callback (reserved, no-op in sandbox) ── */
static void on_camera(const Message* msg, void* user_data) {
    (void)msg;
    (void)user_data;
    /* Reserved for HAVE_CV real vision processing */
}

/* ── Add confidence=1.0 to each light in the lights array ─
 *
 * Mutates the incoming parsed JSON: adds "confidence": 1.0 to
 * every object in the "lights" array if not already present.
 */
static void add_confidence_to_lights(cJSON* root) {
    cJSON* lights = cJSON_GetObjectItemCaseSensitive(root, "lights");
    if (!lights || !cJSON_IsArray(lights)) return;

    int count = cJSON_GetArraySize(lights);
    for (int i = 0; i < count; i++) {
        cJSON* light = cJSON_GetArrayItem(lights, i);
        if (!light) continue;
        cJSON* conf = cJSON_GetObjectItemCaseSensitive(light, "confidence");
        if (!conf) {
            cJSON_AddNumberToObject(light, "confidence", 1.0);
        }
    }
}

/* ── Main thread: republish cached traffic light state at 10 Hz ─ */
static void* traffic_light_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "tl_recog");

    long period_us = (g.frequency_hz > 0.0)
        ? (long)(1000000.0 / g.frequency_hz)
        : 100000L;  /* default 10 Hz */

    while (!g.should_stop) {
        usleep((unsigned long)period_us);
        if (g.should_stop) break;

        if (!g.has_cached) {
            /* No traffic light data yet — skip */
            g.frame_id++;
            continue;
        }

        /* Parse cached JSON */
        cJSON* root = cJSON_Parse(g.cached_json);
        if (!root) {
            g.frame_id++;
            continue;
        }

        /* Add confidence to each detected light */
        add_confidence_to_lights(root);

        /* Add frame-level metadata */
        cJSON_AddNumberToObject(root, "frame_id", (double)g.frame_id);
        cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());
        cJSON_AddStringToObject(root, "source", "simulation");

        /* Serialize and publish */
        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            transport_publish(g.transport, "perception/traffic_lights",
                              (const uint8_t*)json_str, (uint32_t)strlen(json_str) + 1);
            free(json_str);
        }
        cJSON_Delete(root);

        g.frame_id++;
    }

    LOG_INFO("traffic_light_recognition", "stopped (%u frames)", g.frame_id);
    return NULL;
}

/* ── NodePlugin lifecycle ────────────────────────────────── */

static const char* s_inputs[]  = { "road/traffic_lights", "sensor/camera", NULL };
static const char* s_outputs[] = { "perception/traffic_lights", NULL };

static NodePlugin s_plugin;

static int tl_recognition_init(MessageBus* bus, Transport* transport,
                                DiscoveryManager* discovery, Scheduler* scheduler,
                                const char* params_json) {
    (void)bus;
    (void)scheduler;

    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.should_stop = 0;
    g.frequency_hz = 10.0;

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "frequency_hz")) && cJSON_IsNumber(j))
                g.frequency_hz = j->valuedouble;
            cJSON_Delete(p);
        }
    }

    /* Subscribe to road/traffic_lights (sim_world source) and camera (reserved) */
    transport_subscribe(transport, "road/traffic_lights", on_road_traffic_lights, NULL);
    transport_subscribe(transport, "sensor/camera", on_camera, NULL);

    /* Advertise output */
    transport_advertise(transport, "perception/traffic_lights", PERCEPTION_TRAFFIC_LIGHTS_TYPE_ID);
    discovery_advertise(discovery, "perception/traffic_lights",
                        PERCEPTION_TRAFFIC_LIGHTS_TYPE_ID,
                        CAP_PUBLISHER, g.frequency_hz);

    LOG_INFO("traffic_light_recognition", "initialized (%.1f Hz)", g.frequency_hz);
    return 0;
}

static int tl_recognition_start(void) {
    g.running = 1;
    g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, traffic_light_thread, NULL) != 0) {
        LOG_WARN("traffic_light_recognition", "pthread_create failed: %s", strerror(errno));
        g.running = 0;
        return -1;
    }
    LOG_INFO("traffic_light_recognition", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void tl_recognition_stop(void) {
    g.should_stop = 1;
}

static void tl_recognition_cleanup(void) {
    if (g.running) {
        g.should_stop = 1;
        pthread_join(g.thread, NULL);
        g.running = 0;
    }
    LOG_INFO("traffic_light_recognition", "cleanup done");
}

static int tl_recognition_health(void) {
    return g.has_cached ? 0 : 1;  /* degraded if no traffic light data */
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "traffic_light_recognition",
    .version       = "1.0.0",
    .description   = "Traffic light recognition (sandbox: from road/traffic_lights)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = tl_recognition_init,
    .start         = tl_recognition_start,
    .stop          = tl_recognition_stop,
    .cleanup       = tl_recognition_cleanup,
    .health        = tl_recognition_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
