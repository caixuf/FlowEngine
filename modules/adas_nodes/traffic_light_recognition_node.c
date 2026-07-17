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
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 tl_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase   taskbase;

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

/* ── Managed-mode main loop: republish cached traffic light state at 10 Hz
 *
 * task_thread_fn 调用本函数一次（完整主循环），循环中检查 task->should_stop
 * 退出；task_stop() 置 should_stop=true 并 join 本线程。这与原先自管 pthread
 * 的 traffic_light_thread 行为等价，只是 should_stop 改由 TaskBase 提供。 */
static int tl_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "tl_recog");

    long period_us = (g.frequency_hz > 0.0)
        ? (long)(1000000.0 / g.frequency_hz)
        : 100000L;  /* default 10 Hz */

    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

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
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface tl_vtable = {
    .execute = tl_execute,
};

/* ── NodePlugin lifecycle ────────────────────────────────── */

static const char* s_inputs[]  = { "road/traffic_lights", "sensor/camera", NULL };
static const char* s_outputs[] = { "perception/traffic_lights", NULL };

static NodePlugin s_plugin;

static int tl_recognition_init(MessageBus* bus, Transport* transport,
                                DiscoveryManager* discovery, Scheduler* scheduler,
                                const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;
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

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "traffic_light_recognition");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = g.frequency_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &tl_vtable, &cfg) != 0) {
        LOG_WARN("traffic_light_recognition", "task_base_init failed");
        return -1;
    }

    LOG_INFO("traffic_light_recognition", "initialized (%.1f Hz, managed mode)", g.frequency_hz);
    return 0;
}

static int tl_recognition_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * tl_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("traffic_light_recognition", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("traffic_light_recognition", "started (managed)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void tl_recognition_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（tl_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void tl_recognition_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源（互斥锁等）。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
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
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子，指向嵌入的 TaskBase */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
