/**
 * lane_detection_node.c — Lane detection node plugin (sandbox)
 *
 * Sandbox: generates virtual lane boundaries from road/geometry scene data.
 * Real version (HAVE_CV): would do Canny edge detection + Hough transform
 * on camera image to detect lane markings.
 *
 * Input topics:  road/geometry      — JSON road curve + lane params (from sim_world)
 *                sensor/camera      — reserved for real vision (no-op in sandbox)
 * Output topics: perception/lanes   — JSON array of LaneBoundary objects
 *
 * Sandbox algorithm:
 *   From road_geometry JSON (curve_start_x, curve_length_m, curve_offset_m,
 *   lane_width, lane_count), generate lane_count+1 boundaries spanning
 *   from -(lane_count/2)*lane_width to (lane_count/2)*lane_width, stepped
 *   by lane_width. Each boundary gets cubic polynomial coefficients fit
 *   to the smoothstep road center curve plus its lateral offset.
 *   confidence = 1.0 (ground truth from scenario).
 *
 * NodePlugin interface, compiled to liblane_detection_node.so.
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
#define PERCEPTION_LANES_TYPE_ID  0x4C414E45u  /* "LANE" */
#define ROAD_GEOMETRY_TYPE_ID     0x80AD5C12u

/* ── Node local state ────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 lane_detection_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase   taskbase;

    /* Road geometry cache (from road/geometry callback) */
    double curve_start_x;
    double curve_length_m;
    double curve_offset_m;
    double lane_width;
    int    lane_count;
    volatile int has_road_geometry;

    uint32_t frame_id;
    double   frequency_hz;

    /* Current ego x-position — used for evaluating the road curve.
     * Updated from vehicle/state so the lane boundaries track correctly
     * as the vehicle moves. */
    double ego_x;
    volatile int has_ego_x;
} g;

/* ── Smoothstep road center (matches road_geometry.h) ───── */
static inline double road_center_y_at(double x) {
    if (g.curve_length_m <= 0.0 || g.curve_offset_m == 0.0) return 0.0;
    if (x <= g.curve_start_x) return 0.0;
    double t = (x - g.curve_start_x) / g.curve_length_m;
    if (t >= 1.0) return g.curve_offset_m;
    /* smoothstep: 3t^2 - 2t^3, zero derivative at both ends */
    return g.curve_offset_m * (3.0 * t * t - 2.0 * t * t * t);
}

/* ── Vehicle state callback (track ego x for curve evaluation) ─ */
static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j)) {
        g.ego_x = j->valuedouble;
        g.has_ego_x = 1;
    }
    cJSON_Delete(root);
}

/* ── Road geometry callback ──────────────────────────────── */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "curve_start_x")) && cJSON_IsNumber(j))
        g.curve_start_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "curve_length_m")) && cJSON_IsNumber(j))
        g.curve_length_m = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "curve_offset_m")) && cJSON_IsNumber(j))
        g.curve_offset_m = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "lane_width")) && cJSON_IsNumber(j))
        g.lane_width = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "lane_count")) && cJSON_IsNumber(j))
        g.lane_count = (int)j->valuedouble;
    g.has_road_geometry = 1;
    cJSON_Delete(root);
}

/* ── sensor/camera callback (reserved, no-op in sandbox) ── */
static void on_camera(const Message* msg, void* user_data) {
    (void)msg;
    (void)user_data;
    /* Reserved for HAVE_CV real vision processing — no-op in sandbox */
}

/* ── Fit cubic polynomial coefficients for a lane boundary ─
 *
 * For the sandbox we sample the smoothstep road-center curve at 4
 * points (x = 0, 20, 40, 60 m ahead) and fit a cubic:
 *   y(x) = c0 + c1*x + c2*x^2 + c3*x^3
 *
 * The boundary offset (lateral position at x=0) is added to c0.
 * For straight roads all higher terms are zero.
 *
 * Solving the 4x4 Vandermonde system analytically for
 * evenly-spaced x = [0, 20, 40, 60]:
 *
 *   c0 = y0
 *   20 c1 +  400 c2 +   8000 c3 = y1 - y0
 *   40 c1 + 1600 c2 +  64000 c3 = y2 - y0
 *   60 c1 + 3600 c2 + 216000 c3 = y3 - y0
 *
 * Gaussian elimination yields:
 *   c3 = (  (y3-3y2+3y1-y0) ) / 24000
 *   c2 = ( (y2-2y1+y0) - 800*c3 ) / 400
 *   c1 = ( (y1-y0) - 400*c2 - 8000*c3 ) / 20
 */
static void fit_boundary_poly(double offset, double c[4]) {
    /* Evaluate road center at four sample points */
    double y[4];
    y[0] = road_center_y_at(0.0);
    y[1] = road_center_y_at(20.0);
    y[2] = road_center_y_at(40.0);
    y[3] = road_center_y_at(60.0);

    /* Diff vector relative to y0 */
    double d1 = y[1] - y[0];
    double d2 = y[2] - y[0];
    double d3 = y[3] - y[0];

    /* Solve cubic coefficients for center line */
    double c3 = (d3 - 3.0 * d2 + 3.0 * d1) / 24000.0;
    double c2 = (d2 - 2.0 * d1 - 800.0 * c3) / 400.0;
    double c1 = (d1 - 400.0 * c2 - 8000.0 * c3) / 20.0;

    c[0] = y[0] + offset;
    c[1] = c1;
    c[2] = c2;
    c[3] = c3;
}

/* ── Determine lane type string ──────────────────────────── */
static const char* boundary_type(int boundary_idx, int total_boundaries) {
    /* Outer boundaries are SOLID, inner ones are DASHED */
    if (boundary_idx == 0 || boundary_idx == total_boundaries - 1)
        return "SOLID";
    return "DASHED";
}

/* ── Managed-mode main loop: publish lane boundaries at 10 Hz ─── */
static int lane_detection_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "lane_detect");

    long period_us = (g.frequency_hz > 0.0)
        ? (long)(1000000.0 / g.frequency_hz)
        : 100000L;  /* default 10 Hz */

    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

        if (!g.has_road_geometry) {
            /* No road data yet — skip this frame */
            g.frame_id++;
            continue;
        }

        /* Number of boundaries = lane_count + 1 */
        int num = g.lane_count + 1;
        if (num < 2) num = 2;          /* at least 2 boundaries */
        if (num > 8) num = 8;          /* clamp to MAX boundaries */

        cJSON* root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "frame_id", (double)g.frame_id);
        cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());

        cJSON* boundaries = cJSON_CreateArray();

        for (int i = 0; i < num; i++) {
            /* Lateral offset: boundaries equally spaced across road width.
             * lane_width is per-lane width; total road spans
             * from -(lane_count/2)*lane_width to (lane_count/2)*lane_width. */
            double offset = ((double)i - (double)g.lane_count * 0.5) * g.lane_width;

            double coeffs[4];
            fit_boundary_poly(offset, coeffs);

            /* lane_id: positive on left, negative on right, 0 at center */
            int lane_id = g.lane_count / 2 - i;

            cJSON* b = cJSON_CreateObject();
            cJSON_AddNumberToObject(b, "lane_id", lane_id);
            cJSON_AddStringToObject(b, "type", boundary_type(i, num));

            cJSON* coeff_arr = cJSON_CreateDoubleArray(coeffs, 4);
            cJSON_AddItemToObject(b, "coeffs", coeff_arr);

            cJSON_AddNumberToObject(b, "range", 80.0);
            cJSON_AddNumberToObject(b, "confidence", 1.0);
            cJSON_AddItemToArray(boundaries, b);
        }

        cJSON_AddItemToObject(root, "boundaries", boundaries);
        char* json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            transport_publish(g.transport, "perception/lanes",
                              (const uint8_t*)json_str, (uint32_t)strlen(json_str) + 1);
            free(json_str);
        }
        cJSON_Delete(root);

        g.frame_id++;
    }

    LOG_INFO("lane_detection", "stopped (%u frames)", g.frame_id);
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface lane_detection_vtable = {
    .execute = lane_detection_execute,
};

/* ── NodePlugin lifecycle ────────────────────────────────── */

static const char* s_inputs[]  = { "road/geometry", "sensor/camera", "vehicle/state", NULL };
static const char* s_outputs[] = { "perception/lanes", NULL };

static NodePlugin s_plugin;

static int lane_detection_init(MessageBus* bus, Transport* transport,
                                DiscoveryManager* discovery, Scheduler* scheduler,
                                const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;
    g.frequency_hz = 10.0;

    /* Defaults (will be overwritten by road/geometry incoming data) */
    g.lane_width = 3.5;
    g.lane_count = 2;

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "frequency_hz")) && cJSON_IsNumber(j))
                g.frequency_hz = j->valuedouble;
            cJSON_Delete(p);
        }
    }

    /* Subscribe to road geometry and camera (reserved) */
    transport_subscribe(transport, "road/geometry", on_road_geometry, NULL);
    transport_subscribe(transport, "sensor/camera", on_camera, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);

    /* Advertise output */
    transport_advertise(transport, "perception/lanes", PERCEPTION_LANES_TYPE_ID);
    discovery_advertise(discovery, "perception/lanes", PERCEPTION_LANES_TYPE_ID,
                        CAP_PUBLISHER, g.frequency_hz);

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "lane_detection");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = g.frequency_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &lane_detection_vtable, &cfg) != 0) {
        LOG_WARN("lane_detection", "task_base_init failed");
        return -1;
    }

    LOG_INFO("lane_detection", "initialized (%.1f Hz)", g.frequency_hz);
    return 0;
}

static int lane_detection_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * lane_detection_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("lane_detection", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("lane_detection", "started (managed)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void lane_detection_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（lane_detection_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void lane_detection_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源（互斥锁等）。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    LOG_INFO("lane_detection", "cleanup done");
}

static int lane_detection_health(void) {
    return g.has_road_geometry ? 0 : 1;  /* degraded if no geometry data */
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "lane_detection",
    .version       = "1.0.0",
    .description   = "Lane boundary detection (sandbox: from road/geometry)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = lane_detection_init,
    .start         = lane_detection_start,
    .stop          = lane_detection_stop,
    .cleanup       = lane_detection_cleanup,
    .health        = lane_detection_health,
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子，指向嵌入的 TaskBase */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
