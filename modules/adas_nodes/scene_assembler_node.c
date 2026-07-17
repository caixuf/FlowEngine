/**
 * scene_assembler_node.c — 场景聚合器 (场景理解链路 · 统一环境模型输出)
 *
 * 订阅所有感知 topic，聚合成唯一的 scene/environment_model 消息。
 * 这是规范要求的"单一输出通道"：planning/control 只订阅这一个 topic。
 *
 * 输入:
 *   fusion/localization         — 自车定位 (Localization or JSON)
 *   perception/tracked_objects  — 跨帧跟踪目标 (TrackedObject[] JSON)
 *   perception/lanes            — 车道线 (LaneBoundary[] JSON)
 *   perception/traffic_lights   — 信号灯 (JSON)
 *   perception/traversability   — 可行驶区域 (JSON)
 *   prediction/tracks           — 运动预测 (PredictionTrack[] JSON)
 *   road/geometry               — 道路拓扑 (JSON)
 *
 * 输出:
 *   scene/environment_model     — 统一场景模型 (cJSON, 10 Hz)
 *
 * NodePlugin 接口，编译为 libscene_assembler_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

/* 降级级别 */
typedef enum {
    DEGRADE_NORMAL    = 0,  /* 全量正常 */
    DEGRADE_MINOR     = 1,  /* 个别模块降级 */
    DEGRADE_MAJOR     = 2,  /* 关键模块缺失 */
    DEGRADE_CRITICAL  = 3,  /* 严重降级，安全兜底 */
} DegradationLevel;

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 assembler_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase   taskbase;

    ReflectiveStateMachine sm;

    /* ── 最新值锁存 (latest-value latch) ── */
    /* 自车定位 */
    double ego_x, ego_y, ego_z;
    double ego_heading, ego_speed, ego_yaw_rate, ego_accel;
    volatile int has_ego;
    volatile int ego_has_world;
    uint64_t      ego_ts_us;

    /* 跟踪目标 */
    char    tracked_json[16384];
    volatile int has_tracked;
    uint64_t      tracked_ts_us;

    /* 车道线 */
    char    lanes_json[8192];
    volatile int has_lanes;
    uint64_t      lanes_ts_us;

    /* 信号灯 */
    char    traffic_lights_json[4096];
    volatile int has_traffic_lights;
    uint64_t      traffic_lights_ts_us;

    /* 可行驶区域 */
    char    traversability_json[4096];
    volatile int has_traversability;
    uint64_t      traversability_ts_us;

    /* 运动预测 */
    char    prediction_json[16384];
    volatile int has_prediction;
    uint64_t      prediction_ts_us;

    /* 道路拓扑 */
    double curve_start_x, curve_length_m, curve_offset_m;
    double lane_width, lane_count;
    volatile int has_road_geometry;

    /* 统计 */
    int      frame_count;
    uint64_t last_publish_us;
} g;

/* ── 订阅回调 — 锁存最新值 ─────────────────────────────────── */

static void on_localization(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    cJSON* j;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "x")) && cJSON_IsNumber(j))
        g.ego_x = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "y")) && cJSON_IsNumber(j))
        g.ego_y = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "z")) && cJSON_IsNumber(j))
        g.ego_z = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "v")) && cJSON_IsNumber(j))
        g.ego_speed = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "heading")) && cJSON_IsNumber(j))
        g.ego_heading = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "yaw_rate")) && cJSON_IsNumber(j))
        g.ego_yaw_rate = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "accel")) && cJSON_IsNumber(j))
        g.ego_accel = j->valuedouble;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "world_lat")) && cJSON_IsNumber(j))
        g.ego_has_world = 1;
    if ((j = cJSON_GetObjectItemCaseSensitive(root, "world_lon")) && cJSON_IsNumber(j))
        g.ego_has_world = 1;

    g.ego_ts_us = clock_now_us();
    g.has_ego = 1;
    cJSON_Delete(root);
}

static void on_tracked_objects(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    size_t len = strlen((const char*)msg->data);
    if (len >= sizeof(g.tracked_json)) len = sizeof(g.tracked_json) - 1;
    memcpy(g.tracked_json, msg->data, len);
    g.tracked_json[len] = '\0';
    g.tracked_ts_us = clock_now_us();
    g.has_tracked = 1;
}

static void on_lanes(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    size_t len = strlen((const char*)msg->data);
    if (len >= sizeof(g.lanes_json)) len = sizeof(g.lanes_json) - 1;
    memcpy(g.lanes_json, msg->data, len);
    g.lanes_json[len] = '\0';
    g.lanes_ts_us = clock_now_us();
    g.has_lanes = 1;
}

static void on_traffic_lights(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    size_t len = strlen((const char*)msg->data);
    if (len >= sizeof(g.traffic_lights_json)) len = sizeof(g.traffic_lights_json) - 1;
    memcpy(g.traffic_lights_json, msg->data, len);
    g.traffic_lights_json[len] = '\0';
    g.traffic_lights_ts_us = clock_now_us();
    g.has_traffic_lights = 1;
}

static void on_traversability(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    size_t len = strlen((const char*)msg->data);
    if (len >= sizeof(g.traversability_json)) len = sizeof(g.traversability_json) - 1;
    memcpy(g.traversability_json, msg->data, len);
    g.traversability_json[len] = '\0';
    g.traversability_ts_us = clock_now_us();
    g.has_traversability = 1;
}

static void on_prediction(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    size_t len = strlen((const char*)msg->data);
    if (len >= sizeof(g.prediction_json)) len = sizeof(g.prediction_json) - 1;
    memcpy(g.prediction_json, msg->data, len);
    g.prediction_json[len] = '\0';
    g.prediction_ts_us = clock_now_us();
    g.has_prediction = 1;
}

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
        g.lane_count = j->valuedouble;

    g.has_road_geometry = 1;
    cJSON_Delete(root);
}

/* ── 降级评估 ──────────────────────────────────────────────── */

static DegradationLevel assess_degradation(uint64_t now_us) {
    int missing = 0;
    int stale = 0;
#define STALE_US 2000000ULL  /* 2s without update = stale */

    if (!g.has_ego) missing++;
    else if (now_us - g.ego_ts_us > STALE_US) stale++;

    if (!g.has_tracked) missing++;
    else if (now_us - g.tracked_ts_us > STALE_US) stale++;

    if (!g.has_lanes) missing++;
    else if (now_us - g.lanes_ts_us > STALE_US) stale++;

    if (!g.has_road_geometry) missing++;

    if (missing >= 2) return DEGRADE_CRITICAL;
    if (missing >= 1 || stale >= 3) return DEGRADE_MAJOR;
    if (stale >= 1) return DEGRADE_MINOR;
    return DEGRADE_NORMAL;
#undef STALE_US
}

static const char* degradation_str(DegradationLevel lv) {
    switch (lv) {
        case DEGRADE_NORMAL:   return "NORMAL";
        case DEGRADE_MINOR:    return "DEGRADED";
        case DEGRADE_MAJOR:    return "CRITICAL";
        case DEGRADE_CRITICAL: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

/* ── 聚合主循环（托管模式） ─────────────────────────────────────
 *
 * task_thread_fn 调用本函数一次（完整主循环），循环中检查 task->should_stop
 * 退出；task_stop() 置 should_stop=true 并 join 本线程。这与原先自管 pthread
 * 的 assembler_thread 行为等价，只是 should_stop 改由 TaskBase 提供。 */
static int assembler_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "assembler");

    while (!task->should_stop) {
        usleep(100000);  /* 10 Hz */
        if (task->should_stop) break;

        uint64_t now_us = clock_now_us();
        DegradationLevel degrade = assess_degradation(now_us);

        cJSON* root = cJSON_CreateObject();

        /* timestamp + schema */
        cJSON_AddNumberToObject(root, "timestamp_us", (double)now_us);
        cJSON_AddStringToObject(root, "schema_version", "flowengine.scene.v1");

        /* ── degradation ── */
        cJSON* degradation = cJSON_CreateObject();
        cJSON_AddStringToObject(degradation, "level", degradation_str(degrade));
        cJSON_AddBoolToObject(degradation, "fallback_active", (degrade >= DEGRADE_MAJOR));
        cJSON_AddItemToObject(root, "degradation", degradation);

        /* ── ego ── */
        if (g.has_ego) {
            cJSON* ego = cJSON_CreateObject();
            cJSON_AddNumberToObject(ego, "x", g.ego_x);
            cJSON_AddNumberToObject(ego, "y", g.ego_y);
            cJSON_AddNumberToObject(ego, "z", g.ego_z);
            cJSON_AddNumberToObject(ego, "heading", g.ego_heading);
            cJSON_AddNumberToObject(ego, "speed", g.ego_speed);
            cJSON_AddNumberToObject(ego, "yaw_rate", g.ego_yaw_rate);
            cJSON_AddNumberToObject(ego, "accel", g.ego_accel);
            cJSON_AddNumberToObject(ego, "lane_id", 0);
            cJSON_AddItemToObject(root, "ego", ego);
        }

        /* ── objects ── */
        if (g.has_tracked && g.tracked_json[0]) {
            cJSON* tracked = cJSON_Parse(g.tracked_json);
            if (tracked) {
                cJSON* obj_arr = cJSON_GetObjectItemCaseSensitive(tracked, "objects");
                if (obj_arr) {
                    /* duplicate to own tree */
                    cJSON* arr = cJSON_Duplicate(obj_arr, 1);
                    if (arr) cJSON_AddItemToObject(root, "objects", arr);
                }
                cJSON_Delete(tracked);
            }
        }
        if (!cJSON_HasObjectItem(root, "objects")) {
            cJSON_AddItemToObject(root, "objects", cJSON_CreateArray());
        }

        /* ── lanes ── */
        if (g.has_lanes && g.lanes_json[0]) {
            cJSON* lanes = cJSON_Parse(g.lanes_json);
            if (lanes) {
                cJSON* lane_arr = cJSON_GetObjectItemCaseSensitive(lanes, "boundaries");
                if (lane_arr) {
                    cJSON* arr = cJSON_Duplicate(lane_arr, 1);
                    if (arr) cJSON_AddItemToObject(root, "lanes", arr);
                }
                cJSON_Delete(lanes);
            }
        }
        if (!cJSON_HasObjectItem(root, "lanes")) {
            cJSON_AddItemToObject(root, "lanes", cJSON_CreateArray());
        }

        /* ── traffic_elements ── */
        cJSON* traffic = cJSON_CreateObject();
        if (g.has_traffic_lights && g.traffic_lights_json[0]) {
            cJSON* tl = cJSON_Parse(g.traffic_lights_json);
            if (tl) {
                cJSON* lights = cJSON_GetObjectItemCaseSensitive(tl, "lights");
                if (lights) {
                    cJSON* dup = cJSON_Duplicate(lights, 1);
                    if (dup) cJSON_AddItemToObject(traffic, "lights", dup);
                }
                cJSON_Delete(tl);
            }
        }
        if (!cJSON_HasObjectItem(traffic, "lights")) {
            cJSON_AddItemToObject(traffic, "lights", cJSON_CreateArray());
        }
        cJSON_AddItemToObject(traffic, "signs", cJSON_CreateArray()); /* 预留 */
        cJSON_AddItemToObject(root, "traffic_elements", traffic);

        /* ── drivable_area ── */
        if (g.has_traversability && g.traversability_json[0]) {
            cJSON* trav = cJSON_Parse(g.traversability_json);
            if (trav) {
                cJSON_AddItemToObject(root, "drivable_area", trav);
            } else {
                cJSON_AddItemToObject(root, "drivable_area", cJSON_CreateObject());
            }
        } else {
            cJSON_AddItemToObject(root, "drivable_area", cJSON_CreateObject());
        }

        /* ── predictions ── */
        if (g.has_prediction && g.prediction_json[0]) {
            cJSON* pred = cJSON_Parse(g.prediction_json);
            if (pred) {
                cJSON* pred_arr = cJSON_GetObjectItemCaseSensitive(pred, "tracks");
                if (pred_arr) {
                    cJSON* arr = cJSON_Duplicate(pred_arr, 1);
                    if (arr) cJSON_AddItemToObject(root, "predictions", arr);
                }
                cJSON_Delete(pred);
            }
        }
        if (!cJSON_HasObjectItem(root, "predictions")) {
            cJSON_AddItemToObject(root, "predictions", cJSON_CreateArray());
        }

        /* ── road_topology ── */
        cJSON* road = cJSON_CreateObject();
        cJSON_AddNumberToObject(road, "lane_count", g.lane_count > 0 ? g.lane_count : 2);
        cJSON_AddNumberToObject(road, "lane_width", g.lane_width > 0 ? g.lane_width : 3.5);
        if (g.has_road_geometry) {
            cJSON* curve = cJSON_CreateObject();
            cJSON_AddNumberToObject(curve, "start_x", g.curve_start_x);
            cJSON_AddNumberToObject(curve, "length_m", g.curve_length_m);
            cJSON_AddNumberToObject(curve, "offset_m", g.curve_offset_m);
            cJSON_AddItemToObject(road, "curve", curve);
        }
        cJSON_AddItemToObject(root, "road_topology", road);

        /* ── publish ── */
        char* s = cJSON_PrintUnformatted(root);
        size_t slen = strlen(s);
        if (slen < 4096) {
            transport_publish(g.transport, "scene/environment_model",
                              (const uint8_t*)s, (uint32_t)slen + 1);
        } else {
            LOG_WARN("assembler", "EnvironmentModel too large: %zu bytes", slen);
        }
        free(s);
        cJSON_Delete(root);

        g.frame_count++;
        g.last_publish_us = now_us;
    }

    LOG_INFO("assembler", "stopped (%d frames)", g.frame_count);
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整聚合主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface assembler_vtable = {
    .execute = assembler_execute,
};

/* ── NodePlugin 接口 ────────────────────────────────────────── */

static const char* s_inputs[]  = {
    "fusion/localization", "perception/tracked_objects", "perception/lanes",
    "perception/traffic_lights", "perception/traversability",
    "prediction/tracks", "road/geometry", NULL
};
static const char* s_outputs[] = { "scene/environment_model", NULL };

static NodePlugin s_plugin;

static int assembler_init(MessageBus* bus, Transport* transport,
                          DiscoveryManager* discovery, Scheduler* scheduler,
                          const char* params_json) {
    (void)bus;
    (void)params_json;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;

    transport_subscribe(transport, "fusion/localization",          on_localization,    NULL);
    transport_subscribe(transport, "perception/tracked_objects",   on_tracked_objects, NULL);
    transport_subscribe(transport, "perception/lanes",             on_lanes,           NULL);
    transport_subscribe(transport, "perception/traffic_lights",    on_traffic_lights,  NULL);
    transport_subscribe(transport, "perception/traversability",    on_traversability,  NULL);
    transport_subscribe(transport, "prediction/tracks",            on_prediction,      NULL);
    transport_subscribe(transport, "road/geometry",                on_road_geometry,   NULL);

    transport_advertise(transport, "scene/environment_model", 0x5C3E4D10u);
    discovery_advertise(discovery, "scene/environment_model", 0x5C3E4D10u, CAP_PUBLISHER, 10.0f);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "assembler");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("assembler", "initialized (7 inputs → scene/environment_model @ 10 Hz)");

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep(100000) 的 10 Hz 对齐。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "scene_assembler");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = 10.0;  /* usleep(100000) → 10 Hz */
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &assembler_vtable, &cfg) != 0) {
        LOG_WARN("scene_assembler", "task_base_init failed");
        return -1;
    }
    return 0;
}

static int assembler_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * assembler_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("assembler", "node_start_managed failed: %d", rc);
        return rc;
    }
    node_announce_self(g.transport, &s_plugin);
    LOG_INFO("assembler", "started (managed)");
    return 0;
}

static void assembler_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（assembler_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}
static int  assembler_health(void) { return g.has_ego && g.has_tracked ? 0 : 1; }

static void assembler_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源（互斥锁等）。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    LOG_INFO("assembler", "cleanup done");
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "scene_assembler",
    .version       = "1.0.0",
    .description   = "Scene assembler: aggregates all perception into unified EnvironmentModel",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = assembler_init,
    .start         = assembler_start,
    .stop          = assembler_stop,
    .cleanup       = assembler_cleanup,
    .health        = assembler_health,
    .taskbase      = &g.taskbase,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
