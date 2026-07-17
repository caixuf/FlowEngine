/**
 * data_recorder_node.c — 训练样本采集节点 (车端学习闭环 · Stage 0)
 *
 * 订阅 fusion/localization + planning/trajectory，按固定频率把"最新对齐"的
 * (特征, 标签) 样本以 JSONL (每行一个 JSON) 落盘，供 tools/train/ 离线训练。
 *
 * 设计要点:
 *   - 时间对齐: 采用"最新值锁存"(latest-value latch) 的简单对齐策略，采样时刻
 *     取各 topic 最近一次的值。对 10~50Hz 的同源仿真链路足够；真实车端可换成
 *     基于时间戳的插值对齐。
 *   - 数据契约 (与 inference_node / tools/train 一致):
 *       特征 features = [ego_v, ego_y, ego_heading, ego_yaw_rate]
 *       标签 label    = planning_target_speed (模仿学习: 学 planning 的目标速度)
 *     v2 样本额外写入 obstacles/planning/control/features_v2，供 PyTorch
 *     场景特征训练使用；features 保持 v1 兼容。
 *   - 轻量: 只写文本 JSONL，不引入 Bag v2 二进制依赖，便于 Python 直接解析。
 *
 * NodePlugin 接口，编译为 libdata_recorder_node.so。
 */

#include "node_plugin.h"
#include "state_machine.h"
#include "adas_msgs_gen.h"
#include "logger.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define RECORDER_MAX_OBSTACLES 8

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    pthread_t    thread;
    volatile int running;
    volatile int should_stop;

    ReflectiveStateMachine sm;

    FILE* out;
    char  out_path[256];

    /* 最新锁存值 */
    double ego_x, ego_y, ego_v, ego_heading, ego_yaw_rate;
    double planning_target_speed;
    ObstacleList obstacles;
    ControlCmd   control;
    volatile int has_fusion;
    volatile int has_planning;
    volatile int has_obstacles;
    volatile int has_control;

    double cfg_frequency_hz;
    int    sample_count;
} g;

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    /* fusion/localization now publishes cJSON */
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (root) {
        cJSON* j;
        j = cJSON_GetObjectItemCaseSensitive(root, "x");
        if (cJSON_IsNumber(j)) g.ego_x = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "y");
        if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "v");
        if (cJSON_IsNumber(j)) g.ego_v = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "heading");
        if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
        j = cJSON_GetObjectItemCaseSensitive(root, "yaw_rate");
        if (cJSON_IsNumber(j)) g.ego_yaw_rate = j->valuedouble;
        cJSON_Delete(root);
    }
    g.has_fusion = 1;
}

static void on_planning(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* j = cJSON_GetObjectItemCaseSensitive(root, "target_speed");
        if (cJSON_IsNumber(j)) {
            g.planning_target_speed = j->valuedouble;
        }
        cJSON_Delete(root);
    }
    g.has_planning = 1;
}

static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    ObstacleList obs;
    if (ObstacleList_deserialize(&obs, (const uint8_t*)msg->data, msg->data_size) == 0) {
        g.obstacles = obs;
        if (g.obstacles.count > RECORDER_MAX_OBSTACLES) g.obstacles.count = RECORDER_MAX_OBSTACLES;
        g.has_obstacles = 1;
    }
}

static void on_control(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    ControlCmd cmd;
    if (ControlCmd_deserialize(&cmd, (const uint8_t*)msg->data, msg->data_size) == 0) {
        g.control = cmd;
        g.has_control = 1;
    }
}

static void select_front_obstacles(const ObstacleList* src, const Obstacle** first, const Obstacle** second) {
    *first = NULL;
    *second = NULL;
    if (!src) return;
    for (uint32_t i = 0; i < src->count && i < RECORDER_MAX_OBSTACLES; i++) {
        const Obstacle* obs = &src->obstacles[i];
        if (obs->x < 0.0f) continue;
        if (!*first || obs->x < (*first)->x) {
            *second = *first;
            *first = obs;
        } else if (!*second || obs->x < (*second)->x) {
            *second = obs;
        }
    }
}

static cJSON* build_obstacle_json(const Obstacle* obs) {
    cJSON* obj = cJSON_CreateObject();
    if (obs) {
        cJSON_AddNumberToObject(obj, "id", (double)obs->id);
        cJSON_AddNumberToObject(obj, "x", (double)obs->x);
        cJSON_AddNumberToObject(obj, "y", (double)obs->y);
        cJSON_AddNumberToObject(obj, "vx", (double)obs->vx);
        cJSON_AddNumberToObject(obj, "vy", (double)obs->vy);
        cJSON_AddNumberToObject(obj, "type", (int)obs->type);
        cJSON_AddNumberToObject(obj, "confidence", (double)obs->confidence);
    } else {
        cJSON_AddNumberToObject(obj, "id", 0);
        cJSON_AddNumberToObject(obj, "x", 0.0);
        cJSON_AddNumberToObject(obj, "y", 0.0);
        cJSON_AddNumberToObject(obj, "vx", 0.0);
        cJSON_AddNumberToObject(obj, "vy", 0.0);
        cJSON_AddNumberToObject(obj, "type", 0);
        cJSON_AddNumberToObject(obj, "confidence", 0.0);
    }
    return obj;
}

static void* recorder_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "recorder");

    double period = g.cfg_frequency_hz > 0.0 ? 1.0 / g.cfg_frequency_hz : 0.1;
    useconds_t sleep_us = (useconds_t)(period * 1e6);

    while (!g.should_stop) {
        usleep(sleep_us);
        if (g.should_stop) break;
        /* 只在两路数据都到齐时采样，保证 (特征,标签) 完整 */
        if (!g.has_fusion || !g.has_planning || !g.out) continue;

        const Obstacle* front0 = NULL;
        const Obstacle* front1 = NULL;
        select_front_obstacles(g.has_obstacles ? &g.obstacles : NULL, &front0, &front1);
        double control_brake = g.has_control ? g.control.brake : 0.0;
        int control_emergency_stop = g.has_control ? (g.control.emergency_stop ? 1 : 0) : 0;

        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "schema_version", "flowengine.e2e_sample.v2");
        cJSON_AddNumberToObject(root, "t", (double)(clock_now_realtime_us() / 1000));

        {   double arr[] = {g.ego_v, g.ego_y, g.ego_heading, g.ego_yaw_rate};
            cJSON_AddItemToObject(root, "features", cJSON_CreateDoubleArray(arr, 4)); }

        {   double arr[] = {
                g.ego_v, g.ego_y, g.ego_heading, g.ego_yaw_rate,
                front0 ? front0->x : 0.0, front0 ? front0->y : 0.0, front0 ? front0->vx : 0.0,
                (double)(front0 ? (int)front0->type : 0), front0 ? front0->confidence : 0.0,
                front1 ? front1->x : 0.0, front1 ? front1->y : 0.0, front1 ? front1->vx : 0.0,
                (double)(front1 ? (int)front1->type : 0), control_brake, (double)control_emergency_stop };
            cJSON_AddItemToObject(root, "features_v2", cJSON_CreateDoubleArray(arr, 15)); }

        cJSON_AddNumberToObject(root, "label", g.planning_target_speed);

        cJSON* ego = cJSON_CreateObject();
        cJSON_AddNumberToObject(ego, "x", g.ego_x);
        cJSON_AddNumberToObject(ego, "y", g.ego_y);
        cJSON_AddNumberToObject(ego, "v", g.ego_v);
        cJSON_AddNumberToObject(ego, "heading", g.ego_heading);
        cJSON_AddNumberToObject(ego, "yaw_rate", g.ego_yaw_rate);
        cJSON_AddItemToObject(root, "ego", ego);

        cJSON* planning = cJSON_CreateObject();
        cJSON_AddNumberToObject(planning, "target_speed", g.planning_target_speed);
        cJSON_AddItemToObject(root, "planning", planning);

        cJSON* control = cJSON_CreateObject();
        cJSON_AddNumberToObject(control, "throttle", g.has_control ? g.control.throttle : 0.0);
        cJSON_AddNumberToObject(control, "brake", control_brake);
        cJSON_AddNumberToObject(control, "steering", g.has_control ? g.control.steering : 0.0);
        cJSON_AddBoolToObject(control, "emergency_stop", control_emergency_stop ? 1 : 0);
        cJSON_AddItemToObject(root, "control", control);

        cJSON* obstacles = cJSON_CreateArray();
        cJSON_AddItemToArray(obstacles, build_obstacle_json(front0));
        cJSON_AddItemToArray(obstacles, build_obstacle_json(front1));
        cJSON_AddItemToObject(root, "obstacles", obstacles);

        char* s = cJSON_PrintUnformatted(root);
        fprintf(g.out, "%s\n", s);
        free(s);
        cJSON_Delete(root);
        fflush(g.out);
        g.sample_count++;

        if (g.sample_count % 50 == 1) {
            LOG_INFO("recorder", "#%d sample: v=%.1f y=%.2f → label(target_speed)=%.1f",
                     g.sample_count, g.ego_v, g.ego_y, g.planning_target_speed);
        }
    }

    LOG_INFO("recorder", "stopped (%d samples → %s)", g.sample_count, g.out_path);
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return NULL;
}

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", "perception/obstacles", "control/cmd", NULL };
static const char* s_outputs[] = { NULL };

static NodePlugin s_plugin;  /* forward decl */

static int recorder_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;
    g.should_stop = 0;

    g.cfg_frequency_hz = 10.0;
    strncpy(g.out_path, "/tmp/flow_train_samples.jsonl", sizeof(g.out_path) - 1);

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "frequency_hz")) && cJSON_IsNumber(j))
                g.cfg_frequency_hz = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "output_path")) && cJSON_IsString(j))
                strncpy(g.out_path, j->valuestring, sizeof(g.out_path) - 1);
            cJSON_Delete(p);
        }
    }

    g.out = fopen(g.out_path, "w");
    if (!g.out) {
        LOG_ERROR("recorder", "cannot open output file %s", g.out_path);
        return -1;
    }

    transport_subscribe(transport, "fusion/localization", on_fusion, NULL);
    transport_subscribe(transport, "planning/trajectory", on_planning, NULL);
    transport_subscribe(transport, "perception/obstacles", on_obstacles, NULL);
    transport_subscribe(transport, "control/cmd", on_control, NULL);

    discovery_advertise(discovery, "fusion/localization", 0xF0ED10C0u,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "planning/trajectory", 0x3A7B1C2Du,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "perception/obstacles", 0x0B5A010Eu,
                        CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/cmd", 0x2D95C6D2u,
                        CAP_SUBSCRIBER, 0);

    statem_init(&g.sm, NULL, SM_STATE_INITIALIZED, "recorder");
    statem_send_event(&g.sm, SM_EVENT_START, NULL);

    LOG_INFO("recorder", "initialized (%.0f Hz → %s)",
             g.cfg_frequency_hz, g.out_path);
    return 0;
}

static int recorder_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, recorder_thread, NULL) != 0) {
        LOG_WARN("recorder", "pthread_create failed: %s", strerror(errno));
        g.running = 0;
        return -1;
    }
    LOG_INFO("recorder", "started [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void recorder_stop(void) { g.should_stop = 1; }

static void recorder_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    if (g.out) { fclose(g.out); g.out = NULL; }
    LOG_INFO("recorder", "cleanup done");
}

static int recorder_health(void) { return g.out ? 0 : -1; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "data_recorder",
    .version       = "1.0.0",
    .description   = "Training sample recorder (imitation learning JSONL)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = recorder_init,
    .start         = recorder_start,
    .stop          = recorder_stop,
    .cleanup       = recorder_cleanup,
    .health        = recorder_health,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
