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
 *     场景特征训练使用；v3 样本额外写入 features_v3(23维) + scene_context，
 *     含红绿灯/道路几何场景上下文。features 保持 v1 兼容。
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
#include <math.h>

#define RECORDER_MAX_OBSTACLES 128

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 recorder_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase   taskbase;

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

    /* v3 特征: 场景上下文（来自 road/traffic_lights + road/geometry） */
    double tl_state;        /* -1=无, 0=绿, 1=黄, 2=红 */
    double tl_distance;     /* 距最近灯距离 (m), -1=无 */
    double road_curvature;  /* 当前曲率 (1/R) */
    double road_speed_limit;
    double lane_count;
    double lane_width;
    double ego_lane_offset;
    volatile int has_scene;

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

/* ── v3: 红绿灯状态回调 ─────────────────────────────────────── */
static void on_traffic_lights(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* lights = cJSON_GetObjectItem(root, "lights");
    if (!cJSON_IsArray(lights)) { cJSON_Delete(root); return; }

    double nearest_red = 1e9;
    int has_red = 0;
    cJSON* light;
    cJSON_ArrayForEach(light, lights) {
        cJSON* s = cJSON_GetObjectItem(light, "state");
        cJSON* x = cJSON_GetObjectItem(light, "x");
        if (!cJSON_IsString(s) || !cJSON_IsNumber(x)) continue;
        double dist = x->valuedouble - g.ego_x;
        if (dist < 0) continue;
        const char* state = s->valuestring;
        if (strcmp(state, "red") == 0) {
            if (dist < nearest_red) { nearest_red = dist; has_red = 1; }
        } else if (strcmp(state, "yellow") == 0) {
            if (!has_red && dist < nearest_red) nearest_red = dist;
        }
        if (!has_red && g.tl_state < 0.0 && strcmp(state, "green") == 0) {
            g.tl_state = 0.0; g.tl_distance = dist;
        }
    }
    if (has_red) { g.tl_state = 2.0; g.tl_distance = nearest_red; }
    else if (g.tl_state < 0.0) g.tl_state = 0.0;
    g.has_scene = 1;
    cJSON_Delete(root);
}

/* ── v3: 道路几何回调 ───────────────────────────────────────── */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* j;
    j = cJSON_GetObjectItem(root, "lane_width");
    if (cJSON_IsNumber(j)) g.lane_width = j->valuedouble;
    j = cJSON_GetObjectItem(root, "lane_count");
    if (cJSON_IsNumber(j)) g.lane_count = j->valuedouble;
    j = cJSON_GetObjectItem(root, "speed_limit");
    if (cJSON_IsNumber(j)) g.road_speed_limit = j->valuedouble;
    j = cJSON_GetObjectItem(root, "curve_offset_m");
    if (cJSON_IsNumber(j)) {
        double off = j->valuedouble;
        j = cJSON_GetObjectItem(root, "curve_length_m");
        double len = cJSON_IsNumber(j) ? j->valuedouble : 0.0;
        if (len > 1.0) g.road_curvature = 2.0 * fabs(off) / (len * len);
    }
    g.has_scene = 1;
    cJSON_Delete(root);
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

/* ── 托管模式主循环：按 cfg_frequency_hz 采样并落盘训练样本 ──
 *
 * task_thread_fn 调用本函数一次（完整主循环），循环中检查 task->should_stop
 * 退出；task_stop() 置 should_stop=true 并 join 本线程。这与原先自管 pthread
 * 的 recorder_thread 行为等价，只是 should_stop 改由 TaskBase 提供。 */
static int recorder_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "recorder");

    double period = g.cfg_frequency_hz > 0.0 ? 1.0 / g.cfg_frequency_hz : 0.1;
    useconds_t sleep_us = (useconds_t)(period * 1e6);

    while (!task->should_stop) {
        usleep(sleep_us);
        if (task->should_stop) break;
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

        /* C-3 修复：features_v2 必须为 16 维，含 front1_confidence。
         * 维度契约（与 tools/train_e2e/feature_schema.py 一致）：
         *   0 ego_v, 1 ego_y, 2 ego_heading, 3 ego_yaw_rate,
         *   4 front0_x, 5 front0_y, 6 front0_vx, 7 front0_type, 8 front0_confidence,
         *   9 front1_x, 10 front1_y, 11 front1_vx, 12 front1_type, 13 front1_confidence,
         *   14 control_brake, 15 control_emergency_stop
         * 旧 15 维数据缺 front1_confidence，导致索引错位；temporal_train.py 已改为
         * 维度不匹配硬报错，禁止静默补零混训。 */
        {   double arr[] = {
                g.ego_v, g.ego_y, g.ego_heading, g.ego_yaw_rate,
                front0 ? front0->x : 0.0, front0 ? front0->y : 0.0, front0 ? front0->vx : 0.0,
                (double)(front0 ? (int)front0->type : 0), front0 ? front0->confidence : 0.0,
                front1 ? front1->x : 0.0, front1 ? front1->y : 0.0, front1 ? front1->vx : 0.0,
                (double)(front1 ? (int)front1->type : 0), front1 ? front1->confidence : 0.0,
                control_brake, (double)control_emergency_stop };
            cJSON_AddItemToObject(root, "features_v2", cJSON_CreateDoubleArray(arr, 16)); }

        /* v3: features_v3 = v2 基础 16 维 + 场景上下文 7 维 = 23 维
         * 维度契约（与 tools/train_e2e/feature_schema.py FEATURE_NAMES_V3 一致）：
         *   0-15: 同 features_v2
         *   16 tl_state, 17 tl_distance, 18 road_curvature,
         *   19 road_speed_limit, 20 lane_count, 21 lane_width, 22 ego_lane_offset */
        g.ego_lane_offset = g.ego_y;
        {   double v3[23] = {
                g.ego_v, g.ego_y, g.ego_heading, g.ego_yaw_rate,
                front0 ? front0->x : 0.0, front0 ? front0->y : 0.0, front0 ? front0->vx : 0.0,
                (double)(front0 ? (int)front0->type : 0), front0 ? front0->confidence : 0.0,
                front1 ? front1->x : 0.0, front1 ? front1->y : 0.0, front1 ? front1->vx : 0.0,
                (double)(front1 ? (int)front1->type : 0), front1 ? front1->confidence : 0.0,
                control_brake, (double)control_emergency_stop,
                g.tl_state, g.tl_distance, g.road_curvature,
                g.road_speed_limit, g.lane_count, g.lane_width, g.ego_lane_offset };
            cJSON_AddItemToObject(root, "features_v3", cJSON_CreateDoubleArray(v3, 23)); }

        /* scene_context 供 PyTorch 训练侧 build_v3_features 重建特征 */
        cJSON* scene_ctx = cJSON_CreateObject();
        cJSON_AddNumberToObject(scene_ctx, "tl_state", g.tl_state);
        cJSON_AddNumberToObject(scene_ctx, "tl_distance", g.tl_distance);
        cJSON_AddNumberToObject(scene_ctx, "curvature", g.road_curvature);
        cJSON_AddNumberToObject(scene_ctx, "speed_limit", g.road_speed_limit);
        cJSON_AddNumberToObject(scene_ctx, "lane_count", g.lane_count);
        cJSON_AddNumberToObject(scene_ctx, "lane_width", g.lane_width);
        cJSON_AddNumberToObject(scene_ctx, "ego_lane_offset", g.ego_lane_offset);
        cJSON_AddItemToObject(root, "scene_context", scene_ctx);

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

        /* ── instant_reward（用于 learner reward-weighted 训练）── */
        {
            double speed = g.ego_v;
            double cte = g.ego_y;  /* 直路场景 ≈ CTE */
            double steer = g.has_control ? g.control.steering : 0.0;
            double r = 0.5;
            if (speed >= 8.0 && speed <= 15.0) r += 0.2;
            else if (speed < 2.0)              r -= 0.3;
            double acte = fabs(cte);
            if      (acte < 0.3) r += 0.2;
            else if (acte < 0.8) r += 0.1;
            else if (acte > 2.0) r -= 0.3;
            if (fabs(steer) < 0.05) r += 0.1;
            else if (fabs(steer) > 0.15) r -= 0.1;
            if (r < 0.0) r = 0.0;
            if (r > 1.0) r = 1.0;
            cJSON_AddNumberToObject(root, "reward", r);
        }

        char* s = cJSON_PrintUnformatted(root);
        fprintf(g.out, "%s\n", s);
        free(s);
        cJSON_Delete(root);
        g.sample_count++;

        if (g.sample_count % 50 == 1) {
            LOG_INFO("recorder", "#%d sample: v=%.1f y=%.2f → label(target_speed)=%.1f",
                     g.sample_count, g.ego_v, g.ego_y, g.planning_target_speed);
        }
    }

    LOG_INFO("recorder", "stopped (%d samples → %s)", g.sample_count, g.out_path);
    statem_send_event(&g.sm, SM_EVENT_STOP, NULL);
    statem_send_event(&g.sm, SM_EVENT_DONE, NULL);
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整采样主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface recorder_vtable = {
    .execute = recorder_execute,
};

static const char* s_inputs[]  = { "fusion/localization", "planning/trajectory", "perception/obstacles", "control/cmd", "road/traffic_lights", "road/geometry", NULL };
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

    g.cfg_frequency_hz = 10.0;
    strncpy(g.out_path, "/tmp/flow_train_samples.jsonl", sizeof(g.out_path) - 1);

    /* v3 场景上下文默认值（与 inference_node.cpp 一致） */
    g.tl_state = -1.0;
    g.tl_distance = -1.0;
    g.road_curvature = 0.0;
    g.road_speed_limit = 30.0;
    g.lane_count = 2.0;
    g.lane_width = 3.5;
    g.ego_lane_offset = 0.0;

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
    transport_subscribe(transport, "road/traffic_lights", on_traffic_lights, NULL);
    transport_subscribe(transport, "road/geometry", on_road_geometry, NULL);

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

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 喂给调度器 RateControl，与 execute() 内 usleep 周期一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "data_recorder");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = g.cfg_frequency_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &recorder_vtable, &cfg) != 0) {
        LOG_WARN("data_recorder", "task_base_init failed");
        return -1;
    }
    return 0;
}

static int recorder_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * recorder_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("recorder", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("recorder", "started (managed) [state=%s]", statem_state_name(&g.sm, g.sm.current));
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void recorder_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（recorder_execute 随即退出）。
     * launcher 保证 stop() 在 cleanup() 前调用，故此处阻塞 join 是安全的。 */
    task_stop(&g.taskbase);
}

static void recorder_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源（互斥锁等）。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
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
    .taskbase      = &g.taskbase,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
