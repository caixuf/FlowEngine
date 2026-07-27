/**
 * guardian_node.c — 独立安全守护节点 (Guardian Safety Watchdog)
 *
 * Apollo Guardian 的简化实现：
 *   1. 独立进程/线程，不依赖 planning/control 正常运转
 *   2. 直接订阅关键 safety-critical topics
 *   3. 多层级安全检查：
 *      - Level 1: 碰撞风险 (TTC < 阈值)
 *      - Level 2: 车道偏离 (CTE > 阈值)
 *      - Level 3: 速度超限 (speed > max_speed)
 *      - Level 4: 控制指令合理性 (steer/throttle/brake 物理边界)
 *      - Level 5: 传感器/定位心跳超时
 *   4. 独立发布紧急制动指令 (control/emergency_stop)
 *   5. 记录所有安全事件到日志
 *
 * 与 control_node 的关系：
 *   - control_node 发布 control/raw_cmd（正常控制指令）
 *   - guardian_node 订阅 control/raw_cmd 并校验合理性
 *   - 若 control_node 失灵或输出危险指令，guardian 发布 control/emergency_stop
 *   - actuator_node 应优先响应 emergency_stop 而非 raw_cmd
 *
 * NodePlugin 接口，编译为 libguardian_node.so。
 */

#include "node_plugin.h"
#include "transport.h"
#include "discovery.h"
#include "topic_registry.h"
#include "logger.h"
#include "clock_service.h"
#include "health.h"
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

#define GUARDIAN_DEFAULT_HZ      20.0f  /**< 检查频率 */
#define GUARDIAN_CHECK_INTERVAL_US  (long)(1000000.0 / GUARDIAN_DEFAULT_HZ)

/* ── 安全阈值（可配置） ──────────────────────────────────────── */

#define GUARDIAN_DEFAULT_TTC_S       1.5   /**< TTC 低于此值触发紧急制动 */
#define GUARDIAN_DEFAULT_MAX_SPEED   35.0  /**< 最高允许速度 (m/s) */
#define GUARDIAN_DEFAULT_MAX_STEER   0.50  /**< 最大转向角 (rad) ~28.6° */
#define GUARDIAN_DEFAULT_MAX_LAT_ERR 2.5   /**< 最大横向误差 (m) */
#define GUARDIAN_DEFAULT_TIMEOUT_MS  500   /**< 传感器/定位心跳超时 (ms) */
#define GUARDIAN_DEFAULT_BRAKE_COOLDOWN_S 1.0 /**< 连续紧急制动冷却 (s) */

/* ── 安全事件类型 ────────────────────────────────────────────── */

typedef enum {
    GUARDIAN_EVENT_NONE         = 0,
    GUARDIAN_EVENT_COLLISION    = 1,  /**< 碰撞风险 */
    GUARDIAN_EVENT_LANE_DEPART  = 2,  /**< 车道偏离 */
    GUARDIAN_EVENT_OVERSPEED    = 3,  /**< 超速 */
    GUARDIAN_EVENT_STEER_LIMIT  = 4,  /**< 转向超限 */
    GUARDIAN_EVENT_TIMEOUT      = 5,  /**< 心跳超时 */
    GUARDIAN_EVENT_CMD_SANITY   = 6,  /**< 控制指令不合理 */
} GuardianEventType;

/* ── Module state ─────────────────────────────────────────────── */

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;
    MessageBus*       bus;

    TaskBase          taskbase;
    volatile int      running;
    volatile int      should_stop;
    pthread_t         thread;

    /* ── 订阅数据缓冲 ── */
    /* 自车状态 */
    double  ego_x, ego_y, ego_heading, ego_speed, ego_yaw_rate;
    volatile int has_ego_state;
    uint64_t last_ego_state_us;

    /* 障碍物 */
    struct { double x, y, vx, vy; int valid; } obstacles[8];
    volatile int has_obstacles;
    uint64_t last_obstacles_us;

    /* 控制指令 */
    double  cmd_throttle, cmd_brake, cmd_steer, cmd_speed;
    volatile int has_cmd;
    uint64_t last_cmd_us;

    /* 融合定位 */
    double  loc_x, loc_y, loc_heading, loc_speed;
    volatile int has_loc;
    uint64_t last_loc_us;

    /* 车道线 */
    double  lane_left_y, lane_right_y;
    volatile int has_lanes;
    uint64_t last_lanes_us;

    /* ── 安全状态 ── */
    int     emergency_active;         /**< 紧急制动是否激活 */
    double  emergency_brake_time;     /**< 紧急制动开始时间 (s) */
    double  last_event_time;          /**< 上次事件时间 (s) */
    GuardianEventType last_event;     /**< 上次事件类型 */

    /* ── 配置参数 ── */
    double  ttc_threshold_s;          /**< TTC 阈值 */
    double  max_speed_mps;            /**< 最高速度 */
    double  max_steer_rad;            /**< 最大转向角 */
    double  max_lat_error_m;          /**< 最大横向误差 */
    double  timeout_ms;               /**< 心跳超时 */
    double  brake_cooldown_s;         /**< 紧急制动冷却 */
    double  rate_hz;

    /* ── 健康上报 ── */
    char health_name[32];

    pthread_mutex_t mutex;
} g;

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_ego_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    pthread_mutex_lock(&g.mutex);
    cJSON* j;
    j = cJSON_GetObjectItemCaseSensitive(root, "v");
    if (cJSON_IsNumber(j)) g.ego_speed = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "x");
    if (cJSON_IsNumber(j)) g.ego_x = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "y");
    if (cJSON_IsNumber(j)) g.ego_y = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "heading");
    if (cJSON_IsNumber(j)) g.ego_heading = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "yaw_rate");
    if (cJSON_IsNumber(j)) g.ego_yaw_rate = j->valuedouble;
    g.has_ego_state = 1;
    g.last_ego_state_us = clock_now_us();
    pthread_mutex_unlock(&g.mutex);
    cJSON_Delete(root);
}

static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    pthread_mutex_lock(&g.mutex);
    for (int i = 0; i < 8; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ox%d", i);
        cJSON* j = cJSON_GetObjectItemCaseSensitive(root, key);
        if (cJSON_IsNumber(j)) {
            g.obstacles[i].x = j->valuedouble;
            g.obstacles[i].valid = 1;
        } else { g.obstacles[i].valid = 0; continue; }
        snprintf(key, sizeof(key), "oy%d", i);
        j = cJSON_GetObjectItemCaseSensitive(root, key);
        if (cJSON_IsNumber(j)) g.obstacles[i].y = j->valuedouble;
        snprintf(key, sizeof(key), "ov%d", i);
        j = cJSON_GetObjectItemCaseSensitive(root, key);
        if (cJSON_IsNumber(j)) g.obstacles[i].vx = j->valuedouble;
        snprintf(key, sizeof(key), "ovy%d", i);
        j = cJSON_GetObjectItemCaseSensitive(root, key);
        if (cJSON_IsNumber(j)) g.obstacles[i].vy = j->valuedouble;
    }
    g.has_obstacles = 1;
    g.last_obstacles_us = clock_now_us();
    pthread_mutex_unlock(&g.mutex);
    cJSON_Delete(root);
}

static void on_control_cmd(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    /* control/raw_cmd/text 是文本格式 */
    const char* d = (const char*)msg->data;
    pthread_mutex_lock(&g.mutex);
    sscanf(d, "throttle=%lf brake=%lf steer=%lf speed=%lf",
           &g.cmd_throttle, &g.cmd_brake, &g.cmd_steer, &g.cmd_speed);
    g.has_cmd = 1;
    g.last_cmd_us = clock_now_us();
    pthread_mutex_unlock(&g.mutex);
}

static void on_localization(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;

    pthread_mutex_lock(&g.mutex);
    cJSON* j;
    j = cJSON_GetObjectItemCaseSensitive(root, "x");
    if (cJSON_IsNumber(j)) g.loc_x = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "y");
    if (cJSON_IsNumber(j)) g.loc_y = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "heading");
    if (cJSON_IsNumber(j)) g.loc_heading = j->valuedouble;
    j = cJSON_GetObjectItemCaseSensitive(root, "v");
    if (cJSON_IsNumber(j)) g.loc_speed = j->valuedouble;
    g.has_loc = 1;
    g.last_loc_us = clock_now_us();
    pthread_mutex_unlock(&g.mutex);
    cJSON_Delete(root);
}

/* ── 安全检查函数 ────────────────────────────────────────────── */

/**
 * Level 1: TTC 碰撞检查。
 * 对每个前方障碍物计算 TTC (Time-To-Collision)，
 * 若 TTC < ttc_threshold_s 则触发紧急制动。
 */
static int check_collision(double* ttc_out) {
    if (!g.has_ego_state || !g.has_obstacles) return 0;
    *ttc_out = 999.0;
    int risk = 0;

    for (int i = 0; i < 8; i++) {
        if (!g.obstacles[i].valid) continue;
        double dx = g.obstacles[i].x - g.ego_x;
        if (dx <= 0.0) continue;  /* 后方 */
        if (fabs(g.obstacles[i].y - g.ego_y) > 3.0) continue;  /* 不在同车道 */

        double rel_speed = g.ego_speed - g.obstacles[i].vx;
        if (rel_speed <= 0.5) continue;  /* 无碰撞风险 */

        double ttc = dx / rel_speed;
        if (ttc < *ttc_out) *ttc_out = ttc;
        if (ttc < g.ttc_threshold_s) risk = 1;
    }
    return risk;
}

/**
 * Level 2: 车道偏离检查。
 * 若没有 lane 数据，检查 ego_y 是否偏离合理范围。
 */
static int check_lane_departure(double* cte_out) {
    if (!g.has_ego_state) return 0;
    *cte_out = 0.0;

    if (g.has_lanes) {
        /* 有车道线数据：检查 ego_y 是否在 lane_left 和 lane_right 之间 */
        double mid = (g.lane_left_y + g.lane_right_y) * 0.5;
        *cte_out = g.ego_y - mid;
    } else {
        /* 无车道线：检查 ego_y 是否偏离道路中心太远 */
        *cte_out = g.ego_y;  /* 假设道路中心 y=0 */
    }

    return (fabs(*cte_out) > g.max_lat_error_m) ? 1 : 0;
}

/**
 * Level 3: 速度超限检查。
 */
static int check_overspeed(double* speed_out) {
    if (!g.has_ego_state) return 0;
    *speed_out = g.ego_speed;
    return (g.ego_speed > g.max_speed_mps) ? 1 : 0;
}

/**
 * Level 4: 控制指令合理性检查。
 * 转向角、油门、刹车是否在物理可行范围内。
 */
static int check_cmd_sanity(void) {
    if (!g.has_cmd) return 0;

    /* 转向角超限 */
    if (fabs(g.cmd_steer) > g.max_steer_rad) return 1;

    /* 油门 + 刹车同时非零 */
    if (g.cmd_throttle > 0.1 && g.cmd_brake > 0.1) return 1;

    /* 油门 > 1.0 或 刹车 > 1.0 */
    if (g.cmd_throttle > 1.01 || g.cmd_brake > 1.01) return 1;

    /* 高速时大转向角 */
    if (g.cmd_speed > 20.0 && fabs(g.cmd_steer) > 0.25) return 1;

    return 0;
}

/**
 * Level 5: 传感器/定位心跳超时检查。
 */
static int check_timeout(void) {
    uint64_t now = clock_now_us();
    double timeout_us = g.timeout_ms * 1000.0;

    /* 自车状态超时 */
    if (g.last_ego_state_us > 0 &&
        (now - g.last_ego_state_us) > (uint64_t)timeout_us) {
        return 1;
    }

    /* 融合定位超时 */
    if (g.last_loc_us > 0 &&
        (now - g.last_loc_us) > (uint64_t)timeout_us) {
        return 1;
    }

    /* 控制指令超时（控制节点可能挂了） */
    if (g.last_cmd_us > 0 &&
        (now - g.last_cmd_us) > (uint64_t)(timeout_us * 3.0)) {
        return 1;
    }

    return 0;
}

/* ── 发布紧急制动 ────────────────────────────────────────────── */

static void publish_emergency_stop(GuardianEventType event, const char* reason) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "emergency_stop", 1);
    cJSON_AddNumberToObject(root, "event_type", (double)event);
    cJSON_AddStringToObject(root, "reason", reason);
    cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());

    /* 也发布标准 ControlCmd 格式，让 actuator 可以直接处理 */
    cJSON* cmd = cJSON_CreateObject();
    cJSON_AddNumberToObject(cmd, "throttle", 0.0);
    cJSON_AddNumberToObject(cmd, "brake", 1.0);
    cJSON_AddNumberToObject(cmd, "steering", 0.0);
    cJSON_AddNumberToObject(cmd, "gear", 1);  /* GEAR_DRIVE */
    cJSON_AddBoolToObject(cmd, "emergency_stop", true);
    cJSON_AddItemToObject(root, "control_cmd", cmd);

    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        transport_publish(g.transport, "control/emergency_stop",
                          (const uint8_t*)json_str, (uint32_t)strlen(json_str) + 1);
        free(json_str);
    }

    cJSON_Delete(root);
}

/* ── 主循环 ──────────────────────────────────────────────────── */

static int guardian_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "guardian");

    const long period_us = (long)(1000000.0 / g.rate_hz);

    while (!task->should_stop) {
        usleep(period_us);
        if (task->should_stop) break;

        double now_s = (double)clock_now_us() * 1e-6;
        int triggered = 0;
        const char* reason = "unknown";
        GuardianEventType event = GUARDIAN_EVENT_NONE;
        double ttc = 999.0, cte = 0.0, speed = 0.0;

        pthread_mutex_lock(&g.mutex);

        /* ── Level 1: 碰撞检查 ── */
        if (check_collision(&ttc)) {
            triggered = 1;
            event = GUARDIAN_EVENT_COLLISION;
            reason = "collision_risk";
        }

        /* ── Level 2: 车道偏离 ── */
        if (!triggered && check_lane_departure(&cte)) {
            /* 车道偏离 + 高速 → 紧急 */
            if (g.ego_speed > 10.0) {
                triggered = 1;
                event = GUARDIAN_EVENT_LANE_DEPART;
                reason = "lane_departure";
            }
        }

        /* ── Level 3: 超速 ── */
        if (!triggered && check_overspeed(&speed)) {
            triggered = 1;
            event = GUARDIAN_EVENT_OVERSPEED;
            reason = "overspeed";
        }

        /* ── Level 4: 控制指令合理性 ── */
        if (!triggered && check_cmd_sanity()) {
            triggered = 1;
            event = GUARDIAN_EVENT_CMD_SANITY;
            reason = "cmd_sanity";
        }

        /* ── Level 5: 心跳超时 ── */
        if (!triggered && check_timeout()) {
            triggered = 1;
            event = GUARDIAN_EVENT_TIMEOUT;
            reason = "sensor_timeout";
        }

        pthread_mutex_unlock(&g.mutex);

        /* ── 发布紧急制动 ── */
        if (triggered) {
            /* 冷却期检查：避免连续触发 */
            if (now_s - g.last_event_time > g.brake_cooldown_s ||
                event != g.last_event) {
                g.emergency_active = 1;
                g.emergency_brake_time = now_s;
                g.last_event = event;
                g.last_event_time = now_s;

                publish_emergency_stop(event, reason);

                /* 记录到健康系统 */
                health_record_error(g.health_name, reason);

                LOG_WARN("guardian",
                         "!!! EMERGENCY STOP: %s (event=%d, ttc=%.2f, cte=%.2f, speed=%.1f)",
                         reason, (int)event, ttc, cte, speed);
            }
        } else {
            /* 风险解除 */
            if (g.emergency_active && now_s - g.emergency_brake_time > 2.0) {
                g.emergency_active = 0;
                LOG_INFO("guardian", "emergency cleared, resuming normal operation");
            }
        }

        /* 心跳 */
        health_heartbeat(g.health_name);
    }

    LOG_INFO("guardian", "stopped");
    return 0;
}

/* ── TaskInterface ────────────────────────────────────────────── */

static const TaskInterface guardian_vtable = {
    .execute = guardian_execute,
};

/* ── NodePlugin lifecycle ────────────────────────────────────── */

static const char* s_inputs[] = {
    TOPIC_VEHICLE_STATE,
    TOPIC_CONTROL_RAW_CMD_TEXT,
    TOPIC_FUSION_LOCALIZATION,
    "perception/obstacles",
    NULL
};
static const char* s_outputs[] = { "control/emergency_stop", NULL };

static NodePlugin s_plugin;

static int guardian_init(MessageBus* bus, Transport* transport,
                         DiscoveryManager* discovery, Scheduler* scheduler,
                         const char* params_json) {
    (void)bus;

    memset(&g, 0, sizeof(g));
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;
    g.bus         = bus;

    /* 默认参数 */
    g.ttc_threshold_s   = GUARDIAN_DEFAULT_TTC_S;
    g.max_speed_mps     = GUARDIAN_DEFAULT_MAX_SPEED;
    g.max_steer_rad     = GUARDIAN_DEFAULT_MAX_STEER;
    g.max_lat_error_m   = GUARDIAN_DEFAULT_MAX_LAT_ERR;
    g.timeout_ms        = GUARDIAN_DEFAULT_TIMEOUT_MS;
    g.brake_cooldown_s  = GUARDIAN_DEFAULT_BRAKE_COOLDOWN_S;
    g.rate_hz           = GUARDIAN_DEFAULT_HZ;

    /* 解析参数 */
    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "ttc_threshold_s")) && cJSON_IsNumber(j))
                g.ttc_threshold_s = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "max_speed_mps")) && cJSON_IsNumber(j))
                g.max_speed_mps = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "max_steer_rad")) && cJSON_IsNumber(j))
                g.max_steer_rad = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "max_lat_error_m")) && cJSON_IsNumber(j))
                g.max_lat_error_m = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "timeout_ms")) && cJSON_IsNumber(j))
                g.timeout_ms = j->valuedouble;
            if ((j = cJSON_GetObjectItemCaseSensitive(p, "rate_hz")) && cJSON_IsNumber(j))
                g.rate_hz = j->valuedouble;
            cJSON_Delete(p);
        }
    }

    /* 订阅 */
    transport_subscribe(transport, TOPIC_VEHICLE_STATE, on_ego_state, NULL);
    transport_subscribe(transport, TOPIC_CONTROL_RAW_CMD_TEXT, on_control_cmd, NULL);
    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION, on_localization, NULL);
    transport_subscribe(transport, "perception/obstacles", on_obstacles, NULL);
    transport_advertise(transport, "control/emergency_stop", 0);

    /* Discovery */
    discovery_advertise(discovery, TOPIC_VEHICLE_STATE, 0x1C0E5A7Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_CONTROL_RAW_CMD_TEXT, 0, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "perception/obstacles", 0x308F5F71u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "control/emergency_stop", 0, CAP_PUBLISHER, g.rate_hz);

    pthread_mutex_init(&g.mutex, NULL);

    /* 健康注册 */
    snprintf(g.health_name, sizeof(g.health_name), "guardian");
    health_register(g.health_name, HEALTH_CAP_SAFETY_CRITICAL);

    /* 托管模式 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "guardian");
    cfg.priority         = TASK_PRIORITY_HIGH;  /* 安全节点优先级最高 */
    cfg.max_frequency_hz = g.rate_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &guardian_vtable, &cfg) != 0) {
        LOG_WARN("guardian", "task_base_init failed");
        return -1;
    }

    LOG_INFO("guardian", "initialized (%.1f Hz, TTC=%.1fs, max_speed=%.1f, max_steer=%.2f)",
             g.rate_hz, g.ttc_threshold_s, g.max_speed_mps, g.max_steer_rad);
    return 0;
}

static int guardian_start(void) {
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("guardian", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("guardian", "started (managed, HIGH priority)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void guardian_stop(void) {
    task_stop(&g.taskbase);
}

static void guardian_cleanup(void) {
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    pthread_mutex_destroy(&g.mutex);
    /* health_unregister not needed — health system uses name-based lookup */
    LOG_INFO("guardian", "cleanup done");
}

static int guardian_health(void) {
    return g.emergency_active ? 1 : 0;
}

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "guardian",
    .version       = "1.0.0",
    .description   = "Independent safety watchdog — collision/lane/overspeed/CMD sanity checks",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = guardian_init,
    .start         = guardian_start,
    .stop          = guardian_stop,
    .cleanup       = guardian_cleanup,
    .health        = guardian_health,
    .taskbase      = &g.taskbase,
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }