/**
 * navigation_node.c — 导航路线节点（单轨）
 *
 * 职责：
 *   - 从 scenario_file 加载 route[] 导航步骤
 *   - 订阅 fusion/localization，按 ego_x 触发 route step
 *   - 发布 navigation/path（JSON），供 planning_node 唯一消费
 *
 * 输出消息：
 *   route_status:
 *     {"type":"route_status","route_count":N,"next_idx":k,...}
 *   route_step:
 *     {"type":"route_step","step_index":k,"step_type":"merge|branch_select|lane_change",
 *      "target_lane":...,"target_speed":...,"branch_id":...,"trigger_x":...}
 */

#include "node_plugin.h"
#include "topic_registry.h"
#include "scenario_loader.h"
#include "clock_service.h"
#include "logger.h"
#include <cjson/cJSON.h>

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define NAV_DEFAULT_RATE_HZ 10.0

static struct {
    Transport* transport;
    DiscoveryManager* discovery;
    Scheduler* scheduler;
    TaskBase taskbase;

    pthread_mutex_t mu;
    double ego_x;
    int has_fusion;

    ScenarioRouteStep route[SCENARIO_MAX_ROUTE_STEPS];
    int route_count;
    int next_idx;
    uint32_t seq;

    double rate_hz;
    char scenario_file[256];
    uint64_t last_status_us;
} g;

static const char* step_type_to_str(RouteStepType t) {
    switch (t) {
        case ROUTE_BRANCH_SELECT: return "branch_select";
        case ROUTE_MERGE: return "merge";
        case ROUTE_LANE_CHANGE:
        default: return "lane_change";
    }
}

static void publish_route_status(void) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "route_status");
    cJSON_AddNumberToObject(root, "route_count", g.route_count);
    cJSON_AddNumberToObject(root, "next_idx", g.next_idx);
    cJSON_AddNumberToObject(root, "ego_x", g.ego_x);
    cJSON_AddNumberToObject(root, "seq", (double)g.seq++);
    cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());
    char* s = cJSON_PrintUnformatted(root);
    if (s) {
        transport_publish(g.transport, TOPIC_NAVIGATION_PATH, (const uint8_t*)s, (uint32_t)strlen(s) + 1);
        free(s);
    }
    cJSON_Delete(root);
}

static void publish_route_step(const ScenarioRouteStep* step, int step_index, double ego_x) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "route_step");
    cJSON_AddNumberToObject(root, "step_index", step_index);
    cJSON_AddNumberToObject(root, "route_count", g.route_count);
    cJSON_AddStringToObject(root, "step_type", step_type_to_str(step->type));
    cJSON_AddNumberToObject(root, "trigger_x", step->trigger_x);
    cJSON_AddNumberToObject(root, "ego_x", ego_x);
    cJSON_AddNumberToObject(root, "target_lane", step->target_lane);
    cJSON_AddNumberToObject(root, "target_speed", step->target_speed);
    cJSON_AddNumberToObject(root, "branch_id", step->branch_id);
    if (step->label[0] != '\0') cJSON_AddStringToObject(root, "label", step->label);
    cJSON_AddNumberToObject(root, "seq", (double)g.seq++);
    cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());
    char* s = cJSON_PrintUnformatted(root);
    if (s) {
        transport_publish(g.transport, TOPIC_NAVIGATION_PATH, (const uint8_t*)s, (uint32_t)strlen(s) + 1);
        free(s);
    }
    cJSON_Delete(root);
}

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* jx = cJSON_GetObjectItemCaseSensitive(root, "x");
    if (cJSON_IsNumber(jx)) {
        pthread_mutex_lock(&g.mu);
        g.ego_x = jx->valuedouble;
        g.has_fusion = 1;
        pthread_mutex_unlock(&g.mu);
    }
    cJSON_Delete(root);
}

static int navigation_execute(TaskBase* task) {
    const long period_us = (long)(1000000.0 / g.rate_hz);
    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

        double ego_x = 0.0;
        int has_fusion = 0;
        pthread_mutex_lock(&g.mu);
        ego_x = g.ego_x;
        has_fusion = g.has_fusion;
        pthread_mutex_unlock(&g.mu);

        if (has_fusion && g.route_count > 0) {
            while (g.next_idx < g.route_count && ego_x >= g.route[g.next_idx].trigger_x) {
                const ScenarioRouteStep* step = &g.route[g.next_idx];
                publish_route_step(step, g.next_idx, ego_x);
                LOG_INFO("navigation", "route step #%d triggered @x=%.1f type=%s lane=%d speed=%.1f",
                         g.next_idx, ego_x, step_type_to_str(step->type),
                         step->target_lane, step->target_speed);
                g.next_idx++;
            }
        }

        uint64_t now_us = clock_now_us();
        if (now_us - g.last_status_us >= 1000000ULL) {
            g.last_status_us = now_us;
            publish_route_status();
        }
    }
    return 0;
}

static const TaskInterface nav_vtable = {
    .execute = navigation_execute,
};

static const char* s_inputs[] = {
    TOPIC_FUSION_LOCALIZATION,
    NULL
};
static const char* s_outputs[] = {
    TOPIC_NAVIGATION_PATH,
    NULL
};

static NodePlugin s_plugin;

static int navigation_init(MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler,
                           const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport = transport;
    g.discovery = discovery;
    g.scheduler = scheduler;
    g.rate_hz = NAV_DEFAULT_RATE_HZ;
    pthread_mutex_init(&g.mu, NULL);

    if (params_json) {
        cJSON* p = cJSON_Parse(params_json);
        if (p) {
            cJSON* j;
            j = cJSON_GetObjectItemCaseSensitive(p, "rate_hz");
            if (cJSON_IsNumber(j) && j->valuedouble > 0.1) g.rate_hz = j->valuedouble;
            j = cJSON_GetObjectItemCaseSensitive(p, "scenario_file");
            if (cJSON_IsString(j) && j->valuestring) {
                size_t n = strlen(j->valuestring);
                if (n >= sizeof(g.scenario_file)) n = sizeof(g.scenario_file) - 1;
                memcpy(g.scenario_file, j->valuestring, n);
                g.scenario_file[n] = '\0';
            }
            cJSON_Delete(p);
        }
    }

    if (g.scenario_file[0] == '\0') {
        LOG_WARN("navigation", "scenario_file missing — route disabled");
    } else {
        ScenarioConfig* sc = scenario_load(g.scenario_file);
        if (!sc) {
            LOG_WARN("navigation", "failed to load scenario_file='%s'", g.scenario_file);
        } else {
            g.route_count = sc->route_count;
            memcpy(g.route, sc->route, sizeof(ScenarioRouteStep) * (size_t)g.route_count);
            scenario_free(sc);
            LOG_INFO("navigation", "loaded %d route step(s) from '%s'", g.route_count, g.scenario_file);
        }
    }

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION, on_fusion, NULL);
    transport_advertise(transport, TOPIC_NAVIGATION_PATH, 0u);

    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_NAVIGATION_PATH, 0u, CAP_PUBLISHER, g.rate_hz);

    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "navigation");
    cfg.priority = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = g.rate_hz;
    cfg.enable_stats = true;
    if (task_base_init(&g.taskbase, &nav_vtable, &cfg) != 0) {
        LOG_WARN("navigation", "task_base_init failed");
        return -1;
    }

    return 0;
}

static int navigation_start(void) {
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) return rc;
    node_announce_self(g.transport, &s_plugin);
    publish_route_status();
    return 0;
}

static void navigation_stop(void) {
    task_stop(&g.taskbase);
}

static void navigation_cleanup(void) {
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    pthread_mutex_destroy(&g.mu);
}

static int navigation_health(void) {
    return 0;
}

static NodePlugin s_plugin = {
    .api_version = NODE_PLUGIN_API_VERSION,
    .name = "navigation",
    .version = "1.0.0",
    .description = "Scenario route navigator (publishes navigation/path)",
    .input_topics = s_inputs,
    .output_topics = s_outputs,
    .init = navigation_init,
    .start = navigation_start,
    .stop = navigation_stop,
    .cleanup = navigation_cleanup,
    .health = navigation_health,
    .taskbase = &g.taskbase,
};

NodePlugin* node_get_plugin(void) {
    return &s_plugin;
}
