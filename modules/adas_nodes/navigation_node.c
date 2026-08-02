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
#include "scenario_router.h"
#include "clock_service.h"
#include "logger.h"
#include <cjson/cJSON.h>

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define NAV_DEFAULT_RATE_HZ 10.0

static struct {
    Transport* transport;
    DiscoveryManager* discovery;
    Scheduler* scheduler;
    TaskBase taskbase;

    pthread_mutex_t mu;
    double ego_x;
    double ego_y;
    int has_fusion;
    int on_return;          /* 权威行进方向：flowsim road/ref_path.reverse（掉头返程=1） */
    int has_return_signal;  /* 是否已收到过 ref_path.reverse 字段 */

    ScenarioRouteStep route[SCENARIO_MAX_ROUTE_STEPS];
    int route_count;
    int next_idx;
    int travel_dir;         /* +1: x 递增（东向前进），-1: x 递减（西向回程）。
                             *   唯一事实源 = flowsim road/ref_path.reverse
                             *   （on_ref_path → on_return），不做 dx/heading 猜测。 */
    int active_step_index;
    int lane_count;
    double lane_width;
    double road_length_m;
    RouterGraph lane_graph;
    int graph_ready;

    int active_goal_lane;
    double active_goal_speed;
    int last_hop_target_lane;
    uint64_t last_hop_pub_us;
    uint32_t seq;

    double rate_hz;
    char scenario_file[256];
    uint64_t last_status_us;
} g;

static const char* step_type_to_str(RouteStepType t) {
    switch (t) {
        case ROUTE_BRANCH_SELECT: return "branch_select";
        case ROUTE_MERGE:         return "merge";
        case ROUTE_U_TURN:        return "u_turn";
        case ROUTE_LANE_CHANGE:
        default:                  return "lane_change";
    }
}

static void publish_route_status(void) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "route_status");
    cJSON_AddNumberToObject(root, "route_count", g.route_count);
    cJSON_AddNumberToObject(root, "next_idx", g.next_idx);
    cJSON_AddNumberToObject(root, "travel_dir", g.travel_dir);
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

static void publish_route_step_fields(RouteStepType type,
                                      int target_lane,
                                      double target_speed,
                                      int branch_id,
                                      double trigger_x,
                                      int step_index,
                                      double ego_x,
                                      const char* label) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "route_step");
    cJSON_AddNumberToObject(root, "step_index", step_index);
    cJSON_AddNumberToObject(root, "route_count", g.route_count);
    cJSON_AddStringToObject(root, "step_type", step_type_to_str(type));
    cJSON_AddNumberToObject(root, "trigger_x", trigger_x);
    cJSON_AddNumberToObject(root, "ego_x", ego_x);
    cJSON_AddNumberToObject(root, "target_lane", target_lane);
    cJSON_AddNumberToObject(root, "target_speed", target_speed);
    cJSON_AddNumberToObject(root, "branch_id", branch_id);
    if (label && label[0] != '\0') cJSON_AddStringToObject(root, "label", label);
    cJSON_AddNumberToObject(root, "seq", (double)g.seq++);
    cJSON_AddNumberToObject(root, "timestamp_us", (double)clock_now_us());
    char* s = cJSON_PrintUnformatted(root);
    if (s) {
        transport_publish(g.transport, TOPIC_NAVIGATION_PATH, (const uint8_t*)s, (uint32_t)strlen(s) + 1);
        free(s);
    }
    cJSON_Delete(root);
}

static double lane_center_y(int lane_idx) {
    return ((double)(g.lane_count - 1) * 0.5 - (double)lane_idx) * g.lane_width;
}

static int lane_idx_from_y(double y) {
    if (g.lane_count <= 0 || g.lane_width <= 0.0) return -1;
    int best = 0;
    double best_abs = fabs(y - lane_center_y(0));
    for (int i = 1; i < g.lane_count; ++i) {
        double d = fabs(y - lane_center_y(i));
        if (d < best_abs) { best_abs = d; best = i; }
    }
    return best;
}

static int resolve_target_lane(int raw_target_lane, int current_lane) {
    if (g.lane_count <= 0) return -1;
    if (raw_target_lane >= 0 && raw_target_lane < g.lane_count) return raw_target_lane;
    if (raw_target_lane == -1) {
        int t = current_lane - 1;
        if (t < 0) t = 0;
        return t;
    }
    if (raw_target_lane == 1) {
        int t = current_lane + 1;
        if (t >= g.lane_count) t = g.lane_count - 1;
        return t;
    }
    return -1;
}

static double route_step_sort_key(const ScenarioRouteStep* step) {
    if (!step) return 0.0;
    return step->has_trigger_y ? 1e15 : step->trigger_x;
}

static void maybe_publish_next_lane_hop(double ego_x, double ego_y) {
    if (!g.graph_ready || g.active_goal_lane < 0) return;

    int current_lane = lane_idx_from_y(ego_y);
    if (current_lane < 0) return;
    if (current_lane == g.active_goal_lane) {
        g.active_goal_lane = -1;
        g.active_goal_speed = -1.0;
        g.last_hop_target_lane = -1;
        return;
    }

    RouterPath p;
    if (router_astar(&g.lane_graph, current_lane, g.active_goal_lane, &p) != 0 || p.count < 2) {
        return;
    }

    int next_lane = p.lane_ids[1];
    uint64_t now_us = clock_now_us();
    if (next_lane == g.last_hop_target_lane && now_us - g.last_hop_pub_us < 1500000ULL) {
        return;
    }

    double hop_speed = (next_lane == g.active_goal_lane) ? g.active_goal_speed : -1.0;
    publish_route_step_fields(ROUTE_LANE_CHANGE, next_lane, hop_speed, -1, ego_x,
                              g.active_step_index >= 0 ? g.active_step_index : 0, ego_x, "astar_lane_hop");
    LOG_INFO("navigation", "A* hop lane %d -> %d (goal=%d)", current_lane, next_lane, g.active_goal_lane);
    g.last_hop_target_lane = next_lane;
    g.last_hop_pub_us = now_us;
}

static void on_fusion(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* jx = cJSON_GetObjectItemCaseSensitive(root, "x");
    cJSON* jy = cJSON_GetObjectItemCaseSensitive(root, "y");
    if (cJSON_IsNumber(jx)) {
        pthread_mutex_lock(&g.mu);
        g.ego_x = jx->valuedouble;
        if (cJSON_IsNumber(jy)) g.ego_y = jy->valuedouble;
        g.has_fusion = 1;
        pthread_mutex_unlock(&g.mu);
    }
    cJSON_Delete(root);
}

/* road/ref_path 携带 flowsim 权威行进方向标志 reverse（掉头返程=true）。
 * navigation 以此作为唯一 travel_dir 事实源，避免基于 dx/heading 猜测在掉头/
 * 绕圈时来回翻转。 */
static void on_ref_path(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg) return;
    cJSON* root = cJSON_Parse((const char*)msg->data);
    if (!root) return;
    cJSON* jr = cJSON_GetObjectItemCaseSensitive(root, "reverse");
    if (cJSON_IsBool(jr)) {
        pthread_mutex_lock(&g.mu);
        g.on_return = cJSON_IsTrue(jr) ? 1 : 0;
        g.has_return_signal = 1;
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
        double ego_y = 0.0;
        int has_fusion = 0;
        int on_return = 0;
        int has_return_signal = 0;
        pthread_mutex_lock(&g.mu);
        ego_x = g.ego_x;
        ego_y = g.ego_y;
        has_fusion = g.has_fusion;
        on_return = g.on_return;
        has_return_signal = g.has_return_signal;
        pthread_mutex_unlock(&g.mu);

        /* 行进方向 = flowsim 权威标志 road/ref_path.reverse。
         * reverse=true（掉头返程）→ travel_dir=-1；否则前进 +1。此标志由 flowsim
         * 掉头状态机在 finalize 时置位、整段返程保持，永不抖动 —— 彻底消除旧版基于
         * dx/heading 猜测在掉头/绕圈时反复翻转、把 route 步骤/A* hop 反复重放、
         * 将 ego 拽离对向车道绕圈的正反馈环。未收到信号前默认前进（+1）。 */
        if (has_return_signal) {
            int new_dir = on_return ? -1 : +1;
            if (new_dir != g.travel_dir) {
                g.travel_dir = new_dir;
                g.active_goal_lane = -1;
                g.active_goal_speed = -1.0;
                g.last_hop_target_lane = -1;
                g.active_step_index = -1;
                /* 前进(+x)：从头执行 route 步骤（enter_noa / prepare_u_turn）。
                 * 返回(-x)：route 步骤是「前进方向专用」的相对变道，返程重放会把 ego
                 *   拽离掉头后所在的对向车道、绕圈。返程「保持当前对向车道直线开回起点」，
                 *   起点处的二次掉头由 flowsim 路端检测触发，navigation 返程不发变道指令。
                 *   next_idx=-1 关闭步骤重放；active_goal_lane 已清空 → 无 A* hop。 */
                if (g.route_count > 0 && g.travel_dir > 0) {
                    g.next_idx = 0;
                } else {
                    g.next_idx = -1;  /* 返程或无 route：保持车道，不重放步骤 */
                }
                LOG_INFO("navigation", "travel direction -> %s (flowsim reverse=%d), route cursor=%d (%s)",
                         g.travel_dir > 0 ? "forward(+x)" : "return(-x)", on_return, g.next_idx,
                         g.travel_dir > 0 ? "replay forward steps" : "hold lane, drive back");
            }
        }

        if (has_fusion && g.route_count > 0) {
            while (g.next_idx >= 0 && g.next_idx < g.route_count) {
                const ScenarioRouteStep* step = &g.route[g.next_idx];
                int trigger_hit;
                if (step->has_trigger_y) {
                    trigger_hit = (ego_y <= step->trigger_y);
                } else {
                    trigger_hit = (g.travel_dir >= 0)
                        ? (ego_x >= step->trigger_x)
                        : (ego_x <= step->trigger_x);
                }
                if (!trigger_hit) break;

                int current_lane = lane_idx_from_y(ego_y);
                int target_lane = step->target_lane;
                if (step->type == ROUTE_LANE_CHANGE && current_lane >= 0) {
                    int resolved = resolve_target_lane(step->target_lane, current_lane);
                    if (resolved >= 0) target_lane = resolved;
                }
                publish_route_step_fields(step->type, target_lane, step->target_speed,
                                          step->branch_id, step->trigger_x,
                                          g.next_idx, ego_x, step->label);
                LOG_INFO("navigation", "route step #%d triggered @x=%.1f type=%s lane=%d speed=%.1f",
                         g.next_idx, ego_x, step_type_to_str(step->type),
                         target_lane, step->target_speed);
                g.active_step_index = g.next_idx;
                if (step->type == ROUTE_LANE_CHANGE && target_lane >= 0) {
                    g.active_goal_lane = target_lane;
                    g.active_goal_speed = step->target_speed;
                }
                if (g.travel_dir >= 0) g.next_idx++;
                else                   g.next_idx--;
            }
        }

        if (has_fusion && g.active_goal_lane >= 0) {
            maybe_publish_next_lane_hop(ego_x, ego_y);
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
    TOPIC_ROAD_REF_PATH,
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
            if (g.route_count > 1) {
                for (int i = 0; i < g.route_count - 1; ++i) {
                    for (int j = i + 1; j < g.route_count; ++j) {
                        if (route_step_sort_key(&g.route[i]) > route_step_sort_key(&g.route[j])) {
                            ScenarioRouteStep tmp = g.route[i];
                            g.route[i] = g.route[j];
                            g.route[j] = tmp;
                        }
                    }
                }
            }
            g.lane_count = sc->road.lanes > 0 ? sc->road.lanes : 2;
            g.lane_width = sc->road.lane_width > 0.0 ? sc->road.lane_width : 3.5;
            g.ego_y = sc->ego.y;
            g.road_length_m = 3000.0;
            if (sc->duration_s > 0.0) (void)0;
            if (g.lane_count > 0) {
                router_graph_init(&g.lane_graph);
                for (int i = 0; i < g.lane_count; ++i) {
                    router_add_lane(&g.lane_graph, i, 0.0, g.road_length_m, i, 0, 22.0);
                }
                router_build_topology(&g.lane_graph, 8.0);
                g.graph_ready = 1;
            }
            scenario_free(sc);
            g.active_goal_lane = -1;
            g.active_goal_speed = -1.0;
            g.last_hop_target_lane = -1;
            g.last_hop_pub_us = 0;
            g.active_step_index = -1;
            g.travel_dir = +1;
            g.next_idx = 0;
            LOG_INFO("navigation", "loaded %d route step(s) from '%s' lanes=%d lane_width=%.2f",
                     g.route_count, g.scenario_file, g.lane_count, g.lane_width);
        }
    }

    transport_subscribe(transport, TOPIC_FUSION_LOCALIZATION, on_fusion, NULL);
    transport_subscribe(transport, TOPIC_ROAD_REF_PATH, on_ref_path, NULL);
    transport_advertise(transport, TOPIC_NAVIGATION_PATH, 0u);

    discovery_advertise(discovery, TOPIC_FUSION_LOCALIZATION, 0xF0ED10C0u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, TOPIC_ROAD_REF_PATH, 0u, CAP_SUBSCRIBER, 0);
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
