/**
 * monitor_node.c — 系统监控节点插件
 *
 * 订阅感知、车辆状态、融合延迟等 topic，
 * 收集系统资源指标，导出 JSON 供 FlowBoard 仪表盘展示。
 *
 * NodePlugin 接口，编译为 libmonitor_node.so。
 */

#define _GNU_SOURCE
#include "node_plugin.h"
#include "sysmonitor.h"
#include "flow_registry.h"
#include "logger.h"
#include "stats_bridge.h"
#include "dashboard_bridge.h"
#include "adas_msgs_gen.h"
#include "json_extract.h"
#include "clock_service.h"
#include <cjson/cJSON.h>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>

/* ── 节点本地状态 ───────────────────────────────────────────── */

static struct {
    MessageBus*       bus;
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    /* 托管模式：嵌入 TaskBase，由 node_start_managed 派生线程跑 monitor_execute。
     * 取代原先自管的 pthread thread / running / should_stop 三件套。 */
    TaskBase   taskbase;

    SysMonitor* sysmon;

    /* 订阅数据缓存 */
    char latest_obstacles_json[2048];
    char latest_vehicle_state[1024];
    double fusion_lat_avg_us;
    double fusion_lat_p50_us;
    double fusion_lat_p99_us;
    volatile int has_obstacles;
    volatile int has_vehicle_state;

    /* NOA 驾驶模式 (来自 planning/trajectory 的 "mode=" / "route_lane="
     * 追加字段，见 planning_node.c / control_node.c)。用于仪表盘展示当前
     * 模式层级 (NA/ACC/CP/NP/NOA) 及 NOA 导航驱动的目标车道。
     *
     * NOA Phase 6: 同时缓存 trajectory 的 path 数组（Frenet [s,d,spd]），
     * 透传给 3D 前端画规划轨迹线。planning 消息是 JSON + 尾部文本混合，
     * path 在 JSON 部分内，cJSON 解析时忽略尾部文本。 */
    char driver_mode[32];
    int  route_lane;
    char trajectory_path_json[4096];  /* path 数组 JSON 文本，如 [[s,d,spd],...] */
    volatile int has_planning;

    /* Phase 2: 道路几何缓存（从 road/geometry topic 获取，sim_world 发布） */
    double road_curve_start_x;
    double road_curve_length_m;
    double road_curve_offset_m;
    volatile int has_road_geometry;

    /* Phase 2: 红绿灯状态缓存（从 road/traffic_lights topic 获取，sim_world 发布）
     * 直接缓存原始 JSON 文本，在拼装 scene.traffic_lights 时透传给 FlowBoard。
     * 这样 monitor 不需要解析红绿灯语义，只做数据管道。 */
    char   traffic_lights_json[512];
    volatile int has_traffic_lights;

    /* Phase 3: scene/frame 缓存（从 flowsim_node 发布）。
     * 只缓存 road_network 部分（entities 由 vehicle/state 覆盖，无需重复）。
     * road_network 含 edges[]，每条 edge 有 nodes[[x,y],...]，供 3D 前端
     * 用 CatmullRomCurve3 + TubeGeometry 构建多段道路网络。
     *
     * NOA Phase 2.2: 透传 scene/frame 的完整 entities 数组（不再过滤
     * etc_gate / stop_line）。vehicle/state 的 obstacles 仅承载前 16 个 NPC
     * 车辆/行人（MAX_OBS_SCENE=16），而 NOA 24-NPC 场景需要前端能渲染全部 24
     * 个 NPC + ego + 红绿灯 + ETC 门架 + 停止线。完整 entities 透传后，前端
     * (scene3d.js) 优先消费 scn.entities，scn.obstacles 作为旧场景 fallback。 */
    char   scene_road_network_json[4096];
    char   scene_entities_json[16384];  /* 完整 entities（24 NPC + ego + TL/ETC/StopLine） */
    volatile int has_scene_frame;

    /* 节点拓扑: 从 flowengine/node_info topic 收集 (B 方案) */
#define MAX_TOPO_NODES 16
    char node_info_json[MAX_TOPO_NODES][2048];  /* 每个节点的原始 JSON（需能容纳最长节点的 self-description JSON） */
    int  node_info_count;

    /* 导出路径 */
    char state_file[512];

    /* 跨进程 stats bridge */
    IpcChannel* stats_ch;
    /* stats bridge subscriber：聚合其它进程的 bus/topic 统计，
     * 供 export_dashboard_json 输出全局指标（而非仅本进程）。
     * 注意：本节点也会 publish stats，subscriber 回调里按 source_name
     * 过滤掉自己发的包，避免自收自发导致重复计数。 */
    IpcChannel* stats_sub;
    pthread_mutex_t remote_stats_mutex;
#define MONITOR_MAX_REMOTE_SRCS 16
    struct {
        char            source_name[64];
        StatsPacket     pkt;
        int             valid;
    } remote_stats[MONITOR_MAX_REMOTE_SRCS];
    int remote_stats_count;

    /* 跨进程 dashboard JSON bridge */
    IpcChannel* dashboard_ch;

    /* 配置 */
    double frequency_hz;
    double lane_width;
    int    lane_count;
} g;

/* ── 订阅回调 ────────────────────────────────────────────────── */

static void on_obstacles(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    size_t copy = msg->data_size < sizeof(g.latest_obstacles_json) - 1
                  ? msg->data_size : sizeof(g.latest_obstacles_json) - 1;
    memcpy(g.latest_obstacles_json, msg->data, copy);
    g.latest_obstacles_json[copy] = '\0';
    g.has_obstacles = 1;
}

static void on_vehicle_state(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    size_t copy = msg->data_size < sizeof(g.latest_vehicle_state) - 1
                  ? msg->data_size : sizeof(g.latest_vehicle_state) - 1;
    memcpy(g.latest_vehicle_state, msg->data, copy);
    g.latest_vehicle_state[copy] = '\0';
    g.has_vehicle_state = 1;
}

/* 收集其他节点的自描述广播 */
static void on_node_info(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data || g.node_info_count >= MAX_TOPO_NODES) return;
    size_t copy = msg->data_size < sizeof(g.node_info_json[0]) - 3
                  ? msg->data_size : sizeof(g.node_info_json[0]) - 3;
    memcpy(g.node_info_json[g.node_info_count], msg->data, copy);
    /* 截断保护: 若原始 JSON 被截断（msg->data_size > 缓冲区容量）, 确保闭括号完整,
     * 避免生成非法 JSON 导致整个 state file 解析失败（scene 等数据丢失）。 */
    if (msg->data_size >= sizeof(g.node_info_json[0]) - 3) {
        size_t end = copy;
        while (end > 0 && g.node_info_json[g.node_info_count][end - 1] != '}') end--;
        if (end > 0) {
            /* 确保末尾有 ]} 或至少 } */
            if (end >= 2 && g.node_info_json[g.node_info_count][end - 2] == ']') {
                copy = end;
            } else if (end >= 1) {
                /* 在截断处补上 ]} */
                copy = end;
                g.node_info_json[g.node_info_count][copy++] = ']';
                g.node_info_json[g.node_info_count][copy++] = '}';
            }
        }
    }
    g.node_info_json[g.node_info_count][copy] = '\0';
    g.node_info_count++;
}

/* ── 跨进程 stats bridge 订阅回调 ──────────────────────────── */
/*
 * 收到其它进程的 StatsPacket 时，按 source_name 聚合到 remote_stats[]。
 * 过滤掉本节点自己发的包（source_name == "monitor_node"），避免自收自发
 * 导致重复计数。export_dashboard_json 读取这些聚合值输出全局指标。
 */
static void on_remote_stats(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || msg->data_size < sizeof(StatsPacket)) return;
    const StatsPacket* pkt = (const StatsPacket*)msg->data;
    if (strcmp(pkt->source_name, "monitor_node") == 0) return;  /* 跳过自己 */

    pthread_mutex_lock(&g.remote_stats_mutex);
    int slot = -1;
    for (int i = 0; i < g.remote_stats_count; i++) {
        if (strcmp(g.remote_stats[i].source_name, pkt->source_name) == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0 && g.remote_stats_count < MONITOR_MAX_REMOTE_SRCS) {
        slot = g.remote_stats_count++;
    }
    if (slot >= 0) {
        snprintf(g.remote_stats[slot].source_name,
                 sizeof(g.remote_stats[slot].source_name),
                 "%s", pkt->source_name);
        g.remote_stats[slot].pkt   = *pkt;
        g.remote_stats[slot].valid = 1;
    }
    pthread_mutex_unlock(&g.remote_stats_mutex);
}

/* planning/trajectory 订阅 — 提取驾驶模式 + NOA 导航路线目标车道，供仪表盘展示
 * (见 planning_node.c 追加的 "mode=" / "route_lane=" 字段，与 control_node.c 的
 * 解析方式一致，采用宽松的 strstr + sscanf)。
 *
 * NOA Phase 6: 同时提取 JSON 部分的 path 数组（Frenet [s,d,spd]），透传给
 * 3D 前端画规划轨迹线。planning 消息格式: <json>{"path":[[s,d,spd],...],...}
 * speed=... mode=... route_lane=...，cJSON_Parse 只解析前缀 JSON，尾部文本
 * 自动忽略。path 数组重新序列化为紧凑 JSON 文本缓存，避免每帧重复解析。 */
static void on_planning_trajectory(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;

    const char* p = strstr(d, "mode=");
    if (p) {
        char buf[32] = {0};
        sscanf(p + 5, "%31s", buf);
        snprintf(g.driver_mode, sizeof(g.driver_mode), "%s", buf);
    }
    p = strstr(d, "route_lane=");
    if (p) sscanf(p + 11, "%d", &g.route_lane);

    /* NOA Phase 6: 提取 path 数组并缓存为 JSON 文本 */
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* path = cJSON_GetObjectItem(root, "path");
        if (path && cJSON_IsArray(path)) {
            char* path_str = cJSON_PrintUnformatted(path);
            if (path_str) {
                size_t len = strlen(path_str);
                if (len >= sizeof(g.trajectory_path_json))
                    len = sizeof(g.trajectory_path_json) - 1;
                memcpy(g.trajectory_path_json, path_str, len);
                g.trajectory_path_json[len] = '\0';
                free(path_str);
            }
        }
        cJSON_Delete(root);
    }

    g.has_planning = 1;
}

/* Phase 2: road/geometry 订阅 — 从 sim_world 获取弯道参数，
 * 替代此前从 vehicle/state 间接读取 road_curve_sx/len/off 的方式。 */
static void on_road_geometry(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* item;
        if ((item = cJSON_GetObjectItem(root, "curve_start_x")))  g.road_curve_start_x = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "curve_length_m"))) g.road_curve_length_m = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "curve_offset_m"))) g.road_curve_offset_m = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "lane_width")))     g.lane_width = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "lane_count")))     g.lane_count = (int)item->valuedouble;
        cJSON_Delete(root);
    }
    g.has_road_geometry = 1;
}

/* Phase 2: road/traffic_lights 订阅 — 从 sim_world 获取红绿灯状态。
 * 直接缓存原始 JSON 文本（{"lights":[...]}），在拼装 scene.traffic_lights 时
 * 透传给 FlowBoard 3D 渲染。monitor 不解析红绿灯语义，只做数据管道。 */
static void on_traffic_lights(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    /* 安全拷贝：确保 null 结尾，防止越界 */
    size_t len = strlen(d);
    if (len >= sizeof(g.traffic_lights_json)) len = sizeof(g.traffic_lights_json) - 1;
    memcpy(g.traffic_lights_json, d, len);
    g.traffic_lights_json[len] = '\0';
    g.has_traffic_lights = 1;
}

/* Phase 3: scene/frame 订阅 — 从 flowsim_node 获取完整场景帧。
 * 提取 road_network（多段道路）+ 完整 entities 数组（NOA Phase 2.2 改为
 * 透传全部实体，不再过滤 etc_gate/stop_line）。
 * road_network 透传给 3D 前端用于 CatmullRomCurve3 + TubeGeometry
 * 多段道路渲染；entities 透传给前端用于渲染全部 24 NPC + 事件触发器
 * （vehicle/state 仅承载前 16 个 obstacle，不足以覆盖 NOA 场景）。 */
static void on_scene_frame(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (!root) return;

    /* 缓存 road_network */
    cJSON* rn = cJSON_GetObjectItem(root, "road_network");
    if (rn) {
        char* rn_str = cJSON_PrintUnformatted(rn);
        if (rn_str) {
            size_t len = strlen(rn_str);
            if (len >= sizeof(g.scene_road_network_json))
                len = sizeof(g.scene_road_network_json) - 1;
            memcpy(g.scene_road_network_json, rn_str, len);
            g.scene_road_network_json[len] = '\0';
            free(rn_str);
        }
    }

    /* NOA Phase 2.2: 透传完整 entities 数组（不再过滤类型）。
     * 前端 scene3d.js 按 type 分发渲染：ego/NPC/pedestrian/tl/etc_gate/stop_line，
     * scn.obstacles (vehicle/state) 作为旧场景 fallback。 */
    cJSON* entities = cJSON_GetObjectItem(root, "entities");
    if (entities && cJSON_IsArray(entities)) {
        char* ent_str = cJSON_PrintUnformatted(entities);
        if (ent_str) {
            size_t len = strlen(ent_str);
            if (len >= sizeof(g.scene_entities_json))
                len = sizeof(g.scene_entities_json) - 1;
            memcpy(g.scene_entities_json, ent_str, len);
            g.scene_entities_json[len] = '\0';
            free(ent_str);
        }
    }

    g.has_scene_frame = 1;
    cJSON_Delete(root);
}

static void on_fusion_latency(const Message* msg, void* user_data) {
    (void)user_data;
    if (!msg || !msg->data) return;

    /* Try binary deserialization (serializer path) */
    {
        LatencyReport lr;
        if (LatencyReport_deserialize(&lr, (const uint8_t*)msg->data, msg->data_size) == 0) {
            g.fusion_lat_avg_us = lr.avg_us;
            g.fusion_lat_p50_us = lr.p50_us;
            g.fusion_lat_p99_us = lr.p99_us;
            return;
        }
    }

    /* Fallback: text JSON parsing */
    const char* d = (const char*)msg->data;
    cJSON* root = cJSON_Parse(d);
    if (root) {
        cJSON* item;
        if ((item = cJSON_GetObjectItem(root, "avg_us"))) g.fusion_lat_avg_us = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "p50_us"))) g.fusion_lat_p50_us = item->valuedouble;
        if ((item = cJSON_GetObjectItem(root, "p99_us"))) g.fusion_lat_p99_us = item->valuedouble;
        cJSON_Delete(root);
    }
}

/* JSON 标量提取辅助（json_extract_double / json_extract_int / json_extract_string）
 * 已迁移至共享工具 include/json_extract.h，避免与其他模块（如
 * src/algorithms/nuscenes_loader.c）各自维护一份不一致的实现。 */

/* 从 discovery 拓扑写出 nodes 数组 (多进程模式回退路径)。
 *
 * 单进程 (dlopen) 模式下所有节点共享同一个 DiscoveryManager, 拓扑里只有一个
 * 聚合节点 "flow_launcher"; 此时 flowengine/node_info 广播能覆盖全部节点, 走
 * 方案B。但多进程模式下每个节点是独立进程 + 独立 discovery, node_info 广播只在
 * 本进程 bus 内可见 (不跨进程), 于是 monitor 只能看到自己 → 拓扑图只有一个节点。
 *
 * discovery 通过 UDP 组播天然跨进程共享全网拓扑, 因此当 node_info 广播数不足以
 * 覆盖 discovery 已知的节点数时, 改用 discovery 拓扑构建 nodes 数组, 输出与
 * node_announce_self 相同的 JSON 形状。返回写出的节点数。 */
static int emit_nodes_from_discovery(cJSON* nodes_arr, const TopologyGraph* g_topo) {
    int written = 0;
    for (uint32_t i = 0; i < g_topo->node_count; i++) {
        const NodeInfo* n = &g_topo->nodes[i];
        if (!n->alive) continue;
        cJSON* node = cJSON_CreateObject();
        cJSON_AddStringToObject(node, "name", n->name);
        cJSON_AddStringToObject(node, "version", "");
        cJSON_AddStringToObject(node, "description", "");
        cJSON_AddNumberToObject(node, "pid", (double)n->pid);
        cJSON_AddTrueToObject(node, "alive");
        cJSON* topics = cJSON_AddArrayToObject(node, "topics");
        for (uint32_t j = 0; j < n->topic_count; j++) {
            bool is_pub = (n->topics[j].capabilities & CAP_PUBLISHER) != 0;
            bool is_sub = (n->topics[j].capabilities & CAP_SUBSCRIBER) != 0;
            const char* role = is_pub && is_sub ? "pubsub" : (is_pub ? "pub" : "sub");
            cJSON* t = cJSON_CreateObject();
            cJSON_AddStringToObject(t, "topic", n->topics[j].topic);
            cJSON_AddStringToObject(t, "role", role);
            cJSON_AddNumberToObject(t, "caps", (double)n->topics[j].capabilities);
            cJSON_AddItemToArray(topics, t);
        }
        cJSON_AddItemToArray(nodes_arr, node);
        written++;
    }
    return written;
}

/* ── 导出 JSON 到 state_file ──────────────────────────────── */

static void export_dashboard_json(void) {
    uint64_t now_realtime_us = clock_now_realtime_us();
    double timestamp = (double)(now_realtime_us / 1000000ULL)
                     + (double)(now_realtime_us % 1000000ULL) / 1000000.0;

    /* 收集指标：本进程 bus stats + 跨进程聚合（stats bridge）。
     * 多进程部署下，monitor_node 自己的 bus 几乎无业务消息，bus stats ≈ 0；
     * 必须聚合其它进程（fusion/perception/planning/control/...）经 stats bridge
     * 上报的统计，dashboard 才能反映全局真实吞吐，否则 charts 恒为 0。 */
    uint64_t pub = 0, del = 0, drop = 0;
    if (g.bus) message_bus_get_stats(g.bus, &pub, &del, &drop);

    /* 聚合远程进程的 bus 统计 */
    pthread_mutex_lock(&g.remote_stats_mutex);
    for (int i = 0; i < g.remote_stats_count; i++) {
        if (!g.remote_stats[i].valid) continue;
        pub  += g.remote_stats[i].pkt.bus_pub;
        del  += g.remote_stats[i].pkt.bus_del;
        drop += g.remote_stats[i].pkt.bus_drop;
    }
    pthread_mutex_unlock(&g.remote_stats_mutex);

    TransportStats ts;
    memset(&ts, 0, sizeof(ts));
    if (g.transport) transport_get_stats(g.transport, &ts);

    int task_count = 0;
    if (g.scheduler) task_count = scheduler_task_count(g.scheduler);

    SysMonitorSnapshot ssnap;
    memset(&ssnap, 0, sizeof(ssnap));
    if (g.sysmon) sysmonitor_snapshot(g.sysmon, &ssnap);

    char* topo_json = g.discovery ? discovery_export_json(g.discovery) : NULL;

    /* ── Build cJSON tree ── */
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "self", "flow_launcher");
    cJSON_AddNumberToObject(root, "timestamp", timestamp);

    /* nodes 数组来源:
     *  - 单进程 (dlopen): 各节点广播的 flowengine/node_info (方案B, 含 version/desc)
     *  - 多进程 (fork+exec): node_info 不跨进程, 回退到 discovery 跨进程拓扑
     * 判据: discovery 已知的存活节点数 > 收到的 node_info 广播数, 说明有节点的
     *       自描述广播没能抵达 monitor (多进程), 改用 discovery 补全拓扑。 */
    const TopologyGraph* topo = g.discovery ? discovery_get_topology(g.discovery) : NULL;
    int disc_alive = 0;
    if (topo) {
        for (uint32_t i = 0; i < topo->node_count; i++)
            if (topo->nodes[i].alive) disc_alive++;
    }

    cJSON* nodes = cJSON_AddArrayToObject(root, "nodes");
    if (topo && disc_alive > g.node_info_count) {
        emit_nodes_from_discovery(nodes, topo);
        /* discovery 拓扑不含本节点自身 (self 只广播 my_topics, 不进自己的
         * topology)。补上 monitor 收到的本地 node_info 广播 (通常就是它自己),
         * 按 name 去重, 使拓扑图包含 monitor 节点。 */
        for (int i = 0; i < g.node_info_count; i++) {
            char nm[64] = "";
            json_extract_string(g.node_info_json[i], "name", nm, sizeof(nm));
            bool dup = false;
            for (uint32_t k = 0; nm[0] && k < topo->node_count; k++) {
                if (topo->nodes[k].alive && strcmp(topo->nodes[k].name, nm) == 0) {
                    dup = true; break;
                }
            }
            if (!dup) {
                cJSON* ni = cJSON_Parse(g.node_info_json[i]);
                if (ni) cJSON_AddItemToArray(nodes, ni);
            }
        }
    } else {
        for (int i = 0; i < g.node_info_count; i++) {
            cJSON* ni = cJSON_Parse(g.node_info_json[i]);
            if (ni) cJSON_AddItemToArray(nodes, ni);
        }
    }

    /* ── metrics sub-object ── */
    cJSON* metrics = cJSON_AddObjectToObject(root, "metrics");

    cJSON* bus_o = cJSON_AddObjectToObject(metrics, "bus");
    cJSON_AddNumberToObject(bus_o, "published", (double)pub);
    cJSON_AddNumberToObject(bus_o, "delivered", (double)del);
    cJSON_AddNumberToObject(bus_o, "dropped", (double)drop);

    cJSON* transport_o = cJSON_AddObjectToObject(metrics, "transport");
    cJSON_AddNumberToObject(transport_o, "local_pub", (double)ts.local_published);
    cJSON_AddNumberToObject(transport_o, "remote_pub", (double)ts.remote_published);

    cJSON* sched_o = cJSON_AddObjectToObject(metrics, "scheduler");
    cJSON_AddNumberToObject(sched_o, "tasks", task_count);
    cJSON_AddStringToObject(sched_o, "mode", "CHOREO");

    /* 融合延迟 */
    cJSON* lat_o = cJSON_AddObjectToObject(metrics, "latency");
    cJSON_AddNumberToObject(lat_o, "avg_us", g.fusion_lat_avg_us);
    cJSON_AddNumberToObject(lat_o, "p50_us", g.fusion_lat_p50_us);
    cJSON_AddNumberToObject(lat_o, "p99_us", g.fusion_lat_p99_us);

    /* NOA 驾驶模式 (来自 planning/trajectory)，未收到数据前默认 "NA:READY" */
    cJSON_AddStringToObject(metrics, "driver_mode",
                            g.driver_mode[0] ? g.driver_mode : "NA:READY");
    cJSON_AddNumberToObject(metrics, "route_lane", g.route_lane);

    /* Topic 统计：合并本进程 + 跨进程（stats bridge）。
     * 同名 topic 跨进程累加 pub/del/drop，freq 取最大值（代表发布频率），
     * subs 取最大值。这样 dashboard 的 charts 能看到全局真实吞吐。 */
    struct { char topic[64]; uint64_t pub, del, drop, lat_us; double freq; uint32_t subs; int has_lat; } merged[64];
    int merged_n = 0;

    /* 先放入本进程 topics */
    TopicStats tstats[16];
    int nt = g.bus ? message_bus_get_all_topic_stats(g.bus, tstats, 16) : 0;
    for (int ti = 0; ti < nt && merged_n < 64; ti++) {
        snprintf(merged[merged_n].topic, sizeof(merged[merged_n].topic), "%s", tstats[ti].topic);
        merged[merged_n].pub  = tstats[ti].publish_count;
        merged[merged_n].del  = tstats[ti].deliver_count;
        merged[merged_n].drop = tstats[ti].drop_count;
        merged[merged_n].lat_us = tstats[ti].deliver_count > 0
            ? tstats[ti].total_latency_us / tstats[ti].deliver_count : 0;
        merged[merged_n].freq = tstats[ti].frequency_hz;
        merged[merged_n].subs = tstats[ti].subscriber_count;
        merged[merged_n].has_lat = 1;
        merged_n++;
    }

    /* 合并远程进程 topics */
    pthread_mutex_lock(&g.remote_stats_mutex);
    for (int ri = 0; ri < g.remote_stats_count; ri++) {
        if (!g.remote_stats[ri].valid) continue;
        const StatsPacket* rp = &g.remote_stats[ri].pkt;
        for (uint32_t rti = 0; rti < rp->topic_count && rti < STATS_BRIDGE_MAX_TOPICS; rti++) {
            const RemoteTopicStat* rt = &rp->topics[rti];
            int found = -1;
            for (int mi = 0; mi < merged_n; mi++) {
                if (strcmp(merged[mi].topic, rt->topic) == 0) { found = mi; break; }
            }
            if (found >= 0) {
                merged[found].pub  += rt->publish_count;
                merged[found].del  += rt->deliver_count;
                merged[found].drop += rt->drop_count;
                if (rt->frequency_hz > merged[found].freq) merged[found].freq = rt->frequency_hz;
                if (rt->subscriber_count > merged[found].subs) merged[found].subs = rt->subscriber_count;
            } else if (merged_n < 64) {
                snprintf(merged[merged_n].topic, sizeof(merged[merged_n].topic), "%s", rt->topic);
                merged[merged_n].pub  = rt->publish_count;
                merged[merged_n].del  = rt->deliver_count;
                merged[merged_n].drop = rt->drop_count;
                merged[merged_n].lat_us = rt->p50_latency_us;
                merged[merged_n].freq = rt->frequency_hz;
                merged[merged_n].subs = rt->subscriber_count;
                merged[merged_n].has_lat = 1;
                merged_n++;
            }
        }
    }
    pthread_mutex_unlock(&g.remote_stats_mutex);

    cJSON* topics_arr = cJSON_AddArrayToObject(metrics, "topics");
    for (int mi = 0; mi < merged_n; mi++) {
        cJSON* t = cJSON_CreateObject();
        cJSON_AddStringToObject(t, "topic", merged[mi].topic);
        cJSON_AddNumberToObject(t, "pub", (double)merged[mi].pub);
        cJSON_AddNumberToObject(t, "del", (double)merged[mi].del);
        cJSON_AddNumberToObject(t, "drop", (double)merged[mi].drop);
        cJSON_AddNumberToObject(t, "lat_us", (double)merged[mi].lat_us);
        cJSON_AddNumberToObject(t, "freq", merged[mi].freq);
        cJSON_AddNumberToObject(t, "subs", (double)merged[mi].subs);
        cJSON_AddItemToArray(topics_arr, t);
    }

    /* 车辆状态 */
    double spd = json_extract_double(g.latest_vehicle_state, "spd");
    double tgt = json_extract_double(g.latest_vehicle_state, "tgt");
    double thr = json_extract_double(g.latest_vehicle_state, "thr");
    double brk = json_extract_double(g.latest_vehicle_state, "brk");
    double vx  = json_extract_double(g.latest_vehicle_state, "x");
    cJSON* vehicle_o = cJSON_AddObjectToObject(metrics, "vehicle");
    cJSON_AddNumberToObject(vehicle_o, "speed", spd);
    cJSON_AddNumberToObject(vehicle_o, "target_speed", tgt);
    cJSON_AddNumberToObject(vehicle_o, "throttle", thr);
    cJSON_AddNumberToObject(vehicle_o, "brake", brk);
    cJSON_AddNumberToObject(vehicle_o, "x", vx);
    cJSON_AddNumberToObject(vehicle_o, "error", tgt - spd);

    /* 3D 场景（从 vehicle/state 提取） */
    double ego_x = json_extract_double(g.latest_vehicle_state, "x");
    double ego_y = json_extract_double(g.latest_vehicle_state, "y");
    double hdg   = json_extract_double(g.latest_vehicle_state, "hdg");
    double steer = json_extract_double(g.latest_vehicle_state, "st");

    cJSON* scene = cJSON_AddObjectToObject(metrics, "scene");
    cJSON* ego_o = cJSON_AddObjectToObject(scene, "ego");
    cJSON_AddNumberToObject(ego_o, "x", ego_x);
    cJSON_AddNumberToObject(ego_o, "y", ego_y);
    cJSON_AddNumberToObject(ego_o, "heading", hdg);
    cJSON_AddNumberToObject(ego_o, "speed", spd);
    cJSON_AddNumberToObject(ego_o, "steer", steer);

    cJSON* lane_o = cJSON_AddObjectToObject(scene, "lane");
    cJSON_AddNumberToObject(lane_o, "width", g.lane_width);
    cJSON_AddNumberToObject(lane_o, "count", g.lane_count);
    cJSON_AddNumberToObject(lane_o, "center", 0.0);

    /* Phase 2: 道路弯道几何从 road/geometry topic 获取 */
    if (g.road_curve_length_m > 0.0) {
        cJSON* road_o = cJSON_AddObjectToObject(scene, "road");
        cJSON_AddNumberToObject(road_o, "curve_start_x", g.road_curve_start_x);
        cJSON_AddNumberToObject(road_o, "curve_length_m", g.road_curve_length_m);
        cJSON_AddNumberToObject(road_o, "curve_offset_m", g.road_curve_offset_m);
    }

    /* Phase 3: road_network 从 scene/frame topic 获取（flowsim_node 发布）。
     * 透传 {"edges":[...]} 给 3D 前端，每个 edge 含 nodes[[x,y],...] 供
     * CatmullRomCurve3 构建多段道路。旧场景无 scene/frame 时此字段缺省，
     * 前端 fallback 到 scene.road 的单段弯道几何。 */
    if (g.has_scene_frame && g.scene_road_network_json[0] != '\0') {
        cJSON* rn = cJSON_Parse(g.scene_road_network_json);
        if (rn) {
            cJSON_AddItemToObject(scene, "road_network", rn);
        }
    }

    /* NOA Phase 2.2: 完整 entities 从 scene/frame 透传。
     * 含全部 NPC（最多 24）+ ego + 红绿灯 + ETC 门架 + 停止线。前端 scene3d.js
     * 优先消费 scn.entities 渲染障碍物池（扩到 24），scn.obstacles 作为旧场景
     * fallback。 */
    if (g.has_scene_frame && g.scene_entities_json[0] != '\0') {
        cJSON* ents = cJSON_Parse(g.scene_entities_json);
        if (ents) {
            cJSON_AddItemToObject(scene, "entities", ents);
        }
    }

    /* Phase 2: 红绿灯状态从 road/traffic_lights topic 获取，透传给 FlowBoard 3D */
    if (g.has_traffic_lights && g.traffic_lights_json[0] != '\0') {
        cJSON* tl_root = cJSON_Parse(g.traffic_lights_json);
        if (tl_root) {
            cJSON* tl_arr = cJSON_GetObjectItem(tl_root, "lights");
            if (tl_arr && cJSON_IsArray(tl_arr)) {
                cJSON_AddItemToObject(scene, "traffic_lights", cJSON_Duplicate(tl_arr, 1));
            }
            cJSON_Delete(tl_root);
        }
    }

    /* NOA Phase 6: 规划轨迹 path 数组透传给 3D 前端。
     * path 是 Frenet 坐标 [[s,d,spd],...]，前端从 ego 当前位置出发沿 heading
     * 方向延伸 s、横向偏移 d 近似绘制世界坐标轨迹线（无 esmini Frenet→World
     * 转换的纯前端近似，直道/大半径弯道下足够可视化）。 */
    if (g.has_planning && g.trajectory_path_json[0] != '\0') {
        cJSON* path = cJSON_Parse(g.trajectory_path_json);
        if (path) {
            cJSON_AddItemToObject(scene, "trajectory_path", path);
        }
    }

    /* 障碍物（从 vehicle/state 动态读取） */
    int n_obs = json_extract_int(g.latest_vehicle_state, "n_obs");
    if (n_obs < 0 || n_obs > 16) n_obs = 0;

#define MAX_OBS_SCENE 16
#define OBS_FALLBACK_CAR_LEN   4.6
#define OBS_FALLBACK_CAR_WID   2.0
#define OBS_FALLBACK_PED_SIZE  0.6
    double ox[MAX_OBS_SCENE], oy[MAX_OBS_SCENE], ovx[MAX_OBS_SCENE], ovy[MAX_OBS_SCENE];
    double olen[MAX_OBS_SCENE], owid[MAX_OBS_SCENE];
    char   otype[MAX_OBS_SCENE][16];
    char kn[20];
    for (int i = 0; i < n_obs; i++) {
        snprintf(kn, sizeof(kn), "ox%d", i);
        ox[i] = json_extract_double(g.latest_vehicle_state, kn);
        snprintf(kn, sizeof(kn), "oy%d", i);
        oy[i] = json_extract_double(g.latest_vehicle_state, kn);
        snprintf(kn, sizeof(kn), "ov%d", i);
        ovx[i] = json_extract_double(g.latest_vehicle_state, kn);
        snprintf(kn, sizeof(kn), "ovy%d", i);
        ovy[i] = json_extract_double(g.latest_vehicle_state, kn);
        snprintf(kn, sizeof(kn), "ot%d", i);
        json_extract_string(g.latest_vehicle_state, kn, otype[i], sizeof(otype[i]));
        if (otype[i][0] == '\0') {
            snprintf(otype[i], sizeof(otype[i]), "car");
        }
        int is_ped = strcmp(otype[i], "pedestrian") == 0;
        snprintf(kn, sizeof(kn), "ol%d", i);
        olen[i] = json_extract_double(g.latest_vehicle_state, kn);
        if (olen[i] < 0.1) olen[i] = is_ped ? OBS_FALLBACK_PED_SIZE : OBS_FALLBACK_CAR_LEN;
        snprintf(kn, sizeof(kn), "ow%d", i);
        owid[i] = json_extract_double(g.latest_vehicle_state, kn);
        if (owid[i] < 0.1) owid[i] = is_ped ? OBS_FALLBACK_PED_SIZE : OBS_FALLBACK_CAR_WID;
    }
    cJSON* obs_arr = cJSON_AddArrayToObject(scene, "obstacles");
    for (int i = 0; i < n_obs; i++) {
        double rx = ox[i] - ego_x;
        double ry = oy[i] - ego_y;
        cJSON* ob = cJSON_CreateObject();
        cJSON_AddNumberToObject(ob, "id", i);
        cJSON_AddStringToObject(ob, "type", otype[i]);
        cJSON_AddNumberToObject(ob, "x", rx);
        cJSON_AddNumberToObject(ob, "y", ry);
        cJSON_AddNumberToObject(ob, "vx", ovx[i]);
        cJSON_AddNumberToObject(ob, "vy", ovy[i]);
        cJSON_AddNumberToObject(ob, "len", olen[i]);
        cJSON_AddNumberToObject(ob, "wid", owid[i]);
        cJSON_AddItemToArray(obs_arr, ob);
    }

    /* LiDAR 点云（对每个障碍物生成环形点，加地面环带） */
    cJSON* lidar_arr = cJSON_AddArrayToObject(scene, "lidar");
    for (int oi = 0; oi < n_obs; oi++) {
        double rx = ox[oi] - ego_x;
        double ry = oy[oi] - ego_y;
        if (rx < -20 || rx > 60) continue;
        for (int k = 0; k < 6; k++) {
            double a = (double)k / 6.0 * (2.0 * M_PI);
            cJSON* pt = cJSON_CreateArray();
            cJSON_AddItemToArray(pt, cJSON_CreateNumber(rx + cos(a) * 1.0));
            cJSON_AddItemToArray(pt, cJSON_CreateNumber(ry + sin(a) * 2.3));
            cJSON_AddItemToArray(pt, cJSON_CreateNumber(0.4));
            cJSON_AddItemToArray(lidar_arr, pt);
        }
    }
    for (int k = 0; k < 12; k++) {
        double a = (double)k / 12.0 * (2.0 * M_PI);
        cJSON* pt = cJSON_CreateArray();
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(cos(a) * (10.0 + (k % 3) * 4.0)));
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(sin(a) * (10.0 + (k % 3) * 4.0)));
        cJSON_AddItemToArray(pt, cJSON_CreateNumber(0.0));
        cJSON_AddItemToArray(lidar_arr, pt);
    }

    /* Registry */
    char* reg_json = flow_registry_export_json();
    if (reg_json) {
        cJSON* reg = cJSON_Parse(reg_json);
        if (reg) {
            cJSON_AddItemToObject(metrics, "registry", reg);
        }
        free(reg_json);
    }

    /* Sysmon */
    cJSON* sysmon_o = cJSON_AddObjectToObject(metrics, "sysmon");
    cJSON_AddNumberToObject(sysmon_o, "cpu_total_pct", ssnap.cpu_total_pct);
    cJSON_AddNumberToObject(sysmon_o, "cpu_user_pct", ssnap.cpu_user_pct);
    cJSON_AddNumberToObject(sysmon_o, "cpu_sys_pct", ssnap.cpu_sys_pct);
    cJSON_AddNumberToObject(sysmon_o, "cpu_iowait_pct", ssnap.cpu_iowait_pct);
    cJSON_AddNumberToObject(sysmon_o, "cpu_idle_pct", ssnap.cpu_idle_pct);
    cJSON_AddNumberToObject(sysmon_o, "cpu_count", ssnap.cpu_count);
    cJSON_AddNumberToObject(sysmon_o, "mem_total_kb", (double)ssnap.mem_total_kb);
    cJSON_AddNumberToObject(sysmon_o, "mem_used_kb", (double)ssnap.mem_used_kb);
    cJSON_AddNumberToObject(sysmon_o, "mem_used_pct", ssnap.mem_used_pct);
    cJSON_AddNumberToObject(sysmon_o, "mem_available_kb", (double)ssnap.mem_available_kb);
    cJSON_AddNumberToObject(sysmon_o, "proc_rss_kb", (double)ssnap.proc_rss_kb);
    cJSON_AddNumberToObject(sysmon_o, "proc_vms_kb", (double)ssnap.proc_vms_kb);
    cJSON_AddNumberToObject(sysmon_o, "disk_read_bps", ssnap.disk_read_bps);
    cJSON_AddNumberToObject(sysmon_o, "disk_write_bps", ssnap.disk_write_bps);
    cJSON_AddNumberToObject(sysmon_o, "load1", ssnap.load1);
    cJSON_AddNumberToObject(sysmon_o, "load5", ssnap.load5);
    cJSON_AddNumberToObject(sysmon_o, "load15", ssnap.load15);
    cJSON_AddNumberToObject(sysmon_o, "uptime_sec", ssnap.uptime_sec);
    cJSON_AddNumberToObject(sysmon_o, "thread_count", ssnap.thread_count);

    cJSON* threads_arr = cJSON_AddArrayToObject(sysmon_o, "threads");
    for (int ti = 0; ti < ssnap.thread_count && ti < 16; ti++) {
        SysMonitorThreadSnapshot* th = &ssnap.threads[ti];
        cJSON* thr = cJSON_CreateObject();
        cJSON_AddNumberToObject(thr, "tid", (double)(int)th->tid);
        cJSON_AddStringToObject(thr, "name", th->name);
        cJSON_AddNumberToObject(thr, "cpu_pct", th->cpu_pct);
        char state_str[2] = { th->state, '\0' };
        cJSON_AddStringToObject(thr, "state", state_str);
        cJSON_AddItemToArray(threads_arr, thr);
    }

    /* ── Print to string and write to file ── */
    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", g.state_file);
    FILE* jf = fopen(tmp_path, "w");
    if (jf) {
        fprintf(jf, "%s", json_str);
        fclose(jf);
        rename(tmp_path, g.state_file);
    }
    cJSON_free(json_str);

    /* Publish the same JSON via IPC dashboard bridge for flowmond */
    if (g.dashboard_ch) {
        size_t json_len;
        char* json_buf = NULL;
        FILE* rf = fopen(g.state_file, "rb");
        if (rf) {
            fseek(rf, 0, SEEK_END);
            long flen = ftell(rf);
            fseek(rf, 0, SEEK_SET);
            if (flen > 0) {
                json_buf = (char*)malloc((size_t)flen + 1);
                if (json_buf) {
                    json_len = fread(json_buf, 1, (size_t)flen, rf);
                    json_buf[json_len] = '\0';
                    dashboard_bridge_publish(g.dashboard_ch, json_buf, json_len);
                    free(json_buf);
                }
            }
            fclose(rf);
        }
    }
    free(topo_json);
}

/* ── 任务主循环（托管模式 execute） ──────────────────────────── */

static int monitor_execute(TaskBase* task) {
    pthread_setname_np(pthread_self(), "monitor");
    long period_us = (long)(1.0 / g.frequency_hz * 1e6);
    /* stats bridge subscriber 重试计数器：多进程启动顺序不定，若 monitor
     * 先于其它节点 publisher 启动，subscriber_open 返回 NULL。在主循环里
     * 重试，直到连上其它进程创建的共享内存通道（限 120 周期，2Hz≈60s）。 */
    int stats_sub_retry = 0;

    while (!task->should_stop) {
        usleep((unsigned long)period_us);
        if (task->should_stop) break;

        /* 重试 stats bridge subscriber（每个周期试一次，连上即止） */
        if (!g.stats_sub && stats_sub_retry < 120) {
            stats_sub_retry++;
            g.stats_sub = stats_bridge_subscriber_open(on_remote_stats, NULL);
            if (g.stats_sub) {
                ipc_channel_start(g.stats_sub);
                LOG_INFO("monitor", "stats bridge subscriber opened on retry #%d", stats_sub_retry);
            }
        }

        /* 收集并导出仪表盘 JSON */
        export_dashboard_json();

        /* Publish stats via IPC bridge for flowmond */
        if (g.stats_ch) {
            stats_bridge_publish(g.stats_ch, g.bus, "monitor_node");
        }

        /* 简略控制台输出 */
        uint64_t pub = 0, del = 0, drop = 0;
        if (g.bus) message_bus_get_stats(g.bus, &pub, &del, &drop);

        int task_count = 0;
        if (g.scheduler) task_count = scheduler_task_count(g.scheduler);

        LOG_INFO("monitor", "bus pub=%lu del=%lu drop=%lu tasks=%d",
                 (unsigned long)pub, (unsigned long)del, (unsigned long)drop, task_count);
    }

    LOG_INFO("monitor", "stopped");
    return 0;
}

/* 托管模式虚函数表：仅实现 execute()（完整主循环）。initialize/cleanup 由
 * task_thread_fn 在 execute 前后按需调用，这里不需要——节点初始化在
 * NodePlugin.init，资源释放在 NodePlugin.cleanup。 */
static const TaskInterface monitor_vtable = {
    .execute = monitor_execute,
};

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "perception/obstacles", "vehicle/state",
                                   "fusion/latency", "flowengine/node_info",
                                   "planning/trajectory", "road/geometry",
                                   "road/traffic_lights", "scene/frame", NULL };
static const char* s_outputs[] = { NULL };

static NodePlugin s_plugin;

static int monitor_init(MessageBus* bus, Transport* transport,
                        DiscoveryManager* discovery, Scheduler* scheduler,
                        const char* params_json) {
    memset(&g, 0, sizeof(g));
    g.bus         = bus;
    g.transport   = transport;
    g.discovery   = discovery;
    g.scheduler   = scheduler;

    /* state_file 可从环境变量或默认路径获得 */
    const char* sf = flowengine_state_file();
    snprintf(g.state_file, sizeof(g.state_file), "%s", sf ? sf : "/tmp/flow_topology.json");

    /* 解析参数 */
    g.frequency_hz = 10.0;
    g.lane_width   = 3.5;
    g.lane_count   = 2;
    if (params_json) {
        cJSON* root = cJSON_Parse(params_json);
        if (root) {
            cJSON* item;
            if ((item = cJSON_GetObjectItem(root, "state_file")) && cJSON_IsString(item)) {
                snprintf(g.state_file, sizeof(g.state_file), "%s", item->valuestring);
            }
            if ((item = cJSON_GetObjectItem(root, "lane_width"))) {
                g.lane_width = item->valuedouble;
            }
            if ((item = cJSON_GetObjectItem(root, "lane_count"))) {
                int val = (int)item->valuedouble;
                if (val >= 1 && val <= 8) g.lane_count = val;
            }
            cJSON_Delete(root);
        }
    }

    /* 创建 state_file 目录 */
    {
        char dir[512];
        snprintf(dir, sizeof(dir), "%s", g.state_file);
        char* slash = strrchr(dir, '/');
        if (slash) { *slash = '\0'; mkdir(dir, 0755); }
    }

    /* sysmon */
    g.sysmon = sysmonitor_create();

    /* 订阅 */
    transport_subscribe(transport, "perception/obstacles", on_obstacles, NULL);
    transport_subscribe(transport, "vehicle/state", on_vehicle_state, NULL);
    transport_subscribe(transport, "fusion/latency", on_fusion_latency, NULL);
    transport_subscribe(transport, "planning/trajectory", on_planning_trajectory, NULL);
    transport_subscribe(transport, "road/geometry", on_road_geometry, NULL);
    transport_subscribe(transport, "road/traffic_lights", on_traffic_lights, NULL);
    /* Phase 3: scene/frame — 从 flowsim_node 获取 road_network 供 3D 多段道路渲染 */
    transport_subscribe(transport, "scene/frame", on_scene_frame, NULL);
    /* 收集其他节点的自描述广播 (方案B: 数据驱动拓扑感知) */
    transport_subscribe(transport, "flowengine/node_info", on_node_info, NULL);
    g.node_info_count = 0;

    discovery_advertise(discovery, "perception/obstacles", 0x0B5A010Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state",        0x1C0E5A7Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "fusion/latency",       0x1A7E9C01u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/geometry",        0x80AD5C12u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "road/traffic_lights",  0x7E5C0FFEu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "scene/frame",          0x5CE4E011u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "flowengine/node_info", 0xF10E10F0u, CAP_SUBSCRIBER, 0);

    /* Open IPC stats bridge for flowmond (publisher) */
    g.stats_ch = stats_bridge_publisher_open();
    if (!g.stats_ch) {
        LOG_WARN("monitor", "stats bridge publisher open failed (flowmond not running yet)");
    } else {
        LOG_INFO("monitor", "stats bridge publisher opened");
    }

    /* 同时订阅 stats bridge，聚合其它进程的 bus/topic 统计。
     * flowmond 也会订阅，但 flowmond 的聚合只在 fallback 路径生效；
     * monitor_node 自己聚合后写进 dashboard JSON，保证主路径数据完整。
     * 注意：subscriber 必须在 publisher 之后 open，否则 publisher 端的
     * ipc_channel_publish 会因无 subscriber 而丢弃早期包（可接受，启动竞态）。 */
    pthread_mutex_init(&g.remote_stats_mutex, NULL);
    g.stats_sub = stats_bridge_subscriber_open(on_remote_stats, NULL);
    if (g.stats_sub) {
        ipc_channel_start(g.stats_sub);
        LOG_INFO("monitor", "stats bridge subscriber opened (aggregating remote stats)");
    } else {
        LOG_INFO("monitor", "stats bridge subscriber not yet available (single-process mode)");
    }

    /* Open IPC dashboard JSON bridge for flowmond */
    g.dashboard_ch = dashboard_bridge_publisher_open();
    if (!g.dashboard_ch) {
        LOG_WARN("monitor", "dashboard bridge publisher open failed (flowmond not running yet)");
    } else {
        LOG_INFO("monitor", "dashboard bridge publisher opened");
    }

    /* 托管模式：初始化嵌入的 TaskBase 并挂上 vtable。s_plugin.taskbase 在
     * 静态初始化里已指向 &g.taskbase，故此处只需填好其内容。max_frequency_hz
     * 取 g.frequency_hz，与 execute() 内 usleep 周期（1/frequency_hz）一致。 */
    TaskConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    snprintf(cfg.name, sizeof(cfg.name), "monitor");
    cfg.priority         = TASK_PRIORITY_NORMAL;
    cfg.max_frequency_hz = g.frequency_hz;
    cfg.enable_stats     = true;
    if (task_base_init(&g.taskbase, &monitor_vtable, &cfg) != 0) {
        LOG_WARN("monitor", "task_base_init failed");
        return -1;
    }

    LOG_INFO("monitor", "initialized (%.0f Hz, state_file=%s)",
             g.frequency_hz, g.state_file);
    return 0;
}

static int monitor_start(void) {
    /* 托管模式：node_start_managed 注册 taskbase 到调度器并派生工作线程跑
     * monitor_execute()。节点不再 pthread_create 自建线程。 */
    int rc = node_start_managed(&s_plugin, g.scheduler);
    if (rc != 0) {
        LOG_WARN("monitor", "node_start_managed failed: %d", rc);
        return rc;
    }
    LOG_INFO("monitor", "started (managed)");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void monitor_stop(void) {
    /* task_stop 置 should_stop=true 并 join 工作线程（monitor_execute 随即退出）。 */
    task_stop(&g.taskbase);
}
static void monitor_cleanup(void) {
    /* stop() 已 join 线程；此处再 task_stop 一次作幂等保险（STOPPED 态直接
     * 返回 0），随后释放 TaskBase 资源。sysmon / IPC 通道 / remote_stats_mutex
     * 与 taskbase 无关，保持原有清理流程不变。 */
    task_stop(&g.taskbase);
    task_base_destroy(&g.taskbase);
    if (g.sysmon) { sysmonitor_destroy(g.sysmon); g.sysmon = NULL; }
    if (g.stats_sub) { ipc_channel_stop(g.stats_sub); ipc_channel_close(g.stats_sub); g.stats_sub = NULL; }
    if (g.stats_ch) { ipc_channel_close(g.stats_ch); g.stats_ch = NULL; }
    if (g.dashboard_ch) { ipc_channel_close(g.dashboard_ch); g.dashboard_ch = NULL; }
    pthread_mutex_destroy(&g.remote_stats_mutex);
    LOG_INFO("monitor", "cleanup done");
}
static int  monitor_health(void)        { return 0; }

static NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "monitor",
    .version       = "1.0.0",
    .description   = "System monitor + dashboard JSON exporter",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = monitor_init,
    .start         = monitor_start,
    .stop          = monitor_stop,
    .cleanup       = monitor_cleanup,
    .health        = monitor_health,
    .taskbase      = &g.taskbase,   /* v2: 托管模式钩子，指向嵌入的 TaskBase */
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
