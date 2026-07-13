/**
 * monitor_node.c — 系统监控节点插件
 *
 * 订阅感知、车辆状态、融合延迟等 topic，
 * 收集系统资源指标，导出 JSON 供 FlowBoard 仪表盘展示。
 *
 * NodePlugin 接口，编译为 libmonitor_node.so。
 */

#include "node_plugin.h"
#include "sysmonitor.h"
#include "flow_registry.h"
#include "logger.h"
#include "stats_bridge.h"
#include "dashboard_bridge.h"
#include "adas_msgs_gen.h"

#include <stdlib.h>
#include <string.h>
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

    pthread_t   thread;
    volatile int running;
    volatile int should_stop;

    SysMonitor* sysmon;

    /* 订阅数据缓存 */
    char latest_obstacles_json[2048];
    char latest_vehicle_state[1024];
    double fusion_lat_avg_us;
    double fusion_lat_p50_us;
    double fusion_lat_p99_us;
    volatile int has_obstacles;
    volatile int has_vehicle_state;

    /* 节点拓扑: 从 flowengine/node_info topic 收集 (B 方案) */
#define MAX_TOPO_NODES 16
    char node_info_json[MAX_TOPO_NODES][512];  /* 每个节点的原始 JSON */
    int  node_info_count;

    /* 导出路径 */
    char state_file[512];

    /* 跨进程 stats bridge */
    IpcChannel* stats_ch;

    /* 跨进程 dashboard JSON bridge */
    IpcChannel* dashboard_ch;

    /* 配置 */
    double frequency_hz;
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
    size_t copy = msg->data_size < sizeof(g.node_info_json[0]) - 1
                  ? msg->data_size : sizeof(g.node_info_json[0]) - 1;
    memcpy(g.node_info_json[g.node_info_count], msg->data, copy);
    g.node_info_json[g.node_info_count][copy] = '\0';
    g.node_info_count++;
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
    const char* p;
    if ((p = strstr(d, "\"avg_us\":")))  sscanf(p + 9, "%lf", &g.fusion_lat_avg_us);
    if ((p = strstr(d, "\"p50_us\":")))  sscanf(p + 9, "%lf", &g.fusion_lat_p50_us);
    if ((p = strstr(d, "\"p99_us\":")))  sscanf(p + 9, "%lf", &g.fusion_lat_p99_us);
}

/* ── JSON 辅助: 从 vehicle/state JSON 提取字段 ─────────────── */

static double json_extract_double(const char* json, const char* key) {
    if (!json || !key) return 0.0;
    char search[64];
    int klen = snprintf(search, sizeof(search), "\"%s\":", key);
    const char* p = strstr(json, search);
    if (!p) return 0.0;
    double val = 0.0;
    sscanf(p + klen, "%lf", &val);
    return val;
}

/* Extract an integer value from a JSON string, e.g. "n_obs":4 → 4. */
static int json_extract_int(const char* json, const char* key) {
    if (!json || !key) return 0;
    char search[64];
    int klen = snprintf(search, sizeof(search), "\"%s\":", key);
    const char* p = strstr(json, search);
    if (!p) return 0;
    int val = 0;
    sscanf(p + klen, "%d", &val);
    return val;
}

/* Extract a quoted string value from a JSON string, e.g. "ot0":"car" → "car".
 * dst is NUL-terminated and truncated to dst_size-1 bytes. */
static void json_extract_str(const char* json, const char* key,
                             char* dst, size_t dst_size) {
    if (!json || !key || !dst || dst_size == 0) return;
    dst[0] = '\0';
    char search[64];
    int klen = snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char* p = strstr(json, search);
    if (!p) return;
    p += klen;
    size_t n = 0;
    while (*p && *p != '"' && n < dst_size - 1) dst[n++] = *p++;
    dst[n] = '\0';
}

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
static int emit_nodes_from_discovery(FILE* jf, const TopologyGraph* g_topo) {
    int written = 0;
    for (uint32_t i = 0; i < g_topo->node_count; i++) {
        const NodeInfo* n = &g_topo->nodes[i];
        if (!n->alive) continue;
        fprintf(jf, "%s{\"name\":\"%s\",\"version\":\"\",\"description\":\"\","
                    "\"pid\":%u,\"alive\":true,\"topics\":[",
                written ? "," : "", n->name, n->pid);
        for (uint32_t j = 0; j < n->topic_count; j++) {
            bool is_pub = (n->topics[j].capabilities & CAP_PUBLISHER) != 0;
            bool is_sub = (n->topics[j].capabilities & CAP_SUBSCRIBER) != 0;
            const char* role = is_pub && is_sub ? "pubsub" : (is_pub ? "pub" : "sub");
            fprintf(jf, "%s{\"topic\":\"%s\",\"role\":\"%s\",\"caps\":%u}",
                    j ? "," : "", n->topics[j].topic, role, n->topics[j].capabilities);
        }
        fprintf(jf, "]}");
        written++;
    }
    return written;
}

/* ── 导出 JSON 到 state_file ──────────────────────────────── */

static void export_dashboard_json(void) {
    struct timespec now_ts;
    clock_gettime(CLOCK_REALTIME, &now_ts);
    double timestamp = (double)now_ts.tv_sec + (double)now_ts.tv_nsec / 1000000000.0;

    /* 收集指标 */
    uint64_t pub = 0, del = 0, drop = 0;
    if (g.bus) message_bus_get_stats(g.bus, &pub, &del, &drop);

    TransportStats ts;
    memset(&ts, 0, sizeof(ts));
    if (g.transport) transport_get_stats(g.transport, &ts);

    int task_count = 0;
    if (g.scheduler) task_count = scheduler_task_count(g.scheduler);

    SysMonitorSnapshot ssnap;
    memset(&ssnap, 0, sizeof(ssnap));
    if (g.sysmon) sysmonitor_snapshot(g.sysmon, &ssnap);

    char* topo_json = g.discovery ? discovery_export_json(g.discovery) : NULL;

    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", g.state_file);
    FILE* jf = fopen(tmp_path, "w");
    if (!jf) { free(topo_json); return; }

    /* nodes 数组来源:
     *  - 单进程 (dlopen): 各节点广播的 flowengine/node_info (方案B, 含 version/desc)
     *  - 多进程 (fork+exec): node_info 不跨进程, 回退到 discovery 跨进程拓扑
     * 判据: discovery 已知的存活节点数 > 收到的 node_info 广播数, 说明有节点的
     *       自描述广播没能抵达 monitor (多进程), 改用 discovery 补全拓扑。 */
    const char* self_name = "flow_launcher";
    const TopologyGraph* topo = g.discovery ? discovery_get_topology(g.discovery) : NULL;
    int disc_alive = 0;
    if (topo) {
        for (uint32_t i = 0; i < topo->node_count; i++)
            if (topo->nodes[i].alive) disc_alive++;
    }

    fprintf(jf, "{\"self\":\"%s\",\"timestamp\":%.3f,\"nodes\":[", self_name, timestamp);
    if (topo && disc_alive > g.node_info_count) {
        int written = emit_nodes_from_discovery(jf, topo);
        /* discovery 拓扑不含本节点自身 (self 只广播 my_topics, 不进自己的
         * topology)。补上 monitor 收到的本地 node_info 广播 (通常就是它自己),
         * 按 name 去重, 使拓扑图包含 monitor 节点。 */
        for (int i = 0; i < g.node_info_count; i++) {
            char nm[64] = "";
            json_extract_str(g.node_info_json[i], "name", nm, sizeof(nm));
            bool dup = false;
            for (uint32_t k = 0; nm[0] && k < topo->node_count; k++) {
                if (topo->nodes[k].alive && strcmp(topo->nodes[k].name, nm) == 0) {
                    dup = true; break;
                }
            }
            if (!dup) fprintf(jf, "%s%s", written++ ? "," : "", g.node_info_json[i]);
        }
    } else {
        for (int i = 0; i < g.node_info_count; i++) {
            fprintf(jf, "%s%s", i ? "," : "", g.node_info_json[i]);
        }
    }
    fprintf(jf, "]");

    fprintf(jf, ",\"metrics\":{"
            "\"bus\":{\"published\":%lu,\"delivered\":%lu,\"dropped\":%lu},"
            "\"transport\":{\"local_pub\":%lu,\"remote_pub\":%lu},"
            "\"scheduler\":{\"tasks\":%d,\"mode\":\"CHOREO\"},",
            (unsigned long)pub, (unsigned long)del, (unsigned long)drop,
            (unsigned long)ts.local_published, (unsigned long)ts.remote_published,
            task_count);

    /* 融合延迟 */
    fprintf(jf, "\"latency\":{\"avg_us\":%.0f,\"p50_us\":%.0f,\"p99_us\":%.0f},",
            g.fusion_lat_avg_us, g.fusion_lat_p50_us, g.fusion_lat_p99_us);

    /* Topic 统计 */
    TopicStats tstats[16];
    int nt = g.bus ? message_bus_get_all_topic_stats(g.bus, tstats, 16) : 0;
    fprintf(jf, "\"topics\":[");
    for (int ti = 0; ti < nt; ti++) {
        uint64_t avg_lat = tstats[ti].deliver_count > 0
            ? tstats[ti].total_latency_us / tstats[ti].deliver_count : 0;
        fprintf(jf, "%s{\"topic\":\"%s\",\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
                "\"lat_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
                ti > 0 ? "," : "",
                tstats[ti].topic,
                (unsigned long)tstats[ti].publish_count,
                (unsigned long)tstats[ti].deliver_count,
                (unsigned long)tstats[ti].drop_count,
                (unsigned long)avg_lat,
                tstats[ti].frequency_hz,
                tstats[ti].subscriber_count);
    }
    fprintf(jf, "],");

    /* 车辆状态 */
    double spd = json_extract_double(g.latest_vehicle_state, "spd");
    double tgt = json_extract_double(g.latest_vehicle_state, "tgt");
    double thr = json_extract_double(g.latest_vehicle_state, "thr");
    double brk = json_extract_double(g.latest_vehicle_state, "brk");
    double vx  = json_extract_double(g.latest_vehicle_state, "x");
    fprintf(jf, "\"vehicle\":{"
            "\"speed\":%.1f,\"target_speed\":%.1f,"
            "\"throttle\":%.3f,\"brake\":%.3f,"
            "\"x\":%.1f,\"error\":%.1f},",
            spd, tgt, thr, brk, vx, tgt - spd);

    /* 3D 场景（从 vehicle/state 提取） */
    double ego_x = json_extract_double(g.latest_vehicle_state, "x");
    double ego_y = json_extract_double(g.latest_vehicle_state, "y");
    double hdg   = json_extract_double(g.latest_vehicle_state, "hdg");
    double steer = json_extract_double(g.latest_vehicle_state, "st");

    fprintf(jf, "\"scene\":{");
    fprintf(jf, "\"ego\":{\"x\":%.2f,\"y\":%.2f,\"heading\":%.3f,"
            "\"speed\":%.2f,\"steer\":%.3f},",
            ego_x, ego_y, hdg, spd, steer);
    fprintf(jf, "\"lane\":{\"width\":3.5,\"count\":2},");

    /* 障碍物（从 vehicle/state 动态读取，障碍物数量由 n_obs 决定） */
    int n_obs = json_extract_int(g.latest_vehicle_state, "n_obs");
    if (n_obs < 0 || n_obs > 16) n_obs = 0;

    /* Must match SIM_OBSTACLE_COUNT in sim_world_node.c */
#define MAX_OBS_SCENE 16
    /* Default dimensions used as fallback for legacy vehicle/state messages that
     * pre-date the ot%d/ol%d/ow%d fields (matches scenario_loader defaults). */
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
        json_extract_str(g.latest_vehicle_state, kn, otype[i], sizeof(otype[i]));
        if (otype[i][0] == '\0') {
            /* Legacy messages without type field: default to car */
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
    fprintf(jf, "\"obstacles\":[");
    for (int i = 0; i < n_obs; i++) {
        double rx = ox[i] - ego_x;
        double ry = oy[i] - ego_y;
        fprintf(jf, "%s{\"id\":%d,\"type\":\"%s\",\"x\":%.2f,\"y\":%.2f,"
            "\"vx\":%.2f,\"vy\":%.2f,\"len\":%.2f,\"wid\":%.2f}",
            i > 0 ? "," : "", i, otype[i], rx, ry, ovx[i], ovy[i], olen[i], owid[i]);
    }
    fprintf(jf, "],");

    /* LiDAR 点云（对每个障碍物生成环形点，加地面环带） */
    fprintf(jf, "\"lidar\":[");
    int lp = 0;
    for (int oi = 0; oi < n_obs; oi++) {
        double rx = ox[oi] - ego_x;
        double ry = oy[oi] - ego_y;
        if (rx < -20 || rx > 60) continue;
        for (int k = 0; k < 6; k++) {
            double a = (double)k / 6.0 * (2.0 * M_PI);
            fprintf(jf, "%s[%.2f,%.2f,%.2f]", lp++ ? "," : "",
                    rx + cos(a) * 1.0, ry + sin(a) * 2.3, 0.4);
        }
    }
    for (int k = 0; k < 12; k++) {
        double a = (double)k / 12.0 * (2.0 * M_PI);
        fprintf(jf, "%s[%.2f,%.2f,%.2f]", lp++ ? "," : "",
                cos(a) * (10.0 + (k % 3) * 4.0),
                sin(a) * (10.0 + (k % 3) * 4.0), 0.0);
    }
    fprintf(jf, "]},");

    /* Registry */
    char* reg_json = flow_registry_export_json();
    if (reg_json) {
        fprintf(jf, "\"registry\":%s,", reg_json);
        free(reg_json);
    }

    /* Sysmon */
    fprintf(jf, "\"sysmon\":{"
            "\"cpu_total_pct\":%.1f,"
            "\"cpu_user_pct\":%.1f,"
            "\"cpu_sys_pct\":%.1f,"
            "\"cpu_iowait_pct\":%.1f,"
            "\"cpu_idle_pct\":%.1f,"
            "\"cpu_count\":%d,"
            "\"mem_total_kb\":%llu,"
            "\"mem_used_kb\":%llu,"
            "\"mem_used_pct\":%.1f,"
            "\"mem_available_kb\":%llu,"
            "\"proc_rss_kb\":%llu,"
            "\"proc_vms_kb\":%llu,"
            "\"disk_read_bps\":%.0f,"
            "\"disk_write_bps\":%.0f,"
            "\"load1\":%.2f,"
            "\"load5\":%.2f,"
            "\"load15\":%.2f,"
            "\"uptime_sec\":%.0f,"
            "\"thread_count\":%d",
            ssnap.cpu_total_pct, ssnap.cpu_user_pct,
            ssnap.cpu_sys_pct, ssnap.cpu_iowait_pct,
            ssnap.cpu_idle_pct, ssnap.cpu_count,
            (unsigned long long)ssnap.mem_total_kb,
            (unsigned long long)ssnap.mem_used_kb,
            ssnap.mem_used_pct,
            (unsigned long long)ssnap.mem_available_kb,
            (unsigned long long)ssnap.proc_rss_kb,
            (unsigned long long)ssnap.proc_vms_kb,
            ssnap.disk_read_bps, ssnap.disk_write_bps,
            ssnap.load1, ssnap.load5, ssnap.load15,
            ssnap.uptime_sec,
            ssnap.thread_count);

    fprintf(jf, ",\"threads\":[");
    for (int ti = 0; ti < ssnap.thread_count && ti < 16; ti++) {
        SysMonitorThreadSnapshot* th = &ssnap.threads[ti];
        fprintf(jf, "%s{\"tid\":%d,\"name\":\"%s\","
                "\"cpu_pct\":%.1f,\"state\":\"%c\"}",
                ti > 0 ? "," : "",
                (int)th->tid, th->name,
                th->cpu_pct, th->state);
    }
    fprintf(jf, "]}");

    fprintf(jf, "}}");  /* close metrics + close top-level */
    fclose(jf);
    rename(tmp_path, g.state_file);

    /* Publish the same JSON via IPC dashboard bridge for flowmond */
    if (g.dashboard_ch) {
        /* Read the file back and publish */
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

/* ── 任务线程 ────────────────────────────────────────────────── */

static void* monitor_thread(void* arg) {
    (void)arg;
    pthread_setname_np(pthread_self(), "monitor");
    long period_us = (long)(1.0 / g.frequency_hz * 1e6);

    while (!g.should_stop) {
        usleep((unsigned long)period_us);
        if (g.should_stop) break;

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
    return NULL;
}

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { "perception/obstacles", "vehicle/state",
                                   "fusion/latency", "flowengine/node_info", NULL };
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
    g.should_stop = 0;

    /* state_file 可从环境变量或默认路径获得 */
    const char* sf = flowengine_state_file();
    snprintf(g.state_file, sizeof(g.state_file), "%s", sf ? sf : "/tmp/flow_topology.json");

    /* 解析参数 */
    g.frequency_hz = 10.0;
    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"state_file\":"))) {
            char val[256] = "";
            sscanf(p + 13, "%255s", val);
            /* strip quotes */
            size_t vl = strlen(val);
            if (vl > 2 && val[0] == '"' && val[vl-1] == '"') {
                val[vl-1] = '\0';
                snprintf(g.state_file, sizeof(g.state_file), "%s", val + 1);
            }
        }
        if ((p = strstr(params_json, "\"export_scene\":"))) {
            /* 当前始终导出 */
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
    /* 收集其他节点的自描述广播 (方案B: 数据驱动拓扑感知) */
    transport_subscribe(transport, "flowengine/node_info", on_node_info, NULL);
    g.node_info_count = 0;

    discovery_advertise(discovery, "perception/obstacles", 0x0B5A010Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "vehicle/state",        0x1C0E5A7Eu, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "fusion/latency",       0x1A7E9C01u, CAP_SUBSCRIBER, 0);
    discovery_advertise(discovery, "flowengine/node_info", 0xF10E10F0u, CAP_SUBSCRIBER, 0);

    /* Open IPC stats bridge for flowmond */
    g.stats_ch = stats_bridge_publisher_open();
    if (!g.stats_ch) {
        LOG_WARN("monitor", "stats bridge publisher open failed (flowmond not running yet)");
    } else {
        LOG_INFO("monitor", "stats bridge publisher opened");
    }

    /* Open IPC dashboard JSON bridge for flowmond */
    g.dashboard_ch = dashboard_bridge_publisher_open();
    if (!g.dashboard_ch) {
        LOG_WARN("monitor", "dashboard bridge publisher open failed (flowmond not running yet)");
    } else {
        LOG_INFO("monitor", "dashboard bridge publisher opened");
    }

    LOG_INFO("monitor", "initialized (%.0f Hz, state_file=%s)",
             g.frequency_hz, g.state_file);
    return 0;
}

static int monitor_start(void) {
    g.running = 1; g.should_stop = 0;
    if (pthread_create(&g.thread, NULL, monitor_thread, NULL) != 0) return -1;
    LOG_INFO("monitor", "started");
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void monitor_stop(void)          { g.should_stop = 1; }
static void monitor_cleanup(void) {
    if (g.running) { g.should_stop = 1; pthread_join(g.thread, NULL); g.running = 0; }
    if (g.sysmon) { sysmonitor_destroy(g.sysmon); g.sysmon = NULL; }
    if (g.stats_ch) { ipc_channel_close(g.stats_ch); g.stats_ch = NULL; }
    if (g.dashboard_ch) { ipc_channel_close(g.dashboard_ch); g.dashboard_ch = NULL; }
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
};

NodePlugin* node_get_plugin(void) { return &s_plugin; }
