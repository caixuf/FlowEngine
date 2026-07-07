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

    /* 从 discovery JSON 取 self 字段，用收集到的 node_info 填 nodes 数组
     * (方案B: nodes 来自各节点广播的 flowengine/node_info，
     *  而不是 flow_registry / discovery peer，解决静态库全局状态不跨 dlopen 共享的问题) */
    const char* self_name = "flow_launcher";
    fprintf(jf, "{\"self\":\"%s\",\"timestamp\":%.3f,\"nodes\":[", self_name, timestamp);
    for (int i = 0; i < g.node_info_count; i++) {
        fprintf(jf, "%s%s", i ? "," : "", g.node_info_json[i]);
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

    /* 障碍物 (简化为从 vehicle/state 提取) */
    double ox[3], oy[3], ovx[3];
    char kn[16];
    for (int i = 0; i < 3; i++) {
        snprintf(kn, sizeof(kn), "ox%d", i);
        ox[i] = json_extract_double(g.latest_vehicle_state, kn);
        snprintf(kn, sizeof(kn), "oy%d", i);
        oy[i] = json_extract_double(g.latest_vehicle_state, kn);
        snprintf(kn, sizeof(kn), "ov%d", i);
        ovx[i] = json_extract_double(g.latest_vehicle_state, kn);
    }
    fprintf(jf, "\"obstacles\":[");
    for (int i = 0; i < 3; i++) {
        double rx = ox[i] - ego_x;
        double ry = oy[i] - ego_y;
        fprintf(jf, "%s{\"id\":%d,\"type\":\"car\",\"x\":%.2f,\"y\":%.2f,"
                "\"vx\":%.2f,\"vy\":0,\"len\":4.6,\"wid\":2.0}",
                i > 0 ? "," : "", i, rx, ry, ovx[i]);
    }
    fprintf(jf, "],");

    /* LiDAR 点云 (简略) */
    fprintf(jf, "\"lidar\":[");
    int lp = 0;
    for (int oi = 0; oi < 3; oi++) {
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
                                   "fusion/latency", NULL };
static const char* s_outputs[] = { NULL };

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
