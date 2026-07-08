/**
 * flowmond.c — FlowEngine 监控守护进程 (独立进程, cyber_monitor 等价物)
 *
 * 用法:
 *   flowmond [--port 8800] [--config monitor.json]
 *
 * 功能:
 *   - UDP 发现所有业务节点
 *   - HTTP Dashboard (:8800) — 实时 topic 监控表（本地 + 远程进程统计）
 *   - JSON API — /api/topology /api/topics /api/stream
 *   - IPC stats bridge — 聚合 flow_e2e 等进程的 bus 统计数据
 *   - 告警规则引擎
 *   - 独立进程，不影响业务节点性能
 *
 * 编译: 由 CMakeLists.txt 自动处理
 */

#include "message_bus.h"
#include "discovery.h"
#include "monitor_server.h"
#include "stats_bridge.h"
#include "dashboard_bridge.h"
#include "logger.h"
#include <libgen.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

static volatile bool g_running = true;

static void sig_handler(int sig) {
    (void)sig;
    g_running = false;
}

/* ── IPC stats bridge ────────────────────────────────────── */

/** Called by the IPC background thread when a StatsPacket arrives */
static void on_remote_stats(const Message* msg, void* user_data) {
    if (!msg || msg->data_size < sizeof(StatsPacket)) return;
    MonitorServer* ms = (MonitorServer*)user_data;
    const StatsPacket* pkt = (const StatsPacket*)msg->data;
    monitor_server_inject_remote_stats(ms, pkt);
    /* Stats arrive at ~10Hz — only log count changes to avoid log spam. */
    static uint32_t last_count = 0;
    if (pkt->topic_count != last_count) {
        LOG_INFO("flowmond", "stats bridge: received %u topics from '%s'",
                 pkt->topic_count, pkt->source_name);
        last_count = pkt->topic_count;
    }
}

typedef struct {
    MonitorServer* ms;
} ReconnectArgs;

/**
 * Background thread: keep trying to connect to the stats IPC channel
 * published by business processes (e.g., flow_e2e).
 *
 * Same reconnect-on-idle logic as the dashboard bridge: when the publisher
 * restarts it creates new shm/sem, so the subscriber must detect the silence
 * and reconnect.
 */
static void* stats_bridge_reconnect_fn(void* arg) {
    ReconnectArgs* ra = (ReconnectArgs*)arg;
    IpcChannel* sub = NULL;

    while (g_running) {
        if (!sub) {
            sub = stats_bridge_subscriber_open(on_remote_stats, ra->ms);
            if (sub) {
                ipc_channel_start(sub);
                LOG_INFO("flowmond", "stats bridge: connected to IPC channel '%s'",
                         STATS_BRIDGE_CHANNEL);
            } else {
                sleep(2);  /* publisher not up yet, retry */
            }
        } else {
            sleep(1);
            double age = monitor_server_stats_age_sec(ra->ms);
            if (age > (double)IPC_RECONNECT_STALE_SEC) {
                ipc_channel_stop(sub);
                ipc_channel_close(sub);
                sub = NULL;
                sleep(1);  /* let publisher unlink old shm before retry */
            }
        }
    }

    if (sub) ipc_channel_close(sub);
    free(ra);
    return NULL;
}

/* ── Dashboard JSON bridge ──────────────────────────── */

/** Called when a complete dashboard JSON snapshot arrives from monitor_node */
static void on_dashboard_json(const char* json, size_t len, void* user_data) {
    MonitorServer* ms = (MonitorServer*)user_data;
    monitor_server_inject_dashboard_json(ms, json, len);
}

/**
 * Background thread: keep trying to connect to the dashboard IPC channel
 * published by monitor_node.
 *
 * When the pipeline is killed and restarted, the IPC publisher creates NEW
 * shared memory and semaphores.  The subscriber must detect this (by
 * observing that no data has arrived for IPC_RECONNECT_STALE_SEC seconds)
 * and reconnect to the new channel.
 */
static void* dashboard_bridge_reconnect_fn(void* arg) {
    MonitorServer* ms = (MonitorServer*)arg;
    IpcChannel* sub = NULL;

    while (g_running) {
        if (!sub) {
            sub = dashboard_bridge_subscriber_open(on_dashboard_json, ms);
            if (sub) {
                ipc_channel_start(sub);
                LOG_INFO("flowmond", "dashboard bridge: connected to IPC channel '%s'",
                         DASHBOARD_BRIDGE_CHANNEL);
            } else {
                sleep(2);  /* publisher not up yet, retry */
            }
        } else {
            sleep(1);
            double age = monitor_server_dashboard_age_sec(ms);
            if (age > (double)IPC_RECONNECT_STALE_SEC) {
                ipc_channel_stop(sub);
                ipc_channel_close(sub);
                sub = NULL;
                /* Brief sleep so the publisher has time to unlink old shm
                 * before we retry — avoids reconnecting to a dead channel. */
                sleep(1);
            }
        }
    }

    if (sub) ipc_channel_close(sub);
    return NULL;
}

/* ── 告警检查 ────────────────────────────────────────────── */

typedef struct {
    const char* name;
    int         max_drop_rate;       /**< 告警阈值 (每周期) */
    int         max_latency_us;      /**< 延迟告警阈值 */
    int         max_node_offline_sec; /**< 节点离线告警 */
} AlertRule;

static void check_alerts(MessageBus* bus, const AlertRule* rules, int rule_count) {
    TopicStats tstats[32];
    int nt = message_bus_get_all_topic_stats(bus, tstats, 32);

    for (int r = 0; r < rule_count; r++) {
        const AlertRule* rule = &rules[r];

        for (int i = 0; i < nt; i++) {
            /* Check drop rate */
            if (rule->max_drop_rate > 0 && tstats[i].drop_count > 0) {
                double drop_rate = tstats[i].deliver_count > 0
                    ? (double)tstats[i].drop_count / (double)(tstats[i].publish_count + 1) * 100.0
                    : 0;
                if (drop_rate > (double)rule->max_drop_rate) {
                    LOG_WARN("alert", "[%s] %s: drop rate %.1f%% (threshold %d%%)",
                             rule->name, tstats[i].topic, drop_rate, rule->max_drop_rate);
                }
            }

            /* Check latency */
            if (rule->max_latency_us > 0 && tstats[i].deliver_count > 0) {
                uint64_t avg_lat = tstats[i].total_latency_us / tstats[i].deliver_count;
                if ((int64_t)avg_lat > (int64_t)rule->max_latency_us) {
                    LOG_WARN("alert", "[%s] %s: avg latency %luus (threshold %dus)",
                             rule->name, tstats[i].topic,
                             (unsigned long)avg_lat, rule->max_latency_us);
                }
            }
        }
    }
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    int port = 8800;
    const char* config_file = NULL;
    char html_path[512] = "";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
            port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--config") == 0 && i + 1 < argc)
            config_file = argv[++i];
        else if (strcmp(argv[i], "--html-path") == 0 && i + 1 < argc) {
            snprintf(html_path, sizeof(html_path), "%s", argv[++i]);
        }
    }

    /* Auto-detect html_path if not specified: look next to the binary */
    if (!html_path[0]) {
        char self_path[1024];
        ssize_t len = readlink("/proc/self/exe", self_path, sizeof(self_path) - 1);
        if (len > 0) {
            self_path[len] = '\0';
            /* dirname may modify input, use a copy */
            char dir_buf[1024];
            snprintf(dir_buf, sizeof(dir_buf), "%s", self_path);
            char* dir = dirname(dir_buf);
            snprintf(html_path, sizeof(html_path),
                     "%s/../../tools/flowboard.html", dir);
        }
    }

    log_init(LOG_INFO, NULL);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("╔══════════════════════════════════════════╗\n");
    printf("║  FlowEngine Monitor Daemon (flowmond)     ║\n");
    printf("║  Port: %-5d  Config: %-20s ║\n", port, config_file ? config_file : "(defaults)");
    printf("╚══════════════════════════════════════════╝\n\n");

    /* ── 创建本地总线 (监控专用) ── */
    MessageBus* bus = message_bus_create("flowmond_bus");
    LOG_INFO("flowmond", "message bus created");

    /* ── 启动服务发现 ── */
    DiscoveryManager* dm = discovery_create("flowmond", CAP_SUBSCRIBER);
    discovery_start(dm);
    LOG_INFO("flowmond", "discovery started — watching for nodes");

    /* ── 启动 HTTP 监控服务器 ── */
    MonitorServer* ms = monitor_server_create(bus, dm, port,
                               html_path[0] ? html_path : NULL);
    monitor_server_start(ms);
    LOG_INFO("flowmond", "dashboard: http://localhost:%d", port);
    printf("  Endpoints:\n");
    printf("    /              Live dashboard\n");
    printf("    /api/topology  Bus + topology JSON\n");
    printf("    /api/topics    Per-topic QoS stats (local + remote)\n");
    printf("    /api/stream    SSE real-time push\n");
    printf("\n");

    /* ── IPC stats bridge: subscribe to stats from other processes ── */
    ReconnectArgs* ra = (ReconnectArgs*)malloc(sizeof(ReconnectArgs));
    ra->ms = ms;
    pthread_t stats_reconnect_thread;
    pthread_create(&stats_reconnect_thread, NULL, stats_bridge_reconnect_fn, ra);
    LOG_INFO("flowmond", "stats bridge: watching for IPC channel '%s'",
             STATS_BRIDGE_CHANNEL);

    /* ── IPC dashboard JSON bridge: subscribe to full JSON from monitor_node ── */
    pthread_t dashboard_reconnect_thread;
    pthread_create(&dashboard_reconnect_thread, NULL, dashboard_bridge_reconnect_fn, ms);
    LOG_INFO("flowmond", "dashboard bridge: watching for IPC channel '%s'",
             DASHBOARD_BRIDGE_CHANNEL);

    /* ── 告警规则 ── */
    AlertRule rules[] = {
        { .name = "default", .max_drop_rate = 10, .max_latency_us = 5000, .max_node_offline_sec = 30 },
    };

    /* ── 主循环 ── */
    LOG_INFO("flowmond", "running (Ctrl+C to stop)");
    while (g_running) {
        sleep(5);
        check_alerts(bus, rules, 1);

        /* Print status summary */
        uint64_t pub, del, drop;
        message_bus_get_stats(bus, &pub, &del, &drop);
        TopicStats tstats[32];
        int nt = message_bus_get_all_topic_stats(bus, tstats, 32);
        const TopologyGraph* topo = discovery_get_topology(dm);

        printf("[flowmond] topics=%d nodes=%u pub=%lu del=%lu drop=%lu\n",
               nt, topo ? topo->node_count : 0,
               (unsigned long)pub, (unsigned long)del, (unsigned long)drop);
    }

    /* ── 清理 ── */
    pthread_join(stats_reconnect_thread, NULL);
    pthread_join(dashboard_reconnect_thread, NULL);
    monitor_server_stop(ms);
    monitor_server_destroy(ms);
    discovery_stop(dm);
    discovery_destroy(dm);
    message_bus_destroy(bus);
    log_shutdown();

    printf("[flowmond] stopped\n");
    return 0;
}
