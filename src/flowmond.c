/**
 * flowmond.c — FlowEngine 监控守护进程 (独立进程, cyber_monitor 等价物)
 *
 * 用法:
 *   flowmond [--port 8800] [--config monitor.json]
 *
 * 功能:
 *   - UDP 发现所有业务节点
 *   - 订阅 topic 统计 (通配符 *)
 *   - HTTP Dashboard (:8800) — 实时 topic 监控表
 *   - JSON API — /api/topology /api/topics /api/stream
 *   - 告警规则引擎
 *   - 独立进程，不影响业务节点性能
 *
 * 编译: 由 CMakeLists.txt 自动处理
 */

#include "message_bus.h"
#include "discovery.h"
#include "monitor_server.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

static volatile bool g_running = true;

static void sig_handler(int sig) {
    (void)sig;
    g_running = false;
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

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
            port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--config") == 0 && i + 1 < argc)
            config_file = argv[++i];
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

    /* ── 订阅所有 topic (通配符) ── */
    /* In production, the monitor subscribes to each node's stats topic
     * via IPC or TCP bridge. For single-process demo, subscribe to local bus. */
    message_bus_subscribe(bus, "*", NULL, NULL);
    LOG_INFO("flowmond", "subscribed to all topics");

    /* ── 启动 HTTP 监控服务器 ── */
    MonitorServer* ms = monitor_server_create(bus, dm, port);
    monitor_server_start(ms);
    LOG_INFO("flowmond", "dashboard: http://localhost:%d", port);
    printf("  Endpoints:\n");
    printf("    /              Live dashboard\n");
    printf("    /api/topology  Bus + topology JSON\n");
    printf("    /api/topics    Per-topic QoS stats\n");
    printf("    /api/stream    SSE real-time push\n");
    printf("\n");

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
    monitor_server_stop(ms);
    monitor_server_destroy(ms);
    discovery_stop(dm);
    discovery_destroy(dm);
    message_bus_destroy(bus);
    log_shutdown();

    printf("[flowmond] stopped\n");
    return 0;
}
