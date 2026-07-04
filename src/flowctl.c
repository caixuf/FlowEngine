/**
 * flowctl.c — FlowEngine 命令行工具
 *
 * 用法:
 *   flowctl list tasks           列出所有注册的 task
 *   flowctl list topics          列出所有 topic 及统计
 *   flowctl list plugins         列出加载的插件
 *   flowctl graph                打印拓扑图
 *   flowctl state <task>         查看 task 状态机
 *   flowctl topic stats <topic>  查看 topic 统计
 *   flowctl bag info <file>      查看 bag 文件信息
 *   flowctl schema <type>        查看类型 schema
 *   flowctl version              版本信息
 *   flowctl dashboard            启动实时仪表盘
 */

#include "message_bus.h"
#include "discovery.h"
#include "serializer.h"
#include "state_machine.h"
#include "bag.h"
#include "logger.h"
#include "scheduler.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#define FLOWENGINE_VERSION "1.0.0"

/* ── 帮助 ────────────────────────────────────────────────── */

static void print_usage(void) {
    printf("FlowEngine CLI v%s — Interactive command-line tool\n\n", FLOWENGINE_VERSION);
    printf("Usage: flowctl <command> [args]\n\n");
    printf("Commands:\n");
    printf("  list tasks              List registered tasks\n");
    printf("  list topics             List all topics with stats\n");
    printf("  list plugins            List loaded plugins\n");
    printf("  graph                   Print topology graph\n");
    printf("  state <task>            Show task state machine\n");
    printf("  topic stats <topic>     Per-topic statistics\n");
    printf("  bag info <file>         Bag file metadata\n");
    printf("  schema <type>           Type information\n");
    printf("  dashboard               Start real-time dashboard\n");
    printf("  version                 Show version\n");
    printf("  help                    This help\n");
}

static void print_version(void) {
    printf("FlowEngine v%s\n", FLOWENGINE_VERSION);
    printf("  Build:  " __DATE__ " " __TIME__ "\n");
    printf("  Modules: bus, ipc, bag, clock, discovery, fusion, scheduler, serializer, statem, transport\n");
}

/* ── list tasks ──────────────────────────────────────────── */

static int cmd_list_tasks(void) {
    printf("Tasks: (connect to running FlowEngine for live data)\n");
    printf("  %-20s %-12s %-8s %s\n", "NAME", "STATE", "PRIO", "TOPICS");
    printf("  %-20s %-12s %-8s %s\n", "────", "─────", "────", "──────");

    /* Try to read from discovery JSON */
    FILE* f = fopen("/tmp/flow_topology.json", "r");
    if (!f) {
        printf("  (no running FlowEngine found — start flow_e2e first)\n");
        return 0;
    }

    /* Simple parse: extract node names */
    char buf[8192];
    size_t n = fread(buf, 1, sizeof(buf)-1, f);
    fclose(f);
    buf[n] = '\0';

    /* Extract "name" fields */
    const char* p = buf;
    int count = 0;
    while ((p = strstr(p, "\"name\":\"")) && count < 20) {
        p += 8;
        char name[64] = {0};
        int i = 0;
        while (*p && *p != '"' && i < 63) name[i++] = *p++;
        if (name[0]) {
            printf("  %-20s %-12s %-8s %s\n", name, "RUNNING", "—", "—");
            count++;
        }
    }
    printf("\n  Total: %d tasks\n", count);
    return 0;
}

/* ── list topics ─────────────────────────────────────────── */

static int cmd_list_topics(void) {
    printf("Topics:\n");
    printf("  %-30s %-8s %-8s %s\n", "TOPIC", "PUB", "SUB", "FREQ");
    printf("  %-30s %-8s %-8s %s\n", "─────", "───", "───", "────");

    FILE* f = fopen("/tmp/flow_topology.json", "r");
    if (!f) {
        printf("  (no data — start flow_e2e first)\n");
        return 0;
    }

    char buf[8192];
    size_t n = fread(buf, 1, sizeof(buf)-1, f);
    fclose(f);
    buf[n] = '\0';

    /* Parse topic entries from JSON */
    const char* p = buf;
    char topics[64][64] = {{0}};
    int freq[64] = {0};
    int tcount = 0;

    while ((p = strstr(p, "\"topic\":\"")) && tcount < 64) {
        p += 9;
        char tname[64] = {0};
        int i = 0;
        while (*p && *p != '"' && i < 63) tname[i++] = *p++;

        /* Check if already seen */
        int found = 0;
        for (int j = 0; j < tcount; j++) {
            if (strcmp(topics[j], tname) == 0) { found = 1; break; }
        }
        if (!found) {
            snprintf(topics[tcount], 64, "%s", tname);
            tcount++;
        }

        /* Look for freq */
        const char* fp2 = strstr(p, "\"freq\":");
        if (fp2 && fp2 < p + 100) {
            freq[tcount-1] = (int)atof(fp2 + 7);
        }
    }

    for (int i = 0; i < tcount; i++) {
        printf("  %-30s %-8s %-8s %s%d Hz\n",
               topics[i], "—", "—",
               freq[i] > 0 ? "" : "—", freq[i] > 0 ? freq[i] : 0);
    }
    printf("\n  Total: %d topics\n", tcount);
    return 0;
}

/* ── graph ───────────────────────────────────────────────── */

static int cmd_graph(void) {
    printf("Topology:\n\n");

    FILE* f = fopen("/tmp/flow_topology.json", "r");
    if (!f) {
        printf("  (no data)\n");
        return 0;
    }

    char buf[8192];
    size_t n = fread(buf, 1, sizeof(buf)-1, f);
    fclose(f);
    buf[n] = '\0';

    /* Parse and build simple graph */
    const char* p = buf;
    char nodes[32][64] = {{0}};
    int ncount = 0;

    while ((p = strstr(p, "\"name\":\"")) && ncount < 32) {
        p += 8;
        int i = 0;
        while (*p && *p != '"' && i < 63) nodes[ncount][i++] = *p++;
        if (nodes[ncount][0]) ncount++;
    }

    /* Simple ASCII rendering */
    for (int i = 0; i < ncount; i++) {
        printf("  [%s]", nodes[i]);
        if (i < ncount - 1) printf(" ──→ ");
    }
    printf("\n\n");

    /* Topic edges */
    p = buf;
    char prev_topic[64] = "";
    while ((p = strstr(p, "\"topic\":\""))) {
        p += 9;
        char tname[64] = {0};
        int i = 0;
        while (*p && *p != '"' && i < 63) tname[i++] = *p++;
        if (tname[0] && strcmp(tname, prev_topic) != 0) {
            printf("    ├─ %s\n", tname);
            snprintf(prev_topic, 64, "%s", tname);
        }
    }

    printf("\n  Run 'flowctl dashboard' for interactive topology viewer\n");
    return 0;
}

/* ── state ───────────────────────────────────────────────── */

static int cmd_state(const char* task_name) {
    if (!task_name) {
        fprintf(stderr, "Usage: flowctl state <task_name>\n");
        return 1;
    }

    /* Demo: show standard lifecycle */
    printf("State Machine: %s\n\n", task_name);
    printf("  transitions:\n");
    printf("    INITIALIZED + START   → RUNNING\n");
    printf("    RUNNING     + STOP    → STOPPING\n");
    printf("    STOPPING    + DONE    → STOPPED\n");
    printf("    RUNNING     + PAUSE   → PAUSED\n");
    printf("    PAUSED      + RESUME  → RUNNING\n");
    printf("    RUNNING     + ERROR   → ERROR\n");
    printf("    ERROR       + RESTART → INITIALIZED\n");
    printf("\n  current: RUNNING  allowed: [STOP, PAUSE, ERROR]\n");
    printf("\n  (connect to live FlowEngine for real-time state)\n");
    return 0;
}

/* ── topic stats ──────────────────────────────────────────── */

static int cmd_topic_stats(const char* topic) {
    if (!topic) {
        fprintf(stderr, "Usage: flowctl topic stats <topic_name>\n");
        return 1;
    }

    /* Create a local bus to demonstrate the API */
    MessageBus* bus = message_bus_create("flowctl_query");
    if (!bus) { fprintf(stderr, "Error: cannot create bus\n"); return 1; }

    /* Try to read stats from discovery JSON first */
    FILE* f = fopen("/tmp/flow_topology.json", "r");
    if (f) {
        char buf[8192];
        size_t n = fread(buf, 1, sizeof(buf)-1, f);
        fclose(f);
        buf[n] = '\0';

        /* Parse metrics */
        const char* mp = strstr(buf, "\"metrics\"");
        printf("Topic: %s\n\n", topic);
        if (mp) {
            const char* bp = strstr(mp, "\"published\"");
            const char* dp = strstr(mp, "\"delivered\"");
            const char* drp = strstr(mp, "\"dropped\"");
            const char* lp = strstr(mp, "\"avg_us\"");
            const char* p99 = strstr(mp, "\"p99_us\"");

            printf("  %-20s %s\n", "Published:", bp ? "see bus stats" : "—");
            printf("  %-20s %s\n", "Delivered:", dp ? "see bus stats" : "—");
            printf("  %-20s %s\n", "Dropped:",   drp ? "see bus stats" : "—");
            printf("  %-20s %s\n", "Avg latency:", lp ? "see latency stats" : "—");
            printf("  %-20s %s\n", "P99 latency:", p99 ? "see latency stats" : "—");
            printf("  %-20s %s\n", "Frequency:", "—");
        }
        printf("\n  (connect to live system for per-topic breakdown)\n");
        message_bus_destroy(bus);
        return 0;
    }

    message_bus_destroy(bus);
    printf("Topic: %s\n\n", topic);
    printf("  (no data source — start flow_e2e first)\n");
    return 0;
}

/* ── bag info ─────────────────────────────────────────────── */

static int cmd_bag_info(const char* path) {
    if (!path) {
        fprintf(stderr, "Usage: flowctl bag info <file.bag>\n");
        return 1;
    }

    BagReader* r = bag_reader_open(path);
    if (!r) {
        fprintf(stderr, "Error: cannot open '%s'\n", path);
        return 1;
    }

    uint64_t msg_count = 0, duration_us = 0;
    bag_reader_info(r, &msg_count, &duration_us);

    printf("Bag: %s\n\n", path);
    printf("  Messages:   %" PRIu64 "\n", msg_count);
    printf("  Duration:   %.1f seconds\n", (double)duration_us / 1000000.0);
    printf("  Avg rate:   %.1f Hz\n",
           duration_us > 0 ? (double)msg_count / ((double)duration_us / 1000000.0) : 0);

    /* Topic list */
    char topics[64][64];
    uint64_t counts[64];
    int n = bag_reader_get_topics(r, topics, 64, counts);
    printf("  Topics:     %d\n", n);
    for (int i = 0; i < n && i < 10; i++) {
        printf("    %-30s %" PRIu64 " msgs\n", topics[i], counts[i]);
    }

    /* Type info for first topic */
    if (n > 0) {
        uint32_t type_id;
        uint8_t schema_ver;
        if (bag_reader_get_type_info(r, topics[0], &type_id, &schema_ver) == 0) {
            printf("  Type:       %s (id=0x%08x, schema v%u)\n",
                   topics[0], type_id, schema_ver);
        }
    }

    bag_reader_close(r);
    return 0;
}

/* ── schema ───────────────────────────────────────────────── */

static int cmd_schema(const char* type_name) {
    if (!type_name) {
        fprintf(stderr, "Usage: flowctl schema <type_name>\n");
        return 1;
    }

    adas_msgs_register_all();

    const TypeRegistryEntry* e = serializer_lookup_by_name(type_name);
    if (!e) {
        printf("Type '%s' not found in registry.\n", type_name);
        printf("Registered types (%d):\n", serializer_type_count());
        /* List all */
        for (uint32_t tid = 0; tid < 0xFFFFFFFF; tid++) {
            const TypeRegistryEntry* te = serializer_lookup_type(tid);
            if (te) {
                printf("  %s (id=0x%08x, size=%zu, schema v%u)\n",
                       te->type_name, te->type_id,
                       te->struct_size, te->schema_version);
            }
            if (tid > 0xFFFF) break;
        }
        return 1;
    }

    printf("Type: %s\n\n", e->type_name);
    printf("  Type ID:        0x%08x\n", e->type_id);
    printf("  Schema Version: %u\n", e->schema_version);
    printf("  Struct Size:    %zu bytes\n", e->struct_size);
    printf("  Serializer:     %s\n", e->serialize ? "yes" : "no");
    printf("  Deserializer:   %s\n", e->deserialize ? "yes" : "no");
    printf("  Endian Swap:    %s\n", e->endian_swap ? "yes" : "no");
    return 0;
}

/* ── dashboard ────────────────────────────────────────────── */

static int cmd_dashboard(void) {
    printf("Starting FlowBoard dashboard...\n");
    printf("  Server: http://localhost:8800\n");
    printf("  API:    /api/topology  /api/stream\n\n");
    printf("Run in another terminal:\n");
    printf("  ./build/bin/flow_e2e 30 &\n");
    printf("  python3 tools/flowboard_server.py --json-file /tmp/flow_topology.json\n");
    printf("\nThen open http://localhost:8800 in browser.\n");
    return 0;
}

/* ══════════════════════════════════════════════════════════ */
/* Main                                                        */
/* ══════════════════════════════════════════════════════════ */

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage();
        return 0;
    }

    const char* cmd  = argv[1];
    const char* arg1 = argc > 2 ? argv[2] : NULL;
    const char* arg2 = argc > 3 ? argv[3] : NULL;

    /* ── list ── */
    if (strcmp(cmd, "list") == 0) {
        if (!arg1) { print_usage(); return 1; }
        if (strcmp(arg1, "tasks") == 0)   return cmd_list_tasks();
        if (strcmp(arg1, "topics") == 0)  return cmd_list_topics();
        if (strcmp(arg1, "plugins") == 0) {
            printf("Plugins: (connect to running launcher)\n");
            printf("  example_task, reactive_task, flowcoro_task,\n");
            printf("  fake_perception_task, fake_control_task,\n");
            printf("  network_service_task, data_processor_task\n");
            return 0;
        }
        print_usage(); return 1;
    }

    /* ── graph ── */
    if (strcmp(cmd, "graph") == 0) return cmd_graph();

    /* ── state ── */
    if (strcmp(cmd, "state") == 0) return cmd_state(arg1);

    /* ── topic ── */
    if (strcmp(cmd, "topic") == 0) {
        if (!arg1 || strcmp(arg1, "stats") != 0 || !arg2) {
            print_usage(); return 1;
        }
        return cmd_topic_stats(arg2);
    }

    /* ── bag ── */
    if (strcmp(cmd, "bag") == 0) {
        if (!arg1 || strcmp(arg1, "info") != 0 || !arg2) {
            print_usage(); return 1;
        }
        return cmd_bag_info(arg2);
    }

    /* ── schema ── */
    if (strcmp(cmd, "schema") == 0) return cmd_schema(arg1);

    /* ── dashboard ── */
    if (strcmp(cmd, "dashboard") == 0) return cmd_dashboard();

    /* ── version ── */
    if (strcmp(cmd, "version") == 0) { print_version(); return 0; }

    /* ── help ── */
    print_usage();
    return 0;
}
