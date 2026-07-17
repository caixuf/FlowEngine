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
#include "flow_registry.h"
#include "param_registry.h"
#include "config_manager.h"
#include "error_codes.h"
#include "adas_msgs_gen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#define FLOWENGINE_VERSION "1.0.0"
#define FLOWCTL_MAX_TOPICS 64

/* ── 帮助 ────────────────────────────────────────────────── */

static void print_usage(void) {
    printf("FlowEngine CLI v%s — Interactive command-line tool\n\n", FLOWENGINE_VERSION);
    printf("Usage: flowctl <command> [args]\n\n");
    printf("Commands:\n");
    printf("  list tasks              List registered tasks\n");
    printf("  list topics             List all topics with stats\n");
    printf("  list types              List registered message types\n");
    printf("  list plugins            List loaded plugins\n");
    printf("  list params             List registered parameters\n");
    printf("  graph                   Print topology graph\n");
    printf("  state <task>            Show task state machine\n");
    printf("  topic stats <topic>     Per-topic statistics\n");
    printf("  bag info <file>         Bag file metadata\n");
    printf("  schema <type>           Type information\n");
    printf("  launch <config>         Validate and show launch config\n");
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
    printf("Tasks:\n");
    printf("  %-20s %-30s %s\n", "NAME", "DESCRIPTION", "I/O");
    printf("  %-20s %-30s %s\n", "────", "───────────", "───");

    /* Try JSON file first */
    FILE* f = fopen(flowengine_state_file(), "r");
    if (f) {
        char buf[32768];
        size_t n = fread(buf, 1, sizeof(buf)-1, f);
        fclose(f);
        buf[n] = '\0';
        /* Parse registry tasks */
        const char* r = strstr(buf, "\"registry\":");
        if (r) {
            const char* tp = strstr(r, "\"tasks\":[");
            if (tp) {
                const char* p = tp + 9;
                int count = 0;
                while (*p && *p != ']' && count < 20) {
                    const char* nm = strstr(p, "\"name\":\"");
                    const char* end = strstr(p, "}]");
                    if (!nm) break;
                    if (end && nm > end) break;
                    nm += 8;
                    char name[64] = {0}, desc[256] = {0};
                    int i = 0; while (*nm && *nm != '"' && i < 63) name[i++] = *nm++;
                    const char* ds = strstr(nm, "\"desc\":\"");
                    if (ds) { ds += 8; i = 0; while (*ds && *ds != '"' && i < 255) desc[i++] = *ds++; }
                    printf("  %-20s %-30s %s\n", name, desc, "—");
                    p = strstr(nm, "},{");
                    if (!p) break;
                    p += 3;
                    count++;
                }
                printf("\n  Total: %d tasks (from registry)\n", count);
                return 0;
            }
        }
    }

    /* Fallback: local registry */
    TaskMeta tasks[64];
    int n = flow_registry_list_tasks(tasks, 64);
    for (int i = 0; i < n; i++)
        printf("  %-20s %-30s in:%d out:%d\n", tasks[i].name, tasks[i].description, tasks[i].input_count, tasks[i].output_count);
    printf("\n  Total: %d tasks\n", n);
    return 0;
}

/* ── list topics ─────────────────────────────────────────── */

static int cmd_list_topics(void) {
    printf("Topics:\n");
    printf("  %-30s %-8s %-10s %s\n", "TOPIC", "TYPE_ID", "QOS", "FREQ");
    printf("  %-30s %-8s %-10s %s\n", "─────", "───────", "───", "────");

    /* Primary: query FlowRegistry */
    TopicMeta topics[FLOWCTL_MAX_TOPICS];
    int n = flow_registry_list_topics(topics, FLOWCTL_MAX_TOPICS);
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            printf("  %-30s 0x%08x  depth=%-3d %s%.0f Hz\n",
                   topics[i].name, topics[i].type_id,
                   topics[i].qos.depth,
                   topics[i].qos.reliability == 1 ? "reliable " : "",
                   0.0);  /* freq from live stats, not registry */
        }
        printf("\n  Total: %d topics (from registry)\n", n);
        return 0;
    }

    /* Fallback: parse state file for live stats */
    FILE* f = fopen(flowengine_state_file(), "r");
    if (!f) {
        printf("  (no data — start flow_launcher first)\n");
        return 0;
    }
    char buf[8192];
    size_t sz = fread(buf, 1, sizeof(buf)-1, f);
    fclose(f);
    buf[sz] = '\0';

    const char* p = buf;
    int tcount = 0;
    char seen[FLOWCTL_MAX_TOPICS][64];
    while ((p = strstr(p, "\"topic\":\"")) && tcount < FLOWCTL_MAX_TOPICS) {
        p += 9; char tname[64] = {0}; int i = 0;
        while (*p && *p != '"' && i < 63) tname[i++] = *p++;
        /* Deduplicate */
        int dup = 0;
        for (int j = 0; j < tcount; j++) {
            if (strcmp(seen[j], tname) == 0) { dup = 1; break; }
        }
        if (!dup) {
            snprintf(seen[tcount], 64, "%s", tname);
            const char* fp2 = strstr(p, "\"freq\":");
            double freq = (fp2 && fp2 < p + 100) ? atof(fp2 + 7) : 0.0;
            printf("  %-30s %-8s %-10s %s%.1f Hz\n",
                   tname, "—", "—",
                   freq > 0 ? "" : "—", freq);
            tcount++;
        }
    }
    printf("\n  Total: %d topics (from state file)\n", tcount);
    return 0;
}

/* ── list types ──────────────────────────────────────────── */

static int cmd_list_types(void) {
    /* Populate registry via serializer notifications (same as cmd_schema) */
    adas_msgs_register_all();

    printf("Types:\n");
    printf("  %-30s %-12s %s\n", "NAME", "TYPE_ID", "SIZE");
    printf("  %-30s %-12s %s\n", "────", "───────", "────");
    FlowTypeMeta types[64];
    int n = flow_registry_list_types(types, 64);
    if (n == 0) {
        printf("  (none registered — start flow_launcher first)\n");
        return 0;
    }
    for (int i = 0; i < n; i++)
        printf("  %-30s 0x%08x   %zu bytes\n",
               types[i].name, types[i].type_id, types[i].struct_size);
    printf("\n  Total: %d types (from registry)\n", n);
    return 0;
}

/* ── list params ─────────────────────────────────────────── */

static int cmd_list_params(void) {
    printf("Params:\n");
    printf("  %-28s %-8s %-12s %s\n", "NAME", "TYPE", "VALUE", "DESC");
    printf("  %-28s %-8s %-12s %s\n", "────", "────", "─────", "────");
    FlowParamMeta params[64];
    int n = flow_registry_list_params(params, 64);
    for (int i = 0; i < n; i++) {
        const char* type_str = params[i].type == (int)PARAM_INT   ? "int" :
                               params[i].type == (int)PARAM_FLOAT ? "float" :
                               params[i].type == (int)PARAM_BOOL  ? "bool" : "str";
        printf("  %-28s %-8s %-12s %s%s\n", params[i].name, type_str,
               params[i].value_str, params[i].description,
               params[i].hot_reload ? " 🔥" : "");
    }
    printf("\n  Total: %d params (from registry)\n", n);
    return 0;
}

/* ── graph ───────────────────────────────────────────────── */

static int cmd_graph(void) {
    printf("Topology:\n\n");

    FILE* f = fopen(flowengine_state_file(), "r");
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

    /* Try registry first */
    const TaskMeta* tm = flow_registry_get_task(task_name);
    if (tm) {
        printf("Task: %s\n", tm->name);
        printf("  Description: %s\n", tm->description);
        printf("  Plugin:      %s\n", tm->plugin[0] ? tm->plugin : "(none)");
        printf("  Inputs:      %d topic(s)\n", tm->input_count);
        for (int j = 0; j < tm->input_count; j++)
            printf("    ← %s\n", tm->inputs[j]);
        printf("  Outputs:     %d topic(s)\n", tm->output_count);
        for (int j = 0; j < tm->output_count; j++)
            printf("    → %s\n", tm->outputs[j]);
        return 0;
    }

    /* Fallback: show standard lifecycle diagram */
    printf("State Machine: %s\n\n", task_name);
    printf("  transitions:\n");
    printf("    INITIALIZED + START   → RUNNING\n");
    printf("    RUNNING     + STOP    → STOPPING\n");
    printf("    STOPPING    + DONE    → STOPPED\n");
    printf("    RUNNING     + PAUSE   → PAUSED\n");
    printf("    PAUSED      + RESUME  → RUNNING\n");
    printf("    RUNNING     + ERROR   → ERROR\n");
    printf("    ERROR       + RESTART → INITIALIZED\n");
    printf("\n  (start flow_launcher for live task state)\n");
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
    FILE* f = fopen(flowengine_state_file(), "r");
    if (f) {
        char buf[8192];
        size_t n = fread(buf, 1, sizeof(buf)-1, f);
        fclose(f);
        buf[n] = '\0';

        printf("Topic: %s\n\n", topic);

        /* Search per-topic stats in "topics":[...] array */
        const char* tp = strstr(buf, "\"topics\":[");
        if (tp) {
            const char* p = tp + 10;  /* skip past "topics":[ */
            while ((p = strstr(p, "\"topic\":\""))) {
                p += 9;
                char tname[64] = {0};
                int i = 0;
                while (*p && *p != '"' && i < 63) tname[i++] = *p++;

                if (strcmp(tname, topic) == 0) {
                    /* Found matching entry — extract fields */
                    unsigned long pub = 0, del = 0, drop = 0;
                    unsigned long lat_avg = 0, p50 = 0, p99 = 0;
                    double freq = 0.0;
                    unsigned int subs = 0;

                    const char* kp;
                    if ((kp = strstr(p, "\"pub\":")))      pub      = strtoul(kp + 6,  NULL, 10);
                    if ((kp = strstr(p, "\"del\":")))      del      = strtoul(kp + 6,  NULL, 10);
                    if ((kp = strstr(p, "\"drop\":")))     drop     = strtoul(kp + 7,  NULL, 10);
                    if ((kp = strstr(p, "\"lat_us\":")))   lat_avg  = strtoul(kp + 9,  NULL, 10);
                    if ((kp = strstr(p, "\"p50_us\":")))   p50      = strtoul(kp + 9,  NULL, 10);
                    if ((kp = strstr(p, "\"p99_us\":")))   p99      = strtoul(kp + 9,  NULL, 10);
                    if ((kp = strstr(p, "\"freq\":")))     freq     = atof(kp + 7);
                    if ((kp = strstr(p, "\"subs\":")))     subs     = (unsigned int)strtoul(kp + 7, NULL, 10);

                    /* Limit field parsing to current object (stop at next '}') */
                    printf("  %-22s %lu\n",      "Published:",    pub);
                    printf("  %-22s %lu\n",      "Delivered:",    del);
                    printf("  %-22s %lu\n",      "Dropped:",      drop);
                    printf("  %-22s %u\n",       "Subscribers:",  subs);
                    printf("  %-22s %.1f Hz\n",  "Frequency:",    freq);
                    printf("  %-22s %lu µs\n",   "Avg latency:",  lat_avg);
                    printf("  %-22s %lu µs\n",   "P50 latency:",  p50);
                    printf("  %-22s %lu µs\n",   "P99 latency:",  p99);
                    message_bus_destroy(bus);
                    return 0;
                }
            }
        }
        printf("  (topic not found in state file)\n");
        message_bus_destroy(bus);
        return 0;
    }

    message_bus_destroy(bus);
    printf("Topic: %s\n\n", topic);
    printf("  (no data source — start flow_launcher first)\n");
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
    double duration_sec = (double)duration_us / 1000000.0;
    printf("  Topics:     %d\n", n);
    for (int i = 0; i < n && i < 10; i++) {
        double freq = (duration_sec > 0.0) ? counts[i] / duration_sec : 0.0;
        printf("    %-30s %6" PRIu64 " msgs  %6.1f Hz\n",
               topics[i], counts[i], freq);
    }

    /* Type info per topic (v2 format) */
    printf("\n  Type info:\n");
    for (int i = 0; i < n && i < 10; i++) {
        uint32_t type_id;
        uint8_t schema_ver;
        if (bag_reader_get_type_info(r, topics[i], &type_id, &schema_ver) == 0) {
            printf("    %-30s id=0x%08x  schema v%u\n",
                   topics[i], type_id, schema_ver);
        } else {
            printf("    %-30s (no type info)\n", topics[i]);
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
    printf("  Schema Hash:    0x%08x\n", e->schema_hash);
    printf("  Struct Size:    %zu bytes\n", e->struct_size);
    printf("  Serializer:     %s\n", e->serialize ? "yes" : "no");
    printf("  Deserializer:   %s\n", e->deserialize ? "yes" : "no");
    printf("  Endian Swap:    %s\n", e->endian_swap ? "yes" : "no");
    if (e->fields && e->field_count > 0) {
        printf("\n  Fields (%u):\n", e->field_count);
        printf("    %-20s %-8s %-8s %-8s %s\n",
               "name", "kind", "offset", "elem_sz", "array");
        for (uint16_t i = 0; i < e->field_count; i++) {
            const FieldDesc* f = &e->fields[i];
            printf("    %-20s %-8s %-8u %-8u %u\n",
                   f->name, field_kind_str(f->kind),
                   f->offset, f->elem_size, f->array_len);
        }
    }
    return 0;
}

/* ── dashboard ────────────────────────────────────────────── */

static int cmd_dashboard(void) {
    printf("Starting FlowBoard dashboard...\n");
    printf("  Server: http://localhost:8800\n");
    printf("  API:    /api/topology  /api/stream\n\n");
    printf("Run in another terminal:\n");
    printf("  ./build/bin/flow_launcher config/pipeline.json --duration 30 &\n");
    printf("  python3 tools/flowboard_server.py --json-file $FLOWENGINE_STATE_FILE\n");
    printf("  (default: %s)\n", FLOWENGINE_DEFAULT_STATE_FILE);
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

    /* Init demo params for standalone usage */
    param_register_int("control.max_speed", 120, 0, 200, "Max speed km/h");
    param_register_float("fusion.max_delta_ms", 50.0, 10.0, 500.0, "Alignment window ms");
    param_register_bool("control.emergency_brake", true, "Enable AEB");
    param_register_int("perception.lidar_rate_hz", 10, 1, 100, "LiDAR scan rate");

    /* ── list ── */
    if (strcmp(cmd, "list") == 0) {
        if (!arg1) { print_usage(); return 1; }
        if (strcmp(arg1, "tasks") == 0)   return cmd_list_tasks();
        if (strcmp(arg1, "topics") == 0)  return cmd_list_topics();
        if (strcmp(arg1, "types") == 0)   return cmd_list_types();
        if (strcmp(arg1, "params") == 0)  return cmd_list_params();
        if (strcmp(arg1, "plugins") == 0) {
            PluginMeta plugins[32];
            int n = flow_registry_list_plugins(plugins, 32);
            if (n == 0) {
                printf("Plugins: (none registered — start flow_launcher first)\n");
            } else {
                printf("%-20s %-40s %s\n", "NAME", "PATH", "TASKS");
                for (int i = 0; i < n; i++)
                    printf("  %-18s %-40s %d tasks\n", plugins[i].name, plugins[i].path, plugins[i].task_count);
                printf("  Total: %d plugins (from registry)\n", n);
            }
            return 0;
        }
        print_usage(); return 1;
    }

    /* ── graph ── */
    if (strcmp(cmd, "graph") == 0) return cmd_graph();

    /* ── topology ── */
    if (strcmp(cmd, "topology") == 0) {
        if (!arg1) { fprintf(stderr, "Usage: flowctl topology dot\n"); return 1; }
        if (strcmp(arg1, "dot") == 0) {
            /* Generate Graphviz DOT format from FlowRegistry */
            char* json = flow_registry_export_json();
            if (!json) { fprintf(stderr, "Error: registry export failed\n"); return 1; }
            printf("// Graphviz DOT — pipe to:  dot -Tsvg -o topology.svg\n");
            printf("digraph FlowEngine {\n");
            printf("  rankdir=LR;\n");
            printf("  node [shape=box, style=filled, fillcolor=\"#1a1a2e\","
                   " fontcolor=\"#58a6ff\", fontname=\"monospace\", fontsize=10];\n");
            printf("  edge [color=\"#30363d\", fontcolor=\"#8b949e\", fontsize=9];\n\n");
            /* Parse registry JSON for tasks and topics */
            const char* p = json;
            /* Extract tasks */
            const char* tp = strstr(p, "\"tasks\":[");
            if (tp) {
                tp = strchr(tp, '[') + 1;
                while (*tp && *tp != ']') {
                    const char* nm = strstr(tp, "\"name\":\"");
                    const char* end = strstr(tp, "}]");
                    if (!nm) break;
                    if (end && nm > end) break;
                    nm += 8; char name[64] = {0}; int i = 0;
                    while (*nm && *nm != '"' && i < 63) name[i++] = *nm++;
                    printf("  \"%s\" [fillcolor=\"#0d1117\", fontcolor=\"#c9d1d9\"];\n", name);
                    tp = strstr(nm, "},{");
                    if (!tp) break;
                    tp += 3;
                }
            }
            /* Extract topics as edges */
            p = json;
            while ((p = strstr(p, "\"topic\":\""))) {
                p += 9; char tname[64] = {0}; int i = 0;
                while (*p && *p != '"' && i < 63) tname[i++] = *p++;
                const char* nm = strstr(p, "\"name\":\"");
                if (nm && nm < p + 200) {
                    nm += 8; char nname[64] = {0}; i = 0;
                    while (*nm && *nm != '"' && i < 63) nname[i++] = *nm++;
                    printf("  \"%s\" -> \"%s\" [label=\"%s\"];\n", nname, tname, tname);
                }
            }
            printf("}\n");
            free(json);
            return 0;
        }
        fprintf(stderr, "Usage: flowctl topology dot\n");
        return 1;
    }

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
        if (!arg1) { print_usage(); return 1; }
        if (strcmp(arg1, "info") == 0 && arg2) return cmd_bag_info(arg2);
        if (strcmp(arg1, "check") == 0 && arg2) {
            BagReader* r = bag_reader_open(arg2);
            if (!r) { printf("✗ Cannot open '%s'\n", arg2); return 1; }
            uint64_t count, dur;
            bag_reader_info(r, &count, &dur);
            printf("✓ Bag file valid: %" PRIu64 " msgs, %.1fs\n", count, (double)dur/1000000.0);
            char topics[64][64];
            int n = bag_reader_get_topics(r, topics, 64, NULL);
            printf("  Topics: %d\n", n);
            bag_reader_close(r);
            return 0;
        }
        print_usage(); return 1;
    }

    /* ── schema ── */
    if (strcmp(cmd, "schema") == 0) return cmd_schema(arg1);

    /* ── param ── */
    if (strcmp(cmd, "param") == 0) {
        if (!arg1) { fprintf(stderr, "Usage: flowctl param list|get|set [name] [value]\n"); return 1; }
        if (strcmp(arg1, "list") == 0) {
            ParamEntry params[64];
            int n = param_list_all(params, 64);
            printf("%-30s %-8s %-12s %s\n", "NAME", "TYPE", "VALUE", "DESC");
            for (int i = 0; i < n; i++) {
                char val[64];
                switch (params[i].type) {
                    case PARAM_INT: snprintf(val, 64, "%ld", (long)params[i].current_value.int_val); break;
                    case PARAM_FLOAT: snprintf(val, 64, "%.1f", params[i].current_value.float_val); break;
                    case PARAM_BOOL: snprintf(val, 64, "%s", params[i].current_value.bool_val ? "true" : "false"); break;
                    case PARAM_STRING: snprintf(val, 64, "%s", params[i].current_value.str_val); break;
                    default: snprintf(val, 64, "?"); break;
                }
                printf("  %-28s %-8s %-12s %s%s\n", params[i].name,
                    params[i].type == PARAM_INT ? "int" : params[i].type == PARAM_FLOAT ? "float" : params[i].type == PARAM_BOOL ? "bool" : "str",
                    val, params[i].description, params[i].hot_reload ? " 🔥" : "");
            }
            printf("  Total: %d params\n", n);
            return 0;
        }
        if (strcmp(arg1, "get") == 0) {
            if (!arg2) { fprintf(stderr, "Usage: flowctl param get <name>\n"); return 1; }
            const ParamEntry* e = param_get_entry(arg2);
            if (!e) { printf("Param '%s' not found\n", arg2); return 1; }
            printf("%s = ", e->name);
            switch (e->type) {
                case PARAM_INT: printf("%ld\n", (long)e->current_value.int_val); break;
                case PARAM_FLOAT: printf("%.1f\n", e->current_value.float_val); break;
                case PARAM_BOOL: printf("%s\n", e->current_value.bool_val ? "true" : "false"); break;
                case PARAM_STRING: printf("%s\n", e->current_value.str_val); break;
                default: printf("?\n"); break;
            }
            printf("  type: %s  range: [", e->type == PARAM_INT ? "int" : e->type == PARAM_FLOAT ? "float" : e->type == PARAM_BOOL ? "bool" : "str");
            if (e->type == PARAM_INT) printf("%ld,%ld", (long)e->min_value.int_val, (long)e->max_value.int_val);
            else if (e->type == PARAM_FLOAT) printf("%.1f,%.1f", e->min_value.float_val, e->max_value.float_val);
            printf("]  hot_reload: %s\n  %s\n", e->hot_reload ? "yes" : "no", e->description);
            return 0;
        }
        if (strcmp(arg1, "set") == 0) {
            if (!arg2 || !argv[4]) { fprintf(stderr, "Usage: flowctl param set <name> <value>\n"); return 1; }
            const ParamEntry* e = param_get_entry(arg2);
            if (!e) { printf("Param '%s' not found\n", arg2); return 1; }
            int ret;
            switch (e->type) {
                case PARAM_INT: ret = param_set_int(arg2, atol(argv[4])); break;
                case PARAM_FLOAT: ret = param_set_float(arg2, atof(argv[4])); break;
                case PARAM_BOOL: ret = param_set_bool(arg2, strcmp(argv[4],"true")==0||strcmp(argv[4],"1")==0); break;
                case PARAM_STRING: ret = param_set_string(arg2, argv[4]); break;
                default: ret = -1; break;
            }
            if (ret == 0) printf("✓ %s updated\n", arg2);
            else printf("✗ failed (%s)\n", err_str(ret));
            return ret;
        }
        fprintf(stderr, "Usage: flowctl param list|get|set [name] [value]\n");
        return 1;
    }

    /* ── registry ── */
    if (strcmp(cmd, "registry") == 0) {
        /* Try JSON file first (live data from e2e) */
        FILE* f = fopen(flowengine_state_file(), "r");
        if (f) {
            char buf[32768];
            size_t n = fread(buf, 1, sizeof(buf)-1, f);
            fclose(f);
            buf[n] = '\0';
            const char* r = strstr(buf, "\"registry\":");
            if (r) {
                /* extract just the registry JSON object */
                const char* start = strchr(r, '{');
                if (start) {
                    int depth = 1;
                    const char* end = start + 1;
                    while (*end && depth > 0) {
                        if (*end == '{') depth++;
                        else if (*end == '}') depth--;
                        end++;
                    }
                    printf("%.*s\n", (int)(end - start), start);
                    return 0;
                }
            }
        }
        /* Fallback: local registry */
        char* json = flow_registry_export_json();
        if (json) { printf("%s\n", json); free(json); }
        return 0;
    }

    /* ── launch ── */
    if (strcmp(cmd, "launch") == 0) {
        if (!arg1) { fprintf(stderr, "Usage: flowctl launch <config.json>\n"); return 1; }
        LauncherConfig* cfg = config_load(arg1);
        if (!cfg) { fprintf(stderr, "Error: cannot load config '%s'\n", arg1); return 1; }
        printf("Configuration loaded: %d processes\n\n", cfg->process_count);
        for (int i = 0; i < cfg->process_count; i++) {
            ProcessConfig* pc = &cfg->processes[i];
            printf("  %-16s  lib=%-40s  pub=%d  sub=%d  deps=%d\n",
                   pc->name, pc->library_path,
                   pc->publish_count, pc->subscribe_count, pc->depends_count);
        }
        printf("\nScheduler: %s  workers=%d  tick=%dus\n",
               cfg->scheduler.mode == 1 ? "choreo" : "classic",
               cfg->scheduler.worker_threads, cfg->scheduler.tick_us);
        printf("\nRun with: ./build/bin/flow_launcher %s\n", arg1);
        config_free(cfg);
        return 0;
    }

    /* ── dashboard ── */
    if (strcmp(cmd, "dashboard") == 0) return cmd_dashboard();

    /* ── version ── */
    if (strcmp(cmd, "version") == 0) { print_version(); return 0; }

    /* ── help ── */
    print_usage();
    return 0;
}
