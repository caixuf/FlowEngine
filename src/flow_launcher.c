/**
 * flow_launcher.c — FlowEngine 纯配置驱动启动器
 *
 * 设计意图: 这是整个项目的「生产入口」。
 *
 * 核心原则:
 *   - 启动器不含任何业务逻辑，只读配置文件
 *   - 节点是独立 .so 插件，通过 dlopen + NodePlugin 接口加载
 *   - pipeline 拓扑、启动顺序、参数全部在 config/pipeline.json 里声明
 *
 * 对比 CyberRT / ROS2:
 *   CyberRT: mainboard + dag 配置文件 → dlopen 加载 Component
 *   FlowEngine: flow_launcher + pipeline.json → dlopen 加载 NodePlugin
 *
 * 用法:
 *   ./build/bin/flow_launcher config/pipeline.json [--duration 30]
 *   ./build/bin/flow_launcher config/pipeline.json --multi [--duration 30]
 *
 * 目录结构:
 *   config/pipeline.json        — pipeline 声明
 *   modules/adas_nodes/*.so     — 节点插件（感知/融合/规划/控制/监控）
 *   include/node_plugin.h       — NodePlugin 接口定义
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <time.h>
#include <dlfcn.h>

#include "logger.h"
#include "node_plugin.h"
#include "message_bus.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include "flow_registry.h"
#include "adas_msgs_gen.h"

/* ── 简单 JSON 字段提取 ─────────────────────────────────────── */

static int json_get_str(const char* json, const char* key, char* out, int out_len) {
    char search[128];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '"') return 0;
    p++;
    int i = 0;
    while (*p && *p != '"' && i < out_len - 1) out[i++] = *p++;
    out[i] = '\0';
    return 1;
}

static int json_get_str_array(const char* json, const char* key,
                               char out[][64], int max_items) {
    char search[128];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '[') return 0;
    p++;
    int count = 0;
    while (*p && *p != ']' && count < max_items) {
        while (*p == ' ' || *p == ',') p++;
        if (*p == '"') {
            p++;
            int i = 0;
            while (*p && *p != '"' && i < 63) out[count][i++] = *p++;
            out[count][i] = '\0';
            if (i > 0) count++;
            if (*p == '"') p++;
        } else break;
    }
    return count;
}

static int json_get_int(const char* json, const char* key, int* out) {
    char search[128];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char* p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    while (*p == ' ') p++;
    if (*p != '-' && (*p < '0' || *p > '9')) return 0;
    *out = atoi(p);
    return 1;
}

/* ── 节点描述 ──────────────────────────────────────────────── */

#define MAX_NODES 16
#define MAX_TOPICS_PER_NODE 8
typedef struct {
    char name[64];
    char library[256];
    char params_json[1024];
    int  stagger_after_ms;
    /* 输入/输出 topics（从 pipeline.json 解析）*/
    char inputs[MAX_TOPICS_PER_NODE][64];
    char outputs[MAX_TOPICS_PER_NODE][64];
    int  input_count;
    int  output_count;
    /* dlopen 模式 */
    void*       lib_handle;
    NodePlugin* plugin;
    /* 多进程模式 */
    pid_t       pid;
} NodeDesc;

static volatile int g_running = 1;
static NodeDesc g_nodes[MAX_NODES];
static int      g_node_count = 0;

static void sig_handler(int sig) { (void)sig; g_running = 0; }

/* ── 解析 pipeline.json ────────────────────────────────────── */

static int parse_pipeline(const char* path, int* stagger_ms_out) {
    FILE* f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f); rewind(f);
    char* buf = (char*)malloc((size_t)fsize + 1);
    if (!buf) { fclose(f); return -1; }
    if (fread(buf, 1, (size_t)fsize, f) != (size_t)fsize && fsize > 0) {
        free(buf); fclose(f); return -1;
    }
    buf[fsize] = '\0';
    fclose(f);

    int stagger = 300;
    json_get_int(buf, "startup_stagger_ms", &stagger);
    *stagger_ms_out = stagger;

    const char* nodes_start = strstr(buf, "\"nodes\":");
    if (!nodes_start) { free(buf); return 0; }
    nodes_start = strchr(nodes_start, '[');
    if (!nodes_start) { free(buf); return 0; }
    nodes_start++;

    const char* p = nodes_start;
    g_node_count = 0;

    while (*p && g_node_count < MAX_NODES) {
        while (*p && *p != '{' && *p != ']') p++;
        if (!*p || *p == ']') break;
        const char* obj_start = p;
        int depth = 0;
        const char* q = p;
        while (*q) {
            if (*q == '{') depth++;
            else if (*q == '}') { depth--; if (depth == 0) { q++; break; } }
            q++;
        }
        size_t obj_len = (size_t)(q - obj_start);
        char* obj = (char*)malloc(obj_len + 1);
        if (!obj) break;
        memcpy(obj, obj_start, obj_len);
        obj[obj_len] = '\0';

        NodeDesc* nd = &g_nodes[g_node_count];
        memset(nd, 0, sizeof(*nd));
        nd->stagger_after_ms = stagger;
        nd->pid = -1;

        json_get_str(obj, "name",    nd->name,    sizeof(nd->name));
        json_get_str(obj, "library", nd->library, sizeof(nd->library));
        nd->input_count  = json_get_str_array(obj, "inputs",  nd->inputs,  MAX_TOPICS_PER_NODE);
        nd->output_count = json_get_str_array(obj, "outputs", nd->outputs, MAX_TOPICS_PER_NODE);

        const char* params_p = strstr(obj, "\"params\":");
        if (params_p) {
            params_p = strchr(params_p, '{');
            if (params_p) {
                int d2 = 0; const char* pq = params_p;
                while (*pq) {
                    if (*pq == '{') d2++;
                    else if (*pq == '}') { d2--; if (d2 == 0) { pq++; break; } }
                    pq++;
                }
                size_t plen = (size_t)(pq - params_p);
                if (plen < sizeof(nd->params_json) - 1) {
                    memcpy(nd->params_json, params_p, plen);
                    nd->params_json[plen] = '\0';
                }
            }
        }
        free(obj);
        if (nd->name[0]) g_node_count++;
        p = q;
    }
    free(buf);
    LOG_INFO("launcher", "pipeline parsed: %d nodes", g_node_count);
    return g_node_count;
}

/* ── dlopen 单进程模式 ─────────────────────────────────────── */

static int run_dlopen_mode(int duration, int stagger_ms,
                           MessageBus* bus, Transport* transport,
                           DiscoveryManager* discovery, Scheduler* scheduler) {

    LOG_INFO("launcher", "mode: dlopen (single-process)");

    /* 初始化所有节点插件 */
    for (int i = 0; i < g_node_count; i++) {
        NodeDesc* nd = &g_nodes[i];
        if (!nd->library[0]) {
            LOG_WARN("launcher", "node %s: no library path, skipping", nd->name);
            continue;
        }

        /* 尝试加载 .so — 支持绝对路径、相对路径和库名三种格式 */
        /* RTLD_GLOBAL: 把 .so 的符号暴露给后续加载的库 (依赖传递) */
        /* RTLD_LAZY:   推迟解析, 让 .so 能从主进程继承 net_transport* 等符号 */
        int dlflags = RTLD_LAZY | RTLD_GLOBAL;
        nd->lib_handle = dlopen(nd->library, dlflags);
        if (!nd->lib_handle) {
            /* 回退1: build/lib/<libname> */
            char alt[512];
            const char* basename = strrchr(nd->library, '/');
            basename = basename ? basename + 1 : nd->library;
            snprintf(alt, sizeof(alt), "build/lib/%s", basename);
            nd->lib_handle = dlopen(alt, dlflags);
        }
        if (!nd->lib_handle) {
            LOG_WARN("launcher", "dlopen %s failed: %s (skipping)", nd->library, dlerror());
            continue;
        }

        NodeGetPluginFn get_fn = (NodeGetPluginFn)dlsym(nd->lib_handle, NODE_PLUGIN_SYMBOL);
        if (!get_fn) {
            LOG_WARN("launcher", "%s: symbol '%s' not found", nd->library, NODE_PLUGIN_SYMBOL);
            dlclose(nd->lib_handle); nd->lib_handle = NULL;
            continue;
        }

        nd->plugin = get_fn();
        if (!nd->plugin) {
            LOG_WARN("launcher", "%s: node_get_plugin() returned NULL", nd->library);
            dlclose(nd->lib_handle); nd->lib_handle = NULL;
            continue;
        }

        if (nd->plugin->init(bus, transport, discovery, scheduler,
                             nd->params_json[0] ? nd->params_json : NULL) != 0) {
            LOG_WARN("launcher", "node %s init() failed", nd->name);
            nd->plugin = NULL;
            dlclose(nd->lib_handle); nd->lib_handle = NULL;
            continue;
        }

        LOG_INFO("launcher", "[%d/%d] loaded  %-16s  v%s — %s",
                 i + 1, g_node_count, nd->name,
                 nd->plugin->version, nd->plugin->description);
        usleep((unsigned)stagger_ms * 1000);
    }

    /* 启动所有已初始化的节点 */
    for (int i = 0; i < g_node_count; i++) {
        if (g_nodes[i].plugin) {
            g_nodes[i].plugin->start();
            LOG_INFO("launcher", "  started %s", g_nodes[i].name);
        }
    }

    LOG_INFO("launcher", "all nodes running (%ds) — dashboard: http://localhost:8800", duration);

    /* 等待运行时间或信号: duration ≤ 0 表示持续运行直到 Ctrl+C */
    if (duration > 0) {
        for (int t = 0; t < duration && g_running; t++) sleep(1);
    } else {
        LOG_INFO("launcher", "running indefinitely — press Ctrl+C to stop");
        while (g_running) sleep(1);
    }

    /* 优雅停止 */
    LOG_INFO("launcher", "stopping nodes...");
    for (int i = g_node_count - 1; i >= 0; i--) {
        if (g_nodes[i].plugin) { g_nodes[i].plugin->stop(); }
    }
    sleep(1);  /* 给线程时间退出 */
    for (int i = g_node_count - 1; i >= 0; i--) {
        if (g_nodes[i].plugin) {
            g_nodes[i].plugin->cleanup();
            LOG_INFO("launcher", "  stopped %s", g_nodes[i].name);
        }
        if (g_nodes[i].lib_handle) { dlclose(g_nodes[i].lib_handle); }
    }
    return 0;
}

/* ── fork+exec 多进程模式 ─────────────────────────────────── */

static pid_t launch_node_process(const NodeDesc* nd, const char* self_exe, int duration) {
    pid_t pid = fork();
    if (pid < 0) { LOG_WARN("launcher", "fork %s: %s", nd->name, strerror(errno)); return -1; }
    if (pid == 0) {
        char dur_str[16];
        snprintf(dur_str, sizeof(dur_str), "%d", duration);
        char e2e_path[512];
        snprintf(e2e_path, sizeof(e2e_path), "%s", self_exe);
        char* slash = strrchr(e2e_path, '/');
        if (slash) strcpy(slash + 1, "flow_e2e"); else strcpy(e2e_path, "./flow_e2e");
        char* args[] = { e2e_path, "--role", (char*)nd->name, dur_str, NULL };
        execv(e2e_path, args);
        perror("execv"); _exit(1);
    }
    return pid;
}

static int run_multi_process_mode(int duration, int stagger_ms, const char* self_exe) {
    LOG_INFO("launcher", "mode: multi-process (fork+exec)");
    for (int i = 0; i < g_node_count && g_running; i++) {
        NodeDesc* nd = &g_nodes[i];
        nd->pid = launch_node_process(nd, self_exe, duration);
        if (nd->pid > 0)
            LOG_INFO("launcher", "[%d/%d] started %-16s  pid=%d", i+1, g_node_count, nd->name, (int)nd->pid);
        usleep((unsigned)stagger_ms * 1000);
    }
    LOG_INFO("launcher", "all nodes started (%ds) — dashboard: http://localhost:8800", duration);
    while (g_running) {
        int status;
        pid_t done = waitpid(-1, &status, WNOHANG);
        if (done < 0 && errno == ECHILD) break;
        usleep(200000);
    }
    for (int i = 0; i < g_node_count; i++)
        if (g_nodes[i].pid > 0) kill(g_nodes[i].pid, SIGTERM);
    for (int i = 0; i < g_node_count; i++)
        if (g_nodes[i].pid > 0) { waitpid(g_nodes[i].pid, NULL, 0); g_nodes[i].pid = -1; }
    return 0;
}

/* ── main ────────────────────────────────────────────────── */

int main(int argc, char** argv) {
    const char* config_path = "config/pipeline.json";
    int duration = 30;
    int multi_mode = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) duration = atoi(argv[++i]);
        else if (strcmp(argv[i], "--multi") == 0) multi_mode = 1;
        else if (argv[i][0] != '-') config_path = argv[i];
    }

    log_init(LOG_INFO, NULL);

    LOG_INFO("launcher", "╔══════════════════════════════════════════╗");
    LOG_INFO("launcher", "║  FlowEngine Launcher v2                  ║");
    LOG_INFO("launcher", "║  config:   %-29s ║", config_path);
    LOG_INFO("launcher", "║  mode:     %-29s ║", multi_mode ? "multi-process (fork+exec)" : "single-process (dlopen)");
    LOG_INFO("launcher", "║  duration: %-29ds ║", duration);
    LOG_INFO("launcher", "╚══════════════════════════════════════════╝");

    int stagger_ms = 300;
    if (parse_pipeline(config_path, &stagger_ms) <= 0) {
        LOG_WARN("launcher", "no nodes found in %s", config_path);
        log_shutdown(); return 1;
    }

    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);

    if (multi_mode) {
        signal(SIGCHLD, SIG_DFL);
        run_multi_process_mode(duration, stagger_ms, argv[0]);
    } else {
        /* dlopen 模式: 需要初始化基础设施 */
        adas_msgs_register_all();

        /* 注册 pipeline 节点到 flow_registry */
        for (int i = 0; i < g_node_count; i++) {
            NodeDesc* nd = &g_nodes[i];
            /* 构建 NULL 结尾的 inputs/outputs 数组 */
            const char* inputs[MAX_TOPICS_PER_NODE + 1];
            const char* outputs[MAX_TOPICS_PER_NODE + 1];
            int ni = nd->input_count < MAX_TOPICS_PER_NODE ? nd->input_count : MAX_TOPICS_PER_NODE;
            int no = nd->output_count < MAX_TOPICS_PER_NODE ? nd->output_count : MAX_TOPICS_PER_NODE;
            for (int j = 0; j < ni; j++) inputs[j] = nd->inputs[j];
            inputs[ni] = NULL;
            for (int j = 0; j < no; j++) outputs[j] = nd->outputs[j];
            outputs[no] = NULL;
            int r = flow_registry_register_task(nd->name, nd->name, nd->library,
                                        ni > 0 ? inputs : NULL,
                                        no > 0 ? outputs : NULL,
                                        NULL);
            /* 注册 input/output topics */
            for (int j = 0; j < ni; j++) {
                flow_registry_register_topic(nd->inputs[j], 0, NULL);
            }
            for (int j = 0; j < no; j++) {
                flow_registry_register_topic(nd->outputs[j], 0, NULL);
            flow_registry_register_topic(nd->outputs[j], 0, NULL);
            }
            /* 注册 plugin: 关联此节点的 task name */
            const char* tasks[2] = { nd->name, NULL };
            flow_registry_register_plugin(nd->name, nd->library, tasks, NULL);
        }
        LOG_INFO("launcher", "registry: %d total entries", flow_registry_total_count());

        MessageBus*       bus       = message_bus_create("launcher_bus");
        DiscoveryManager* discovery = discovery_create("flow_launcher",
            CAP_PUBLISHER | CAP_SUBSCRIBER | CAP_FUSION);
        discovery_start(discovery);
        /* TRANSPORT_LOCAL: 单进程内节点共享本地总线，不需要 TCP/IPC */
        Transport*        transport = transport_create(bus, discovery, TRANSPORT_LOCAL);
        transport_start(transport);
        SchedulerConfig   scfg      = SCHEDULER_CONFIG_DEFAULT;
        scfg.mode                  = SCHEDULER_MODE_CHOREO;
        Scheduler*        scheduler = scheduler_create(&scfg);
        scheduler_set_choreo_bus(scheduler, bus);
        scheduler_start(scheduler);

        run_dlopen_mode(duration, stagger_ms, bus, transport, discovery, scheduler);

        scheduler_stop(scheduler);  scheduler_destroy(scheduler);
        transport_stop(transport);  transport_destroy(transport);
        discovery_stop(discovery);  discovery_destroy(discovery);
        message_bus_destroy(bus);
    }

    log_shutdown();
    return 0;
}


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
