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
#include "config_manager.h"
#include "bag.h"
#include "adas_msgs_gen.h"

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

/* ── 加载配置 (统一使用 config_manager) ──────────────────────── */

static int parse_pipeline(const char* path, int* stagger_ms_out) {
    LauncherConfig* cfg = config_load(path);
    if (!cfg) {
        LOG_WARN("launcher", "failed to load config: %s", path);
        return -1;
    }

    *stagger_ms_out = 300;  /* default stagger */

    g_node_count = 0;
    for (int i = 0; i < cfg->process_count && g_node_count < MAX_NODES; i++) {
        ProcessConfig* pc = &cfg->processes[i];
        NodeDesc* nd = &g_nodes[g_node_count];
        memset(nd, 0, sizeof(*nd));
        nd->stagger_after_ms = *stagger_ms_out;
        nd->pid = -1;

        snprintf(nd->name, sizeof(nd->name), "%s", pc->name);
        snprintf(nd->library, sizeof(nd->library), "%s", pc->library_path);

        /* Map publish → outputs, subscribe → inputs (from node's perspective) */
        for (int k = 0; k < pc->subscribe_count && nd->input_count < MAX_TOPICS_PER_NODE; k++)
            snprintf(nd->inputs[nd->input_count++], 64, "%s", pc->subscribe[k].topic);
        for (int k = 0; k < pc->publish_count && nd->output_count < MAX_TOPICS_PER_NODE; k++)
            snprintf(nd->outputs[nd->output_count++], 64, "%s", pc->publish[k].topic);

        /* Copy params */
        if (pc->params[0])
            snprintf(nd->params_json, sizeof(nd->params_json), "%s", pc->params);

        if (nd->name[0]) g_node_count++;
    }

    LOG_INFO("launcher", "config loaded: %d nodes (from %d processes)",
             g_node_count, cfg->process_count);
    config_free(cfg);
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

    /* 启动所有已初始化的节点（留微小间隔避免多节点广播同一 topic 时
     * 在微秒级内连续 publish 导致频率估算出现尖峰，如 node_info 的 52kHz） */
    for (int i = 0; i < g_node_count; i++) {
        if (g_nodes[i].plugin) {
            g_nodes[i].plugin->start();
            LOG_INFO("launcher", "  started %s", g_nodes[i].name);
            usleep(5000);  /* 5ms stagger — 总计 <50ms，用户无感 */
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
    const char* bag_path    = NULL;
    int duration = 0;  /* 0 = 持续运行直到 Ctrl+C */
    int multi_mode = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) duration = atoi(argv[++i]);
        else if (strcmp(argv[i], "--bag") == 0 && i + 1 < argc) bag_path = argv[++i];
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

        MessageBus* bus       = message_bus_create("launcher_bus");

        /* Bag recording (optional: --bag /path/to/output.bag) */
        BagWriter* bag_writer = NULL;
        if (bag_path) {
            bag_writer = bag_writer_open(bag_path);
            if (bag_writer) {
                bag_writer_attach(bag_writer, bus);
                LOG_INFO("launcher", "bag recording: %s", bag_path);
            }
        }

        /* 从配置加载 QoS 并应用到 message_bus */
        LauncherConfig* qos_cfg = config_load(config_path);
        if (qos_cfg) {
            int qos_count = 0;
            for (int i = 0; i < qos_cfg->process_count; i++) {
                ProcessConfig* pc = &qos_cfg->processes[i];
                for (int k = 0; k < pc->publish_count; k++) {
                    TopicDecl* td = &pc->publish[k];
                    if (td->qos_depth > 0 || td->qos_policy[0]) {
                        TopicQos tq = {0};
                        tq.depth       = (uint32_t)(td->qos_depth > 0 ? td->qos_depth : 8);
                        tq.policy      = strcmp(td->qos_policy, "block") == 0 ? QOS_BLOCK :
                                         strcmp(td->qos_policy, "drop_latest") == 0 ? QOS_DROP_LATEST :
                                         QOS_DROP_OLDEST;
                        tq.reliability = strcmp(td->qos_reliability, "reliable") == 0 ? QOS_RELIABLE : QOS_BEST_EFFORT;
                        tq.deadline_ms = (uint32_t)(td->qos_deadline_ms > 0 ? td->qos_deadline_ms : 0);
                        tq.lifespan_ms = (uint32_t)(td->qos_lifespan_ms > 0 ? td->qos_lifespan_ms : 0);
                        message_bus_set_topic_qos(bus, td->topic, &tq);
                        qos_count++;
                    }
                }
            }
            if (qos_count > 0)
                LOG_INFO("launcher", "QoS applied to %d topics", qos_count);
            config_free(qos_cfg);
        }

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

        if (bag_writer) {
            bag_writer_close(bag_writer);
            LOG_INFO("launcher", "bag saved: %s", bag_path);
        }

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
