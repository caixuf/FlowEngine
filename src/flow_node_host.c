/**
 * flow_node_host.c — 单节点插件宿主进程
 *
 * 设计意图: 让「多进程模式」与「单进程 dlopen 模式」复用同一份 NodePlugin .so。
 *
 * 背景:
 *   flow_launcher 有两种启动模式:
 *     - 单进程 (dlopen): 所有 .so 加载进同一进程, 共享 MessageBus, 进程内零拷贝
 *     - 多进程 (fork+exec): 每个节点一个独立进程, 通过 discovery + IPC 桥接通信
 *
 *   历史上多进程模式 exec 的是已废弃的 `flow_e2e --role <name>` 单体 demo,
 *   导致节点逻辑在 e2e_demo.c 和 modules/adas_nodes/*.c 里各存一份 (copy-paste 债)。
 *
 *   flow_node_host 是「进程模式」的正确载体: 它只做一件事 —— 在自己的进程里
 *   dlopen 一个 NodePlugin .so, 自建 IPC 桥接基础设施, 跑该节点的生命周期。
 *   这样同一份 .so 既能被 launcher 拉进共享进程当线程, 也能被 host 起成独立进程。
 *
 * 用法:
 *   flow_node_host <config_path> <node_name> [duration_sec]
 *
 *   config_path  — pipeline.json (与 flow_launcher 相同)
 *   node_name    — 要在本进程中启动的节点名 (config 里的 processes[].name)
 *   duration_sec — 运行秒数 (<=0 表示持续运行直到 SIGINT/SIGTERM)
 *
 * 与 flow_launcher run_dlopen_mode 的一致性:
 *   本宿主对 params 来源、QoS 应用逻辑与单进程模式保持完全一致,
 *   区别仅在于 transport 使用 TRANSPORT_IPC (跨进程) 而非 TRANSPORT_LOCAL。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>

#include "logger.h"
#include "node_plugin.h"
#include "message_bus.h"
#include "transport.h"
#include "discovery.h"
#include "scheduler.h"
#include "config_manager.h"
#include "adas_msgs_gen.h"

static volatile int g_running = 1;
static void sig_handler(int sig) { (void)sig; g_running = 0; }

/* 在 config 中按名字查找节点。返回索引, 未找到返回 -1。 */
static int find_node(const LauncherConfig* cfg, const char* name) {
    for (int i = 0; i < cfg->process_count; i++) {
        if (strcmp(cfg->processes[i].name, name) == 0) return i;
    }
    return -1;
}

/* 为该节点的所有 publish topic 应用 QoS (逻辑与 flow_launcher 一致)。 */
static void apply_qos(MessageBus* bus, const ProcessConfig* pc) {
    for (int k = 0; k < pc->publish_count; k++) {
        const TopicDecl* td = &pc->publish[k];
        if (td->qos_depth > 0 || td->qos_policy[0]) {
            TopicQos tq = {0};
            tq.depth       = (uint32_t)(td->qos_depth > 0 ? td->qos_depth : 8);
            tq.policy      = strcmp(td->qos_policy, "block") == 0 ? QOS_BLOCK :
                             strcmp(td->qos_policy, "drop_latest") == 0 ? QOS_DROP_LATEST :
                             QOS_DROP_OLDEST;
            tq.reliability = strcmp(td->qos_reliability, "reliable") == 0 ? QOS_RELIABLE
                                                                          : QOS_BEST_EFFORT;
            tq.deadline_ms = (uint32_t)(td->qos_deadline_ms > 0 ? td->qos_deadline_ms : 0);
            tq.lifespan_ms = (uint32_t)(td->qos_lifespan_ms > 0 ? td->qos_lifespan_ms : 0);
            message_bus_set_topic_qos(bus, td->topic, &tq);
        }
    }
}

/* 加载节点 .so, 支持绝对/相对路径与 build/lib/<name> 回退。 */
static void* load_plugin_lib(const char* library) {
    /* 与 flow_launcher run_dlopen_mode 保持一致的 dlopen flags */
    int dlflags = RTLD_LAZY | RTLD_GLOBAL;
    void* h = dlopen(library, dlflags);
    if (!h) {
        char alt[512];
        const char* base = strrchr(library, '/');
        base = base ? base + 1 : library;
        snprintf(alt, sizeof(alt), "build/lib/%s", base);
        h = dlopen(alt, dlflags);
    }
    return h;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <config_path> <node_name> [duration_sec]\n", argv[0]);
        return 2;
    }
    const char* config_path = argv[1];
    const char* node_name   = argv[2];
    int duration            = argc > 3 ? atoi(argv[3]) : 0;

    log_init(LOG_INFO, NULL);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    adas_msgs_register_all();

    LauncherConfig* cfg = config_load(config_path);
    if (!cfg) {
        LOG_WARN("node_host", "failed to load config: %s", config_path);
        log_shutdown();
        return 1;
    }
    int idx = find_node(cfg, node_name);
    if (idx < 0) {
        LOG_WARN("node_host", "node '%s' not found in %s", node_name, config_path);
        config_free(cfg);
        log_shutdown();
        return 1;
    }
    ProcessConfig* pc = &cfg->processes[idx];

    if (!pc->library_path[0]) {
        LOG_WARN("node_host", "node '%s' has no library_path", node_name);
        config_free(cfg);
        log_shutdown();
        return 1;
    }

    LOG_INFO("node_host", "starting node '%s' (%s) as standalone process pid=%d",
             node_name, pc->library_path, (int)getpid());

    /* ── 基础设施 (跨进程: IPC 桥接) ─────────────────────────── */
    MessageBus* bus = message_bus_create(node_name);
    apply_qos(bus, pc);

    DiscoveryManager* discovery =
        discovery_create(node_name, CAP_PUBLISHER | CAP_SUBSCRIBER | CAP_FUSION);
    discovery_start(discovery);

    /* TRANSPORT_IPC: 跨进程节点通过共享内存 + discovery 桥接 topic */
    Transport* transport = transport_create(bus, discovery, TRANSPORT_IPC);
    transport_start(transport);

    SchedulerConfig scfg = SCHEDULER_CONFIG_DEFAULT;
    scfg.mode = SCHEDULER_MODE_CHOREO;
    Scheduler* scheduler = scheduler_create(&scfg);
    scheduler_set_choreo_bus(scheduler, bus);
    scheduler_start(scheduler);

    /* ── 加载并启动节点插件 ──────────────────────────────────── */
    void* lib = load_plugin_lib(pc->library_path);
    if (!lib) {
        LOG_WARN("node_host", "dlopen %s failed: %s", pc->library_path, dlerror());
        goto teardown;
    }

    NodeGetPluginFn get_fn = (NodeGetPluginFn)dlsym(lib, NODE_PLUGIN_SYMBOL);
    if (!get_fn) {
        LOG_WARN("node_host", "%s: symbol '%s' not found", pc->library_path, NODE_PLUGIN_SYMBOL);
        dlclose(lib);
        goto teardown;
    }

    NodePlugin* plugin = get_fn();
    if (!plugin) {
        LOG_WARN("node_host", "%s: node_get_plugin() returned NULL", pc->library_path);
        dlclose(lib);
        goto teardown;
    }

    if (plugin->init(bus, transport, discovery, scheduler,
                     pc->params[0] ? pc->params : NULL) != 0) {
        LOG_WARN("node_host", "node '%s' init() failed", node_name);
        dlclose(lib);
        goto teardown;
    }

    plugin->start();
    LOG_INFO("node_host", "node '%s' running (%s) — v%s: %s",
             node_name, duration > 0 ? "timed" : "until-signal",
             plugin->version, plugin->description);

    if (duration > 0) {
        for (int t = 0; t < duration && g_running; t++) sleep(1);
    } else {
        while (g_running) sleep(1);
    }

    /* 优雅停止 */
    plugin->stop();
    sleep(1);
    plugin->cleanup();
    dlclose(lib);
    LOG_INFO("node_host", "node '%s' stopped", node_name);

teardown:
    scheduler_stop(scheduler);  scheduler_destroy(scheduler);
    transport_stop(transport);  transport_destroy(transport);
    discovery_stop(discovery);  discovery_destroy(discovery);
    message_bus_destroy(bus);
    config_free(cfg);
    log_shutdown();
    return 0;
}
