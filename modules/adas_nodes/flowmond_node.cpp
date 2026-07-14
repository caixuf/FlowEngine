/**
 * flowmond_node.cpp — HTTP 监控服务节点 (NodePlugin C++)
 *
 * 将 flowmond 的 HTTP server 包装为 NodePlugin，由 flow_launcher 动态加载。
 * 不再需要单独运行 flowmond 二进制；在 pipeline.json 中添加此节点即可
 * 在仿真进程内直接提供仪表盘 HTTP 服务。
 *
 * 设计要点:
 *   1. 零依赖: 复用已有的 monitor_server.c（C 链接），无需额外库。
 *   2. 生命周期: init() 配置 HTTP 端口等参数，start() 启动 server 线程，
 *      stop() 优雅停止，cleanup() 回收资源。
 *   3. 配置: 从 params_json 读取 port（默认 8800）、bind（默认 127.0.0.1）。
 *
 * NodePlugin 接口，编译为 libflowmond_node.so。
 */

#include "node_plugin.h"
#include "monitor_server.h"
#include "logger.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <dlfcn.h>
#include <pthread.h>
#include <sys/stat.h>

extern "C" {
#include <unistd.h>
}

/* ── 节点本地状态 ───────────────────────────────────────────── */

/* dladdr anchor: dladdr 需要一个动态导出的符号来定位 .so 文件路径。
 * 用 __attribute__((used)) 确保编译器不把它优化掉。 */
extern "C" void flowmond_dladdr_anchor(void) __attribute__((used));
extern "C" void flowmond_dladdr_anchor(void) {}

static struct {
    Transport*        transport;
    DiscoveryManager* discovery;
    Scheduler*        scheduler;

    MonitorServer*    server;
    int               port;
    char              bind_addr[64];
    char              html_path[512];
    char              pipeline[512];
} g;

/* ── NodePlugin 实现 ─────────────────────────────────────────── */

static const char* s_inputs[]  = { NULL };
static const char* s_outputs[] = { NULL };

/* Plugin descriptor - defined after function bodies but forward-declared
 * here so functions like flowmond_start can reference it. */
extern NodePlugin s_plugin;

static int flowmond_init(MessageBus* bus, Transport* transport,
                          DiscoveryManager* discovery, Scheduler* scheduler,
                          const char* params_json) {
    (void)bus;
    memset(&g, 0, sizeof(g));
    g.transport    = transport;
    g.discovery    = discovery;
    g.scheduler    = scheduler;
    g.port         = 8800;
    snprintf(g.bind_addr, sizeof(g.bind_addr), "127.0.0.1");
    g.pipeline[0]  = '\0';

    /* 解析参数 */
    if (params_json) {
        const char* p;
        if ((p = strstr(params_json, "\"port\":")))
            sscanf(p + 7, "%d", &g.port);
        if ((p = strstr(params_json, "\"bind\":"))) {
            char val[64] = "";
            sscanf(p + 7, "%63s", val);
            size_t vl = strlen(val);
            if (vl > 2 && val[0] == '"' && val[vl-1] == '"') {
                val[vl-1] = '\0';
                snprintf(g.bind_addr, sizeof(g.bind_addr), "%s", val + 1);
            } else if (vl > 0) {
                snprintf(g.bind_addr, sizeof(g.bind_addr), "%s", val);
            }
        }
        if ((p = strstr(params_json, "\"pipeline\":"))) {
            char val[256] = "";
            sscanf(p + 11, "%255s", val);
            size_t vl = strlen(val);
            if (vl > 2 && val[0] == '"' && val[vl-1] == '"') {
                val[vl-1] = '\0';
                snprintf(g.pipeline, sizeof(g.pipeline), "%s", val + 1);
            } else if (vl > 0) {
                snprintf(g.pipeline, sizeof(g.pipeline), "%s", val);
            }
            if (g.pipeline[0]) setenv("FLOW_PIPELINE", g.pipeline, 1);
        }
    }

    /* 自动检测 html_path: flow_launcher 的 CWD 是项目根目录，
     * 直接使用相对路径 tools/flowboard/index.html */
    {
        const char* candidates[] = {
            "tools/flowboard/index.html",
            "tools/flowboard.html",
            NULL
        };
        g.html_path[0] = '\0';
        for (int ci = 0; candidates[ci]; ci++) {
            if (access(candidates[ci], F_OK) == 0) {
                /* 转为绝对路径以便 monitor_server 后续使用 */
                char* abs = realpath(candidates[ci], NULL);
                if (abs) {
                    snprintf(g.html_path, sizeof(g.html_path), "%s", abs);
                    free(abs);
                } else {
                    snprintf(g.html_path, sizeof(g.html_path), "%s", candidates[ci]);
                }
                break;
            }
        }
        if (!g.html_path[0]) {
            LOG_WARN("flowmond", "could not find flowboard/index.html in CWD=%s, using embedded fallback",
                     getcwd(NULL, 0) ? getcwd(NULL, 0) : "?");
        } else {
            LOG_INFO("flowmond", "html_path: %s", g.html_path);
        }
    }

    /* 设置 bind addr（通过环境变量，与 flowmond 二进制行为一致） */
    setenv("FLOWMOND_BIND_ADDR", g.bind_addr, 1);

    /* 创建 monitor server */
    g.server = monitor_server_create(NULL, g.discovery, g.port,
                                      g.html_path[0] ? g.html_path : NULL);
    if (!g.server) {
        LOG_ERROR("flowmond", "monitor_server_create failed");
        return -1;
    }

    LOG_INFO("flowmond", "initialized (port=%d, bind=%s, html=%s%s%s)",
             g.port, g.bind_addr,
             g.html_path[0] ? g.html_path : "<embedded>",
             g.pipeline[0] ? ", pipeline=" : "",
             g.pipeline[0] ? g.pipeline : "");
    return 0;
}

/* 后台线程：监控 /tmp/flow_topology.json 并注入到 monitor_server，
 * 使 3D scene 数据（自车、障碍物、弯道等）能通过 HTTP API 获取。 */
static pthread_t g_watcher_thread;
static volatile int g_watcher_running = 0;
static void* dashboard_file_watcher(void*) {
    const char* path = "/tmp/flow_topology.json";
    uint64_t last_mtime = 0;
    while (g_watcher_running) {
        struct stat st;
        if (stat(path, &st) == 0) {
            uint64_t mtime = (uint64_t)st.st_mtime;
            if (mtime != last_mtime) {
                last_mtime = mtime;
                size_t len = 0;
                FILE* fp = fopen(path, "rb");
                if (fp) {
                    fseek(fp, 0, SEEK_END);
                    len = (size_t)ftell(fp);
                    fseek(fp, 0, SEEK_SET);
                    char* data = (char*)malloc(len + 1);
                    if (data) {
                        size_t n = fread(data, 1, len, fp);
                        data[n] = '\0';
                        monitor_server_inject_dashboard_json(g.server, data, n);
                        free(data);
                    }
                    fclose(fp);
                }
            }
        }
        usleep(200000); /* 5Hz */
    }
    return NULL;
}

static int flowmond_start(void) {
    if (!g.server) return -1;
    monitor_server_start(g.server);
    /* 启动文件监控线程 */
    g_watcher_running = 1;
    pthread_create(&g_watcher_thread, NULL, dashboard_file_watcher, NULL);
    LOG_INFO("flowmond", "started on http://%s:%d", g.bind_addr, g.port);
    node_announce_self(g.transport, &s_plugin);
    return 0;
}

static void flowmond_stop(void) {
    /* 先停 watcher 线程，再停 server */
    g_watcher_running = 0;
    pthread_join(g_watcher_thread, NULL);
    if (g.server) monitor_server_stop(g.server);
    LOG_INFO("flowmond", "stopped");
}

static void flowmond_cleanup(void) {
    if (g.server) {
        monitor_server_destroy(g.server);
        g.server = NULL;
    }
    LOG_INFO("flowmond", "cleanup done");
}

static int flowmond_health(void) { return 0; }

/* The extern declaration at top of file matches this definition.
 * Non-static linkage so the forward declaration works in C++. */
NodePlugin s_plugin = {
    .api_version   = NODE_PLUGIN_API_VERSION,
    .name          = "flowmond",
    .version       = "1.0.0",
    .description   = "HTTP monitoring dashboard server (embedded)",
    .input_topics  = s_inputs,
    .output_topics = s_outputs,
    .init          = flowmond_init,
    .start         = flowmond_start,
    .stop          = flowmond_stop,
    .cleanup       = flowmond_cleanup,
    .health        = flowmond_health,
};

extern "C" NodePlugin* node_get_plugin(void) { return &s_plugin; }
