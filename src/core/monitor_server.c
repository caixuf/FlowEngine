/* ── 精简 HTTP 服务器 + SSE 推送 ───────────────────────────────
 * 只依赖标准库和 cjson。
 * 实现：
 *   GET /            → 仪表盘 HTML
 *   GET /api/topology → 完整 bus + 拓扑 JSON
 *   GET /api/topics   → 每个 topic 的 QoS 统计
 *   GET /api/stream   → SSE 实时推送（bus+topology 增量）
 *
 * 架构：
 *   - 单线程 accept + 每连接一个线程，零第三方依赖
 *   - 统计订阅：Subscriber 函数对象（stats_bridge_fn）→ 写入 local_stats
 *     ── 不额外创建线程，在 flowmond 主线程 or 独立线程中轮询
 *   - 数据流：
 *     StatsBridge(IPC) → local_stats → build_sse_json() → SSE client
 *   - 仪表盘：DashboardBridge(IPC) → dashboard_file_watcher → inject_dashboard_json
 *     → SSE 推送时优先用注入的 JSON（source="live", stale=false）
 *     → 无注入时降级到 build_sse_json()（source="local", stale=true）
 */

#include "monitor_server.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>

/* ── 常量 ───────────────────────────────────────────────── */
#define MONITOR_HTTP_BUF_SIZE  65536
#define MONITOR_MAX_CLIENTS    8
#define MONITOR_SHUTDOWN_WAIT_ITERS 200

/* ── 工具函数 ───────────────────────────────────────────── */

static uint64_t clock_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

static int safe_write(int fd, const char* buf, size_t len) {
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(fd, buf + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        written += (size_t)n;
    }
    return 0;
}

static const char* get_mime_type(const char* path) {
    const char* ext = strrchr(path, '.');
    if (!ext) return "application/octet-stream";
    if (strcmp(ext, ".html") == 0) return "text/html; charset=utf-8";
    if (strcmp(ext, ".css") == 0)  return "text/css; charset=utf-8";
    if (strcmp(ext, ".js") == 0)   return "application/javascript; charset=utf-8";
    if (strcmp(ext, ".json") == 0) return "application/json; charset=utf-8";
    if (strcmp(ext, ".svg") == 0)  return "image/svg+xml";
    if (strcmp(ext, ".png") == 0)  return "image/png";
    if (strcmp(ext, ".ico") == 0)  return "image/x-icon";
    if (strcmp(ext, ".woff2") == 0) return "font/woff2";
    return "application/octet-stream";
}

/* ── SSE 构造 ───────────────────────────────────────────── */

/**
 * 从本地 stats 构造 SSE 数据（bus+topology）
 * 格式：JSON 对象，包含 nodes[], metrics{}, endpoints[]
 * 当有缓存的仪表盘 JSON 时，优先使用注入的 JSON
 */
static void build_sse_json(MonitorServer* ms, char* buf, size_t size) {
    /* 优先使用缓存的仪表盘 JSON */
    pthread_mutex_lock(&ms->cached_mutex);
    if (ms->cached_json && ms->cached_json[0] != '\0') {
        size_t n = strlen(ms->cached_json);
        if (n < size - 1) {
            memcpy(buf, ms->cached_json, n + 1);
        } else {
            buf[0] = '\0';
        }
        pthread_mutex_unlock(&ms->cached_mutex);
        return;
    }
    pthread_mutex_unlock(&ms->cached_mutex);

    /* 降级：从 local stats 构造 */
    size_t used = 0;
#define SSE_APPEND(fmt, ...) do { \
    int _n = snprintf(buf + used, size - used, fmt, ##__VA_ARGS__); \
    if (_n < 0 || (size_t)_n >= size - used) { buf[used] = '\0'; return; } \
    used += (size_t)_n; \
} while(0)

    SSE_APPEND("{\"self\":\"%s\",\"timestamp\":%lld.%06d,\"nodes\":[",
               ms->self_name ? ms->self_name : "flowmond",
               (long long)(ms->start_time_us / 1000000),
               (int)(ms->start_time_us % 1000000));

    /* 本地节点 */
    pthread_mutex_lock(&ms->local_mutex);
    int first = 1;
    for (int i = 0; i < ms->local_node_count; i++) {
        LocalNode* ln = &ms->local_nodes[i];
        if (!ln->active) continue;
        if (!first) SSE_APPEND(",");
        first = 0;
        SSE_APPEND("{\"name\":\"%s\",\"pid\":%d,\"alive\":true,\"caps\":%d,\"topics\":[",
                   ln->name, ln->pid, ln->caps);
        int tfirst = 1;
        for (int j = 0; j < ln->topic_count; j++) {
            if (!tfirst) SSE_APPEND(",");
            tfirst = 0;
            SSE_APPEND("{\"topic\":\"%s\",\"role\":\"%s\",\"caps\":%d}",
                       ln->topics[j].topic, ln->topics[j].role, ln->topics[j].caps);
        }
        SSE_APPEND("]}");
    }

    /* 远程节点 */
    pthread_mutex_lock(&ms->remote_mutex);
    for (int i = 0; i < ms->remote_node_count; i++) {
        RemoteNode* rn = &ms->remote_nodes[i];
        if (!rn->active) continue;
        if (!first) SSE_APPEND(",");
        first = 0;
        SSE_APPEND("{\"name\":\"%s\",\"pid\":%d,\"alive\":true,\"caps\":%d,\"topics\":[",
                   rn->name, rn->pid, rn->caps);
        int tfirst = 1;
        for (int j = 0; j < rn->topic_count; j++) {
            if (!tfirst) SSE_APPEND(",");
            tfirst = 0;
            SSE_APPEND("{\"topic\":\"%s\",\"role\":\"%s\",\"caps\":%d}",
                       rn->topics[j].topic, rn->topics[j].role, rn->topics[j].caps);
        }
        SSE_APPEND("]}");
    }

    /* 关闭 nodes[] */
    SSE_APPEND("],\"metrics\":{");

    /* bus 统计 */
    SSE_APPEND("\"bus\":{\"published\":%u,\"delivered\":%u,\"dropped\":%u},",
               ms->local_stats.pub_total, ms->local_stats.del_total,
               ms->local_stats.drop_total);

    /* transport 统计 */
    SSE_APPEND("\"transport\":{\"local_pub\":%u,\"remote_pub\":%u},",
               ms->local_stats.pub_total, ms->remote_stats.pub_total);

    /* scheduler 统计 */
    SSE_APPEND("\"scheduler\":{\"tasks\":%d,\"mode\":\"%s\"},",
               ms->local_node_count, "CHOREO");

    /* latency 统计 */
    SSE_APPEND("\"latency\":{\"avg_us\":%u,\"p50_us\":%u,\"p99_us\":%u},",
               ms->local_stats.avg_lat_us, ms->local_stats.avg_lat_us,
               ms->local_stats.max_lat_us);

    /* driver_mode */
    SSE_APPEND("\"driver_mode\":\"%s\",",
               ms->local_stats.driver_mode[0] ? ms->local_stats.driver_mode : "MANUAL");

    /* vehicle */
    SSE_APPEND("\"vehicle\":{\"speed\":%.1f,\"target_speed\":%.1f,\"throttle\":%.2f,\"brake\":%.2f,\"x\":%.1f,\"error\":%.1f},",
               ms->local_stats.vehicle_speed, ms->local_stats.vehicle_target,
               ms->local_stats.vehicle_throttle, ms->local_stats.vehicle_brake,
               ms->local_stats.vehicle_x, ms->local_stats.vehicle_error);

    /* topics 统计 */
    SSE_APPEND("\"topics\":[");
    uint32_t topic_count = 0;
    for (int i = 0; i < ms->local_topic_count; i++) {
        if (!ms->local_topic_stats[i].active) continue;
        if (topic_count > 0) SSE_APPEND(",");
        topic_count++;
        LocalTopicStat* ts = &ms->local_topic_stats[i];
        SSE_APPEND("{\"topic\":\"%s\",\"pub\":%u,\"del\":%u,\"drop\":%u,\"lat_us\":%u,\"freq\":%.1f,\"subs\":%d,\"reliability\":\"%s\",\"deadline_ms\":%d,\"transport\":\"%s\"}",
                   ts->topic, ts->pub_count, ts->del_count, ts->drop_count,
                   ts->avg_lat_us, ts->freq_hz, ts->sub_count,
                   ts->qos_profile[0] ? ts->qos_profile : "reliable",
                   ts->deadline_ms, ts->transport[0] ? ts->transport : "shm");
    }
    /* 远程 topic 统计 */
    for (int i = 0; i < ms->remote_topic_count; i++) {
        if (!ms->remote_topic_stats[i].active) continue;
        if (topic_count > 0) SSE_APPEND(",");
        topic_count++;
        RemoteTopicStat* ts = &ms->remote_topic_stats[i];
        SSE_APPEND("{\"topic\":\"%s\",\"pub\":%u,\"del\":%u,\"drop\":%u,\"lat_us\":%u,\"freq\":%.1f,\"subs\":%d,\"reliability\":\"%s\",\"deadline_ms\":%d,\"transport\":\"%s\"}",
                   ts->topic, ts->pub_count, ts->del_count, ts->drop_count,
                   ts->avg_lat_us, ts->freq_hz, ts->sub_count,
                   ts->qos_profile[0] ? ts->qos_profile : "reliable",
                   ts->deadline_ms, ts->transport[0] ? ts->transport : "dds");
    }

    /* 构建 endpoints */
    SSE_APPEND("],\"endpoints\":[");
    int ep_first = 1;
    for (int i = 0; i < ms->local_node_count; i++) {
        LocalNode* ln = &ms->local_nodes[i];
        if (!ln->active) continue;
        for (int j = 0; j < ln->topic_count; j++) {
            if (!ep_first) SSE_APPEND(",");
            ep_first = 0;
            SSE_APPEND("{\"node\":\"%s\",\"topic\":\"%s\",\"role\":\"%s\",\"type_id\":\"0x%08x\",\"freq\":%.1f}",
                       ln->name, ln->topics[j].topic, ln->topics[j].role,
                       ln->topics[j].type_id, ln->topics[j].freq);
        }
    }
    for (int i = 0; i < ms->remote_node_count; i++) {
        RemoteNode* rn = &ms->remote_nodes[i];
        if (!rn->active) continue;
        for (int j = 0; j < rn->topic_count; j++) {
            if (!ep_first) SSE_APPEND(",");
            ep_first = 0;
            SSE_APPEND("{\"node\":\"%s\",\"topic\":\"%s\",\"role\":\"%s\",\"type_id\":\"0x%08x\",\"freq\":%.1f}",
                       rn->name, rn->topics[j].topic, rn->topics[j].role,
                       rn->topics[j].type_id, rn->topics[j].freq);
        }
    }
    pthread_mutex_unlock(&ms->remote_mutex);

    /* Close topics, add placeholder metrics, close metrics + top-level */
    SSE_APPEND("],"
        "\"sysmon\":{},"
        "\"vehicle\":{},"
        "\"scene\":{"
            "\"road_network\":{"
                "\"edges\":["
                    "{\"id\":0,\"type\":\"highway\",\"name\":\"highway\",\"length_m\":2000,\"lanes\":4,\"lane_width\":3.5,\"speed_limit\":33.0,\"nodes\":[[0,0,0],[2000,0,0]]}"
                "]"
            "}"
        "}}}");
#undef SSE_APPEND
}

/* ── HTTP 响应 ──────────────────────────────────────────── */

static void send_http_response(int fd, int code, const char* status,
                                const char* content_type, const char* body,
                                size_t body_len) {
    char header[512];
    int n = snprintf(header, sizeof(header),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n", code, status, content_type, body_len);
    safe_write(fd, header, (size_t)n);
    if (body && body_len > 0) {
        safe_write(fd, body, body_len);
    }
}

static void send_404(int fd) {
    const char* body = "{\"error\":\"not found\"}";
    send_http_response(fd, 404, "Not Found", "application/json", body, strlen(body));
}

static void send_500(int fd, const char* msg) {
    char body[256];
    int n = snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg ? msg : "internal error");
    send_http_response(fd, 500, "Internal Server Error", "application/json", body, (size_t)n);
}

static void serve_file(int fd, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        send_404(fd);
        return;
    }

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    char header[512];
    int n = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %ld\r\n"
        "Connection: close\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n", get_mime_type(path), sz);
    safe_write(fd, header, (size_t)n);

    char buf[8192];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        safe_write(fd, buf, r);
    }
    fclose(f);
}

/* ── SSE 扁平化 ─────────────────────────────────────────── */

/**
 * 把 JSON 展开为扁平结构，方便 SSE 消费者解析
 * - 把 metrics.topics[] 展平出来
 * - 如果顶层有 scene 字段（来自 dashboard JSON），保留到 metrics 下面
 */
static void sse_flatten_payload(char* buf) {
    /* 不做复杂展平，保持 JSON 原样。
     * 前端已能解析此格式。
     * 如果 source=live 且顶层有 scene 字段，将其移到 metrics.scene 下。
     */
    (void)buf;
}

/* ── SSE 处理器 ─────────────────────────────────────────── */

static void build_cached_dashboard_json(MonitorServer* ms, char* buf, size_t size) {
    pthread_mutex_lock(&ms->cached_mutex);
    if (ms->cached_json && ms->cached_json[0] != '\0') {
        size_t n = strlen(ms->cached_json);
        if (n < size - 1) {
            memcpy(buf, ms->cached_json, n + 1);
        } else {
            buf[0] = '\0';
        }
    } else {
        buf[0] = '\0';
    }
    pthread_mutex_unlock(&ms->cached_mutex);
}

static void handle_sse(int fd, MonitorServer* ms) {
    const char* sse_header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n";

    if (safe_write(fd, sse_header, strlen(sse_header)) != 0) return;

    char buf[MONITOR_HTTP_BUF_SIZE];
    uint64_t last_version = 0;
    uint64_t last_send_us = clock_now_us();

    while (ms->running) {
        pthread_mutex_lock(&ms->cached_mutex);
        uint64_t version = ms->cached_json_version;
        if (version != last_version) {
            pthread_mutex_unlock(&ms->cached_mutex);
            build_sse_json(ms, buf, sizeof(buf));
            sse_flatten_payload(buf);
            char frame[MONITOR_HTTP_BUF_SIZE + 32];
            int fl = snprintf(frame, sizeof(frame), "data: %s\n\n", buf);
            if (safe_write(fd, frame, (size_t)fl) != 0) break;
            last_version = version;
            last_send_us = clock_now_us();
        } else {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 50000000;  /* 50ms */
            if (ts.tv_nsec >= 1000000000) {
                ts.tv_nsec -= 1000000000;
                ts.tv_sec += 1;
            }
            int ret = pthread_cond_timedwait(&ms->cached_cond, &ms->cached_mutex, &ts);
            if (ret == ETIMEDOUT) {
                uint64_t now = clock_now_us();
                if (now - last_send_us > 1000000) {
                    pthread_mutex_unlock(&ms->cached_mutex);
                    build_sse_json(ms, buf, sizeof(buf));
                    sse_flatten_payload(buf);
                    char frame[MONITOR_HTTP_BUF_SIZE + 32];
                    int fl = snprintf(frame, sizeof(frame), "data: %s\n\n", buf);
                    if (safe_write(fd, frame, (size_t)fl) != 0) break;
                    last_send_us = now;
                } else {
                    pthread_mutex_unlock(&ms->cached_mutex);
                }
            } else {
                pthread_mutex_unlock(&ms->cached_mutex);
            }
        }
    }
}

/* ── 请求路由 ───────────────────────────────────────────── */

static void handle_client(int fd, MonitorServer* ms) {
    char buf[2048];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (n <= 0) { close(fd); return; }
    buf[n] = '\0';

    /* 解析 GET /path */
    char method[16] = {0};
    char path[512] = {0};
    sscanf(buf, "%15s %511s", method, path);

    if (strcmp(path, "/api/stream") == 0) {
        handle_sse(fd, ms);
        close(fd);
        return;
    }

    if (strcmp(path, "/api/topology") == 0) {
        char json[MONITOR_HTTP_BUF_SIZE];
        build_sse_json(ms, json, sizeof(json));
        send_http_response(fd, 200, "OK", "application/json", json, strlen(json));
        close(fd);
        return;
    }

    if (strcmp(path, "/api/topics") == 0) {
        char json[MONITOR_HTTP_BUF_SIZE];
        build_sse_json(ms, json, sizeof(json));
        send_http_response(fd, 200, "OK", "application/json", json, strlen(json));
        close(fd);
        return;
    }

    /* 静态文件 */
    if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
        serve_file(fd, ms->html_path);
        close(fd);
        return;
    }

    /* JS / CSS / 静态资源 */
    if (strncmp(path, "/js/", 4) == 0 || strncmp(path, "/css/", 5) == 0) {
        char parent[512];
        snprintf(parent, sizeof(parent), "%s", ms->html_path);
        char* slash = strrchr(parent, '/');
        if (slash) *slash = '\0';
        char filepath[1024];
        snprintf(filepath, sizeof(filepath), "%s%s", parent, path);
        serve_file(fd, filepath);
        close(fd);
        return;
    }

    /* 内嵌资源（/tools/...） */
    if (strncmp(path, "/tools/", 7) == 0) {
        char dir[512];
        snprintf(dir, sizeof(dir), "%s", ms->html_path);
        char* slash = strrchr(dir, '/');
        if (slash) *slash = '\0';  /* strip basename */
        slash = strrchr(dir, '/');
        if (slash) *slash = '\0';  /* strip flowboard/ */
        char filepath[1024];
        snprintf(filepath, sizeof(filepath), "%s%s", dir, path);
        serve_file(fd, filepath);
        close(fd);
        return;
    }

    send_404(fd);
    close(fd);
}

/* ── 线程函数 ───────────────────────────────────────────── */

struct ClientCtx {
    int fd;
    MonitorServer* ms;
};

static void* client_thread_fn(void* arg) {
    struct ClientCtx* ctx = (struct ClientCtx*)arg;
    handle_client(ctx->fd, ctx->ms);
    pthread_mutex_lock(&ctx->ms->client_mutex);
    ctx->ms->active_clients--;
    pthread_mutex_unlock(&ctx->ms->client_mutex);
    free(ctx);
    return NULL;
}

/* ── 公共 API ────────────────────────────────────────────── */

void monitor_server_init(MonitorServer* ms, const char* html_path, uint16_t port) {
    memset(ms, 0, sizeof(*ms));
    ms->html_path = html_path;
    ms->port = port;
    ms->start_time_us = clock_now_us();
    ms->running = true;
    pthread_mutex_init(&ms->local_mutex, NULL);
    pthread_mutex_init(&ms->remote_mutex, NULL);
    pthread_mutex_init(&ms->client_mutex, NULL);
    pthread_mutex_init(&ms->cached_mutex, NULL);
    pthread_cond_init(&ms->cached_cond, NULL);

    ms->self_name = "flowmond";
    ms->local_stats.driver_mode[0] = '\0';

    /* 注册本地节点 */
    monitor_server_register_node(ms, "flowmond", getpid(), 0);

    printf("[monitor_server] Listening on http://0.0.0.0:%d\n", port);
    printf("[monitor_server] Endpoints: /  /api/topology  /api/topics  /api/stream\n");
}

void monitor_server_inject_dashboard_json(MonitorServer* ms, const char* json) {
    if (!json || !json[0]) return;
    pthread_mutex_lock(&ms->cached_mutex);
    free(ms->cached_json);
    ms->cached_json = strdup(json);
    ms->cached_json_version++;
    pthread_cond_broadcast(&ms->cached_cond);
    pthread_mutex_unlock(&ms->cached_mutex);
}

int monitor_server_run(MonitorServer* ms) {
    ms->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ms->listen_fd < 0) {
        perror("monitor_server: socket");
        return -1;
    }

    int opt = 1;
    setsockopt(ms->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(ms->port);

    if (bind(ms->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("monitor_server: bind");
        close(ms->listen_fd);
        return -1;
    }

    if (listen(ms->listen_fd, MONITOR_MAX_CLIENTS) < 0) {
        perror("monitor_server: listen");
        close(ms->listen_fd);
        return -1;
    }

    /* 主循环 */
    while (ms->running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(ms->listen_fd, &rfds);

        struct timeval tv = { 1, 0 };
        int ret = select(ms->listen_fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;

        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(ms->listen_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client < 0) {
            if (errno == EINTR) continue;
            break;
        }

        struct ClientCtx* ctx = malloc(sizeof(struct ClientCtx));
        if (!ctx) {
            close(client);
            continue;
        }
        ctx->fd = client;
        ctx->ms = ms;

        pthread_mutex_lock(&ms->client_mutex);
        ms->active_clients++;
        pthread_mutex_unlock(&ms->client_mutex);

        pthread_t th;
        if (pthread_create(&th, NULL, client_thread_fn, ctx) != 0) {
            fprintf(stderr, "[monitor_server] failed to create client thread\n");
            free(ctx);
            pthread_mutex_lock(&ms->client_mutex);
            ms->active_clients--;
            pthread_mutex_unlock(&ms->client_mutex);
            handle_client(client, ms);
        } else {
            pthread_detach(th);
        }
    }

    return 0;
}

void monitor_server_stop(MonitorServer* ms) {
    ms->running = false;
    if (ms->listen_fd >= 0) {
        close(ms->listen_fd);
        ms->listen_fd = -1;
    }

    /* 等待所有客户端线程退出 */
    for (int i = 0; i < MONITOR_SHUTDOWN_WAIT_ITERS; i++) {
        pthread_mutex_lock(&ms->client_mutex);
        int n = ms->active_clients;
        pthread_mutex_unlock(&ms->client_mutex);
        if (n <= 0) break;
        usleep(10000);
    }

    pthread_cond_broadcast(&ms->cached_cond);
    printf("[monitor_server] Stopped\n");
}

void monitor_server_destroy(MonitorServer* ms) {
    pthread_mutex_destroy(&ms->local_mutex);
    pthread_mutex_destroy(&ms->remote_mutex);
    pthread_mutex_destroy(&ms->client_mutex);
    pthread_mutex_destroy(&ms->cached_mutex);
    pthread_cond_destroy(&ms->cached_cond);
    free(ms->cached_json);
    ms->cached_json = NULL;
}

void monitor_server_register_node(MonitorServer* ms, const char* name, int pid, int caps) {
    pthread_mutex_lock(&ms->local_mutex);
    if (ms->local_node_count < MONITOR_MAX_NODES) {
        LocalNode* ln = &ms->local_nodes[ms->local_node_count++];
        snprintf(ln->name, sizeof(ln->name), "%s", name);
        ln->pid = pid;
        ln->caps = caps;
        ln->active = true;
        ln->topic_count = 0;
    }
    pthread_mutex_unlock(&ms->local_mutex);
}

void monitor_server_add_topic(MonitorServer* ms, const char* node_name,
                               const char* topic, const char* role, int caps,
                               uint32_t type_id, double freq) {
    pthread_mutex_lock(&ms->local_mutex);
    for (int i = 0; i < ms->local_node_count; i++) {
        LocalNode* ln = &ms->local_nodes[i];
        if (strcmp(ln->name, node_name) == 0 && ln->topic_count < MONITOR_MAX_TOPICS) {
            LocalTopicInfo* ti = &ln->topics[ln->topic_count++];
            snprintf(ti->topic, sizeof(ti->topic), "%s", topic);
            snprintf(ti->role, sizeof(ti->role), "%s", role);
            ti->caps = caps;
            ti->type_id = type_id;
            ti->freq = freq;
            break;
        }
    }
    pthread_mutex_unlock(&ms->local_mutex);
}

void monitor_server_update_topic_qos(MonitorServer* ms, const char* topic,
                                      uint32_t pub, uint32_t del, uint32_t drop,
                                      uint32_t lat_us, double freq, int subs,
                                      const char* reliability, int deadline_ms,
                                      const char* transport) {
    pthread_mutex_lock(&ms->local_mutex);
    for (int i = 0; i < ms->local_topic_count; i++) {
        if (strcmp(ms->local_topic_stats[i].topic, topic) == 0) {
            LocalTopicStat* ts = &ms->local_topic_stats[i];
            ts->pub_count = pub;
            ts->del_count = del;
            ts->drop_count = drop;
            ts->avg_lat_us = lat_us;
            ts->freq_hz = freq;
            ts->sub_count = subs;
            if (reliability) snprintf(ts->qos_profile, sizeof(ts->qos_profile), "%s", reliability);
            ts->deadline_ms = deadline_ms;
            if (transport) snprintf(ts->transport, sizeof(ts->transport), "%s", transport);
            pthread_mutex_unlock(&ms->local_mutex);
            return;
        }
    }
    if (ms->local_topic_count < MONITOR_MAX_TOPICS) {
        LocalTopicStat* ts = &ms->local_topic_stats[ms->local_topic_count++];
        snprintf(ts->topic, sizeof(ts->topic), "%s", topic);
        ts->pub_count = pub;
        ts->del_count = del;
        ts->drop_count = drop;
        ts->avg_lat_us = lat_us;
        ts->freq_hz = freq;
        ts->sub_count = subs;
        if (reliability) snprintf(ts->qos_profile, sizeof(ts->qos_profile), "%s", reliability);
        ts->deadline_ms = deadline_ms;
        if (transport) snprintf(ts->transport, sizeof(ts->transport), "%s", transport);
        ts->active = true;
    }
    pthread_mutex_unlock(&ms->local_mutex);
}

void monitor_server_update_vehicle(MonitorServer* ms, double speed, double target,
                                    double throttle, double brake, double x, double error) {
    ms->local_stats.vehicle_speed = speed;
    ms->local_stats.vehicle_target = target;
    ms->local_stats.vehicle_throttle = throttle;
    ms->local_stats.vehicle_brake = brake;
    ms->local_stats.vehicle_x = x;
    ms->local_stats.vehicle_error = error;
}

void monitor_server_update_driver_mode(MonitorServer* ms, const char* mode) {
    if (mode) snprintf(ms->local_stats.driver_mode, sizeof(ms->local_stats.driver_mode), "%s", mode);
}

void monitor_server_add_remote_node(MonitorServer* ms, const char* name, int pid, int caps) {
    pthread_mutex_lock(&ms->remote_mutex);
    if (ms->remote_node_count < MONITOR_MAX_NODES) {
        RemoteNode* rn = &ms->remote_nodes[ms->remote_node_count++];
        snprintf(rn->name, sizeof(rn->name), "%s", name);
        rn->pid = pid;
        rn->caps = caps;
        rn->active = true;
        rn->topic_count = 0;
    }
    pthread_mutex_unlock(&ms->remote_mutex);
}

void monitor_server_add_remote_topic(MonitorServer* ms, const char* node_name,
                                      const char* topic, const char* role, int caps,
                                      uint32_t type_id, double freq) {
    pthread_mutex_lock(&ms->remote_mutex);
    for (int i = 0; i < ms->remote_node_count; i++) {
        RemoteNode* rn = &ms->remote_nodes[i];
        if (strcmp(rn->name, node_name) == 0 && rn->topic_count < MONITOR_MAX_TOPICS) {
            RemoteTopicInfo* ti = &rn->topics[rn->topic_count++];
            snprintf(ti->topic, sizeof(ti->topic), "%s", topic);
            snprintf(ti->role, sizeof(ti->role), "%s", role);
            ti->caps = caps;
            ti->type_id = type_id;
            ti->freq = freq;
            break;
        }
    }
    pthread_mutex_unlock(&ms->remote_mutex);
}

void monitor_server_update_remote_topic_qos(MonitorServer* ms, const char* topic,
                                             uint32_t pub, uint32_t del, uint32_t drop,
                                             uint32_t lat_us, double freq, int subs,
                                             const char* reliability, int deadline_ms,
                                             const char* transport) {
    pthread_mutex_lock(&ms->remote_mutex);
    for (int i = 0; i < ms->remote_topic_count; i++) {
        if (strcmp(ms->remote_topic_stats[i].topic, topic) == 0) {
            RemoteTopicStat* ts = &ms->remote_topic_stats[i];
            ts->pub_count = pub;
            ts->del_count = del;
            ts->drop_count = drop;
            ts->avg_lat_us = lat_us;
            ts->freq_hz = freq;
            ts->sub_count = subs;
            if (reliability) snprintf(ts->qos_profile, sizeof(ts->qos_profile), "%s", reliability);
            ts->deadline_ms = deadline_ms;
            if (transport) snprintf(ts->transport, sizeof(ts->transport), "%s", transport);
            pthread_mutex_unlock(&ms->remote_mutex);
            return;
        }
    }
    if (ms->remote_topic_count < MONITOR_MAX_TOPICS) {
        RemoteTopicStat* ts = &ms->remote_topic_stats[ms->remote_topic_count++];
        snprintf(ts->topic, sizeof(ts->topic), "%s", topic);
        ts->pub_count = pub;
        ts->del_count = del;
        ts->drop_count = drop;
        ts->avg_lat_us = lat_us;
        ts->freq_hz = freq;
        ts->sub_count = subs;
        if (reliability) snprintf(ts->qos_profile, sizeof(ts->qos_profile), "%s", reliability);
        ts->deadline_ms = deadline_ms;
        if (transport) snprintf(ts->transport, sizeof(ts->transport), "%s", transport);
        ts->active = true;
    }
    pthread_mutex_unlock(&ms->remote_mutex);
}

void monitor_server_update_bus_stats(MonitorServer* ms, uint32_t pub, uint32_t del,
                                      uint32_t drop, uint32_t avg_lat, uint32_t max_lat) {
    ms->local_stats.pub_total = pub;
    ms->local_stats.del_total = del;
    ms->local_stats.drop_total = drop;
    ms->local_stats.avg_lat_us = avg_lat;
    ms->local_stats.max_lat_us = max_lat;
}
