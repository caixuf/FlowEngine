/**
 * monitor_server.c — 内嵌 HTTP 监控服务器 (cyber_monitor 等价物)
 *
 * 直接在 FlowEngine 进程内启动一个微型 HTTP 服务器，
 * 实时读取 message_bus 统计、discovery 拓扑、per-topic QoS 数据，
 * 通过 SSE (Server-Sent Events) 推送到浏览器。
 *
 * 无需外部 Python 进程，无需 JSON 文件中转。
 *
 * 用法:
 *   MonitorServer* ms = monitor_server_create(bus, discovery, 8800);
 *   monitor_server_start(ms);
 *   // 浏览器打开 http://localhost:8800
 *   // 实时看到: 拓扑图 + 帧监控 + 时序图 + QoS 表
 *   monitor_server_stop(ms);
 */

#include "message_bus.h"
#include "discovery.h"
#include "serializer.h"
#include "stats_bridge.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <sys/time.h>

#define MONITOR_MAX_CLIENTS       8
#define MONITOR_HTTP_BUF_SIZE     65536
#define MONITOR_MAX_REMOTE_SRCS   8
#define DASHBOARD_CACHE_STALE_SEC 3  /* seconds before cached JSON considered stale */

/* ── Remote source entry (stats from another process via IPC) */
typedef struct {
    char        source_name[64];
    StatsPacket pkt;
    bool        valid;
} RemoteSource;
typedef struct {
    MessageBus*       bus;
    DiscoveryManager* discovery;
    int               port;
    int               listen_fd;
    volatile bool     running;
    pthread_t         server_thread;
    char              html_path[512];  /**< Path to flowboard.html (empty = embedded) */
    /* Remote stats injected via IPC bridge */
    RemoteSource      remote[MONITOR_MAX_REMOTE_SRCS];
    int               remote_count;
    pthread_mutex_t   remote_mutex;
    /* Cached full dashboard JSON from monitor_node via IPC */
    char*             cached_json;
    size_t            cached_json_len;
    uint64_t          cached_json_time_us;
    pthread_mutex_t   cached_mutex;
    /* Count of in-flight per-connection handler threads (e.g. SSE streams).
     * Used so shutdown can wait for them to drain before the struct is freed. */
    volatile int      active_clients;
    pthread_mutex_t   client_mutex;
} MonitorServer;

/* ── File read helper ────────────────────────────────────── */

static char* read_file(const char* path, size_t* out_len) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0) { fclose(f); return NULL; }
    char* buf = (char*)malloc((size_t)len + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t n = fread(buf, 1, (size_t)len, f);
    fclose(f);
    buf[n] = '\0';
    if (out_len) *out_len = n;
    return buf;
}

/* ── SSE 数据生成 (flowboard.html 兼容格式) ────────────────── */

/**
 * Include <time.h> for clock_gettime if not already included.
 * It's already included above.
 */

/**
 * Build dashboard JSON from cached data (from monitor_node).
 * Wraps the cached JSON with source/stale/age_sec fields.
 * @return Length written to buf, or 0 if no cache available.
 */
static int build_cached_dashboard_json(MonitorServer* ms, char* buf, size_t sz) {
    if (!ms || !buf || sz == 0) return 0;

    pthread_mutex_lock(&ms->cached_mutex);
    int ret = 0;
    if (ms->cached_json && ms->cached_json_len > 1) {
        uint64_t now_us;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        now_us = (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;

        uint64_t age_us = (now_us > ms->cached_json_time_us)
                          ? now_us - ms->cached_json_time_us : 0;
        double age_sec = (double)age_us / 1000000.0;
        bool stale = (age_sec > (double)DASHBOARD_CACHE_STALE_SEC);

        /* Inject source/stale/age_sec right after the opening '{' */
        const char* src = stale ? "stale" : "live";
        int off = snprintf(buf, sz,
            "{\"source\":\"%s\",\"stale\":%s,\"age_sec\":%.1f,",
            src, stale ? "true" : "false", age_sec);

        size_t remain = sz - (size_t)off;
        if (off > 0 && remain > 1) {
            /* Skip the leading '{' of cached JSON */
            const char* body = ms->cached_json + 1;
            size_t body_len = ms->cached_json_len - 1;
            size_t copy = body_len < remain - 1 ? body_len : remain - 1;
            memcpy(buf + off, body, copy);
            off += (int)copy;
            buf[off] = '\0';
            ret = off;
        }
    }
    pthread_mutex_unlock(&ms->cached_mutex);
    return ret;
}

static void build_sse_json(MonitorServer* ms, char* buf, size_t sz) {
    /* If we have a cached full dashboard JSON from monitor_node, use it */
    int cached_len = build_cached_dashboard_json(ms, buf, sz);
    if (cached_len > 0) return;

    /* Fallback: build from local bus + discovery */
    MessageBus* bus = ms->bus;
    DiscoveryManager* dm = ms->discovery;

    uint64_t pub, del, drop;
    message_bus_get_stats(bus, &pub, &del, &drop);

    /* Local topic stats */
    TopicStats tstats[32];
    int nt = message_bus_get_all_topic_stats(bus, tstats, 32);

    /* Compute avg latency from local topics */
    uint64_t lat_total = 0, lat_count = 0;
    for (int i = 0; i < nt; i++) {
        if (tstats[i].deliver_count > 0) {
            lat_total += tstats[i].total_latency_us;
            lat_count += tstats[i].deliver_count;
        }
    }

    /* Safe append helper */
#define SSE_APPEND(fmt, ...) \
    do { \
        if (off >= 0 && (size_t)off < sz) \
            off += snprintf(buf + off, sz - (size_t)off, fmt, ##__VA_ARGS__); \
    } while (0)

    int off = snprintf(buf, sz,
        "{\"self\":\"flowmond\",");

    /* Nodes + endpoints from discovery topology */
    const TopologyGraph* g = dm ? discovery_get_topology(dm) : NULL;
    if (g && g->node_count > 0) {
        SSE_APPEND("\"nodes\":[");
        for (uint32_t ni = 0; ni < g->node_count; ni++) {
            const NodeInfo* n = &g->nodes[ni];
            SSE_APPEND(
                "%s{\"name\":\"%s\",\"pid\":%u,\"alive\":%s,\"topics\":[",
                ni > 0 ? "," : "", n->name, n->pid,
                n->alive ? "true" : "false");
            for (uint32_t tj = 0; tj < n->topic_count; tj++) {
                SSE_APPEND(
                    "%s{\"topic\":\"%s\",\"freq\":%.1f}",
                    tj > 0 ? "," : "", n->topics[tj].topic,
                    n->topics[tj].frequency_hz);
            }
            SSE_APPEND("]}");
        }
        SSE_APPEND("],");

        /* endpoints */
        SSE_APPEND("\"endpoints\":[");
        int ep_count = 0;
        for (uint32_t ni = 0; ni < g->node_count; ni++) {
            const NodeInfo* n = &g->nodes[ni];
            for (uint32_t tj = 0; tj < n->topic_count; tj++) {
                const char* role = (n->topics[tj].capabilities & 0x01) ? "pub" : "sub";
                SSE_APPEND(
                    "%s{\"node\":\"%s\",\"topic\":\"%s\","
                    "\"role\":\"%s\",\"type_id\":\"0x00000000\",\"freq\":%.1f}",
                    ep_count > 0 ? "," : "",
                    n->name, n->topics[tj].topic, role,
                    n->topics[tj].frequency_hz);
                ep_count++;
            }
        }
        SSE_APPEND("],");
    } else {
        SSE_APPEND("\"nodes\":[],\"endpoints\":[],");
    }

    /* metrics wrapper */
    SSE_APPEND("\"metrics\":{");

    /* bus */
    SSE_APPEND("\"bus\":{\"published\":%lu,\"delivered\":%lu,\"dropped\":%lu},",
        (unsigned long)pub, (unsigned long)del, (unsigned long)drop);

    /* transport / scheduler / latency (static/placeholder) */
    SSE_APPEND("\"transport\":{\"local_pub\":%lu,\"remote_pub\":0},",
        (unsigned long)pub);
    uint64_t avg_latency = lat_count > 0 ? lat_total / lat_count : 0;
    SSE_APPEND("\"scheduler\":{\"tasks\":0,\"mode\":\"CHOREO\"},"
        "\"latency\":{\"avg_us\":%lu,\"p50_us\":0,\"p99_us\":0},",
        (unsigned long)avg_latency);

    /* topics (local + remote) */
    SSE_APPEND("\"topics\":[");
    int total = 0;
    for (int i = 0; i < nt; i++) {
        uint64_t lat = tstats[i].deliver_count > 0
            ? tstats[i].total_latency_us / tstats[i].deliver_count : 0;
        SSE_APPEND(
            "%s{\"topic\":\"%s\",\"source\":\"local\","
            "\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
            "\"lat_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
            total > 0 ? "," : "", tstats[i].topic,
            (unsigned long)tstats[i].publish_count,
            (unsigned long)tstats[i].deliver_count,
            (unsigned long)tstats[i].drop_count,
            (unsigned long)lat,
            tstats[i].frequency_hz,
            tstats[i].subscriber_count);
        total++;
    }

    /* Remote topic stats (from other processes via IPC bridge) */
    pthread_mutex_lock(&ms->remote_mutex);
    for (int r = 0; r < ms->remote_count; r++) {
        const RemoteSource* src = &ms->remote[r];
        if (!src->valid) continue;
        for (uint32_t i = 0; i < src->pkt.topic_count; i++) {
            const RemoteTopicStat* t = &src->pkt.topics[i];
            uint64_t lat = t->deliver_count > 0
                ? t->total_latency_us / t->deliver_count : 0;
            SSE_APPEND(
                "%s{\"topic\":\"%s\",\"source\":\"%s\","
                "\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
                "\"lat_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
                total > 0 ? "," : "", t->topic, src->source_name,
                (unsigned long)t->publish_count,
                (unsigned long)t->deliver_count,
                (unsigned long)t->drop_count,
                (unsigned long)lat,
                t->frequency_hz,
                t->subscriber_count);
            total++;
        }
    }
    pthread_mutex_unlock(&ms->remote_mutex);

    /* Close topics, add placeholder metrics, close metrics + top-level */
    SSE_APPEND("],"
        "\"sysmon\":{},"
        "\"vehicle\":{},"
        "\"scene\":{}}}");
#undef SSE_APPEND
}

/* ── HTTP 响应 ──────────────────────────────────────────── */

static void send_response(int fd, const char* status, const char* content_type,
                          const char* body, int body_len) {
    char header[512];
    int hl = snprintf(header, sizeof(header),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n",
        status, content_type, body_len);
    write(fd, header, (size_t)hl);
    if (body && body_len > 0) write(fd, body, (size_t)body_len);
}

/* ── SSE 流 ──────────────────────────────────────────────── */

static void handle_sse(int fd, MonitorServer* ms) {
    const char* sse_header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "\r\n";
    write(fd, sse_header, strlen(sse_header));

    char buf[MONITOR_HTTP_BUF_SIZE];
    for (int tick = 0; tick < 3000 && ms->running; tick++) {  /* ~5 min max */
        build_sse_json(ms, buf, sizeof(buf));
        char frame[MONITOR_HTTP_BUF_SIZE + 32];
        int fl = snprintf(frame, sizeof(frame), "data: %s\n\n", buf);
        if (write(fd, frame, (size_t)fl) <= 0) break;
        usleep(500000);  /* 500ms push interval */
    }
}

/* ── 处理单个连接 ────────────────────────────────────────── */

static void handle_client(int fd, MonitorServer* ms) {
    char req[4096];
    ssize_t n = read(fd, req, sizeof(req) - 1);
    if (n <= 0) { close(fd); return; }
    req[n] = '\0';

    /* CORS preflight */
    if (strncmp(req, "OPTIONS", 7) == 0) {
        const char* cors = "HTTP/1.1 204\r\nAccess-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, OPTIONS\r\n\r\n";
        write(fd, cors, strlen(cors));
        close(fd);
        return;
    }

    /* Route: /api/stream → SSE */
    if (strstr(req, "GET /api/stream")) {
        handle_sse(fd, ms);
        close(fd);
        return;
    }

    /* Route: /api/topology → JSON (prefer cached dashboard JSON) */
    if (strstr(req, "GET /api/topology")) {
        char buf[MONITOR_HTTP_BUF_SIZE];
        int cached_len = build_cached_dashboard_json(ms, buf, sizeof(buf));
        if (cached_len > 0) {
            send_response(fd, "200 OK", "application/json", buf, cached_len);
        } else {
            build_sse_json(ms, buf, sizeof(buf));
            send_response(fd, "200 OK", "application/json", buf, (int)strlen(buf));
        }
        close(fd);
        return;
    }

    /* Route: /api/topics → per-topic stats (local + remote) */
    if (strstr(req, "GET /api/topics")) {
        char buf[MONITOR_HTTP_BUF_SIZE];
        int cached_len = build_cached_dashboard_json(ms, buf, sizeof(buf));
        if (cached_len > 0) {
            send_response(fd, "200 OK", "application/json", buf, cached_len);
        } else {
            build_sse_json(ms, buf, sizeof(buf));
            send_response(fd, "200 OK", "application/json", buf, (int)strlen(buf));
        }
        close(fd);
        return;
    }

    /* Route: / → flowboard.html (from disk) or embedded fallback */
    if (strstr(req, "GET / ") || strstr(req, "GET /index.html")) {
        char* html = NULL;
        size_t html_len = 0;
        /* Try reading from disk first */
        if (ms->html_path[0]) {
            html = read_file(ms->html_path, &html_len);
        }
        if (html) {
            send_response(fd, "200 OK", "text/html; charset=utf-8", html, (int)html_len);
            free(html);
        } else {
            /* Fallback: minimal embedded dashboard */
            const char* fallback =
                "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>FlowBoard</title>"
                "<style>body{background:#0d1117;color:#c9d1d9;font:13px system-ui;padding:16px}"
                "h1{color:#58a6ff}a{color:#3fb950}.info{color:#8b949e;margin-top:8px}</style></head><body>"
                "<h1>⚠ FlowBoard HTML not found</h1>"
                "<p class=info>Set <code>--html-path</code> or check the tools/ directory.</p>"
                "<p><a href='/api/topology'>/api/topology</a> — JSON data</p>"
                "<p><a href='/api/stream'>/api/stream</a> — SSE live feed</p>"
                "</body></html>";
            send_response(fd, "200 OK", "text/html; charset=utf-8",
                          fallback, (int)strlen(fallback));
        }
        close(fd);
        return;
    }

    /* Route: /tools/<file> → static asset served from the directory that
     * contains flowboard.html (i.e. the repo's tools/ folder). The dashboard
     * loads three.min.js / d3.v7.min.js from here so the 3D view and topology
     * graph work offline without relying on external CDNs. */
    if (strstr(req, "GET /tools/") && ms->html_path[0]) {
        /* Extract the requested path from the request line. */
        const char* tools_hit = strstr(req, "GET /tools/");
        const char* p = tools_hit + 4;  /* points at "/tools/..." */
        char reqpath[256];
        int j = 0;
        while (*p && *p != ' ' && *p != '?' && *p != '\r' && *p != '\n' &&
               j < (int)sizeof(reqpath) - 1) {
            reqpath[j++] = *p++;
        }
        reqpath[j] = '\0';

        /* Reject path traversal — block ".." and backslash.
         * Note: the basename extraction below (strrchr) already neutralises
         * embedded '/' by only using the part after the last '/'. */
        if (strstr(reqpath, "..") || strchr(reqpath, '\\')) {
            const char* forbidden = "{\"error\":\"forbidden\"}";
            send_response(fd, "403 Forbidden", "application/json",
                          forbidden, (int)strlen(forbidden));
            close(fd);
            return;
        }

        /* Basename after the last '/'. */
        const char* base = strrchr(reqpath, '/');
        base = base ? base + 1 : reqpath;

        /* Directory of html_path (the tools/ folder). */
        char dir[512];
        snprintf(dir, sizeof(dir), "%s", ms->html_path);
        char* slash = strrchr(dir, '/');
        if (slash) *slash = '\0'; else dir[0] = '\0';

        char filepath[768];
        snprintf(filepath, sizeof(filepath), "%s/%s", dir, base);

        size_t flen = 0;
        char* fbuf = read_file(filepath, &flen);
        if (fbuf) {
            /* Content type by extension. */
            const char* ctype = "application/octet-stream";
            const char* dot = strrchr(base, '.');
            if (dot) {
                if (strcmp(dot, ".js") == 0)        ctype = "application/javascript; charset=utf-8";
                else if (strcmp(dot, ".css") == 0)  ctype = "text/css; charset=utf-8";
                else if (strcmp(dot, ".html") == 0) ctype = "text/html; charset=utf-8";
                else if (strcmp(dot, ".json") == 0) ctype = "application/json";
                else if (strcmp(dot, ".svg") == 0)  ctype = "image/svg+xml";
                else if (strcmp(dot, ".wasm") == 0) ctype = "application/wasm";
                else if (strcmp(dot, ".png") == 0)  ctype = "image/png";
            }
            send_response(fd, "200 OK", ctype, fbuf, (int)flen);
            free(fbuf);
        } else {
            const char* notfound = "{\"error\":\"not found\"}";
            send_response(fd, "404 Not Found", "application/json",
                          notfound, (int)strlen(notfound));
        }
        close(fd);
        return;
    }

    /* 404 */
    const char* notfound = "{\"error\":\"not found\"}";
    send_response(fd, "404 Not Found", "application/json", notfound, (int)strlen(notfound));
    close(fd);
}

/* ── Server thread ───────────────────────────────────────── */

/* Per-connection context passed to the client handler thread. */
typedef struct {
    int            fd;
    MonitorServer* ms;
} ClientCtx;

static void* client_thread_fn(void* arg) {
    ClientCtx* ctx = (ClientCtx*)arg;
    MonitorServer* ms = ctx->ms;
    handle_client(ctx->fd, ms);
    free(ctx);
    pthread_mutex_lock(&ms->client_mutex);
    ms->active_clients--;
    pthread_mutex_unlock(&ms->client_mutex);
    return NULL;
}

static void* server_thread_fn(void* arg) {
    MonitorServer* ms = (MonitorServer*)arg;

    ms->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (ms->listen_fd < 0) return NULL;

    int reuse = 1;
    setsockopt(ms->listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = { .sin_family = AF_INET,
                                .sin_addr.s_addr = htonl(INADDR_ANY),
                                .sin_port = htons((uint16_t)ms->port) };
    if (bind(ms->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(ms->listen_fd);
        return NULL;
    }
    listen(ms->listen_fd, MONITOR_MAX_CLIENTS);

    printf("[monitor_server] Listening on http://0.0.0.0:%d\n", ms->port);
    printf("[monitor_server] Endpoints: /  /api/topology  /api/topics  /api/stream\n");

    while (ms->running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(ms->listen_fd, &fds);
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };

        if (select(ms->listen_fd + 1, &fds, NULL, NULL, &tv) > 0) {
            int client = accept(ms->listen_fd, NULL, NULL);
            if (client < 0) continue;

            /* Handle each connection in its own detached thread so that a
             * long-lived SSE stream (/api/stream) cannot block the accept
             * loop — otherwise a single open dashboard tab would monopolise
             * the server and every subsequent request (page reload, new tab,
             * /api/topology, static assets) would hang. */
            ClientCtx* ctx = (ClientCtx*)malloc(sizeof(ClientCtx));
            if (!ctx) { close(client); continue; }
            ctx->fd = client;
            ctx->ms = ms;

            pthread_mutex_lock(&ms->client_mutex);
            ms->active_clients++;
            pthread_mutex_unlock(&ms->client_mutex);

            pthread_t th;
            if (pthread_create(&th, NULL, client_thread_fn, ctx) != 0) {
                /* Thread creation failed — fall back to inline handling so the
                 * request is still served. Note: this reverts to the old
                 * blocking behaviour for this connection, so an SSE stream here
                 * could still stall the accept loop. Log it so operators can
                 * spot resource exhaustion. */
                fprintf(stderr,
                        "[monitor_server] WARN: pthread_create failed, handling "
                        "connection inline (may block)\n");
                free(ctx);
                pthread_mutex_lock(&ms->client_mutex);
                ms->active_clients--;
                pthread_mutex_unlock(&ms->client_mutex);
                handle_client(client, ms);
            } else {
                pthread_detach(th);
            }
        }
    }
    close(ms->listen_fd);
    return NULL;
}

/* ══════════════════════════════════════════════════════════ */
/* Public API                                                  */
/* ══════════════════════════════════════════════════════════ */

MonitorServer* monitor_server_create(MessageBus* bus, DiscoveryManager* discovery,
                                     int port, const char* html_path) {
    MonitorServer* ms = (MonitorServer*)calloc(1, sizeof(MonitorServer));
    ms->bus       = bus;
    ms->discovery = discovery;
    ms->port      = port > 0 ? port : 8800;
    if (html_path && html_path[0])
        snprintf(ms->html_path, sizeof(ms->html_path), "%s", html_path);
    pthread_mutex_init(&ms->remote_mutex, NULL);
    pthread_mutex_init(&ms->cached_mutex, NULL);
    pthread_mutex_init(&ms->client_mutex, NULL);
    return ms;
}

void monitor_server_start(MonitorServer* ms) {
    if (!ms || ms->running) return;
    ms->running = true;
    pthread_create(&ms->server_thread, NULL, server_thread_fn, ms);
}

void monitor_server_stop(MonitorServer* ms) {
    if (!ms || !ms->running) return;
    ms->running = false;
    pthread_join(ms->server_thread, NULL);

    /* Wait for in-flight per-connection handler threads (SSE streams check
     * ms->running every ~500ms) to finish so they don't touch a freed struct. */
    #define MONITOR_SHUTDOWN_WAIT_ITERS 200  /* 200 * 10ms = ~2s max */
    for (int i = 0; i < MONITOR_SHUTDOWN_WAIT_ITERS; i++) {
        pthread_mutex_lock(&ms->client_mutex);
        int n = ms->active_clients;
        pthread_mutex_unlock(&ms->client_mutex);
        if (n <= 0) break;
        usleep(10000);
    }
    #undef MONITOR_SHUTDOWN_WAIT_ITERS
    printf("[monitor_server] Stopped\n");
}

void monitor_server_destroy(MonitorServer* ms) {
    if (!ms) return;
    if (ms->running) monitor_server_stop(ms);
    pthread_mutex_destroy(&ms->cached_mutex);
    if (ms->cached_json) { free(ms->cached_json); ms->cached_json = NULL; }
    pthread_mutex_destroy(&ms->remote_mutex);
    pthread_mutex_destroy(&ms->client_mutex);
    free(ms);
}

void monitor_server_inject_remote_stats(MonitorServer* ms, const StatsPacket* pkt) {
    if (!ms || !pkt) return;

    pthread_mutex_lock(&ms->remote_mutex);

    /* Find existing slot for this source or allocate a new one */
    RemoteSource* slot = NULL;
    for (int i = 0; i < ms->remote_count; i++) {
        if (strcmp(ms->remote[i].source_name, pkt->source_name) == 0) {
            slot = &ms->remote[i];
            break;
        }
    }
    if (!slot && ms->remote_count < MONITOR_MAX_REMOTE_SRCS) {
        slot = &ms->remote[ms->remote_count++];
    }

    if (slot) {
        snprintf(slot->source_name, sizeof(slot->source_name),
                 "%s", pkt->source_name);
        slot->pkt   = *pkt;
        slot->valid = true;
    }

    pthread_mutex_unlock(&ms->remote_mutex);
}

void monitor_server_inject_dashboard_json(MonitorServer* ms,
                                          const char* json, size_t len) {
    if (!ms || !json || len == 0) return;

    pthread_mutex_lock(&ms->cached_mutex);

    /* Free old cache */
    if (ms->cached_json) free(ms->cached_json);

    /* Copy the JSON string */
    ms->cached_json = (char*)malloc(len + 1);
    if (ms->cached_json) {
        memcpy(ms->cached_json, json, len);
        ms->cached_json[len] = '\0';
        ms->cached_json_len = len;

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ms->cached_json_time_us = (uint64_t)ts.tv_sec * 1000000
                                  + (uint64_t)ts.tv_nsec / 1000;
    }

    pthread_mutex_unlock(&ms->cached_mutex);
}
