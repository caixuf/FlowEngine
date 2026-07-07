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
    /* Remote stats injected via IPC bridge */
    RemoteSource      remote[MONITOR_MAX_REMOTE_SRCS];
    int               remote_count;
    pthread_mutex_t   remote_mutex;
} MonitorServer;

/* ── SSE 数据生成 ────────────────────────────────────────── */

static void build_sse_json(MonitorServer* ms, char* buf, size_t sz) {
    MessageBus* bus = ms->bus;
    DiscoveryManager* dm = ms->discovery;

    uint64_t pub, del, drop;
    message_bus_get_stats(bus, &pub, &del, &drop);

    /* Topology */
    char* topo = dm ? discovery_export_json(dm) : strdup("{}");

    /* Local topic stats */
    TopicStats tstats[32];
    int nt = message_bus_get_all_topic_stats(bus, tstats, 32);

    /* Safe append helper: never write past buf+sz */
#define SSE_APPEND(fmt, ...) \
    do { \
        if (off >= 0 && (size_t)off < sz) \
            off += snprintf(buf + off, sz - (size_t)off, fmt, ##__VA_ARGS__); \
    } while (0)

    int off = snprintf(buf, sz,
        "{\"bus\":{\"published\":%lu,\"delivered\":%lu,\"dropped\":%lu},"
        "\"topology\":%s,"
        "\"topics\":[",
        (unsigned long)pub, (unsigned long)del, (unsigned long)drop,
        topo);
    free(topo);

    int total = 0;
    for (int i = 0; i < nt; i++) {
        uint64_t avg_lat = tstats[i].deliver_count > 0
            ? tstats[i].total_latency_us / tstats[i].deliver_count : 0;
        SSE_APPEND(
            "%s{\"topic\":\"%s\",\"source\":\"local\","
            "\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
            "\"deadline_violations\":%lu,"
            "\"lat_us\":%lu,\"p50_us\":%lu,\"p99_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
            total > 0 ? "," : "", tstats[i].topic,
            (unsigned long)tstats[i].publish_count,
            (unsigned long)tstats[i].deliver_count,
            (unsigned long)tstats[i].drop_count,
            (unsigned long)tstats[i].deadline_violations,
            (unsigned long)avg_lat,
            (unsigned long)tstats[i].p50_latency_us,
            (unsigned long)tstats[i].p99_latency_us,
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
            uint64_t avg_lat = t->deliver_count > 0
                ? t->total_latency_us / t->deliver_count : 0;
            SSE_APPEND(
                "%s{\"topic\":\"%s\",\"source\":\"%s\","
                "\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
                "\"deadline_violations\":0,"
                "\"lat_us\":%lu,\"freq\":%.1f,\"subs\":%u}",
                total > 0 ? "," : "", t->topic, src->source_name,
                (unsigned long)t->publish_count,
                (unsigned long)t->deliver_count,
                (unsigned long)t->drop_count,
                (unsigned long)avg_lat, t->frequency_hz,
                t->subscriber_count);
            total++;
        }
    }
    pthread_mutex_unlock(&ms->remote_mutex);

    SSE_APPEND("]}");
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

    /* Route: /api/topology → JSON */
    if (strstr(req, "GET /api/topology")) {
        char buf[MONITOR_HTTP_BUF_SIZE];
        build_sse_json(ms, buf, sizeof(buf));
        send_response(fd, "200 OK", "application/json", buf, (int)strlen(buf));
        close(fd);
        return;
    }

    /* Route: /api/topics → per-topic stats (local + remote) */
    if (strstr(req, "GET /api/topics")) {
        char buf[MONITOR_HTTP_BUF_SIZE];
        build_sse_json(ms, buf, sizeof(buf));
        /* build_sse_json includes topics already; return full payload */
        send_response(fd, "200 OK", "application/json", buf, (int)strlen(buf));
        close(fd);
        return;
    }

    /* Route: / → dashboard HTML (embedded) */
    if (strstr(req, "GET / ") || strstr(req, "GET /index.html")) {
        /* Serve a minimal live dashboard */
        const char* html =
            "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>FlowBoard Live</title>"
            "<style>*{margin:0;padding:0;box-sizing:border-box}body{background:#0d1117;color:#c9d1d9;font:13px system-ui;padding:16px}"
            "h1{color:#58a6ff;margin-bottom:12px}.card{background:#161b22;border:1px solid#21262d;border-radius:8px;padding:12px;margin-bottom:8px}"
            "h2{font-size:12px;color:#8b949e;margin-bottom:4px}.stat{display:flex;justify-content:space-between;font-size:11px;padding:2px 0}"
            ".v{color:#58a6ff;font-weight:600}.topic-table{width:100%;font-size:11px;border-collapse:collapse}"
            ".topic-table th,.topic-table td{padding:4px 8px;text-align:right;border-bottom:1px solid#161b22}"
            ".topic-table th{color:#8b949e}.topic-table td:first-child{text-align:left;color:#58a6ff}"
            ".green{color:#3fb950}.red{color:#f85149}.yellow{color:#d29922}"
            ".live-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#3fb950;animation:pulse 2s infinite;margin-right:4px}"
            "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}"
            "</style></head><body>"
            "<h1><span class='live-dot'></span>FlowBoard Live</h1>"
            "<div class='card'><h2>Bus Stats</h2><div id='bus'></div></div>"
            "<div class='card'><h2>Topic Monitor</h2><table class='topic-table'>"
            "<thead><tr><th>Topic</th><th>Pub</th><th>Del</th><th>Drop</th><th>Lat</th><th>Freq</th><th>Subs</th></tr></thead>"
            "<tbody id='topics'></tbody></table></div>"
            "<div style='font-size:10px;color:#484f58;margin-top:8px' id='status'>Connecting...</div>"
            "<script>"
            "var src=new EventSource('/api/stream');"
            "src.onmessage=function(e){"
            "var d=JSON.parse(e.data);"
            "document.getElementById('bus').innerHTML="
            "'<div class=stat><span>Published</span><span class=v>'+d.bus.published+'</span></div>'"
            "+'<div class=stat><span>Delivered</span><span class=v>'+d.bus.delivered+'</span></div>'"
            "+'<div class=stat><span>Dropped</span><span class=v style=color:'+(d.bus.dropped>0?'#f85149':'#3fb950')+'>'+d.bus.dropped+'</span></div>';"
            "var rows='';"
            "(d.topics||[]).forEach(function(t){"
            "rows+='<tr><td>'+t.topic+'</td><td>'+t.pub+'</td><td>'+t.del+'</td>'"
            "+'<td class='+(t.drop>0?'red':'green')+'>'+t.drop+'</td>'"
            "+'<td class=yellow>'+t.lat_us+'us</td><td>'+t.freq.toFixed(1)+'Hz</td><td>'+t.subs+'</td></tr>';"
            "});"
            "document.getElementById('topics').innerHTML=rows||'<tr><td colspan=7>Waiting for data...</td></tr>';"
            "document.getElementById('status').textContent='Live | '+new Date().toLocaleTimeString();"
            "};"
            "src.onerror=function(){document.getElementById('status').innerHTML='<span class=red>Disconnected — retrying...</span>';};"
            "</script></body></html>";
        send_response(fd, "200 OK", "text/html; charset=utf-8", html, (int)strlen(html));
        close(fd);
        return;
    }

    /* 404 */
    const char* notfound = "{\"error\":\"not found\"}";
    send_response(fd, "404 Not Found", "application/json", notfound, (int)strlen(notfound));
    close(fd);
}

/* ── Server thread ───────────────────────────────────────── */

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
            if (client >= 0) handle_client(client, ms);
        }
    }
    close(ms->listen_fd);
    return NULL;
}

/* ══════════════════════════════════════════════════════════ */
/* Public API                                                  */
/* ══════════════════════════════════════════════════════════ */

MonitorServer* monitor_server_create(MessageBus* bus, DiscoveryManager* discovery,
                                     int port) {
    MonitorServer* ms = (MonitorServer*)calloc(1, sizeof(MonitorServer));
    ms->bus       = bus;
    ms->discovery = discovery;
    ms->port      = port > 0 ? port : 8800;
    pthread_mutex_init(&ms->remote_mutex, NULL);
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
    printf("[monitor_server] Stopped\n");
}

void monitor_server_destroy(MonitorServer* ms) {
    if (!ms) return;
    if (ms->running) monitor_server_stop(ms);
    pthread_mutex_destroy(&ms->remote_mutex);
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
