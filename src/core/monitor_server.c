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
#include "clock_service.h"
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
#include <errno.h>
#include <signal.h>
#include <sys/wait.h>

#define MONITOR_MAX_CLIENTS       8
#define MONITOR_HTTP_BUF_SIZE     65536
#define MONITOR_MAX_REMOTE_SRCS   8
#define DASHBOARD_CACHE_STALE_SEC 3   /* seconds before cached JSON is flagged stale */
#define DASHBOARD_CACHE_DROP_SEC  30  /* seconds before cached JSON is dropped entirely */

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
    char              bind_addr[64];   /**< Listen address (default 127.0.0.1) */
    int               listen_fd;
    volatile bool     running;
    pthread_t         server_thread;
    char              html_path[512];  /**< Path to flowboard/index.html (empty = embedded) */
    /* Remote stats injected via IPC bridge */
    RemoteSource      remote[MONITOR_MAX_REMOTE_SRCS];
    int               remote_count;
    pthread_mutex_t   remote_mutex;
    /* Cached full dashboard JSON from monitor_node via IPC */
    char*             cached_json;
    size_t            cached_json_len;
    uint64_t          cached_json_time_us;
    uint64_t          cached_json_version;  /**< bumped on every fresh dashboard payload */
    pthread_mutex_t   cached_mutex;
    pthread_cond_t    cached_cond;           /**< signaled when cached_json_version changes */
    /* Count of in-flight per-connection handler threads (e.g. SSE streams).
     * Used so shutdown can wait for them to drain before the struct is freed. */
    volatile int      active_clients;
    pthread_mutex_t   client_mutex;
    /* Timestamps for IPC data freshness (used by reconnect logic in flowmond) */
    uint64_t          last_dashboard_data_us;
    uint64_t          last_stats_data_us;
    pthread_mutex_t   freshness_mutex;
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

/* ── SSE 数据生成 (flowboard 兼容格式) ────────────────── */

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
        uint64_t now_us = clock_now_us();
        uint64_t age_us = (now_us > ms->cached_json_time_us)
                          ? now_us - ms->cached_json_time_us : 0;
        double age_sec = (double)age_us / 1000000.0;
        bool stale = (age_sec > (double)DASHBOARD_CACHE_STALE_SEC);

        /* Only drop the cache entirely once it is *very* old (pipeline truly
         * stopped). A brief IPC hiccup (a few late dashboard packets) used to
         * fall straight through to the local-bus data, which — in a standalone
         * flowmond process — is essentially empty. The node list would collapse
         * and re-expand on the next fresh packet, producing the visible
         * "flicker" in multi-process mode. Instead we keep serving the last
         * known-good frame, flagged stale, so the topology stays stable and the
         * front-end shows a "● stale" badge rather than blanking out. */
        if (age_sec > (double)DASHBOARD_CACHE_DROP_SEC) {
            pthread_mutex_unlock(&ms->cached_mutex);
            return 0;
        }

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

/**
 * Write a JSON-safe copy of src to dst (max dst_sz bytes).
 * Non-printable / control characters are replaced with '?'.
 */
static int json_safe_str(char* dst, size_t dst_sz, const char* src) {
    if (!dst || dst_sz == 0) return 0;
    size_t j = 0;
    for (const char* p = src; p && *p && j + 1 < dst_sz; p++) {
        unsigned char c = (unsigned char)*p;
        dst[j++] = (c >= 0x20 && c != '"' && c != '\\') ? (char)c : '?';
    }
    dst[j] = '\0';
    return (int)j;
}

static void build_sse_json(MonitorServer* ms, char* buf, size_t sz) {
    /* Only serve cached dashboard JSON when it's fresh.
     * Stale cache means the pipeline stopped → fall through to local bus data. */
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

    char safe[128], safe2[128];
    int off = snprintf(buf, sz,
        "{\"self\":\"flowmond\",");

    /* Nodes + endpoints from discovery topology */
    const TopologyGraph* g = dm ? discovery_get_topology(dm) : NULL;
    if (g && g->node_count > 0) {
        SSE_APPEND("\"nodes\":[");
        for (uint32_t ni = 0; ni < g->node_count; ni++) {
            const NodeInfo* n = &g->nodes[ni];
            json_safe_str(safe, sizeof(safe), n->name);
            SSE_APPEND(
                "%s{\"name\":\"%s\",\"pid\":%u,\"alive\":%s,\"topics\":[",
                ni > 0 ? "," : "", safe, n->pid,
                n->alive ? "true" : "false");
            for (uint32_t tj = 0; tj < n->topic_count; tj++) {
                json_safe_str(safe, sizeof(safe), n->topics[tj].topic);
                SSE_APPEND(
                    "%s{\"topic\":\"%s\",\"freq\":%.1f}",
                    tj > 0 ? "," : "", safe,
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
                json_safe_str(safe, sizeof(safe), n->name);
                json_safe_str(safe2, sizeof(safe2), n->topics[tj].topic);
                SSE_APPEND(
                    "%s{\"node\":\"%s\",\"topic\":\"%s\","
                    "\"role\":\"%s\",\"type_id\":\"0x00000000\",\"freq\":%.1f}",
                    ep_count > 0 ? "," : "",
                    safe, safe2, role,
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
        json_safe_str(safe, sizeof(safe), tstats[i].topic);
        SSE_APPEND(
            "%s{\"topic\":\"%s\",\"source\":\"local\","
            "\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
            "\"lat_us\":%lu,\"p50_us\":%lu,\"p99_us\":%lu,"
            "\"freq\":%.1f,\"subs\":%u,"
            "\"deadline_violations\":%lu,"
            "\"qos_depth\":%u,\"qos_reliability\":\"%s\""
            "}",
            total > 0 ? "," : "", safe,
            (unsigned long)tstats[i].publish_count,
            (unsigned long)tstats[i].deliver_count,
            (unsigned long)tstats[i].drop_count,
            (unsigned long)lat,
            (unsigned long)tstats[i].p50_latency_us,
            (unsigned long)tstats[i].p99_latency_us,
            tstats[i].frequency_hz,
            tstats[i].subscriber_count,
            (unsigned long)tstats[i].deadline_violations,
            tstats[i].qos.depth,
            tstats[i].qos.reliability == QOS_RELIABLE ? "reliable" : "best_effort");
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
            json_safe_str(safe, sizeof(safe), t->topic);
            json_safe_str(safe2, sizeof(safe2), src->source_name);
            SSE_APPEND(
                "%s{\"topic\":\"%s\",\"source\":\"%s\","
                "\"pub\":%lu,\"del\":%lu,\"drop\":%lu,"
                "\"lat_us\":%lu,\"p50_us\":%lu,\"p99_us\":%lu,"
                "\"freq\":%.1f,\"subs\":%u,"
                "\"deadline_violations\":%lu}",
                total > 0 ? "," : "", safe, safe2,
                (unsigned long)t->publish_count,
                (unsigned long)t->deliver_count,
                (unsigned long)t->drop_count,
                (unsigned long)lat,
                (unsigned long)t->p50_latency_us,
                (unsigned long)t->p99_latency_us,
                t->frequency_hz,
                t->subscriber_count,
                (unsigned long)t->deadline_violations);
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
                          const char* body) {
    int body_len = body ? (int)strlen(body) : 0;
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
    ssize_t w = write(fd, header, (size_t)hl);
    (void)w;
    if (body && body_len > 0) {
        w = write(fd, body, (size_t)body_len);
        (void)w;
    }
}

/* ── SSE 流 ──────────────────────────────────────────────── */

/**
 * Safe write wrapper: handles EPIPE (client disconnected) and EAGAIN
 * (socket buffer full). Returns -1 on fatal error, 0 on success.
 * EPIPE is the normal case when a browser tab closes the SSE connection
 * — we should not SIGPIPE or log-spam on it.
 */
static int safe_write(int fd, const void* buf, size_t len) {
    const char* p = (const char*)buf;
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t w = write(fd, p, remaining);
        if (w > 0) {
            p += w;
            remaining -= (size_t)w;
        } else if (w == 0) {
            return -1;  /* EOF — client closed */
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* Socket buffer full; brief yield and retry */
                usleep(1000);
                continue;
            }
            /* EPIPE, ECONNRESET, etc. — client disconnected, normal */
            return -1;
        }
    }
    return 0;
}

/**
 * Flatten a JSON payload to a single line, in place.
 *
 * SSE frames are line-delimited: a raw '\n' inside the payload terminates the
 * "data:" line, and EventSource silently drops every following line as an
 * unknown field. The cached dashboard JSON is pretty-printed (cJSON_Print in
 * monitor_node), so without this the browser receives only the first line —
 * the 45-byte "{"source":...,"age_sec":0.0," prefix — and the dashboard/3D
 * view starves ("Waiting for data"). Raw \n/\r/\t bytes in serialized JSON
 * are always structural whitespace (cJSON escapes them inside strings), so
 * stripping them is lossless.
 */
static void sse_flatten_payload(char* s) {
    char* wr = s;
    for (char* rd = s; *rd; rd++) {
        if (*rd != '\n' && *rd != '\r' && *rd != '\t') *wr++ = *rd;
    }
    *wr = '\0';
}

static void handle_sse(int fd, MonitorServer* ms) {
    const char* sse_header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "\r\n";
    if (safe_write(fd, sse_header, strlen(sse_header)) != 0) return;

    char buf[MONITOR_HTTP_BUF_SIZE];
    /* ── 事件驱动 SSE 推送 (条件变量) ──────────────────────────
     * 旧代码用 usleep(500000) 轮询，即使 monitor_node 以 10Hz 产出数据，
     * 浏览器也只能以 2Hz 收到——这是节点界面 "一卡一卡" 的首要根因。
     *
     * 新方案：dashboard_bridge 收到新 JSON 时通过 cached_cond 唤醒 SSE 线程，
     * 数据到达后 50ms 内即可推送到浏览器。空闲时 50ms 超时用于心跳检测。 */
    uint64_t last_version  = 0;
    uint64_t last_send_us  = clock_now_us();

    while (ms->running) {
        pthread_mutex_lock(&ms->cached_mutex);
        uint64_t version = ms->cached_json_version;

        if (version != last_version) {
            /* 新数据到达：立即构建并推送 */
            pthread_mutex_unlock(&ms->cached_mutex);

            build_sse_json(ms, buf, sizeof(buf));
            sse_flatten_payload(buf);
            char frame[MONITOR_HTTP_BUF_SIZE + 32];
            int fl = snprintf(frame, sizeof(frame), "data: %s\n\n", buf);
            if (safe_write(fd, frame, (size_t)fl) != 0) break;
            last_version = version;
            last_send_us  = clock_now_us();
        } else {
            /* 无新数据：等待条件变量信号，50ms 超时用于心跳 */
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 50000000;  /* 50ms */
            if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
            int ret = pthread_cond_timedwait(&ms->cached_cond, &ms->cached_mutex, &ts);
            /* timedwait 返回时已重新获取 mutex */

            uint64_t now_us = clock_now_us();
            if (ret == ETIMEDOUT) {
                /* 50ms 空闲超时 → 检查是否需要心跳 */
                uint64_t elapsed = now_us - last_send_us;
                if (elapsed > 10000000ULL) {  /* ~10s 无数据推送 → 保活 */
                    pthread_mutex_unlock(&ms->cached_mutex);
                    if (safe_write(fd, ": keep-alive\n\n", 14) != 0) break;
                    last_send_us = now_us;
                    continue;  /* 跳过 unlock，因为已经 unlock 了 */
                }
            }
            pthread_mutex_unlock(&ms->cached_mutex);
        }
    }
}

/* ── POST 请求体读取 ────────────────────────────────────── */

static char* read_post_body(int fd, const char* headers, size_t max_size) {
    const char* content_len_header = strstr(headers, "Content-Length:");
    if (!content_len_header) return NULL;

    int content_len = atoi(content_len_header + 15);
    if (content_len <= 0 || (size_t)content_len > max_size) return NULL;

    char* body = (char*)malloc(content_len + 1);
    if (!body) return NULL;

    /* Loop to handle partial reads on sockets */
    size_t total = 0;
    while (total < (size_t)content_len) {
        ssize_t n = read(fd, body + total, (size_t)content_len - total);
        if (n <= 0) {
            free(body);
            return NULL;
        }
        total += (size_t)n;
    }
    body[content_len] = '\0';
    return body;
}

/* ── 执行 modelctl.py 子命令（训练管理的统一入口） ──────────
 *
 * 架构原则: C++ 服务器只做实时数据+静态文件; 训练管理归 modelctl.py CLI。
 * 本函数是两者间的最薄桥接层:
 *   fork → pipe stdin(JSON body) → exec modelctl.py <cmd> --json
 * 不再需要 training_bridge.py 中间层。
 */

static void exec_modelctl(int fd, const char* cmd, const char* json_body,
                          MonitorServer* ms) {
    int stdin_pipe[2];
    int stdout_pipe[2];

    if (pipe(stdin_pipe) == -1 || pipe(stdout_pipe) == -1) {
        if (stdin_pipe[0] >= 0) { close(stdin_pipe[0]); close(stdin_pipe[1]); }
        send_response(fd, "500 Internal Server Error", "application/json",
                      "{\"ok\":false,\"error\":\"pipe failed\"}");
        close(fd);
        return;
    }

    pid_t pid = fork();
    if (pid == -1) {
        close(stdin_pipe[0]);  close(stdin_pipe[1]);
        close(stdout_pipe[0]); close(stdout_pipe[1]);
        send_response(fd, "500 Internal Server Error", "application/json",
                      "{\"ok\":false,\"error\":\"fork failed\"}");
        close(fd);
        return;
    }

    if (pid == 0) {
        /* 子进程: exec modelctl.py <cmd> --json */
        close(stdin_pipe[1]);
        close(stdout_pipe[0]);
        dup2(stdin_pipe[0], STDIN_FILENO);
        dup2(stdout_pipe[1], STDOUT_FILENO);
        dup2(stdout_pipe[1], STDERR_FILENO);
        close(stdin_pipe[0]);
        close(stdout_pipe[1]);

        /* 从 html_path 推导 tools/ 目录路径 */
        char script[768];
        if (ms && ms->html_path[0]) {
            char dir[512];
            snprintf(dir, sizeof(dir), "%s", ms->html_path);
            char* slash = strrchr(dir, '/');
            if (slash) *slash = '\0';
            slash = strrchr(dir, '/');
            if (slash) *slash = '\0';
            snprintf(script, sizeof(script), "%s/modelctl.py", dir);
        } else {
            snprintf(script, sizeof(script), "tools/modelctl.py");
        }

        const char* argv[] = {"python3", script, cmd, "--json", NULL};
        execvp("python3", (char* const*)argv);
        _exit(1);
    }

    /* 父进程: 写 stdin → 读 stdout → 发 HTTP 响应 */
    close(stdin_pipe[0]);
    close(stdout_pipe[1]);

    if (json_body) {
        size_t len = strlen(json_body);
        ssize_t w = write(stdin_pipe[1], json_body, len);
        (void)w;
    }
    close(stdin_pipe[1]);

    char result[8192];
    size_t total = 0;
    ssize_t nread;
    while (total < sizeof(result) - 1 &&
           (nread = read(stdout_pipe[0], result + total,
                         sizeof(result) - 1 - total)) > 0) {
        total += (size_t)nread;
    }
    result[total] = '\0';
    close(stdout_pipe[0]);
    waitpid(pid, NULL, 0);

    if (total > 0) {
        send_response(fd, "200 OK", "application/json", result);
    } else {
        send_response(fd, "500 Internal Server Error", "application/json",
                      "{\"ok\":false,\"error\":\"no output\"}");
    }
    close(fd);
}

/* ── 处理单个连接 ────────────────────────────────────────── */

static void handle_client(int fd, MonitorServer* ms) {
    char req[4096];
    ssize_t n = read(fd, req, sizeof(req) - 1);
    if (n <= 0) { close(fd); return; }
    req[n] = '\0';

    /* Parse the HTTP request line (first line) into method + path so routing
     * matches on the real request target rather than a substring that could
     * appear anywhere in the headers/body (e.g. a Referer header). */
    char method[8] = {0};
    char path[512] = {0};
    {
        const char* sp1 = strchr(req, ' ');
        if (sp1) {
            size_t mlen = (size_t)(sp1 - req);
            if (mlen >= sizeof(method)) mlen = sizeof(method) - 1;
            memcpy(method, req, mlen);
            method[mlen] = '\0';
            const char* pstart = sp1 + 1;
            const char* sp2 = strpbrk(pstart, " \r\n");
            size_t plen = sp2 ? (size_t)(sp2 - pstart) : strlen(pstart);
            if (plen >= sizeof(path)) plen = sizeof(path) - 1;
            memcpy(path, pstart, plen);
            path[plen] = '\0';
        }
    }
    /* Strip an optional query string so exact path matching still works. */
    char* qs = strchr(path, '?');
    if (qs) *qs = '\0';

    /* Malformed request line (no method/path) → 400 Bad Request. */
    if (method[0] == '\0' || path[0] == '\0') {
        send_response(fd, "400 Bad Request", "text/plain", "bad request");
        close(fd);
        return;
    }

    /* CORS preflight */
    if (strcmp(method, "OPTIONS") == 0) {
        const char* cors = "HTTP/1.1 204\r\nAccess-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n\r\n";
        ssize_t w = write(fd, cors, strlen(cors));
        (void)w;
        close(fd);
        return;
    }

    /* Support GET and POST; reject everything else */
    if (strcmp(method, "GET") != 0 && strcmp(method, "POST") != 0) {
        send_response(fd, "405 Method Not Allowed", "text/plain", "method not allowed");
        close(fd);
        return;
    }

    /* POST: /api/training/start|promote → fork+exec modelctl.py */
    if (strcmp(method, "POST") == 0) {
        if (strcmp(path, "/api/training/start") == 0 ||
            strcmp(path, "/api/training/promote") == 0) {
            char* body = read_post_body(fd, req, 8192);
            if (body) {
                const char* cmd = (strcmp(path, "/api/training/start") == 0)
                                  ? "train-start" : "promote";
                exec_modelctl(fd, cmd, body, ms);
                free(body);
            } else {
                send_response(fd, "400 Bad Request", "application/json",
                              "{\"ok\":false,\"error\":\"failed to read body\"}");
                close(fd);
            }
            return;
        }
        send_response(fd, "404 Not Found", "text/plain", "not found");
        close(fd);
        return;
    }

    /* GET: /api/health → liveness check for dashboard/script polling */
    if (strcmp(path, "/api/health") == 0) {
        send_response(fd, "200 OK", "application/json",
                      "{\"status\":\"ok\"}");
        close(fd);
        return;
    }

    /* GET: /api/training/status → modelctl.py train-status (无 JSON body) */
    if (strcmp(path, "/api/training/status") == 0) {
        exec_modelctl(fd, "train-status", "{}", ms);
        return;
    }

    /* Route: /api/stream → SSE */
    if (strcmp(path, "/api/stream") == 0) {
        handle_sse(fd, ms);
        close(fd);
        return;
    }

    /* Route: /api/topology → JSON (prefer cached dashboard JSON) */
    if (strcmp(path, "/api/topology") == 0) {
        char buf[MONITOR_HTTP_BUF_SIZE];
        int cached_len = build_cached_dashboard_json(ms, buf, sizeof(buf));
        if (cached_len > 0) {
            send_response(fd, "200 OK", "application/json", buf);
        } else {
            build_sse_json(ms, buf, sizeof(buf));
            send_response(fd, "200 OK", "application/json", buf);
        }
        close(fd);
        return;
    }

    /* Route: /api/topics → per-topic stats (local + remote) */
    if (strcmp(path, "/api/topics") == 0) {
        char buf[MONITOR_HTTP_BUF_SIZE];
        int cached_len = build_cached_dashboard_json(ms, buf, sizeof(buf));
        if (cached_len > 0) {
            send_response(fd, "200 OK", "application/json", buf);
        } else {
            build_sse_json(ms, buf, sizeof(buf));
            send_response(fd, "200 OK", "application/json", buf);
        }
        close(fd);
        return;
    }

    /* Route: / → flowboard/index.html (from --html-path) */
    if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
        char* html = NULL;
        if (ms->html_path[0]) {
            html = read_file(ms->html_path, NULL);
        }
        if (html) {
            send_response(fd, "200 OK", "text/html; charset=utf-8", html);
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
                          fallback);
        }
        close(fd);
        return;
    }

    /* Route: /js/<file> or /css/<file> → modular frontend from flowboard/ subdir.
     * Maps /js/foo.js → tools/flowboard/js/foo.js, /css/style.css → tools/flowboard/css/style.css */
    if ((strncmp(path, "/js/", 4) == 0 || strncmp(path, "/css/", 5) == 0) && ms->html_path[0]) {
        char reqpath[512];
        snprintf(reqpath, sizeof(reqpath), "%s", path);
        if (strstr(reqpath, "..") || strchr(reqpath, '\\')) {
            const char* forbidden = "{\"error\":\"forbidden\"}";
            send_response(fd, "403 Forbidden", "application/json", forbidden);
            close(fd); return;
        }
        /* Derive parent dir from html_path, then go up once more to reach
         * tools/ (since html_path is tools/flowboard/index.html, we strip
         * "flowboard/index.html" → tools/). Then append flowboard/<rel>.
         * e.g. /js/app.js → <tools>/flowboard/js/app.js */
        char parent[512];
        snprintf(parent, sizeof(parent), "%s", ms->html_path);
        /* Strip basename (index.html) */
        char* slash = strrchr(parent, '/');
        if (slash) *slash = '\0';
        /* Strip flowboard/ */
        slash = strrchr(parent, '/');
        if (slash) *slash = '\0'; else parent[0] = '\0';
        char filepath[768];
        const char* rel = path + 1;  /* "/js/app.js" → "js/app.js" */
        snprintf(filepath, sizeof(filepath), "%s/flowboard/%s", parent, rel);
        char* fbuf = read_file(filepath, NULL);
        if (fbuf) {
            const char* ctype = "application/octet-stream";
            const char* dot = strrchr(filepath, '.');
            if (dot) {
                if (strcmp(dot, ".js") == 0)        ctype = "application/javascript; charset=utf-8";
                else if (strcmp(dot, ".css") == 0)  ctype = "text/css; charset=utf-8";
                else if (strcmp(dot, ".html") == 0) ctype = "text/html; charset=utf-8";
                else if (strcmp(dot, ".json") == 0) ctype = "application/json";
            }
            send_response(fd, "200 OK", ctype, fbuf);
            free(fbuf);
        } else {
            const char* notfound = "{\"error\":\"not found\"}";
            send_response(fd, "404 Not Found", "application/json", notfound);
        }
        close(fd);
        return;
    }

    /* Route: /tools/<file> → static asset served from the directory that
     * contains flowboard/index.html (i.e. the repo's tools/ folder). The dashboard
     * loads three.min.js / d3.v7.min.js from here so the 3D view and topology
     * graph work offline without relying on external CDNs. */
    if (strncmp(path, "/tools/", 7) == 0 && ms->html_path[0]) {
        /* Use the already-parsed request path (query string stripped). */
        char reqpath[512];
        snprintf(reqpath, sizeof(reqpath), "%s", path);

        /* Reject path traversal — block ".." and backslash.
         * Note: the basename extraction below (strrchr) already neutralises
         * embedded '/' by only using the part after the last '/'. */
        if (strstr(reqpath, "..") || strchr(reqpath, '\\')) {
            const char* forbidden = "{\"error\":\"forbidden\"}";
            send_response(fd, "403 Forbidden", "application/json", forbidden);
            close(fd);
            return;
        }

        /* Path relative to tools/ dir: skip the "/tools/" prefix (7 chars).
         * e.g. /tools/flowboard/models/sedan.gltf → flowboard/models/sedan.gltf
         *      /tools/three.min.js              → three.min.js */
        const char* rel = reqpath + 7;  /* after "/tools/" */
        if (*rel == '\0') rel = "index.html";

        /* Directory of html_path, stripped to the tools/ folder.
         * html_path may be tools/flowboard.html (legacy) or
         * tools/flowboard/index.html (modular) — strip basename,
         * then strip "flowboard" if present, so dir always ends at
         * the tools/ directory where three.min.js / d3.v7.min.js live. */
        char dir[512];
        snprintf(dir, sizeof(dir), "%s", ms->html_path);
        char* slash = strrchr(dir, '/');
        if (slash) *slash = '\0'; else dir[0] = '\0';
        slash = strrchr(dir, '/');
        if (slash && strcmp(slash + 1, "flowboard") == 0)
            *slash = '\0';

        char filepath[1024];
        snprintf(filepath, sizeof(filepath), "%s/%s", dir, rel);

        char* fbuf = read_file(filepath, NULL);
        if (fbuf) {
            /* Content type by extension. */
            const char* ctype = "application/octet-stream";
            const char* dot = strrchr(reqpath, '.');
            if (dot) {
                if (strcmp(dot, ".js") == 0)        ctype = "application/javascript; charset=utf-8";
                else if (strcmp(dot, ".css") == 0)  ctype = "text/css; charset=utf-8";
                else if (strcmp(dot, ".html") == 0) ctype = "text/html; charset=utf-8";
                else if (strcmp(dot, ".json") == 0) ctype = "application/json";
                else if (strcmp(dot, ".svg") == 0)  ctype = "image/svg+xml";
                else if (strcmp(dot, ".wasm") == 0) ctype = "application/wasm";
                else if (strcmp(dot, ".png") == 0)  ctype = "image/png";
            }
            send_response(fd, "200 OK", ctype, fbuf);
            free(fbuf);
        } else {
            const char* notfound = "{\"error\":\"not found\"}";
            send_response(fd, "404 Not Found", "application/json", notfound);
        }
        close(fd);
        return;
    }

    /* 404 */
    const char* notfound = "{\"error\":\"not found\"}";
    send_response(fd, "404 Not Found", "application/json", notfound);
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
                                .sin_port = htons((uint16_t)ms->port) };
    if (inet_pton(AF_INET, ms->bind_addr, &addr.sin_addr) != 1) {
        /* Fall back to loopback if the configured address is invalid. */
        fprintf(stderr, "[monitor_server] WARN: invalid bind address '%s', "
                        "falling back to 127.0.0.1\n", ms->bind_addr);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        snprintf(ms->bind_addr, sizeof(ms->bind_addr), "%s", "127.0.0.1");
    }
    if (bind(ms->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(ms->listen_fd);
        return NULL;
    }
    listen(ms->listen_fd, MONITOR_MAX_CLIENTS);

    printf("[monitor_server] Listening on http://%s:%d\n", ms->bind_addr, ms->port);
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
    /* Listen address: default to loopback (127.0.0.1) so the dashboard is not
     * exposed on all interfaces by default. Override with FLOWMOND_BIND_ADDR
     * (e.g. "0.0.0.0" for container/remote access). */
    {
        const char* env = getenv("FLOWMOND_BIND_ADDR");
        snprintf(ms->bind_addr, sizeof(ms->bind_addr), "%s",
                 (env && env[0]) ? env : "127.0.0.1");
    }
    if (html_path && html_path[0])
        snprintf(ms->html_path, sizeof(ms->html_path), "%s", html_path);
    pthread_mutex_init(&ms->remote_mutex, NULL);
    pthread_mutex_init(&ms->cached_mutex, NULL);
    pthread_cond_init(&ms->cached_cond, NULL);
    pthread_mutex_init(&ms->client_mutex, NULL);
    pthread_mutex_init(&ms->freshness_mutex, NULL);
    return ms;
}

void monitor_server_start(MonitorServer* ms) {
    if (!ms || ms->running) return;

    /* A browser tab closing/reloading mid-response (very common with the
     * long-lived /api/stream SSE connection, and more likely to happen the
     * more dashboard tabs are open at once) makes write() hit a socket the
     * peer has already closed. Without this, the default SIGPIPE disposition
     * kills the *entire* process on the first such write — taking down the
     * dashboard for every other connected tab too. Ignoring it here makes
     * write() return -1/EPIPE instead, which handle_sse()/handle_client()
     * already check for and handle by closing just that one connection. */
    signal(SIGPIPE, SIG_IGN);

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
    pthread_cond_destroy(&ms->cached_cond);
    if (ms->cached_json) { free(ms->cached_json); ms->cached_json = NULL; }
    pthread_mutex_destroy(&ms->remote_mutex);
    pthread_mutex_destroy(&ms->client_mutex);
    pthread_mutex_destroy(&ms->freshness_mutex);
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

    /* Update freshness timestamp for IPC reconnect detection */
    pthread_mutex_lock(&ms->freshness_mutex);
    ms->last_stats_data_us = clock_now_us();
    pthread_mutex_unlock(&ms->freshness_mutex);
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
        ms->cached_json_time_us = clock_now_us();
        ms->cached_json_version++;
        pthread_cond_signal(&ms->cached_cond);  /* wake SSE handler */
    }

    pthread_mutex_unlock(&ms->cached_mutex);

    /* Update freshness timestamp for IPC reconnect detection */
    pthread_mutex_lock(&ms->freshness_mutex);
    ms->last_dashboard_data_us = clock_now_us();
    pthread_mutex_unlock(&ms->freshness_mutex);
}

double monitor_server_dashboard_age_sec(MonitorServer* ms) {
    if (!ms) return 1e9;
    pthread_mutex_lock(&ms->freshness_mutex);
    uint64_t last = ms->last_dashboard_data_us;
    pthread_mutex_unlock(&ms->freshness_mutex);
    if (last == 0) return 1e9;  /* never received data */
    uint64_t now = clock_now_us();
    return (double)(now - last) / 1000000.0;
}

double monitor_server_stats_age_sec(MonitorServer* ms) {
    if (!ms) return 1e9;
    pthread_mutex_lock(&ms->freshness_mutex);
    uint64_t last = ms->last_stats_data_us;
    pthread_mutex_unlock(&ms->freshness_mutex);
    if (last == 0) return 1e9;  /* never received data */
    uint64_t now = clock_now_us();
    return (double)(now - last) / 1000000.0;
}
