/**
 * param_bridge.c — 跨进程参数读写通道（AF_UNIX 行协议）
 *
 * 见 param_bridge.h 的架构说明。服务端跑在 flow_launcher 里，客户端是 flowctl。
 */

#include "param_bridge.h"
#include "param_registry.h"
#include "error_codes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>

/* ── 服务端 ───────────────────────────────────────────────── */

struct ParamBridgeServer {
    int       fd;
    char      path[108];      /* sockaddr_un.sun_path 容量 */
    pthread_t thread;
    volatile int running;
};

static const char* resolve_sock_path(const char* sock_path) {
    if (sock_path && sock_path[0]) return sock_path;
    const char* env = getenv(PARAM_BRIDGE_SOCK_ENV);
    if (env && env[0]) return env;
    return PARAM_BRIDGE_DEFAULT_SOCK;
}

/* 按参数实际类型分派 set，并把结果格式化成响应里的 value。 */
static int apply_set(const ParamEntry* e, const char* val_str,
                     char* out, size_t out_size) {
    int rc;
    switch (e->type) {
        case PARAM_INT: {
            char* end = NULL;
            long long v = strtoll(val_str, &end, 10);
            if (end == val_str || (end && *end)) return ERR_INVALID_PARAM;
            rc = param_set_int(e->name, (int64_t)v);
            if (rc == ERR_OK) snprintf(out, out_size, "%lld", v);
            return rc;
        }
        case PARAM_FLOAT: {
            char* end = NULL;
            double v = strtod(val_str, &end);
            if (end == val_str || (end && *end)) return ERR_INVALID_PARAM;
            rc = param_set_float(e->name, v);
            if (rc == ERR_OK) snprintf(out, out_size, "%g", v);
            return rc;
        }
        case PARAM_BOOL: {
            bool v;
            if (!strcmp(val_str, "true") || !strcmp(val_str, "1"))       v = true;
            else if (!strcmp(val_str, "false") || !strcmp(val_str, "0")) v = false;
            else return ERR_INVALID_PARAM;
            rc = param_set_bool(e->name, v);
            if (rc == ERR_OK) snprintf(out, out_size, "%s", v ? "true" : "false");
            return rc;
        }
        case PARAM_STRING:
            rc = param_set_string(e->name, val_str);
            if (rc == ERR_OK) snprintf(out, out_size, "%s", val_str);
            return rc;
        default:
            return ERR_INVALID_PARAM;
    }
}

static void format_value(const ParamEntry* e, char* buf, size_t n) {
    switch (e->type) {
        case PARAM_INT:    snprintf(buf, n, "%lld", (long long)e->current_value.int_val); break;
        case PARAM_FLOAT:  snprintf(buf, n, "%g",   e->current_value.float_val);          break;
        case PARAM_BOOL:   snprintf(buf, n, "%s",   e->current_value.bool_val ? "true" : "false"); break;
        case PARAM_STRING: snprintf(buf, n, "%s",   e->current_value.str_val);            break;
        default:           snprintf(buf, n, "?");                                          break;
    }
}

static const char* type_name(ParamType t) {
    switch (t) {
        case PARAM_INT:    return "int";
        case PARAM_FLOAT:  return "float";
        case PARAM_BOOL:   return "bool";
        case PARAM_STRING: return "string";
        default:           return "unknown";
    }
}

static void write_all(int fd, const char* buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return;   /* 客户端已断开，丢弃剩余输出即可 */
        }
        off += (size_t)n;
    }
}

static void handle_list(int cfd) {
    ParamEntry buf[PARAM_MAX_ENTRIES];
    int n = param_list_all(buf, PARAM_MAX_ENTRIES);

    char line[PARAM_BRIDGE_MAX_LINE];
    int len = snprintf(line, sizeof(line), "OK %d\n", n);
    write_all(cfd, line, (size_t)len);

    for (int i = 0; i < n; i++) {
        char val[64], lo[64], hi[64];
        format_value(&buf[i], val, sizeof(val));
        /* min/max 只对数值类型有意义，其余给 "-" 占位保持列数固定 */
        if (buf[i].type == PARAM_INT) {
            snprintf(lo, sizeof(lo), "%lld", (long long)buf[i].min_value.int_val);
            snprintf(hi, sizeof(hi), "%lld", (long long)buf[i].max_value.int_val);
        } else if (buf[i].type == PARAM_FLOAT) {
            snprintf(lo, sizeof(lo), "%g", buf[i].min_value.float_val);
            snprintf(hi, sizeof(hi), "%g", buf[i].max_value.float_val);
        } else {
            snprintf(lo, sizeof(lo), "-");
            snprintf(hi, sizeof(hi), "-");
        }
        len = snprintf(line, sizeof(line), "%s %s %s %s %s\n",
                       buf[i].name, type_name(buf[i].type), val, lo, hi);
        write_all(cfd, line, (size_t)len);
    }
}

static void handle_request(int cfd, char* req) {
    char resp[PARAM_BRIDGE_MAX_LINE];

    if (!strcmp(req, "LIST")) { handle_list(cfd); return; }

    char verb[16] = {0}, name[PARAM_NAME_LEN] = {0}, value[128] = {0};
    int nf = sscanf(req, "%15s %63s %127[^\n]", verb, name, value);

    if (nf >= 2 && !strcmp(verb, "GET")) {
        const ParamEntry* e = param_get_entry(name);
        if (!e) {
            snprintf(resp, sizeof(resp), "ERR %d unknown param '%s'\n", ERR_NOT_FOUND, name);
        } else {
            char val[64];
            format_value(e, val, sizeof(val));
            snprintf(resp, sizeof(resp), "OK %s\n", val);
        }
        write_all(cfd, resp, strlen(resp));
        return;
    }

    if (nf >= 3 && !strcmp(verb, "SET")) {
        const ParamEntry* e = param_get_entry(name);
        if (!e) {
            snprintf(resp, sizeof(resp), "ERR %d unknown param '%s'\n", ERR_NOT_FOUND, name);
        } else {
            char applied[64] = {0};
            int rc = apply_set(e, value, applied, sizeof(applied));
            if (rc == ERR_OK) {
                snprintf(resp, sizeof(resp), "OK %s\n", applied);
            } else if (rc == ERR_INVALID_PARAM) {
                /* 越界是最常见的失败，把区间回给用户，省得他去翻源码 */
                char lo[64], hi[64];
                if (e->type == PARAM_FLOAT) {
                    snprintf(lo, sizeof(lo), "%g", e->min_value.float_val);
                    snprintf(hi, sizeof(hi), "%g", e->max_value.float_val);
                } else if (e->type == PARAM_INT) {
                    snprintf(lo, sizeof(lo), "%lld", (long long)e->min_value.int_val);
                    snprintf(hi, sizeof(hi), "%lld", (long long)e->max_value.int_val);
                } else {
                    snprintf(lo, sizeof(lo), "-"); snprintf(hi, sizeof(hi), "-");
                }
                snprintf(resp, sizeof(resp),
                         "ERR %d value '%s' rejected (range [%s, %s])\n",
                         ERR_INVALID_PARAM, value, lo, hi);
            } else {
                snprintf(resp, sizeof(resp), "ERR %d set failed\n", rc);
            }
        }
        write_all(cfd, resp, strlen(resp));
        return;
    }

    snprintf(resp, sizeof(resp),
             "ERR %d bad request (want LIST | GET <name> | SET <name> <value>)\n",
             ERR_INVALID_PARAM);
    write_all(cfd, resp, strlen(resp));
}

static void* server_loop(void* arg) {
    ParamBridgeServer* s = (ParamBridgeServer*)arg;

    while (s->running) {
        /* poll 而非裸 accept：让 running 标志能在 100ms 内被看到，
         * 否则 stop() 要等到下一个客户端连上才能退出。 */
        struct pollfd pfd = { .fd = s->fd, .events = POLLIN, .revents = 0 };
        int pr = poll(&pfd, 1, 100);
        if (pr <= 0) continue;

        int cfd = accept(s->fd, NULL, NULL);
        if (cfd < 0) continue;

        /* 单条请求、短连接。加读超时防止客户端连上不发数据把服务线程挂住。 */
        struct pollfd cp = { .fd = cfd, .events = POLLIN, .revents = 0 };
        if (poll(&cp, 1, 1000) > 0) {
            char req[PARAM_BRIDGE_MAX_LINE];
            ssize_t n = read(cfd, req, sizeof(req) - 1);
            if (n > 0) {
                req[n] = '\0';
                char* nl = strchr(req, '\n');
                if (nl) *nl = '\0';
                handle_request(cfd, req);
            }
        }
        close(cfd);
    }
    return NULL;
}

ParamBridgeServer* param_bridge_server_start(const char* sock_path) {
    const char* path = resolve_sock_path(sock_path);

    ParamBridgeServer* s = (ParamBridgeServer*)calloc(1, sizeof(*s));
    if (!s) return NULL;
    snprintf(s->path, sizeof(s->path), "%s", path);

    s->fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s->fd < 0) { free(s); return NULL; }

    /* 清掉上次进程崩溃留下的 socket 文件，否则 bind 报 EADDRINUSE */
    unlink(s->path);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", s->path);

    if (bind(s->fd, (struct sockaddr*)&addr, sizeof(addr)) < 0 ||
        listen(s->fd, 4) < 0) {
        close(s->fd); free(s); return NULL;
    }

    s->running = 1;
    if (pthread_create(&s->thread, NULL, server_loop, s) != 0) {
        close(s->fd); unlink(s->path); free(s); return NULL;
    }
    return s;
}

void param_bridge_server_stop(ParamBridgeServer* s) {
    if (!s) return;
    s->running = 0;
    pthread_join(s->thread, NULL);
    close(s->fd);
    unlink(s->path);
    free(s);
}

/* ── 客户端 ───────────────────────────────────────────────── */

int param_bridge_client_request(const char* request, char* out, size_t out_size) {
    if (!request || !out || out_size == 0) return ERR_INVALID_PARAM;
    out[0] = '\0';

    const char* path = resolve_sock_path(NULL);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return ERR_IO;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        /* 最常见的情况：launcher 没跑。区分出来，好给用户可操作的提示。 */
        snprintf(out, out_size,
                 "no running FlowEngine process at %s (start flow_launcher first)", path);
        return ERR_NOT_FOUND;
    }

    char line[PARAM_BRIDGE_MAX_LINE];
    int len = snprintf(line, sizeof(line), "%s\n", request);
    if (write(fd, line, (size_t)len) != len) { close(fd); return ERR_IO; }

    /* 读到 EOF —— LIST 的响应是多行，服务端应答完就 close */
    size_t total = 0;
    for (;;) {
        ssize_t n = read(fd, out + total, out_size - total - 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            close(fd); return ERR_IO;
        }
        if (n == 0) break;
        total += (size_t)n;
        if (total >= out_size - 1) break;
    }
    close(fd);
    out[total] = '\0';

    /* 去掉末尾换行，调用方直接 printf 即可 */
    while (total > 0 && (out[total - 1] == '\n' || out[total - 1] == '\r'))
        out[--total] = '\0';

    if (!strncmp(out, "OK", 2)) {
        /* 剥掉 "OK " 前缀，只留内容 */
        size_t skip = (out[2] == ' ') ? 3 : 2;
        memmove(out, out + skip, total - skip + 1);
        return ERR_OK;
    }
    if (!strncmp(out, "ERR", 3)) {
        int code = ERR_INVALID_PARAM;
        char msg[PARAM_BRIDGE_MAX_LINE] = {0};
        if (sscanf(out, "ERR %d %[^\n]", &code, msg) >= 2)
            snprintf(out, out_size, "%s", msg);
        return code;
    }
    return ERR_IO;
}
