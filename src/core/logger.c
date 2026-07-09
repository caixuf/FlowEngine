/**
 * logger.c — 统一日志系统实现
 *
 * 全局单例模式：进程内所有模块共享一个 Logger 实例。
 * 线程安全：pthread_mutex_t 保护并发写入。
 *
 * 输出格式:
 *   [2026-07-04 10:30:45.123] [INFO ] [module_name   ] message
 */

#include "logger.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>

/* ── 模块级别覆盖 ────────────────────────────────────────── */

#define LOG_MAX_MODULES 64

typedef struct {
    char     name[32];
    LogLevel level;
} ModuleOverride;

/* ── 全局日志器状态 ──────────────────────────────────────── */

static struct {
    FILE*            output;
    LogLevel         min_level;
    pthread_mutex_t  mutex;
    bool             initialized;

    ModuleOverride   modules[LOG_MAX_MODULES];
    int              module_count;
} g_log = { NULL, LOG_INFO, PTHREAD_MUTEX_INITIALIZER, false, {{0}}, 0 };

/* ══════════════════════════════════════════════════════════ */
/* 初始化 / 关闭                                              */
/* ══════════════════════════════════════════════════════════ */

void log_init(LogLevel min_level, const char* filename) {
    if (g_log.initialized) return;

    g_log.min_level = min_level;

    if (filename && filename[0]) {
        g_log.output = fopen(filename, "a");
        if (!g_log.output) {
            g_log.output = stderr;
        }
    } else {
        g_log.output = stderr;
    }

    g_log.initialized = true;

    LOG_INFO("logger", "initialized (level=%s, output=%s)",
             log_level_str(min_level),
             (filename && filename[0]) ? filename : "stderr");
}

void log_shutdown(void) {
    if (!g_log.initialized) return;

    LOG_INFO("logger", "shutting down");

    if (g_log.output && g_log.output != stderr) {
        fclose(g_log.output);
    }
    g_log.output     = stderr;
    g_log.initialized = false;
}

/* ══════════════════════════════════════════════════════════ */
/* 运行时配置                                                 */
/* ══════════════════════════════════════════════════════════ */

void log_set_level(LogLevel level) {
    g_log.min_level = level;
}

LogLevel log_get_level(void) {
    return g_log.min_level;
}

void log_set_module_level(const char* module, LogLevel level) {
    if (!module) return;

    for (int i = 0; i < g_log.module_count; i++) {
        if (strcmp(g_log.modules[i].name, module) == 0) {
            g_log.modules[i].level = level;
            return;
        }
    }

    if (g_log.module_count < LOG_MAX_MODULES) {
        ModuleOverride* m = &g_log.modules[g_log.module_count++];
        snprintf(m->name, sizeof(m->name), "%s", module);
        m->level = level;
    }
}

LogLevel log_get_module_level(const char* module) {
    if (!module) return g_log.min_level;
    for (int i = 0; i < g_log.module_count; i++) {
        if (strcmp(g_log.modules[i].name, module) == 0)
            return g_log.modules[i].level;
    }
    return g_log.min_level;
}

void log_set_output_file(const char* filename) {
    pthread_mutex_lock(&g_log.mutex);
    if (g_log.output && g_log.output != stderr) {
        fclose(g_log.output);
        g_log.output = NULL;
    }
    if (filename && filename[0]) {
        g_log.output = fopen(filename, "a");
    }
    if (!g_log.output) {
        g_log.output = stderr;
    }
    pthread_mutex_unlock(&g_log.mutex);
}

FILE* log_get_output(void) {
    return g_log.output ? g_log.output : stderr;
}

/* ══════════════════════════════════════════════════════════ */
/* 核心日志输出                                               */
/* ══════════════════════════════════════════════════════════ */

void log_write(const char* module, LogLevel level,
               const char* file, int line, const char* func,
               const char* fmt, ...) {
    /* Auto-init if never initialized */
    if (!g_log.initialized) {
        log_init(LOG_INFO, NULL);
    }

    /* Level check */
    LogLevel effective = log_get_module_level(module);
    if (level < effective) return;

    /* Timestamp */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_buf;
    time_t sec = tv.tv_sec;
    localtime_r(&sec, &tm_buf);

    pthread_mutex_lock(&g_log.mutex);

    FILE* out = g_log.output ? g_log.output : stderr;

    /* [2026-07-04 10:30:45.123] [INFO ] [module       ] */
    fprintf(out, "[%04d-%02d-%02d %02d:%02d:%02d.%03d] [%-5s] [%-14s] ",
            tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
            tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
            (int)(tv.tv_usec / 1000),
            log_level_str(level),
            module ? module : "?");

    /* Body */
    va_list args;
    va_start(args, fmt);
    vfprintf(out, fmt, args);
    va_end(args);

    /* Source location for DEBUG/TRACE */
    if (level <= LOG_DEBUG) {
        const char* fname = strrchr(file, '/');
        fname = fname ? fname + 1 : file;
        fprintf(out, "  [%s:%d %s()]", fname, line, func ? func : "?");
    }

    fprintf(out, "\n");
    fflush(out);

    pthread_mutex_unlock(&g_log.mutex);
}

/* ══════════════════════════════════════════════════════════ */
/* 兼容旧 API                                                 */
/* ══════════════════════════════════════════════════════════ */

void default_log_callback(LogLevel level, const char* message) {
    log_write("core", level, __FILE__, __LINE__, __func__, "%s", message);
}
