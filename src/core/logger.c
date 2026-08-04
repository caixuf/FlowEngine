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
#include <sys/stat.h>
#include <errno.h>

/* ── 模块级别覆盖 ────────────────────────────────────────── */

#define LOG_MAX_MODULES 64

typedef struct {
    char     name[32];
    LogLevel level;
} ModuleOverride;

/* ── 按模块分文件输出 ────────────────────────────────────── */

#define LOG_MAX_MODULE_FILES 32

typedef struct {
    char  name[32];
    FILE* fp;
} ModuleFile;

/* ── 全局日志器状态 ──────────────────────────────────────── */

static struct {
    FILE*            output;
    LogLevel         min_level;
    pthread_mutex_t  mutex;
    bool             initialized;

    ModuleOverride   modules[LOG_MAX_MODULES];
    int              module_count;

    /* 按模块分文件输出 */
    char             module_dir[256];
    ModuleFile       module_files[LOG_MAX_MODULE_FILES];
    int              module_file_count;
} g_log = { .output = NULL, .min_level = LOG_INFO,
            .mutex = PTHREAD_MUTEX_INITIALIZER, .initialized = false,
            .module_count = 0, .module_dir = {0},
            .module_file_count = 0 };

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

    /* 行缓冲模式：遇 \n 自动 flush，无需每行手动 fflush。
     * stderr 默认无缓冲（_IONBF），文件默认全缓冲（_IOFBF），
     * 统一设为行缓冲（_IOLBF）保证日志实时可见且不阻塞主循环。 */
#if defined(_WIN32)
    setvbuf(g_log.output, NULL, _IONBF, 0);
#else
    setvbuf(g_log.output, NULL, _IOLBF, 0);
#endif

    g_log.initialized = true;

    LOG_INFO("logger", "initialized (level=%s, output=%s)",
             log_level_str(min_level),
             (filename && filename[0]) ? filename : "stderr");
}

void log_shutdown(void) {
    if (!g_log.initialized) return;

    LOG_INFO("logger", "shutting down");

    /* 关闭所有模块文件 */
    pthread_mutex_lock(&g_log.mutex);
    for (int i = 0; i < g_log.module_file_count; i++) {
        if (g_log.module_files[i].fp) {
            fflush(g_log.module_files[i].fp);
            fclose(g_log.module_files[i].fp);
            g_log.module_files[i].fp = NULL;
        }
    }
    g_log.module_file_count = 0;
    pthread_mutex_unlock(&g_log.mutex);

    if (g_log.output && g_log.output != stderr) {
        fflush(g_log.output);
        fclose(g_log.output);
    } else if (g_log.output) {
        fflush(g_log.output);
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

void log_set_module_dir(const char* dir) {
    if (!dir || !dir[0]) return;
    pthread_mutex_lock(&g_log.mutex);
    snprintf(g_log.module_dir, sizeof(g_log.module_dir), "%s", dir);
    mkdir(dir, 0755);
    /* 关闭已有模块文件，下次写入时重新打开 */
    for (int i = 0; i < g_log.module_file_count; i++) {
        if (g_log.module_files[i].fp) {
            fclose(g_log.module_files[i].fp);
            g_log.module_files[i].fp = NULL;
        }
    }
    g_log.module_file_count = 0;
    pthread_mutex_unlock(&g_log.mutex);
}

/** 查找或创建模块文件句柄（调用者需持有 mutex） */
static FILE* get_module_file(const char* module) {
    if (!module || !g_log.module_dir[0]) return NULL;
    for (int i = 0; i < g_log.module_file_count; i++) {
        if (strcmp(g_log.module_files[i].name, module) == 0)
            return g_log.module_files[i].fp;
    }
    if (g_log.module_file_count >= LOG_MAX_MODULE_FILES) return NULL;
    char path[320];
    snprintf(path, sizeof(path), "%s/%s.log", g_log.module_dir, module);
    FILE* fp = fopen(path, "a");
    if (!fp) return NULL;
    setvbuf(fp, NULL, _IOLBF, 0);
    ModuleFile* mf = &g_log.module_files[g_log.module_file_count++];
    snprintf(mf->name, sizeof(mf->name), "%s", module);
    mf->fp = fp;
    return fp;
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
    va_list args, args_copy;
    va_start(args, fmt);
    va_copy(args_copy, args);
    vfprintf(out, fmt, args);
    va_end(args);

    /* Source location for DEBUG/TRACE */
    if (level <= LOG_DEBUG) {
        const char* fname = strrchr(file, '/');
        fname = fname ? fname + 1 : file;
        fprintf(out, "  [%s:%d %s()]", fname, line, func ? func : "?");
    }

    fprintf(out, "\n");

    /* 同时写入模块专属日志文件（如 /tmp/flow_logs/planning.log） */
#if !defined(_WIN32)
    if (g_log.module_dir[0] && module) {
        FILE* mf = get_module_file(module);
        if (mf && mf != out) {
            fprintf(mf, "[%04d-%02d-%02d %02d:%02d:%02d.%03d] [%-5s] ",
                    tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                    tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
                    (int)(tv.tv_usec / 1000),
                    log_level_str(level));
            vfprintf(mf, fmt, args_copy);
            if (level <= LOG_DEBUG) {
                const char* fname = strrchr(file, '/');
                fname = fname ? fname + 1 : file;
                fprintf(mf, "  [%s:%d %s()]", fname, line, func ? func : "?");
            }
            fprintf(mf, "\n");
        }
    }
#endif
    va_end(args_copy);

    pthread_mutex_unlock(&g_log.mutex);
}

/* ══════════════════════════════════════════════════════════ */
/* 兼容旧 API                                                 */
/* ══════════════════════════════════════════════════════════ */

void default_log_callback(LogLevel level, const char* message) {
    log_write("core", level, __FILE__, __LINE__, __func__, "%s", message);
}
