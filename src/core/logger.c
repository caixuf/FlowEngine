#include "logger.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>

static const char* level_str[] = {
    "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
};

Logger* logger_create(const char* filename, LogLevel level) {
    Logger* logger = (Logger*)calloc(1, sizeof(Logger));
    if (!logger) return NULL;

    logger->level = level;

    if (pthread_mutex_init(&logger->mutex, NULL) != 0) {
        free(logger);
        return NULL;
    }

    if (filename && filename[0] != '\0') {
        snprintf(logger->filename, sizeof(logger->filename), "%s", filename);
        logger->file = fopen(filename, "a");
        if (!logger->file) {
            /* Fall back to stderr if file cannot be opened */
            logger->file = stderr;
        }
    } else {
        logger->file = stderr;
    }

    return logger;
}

void logger_destroy(Logger* logger) {
    if (!logger) return;
    if (logger->file && logger->file != stderr && logger->file != stdout) {
        fclose(logger->file);
    }
    pthread_mutex_destroy(&logger->mutex);
    free(logger);
}

void logger_log(Logger* logger, LogLevel level, const char* message) {
    if (!logger || !message) return;
    if (level < logger->level) return;

    time_t now = time(NULL);
    struct tm tm_buf;
    localtime_r(&now, &tm_buf);
    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &tm_buf);

    const char* lvl = (level >= 0 && level <= LOG_LEVEL_FATAL)
                      ? level_str[level] : "UNKNOWN";

    pthread_mutex_lock(&logger->mutex);
    fprintf(logger->file, "[%s][%s] %s\n", timebuf, lvl, message);
    fflush(logger->file);
    pthread_mutex_unlock(&logger->mutex);
}

void logger_logf(Logger* logger, LogLevel level, const char* fmt, ...) {
    if (!logger || !fmt) return;
    if (level < logger->level) return;

    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    logger_log(logger, level, buf);
}

/* Global default logger for default_log_callback */
static Logger g_default_logger = {
    .file  = NULL,
    .level = LOG_LEVEL_DEBUG,
    .filename = ""
};
static pthread_once_t g_default_logger_once = PTHREAD_ONCE_INIT;

static void init_default_logger_mutex(void) {
    if (!g_default_logger.file) g_default_logger.file = stderr;
    pthread_mutex_init(&g_default_logger.mutex, NULL);
}

void default_log_callback(LogLevel level, const char* message) {
    pthread_once(&g_default_logger_once, init_default_logger_mutex);
    logger_log(&g_default_logger, level, message);
}
