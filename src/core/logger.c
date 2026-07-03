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

    if (filename && filename[0] != '\0') {
        strncpy(logger->filename, filename, sizeof(logger->filename) - 1);
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

    fprintf(logger->file, "[%s][%s] %s\n", timebuf, lvl, message);
    fflush(logger->file);
}

/* Global default logger for default_log_callback */
static Logger g_default_logger = {
    .file  = NULL,
    .level = LOG_LEVEL_DEBUG,
    .filename = ""
};

void default_log_callback(LogLevel level, const char* message) {
    if (!g_default_logger.file) {
        g_default_logger.file = stderr;
    }
    logger_log(&g_default_logger, level, message);
}
