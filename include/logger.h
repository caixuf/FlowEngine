#ifndef LOGGER_H
#define LOGGER_H

#include "process_interface.h"
#include <stdio.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 日志器结构体（线程安全）
 */
typedef struct {
    FILE*           file;
    LogLevel        level;
    char            filename[256];
    pthread_mutex_t mutex;   /**< 保护并发写入 */
} Logger;

/**
 * 创建日志器
 * @param filename 日志文件名，传 NULL 或空字符串输出到 stderr
 * @param level    最低输出级别
 * @return 日志器指针，失败返回 NULL
 */
Logger* logger_create(const char* filename, LogLevel level);

/**
 * 销毁日志器并释放资源
 */
void logger_destroy(Logger* logger);

/**
 * 写入一条固定字符串日志
 * @param logger  日志器指针
 * @param level   日志级别
 * @param message 消息字符串
 */
void logger_log(Logger* logger, LogLevel level, const char* message);

/**
 * 按格式写入日志（线程安全，类 printf 语法）
 *
 * 示例：
 *   logger_logf(logger, LOG_LEVEL_INFO, "加载插件 %s 成功", name);
 *
 * @param logger 日志器指针
 * @param level  日志级别
 * @param fmt    printf 格式字符串
 * @param ...    可变参数
 */
void logger_logf(Logger* logger, LogLevel level, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * 全局默认日志回调（输出到 stderr），可传给 process_manager_create()
 */
void default_log_callback(LogLevel level, const char* message);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
