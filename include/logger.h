#ifndef LOGGER_H
#define LOGGER_H

/**
 * @file logger.h
 * @brief 统一日志系统 — 全局单例 + 级别控制 + 时间戳 + 模块名
 *
 * 用法：
 *   #include "logger.h"
 *   LOG_INFO("sensor", "LiDAR frame #%u published", frame_id);
 *   LOG_WARN("control", "distance=%.1f below threshold", dist);
 *   LOG_ERROR("fusion", "alignment failed for topic %s", topic);
 *
 * 输出格式: [2026-07-04 10:30:45.123] [INFO ] [sensor] LiDAR frame #42 published
 *
 * 运行时控制：
 *   log_set_level(LOG_DEBUG);           // 全局级别
 *   log_set_module_level("discovery", LOG_WARN);  // 模块级别覆盖
 *   log_set_output(stderr);             // 默认输出到 stderr
 *   log_set_output_file("/var/log/flowengine.log");  // 输出到文件
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 日志级别 ─────────────────────────────────────────────── */

typedef enum {
    LOG_TRACE = 0,   /**< 最详细，用于逐行追踪 */
    LOG_DEBUG = 1,   /**< 调试信息 */
    LOG_INFO  = 2,   /**< 常规运行信息 */
    LOG_WARN  = 3,   /**< 警告（可恢复） */
    LOG_ERROR = 4,   /**< 错误（可能影响功能） */
    LOG_FATAL = 5,   /**< 致命错误（即将退出） */
    LOG_OFF   = 6,   /**< 关闭所有日志 */
} LogLevel;

static inline const char* log_level_str(LogLevel lv) {
    static const char* names[] = {"TRACE","DEBUG","INFO ","WARN ","ERROR","FATAL"};
    return (lv >= LOG_TRACE && lv <= LOG_FATAL) ? names[lv] : "????";
}

/* ── 全局日志器初始化 ─────────────────────────────────────── */

/**
 * 初始化全局日志器（进程启动时调用一次）。
 * @param min_level  全局最低输出级别
 * @param filename   输出文件（NULL=stderr）
 */
void log_init(LogLevel min_level, const char* filename);

/** 关闭全局日志器 */
void log_shutdown(void);

/* ── 运行时配置 ──────────────────────────────────────────── */

/** 设置全局最低输出级别 */
void log_set_level(LogLevel level);

/** 获取当前全局日志级别 */
LogLevel log_get_level(void);

/**
 * 为特定模块设置最低输出级别（覆盖全局级别）。
 * 例如: log_set_module_level("discovery", LOG_WARN)
 *        discovery 模块只输出 WARN 及以上。
 */
void log_set_module_level(const char* module, LogLevel level);

/** 获取模块的日志级别（若未设置则返回全局级别） */
LogLevel log_get_module_level(const char* module);

/** 切换输出目标（NULL=stderr） */
void log_set_output_file(const char* filename);

/** 获取当前输出文件指针 */
FILE* log_get_output(void);

/* ── 核心日志函数 ────────────────────────────────────────── */

/**
 * 格式化日志输出（线程安全）。
 *
 * @param module   模块名（如 "sensor"、"fusion"、"discovery"）
 * @param level    日志级别
 * @param file     源文件名（__FILE__）
 * @param line     行号（__LINE__）
 * @param func     函数名（__func__）
 * @param fmt      printf 格式字符串
 * @param ...      可变参数
 */
void log_write(const char* module, LogLevel level,
               const char* file, int line, const char* func,
               const char* fmt, ...)
    __attribute__((format(printf, 6, 7)));

/* ── 便捷宏 ────────────────────────────────────────────────── */

/**
 * 使用方式:
 *   LOG_INFO("mod", "message %d", val);
 *   LOG_ERROR("mod", "something broke: %s", reason);
 *
 * 模块名建议:
 *   "core"       — 核心库 (task, bus, bag)
 *   "scheduler"  — 调度器
 *   "statem"     — 状态机
 *   "discovery"  — 服务发现
 *   "fusion"     — 数据融合
 *   "serializer" — 序列化
 *   以及具体的 task 名
 */

#define LOG_TRACE(module, fmt, ...) \
    log_write(module, LOG_TRACE, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_DEBUG(module, fmt, ...) \
    log_write(module, LOG_DEBUG, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_INFO(module, fmt, ...) \
    log_write(module, LOG_INFO,  __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_WARN(module, fmt, ...) \
    log_write(module, LOG_WARN,  __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_ERROR(module, fmt, ...) \
    log_write(module, LOG_ERROR, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_FATAL(module, fmt, ...) \
    log_write(module, LOG_FATAL, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

/* ── 兼容旧 API ──────────────────────────────────────────── */

/** Legacy callback for process_manager */
void default_log_callback(LogLevel level, const char* message);

/* ── State file path (shared by e2e, flowctl, dashboard) ── */

#define FLOWENGINE_DEFAULT_STATE_FILE "/tmp/flow_topology.json"

/** Get the state file path from FLOWENGINE_STATE_FILE env, or default */
static inline const char* flowengine_state_file(void) {
    const char* env = getenv("FLOWENGINE_STATE_FILE");
    return env ? env : FLOWENGINE_DEFAULT_STATE_FILE;
}

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
