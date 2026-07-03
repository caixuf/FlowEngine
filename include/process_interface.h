#ifndef PROCESS_INTERFACE_H
#define PROCESS_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 进程状态枚举
 */
typedef enum {
    PROCESS_STATE_UNKNOWN = 0,
    PROCESS_STATE_INITIALIZING,
    PROCESS_STATE_RUNNING,
    PROCESS_STATE_STOPPING,
    PROCESS_STATE_STOPPED,
    PROCESS_STATE_ERROR
} ProcessState;

/**
 * 进程信息结构体
 */
typedef struct {
    char name[64];          // 进程名称
    char version[32];       // 版本号
    char description[256];  // 描述
    uint32_t priority;      // 优先级
    uint32_t restart_count; // 重启次数限制
    bool auto_restart;      // 是否自动重启
} ProcessInfo;

/**
 * 进程统计信息
 */
typedef struct {
    uint64_t start_time;    // 启动时间
    uint64_t run_time;      // 运行时间
    uint32_t cpu_usage;     // CPU使用率(百分比)
    uint64_t memory_usage;  // 内存使用量(字节)
    uint32_t restart_times; // 已重启次数
} ProcessStats;

/* LogLevel is now defined in logger.h (unified logging system).
 * Legacy aliases provided for backward compatibility. */
#define LOG_LEVEL_DEBUG  LOG_DEBUG
#define LOG_LEVEL_INFO   LOG_INFO
#define LOG_LEVEL_WARN   LOG_WARN
#define LOG_LEVEL_ERROR  LOG_ERROR
#define LOG_LEVEL_FATAL  LOG_FATAL

/**
 * 日志回调函数类型
 */
typedef void (*LogCallback)(LogLevel level, const char* message);

/**
 * 进程接口结构体 - 每个插件进程必须实现这些接口
 */
typedef struct {
    /**
     * 获取进程信息
     * @return 进程信息指针
     */
    const ProcessInfo* (*get_process_info)(void);
    
    /**
     * 初始化进程
     * @param config_data 配置数据
     * @param log_callback 日志回调函数
     * @return 0成功，非0失败
     */
    int (*initialize)(const char* config_data, LogCallback log_callback);
    
    /**
     * 启动进程主循环
     * @return 0成功，非0失败
     */
    int (*start)(void);
    
    /**
     * 停止进程
     * @return 0成功，非0失败
     */
    int (*stop)(void);
    
    /**
     * 清理资源
     */
    void (*cleanup)(void);
    
    /**
     * 获取进程状态
     * @return 进程状态
     */
    ProcessState (*get_state)(void);
    
    /**
     * 获取进程统计信息
     * @return 统计信息指针
     */
    const ProcessStats* (*get_stats)(void);
    
    /**
     * 处理信号
     * @param signal 信号值
     */
    void (*handle_signal)(int signal);
    
    /**
     * 健康检查
     * @return true健康，false不健康
     */
    bool (*health_check)(void);
    
} ProcessInterface;

/**
 * 插件导出函数 - 每个插件动态库必须实现
 */

/**
 * 获取进程接口
 * @return 进程接口指针
 */
extern ProcessInterface* get_process_interface(void);

/**
 * 获取接口版本
 * @return 版本号
 */
extern uint32_t get_interface_version(void);

// 当前接口版本
#define PROCESS_INTERFACE_VERSION 0x00010000

#ifdef __cplusplus
}
#endif

#endif // PROCESS_INTERFACE_H
