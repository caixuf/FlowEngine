#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Topic QoS (per-topic)
 */
typedef struct {
    int      depth;              /**< 队列深度 */
    char     policy[16];         /**< "drop_oldest"|"drop_latest"|"block" */
} TopicQosConfig;

/** Topic publish/subscribe declaration */
typedef struct {
    char     topic[64];          /**< Topic name */
    char     type[64];           /**< Message type name */
    char     remap[64];          /**< Remap target (empty = no remap) */
    int      qos_depth;          /**< QoS queue depth (0=default) */
    char     qos_policy[16];     /**< QoS policy */
    char     qos_reliability[16]; /**< "best_effort" or "reliable" */
    int      qos_deadline_ms;    /**< Deadline in ms (0=off) */
    int      qos_lifespan_ms;    /**< Lifespan in ms (0=off) */
} TopicDecl;

/** Resource limits */
typedef struct {
    int      max_memory_mb;      /**< Max memory in MB (0=unlimited) */
    int      max_cpu_percent;    /**< Max CPU % (0=unlimited) */
} ResourceLimits;

/**
 * 调度配置 (per-process)
 */
typedef struct {
    int      priority;           /**< 0=low 1=normal 2=high 3=critical */
    uint64_t cpu_affinity_mask;  /**< CPU 亲和性位掩码 (bit0=core0, 0=不绑定) */
    double   max_frequency_hz;   /**< 最大执行频率 (0=不限制) */
    int      cpuset_start;       /**< CPU 集合起始核心 (for simple range) */
    int      cpuset_end;         /**< CPU 集合结束核心 (-1=不使用范围) */
} SchedulingConfig;

/**
 * 进程配置项
 */
#define PROC_MAX_TOPICS 16
#define PROC_MAX_DEPS    8

typedef struct {
    char name[64];           // 进程名称
    char library_path[256];  // 动态库路径
    char config_data[1024];  // 配置数据
    int priority;            // 优先级 (deprecated, use scheduling.priority)
    bool auto_start;         // 是否自动启动
    SchedulingConfig scheduling; /**< 调度参数 */
    ResourceLimits    resources;  /**< 资源限制 */

    /* Publish/Subscribe with remap */
    TopicDecl publish[PROC_MAX_TOPICS];
    int       publish_count;
    TopicDecl subscribe[PROC_MAX_TOPICS];
    int       subscribe_count;

    /* Dependencies */
    char depends[PROC_MAX_DEPS][64];
    int  depends_count;

    /* Per-node params (key=value strings from JSON) */
    char params[256];
} ProcessConfig;

/**
 * 调度器全局配置
 */
typedef struct {
    int  mode;               /**< 0=classic 1=choreo */
    int  worker_threads;     /**< 0=auto (hardware_concurrency) */
    int  tick_us;            /**< 调度 tick (默认 1000us) */
} SchedulerGlobalConfig;

/**
 * 启动器配置
 */
typedef struct {
    char log_file[256];      // 日志文件路径
    int log_level;           // 日志级别
    int monitor_interval;    // 监控间隔(秒)
    bool enable_monitor;     // 是否启用监控
    SchedulerGlobalConfig scheduler; /**< 全局调度器配置 */
    ProcessConfig* processes; // 进程配置数组
    int process_count;       // 进程数量
} LauncherConfig;

/**
 * 加载配置文件
 * @param config_file 配置文件路径
 * @return 配置结构体指针，失败返回NULL
 */
LauncherConfig* config_load(const char* config_file);

/**
 * 释放配置
 * @param config 配置结构体指针
 */
void config_free(LauncherConfig* config);

/**
 * 保存配置文件
 * @param config 配置结构体指针
 * @param config_file 配置文件路径
 * @return 0成功，非0失败
 */
int config_save(const LauncherConfig* config, const char* config_file);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H
