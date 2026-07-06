#ifndef PROCESS_MANAGER_H
#define PROCESS_MANAGER_H

#include "process_interface.h"
#include <pthread.h>
#include <sys/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 进程节点
 */
typedef struct ProcessNode {
    char name[64];                    // 进程名称
    char library_path[256];           // 动态库路径
    char config_data[1024];           // 配置数据
    void* lib_handle;                 // 动态库句柄
    ProcessInterface* interface;      // 进程接口
    pthread_t thread;                 // 进程线程
    bool is_running;                  // 是否正在运行
    bool should_restart;              // 是否需要重启
    uint32_t restart_count;           // 当前重启次数

    /* ── 调度配置 ─────────────────────────────────── */
    int      sched_priority;          /**< 0=low 1=normal 2=high 3=critical */
    uint64_t sched_cpu_mask;          /**< CPU 亲和性位掩码 */
    double   sched_max_freq_hz;       /**< 最大执行频率 */

    TAILQ_ENTRY(ProcessNode) entries; // 队列链接
} ProcessNode;

/**
 * 进程管理器
 */
typedef struct {
    TAILQ_HEAD(ProcessList, ProcessNode) process_list; // 进程列表
    pthread_mutex_t mutex;                             // 互斥锁
    pthread_t monitor_thread;                          // 监控线程
    bool is_running;                                   // 管理器运行状态
    LogCallback log_callback;                          // 日志回调
} ProcessManager;

/**
 * 创建进程管理器
 * @param log_callback 日志回调函数
 * @return 进程管理器指针，失败返回NULL
 */
ProcessManager* process_manager_create(LogCallback log_callback);

/**
 * 销毁进程管理器
 * @param manager 进程管理器
 */
void process_manager_destroy(ProcessManager* manager);

/**
 * 加载进程插件
 * @param manager 进程管理器
 * @param name 进程名称
 * @param library_path 动态库路径
 * @param config_data 配置数据
 * @return 0成功，非0失败
 */
int process_manager_load_plugin(ProcessManager* manager, 
                               const char* name,
                               const char* library_path,
                               const char* config_data);

/**
 * 设置进程的调度参数（优先级、CPU亲和性、频率限制）。
 * 必须在进程启动前调用。
 *
 * @param manager     进程管理器
 * @param name        进程名称
 * @param priority    优先级 (0=low 1=normal 2=high 3=critical)
 * @param cpu_mask    CPU 亲和性位掩码
 * @param max_freq_hz 最大执行频率 (0=不限制)
 * @return 0成功，非0失败
 */
int process_manager_set_scheduling(ProcessManager* manager, const char* name,
                                   int priority, uint64_t cpu_mask,
                                   double max_freq_hz);

/**
 * 启动进程
 * @param manager 进程管理器
 * @param name 进程名称
 * @return 0成功，非0失败
 */
int process_manager_start_process(ProcessManager* manager, const char* name);

/**
 * 停止进程
 * @param manager 进程管理器
 * @param name 进程名称
 * @return 0成功，非0失败
 */
int process_manager_stop_process(ProcessManager* manager, const char* name);

/**
 * 重启进程
 * @param manager 进程管理器
 * @param name 进程名称
 * @return 0成功，非0失败
 */
int process_manager_restart_process(ProcessManager* manager, const char* name);

/**
 * 启动所有进程
 * @param manager 进程管理器
 * @return 0成功，非0失败
 */
int process_manager_start_all(ProcessManager* manager);

/**
 * 停止所有进程
 * @param manager 进程管理器
 * @return 0成功，非0失败
 */
int process_manager_stop_all(ProcessManager* manager);

/**
 * 获取进程状态
 * @param manager 进程管理器
 * @param name 进程名称
 * @return 进程状态
 */
ProcessState process_manager_get_process_state(ProcessManager* manager, const char* name);

/**
 * 获取进程统计信息
 * @param manager 进程管理器
 * @param name 进程名称
 * @return 统计信息指针
 */
const ProcessStats* process_manager_get_process_stats(ProcessManager* manager, const char* name);

/**
 * 启动监控线程
 * @param manager 进程管理器
 * @return 0成功，非0失败
 */
int process_manager_start_monitor(ProcessManager* manager);

/**
 * 停止监控线程
 * @param manager 进程管理器
 */
void process_manager_stop_monitor(ProcessManager* manager);

/* ── Process enumeration ─────────────────────────────── */

/** 进程快照（用于 `list` 命令）*/
typedef struct {
    char name[64];
    char library_path[256];
    bool is_running;
    uint32_t restart_count;
    int sched_priority;
} ProcessSnapshot;

/**
 * 列出所有已加载的进程
 * @param manager 进程管理器
 * @param buf     输出缓冲区
 * @param max     缓冲区最多可容纳的条目数
 * @return 实际进程数（<= max）
 */
int process_manager_list_processes(ProcessManager* manager,
                                   ProcessSnapshot* buf, int max);

#ifdef __cplusplus
}
#endif

#endif // PROCESS_MANAGER_H
