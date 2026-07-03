#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "task_interface.h"
#include <sys/queue.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 任务管理器回调函数类型
 */
typedef void (*TaskEventCallback)(const char* task_name, TaskState old_state, TaskState new_state, void* user_data);

/**
 * 依赖项条目（单链表）
 */
typedef struct DepEntry {
    char dep_name[64];
    struct DepEntry* next;
} DepEntry;

/**
 * 任务节点 - 链表节点
 */
typedef struct TaskNode {
    TaskBase* task;                    // 任务指针
    char name[64];                     // 任务名称(用于快速查找)
    DepEntry* deps;                    // 依赖任务名链表
    TAILQ_ENTRY(TaskNode) entries;     // 队列链接
} TaskNode;

/**
 * 任务管理器
 */
typedef struct TaskManager {
    TAILQ_HEAD(TaskList, TaskNode) task_list;  // 任务链表
    pthread_mutex_t mutex;                     // 保护链表的互斥锁
    pthread_t monitor_thread;                  // 监控线程
    bool is_running;                           // 管理器运行状态
    uint32_t task_count;                       // 任务计数
    uint32_t running_task_count;               // 运行中任务计数
    TaskEventCallback event_callback;          // 任务状态变更回调
    void* event_user_data;                     // 回调用户数据
} TaskManager;

/**
 * 创建任务管理器
 * @return 任务管理器指针，失败返回NULL
 */
TaskManager* task_manager_create(void);

/**
 * 销毁任务管理器
 * @param manager 任务管理器指针
 */
void task_manager_destroy(TaskManager* manager);

/**
 * 注册任务到管理器
 * @param manager 任务管理器指针
 * @param task 任务指针
 * @param name 任务名称
 * @return 0成功，非0失败
 */
int task_manager_register(TaskManager* manager, TaskBase* task, const char* name);

/**
 * 从管理器注销任务
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 0成功，非0失败
 */
int task_manager_unregister(TaskManager* manager, const char* name);

/**
 * 启动指定任务
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 0成功，非0失败
 */
int task_manager_start_task(TaskManager* manager, const char* name);

/**
 * 停止指定任务
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 0成功，非0失败
 */
int task_manager_stop_task(TaskManager* manager, const char* name);

/**
 * 重启指定任务
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 0成功，非0失败
 */
int task_manager_restart_task(TaskManager* manager, const char* name);

/**
 * 启动所有任务
 * @param manager 任务管理器指针
 * @return 失败任务的数量
 */
int task_manager_start_all(TaskManager* manager);

/**
 * 停止所有任务
 * @param manager 任务管理器指针
 * @return 失败任务的数量
 */
int task_manager_stop_all(TaskManager* manager);

/**
 * 获取任务指针
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 任务指针，未找到返回NULL
 */
TaskBase* task_manager_get_task(TaskManager* manager, const char* name);

/**
 * 获取任务状态
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 任务状态
 */
TaskState task_manager_get_task_state(TaskManager* manager, const char* name);

/**
 * 获取任务统计信息
 * @param manager 任务管理器指针
 * @param name 任务名称
 * @return 统计信息指针，未找到返回NULL
 */
const TaskStats* task_manager_get_task_stats(TaskManager* manager, const char* name);

/**
 * 列出所有任务
 * @param manager 任务管理器指针
 * @param names 任务名称数组(输出)
 * @param max_count 数组最大容量
 * @return 实际任务数量
 */
int task_manager_list_tasks(TaskManager* manager, char names[][64], int max_count);

/**
 * 启动任务监控
 * @param manager 任务管理器指针
 * @return 0成功，非0失败
 */
int task_manager_start_monitor(TaskManager* manager);

/**
 * 停止任务监控
 * @param manager 任务管理器指针
 */
void task_manager_stop_monitor(TaskManager* manager);

/**
 * 设置任务事件回调
 * @param manager 任务管理器指针
 * @param callback 回调函数
 * @param user_data 用户数据
 */
void task_manager_set_event_callback(TaskManager* manager, TaskEventCallback callback, void* user_data);

/**
 * 执行任务健康检查
 * @param manager 任务管理器指针
 * @return 不健康任务数量
 */
int task_manager_health_check(TaskManager* manager);

/**
 * 获取管理器统计信息
 * @param manager 任务管理器指针
 * @param total_tasks 总任务数(输出)
 * @param running_tasks 运行中任务数(输出)
 * @param error_tasks 错误任务数(输出)
 */
void task_manager_get_stats(TaskManager* manager, uint32_t* total_tasks, uint32_t* running_tasks, uint32_t* error_tasks);

/**
 * 添加任务依赖关系（用于拓扑排序启动）
 * task_name 的启动将在 dep_name 成功运行后进行。
 * @param manager    任务管理器指针
 * @param task_name  依赖其他任务的任务名
 * @param dep_name   被依赖的任务名
 * @return 0 成功，-1 失败（任务未找到或内存不足）
 */
int task_manager_add_dependency(TaskManager* manager, const char* task_name, const char* dep_name);

#ifdef __cplusplus
}
#endif

#endif // TASK_MANAGER_H
