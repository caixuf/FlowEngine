#ifndef SCHEDULER_H
#define SCHEDULER_H

/**
 * @file scheduler.h
 * @brief 任务调度器 (FlowEngine Phase 2)
 *
 * 提供：
 *   - RateControl — 频率限制，防止任务过频执行
 *   - LatencyTracker — 环形缓冲延迟统计（P50/P99）
 *   - ResourceQuota — CPU 时间/内存/执行次数上限
 *   - Scheduler — M:N 协程调度器（N worker × M coroutine）
 *
 * 典型用法：
 *   Scheduler* sched = scheduler_create(NULL);  // default config
 *   scheduler_register_task(sched, task, "sensor_fusion");
 *   scheduler_set_params(sched, tid, TASK_PRIORITY_HIGH, 0x03, 100.0);
 *   scheduler_start(sched);
 *   // ... tasks execute ...
 *   scheduler_stop(sched);
 *   scheduler_destroy(sched);
 */

#include "task_interface.h"
#include "message_bus.h"
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Rate Control ─────────────────────────────────────────── */

typedef struct {
    uint64_t  period_us;        /**< Minimum interval between executions */
    uint64_t  last_run_us;      /**< Last execution timestamp (monotonic) */
    double    max_frequency_hz; /**< Cap on executions per second (0 = no limit) */
} RateControl;

/** Initialize rate control. @param max_hz  Maximum frequency (0 = unlimited) */
void rate_control_init(RateControl* rc, double max_hz);

/**
 * Try to acquire execution permission.
 * Returns true if enough time has elapsed since last run.
 * Atomically updates last_run_us if granted.
 */
bool rate_control_acquire(RateControl* rc);

/* ── Latency Tracker ──────────────────────────────────────── */

#define LATENCY_BUFFER_SIZE 1024

typedef struct {
    uint64_t  recent[LATENCY_BUFFER_SIZE];  /**< Circular buffer of samples */
    uint32_t  head;                         /**< Write position */
    uint32_t  count;                        /**< Valid samples (0..LATENCY_BUFFER_SIZE) */
    uint64_t  sample_total;                 /**< Running sum */
    uint64_t  sample_count;                 /**< Total samples recorded */
    uint64_t  min_us;                       /**< Minimum observed */
    uint64_t  max_us;                       /**< Maximum observed */
} LatencyTracker;

/** Record a latency sample (thread-safe). */
void latency_tracker_record(LatencyTracker* lt, uint64_t latency_us);

/** Get latency statistics. */
typedef struct {
    uint64_t  avg_us;
    uint64_t  min_us;
    uint64_t  max_us;
    uint64_t  p50_us;
    uint64_t  p99_us;
    uint64_t  sample_count;
} LatencyStats;

/** Compute latency statistics from the tracker. */
LatencyStats latency_tracker_stats(LatencyTracker* lt);

/* ── Resource Quota ───────────────────────────────────────── */

typedef struct {
    uint64_t  max_cpu_time_us;      /**< Max CPU time per execution burst */
    uint64_t  max_execution_count;  /**< Max total executions (-1 = unlimited) */
    size_t    max_memory_bytes;     /**< Max heap allocation (0 = unlimited) */
} ResourceQuota;

typedef struct {
    ResourceQuota  quota;
    uint64_t       cpu_time_used_us;
    uint64_t       execution_count;
    volatile size_t allocated_bytes;  /**< Approximate tracked allocation */
} ResourceUsage;

/** Initialize resource usage tracking. */
void resource_usage_init(ResourceUsage* ru, const ResourceQuota* quota);

/** Check if a resource has exceeded its quota. */
bool resource_usage_check(const ResourceUsage* ru);

/* ── Scheduler Configuration ──────────────────────────────── */

typedef enum {
    SCHEDULER_MODE_CLASSIC = 0,  /**< thread-per-task，轮询执行 */
    SCHEDULER_MODE_CHOREO  = 1,  /**< DAG编排：上游publish → 自动触发下游 */
} SchedulerMode;

typedef struct {
    uint32_t       worker_thread_count;   /**< 0 = use hardware_concurrency() */
    uint32_t       max_coroutine_count;   /**< 0 = unlimited */
    bool           enable_work_stealing;  /**< Work stealing between workers */
    uint64_t       tick_us;               /**< Scheduler tick (default 1000 us) */
    SchedulerMode  mode;                  /**< Scheduling mode */
} SchedulerConfig;

#define SCHEDULER_CONFIG_DEFAULT \
    { .worker_thread_count  = 0,   \
      .max_coroutine_count  = 0,   \
      .enable_work_stealing = false, \
      .tick_us              = 1000, \
      .mode                 = SCHEDULER_MODE_CLASSIC }

/* ── Scheduler Handle ──────────────────────────────────────── */

typedef struct Scheduler Scheduler;

/** Create a new scheduler. @param config  Configuration (NULL = defaults). */
Scheduler* scheduler_create(const SchedulerConfig* config);

/** Destroy scheduler (must be stopped first). */
void scheduler_destroy(Scheduler* scheduler);

/** Start the scheduler (creates worker threads). */
int scheduler_start(Scheduler* sched);

/** Set the message bus for choreo trigger routing. */
void scheduler_set_choreo_bus(Scheduler* sched, MessageBus* bus);

/** Stop the scheduler (joins worker threads). */
void scheduler_stop(Scheduler* sched);

/** Register a task with the scheduler. Returns internal task ID. */
int scheduler_register_task(Scheduler* sched, TaskBase* task, const char* name);

/** Unregister a task. */
int scheduler_unregister_task(Scheduler* sched, int task_id);

/**
 * Set scheduling parameters for a registered task.
 * @param task_id    Task handle from scheduler_register_task
 * @param prio       Priority
 * @param cpu_mask   CPU affinity mask (0 = inherit)
 * @param max_freq_hz Maximum rate (0 = unlimited)
 */
int scheduler_set_params(Scheduler* sched, int task_id,
                         TaskPriority prio, uint64_t cpu_mask,
                         double max_freq_hz);

/** Set resource quota for a registered task. */
int scheduler_set_quota(Scheduler* sched, int task_id,
                        const ResourceQuota* quota);

/** Get the latency tracker for a registered task. */
LatencyTracker* scheduler_get_latency(Scheduler* sched, int task_id);

/** Get the rate controller for a registered task. */
RateControl* scheduler_get_rate_control(Scheduler* sched, int task_id);

/** Get the number of registered tasks. */
int scheduler_task_count(Scheduler* sched);

/* ── Choreo 模式: 数据流驱动 ─────────────────────────────── */

/**
 * 注册 topic 触发关系：当 topic 上有消息发布时，自动唤醒 task。
 *
 * Choreo 模式下，task 不需要轮询——它在 condition_variable 上等待，
 * 直到上游 publish 触发它执行。
 *
 * @param sched  调度器
 * @param task_id 目标任务 ID
 * @param topic   触发 topic（上游 publish 此 topic 时唤醒）
 * @return 0 成功
 */
int scheduler_choreo_trigger_on(Scheduler* sched, int task_id, const char* topic);

/**
 * Choreo 模式下 task 的执行入口。
 * task 在 execute() 中调用此函数，阻塞等待触发信号。
 *
 * @param sched    调度器
 * @param task_id  任务 ID
 * @param timeout_us 超时（微秒, 0=无限等待）
 * @return 0=触发, -1=超时, -2=停止
 */
int scheduler_choreo_wait(Scheduler* sched, int task_id, uint64_t timeout_us);

/**
 * 获取 choreo 触发统计。
 */
typedef struct {
    uint64_t triggers_fired;     /**< 触发次数 */
    uint64_t triggers_missed;    /**< 错过（任务忙） */
    uint64_t wait_timeouts;      /**< 等待超时 */
} ChoreoStats;

void scheduler_get_choreo_stats(Scheduler* sched, int task_id, ChoreoStats* stats);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
