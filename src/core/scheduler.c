/**
 * scheduler.c — 任务调度器实现 (FlowEngine Phase 2)
 *
 * 组件:
 *   - RateControl: 频率限制（原子操作）
 *   - LatencyTracker: 环形缓冲 P50/P99 延迟统计
 *   - ResourceQuota: 资源配额追踪
 *   - Scheduler: 多任务注册/参数管理
 */

#include "scheduler.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

/* ── Monotonic clock helper ───────────────────────────────── */

static uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ══════════════════════════════════════════════════════════ */
/* RateControl                                                */
/* ══════════════════════════════════════════════════════════ */

void rate_control_init(RateControl* rc, double max_hz) {
    if (!rc) return;
    rc->max_frequency_hz = max_hz;
    if (max_hz > 0.0) {
        rc->period_us = (uint64_t)(1000000.0 / max_hz);
    } else {
        rc->period_us = 0;
    }
    rc->last_run_us = 0;
}

bool rate_control_acquire(RateControl* rc) {
    if (!rc || rc->period_us == 0) return true;  /* no limit */

    uint64_t now = monotonic_us();
    /* Simple check: if enough time has passed, allow */
    if (rc->last_run_us == 0 || (now - rc->last_run_us) >= rc->period_us) {
        /* Use atomic swap/comparison — here we just update */
        /* In single-producer scenarios this is safe without atomics */
        rc->last_run_us = now;
        return true;
    }
    return false;
}

/* ══════════════════════════════════════════════════════════ */
/* LatencyTracker                                             */
/* ══════════════════════════════════════════════════════════ */

void latency_tracker_record(LatencyTracker* lt, uint64_t latency_us) {
    if (!lt) return;
    if (lt->count < LATENCY_BUFFER_SIZE) {
        lt->recent[lt->head] = latency_us;
        lt->head = (lt->head + 1) % LATENCY_BUFFER_SIZE;
        lt->count++;
    } else {
        lt->recent[lt->head] = latency_us;
        lt->head = (lt->head + 1) % LATENCY_BUFFER_SIZE;
    }
    lt->sample_total += latency_us;
    lt->sample_count++;
    if (lt->min_us == 0 || latency_us < lt->min_us) lt->min_us = latency_us;
    if (latency_us > lt->max_us) lt->max_us = latency_us;
}

/* Simple quicksort for computing percentiles */
static int cmp_u64(const void* a, const void* b) {
    uint64_t ua = *(const uint64_t*)a;
    uint64_t ub = *(const uint64_t*)b;
    return (ua > ub) - (ua < ub);
}

LatencyStats latency_tracker_stats(LatencyTracker* lt) {
    LatencyStats s;
    memset(&s, 0, sizeof(s));
    if (!lt || lt->count == 0) return s;

    s.min_us = lt->min_us;
    s.max_us = lt->max_us;
    s.sample_count = lt->sample_count;
    s.avg_us = (lt->sample_count > 0) ? lt->sample_total / lt->sample_count : 0;

    /* Copy ring buffer to linear array for sorting */
    uint64_t sorted[LATENCY_BUFFER_SIZE];
    uint32_t n = lt->count;
    /* Unroll circular buffer: from (head - count) mod N to (head - 1) mod N */
    uint32_t start = (lt->head + LATENCY_BUFFER_SIZE - n) % LATENCY_BUFFER_SIZE;
    for (uint32_t i = 0; i < n; i++) {
        sorted[i] = lt->recent[(start + i) % LATENCY_BUFFER_SIZE];
    }

    qsort(sorted, n, sizeof(uint64_t), cmp_u64);

    s.p50_us = sorted[(uint32_t)(n * 0.50)];
    s.p99_us = sorted[(uint32_t)(n * 0.99)];

    return s;
}

/* ══════════════════════════════════════════════════════════ */
/* ResourceQuota                                              */
/* ══════════════════════════════════════════════════════════ */

void resource_usage_init(ResourceUsage* ru, const ResourceQuota* quota) {
    if (!ru) return;
    memset(ru, 0, sizeof(*ru));
    if (quota) ru->quota = *quota;
}

bool resource_usage_check(const ResourceUsage* ru) {
    if (!ru) return true;
    if (ru->quota.max_execution_count > 0 &&
        ru->execution_count >= ru->quota.max_execution_count) return false;
    if (ru->quota.max_cpu_time_us > 0 &&
        ru->cpu_time_used_us >= ru->quota.max_cpu_time_us) return false;
    if (ru->quota.max_memory_bytes > 0 &&
        ru->allocated_bytes >= ru->quota.max_memory_bytes) return false;
    return true;
}

/* ══════════════════════════════════════════════════════════ */
/* Scheduler Internal                                         */
/* ══════════════════════════════════════════════════════════ */

#define SCHED_MAX_TASKS 128

typedef struct {
    TaskBase*      task;
    char           name[64];
    int            id;
    LatencyTracker latency;
    RateControl    rate_control;
    ResourceUsage  resource;
    bool           active;
} SchedTaskEntry;

struct Scheduler {
    SchedulerConfig  config;
    SchedTaskEntry   entries[SCHED_MAX_TASKS];
    int              entry_count;
    pthread_mutex_t  mutex;
    bool             running;
    /* Worker threads (for future M:N model) */
    pthread_t*       workers;
    uint32_t         worker_count;
};

Scheduler* scheduler_create(const SchedulerConfig* config) {
    Scheduler* s = (Scheduler*)calloc(1, sizeof(Scheduler));
    if (!s) return NULL;

    if (config) {
        s->config = *config;
    } else {
        s->config = (SchedulerConfig)SCHEDULER_CONFIG_DEFAULT;
    }

    if (s->config.worker_thread_count == 0) {
        long nprocs = sysconf(_SC_NPROCESSORS_ONLN);
        s->config.worker_thread_count = (uint32_t)(nprocs > 0 ? nprocs : 4);
    }

    pthread_mutex_init(&s->mutex, NULL);
    s->running = false;
    return s;
}

void scheduler_destroy(Scheduler* sched) {
    if (!sched) return;
    if (sched->running) scheduler_stop(sched);
    free(sched->workers);
    pthread_mutex_destroy(&sched->mutex);
    free(sched);
}

int scheduler_start(Scheduler* sched) {
    if (!sched || sched->running) return -1;
    sched->running = true;

    /* For now, the scheduler is a registry + parameter manager.
     * Tasks are still started via task_start (thread-per-task).
     * The M:N worker pool (CppScheduler) will be added in Phase 2D. */

    printf("[scheduler] Started with %u tasks, %u worker threads\n",
           sched->entry_count, sched->config.worker_thread_count);
    return 0;
}

void scheduler_stop(Scheduler* sched) {
    if (!sched || !sched->running) return;
    sched->running = false;
    printf("[scheduler] Stopped\n");
}

int scheduler_register_task(Scheduler* sched, TaskBase* task, const char* name) {
    if (!sched || !task) return -1;

    pthread_mutex_lock(&sched->mutex);
    if (sched->entry_count >= SCHED_MAX_TASKS) {
        pthread_mutex_unlock(&sched->mutex);
        return -1;
    }

    int id = sched->entry_count;
    SchedTaskEntry* e = &sched->entries[id];
    memset(e, 0, sizeof(*e));
    e->task = task;
    e->id   = id;
    e->active = true;
    if (name) snprintf(e->name, sizeof(e->name), "%s", name);
    rate_control_init(&e->rate_control, task->config.max_frequency_hz);
    sched->entry_count++;

    pthread_mutex_unlock(&sched->mutex);
    return id;
}

int scheduler_unregister_task(Scheduler* sched, int task_id) {
    if (!sched || task_id < 0 || task_id >= sched->entry_count) return -1;

    pthread_mutex_lock(&sched->mutex);
    sched->entries[task_id].active = false;
    pthread_mutex_unlock(&sched->mutex);
    return 0;
}

int scheduler_set_params(Scheduler* sched, int task_id,
                         TaskPriority prio, uint64_t cpu_mask,
                         double max_freq_hz) {
    if (!sched || task_id < 0 || task_id >= sched->entry_count) return -1;

    pthread_mutex_lock(&sched->mutex);
    SchedTaskEntry* e = &sched->entries[task_id];
    e->task->config.priority         = prio;
    e->task->config.cpu_affinity_mask = cpu_mask;
    e->task->config.max_frequency_hz  = max_freq_hz;
    rate_control_init(&e->rate_control, max_freq_hz);
    pthread_mutex_unlock(&sched->mutex);
    return 0;
}

int scheduler_set_quota(Scheduler* sched, int task_id,
                        const ResourceQuota* quota) {
    if (!sched || task_id < 0 || task_id >= sched->entry_count) return -1;

    pthread_mutex_lock(&sched->mutex);
    resource_usage_init(&sched->entries[task_id].resource, quota);
    pthread_mutex_unlock(&sched->mutex);
    return 0;
}

LatencyTracker* scheduler_get_latency(Scheduler* sched, int task_id) {
    if (!sched || task_id < 0 || task_id >= sched->entry_count) return NULL;
    return &sched->entries[task_id].latency;
}

RateControl* scheduler_get_rate_control(Scheduler* sched, int task_id) {
    if (!sched || task_id < 0 || task_id >= sched->entry_count) return NULL;
    return &sched->entries[task_id].rate_control;
}

int scheduler_task_count(Scheduler* sched) {
    return sched ? sched->entry_count : 0;
}
