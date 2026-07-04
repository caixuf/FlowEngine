#include "task_interface.h"
#include "message_bus.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <errno.h>

/* ── helpers ─────────────────────────────────────────── */

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* Map TaskState to StateMachine event for state transitions */
static EventId task_state_to_event(TaskState old, TaskState new) {
    switch (new) {
        case TASK_STATE_RUNNING:     return SM_EVENT_START;
        case TASK_STATE_STOPPING:    return SM_EVENT_STOP;
        case TASK_STATE_STOPPED:     return SM_EVENT_DONE;
        case TASK_STATE_ERROR:       return SM_EVENT_ERROR;
        case TASK_STATE_INITIALIZED: return SM_EVENT_RESTART;
        default:                     return SM_EVENT_NONE;
    }
    (void)old;
}

static void set_state(TaskBase* task, TaskState new_state) {
    TaskState old;
    pthread_mutex_lock(&task->mutex);
    old = task->state;
    task->state = new_state;
    pthread_mutex_unlock(&task->mutex);

    /* Sync with reflective state machine if enabled */
    if (task->sm_enabled && new_state != old) {
        EventId ev = task_state_to_event(old, new_state);
        if (ev != SM_EVENT_NONE) {
            statem_send_event(&task->sm, ev, task);
        }
    }
}

/* ── Thread entry ─────────────────────────────────────── */

typedef struct {
    TaskBase* task;
} TaskThreadArg;

static void* task_thread_fn(void* arg) {
    TaskBase* task = (TaskBase*)arg;

    /* initialize */
    if (task->vtable && task->vtable->initialize) {
        int ret = task->vtable->initialize(task);
        if (ret != 0) {
            set_state(task, TASK_STATE_ERROR);
            return NULL;
        }
    }

    set_state(task, TASK_STATE_RUNNING);
    pthread_mutex_lock(&task->mutex);
    task->stats.start_time = now_us();
    task->stats.execution_count++;
    pthread_mutex_unlock(&task->mutex);

    /* execute */
    if (task->vtable && task->vtable->execute) {
        int ret = task->vtable->execute(task);
        if (ret != 0) {
            pthread_mutex_lock(&task->mutex);
            task->stats.error_count++;
            pthread_mutex_unlock(&task->mutex);
            set_state(task, TASK_STATE_ERROR);
        }
    }

    /* cleanup */
    if (task->vtable && task->vtable->cleanup) {
        task->vtable->cleanup(task);
    }

    pthread_mutex_lock(&task->mutex);
    uint64_t end = now_us();
    if (task->stats.start_time > 0 && end > task->stats.start_time)
        task->stats.total_run_time += end - task->stats.start_time;
    if (task->state != TASK_STATE_ERROR)
        task->state = TASK_STATE_STOPPED;
    pthread_mutex_unlock(&task->mutex);

    return NULL;
}

/* ── Public API ───────────────────────────────────────── */

int task_base_init(TaskBase* task, const TaskInterface* vtable, const TaskConfig* config) {
    if (!task || !vtable || !config) return ERR_INVALID_PARAM;

    memset(task, 0, sizeof(*task));
    task->vtable = vtable;
    task->config = *config;
    task->state  = TASK_STATE_INITIALIZED;

    /* Initialize reflective state machine */
    statem_init(&task->sm, SM_TABLE_STANDARD,
                SM_STATE_INITIALIZED, config->name);
    task->sm_enabled = true;  /* enabled by default */
    /* Enable trace by default for easy debugging */
    task->sm.trace_enabled = true;

    if (pthread_mutex_init(&task->mutex, NULL) != 0) return ERR_INTERNAL;
    return 0;
}

void task_base_destroy(TaskBase* task) {
    if (!task) return;
    pthread_mutex_destroy(&task->mutex);
}

int task_start(TaskBase* task) {
    if (!task) return ERR_INTERNAL;

    pthread_mutex_lock(&task->mutex);
    TaskState s = task->state;
    pthread_mutex_unlock(&task->mutex);

    if (s == TASK_STATE_RUNNING) return 0; /* already running */

    pthread_mutex_lock(&task->mutex);
    task->should_stop = false;
    pthread_mutex_unlock(&task->mutex);

    /* ── Phase 2: Build thread attributes with scheduling + affinity ── */
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /* Apply priority-based scheduling (best-effort; SCHED_FIFO requires root) */
    if (task->config.priority >= TASK_PRIORITY_HIGH) {
        struct sched_param sp;
        int policy = SCHED_FIFO;
        sp.sched_priority = (task->config.priority == TASK_PRIORITY_CRITICAL) ? 80 : 60;
        int r = pthread_attr_setschedpolicy(&attr, policy);
        if (r == 0) {
            pthread_attr_setschedparam(&attr, &sp);
            /* Inherit scheduling from attr (don't need explicit inherit) */
        }
        /* If SCHED_FIFO fails (e.g., no root), silently fall back to SCHED_OTHER */
    }

    /* Apply CPU affinity if mask is set */
    if (task->config.cpu_affinity_mask != 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        for (int core = 0; core < CPU_SETSIZE && core < 64; core++) {
            if (task->config.cpu_affinity_mask & (1ULL << core)) {
                CPU_SET(core, &cpuset);
            }
        }
        pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
    }

    int ret = pthread_create(&task->thread, &attr, task_thread_fn, task);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        fprintf(stderr, "[task_start] pthread_create failed for '%s': %s\n",
                task->config.name, strerror(ret));
        set_state(task, TASK_STATE_ERROR);
        return ERR_INTERNAL;
    }

    /* Log scheduling info for debug */
    if (task->config.priority >= TASK_PRIORITY_HIGH || task->config.cpu_affinity_mask) {
        printf("[task_start] '%s': prio=%d, policy=%s, cpu_mask=0x%lx\n",
               task->config.name, task->config.priority,
               (task->config.priority >= TASK_PRIORITY_HIGH) ? "FIFO" : "OTHER",
               (unsigned long)task->config.cpu_affinity_mask);
    }

    return 0;
}

int task_stop(TaskBase* task) {
    if (!task) return ERR_INTERNAL;

    pthread_mutex_lock(&task->mutex);
    TaskState s = task->state;
    pthread_mutex_unlock(&task->mutex);

    if (s == TASK_STATE_STOPPED || s == TASK_STATE_UNKNOWN || s == TASK_STATE_INITIALIZED) return 0;

    set_state(task, TASK_STATE_STOPPING);

    pthread_mutex_lock(&task->mutex);
    task->should_stop = true;
    pthread_mutex_unlock(&task->mutex);

    pthread_join(task->thread, NULL);
    return 0;
}

int task_restart(TaskBase* task) {
    if (!task) return ERR_INTERNAL;
    task_stop(task);

    pthread_mutex_lock(&task->mutex);
    task->state = TASK_STATE_INITIALIZED;
    task->restart_count++;
    pthread_mutex_unlock(&task->mutex);

    return task_start(task);
}

TaskState task_get_state(TaskBase* task) {
    if (!task) return TASK_STATE_UNKNOWN;
    pthread_mutex_lock(&task->mutex);
    TaskState s = task->state;
    pthread_mutex_unlock(&task->mutex);
    return s;
}

const TaskStats* task_get_stats(TaskBase* task) {
    if (!task) return NULL;
    return &task->stats;
}

void task_update_heartbeat(TaskBase* task) {
    if (!task) return;
    pthread_mutex_lock(&task->mutex);
    task->stats.last_heartbeat = now_us();
    pthread_mutex_unlock(&task->mutex);
}

bool task_should_stop(TaskBase* task) {
    if (!task) return true;
    pthread_mutex_lock(&task->mutex);
    bool s = task->should_stop;
    pthread_mutex_unlock(&task->mutex);
    return s;
}

void task_set_custom_config(TaskBase* task, void* config) {
    if (!task) return;
    task->config.custom_config = config;
}

void* task_get_custom_config(TaskBase* task) {
    if (!task) return NULL;
    return task->config.custom_config;
}

/* ── Message-driven subscription (Step 4) ──────────────── */

typedef struct {
    TaskBase*    task;
    MessageBus*  bus;
    char         topic[MSG_BUS_MAX_TOPIC_LEN];
} TaskSubCtx;

static void task_sub_callback(const Message* msg, void* user_data) {
    TaskSubCtx* ctx = (TaskSubCtx*)user_data;
    if (!ctx || !ctx->task) return;
    if (ctx->task->vtable && ctx->task->vtable->on_message) {
        ctx->task->vtable->on_message(ctx->task, msg);
    }
}

int task_subscribe(TaskBase* task, struct MessageBus* bus, const char* topic) {
    if (!task || !bus || !topic) return ERR_INTERNAL;

    TaskSubCtx* ctx = (TaskSubCtx*)malloc(sizeof(TaskSubCtx));
    if (!ctx) return ERR_INTERNAL;
    ctx->task = task;
    ctx->bus  = bus;
    snprintf(ctx->topic, sizeof(ctx->topic), "%s", topic);

    int ret = message_bus_subscribe(bus, topic, task_sub_callback, ctx);
    if (ret != 0) { free(ctx); return ERR_INTERNAL; }
    return 0;
}
