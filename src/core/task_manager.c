#include "task_manager.h"
#include "error_codes.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

/* ── Internal helpers ─────────────────────────────────── */

static TaskNode* find_node(TaskManager* mgr, const char* name) {
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        if (strcmp(node->name, name) == 0) return node;
    }
    return NULL;
}

static void notify_event(TaskManager* mgr, const char* name,
                         TaskState old_s, TaskState new_s) {
    if (mgr->event_callback)
        mgr->event_callback(name, old_s, new_s, mgr->event_user_data);
}

/* ── Monitor thread ───────────────────────────────────── */

static void* monitor_thread_fn(void* arg) {
    TaskManager* mgr = (TaskManager*)arg;

    while (mgr->is_running) {
        sleep(5);
        if (!mgr->is_running) break;

        pthread_mutex_lock(&mgr->mutex);
        TaskNode* node;
        TAILQ_FOREACH(node, &mgr->task_list, entries) {
            TaskBase* task = node->task;
            if (!task) continue;

            TaskState cur = task_get_state(task);

            if (cur == TASK_STATE_ERROR && task->config.auto_restart) {
                if (task->restart_count < task->config.max_restart_count) {
                    fprintf(stderr, "[TaskManager] Task '%s' in ERROR, restarting...\n", node->name);
                    TaskState old = cur;
                    task_restart(task);
                    notify_event(mgr, node->name, old, TASK_STATE_RUNNING);
                }
            }

            /* heartbeat check */
            if (cur == TASK_STATE_RUNNING && task->config.heartbeat_interval > 0) {
                uint64_t hb = task->stats.last_heartbeat;
                if (hb > 0) {
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    uint64_t now = (uint64_t)ts.tv_sec * 1000000ULL
                                 + (uint64_t)ts.tv_nsec / 1000ULL;
                    uint64_t interval_us = (uint64_t)task->config.heartbeat_interval * 1000000ULL;
                    if (now - hb > interval_us * 3) {
                        fprintf(stderr, "[TaskManager] Task '%s' heartbeat timeout\n", node->name);
                    }
                }
            }
        }
        pthread_mutex_unlock(&mgr->mutex);
    }
    return NULL;
}

/* ── Lifecycle ────────────────────────────────────────── */

TaskManager* task_manager_create(void) {
    TaskManager* mgr = (TaskManager*)calloc(1, sizeof(TaskManager));
    if (!mgr) return NULL;

    TAILQ_INIT(&mgr->task_list);
    pthread_mutex_init(&mgr->mutex, NULL);
    mgr->is_running = false;
    return mgr;
}

void task_manager_destroy(TaskManager* mgr) {
    if (!mgr) return;

    task_manager_stop_monitor(mgr);
    task_manager_stop_all(mgr);

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    while ((node = TAILQ_FIRST(&mgr->task_list)) != NULL) {
        TAILQ_REMOVE(&mgr->task_list, node, entries);
        /* free deps */
        DepEntry* dep = node->deps;
        while (dep) {
            DepEntry* next = dep->next;
            free(dep);
            dep = next;
        }
        free(node);
    }
    pthread_mutex_unlock(&mgr->mutex);
    pthread_mutex_destroy(&mgr->mutex);
    free(mgr);
}

/* ── Register / unregister ────────────────────────────── */

int task_manager_register(TaskManager* mgr, TaskBase* task, const char* name) {
    if (!mgr || !task || !name) return ERR_NOT_FOUND;

    pthread_mutex_lock(&mgr->mutex);
    if (find_node(mgr, name)) {
        pthread_mutex_unlock(&mgr->mutex);
        return ERR_NOT_FOUND; /* duplicate */
    }

    TaskNode* node = (TaskNode*)calloc(1, sizeof(TaskNode));
    if (!node) { pthread_mutex_unlock(&mgr->mutex); return ERR_NOT_FOUND; }
    node->task = task;
    snprintf(node->name, sizeof(node->name), "%s", name);
    TAILQ_INSERT_TAIL(&mgr->task_list, node, entries);
    mgr->task_count++;
    pthread_mutex_unlock(&mgr->mutex);
    return 0;
}

int task_manager_unregister(TaskManager* mgr, const char* name) {
    if (!mgr || !name) return ERR_NOT_FOUND;

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node = find_node(mgr, name);
    if (!node) { pthread_mutex_unlock(&mgr->mutex); return ERR_NOT_FOUND; }

    task_stop(node->task);
    TAILQ_REMOVE(&mgr->task_list, node, entries);
    mgr->task_count--;

    DepEntry* dep = node->deps;
    while (dep) {
        DepEntry* next = dep->next;
        free(dep);
        dep = next;
    }
    free(node);
    pthread_mutex_unlock(&mgr->mutex);
    return 0;
}

/* ── Start / Stop / Restart ───────────────────────────── */

int task_manager_start_task(TaskManager* mgr, const char* name) {
    if (!mgr || !name) return ERR_NOT_FOUND;

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node = find_node(mgr, name);
    pthread_mutex_unlock(&mgr->mutex);
    if (!node) return ERR_NOT_FOUND;

    /* Reflective check: can this task accept START? */
    if (node->task->sm_enabled && !task_can_event(node->task, SM_EVENT_START)) {
        fprintf(stderr, "[task_manager] '%s': cannot START — current state=%s, allowed=[",
                name, statem_state_name(&node->task->sm, statem_current(&node->task->sm)));
        EventId allowed[8];
        int n = task_allowed_events(node->task, allowed, 8);
        for (int i = 0; i < n; i++)
            fprintf(stderr, "%s%s", statem_event_name(&node->task->sm, allowed[i]),
                    (i < n - 1) ? "," : "");
        fprintf(stderr, "]\n");
        return ERR_NOT_FOUND;
    }

    TaskState old = task_get_state(node->task);
    int ret = task_start(node->task);
    if (ret == 0) notify_event(mgr, name, old, TASK_STATE_RUNNING);
    return ret;
}

int task_manager_stop_task(TaskManager* mgr, const char* name) {
    if (!mgr || !name) return ERR_NOT_FOUND;

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node = find_node(mgr, name);
    pthread_mutex_unlock(&mgr->mutex);
    if (!node) return ERR_NOT_FOUND;

    TaskState old = task_get_state(node->task);
    int ret = task_stop(node->task);
    if (ret == 0) notify_event(mgr, name, old, TASK_STATE_STOPPED);
    return ret;
}

int task_manager_restart_task(TaskManager* mgr, const char* name) {
    if (!mgr || !name) return ERR_NOT_FOUND;

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node = find_node(mgr, name);
    pthread_mutex_unlock(&mgr->mutex);
    if (!node) return ERR_NOT_FOUND;

    TaskState old = task_get_state(node->task);
    int ret = task_restart(node->task);
    if (ret == 0) notify_event(mgr, name, old, TASK_STATE_RUNNING);
    return ret;
}

int task_manager_start_all(TaskManager* mgr) {
    if (!mgr) return ERR_NOT_FOUND;
    int failed = 0;
    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        pthread_mutex_unlock(&mgr->mutex);
        if (task_manager_start_task(mgr, node->name) != 0) failed++;
        pthread_mutex_lock(&mgr->mutex);
    }
    pthread_mutex_unlock(&mgr->mutex);
    return failed;
}

/**
 * Start all tasks in dependency order (Kahn's algorithm).
 *
 * Tasks with no dependencies start first.  If a dependency cycle is detected,
 * remaining tasks are started in registration order with a warning.
 *
 * @return number of tasks that failed to start
 */
int task_manager_start_all_deps(TaskManager* mgr) {
    if (!mgr) return ERR_NOT_FOUND;

    /* Build name-to-index map */
    char  names[64][64];
    int   indeg[64];
    int   n = 0;

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        if (n >= 64) break;
        snprintf(names[n], 64, "%s", node->name);
        indeg[n] = 0;
        n++;
    }

    /* Compute in-degrees from dependency graph */
    for (int i = 0; i < n; i++) {
        /* Find node i's dependencies */
        TAILQ_FOREACH(node, &mgr->task_list, entries) {
            if (strcmp(node->name, names[i]) == 0) break;
        }
        if (!node) continue;
        DepEntry* dep = node->deps;
        while (dep) {
            for (int j = 0; j < n; j++) {
                if (strcmp(names[j], dep->dep_name) == 0) {
                    indeg[i]++;  /* task i depends on task j */
                    break;
                }
            }
            dep = dep->next;
        }
    }
    pthread_mutex_unlock(&mgr->mutex);

    /* Kahn's algorithm */
    bool started[64] = {false};
    int  started_count = 0;
    int  failed = 0;

    while (started_count < n) {
        bool progress = false;
        for (int i = 0; i < n; i++) {
            if (started[i]) continue;
            if (indeg[i] == 0) {
                if (task_manager_start_task(mgr, names[i]) != 0) {
                    fprintf(stderr, "[task_manager] failed to start '%s'\n", names[i]);
                    failed++;
                }
                started[i] = true;
                started_count++;
                progress = true;

                /* Decrement in-degrees of tasks that depend on i */
                pthread_mutex_lock(&mgr->mutex);
                TAILQ_FOREACH(node, &mgr->task_list, entries) {
                    DepEntry* dep = node->deps;
                    while (dep) {
                        if (strcmp(dep->dep_name, names[i]) == 0) {
                            /* node depends on task i — find node's index */
                            for (int k = 0; k < n; k++) {
                                if (strcmp(names[k], node->name) == 0 && indeg[k] > 0) {
                                    indeg[k]--;
                                    break;
                                }
                            }
                        }
                        dep = dep->next;
                    }
                }
                pthread_mutex_unlock(&mgr->mutex);
            }
        }
        if (!progress) {
            /* Cycle detected — start remaining in registration order */
            fprintf(stderr, "[task_manager] WARNING: dependency cycle detected. "
                    "Starting %d remaining tasks in registration order.\n",
                    n - started_count);
            for (int i = 0; i < n; i++) {
                if (!started[i]) {
                    if (task_manager_start_task(mgr, names[i]) != 0) failed++;
                    started[i] = true;
                }
            }
            break;
        }
    }

    return failed;
}

int task_manager_stop_all(TaskManager* mgr) {
    if (!mgr) return ERR_NOT_FOUND;
    int failed = 0;
    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        pthread_mutex_unlock(&mgr->mutex);
        if (task_manager_stop_task(mgr, node->name) != 0) failed++;
        pthread_mutex_lock(&mgr->mutex);
    }
    pthread_mutex_unlock(&mgr->mutex);
    return failed;
}

/* ── Queries ──────────────────────────────────────────── */

TaskBase* task_manager_get_task(TaskManager* mgr, const char* name) {
    if (!mgr || !name) return NULL;
    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node = find_node(mgr, name);
    TaskBase* task = node ? node->task : NULL;
    pthread_mutex_unlock(&mgr->mutex);
    return task;
}

TaskState task_manager_get_task_state(TaskManager* mgr, const char* name) {
    TaskBase* task = task_manager_get_task(mgr, name);
    return task ? task_get_state(task) : TASK_STATE_UNKNOWN;
}

const TaskStats* task_manager_get_task_stats(TaskManager* mgr, const char* name) {
    TaskBase* task = task_manager_get_task(mgr, name);
    return task ? task_get_stats(task) : NULL;
}

int task_manager_list_tasks(TaskManager* mgr, char names[][64], int max_count) {
    if (!mgr || !names || max_count <= 0) return 0;
    int count = 0;
    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        if (count >= max_count) break;
        snprintf(names[count], 64, "%s", node->name);
        names[count][63] = '\0';
        count++;
    }
    pthread_mutex_unlock(&mgr->mutex);
    return count;
}

/* ── Monitor ──────────────────────────────────────────── */

int task_manager_start_monitor(TaskManager* mgr) {
    if (!mgr || mgr->is_running) return 0;
    mgr->is_running = true;
    int ret = pthread_create(&mgr->monitor_thread, NULL, monitor_thread_fn, mgr);
    if (ret != 0) {
        mgr->is_running = false;
        return ERR_NOT_FOUND;
    }
    return 0;
}

void task_manager_stop_monitor(TaskManager* mgr) {
    if (!mgr || !mgr->is_running) return;
    mgr->is_running = false;
    pthread_join(mgr->monitor_thread, NULL);
}

/* ── Callback ─────────────────────────────────────────── */

void task_manager_set_event_callback(TaskManager* mgr, TaskEventCallback cb, void* user_data) {
    if (!mgr) return;
    mgr->event_callback  = cb;
    mgr->event_user_data = user_data;
}

/* ── Health check ─────────────────────────────────────── */

int task_manager_health_check(TaskManager* mgr) {
    if (!mgr) return 0;
    int unhealthy = 0;
    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        TaskBase* task = node->task;
        if (!task) continue;
        bool ok = true;
        if (task->vtable && task->vtable->health_check) {
            ok = task->vtable->health_check(task);
        } else {
            TaskState s = task_get_state(task);
            ok = (s != TASK_STATE_ERROR);
        }
        if (!ok) unhealthy++;
    }
    pthread_mutex_unlock(&mgr->mutex);
    return unhealthy;
}

/* ── Stats ────────────────────────────────────────────── */

void task_manager_get_stats(TaskManager* mgr, uint32_t* total_tasks,
                            uint32_t* running_tasks, uint32_t* error_tasks) {
    if (!mgr) return;
    uint32_t total = 0, running = 0, error = 0;
    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node;
    TAILQ_FOREACH(node, &mgr->task_list, entries) {
        total++;
        TaskState s = task_get_state(node->task);
        if (s == TASK_STATE_RUNNING) running++;
        if (s == TASK_STATE_ERROR)   error++;
    }
    pthread_mutex_unlock(&mgr->mutex);
    if (total_tasks)   *total_tasks   = total;
    if (running_tasks) *running_tasks = running;
    if (error_tasks)   *error_tasks   = error;
}

/* ── Dependency ───────────────────────────────────────── */

int task_manager_add_dependency(TaskManager* mgr,
                                const char* task_name, const char* dep_name) {
    if (!mgr || !task_name || !dep_name) return ERR_NOT_FOUND;

    pthread_mutex_lock(&mgr->mutex);
    TaskNode* node = find_node(mgr, task_name);
    if (!node) { pthread_mutex_unlock(&mgr->mutex); return ERR_NOT_FOUND; }

    DepEntry* dep = (DepEntry*)calloc(1, sizeof(DepEntry));
    if (!dep) { pthread_mutex_unlock(&mgr->mutex); return ERR_NOT_FOUND; }
    snprintf(dep->dep_name, sizeof(dep->dep_name), "%s", dep_name);
    dep->next  = node->deps;
    node->deps = dep;
    pthread_mutex_unlock(&mgr->mutex);
    return 0;
}
