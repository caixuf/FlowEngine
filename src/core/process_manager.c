#include "process_manager.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <dlfcn.h>
#include <unistd.h>

/* ── Thread entry: runs a loaded plugin ──────────────── */

static void* process_thread_fn(void* arg) {
    ProcessNode* node = (ProcessNode*)arg;
    if (!node || !node->interface) return NULL;

    node->interface->start();
    return NULL;
}

/* ── Monitor thread: health check & auto-restart ──────── */

static void* monitor_thread_fn(void* arg) {
    ProcessManager* mgr = (ProcessManager*)arg;

    while (mgr->is_running) {
        sleep(5);
        if (!mgr->is_running) break;

        pthread_mutex_lock(&mgr->mutex);
        ProcessNode* node;
        TAILQ_FOREACH(node, &mgr->process_list, entries) {
            if (!node->interface) continue;
            if (!node->is_running) continue;

            bool ok = node->interface->health_check
                      ? node->interface->health_check()
                      : true;
            if (!ok) {
                char msg[256];
                snprintf(msg, sizeof(msg),
                         "[ProcessManager] Process '%s' unhealthy, restarting...", node->name);
                if (mgr->log_callback)
                    mgr->log_callback(LOG_LEVEL_WARN, msg);

                node->interface->stop();
                pthread_join(node->thread, NULL);
                node->is_running = false;
                node->restart_count++;

                /* Restart */
                if (node->should_restart) {
                    node->interface->start();
                    pthread_create(&node->thread, NULL, process_thread_fn, node);
                    node->is_running = true;
                }
            }
        }
        pthread_mutex_unlock(&mgr->mutex);
    }
    return NULL;
}

/* ── Lifecycle ────────────────────────────────────────── */

ProcessManager* process_manager_create(LogCallback log_callback) {
    ProcessManager* mgr = (ProcessManager*)calloc(1, sizeof(ProcessManager));
    if (!mgr) return NULL;

    TAILQ_INIT(&mgr->process_list);
    pthread_mutex_init(&mgr->mutex, NULL);
    mgr->log_callback = log_callback;
    mgr->is_running   = false;
    return mgr;
}

void process_manager_destroy(ProcessManager* mgr) {
    if (!mgr) return;

    process_manager_stop_monitor(mgr);
    process_manager_stop_all(mgr);

    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node;
    while ((node = TAILQ_FIRST(&mgr->process_list)) != NULL) {
        TAILQ_REMOVE(&mgr->process_list, node, entries);
        if (node->interface && node->interface->cleanup)
            node->interface->cleanup();
        if (node->lib_handle)
            dlclose(node->lib_handle);
        free(node);
    }
    pthread_mutex_unlock(&mgr->mutex);
    pthread_mutex_destroy(&mgr->mutex);
    free(mgr);
}

/* ── Plugin loading ───────────────────────────────────── */

int process_manager_load_plugin(ProcessManager* mgr,
                               const char* name,
                               const char* library_path,
                               const char* config_data) {
    if (!mgr || !name || !library_path) return -1;

    void* handle = dlopen(library_path, RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        if (mgr->log_callback) {
            char msg[512];
            snprintf(msg, sizeof(msg), "dlopen failed for '%s': %s", library_path, dlerror());
            mgr->log_callback(LOG_LEVEL_ERROR, msg);
        }
        return -1;
    }

    /* Look up get_process_interface symbol */
    typedef ProcessInterface* (*GetIfFn)(void);
    GetIfFn get_if = (GetIfFn)(uintptr_t)dlsym(handle, "get_process_interface");
    if (!get_if) {
        if (mgr->log_callback)
            mgr->log_callback(LOG_LEVEL_ERROR, "Symbol 'get_process_interface' not found");
        dlclose(handle);
        return -1;
    }

    ProcessInterface* iface = get_if();
    if (!iface) {
        if (mgr->log_callback)
            mgr->log_callback(LOG_LEVEL_ERROR, "get_process_interface returned NULL");
        dlclose(handle);
        return -1;
    }

    /* Initialize plugin */
    if (iface->initialize) {
        int ret = iface->initialize(config_data ? config_data : "", mgr->log_callback);
        if (ret != 0) {
            if (mgr->log_callback)
                mgr->log_callback(LOG_LEVEL_ERROR, "Plugin initialize() failed");
            dlclose(handle);
            return -1;
        }
    }

    ProcessNode* node = (ProcessNode*)calloc(1, sizeof(ProcessNode));
    if (!node) { dlclose(handle); return -1; }

    snprintf(node->name, sizeof(node->name), "%s", name);
    snprintf(node->library_path, sizeof(node->library_path), "%s", library_path);
    if (config_data)
        snprintf(node->config_data, sizeof(node->config_data), "%s", config_data);
    node->lib_handle    = handle;
    node->interface     = iface;
    node->is_running    = false;
    node->should_restart = true;

    pthread_mutex_lock(&mgr->mutex);
    TAILQ_INSERT_TAIL(&mgr->process_list, node, entries);
    pthread_mutex_unlock(&mgr->mutex);
    return 0;
}

/* ── Start / Stop / Restart ───────────────────────────── */

static ProcessNode* find_node(ProcessManager* mgr, const char* name) {
    ProcessNode* node;
    TAILQ_FOREACH(node, &mgr->process_list, entries) {
        if (strcmp(node->name, name) == 0) return node;
    }
    return NULL;
}

int process_manager_start_process(ProcessManager* mgr, const char* name) {
    if (!mgr || !name) return -1;

    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node = find_node(mgr, name);
    if (!node || node->is_running) {
        pthread_mutex_unlock(&mgr->mutex);
        return node ? 0 : -1;
    }

    int ret = pthread_create(&node->thread, NULL, process_thread_fn, node);
    if (ret == 0) node->is_running = true;
    pthread_mutex_unlock(&mgr->mutex);
    return ret == 0 ? 0 : -1;
}

int process_manager_stop_process(ProcessManager* mgr, const char* name) {
    if (!mgr || !name) return -1;

    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node = find_node(mgr, name);
    if (!node || !node->is_running) {
        pthread_mutex_unlock(&mgr->mutex);
        return 0;
    }
    if (node->interface && node->interface->stop)
        node->interface->stop();
    pthread_mutex_unlock(&mgr->mutex);

    pthread_join(node->thread, NULL);

    pthread_mutex_lock(&mgr->mutex);
    node->is_running = false;
    pthread_mutex_unlock(&mgr->mutex);
    return 0;
}

int process_manager_restart_process(ProcessManager* mgr, const char* name) {
    process_manager_stop_process(mgr, name);
    return process_manager_start_process(mgr, name);
}

int process_manager_start_all(ProcessManager* mgr) {
    if (!mgr) return -1;
    int failed = 0;
    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node;
    TAILQ_FOREACH(node, &mgr->process_list, entries) {
        pthread_mutex_unlock(&mgr->mutex);
        if (process_manager_start_process(mgr, node->name) != 0) failed++;
        pthread_mutex_lock(&mgr->mutex);
    }
    pthread_mutex_unlock(&mgr->mutex);
    return failed;
}

int process_manager_stop_all(ProcessManager* mgr) {
    if (!mgr) return -1;
    int failed = 0;
    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node;
    TAILQ_FOREACH(node, &mgr->process_list, entries) {
        pthread_mutex_unlock(&mgr->mutex);
        if (process_manager_stop_process(mgr, node->name) != 0) failed++;
        pthread_mutex_lock(&mgr->mutex);
    }
    pthread_mutex_unlock(&mgr->mutex);
    return failed;
}

/* ── State / Stats ────────────────────────────────────── */

ProcessState process_manager_get_process_state(ProcessManager* mgr, const char* name) {
    if (!mgr || !name) return PROCESS_STATE_UNKNOWN;
    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node = find_node(mgr, name);
    ProcessState s = PROCESS_STATE_UNKNOWN;
    if (node && node->interface && node->interface->get_state)
        s = node->interface->get_state();
    pthread_mutex_unlock(&mgr->mutex);
    return s;
}

const ProcessStats* process_manager_get_process_stats(ProcessManager* mgr, const char* name) {
    if (!mgr || !name) return NULL;
    pthread_mutex_lock(&mgr->mutex);
    ProcessNode* node = find_node(mgr, name);
    const ProcessStats* stats = NULL;
    if (node && node->interface && node->interface->get_stats)
        stats = node->interface->get_stats();
    pthread_mutex_unlock(&mgr->mutex);
    return stats;
}

/* ── Monitor ──────────────────────────────────────────── */

int process_manager_start_monitor(ProcessManager* mgr) {
    if (!mgr || mgr->is_running) return 0;
    mgr->is_running = true;
    int ret = pthread_create(&mgr->monitor_thread, NULL, monitor_thread_fn, mgr);
    if (ret != 0) { mgr->is_running = false; return -1; }
    return 0;
}

void process_manager_stop_monitor(ProcessManager* mgr) {
    if (!mgr || !mgr->is_running) return;
    mgr->is_running = false;
    pthread_join(mgr->monitor_thread, NULL);
}
