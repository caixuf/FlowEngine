/**
 * health.c — 节点健康上报系统实现
 *
 * 线程安全的环形健康记录，每个节点独立追踪：
 *   - 心跳时间戳 → 检测节点挂死
 *   - 延迟统计 (avg/p99/max) → 检测性能退化
 *   - 错误计数 + 最近错误 → 快速定位问题
 *   - 卡顿计数 → 检测调度抖动
 */

#include "health.h"
#include "clock_service.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

/* ── 内部状态 ────────────────────────────────────────────── */

#define HL_HISTORY_SIZE 256

typedef struct {
    char            name[32];
    HealthCapability caps;
    uint64_t        last_heartbeat_us;
    uint64_t        error_count;
    uint64_t        total_cycles;
    uint64_t        latency_history[HL_HISTORY_SIZE];
    uint32_t        latency_head;
    uint32_t        latency_count;
    uint64_t        latency_total;
    uint64_t        max_latency_us;
    uint64_t        stall_count;
    char            last_error[128];
    uint64_t        last_error_time_us;
    bool            registered;
} HealthEntry;

static struct {
    HealthEntry     entries[HEALTH_MAX_NODES];
    int             count;
    pthread_mutex_t mutex;
    bool            initialized;
} g_health = { .mutex = PTHREAD_MUTEX_INITIALIZER };

/* ── 内在函数 ────────────────────────────────────────────── */

static HealthEntry* find_entry(const char* name) {
    for (int i = 0; i < g_health.count; i++) {
        if (strcmp(g_health.entries[i].name, name) == 0)
            return &g_health.entries[i];
    }
    return NULL;
}

static uint64_t percentile(const uint64_t* sorted, uint32_t n, double p) {
    if (n == 0) return 0;
    uint32_t idx = (uint32_t)(n * p);
    if (idx >= n) idx = n - 1;
    return sorted[idx];
}

static int cmp_u64(const void* a, const void* b) {
    uint64_t ua = *(const uint64_t*)a;
    uint64_t ub = *(const uint64_t*)b;
    return (ua > ub) - (ua < ub);
}

/* ── 公开 API ────────────────────────────────────────────── */

void health_init(void) {
    if (g_health.initialized) return;
    memset(&g_health, 0, sizeof(g_health));
    pthread_mutex_init(&g_health.mutex, NULL);
    g_health.initialized = true;
}

int health_register(const char* name, HealthCapability caps) {
    if (!name || !g_health.initialized) return -1;

    pthread_mutex_lock(&g_health.mutex);
    if (find_entry(name)) {
        pthread_mutex_unlock(&g_health.mutex);
        return 0;  /* 已注册 */
    }
    if (g_health.count >= HEALTH_MAX_NODES) {
        pthread_mutex_unlock(&g_health.mutex);
        return -1;
    }

    HealthEntry* e = &g_health.entries[g_health.count++];
    memset(e, 0, sizeof(*e));
    snprintf(e->name, sizeof(e->name), "%s", name);
    e->caps = caps;
    e->registered = true;
    pthread_mutex_unlock(&g_health.mutex);
    return 0;
}

void health_heartbeat(const char* name) {
    if (!name) return;
    pthread_mutex_lock(&g_health.mutex);
    HealthEntry* e = find_entry(name);
    if (e) {
        e->last_heartbeat_us = clock_now_us();
        e->total_cycles++;
    }
    pthread_mutex_unlock(&g_health.mutex);
}

void health_record_latency(const char* name, uint64_t latency_us) {
    if (!name) return;
    pthread_mutex_lock(&g_health.mutex);
    HealthEntry* e = find_entry(name);
    if (e) {
        e->latency_history[e->latency_head] = latency_us;
        e->latency_head = (e->latency_head + 1) % HL_HISTORY_SIZE;
        if (e->latency_count < HL_HISTORY_SIZE) e->latency_count++;
        e->latency_total += latency_us;
        if (latency_us > e->max_latency_us) e->max_latency_us = latency_us;
    }
    pthread_mutex_unlock(&g_health.mutex);
}

void health_record_error(const char* name, const char* err_msg) {
    if (!name) return;
    pthread_mutex_lock(&g_health.mutex);
    HealthEntry* e = find_entry(name);
    if (e) {
        e->error_count++;
        e->last_error_time_us = clock_now_us();
        if (err_msg) snprintf(e->last_error, sizeof(e->last_error), "%s", err_msg);
    }
    pthread_mutex_unlock(&g_health.mutex);
}

void health_record_stall(const char* name) {
    if (!name) return;
    pthread_mutex_lock(&g_health.mutex);
    HealthEntry* e = find_entry(name);
    if (e) e->stall_count++;
    pthread_mutex_unlock(&g_health.mutex);
}

int health_get_all(HealthSnapshot* out, int max_count) {
    if (!out || max_count <= 0) return 0;

    pthread_mutex_lock(&g_health.mutex);
    int n = g_health.count;
    if (n > max_count) n = max_count;

    uint64_t now = clock_now_us();
    for (int i = 0; i < n; i++) {
        HealthEntry* e = &g_health.entries[i];
        HealthSnapshot* s = &out[i];
        memset(s, 0, sizeof(*s));
        snprintf(s->name, sizeof(s->name), "%s", e->name);
        s->caps = e->caps;
        s->last_heartbeat_us = e->last_heartbeat_us;
        s->error_count = e->error_count;
        s->total_cycles = e->total_cycles;
        s->max_latency_us = e->max_latency_us;
        s->stall_count = e->stall_count;

        /* 延迟统计 */
        if (e->latency_count > 0) {
            s->avg_latency_us = e->latency_total / e->latency_count;

            uint64_t sorted[HL_HISTORY_SIZE];
            uint32_t start = (e->latency_head + HL_HISTORY_SIZE - e->latency_count)
                           % HL_HISTORY_SIZE;
            for (uint32_t j = 0; j < e->latency_count; j++) {
                sorted[j] = e->latency_history[(start + j) % HL_HISTORY_SIZE];
            }
            qsort(sorted, e->latency_count, sizeof(uint64_t), cmp_u64);
            s->p99_latency_us = percentile(sorted, e->latency_count, 0.99);
        }

        /* 状态判定 */
        if (e->error_count > 0 &&
            now - e->last_error_time_us < 5000000ULL) {
            s->status = HEALTH_ERROR;
        } else if (e->last_heartbeat_us == 0 ||
                   now - e->last_heartbeat_us > 5000000ULL) {
            s->status = HEALTH_STALE;
        } else if (s->avg_latency_us > 50000ULL) {
            s->status = HEALTH_DEGRADED;
        } else {
            s->status = HEALTH_OK;
        }

        snprintf(s->last_error, sizeof(s->last_error), "%s", e->last_error);
        s->last_error_time_us = e->last_error_time_us;
    }
    pthread_mutex_unlock(&g_health.mutex);
    return n;
}

int health_get(const char* name, HealthSnapshot* out) {
    if (!name || !out) return -1;
    HealthSnapshot snap[1];
    if (health_get_all(snap, 1) <= 0) return -1;
    if (strcmp(snap[0].name, name) != 0) return -1;
    *out = snap[0];
    return 0;
}

bool health_is_all_ok(void) {
    HealthSnapshot snaps[HEALTH_MAX_NODES];
    int n = health_get_all(snaps, HEALTH_MAX_NODES);
    for (int i = 0; i < n; i++) {
        if (snaps[i].status != HEALTH_OK) return false;
    }
    return true;
}