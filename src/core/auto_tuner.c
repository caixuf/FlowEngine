/**
 * auto_tuner.c — 自动频率调优器实现
 *
 * 基于反馈控制的自适应频率调整：
 *   - 读取 scheduler 的 LatencyTracker 获取延迟数据
 *   - 对比目标延迟阈值，决定升频/降频/保持
 *   - 内置冷却期防止频繁振荡
 *   - 保守策略：P99 延迟 > 目标 → 降频 20%；P99 < 目标/2 → 升频 10%
 *   - 激进策略：P50 延迟 < 目标/3 → 升频 20%；P50 > 目标 → 降频 30%
 */

#include "auto_tuner.h"
#include "clock_service.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

/* ── 内部状态 ────────────────────────────────────────────── */

typedef struct {
    char          name[32];
    TuneStrategy  strategy;
    double        current_freq_hz;
    double        max_node_hz;
    double        last_latency_us;
    double        avg_latency_us;
    uint64_t      adjustments;
    uint64_t      last_adjust_us;
    uint64_t      last_sample_us;
    double        min_observed_hz;
    double        max_observed_hz;
    bool          registered;
} TuneEntry;

static struct {
    AutoTunerConfig config;
    TuneEntry       entries[AUTO_TUNE_MAX_NODES];
    int             count;
    pthread_mutex_t mutex;
    bool            initialized;
} g_tuner = { .mutex = PTHREAD_MUTEX_INITIALIZER };

/* ── 内在函数 ────────────────────────────────────────────── */

static TuneEntry* find_entry(const char* name) {
    for (int i = 0; i < g_tuner.count; i++) {
        if (strcmp(g_tuner.entries[i].name, name) == 0)
            return &g_tuner.entries[i];
    }
    return NULL;
}

static double clamp_freq(double val, double lo, double hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ── 公开 API ────────────────────────────────────────────── */

void auto_tuner_init(const AutoTunerConfig* config) {
    if (g_tuner.initialized) return;
    memset(&g_tuner, 0, sizeof(g_tuner));
    pthread_mutex_init(&g_tuner.mutex, NULL);
    g_tuner.config = config ? *config : (AutoTunerConfig)AUTO_TUNER_CONFIG_DEFAULT;
    g_tuner.initialized = true;
}

int auto_tuner_register(const char* name, TuneStrategy strategy,
                        double init_hz, double max_hz) {
    if (!name || !g_tuner.initialized) return -1;

    pthread_mutex_lock(&g_tuner.mutex);
    if (find_entry(name)) {
        pthread_mutex_unlock(&g_tuner.mutex);
        return 0;
    }
    if (g_tuner.count >= AUTO_TUNE_MAX_NODES) {
        pthread_mutex_unlock(&g_tuner.mutex);
        return -1;
    }

    TuneEntry* e = &g_tuner.entries[g_tuner.count++];
    memset(e, 0, sizeof(*e));
    snprintf(e->name, sizeof(e->name), "%s", name);
    e->strategy = strategy;
    e->current_freq_hz = init_hz;
    e->max_node_hz = max_hz;
    e->min_observed_hz = init_hz;
    e->max_observed_hz = init_hz;
    e->registered = true;
    pthread_mutex_unlock(&g_tuner.mutex);
    return 0;
}

double auto_tuner_tick(const char* name, Scheduler* sched, int task_id) {
    if (!name || !sched || !g_tuner.initialized) return -1.0;

    pthread_mutex_lock(&g_tuner.mutex);
    TuneEntry* e = find_entry(name);
    if (!e || e->strategy == TUNE_FIXED) {
        pthread_mutex_unlock(&g_tuner.mutex);
        return e ? e->current_freq_hz : -1.0;
    }

    uint64_t now = clock_now_us();

    /* 采样间隔未到，跳过 */
    if (now - e->last_sample_us < g_tuner.config.sample_interval_us) {
        pthread_mutex_unlock(&g_tuner.mutex);
        return e->current_freq_hz;
    }
    e->last_sample_us = now;

    /* 读取延迟数据 */
    LatencyTracker* lt = scheduler_get_latency(sched, task_id);
    if (!lt) {
        pthread_mutex_unlock(&g_tuner.mutex);
        return e->current_freq_hz;
    }

    LatencyStats stats = latency_tracker_stats(lt);
    double p50 = (double)stats.p50_us;
    double p99 = (double)stats.p99_us;
    double target = (double)g_tuner.config.latency_target_us;

    double new_freq = e->current_freq_hz;
    bool adjusted = false;

    switch (e->strategy) {
    case TUNE_CONSERVATIVE: {
        /* P99 > target → 降频 20%；P99 < target/2 且 cooldown 已过 → 升频 10% */
        if (p99 > target && e->current_freq_hz > g_tuner.config.min_freq_hz) {
            new_freq = e->current_freq_hz * 0.80;
            adjusted = true;
        } else if (p99 < target * 0.5 &&
                   now - e->last_adjust_us > g_tuner.config.cooldown_us) {
            new_freq = e->current_freq_hz * 1.10;
            adjusted = true;
        }
        break;
    }
    case TUNE_AGGRESSIVE: {
        /* P50 > target → 降频 30%；P50 < target/3 → 升频 20% */
        if (p50 > target && e->current_freq_hz > g_tuner.config.min_freq_hz) {
            new_freq = e->current_freq_hz * 0.70;
            adjusted = true;
        } else if (p50 < target * 0.33 &&
                   now - e->last_adjust_us > g_tuner.config.cooldown_us) {
            new_freq = e->current_freq_hz * 1.20;
            adjusted = true;
        }
        break;
    }
    default:
        break;
    }

    new_freq = clamp_freq(new_freq,
                          g_tuner.config.min_freq_hz,
                          e->max_node_hz < g_tuner.config.max_freq_hz
                          ? e->max_node_hz : g_tuner.config.max_freq_hz);

    if (adjusted && new_freq != e->current_freq_hz) {
        double old_freq = e->current_freq_hz;
        e->current_freq_hz = new_freq;
        e->adjustments++;
        e->last_adjust_us = now;

        if (new_freq < e->min_observed_hz) e->min_observed_hz = new_freq;
        if (new_freq > e->max_observed_hz) e->max_observed_hz = new_freq;

        /* 同步到 scheduler */
        RateControl* rc = scheduler_get_rate_control(sched, task_id);
        if (rc) {
            rate_control_init(rc, new_freq);
        }

        printf("[auto_tuner] %s: %.1f Hz → %.1f Hz (p50=%.0fus p99=%.0fus target=%.0fus)\n",
               e->name, old_freq, new_freq, p50, p99, target);
    }

    e->last_latency_us = p99;
    e->avg_latency_us = (double)stats.avg_us;
    pthread_mutex_unlock(&g_tuner.mutex);
    return e->current_freq_hz;
}

int auto_tuner_get_all(AutoTuneSnapshot* out, int max_count) {
    if (!out || max_count <= 0) return 0;

    pthread_mutex_lock(&g_tuner.mutex);
    int n = g_tuner.count;
    if (n > max_count) n = max_count;

    for (int i = 0; i < n; i++) {
        TuneEntry* e = &g_tuner.entries[i];
        AutoTuneSnapshot* s = &out[i];
        memset(s, 0, sizeof(*s));
        snprintf(s->name, sizeof(s->name), "%s", e->name);
        s->strategy = e->strategy;
        s->current_freq_hz = e->current_freq_hz;
        s->last_latency_us = e->last_latency_us;
        s->avg_latency_us = e->avg_latency_us;
        s->adjustments = e->adjustments;
        s->last_adjust_us = e->last_adjust_us;
        s->min_observed_hz = e->min_observed_hz;
        s->max_observed_hz = e->max_observed_hz;
    }
    pthread_mutex_unlock(&g_tuner.mutex);
    return n;
}