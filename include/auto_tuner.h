#ifndef AUTO_TUNER_H
#define AUTO_TUNER_H

/**
 * @file auto_tuner.h
 * @brief 自动频率调优器 — 基于延迟/CPU 反馈的自适应频率调整
 *
 * 每个节点注册后，调优器周期性采样其延迟和心跳数据，
 * 根据目标策略自动调整 max_frequency_hz：
 *
 * 策略：
 *   - TUNE_CONSERVATIVE: 延迟 > 阈值 → 降频，延迟 << 阈值 → 升频（保守）
 *   - TUNE_AGGRESSIVE:   尽可能升频，仅在超限时降频
 *   - TUNE_FIXED:        不调整，仅记录
 *
 * 用法：
 *   auto_tuner_init(NULL);
 *   auto_tuner_register("perception", TUNE_CONSERVATIVE, 10.0, 100.0);
 *   // 在每个周期后调用
 *   double new_freq = auto_tuner_tick("perception", scheduler, task_id);
 */

#include "scheduler.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 调优策略 ────────────────────────────────────────────── */

typedef enum {
    TUNE_CONSERVATIVE = 0,  /**< 保守：优先稳定，延迟超标才降频 */
    TUNE_AGGRESSIVE   = 1,  /**< 激进：尽可能高频，逼近上限 */
    TUNE_FIXED        = 2,  /**< 固定：不调整，仅观测 */
} TuneStrategy;

/* ── 调优器配置 ──────────────────────────────────────────── */

typedef struct {
    uint64_t  sample_interval_us;  /**< 采样间隔（默认 5s） */
    double    latency_target_us;   /**< 目标延迟上限（默认 50000us = 50ms） */
    double    freq_step_pct;       /**< 每次调整步长（默认 10% = 0.1） */
    double    min_freq_hz;         /**< 全局最低频率（默认 1.0） */
    double    max_freq_hz;         /**< 全局最高频率（默认 200.0） */
    uint64_t  cooldown_us;         /**< 调整冷却时间（默认 10s），避免振荡 */
} AutoTunerConfig;

#define AUTO_TUNER_CONFIG_DEFAULT \
    { .sample_interval_us = 5000000ULL, \
      .latency_target_us  = 50000ULL,   \
      .freq_step_pct      = 0.10,       \
      .min_freq_hz        = 1.0,        \
      .max_freq_hz        = 200.0,      \
      .cooldown_us        = 10000000ULL }

/* ── API ─────────────────────────────────────────────────── */

/** 初始化全局调优器 */
void auto_tuner_init(const AutoTunerConfig* config);

/**
 * 注册一个可调优节点。
 * @param name      节点名称
 * @param strategy  调优策略
 * @param init_hz   初始频率
 * @param max_hz    节点允许的最大频率
 */
int auto_tuner_register(const char* name, TuneStrategy strategy,
                        double init_hz, double max_hz);

/**
 * 执行一次调优 tick（每个周期调用）。
 * 内部自动做采样间隔判断，仅在需要时计算新频率。
 *
 * @param name      节点名称
 * @param sched     调度器（用于读取 LatencyTracker）
 * @param task_id   调度器任务 ID
 * @return 当前建议频率（如果无变化则返回上次值）
 */
double auto_tuner_tick(const char* name, Scheduler* sched, int task_id);

/**
 * 获取节点的调优快照（用于 /api/debug/autotune）。
 */
typedef struct {
    char          name[32];
    TuneStrategy  strategy;
    double        current_freq_hz;
    double        last_latency_us;
    double        avg_latency_us;
    uint64_t      adjustments;       /**< 累计调整次数 */
    uint64_t      last_adjust_us;    /**< 上次调整时间 */
    double        min_observed_hz;   /**< 历史最低频率 */
    double        max_observed_hz;   /**< 历史最高频率 */
} AutoTuneSnapshot;

#define AUTO_TUNE_MAX_NODES 16

int auto_tuner_get_all(AutoTuneSnapshot* out, int max_count);

#ifdef __cplusplus
}
#endif

#endif /* AUTO_TUNER_H */