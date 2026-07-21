#ifndef HEALTH_H
#define HEALTH_H

/**
 * @file health.h
 * @brief 节点健康上报 — 心跳 + 错误计数 + 自诊断
 *
 * 每个节点在 init() 时注册，在 execute() 循环中上报心跳。
 * 健康数据通过 monitor_server 的 /api/debug/health 暴露，
 * 供 dashboard 和自动调优器消费。
 *
 * 用法：
 *   health_register("perception", HEALTH_CAP_PERCEPTION);
 *   while (running) {
 *       health_heartbeat("perception");
 *       // ... work ...
 *       health_record_latency("perception", lat_us);
 *   }
 */

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 节点能力标记 ────────────────────────────────────────── */

typedef enum {
    HEALTH_CAP_NONE       = 0,
    HEALTH_CAP_PERCEPTION = 1 << 0,  /**< 感知类节点 */
    HEALTH_CAP_CONTROL    = 1 << 1,  /**< 控制类节点 */
    HEALTH_CAP_PLANNING   = 1 << 2,  /**< 规划类节点 */
    HEALTH_CAP_SIMULATION = 1 << 3,  /**< 仿真类节点 */
    HEALTH_CAP_MONITOR    = 1 << 4,  /**< 监控类节点 */
    HEALTH_CAP_LATENCY_SENSITIVE = 1 << 5, /**< 延迟敏感 */
    HEALTH_CAP_SAFETY_CRITICAL   = 1 << 6, /**< 安全关键 */
} HealthCapability;

/* ── 健康状态 ────────────────────────────────────────────── */

typedef enum {
    HEALTH_OK       = 0,  /**< 正常 */
    HEALTH_DEGRADED = 1,  /**< 降级（延迟/丢包超标） */
    HEALTH_STALE    = 2,  /**< 无心跳（可能已挂） */
    HEALTH_ERROR    = 3,  /**< 错误 */
} HealthStatus;

/* ── 节点健康快照 ────────────────────────────────────────── */

#define HEALTH_MAX_NODES 32

typedef struct {
    char            name[32];
    HealthCapability caps;
    HealthStatus    status;
    uint64_t        last_heartbeat_us;   /**< 最后心跳单调时间 */
    uint64_t        error_count;         /**< 累计错误 */
    uint64_t        total_cycles;        /**< 累计执行周期 */
    uint64_t        avg_latency_us;      /**< 平均延迟 (us) */
    uint64_t        p99_latency_us;      /**< P99 延迟 (us) */
    uint64_t        max_latency_us;      /**< 最大延迟 (us) */
    uint64_t        stall_count;         /**< 卡顿次数（延迟 > 阈值） */
    double          cpu_pct;             /**< CPU 占用百分比 */
    char            last_error[128];     /**< 最近一次错误描述 */
    uint64_t        last_error_time_us;  /**< 最近错误时间 */
} HealthSnapshot;

/* ── API ─────────────────────────────────────────────────── */

/** 初始化健康系统 */
void health_init(void);

/** 注册一个节点 */
int health_register(const char* name, HealthCapability caps);

/** 上报心跳（每个周期调用一次） */
void health_heartbeat(const char* name);

/** 记录一次延迟样本 */
void health_record_latency(const char* name, uint64_t latency_us);

/** 记录一次错误 */
void health_record_error(const char* name, const char* err_msg);

/** 记录一次卡顿 */
void health_record_stall(const char* name);

/** 获取所有节点健康快照，返回节点数 */
int health_get_all(HealthSnapshot* out, int max_count);

/** 获取单个节点健康快照，返回 0=找到 */
int health_get(const char* name, HealthSnapshot* out);

/** 获取全局健康摘要（是否所有节点 OK） */
bool health_is_all_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_H */