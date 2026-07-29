#ifndef DEGRADE_LADDER_H
#define DEGRADE_LADDER_H

/**
 * @file degrade_ladder.h
 * @brief 降级阶梯 — L0 全功能 → L1 降级 → L2 MRM → L3 立即停
 *
 * 三层消费（§11.2）：
 *   1. Supervisor — 监控节点健康，自动定级递进
 *   2. 各层策略接口 — degrade_layer_action() 告知当前层该做什么
 *   3. 原因码上报 — 谁触发了降级、为什么
 *
 * 使用方式：
 *   // Supervisor 端（monitor_node 或独立线程）：
 *   degrade_supervisor_record_heartbeat("planning_node");
 *   degrade_supervisor_record_heartbeat("control_node");
 *   degrade_supervisor_tick(clock_now_ms());  // 检查超时
 *
 *   // 消费者端（control_node / planning_node）：
 *   DegradeAction action = degrade_layer_action("control_node");
 *   if (action.disable_lane_change) { ... }
 *   if (action.speed_limit > 0) { target_speed = min(target_speed, action.speed_limit); }
 *   if (action.mrm_stop) { target_speed = 0; }
 *   if (action.immediate_stop) { emergency_brake(); }
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ══════════════════════════════════════════════════════════ */
/*  降级等级                                                     */
/* ══════════════════════════════════════════════════════════ */

#define DEGRADE_L0       0   /**< 全功能 */
#define DEGRADE_L1       1   /**< 降级：禁变道、限速、加大安全余量 */
#define DEGRADE_L2       2   /**< MRM：车道内减速停车 */
#define DEGRADE_L3       3   /**< 立即停 */

/* ══════════════════════════════════════════════════════════ */
/*  降级原因码                                                    */
/* ══════════════════════════════════════════════════════════ */

#define DEGRADE_REASON_NONE         0   /**< 无降级 */
#define DEGRADE_REASON_PLANNING_TO  1   /**< 规划节点超时 */
#define DEGRADE_REASON_CONTROL_TO   2   /**< 控制节点超时 */
#define DEGRADE_REASON_FUSION_TO    3   /**< 融合节点超时 */
#define DEGRADE_REASON_SENSOR_TO    4   /**< 传感器节点超时 */
#define DEGRADE_REASON_LARGE_CTE    5   /**< 横向误差持续超标 */
#define DEGRADE_REASON_COLLISION    6   /**< 碰撞风险 */
#define DEGRADE_REASON_LOCALIZATION 7   /**< 定位发散 */
#define DEGRADE_REASON_MANUAL       8   /**< 手动触发 */
#define DEGRADE_REASON_HEARTBEAT    9   /**< safety_control 数据超时 >1s 视为全线失联 */

/* ══════════════════════════════════════════════════════════ */
/*  降级状态（全局共享）                                          */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    volatile int32_t degrade_level;          /**< 当前降级等级 0-3 */
    volatile int32_t degrade_reason;         /**< 降级原因码 */
    volatile int64_t degrade_timestamp_ms;   /**< 降级时间戳 (ms) */
    /* L1 参数 */
    volatile int32_t l1_disable_lane_change; /**< 禁变道 */
    volatile double  l1_speed_limit;         /**< 速度限制 (m/s), 0=不限 */
    volatile double  l1_safety_margin;       /**< 安全余量倍率 */
} DegradeState;

/* ══════════════════════════════════════════════════════════ */
/*  各层消费输出                                                  */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    bool disable_lane_change;   /**< 禁变道 */
    bool mrm_stop;              /**< MRM 减速停车 */
    bool immediate_stop;        /**< 立即紧急停 */
    double speed_limit;         /**< 速度限制 (m/s), 0=不限 */
    double safety_margin;       /**< 安全余量倍率 (>=1.0) */
    int    degrade_level;       /**< 当前等级（调试用） */
    int    degrade_reason;      /**< 原因码 */
} DegradeAction;

/* ══════════════════════════════════════════════════════════ */
/*  全局状态 API                                                  */
/* ══════════════════════════════════════════════════════════ */

DegradeState* degrade_global_state(void);
void degrade_set_level(int level, int reason);
void degrade_clear(void);

/* ══════════════════════════════════════════════════════════ */
/*  各层消费 API — 输入 degrade_level，输出该层该做什么           */
/* ══════════════════════════════════════════════════════════ */

/**
 * 根据当前降级等级，输出该层应采取的动作。
 * 各节点独立调用，不阻塞。
 */
DegradeAction degrade_layer_action(void);

/* ══════════════════════════════════════════════════════════ */
/*  Supervisor API — 健康监控 + 自动递进                          */
/* ══════════════════════════════════════════════════════════ */

#define DEGRADE_MAX_NODES 16   /**< supervisor 最大监控节点数 */
#define DEGRADE_NODE_NAME_LEN 32

/**
 * 记录某节点的心跳。
 * @param node_name 节点名称，如 "planning_node"
 */
void degrade_supervisor_record_heartbeat(const char* node_name,
                                          int64_t now_ms);

/**
 * Supervisor 滴答：检查所有节点超时，自动递进等级。
 * 建议 20Hz 调用（与 control 同频）。
 *
 * 超时策略：
 *   - 单节点超时 > 500ms → L1
 *   - 单节点超时 > 2000ms → L2
 *   - 多节点同时超时 > 500ms → L2
 *   - 全链路超时 > 2000ms → L3
 *
 * @param now_ms 当前时间 (ms)
 */
void degrade_supervisor_tick(int64_t now_ms);

/**
 * 获取 supervisor 监控摘要（调试输出）。
 */
int degrade_supervisor_summary(char* buf, int buf_size);

#ifdef __cplusplus
}
#endif

#endif /* DEGRADE_LADDER_H */
