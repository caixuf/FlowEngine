/**
 * degrade_ladder.c — 降级阶梯完整实现
 *
 * 三层消费（§11.2）：
 *   1. Supervisor — 监控节点心跳超时，自动递进等级
 *   2. 各层策略 — degrade_layer_action() 输出该层动作
 *   3. 原因码 + 时间戳 — 谁触发的、为什么、何时
 *
 * 时间单位均为毫秒。
 */

#include "degrade_ladder.h"
#include <stdatomic.h>
#include <string.h>
#include <stdio.h>

/* ══════════════════════════════════════════════════════════ */
/*  全局状态                                                     */
/* ══════════════════════════════════════════════════════════ */

static DegradeState g_degrade = {
    .degrade_level          = 0,
    .degrade_reason         = 0,
    .degrade_timestamp_ms   = 0,
    .l1_disable_lane_change = 0,
    .l1_speed_limit         = 0.0,
    .l1_safety_margin       = 1.0,
};

/* ══════════════════════════════════════════════════════════ */
/*  Supervisor 内部状态                                          */
/* ══════════════════════════════════════════════════════════ */

typedef struct {
    char    name[DEGRADE_NODE_NAME_LEN];
    int64_t last_heartbeat_ms;  /* 上次心跳时间 (ms), 0=未注册 */
} DegradeNodeEntry;

static struct {
    DegradeNodeEntry nodes[DEGRADE_MAX_NODES];
    int              node_count;
} g_supervisor;

/* ══════════════════════════════════════════════════════════ */
/*  全局状态 API                                                  */
/* ══════════════════════════════════════════════════════════ */

DegradeState* degrade_global_state(void) {
    return &g_degrade;
}

void degrade_set_level(int level, int reason) {
    /* 不降级（方向错误，L0 是最高） */
    if (level < DEGRADE_L0 || level > DEGRADE_L3) return;
    atomic_store(&g_degrade.degrade_level, level);
    atomic_store(&g_degrade.degrade_reason, reason);
    atomic_store(&g_degrade.degrade_timestamp_ms, 0); /* 由调用方填充 */

    /* 根据等级和原因设置 L1 参数 */
    if (level >= DEGRADE_L1) {
        atomic_store(&g_degrade.l1_disable_lane_change, 1);
    }
    if (level >= DEGRADE_L2) {
        atomic_store(&g_degrade.l1_speed_limit, 3.0);  /* 3 m/s crawl */
    }
    if (level >= DEGRADE_L3) {
        atomic_store(&g_degrade.l1_speed_limit, 0.0);  /* 立即停 */
    }
}

void degrade_clear(void) {
    atomic_store(&g_degrade.degrade_level, 0);
    atomic_store(&g_degrade.degrade_reason, 0);
    atomic_store(&g_degrade.degrade_timestamp_ms, 0);
    atomic_store(&g_degrade.l1_disable_lane_change, 0);
    atomic_store(&g_degrade.l1_speed_limit, 0.0);
    atomic_store(&g_degrade.l1_safety_margin, 1.0);

    /* 清 supervisor 心跳记录 */
    for (int i = 0; i < g_supervisor.node_count; i++)
        g_supervisor.nodes[i].last_heartbeat_ms = 0;
}

/* ══════════════════════════════════════════════════════════ */
/*  各层消费 API                                                  */
/* ══════════════════════════════════════════════════════════ */

DegradeAction degrade_layer_action(void) {
    int level  = (int)atomic_load(&g_degrade.degrade_level);
    int reason = (int)atomic_load(&g_degrade.degrade_reason);
    double speed_limit = (double)atomic_load(&g_degrade.l1_speed_limit);
    double safety_margin = (double)atomic_load(&g_degrade.l1_safety_margin);
    int disable_lc = (int)atomic_load(&g_degrade.l1_disable_lane_change);

    DegradeAction act;
    memset(&act, 0, sizeof(act));
    act.degrade_level  = level;
    act.degrade_reason = reason;
    act.safety_margin  = (safety_margin > 0.1) ? safety_margin : 1.0;

    switch (level) {
    case DEGRADE_L0:
        /* 全功能——无限制 */
        break;

    case DEGRADE_L1:
        /* 降级：禁变道、限速、加大安全余量 */
        act.disable_lane_change = (disable_lc != 0);
        act.speed_limit = speed_limit;
        act.safety_margin = (safety_margin > 1.0) ? safety_margin : 1.5;
        break;

    case DEGRADE_L2:
        /* MRM：车道内减速停车 */
        act.disable_lane_change = true;
        act.mrm_stop = true;
        act.speed_limit = (speed_limit > 0.0) ? speed_limit : 3.0;
        act.safety_margin = 2.0;
        break;

    case DEGRADE_L3:
        /* 立即停 */
        act.disable_lane_change = true;
        act.immediate_stop = true;
        act.mrm_stop = true;
        act.speed_limit = 0.0;
        act.safety_margin = 3.0;
        break;
    }

    return act;
}

/* ══════════════════════════════════════════════════════════ */
/*  Supervisor API — 健康监控 + 自动递进                          */
/* ══════════════════════════════════════════════════════════ */

void degrade_supervisor_record_heartbeat(const char* node_name, int64_t now_ms) {
    if (!node_name) return;

    /* 找已有记录 */
    int idx = -1;
    for (int i = 0; i < g_supervisor.node_count; i++) {
        if (strncmp(g_supervisor.nodes[i].name, node_name,
                    DEGRADE_NODE_NAME_LEN) == 0) {
            idx = i;
            break;
        }
    }

    /* 新节点注册 */
    if (idx < 0) {
        if (g_supervisor.node_count >= DEGRADE_MAX_NODES) return;
        idx = g_supervisor.node_count++;
        snprintf(g_supervisor.nodes[idx].name, sizeof(g_supervisor.nodes[idx].name),
                 "%s", node_name);
    }

    g_supervisor.nodes[idx].last_heartbeat_ms = now_ms;
}

void degrade_supervisor_tick(int64_t now_ms) {
    /* 统计各节点超时状态 */
    int timeout_count = 0;
    int timeout_1s_count = 0;
    int64_t oldest_heartbeat = now_ms;

    for (int i = 0; i < g_supervisor.node_count; i++) {
        int64_t hb = g_supervisor.nodes[i].last_heartbeat_ms;
        if (hb == 0) continue;  /* 未注册，不视为超时 */

        int64_t age = now_ms - hb;
        if (age < 0) age = 0;

        if (age < oldest_heartbeat) oldest_heartbeat = age;

        if (age > 500) timeout_count++;       /* >500ms */
        if (age > 2000) timeout_1s_count++;   /* >2000ms */
    }

    /* 当前等级 */
    int current = (int)atomic_load(&g_degrade.degrade_level);

    /* 递进策略 */
    if (timeout_1s_count >= 2) {
        /* 多节点超时 >2s → L3 */
        if (current < DEGRADE_L3) {
            g_degrade.degrade_timestamp_ms = now_ms;
            degrade_set_level(DEGRADE_L3, DEGRADE_REASON_PLANNING_TO);
        }
    } else if (timeout_1s_count >= 1) {
        /* 单节点超时 >2s → L2 */
        if (current < DEGRADE_L2) {
            g_degrade.degrade_timestamp_ms = now_ms;
            int reason = (g_supervisor.nodes[0].last_heartbeat_ms < now_ms - 2000)
                         ? DEGRADE_REASON_PLANNING_TO
                         : DEGRADE_REASON_CONTROL_TO;
            degrade_set_level(DEGRADE_L2, reason);
        }
    } else if (timeout_count >= 2) {
        /* 多节点超时 >500ms → L2 */
        if (current < DEGRADE_L2) {
            g_degrade.degrade_timestamp_ms = now_ms;
            degrade_set_level(DEGRADE_L2, DEGRADE_REASON_PLANNING_TO);
        }
    } else if (timeout_count >= 1) {
        /* 单节点超时 >500ms → L1 */
        if (current < DEGRADE_L1) {
            g_degrade.degrade_timestamp_ms = now_ms;
            int reason = (g_supervisor.nodes[0].last_heartbeat_ms < now_ms - 500)
                         ? DEGRADE_REASON_PLANNING_TO
                         : DEGRADE_REASON_CONTROL_TO;
            degrade_set_level(DEGRADE_L1, reason);
        }
    }
}

int degrade_supervisor_summary(char* buf, int buf_size) {
    if (!buf || buf_size <= 0) return -1;

    int level  = (int)atomic_load(&g_degrade.degrade_level);
    int reason = (int)atomic_load(&g_degrade.degrade_reason);
    const char* reason_str = "none";
    switch (reason) {
        case DEGRADE_REASON_PLANNING_TO:  reason_str = "planning_timeout"; break;
        case DEGRADE_REASON_CONTROL_TO:   reason_str = "control_timeout";  break;
        case DEGRADE_REASON_FUSION_TO:    reason_str = "fusion_timeout";   break;
        case DEGRADE_REASON_SENSOR_TO:    reason_str = "sensor_timeout";   break;
        case DEGRADE_REASON_LARGE_CTE:    reason_str = "large_cte";        break;
        case DEGRADE_REASON_COLLISION:    reason_str = "collision_risk";   break;
        case DEGRADE_REASON_LOCALIZATION: reason_str = "localization_lost";break;
        case DEGRADE_REASON_MANUAL:       reason_str = "manual";           break;
        case DEGRADE_REASON_HEARTBEAT:    reason_str = "heartbeat_lost";   break;
    }

    int pos = snprintf(buf, buf_size,
        "degrade: level=%d reason=%s nodes=%d",
        level, reason_str, g_supervisor.node_count);

    for (int i = 0; i < g_supervisor.node_count && pos < buf_size - 30; i++) {
        pos += snprintf(buf + pos, buf_size - pos,
                        " %s=%lldms",
                        g_supervisor.nodes[i].name,
                        (long long)(g_supervisor.nodes[i].last_heartbeat_ms > 0
                                    ? g_supervisor.nodes[i].last_heartbeat_ms : 0));
    }

    return 0;
}
