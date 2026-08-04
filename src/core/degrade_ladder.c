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
#include <string.h>
#include <stdio.h>
#if defined(_WIN32)
#include <pthread.h>
#endif

/* ── 原子读写(跨编译器) ─────────────────────────────────────────────────
 * GCC 与 Clang 对 C11 <stdatomic.h> 的 atomic_load/atomic_store 处理不同:
 * GCC 展开为通用 __atomic_load/__atomic_store 内建,接受普通 volatile 标量;
 * Clang 展开为 __c11_atomic_* 内建,严格要求操作数是 _Atomic 限定类型,否则
 * 直接编译报错。本模块的共享状态字段是 volatile(且 degrade_ladder.h 被 extern
 * "C" 的 C++ TU 复用,无法把字段改成 C 专属的 _Atomic 关键字)。
 *
 * 故统一改用两编译器共有的 __atomic_* 内建(通用形式,支持含 double 在内的
 * 任意标量、且接受非 _Atomic 的 volatile 指针)。这正是 GCC stdatomic 宏在
 * Linux 上原本展开成的东西,codegen 与原路径一致,Linux 行为零变化。
 * __typeof__ 与语句表达式 ({...}) 为 GCC/Clang 共有扩展。 */
/* 临时量类型用 `*(ptr) + 0`:算术提升会去掉 volatile 限定,得到纯值类型,
 * 避免把 volatile 指针传给 __atomic_* 的非 volatile 输出/输入形参而告警。 */
#if defined(_WIN32)
static pthread_mutex_t g_degrade_atomic_mutex = PTHREAD_MUTEX_INITIALIZER;
#define FLOW_ATOMIC_STORE(ptr, val)                                       \
    do { pthread_mutex_lock(&g_degrade_atomic_mutex);                     \
         *(ptr) = (val);                                                  \
         pthread_mutex_unlock(&g_degrade_atomic_mutex); } while (0)
#define FLOW_ATOMIC_LOAD(ptr) (*(ptr))
#else
#define FLOW_ATOMIC_STORE(ptr, val)                                       \
    do { __typeof__(*(ptr) + 0) _v = (val);                               \
         __atomic_store((ptr), &_v, __ATOMIC_SEQ_CST); } while (0)
#define FLOW_ATOMIC_LOAD(ptr)                                             \
    ({ __typeof__(*(ptr) + 0) _r;                                         \
       __atomic_load((ptr), &_r, __ATOMIC_SEQ_CST); _r; })
#endif

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
    FLOW_ATOMIC_STORE(&g_degrade.degrade_level, level);
    FLOW_ATOMIC_STORE(&g_degrade.degrade_reason, reason);
    FLOW_ATOMIC_STORE(&g_degrade.degrade_timestamp_ms, 0); /* 由调用方填充 */

    /* 根据等级和原因设置 L1 参数 */
    if (level >= DEGRADE_L1) {
        FLOW_ATOMIC_STORE(&g_degrade.l1_disable_lane_change, 1);
    }
    if (level >= DEGRADE_L2) {
        FLOW_ATOMIC_STORE(&g_degrade.l1_speed_limit, 3.0);  /* 3 m/s crawl */
    }
    if (level >= DEGRADE_L3) {
        FLOW_ATOMIC_STORE(&g_degrade.l1_speed_limit, 0.0);  /* 立即停 */
    }
}

void degrade_clear(void) {
    FLOW_ATOMIC_STORE(&g_degrade.degrade_level, 0);
    FLOW_ATOMIC_STORE(&g_degrade.degrade_reason, 0);
    FLOW_ATOMIC_STORE(&g_degrade.degrade_timestamp_ms, 0);
    FLOW_ATOMIC_STORE(&g_degrade.l1_disable_lane_change, 0);
    FLOW_ATOMIC_STORE(&g_degrade.l1_speed_limit, 0.0);
    FLOW_ATOMIC_STORE(&g_degrade.l1_safety_margin, 1.0);

    /* 清 supervisor 心跳记录 */
    for (int i = 0; i < g_supervisor.node_count; i++)
        g_supervisor.nodes[i].last_heartbeat_ms = 0;
}

/* ══════════════════════════════════════════════════════════ */
/*  各层消费 API                                                  */
/* ══════════════════════════════════════════════════════════ */

DegradeAction degrade_layer_action(void) {
    int level  = (int)FLOW_ATOMIC_LOAD(&g_degrade.degrade_level);
    int reason = (int)FLOW_ATOMIC_LOAD(&g_degrade.degrade_reason);
    double speed_limit = (double)FLOW_ATOMIC_LOAD(&g_degrade.l1_speed_limit);
    double safety_margin = (double)FLOW_ATOMIC_LOAD(&g_degrade.l1_safety_margin);
    int disable_lc = (int)FLOW_ATOMIC_LOAD(&g_degrade.l1_disable_lane_change);

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
    int current = (int)FLOW_ATOMIC_LOAD(&g_degrade.degrade_level);

    /* 自动恢复：全部心跳健康持续 3s → 清降级。
     * 没有这条，supervisor 只升不降——一次瞬时抖动（调度延迟/负载尖峰）
     * 就把系统钉死在 L2/L3 直到重启。恢复必须滞后（3s 去抖）防振荡。 */
    static int64_t healthy_since_ms = 0;
    if (timeout_count == 0 && timeout_1s_count == 0) {
        if (current > DEGRADE_L0) {
            if (healthy_since_ms == 0) healthy_since_ms = now_ms;
            else if (now_ms - healthy_since_ms > 3000) {
                degrade_clear();
                healthy_since_ms = 0;
                return;
            }
        }
    } else {
        healthy_since_ms = 0;
    }

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

    int level  = (int)FLOW_ATOMIC_LOAD(&g_degrade.degrade_level);
    int reason = (int)FLOW_ATOMIC_LOAD(&g_degrade.degrade_reason);
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
