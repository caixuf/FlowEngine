/**
 * state_machine.c — 反射式状态机实现 (v2: debug hooks + 动态 guard + 动态规则)
 */

#include "state_machine.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

/* ── 内置状态/事件名称表 ─────────────────────────────────── */

static const char* g_builtin_state_names[] = {
    [0] = "INITIALIZED",
    [1] = "RUNNING",
    [2] = "STOPPING",
    [3] = "STOPPED",
    [4] = "PAUSED",
    [5] = "ERROR",
};

static const char* g_builtin_event_names[] = {
    [0] = "START",
    [1] = "STOP",
    [2] = "PAUSE",
    [3] = "RESUME",
    [4] = "RESTART",
    [5] = "DONE",
    [6] = "ERROR",
    [7] = "HEARTBEAT_OK",
    [8] = "HEARTBEAT_LOST",
};

static const char* state_name_str(StateId s) {
    if (s >= 0 && s < 6 && g_builtin_state_names[s])
        return g_builtin_state_names[s];
    return "UNKNOWN";
}

static const char* event_name_str(EventId e) {
    if (e >= 0 && e < 9 && g_builtin_event_names[e])
        return g_builtin_event_names[e];
    return "UNKNOWN";
}

/* ── 单调时钟 ────────────────────────────────────────────── */

static uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ══════════════════════════════════════════════════════════ */
/* 标准 task 生命周期转移表                                   */
/* ══════════════════════════════════════════════════════════ */

const TransitionRule SM_TABLE_STANDARD[] = {
    { SM_STATE_INITIALIZED, SM_EVENT_START,   SM_STATE_RUNNING,  "INITIALIZED + START -> RUNNING",  false },
    { SM_STATE_RUNNING,     SM_EVENT_STOP,    SM_STATE_STOPPING, "RUNNING + STOP -> STOPPING",      false },
    { SM_STATE_STOPPING,    SM_EVENT_DONE,    SM_STATE_STOPPED,  "STOPPING + DONE -> STOPPED",      false },
    { SM_STATE_RUNNING,     SM_EVENT_PAUSE,   SM_STATE_PAUSED,   "RUNNING + PAUSE -> PAUSED",       false },
    { SM_STATE_PAUSED,      SM_EVENT_RESUME,  SM_STATE_RUNNING,  "PAUSED + RESUME -> RUNNING",      false },
    { SM_STATE_RUNNING,     SM_EVENT_ERROR,   SM_STATE_ERROR,    "RUNNING + ERROR -> ERROR",        false },
    { SM_STATE_PAUSED,      SM_EVENT_ERROR,   SM_STATE_ERROR,    "PAUSED + ERROR -> ERROR",         false },
    { SM_STATE_ERROR,       SM_EVENT_RESTART, SM_STATE_INITIALIZED, "ERROR + RESTART -> INITIALIZED", false },
    { SM_STATE_STOPPED,     SM_EVENT_RESTART, SM_STATE_INITIALIZED, "STOPPED + RESTART -> INITIALIZED", false },
    TRANSITION_TABLE_END
};

/* ══════════════════════════════════════════════════════════ */
/* ADAS 驾驶模式 — 分层状态机预定义表                          */
/* ══════════════════════════════════════════════════════════ */

/**
 * 模式内子状态转移表（模板）。
 *
 * 每个模式复用此表：mode_id + SM_SUB_READY/ACTIVE/EXITING/FAULT
 *
 *   READY  + ACTIVATE        -> ACTIVE      (驾驶员按下激活)
 *   READY  + SYSTEM_FAULT    -> FAULT       (传感器故障)
 *   ACTIVE + DEACTIVATE      -> EXITING     (驾驶员取消/系统请求退出)
 *   ACTIVE + DRIVER_OVERRIDE -> EXITING     (驾驶员踩刹车/转向)
 *   ACTIVE + CONDITIONS_LOST -> EXITING     (车道线丢失/超出ODD)
 *   ACTIVE + SYSTEM_FAULT    -> FAULT       (运行时故障)
 *   EXITING + EXIT_DONE      -> READY       (安全退出完成)
 *   FAULT  + FAULT_CLEARED   -> READY       (故障恢复)
 */
const TransitionRule SM_TABLE_MODE_LIFECYCLE[] = {
    { SM_SUB_READY,  SM_EVT_ACTIVATE,        SM_SUB_ACTIVE,  "READY + ACTIVATE -> ACTIVE",       false },
    { SM_SUB_READY,  SM_EVT_SYSTEM_FAULT,    SM_SUB_FAULT,   "READY + FAULT -> FAULT",           false },
    { SM_SUB_ACTIVE, SM_EVT_DEACTIVATE,      SM_SUB_EXITING, "ACTIVE + DEACTIVATE -> EXITING",   false },
    { SM_SUB_ACTIVE, SM_EVT_DRIVER_OVERRIDE, SM_SUB_EXITING, "ACTIVE + OVERRIDE -> EXITING",     false },
    { SM_SUB_ACTIVE, SM_EVT_CONDITIONS_LOST, SM_SUB_EXITING, "ACTIVE + CONDITIONS_LOST -> EXITING", false },
    { SM_SUB_ACTIVE, SM_EVT_SYSTEM_FAULT,    SM_SUB_FAULT,   "ACTIVE + FAULT -> FAULT",          false },
    { SM_SUB_EXITING,SM_EVT_EXIT_DONE,       SM_SUB_READY,   "EXITING + EXIT_DONE -> READY",     false },
    { SM_SUB_FAULT,  SM_EVT_FAULT_CLEARED,   SM_SUB_READY,   "FAULT + FAULT_CLEARED -> READY",   false },
    TRANSITION_TABLE_END
};

/**
 * 模式间切换转移表。
 *
 * 模式能力递增: NA → ACC → CP → NP → NOA
 * 可以在相邻模式间上下切换（UPGRADE/DOWNGRADE）。
 * ODD 边界丢失直接退回 NA。
 *
 * 注意：这里的 StateId 使用 SM_MODE_STATE(mode, SM_SUB_READY)
 * 来表示"切换到某模式的就绪状态"。
 */
const TransitionRule SM_TABLE_MODE_SWITCHING[] = {
    /* 从 NA 出发 */
    { SM_MODE_NA, SM_EVT_CONDITIONS_MET, SM_MODE_ACC,
      "NA + CONDITIONS_MET -> ACC:READY", false },

    /* ACC ↔ CP */
    { SM_MODE_ACC, SM_EVT_MODE_UPGRADE,   SM_MODE_CP,
      "ACC + UPGRADE -> CP:READY (lane detected)", false },
    { SM_MODE_CP,  SM_EVT_MODE_DOWNGRADE, SM_MODE_ACC,
      "CP + DOWNGRADE -> ACC:READY", false },

    /* CP ↔ NP */
    { SM_MODE_CP,  SM_EVT_MODE_UPGRADE,   SM_MODE_NP,
      "CP + UPGRADE -> NP:READY (highway)", false },
    { SM_MODE_NP,  SM_EVT_MODE_DOWNGRADE, SM_MODE_CP,
      "NP + DOWNGRADE -> CP:READY", false },

    /* NP ↔ NOA */
    { SM_MODE_NP,  SM_EVT_MODE_UPGRADE,   SM_MODE_NOA,
      "NP + UPGRADE -> NOA:READY (HD map)", false },
    { SM_MODE_NOA, SM_EVT_MODE_DOWNGRADE, SM_MODE_NP,
      "NOA + DOWNGRADE -> NP:READY", false },

    /* LP (拥堵辅助) — 从 ACC/CP 进入 */
    { SM_MODE_ACC, SM_EVT_MODE_UPGRADE + 1, SM_MODE_LP,
      "ACC + LOW_SPEED -> LP:READY (traffic jam)", false },
    { SM_MODE_CP,  SM_EVT_MODE_UPGRADE + 1, SM_MODE_LP,
      "CP + LOW_SPEED -> LP:READY", false },
    { SM_MODE_LP,  SM_EVT_MODE_DOWNGRADE,   SM_MODE_ACC,
      "LP + SPEED_UP -> ACC:READY", false },

    /* 任何模式 + CONDITIONS_LOST or SYSTEM_FAULT -> NA */
    { SM_MODE_ACC, SM_EVT_CONDITIONS_LOST, SM_MODE_NA,
      "ACC + CONDITIONS_LOST -> NA", false },
    { SM_MODE_CP,  SM_EVT_CONDITIONS_LOST, SM_MODE_NA,
      "CP + CONDITIONS_LOST -> NA", false },
    { SM_MODE_NP,  SM_EVT_CONDITIONS_LOST, SM_MODE_NA,
      "NP + CONDITIONS_LOST -> NA", false },
    { SM_MODE_LP,  SM_EVT_CONDITIONS_LOST, SM_MODE_NA,
      "LP + CONDITIONS_LOST -> NA", false },
    { SM_MODE_NOA, SM_EVT_CONDITIONS_LOST, SM_MODE_NA,
      "NOA + CONDITIONS_LOST -> NA", false },

    /* 任何模式 + FAULT -> NA */
    { SM_MODE_ACC, SM_EVT_SYSTEM_FAULT, SM_MODE_NA,
      "ACC + FAULT -> NA", false },
    { SM_MODE_CP,  SM_EVT_SYSTEM_FAULT, SM_MODE_NA,
      "CP + FAULT -> NA", false },
    { SM_MODE_NP,  SM_EVT_SYSTEM_FAULT, SM_MODE_NA,
      "NP + FAULT -> NA", false },
    { SM_MODE_LP,  SM_EVT_SYSTEM_FAULT, SM_MODE_NA,
      "LP + FAULT -> NA", false },
    { SM_MODE_NOA, SM_EVT_SYSTEM_FAULT, SM_MODE_NA,
      "NOA + FAULT -> NA", false },

    TRANSITION_TABLE_END
};

/* ── 模式名称 ─────────────────────────────────────────────── */

static const char* g_mode_names[] = {
    [0] = "NA",     /* SM_MODE_NA - 64 */
    [1] = "ACC",
    [2] = "CP",
    [3] = "NP",
    [4] = "LP",
    [5] = "NOA",
};

static const char* g_sub_names[] = {
    [0] = "READY",
    [1] = "ACTIVE",
    [2] = "EXITING",
    [3] = "FAULT",
};

const char* statem_mode_name(StateId mode_id) {
    int idx = mode_id - SM_MODE_NA;
    if (idx >= 0 && idx < 6 && g_mode_names[idx])
        return g_mode_names[idx];
    return "?";
}

const char* statem_sub_state_name(int sub) {
    if (sub >= 0 && sub < 4) return g_sub_names[sub];
    return "?";
}

void statem_format_hierarchical(StateId state, char* buf, size_t size) {
    if (state < SM_MODE_NA) {
        /* Built-in state (INITIALIZED, RUNNING, etc.) */
        snprintf(buf, size, "%s", state_name_str(state));
    } else {
        /* ADAS mode + sub-state */
        StateId mode = SM_MODE_OF(state);
        int sub = SM_SUB_OF(state);
        snprintf(buf, size, "%s:%s", statem_mode_name(mode), statem_sub_state_name(sub));
    }
}

/* ══════════════════════════════════════════════════════════ */
/* 初始化                                                     */
/* ══════════════════════════════════════════════════════════ */

void statem_init(ReflectiveStateMachine* sm, const TransitionRule* table,
                 StateId initial_state, const char* task_name) {
    if (!sm) return;
    memset(sm, 0, sizeof(*sm));
    sm->current        = initial_state;
    sm->previous       = SM_STATE_UNKNOWN;
    sm->last_event     = SM_EVENT_NONE;
    sm->entered_at_us  = monotonic_us();
    sm->static_table   = table;
    sm->task_name      = task_name;
    sm->trace_enabled  = false;

    /* Count static table entries */
    if (table) {
        int count = 0;
        while (table[count].from != -1) count++;
        sm->static_size = count;
    }
}

/* ══════════════════════════════════════════════════════════ */
/* 转移表查找（静态表 + 动态表）                               */
/* ══════════════════════════════════════════════════════════ */

static int total_rules(const ReflectiveStateMachine* sm) {
    return sm->static_size + sm->dynamic_count;
}

static const TransitionRule* get_rule(const ReflectiveStateMachine* sm, int idx) {
    if (idx < sm->static_size) {
        return &sm->static_table[idx];
    } else {
        return &sm->dynamic_rules[idx - sm->static_size];
    }
}

static const TransitionRule* find_transition(const ReflectiveStateMachine* sm,
                                              StateId from, EventId event) {
    int total = total_rules(sm);

    /* Pass 1: exact (from, event) match */
    for (int i = 0; i < total; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if (r->from == from && r->event == event && !r->is_auto) {
            return r;
        }
    }

    /* Pass 2: auto-transition */
    for (int i = 0; i < total; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if (r->from == from && r->is_auto) {
            return r;
        }
    }

    return NULL;
}

/* ══════════════════════════════════════════════════════════ */
/* 事件驱动                                                   */
/* ══════════════════════════════════════════════════════════ */

bool statem_send_event(ReflectiveStateMachine* sm, EventId event, void* task) {
    if (!sm) return false;

    StateId from = sm->current;
    const TransitionRule* rule = find_transition(sm, from, event);
    const char* desc = rule ? (rule->description ? rule->description : "") : NULL;

    if (!rule) {
        /* Illegal transition */
        if (sm->trace_enabled) {
            LOG_WARN("statem", "%s: ILLEGAL %s + %s -> ???",
                     sm->task_name ? sm->task_name : "?",
                     state_name_str(from), event_name_str(event));
        }
        if (sm->debug_hook) {
            sm->debug_hook(task, from, event, SM_STATE_UNKNOWN, "ILLEGAL", false);
        }
        return false;
    }

    /* Guard check */
    if (sm->guard && !sm->guard(task, from, event, rule->to)) {
        if (sm->trace_enabled) {
            LOG_WARN("statem", "%s: GUARD REJECTED %s + %s -> %s",
                     sm->task_name ? sm->task_name : "?",
                     state_name_str(from), event_name_str(event),
                     state_name_str(rule->to));
        }
        if (sm->debug_hook) {
            sm->debug_hook(task, from, event, rule->to, desc, false);
        }
        return false;
    }

    /* Trace log */
    if (sm->trace_enabled) {
        LOG_INFO("statem", "%s: %s + %s -> %s",
                 sm->task_name ? sm->task_name : "?",
                 state_name_str(from), event_name_str(event),
                 state_name_str(rule->to));
    }

    /* Exit action */
    if (sm->on_exit) sm->on_exit(task, from, event);

    /* Record history */
    uint32_t idx = sm->history_head;
    sm->history[idx].from         = from;
    sm->history[idx].event        = event;
    sm->history[idx].to           = rule->to;
    sm->history[idx].timestamp_us = monotonic_us();
    sm->history_head = (idx + 1) % SM_HISTORY_DEPTH;
    if (sm->history_count < SM_HISTORY_DEPTH) sm->history_count++;

    /* Transition */
    sm->previous       = from;
    sm->current        = rule->to;
    sm->last_event     = event;
    sm->entered_at_us  = monotonic_us();

    /* Entry action */
    if (sm->on_entry) sm->on_entry(task, rule->to, event);

    /* Debug hook (post-transition) */
    if (sm->debug_hook) {
        sm->debug_hook(task, from, event, rule->to, desc, true);
    }

    return true;
}

bool statem_process_auto(ReflectiveStateMachine* sm, void* task) {
    if (!sm) return false;
    int total = total_rules(sm);
    for (int i = 0; i < total; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if (r->from == sm->current && r->is_auto) {
            return statem_send_event(sm, SM_EVENT_NONE, task);
        }
    }
    return false;
}

/* ══════════════════════════════════════════════════════════ */
/* 反射查询                                                   */
/* ══════════════════════════════════════════════════════════ */

bool statem_can_transition(const ReflectiveStateMachine* sm, EventId event) {
    return find_transition(sm, statem_current(sm), event) != NULL;
}

StateId statem_target_state(const ReflectiveStateMachine* sm,
                            StateId from, EventId event) {
    const TransitionRule* r = find_transition(sm, from, event);
    return r ? r->to : SM_STATE_UNKNOWN;
}

int statem_allowed_events(const ReflectiveStateMachine* sm,
                          EventId* events, int max) {
    if (!sm || !events || max <= 0) return 0;

    int count = 0;
    StateId cur = sm->current;
    int total = total_rules(sm);

    for (int i = 0; i < total && count < max; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if (r->from == cur && r->event != SM_EVENT_NONE && !r->is_auto) {
            bool dup = false;
            for (int j = 0; j < count; j++) {
                if (events[j] == r->event) { dup = true; break; }
            }
            if (!dup) events[count++] = r->event;
        }
    }
    return count;
}

const char* statem_state_name(const ReflectiveStateMachine* sm, StateId state) {
    (void)sm;
    return state_name_str(state);
}

const char* statem_event_name(const ReflectiveStateMachine* sm, EventId event) {
    (void)sm;
    return event_name_str(event);
}

bool statem_is_terminal(const ReflectiveStateMachine* sm) {
    if (!sm) return false;
    StateId cur = sm->current;
    int total = total_rules(sm);
    for (int i = 0; i < total; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if (r->from == cur && r->event != SM_EVENT_NONE && !r->is_auto)
            return false;
    }
    return true;
}

const char* statem_last_transition_desc(const ReflectiveStateMachine* sm) {
    if (!sm || sm->history_count == 0) return "(no history)";
    uint32_t idx = (sm->history_head + SM_HISTORY_DEPTH - 1) % SM_HISTORY_DEPTH;
    const TransitionRecord* rec = &sm->history[idx];

    const TransitionRule* rule = find_transition(sm, rec->from, rec->event);
    if (rule && rule->description) return rule->description;

    static char buf[128];
    snprintf(buf, sizeof(buf), "%s + %s -> %s",
             state_name_str(rec->from), event_name_str(rec->event),
             state_name_str(rec->to));
    return buf;
}

/* ══════════════════════════════════════════════════════════ */
/* 自省                                                       */
/* ══════════════════════════════════════════════════════════ */

int statem_all_states(const ReflectiveStateMachine* sm,
                      StateId* states, int max) {
    if (!sm || !states || max <= 0) return 0;

    int count = 0;
    int total = total_rules(sm);

    for (int i = 0; i < total && count < max; i++) {
        const TransitionRule* r = get_rule(sm, i);

        bool found_from = false;
        for (int j = 0; j < count; j++)
            if (states[j] == r->from) { found_from = true; break; }
        if (!found_from) states[count++] = r->from;

        if (count >= max) break;
        bool found_to = false;
        for (int j = 0; j < count; j++)
            if (states[j] == r->to) { found_to = true; break; }
        if (!found_to) states[count++] = r->to;
    }
    return count;
}

void statem_dump_table(const ReflectiveStateMachine* sm) {
    if (!sm) return;

    printf("\n[状态机: %s]\n", sm->task_name ? sm->task_name : "(unnamed)");
    printf("  transitions:\n");

    int total = total_rules(sm);
    for (int i = 0; i < total; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if (r->is_auto) {
            printf("    %-12s --auto--> %-12s\n",
                   state_name_str(r->from), state_name_str(r->to));
        } else {
            printf("    %-12s + %-12s -> %-12s  %s\n",
                   state_name_str(r->from),
                   event_name_str(r->event),
                   state_name_str(r->to),
                   r->description ? r->description : "");
        }
    }

    printf("  current: %s", state_name_str(sm->current));
    if (sm->last_event != SM_EVENT_NONE)
        printf("  (via %s)", event_name_str(sm->last_event));
    printf("\n");

    EventId allowed[16];
    int n = statem_allowed_events(sm, allowed, 16);
    printf("  allowed_events: ");
    if (n == 0) {
        printf("(none — terminal)\n");
    } else {
        for (int i = 0; i < n; i++)
            printf("%s%s", event_name_str(allowed[i]), (i < n - 1) ? ", " : "");
        printf("\n");
    }

    printf("  guard: %s | trace: %s | debug_hook: %s\n",
           sm->guard ? "active" : "none",
           sm->trace_enabled ? "ON" : "OFF",
           sm->debug_hook ? "installed" : "none");

    if (sm->history_count > 0) {
        printf("  recent (%u):\n", sm->history_count);
        uint32_t start = (sm->history_head + SM_HISTORY_DEPTH - sm->history_count)
                         % SM_HISTORY_DEPTH;
        for (uint32_t i = 0; i < sm->history_count && i < 5; i++) {
            uint32_t idx = (start + i) % SM_HISTORY_DEPTH;
            const TransitionRecord* rec = &sm->history[idx];
            printf("    %s + %s -> %s\n",
                   state_name_str(rec->from), event_name_str(rec->event),
                   state_name_str(rec->to));
        }
    }
    printf("\n");
}

void statem_print_status(const ReflectiveStateMachine* sm) {
    if (!sm) return;

    uint64_t elapsed_ms = (monotonic_us() - sm->entered_at_us) / 1000;

    printf("[%s] state=%s", sm->task_name ? sm->task_name : "?",
           state_name_str(sm->current));

    EventId allowed[16];
    int n = statem_allowed_events(sm, allowed, 16);
    if (n > 0) {
        printf(" allowed=[");
        for (int i = 0; i < n && i < 5; i++)
            printf("%s%s", event_name_str(allowed[i]), (i < n - 1) ? "," : "");
        if (n > 5) printf("...");
        printf("]");
    }

    printf(" since=%lums\n", (unsigned long)elapsed_ms);
}

char* statem_export_json(const ReflectiveStateMachine* sm) {
    if (!sm) return strdup("{}");

    size_t sz = 4096;
    char* buf = (char*)malloc(sz);
    if (!buf) return NULL;

    int off = snprintf(buf, sz,
        "{"
        "\"task\":\"%s\","
        "\"current\":\"%s\","
        "\"previous\":\"%s\","
        "\"last_event\":\"%s\","
        "\"is_terminal\":%s,"
        "\"guard_active\":%s,"
        "\"trace\":%s,"
        "\"transitions\":[",
        sm->task_name ? sm->task_name : "",
        state_name_str(sm->current),
        state_name_str(sm->previous),
        event_name_str(sm->last_event),
        statem_is_terminal(sm) ? "true" : "false",
        sm->guard ? "true" : "false",
        sm->trace_enabled ? "true" : "false");

    int total = total_rules(sm);
    for (int i = 0; i < total; i++) {
        const TransitionRule* r = get_rule(sm, i);
        if ((size_t)off + 128 >= sz) {
            sz *= 2;
            buf = (char*)realloc(buf, sz);
            if (!buf) return NULL;
        }
        off += snprintf(buf + off, sz - off,
            "%s{\"from\":\"%s\",\"event\":\"%s\",\"to\":\"%s\",\"desc\":\"%s\",\"auto\":%s}",
            i == 0 ? "" : ",",
            state_name_str(r->from),
            r->is_auto ? "AUTO" : event_name_str(r->event),
            state_name_str(r->to),
            r->description ? r->description : "",
            r->is_auto ? "true" : "false");
    }

    off += snprintf(buf + off, sz - off, "]}");
    return buf;
}

/* ══════════════════════════════════════════════════════════ */
/* 条件管理器 — guard 运行时替换                               */
/* ══════════════════════════════════════════════════════════ */

void statem_set_guard(ReflectiveStateMachine* sm, TransitionGuard new_guard) {
    if (!sm) return;
    sm->guard = new_guard;
    if (sm->trace_enabled) {
        printf("[statem:%s] guard %s\n",
               sm->task_name ? sm->task_name : "?",
               new_guard ? "installed" : "removed (all transitions allowed)");
    }
}

TransitionGuard statem_get_guard(const ReflectiveStateMachine* sm) {
    return sm ? sm->guard : NULL;
}

/* ══════════════════════════════════════════════════════════ */
/* 动态转移规则                                               */
/* ══════════════════════════════════════════════════════════ */

#define DYNAMIC_RULE_CHUNK 8

int statem_add_transition(ReflectiveStateMachine* sm,
                          StateId from, EventId event, StateId to,
                          const char* description) {
    if (!sm) return -1;

    /* Grow dynamic array if needed */
    if (sm->dynamic_count >= sm->dynamic_cap) {
        int new_cap = sm->dynamic_cap + DYNAMIC_RULE_CHUNK;
        TransitionRule* new_rules = (TransitionRule*)realloc(
            sm->dynamic_rules, (size_t)new_cap * sizeof(TransitionRule));
        if (!new_rules) return -1;
        sm->dynamic_rules = new_rules;
        sm->dynamic_cap   = new_cap;
    }

    TransitionRule* r = &sm->dynamic_rules[sm->dynamic_count++];
    r->from        = from;
    r->event       = event;
    r->to          = to;
    r->description = description;
    r->is_auto     = false;

    if (sm->trace_enabled) {
        printf("[statem:%s] dynamic rule added: %s + %s -> %s\n",
               sm->task_name ? sm->task_name : "?",
               state_name_str(from), event_name_str(event),
               state_name_str(to));
    }

    return 0;
}

int statem_remove_transition(ReflectiveStateMachine* sm,
                             StateId from, EventId event) {
    if (!sm) return 0;
    int removed = 0;

    for (int i = 0; i < sm->dynamic_count; i++) {
        TransitionRule* r = &sm->dynamic_rules[i];
        if (r->from == from && r->event == event) {
            /* Shift remaining entries */
            if (i < sm->dynamic_count - 1) {
                memmove(&sm->dynamic_rules[i], &sm->dynamic_rules[i + 1],
                        (size_t)(sm->dynamic_count - i - 1) * sizeof(TransitionRule));
            }
            sm->dynamic_count--;
            removed++;
            i--; /* re-check this index */
        }
    }
    return removed;
}

/* ══════════════════════════════════════════════════════════ */
/* 调试支持                                                   */
/* ══════════════════════════════════════════════════════════ */

void statem_set_debug_hook(ReflectiveStateMachine* sm, StateDebugHook hook) {
    if (!sm) return;
    sm->debug_hook = hook;
}

void statem_set_trace(ReflectiveStateMachine* sm, bool enabled) {
    if (!sm) return;
    sm->trace_enabled = enabled;
    if (enabled) {
        LOG_INFO("statem", "%s: trace enabled (current=%s)",
                 sm->task_name ? sm->task_name : "?",
                 state_name_str(sm->current));
    }
}
