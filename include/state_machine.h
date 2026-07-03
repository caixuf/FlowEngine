#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/**
 * @file state_machine.h
 * @brief 反射式状态机框架 (FlowEngine — TaskBase 内嵌)
 *
 * 核心思想：状态机不仅会跑，还知道自己的结构（transition table）。
 *
 * 每个 task 通过嵌入 ReflectiveStateMachine，暴露：
 *   - current state（当前状态）
 *   - allowed_events（当前状态允许接收的事件列表）
 *   - transition table（完整的状态转移表，可查询/遍历/打印）
 *   - transition history（状态流转历史）
 *
 *   典型用法（task_manager 查询）：
 *     if (!statem_can_transition(&task->base.sm, EVENT_STOP)) {
 *         printf("非法: 当前状态 %s 不能接收 STOP\n",
 *                statem_state_name(&task->base.sm));
 *     }
 *     statem_dump_table(&task->base.sm);  // 打印完整转移表
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 状态/事件 ID ─────────────────────────────────────────── */

typedef int32_t StateId;
typedef int32_t EventId;

/* 内置状态 ID（与 TaskState 对齐，但独立命名空间） */
#define SM_STATE_UNKNOWN       ((StateId)(-1))
#define SM_STATE_INITIALIZED   0
#define SM_STATE_RUNNING       1
#define SM_STATE_STOPPING      2
#define SM_STATE_STOPPED       3
#define SM_STATE_PAUSED        4
#define SM_STATE_ERROR         5
#define SM_STATE_MAX            16

/* 内置事件 ID */
#define SM_EVENT_NONE          ((EventId)(-1))
#define SM_EVENT_START          0
#define SM_EVENT_STOP           1
#define SM_EVENT_PAUSE          2
#define SM_EVENT_RESUME         3
#define SM_EVENT_RESTART        4
#define SM_EVENT_DONE           5   /**< 异步完成（如协程结束） */
#define SM_EVENT_ERROR          6   /**< 运行时错误 */
#define SM_EVENT_HEARTBEAT_OK   7
#define SM_EVENT_HEARTBEAT_LOST 8
/* 用户自定义事件从 16 开始 */
#define SM_EVENT_USER_BASE      16

/* ── 转移条目 ─────────────────────────────────────────────── */

/** 转移表中的一条规则 */
typedef struct {
    StateId   from;           /**< 源状态 */
    EventId   event;          /**< 触发事件 */
    StateId   to;             /**< 目标状态 */
    const char* description;  /**< 人类可读描述，如 "INITIALIZED + START -> RUNNING" */
    bool      is_auto;        /**< true = 到达 from 时自动触发（无需外部事件） */
} TransitionRule;

/** 哨兵值，标记转移表结束 */
#define TRANSITION_TABLE_END  { -1, -1, -1, NULL, false }

/* ── 转移历史 ─────────────────────────────────────────────── */

#define SM_HISTORY_DEPTH 8

typedef struct {
    StateId   from;
    EventId   event;
    StateId   to;
    uint64_t  timestamp_us;   /**< monotonic timestamp */
} TransitionRecord;

/* ── Entry/Exit Action 回调 ───────────────────────────────── */

typedef void (*StateAction)(void* task, StateId state, EventId event);

/* ── Guard 条件回调 ────────────────────────────────────────── */
typedef bool (*TransitionGuard)(void* task, StateId from, EventId event, StateId to);

/* ── Debug 回调 ─────────────────────────────────────────────── */
/**
 * 每次状态转移时调用（用于调试/监控）。
 * @param task   关联的 task
 * @param from   源状态
 * @param event  触发事件
 * @param to     目标状态
 * @param rule   匹配的转移规则描述
 * @param accepted true=转移发生, false=被 guard 拒绝
 */
typedef void (*StateDebugHook)(void* task, StateId from, EventId event,
                               StateId to, const char* rule_desc, bool accepted);

/* ── 反射式状态机 ──────────────────────────────────────────── */

typedef struct {
    /* ── 运行时状态 ──────────────────────────────────── */
    StateId              current;           /**< 当前状态 */
    StateId              previous;          /**< 上一个状态 */
    EventId              last_event;        /**< 最近接收的事件 */
    uint64_t             entered_at_us;     /**< 进入当前状态的时间戳 */

    /* ── 转移历史（环形缓冲）─────────────────────────── */
    TransitionRecord     history[SM_HISTORY_DEPTH];
    uint32_t             history_head;
    uint32_t             history_count;

    /* ── 转移表（可动态扩展）─────────────────────────── */
    const TransitionRule* static_table;     /**< 静态转移表（编译期定义） */
    int                   static_size;      /**< 静态表条目数 */
    TransitionRule*       dynamic_rules;    /**< 动态添加的转移规则（运行时） */
    int                   dynamic_count;    /**< 动态规则数 */
    int                   dynamic_cap;      /**< 动态规则容量 */

    /* ── Action 回调（可运行时替换）─────────────────── */
    StateAction           on_entry;          /**< 进入状态时调用 */
    StateAction           on_exit;           /**< 离开状态时调用 */
    TransitionGuard       guard;             /**< 转移守卫（NULL = 总是允许） */

    /* ── 调试支持 ───────────────────────────────────── */
    StateDebugHook        debug_hook;        /**< 每次转移时调用（NULL = 关闭） */
    bool                  trace_enabled;     /**< 是否打印转移日志到 stderr */

    /* ── 所属任务名（用于日志）───────────────────────── */
    const char*           task_name;
} ReflectiveStateMachine;

/* ══════════════════════════════════════════════════════════ */
/* 初始化                                                     */
/* ══════════════════════════════════════════════════════════ */

/**
 * 初始化状态机。
 * @param sm               状态机
 * @param table            转移表（静态常量，以 TRANSITION_TABLE_END 结尾）
 * @param initial_state    初始状态
 * @param task_name        任务名（日志用，可为 NULL）
 */
void statem_init(ReflectiveStateMachine* sm, const TransitionRule* table,
                 StateId initial_state, const char* task_name);

/* ══════════════════════════════════════════════════════════ */
/* 事件驱动                                                   */
/* ══════════════════════════════════════════════════════════ */

/**
 * 向状态机发送事件，触发状态转移。
 *
 * 流程：
 *   1. 查转移表：from=current && event → to
 *   2. 如未找到匹配 → 返回 false（非法转移）
 *   3. 如有 guard → 调用 guard(task, from, event, to)，false → 拒绝
 *   4. 调用 on_exit(task, from, event)
 *   5. 更新 current/previous/entered_at_us
 *   6. 记录转移历史
 *   7. 调用 on_entry(task, to, event)
 *   8. 返回 true
 *
 * @param sm     状态机
 * @param event  事件 ID
 * @param task   关联的 task 指针（传给 action/guard 回调）
 * @return true = 转移成功，false = 非法转移或被 guard 拒绝
 */
bool statem_send_event(ReflectiveStateMachine* sm, EventId event, void* task);

/**
 * 处理自动转移（到达某状态后自动跳到下一状态）。
 * 通常在每个执行周期末尾调用。
 * @return true 如果发生了自动转移
 */
bool statem_process_auto(ReflectiveStateMachine* sm, void* task);

/* ══════════════════════════════════════════════════════════ */
/* 反射查询 — 状态机知道自己怎么跑的                           */
/* ══════════════════════════════════════════════════════════ */

/** 当前状态 ID */
static inline StateId statem_current(const ReflectiveStateMachine* sm) {
    return sm ? sm->current : SM_STATE_UNKNOWN;
}

/** 上一个状态 ID */
static inline StateId statem_previous(const ReflectiveStateMachine* sm) {
    return sm ? sm->previous : SM_STATE_UNKNOWN;
}

/** 最近事件 ID */
static inline EventId statem_last_event(const ReflectiveStateMachine* sm) {
    return sm ? sm->last_event : SM_EVENT_NONE;
}

/**
 * 查询：当前状态下能否接收 event？
 * @return true = 转移表中有匹配条目
 */
bool statem_can_transition(const ReflectiveStateMachine* sm, EventId event);

/**
 * 查询：从 from 状态接收 event 后的目标状态。
 * @return 目标状态 ID，未找到返回 SM_STATE_UNKNOWN
 */
StateId statem_target_state(const ReflectiveStateMachine* sm,
                            StateId from, EventId event);

/**
 * 获取当前状态允许接收的事件列表。
 * @param events  输出缓冲区
 * @param max     缓冲区大小
 * @return 实际事件数量
 */
int statem_allowed_events(const ReflectiveStateMachine* sm,
                          EventId* events, int max);

/**
 * 获取指定状态的名称字符串。
 */
const char* statem_state_name(const ReflectiveStateMachine* sm, StateId state);

/**
 * 获取指定事件的名称字符串。
 */
const char* statem_event_name(const ReflectiveStateMachine* sm, EventId event);

/**
 * 查询状态机是否处于终态（无任何出转移的状态）。
 */
bool statem_is_terminal(const ReflectiveStateMachine* sm);

/**
 * 获取最近一次转移的描述字符串。
 * 格式: "INITIALIZED + START -> RUNNING"
 */
const char* statem_last_transition_desc(const ReflectiveStateMachine* sm);

/* ══════════════════════════════════════════════════════════ */
/* 自省 — 打印/导出转移表                                     */
/* ══════════════════════════════════════════════════════════ */

/**
 * 打印完整的状态转移表到 stdout。
 * 输出格式：
 *   [状态机: sensor_fusion]
 *     INITIALIZED + START -> RUNNING
 *     RUNNING     + STOP  -> STOPPING
 *     ...
 *   当前状态: RUNNING  允许事件: STOP, PAUSE
 */
void statem_dump_table(const ReflectiveStateMachine* sm);

/**
 * 导出转移表为 JSON 片段（调用者需 free 返回值）。
 * 可用于 web 可视化或监控工具。
 */
char* statem_export_json(const ReflectiveStateMachine* sm);

/**
 * 获取状态机中定义的所有状态 ID 列表（从转移表推导）。
 * @param states 输出缓冲区
 * @param max    缓冲区大小
 * @return 实际状态数量
 */
int statem_all_states(const ReflectiveStateMachine* sm,
                      StateId* states, int max);

/* ══════════════════════════════════════════════════════════ */
/* 条件管理器 — 运行时便捷修改 guard 条件                      */
/* ══════════════════════════════════════════════════════════ */

/**
 * 替换状态机的 guard 条件（运行时动态修改）。
 * 例如：调试时临时关闭所有 guard、或在配置加载后更换 guard 逻辑。
 *
 * 用法：
 *   statem_set_guard(&task->base.sm, my_runtime_guard);  // 替换
 *   statem_set_guard(&task->base.sm, NULL);              // 移除所有 guard
 */
void statem_set_guard(ReflectiveStateMachine* sm, TransitionGuard new_guard);

/**
 * 获取当前 guard 函数指针（可用于保存/恢复）。
 */
TransitionGuard statem_get_guard(const ReflectiveStateMachine* sm);

/* ══════════════════════════════════════════════════════════ */
/* 动态转移规则 — 运行时添加/删除转移                          */
/* ══════════════════════════════════════════════════════════ */

/**
 * 动态添加一条转移规则（运行时扩展转移表）。
 *
 * 适用于：运行时根据配置文件加载额外转移、插件注册新事件等场景。
 *
 * @return 0 = 成功, -1 = 内存不足
 */
int statem_add_transition(ReflectiveStateMachine* sm,
                          StateId from, EventId event, StateId to,
                          const char* description);

/**
 * 删除匹配 (from, event) 的动态转移规则。
 * 只删除动态添加的规则，不删除静态表中的规则。
 * @return 删除的条目数（0 表示未找到）
 */
int statem_remove_transition(ReflectiveStateMachine* sm,
                             StateId from, EventId event);

/* ══════════════════════════════════════════════════════════ */
/* 调试支持                                                    */
/* ══════════════════════════════════════════════════════════ */

/**
 * 设置调试 hook — 每次状态转移时回调。
 * 用于监控、profiling、远程调试等场景。
 *
 * hook 参数: (task, from, event, to, rule_desc, accepted)
 *   accepted=false 表示转移被 guard 拒绝了。
 */
void statem_set_debug_hook(ReflectiveStateMachine* sm, StateDebugHook hook);

/**
 * 启用/关闭转移跟踪日志（输出到 stderr）。
 * 开启后每次转移打印：
 *   [statem:task_name] INITIALIZED + START -> RUNNING
 *   [statem:task_name] GUARD REJECTED: RUNNING + STOP -> STOPPING (reason)
 */
void statem_set_trace(ReflectiveStateMachine* sm, bool enabled);

/**
 * 打印当前状态摘要（一行，适合高频日志）。
 * 输出格式: [task_name] state=RUNNING allowed=[STOP,PAUSE] since=12345ms
 */
void statem_print_status(const ReflectiveStateMachine* sm);

/* ══════════════════════════════════════════════════════════ */
/* 预定义转移表（常见 task 生命周期）                         */
/* ══════════════════════════════════════════════════════════ */

/** 标准 task 生命周期转移表 */
extern const TransitionRule SM_TABLE_STANDARD[];

/**
 * 标准转移表内容：
 *   INITIALIZED + START   -> RUNNING
 *   RUNNING     + STOP    -> STOPPING
 *   STOPPING    + DONE    -> STOPPED
 *   RUNNING     + PAUSE   -> PAUSED
 *   PAUSED      + RESUME  -> RUNNING
 *   RUNNING     + ERROR   -> ERROR
 *   PAUSED      + ERROR   -> ERROR
 *   ERROR       + RESTART -> INITIALIZED
 *   STOPPED     + RESTART -> INITIALIZED
 */

/* ══════════════════════════════════════════════════════════ */
/* 智能驾驶功能状态机 — 分层状态（mode + sub-state）          */
/* ══════════════════════════════════════════════════════════ */

/**
 * 驾驶模式 ID (64+ 开始，避免与内置状态冲突)
 *
 *    NA  — Not Available（功能不可用/待机）
 *    ACC — Adaptive Cruise Control（自适应巡航）
 *    CP  — Conventional Pilot（基础车道保持 L2）
 *    NP  — Navigate Pilot（高速导航辅助 L2+）
 *    LP  — Low-speed Pilot（拥堵/低速辅助 TJP）
 *    NOA — Navigate on Autopilot（全域导航辅助 L2++）
 *
 * 每个模式有 4 个子状态：READY → ACTIVE → EXITING → (READY or FAULT)
 */
#define SM_MODE_NA    64
#define SM_MODE_ACC   65
#define SM_MODE_CP    66
#define SM_MODE_NP    67
#define SM_MODE_LP    68
#define SM_MODE_NOA   69

/** 模式子状态（偏移量，mode_id + sub = 实际 StateId） */
#define SM_SUB_READY    0   /**< 就绪：条件满足，等待激活 */
#define SM_SUB_ACTIVE   1   /**< 激活：正在控制车辆 */
#define SM_SUB_EXITING  2   /**< 退出中：交还控制权 */
#define SM_SUB_FAULT    3   /**< 故障：功能不可用 */

/** 构造完整状态 ID */
#define SM_MODE_STATE(mode, sub)  ((mode) + (sub))
#define SM_MODE_OF(state)         ((state) & ~3)
#define SM_SUB_OF(state)          ((state) & 3)

/* 驾驶模式事件 (128+) */
#define SM_EVT_MODE_REQUEST      128  /**< 请求切换到某模式（data=target_mode） */
#define SM_EVT_ACTIVATE          129  /**< 激活当前模式（READY → ACTIVE） */
#define SM_EVT_DEACTIVATE        130  /**< 退出当前模式（ACTIVE → EXITING） */
#define SM_EVT_DRIVER_OVERRIDE   131  /**< 驾驶员接管 */
#define SM_EVT_CONDITIONS_MET    132  /**< 前置条件满足（→ READY） */
#define SM_EVT_CONDITIONS_LOST   133  /**< 前置条件丢失（→ EXITING or NA） */
#define SM_EVT_SYSTEM_FAULT      134  /**< 系统故障（→ FAULT） */
#define SM_EVT_FAULT_CLEARED     135  /**< 故障清除（→ READY or NA） */
#define SM_EVT_EXIT_DONE         136  /**< 退出完成（→ READY or NA） */
#define SM_EVT_MODE_UPGRADE      137  /**< 模式升级（CP→NP→NOA） */
#define SM_EVT_MODE_DOWNGRADE    138  /**< 模式降级（NOA→NP→CP→ACC） */

/* ── 预定义转移表 ──────────────────────────────────────────── */

/** 单个模式的子状态转移表（READY/ACTIVE/EXITING/FAULT 循环） */
extern const TransitionRule SM_TABLE_MODE_LIFECYCLE[];

/**
 * 模式内转移表：
 *   READY  + ACTIVATE        -> ACTIVE
 *   ACTIVE + DEACTIVATE      -> EXITING
 *   EXITING + EXIT_DONE      -> READY
 *   READY  + SYSTEM_FAULT    -> FAULT
 *   ACTIVE + SYSTEM_FAULT    -> FAULT
 *   ACTIVE + DRIVER_OVERRIDE -> EXITING
 *   ACTIVE + CONDITIONS_LOST -> EXITING
 *   FAULT  + FAULT_CLEARED   -> READY
 */

/** 模式间切换转移表（NA/ACC/CP/NP/LP/NOA 之间） */
extern const TransitionRule SM_TABLE_MODE_SWITCHING[];

/**
 * 模式切换转移表：
 *   NA  + CONDITIONS_MET -> ACC (ACC_READY)
 *   ACC + MODE_UPGRADE   -> CP  (if lane detected)
 *   CP  + MODE_UPGRADE   -> NP  (if highway)
 *   NP  + MODE_UPGRADE   -> NOA (if HD map available)
 *   NOA + MODE_DOWNGRADE -> NP
 *   NP  + MODE_DOWNGRADE -> CP
 *   CP  + MODE_DOWNGRADE -> ACC
 *   ACC + MODE_DOWNGRADE -> NA
 *   任何模式 + CONDITIONS_LOST -> NA (NA_READY)
 *   任何模式 + SYSTEM_FAULT -> NA (NA_FAULT)
 */

/**
 * 获取驾驶模式名称字符串。
 * @param mode_id  SM_MODE_NA / ACC / CP / NP / LP / NOA
 */
const char* statem_mode_name(StateId mode_id);

/**
 * 获取子状态名称字符串。
 * @param sub  SM_SUB_READY / ACTIVE / EXITING / FAULT
 */
const char* statem_sub_state_name(int sub);

/**
 * 格式化完整分层状态名，如 "ACC:ACTIVE"、"CP:READY"。
 * @param state  完整 StateId = SM_MODE_STATE(mode, sub)
 * @param buf    输出缓冲区
 * @param size   缓冲区大小
 */
void statem_format_hierarchical(StateId state, char* buf, size_t size);

#ifdef __cplusplus
}
#endif

/* ══════════════════════════════════════════════════════════ */
/* C++ 包装（模板化，类型安全）                                */
/* ══════════════════════════════════════════════════════════ */

#ifdef __cplusplus

#include <string>
#include <vector>

/**
 * C++ 状态机包装器。
 *
 * 用法：
 *   struct MyTask : public CoroutineTask {
 *       ReflectiveStateMachine sm;
 *
 *       MyTask() {
 *           statem_init(&sm, SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "my_task");
 *           sm.on_entry = my_entry_action;
 *           sm.guard    = my_guard;
 *       }
 *
 *       bool start() { return statem_send_event(&sm, SM_EVENT_START, this); }
 *   };
 *
 * 反射查询：
 *   if (sm.can_transition(SM_EVENT_STOP)) { ... }
 *   for (auto ev : sm.allowed_events()) { ... }
 *   sm.dump();  // print transition table
 */
struct StateMachineCpp {
    ReflectiveStateMachine c;

    StateMachineCpp(const TransitionRule* table, StateId initial,
                    const char* name = nullptr) {
        statem_init(&c, table, initial, name);
    }

    /* ── 事件 ─────────────────────────────────────── */
    bool send(EventId event, void* task = nullptr) {
        return statem_send_event(&c, event, task);
    }
    bool start(void* task = nullptr)  { return send(SM_EVENT_START,  task); }
    bool stop(void* task = nullptr)   { return send(SM_EVENT_STOP,   task); }
    bool pause(void* task = nullptr)  { return send(SM_EVENT_PAUSE,  task); }
    bool resume(void* task = nullptr) { return send(SM_EVENT_RESUME, task); }
    bool restart(void* task = nullptr){ return send(SM_EVENT_RESTART,task); }
    bool error(void* task = nullptr)  { return send(SM_EVENT_ERROR,  task); }

    /* ── 查询 ─────────────────────────────────────── */
    StateId current() const      { return statem_current(&c); }
    bool can(EventId ev) const   { return statem_can_transition(&c, ev); }
    bool is_terminal() const     { return statem_is_terminal(&c); }
    const char* state_name() const { return statem_state_name(&c, c.current); }

    std::vector<EventId> allowed_events() const {
        EventId buf[16];
        int n = statem_allowed_events(&c, buf, 16);
        return std::vector<EventId>(buf, buf + n);
    }

    void dump() const { statem_dump_table(&c); }
    std::string json() const {
        char* j = statem_export_json(&c);
        std::string s(j ? j : "{}");
        free(j);
        return s;
    }
};

#endif /* __cplusplus */

#endif /* STATE_MACHINE_H */
