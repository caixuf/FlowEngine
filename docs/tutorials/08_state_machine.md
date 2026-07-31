# 08 — 反射式状态机

## 核心理念

状态机**不仅会跑，还知道自己的结构**。这就是"反射"——运行时能拿出转移表、事件表、转移历史。

```
[statem:fusion] INITIALIZED + START -> RUNNING
[statem:fusion] RUNNING + STOP -> STOPPING
```

## 快速开始

### C — 嵌入 TaskBase

```c
// TaskBase 自动初始化 sm（标准 task 生命周期转移表）
// task_base_init 后 sm_enabled = true, trace_enabled = true

// 反射查询
if (task_can_event(task, SM_EVENT_STOP)) {
    task_stop(task);
} else {
    printf("当前状态 %s 不允许 STOP\n", statem_state_name(&task->sm, statem_current(&task->sm)));
}

// 打印转移表
task_dump_sm(task);
```

### C++ — 独立使用

```cpp
StateMachineCpp sm(SM_TABLE_STANDARD, SM_STATE_INITIALIZED, "my_task");
sm.start();       // INITIALIZED → RUNNING
sm.can(STOP);     // true
sm.dump();        // 打印完整转移表 + 允许事件 + 历史
sm.json();        // 导出 JSON
sm.stop();        // RUNNING → STOPPING
```

### 条件管理器（运行时换 guard）

```c
// 移除所有限制
statem_set_guard(&task->sm, NULL);

// 运行时安装新 guard
bool my_runtime_guard(void* task, StateId from, EventId ev, StateId to) {
    return is_safe_to_transition(task, from, ev);
}
statem_set_guard(&task->sm, my_runtime_guard);
```

### 调试 hook

```c
void my_debug(void* task, StateId from, EventId ev, StateId to,
              const char* desc, bool accepted) {
    printf("[TRACE] %s: %s + %s -> %s (%s)\n",
           ((TaskBase*)task)->config.name,
           statem_state_name(NULL, from),
           statem_event_name(NULL, ev),
           statem_state_name(NULL, to),
           accepted ? "OK" : "REJECTED");
}
statem_set_debug_hook(&task->sm, my_debug);
```

## ADAS 驾驶模式状态机

预定义的驾驶模式分层状态机：

```
模式层:  NA ─→ ACC ─→ CP ─→ NP ─→ NOA
         ↑                        │
         └── CONDITIONS_LOST ──────┘

子状态层 (每个模式内部):
  READY ─ACTIVATE→ ACTIVE ─DEACTIVATE→ EXITING ─EXIT_DONE→ READY
    │                  │                  │
    └─── SYSTEM_FAULT ─┴─ SYSTEM_FAULT ───┘
         → FAULT ─FAULT_CLEARED→ READY
```

```c
// 使用驾驶模式状态机
statem_init(&sm, SM_TABLE_MODE_SWITCHING, SM_MODE_NA, "driving");
statem_send_event(&sm, SM_EVT_CONDITIONS_MET, NULL);  // NA → ACC
statem_send_event(&sm, SM_EVT_MODE_UPGRADE, NULL);    // ACC → CP

// 格式化输出
char buf[64];
statem_format_hierarchical(statem_current(&sm), buf, sizeof(buf));
// buf = "CP:READY"
```

### 实车接入：planning_node 中的模式仲裁

`modules/adas_nodes/planning_node.c` 已经把这个状态机接入了真实 pipeline（不再只是
demo 里的一次性示例）：

- 每 1s 检查一次升级条件（`mode_transition_guard`）：
  `NA→ACC` 需要有传感器数据；`ACC→CP` 需要融合定位就绪；`CP→NP` 需要车速持续
  高于 `highway_speed_mps`（`pipeline.json` 参数）达到 3s；`NP→NOA` 需要场景文件
  (`scenario_file` 参数) 里定义了至少一条 `route` 导航路线。
- 车速骤降 / 融合数据超时 (`CONDITIONS_LOST`) 会触发降级（`check_mode_downgrade`）。
- 当前模式通过 `mode=<hier>` 字段附加在 `planning/trajectory` 消息里发布。
- 进入 NOA 后，场景 `route[]` 里到达 `trigger_x` 的步骤会把目标车道写入
  `route_lane=<±1>` 字段；`control_node.cpp` 消费该字段，复用既有的安全变道
  状态机（`lane_rear_safe`/`lane_has_pedestrian_risk`）主动发起变道
  （日志标记为 `NOA_ROUTE`），从而实现"导航驱动的主动变道"而非仅有的
  "障碍物触发的被动超车"。
- 默认 demo 场景 (`infinite_straight.json`) 没有 `route` 字段，所以模式最高
  只能到 `NP`（NOA 的 guard 会一直拒绝），行为与升级前保持一致；
  `scenarios/zhongkai_road_full.json` 提供了一个演示 NOA 主动变道的示例场景。

## API 速查

| 函数 | 用途 |
|------|------|
| `task_can_event(task, event)` | 当前状态能否接收事件？ |
| `task_allowed_events(task, buf, max)` | 获取允许的事件列表 |
| `task_dump_sm(task)` | 打印完整转移表 |
| `statem_set_guard()` | 运行时替换 guard 条件 |
| `statem_set_trace()` | 开启/关闭转移日志 |
| `statem_set_debug_hook()` | 注入调试回调 |
| `statem_add_transition()` | 动态添加转移规则 |
| `statem_export_json()` | 导出 JSON |
