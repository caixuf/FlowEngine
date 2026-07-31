# 2026-07-31 碰撞根因排查与修复记录

> 本文记录 straight_road 场景反复追尾事故的完整排查链、已修复项与遗留问题。
> 配套调试手段见文末「新增调试手段」。

## 事故现象

120s demo 反复出现 `collision detected`（entity1/entity3/entity6 轮换），
`min_forward_gap` 负值（-1.9 ~ -3.3m），`invariant` spatial+temporal 失败。

## 根因链（三层）

### 第 1 层：决策链正常但执行链断流（主根因）

```
behavior FOLLOW/TTC/planning 目标在碰撞前 3.5s 全部正确（gap 32.7→4.9 渐进限速 6.9→0）
control MRM 全刹 brk=1.00 持续输出
safety_control 正常发布 control/cmd（tgt=0.0）
        ↓
flowsim cruise=1（内置巡航 thr=0.10）持续 15~30s+ —— 实证：cmd_age 从 50ms 一帧跳到 650ms 后持续增长
        ↓ 内置巡航以 15.1 m/s 巡航前进 → 追尾
```

**机制**：flowsim 的 `select_for(control/cmd)` + transport 回调 + peek 三通道
同时在某帧后永久收不到消息。已确认：

- 不是 bus 队列问题（depth=1 无积压、无 evict）
- 不是 unsubscribe_ex 误杀（精确匹配）
- **safety_control 的 `when_any_bus_for` 协程在运行 ~3s 后永久挂起**
  （#141 后零日志零发布，cleanup 正常 = 线程活着但 run 循环不再被 fire；
  同一时刻 control 正常发布 raw_cmd 但 safety 收不到）
- **只有 safety 卡死，其他节点（behavior/planning/control/flowsim/monitor）正常**

**遗留**：`WhenAnyBusAwaitableT`（coroutine_task.h）在同步执行器下的
fire/resume 生命周期问题——某帧后 timer 与消息双 fire 失效。**根因未完全
定位**，需要中间件专项调试（本轮以失效安全兜底止损）。

### 第 2 层：MRM 降级后置，停车指令不生效

`control_node.cpp` MRM 分支在 PID 之后覆盖 `acc_target`——throttle/brake
已按旧目标（巡航 12）算出，err 为正 → 输出油门（实测 MRM 下 thr=0.57/1.00）。
safety 的 gap/TTC 保护兜底全刹，但 control 自身指令无效。

**已修**：MRM 前置到 error 计算之前，MRM 时输出 thr=0/brk=1。

### 第 3 层：degrade 只升不降，MRM 永久卡死（幽灵停车）

`degrade_clear` 全代码库零调用者——L2 触发后永不恢复，车停在路中间
5s+（实测 avg_speed 掉到 7.9，停滞 12.5s）。

**已修**：control 停稳 3s 自动 `degrade_clear`。

## 已修复清单

| 修复 | 位置 | 效果 |
|------|------|------|
| 失效安全停车（控制丢失 → 减速停车而非内置巡航） | flowsim_node.cpp | 断流不再追尾，碰撞归零 |
| `message_bus_peek_latest` 主动窥视兜底 | message_bus.h/c + flowsim | 队列积压时指令持续新鲜 |
| 跟车 TTC 兜底（80m 窗口，4s TTC，感知+真值双源） | planning_node.cpp | 感知正常时渐进限速 |
| MRM 前置 + 停稳 3s 自动恢复 | control_node.cpp | 幽灵停车消失（avg 7.9→14.2） |
| blocked_range_mult 8→3.5 | behavior_planner_node.cpp | 启动 2s 即变道问题收敛 |
| 超车完成回内侧道归位 | behavior_planner_node.cpp | 红绿灯管辖车道恢复可见 |
| esmini 双平台库 + 格式门禁 + 离线自检 | lib/ + CMake + test_road_network | Linux 构建回归根治 |

## 新增调试手段

| 手段 | 内容 |
|------|------|
| `cmd_age` 字段 | flowsim 周期日志显示最近 control 指令新鲜度（ms）——一眼识别断流 |
| `cruise=%d` 字段 | flowsim 是否在内置巡航（断流标志） |
| `[BRK]` 日志 | 低速急刹时打印实际 thr/brk/指令年龄——刹车执行现场 |
| `[FSAFE]` 日志 | 失效安全停车触发记录 |
| `--json-out` samples 持久化 | demo_evaluator 保存全量 ego 轨迹，离线复盘 |
| `message_bus_peek_latest` | 高频关键指令的主动读取通道 |

## 验证命令

```bash
python3 ci/evaluators/demo_evaluator.py --duration 120 --interval 0.5 --json-out /tmp/eval.json
# 期望：无 collision FAIL；[FSAFE] 出现时车停不追尾
# 已知残余：safety 协程挂起 → 断流 → FSAFE 停车 → 演示中段卡停（不撞）
```
