# Debugging — FlowEngine 分层排查方法论

排查"现象不对劲"的系统性方法。提炼自 2026-07-31 一次连续修复 8 个连环
bug 的真实会话（转向灯反 → 变道全刹 → 归位遇红灯 → 会车让行过度保守 →
红灯闯行 → 缓存掩盖修复 …）。

## 什么时候用

- 仪表盘/3D 行为异常：转向灯方向反、该停不停、该走不走、莫名刹停到 0
- "改了代码但现象不变"（最常见：改错层了，或被缓存/旧构建掩盖）
- 同一现象修了好几次都没好 → 大概率根因不止一层，或根本没改对层

## 核心原则：先复现 → 分层 → 逐层实证，别猜

现象出现在"最终效果层"，但根因可能在任何一层。**不要凭直觉判断哪层错**，
用探针逐层确认后再动手。FlowEngine 典型链路（按数据流向）：

```
behavior 决策 → planning 轨迹(command_speed→spd_out→points[].v)
  → control target_speed(PID) → safety_control 兜底 → flowsim 物理
  → scene_pub 序列化 → 前端 deriveLightState → _setVehicleLights → 模型 mesh
  → 浏览器渲染（还有一层：HTTP 缓存可能给你旧资产！）
```

## 方法 1：探针法（本会话用得最多的一招）

在可疑的每一层加临时日志，打印它**实际发送/看到**的值，跑一遍 demo，
用数据定位是哪一层出的问题，然后把探针删掉。

```c
/* control 层探针：变道时实际发的 turn_signal + ego_y（用于对比移动方向） */
if (turn_signal != 0) {
    LOG_WARN("control", "[TS] cyc=%d beh_cmd=%d ts=%d ego_y=%.2f",
             g.cycle, (int)g.beh_command, (int)turn_signal, g.ego_y);
}
```

案例：转向灯"打反"——control 探针显示 `RIGHT_CHANGE→ts=2` 且 ego_y 减小
（向右），逻辑层全对；问题实际在**浏览器缓存了旧模型**（见方法 5）。

## 方法 2：验证值真的传播到了消费者

"修"了 A 层的值，必须确认它**到了 B 层的消费点**。只打印"已设置"不算数，
要看消费者读到的值。

案例：planning 红灯 override 设了 `command_speed=0`（探针显示 `[RED] cmd=0`
已触发），但 `spd_out` 在 override **之前**已按巡航速度生成，轨迹
`points[].v` 读的是旧 `spd_out` → control 拿到的 target 还是 12，**直接闯
红灯**。这个 bug 被"跟停红灯前停着的车"掩盖了很久。修复：override 触发时
同步重建 spd_out（当前速度→0 斜坡）。

## 方法 3：警惕状态锁死

状态机/决策分支里设的值，如果**没有别的分支重置它**，会永久锁住，且看起来
"偶尔正常"（因为只在某个条件分支触发）。

案例：变道中 P5 分支 `else if (blocked) target = lead_speed`——变道转移首帧
blocked=1、前车停着（lead_speed=0）→ target=0；后续 blocked=0 的帧没有任何
分支重置它 → **锁死** → planning command_speed=0 → 变道全刹（spd 10.7→0）。
修复：删除该分支，防追尾交给下层 TTC 兜底。

排查法：如果一个量"设了就不变"，检查所有能重置它的路径；没有就是锁死。

## 方法 4：同模式跨层搜索

一个行为错了，可能**同一段逻辑在多个节点各复制了一份**，只修一处不够。
修完后 `grep` 相同的判定模式，把所有副本一起改。

案例：会车让行过度保守——planning 会车让行 **和** safety_control 对向 TTC
都有 `|dy|>2.0` 判"对向车"。只修 planning 后 ego 仍刹停（safety 还在兜），
两处都要加横向相邻上界（`1.5×路宽` / `6.0m`）。

## 方法 5：缓存/部署层

改了代码/资产但现象不变 → **先查客户端是否真的拿到了新版本**，再回头查代码。
浏览器缓存、旧构建、CDN 都可能掩盖修复。

案例：模型 L/R 修复后用户仍看到反的转向灯。根因：`monitor_server.c` 把
`/models/*` 标 `Cache-Control: immutable`（1 年），浏览器永不重拉。修复：
models/ 改 `no-cache` + 前端模型 URL 加 `?v=` 缓存破坏号。

排查法：改了资产后，`curl -sI <url>` 看 Cache-Control，或硬刷新/清缓存再测。

## 方法 6：可观察性设计

如果 demo **从不出现在某个行为**，你无法验证它，也没法判断修没修好。
调场景/相位让行为可观察。

案例：用户说"每次到红绿灯前都是绿灯，判断不了红灯停绿灯行"——两个原因：
(1) planning 红灯 override 失效（方法 2 的 bug）；(2) 灯相位凑巧在 ego 到达
时是绿灯/黄灯。修复 override 后，再把第一个灯相位 offset 调到让 ego 在普通
45s demo 里真遇到红灯，红灯停/绿灯行就可观察了。

## 修完必跑（避免边修边引入新 bug）

```
bash scripts/demo.sh --no-browser 45          # 看行为时序
python3 ci/evaluators/demo_evaluator.py       # 端到端回归（碰撞/频率/拓扑）
python3 tools/pipeline_check.py               # 秒级离线检查
npm run vis:check:all                         # 3D 门禁（改了 flowboard/ 时）
./build/bin/test_adas_nodes_logic && ./build/bin/test_new_modules
# 最后：删掉所有 TEMP-PROBE 日志，确认无残留
```

## 故障模式速查（2026-07 新增，详见 CLAUDE.md 故障表）

| 现象 | 根因 | 本会话教训 |
|------|------|-----------|
| 转向灯左右反 | 模型 L/R 标反 + 浏览器 immutable 缓存掩盖修复 | 分层验证 + 查缓存（方法 1、5） |
| 变道全刹/超车没意义 | 变道中 target 被设 0 且锁死 | 状态锁死（方法 3） |
| 归位/超车进红灯车道刹停 | 决策不看目标车道红绿灯 | 跨层数据要订阅（lane_ahead_stop_light） |
| 会车让行把 2 车道外对向车当威胁刹停 | planning + safety 各有一份 `\|dy\|>2.0` | 同模式跨层搜索（方法 4） |
| 红灯不停直接闯 | override 设 cmd=0 但轨迹没重建 | 值传播到消费者（方法 2） |
| 改模型但现象不变 | models/ immutable 缓存 | 缓存层（方法 5） |
| demo 从不演示红灯停 | 灯相位凑巧绿灯 | 可观察性设计（方法 6） |

## 纪律

1. **一次只改一个变量**，改完立刻验证，别攒一堆改动再猜。
2. **探针必删**：临时日志标记 `TEMP-PROBE`/`TEMP-DEBUG`，验证完删除，不留库。
3. **每层都验证过再下结论**：本会话最大的教训是——"现象错"不一定"逻辑错"，
   中间可能隔着缓存、序列化、渲染、物理模型任何一层。
