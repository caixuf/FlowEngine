# Planning 速度规划升级设计 — ST 图 + DP/QP（2026-08）

> 状态：**M1/M2/M4 已实施**（b5fd449 / 8ab7327 / ed3136c）。M3（QP 精修）待做。
> 配套路线：`ALGORITHM_INTEGRATION.md`「规划-速度：自研 ST 图 + DP/QP」条目落地。
> 前置阅读：`CLAUDE.md` 故障模式表（本设计引用其中 10+ 条 planning 层根因）。
> 实现：`modules/adas_nodes/st_graph.{h,c}` + `tools/speed_planner_sim.py`（12/12 PASS）。

## 0. 实施记录（2026-08-05）

已落地（每个 milestone 独立可回退，见各 commit message）：

| 里程碑 | commit | 内容 |
|--------|--------|------|
| M1 | b5fd449 | ST 图 + DP 替代线性斜坡与红灯 override；视界动态扩展；全局时间 |
| M2 | 8ab7327 | 同向/静止障碍占据；时间基准 double-count 修复；占据区无条件禁止 |
| M4 | ed3136c | 掉头曲率-速度约束防线（κ=0.2534 → 4.22 m/s 极限） |
| M3 | — | QP 精修（`include/piecewise_jerk_qp.h` 已有 `pjqp_speed_solve`，待接） |

**仿真证伪的三个设计假设**（Python 先行抓到的真 bug，见各 commit）：
1. 占据检查时间基准 double-count：障碍位置用全局 t 会把 s0 再叠加 v·t0 位移，
   对向车 (v<0) 被算到车后 → 剖面全巡航 → 撞车。修复：障碍用相对时间 (t−t0)。
2. 占据区允许 v=0 停在里面：移动障碍会撞停着的车 → 占据区无条件禁止。
3. 同车道头对头「停车让行」模型错误：对向车会撞停着的车 → 对向车不进 ST 图
   （横向决策是 behavior 职责），会车 override（0.4× 降速）保留；
   behavior 掉头兜底（v≤5 触发）是触发事件不是速度 override，保留。
   Python 仿真数学验证：掉头弧 κ=0.2534 → 极限 4.22 m/s，behavior 兜底 v=5
   时 a_lat=6.33 超限 —— 2026-08-04 撞护栏根因的数学解释。

## 1. 背景与动机

CLAUDE.md 故障模式表按根因层统计：**planning/behavior 层 ≈10 条，safety 层 2 条（且诱因是 planning 输出的坏轨迹），control 层 3 条，flowsim/前端 3 条**。一半以上根因在决策+轨迹生成层。

这些 bug 表面各异（闯红灯/刹死/掉头撞护栏/变道全刹），机制却是同一个：

> **速度剖面 = 线性斜坡 + override 堆。每新增一种场景 = 新增一个 override = 新增一个 bug。**

升级目标：把「标量 command_speed + 10 个 override」重构为「**沿 s 的速度剖面求解器（ST 图 + DP/QP）**」，让红绿灯、障碍物、目标速度、曲率、制动距离统一进一个机制。删除全部 override 堆。

## 2. 现状解剖（问题定义）

### 2.1 速度剖面 = 线性斜坡

`planning_node.cpp:1924-1930`：

```c
double v0 = g.ego_v;  /* 用车子实际速度 */
for (int i = 0; i < n_wp; i++) {
    double t = (double)i / (double)(n_wp - 1);
    spd_out[i] = v0 * (1.0 - t) + command_speed * t;   /* 线性斜坡 */
}
```

- 50m 轨迹 / 10 点 → 每点 100ms、2s 内从 v0 线性过渡到 command_speed
- **无 jerk 约束**（2s 内 15→0 = 平均 7.5m/s²，远超舒适/安全减速度）
- **无制动距离约束**（刹不刹得住完全取决于 50m 轨迹够不够长）
- **无曲率约束**（`v ≤ sqrt(a_lat_max/|κ|)` 没人算 → 掉头进弯太快撞护栏）
- **无红绿灯距离自适应**（固定 2s 斜坡，灯前 60m 和 15m 是同一形状）

### 2.2 override 堆的 bug 机制

现状每加一种场景就叠一个 override：

| override | 位置 | 机制 | 历史 bug |
|----------|------|------|----------|
| 红绿灯强制停车 | `planning_node.cpp:1986-2034` | 来灯后整段重建 spd_out 斜坡到 0 | ① spd_out 在 command_speed 改后没同步重建 → 闯红灯（2026-07-31 修复）；② 停稳后 v=0 闭锁不走；③ 返程误停对向灯 |
| 会车让行 | planning 内（1221fad 后改为方向投影） | 对向车距离 → 压低速度 | 横向范围过宽 → 2+ 车道外对向车也刹停 |
| TTC 减速 | planning 内 | 前车 TTC → 压速度 | 与 behavior 的 target_speed 打架 |
| 掉头减速兜底 | behavior 侧 `v≤7` 触发 | 距离兜底强制减速 | 触发太早/太快 → 进弯 6.8m/s → 撞护栏（已改 5） |
| 变道锁死 | behavior P5 分支 | 变道首帧 blocked → target=0 | 锁死全刹（已删，改 TTC 兜底） |

**每个 override 都是独立代码路径，互不知晓、互相覆盖、各自有边界条件 bug。** 20+ commit 就是在给这堆 override 打补丁。

### 2.3 可行性检查是「事后判死刑」，不是「事前调速度」

`planning_node.cpp:2111-2132`（§8.5）：

```c
if (fabs(traj.points[i].kappa) > 0.25)  { traj.valid = 0; break; }
double a_lat = v² × |kappa|;
if (a_lat > 5.0) { traj.valid = 0; break; }
```

- 正确做法：**速度剖面生成时就把曲率约束算进去**（`v ≤ sqrt(5.0/|κ|)`），轨迹永远可行
- 现状：先按巡航速度生成，再判死刑 → 掉头场景「进弯速度从没被压过」，只能靠 behavior 距离兜底（7→5 m/s）当最后防线。**设计上不该靠兜底。**

## 3. 目标架构

### 3.1 职责重划（不越界，符合 CLAUDE.md 铁律）

```
behavior（做什么：变道/掉头/跟车/停车意图）
    │  planning/behavior: command, target_speed, target_lane_idx
    ▼
planning（怎么走）—— 速度与轨迹的唯一权威
    ├─ 路径生成：Frenet 横向（已有，不动）
    ├─ 速度规划：ST 图 + DP/QP（新增）← 速度剖面唯一来源
    │    输入：command_speed（行为目标，不再是硬约束）、
    │          障碍物 ST 占据、红绿灯时距、路径 kappa 剖面、限速
    │    输出：沿 s 的 v(s)/a(s)/jerk(s) → spd_out[i]
    └─ 轨迹组装 + 可行性校验（校验改为「违反即回退重解」，不再是发布 invalid）
    ▼
control（纯轨迹跟随）+ safety_control（纯闸门，机动窗口豁免不变）
```

- **behavior 的 target_speed 从「硬约束」降级为「优化目标」**：行为说"跟停"就是 v_target=0 的剖面求解，不是把斜坡砍成 0
- **planning 内部新增一个速度规划子模块**，红绿灯/会车/TTC/曲率全部变成它的输入约束，全部 override 删除
- **safety_control 不动**：闸门职责不变，planning 输出可行动迹后它自然少触发（幽灵刹车/TTC 误报的诱因消失）

### 3.2 ST 图 + DP/QP 流程

```
每帧（10Hz）：
  1. 投影障碍物到 Frenet：obstacle(s, d, v_s, t) → 本车道 s-t 占据图
     （本车道 ± 半路宽内的障碍物画入，其余车道交给 behavior 决策）
  2. 红绿灯转成时距约束：灯距 s_l、剩余时间 t_rem →
     红线: s > s_l - 安全余量 的时间段内 v(t) 必须满足「灯变绿前不越过停止线」
     黄灯闪烁期 → 保守减速段
  3. 静态约束：限速 v_max(s)、曲率约束 v_curve(s) = sqrt(a_lat_max/|κ(s)|)、
     制动距离自洽（任一点 v 需保证能在最近停点前刹住）
  4. DP 粗解：s-t 网格采样，cost = Σ(ω1·|v-v_target| + ω2·|a| + ω3·风险项)，
     搜索一条避让占据区、满足静态约束的粗剖面
  5. QP 精修（可选，M3）：对粗剖面做 jerk/加速度约束的二次规划平滑
  6. 输出 spd_out[]（复用现有 Trajectory 构建路径）
```

## 4. 详细设计

### 4.1 输入契约（全部已有 topic，无新增订阅）

| 输入 | 来源 | 用途 |
|------|------|------|
| command / target_speed / target_lane_idx | behavior（`planning/behavior`，已订阅） | 优化目标 + 横向路径 |
| 障碍物 (x,y,vx,vy,类别) | perception（`perception/obstacles`，已订阅 g.obs_*） | ST 占据 |
| 红绿灯 (x, state, 管辖车道) | scene（`road/traffic_lights`，已订阅 g.tl_*） | 时距约束 |
| kappa 剖面 | 本模块路径生成（Frenet→Cartesian 已算 κ） | 曲率约束 |
| 限速 / 车道数 / 路宽 | flowsim scene（已订阅） | 静态约束 |

### 4.2 数据结构（新增，planning_node.cpp 内私有）

```c
/* ST 图：s (0..50m) × t (0..2s)，分辨率 1m × 0.1s（50×20 网格，栈上够装） */
typedef struct {
    double s;          /* Frenet 弧长 */
    double t;          /* 时间 */
    uint8_t occupied;  /* 1=占据（障碍物时空交集） */
    double v_lim;      /* 该点速度上限（曲率/限速/制动自洽取 min） */
} StCell;

typedef struct {
    StCell grid[50][20];
    double v_target;   /* behavior 目标（0=停车） */
    double v0;         /* 当前车速 */
    double a_max;      /* 最大减速度（默认 -4.0） */
    double a_lat_max;  /* 横向加速度上限（= 可行性检查 5.0 同一常量） */
    /* 结果 */
    double v_out[50];  /* 每个 s 网格的速度（下采样到 spd_out） */
} StGraph;
```

### 4.3 速度剖面算法（DP 粗解，纯 C，无第三方依赖）

```
1. 静态约束初始化
   v_lim[s] = min(v_max, sqrt(a_lat_max / |κ(s)|))
   → 掉头/弯道段自动压速，不再需要 behavior 距离兜底

2. 障碍物占据（只画本车道 ± 半路宽内、同向运动学可交集的）
   for each obs: 投影到 (s, d)，若 |d - ego_d| < 半路宽：
      t 区间 = [(s-车长-余量)/obs_v, (s+车长+余量)/obs_v]，网格置 occupied
   → 会车/TTC override 全部删除，统一进占据图

3. 红绿灯时距约束
   灯前 s_l：剩余 t_rem 内禁止越过停止线 →
     红线 [t_in, 2s] 内 v(t) ≤ (s_l - 余量) / t
   → 红灯 override（1986-2034 整段删除）

4. DP 搜索（网格列主序，每列取使累计 cost 最小的 v）
   cost = ω1·(v-v_target)² + ω2·a² + ω3·occupied 惩罚
   约束：|a| ≤ a_max，v ≤ v_lim，occupied 格 v ≤ 0（或换道由 behavior 决策）
   向前推进：v[k+1] = clamp(v[k] + a·dt, 0, v_lim[k+1])
5. 下采样 spd_out[i] = v_out[s_i]
```

### 4.4 与现有代码的接入点（云端实现改动清单）

| 位置 | 改动 |
|------|------|
| `planning_node.cpp:1924-1930`（线性斜坡） | 替换为 `st_graph_solve()` 输出 |
| `planning_node.cpp:1986-2034`（红灯 override 整段） | **删除**，红绿灯进 ST 图约束 |
| 会车让行（planning 内） | **删除**，对向车占据图按横向过滤后自然避让 |
| TTC 减速（planning 内） | **删除**，前车占据图自然避让 |
| `planning_node.cpp:2111-2132`（§8.5 可行性） | 改为「违反 → 用违反点做约束重解一次」，仍违反才 invalid |
| `generate_uturn_trajectory` 段（1741-1813） | 掉头轨迹速度段接入 ST 图曲率约束（Phase M4） |
| behavior 掉头距离兜底（`v≤5` 触发） | **删除**（曲率约束替代后成为死代码） |

### 4.5 删除的 override 清单（回归验证据此核对）

- [ ] 红灯强制停车 override（`planning_node.cpp:1986-2034`）
- [ ] 会车让行（planning 内）
- [ ] TTC 减速（planning 内）
- [ ] behavior 掉头距离兜底（`v≤7→5` 触发）
- [ ] 变道首帧 blocked 锁速（已删过，确认无残留）

## 5. 里程碑拆分（每步可验证、Python 仿真先行）

| 里程碑 | 内容 | 验证 |
|--------|------|------|
| **M1 静态剖面** | ST 图 + DP（无障碍物）：红绿灯时距约束 + 曲率约束 + 制动自洽，替换线性斜坡与红灯 override | Python 仿真：红灯刹停距离/曲线/无闯行；C++ 移植后故障表回归用例 ①③ |
| **M2 障碍物避让** | ST 占据图：前车/对向车进图，删会车与 TTC override | Python 仿真：跟停/会车/加塞场景；故障表回归 ② |
| **M3 QP 平滑**（可选） | 粗剖面 + jerk 约束二次规划（先手写投影/内点，必要时引 OSQP） | 仿真输出 jerk 曲线 vs M1/M2 对比；无抖动回归 |
| **M4 掉头接入** | UTurn 轨迹速度段走 ST 图曲率约束；删 behavior 兜底 | 掉头场景：进弯速度 ≤ sqrt(5/κ_max)、无护栏碰撞 |

> **每个里程碑独立可合并、可回退**。M1 落地即删除一半 override。

## 6. 验证方案

### 6.1 Python 仿真先行（CLAUDE.md 强制流程）

新建 `tools/speed_planner_sim.py`（对齐 `control_sim.py` 风格）：

- 场景：红灯刹停（60m 预警/30m/15m 三档起始距离）、跟停、会车、曲率段进弯、掉头
- 输入：路径 κ(s) 剖面 + 障碍物 + 灯 + v_target；输出：v(s) 曲线
- 断言：不闯停止线、|a| ≤ a_max、|a_lat| ≤ 5、jerk 有界、无死锁（v 收敛到 target）
- `--run-all` / `--tune-*` 参数扫描同 `control_sim.py`

### 6.2 故障表回归用例（移植后必跑）

| # | 场景 | 曾经的现象 | 通过标准 |
|---|------|-----------|----------|
| ① | 红灯前 15m 内 12m/s | 只缓到 6.2 闯灯 | 停止线前 v→0，不越过 |
| ② | 多车道对向车（2+ 车道外） | 全刹到 0 | 横向 > 1.5 路宽的车辆不触发减速 |
| ③ | 掉头进弯 | 6.8m/s 撞护栏 | 进弯 v ≤ sqrt(5/κ_max)，OFFRAILS exit 在路内 |
| ④ | 变道遇静止前车 | 首帧全刹 | 剖面平滑减速，不瞬间归零 |

### 6.3 现有门禁

- `python3 tools/pipeline_check.py`（L0）
- `python3 ci/evaluators/demo_evaluator.py --duration 45`（L1，跑故障表回归场景）
- `ci/evaluators/scenario_regression.py`（L1.5，改前 --update-baseline）
- Python 仿真 `--run-all` 全量 PASS 后才允许移植

## 7. 风险与边界（对照职责铁律）

- **planning 仍是速度与轨迹的唯一权威**：ST 图是 planning 内部实现，behavior 的 target_speed 只是优化目标，control/safety 接口不变
- **不为 ST 图引入新依赖**：DP 是纯 C 网格搜索，QP 先手写、OSQP 仅当 M3 平滑质量不够再引（嵌入式零依赖优先）
- **障碍物画图只画本车道**：跨车道决策是 behavior 的职责（变道/绕行），速度规划不越界做「该不该换道」的判断
- **返程/掉头方向语义**：障碍物投影沿用现有 `road/ref_path.reverse` 同源方向推导，不许再各自猜
- **停稳闭锁防复发**：M1 起 DP 搜索必须显式支持 v_target=0 的收敛（红灯绿了 target 恢复 → 剖面自然回正），回归用例 ① 含停→走的完整闭环
