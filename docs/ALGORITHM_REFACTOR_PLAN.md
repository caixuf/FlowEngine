# 算法层重构方案：从 demo 到量产级

> 本文是**执行计划**，不是参考设计。每一层给「demo 现状 / 量产判据 / 改造 / 门禁」。
>
> 相关文档：`REAL_VEHICLE_ROADMAP.md`（硬件与驱动侧待办，与本文互补，本文只谈算法层）、
> `ALGORITHM_STACK.md`（参考设计与性能目标，其中示例文件未实现）、
> `CALIBRATION_GUIDE.md`（标定流程）。
>
> 事实来源：接口/topic/字段/配置值来自 2026-07-29 对仓库的读取。
> 标注「待复核」的行号请在落地时重新确认。

---

## 0. demo → 量产的三条主轴

这三条是 demo 与量产的真正分界，比「哪个算法更高级」更具决定性。任何一层，这三条没过，
堆多少建模精度都还是 demo。

**主轴一：去真值捷径。** demo 允许从仿真真值取数，量产不允许。见 §1 清单。

**主轴二：定时有界。** demo 里「点云多了聚类超 deadline 就降频」可接受；量产必须 WCET 有上界。
具体含义：循环内零动态分配、算法规模有固定上限、求解器有迭代上限且失败有确定返回、
每层有 p99 耗时预算门禁。

**主轴三：故障可观测 + 有降级路径。** demo 的失败方式是「效果变差」，量产的失败方式必须是
「进入已定义的降级态并上报原因码」。当前 `mpc_solve` 不收敛、求逆失败、线搜索耗尽全部
`return 0`（`src/core/mpc_controller.c`，待复核行号），这在量产是零容忍。

---

## 1. 真值捷径清单

量产化的实际工作量集中在这里。

| # | 捷径 | 位置 | 拆掉后立刻暴露什么 |
|---|------|------|------------------|
| 1 | 障碍物速度 = `vehicle/state` 真值 3 m 最近邻匹配（`best_d2 = 9.0`）后旋回车体系 | `modules/adas_nodes/perception_node.cpp` | 预测输入变成带噪 KF 速度，预测误差指标才有意义 |
| 2 | 红绿灯真值 `road/traffic_lights` 直连 planning，并硬加 20 m「补 fusion 滞后」余量 | `modules/adas_nodes/planning_node.cpp` | 停车线控制要重做；那 20 m 魔法数本身就是捷径的补丁 |
| 3 | 车道几何真值 `road/geometry` 直连 planning + control，无车道线感知 | 两处各自独立推导 | 参考线必须成为唯一事实源，否则 planning/control 看到不同几何 |
| 4 | ego heading 每帧被重置为道路切线 | `modules/adas_nodes/flowsim_node.cpp` | 横向阻尼三项恒 0，极限环是结构性的 |
| 5 | route = 场景脚本 `route[8]` + `ego_x >= trigger_x` 触发 | `include/scenario_loader.h` / `planning_node.cpp` | 变道时机来自脚本而非导航，NOA 类行为无法泛化 |
| 6 | 未标定魔法数（匹配半径 9.0、聚类 min 3 点、Frenet 权重等） | 多处 | 需要标定流程而非手调 |

第 4 条性质特殊：它不是感知作弊，是**被控对象**作弊。但影响相同 —— 在它拆掉之前，
本仿真里测出的任何控制器数字都不可信。

**M1 结束时 `demo_evaluator` 大概会变红。** 那是拆掉捷径的必然结果，不是回退。
先接受这次变红，是整个计划能否成立的分水岭；若为保住绿灯而留着捷径，
M2 做出来的所有算法都是在真值上调参。

---

## 2. 消息契约（阻塞五层中的四层，必须先做）

三个 P0 窟窿都卡在同一个前置：`planning/trajectory` 现在是 50 个 `(s, d, speed)` 三元组，
**无时间轴、无 heading、无曲率**，外挂一段 `strstr` 解析的文本尾巴。

- 预测要给规划用，得把预测轨迹投到 ST 图 → 需要时间轴
- QP 平滑要约束 jerk、要曲率限速 → 需要 κ 和 a
- 轨迹拼接的整个机制是「在上一条轨迹的 t_start 处取点作为本次初值」→ 没有绝对时间无从谈起

用 `msg/adas_msgs.msg` 生成二进制，与 `ObstacleList` 一致，不再手拼 JSON + 文本尾。

```
TrajectoryPoint: t_rel_us u32 | x,y f | s,l f | heading f | kappa f | v,a,jerk f
Trajectory:      seq u32 | stamp_us u64 | ref_line_id u32 | point_count u16
                 | points[64] | planner_state u8 | is_stitched u8 | valid u8
```

三个设计点：

- **同时带 `(x,y)` 和 `(s,l)`**：控制器要世界系，拼接与 ST 要 Frenet
- **`ref_line_id`**：参考线变了就禁止拼接、强制重规划。否则会在几何不连续处静默拼接
- **逐点 `t_rel_us`**：拼接、延迟补偿、按时刻做碰撞检查，全靠它

同批新增：

- `PredictionSet` —— 每障碍 ≤3 模态，每模态 25 点，带 `sigma_x/sigma_y`
- `planning/reference_line` —— `s,x,y,θ,κ,dκ,left_bound,right_bound,speed_limit`，变化时才发。
  单独成 topic 很关键：现在 prediction 要用的车道几何、planning 用的、control 用的各自独立推导，
  这本身就是发散源
- 帧约定写进 msg 文件注释：`Obstacle.x/y/vx/vy` 是**车体系**（每个消费者各自旋转），
  跟踪与预测输出统一**世界系**。「多份位置表示手工同步」是本仓库一族 bug 的共同来源

调试字符串挪到 `planning/debug`。

估时 2 d。改造清单需覆盖 planning / control / safety_control / inference / data_recorder / monitor
六个消费者。

---

## 3. L0 仿真 / 被控对象层

不在算法分层表里，但门着控制层。

| | |
|---|---|
| demo 现状 | 运动学自行车模型；`dynamics` 是桩，降级到运动学；`flowsim/entity.h` 的 `v_x_body/v_y_body/yaw_rate/F_yf/F_yr` 字段存在但从不写入；heading 每帧重置为道路切线 |
| 量产判据 | 被控对象至少有横向动力学与 yaw 惯量，能复现真车的相位滞后与执行器延迟 |

### 改造

1. **heading 由自行车模型自由积分，同时横向位移改由 `v·sin(heading − 道路切线)` 驱动
   —— 必须同一个 commit。** 只改前者上次已试过并被回滚：位移不吃 heading 就是开环积分器，
   任何 steer 偏置让车斜行（crab）。两处同改才闭环：heading 偏 → 横向漂 →
   控制器见 lat_error → 纠 steer → heading 回正，结构上跑不飞
2. 补线性轮胎自行车模型（`flowsim/physics.cpp`）：`v_y`、`yaw_rate` 为状态，
   3 m/s 以下向运动学混合以避开 `1/v` 奇点
3. 执行器模型：一阶滞后 + 速率限幅 + 死区，参数可配 —— 控制层的延迟补偿需要一个
   可标定的对象来验证

### 陷阱

不要写 `heading = road_h + atan2(lat_rate, v)`。因 `lat_rate = v·tan(steer)`，
它代数化简成 `heading = road_h + steer`，是比例反馈不是阻尼，只会把环路增益降约 2.75 倍，
顺带让变道慢 2.75 倍（即 commit `3d092ad` 修过的「变道横向不动」）。
阻尼必须来自有相位滞后的真实状态积分。

### 门禁

阶跃 steer 输入的 yaw 响应对比解析解；无源性/能量检查；`yaw_rate` 非零且发布。

估时 2 d。

---

## 4. L1 感知层

| 维度 | demo 现状 | 量产判据 |
|------|-----------|----------|
| 聚类 | DBSCAN，点云多时超 deadline 降频 | WCET 有界：体素下采样到固定上限点数 + 网格化近邻，O(n) |
| 速度 | 真值最近邻（捷径 #1） | 纯量测推导 |
| 跟踪 | 无 —— `object_tracker_node.c` / `src/algorithms/kalman_tracker.c` 在仓库里但不在 pipeline.json | 稳定 id + 协方差 + age + 生命周期 |
| 相机 | `sensor/camera` 在发但零消费者 | 车道线多项式 + TL 状态 + 交通标志 |
| 时间戳 | 发布时刻 | 采集时刻，融合按采集时刻对齐 |
| 帧约定 | 车体系，每个消费者各自旋转 | 在 msg 文件显式声明，跟踪输出统一世界系 |

### 改造

1. **`object_tracker_node` 进 pipeline**：per-object CV/CA Kalman，马氏距离门控 + 匈牙利关联，
   tentative → confirmed → lost 的 N-hit/M-miss 生命周期，输出 `perception/tracked_objects`
2. **删掉真值匹配分支**，速度全部来自 KF
3. 输出补量产必需的元信息：`cov`、`age`、`occluded`、`truncated`、`out_of_fov`
   —— 预测和规划靠这些决定信任度
4. **相机链路诚实化**：`sensor_model_node` 输出带噪声/漏检/有效距离的车道线与 TL；
   perception 发 `perception/lane_lines`、`perception/traffic_lights`。
   这会立刻暴露捷径 #2 那个 20 m 魔法数

### 门禁（FAIL 级）

MOTA/MOTP、id switch 计数、漏检率-距离曲线、聚类 p99 耗时上界、
`sensor/lidar` → `perception/obstacles` 端到端延迟。

估时 1.5 w（跟踪 4 d、相机链路 4 d、门禁 2 d）。

---

## 5. L2 预测层（P0 #1）

| 维度 | demo 现状 | 量产判据 |
|------|-----------|----------|
| 存在性 | `prediction_node.c` 不在 pipeline；三模态固定先验 0.7/0.2/0.1 | 概率随观测变化 |
| 不确定度 | 无 | σ(t) 随时域增长 |
| 交互 | 无（冻结世界） | 至少单次 ego-conditioned 迭代 |
| 计算 | — | `N_obs × N_mode × N_step` 固定上限，循环内零动态分配 |

固定先验这点值得单列：它意味着规划分不出「正在切入的车」和「保持车道的车」，
那预测对规划就是零信息。所以意图分类的概率必须真的动。

### 改造

1. **前置：修数据源。** 拿 oracle 速度做预测，预测模块永远测不出问题。依赖 §4.1
2. **意图分类**：每 track 投影到参考线得 Frenet `(s, l, ṡ, l̇)`。特征 6 个 ——
   横向偏移、横向速度、heading 与车道切线夹角、到车道边界距离、目标车道前后 gap、
   转向灯（若有）。logistic/softmax 输出
   `{LANE_KEEP, LEFT_CHANGE, RIGHT_CHANGE, TURN_LEFT, TURN_RIGHT, YIELD}`。
   **先手标定线性模型，不要一上来上学习模型** —— 可标定性优先于表达力
3. **多模态轨迹**：LANE_KEEP 沿参考线等速 + 对其自身前车做 IDM 减速；
   LANE_CHANGE 五次多项式横向过渡，时长由意图强度定（3–5 s）；TURN 跟随 routing 后继车道。
   时域 5 s，dt 0.2 s → 25 点
4. **不确定度**：附 σ(t)（线性或 √t 增长）。有 σ 才能做风险感知规划，而非二值碰撞判定
5. **交互（第二步再做）**：ego-conditioned 是正解但贵。务实中间态 —— 取上一周期 ego 轨迹，
   对 ego 后方/侧方车辆用 IDM（ego 当作其前车）修正纵向 profile，**单次迭代**。
   便宜，但能消掉「冻结世界」的过度保守。第一阶段不碰博弈论

### 孤儿处置

`include/intent_predictor.h` + `src/core/intent_predictor.c` 头文件自己写了 zero callers。
建议删 —— 交互模型重新推导比复活便宜。

### 门禁（FAIL 级）

flowsim 知道 NPC 真实未来，离线可算：

- ADE@3s、FDE@5s、miss-rate@2m
- **变道意图在实际跨线前 ≥1.5 s 的召回率** ← 最关键

最后一条最重要：车已压线才报变道等于没预测，而前三个指标察觉不到这件事。

真值只能进离线打分器，**不得进在线链路**。

估时 1.5 w。

---

## 6. L3 定位层

| 维度 | demo 现状 | 量产判据 |
|------|-----------|----------|
| EKF | `src/algorithms/ekf_fusion.c` 5 状态，`predict` 由测量到达驱动 | IMU 高频驱动 predict，观测异步注入 |
| 观测 | GPS + LiDAR | 加车道线横向偏移观测 |
| 完好性 | 发布了 `cov_yy` / `diverged`，**零消费者** | innovation χ² 检验 → 降级；协方差进规划安全余量 |
| 延迟 | 无补偿 | OOSM（量测时刻回溯更新）或状态缓冲重传播 |
| 冗余 | 单源 | GNSS + DR 双源，输出 protection level |

「车道线横向观测」在真实世界里是压住横向不确定度的主力项，比再提一档 GPS 精度有用得多，
且它把 §4 相机链路的价值变现。

### 孤儿处置

`modules/adas_nodes/ekf_slam.{c,h}` + `slam_node.cpp` 不在 pipeline。先打
`@deprecated superseded-by=... remove-by=<日期>`，量产阶段决定纳入或删除
—— 项目规范不允许无 `remove-by` 的死实现长期并存。

### 门禁

注入已知轨迹 + 合成量测，断言位置/heading RMS 与协方差一致性（NEES 检验）；
GPS 中断 10 s 的 DR 漂移上界。

估时 1 w。

---

## 7. L4 路由层

| | demo 现状 | 量产判据 |
|---|---|---|
| 全部 | `ScenarioRouteStep route[8]` + `ego_x >= trigger_x` 脚本触发 | 车道级拓扑图 + 图搜索 |

这不是路由，是脚本。

### 改造

1. **车道级图**：节点 = 车道段，边 = 后继 / 左邻 / 右邻，代价含长度 + 变道罚 + 限速 + 转向罚
2. Dijkstra/A\* 出车道序列，输出**到强制变道点剩余距离** —— 规划的变道紧迫度靠它，
   不再靠 `trigger_x`
3. **地图数据与场景解耦、版本化**。现在地图信息埋在场景 JSON 里，量产必须分离
   （地图有独立更新周期）
4. 路口连接关系、可通行性、限行时段

约 400 行，但它拆掉规划里最大的 hack，也是 NOA 出口类行为的硬依赖。

### 门禁

图连通性自检、路线可行性（每对相邻车道确有拓扑边）、无路可达时的确定失败返回。

估时 1 w。

---

## 8. L5 规划层（P0 #2，最大一块）

| 维度 | demo 现状 | 量产判据 |
|------|-----------|----------|
| 输出 | 50 个 `(s,d,speed)`，无时间无 heading 无 κ；cJSON + `strstr` 文本尾 | 定时轨迹，带 `t/x/y/s/l/θ/κ/v/a/jerk`，二进制 |
| 平滑 | 纯采样格（`dt=0.25`, `t∈[2,6]`, 50 候选） | 采样格出初值 + piecewise-jerk QP |
| 障碍 | 静态圆盘、无时间维 | ST/SL 图，消费预测轨迹 |
| 参考线 | 无独立参考线，κ 不可靠 | 平滑 QP 出 `(s,x,y,θ,κ,dκ)`，唯一事实源 |
| 决策 | 无机动决策层（`mode_sm` 是产品态，非机动决策） | 每障碍 `{IGNORE/FOLLOW/OVERTAKE/YIELD/NUDGE_L/NUDGE_R/STOP}` + 原因码 |
| 拼接 | 每帧独立规划，上帧丢弃 | 定时拼接 + 一致性裁决（§9） |
| 可行性 | 有 `failsafe` type 但无显式裁决 | 发布前硬检查，不过则拒发 + 降级轨迹 |
| 求解 | — | 固定 QP 规模、迭代上限、失败确定返回、零动态分配 |

采样格不用扔 —— 它是 QP 很好的初值生成器。

### 8.0 可复用件：带状 QP 求解器（约 300 行）

参考线平滑、path QP、speed QP、MPC 四处都是同一个 piecewise-jerk 结构
（三对角/带状 Hessian + box 约束）。写一次 banded LDLᵀ + active-set（或小 ADMM），
不引外部依赖。**对实时中间件，时序确定性优先于通用性。**

> `ALGORITHM_STACK.md` 的「性能预算（目标）」表列有「OSQP MPC < 5ms（20 步时域）」。
> 该文档已自我标注为参考设计（示例文件未实现），故这是目标值而非实现描述。
> 本方案不引入 OSQP；落地后应在该表注明目标值与自研带状求解器实测值的对照。

### 8.1 参考线平滑

routing 车道序列 → 原始中心线 → 离散点平滑 QP：

```
min Σ‖p_{i-1} − 2p_i + p_{i+1}‖²   s.t. ‖p_i − p_i^raw‖_∞ ≤ ε
```

按 0.5 m 重采样，输出 `(s,x,y,θ,κ,dκ)`。

放最前面是因为 Frenet↔Cartesian 精度和 κ 质量决定下游一切。现在没有可靠 κ，
所以曲率限速根本无从实现。

### 8.2 决策层（当前完全缺失）

保留 `mode_sm` 作为产品态（ACC/CP/NP/LP/NOA 有意义）。**新增机动决策**：结合预测轨迹，
对每个障碍物判 `{IGNORE, FOLLOW, OVERTAKE, YIELD, NUDGE_L/R, STOP}`，
输出决策集作为 ST/SL 的边界生成器。

这是把「障碍物是静态圆盘」变成「障碍物在时空中占一块」的接口层 ——
预测在规划侧的落点就在这里。

### 8.3 Path QP（SL）

采样格出最优 `l_ref(s)` 作参考，然后 piecewise-jerk QP：

- 变量：各点 `l, l', l''`
- 目标：`w_l Σl² + w_dl Σl'² + w_ddl Σl''² + w_dddl Σl'''² + w_ref Σ(l−l_ref)² + 终端项`
- 约束：`l ∈ [l_min(s), l_max(s)]`（静态障碍 SL 边界 ∩ 路沿）、`|l''| ≤ κ_max`、
  分段 jerk 连续性等式、**初值 = 拼接点的 `(l, l', l'')`**

piecewise-jerk 是对的形式：Hessian 稀疏带状、连续性解析、N=100 亚毫秒。

### 8.4 Speed QP（ST）

1. **建 ST 图**：把每个 FOLLOW/YIELD/OVERTAKE 障碍的**预测**轨迹投到已规划 path 上
   → 每模态一块 `(s,t)` 多边形，按概率加权。**预测的价值在这里变现**
2. **粗 DP**（dt=0.5 s, ds=1 m）出 `s(t)` 走廊上下界
3. **Speed QP**：`min w_a Σa² + w_j Σj² + w_ref Σ(v−v_ref)²`，
   约束 s 单调、`v ∈ [0, v_limit(s)]`、`a`/`jerk` 界、落在 ST 走廊内、
   **初值 = 拼接点 `(s, v, a)`**

`v_limit(s)` 必须含**曲率限速** `v ≤ √(a_lat_max / |κ(s)|)`。
「速度跳变、控制跟不上」在弯道段主要就是缺这一项，而它依赖 §8.1 的真 κ。

### 8.5 合成 + 可行性判定

`l(s) × s(t)` → 定时轨迹点，heading/κ 由 path 导数算，v/a/jerk 由速度 profile 算。

发布前**硬可行性检查**：κ ≤ 转向限幅对应值、`a_lat` 界、`|a|`/`|jerk|` 界、
逐时刻 ego footprint 对预测集无碰撞。

任一项不过 → **不发布**，退回上一条轨迹剩余段 + 紧急减速 profile，置 `valid=0`。

### 8.6 参数物理化

Frenet 的 `kd/kv/ka/kj/kt/ko/klat/klon` 现在是硬编码在 `frenet_create` 里的无量纲权重。
量产要么改成有物理含义的量（舒适度上界、最小间距、期望 TTC），要么建立标定流程 +
artifact 记录（见 `CALIBRATION_GUIDE.md`）。

### 门禁

轨迹跨帧 C² 连续性、约束满足率、**重规划比例 > 5% 判 FAIL**（它高 = 车没在跟计划走，
是规划/控制不匹配的直接度量）、QP 迭代次数 p99、单帧规划耗时 p99。

估时 3 w（求解器 4 d、参考线 3 d、决策 3 d、Path QP 4 d、ST+Speed QP 5 d、合成与可行性 3 d）。

---

## 9. 轨迹拼接（P1 #3）

机制反直觉，值得写清楚：**不要用当前测量的 ego 状态做规划初值。**

每周期在 `t_now`：

1. `t_start = t_now + t_plan_latency`（一个规划周期 + 通信，约 100 ms，**实测得来，不要猜**）
2. 在上一条轨迹上按时间插值取 `t_start` → 拼接点 `(x, y, θ, κ, v, a)`
3. **一致性检查**：`|测量位置 − prev_traj(t_now)|` 横向 > ~0.5 m、或速度差 > ~2 m/s、
   或上条无效、或 `ref_line_id` 变了 → 丢弃，从测量状态重规划（`is_stitched=0`）并计数；
   否则拼接点作为 §8.3/§8.4 的 QP 初值
4. 发布的轨迹**首点在 `t_now`**，并把上一条 `[t_now, t_start]` 段前置进去

三个收益：

- 控制器手上永远有「当前时刻」的点，外加一个周期余量 —— 不再从过期终点外推
- 相邻两次规划 C² 连续是**构造性**保证（初值来自上一条计划，不是带噪测量）——
  重规划跳变导致的速度台阶和方向盘抖动直接消失
- 「是否重规划」的比例成为一等健康指标，进门禁（> 5% FAIL）

**依赖顺序**：拼接必须在 MPC 之前落地。反了 MPC 会在一条不连续的参考上调参
—— 就是 2026-07 那六个 commit 反复「修复」横向极限环而不收敛的循环。

估时 3 d。

---

## 10. L6 控制层

| 维度 | demo 现状 | 量产判据 |
|------|-----------|----------|
| 实际在跑 | PID 纵向 + Stanley 式横向 PD + 梯形 v_y 前馈 | 沿轨迹线性化的 LTV MPC |
| MPC/LQR | `pipeline.json` 里 `mpc_horizon: 0`（默认禁用）、`src/algorithms/ltv_mpc.c` 已实现 | 删旧写新 |
| 车辆模型 | 无（仿真 `step_bicycle` 运动学 + 执行器一阶滞后） | 误差动力学 + 执行器滞后作为状态 |
| 延迟 | planning 50 ms + control 50 ms + 仿真 ≈150 ms，无补偿 | 实测延迟 + 模型内补偿 |
| 耦合 | 横纵独立 | LTV 吃规划 v + 摩擦椭圆裁 a_x |
| 降级 | Stanley 是默认而非降级 | 显式降级模式，由健康监控选中 |
| 失败 | 不收敛/求逆失败/线搜索耗尽全部 `return 0` | 出声上报 |

### 10.1 先删

`mpc_horizon: 0` → `mpc_initialized = (mpc && horizon > 0)` = 0 → 整个 MPC 分支不执行；
`lc_use_lqr: 0.0` → 约 50 行 Riccati 迭代不执行。

删 `src/core/mpc_controller.{c,h}` 与 control_node 的 LQR 分支
（引用方只有 `control_node.cpp` + `CMakeLists.txt`，删除是干净的）。三条理由：

1. **从未运行过的代码不可信** —— commit `5db4fc4`（修 MPC bang-bang）、
   `02b598c`（状态向量扩 5 维）、`1ced414`（变道参考注入）、`65c430d`（学习增量 EMA）
   改的都是死代码
2. `mpc_initialized` 只在 init 赋值一次，`flowctl param set control.mpc_horizon 10`
   **打不开它**（热重载改的是 `mpc_config.horizon`，翻不动开关）
3. 求解器内部 `max_steer=0.35` 与外部限幅 ≈0.027 差 12.9 倍，已产生过一整类 bug
   （解从不触及自身约束边界 → 平滑项全部失效）

项目规范亦禁止留第二份死实现。

### 10.2 再写一个

- **模型**：绕轨迹线性化的 LTV 误差动力学 `(e_y, e_ψ, ψ̇, δ, δ_act)`，
  每步 `v(t)`、`κ(t)` **取自轨迹** —— 只有在新消息带 κ 和逐点时间之后才可能
- **纵向基本变成前馈**：轨迹给 `(s, v, a)`，指令 = `a_ff` + PI on `(v_plan − v_meas)` +
  阻力补偿。「PID 追 target_speed」这个结构消失
- **求解**：condensed QP over `Δδ`，N=20、dt=0.05 → 20 变量 + box，
  用 §8.0 的求解器，亚毫秒确定性
- **执行器延迟补偿**：先实测延迟（steer 指令 → yaw 响应），把一阶执行器滞后作为
  **一个状态**加进模型（优于「先按 τ 前推状态再求解」，代价是一个状态）
- **横纵耦合**：LTV 已隐含把规划 v 喂进横向模型；再加摩擦椭圆
  `(a_x/a_x_max)² + (a_y/a_y_max)² ≤ 1` 裁 a_x。完整耦合动力学需轮胎模型，即 §3.2
- **最后一道守门**（与算法无关，量产必需）：指令绝对限幅 + 速率限幅 + 看门狗 +
  上游掉线的保持/减速行为
- **降级**：Stanley/PD 保留为**显式降级模式**，MPC 不收敛或轨迹 `valid=0` 时
  由健康监控切入并上报原因码 —— 不是并行默认

### 门禁（离线单测，控制器全速率）

`steer_flip_rate`、`steer_rate_rms`、横向 RMS、阶跃响应超调/整定时间、约束满足、
求解耗时 p99。

估时 1.5 w。

---

## 11. L7 系统层

| 维度 | demo 现状 | 量产判据 |
|------|-----------|----------|
| 安全 | `safety_control_node` TTC override —— 一个反射 | L0/L1/L2/L3 降级阶梯 |
| 健康 | 各节点 `health()` 钩子存在，无聚合 | 原因码 + supervisor 定级 + 各层消费级别 |
| 回放 | 有 bag，未验证确定性 | 注入时钟回放 bit-identical |
| 延迟 | `fusion/latency` 单点 | 全链路预算门禁 |
| 测量回路 | monitor 硬编码 5 Hz → 评估器 2 Hz；横向判据是 WARN | 20 Hz 控制通道 + FAIL 级判据 |

### 11.1 测量回路（整份计划中价值最高的一项，排最前执行）

现状三条机制性缺陷：

1. **欠采样**：控制器 20 Hz → `monitor_node.c` 硬编码 `g.frequency_hz = 5.0`
   写 `/tmp/flow_topology.json` → 评估器 `--interval 0.5` 读 2 Hz。
   要抓的极限环是 1–2 Hz，在 5 Hz 的 2.5 Hz Nyquist 下混叠。调 `--interval` 无效
   （上游只有 5 Hz，且 `collect_samples` 靠 mtime 去重，读快了只拿重复样本）
2. **判据没牙**：`tools/demo_evaluator.py` 的 `steer_rate_rms` / `max_steer_rate` /
   `steer_flip_rate` / yaw wobble 全写进 `warnings`。按项目约定「WARN 可忽略」，
   横向回归**从未阻断过一次合并**
3. 有几次修的是不执行的代码（见 §10.1）

要做三件：

- 控制通道独立高频日志（`control/raw_cmd` → ring buffer → 文件），或把 monitor 提到 20 Hz
- **离线单测**：planning / control 当库链进单测，喂脚本化参考线 + plant 模型跑满速率
  —— 这是唯一能可靠测极限环的手段；45 s 真跑只能抓碰撞/偏离/卡死这类低频事件
- 上述判据从 WARN 提到 **FAIL**，并**先在现有代码上跑出红灯基线**

顺序不能反。先有能拦住的基线，才有资格说「这次和前六次不一样」。

> 附带修文档：`CLAUDE.md` 写的「monitor_node → 10Hz 写 /tmp/flow_topology.json」
> 与代码不符（实际 5 Hz）。

### 11.2 降级阶梯（2026-07-29 已实现）

```
L0 全功能 → L1 降级(禁变道、限速、加大安全余量) → L2 MRM(车道内减速停车) → L3 立即停
```

各节点上报 `{health, reason_code}`，supervisor 定级，planning/control 读级别调整行为。
触发源：定位协方差超限、预测超时、感知漏检率异常、MPC 不收敛、上游 topic 超 deadline。

与 `REAL_VEHICLE_ROADMAP.md` 的「分级降级状态机」「e-stop 双回路」是同一件事的算法侧。

**当前实现状态：**

- `include/degrade_ladder.h` + `src/core/degrade_ladder.c` — 完整实现，248 行
- 9 个原因码：`PLANNING_TO` / `CONTROL_TO` / `FUSION_TO` / `SENSOR_TO` / `LARGE_CTE` / `COLLISION` / `LOCALIZATION` / `MANUAL` / `HEARTBEAT`
- 三层消费：Supervisor (`degrade_supervisor_tick` 自动递进 L1/L2/L3)、各层策略 (`degrade_layer_action`)、原因码上报
- `monitor_node.c` — 每 20 Hz 调用 `degrade_supervisor_tick()`
- `control_node.cpp` / `planning_node.cpp` / `fusion_node.cpp` — 每帧上报 `degrade_supervisor_record_heartbeat()`
- `safety_control_node.cpp` — 数据超时 >1s 时调用 `degrade_set_level(L3, HEARTBEAT)`
- `control_node.cpp` — L2+ 时强制 `target_speed=0` 执行 MRM

### 11.3 确定性回放

全 topic 录包，注入时钟后回放要 bit-identical（`clock_service` 已支持注入）。
**这是 §4–§10 所有离线门禁的地基** —— 没有它，算法回归测试不成立。

### 11.4 延迟预算门禁

各节点 p99 计算耗时 + 端到端 sensor→actuator 延迟，设 FAIL 阈值。
`fusion/latency` 已有单点实现，推广到全链路。

估时 2 w。

---

## 12. 孤儿模块清账

仓库里有六份不在 pipeline.json 的实现。每一份要么进 pipeline，要么同 commit 删除，
要么打 `@deprecated ... remove-by=<日期>`（规范：无 `remove-by` 视为违规）。

| 模块 | 处置 | 依据 |
|------|------|------|
| `src/algorithms/kalman_tracker.c` + `object_tracker_node.c` | **进 pipeline** | §5 预测的硬前置 |
| `modules/adas_nodes/prediction_node.c` | **重写** | 固定先验 0.7/0.2/0.1 无信息量 |
| `src/core/intent_predictor.c` + 头文件 | **删** | 头文件自述 zero callers；重新推导比复活便宜 |
| `src/core/mpc_controller.{c,h}` + LQR 分支 | **删** | 见 §10.1 |
| `modules/adas_nodes/ekf_slam.{c,h}` + `slam_node.cpp` | 打 deprecated，L3 阶段处理 | §6 |
| `modules/adas_nodes/perception_fusion_node.cpp` | 打 deprecated，L1 阶段处理 | §4 |

估时 1 d。

---

## 13. 依赖图与排期

```
消息契约(§2) ─┬─→ L2 预测(§5) ─┐
孤儿清账(§12) ┘                ├─→ L5 规划 QP(§8) ─→ 拼接(§9) ─→ L6 MPC(§10)
L0 被控对象(§3) ────────────────┘                                    ↑
L7 测量回路(§11.1) ───────────────────────────────────────────────────┘

L1 感知(§4) ─→ L3 定位(§6)        （可与上面并行）
L4 路由(§7) ─→ L5 规划(§8)
```

| 项 | 估时 | 阻塞关系 |
|---|---|---|
| §2 消息契约 | 2 d | 阻塞 §5/§8/§9/§10 |
| §3 L0 被控对象 | 2 d | 阻塞 §10 |
| §11.1 测量回路 | 3 d | 阻塞 §10，且决定能否验证 |
| §12 孤儿清账 | 1 d | 阻塞 §5 |
| §4 L1 感知 | 1.5 w | 阻塞 §5、§6 |
| §5 L2 预测 | 1.5 w | 阻塞 §8.2/§8.4 |
| §6 L3 定位 | 1 w | 需 §4 相机链路 |
| §7 L4 路由 | 1 w | 阻塞 §8.1 |
| §8 L5 规划 QP | 3 w | 需 §5、§7 |
| §9 拼接 | 3 d | 需 §2 + §8 |
| §10 L6 控制 | 1.5 w | 需 §3、§11.1、§8、§9 |
| §11 L7 系统 | 2 w | §11.1 优先，其余可并行 |

**两个容易踩的顺序坑：**

- 拼接排在 QP 之前没意义 —— 没有可连续的对象
- MPC 排在拼接之前也没意义 —— 参考线在「当前时刻」没有有效点

---

## 14. 里程碑

| 档 | 目标 | 含哪些 | 估时 |
|---|------|--------|------|
| **M1 诚实化** | 拆真值捷径 + 建可信测量。**指标可能变差，这是正常的** | §2 消息契约、§3 被控对象、§11.1 测量回路、§12 清账、捷径 #1 拆除 | 2 w |
| **M2 算法补全** | 补预测/QP/拼接/MPC，回到并超过 demo 观感 | §5 预测、§8 规划、§9 拼接、§10 控制 | 6.5 w |
| **M3 量产化** | 定时有界 + 降级 + 冗余 + 标定流程 | §4 感知门禁、§6 定位、§7 路由、§11.2–11.4 | 5 w |

**2026-07-29 已完成项**：§10 控制 — LTV MPC 接口定义 + 基础实现（`src/algorithms/ltv_mpc.c`）、
§11.2 降级阶梯 — 9 原因码 + supervisor + 消费层完整实现（`src/core/degrade_ladder.c` 248 行）、
§3 被控对象 — `step_bicycle` 执行器滞后 + 显式 yaw_rate、`curve_road.json` 弯道场景、
`lane_change_traffic.json` MOBIL 变道场景。

M1 的验收标准不是「评估器全绿」，而是「捷径清单已拆 + 门禁能拦住现有代码的横向极限环」。

---

## 附：本文事实来源

- **本次仓库读取（2026-07-29）**：`config/pipeline.json` 节点顺序与 params
  （`mpc_horizon: 0`、`lc_use_lqr: 0.0`、`lc_use_trapezoid: 1.0`）、各节点 topic 与消息字段、
  `msg/adas_msgs.msg`、Frenet 超参与代价权重、`best_d2 = 9.0` 真值匹配、
  `object_tracker_node` / `prediction_node` / `slam_node` / `perception_fusion_node`
  不在 pipeline、`modules/adas_nodes/CMakeLists.txt` 是独立 CMake 工程
  （`cmake --build build` 不重建节点 .so）
- **此前排查记录**：monitor 5 Hz 欠采样、评估器判据为 WARN、flowsim heading 每帧重置、
  横向极限环六次修复未收敛、commit `5db4fc4`/`02b598c`/`1ced414`/`65c430d` 改死代码。
  **具体行号需落地时复核。**
- `ALGORITHM_STACK.md` 的 OSQP 条目属该文档自述的「参考设计/目标值」，非实现描述。
