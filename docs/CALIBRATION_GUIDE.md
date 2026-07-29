# 控制参数标定指南

> 仿真（运动学模型）与真车之间的参数映射关系、标定工作流、常见问题排查。

---

## 一、核心原则

### 1.1 仿真与真车是两条独立路径

```
仿真: control_node → flowsim（运动学模型）→ 模拟传感器反馈
真车: control_node → 执行器（PWM/CAN）→ 真实物理 → 真实传感器
```

`control_node` 是同一套代码，但**仿真模型和真实物理之间永远有 gap**。即使仿真用了完美动力学模型，真实小车的轮胎磨损、地面摩擦、电机响应延迟、IMU 噪声也无法完全复现。**真车永远需要上实车标定。**

### 1.2 运动学模型 vs 动力学模型

| 特性 | 运动学模型（当前） | 动力学模型 |
|------|-------------------|-----------|
| heading 来源 | 自行车模型积分（自由演化） | 轮胎侧偏力积分 |
| 侧偏角 | 假设为 0 | 有（由轮胎模型计算） |
| 转向不足/过度 | 不体现 | 可体现 |
| 参数数量 | 少（wheelbase + PID + Stanley） | 多（+ 轮胎刚度 + 转动惯量 + 松弛长度） |
| 标定难度 | 低 | 高 |
| 适用场景 | 直道、高速公路巡航 | 急弯、极限工况 |

**当前项目使用运动学模型**，heading 由 `heading += steer * v / L * dt` 每帧自由积分，不强制重置为道路切线。这使得车辆能在变道/超车时自然累积横向偏移。高速公路巡航场景下，运动学模型是轨迹追踪验证的合理简化。

### 1.3 物理模型切换

通过 `pipeline.json` 中 flowsim 节点的 `physics_model` 参数控制：

```json
// 运动学模型（默认，当前有效）
"params": "{..., \"physics_model\": \"kinematic\"}"

// 动力学模型（预留接口，未实现）
"params": "{..., \"physics_model\": \"dynamic\"}"
```

- `"kinematic"`：运动学自行车模型，heading 自由积分。稳定、可预测。
- `"dynamic"`：调用 `step_bicycle_dynamic()` 桩函数，当前降级到 kinematic 并打印一次 LOG_WARN。未来实现时，heading 由轮胎侧偏力积分自主演化。

**实现动力学模型前需要准备：**
1. 在 `entity.h` 的预留字段中填入 `tire_stiffness_f/r`、`yaw_inertia` 等参数
2. 在 `physics.cpp` 的 `step_bicycle_dynamic()` 中实现轮胎模型（Pacejka 魔术公式 / 简化刷子模型）
3. 重新标定所有横向控制参数（`lat_kp`、`lat_kd_heading`、`yaw_damping`）
4. 在 `control_node.cpp` 中增加动力学模式专用的控制参数分支

---

## 二、参数分类

### 2.1 车辆几何参数 — 改配置即生效

| 参数 | 仿真默认值 | RC 小车值 | 含义 |
|------|-----------|----------|------|
| `wheelbase` | 2.7 m | 0.3 m | 轴距。真车 ~2.5-3.0m，RC 小车 ~0.25-0.4m |
| `target_speed` | 12.0 m/s | 2.0 m/s | 巡航目标速度 |
| `steer_tau` | 0.15 s | 0.10 s | EPS 转向一阶滞后时间常数，仿真 `step_bicycle` 中使用 |
| `steer_rate_max` | 0.6 rad/s | 1.2 rad/s | 最大转向速率，仿真 `step_bicycle` 中使用 |

> **steer_tau / steer_rate_max** 是 2026-07-29 新增的仿真物理参数。`steer_tau` 控制转向响应的滞后程度（越大越滞后），`steer_rate_max` 限制每帧转角变化量。RC 小车舵机响应快，`steer_tau` 可设小，`steer_rate_max` 设大。

> **轴距是最关键的几何参数。** 它直接影响 `steer_limit_for_speed()` 的计算：
> `limit = atan(lat_accel_max * wheelbase / speed²)`。轴距缩小 9 倍，同样 steer 下转弯半径缩 9 倍，必须同步调整。

### 2.2 纵向 PID — 仿真调好，真车微调

| 参数 | 仿真默认值 | RC 小车值 | 含义 |
|------|-----------|----------|------|
| `pid_kp` | 800 | 300 | 比例增益 |
| `pid_ki` | 50 | 20 | 积分增益 |
| `pid_kd` | 100 | 40 | 微分增益 |

- 仿真调好的 PID 在真车上通常需要降低（真实执行器有延迟，高增益容易振荡）。
- 标定顺序：先 Kp（让速度跟上），再 Kd（抑制超调），最后 Ki（消除稳态误差）。

### 2.3 横向控制 — 仿真和真车差异最大

| 参数 | 仿真默认值 | RC 小车值 | 含义 |
|------|-----------|----------|------|
| `lat_kp` | 0.5 | 0.8 | 横向误差 → 期望航向增益 (rad/m) |
| `lat_kd_heading` | 1.35 | 1.35 | 航向误差 → steer 阻尼 |
| `yaw_damping` | 0.15 | 0.15 | 偏航角速度 → steer 阻尼 |
| `steer_min_clamp` | 0.016 | 0.02 | 高速最小转向钳位 (rad) |

- `lat_kp`：越高响应越快，但过高会振荡。RC 小车轴距短、响应快，可以设高一点。
- `lat_kd_heading`：提供阻尼，抑制左右摇摆。过高会导致转向"肉"。
- `yaw_damping`：抑制偏航振荡。如果小车原地转圈或左右摆，先加大这个值。
- `steer_min_clamp`：防止高速下 steer 被限幅到 0 导致无转向。RC 小车舵机死区大，需要设得比仿真高。

### 2.4 变道/超车参数 — 仅仿真使用

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `lc_stable_wait_s` | 4.0 | 变道后稳定巡航多久再评估变道 |
| `lc_cooldown_after_stable_s` | 1.5 | 稳定期结束后冷却时间 |
| `lc_cooldown_after_return_s` | 2.0 | 回原车道后冷却时间 |
| `min_overtake_gap_base` | 14.0 | 触发超车所需前车间距基准 (m) |
| `min_overtake_gap_cap` | 90.0 | 间距上限 (m) |
| `min_overtake_gap_speed_mult` | 0.7 | 间距的速度乘数 |
| `lane_change_blocked_timeout_s` | 0.6 | 被阻挡多久触发变道 |

> 真实小车 L2 模式（waypoint_follower）不做变道，这些参数不影响真车。

### 2.5 弯道前馈 — 仅仿真使用

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `curve_ff_boost_radius_m` | 60.0 | 曲率半径小于此值触发前馈提升 |
| `curve_ff_boost_factor` | 1.5 | 前馈权重乘数 |

### 2.6 NPC 行为参数（场景 JSON `npc_lane_change` 开关）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `enable_mobil` | false | NpcAiConfig 结构体中的运行时开关（默认关闭，各守其道） |
| `mobil_politeness` | 0.5 | MOBIL 礼貌因子 [0,1]，0=纯利己，1=考虑他人 |
| `mobil_safe_brake` | 4.5 | MOBIL 安全减速度阈值 (m/s²)，新跟随者不得低于此值 |
| `mobil_gain_threshold` | 0.5 | MOBIL 增益阈值 (m/s²)，gain>此值才变道 |
| `mobil_lane_change_cooldown` | 8.0 | 变道冷却时间 (s) |

> NPC 自主变道通过场景 JSON 顶层 `"npc_lane_change": true` 启用。
> `straight_road.json` 保持关闭，`lane_change_traffic.json` 开启。
> 开启后 NPC 用 MOBIL 代价函数评估变道收益，频繁变道会阻塞 ego，需配合车流密度调参。

> 真车 L2 不依赖 road/geometry 发布弯道信息，不受这些参数影响。

---

## 三、标定工作流

### 3.1 仿真标定（先做）

```
1. 设好 wheelbase（真车轴距）
2. 调纵向 PID：观察速度曲线，目标：无超调、稳态误差 < 0.5 m/s
3. 调横向参数（直道巡航）：
   a. lat_kd_heading=1.0, yaw_damping=0.0, lat_kp=0.3 起步
   b. 逐步加大 lat_kp 直到车能稳定走直线（不蛇形）
   c. 加入 yaw_damping=0.1，观察是否抑制了高频摆动
   d. 微调 lat_kd_heading，平衡"响应快"和"不振荡"
4. 跑 demo_evaluator.py 验证
```

### 3.2 真车标定（仿真基础上微调）

```
1. 保持 wheelbase 不变（几何参数，不是调的）
2. 纵向 PID：仿真值 × 0.5 起步，观察实际速度跟踪
3. 横向参数：
   a. steer_min_clamp 先设 0.03（舵机死区大）
   b. lat_kp 从仿真值 × 0.7 起步
   c. 观察小车直线行驶是否蛇形，有则降 lat_kp 或加 yaw_damping
   d. 观察转弯是否"转不动"，是则降 steer_min_clamp
4. 每次只改一个参数，记录效果
```

### 3.3 标定记录模板

```
日期: 2026-07-26
平台: 仿真 / RC小车 / 真车
改动: lat_kp 0.5 → 0.6
现象: 直道出现轻微蛇形，振幅 ~0.1m，频率 ~1Hz
结论: 回退到 0.5，yaw_damping 从 0.15 提到 0.20
```

---

## 四、常见问题

### 4.1 车左右摇晃（蛇形/振荡）

**根本原因**：横向控制回路增益过高，形成闭环振荡。

排查顺序：
1. 先降 `lat_kp`（减半），观察是否消失
2. 加 `yaw_damping`（0.15 → 0.25）
3. 检查 `steer_min_clamp` 是否过小（太小导致 steer 频繁被 clamp 到 0 又弹回，形成极限环）
4. 真车额外检查：舵机是否在抖动（中位不稳），机械间隙是否过大

### 4.2 转弯转不动

**根本原因**：`steer_min_clamp` 过大，低速时 steer 被限幅。

- 仿真：`steer_min_clamp` 通常 0.016-0.02
- RC 小车：舵机有死区，可能需要 0.02-0.03
- 如果 `steer_min_clamp` 已经很小还是转不动，检查 `lat_kp` 是否过低

### 4.3 真车速度跟踪不准

**根本原因**：仿真里驱动力的物理参数（质量、阻力系数）与真实小车不同。

- 仿真：`drive_force = throttle * 5000 N`，`mass = 1500 kg`
- 真实小车：质量可能 2-5 kg，电机推力曲线完全不同

解决：调纵向 PID，不要试图匹配物理参数。PID 是反馈控制，能自动补偿模型误差。

### 4.4 仿真和真车同一套参数，表现差异很大

**这是正常现象。** 因为：
- 仿真模型是运动学的（heading 自由积分），不是带轮胎侧偏的完整物理
- 真实小车有舵机延迟、轮胎侧滑、地面不平
- 仿真没有传感器噪声，真车 GPS/IMU 有噪声

**不需要强行让仿真参数和真车参数一致。** 把仿真和真车当作两套独立标定即可。

---

## 五、参考

- [pipeline.json](../config/pipeline.json) — 仿真默认配置
- [pipeline_car.json](../config/pipeline_car.json) — 真车 RC 小车配置
- [HARDWARE_DEPLOYMENT.md](HARDWARE_DEPLOYMENT.md) — 真车硬件部署
- [REAL_VEHICLE_ROADMAP.md](REAL_VEHICLE_ROADMAP.md) — 真车部署待办清单
- [control_node.cpp](../modules/adas_nodes/control_node.cpp) — 控制算法实现（含所有参数含义和默认值）