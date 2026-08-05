# FlowEngine Code Wiki

> 面向自动驾驶与机器人的仿真优先中间件框架 —— C11 内核、C++20 协程外壳、dlopen 插件化 ADAS Pipeline。
>
> 本文档从源码角度系统梳理项目架构、模块职责、关键类与函数、依赖关系与运行方式，**重点覆盖控制子系统**。

---

## 目录

- [1. 项目概览](#1-项目概览)
- [2. 整体架构](#2-整体架构)
- [3. 核心中间件层](#3-核心中间件层)
- [4. ADAS Pipeline](#4-adas-pipeline)
- [5. 控制子系统详解（重点）](#5-控制子系统详解重点)
- [6. 关键类与函数索引](#6-关键类与函数索引)
- [7. 依赖关系](#7-依赖关系)
- [8. 项目运行方式](#8-项目运行方式)
- [9. 控制调参指南（k_vy 专题）](#9-控制调参指南k_vy-专题)

---

## 1. 项目概览

FlowEngine 是一个从零搭建的自动驾驶中间件，灵感来自 Apollo CyberRT。定位是**仿真优先、可复现的实验平台**：感知、融合、规划、控制、学习首先在仿真内运行、观察、测试、回放与评分，再逐步向真车部署演进。

| 维度 | 说明 |
|------|------|
| 内核语言 | C11（轻量、可嵌入） |
| 外壳语言 | C++20（协程、类型安全） |
| 插件机制 | `dlopen` 加载 `.so`，节点间仅通过 Message Bus 通信 |
| 节点数 | 默认 pipeline 15 个插件节点 |
| 默认场景 | `scenarios/straight_road.json`（3000m 双向 4 车道直路，路尾掉头） |
| 许可证 | MIT |

核心能力一览：Pub/Sub 消息总线（零拷贝）、IPC 共享内存、TCP/网络传输、C++20 协程调度器、反射式状态机、UDP 服务发现、Bag v2/MCAP 录制回放、EKF 传感器融合、行为 FSM、Frenet 最优轨迹规划、ST 图 + DP 速度规划、N 把方向掉头/泊车（ManeuverTracker）、Stanley 横向（可选 LTV MPC）+ PID 纵向、FlowCoro 安全包络、FlowBoard 3D 仪表盘、车端学习闭环（数据采集→训练/DAgger/PPO→影子推理→SGD 微调→OTA）。

---

## 2. 整体架构

### 2.1 三层架构

```
┌──────────────────────────────────────────────────────────────────────┐
│                        FlowEngine Core (C11)                          │
│  Message Bus │ IPC(SHM) │ Bag(v2/MCAP) │ Clock │ State Machine       │
│  FlowRegistry│ ParamRegistry │ Discovery │ Serializer(IDL) │ TaskMgr  │
├──────────────────────────────────────────────────────────────────────┤
│                     FlowEngine Shell (C++20)                          │
│  Coroutine Tasks │ Scheduler(Choreo) │ Fusion(EKF) │ Transport(TCP)  │
├──────────────────────────────────────────────────────────────────────┤
│                     ADAS Pipeline (dlopen plugins)                     │
│  flowsim → sensor_model → perception → fusion → planning → control   │
│    → safety_control → inference → data_recorder → learner → model_ota│
│    → monitor                                                         │
└──────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
                flowmond 监控守护进程 :8800
                (IPC 桥接 + 文件桥接 + HTTP/SSE)
```

### 2.2 控制链路数据流

```
flowsim ──vehicle/state──┐
   │ fusion/localization ──────────┤
   │ perception/obstacles ─────────┤
   │ road/geometry, traffic_lights─┤
   │ scene/frame ──────────────────┘
   ▼
[planning_node OR waypoint_follower] ──planning/trajectory──┐
   ▼                                                        │
control_node ──control/raw_cmd──► [safety_control_node] ──control/cmd──► [actuator] ──► CAN/PWM
                                  (限幅+TTC+行人+横向交叉)        (3s 看门狗)
   │                                                        │
   └── control/raw_cmd/text ──► [guardian_node] ──control/emergency_stop──► (旁路紧急制动)

mpc_controller  ← mpc_set_*  ← control_node (每帧)  【已废弃，代码已删除】
frenet_bridge   ← frenet_plan ← planning_node (20Hz)
flowsim/physics ← step_bicycle ← flowsim_node (20Hz tick, 闭环 ego)
```

**关键设计原则**：节点之间**没有直接函数调用**，仅通过 Message Bus 的 topic 解耦。每个节点是独立 `.so`，由 `flow_launcher` 在运行时 `dlopen` 加载。

---

## 3. 核心中间件层

| 模块 | 关键文件 | 职责 |
|------|----------|------|
| 消息总线 | [message_bus.c](file:///workspace/src/core/message_bus.c) / [message_bus.h](file:///workspace/include/message_bus.h) | 进程内 Pub/Sub + 请求/应答 + 零拷贝 |
| 传输层 | [transport.c](file:///workspace/src/core/transport.c) / [transport.h](file:///workspace/include/transport.h) | 统一传输抽象（local/IPC/TCP），per-topic QoS（深度+丢弃策略+deadline+reliability） |
| IPC | [ipc_channel.c](file:///workspace/src/core/ipc_channel.c) | POSIX SHM 跨进程通信 |
| 调度器 | [scheduler.c](file:///workspace/src/core/scheduler.c) / [scheduler_cpp.cpp](file:///workspace/src/cpp/scheduler_cpp.cpp) | classic/choreo DAG 模式，CPU 亲和+限频 |
| 协程 | [coroutine_task.h](file:///workspace/include/coroutine_task.h) | C++20 协程原语（sleep/await/select/timer/req-reply，可取消） |
| 状态机 | [state_machine.c](file:///workspace/src/core/state_machine.c) | 反射式状态机（INITIALIZED→RUNNING→STOPPING→STOPPED） |
| 元信息 | [flow_registry.c](file:///workspace/src/core/flow_registry.c) | 统一注册中心（Task/Topic/Type/Plugin/Schema） |
| 参数 | [param_registry.c](file:///workspace/src/core/param_registry.c) / [param_bridge.c](file:///workspace/src/core/param_bridge.c) | int/float/bool/string 参数，范围校验，**hot-reload**（`flowctl param set` 边跑边调，AF_UNIX 跨进程通道） |
| 时钟 | [clock_service.c](file:///workspace/src/core/clock_service.c) | `clock_now_us()`（MONOTONIC，仿真/回放可注入）/ `clock_now_realtime_us()`（REALTIME） |
| 序列化 | [serializer.c](file:///workspace/src/core/serializer.c) / [msg_schema.c](file:///workspace/src/core/msg_schema.c) | IDL + 代码生成 + FNV 类型哈希；全链路 `timestamp_us` 统一 `uint64`（GNSS 采集时刻优先） |
| 录制 | [bag.c](file:///workspace/src/core/bag.c) / [mcap_writer.c](file:///workspace/src/core/mcap_writer.c) | Bag v2 + MCAP 格式录制/回放 |
| 服务发现 | [discovery.c](file:///workspace/src/core/discovery.c) | UDP 服务发现 + 拓扑追踪 |
| 监控 | [monitor_server.c](file:///workspace/src/core/monitor_server.c) / [stats_bridge.c](file:///workspace/src/core/stats_bridge.c) / [dashboard_bridge.c](file:///workspace/src/core/dashboard_bridge.c) | 内嵌 HTTP 服务器、跨进程统计/仪表盘 IPC 桥接 |
| 插件宿主 | [node_plugin.h](file:///workspace/include/node_plugin.h) / [node_context.c](file:///workspace/src/core/node_context.c) | NodePlugin 接口，节点 `.so` 的统一 ABI |

### 参数系统（控制调参的核心基础设施）

参数热重载是控制调优的关键。**新增一个可调参数必须三处都通**（见 [CLAUDE.md](file:///workspace/CLAUDE.md) 调参章节）：

1. `params_json` 里加 `cJSON_GetObjectItemCaseSensitive` 解析分支
2. `param_register_*` 的默认值用 `g.<字段>` 而非硬编码字面量
3. 逐帧 tick 里 `param_get_float` 重读

```bash
flowctl param list                     # 运行中进程的实时参数
flowctl param get control.k_vy         # 读取
flowctl param set control.k_vy 0.5     # 下一帧生效，无需重启
```

---

## 4. ADAS Pipeline

### 4.1 15 节点概览

默认配置 [pipeline.json](file:///workspace/config/pipeline.json) 启动 15 个插件节点：

| 节点 | 插件 (.so) | 频率 | 功能 |
|------|-----------|------|------|
| `flowsim` | libflowsim_node.so | 50Hz | 车辆动力学 + NPC（IDM）+ 场景加载 + 真值发布 |
| `sensor_model` | libsensor_model.so | 20Hz | LiDAR/GPS/Camera 传感器模型（FOV/遮挡/噪声） |
| `perception` | libperception_node.so | 10Hz | DBSCAN 点云聚类 + 目标检测 |
| `object_tracker` | libobject_tracker.so | 20Hz | 卡尔曼多目标跟踪 |
| `fusion` | libfusion_node.so | 20Hz | EKF 传感器融合（定位 + 时间对齐） |
| `behavior_planner` | libbehavior_planner.so | 10Hz | 8 状态 FSM 行为决策（跟车/变道/停车/让行/掉头） |
| `navigation` | libnavigation_node.so | 10Hz | 路由步骤 + 行进方向（消费 `ref_path.reverse`） |
| `planning` | libplanning_node.so | 20Hz | Frenet 轨迹 + ST 图 DP 速度规划 + N 把方向掉头 |
| `control` | libcontrol_node.so | 50Hz | Stanley 横向（可选 LTV MPC）+ PID/ACC 纵向 + ManeuverTracker |
| `safety_control` | libsafety_control_node.so | 协程 | FlowCoro 安全包络（TTC/横向交叉/行人/MRM） |
| `inference` | libinference_node.so | 20Hz | tiny-MLP/ONNX 影子推理（shadow mode） |
| `data_recorder` | libdata_recorder_node.so | 20Hz | 训练样本采集（模仿学习 JSONL） |
| `learner` | liblearner_node.so | 0.5Hz | 车端增量 SGD 微调 |
| `model_ota` | libmodel_ota_node.so | 1Hz | 模型 OTA + 版本管理 + A-B 对比 |
| `monitor` | libmonitor_node.so | 10Hz | 系统监控 + 仪表盘 JSON 导出 |

### 4.2 节点插件接口

所有节点实现 `NodePlugin` 接口（[node_plugin.h](file:///workspace/include/node_plugin.h)），编译为 `.so`，由 `flow_launcher`（[flow_launcher.c](file:///workspace/src/flow_launcher.c)）读取 `pipeline.json` 后 `dlopen` 加载。C 节点用 `TaskBase` + `TaskInterface` vtable，C++ 节点用 `CoroutineTask` 协程。

```c
// C 插件 ABI
TaskBase* create_task(const TaskConfig* cfg);
static TaskInterface vtable = { .execute = my_execute };
```

```cpp
// C++ 协程插件
class MyTask : public CoroutineTask {
    Task run() override {
        while (!should_stop()) {
            co_await sleep_us(50000);  // 20Hz，可被 stop 立即取消
            /* 业务逻辑 */
        }
    }
};
```

> **节点线程规范**（见 [CLAUDE.md](file:///workspace/CLAUDE.md)）：禁止裸 `while(!stop) ex.run();`（忙等自旋占满核），必须用 `node_pump(ex, []{return g.should_stop;})`。

---

## 5. 控制子系统详解（重点）

控制链路是 `planning → control → safety_control → actuator`，外加 `guardian` 旁路熔断。本节逐模块剖析。

### 5.1 control_node — 控制核心

**文件**：[control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp)（~2100 行）
**插件**：`libcontrol_node.so`，20Hz 协程节点
**订阅**：`fusion/localization`、`planning/trajectory`、`vehicle/state`、`road/geometry`、`scene/frame`
**发布**：`control/raw_cmd`（ControlRaw，QoS: drop_oldest / best_effort / 100ms deadline）

#### 5.1.1 核心结构 `ControlContext`（[L112](file:///workspace/modules/adas_nodes/control_node.cpp#L112)）

节点全局状态，关键字段分组：

| 字段组 | 字段 | 说明 |
|--------|------|------|
| 纵向 PID | `kp/ki/kd`、`integral`、`prev_error` | 目标速度→throttle/brake |
| 横向级联 PD | `lat_kp`、`lat_kd_heading`、`yaw_damping`、`ego_heading`、`ego_yaw_rate`、`prev_steer` | Stanley 横向控制 |
| 真车级横向 | `lat_lookahead_gain`、`k_v_lat` | Apollo LQR 风格前视+横向速度阻尼 |
| **横向速度规划** | **`k_vy`、`k_vy_damp`** | **v_y_des = k_vy*lat_error - k_vy_damp*v_lat**（详见 5.1.4） |
| 障碍物 | `obs_x/y/vx[128]`、`obs_valid[128]`、`ped_index` | 128 槽位（0-3 来自 vehicle/state，4-7 来自 scene/frame） |
| 变道状态机 | `lc_state`、`lc_target_y`、`lc_target_idx`、`committed_lane_side`、`lc_timer/wait/cooldown` | 0=巡航 1=左变道 2=稳定 3=回切 |

#### 5.1.2 主循环 `ControlTask::run()`（[L837](file:///workspace/modules/adas_nodes/control_node.cpp#L837)）

```cpp
class ControlTask : public CoroutineTask {
    Task run() override {
        pthread_setname_np(pthread_self(), "control");
        while (!should_stop()) {
            co_await sleep_us(50000);  // 20Hz，stop 可立即唤醒
            g.cycle++;
            /* 1. 热重载所有参数（param_get_float）—— 支持运行时调参 */
            g.k_vy = param_get_float("control.k_vy");
            g.k_vy_damp = param_get_float("control.k_vy_damp");
            /* 2. 数据陈旧检测（>1s 清 flag），陈旧时发安全减速 fallback */
            /* 3. ACC 跟车：算 acc_target（gap-based 速度匹配） */
            /* 4. 纵向 PID：output = kp*err + ki*integral + kd*deriv */
            /* 5. 横向控制：Stanley（原 MPC 已删除） */
            /* 6. Safety overrides：ROAD_GUARD / 超速限幅 / 死锁恢复 */
            /* 7. 序列化发布 control/raw_cmd */
        }
    }
};
```

> **为何用 CoroutineTask 而非 FlowCoroTask**（文件头注释）：control 是延迟敏感闭环控制，周期精度直接影响横向稳定性。FlowCoroTask 线程池 resume 会引入调度抖动，导致 20Hz 周期不一致、prev_steer 低通滤波间隔波动、steer 小幅振荡。CoroutineTask 同步 resume 周期精确。

#### 5.1.3 纵向控制：PID + ACC（[L1392](file:///workspace/modules/adas_nodes/control_node.cpp#L1392)）

纵向**始终用 PID+ACC**，横向用 Stanley（原 MPC 已删除）。

```
acc_target 计算（L1130-1218）:
  基础 = boost_target（≈cruise_speed）
  跟车  = lead_speed + (boost_target - lead_speed) * ratio   // gap=0 匹配前车，gap≥safe 恢复巡航
  超车  = current_speed + 2.0                                 // 主动加速
  红绿灯 = 0（停车）或 3.0（变道中减速，防横向动量+急刹→overshoot）
  夹紧  = cruise_speed

PID（L1402）:
  error = acc_target - current_speed
  integral += error * 0.05   // anti-windup 限幅 [-200, 500]
  output = kp*error + ki*integral + kd*derivative
  output > 0 → throttle = output / 5000   (clamp [0,1])
  output < 0 → brake    = -output / 8000  (clamp [0,1])
```

#### 5.1.4 横向控制：Stanley（[L1428](file:///workspace/modules/adas_nodes/control_node.cpp#L1428)）

巡航横向默认走 Stanley；`use_ltv_mpc` 参数可启用 **LTV MPC**（线性时变模型预测，
求解失败回退 Stanley；机动模式跳过 MPC——满舵弧 + 倒挡在其线性化域外，由
ManeuverTracker 负责）。原 iLQR MPC 已删除。

**Stanley + 横向速度规划（变道 `lc_active`，[L1513](file:///workspace/modules/adas_nodes/control_node.cpp#L1513)）**

这是用户调参纠结的核心。变道时启用「横向速度规划 PD」：

```
v_y_des = k_vy * lat_error - k_vy_damp * v_lat_actual        // L1570
  P 项 (k_vy * lat_error)    : 拉向目标车道，距离越远推力越大
  D 项 (-k_d * v_lat)        : 阻尼，对抗当前横向速度

ψ_des = road_heading + asin(v_y_des / v)     // 期望航向（clamp ±30°）  L1575
δ_ff  = atan(wheelbase * v_y_des / v²)       // 自行车稳态前馈          L1577

steer = cte_term - heading_term - yaw_damp_term + ff_term + delta_ff   // L1614
  cte_term     = atan2(lat_kp * lat_error, speed)                      // L1590
  heading_term = lat_kd_heading * (ego_heading - ψ_des)                // L1591
  yaw_damp     = yaw_damping * ego_yaw_rate                            // L1592
  ff_term      = wheelbase * kappa * ff_weight                         // L1602

steer = clamp(±steer_limit_for_speed)                                  // L1615-1617
steer = filter_new*steer + (1-filter_new)*prev_steer                   // L1618 一阶低通
```

变道时增益调整（[L1542](file:///workspace/modules/adas_nodes/control_node.cpp#L1542)）：`lat_kd *= 0.8`、`lat_kp *= 2.5`、`lat_accel_max=8.0`、`filter_new=0.7`。

**动态行为解读**（注释 L1555-1559）：
- 起步（v_lat=0）：`v_y_des = k_vy*lat_error`（全力推）
- 中段（v_lat≈v_y_des）：D 项抵消 P 项，v_y_des 减小
- 近目标（lat_error→0, v_lat>0）：`v_y_des = -k_d*v_lat`（纯制动）
- 到目标（都=0）：`v_y_des = 0`（稳定）

#### 5.1.5 Safety Overrides（[L1624](file:///workspace/modules/adas_nodes/control_node.cpp#L1624)）

对 Stanley 和 PID 回退都生效的硬保护：
- **接近路沿增强拉回**（|y|>4.5）：steer_limit 降到 0.165，但拉回力矩≈8.8 m/s² 对抗残余过冲
- **超速限幅**：speed > cruise+1 → throttle=0 + 比例刹车
- **全域速度死锁恢复**（`SPEED_ZERO_RECOVER_S=5.0`）：速度持续为 0 超阈值 → 小油门 thr=0.15
- **ROAD_GUARD**（偏离过远）：steer 强制满限幅回正 + 低速给油/高速刹车
- **转向灯/双闪**（[L1678](file:///workspace/modules/adas_nodes/control_node.cpp#L1678)）：变道意图先行，提前亮灯

#### 5.1.6 变道状态机（[L1295](file:///workspace/modules/adas_nodes/control_node.cpp#L1295)）

```
lc_state: 0=巡航  1=变道中  2=稳定巡航  3=回切中
触发: blocked 超时 / NOA 路线 route_lane
完成: |ego_y - target_y| < LC_COMPLETE_THRESH(0.15m)
回切: 稳定期 lc_stable_wait_s 后评估原车道是否清空（gap + 速度 + 行人 + 后方）
```

车道判定有迟滞（`LANE_HYSTERESIS_M=0.5`），避免骑线时目标车道每帧翻转抖振。

### 5.2 mpc_controller — iLQR MPC 【已废弃，代码已删除】

原 iLQR MPC 已移除。现横向控制 = Stanley（默认）+ LTV MPC（`ltv_mpc.h`，参数
`use_ltv_mpc` 启用）+ ManeuverTracker（机动模式：掉头/倒车/泊车，`maneuver_tracker.h`）。

### 5.3 planning_node — Frenet 轨迹 + ST 图速度规划

**文件**：[planning_node.cpp](file:///workspace/modules/adas_nodes/planning_node.cpp)（~2700 行），20Hz 协程节点
**订阅**：`fusion/localization`、`perception/obstacles`、`road/geometry`、`road/traffic_lights`、`scene/frame`
**发布**：`planning/trajectory`（type_id `0x3A7B1C2D`）

| 关键结构/函数 | 说明 |
|----------------|------|
| `PlanningContext` | 节点状态：双状态机（反射式 + 驾驶模式 NA/ACC/CP/NP/NOA）、Frenet 句柄、ego、障碍物 128 槽 |
| `frenet_plan()` | Frenet 最优轨迹（横向路径） |
| `st_graph.c` `stg_plan()` | **ST 图 + DP 速度规划**：红灯墙 + 动态障碍占据 + 曲率限速，90×101 DP 表（142KB 静态）。1:1 移植自 `tools/speed_planner_sim.py`（11/11 场景 PASS 后冻结），替代旧线性斜坡 + 红灯 override 堆 |
| `generate_uturn_trajectory()` | N 把方向掉头：前进满舵弧 → 刹停换 R → 倒车反打 → 循环（≤5 把）；512 点细生成 + 段感知下采样 64；生成一次即缓存重放防重规划抖动 |
| `project_to_reference_path()` | ego 投影到 map_ref 参考线 Frenet 弧长（右转/支路场景） |

**驾驶模式状态机**：NA→ACC→CP→NP→NOA 逐级升级（guard 检查 fusion/vstate/路线就绪），fusion 超 1.5s 未更新降级回 NA。

**轨迹下发格式**：二进制 `Trajectory` 结构体（`adas_msgs_gen.h` 定义），
含 `seq`, `stamp_us`, `ref_line_id`, `point_count`, `points[64]` 及 `valid` 标志。
每个 `TrajectoryPoint` 含 `t_rel_us`, `x`, `y`, `s`, `l`, `heading`, `kappa`, `v`, `a`, `jerk`。
不再是 JSON `{"type":"frenet", "path":[[s,d,speed],...]}`。
下游用 `Trajectory_deserialize()` 反序列化，见 `planning_node.cpp` 的发布端和 `inference_node.cpp` 的消费端。

> **速度剖面演进**：早期速度剖面由 FOT 代价函数隐式产生 + 红灯 override 补丁堆，
> 无显式减速语义。现由 `st_graph.c`（ST 图 + DP）统一生成：红灯墙、动态障碍占据、
> 曲率限速全部进 DP 代价，替代了旧 override 堆。设计见
> [PLANNING_SPEED_UPGRADE_DESIGN.md](file:///workspace/docs/PLANNING_SPEED_UPGRADE_DESIGN.md)。

> **已修复 bug — 停稳死锁**（[L787](file:///workspace/modules/adas_nodes/planning_node.cpp#L787)）：旧版 `command_speed = spd_out[0]`（≈当前车速）覆盖导致 `v=0→target=0→油门=0→v=0` 自维持闭锁。已改为 `if (spd_out[0] > command_speed) command_speed = spd_out[0]`（取 max）。

### 5.4 safety_control_node — 安全闸门（in-line）

**文件**：[safety_control_node.cpp](file:///workspace/modules/adas_nodes/safety_control_node.cpp)（642 行），FlowCoro 协程
**订阅**：`control/raw_cmd`、`fusion/localization`、`perception/obstacles`
**发布**：`control/cmd`（ControlCmd 二进制，type_id `0x2D95C6D2`，100Hz）

定位：**控制链路内**必经闸门，对 control_node 原始指令做限幅 + 碰撞制动覆写，是 control→actuator 之间的最后一道安全整形。

**三个安全包络**：

| 检查 | 函数 | 逻辑 |
|------|------|------|
| TTC（同向碰撞） | `min_vehicle_ttc()` L259 | ttc<2.2→throttle=0+比例刹车；ttc<1或dx<6.5→brake=1.0 |
| 横向交叉 | `nearest_vehicle_lateral_cross_risk()` L312 | crossing_intent+risk<9+speed>7→brake≥0.65+steer_guard；仅 risk 车在前方时限方向（防变道过半回正被误覆盖） |
| 行人保护 | `pedestrian_collision_gap()` L224 | gap<ped_stop_gap→throttle=0+比例刹车；gap<0.55*stop→brake=1.0；hold_gap<1.5→brake=1.0 |
| 对向 TTC | `min_oncoming_ttc()` L288 | ttc<4→brake |

**限幅参数**（`SafetyParams`）：max_throttle=0.85、max_steer=0.22（高速）/0.18（低速<3m/s）、min_gap=6.0m、time_headway=1.8s。

**NaN/Inf 兜底**：`publish_cmd` 发布前再查 isfinite，任一非有限→强制 emergency_stop。

> **职责边界**：safety_control **不发起死锁恢复**（已移除原 5s low-speed recovery）。理由：与 control 的 SPEED_ZERO_RECOVERY 互相矛盾（control 设 thr=0.15，safety 覆写 thr=0.20/brk=0.30→既进又刹），且 safety 不订阅红绿灯→红灯停车 5s 后会强制蠕行闯红灯。死锁恢复全部归属 control_node。

### 5.5 guardian_node — 熔断器（off-line）

**文件**：[guardian_node.c](file:///workspace/modules/adas_nodes/guardian_node.c)（589 行），20Hz，TASK_PRIORITY_HIGH
**订阅**：`vehicle/state`、`control/raw_cmd/text`、`fusion/localization`、`perception/obstacles`
**发布**：`control/emergency_stop`（旁路正常链路）

定位：**链路外**独立看门狗，不依赖 planning/control 正常运转，5 级粗粒度检查，仅故障时介入。

| Level | 检查 | 阈值 |
|-------|------|------|
| 1 碰撞 | `check_collision()` | ttc < 1.5s |
| 2 车道偏离 | `check_lane_departure()` | \|cte\| > 2.5m 且 speed>10 |
| 3 超速 | `check_overspeed()` | speed > 35 m/s |
| 4 指令合理性 | `check_cmd_sanity()` | \|steer\|>0.5 / thr+brk 同>0.1 / 高速大转向 |
| 5 心跳超时 | `check_timeout()` | ego/loc >500ms / cmd >1500ms |

**safety_control vs guardian 对比**：

| 维度 | safety_control | guardian |
|------|----------------|----------|
| 定位 | 链路内必经闸门 | 链路外独立看门狗 |
| 干预 | 修改/限幅 control 指令 → control/cmd | 发布 control/emergency_stop（旁路） |
| 检查 | 精细 per-obstacle（TTC/行人/横向交叉） | 5 级粗粒度 |
| 优先级 | normal | HIGH |

### 5.6 actuator — 执行器

两个二选一节点，都订阅 safety_control 限幅后的 `control/cmd`，都有 3s 看门狗（超时强制中位）。

**actuator_node**（[actuator_node.c](file:///workspace/modules/adas_nodes/actuator_node.c)，523 行）：SocketCAN 真车执行器。`can_open`/`can_send` 编码 CAN 帧（throttle→uint16 LE ×throttle_scale，steer→ `steering/0.22` 归一化 ×steering_scale）。非 Linux 或接口不存在自动降级 dry_run。10Hz 心跳。

**actuator_pwm_node**（[actuator_pwm_node.c](file:///workspace/modules/adas_nodes/actuator_pwm_node.c)，581 行）：RC 小车 PWM 执行器。三后端：PCA9685 I2C（主）/ GPIO sysfs / dry_run。映射：`esc_us=1500+throttle*scale`，`steer_us=1500+steer_norm*scale`，脉宽硬钳位[1000,2000]μs。

> pipeline 二选一：真车 ESC 用 actuator_node（CAN），RC 小车用 actuator_pwm_node（PWM）。

### 5.7 flowsim/physics — 车辆动力学仿真真值

**文件**：[physics.cpp](file:///workspace/modules/adas_nodes/flowsim/physics.cpp)（104 行）/ [entity.h](file:///workspace/modules/adas_nodes/flowsim/entity.h)（258 行）/ [flowsim_node.cpp](file:///workspace/modules/adas_nodes/flowsim_node.cpp)

**`step_bicycle()`**（[L29](file:///workspace/modules/adas_nodes/flowsim/physics.cpp#L29)）：运动学自行车模型前向欧拉积分。
```
纵向: drive=throttle*5000, brake=brake*8000, drag=drag_coeff*v², accel=net/mass
横向: heading += (speed/wheelbase)*tan(steer)*dt   (steer 限幅 ±0.25)
位置: x += v*cos(h)*dt
速度: vx=v*cos(h), vy=v*sin(h)
```

**`step_bicycle_dynamic()`**（[L67](file:///workspace/modules/adas_nodes/flowsim/physics.cpp#L67)）：**动力学桩（未实现）**。首次调用打 LOG_WARN 后 fallback 到 step_bicycle。entity.h 的 `v_x_body/v_y_body/yaw_rate/F_yf/F_yr/tire_stiffness/yaw_inertia` 字段运行时恒 0。

> **heading 重置逻辑 — 文档/代码漂移**：[CLAUDE.md L415](file:///workspace/CLAUDE.md) 与 flowsim_node.cpp:1185 注释声称"运动学模式每帧重置 heading 为道路切线"，但 [L1240](file:///workspace/modules/adas_nodes/flowsim_node.cpp#L1240) 实现已改为**保留 bicycle 自由积分 heading**（仅归一化），靠 `sin(heading - road_heading)` 驱动横向位移构成负反馈闭环。**以代码为准**。这一改动直接影响 control 的 v_lat_damp 是否生效——旧实现 heading 被道路切线拽回→heading_err≈0→横向 PD 退化为纯 P。

**`internal_cruise_control()`**（[L1029](file:///workspace/modules/adas_nodes/flowsim_node.cpp#L1029)）：无 control/cmd 时的 ego 闭环 fallback。纵向 P 控制追 target_vx，横向 `steer=heading_err*0.3+y_err*0.03`（cap ±0.15）。

**EPS 转向低通**（[L1170](file:///workspace/modules/adas_nodes/flowsim_node.cpp#L1170)）：`steer=0.4*raw+0.6*prev`，模拟电动助力转向惯性，滤掉 Stanley 小幅振荡。

### 5.8 frenet_bridge — FOT 包装

**文件**：[frenet_bridge.cpp](file:///workspace/src/algorithms/frenet_bridge.cpp)（177 行）/ [frenet_bridge.h](file:///workspace/src/algorithms/frenet_bridge.h)

C wrapper 封装开源 Frenet Optimal Trajectory (FOT) 规划器（Apache-2.0）。planning_node 是唯一调用方。

| API | 说明 |
|-----|------|
| `frenet_create(max_speed, max_accel)` | 创建 handle，设默认超参（max_road_width=7.0m, dt=0.25s, maxt=6.0/mint=2.0s, klat=0.8/klon=0.5） |
| `frenet_set_reference_path()` | 设参考路径（≥3 点） |
| `frenet_set_obstacles_v()` | 带速度障碍物（2s 预测时域外推） |
| `frenet_plan()` | 核心：构建初始条件(s0,c_speed,c_d,...) → run_fot() → 输出(s,d,speeds) |

> Cartesian↔Frenet 转换由 FOT 内部 CubicSpline2D 完成。planning_node 侧简化：直道假设下 `ego_x` 当 s，`ego_y - road_center_y` 当 d。
>
> `#ifdef HAVE_FRENET` 守卫：未安装 libeigen3-dev 时编译为 lane_keep_fallback（恒 d=0，**不变道**）。

### 5.9 waypoint_follower — Pure Pursuit（替代规划）

**文件**：[waypoint_follower_node.c](file:///workspace/modules/adas_nodes/waypoint_follower_node.c)（678 行），L2 级 RC 小车航点跟随。

**关键澄清**：waypoint_follower 是 **planning 层替代品**（发布 `planning/trajectory`，与 planning_node 同 topic，pipeline 二选一），**不是 control 层替代品**。control_node 仍是唯一控制层。Pure Pursuit 找前瞻点→车体坐标 path→cruise 减速。避障只减速不绕行（绕行交 control/safety）。

---

## 6. 关键类与函数索引

### 控制子系统

| 类/函数 | 文件 | 作用 |
|---------|------|------|
| `ControlContext` | [control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp) | 控制节点全局状态（PID/横向/障碍物/变道） |
| `ControlTask::run()` | [control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp) | 20Hz 协程主循环 |
| `control_init()` | [control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp) | 节点初始化（参数解析+注册+状态机） |
| `steer_limit_for_speed()` | control_node.cpp | speed-dependent 转向限幅（lat_accel 约束） |
| `lane_center_y()` / `lane_idx_from_y()` | control_node.cpp | N 车道模型：y↔车道索引互转 |
| `PlanningContext` / `PlanningTask` | [planning_node.cpp](file:///workspace/modules/adas_nodes/planning_node.cpp) | 规划节点 |
| `FrenetHandle` / `frenet_plan()` | [frenet_bridge.cpp](file:///workspace/src/algorithms/frenet_bridge.cpp) | FOT 包装 |
| `step_bicycle()` | [physics.cpp](file:///workspace/modules/adas_nodes/flowsim/physics.cpp) | 运动学自行车积分 |
| `Entity` / `EntityPool` | [entity.h](file:///workspace/modules/adas_nodes/flowsim/entity.h) | 仿真实体固定池（128，Ego=index 0） |
| `apply_safety()` | [safety_control_node.cpp](file:///workspace/modules/adas_nodes/safety_control_node.cpp) | 安全包络总入口（限幅+TTC+行人+横向交叉） |
| `publish_emergency_stop()` | [guardian_node.c](file:///workspace/modules/adas_nodes/guardian_node.c) | 熔断紧急制动 |

### 核心中间件

| 类/函数 | 文件 | 作用 |
|---------|------|------|
| `MessageBus` | [message_bus.h](file:///workspace/include/message_bus.h) | Pub/Sub 总线 |
| `Transport` / `transport_publish()` | [transport.h](file:///workspace/include/transport.h) | 统一传输抽象 |
| `Scheduler` | [scheduler.h](file:///workspace/include/scheduler.h) | 任务调度器 |
| `CoroutineTask` / `node_pump()` | [coroutine_task.h](file:///workspace/include/coroutine_task.h) | C++20 协程基类 |
| `StateMachine` / `statem_send_event()` | [state_machine.h](file:///workspace/include/state_machine.h) | 反射式状态机 |
| `param_register_float()` / `param_get_float()` | [param_registry.h](file:///workspace/include/param_registry.h) | 参数热重载 |
| `clock_now_us()` | [clock_service.h](file:///workspace/include/clock_service.h) | 统一时间戳 |
| `NodePlugin` | [node_plugin.h](file:///workspace/include/node_plugin.h) | 节点插件 ABI |

---

## 7. 依赖关系

### 7.1 构建依赖

| 依赖 | 版本 | 用途 | 缺失影响 |
|------|------|------|----------|
| GCC | 11+（C++20 协程） | 编译 | 必需 |
| CMake | 3.16+ | 构建 | 必需 |
| libcjson | 任意 | JSON 序列化（全链路强制） | 必需（CMake 会 FetchContent 从源码构建） |
| libeigen3 | 3.3+ | Frenet 规划器 | planning 回退 lane-keep-only，**不变道** |
| Python | 3.8+ | 代码生成 + 仪表盘 + 训练 | 工具链 |
| flowcoro | C++20 协程库 | 协程调度 | CMake FetchContent 拉取 |
| libprotobuf-c | 可选 | protobuf 支持 | 禁用 protobuf |

### 7.2 模块依赖图（控制相关）

```
control_node ─┬─► param_registry (热重载)  【原 mpc_controller 已删除】
              ├─► state_machine (变道状态机)
              ├─► road_geometry / topic_registry (道路几何)
              └─► adas_msgs_gen (ControlRaw 序列化)

planning_node ─┬─► frenet_bridge ─► FOT(第三方) ─► Eigen3
               ├─► CubicSpline2D (third_party/frenet_planner)
               └─► param_registry

safety_control ─► FlowCoro (协程) ─► ObstacleList 反序列化
actuator ─► SocketCAN / PCA9685 I2C / GPIO sysfs
flowsim ─► esminiRMLib (RoadPosition) ─► road_network / route
```

### 7.3 第三方

- [third_party/frenet_planner/](file:///workspace/third_party/frenet_planner) — Frenet Optimal Trajectory（含 CubicSpline1D/2D、QuarticPolynomial、QuinticPolynomial）
- `third_party/esmini`（git submodule）— OpenDRIVE 道路位置

---

## 8. 项目运行方式

### 8.1 环境要求

Linux（Ubuntu 20.04+），GCC 11+，CMake 3.16+。安装依赖：
```bash
sudo apt-get install -y libcjson-dev libeigen3-dev cmake g++ python3
```

> **离线/无网络环境**：CMake 对 cJSON 和 flowcoro 用 FetchContent 自动从 GitHub 拉取；eigen 可手动 clone 后放入 `build/_deps/eigen-src/`（需含 `Eigen/Core`），CMake 会识别为"已下载"。

### 8.2 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 或一键脚本
bash build.sh release
```

构建产物：
- `build/bin/flow_launcher` — 配置驱动 pipeline 启动器
- `build/bin/flowmond` — 监控守护进程
- `build/bin/flowctl` — CLI 工具
- `build/lib/lib*.so` — 各节点插件

### 8.3 运行

```bash
# 一键演示（构建+运行，默认 15s）
bash scripts/demo.sh
bash scripts/demo.sh 30          # 30 秒
bash scripts/demo.sh --multi     # 多进程模式
bash scripts/demo.sh --record    # 录制 Bag

# 手动：终端1 启动监控守护
./build/bin/flowmond --html-path tools/flowboard/index.html
# 终端2 运行 pipeline
./build/bin/flow_launcher config/pipeline.json --duration 3600
# 浏览器打开 http://localhost:8800
```

### 8.4 调参（边跑边调，无需重启）

```bash
flowctl param list                       # 所有实时参数
flowctl param get control.k_vy           # 读取
flowctl param set control.k_vy 0.5       # 下一帧生效
flowctl param set control.mpc_horizon 10 # 启用 MPC（已废弃，代码已删除）
flowctl topic stats control/raw_cmd      # topic 延迟/吞吐
flowctl graph                           # ASCII 拓扑
```

### 8.5 验证

```bash
# 改完 pipeline 链路必跑评估器（45s 真跑，PR 门禁）
python3 tools/demo_evaluator.py --duration 45 --interval 0.5
python3 tools/demo_evaluator.py --no-run    # 仅分析当前数据
```

评估器采样 `/tmp/flow_topology.json`，检查：拓扑完整性、topic 频率、碰撞、路沿偏离、停滞、变道次数、yaw/steer 振荡、NPC 瞬移、丢帧。WARN 可忽略，FAIL 必须修复。

### 8.6 Docker

```bash
docker build -t flowengine .
docker run --rm flowengine          # e2e 演示
docker run --rm flowengine demo 30
```

---

## 9. 控制调参指南（k_vy 专题）

### 9.1 当前横向控制架构

MPC 已从代码中删除。横向控制统一走 **Stanley + 横向速度规划**路径。变道时（`lc_active`）的核心公式：

```
v_y_des = k_vy * lat_error - k_vy_damp * v_lat_actual
ψ_des   = road_heading + asin(v_y_des / v)
steer   = cte_term - heading_term - yaw_damp + ff + delta_ff
```

默认值（[pipeline.json](file:///workspace/config/pipeline.json)）：`k_vy=0.35`、`k_vy_damp=0.6`、`lat_kp=0.5`、`lat_kd_heading=3.5`、`yaw_damping=0.28`。

### 9.2 参数矛盾矩阵（用户痛点）

| 参数 | 过高 | 过低 |
|------|------|------|
| `k_vy` | 横向速度过大 → 过冲冲出路面 | 变道太慢 → 被交通流碰撞 |
| `k_vy_damp` | 变道太慢（D 项过度抵消 P 项）→ 被碰撞 | 阻尼不足 → 灾难性过冲 |

这是一个**本质矛盾**：PD 控制器只看当前误差与变化趋势，没有"前方多远到目标"的预见性。`k_vy` 推、`k_vy_damp` 拉二者在时间轴上错位，加上执行器限幅（steer_limit）和级联延迟（位置→速度→航向→转角），形成难调的根因。

### 9.3 根因分析（为何"调旋钮"难以根治）

1. **级联控制延迟放大**：位置→速度→航向→转角四层嵌套，每层引入相位滞后。外环喊一声要等内环回音，回过头再收又要等——所有 P 增益都在跟物理延迟拔河。
2. **执行器饱和天花板**：`steer_limit_for_speed()` 在高速时仅 ~0.04 rad。数学公式算出需要 0.3 rad，物理只能给 0.04 rad。`k_vy` 调大 10 倍，转向机仍只输出满限幅——只是让算法"喊更大声"，执行器"腿已跑极限"。
3. **PD 无预见性**：只看此刻误差，不知"还剩 1 米到目标该减速"。这是与"梯形速度剖面"（带时刻表的规划器）的本质区别。

### 9.4 调参建议

**A. 在当前 PD 框架内缓解**（`flowctl param set` 边跑边调）：
- 先固定 `k_vy_damp`，扫 `k_vy`
- 观察过冲量 vs 变道耗时，找权衡点
- 配合 `lat_kd_heading`（heading 阻尼）和 `yaw_damping` 抑制极限环
- 注意 `steer_min_clamp`（高速最小转向限幅）和 `lc_lat_accel_max`

**B. 启用 MPC**（已删除，不再可用）

**C. 若需真正的梯形速度剖面**：当前代码**不存在**该模块。需在 planning 层新增：规划变道横向位移的梯形速度曲线（加速段→匀速段→减速段），下发 `v_y_des` 时间序列给 control 跟踪，把"规划"与"控制"剥离。这样调参维度从三维（P/D/限幅）降为一维（减速时机）。

### 9.5 常见故障速查（来自 [CLAUDE.md](file:///workspace/CLAUDE.md) 故障表）

| 现象 | 根因 | 位置 |
|------|------|------|
| MPC 输出每帧翻符号（bang-bang）【已删除】 | 求解器 max_steer=0.35 与外部限幅差 12.9 倍，平滑项失效 | 原 `control_node.cpp` |
| 变道冲出车道 | Stanley heading 阻尼硬编码，`lat_kd_heading` 未生效 | [control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp) |
| 车身左右晃（1-2Hz 极限环） | heading 重置逻辑使 v_lat_damp 失效，退化为纯 P | [flowsim_node.cpp](file:///workspace/modules/adas_nodes/flowsim_node.cpp) |
| steer 打到硬限幅抖动 | heading 漂移撞 clamp，`lc_lat_accel_max` 2.4→4.5，`steer_min_clamp` 0.016→0.030 | [control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp) |
| 红灯停稳后转绿不走 | planning 用 spd_out[0] 覆盖 command_speed 自维持闭锁 | [planning_node.cpp](file:///workspace/modules/adas_nodes/planning_node.cpp) |
| 车速降到 0 永久卡死 | ROAD_GUARD 恢复条件盲区，改为 speed<2.5 给小油门 | [control_node.cpp](file:///workspace/modules/adas_nodes/control_node.cpp) |

---

## 附录：文档导航

| 文档 | 主题 |
|-----|------|
| [QUICK_START.md](file:///workspace/docs/QUICK_START.md) | 30 分钟教程 |
| [TECHNICAL_DESIGN.md](file:///workspace/docs/TECHNICAL_DESIGN.md) | 架构设计 |
| [PIPELINE_ARCHITECTURE.md](file:///workspace/docs/PIPELINE_ARCHITECTURE.md) | Pipeline 设计 |
| [ALGORITHM_STACK.md](file:///workspace/docs/ALGORITHM_STACK.md) | 算法总览 |
| [CALIBRATION_GUIDE.md](file:///workspace/docs/CALIBRATION_GUIDE.md) | 标定指南（含 k_vy 等） |
| [LEARNING_LOOP.md](file:///workspace/docs/LEARNING_LOOP.md) | 车端学习闭环 |
| `docs/tutorials/` 目录 | 17 篇深度教程（OOP in C、插件、消息总线、IPC、协程、Fusion 等） |
