# FlowEngine 算法栈（实际实现）

> 本文是**当前代码里真实跑着的算法**总览：每个模块用什么算法、在哪个文件、
> 关键约束是什么。深入某一模块请跳对应教程/设计文档。

## 数据流

```
flowsim(60Hz 真值) → sensor_model(FOV/遮挡/噪声) → perception(DBSCAN)
    → object_tracker(卡尔曼跟踪) → fusion(EKF 定位)
    → behavior_planner(FSM 决策) ⇄ navigation(路由/travel_dir)
    → planning(Frenet 轨迹 + ST 图 DP 速度 + 掉头 N 把方向)
    → control(LTV MPC 横向 + PID 纵向 + ManeuverTracker)
    → safety_control(TTC 安全闸门) → 执行
旁路: inference(tiny-MLP 影子) / data_recorder / learner / model_ota(学习闭环)
```

## 模块 × 算法一览

| 模块 | 算法 | 文件 | 要点 |
|------|------|------|------|
| 仿真 | 运动学自行车模型（中心参考点含 half_wb·yaw_rate 切向项） | `flowsim/physics.cpp` | 纯被控对象，只执行指令不做驾驶决策 |
| NPC | IDM 跟车 + 中央 Route Frenet 车道跟随（MOBIL 变道已 `#if 0` 禁用） | `flowsim/npc_ai.cpp` `flowsim/route.cpp` | 安全间距 = 5.0 + v×1.5 |
| 传感器模型 | FOV 裁剪 + 视线遮挡 + 高斯噪声（LiDAR/GPS/Camera） | `sensor_model_node.c` | |
| 感知 | DBSCAN 点云聚类 | `perception_node.cpp` | 点云过多时聚类耗时可能超 deadline（已知降频点） |
| 跟踪 | 卡尔曼多目标跟踪 | `object_tracker_node.c` | 20Hz |
| 融合 | EKF（GPS+IMU 定位、时间对齐） | `fusion_node.cpp` | 20Hz |
| 行为 | 8 状态 FSM：CRUISE / FOLLOW / LEFT_CHANGE / RIGHT_CHANGE / STOP / YIELD / U_TURN / EMERGENCY | `behavior_planner_node.cpp` | 只决定"做什么"，不输出连续控制量 |
| 导航 | 路由步骤（enter_noa / prepare_u_turn）+ travel_dir（唯一事实源 = flowsim `ref_path.reverse`） | `navigation_node.c` | 10Hz |
| 规划-路径 | Frenet 最优轨迹（横向五次多项式；变道段固定曲率圆弧回填 kappa） | `planning_node.cpp` | 速度/轨迹唯一权威 |
| 规划-速度 | **ST 图 + DP 速度规划**（红灯墙 + 动态障碍占据 + 曲率限速，90×101 DP 表） | `st_graph.c` | 1:1 移植自 `tools/speed_planner_sim.py`（11/11 场景 PASS 后冻结） |
| 规划-掉头 | N 把方向多段掉头（前进满舵弧 → 刹停换 R → 倒车反打 → 循环，≤5 把），512 点细生成 + 段感知下采样 64 | `planning_node.cpp` generate_uturn_trajectory | 生成一次即缓存重放，防「重规划抖动死循环」 |
| 控制-横向 | Stanley + kappa 前馈（默认）；LTV MPC（`use_ltv_mpc` 参数启用，机动模式跳过） | `control_node.cpp` `ltv_mpc.c` | MPC 求解前注入真实 steer 限幅（内外限幅必须一致） |
| 控制-纵向 | PID + ACC 跟车（anti-windup：error 翻负清正积分） | `control_node.cpp` | 20Hz |
| 控制-机动 | ManeuverTracker：弧长推进 + D/R 挡位状态机 + 倒挡横向反馈反号 | `maneuver_tracker.h` | 掉头/泊车共用；header-only 可独立单测 |
| 安全 | TTC 闸门（前车/迎头/行人/横向交叉，全部沿车头方向投影）+ MRM + 机动窗口豁免 | `safety_control_node.cpp` | FlowCoro 协程；只做限幅+紧急制动，不理解任务意图 |
| 学习 | BC（tiny-MLP / PyTorch）→ DAgger 自我对弈回灌 → PPO（换老师路线）；ONNX 导出 + 等价性门禁；影子评估 + promote 门禁 + OTA | `inference_node.cpp` `tools/train_e2e/` | 详见 [LEARNING_LOOP.md](LEARNING_LOOP.md) |

## 关键设计约束（跨模块铁律）

- **速度与轨迹唯一权威 = planning**；control 纯跟随不质疑轨迹；safety 只限幅不改任务性指令。
- **行进方向唯一事实源 = flowsim `road/ref_path.reverse`**，navigation/behavior/control 全部消费它。
- control→safety 带外信号唯一通道 = `raw.mode` 字符串（如 `"+MANEUVER"` 后缀）。
- 算法升级必须**先 Python 仿真验证再移植 C++**（`tools/control_sim.py` 6 场景 + `tools/speed_planner_sim.py`），
  移植后跑 `ci/evaluators/demo_evaluator.py` 验证行为一致。

## 性能实测参考

| 阶段 | 频率 | 备注 |
|------|------|------|
| flowsim 物理 | 60Hz | 场景 JSON 加载 actor |
| 感知 DBSCAN | 10Hz | 点云规模敏感 |
| 融合 EKF | 20Hz | |
| 规划（Frenet+ST 图） | 20Hz | DP 表静态 142KB，单线程 |
| 控制（MPC+PID） | 20Hz | |
| 消息总线 | — | 进程内 <100µs |

## 第三方算法库集成

想把 OpenCV/YOLO、Eigen、OSQP 等外部算法库接成插件节点？集成模式
（dlopen 插件 API、topic 订阅发布、构建方法）见
[tutorials/02_plugin_system.md](tutorials/02_plugin_system.md) 与
[ALGORITHM_INTEGRATION.md](ALGORITHM_INTEGRATION.md)。本仓库不内置这些依赖。
