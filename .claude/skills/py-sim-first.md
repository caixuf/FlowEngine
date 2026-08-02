---
name: py-sim-first
description: 算法模块升级的 Python 仿真先行流程。任何控制/规划/感知/行为决策类改动，先 Python 验证再移植到 C++。包含 6 场景仿真、参数扫描、debug 工具。
---

# Python 仿真先行 — 算法升级工作流

> 任何算法模块升级（控制/规划/感知/行为决策），**必须先 Python 仿真验证**，
> 再移植到 C++。`tools/control_sim.py` 提供 6 种上路操作场景 + 参数扫描。

## 一、什么时候用

| 场景 | 示例 | 是否必须 py-sim-first |
|------|------|----------------------|
| 新增控制器 | 把 Stanley 换成 LQR 或 MPC | **是** |
| 调参 | 改 `lat_kp` / `r_ddelta` / 权重 | **是**（`--tune-*` 自动扫） |
| 新增场景 | 增加掉头/回环/多弯道 | **是**（先注册到 `--run-all`） |
| 修 bug | 转弯冲出/紧急制动撞了 | **是**（Python 复现再 C++ 修） |
| 改物理模型 | 轮胎模型/执行器延迟 | 需要；仿真对齐后再 C++ |
| 改 JSON 配置 | pipeline.json 字段 | 不需要（直接改 + 跑 demo_evaluator） |
| 修复前端 | 3D 渲染 / 仪表盘 | 不需要（走 `npm run vis:check`） |

## 二、工具总览

### 核心文件

| 文件 | 作用 |
|------|------|
| `tools/control_sim.py` | 控制仿真 + 6 场景 + 参数扫描 + 掉头 |
| `tools/control_sim.py --run-all` | 一键全量 6 场景，看汇总 |
| `tools/control_sim.py --scene-<name>` | 单个场景 |
| `tools/control_sim.py --tune-<name>` | 自动参数扫描 |

### 6 场景概览

| 场景 | CLI 参数 | 默认参数 | 测试什么 |
|------|---------|---------|----------|
| **curve** 曲线跟随 | `--scene-curve` | kappa=0.005, 12m/s, 15s | Stanley/MPC 弯道跟踪精度 |
| **emergency** 紧急制动 | `--scene-emergency` | 15m/s, 障碍物50m | 纵向刹停，碰撞避免 |
| **stop_go** 跟停再起步 | `--scene-stop-go` | 10m/s, 前车gap=30m, 20s | ACC 跟车，前车停→走 |
| **obstacle** 障碍物避让 | `--scene-obstacle` | 12m/s, 障碍物60m | 变道绕行，横向+纵向协同 |
| **merge** 匝道汇入 | `--scene-merge` | 8→15m/s, merge_x=50m | 加速车道汇入主路 |
| **cutin** 加塞处理 | `--scene-cutin` | 12m/s, cutin_v=10m/s, 30m | TTC 预警+减速避让 |

### 参数扫描

| 命令 | 扫描维度 | 输出 |
|------|---------|------|
| `--tune-curve` | kp, kd_heading, yaw_damp, speed (4×5×4×3=240 组合) | 最优参数 + score |
| `--tune-emergency` | init_speed (3 档) | 各档位 PASS/FAIL |
| `--tune-stop-go` | init_speed, initial_gap (3×3=9 组合) | 各组合 PASS/FAIL |
| `--tune-merge` | init_v, target_v, merge_x (3×3×3=27 组合) | 各组合 PASS/FAIL |
| `--tune-cutin` | init_v, cutin_v, cutin_x (3×3×3=27 组合) | 各组合 PASS/FAIL |
| `--tune-uturn` | steer, speed, duration (网格搜索) | 最优掉头参数 |

### 掉头仿真

| 命令 | 说明 |
|------|------|
| `--uturn` | 默认三把方向掉头（匹配 C 代码参数） |
| `--tune-uturn` | 扫描掉头最优参数（转向角/速度/持续时间） |

## 三、工作流详解

### 3.1 快速验证：`--run-all`

```bash
cd /path/to/FlowEngine
python3 tools/control_sim.py --run-all
```

预期输出：
```
  ============================================================
  场景全集 — 6 种上路操作全量验证
  ============================================================
  ── 场景 1: 曲线跟随 ──
  [PASS] 曲线跟随 (kappa=0.005, R=200m, 12m/s)
  avg_heading_err=0.0007rad, avg_lat_err=0.23m
  ...
  ── 场景 6: 加塞处理 ──
  [PASS] 加塞处理 (v0=12m/s, cutin_v=10m/s, cutin_x=30m)
  min_gap=25.0m, 无碰撞
  ...
  场景全集汇总: 6/6 PASS, 0/6 FAIL
```

**全量 PASS 后才能进入下一步。**

### 3.2 参数扫描：`--tune-*`

```bash
# 曲线跟随最优参数
python3 tools/control_sim.py --tune-curve

# 紧急制动参数
python3 tools/control_sim.py --tune-emergency

# 跟停再起步参数
python3 tools/control_sim.py --tune-stop-go

# 匝道汇入参数
python3 tools/control_sim.py --tune-merge

# 加塞处理参数
python3 tools/control_sim.py --tune-cutin

# 掉头参数
python3 tools/control_sim.py --tune-uturn
```

### 3.3 单场景调试

```bash
# 指定速度/时长
python3 tools/control_sim.py --scene-curve --speed 15 --duration 20

# 紧急制动测不同距离
python3 tools/control_sim.py --scene-emergency --speed 20

# 跟停再起步
python3 tools/control_sim.py --scene-stop-go --speed 10
```

### 3.4 控制器选择

默认使用 Stanley 控制器。可通过修改 `run_*_scenario` 函数中的 `use_mpc=True` 参数启用 LTV-MPC。

```python
# 在 control_sim.py 中修改
r = run_curve_scenario(kappa=0.005, target_speed=12.0, use_mpc=True)
```

### 3.5 输出 CSV 轨迹

```bash
python3 tools/control_sim.py --scene-curve --csv /tmp/curve_traj.csv
```

CSV 包含：`t, x, y, v, heading, steer`，可用于逐帧对比 C++ 输出。

## 四、代码结构

### 目录结构

```
tools/control_sim.py
├── LtvMpcSolver            # LTV-MPC 求解器（完全移植 ltv_mpc.c）
├── StanleyParams            # Stanley 控制器参数
├── steer_limit_for_speed()  # 速度相关转向限幅
├── stanley_control()        # 改进版 Stanley 控制器
├── VehicleState             # 运动学自行车模型
├── PlanningLayer            # 规划层仿真
├── ControlLayer             # 控制层仿真
├── ScenarioResult           # 场景仿真结果
├── CurvedRoad               # 常曲率弯道模型
├── LongitudinalController   # 纵向控制器（ACC + 紧急制动）
├── run_curve_scenario()     # 场景1: 曲线跟随
├── run_emergency_brake_scenario()  # 场景2: 紧急制动
├── run_stop_go_scenario()   # 场景3: 跟停再起步
├── run_obstacle_avoid_scenario()   # 场景4: 障碍物避让
├── run_merge_scenario()     # 场景5: 匝道汇入
├── run_cutin_scenario()     # 场景6: 加塞处理
├── tune_*()                 # 参数扫描函数
└── main()                   # CLI 入口
```

### 添加新场景

1. 在 `control_sim.py` 中实现 `run_new_scenario()` 函数，返回 `ScenarioResult`
2. 在 `main()` 中添加 `--scene-new` 参数
3. 在 `--run-all` 分支中添加新场景调用
4. 更新本文档的场景列表

## 五、常见故障排查

### 5.1 参考路径链路不一致（最常见故障）

**现象**：右转/支路场景下 control 跟踪偏出路沿的坏轨迹，但单测各节点都正常。

**根因**：flowsim→planning→control 参考路径不一致：
1. flowsim `publish_ref_path` 用 lane centerline（含当前车道偏移）→ `ref_path` 已经偏离道路中心
2. planning 在偏移的 ref_path 上再叠加 `target_lane_offset` → 双重偏移
3. control 忠实跟踪这条已经偏出路沿的坏轨迹

**修复**：
- `publish_ref_path` 优先用 route centerline（d=0=道路中心），lane centerline 仅作 fallback
- `planning_node.cpp` 用 `project_to_reference_path` 把 ego 投影到 map_ref 的 Frenet 弧长坐标系
- `control_node.cpp` 直接从 trajectory 前视点取 `target_path_y`，替代 `road_center_y + lane_d` 混拼

**排查方法**：在 planning/debug 中对比 `ref_heading`、`ref_x/ref_y`、`ego_d` 和 `target_lane_offset` 的值。

**对向行驶（掉头返程）变体**：掉头后 ego 在对向车道，实际行进方向 = **-route_s**。
- flowsim `Route::sample_ahead` 加 `reverse` 参数：沿 -route_s 采样、每点 heading 翻转 π、out 按行进方向有序、kappa 在行进坐标系计算 —— `ref_path` 始终代表"ego 真实行进方向前方的道路"，planning/control 无需知道 +x/-x
- 掉头 finalize 对向车道 heading 翻转 π，否则车头仍指 +x 撞路尾卡死
- **方向检测不得用 dx/heading 猜测**：三把方向掉头中 heading 扫过 ±x 两个半平面，去抖压不住会反复翻转 → route 步骤重放把返程 ego 拽回前进车道绕圈。唯一事实源 = flowsim 每帧发布的 `road/ref_path.reverse`（= `u_turn_active`），navigation/behavior 据此切返程
- 排查：日志里 `travel direction` 一行每掉头应只翻一次；掉头处反复出现 → 有人在用 dx/heading 猜方向

### 5.2 Python 仿真 PASS 但 C++ 不工作

| 排查项 | 方法 |
|--------|------|
| 物理模型不一致 | 输出 CSV 轨迹，与 C++ 逐帧对比 `steer/v/heading` |
| 转向限幅差异 | C++ `physics.cpp` 的 `steer_limit` 是否与 Python `steer_limit_for_speed` 一致 |
| 执行器延迟 | C++ `steer_tau` / `steer_rate_max` 是否与 Python `VehicleState.step` 一致 |
| 控制器参数 | C++ pipeline.json 的 `lat_kp` / `lat_kd_heading` 是否与 Python `StanleyParams` 一致 |
| 时序 | C++ 控制周期 20Hz vs Python `DT=0.05s` 是否一致 |

### 5.2 场景 FAIL

| 现象 | 常见原因 | 修复 |
|------|---------|------|
| curve: avg_heading_err > 0.08 | 弯道太急/速度太快 | 降低 kappa 或 speed |
| curve: avg_lat_err > 0.5 | Stanley 参数不合适 | `--tune-curve` 找最优参数 |
| emergency: collision | 制动距离不够 | 降低 init_speed 或增大 obstacle_dist |
| stop_go: collision | 跟车太近 | 增大 initial_gap 或降低 init_speed |
| obstacle: collision | 变道空间不够 | 增大 obs_x 或降低 speed |
| merge: FAIL | 速度差太大/merge_x 太短 | 降低 target_speed 或增大 merge_x |
| cutin: collision | cutin_x 太近 | 增大 cutin_x 或降低 init_speed |

### 5.3 参数扫描结果异常

```bash
# 确认是在用正确的参数做扫描
# tune_curve 必须传 stanley_params 参数，否则扫的是默认参数
# 正确：run_curve_scenario(..., stanley_params=p)
# 错误：run_curve_scenario(...)  # 没传自定义参数
```

## 六、移植到 C++ 的 checklist

- [ ] Python `--run-all` 全量 PASS
- [ ] `--tune-*` 已找到最优参数
- [ ] 控制器类型已确定（Stanley / MPC / LQR）
- [ ] 确认 C++ 物理模型与 Python `VehicleState` 一致
  - [ ] 轴距 `WHEELBASE` = 2.7m
  - [ ] 控制周期 `DT` = 0.05s (20Hz)
  - [ ] 运动学积分公式一致
  - [ ] 转向限幅公式一致
- [ ] 移植后跑 `demo_evaluator.py` 验证
- [ ] 跑 `pipeline_check.py` 检查完整性
- [ ] 更新 `CLAUDE.md` 常见故障模式表（如有新发现）