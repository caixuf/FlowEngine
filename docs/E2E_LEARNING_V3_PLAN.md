# 端到端学习闭环 v3 — 升级计划

> **状态：设计阶段。**
> v1（4 维特征 → 1 维 speed）和 v2（16 维特征 × 5 帧时序 → 5 维控制）
> 的基础设施已全部落地。v3 的目标是从"模仿 planning 的速度"升级为
> **"从感知直接输出控制指令"的端到端驾驶模型**，并在闭环中验证。

---

## 零、现状盘点

### 已完成的 v1/v2 基础设施

| 组件 | 能力 | 位置 |
|------|------|------|
| `data_recorder_node` | 采集 JSONL 样本（v1 features + v2 features_v2 + obstacles + control） | `modules/adas_nodes/data_recorder_node.c` |
| `train.py` | 零依赖 MLP 训练（v1: 4→8→1） | `tools/train/train.py` |
| `temporal_train.py` | 时序窗口训练（v2: 5×16=80→32→5），输出 throttle/brake/steer/lane_change/confidence | `tools/train_e2e/temporal_train.py` |
| `feature_schema.py` | v1/v2 特征名统一定义 + 多源特征拼装 | `tools/train_e2e/feature_schema.py` |
| `train_demo_model.py` | 一键训练入口（torch/tiny backend） | `tools/train_demo_model.py` |
| `inference_node` | 影子模式推理，支持时序窗口（5 帧缓冲），OTA 热重载，3 种控制模式 | `modules/adas_nodes/inference_node.cpp` |
| `tiny_mlp.h` | 纯 C 推理内核，v2 格式支持多输出维度 | `modules/adas_nodes/tiny_mlp.h` |
| `learner_node` | 在线 SGD 微调，ring buffer 采样，防灾难性遗忘 | `modules/adas_nodes/learner_node.c` |
| `model_ota_node` | 模型版本注册表 + 热重载 + 回滚 + A-B 测试 | `modules/adas_nodes/model_ota_node.c` |
| `modelctl.py` | CLI：list/promote/ota push/rollback/ab-test | `tools/modelctl.py` |

### 当前特征维度（v2, 16 维/帧）

```
索引  特征名              来源 topic              说明
─────────────────────────────────────────────────────────
 0    ego_v               fusion/localization     自车速度
 1    ego_y               fusion/localization     自车横向位置
 2    ego_heading         fusion/localization     自车航向
 3    ego_yaw_rate        fusion/localization     自车横摆角速度
 4    front0_x            perception/obstacles    最近前车 x
 5    front0_y            perception/obstacles    最近前车 y
 6    front0_vx           perception/obstacles    最近前车 vx
 7    front0_type         perception/obstacles    类型 (0=未知,1=车,2=行人)
 8    front0_confidence   perception/obstacles    检测置信度
 9    front1_x            perception/obstacles    次近前车 x
10    front1_y            perception/obstacles    次近前车 y
11    front1_vx           perception/obstacles    次近前车 vx
12    front1_type         perception/obstacles    类型
13    front1_confidence   perception/obstacles    置信度
14    control_brake       control/cmd             当前刹车量
15    control_emergency   control/cmd             紧急停车标志
```

### v2 的核心限制

| 限制 | 影响 |
|------|------|
| **障碍物来自真值** | perception_node 从 vehicle/state 读 ox/oy 撒点做 DBSCAN，本质是真值作弊。特征里的 front0_x/y/vx 不经过传感器噪声 |
| **无红绿灯/道路特征** | 模型不知道前方是红灯还是绿灯，不知道弯道曲率 |
| **感知特征太粗** | 只取前 2 个障碍物，没有障碍物列表全貌；没有 LiDAR 点云密度/分布统计 |
| **时序处理弱** | 5 帧直接拼接（80 维 → MLP），没有真正的时序模型（RNN/Transformer） |
| **无闭环评估** | 影子模式只做对比，从未让模型实际控制车辆跑一段并评分 |
| **单场景训练** | 每个场景独立训练，没有多场景泛化能力测试 |
| **单隐层 MLP** | 80→32→5，表达能力有限 |

---

## 一、v3 目标架构

```
                         ┌──────────────────────────┐
  perception/obstacles ──┤                          │
  sensor/lidar ──────────┤   Feature Builder        │
  road/geometry ─────────┤   (多源融合 + 时序缓冲)    │──► 80+ 维/帧
  road/traffic_lights ───┤                          │
  vehicle/state ─────────┤                          │
  control/cmd ───────────┘                          │
                                                         │
                                                         ▼
                                              ┌──────────────────┐
                                              │  Temporal Model  │
                                              │  (MLP / LSTM /   │
                                              │   Transformer)   │
                                              └────────┬─────────┘
                                                       │
                                                       ▼
                                              throttle, brake, steer
                                              (3 维控制输出)
                                                       │
                                          ┌────────────┴────────────┐
                                          ▼                         ▼
                                   shadow mode               direct_ctrl mode
                                   (对比 planning)           (经 safety_control 兜底)
```

### v3 升级清单

| 编号 | 升级 | 说明 |
|------|------|------|
| **F1** | 感知特征增强 | 障碍物从真值切换为 sensor/lidar 点云统计特征（密度/分布/聚类数） |
| **F2** | 场景上下文特征 | 新增红绿灯状态、道路曲率、限速、当前段类型 |
| **F3** | 全障碍物特征 | 不再只取前 2 个，编码整个障碍物列表（embedding 或统计量） |
| **M1** | 模型升级 | 支持更深 MLP（多隐层）+ 可选 LSTM/GRU 时序建模 |
| **M2** | ONNX 推理后端 | 保留 tiny_mlp.h 纯 C 路径，新增 ONNX Runtime 可选后端 |
| **T1** | 多场景训练 | 多场景混合训练 → 单一模型泛化 |
| **T2** | 数据增强 | 传感器噪声注入、障碍物位置扰动、速度抖动 |
| **E1** | 闭环评估 | 定期把推理结果注入控制链路跑一段（带 safety_control），自动评分 |
| **E2** | 渐进式灰度 | shadow → plan_assist → direct_ctrl（已有控制模式枚举，只需打通） |
| **G1** | 模型回归测试 | 每次训练后自动跑 14 个场景，对比 planning 的 speed/steer 误差 |

---

## 二、特征工程：v2（16 维）→ v3（~40 维/帧）

### 2.1 新增感知特征（替代真值）

当前障碍物来自 `perception/obstacles`（DBSCAN 聚类输出），但 perception 本身从真值撒点——本质没经过传感器噪声。
v3 在 **perception↔sensor_model 数据流修复** 完成后，障碍物特征才具有真实的传感器噪声/漏检/FOV 特性。

新增的感知统计特征（从 LiDAR 点云计算，不依赖聚类结果）：

```
索引  特征名              来源          说明
─────────────────────────────────────────────────
16    lidar_point_count    sensor/lidar  点云总点数
17    lidar_front_density  sensor/lidar  前方 30m 点密度
18    lidar_clutter_ratio  sensor/lidar  杂波比例 (z<0.3m 的点/总点)
19    perception_obj_count perception/obstacles  检测到的障碍物数
20    perception_max_conf  perception/obstacles  最高置信度
21    perception_mean_conf perception/obstacles  平均置信度
```

### 2.2 新增场景上下文特征

```
索引  特征名              来源              说明
───────────────────────────────────────────────────
22    tl_state             road/traffic_lights  0=green, 1=yellow, 2=red, -1=无灯
23    tl_distance          road/traffic_lights  距最近红灯的距离 (m)，无灯时=-1
24    road_curvature       road/geometry        当前道路曲率 (1/R, m⁻¹)
25    road_speed_limit     road/geometry        当前限速 (m/s)
26    lane_count           road/geometry        当前车道数
27    lane_width           road/geometry        车道宽度 (m)
28    ego_lane_offset      vehicle/state        自车距车道中心线偏移 (m)
```

### 2.3 感知输出增强

DBSCAN 聚类障碍物列表的编码（替代只取前 2 个的五元组）：

```
索引  特征名              说明
─────────────────────────────────────────
29    obs_count            障碍物总数（≤16）
30-34 obs_N_type          障碍物类型分布 (车/卡车/行人/未知/总计)
35-39 obs_N_distance      距离分布统计 (min/p25/p50/p75/max)
40-44 obs_N_speed         速度分布统计
45-49 obs_front_min_gap   前方最近间距
50-54 obs_left_min_gap    左侧最近间距
55-59 obs_right_min_gap   右侧最近间距
```

> **简化原则**：v3 第一阶段不追求完整 embedding 向量，用统计量把任意数量障碍物压缩到固定维度。统计量对 MLP 友好且可解释。

### 2.4 总特征维度

| 组 | 维数 | 来源 |
|----|------|------|
| v2 基础（ego + 前2障碍物 + control） | 16 | 现有，不变 |
| 感知统计 | 6 | sensor/lidar + perception/obstacles |
| 场景上下文 | 7 | road/traffic_lights + road/geometry |
| 障碍物全貌统计 | 30 | perception/obstacles |
| **合计** | **~59 维/帧** | |

5 帧时序窗口：59×5 = **295 维**（比当前 80 维扩大 3.7×）

---

## 三、模型架构

### 3.1 三档模型选择

| 档位 | 架构 | 参数量 | 推理耗时 (ARM A76) | 适用场景 |
|------|------|--------|---------------------|---------|
| **Tiny**（当前） | 单隐层 MLP: 80→32→5 | ~2.8K | <5μs | 嵌入式 MCU |
| **Base**（v3 默认）| 双隐层 MLP: 295→128→64→3 | ~47K | <20μs | 车端 SoC (Orin/RK3588) |
| **Pro**（可选）| LSTM: 59×5 → hidden 128 → 3 | ~100K | <200μs | GPU/TPU 推理 |

v3 **默认使用 Base 档**（双隐层 MLP），保留 Tiny 档向后兼容。
Pro 档（LSTM/Transformer）作为可选实验路径，不阻塞 v3 主流程。

### 3.2 输出维度

```
v1:  [target_speed]                         1 维
v2:  [throttle, brake, steer, lc, conf]    5 维
v3:  [throttle, brake, steer]               3 维（精简，去掉冗余的 lc/conf）
```

`lane_change` 和 `confidence` 是 v2 的辅助输出，在 v3 中：
- 变道决策由 planning_node 的 Frenet 规划器显式处理（不是 MLP 隐式学）
- 置信度由模型输出方差估算（epistemic uncertainty），不在输出向量里

### 3.3 模型格式扩展

`tiny_mlp.h` 的 `model.txt` 格式扩展为 v3：

```
# flowengine-tinymlp v3
# features: 295-dim temporal (5x59) — ego + perception stats + scene context
# output:   [throttle, brake, steer]
in 295
hidden 128 64       # 双隐层：hid1=128, hid2=64
out 3
norm_mean   <295 floats>
norm_scale  <295 floats>
out_mean    <3 floats>
out_scale   <3 floats>
w1 <128*295 floats>
b1 <128 floats>
w2 <64*128 floats>
b2 <64 floats>
w3 <3*64 floats>
b3 <3 floats>
```

向后兼容：`hidden` 字段只有一个数时视为单隐层（v1/v2 格式），多个数时依次为各隐层大小。

---

## 四、训练管线升级

### 4.1 多场景混合训练

```bash
# 采集所有 14 个场景的样本
python3 tools/scenario_regression.py --suite scenarios/suite.json --record

# 混合训练（自动划分 train/val/test）
python3 tools/train_e2e/temporal_train.py \
  --input /tmp/flow_train_samples_all.jsonl \
  --output tools/train/model_v3.txt \
  --hidden 128 64 \
  --epochs 500 \
  --lr 0.001 \
  --val-split 0.15
```

### 4.2 数据增强

在训练时对障碍物位置/速度注入高斯噪声，模拟传感器不确定性：

```
增强策略:
  - 障碍物 x:  ±0.15m 高斯噪声
  - 障碍物 y:  ±0.10m 高斯噪声
  - 障碍物 vx: ±0.5m/s 高斯噪声
  - 随机丢弃: 20% 概率丢弃某个障碍物（模拟漏检）
  - 随机新增: 10% 概率新增假障碍物（模拟 false positive）
  - 红绿灯误读: 5% 概率翻转 tl_state（模拟感知错误）
```

### 4.3 训练指标

| 指标 | 说明 | 目标 |
|------|------|------|
| `val_loss` (MSE) | 验证集 MSE | < 0.01 |
| `speed_mae` | 速度预测平均绝对误差 | < 0.5 m/s |
| `steer_mae` | 转向预测 MAE | < 0.02 rad |
| `lane_deviation` | 闭环 30s 内最大车道偏离 | < 0.5m |
| `collision_rate` | 闭环 30s 内碰撞率 | 0% |

---

## 五、闭环评估系统

### 5.1 核心思路

当前影子模式只做"预测 vs planning"的数值对比。v3 新增**周期性闭环测试**：

```
每 N 步训练后:
  1. 加载当前模型到 inference_node (control_mode=direct_ctrl)
  2. 跑 14 个场景各 30s
  3. demo_evaluator.py 自动评分
  4. 记录 metrics → 模型注册表
  5. 如果比上一版差 → 自动回滚
```

### 5.2 安全屏障

`direct_ctrl` 模式下，推理输出必须经过 `safety_control_node`：

```
inference_node → inference/raw_cmd → safety_control_node → control/cmd → flowsim
                                         ↑
                                    TTC < 2s → 强制刹车
                                    偏离车道 > 1m → 强制回正
                                    速度 > 限速 × 1.2 → 强制限速
```

`safety_control_node` 是最后一道防线——即便模型输出全错，也不会撞车。

### 5.3 闭环评估脚本

```bash
# 单模型闭环评估
python3 tools/eval_closed_loop.py \
  --model tools/train/model_v3.txt \
  --scenarios scenarios/suite.json \
  --duration 30 \
  --output models/model_v3/eval_results.json

# 输出示例:
# {
#   "model": "model_v3",
#   "scenarios_passed": "13/14",
#   "avg_speed_mae": 0.38,
#   "avg_steer_mae": 0.015,
#   "max_lane_deviation_m": 0.42,
#   "collisions": 0,
#   "emergency_stops_triggered": 2,
#   "grade": "A"
# }
```

---

## 六、渐进式灰度上车

v2 已定义三种控制模式（`shadow` / `plan_assist` / `direct_ctrl`），v3 新增自动化灰度流程：

```
Step 1: shadow mode (当前默认)
  └── 模型与 planning 并行跑，只对比不控制
  └── 持续时间: 至少 1000 帧影子对比

Step 2: plan_assist
  └── 模型输出作为 planning 的参考（如变道时机建议）
  └── planning 仍做最终决策
  └── 触发条件: shadow MAE < 阈值 持续 500 帧

Step 3: direct_ctrl + safety
  └── 模型直接输出控制指令
  └── safety_control_node 兜底
  └── 触发条件: plan_assist 通过 14 场景闭环

Step 4: direct_ctrl (无 safety 干预)
  └── 仅当 1000 帧 direct_ctrl 中 safety 从未介入
  └── 持续监控，一有 safety 介入 → 降级回 Step 3
```

---

## 七、实施路径

### Phase 1 — 特征升级（~2 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 1.1 扩展 feature_schema v3 | `tools/train_e2e/feature_schema.py` | FEATURE_NAMES_V3 (59 维) + build_v3_features() |
| 1.2 data_recorder 记录 v3 特征 | `modules/adas_nodes/data_recorder_node.c` | 新增 lidar / tl / road_curve 字段锁存 + features_v3 写入 |
| 1.3 inference_node 支持 v3 特征 | `modules/adas_nodes/inference_node.cpp` | frame_buf 扩展为 59 维，订阅新增 topic |
| 1.4 单元测试 | `tests/` | 验证 features_v3 维度 + 归一化参数一致性 |

### Phase 2 — 模型升级（~2 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 2.1 多隐层 MLP 实现 | `tools/train_e2e/temporal_train.py` | 支持 `--hidden 128 64` |
| 2.2 tiny_mlp.h v3 格式 | `modules/adas_nodes/tiny_mlp.h` | 支持多隐层 + v3 header |
| 2.3 model.txt v3 导出/加载 | inference_node + train 两端 | 双向兼容 v1/v2/v3 格式 |
| 2.4 合成数据训练验证 | 用合成数据训一个 v3 模型，确认推理输出合理 |

### Phase 3 — 训练管线（~2 天）

| 步骤 | 产出 |
|------|------|
| 3.1 多场景 JSONL 合并工具 | `tools/dataset/merge_samples.py` |
| 3.2 数据增强管线 | `temporal_train.py --augment` |
| 3.3 train/val/test 自动划分 | `temporal_train.py --val-split --test-split` |
| 3.4 训练日志 + TensorBoard 兼容输出 | CSV epochs log |
| 3.5 14 场景全量采集脚本 | `scripts/collect_all_samples.sh` |

### Phase 4 — 闭环评估（~2 天）

| 步骤 | 文件 | 产出 |
|------|------|------|
| 4.1 闭环评估脚本 | `tools/eval_closed_loop.py` | 自动切换 direct_ctrl → 跑场景 → 评分 |
| 4.2 模型评分卡 | 同上 | speed_mae / steer_mae / lane_dev / collision / grade |
| 4.3 自动回滚逻辑 | `tools/modelctl.py` | 评分低于基线 → 自动 rollback |
| 4.4 灰度状态机 | `tools/modelctl.py ota promote --canary` | 逐步从 shadow→direct_ctrl |

### Phase 5 — 调优与文档（~1 天）

| 步骤 | 产出 |
|------|------|
| 5.1 超参数搜索 | 隐层大小 / 学习率 / 时序窗口 网格搜索 |
| 5.2 更新 LEARNING_LOOP.md | v3 状态 + 快速上手 |
| 5.3 更新 EVOLUTION_ROADMAP.md | v3 阶段标记 |

**总计：约 9 天（比 NOA 场景的 10 天略少，因为学习闭环基础设施已很完整）**

---

## 八、向后兼容

| 旧特性 | 兼容方式 |
|--------|----------|
| v1 model.txt (`in 4 hidden 8 out 1`) | `tiny_mlp.h` 自动检测 header 版本，走 v1 forward |
| v2 model.txt (`in 80 hidden 32 out 5`) | `hidden` 字段只有一个数 → 单隐层路径 |
| v3 model.txt (`in 295 hidden 128 64 out 3`) | `hidden` 字段多个数 → 多隐层路径 |
| 旧 pipeline.json（`control_mode: shadow`） | 不变，v3 模型在 shadow 模式下跑 |
| 旧 data_recorder JSONL 样本 | 缺 v3 字段 → 退化为 v2 特征训练 |
| 14 个旧场景 | 全部作为闭环评估的测试集 |

---

## 九、与 NOA 场景计划的协同

| NOA 场景计划 (Phase) | 学习闭环 v3 依赖 |
|----------------------|-----------------|
| perception↔sensor_model 对接 | ✅ v3 特征 F1（感知统计特征需要真实 sensor/lidar） |
| 前端 entities 消费 | 不直接依赖 |
| planning branch_select + merge | v3 的 road_curvature/tl_state 特征在分叉/汇入段更有价值 |
| json_to_xodr junction | RoadGeometry 提供 curvature/speed_limit → v3 特征 F2 |

> **建议顺序**：先修 perception↔sensor_model（NOA Phase 2）→ 再做 v3 Phase 1（特征升级），
> 否则 v3 的"感知统计特征"仍然是真值作弊，没有实际提升。
