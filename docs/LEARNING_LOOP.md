# 车端学习闭环 (Vehicle-side Learning Loop)

> FlowEngine 不做训练框架，而是做「车端学习闭环」的中间件底座。
> 围绕现有 Pub/Sub 总线扩展一条 **数据采集 → 训练 → 推理** 的闭环，
> 把「训练」交给成熟框架，把「实时性 / 部署 / 安全兜底」交给 FlowEngine。

## 为什么不内置大模型训练框架

从零造训练框架是 PyTorch / JAX 量级的工作，与 FlowEngine「轻量级自动驾驶中间件」
的定位不匹配。更有价值的做法是：**把 FlowEngine 作为闭环的调度 / 通信 / 可视化底座**，
每个环节对应一门可落地的前沿技术，工程量可控且互相解耦。

## 闭环总览

```
                    ┌──────────────── 离线 ────────────────┐
 fusion/localization │                                       │
 planning/trajectory ┼─► data_recorder_node ─► JSONL 样本 ─► tools/train_e2e/train.py
        (teacher)     │        (Stage 0)                         (Stage 1)
                     │                                              │ 导出 model.txt
                     │                                              ▼
 fusion/localization ┼─────────────► inference_node ◄─── models/e2e_tiny/model.txt
        (实时)        │                (Stage 2)
                     │                    │ 影子模式发布
                     │                    ▼
                     └──────────► inference/trajectory ─► monitor / FlowBoard
```

安全永远由 `planning → control → safety_control` 兜底；`inference_node` 运行在
**影子模式 (shadow mode)**，只发布 `inference/trajectory` 供对比 / 监控，
**不**接入真实控制链路。即便模型是随机权重也不会影响车辆行为。

## 数据契约 (三方一致)

当前**默认使用 v3**（59 维特征、多隐层 MLP、5 维输出）。v1/v2 保持向后兼容。

### v3 特征（59 维，每帧）

由 `tools/train_e2e/feature_schema.py` 统一构建，包含四个部分：

| 组 | 维度 | 说明 |
|----|------|------|
| v2 基础 | 16 | ego_v, ego_y, ego_heading, ego_yaw_rate + 前方 2 个障碍物(x,y,vx,type,confidence) + control(brake,emergency_stop) |
| 感知统计 | 6 | lidar_point_count, lidar_front_density, lidar_clutter_ratio, perception_obj_count, perception_max_conf, perception_mean_conf |
| 场景上下文 | 7 | tl_state, tl_distance, road_curvature, road_speed_limit, lane_count, lane_width, ego_lane_offset |
| 障碍物全貌 | 30 | 类型分布(5)、距离分布(5)、速度分布(5)、前后左右间距(5)、后方来车(5)、左右车道占用(5) |

### v3 输出（5 维）

| 维度 | 含义 |
|------|------|
| throttle | 油门 (-1.0 ~ 1.0) |
| brake | 刹车 (0 ~ 1.0) |
| steer | 转向 (-1.0 ~ 1.0) |
| lane_change | 换道意图 (-1=左, 0=直, 1=右) |
| confidence | 置信度 (0 ~ 1.0) |

### 样本格式 (JSONL)

```json
{"t":1783660681184,"features_v3":[5.22,-1.77,-0.0008,0.0,...],"label":[0.5,0,0.1,0,0.9],"ego":{"x":0.47,"y":-1.77}}
```

特征名集中定义在 `tools/train_e2e/feature_schema.py` 的 `FEATURE_NAMES_V3`，exporter、trainer、evaluator、sidecar 都从这里读取，避免同一套拼接逻辑散在多个脚本里。

## 模型权重契约 (`tiny_mlp.h` 纯文本格式)

**v3: 多隐层 MLP**，`tanh` 激活，纯文本存储，Python 训练侧与 C 推理侧读写同一格式：

```
# flowengine-tinymlp v3
in 295              # v3: 59 维/帧 × 5 帧时序窗口
hidden 64 32        # 多隐层: 隐层 0=64, 隐层 1=32
out 5               # throttle/brake/steer/lane_change/confidence
norm_mean   <IN floats>     # 输入标准化: xn = (x - mean) / scale
norm_scale  <IN floats>
out_mean    <OUT floats>    # 输出反标准化: y = yn * scale + mean
out_scale   <OUT floats>
w1 <HID0*IN floats>         # 隐层 0 [hid0][in]
b1 <HID0 floats>
w2 <HID1*HID0 floats>       # 隐层 1 [hid1][hid0]
b2 <HID1 floats>
w_out <OUT*HID1 floats>     # 输出层 [out][hid1]
b_out <OUT floats>
```

**v1/v2 向后兼容**（单隐层，输出层用 w2/b2 标签）：

```
# flowengine-tinymlp v1
in 4
hidden 8
out 1
norm_mean   <IN floats>
norm_scale  <IN floats>
out_mean    <OUT floats>
out_scale   <OUT floats>
w1 <8*4 floats>
b1 <8 floats>
w2 <1*8 floats>    # hidden_count==1 时 w2 读作输出层
b2 <1 floats>
```

前向计算：`y = out_denorm( W_out · tanh(... tanh(W1 · norm(x) + b1) + b2 ...) + b_out )`

最多支持 4 层隐层（`TINY_MLP_MAX_HID_LAYERS=4`），隐层维度通过 `hidden` 行的多个值表达。

## 组件

| 组件 | 位置 | 阶段 | 作用 |
|------|------|------|------|
| `data_recorder_node` | `modules/adas_nodes/data_recorder_node.c` | 0 | 订阅 topic，按频率落盘 JSONL 训练样本 |
| `train.py`           | `tools/train_e2e/train.py` | 1 | 读样本训练 tiny-MLP 或 PyTorch 模型，导出 `model.txt` / `model.pt` |
| `inference_node`     | `modules/adas_nodes/inference_node.c` | 2 | 加载模型，影子模式发布 `inference/trajectory`；订阅 `model_ota/active` 支持 OTA 热重载 |
| `tiny_mlp.h`         | `modules/adas_nodes/tiny_mlp.h` | 2/3 | 零依赖推理 + SGD 微调内核（`tiny_mlp_forward` / `tiny_mlp_sgd_step` / `tiny_mlp_save`）|
| `learner_node`       | `modules/adas_nodes/learner_node.c` | 3 | 车端增量 SGD 微调，资源受限调度，发布 `learner/status` |
| `model_ota_node`     | `modules/adas_nodes/model_ota_node.c` | 4 | 模型版本注册表、热加载、回滚、A-B 对比测试 |

## 模型位置约定

日常只需要先看一个入口：

```bash
python3 tools/modelctl.py list
```

FlowEngine 约定只有两类模型位置：

| 位置 | 用途 |
|------|------|
| `models/e2e_tiny/model.txt` | 当前 C runtime 模型；`demo.sh` 里的 `inference_node` 直接加载它 |
| `models/<name>/` | 训练产物目录；包含 `manifest.json` 和 `model.txt` 或 `model.pt` |

`models/<name>/model.txt` 是 tiny-MLP artifact，可以提升为 runtime 模型：

```bash
python3 tools/modelctl.py promote models/e2e_tiny_v001
```

`models/<name>/model.pt` 是 PyTorch artifact，只用于 `eval_model.py` 和
`torch_sidecar.py`，不会自动接管 C runtime。

## 快速上手

```bash
# 1. 构建（含节点插件）
bash scripts/demo.sh --no-browser 12          # 采集 + 推理随 pipeline 一起跑

# 2. 用采集到的样本一键训练模型（默认 PyTorch backend）
python3 tools/train_demo_model.py --backend torch --name e2e_torch_v001

# 3. 基于已有 PyTorch artifact 继续训练，输出一个新版本
python3 tools/train_demo_model.py \
  --backend torch \
  --name e2e_torch_v002 \
  --init-from models/e2e_torch_v001

# 4. 查看当前 runtime 模型和训练产物
python3 tools/modelctl.py list
```

如果想让 C runtime 直接加载新模型，训练 tiny backend 并 promote：

```bash
python3 tools/train_demo_model.py --backend tiny --name e2e_tiny_v001
python3 tools/modelctl.py promote models/e2e_tiny_v001
bash scripts/demo.sh --no-browser 12
```

无采集数据时，可先用合成数据生成一个演示模型：

```bash
python3 tools/train_e2e/train.py --synthetic --output models/e2e_tiny_demo
python3 tools/modelctl.py promote models/e2e_tiny_demo
```

仓库已内置一个用合成数据训练好的 `models/e2e_tiny/model.txt`，因此 `inference_node`
开箱即可加载真实模型；若文件缺失，节点会回退到可解释的启发式策略（不报错）。

## E2E Training Bridge v0.1（无 ONNX 依赖）

当前仓库提供一条不依赖 ONNX Runtime 的端到端训练框架对接骨架。它复用
`data_recorder_node` 的 JSONL 样本：tiny-MLP 后端使用 v1 4 维特征，PyTorch 后端
优先使用 v2 场景特征，把「采集 → 数据集 → 训练产物 → 离线评估 → shadow 推理」
的工程契约先跑通。

```bash
# 1. 采集 recorder 样本
bash scripts/demo.sh --no-browser 30

# 2. 推荐入口：自动导出 dataset、训练、评估、写入 models/<name>/
python3 tools/train_demo_model.py --backend torch --name e2e_torch_v001
```

如果要拆开调试，等价底层命令是：

```bash

# 导出版本化数据集目录
python3 tools/dataset/export_e2e_dataset.py \
  --input /tmp/flow_train_samples.jsonl \
  --output datasets/demo_e2e \
  --scenario pedestrian_crossing

# 训练一个 FlowEngine artifact（tiny-MLP）
python3 tools/train_e2e/train.py \
  --dataset datasets/demo_e2e \
  --output models/e2e_tiny_v001

# 离线评估 teacher imitation 误差
python3 tools/train_e2e/temporal_train.py \
  --dataset datasets/demo_e2e \
  --output models/e2e_tiny_v001 \
  --eval

# 查看 artifact 的 manifest / 训练参数 / metrics
python3 tools/modelctl.py inspect models/e2e_tiny_v001

# 让 inference_node 加载新产物的 model.txt 继续 shadow 运行
python3 tools/modelctl.py promote models/e2e_tiny_v001
bash scripts/demo.sh --no-browser 30
```

产物目录结构：

```text
models/e2e_tiny_v001/
  model.txt              # inference_node 可直接加载
  manifest.json          # 模型格式、输入输出 schema、训练参数、数据集摘要
  dataset_metadata.json  # 数据集元信息快照
  metrics.json           # temporal_train.py --eval 产出（可选）
```

这不是完整的端到端自动驾驶训练平台，而是训练框架接入点：后续把
`tools/train_e2e/train.py` 的 tiny-MLP backend 替换成 PyTorch/ONNX 导出时，数据集目录、
`manifest.json`、shadow mode 和评估入口可以保持稳定。

`export_e2e_dataset.py` 会自动识别 recorder v2 样本并导出
`flowengine.e2e_dataset.v2`，metadata 中的 `feature_names` 会随 artifact 写入
checkpoint。`torch_sidecar.py` 运行时根据 checkpoint 的 `feature_names` 从
`/tmp/flow_topology.json` 抽取同一套特征，而不是硬编码 4 维输入。

如果本机安装了 PyTorch，可以直接使用可选的 PyTorch backend 训练 `model.pt`：

```bash
python3 tools/train_e2e/torch_train.py \
  --dataset datasets/demo_e2e \
  --output models/e2e_torch_v001

# 用 modelctl 查看 metrics.json
python3 tools/modelctl.py inspect models/e2e_torch_v001
```

`torch_train.py` 是可选入口；未安装 PyTorch 时，零依赖的 `tools/train_e2e/train.py`
仍然可用。当前 C 推理节点只直接加载 `model.txt`，因此 PyTorch artifact 先用于训练框架
接入和离线评估；后续再接 Python sidecar / ONNX Runtime / TensorRT 推理后端。

如果暂时不想安装 PyTorch，也可以用零依赖 tiny-MLP sidecar 先验证运行时文件桥接：

```bash
python3 tools/train_e2e/tiny_sidecar.py \
  --model models/e2e_tiny_v001 \
  --state-file /tmp/flow_topology.json \
  --output /tmp/flow_tiny_inference.json
```

也可以先用文件桥接 sidecar 把 PyTorch artifact 放到运行时 shadow 链路旁边：

```bash
# 终端 1：运行 FlowEngine，持续写 /tmp/flow_topology.json
bash scripts/demo.sh --no-browser 30

# 终端 2：读取 state JSON，写出 PyTorch shadow 推理结果
python3 tools/train_e2e/torch_sidecar.py \
  --model models/e2e_torch_v001 \
  --state-file /tmp/flow_topology.json \
  --output /tmp/flow_torch_inference.json
```

`torch_sidecar.py` 当前不向 C transport 发布 topic，而是把一帧
`inference/trajectory` 形状的 JSON 原子写到文件；这样可以先验证 PyTorch runtime、
模型 artifact、实时输入特征抽取和 shadow 输出契约，再决定是否升级为 IPC/HTTP/topic
发布节点。

## 影子模式对比

`inference_node` 同时订阅 `planning/trajectory`，在日志与 `inference/trajectory`
JSON 中输出 `shadow_delta = 推理目标速度 − planning 目标速度`，用于评估模型与
现有规划器的差异，是安全灰度上车前的关键手段。

## Stage 3: 车端增量微调 (learner_node)

`learner_node` 在 pipeline 运行过程中持续订阅 teacher 信号（`planning/trajectory`），
对当前 tiny-MLP 做在线 SGD 更新，实现"边跑边学"的持续学习闭环。

### 设计要点

- **资源受限调度**: 采样以 20 Hz 积累（不阻塞控制链路），训练以 `train_hz`（默认 0.5 Hz）
  低频运行，每次从 ring buffer 中随机抽取 mini-batch，避免抢占算力。
- **防灾难性遗忘**: 默认 `full_finetune=0` — 仅更新顶层 W2/b2，保留 backbone；
  通过 `full_finetune=1` 参数开启全参数微调。
- **持久化**: 每 `save_interval`（默认 50）步将更新权重原子写入 `save_path`，
  不自动替换 runtime 模型，需显式 `modelctl.py ota push` 激活。
- **发布 `learner/status`**: 每训练步广播 `{step, loss, lr, buf_count, ...}`。

### 快速上手

```bash
# pipeline 跑起来后，learner_node 自动开始学习并保存到 /tmp/flow_learner_model.txt
bash scripts/demo.sh --no-browser 60

# 查看训练状态（从 flow_topology.json 或直接订阅 learner/status topic）
# 学习收敛后，通过 OTA 激活新模型：
python3 tools/modelctl.py ota push /tmp/flow_learner_model.txt --id learner_v001
```

### 配置参数 (pipeline.json params)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `train_hz` | 0.5 | 训练频率（Hz） |
| `lr` | 0.001 | SGD 学习率 |
| `batch_size` | 32 | mini-batch 大小 |
| `buffer_size` | 512 | 样本 ring buffer 容量 |
| `full_finetune` | 0 | 0=仅顶层, 1=全参数 |
| `save_interval` | 50 | 每 N 步保存一次 |
| `model_path` | `tools/train/model.txt` | 初始模型路径 |
| `save_path` | `/tmp/flow_learner_model.txt` | 更新权重保存路径 |

## Stage 4: 模型 OTA + 版本管理 (model_ota_node)

`model_ota_node` 管理模型版本注册表，支持热加载、回滚和 A-B 对比推理。
通过 Transport 总线（`model_ota/active` topic）向 `inference_node` 发送热重载信号。

### 架构

```
modelctl.py ota push   ─► /tmp/flow_ota_cmd.json ─┐
model_ota/cmd topic    ─────────────────────────────┤
                                                    ▼
                                        model_ota_node
                                         │   │   │
                             copy+rename  │   │  A-B 影子推理
                                         ▼   ▼   ▼
                               runtime model.txt   model_ota/ab_result
                                         │
                               model_ota/active (reload signal)
                                         │
                                    inference_node (hot-reload)
```

### 支持的操作

| 操作 | CLI | Topic 命令 |
|------|-----|-----------|
| 激活新版本 | `modelctl.py ota push <path>` | `{"cmd":"load","id":"v002","path":"..."}` |
| 回滚 | `modelctl.py ota rollback` | `{"cmd":"rollback"}` |
| A-B 测试 | `modelctl.py ota ab-test --model-b <path>` | `{"cmd":"ab_test","enable":true,...}` |
| 查看状态 | `modelctl.py ota status` | `{"cmd":"status"}` |

### 典型 OTA 工作流

```bash
# 1. 采集并训练新版本
bash scripts/demo.sh --no-browser 30
python3 tools/train_demo_model.py --backend tiny --name learner_v001

# 2. OTA 激活（无需重启 pipeline）
python3 tools/modelctl.py ota push models/learner_v001/model.txt --id learner_v001

# 3. inference_node 收到 model_ota/active 信号后热重载新权重，日志：
#    [INFO] [inference] OTA hot-reload #1 from tools/train/model.txt (in=4 hid=8 out=1)

# 4. 开启 A-B 对比，同时观察新旧模型差异
python3 tools/modelctl.py ota ab-test --model-b models/learner_v001/model.txt --ratio 0.5

# 5. 如果新模型表现不佳，一键回滚
python3 tools/modelctl.py ota rollback

# 6. 查看版本注册表和 A-B 统计
python3 tools/modelctl.py ota status
```

### 版本注册表

自动维护 `models/registry.json`，记录所有激活历史：

```json
{
  "schema": "flowengine-ota-registry v1",
  "current_id": "learner_v001",
  "previous_id": "initial",
  "ab_test": {"enabled": false, "ratio": 0.5},
  "versions": [
    {"id": "initial",      "path": "tools/train/model.txt", "active": false},
    {"id": "learner_v001", "path": "models/learner_v001/model.txt", "active": true}
  ]
}
```

## 后续路线 (对应前沿技术)

| 阶段 | 方向 | 技术点 | 状态 |
|------|------|--------|------|
| 3 | `learner_node` 车端增量微调 | 持续学习 / on-device SGD / 资源受限调度 | ✅ 已实现 |
| 4 | 模型 OTA + 版本管理 | Discovery + Transport 灰度发布 / 回滚 / A-B 对比 | ✅ 已实现 |
| 5 | **端到端 v3** | 59 维特征 / 多隐层 MLP / 5 维输出 / 时序窗口 / 感知统计 / 场景上下文 | ✅ 已实现 |
| — | 推理内核升级 | 把 `tiny_mlp_forward()` 替换为 ONNX Runtime / TensorRT，契约不变 | 待实现 |

> v3 已全面上线，默认使用 59 维特征、多隐层 MLP、5 维输出（throttle/brake/steer/lane_change/confidence）。

## 生产化提示

- `train.py` 用纯 Python 手写反向传播，目的是让「训练→部署」闭环**在任何环境都能跑通**。
  生产环境建议改用 PyTorch 训练并导出 ONNX，只要最终产物遵循同一数据 / 权重契约即可。
- 车端算力有限，建议从小模型 / 端到端小网络起步；把工程闭环跑通比追求模型规模更有价值。
- 学习模型的任何输出，都必须经过 `safety_control_node` 的安全校验后才允许下发执行。
