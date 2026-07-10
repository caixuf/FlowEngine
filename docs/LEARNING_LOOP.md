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
 planning/trajectory ┼─► data_recorder_node ─► JSONL 样本 ─► tools/train/train.py
        (teacher)     │        (Stage 0)                        (Stage 1)
                     │                                              │ 导出 model.txt
                     │                                              ▼
 fusion/localization ┼─────────────► inference_node ◄─── tools/train/model.txt
        (实时)        │                (Stage 2)
                     │                    │ 影子模式发布
                     │                    ▼
                     └──────────► inference/trajectory ─► monitor / FlowBoard
```

安全永远由 `planning → control → safety_control` 兜底；`inference_node` 运行在
**影子模式 (shadow mode)**，只发布 `inference/trajectory` 供对比 / 监控，
**不**接入真实控制链路。即便模型是随机权重也不会影响车辆行为。

## 数据契约 (三方一致)

`data_recorder_node`、`tools/train/train.py`、`inference_node` 共享同一份契约：

| 项 | 定义 |
|----|------|
| 特征 `features` | `[ego_v, ego_y, ego_heading, ego_yaw_rate]`（4 维，来自 `fusion/localization`）|
| 标签 `label`    | `planning_target_speed`（1 维，模仿 planning 输出的目标速度）|

样本文件 (JSONL，每行一个 JSON)：

```json
{"t":1783660681184,"features":[5.22,-1.77,-0.0008,0.0],"label":5.2,"ego":{"x":0.47,"y":-1.77}}
```

## 模型权重契约 (`tiny_mlp.h` 纯文本格式)

单隐层 MLP，`tanh` 激活，纯文本存储，Python 训练侧与 C 推理侧读写同一格式：

```
# flowengine-tinymlp v1
in 4
hidden 8
out 1
norm_mean   <IN floats>     # 输入标准化: xn = (x - mean) / scale
norm_scale  <IN floats>
out_mean    <OUT floats>    # 输出反标准化: y = yn * scale + mean
out_scale   <OUT floats>
w1 <HID*IN floats>          # 行主序 [HID][IN]
b1 <HID floats>
w2 <OUT*HID floats>         # 行主序 [OUT][HID]
b2 <OUT floats>
```

前向计算：`y = out_denorm( W2 · tanh(W1 · norm(x) + b1) + b2 )`

## 组件

| 组件 | 位置 | 阶段 | 作用 |
|------|------|------|------|
| `data_recorder_node` | `modules/adas_nodes/data_recorder_node.c` | 0 | 订阅 topic，按频率落盘 JSONL 训练样本 |
| `train.py`           | `tools/train/train.py` | 1 | 读样本训练 tiny-MLP，导出 `model.txt`（纯 Python，无需 PyTorch/numpy）|
| `inference_node`     | `modules/adas_nodes/inference_node.c` | 2 | 加载模型，影子模式发布 `inference/trajectory` |
| `tiny_mlp.h`         | `modules/adas_nodes/tiny_mlp.h` | 2 | 零依赖推理内核（后续可替换为 ONNX Runtime / TensorRT）|

## 快速上手

```bash
# 1. 构建（含节点插件）
bash scripts/demo.sh --no-browser 12          # 采集 + 推理随 pipeline 一起跑

# 2. 用采集到的样本训练模型
python3 tools/train/train.py \
    --input /tmp/flow_train_samples.jsonl \
    --output tools/train/model.txt

# 3. 再次运行 pipeline，inference_node 会自动加载新模型
bash scripts/demo.sh --no-browser 12
```

无采集数据时，可先用合成数据生成一个演示模型：

```bash
python3 tools/train/train.py --synthetic --output tools/train/model.txt
```

仓库已内置一个用合成数据训练好的 `tools/train/model.txt`，因此 `inference_node`
开箱即可加载真实模型；若文件缺失，节点会回退到可解释的启发式策略（不报错）。

## 影子模式对比

`inference_node` 同时订阅 `planning/trajectory`，在日志与 `inference/trajectory`
JSON 中输出 `shadow_delta = 推理目标速度 − planning 目标速度`，用于评估模型与
现有规划器的差异，是安全灰度上车前的关键手段。

## 后续路线 (对应前沿技术)

| 阶段 | 方向 | 技术点 |
|------|------|--------|
| 3 | `learner_node` 车端增量微调 | 持续学习 / on-device fine-tuning / 资源受限调度 |
| 4 | 模型 OTA + 版本管理 | 复用 Discovery + Transport 做灰度发布 / 回滚 / A-B 对比 |
| — | 推理内核升级 | 把 `tiny_mlp_forward()` 替换为 ONNX Runtime / TensorRT，契约不变 |

## 生产化提示

- `train.py` 用纯 Python 手写反向传播，目的是让「训练→部署」闭环**在任何环境都能跑通**。
  生产环境建议改用 PyTorch 训练并导出 ONNX，只要最终产物遵循同一数据 / 权重契约即可。
- 车端算力有限，建议从小模型 / 端到端小网络起步；把工程闭环跑通比追求模型规模更有价值。
- 学习模型的任何输出，都必须经过 `safety_control_node` 的安全校验后才允许下发执行。
