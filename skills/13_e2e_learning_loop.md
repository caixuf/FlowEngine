# 13 — E2E Learning Loop 训练闭环

FlowEngine 的学习闭环把 demo pipeline 里的 teacher 行为采成 JSONL，导出成版本化 dataset，再训练 tiny 或 PyTorch artifact，最后通过 evaluator、modelctl 或 sidecar 接回运行时。

## 什么时候用

当你要做这些事情时使用本 skill：

- 从 `scripts/demo.sh` 运行结果生成训练样本
- 把 `/tmp/flow_train_samples.jsonl` 转成 dataset
- 训练 `models/<name>` 下的新 artifact
- 基于已有 PyTorch artifact 继续训练
- 评估模型效果，决定是否 promote 到 C runtime
- 排查模型、dataset、runtime 路径混乱

## 核心路径

| 路径 | 作用 |
|------|------|
| `/tmp/flow_train_samples.jsonl` | `data_recorder_node` 写出的原始训练样本 |
| `datasets/<name>/` | `export_e2e_dataset.py` 导出的版本化 dataset |
| `models/<name>/` | 训练产物目录，包含 `manifest.json` 和模型文件 |
| `tools/train/model.txt` | C `inference_node` 直接加载的 tiny-MLP runtime 模型 |
| `/tmp/flow_topology.json` | demo runtime 状态，sidecar 从这里读 shadow inference 输入 |
| `/tmp/flow_torch_inference.json` | PyTorch sidecar 输出 |

## Canonical 入口（其他脚本禁止并存）

| 用途 | canonical 路径 | 替代关系 |
|------|----------------|----------|
| 一站式训练（采+训） | `tools/train_demo_model.py` | 顶层入口，可调度 train_e2e/ 下的 backend |
| Tiny-MLP 训练 | `tools/train_e2e/train.py` | 唯一 tiny-MLP 训练入口 |
| PyTorch artifact | `tools/train_e2e/torch_train.py` | 唯一 PyTorch artifact 训练入口 |
| 时序模型 | `tools/train_e2e/temporal_train.py` | 唯一时序模型训练入口 |
| Sidecar 入口 | `tools/train_e2e/{tiny_sidecar,torch_sidecar}.py` | 唯一旁路推理入口 |
| C 端推理 | `modules/adas_nodes/inference_node.cpp` | C runtime 推理，**唯一**实现 |

> 任何新训练入口必须在 `tools/train_e2e/` 或顶层 `tools/train_demo_model.py`。
> ❌ `tools/train/train.py`（sklearn 旧实现）已 deprecated：被 `tools/train_e2e/train.py` 取代
>   详见 [CLAUDE.md 重构铁律](../../CLAUDE.md)「重构/替代 → 同一 commit 删旧」

## 一键训练

```bash
# 采集 30 秒 demo 样本，并训练 PyTorch artifact
python3 tools/train_demo_model.py \
  --run-demo 30 \
  --backend torch \
  --name e2e_torch_v001

# 不重新跑 demo，直接使用现有 /tmp/flow_train_samples.jsonl
python3 tools/train_demo_model.py \
  --backend torch \
  --name e2e_torch_v001
```

训练完成后查看产物：

```bash
python3 tools/modelctl.py list
```

## 基于已有模型继续训练

这是项目当前最实用的“预训练模型”路线：先沉淀一个自己的 baseline artifact，后续新数据都从它初始化，而不是随机起步。

```bash
python3 tools/train_demo_model.py \
  --backend torch \
  --name e2e_torch_v002 \
  --init-from models/e2e_torch_v001
```

`--init-from` 只支持 PyTorch artifact。训练器会校验：

- `backend == pytorch`
- `feature_names` 与目标 dataset 一致
- `label_names` 与目标 dataset 一致
- `hidden` 与当前 `--hidden` 一致

校验失败时不要硬转；应重新导出匹配 schema 的 dataset，或用相同 `--hidden` 训练。

## Tiny Runtime 路径

C runtime 当前只直接加载 tiny-MLP `model.txt`。如果要把模型真正 promote 给 `inference_node`：

```bash
python3 tools/train_demo_model.py \
  --backend tiny \
  --name e2e_tiny_v001

python3 tools/modelctl.py promote models/e2e_tiny_v001
```

注意：tiny backend 当前适合轻量 runtime 验证。复杂策略先用 PyTorch artifact 和 sidecar shadow 跑旁路评估。

## PyTorch Sidecar Shadow 路径

PyTorch artifact 不直接进 C runtime，而是通过 sidecar 读取 runtime 状态，输出 shadow inference：

```bash
# 终端 1：跑 demo
bash scripts/demo.sh --no-browser 30

# 终端 2：旁路推理
python3 tools/train_e2e/torch_sidecar.py \
  --model models/e2e_torch_v001 \
  --state-file /tmp/flow_topology.json \
  --output /tmp/flow_torch_inference.json
```

sidecar 使用 `tools/train_e2e/feature_schema.py` 构造特征。新增特征时优先改这个文件，避免 exporter、trainer、sidecar 各自复制逻辑。

## 评估与验证

```bash
# 对 dataset 评估 artifact
python3 tools/eval_model.py \
  --model models/e2e_torch_v001 \
  --dataset datasets/e2e_torch_v001 \
  --output models/e2e_torch_v001/metrics.json

# 训练工具测试
python3 tests/test_e2e_training_tools.py
ctest --test-dir build -R e2e_training_tools --output-on-failure
```

模型是否“更好”不能只看训练 loss。至少比较：

- `metrics.json` 里的 `mae_target_speed`
- demo evaluator 是否 PASS
- sidecar shadow 输出是否稳定
- promote 后是否没有碰撞、停滞、偏航抖动等回归

## 常见坑

| 现象 | 原因 | 处理 |
|------|------|------|
| `--init-from` 报 feature mismatch | 旧模型和新 dataset 的特征 schema 不一致 | 用同一版 `feature_schema.py` 重新导出 dataset / 重新训 baseline |
| tiny promote 失败 | artifact 不是 `backend=tiny_mlp` | PyTorch artifact 只能 eval/sidecar，不能直接 promote |
| C runtime 没变化 | 只训练了 `models/<name>`，没有 promote | 运行 `python3 tools/modelctl.py promote models/<tiny_name>` |
| 训练 loss 下降但 demo 变差 | imitation loss 不等于闭环驾驶质量 | 跑 `tools/demo_evaluator.py` 做闭环验证 |
| 路径混乱 | runtime 模型和训练 artifact 混用 | 用 `python3 tools/modelctl.py list` 查权威位置 |

## 推荐迭代节奏

1. `bash scripts/demo.sh --no-browser 30` 采样。
2. `python3 tools/train_demo_model.py --backend torch --name <new> --init-from models/<baseline>` 续训。
3. `python3 tools/eval_model.py ...` 看离线指标。
4. 用 `torch_sidecar.py` 做 shadow runtime 观察。
5. 只有 tiny artifact 才 promote 到 `tools/train/model.txt`。
6. 每次改 recorder/schema/training 工具后跑 `python3 tests/test_e2e_training_tools.py` 和 CTest。
