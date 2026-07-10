# tools/train — 车端学习闭环 · 离线训练桥接 (Stage 1)

读取 `data_recorder_node` 采集的 JSONL 样本，训练单隐层 MLP（模仿学习：学习
`planning` 的目标速度），导出为 `inference_node` (`tiny_mlp.h`) 可加载的纯文本
权重格式。

完整设计见 [`docs/LEARNING_LOOP.md`](../../docs/LEARNING_LOOP.md)。

## 用法

```bash
# 用采集数据训练（data_recorder_node 的输出）
python3 tools/train/train.py \
    --input /tmp/flow_train_samples.jsonl \
    --output tools/train/model.txt

# 无数据时用合成数据生成演示模型
python3 tools/train/train.py --synthetic --output tools/train/model.txt
```

参数：`--hidden`（隐层宽度，默认 8）、`--epochs`（默认 300）、`--lr`（默认 0.05）。

## 依赖

仅 Python 标准库（手写反向传播），**无需 PyTorch / numpy**，保证闭环在任何环境
都能跑通。生产环境可替换为 PyTorch 训练 + 导出 ONNX，只要遵循同一数据 / 权重契约。

## 产物

- `model.txt` — `tiny_mlp.h` 权重格式（格式说明见 `docs/LEARNING_LOOP.md`）。
  仓库内置一份用合成数据训练的 `model.txt`，`inference_node` 开箱即可加载。
