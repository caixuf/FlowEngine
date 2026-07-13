#!/usr/bin/env python3
"""
train.py — 车端学习闭环 · Stage 1: 离线训练桥接

读取 data_recorder_node 采集的 JSONL 训练样本，训练一个单隐层 MLP（模仿学习：
学习 planning 输出的目标速度），并导出为 inference_node (tiny_mlp.h) 可加载的
纯文本权重格式。

刻意只用 Python 标准库实现（手写反向传播），保证"训练→部署"闭环在任何环境都能
跑通，无需安装 PyTorch/numpy。生产环境可把本文件替换为 PyTorch 训练 + 导出 ONNX，
只要最终产物遵循同一份数据契约 / 权重契约即可（见 README.md）。

数据契约（与 data_recorder_node / inference_node 一致）:
    features = [...]                                      # v1 4 维或 v2 16 维输入
    label    = planning_target_speed                        # 1 维输出

用法:
    # 用采集数据训练
    python3 tools/train/train.py --input /tmp/flow_train_samples.jsonl \
                                 --output tools/train/model.txt

    # 无采集数据时用合成数据生成一个演示模型
    python3 tools/train/train.py --synthetic --output tools/train/model.txt
"""

import argparse
import json
import math
import random
import sys

MAX_IN_DIM = 16
OUT_DIM = 1


def load_samples(path):
    feats, labels = [], []
    in_dim = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            x = obj.get("features")
            y = obj.get("label")
            if not isinstance(x, list) or y is None or not 0 < len(x) <= MAX_IN_DIM:
                continue
            if in_dim is None:
                in_dim = len(x)
            if len(x) != in_dim:
                continue
            feats.append([float(v) for v in x])
            labels.append([float(y)])
    return feats, labels


def make_synthetic(n=2000, seed=42):
    """合成一批 (特征, 标签) 样本：目标速度 ~ 关于当前速度/横向的平滑函数。"""
    rng = random.Random(seed)
    feats, labels = [], []
    for _ in range(n):
        v = rng.uniform(0.0, 20.0)
        y = rng.uniform(-3.0, 3.0)
        heading = rng.uniform(-0.3, 0.3)
        yaw = rng.uniform(-0.2, 0.2)
        # 模仿一个合理的驾驶策略：朝 15 m/s 巡航，横向偏大/转向大时减速
        target = 15.0 - 1.5 * abs(y) - 6.0 * abs(heading) + 0.1 * (v - 10.0)
        target = max(0.0, min(20.0, target)) + rng.gauss(0.0, 0.3)
        feats.append([v, y, heading, yaw])
        labels.append([max(0.0, min(20.0, target))])
    return feats, labels


def stats(cols):
    """按列计算 mean / std（std 下限 1e-6 防止除零）。"""
    n = len(cols)
    dim = len(cols[0])
    mean = [0.0] * dim
    for row in cols:
        for j in range(dim):
            mean[j] += row[j]
    mean = [m / n for m in mean]
    var = [0.0] * dim
    for row in cols:
        for j in range(dim):
            d = row[j] - mean[j]
            var[j] += d * d
    std = [max(1e-6, math.sqrt(v / n)) for v in var]
    return mean, std


def normalize(cols, mean, std):
    return [[(row[j] - mean[j]) / std[j] for j in range(len(row))] for row in cols]


def train(feats, labels, hidden=8, epochs=300, lr=0.05, seed=1):
    rng = random.Random(seed)
    n = len(feats)
    in_dim = len(feats[0])

    x_mean, x_std = stats(feats)
    y_mean, y_std = stats(labels)
    xn = normalize(feats, x_mean, x_std)
    yn = normalize(labels, y_mean, y_std)

    # Xavier 风格的小初始化
    def rand_mat(rows, ncol):
        scale = 1.0 / math.sqrt(ncol)
        return [[rng.uniform(-scale, scale) for _ in range(ncol)] for _ in range(rows)]

    W1 = rand_mat(hidden, in_dim)     # [hidden][in]
    b1 = [0.0] * hidden
    W2 = rand_mat(OUT_DIM, hidden)    # [out][hidden]
    b2 = [0.0] * OUT_DIM

    for ep in range(epochs):
        idx = list(range(n))
        rng.shuffle(idx)
        total_loss = 0.0
        for i in idx:
            x = xn[i]
            t = yn[i]
            # 前向
            h_pre = [b1[j] + sum(W1[j][k] * x[k] for k in range(in_dim))
                     for j in range(hidden)]
            h = [math.tanh(v) for v in h_pre]
            out = [b2[o] + sum(W2[o][j] * h[j] for j in range(hidden))
                   for o in range(OUT_DIM)]
            # 损失 (MSE)
            err = [out[o] - t[o] for o in range(OUT_DIM)]
            total_loss += sum(e * e for e in err)
            # 反向
            dout = [2.0 * e / OUT_DIM for e in err]
            dh = [0.0] * hidden
            for o in range(OUT_DIM):
                for j in range(hidden):
                    dh[j] += dout[o] * W2[o][j]
                    W2[o][j] -= lr * dout[o] * h[j]
                b2[o] -= lr * dout[o]
            for j in range(hidden):
                dpre = dh[j] * (1.0 - h[j] * h[j])  # tanh'
                for k in range(in_dim):
                    W1[j][k] -= lr * dpre * x[k]
                b1[j] -= lr * dpre
        if ep % 50 == 0 or ep == epochs - 1:
            print(f"  epoch {ep:4d}  mse(norm)={total_loss / n:.5f}", file=sys.stderr)

    return {
        "hidden": hidden,
        "in_dim": in_dim,
        "x_mean": x_mean, "x_std": x_std,
        "y_mean": y_mean, "y_std": y_std,
        "W1": W1, "b1": b1, "W2": W2, "b2": b2,
    }


def export(model, path):
    hidden = model["hidden"]
    in_dim = model.get("in_dim", len(model["x_mean"]))
    with open(path, "w", encoding="utf-8") as f:
        f.write("# flowengine-tinymlp v1\n")
        f.write(f"# features: {in_dim} dims\n")
        f.write("# output:   [target_speed]\n")
        f.write(f"in {in_dim}\n")
        f.write(f"hidden {hidden}\n")
        f.write(f"out {OUT_DIM}\n")
        f.write("norm_mean " + " ".join(f"{v:.6f}" for v in model["x_mean"]) + "\n")
        f.write("norm_scale " + " ".join(f"{v:.6f}" for v in model["x_std"]) + "\n")
        f.write("out_mean " + " ".join(f"{v:.6f}" for v in model["y_mean"]) + "\n")
        f.write("out_scale " + " ".join(f"{v:.6f}" for v in model["y_std"]) + "\n")
        f.write("w1 " + " ".join(f"{model['W1'][j][k]:.6f}"
                                 for j in range(hidden) for k in range(in_dim)) + "\n")
        f.write("b1 " + " ".join(f"{v:.6f}" for v in model["b1"]) + "\n")
        f.write("w2 " + " ".join(f"{model['W2'][o][j]:.6f}"
                                 for o in range(OUT_DIM) for j in range(hidden)) + "\n")
        f.write("b2 " + " ".join(f"{v:.6f}" for v in model["b2"]) + "\n")
    print(f"model exported → {path}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description="FlowEngine tiny-MLP trainer (imitation learning)")
    ap.add_argument("--input", default="/tmp/flow_train_samples.jsonl",
                    help="JSONL 样本文件 (data_recorder_node 输出)")
    ap.add_argument("--output", default="tools/train/model.txt",
                    help="导出的权重文件 (inference_node 加载)")
    ap.add_argument("--synthetic", action="store_true",
                    help="忽略 --input，使用合成数据训练演示模型")
    ap.add_argument("--hidden", type=int, default=8)
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--lr", type=float, default=0.05)
    args = ap.parse_args()

    if args.synthetic:
        print("using synthetic dataset", file=sys.stderr)
        feats, labels = make_synthetic()
    else:
        try:
            feats, labels = load_samples(args.input)
        except FileNotFoundError:
            print(f"error: input file not found: {args.input}\n"
                  f"       先运行 pipeline (含 data_recorder) 采集样本，"
                  f"或加 --synthetic 生成演示模型。", file=sys.stderr)
            return 1
        if len(feats) < 10:
            print(f"error: too few samples ({len(feats)}) in {args.input}; "
                  f"用 --synthetic 或采集更多数据。", file=sys.stderr)
            return 1

        in_dim = len(feats[0])
        print(f"training on {len(feats)} samples "
            f"(in={in_dim}, hidden={args.hidden}, out={OUT_DIM})", file=sys.stderr)
    model = train(feats, labels, hidden=args.hidden, epochs=args.epochs, lr=args.lr)
    export(model, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
