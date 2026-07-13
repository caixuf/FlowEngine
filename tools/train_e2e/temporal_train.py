#!/usr/bin/env python3
"""
FlowEngine 时序端到端训练器

从 data_recorder 采集的 JSONL 加载 v2 样本（16维帧特征），堆叠为 5帧×16维=80维
时序窗口，训练 MLP 预测 planning.target_speed + control(thr/brk/steer)，
导出为 inference_node (tiny_mlp.h) 可加载的 model.txt。

用法:
  # 采集数据（所有场景各跑一遍）
  python3 tools/scenario_regression.py --suite scenarios/suite.json --record

  # 训练
  python3 tools/train_e2e/temporal_train.py \\
      --input /tmp/flow_train_samples.jsonl \\
      --output tools/train/model.txt \\
      --epochs 300

  # 部署（pipeline.json 改 inference control_mode = direct_ctrl）
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

WINDOW = 5       # 时序滑窗帧数
V2_DIM = 16      # 每帧特征维度
IN_DIM = WINDOW * V2_DIM  # 80
OUT_DIM = 5      # throttle, brake, steer, lane_change, confidence

# ── 数据加载 ──────────────────────────────────────────────────

def load_jsonl(path: str | Path) -> list[dict]:
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                samples.append(json.loads(line))
    return samples


def build_windows(samples: list[dict]) -> tuple[list[list[float]], list[list[float]]]:
    """
    将样本流堆叠为 5 帧时序窗口。
    输入: X = [frame[t-4], frame[t-3], ..., frame[t]] 共 80 维
    输出: Y = [throttle, brake, steer, lane_change, confidence]
    """
    X, Y = [], []
    for i in range(WINDOW - 1, len(samples)):
        window = samples[i - WINDOW + 1 : i + 1]

        # 构建 80 维输入
        x = []
        for s in window:
            fv2 = s.get("features_v2", s.get("features", [0]*V2_DIM))
            # 补齐到 V2_DIM
            while len(fv2) < V2_DIM:
                fv2.append(0.0)
            x.extend(fv2[:V2_DIM])

        # 目标样本 = 窗口最后一帧
        target = samples[i]
        ctrl = target.get("control", {})
        throttle = float(ctrl.get("throttle", 0.0))
        brake    = float(ctrl.get("brake", 0.0))
        steer    = float(ctrl.get("steering", 0.0))
        lc       = 1.0 if abs(steer) > 0.12 else 0.0  # 大转向角视为变道意图
        conf     = 1.0  # expert 演示置信度=1

        # 过滤停滞数据（速度 < 0.5 的帧不训练）
        ego_v = float(target.get("ego", {}).get("v", 0.0))
        if ego_v < 0.5:
            continue

        X.append(x)
        Y.append([throttle, brake, steer, lc, conf])

    return X, Y


# ── 标准化 ──────────────────────────────────────────────────

def column_stats(rows: list[list[float]]) -> tuple[list[float], list[float]]:
    dim = len(rows[0])
    mean = [sum(row[i] for row in rows) / len(rows) for i in range(dim)]
    scale = []
    for i in range(dim):
        var = sum((row[i] - mean[i]) ** 2 for row in rows) / len(rows)
        scale.append(max(var ** 0.5, 1e-6))
    return mean, scale


def normalize(rows: list[list[float]], mean: list[float], scale: list[float]) -> list[list[float]]:
    return [[(row[i] - mean[i]) / scale[i] for i in range(len(row))] for row in rows]


def denormalize(rows: list[list[float]], mean: list[float], scale: list[float]) -> list[list[float]]:
    return [[row[i] * scale[i] + mean[i] for i in range(len(row))] for row in rows]


# ── MLP（纯 Python/NumPy 无依赖实现，和 tiny_mlp.h 一致）──

class TinyMLP:
    """单隐层 MLP: y = out_denorm(W2 * tanh(W1 * norm(x) + b1) + b2)"""

    def __init__(self, in_dim: int, hidden: int, out_dim: int):
        self.in_dim = in_dim
        self.hid_dim = hidden
        self.out_dim = out_dim
        # Xavier 初始化
        self.w1 = [[random.gauss(0, math.sqrt(2.0 / in_dim)) for _ in range(in_dim)] for _ in range(hidden)]
        self.b1 = [0.0] * hidden
        self.w2 = [[random.gauss(0, math.sqrt(2.0 / hidden)) for _ in range(hidden)] for _ in range(out_dim)]
        self.b2 = [0.0] * out_dim

    def forward(self, x: list[float]) -> list[float]:
        h = [sum(w * x[i] for i, w in enumerate(w1_row)) + b for w1_row, b in zip(self.w1, self.b1)]
        h = [math.tanh(v) for v in h]
        y = [sum(w * h[j] for j, w in enumerate(w2_row)) + b for w2_row, b in zip(self.w2, self.b2)]
        return y

    def save(self, path: str | Path, x_mean: list[float], x_scale: list[float],
             y_mean: list[float], y_scale: list[float]) -> None:
        with open(path, "w") as f:
            f.write("# flowengine-tinymlp v2\n")
            f.write(f"# features: 80-dim temporal (5x16) — ego + front obstacles + control\n")
            f.write(f"# output:   [throttle, brake, steer, lane_change, confidence]\n")
            f.write(f"in {self.in_dim}\n")
            f.write(f"hidden {self.hid_dim}\n")
            f.write(f"out {self.out_dim}\n")

            def write_floats(label: str, vals: list[float], limit: int):
                f.write(f"{label} ")
                f.write(" ".join(f"{v:.8f}" for v in vals[:limit]))
                f.write("\n")

            write_floats("norm_mean", x_mean, self.in_dim)
            write_floats("norm_scale", x_scale, self.in_dim)
            write_floats("out_mean", y_mean, self.out_dim)
            write_floats("out_scale", y_scale, self.out_dim)

            w1_flat = [self.w1[i][j] for i in range(self.hid_dim) for j in range(self.in_dim)]
            write_floats("w1", w1_flat, self.hid_dim * self.in_dim)
            write_floats("b1", self.b1, self.hid_dim)
            w2_flat = [self.w2[i][j] for i in range(self.out_dim) for j in range(self.hid_dim)]
            write_floats("w2", w2_flat, self.out_dim * self.hid_dim)
            write_floats("b2", self.b2, self.out_dim)


# ── 训练 ──────────────────────────────────────────────────

def mse_loss(pred: list[float], target: list[float]) -> float:
    return sum((p - t) ** 2 for p, t in zip(pred, target)) / len(pred)


def train(model: TinyMLP, X: list[list[float]], Y: list[list[float]],
          epochs: int, lr: float) -> list[float]:
    """手写 BP 训练"""
    n = len(X)
    losses = []
    for epoch in range(epochs):
        total_loss = 0.0
        # SGD
        for si in range(n):
            x = X[si]
            y_true = Y[si]

            # Forward
            # h = tanh(W1 @ x + b1)
            h_pre = [sum(model.w1[i][j] * x[j] for j in range(model.in_dim)) + model.b1[i]
                     for i in range(model.hid_dim)]
            h = [math.tanh(v) for v in h_pre]
            # y = W2 @ h + b2
            y_pred = [sum(model.w2[o][i] * h[i] for i in range(model.hid_dim)) + model.b2[o]
                      for o in range(model.out_dim)]

            loss = mse_loss(y_pred, y_true)
            total_loss += loss

            # Backward: dL/dy
            dy = [(y_pred[o] - y_true[o]) * 2.0 / model.out_dim for o in range(model.out_dim)]

            # dL/dW2, dL/db2
            dw2 = [[dy[o] * h[i] for i in range(model.hid_dim)] for o in range(model.out_dim)]
            db2 = [dy[o] for o in range(model.out_dim)]

            # dL/dh = W2^T @ dy
            dh = [sum(model.w2[o][i] * dy[o] for o in range(model.out_dim)) for i in range(model.hid_dim)]

            # dL/dh_pre = dL/dh * tanh'(h_pre) = dL/dh * (1 - tanh²(h_pre))
            dh_pre = [dh[i] * (1.0 - h[i] * h[i]) for i in range(model.hid_dim)]

            # dL/dW1, dL/db1
            dw1 = [[dh_pre[i] * x[j] for j in range(model.in_dim)] for i in range(model.hid_dim)]
            db1 = [dh_pre[i] for i in range(model.hid_dim)]

            # SGD update
            for i in range(model.hid_dim):
                for j in range(model.in_dim):
                    model.w1[i][j] -= lr * dw1[i][j]
                model.b1[i] -= lr * db1[i]
            for o in range(model.out_dim):
                for i in range(model.hid_dim):
                    model.w2[o][i] -= lr * dw2[o][i]
                model.b2[o] -= lr * db2[o]

        avg_loss = total_loss / n
        losses.append(avg_loss)
        if epoch % 50 == 0 or epoch == epochs - 1:
            print(f"  epoch {epoch:4d}  mse={avg_loss:.6f}", file=sys.stderr)

    return losses


# ── 主入口 ──────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine temporal end-to-end trainer")
    parser.add_argument("--input", default="/tmp/flow_train_samples.jsonl",
                        help="data_recorder 输出的 JSONL 文件")
    parser.add_argument("--output", default="tools/train/model.txt",
                        help="model.txt 输出路径")
    parser.add_argument("--hidden", type=int, default=32,
                        help="隐层神经元数")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--lr", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)

    print("✦ FlowEngine 时序端到端训练器", file=sys.stderr)
    print(f"  输入: {args.input}", file=sys.stderr)
    print(f"  输出: {args.output}", file=sys.stderr)

    # 加载
    samples = load_jsonl(args.input)
    print(f"  加载: {len(samples)} 条样本", file=sys.stderr)

    # 构建时序窗口
    X_raw, Y_raw = build_windows(samples)
    print(f"  窗口: {len(X_raw)} 条 (5帧×16维={IN_DIM}维输入 → {OUT_DIM}维输出)",
          file=sys.stderr)

    if len(X_raw) < 50:
        print("⚠ 样本太少，建议至少 200+", file=sys.stderr)

    # 标准化
    x_mean, x_scale = column_stats(X_raw)
    y_mean, y_scale = column_stats(Y_raw)
    X_norm = normalize(X_raw, x_mean, x_scale)
    Y_norm = normalize(Y_raw, y_mean, y_scale)

    # 训练
    model = TinyMLP(IN_DIM, args.hidden, OUT_DIM)
    print(f"\n  模型: {IN_DIM}→{args.hidden}→{OUT_DIM}", file=sys.stderr)
    print(f"  训练: {args.epochs} epoch, lr={args.lr}", file=sys.stderr)
    losses = train(model, X_norm, Y_norm, args.epochs, args.lr)
    print(f"  最终 loss: {losses[-1]:.6f}", file=sys.stderr)

    # 导出
    model.save(args.output, x_mean, x_scale, y_mean, y_scale)
    print(f"\n✓ 模型导出: {args.output} ({IN_DIM}→{args.hidden}→{OUT_DIM})",
          file=sys.stderr)

    # 简要验证
    print("\n  验证（取前 5 条样本对比）:", file=sys.stderr)
    for vi in range(min(5, len(X_norm))):
        yp = model.forward(X_norm[vi])
        yd = denormalize([yp], y_mean, y_scale)[0]
        yt = Y_raw[vi]
        print(f"    [{vi}] pred=({yd[0]:.3f},{yd[1]:.3f},{yd[2]:.4f})  true=({yt[0]:.3f},{yt[1]:.3f},{yt[2]:.4f})",
              file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
