#!/usr/bin/env python3
"""
FlowEngine 时序端到端训练器 (v3: 多隐层 MLP)

从 data_recorder 采集的 JSONL 加载样本，堆叠为时序窗口，
训练 MLP（支持多隐层），导出为 tiny_mlp.h 可加载的 model.txt。

用法:
  # 单隐层（v2 兼容）
  python3 tools/train_e2e/temporal_train.py \
      --input /tmp/flow_train_samples.jsonl \
      --hidden 32 \
      --epochs 300

  # 多隐层（v3）
  python3 tools/train_e2e/temporal_train.py \
      --input /tmp/flow_train_samples.jsonl \
      --hidden "128 64" \
      --output tools/train/model_v3.txt

  # 从已有模型继续训练
  python3 tools/train_e2e/temporal_train.py \
      --input /tmp/flow_train_samples.jsonl \
      --init-model tools/train/model.txt \
      --epochs 100
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

WINDOW = 5       # 时序滑窗帧数
V2_DIM = 16      # 每帧特征维度 (v2)
V3_DIM = 59      # 每帧特征维度 (v3 计划)
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


def build_windows(samples: list[dict], feat_dim: int = V2_DIM) -> tuple[list[list[float]], list[list[float]]]:
    """
    将样本流堆叠为 N 帧时序窗口。
    输入: X = [frame[t-WINDOW+1], ..., frame[t]] 共 WINDOW×feat_dim 维
    输出: Y = [throttle, brake, steer, lane_change, confidence]
    """
    X, Y = [], []
    for i in range(WINDOW - 1, len(samples)):
        window = samples[i - WINDOW + 1 : i + 1]

        # 构建输入向量
        x = []
        for s in window:
            fv = s.get("features_v2", s.get("features", [0]*feat_dim))
            fv = s.get("features_v3", fv)  # try v3 features first
            while len(fv) < feat_dim:
                fv.append(0.0)
            x.extend(fv[:feat_dim])

        # 目标样本 = 窗口最后一帧
        target = samples[i]
        ctrl = target.get("control", {})
        throttle = float(ctrl.get("throttle", 0.0))
        brake    = float(ctrl.get("brake", 0.0))
        steer    = float(ctrl.get("steering", 0.0))
        lc       = 1.0 if abs(steer) > 0.12 else 0.0
        conf     = 1.0

        # 过滤停滞数据
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


# ── 评估指标 ────────────────────────────────────────────────

def evaluate_model(model, X_norm: list[list[float]], Y_raw: list[list[float]],
                   y_mean: list[float], y_scale: list[float]) -> dict:
    n = len(X_norm)
    if n == 0:
        return {"sample_count": 0}

    pred_norm = [model.forward(x) for x in X_norm]
    pred_raw = denormalize(pred_norm, y_mean, y_scale)

    out_dim = len(y_mean)
    abs_errors = [[abs(pred_raw[i][j] - Y_raw[i][j]) for j in range(out_dim)] for i in range(n)]

    metrics = {
        "sample_count": n,
        "schema_version": "flowengine.e2e_eval.v3",
    }

    for j, name in enumerate(["throttle", "brake", "steer", "lane_change", "confidence"]):
        if j >= out_dim:
            break
        dim_abs = [ae[j] for ae in abs_errors]
        mae = sum(dim_abs) / n
        rmse = math.sqrt(sum(e * e for e in dim_abs) / n)
        max_err = max(dim_abs)
        metrics[f"mae_{name}"] = mae
        metrics[f"rmse_{name}"] = rmse
        metrics[f"max_abs_error_{name}"] = max_err

    all_abs = [e for dim in abs_errors for e in dim]
    metrics["overall_mae"] = sum(all_abs) / (n * out_dim)
    metrics["overall_rmse"] = math.sqrt(sum(e * e for e in all_abs) / (n * out_dim))

    return metrics


# ── 多隐层 MLP（纯 Python，和 tiny_mlp.h v3 一致）──

class TinyMLP:
    """多隐层 MLP: 支持 1..4 层隐层，纯 Python + math 实现。

    y = out_denorm( W_out·tanh( W_N·...(tanh( W_1·norm(x)+b1 )) ) + b_out )
    """

    def __init__(self, in_dim: int, hidden_dims: list[int], out_dim: int):
        self.in_dim = in_dim
        self.hidden_dims = hidden_dims
        self.hidden_count = len(hidden_dims)
        self.out_dim = out_dim

        # 逐层 Xavier 初始化
        self.w = []
        self.b = []
        prev = in_dim
        for hd in hidden_dims:
            scale = math.sqrt(2.0 / prev)
            self.w.append([[random.gauss(0, scale) for _ in range(prev)] for _ in range(hd)])
            self.b.append([0.0] * hd)
            prev = hd

        # 输出层
        out_scale = math.sqrt(2.0 / prev)
        self.w_out = [[random.gauss(0, out_scale) for _ in range(prev)] for _ in range(out_dim)]
        self.b_out = [0.0] * out_dim

    def forward(self, x: list[float]) -> list[float]:
        """前向: x[in_dim] → y[out_dim]"""
        h = list(x)
        for li in range(self.hidden_count):
            w = self.w[li]
            b = self.b[li]
            h_next = []
            for i in range(len(w)):
                acc = b[i]
                for j in range(len(w[i])):
                    acc += w[i][j] * h[j]
                h_next.append(math.tanh(acc))
            h = h_next
        # 输出层
        y = []
        for i in range(self.out_dim):
            acc = self.b_out[i]
            for j in range(len(h)):
                acc += self.w_out[i][j] * h[j]
            y.append(acc)
        return y

    def save(self, path: str | Path, x_mean: list[float], x_scale: list[float],
             y_mean: list[float], y_scale: list[float]) -> None:
        with open(path, "w") as f:
            f.write(f"# flowengine-tinymlp v{3 if self.hidden_count > 1 else 2}\n")
            f.write(f"in {self.in_dim}\n")
            f.write(f"hidden {' '.join(str(d) for d in self.hidden_dims)}\n")
            f.write(f"out {self.out_dim}\n")

            def write_floats(label: str, vals: list[float]) -> None:
                f.write(f"{label} {' '.join(f'{v:.8f}' for v in vals)}\n")

            write_floats("norm_mean", x_mean)
            write_floats("norm_scale", x_scale)
            write_floats("out_mean", y_mean)
            write_floats("out_scale", y_scale)

            # 各隐层: w1/w2/w3/w4
            for li in range(self.hidden_count):
                w_flat = [self.w[li][i][j] for i in range(len(self.w[li])) for j in range(len(self.w[li][i]))]
                write_floats(f"w{li+1}", w_flat)
                write_floats(f"b{li+1}", self.b[li])

            # 输出层: 单隐层用 w2/b2 标签（向后兼容），多隐层用 w_out/b_out
            out_label_w = "w2" if self.hidden_count <= 1 else "w_out"
            out_label_b = "b2" if self.hidden_count <= 1 else "b_out"
            w_out_flat = [self.w_out[i][j] for i in range(self.out_dim) for j in range(len(self.w_out[i]))]
            write_floats(out_label_w, w_out_flat)
            write_floats(out_label_b, self.b_out)

        print(f"  模型保存: {path} ({self.in_dim}→{'→'.join(str(d) for d in self.hidden_dims)}→{self.out_dim})",
              file=sys.stderr)


def load_model(path: str | Path) -> TinyMLP:
    """从 model.txt 加载模型（v1/v2/v3 格式）"""
    with open(path) as f:
        lines = f.readlines()

    in_dim = 0
    hidden_dims: list[int] = []
    out_dim = 0
    data: dict[str, list[float]] = {}

    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if parts[0] in ("in", "out", "hidden"):
            if parts[0] == "in":
                in_dim = int(parts[1])
            elif parts[0] == "out":
                out_dim = int(parts[1])
            elif parts[0] == "hidden":
                hidden_dims = [int(v) for v in parts[1:]]
            continue
        # 权重行
        key = parts[0]
        vals = [float(v) for v in parts[1:]]
        data[key] = vals

    # 重建模型
    model = TinyMLP(in_dim, hidden_dims, out_dim)
    # 填充权重
    for li in range(model.hidden_count):
        wk = f"w{li+1}"
        bk = f"b{li+1}"
        w = data.get(wk)
        b = data.get(bk)
        if w:
            rows = model.hidden_dims[li]
            cols = model.in_dim if li == 0 else model.hidden_dims[li - 1]
            for i in range(rows):
                for j in range(cols):
                    if i * cols + j < len(w):
                        model.w[li][i][j] = w[i * cols + j]
        if b:
            for i in range(len(b)):
                if i < len(model.b[li]):
                    model.b[li][i] = b[i]

    # 输出层: w_out/b_out 或 w2/b2（单隐层兼容）
    w_key = "w_out" if "w_out" in data else "w2"
    b_key = "b_out" if "b_out" in data else "b2"
    w_ov = data.get(w_key)
    b_ov = data.get(b_key)
    if w_ov:
        last_hid = model.hidden_dims[-1]
        for i in range(model.out_dim):
            for j in range(last_hid):
                if i * last_hid + j < len(w_ov):
                    model.w_out[i][j] = w_ov[i * last_hid + j]
    if b_ov:
        for i in range(len(b_ov)):
            if i < model.out_dim:
                model.b_out[i] = b_ov[i]

    # 归一化参数从 data 读
    norm_mean = data.get("norm_mean", [0.0] * in_dim)
    norm_scale = data.get("norm_scale", [1.0] * in_dim)
    out_mean = data.get("out_mean", [0.0] * out_dim)
    out_scale = data.get("out_scale", [1.0] * out_dim)

    print(f"  模型加载: {path} ({in_dim}→{'→'.join(str(d) for d in hidden_dims)}→{out_dim})",
          file=sys.stderr)

    return model, norm_mean, norm_scale, out_mean, out_scale


# ── 训练 ──────────────────────────────────────────────────

def mse_loss(pred: list[float], target: list[float]) -> float:
    return sum((p - t) ** 2 for p, t in zip(pred, target)) / len(pred)


def train(model: TinyMLP, X: list[list[float]], Y: list[list[float]],
          epochs: int, lr: float) -> list[float]:
    """手写 BP 训练（支持多隐层）"""
    n = len(X)
    losses = []
    nl = model.hidden_count

    for epoch in range(epochs):
        total_loss = 0.0
        for si in range(n):
            x = X[si]
            y_true = Y[si]

            # ── Forward: 存每层激活值 ──
            acts = [list(x)]  # acts[0] = norm 后的输入
            for li in range(nl):
                w, b = model.w[li], model.b[li]
                h = []
                for i in range(len(w)):
                    acc = b[i]
                    for j in range(len(w[i])):
                        acc += w[i][j] * acts[-1][j]
                    h.append(math.tanh(acc))
                acts.append(h)

            # 输出层
            last_h = acts[-1]
            y_pred = []
            for i in range(model.out_dim):
                acc = model.b_out[i]
                for j in range(len(last_h)):
                    acc += model.w_out[i][j] * last_h[j]
                y_pred.append(acc)

            loss = mse_loss(y_pred, y_true)
            total_loss += loss

            # ── Backward ──
            dy = [(y_pred[o] - y_true[o]) * 2.0 / model.out_dim for o in range(model.out_dim)]

            # 输出层梯度
            dw_out = [[dy[k] * acts[-1][j] for j in range(len(acts[-1]))] for k in range(model.out_dim)]
            db_out = list(dy)

            # 反向传播到隐层
            # deltas[nl] = dL/dz for last hidden layer (z = W_nl-1 @ h + b)
            # = W_out^T @ dy * tanh'(acts[-1]) with dim = hidden_dims[nl-1]
            deltas = [None] * (nl + 1)
            last_hid = len(acts[-1])
            deltas[nl] = [
                sum(model.w_out[k][j] * dy[k] for k in range(model.out_dim)) *
                (1 - acts[-1][j] * acts[-1][j])
                for j in range(last_hid)
            ]

            # 反向传播到各隐层: 从 li = nl-1 到 0
            for li in range(nl - 1, -1, -1):
                d_next = deltas[li + 1]             # dL/dz for layer li (dim = hidden_dims[li])
                h_in = acts[li]                     # input to layer li (dim = in_dim if li==0 else hidden_dims[li-1])
                w_cur = model.w[li]                 # [hidden_dims[li]] × [h_in_dim]
                # dL/d(acts[li]) = W_li^T @ d_next (dim = h_in_dim)
                dlda = [0.0] * len(h_in)
                for j in range(len(h_in)):
                    acc = 0.0
                    for i in range(len(d_next)):
                        acc += w_cur[i][j] * d_next[i]
                    dlda[j] = acc
                # deltas[li] = dlda * tanh'(acts[li]) (dim = h_in_dim)
                # acts[li] is the tanh output of layer li-1 (or norm input for li=0)
                # For li=0, acts[0] is normalized input (no tanh), skip tanh'
                if li == 0:
                    deltas[0] = dlda
                else:
                    deltas[li] = [dlda[j] * (1 - acts[li][j] * acts[li][j]) for j in range(len(h_in))]

            # 权重更新：从 li=0 到 nl-1
            for li in range(nl):
                h_in = acts[li]
                d_next = deltas[li + 1]  # dL/dz for layer li
                for i in range(len(d_next)):
                    for j in range(len(h_in)):
                        model.w[li][i][j] -= lr * d_next[i] * h_in[j]
                    model.b[li][i] -= lr * d_next[i]

            for k in range(model.out_dim):
                for j in range(len(acts[-1])):
                    model.w_out[k][j] -= lr * dw_out[k][j]
                model.b_out[k] -= lr * db_out[k]

        avg_loss = total_loss / n
        losses.append(avg_loss)
        if epoch % 50 == 0 or epoch == epochs - 1:
            print(f"  epoch {epoch:4d}  mse={avg_loss:.6f}", file=sys.stderr)

    return losses


# ── 数据增强 ──────────────────────────────────────────────────

def augment(X: list[list[float]], Y: list[list[float]],
            noise_loc: float = 0.15, noise_scale: float = 0.10,
            drop_prob: float = 0.2, flip_prob: float = 0.05) -> tuple[list[list[float]], list[list[float]]]:
    """对输入数据注入传感器噪声增强。

    - 障碍物位置: 高斯噪声 noise_loc/noise_scale
    - 随机丢弃障碍物: drop_prob
    - 红绿灯翻转: flip_prob（需 features_v3 格式）
    """
    X_aug, Y_aug = list(X), list(Y)
    for x in X:
        x_aug = list(x)
        # 对每帧的障碍物特征加噪声（v2: 索引 4-13 为前 2 障碍物）
        for fi in range(WINDOW):
            offset = fi * V2_DIM
            # 前障碍物 x（每次帧第 4/9 维）
            for oi in range(2):
                bx_idx = offset + 4 + oi * 5
                if bx_idx < len(x_aug) and abs(x_aug[bx_idx]) > 0.01:
                    x_aug[bx_idx] += random.gauss(0, noise_loc)
                    by_idx = bx_idx + 1
                    if by_idx < len(x_aug):
                        x_aug[by_idx] += random.gauss(0, noise_scale)
                    bv_idx = bx_idx + 2
                    if bv_idx < len(x_aug):
                        x_aug[bv_idx] += random.gauss(0, 0.5)
                    # 随机丢弃（设为 0）
                    if random.random() < drop_prob:
                        for di in range(5):
                            if bx_idx + di < len(x_aug):
                                x_aug[bx_idx + di] = 0.0
        X_aug.append(x_aug)
        Y_aug.append(Y[len(X_aug) - 1])
    return X_aug, Y_aug


# ── 主入口 ──────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine 时序端到端训练器 (v3 多隐层)")
    parser.add_argument("--input", default="/tmp/flow_train_samples.jsonl",
                        help="data_recorder 输出的 JSONL 文件")
    parser.add_argument("--output", default="tools/train/model.txt",
                        help="model.txt 输出路径")
    parser.add_argument("--hidden", default="32",
                        help='隐层维度，空格分隔（如 "128 64"）')
    parser.add_argument("--init-model",
                        help="从已有 model.txt 加载初始权重（继续训练）")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--lr", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--augment", action="store_true",
                        help="启用数据增强")
    parser.add_argument("--eval", action="store_true",
                        help="评估模式：加载已有模型，计算指标（跳过训练）")
    args = parser.parse_args()

    random.seed(args.seed)

    hidden_dims = [int(v) for v in args.hidden.split()]
    if not hidden_dims:
        hidden_dims = [32]

    print("✦ FlowEngine 时序端到端训练器 (v3)", file=sys.stderr)
    print(f"  输入: {args.input}", file=sys.stderr)
    print(f"  输出: {args.output}", file=sys.stderr)

    # 加载
    samples = load_jsonl(args.input)
    print(f"  加载: {len(samples)} 条样本", file=sys.stderr)

    # 自动检测特征维度
    feat_dim = V2_DIM
    if samples:
        first = samples[0]
        if "features_v3" in first:
            feat_dim = V3_DIM
        elif "features_v2" in first:
            feat_dim = V2_DIM
        elif "features" in first:
            feat_dim = max(V2_DIM, len(first.get("features", [])))

    # 构建时序窗口
    X_raw, Y_raw = build_windows(samples, feat_dim)
    in_dim = WINDOW * feat_dim
    out_dim = OUT_DIM
    print(f"  窗口: {len(X_raw)} 条 ({WINDOW}帧×{feat_dim}维={in_dim}维输入 → {out_dim}维输出)",
          file=sys.stderr)

    if len(X_raw) < 50:
        print("⚠ 样本太少，建议至少 200+", file=sys.stderr)

    # 数据增强
    if args.augment and len(X_raw) > 0:
        X_aug, Y_aug = augment(X_raw, Y_raw)
        print(f"  增强: {len(X_aug)} 条 (原 {len(X_raw)} + 增强 {len(X_aug) - len(X_raw)})",
              file=sys.stderr)
        X_raw, Y_raw = X_aug, Y_aug

    # 标准化
    x_mean, x_scale = column_stats(X_raw)
    y_mean, y_scale = column_stats(Y_raw)
    X_norm = normalize(X_raw, x_mean, x_scale)
    Y_norm = normalize(Y_raw, y_mean, y_scale)

    # 初始化模型（从已有权重或随机初始化）
    if args.init_model:
        model, x_mean, x_scale, y_mean, y_scale = load_model(args.init_model)
        # 如果输入维度不匹配，重建最后一层
        if model.in_dim != in_dim:
            print(f"  输入维度 {model.in_dim}→{in_dim}，重建模型结构...", file=sys.stderr)
            # 保留已加载的隐层权重，重建输入层
            old_w0 = model.w[0]
            model.in_dim = in_dim
            # 重新初始化 w0（输入层大幅变化，不适合复用）
            prev = in_dim
            for li in range(model.hidden_count):
                scale = math.sqrt(2.0 / prev)
                model.w[li] = [[random.gauss(0, scale) for _ in range(prev)] for _ in range(model.hidden_dims[li])]
                prev = model.hidden_dims[li]
            # 复制已加载的隐层权重（层间）
            for li in range(1, min(len(old_w0) if li == 0 else 0, model.hidden_count)):
                pass
        print(f"  从初始模型继续训练", file=sys.stderr)
    else:
        model = TinyMLP(in_dim, hidden_dims, out_dim)

    if args.eval:
        model, x_mean, x_scale, y_mean, y_scale = load_model(args.output)
        print(f"\n  评估模式: 加载模型 {args.output} ({model.in_dim}→{'→'.join(str(d) for d in model.hidden_dims)}→{model.out_dim})", file=sys.stderr)
    else:
        print(f"\n  模型: {in_dim}→{'→'.join(str(d) for d in model.hidden_dims)}→{out_dim} "
              f"({model.hidden_count} 隐层)", file=sys.stderr)
        print(f"  训练: {args.epochs} epoch, lr={args.lr}", file=sys.stderr)
        losses = train(model, X_norm, Y_norm, args.epochs, args.lr)
        print(f"  最终 loss: {losses[-1]:.6f}", file=sys.stderr)

        model.save(args.output, x_mean, x_scale, y_mean, y_scale)

    # 评估指标计算
    import math
    metrics = evaluate_model(model, X_norm, Y_raw, y_mean, y_scale)
    print("\n  评估指标:", file=sys.stderr)
    for key, val in metrics.items():
        if isinstance(val, float):
            print(f"    {key}: {val:.6f}", file=sys.stderr)
        else:
            print(f"    {key}: {val}", file=sys.stderr)

    metrics_path = Path(args.output).parent / "metrics.json"
    import json as _json
    with open(metrics_path, "w", encoding="utf-8") as f:
        _json.dump(metrics, f, indent=2, ensure_ascii=False)
    print(f"\n  指标已保存: {metrics_path}", file=sys.stderr)

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
