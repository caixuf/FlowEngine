#!/usr/bin/env python3
"""PyTorch trainer for FlowEngine E2E planner artifacts.

This script is intentionally optional: FlowEngine can run without PyTorch, but
when PyTorch is installed this provides the first real training-framework bridge.

2026-08-05: 学习闭环「真正用起来」升级
  - GPU 训练：--device cuda 自动检测（RTX 5060 实测 8.5GB）
  - 多层网络：--hidden 可传 "32,32" 多隐层，突破单隐层表达上限
  - 时序窗口：--window N 拼接连续 N 帧特征（复用 temporal_train.build_windows），
    让模型看到前车轨迹/灯相位历史（单帧快照 → 单值 target 的信息论死结）
  - --export-onnx：训练后导出 ONNX → C 侧 onnx_backend 影子推理
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from feature_schema import FEATURE_NAMES_V1, LABEL_NAMES  # noqa: E402


def require_torch():
    try:
        import torch
        from torch import nn
    except ModuleNotFoundError:
        print(
            "error: PyTorch is not installed. Install torch to use tools/train_e2e/torch_train.py, "
            "or use tools/train_e2e/train.py for the zero-dependency tiny-MLP backend.",
            file=sys.stderr,
        )
        raise SystemExit(2)
    return torch, nn


def load_dataset(dataset_dir: Path) -> tuple[list[list[float]], list[list[float]], dict]:
    samples_path = dataset_dir / "samples.jsonl"
    metadata_path = dataset_dir / "metadata.json"
    if not samples_path.exists():
        raise SystemExit(f"error: dataset samples not found: {samples_path}")

    metadata = {}
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    feature_names = metadata.get("feature_names", FEATURE_NAMES_V1)

    features = []
    labels = []
    with samples_path.open("r", encoding="utf-8") as src:
        for line in src:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            x = obj.get("features")
            y = obj.get("label")
            if isinstance(x, list) and len(x) == len(feature_names) and y is not None:
                features.append([float(v) for v in x])
                labels.append([float(y)])

    if len(features) < 10:
        raise SystemExit(f"error: too few samples ({len(features)}) in {samples_path}")

    return features, labels, metadata


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


def load_init_checkpoint(torch, init_from: Path) -> dict:
    manifest_path = init_from / "manifest.json"
    if not manifest_path.exists():
        raise SystemExit(f"error: init artifact manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("backend") != "pytorch":
        raise SystemExit(f"error: --init-from requires a pytorch artifact: {init_from}")
    model_path = init_from / manifest.get("model_path", "model.pt")
    if not model_path.exists():
        raise SystemExit(f"error: init artifact model not found: {model_path}")
    return torch.load(model_path, map_location="cpu")


def validate_init_checkpoint(checkpoint: dict, feature_names: list[str], label_names: list[str], hidden: int) -> None:
    if checkpoint.get("feature_names") != feature_names:
        raise SystemExit("error: init artifact feature_names do not match the target dataset")
    if checkpoint.get("label_names") != label_names:
        raise SystemExit("error: init artifact label_names do not match the target dataset")
    if int(checkpoint.get("hidden", -1)) != hidden:
        raise SystemExit("error: init artifact hidden size does not match --hidden")


def parse_hidden(spec: str) -> list[int]:
    """'32' → [32]；'32,64' → [32,64]（多隐层）。"""
    return [int(x) for x in spec.split(",") if x.strip()]


def build_mlp(nn, in_dim: int, hidden: list[int], out_dim: int):
    """多层 MLP：Linear+Tanh 交替，末层 Linear 无激活。"""
    layers = []
    prev = in_dim
    for h in hidden:
        layers.append(nn.Linear(prev, h))
        layers.append(nn.Tanh())
        prev = h
    layers.append(nn.Linear(prev, out_dim))
    return nn.Sequential(*layers)


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine E2E PyTorch trainer")
    parser.add_argument("--dataset", required=True, help="Dataset directory from export_e2e_dataset.py")
    parser.add_argument("--output", required=True, help="Artifact output directory")
    parser.add_argument("--hidden", default="32", help="隐层，'32' 或 '32,64' 多隐层")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--init-from", default=None, help="Existing PyTorch artifact directory to initialize from")
    parser.add_argument("--device", default="auto", help="auto/cuda/cpu")
    parser.add_argument("--window", type=int, default=0,
                        help="时序窗口：拼接连续 N 帧特征（0=单帧）")
    parser.add_argument("--export-onnx", action="store_true",
                        help="训练后导出 ONNX（C 侧 onnx_backend 推理）")
    args = parser.parse_args()

    torch, nn = require_torch()
    random.seed(args.seed)
    torch.manual_seed(args.seed)

    # GPU 检测
    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    if device == "cuda" and not torch.cuda.is_available():
        print("warning: --device cuda 但 CUDA 不可用，回退 CPU", file=sys.stderr)
        device = "cpu"
    print(f"training on {device}: {torch.cuda.get_device_name(0) if device=='cuda' else 'CPU'}")
    if device == "cuda":
        torch.cuda.manual_seed_all(args.seed)

    dataset_dir = Path(args.dataset)
    features, labels, dataset_meta = load_dataset(dataset_dir)
    feature_names = dataset_meta.get("feature_names", FEATURE_NAMES_V1)
    label_names = dataset_meta.get("label_names", LABEL_NAMES)

    # 时序窗口：拼接连续 N 帧（需样本按时间排序，同 temporal_train.build_windows）
    if args.window > 1:
        sys.path.insert(0, str(Path(__file__).parent))
        from temporal_train import build_windows  # noqa: E402
        features, labels = build_windows(
            [{"features": f} for f in features], feat_dim=len(feature_names))
        # build_windows 的 Y 是标签窗口拼接？直接取每窗最后帧的标签
        labels = [y[-len(label_names):] for y in labels]
        in_dim = len(features[0])
    else:
        in_dim = len(feature_names)

    x_mean, x_scale = column_stats(features)
    y_mean, y_scale = column_stats(labels)
    x_norm = normalize(features, x_mean, x_scale)
    y_norm = normalize(labels, y_mean, y_scale)

    x_tensor = torch.tensor(x_norm, dtype=torch.float32).to(device)
    y_tensor = torch.tensor(y_norm, dtype=torch.float32).to(device)

    hidden = parse_hidden(args.hidden)
    model = build_mlp(nn, in_dim, hidden, len(LABEL_NAMES)).to(device)
    if args.init_from:
        init_checkpoint = load_init_checkpoint(torch, Path(args.init_from))
        validate_init_checkpoint(init_checkpoint, feature_names, label_names, hidden)
        model.load_state_dict(init_checkpoint["state_dict"])
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    for epoch in range(args.epochs):
        optimizer.zero_grad()
        pred = model(x_tensor)
        loss = loss_fn(pred, y_tensor)
        loss.backward()
        optimizer.step()
        if epoch % 50 == 0 or epoch == args.epochs - 1:
            print(f"  epoch {epoch:4d} mse(norm)={float(loss.item()):.6f}", file=sys.stderr)

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = {
        "state_dict": {k: v.to("cpu") for k, v in model.state_dict().items()},
        "input_mean": x_mean,
        "input_scale": x_scale,
        "output_mean": y_mean,
        "output_scale": y_scale,
        "feature_names": feature_names,
        "label_names": label_names,
        "hidden": args.hidden,
        "window": args.window,
    }
    torch.save(checkpoint, output_dir / "model.pt")

    # ONNX 导出（C 侧 onnx_backend 影子推理用）
    if args.export_onnx:
        try:
            import onnx
        except ModuleNotFoundError:
            print("warning: --export-onnx 需要 pip install onnx，跳过导出", file=sys.stderr)
            args.export_onnx = False
        else:
            model.eval()
            # 归一化折进图：输入 x_raw → (x_raw - mean) / scale → 网络。
            # onnx_backend 只喂原始特征、取原始输出（不感知归一化），
            # 未折叠时 C 侧拿到未归一化输入 → 预测离谱（实测 pred=1.13
            # vs planning=20）。mean/scale 用训练集统计（与 checkpoint 一致）。
            # 用 nn.Module 包装（Sub+Div+model），不用闭包——
            # torch.export strict 模式拒绝引用外部变量的闭包。
            class NormWrapper(nn.Module):
                def __init__(self, core, mean, scale, out_mean, out_scale):
                    super().__init__()
                    self.core = core
                    self.register_buffer("mean", mean)
                    self.register_buffer("scale", scale)
                    self.register_buffer("out_mean", out_mean)
                    self.register_buffer("out_scale", out_scale)

                def forward(self, x):
                    # 输入归一化 + 输出反归一化都折进图：
                    # onnx_backend 只喂原始特征、取原始输出。
                    # 缺输出反归一化时 ONNX 返回归一化空间值（实测 pred=0.24
                    # vs PyTorch 18.86 —— 相差一个 out_scale 量级）
                    y = self.core((x - self.mean) / self.scale)
                    return y * self.out_scale + self.out_mean

            wrapped = NormWrapper(
                model,
                torch.tensor(x_mean, dtype=torch.float32).to(device),
                torch.tensor(x_scale, dtype=torch.float32).to(device),
                torch.tensor(y_mean, dtype=torch.float32).to(device),
                torch.tensor(y_scale, dtype=torch.float32).to(device),
            ).to(device)

            dummy = torch.zeros(1, in_dim, dtype=torch.float32).to(device)
            try:
                torch.onnx.export(
                    wrapped, dummy,
                    str(output_dir / "model.onnx"),
                    input_names=["features"],
                    output_names=["output"],
                    opset_version=17,
                    # batch 固定 1：onnx_backend 要求静态 batch（首维==1），
                    # dynamic_axes 导出 [0,23] 会被 C 侧拒绝（静默错推理防护）
                )
                print(f"ONNX 导出: {output_dir / 'model.onnx'}")
            except Exception as e:
                print(f"warning: ONNX 导出失败: {e}", file=sys.stderr)
                args.export_onnx = False

    # backend 语义（2026-08-05）：导出 ONNX 成功 → backend=onnx（C runtime
    # 可加载，modelctl promote 门禁认它）；否则 backend=pytorch（sidecar only）
    is_onnx = bool(args.export_onnx and (output_dir / "model.onnx").exists())
    manifest = {
        "artifact_version": "flowengine.e2e_artifact.v1",
        "created_unix_ms": int(time.time() * 1000),
        "model_format": "onnx-v1" if is_onnx else "torch-state-dict-v1",
        "model_path": "model.onnx" if is_onnx else "model.pt",
        "backend": "onnx" if is_onnx else "pytorch",
        "input_schema": {"features": checkpoint["feature_names"]},
        "output_schema": {"labels": checkpoint["label_names"]},
        "dataset": {
            "path": str(dataset_dir),
            "schema_version": dataset_meta.get("schema_version", "unknown"),
            "sample_count": len(features),
            "scenario": dataset_meta.get("scenario", "unknown"),
        },
        "training": {
            "hidden": args.hidden,
            "epochs": args.epochs,
            "lr": args.lr,
            "seed": args.seed,
            "init_from": str(Path(args.init_from)) if args.init_from else None,
            "final_mse_norm": float(loss.item()),
            "device": device,
            "window": args.window,
        },
        "onnx_path": "model.onnx" if args.export_onnx else None,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"torch artifact exported: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())