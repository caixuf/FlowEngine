#!/usr/bin/env python3
"""PyTorch trainer for FlowEngine E2E planner artifacts.

This script is intentionally optional: FlowEngine can run without PyTorch, but
when PyTorch is installed this provides the first real training-framework bridge.
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


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine E2E PyTorch trainer")
    parser.add_argument("--dataset", required=True, help="Dataset directory from export_e2e_dataset.py")
    parser.add_argument("--output", required=True, help="Artifact output directory")
    parser.add_argument("--hidden", type=int, default=32)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--init-from", default=None, help="Existing PyTorch artifact directory to initialize from")
    args = parser.parse_args()

    torch, nn = require_torch()
    random.seed(args.seed)
    torch.manual_seed(args.seed)

    dataset_dir = Path(args.dataset)
    features, labels, dataset_meta = load_dataset(dataset_dir)
    feature_names = dataset_meta.get("feature_names", FEATURE_NAMES_V1)
    label_names = dataset_meta.get("label_names", LABEL_NAMES)
    x_mean, x_scale = column_stats(features)
    y_mean, y_scale = column_stats(labels)
    x_norm = normalize(features, x_mean, x_scale)
    y_norm = normalize(labels, y_mean, y_scale)

    x_tensor = torch.tensor(x_norm, dtype=torch.float32)
    y_tensor = torch.tensor(y_norm, dtype=torch.float32)

    model = nn.Sequential(
        nn.Linear(len(feature_names), args.hidden),
        nn.Tanh(),
        nn.Linear(args.hidden, len(LABEL_NAMES)),
    )
    if args.init_from:
        init_checkpoint = load_init_checkpoint(torch, Path(args.init_from))
        validate_init_checkpoint(init_checkpoint, feature_names, label_names, args.hidden)
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
        "state_dict": model.state_dict(),
        "input_mean": x_mean,
        "input_scale": x_scale,
        "output_mean": y_mean,
        "output_scale": y_scale,
        "feature_names": feature_names,
        "label_names": label_names,
        "hidden": args.hidden,
    }
    torch.save(checkpoint, output_dir / "model.pt")

    manifest = {
        "artifact_version": "flowengine.e2e_artifact.v1",
        "created_unix_ms": int(time.time() * 1000),
        "model_format": "torch-state-dict-v1",
        "model_path": "model.pt",
        "backend": "pytorch",
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
        },
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"torch artifact exported: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())