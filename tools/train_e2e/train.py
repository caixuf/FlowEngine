#!/usr/bin/env python3
"""Train a lightweight E2E planner artifact (v3 temporal-MLP)."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(ROOT))

from temporal_train import TinyMLP, load_jsonl, build_windows, column_stats, normalize, train  # noqa: E402


OUT_DIM = 5


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine E2E training (v3)")
    parser.add_argument("--dataset", required=True, help="Dataset directory")
    parser.add_argument("--output", required=True, help="Artifact output directory")
    parser.add_argument("--hidden", type=str, default="32")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--lr", type=float, default=0.001)
    args = parser.parse_args()

    dataset_dir = Path(args.dataset)
    samples_path = dataset_dir / "samples.jsonl"
    metadata_path = dataset_dir / "metadata.json"

    if not samples_path.exists():
        raise SystemExit(f"error: dataset samples not found: {samples_path}")

    dataset_meta = {}
    if metadata_path.exists():
        dataset_meta = json.loads(metadata_path.read_text(encoding="utf-8"))

    samples = load_jsonl(samples_path)
    if not samples:
        raise SystemExit(f"error: no samples in {samples_path}")

    X_raw, Y_raw = build_windows(samples)
    if len(X_raw) < 10:
        raise SystemExit(f"error: too few samples ({len(X_raw)}) in {samples_path}")

    x_mean, x_scale = column_stats(X_raw)
    y_mean, y_scale = column_stats(Y_raw)
    X_norm = normalize(X_raw, x_mean, x_scale)
    Y_norm = normalize(Y_raw, y_mean, y_scale)

    hidden_dims = [int(v) for v in args.hidden.split()]
    if not hidden_dims:
        hidden_dims = [32]

    in_dim = len(X_raw[0])
    out_dim = len(Y_raw[0])
    model = TinyMLP(in_dim, hidden_dims, out_dim)

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = output_dir / "model.txt"

    train(model, X_norm, Y_norm, args.epochs, args.lr)
    model.save(model_path, x_mean, x_scale, y_mean, y_scale)

    manifest = {
        "artifact_version": "flowengine.e2e_artifact.v3",
        "created_unix_ms": int(time.time() * 1000),
        "model_format": "flowengine-tinymlp-v3",
        "model_path": "model.txt",
        "backend": "temporal_mlp",
        "input_schema": {
            "features": dataset_meta.get("feature_names", ["temporal_window_features"]),
        },
        "output_schema": {
            "labels": ["throttle", "brake", "steer", "lane_change", "confidence"],
        },
        "dataset": {
            "path": str(dataset_dir),
            "schema_version": dataset_meta.get("schema_version", "unknown"),
            "sample_count": len(X_raw),
            "scenario": dataset_meta.get("scenario", "unknown"),
        },
        "training": {
            "hidden": hidden_dims,
            "epochs": args.epochs,
            "lr": args.lr,
        },
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    if metadata_path.exists():
        shutil.copyfile(metadata_path, output_dir / "dataset_metadata.json")

    print(f"artifact exported: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())