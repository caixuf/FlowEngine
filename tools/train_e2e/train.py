#!/usr/bin/env python3
"""Train a lightweight E2E planner artifact without external ML dependencies."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "train"))

import train as tiny_train  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine E2E training bridge (tiny-MLP backend)")
    parser.add_argument("--dataset", required=True, help="Dataset directory from export_e2e_dataset.py")
    parser.add_argument("--output", required=True, help="Artifact output directory")
    parser.add_argument("--hidden", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--lr", type=float, default=0.05)
    args = parser.parse_args()

    dataset_dir = Path(args.dataset)
    samples_path = dataset_dir / "samples.jsonl"
    metadata_path = dataset_dir / "metadata.json"
    if not samples_path.exists():
        raise SystemExit(f"error: dataset samples not found: {samples_path}")

    dataset_meta = {}
    if metadata_path.exists():
        dataset_meta = json.loads(metadata_path.read_text(encoding="utf-8"))

    feats, labels = tiny_train.load_samples(samples_path)
    if len(feats) < 10:
        raise SystemExit(f"error: too few samples ({len(feats)}) in {samples_path}")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = output_dir / "model.txt"

    model = tiny_train.train(feats, labels, hidden=args.hidden, epochs=args.epochs, lr=args.lr)
    tiny_train.export(model, model_path)

    manifest = {
        "artifact_version": "flowengine.e2e_artifact.v1",
        "created_unix_ms": int(time.time() * 1000),
        "model_format": "flowengine-tinymlp-v1",
        "model_path": "model.txt",
        "backend": "tiny_mlp",
        "input_schema": {
            "features": dataset_meta.get("feature_names", ["ego_v", "ego_y", "ego_heading", "ego_yaw_rate"]),
        },
        "output_schema": {
            "labels": dataset_meta.get("label_names", ["target_speed"]),
        },
        "dataset": {
            "path": str(dataset_dir),
            "schema_version": dataset_meta.get("schema_version", "unknown"),
            "sample_count": len(feats),
            "scenario": dataset_meta.get("scenario", "unknown"),
        },
        "training": {
            "hidden": args.hidden,
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