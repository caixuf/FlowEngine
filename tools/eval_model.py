#!/usr/bin/env python3
"""Offline evaluator for FlowEngine E2E artifacts."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR / "train_e2e"))

from feature_schema import FEATURE_NAMES_V1  # noqa: E402


def load_dataset(dataset_dir: Path) -> list[dict]:
    samples_path = dataset_dir / "samples.jsonl"
    samples = []
    with samples_path.open("r", encoding="utf-8") as src:
        for line in src:
            line = line.strip()
            if line:
                samples.append(json.loads(line))
    if not samples:
        raise SystemExit(f"error: no samples in {samples_path}")
    return samples


def parse_vector(line: str) -> list[float]:
    return [float(v) for v in line.split()[1:]]


def load_tiny_mlp(path: Path) -> dict:
    model = {}
    with path.open("r", encoding="utf-8") as src:
        for raw in src:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key = line.split()[0]
            if key == "in":
                model["in"] = int(line.split()[1])
            elif key == "hidden":
                model["hidden"] = int(line.split()[1])
            elif key == "out":
                model["out"] = int(line.split()[1])
            elif key in {"norm_mean", "norm_scale", "out_mean", "out_scale", "w1", "b1", "w2", "b2"}:
                model[key] = parse_vector(line)
    required = {"in", "hidden", "out", "norm_mean", "norm_scale", "out_mean", "out_scale", "w1", "b1", "w2", "b2"}
    missing = sorted(required - set(model))
    if missing:
        raise SystemExit(f"error: invalid model {path}, missing {missing}")
    return model


def predict(model: dict, features: list[float]) -> float:
    in_dim = model["in"]
    hidden = model["hidden"]
    x = [
        (float(features[i]) - model["norm_mean"][i]) / model["norm_scale"][i]
        for i in range(in_dim)
    ]
    h = []
    for j in range(hidden):
        base = j * in_dim
        acc = model["b1"][j]
        for i in range(in_dim):
            acc += model["w1"][base + i] * x[i]
        h.append(math.tanh(acc))
    out = model["b2"][0]
    for j in range(hidden):
        out += model["w2"][j] * h[j]
    return out * model["out_scale"][0] + model["out_mean"][0]


def evaluate_tiny_mlp(model_dir: Path, dataset_dir: Path, manifest: dict) -> dict:
    model_path = model_dir / manifest.get("model_path", "model.txt")
    model = load_tiny_mlp(model_path)
    samples = load_dataset(dataset_dir)

    abs_errors = []
    signed_errors = []
    for sample in samples:
        pred = predict(model, sample["features"])
        err = pred - float(sample["label"])
        signed_errors.append(err)
        abs_errors.append(abs(err))

    return build_metrics(model_dir, dataset_dir, len(samples), abs_errors, signed_errors)


def evaluate_torch(model_dir: Path, dataset_dir: Path, manifest: dict) -> dict:
    try:
        import torch
        from torch import nn
    except ModuleNotFoundError:
        raise SystemExit(
            "error: PyTorch is not installed, cannot evaluate a pytorch artifact. "
            "Install torch or evaluate a tiny-MLP artifact."
        )

    model_path = model_dir / manifest.get("model_path", "model.pt")
    checkpoint = torch.load(model_path, map_location="cpu")
    feature_names = checkpoint.get("feature_names", FEATURE_NAMES_V1)
    label_names = checkpoint.get("label_names", ["target_speed"])
    hidden = int(checkpoint.get("hidden", 32))

    model = nn.Sequential(
        nn.Linear(len(feature_names), hidden),
        nn.Tanh(),
        nn.Linear(hidden, len(label_names)),
    )
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    input_mean = checkpoint["input_mean"]
    input_scale = checkpoint["input_scale"]
    output_mean = checkpoint["output_mean"]
    output_scale = checkpoint["output_scale"]
    samples = load_dataset(dataset_dir)

    abs_errors = []
    signed_errors = []
    with torch.no_grad():
        for sample in samples:
            x = [
                (float(sample["features"][i]) - input_mean[i]) / input_scale[i]
                for i in range(len(feature_names))
            ]
            y_norm = model(torch.tensor([x], dtype=torch.float32))[0][0].item()
            pred = y_norm * output_scale[0] + output_mean[0]
            err = pred - float(sample["label"])
            signed_errors.append(err)
            abs_errors.append(abs(err))

    return build_metrics(model_dir, dataset_dir, len(samples), abs_errors, signed_errors)


def build_metrics(model_dir: Path, dataset_dir: Path, sample_count: int,
                  abs_errors: list[float], signed_errors: list[float]) -> dict:
    return {
        "schema_version": "flowengine.e2e_eval.v1",
        "model": str(model_dir),
        "dataset": str(dataset_dir),
        "sample_count": sample_count,
        "mae_target_speed": sum(abs_errors) / len(abs_errors),
        "mean_error": sum(signed_errors) / len(signed_errors),
        "max_abs_error": max(abs_errors),
    }


def evaluate(model_dir: Path, dataset_dir: Path) -> dict:
    manifest_path = model_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    backend = manifest.get("backend", "tiny_mlp")
    if backend == "pytorch":
        return evaluate_torch(model_dir, dataset_dir, manifest)
    return evaluate_tiny_mlp(model_dir, dataset_dir, manifest)


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate FlowEngine E2E tiny-MLP artifact offline")
    parser.add_argument("--model", required=True, help="Artifact directory from tools/train_e2e/train.py")
    parser.add_argument("--dataset", required=True, help="Dataset directory from export_e2e_dataset.py")
    parser.add_argument("--output", default=None, help="Optional metrics JSON path")
    args = parser.parse_args()

    metrics = evaluate(Path(args.model), Path(args.dataset))
    text = json.dumps(metrics, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())