#!/usr/bin/env python3
"""Inspect and promote FlowEngine learning-loop model artifacts."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODELS_DIR = ROOT / "models"
DEFAULT_RUNTIME_MODEL = ROOT / "tools" / "train" / "model.txt"


def load_manifest(artifact_dir: Path) -> dict:
    manifest_path = artifact_dir / "manifest.json"
    if not manifest_path.exists():
        return {}
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {"_error": "invalid manifest.json"}


def load_metrics(artifact_dir: Path) -> dict | None:
    metrics_path = artifact_dir / "metrics.json"
    if not metrics_path.exists():
        return None
    try:
        return json.loads(metrics_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {"_error": "invalid metrics.json"}


def iter_artifacts(models_dir: Path) -> list[tuple[Path, dict]]:
    if not models_dir.exists():
        return []
    artifacts = []
    for path in sorted(models_dir.iterdir()):
        if path.is_dir() and (path / "manifest.json").exists():
            artifacts.append((path, load_manifest(path)))
    return artifacts


def model_file_for_artifact(artifact_dir: Path, manifest: dict) -> Path:
    return artifact_dir / manifest.get("model_path", "model.txt")


def describe_artifact(artifact_dir: Path, manifest: dict) -> str:
    backend = manifest.get("backend", "unknown")
    model_format = manifest.get("model_format", "unknown")
    model_path = model_file_for_artifact(artifact_dir, manifest)
    dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
    sample_count = dataset.get("sample_count", "?")
    scenario = dataset.get("scenario", "unknown")
    runtime_note = "runtime-promotable" if backend == "tiny_mlp" else "sidecar/eval only"
    exists = "ok" if model_path.exists() else "missing-model"
    return (
        f"  {artifact_dir.name:<28} backend={backend:<9} format={model_format:<22} "
        f"samples={sample_count!s:<5} scenario={scenario:<12} {runtime_note} {exists}"
    )


def cmd_list(args: argparse.Namespace) -> int:
    runtime_model = Path(args.runtime_model)
    models_dir = Path(args.models_dir)

    print("FlowEngine models")
    print(f"  runtime tiny : {runtime_model}")
    print(f"  runtime state: {'present' if runtime_model.exists() else 'missing'}")
    print(f"  artifact dir : {models_dir}")

    artifacts = iter_artifacts(models_dir)
    if not artifacts:
        print("  artifacts    : none")
        return 0

    print("  artifacts:")
    for artifact_dir, manifest in artifacts:
        print(describe_artifact(artifact_dir, manifest))
    return 0


def cmd_inspect(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact)
    if not artifact_dir.is_dir():
        raise SystemExit(f"error: artifact directory not found: {artifact_dir}")

    manifest = load_manifest(artifact_dir)
    metrics = load_metrics(artifact_dir)

    print(f"artifact : {artifact_dir}")
    print(f"backend  : {manifest.get('backend', 'unknown')}")
    print(f"format   : {manifest.get('model_format', 'unknown')}")

    dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
    if dataset:
        print(f"dataset  : {dataset.get('path', '?')} ({dataset.get('sample_count', '?')} samples,"
              f" scenario={dataset.get('scenario', '?')}, schema={dataset.get('schema_version', '?')})")

    training = manifest.get("training", {}) if isinstance(manifest.get("training"), dict) else {}
    if training:
        parts = [f"epochs={training.get('epochs', '?')}", f"lr={training.get('lr', '?')}",
                 f"hidden={training.get('hidden', '?')}"]
        if training.get("init_from"):
            parts.append(f"init_from={training['init_from']}")
        if training.get("final_mse_norm") is not None:
            parts.append(f"final_mse_norm={training['final_mse_norm']:.6f}")
        print(f"training : {', '.join(parts)}")

    in_schema = manifest.get("input_schema", {})
    out_schema = manifest.get("output_schema", {})
    if in_schema:
        features = in_schema.get("features", [])
        print(f"features : {len(features)} dims — {features}")
    if out_schema:
        print(f"labels   : {out_schema.get('labels', [])}")

    if metrics:
        print("\nmetrics:")
        skip = {"schema_version", "model", "dataset", "sample_count"}
        for key, value in metrics.items():
            if key in skip:
                continue
            print(f"  {key:<28} {value:.6f}" if isinstance(value, float) else f"  {key:<28} {value}")
    else:
        print("\nmetrics : not available (run eval_model.py to generate)")

    return 0


# Metric keys that are meaningful to compare between two artifacts.
_DIFF_METRICS = [
    "mae_target_speed",
    "rmse_target_speed",
    "std_error",
    "mean_error",
    "max_abs_error",
    "p50_abs_error",
    "p90_abs_error",
    "p95_abs_error",
    "p99_abs_error",
]


def cmd_diff(args: argparse.Namespace) -> int:
    dir_a = Path(args.artifact_a)
    dir_b = Path(args.artifact_b)
    for path in (dir_a, dir_b):
        if not path.is_dir():
            raise SystemExit(f"error: artifact directory not found: {path}")

    manifest_a = load_manifest(dir_a)
    manifest_b = load_manifest(dir_b)
    metrics_a = load_metrics(dir_a)
    metrics_b = load_metrics(dir_b)

    def _fmt_backend(m: dict) -> str:
        return f"{m.get('backend', '?')} / {m.get('model_format', '?')}"

    def _fmt_samples(m: dict) -> str:
        d = m.get("dataset", {}) if isinstance(m.get("dataset"), dict) else {}
        return f"{d.get('sample_count', '?')} samples, scenario={d.get('scenario', '?')}"

    col = 32
    print(f"{'':>{col}}  {'A':>14}  {'B':>14}  {'delta':>14}")
    print(f"  {'artifact':<{col-2}}  {dir_a.name:>14}  {dir_b.name:>14}")
    print(f"  {'backend':<{col-2}}  {_fmt_backend(manifest_a):>14}  {_fmt_backend(manifest_b):>14}")
    print(f"  {'dataset':<{col-2}}  {_fmt_samples(manifest_a):>14}  {_fmt_samples(manifest_b):>14}")

    if metrics_a is None and metrics_b is None:
        print("\n  (no metrics.json in either artifact — run eval_model.py)")
        return 0

    print()
    for key in _DIFF_METRICS:
        va = metrics_a.get(key) if metrics_a else None
        vb = metrics_b.get(key) if metrics_b else None
        if va is None and vb is None:
            continue
        sa = f"{va:.4f}" if isinstance(va, float) else "n/a"
        sb = f"{vb:.4f}" if isinstance(vb, float) else "n/a"
        if isinstance(va, float) and isinstance(vb, float):
            delta = vb - va
            sd = f"{delta:+.4f}"
            # Highlight improvement (lower error = better)
            marker = " ✓" if delta < -1e-6 else (" ✗" if delta > 1e-6 else "")
            print(f"  {key:<{col-2}}  {sa:>14}  {sb:>14}  {sd:>14}{marker}")
        else:
            print(f"  {key:<{col-2}}  {sa:>14}  {sb:>14}")

    return 0


def cmd_promote(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact)
    runtime_model = Path(args.runtime_model)
    manifest = load_manifest(artifact_dir)
    backend = manifest.get("backend")
    if backend != "tiny_mlp":
        raise SystemExit(
            f"error: only backend=tiny_mlp artifacts can be promoted to C runtime; got {backend!r}. "
            "Use torch_sidecar.py for PyTorch artifacts."
        )

    source = model_file_for_artifact(artifact_dir, manifest)
    if not source.exists():
        raise SystemExit(f"error: model file not found: {source}")

    runtime_model.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, runtime_model)
    print(f"promoted {source} -> {runtime_model}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine model artifact manager")
    sub = parser.add_subparsers(dest="command", required=True)

    list_parser = sub.add_parser("list", help="Show runtime model and known artifacts")
    list_parser.add_argument("--runtime-model", default=str(DEFAULT_RUNTIME_MODEL))
    list_parser.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    list_parser.set_defaults(func=cmd_list)

    inspect_parser = sub.add_parser("inspect", help="Show detailed manifest and metrics for an artifact")
    inspect_parser.add_argument("artifact", help="Artifact directory containing manifest.json")
    inspect_parser.set_defaults(func=cmd_inspect)

    diff_parser = sub.add_parser("diff", help="Compare metrics between two artifacts")
    diff_parser.add_argument("artifact_a", help="Baseline artifact directory")
    diff_parser.add_argument("artifact_b", help="New artifact directory to compare against baseline")
    diff_parser.set_defaults(func=cmd_diff)

    promote_parser = sub.add_parser("promote", help="Promote a tiny-MLP artifact to the C runtime model")
    promote_parser.add_argument("artifact", help="Artifact directory containing manifest.json and model.txt")
    promote_parser.add_argument("--runtime-model", default=str(DEFAULT_RUNTIME_MODEL))
    promote_parser.set_defaults(func=cmd_promote)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
