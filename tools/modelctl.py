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

    promote_parser = sub.add_parser("promote", help="Promote a tiny-MLP artifact to the C runtime model")
    promote_parser.add_argument("artifact", help="Artifact directory containing manifest.json and model.txt")
    promote_parser.add_argument("--runtime-model", default=str(DEFAULT_RUNTIME_MODEL))
    promote_parser.set_defaults(func=cmd_promote)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
