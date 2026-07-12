#!/usr/bin/env python3
"""One-command training recipe for FlowEngine demo recorder samples."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = Path("/tmp/flow_train_samples.jsonl")
DEFAULT_DATASETS_DIR = ROOT / "datasets"
DEFAULT_MODELS_DIR = ROOT / "models"


def run_command(command: list[str]) -> None:
    subprocess.run(command, cwd=ROOT, check=True)


def default_name(backend: str) -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return f"demo_{backend}_{stamp}"


def ensure_samples(input_path: Path, run_demo: int | None) -> None:
    if run_demo is not None:
        run_command(["bash", "scripts/demo.sh", "--no-browser", str(run_demo)])
    if not input_path.exists():
        raise SystemExit(
            f"error: sample file not found: {input_path}. Run `bash scripts/demo.sh --no-browser 30` "
            "first, or pass `--run-demo 30`."
        )
    if input_path.stat().st_size == 0:
        raise SystemExit(f"error: sample file is empty: {input_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Train a FlowEngine model from current demo recorder samples")
    parser.add_argument("--input", default=str(DEFAULT_INPUT), help="Recorder JSONL path")
    parser.add_argument("--datasets-dir", default=str(DEFAULT_DATASETS_DIR))
    parser.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    parser.add_argument("--name", default=None, help="Dataset/model name under datasets/ and models/")
    parser.add_argument("--backend", choices=["tiny", "torch", "temporal"], default="torch")
    parser.add_argument("--scenario", default="demo")
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--hidden", type=int, default=None)
    parser.add_argument("--run-demo", type=int, default=None, metavar="SECONDS", help="Run demo.sh first")
    parser.add_argument("--init-from", default=None, help="Existing torch artifact to fine-tune from")
    args = parser.parse_args()

    backend = args.backend
    if args.init_from and backend != "torch":
        raise SystemExit("error: --init-from is only supported for --backend torch")
    name = args.name or default_name(backend)
    input_path = Path(args.input)
    dataset_dir = Path(args.datasets_dir) / name
    model_dir = Path(args.models_dir) / name

    ensure_samples(input_path, args.run_demo)

    if backend == "temporal":
        # temporal_train.py reads JSONL directly; no dataset export step needed.
        model_dir.mkdir(parents=True, exist_ok=True)
        model_txt = model_dir / "model.txt"
        train_cmd = [
            sys.executable,
            "tools/train_e2e/temporal_train.py",
            "--input", str(input_path),
            "--output", str(model_txt),
        ]
        if args.epochs is not None:
            train_cmd.extend(["--epochs", str(args.epochs)])
        if args.hidden is not None:
            train_cmd.extend(["--hidden", str(args.hidden)])
        run_command(train_cmd)

        # Write a minimal manifest so modelctl.py and eval_model.py can find the artifact.
        import json, time as _time  # noqa: E401
        manifest = {
            "artifact_version": "flowengine.e2e_artifact.v1",
            "created_unix_ms": int(_time.time() * 1000),
            "model_format": "flowengine-tinymlp-v2",
            "model_path": "model.txt",
            "backend": "tiny_mlp",
            "input_schema": {"features": "80-dim temporal (5x16)"},
            "output_schema": {"labels": ["throttle", "brake", "steer", "lane_change", "confidence"]},
            "dataset": {
                "path": str(input_path),
                "schema_version": "jsonl-raw",
                "sample_count": sum(1 for _ in open(input_path)),
                "scenario": args.scenario,
            },
            "training": {
                "epochs": args.epochs or 300,
                "hidden": args.hidden or 32,
            },
        }
        (model_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )

        print("\nFlowEngine temporal training recipe complete")
        print(f"  model:   {model_dir}")
        print(f"  promote: python3 tools/modelctl.py promote {model_dir}")
        print("  next: python3 tools/modelctl.py list")
        return 0

    # ── tiny / torch backend (original path) ─────────────────────────────────
    run_command(
        [
            sys.executable,
            "tools/dataset/export_e2e_dataset.py",
            "--input",
            str(input_path),
            "--output",
            str(dataset_dir),
            "--scenario",
            args.scenario,
        ]
    )

    train_cmd = [
        sys.executable,
        "tools/train_e2e/torch_train.py" if backend == "torch" else "tools/train_e2e/train.py",
        "--dataset",
        str(dataset_dir),
        "--output",
        str(model_dir),
    ]
    if args.epochs is not None:
        train_cmd.extend(["--epochs", str(args.epochs)])
    if args.hidden is not None:
        train_cmd.extend(["--hidden", str(args.hidden)])
    if args.init_from:
        train_cmd.extend(["--init-from", str(args.init_from)])
    run_command(train_cmd)

    metrics_path = model_dir / "metrics.json"
    run_command(
        [
            sys.executable,
            "tools/eval_model.py",
            "--model",
            str(model_dir),
            "--dataset",
            str(dataset_dir),
            "--output",
            str(metrics_path),
        ]
    )

    print("\nFlowEngine training recipe complete")
    print(f"  dataset: {dataset_dir}")
    print(f"  model:   {model_dir}")
    print(f"  metrics: {metrics_path}")
    if backend == "tiny":
        print(f"  promote: python3 tools/modelctl.py promote {model_dir}")
    else:
        print(
            "  sidecar: python3 tools/train_e2e/torch_sidecar.py "
            f"--model {model_dir} --state-file /tmp/flow_topology.json --output /tmp/flow_torch_inference.json"
        )
    print("  next: python3 tools/modelctl.py list")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
