#!/usr/bin/env python3
"""Zero-dependency tiny-MLP shadow inference sidecar.

Reads FlowEngine's live state JSON, runs a FlowEngine tiny-MLP artifact, and
writes an `inference/trajectory`-shaped shadow JSON frame.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from sidecar_common import (  # noqa: E402
    DEFAULT_STATE_FILE,
    build_shadow_frame,
    extract_features,
    poll_state,
    read_state,
    write_json_atomic,
)


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


def resolve_model_path(model_arg: Path) -> tuple[Path, str]:
    if model_arg.is_dir():
        manifest_path = model_arg / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
        return model_arg / manifest.get("model_path", "model.txt"), manifest.get("model_path", "model.txt")
    return model_arg, model_arg.name


def run_once(model: dict, model_name: str, state_file: Path, output: Path) -> None:
    state = read_state(state_file)
    features, ego = extract_features(state)
    target_speed = predict(model, features)
    frame = build_shadow_frame("tiny-mlp-sidecar", model_name, target_speed, state, features, ego)
    write_json_atomic(output, frame)


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine tiny-MLP shadow inference sidecar")
    parser.add_argument("--model", required=True, help="tiny-MLP artifact directory or model.txt path")
    parser.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    parser.add_argument("--output", default="/tmp/flow_tiny_inference.json")
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    model_path, model_name = resolve_model_path(Path(args.model))
    model = load_tiny_mlp(model_path)
    state_file = Path(args.state_file)
    output = Path(args.output)

    if args.once:
        run_once(model, model_name, state_file, output)
        return 0

    poll_state(state_file, args.interval, lambda: run_once(model, model_name, state_file, output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())