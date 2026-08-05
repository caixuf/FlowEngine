#!/usr/bin/env python3
"""File-bridge PyTorch shadow inference sidecar.

Reads FlowEngine's live state JSON, runs a PyTorch E2E artifact, and writes an
`inference/trajectory`-shaped shadow JSON frame. This keeps the first PyTorch
runtime integration outside the C plugin ABI and ONNX dependency surface.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from sidecar_common import (  # noqa: E402
    DEFAULT_STATE_FILE,
    build_shadow_frame as common_shadow_frame,
    extract_features,
    poll_state,
    read_state,
    write_json_atomic,
)


def require_torch():
    try:
        import torch
        from torch import nn
    except ModuleNotFoundError:
        print(
            "error: PyTorch is not installed. Install torch to use tools/train_e2e/torch_sidecar.py.",
            file=sys.stderr,
        )
        raise SystemExit(2)
    return torch, nn


def load_artifact(model_dir: Path):
    torch, nn = require_torch()
    manifest_path = model_dir / "manifest.json"
    if not manifest_path.exists():
        raise SystemExit(f"error: manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("backend") != "pytorch":
        raise SystemExit(f"error: sidecar expects backend=pytorch, got {manifest.get('backend')!r}")

    model_path = model_dir / manifest.get("model_path", "model.pt")
    checkpoint = torch.load(model_path, map_location="cpu")
    feature_names = checkpoint.get("feature_names", ["ego_v", "ego_y", "ego_heading", "ego_yaw_rate"])
    label_names = checkpoint.get("label_names", ["target_speed"])
    # 2026-08-05: 支持多隐层 "64,32"（torch_train --hidden 升级后）
    hidden_spec = checkpoint.get("hidden", 32)
    if isinstance(hidden_spec, int):
        hidden = [hidden_spec]
    else:
        hidden = [int(x) for x in str(hidden_spec).split(",") if x.strip()]

    layers = []
    prev = len(feature_names)
    for h in hidden:
        layers += [nn.Linear(prev, h), nn.Tanh()]
        prev = h
    layers.append(nn.Linear(prev, len(label_names)))
    model = nn.Sequential(*layers)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    return torch, model, checkpoint, manifest


def predict(torch, model, checkpoint: dict, features: list[float]) -> float:
    input_mean = checkpoint["input_mean"]
    input_scale = checkpoint["input_scale"]
    output_mean = checkpoint["output_mean"]
    output_scale = checkpoint["output_scale"]
    x = [
        (float(features[i]) - input_mean[i]) / input_scale[i]
        for i in range(len(features))
    ]
    with torch.no_grad():
        y_norm = model(torch.tensor([x], dtype=torch.float32))[0][0].item()
    return y_norm * output_scale[0] + output_mean[0]


def build_shadow_frame(torch, model, checkpoint: dict, manifest: dict, state: dict) -> dict:
    feature_names = checkpoint.get("feature_names", ["ego_v", "ego_y", "ego_heading", "ego_yaw_rate"])
    features, ego = extract_features(state, feature_names)
    target_speed = predict(torch, model, checkpoint, features)
    return common_shadow_frame(
        "pytorch-sidecar",
        manifest.get("model_path", "model.pt"),
        target_speed,
        state,
        features,
        ego,
        feature_names,
    )


# 2026-08-05: 累积 shadow 统计（promote 门禁消费 shadow_speed_mae/n，
# 与 C runtime inference_node 的 sidecar 同构）
_stats = {"abs": 0.0, "sq": 0.0, "n": 0}


def run_once(torch, model, checkpoint: dict, manifest: dict, state_file: Path, output: Path) -> None:
    state = read_state(state_file)
    frame = build_shadow_frame(torch, model, checkpoint, manifest, state)
    delta = frame.get("shadow_delta", 0.0)
    _stats["abs"] += abs(delta)
    _stats["sq"] += delta * delta
    _stats["n"] += 1
    n = _stats["n"]
    frame["shadow_n"] = n
    frame["shadow_speed_mae"] = round(_stats["abs"] / n, 4) if n else 0.0
    frame["shadow_speed_rmse"] = round((_stats["sq"] / n) ** 0.5, 4) if n else 0.0
    frame["source"] = "pytorch-sidecar"
    write_json_atomic(output, frame)


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine PyTorch shadow inference sidecar")
    parser.add_argument("--model", required=True, help="PyTorch artifact directory from torch_train.py")
    parser.add_argument("--state-file", default=DEFAULT_STATE_FILE)
    parser.add_argument("--output", default="/tmp/flow_torch_inference.json")
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--once", action="store_true", help="Run one inference and exit")
    args = parser.parse_args()

    torch, model, checkpoint, manifest = load_artifact(Path(args.model))
    state_file = Path(args.state_file)
    output = Path(args.output)

    if args.once:
        run_once(torch, model, checkpoint, manifest, state_file, output)
        return 0

    poll_state(state_file, args.interval, lambda: run_once(torch, model, checkpoint, manifest, state_file, output))


if __name__ == "__main__":
    raise SystemExit(main())