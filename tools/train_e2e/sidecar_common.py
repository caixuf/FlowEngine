"""Shared helpers for FlowEngine file-bridge inference sidecars."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

from feature_schema import FEATURE_NAMES_V1, features_from_state


DEFAULT_STATE_FILE = os.environ.get("FLOWENGINE_STATE_FILE", "/tmp/flow_topology.json")


def read_state(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as src:
        return json.load(src)


def extract_features(state: dict, feature_names: list[str] | None = None) -> tuple[list[float], dict]:
    return features_from_state(state, feature_names or FEATURE_NAMES_V1)


def build_shadow_frame(backend: str, model_name: str, target_speed: float,
                       state: dict, features: list[float], ego: dict,
                       feature_names: list[str] | None = None) -> dict:
    target_speed = max(0.0, min(40.0, float(target_speed)))
    planning_target = state.get("metrics", {}).get("vehicle", {}).get("target_speed")
    shadow_delta = 0.0 if planning_target is None else target_speed - float(planning_target)
    return {
        "topic": "inference/trajectory",
        "type": "inference",
        "backend": backend,
        "model": model_name,
        "shadow": True,
        "target_speed": round(target_speed, 3),
        "lateral_d": 0.0,
        "shadow_delta": round(shadow_delta, 3),
        "ego": ego,
        "features": features,
        "feature_names": feature_names or FEATURE_NAMES_V1,
        "source_timestamp": state.get("timestamp"),
        "sidecar_timestamp": time.time(),
    }


def write_json_atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def poll_state(state_file: Path, interval: float, callback) -> None:
    last_mtime = None
    while True:
        try:
            mtime = state_file.stat().st_mtime
            if mtime != last_mtime:
                last_mtime = mtime
                callback()
        except FileNotFoundError:
            pass
        except json.JSONDecodeError:
            pass
        time.sleep(max(interval, 0.01))