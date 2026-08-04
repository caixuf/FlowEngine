#!/usr/bin/env python3
"""Closed-loop Python simulation for the trained temporal-MLP (E2E) model.

Loads models/<name>/model.txt (v2 temporal: in=115 = 5 frames × 23 V3 features,
out=5 = throttle/brake/steer/lane_change/confidence), then drives the bicycle
model on a straight road, feeding features back each step.

Scenarios:
  cruise      straight 4-lane road, no obstacle, accelerate + hold lane
  lead        a slow lead vehicle ahead (model should follow / slow down)
  emergency   a stationary obstacle ahead (model should brake before hitting)

Output: per-step trajectory + summary metrics (final speed, lane deviation,
off-road, min gap / collision).
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import deque
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]                 # tools/
HERE = Path(__file__).resolve().parent                      # tools/train_e2e/
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(HERE))

from temporal_train import load_model                       # noqa: E402
from feature_schema import FEATURE_NAMES_V3, build_v3_features  # noqa: E402
from control_sim import VehicleState, DT, WHEELBASE          # noqa: E402

WINDOW = 5
V3_DIM = 23
ROAD_LANES = 4
LANE_W = 3.5
ROAD_HALF_WIDTH = ROAD_LANES * LANE_W / 2.0                 # 7.0 m
SPEED_LIMIT = 30.0
LANE_CENTER_REF = 0.0                                       # road center = y=0


def lane_center_y(lane_idx: int, n: int = ROAD_LANES, w: float = LANE_W) -> float:
    """lane 2 (0-indexed) center = -1.75 m (matches demo)."""
    return -(lane_idx - (n - 1) / 2.0) * w


class ModelRunner:
    """Temporal window wrapper around the tiny-MLP."""

    def __init__(self, model_path: str):
        self.model, self.norm_mean, self.norm_scale, \
            self.out_mean, self.out_scale = load_model(model_path)
        if self.model.in_dim != WINDOW * V3_DIM:
            raise SystemExit(
                f"model in_dim={self.model.in_dim} != {WINDOW}*{V3_DIM}={WINDOW * V3_DIM}"
            )
        self.history: deque[list[float]] = deque(maxlen=WINDOW)

    def reset(self) -> None:
        self.history.clear()

    def predict(self, features: list[float]) -> list[float]:
        """features: 23-dim list. Returns denormalized 5 outputs."""
        self.history.append(list(features))
        if len(self.history) < WINDOW:
            pad = [list(features)] * (WINDOW - len(self.history))
            frames = pad + list(self.history)
        else:
            frames = list(self.history)[-WINDOW:]

        x = [v for f in frames for v in f]
        xn = [(x[i] - self.norm_mean[i]) / self.norm_scale[i] for i in range(len(x))]
        y = self.model.forward(xn)
        return [y[i] * self.out_scale[i] + self.out_mean[i] for i in range(len(y))]


def build_features(ego, obstacles, brake, emergency_stop) -> list[float]:
    """Assemble the 23-dim V3 feature vector for the current state."""
    control = {"brake": brake, "emergency_stop": bool(emergency_stop)}
    scene_context = {
        "tl_state": -1.0,          # no light in sim
        "tl_distance": -1.0,
        "curvature": 0.0,          # straight road
        "speed_limit": SPEED_LIMIT,
        "lane_count": ROAD_LANES,
        "lane_width": LANE_W,
        "ego_lane_offset": ego["y"] - LANE_CENTER_REF,
    }
    return build_v3_features(ego, obstacles, control, None, scene_context,
                             fallback_features=[])


def front_obstacles(vehicles, ego_x) -> list[dict]:
    """Two nearest vehicles ahead (x >= ego_x), sorted ascending (matches recorder).

    IMPORTANT: the model was trained with 2 obstacles always present (type=1,
    confidence=1). If only 1 (or none) is provided, the missing front0/front1
    slot defaults to 0.0, and because the type/confidence dims have norm_scale
    ~1e-6, the normalized input explodes to ~1e6 → tanh saturates → the model
    outputs a constant. So we always pad to 2 obstacles with a far placeholder
    (type=1, conf=1) so the normalization stays in range.
    """
    ahead = [v for v in vehicles if v["x"] >= ego_x]
    ahead.sort(key=lambda v: v["x"])
    out = list(ahead[:2])
    while len(out) < 2:
        out.append({"x": ego_x + 500.0, "y": 0.0, "vx": 0.0, "vy": 0.0,
                    "type": 1.0, "confidence": 1.0, "length": 4.5, "width": 1.9})
    return out


class SimResult:
    def __init__(self):
        self.t, self.x, self.y, self.v, self.heading = [], [], [], [], []
        self.steer, self.throttle, self.brake = [], [], []
        self.max_speed = 0.0
        self.max_lane_dev = 0.0
        self.min_gap = 1e9
        self.collision = False
        self.off_road = False


def run_sim(model_path: str, scenario: str, duration: float, start_v: float,
            start_lane: int = 2) -> SimResult:
    runner = ModelRunner(model_path)
    ego = VehicleState(x0=0.0, y0=lane_center_y(start_lane), v0=start_v, heading0=0.0)
    res = SimResult()

    start_y = lane_center_y(start_lane)

    # scenario vehicles (absolute frame)
    lead = None
    if scenario == "lead":
        lead = {"x": 40.0, "y": start_y, "vx": 6.0, "vy": 0.0, "type": 1.0,
                "confidence": 1.0, "length": 4.5, "width": 1.9}
    elif scenario == "emergency":
        lead = {"x": 60.0, "y": start_y, "vx": 0.0, "vy": 0.0, "type": 1.0,
                "confidence": 1.0, "length": 4.5, "width": 1.9}

    n_steps = int(duration / DT)
    for step in range(n_steps):
        t = step * DT

        vehicles = [dict(lead)] if lead else []
        if lead:
            lead["x"] += lead["vx"] * DT

        obs_list = front_obstacles(vehicles, ego.x)
        features = build_features(
            {"x": ego.x, "y": ego.y, "v": ego.v, "heading": ego.heading,
             "yaw_rate": ego.yaw_rate},
            obs_list,
            brake=0.0,
            emergency_stop=False,
        )
        throttle, brake, steer, _lc, _conf = runner.predict(features)

        # apply model output to vehicle
        throttle = max(-1.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))
        steer = max(-0.96, min(0.96, steer))

        # record
        res.t.append(t)
        res.x.append(ego.x)
        res.y.append(ego.y)
        res.v.append(ego.v)
        res.heading.append(ego.heading)
        res.steer.append(steer)
        res.throttle.append(throttle)
        res.brake.append(brake)
        res.max_speed = max(res.max_speed, ego.v)
        res.max_lane_dev = max(res.max_lane_dev, abs(ego.y - start_y))

        # collision / min gap
        if lead:
            gap = lead["x"] - ego.x
            res.min_gap = min(res.min_gap, gap)
            if gap < 0.5:
                res.collision = True
                break

        # off road
        if abs(ego.y) > ROAD_HALF_WIDTH + 1.0:
            res.off_road = True
            break

        ego.step(steer, throttle, brake, dt=DT, v_max=SPEED_LIMIT)

    return res


def print_result(res: SimResult, label: str) -> None:
    print(f"\n{'=' * 62}")
    print(f"  {label}")
    print(f"{'=' * 62}")
    if res.collision:
        print(f"  ❌ COLLISION (min_gap={res.min_gap:.1f}m)")
    elif res.off_road:
        print(f"  ❌ OFF-ROAD (|y|>{ROAD_HALF_WIDTH + 1.0}m), final y={res.y[-1]:.2f}m")
    else:
        print(f"  最终位置:      x={res.x[-1]:.1f}m, y={res.y[-1]:.2f}m")
        print(f"  最终速度/最大: {res.v[-1]:.1f} / {res.max_speed:.1f} m/s")
        print(f"  最大车道偏离:  {res.max_lane_dev:.2f} m")
        tail = res.v[-max(1, int(2.0 / DT)):]
        print(f"  末段平均速度:  {sum(tail) / len(tail):.1f} m/s")
        okay = (res.max_lane_dev < 1.5 and res.v[-1] > 1.0)
        print(f"  状态:          {'✅ 稳健' if okay else '⚠️  需检查'}")
    # trajectory summary
    print(f"  轨迹 (每5s):")
    for i in range(0, len(res.t), int(5.0 / DT)):
        print(f"    t={res.t[i]:5.1f}s  x={res.x[i]:6.1f}  y={res.y[i]:6.2f}  "
              f"v={res.v[i]:5.1f}  steer={res.steer[i]:+.3f}  thr={res.throttle[i]:+.2f}  "
              f"brk={res.brake[i]:.2f}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Closed-loop Py sim for the E2E model")
    parser.add_argument("--model", default="models/loop_retry_20260801/model.txt",
                        help="model.txt path")
    parser.add_argument("--scenario", default="cruise",
                        choices=["cruise", "lead", "emergency"])
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--start-v", type=float, default=0.0)
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"model not found: {model_path}")

    print(f"✦ 模型闭环仿真: {model_path}")
    print(f"  场景={args.scenario}  时长={args.duration:.0f}s  初速={args.start_v:.1f}m/s")

    res = run_sim(str(model_path), args.scenario, args.duration, args.start_v)
    print_result(res, f"{args.scenario} 场景")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())