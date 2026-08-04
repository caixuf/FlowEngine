#!/usr/bin/env python3
"""Formal closed-loop safety evaluator for the E2E (temporal-MLP) model.

Upgrades the ad-hoc `test_model_sim.py` probe into a **gate with teeth**:

  - Runs a scenario matrix in the Python bicycle simulator (no C++ build needed).
  - Scores each scenario on safety metrics (collision / off-road / backward /
    no-progress / lane-hold / speed-convergence / jerk / NaN).
  - Emits a machine-readable `closed_loop_eval.json` (PASS/FAIL) that
    `modelctl promote` requires before a model can reach the C runtime.

Why this matters (Phase 0 of the learning-loop hardening):
  The old promote gate only looked at imitation MAE (shadow speed vs planning).
  A model can fit the teacher's *speed* well yet still drive backward / off-road /
  collide in closed loop — the current 2026-08 model does exactly that (throttle<0,
  v<0, y drifts off-lane). This evaluator makes "usable" a hard, quantifiable gate.

Usage:
  python3 tools/train_e2e/eval_closed_loop.py \
      --model models/loop_retry_20260801/model.txt \
      --output models/loop_retry_20260801/closed_loop_eval.json
  # exit 0 = PASS, 1 = any scenario FAIL

Scenario matrix:
  cruise     straight 4-lane, no obstacle — must accelerate forward & hold lane
  lead       slow lead vehicle — must follow, keep gap, no rear-end
  emergency  stationary obstacle — must brake before impact
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(HERE))

from test_model_sim import (  # noqa: E402
    ModelRunner,
    build_features,
    front_obstacles,
    lane_center_y,
    ROAD_HALF_WIDTH,
    SPEED_LIMIT,
    LANE_CENTER_REF,
)
from control_sim import VehicleState, DT, WHEELBASE  # noqa: E402

# ── 场景与阈值 ────────────────────────────────────────────────
# 每个场景可覆盖启动参数；阈值与 C 侧 demo_evaluator 语义对齐。

SAFE_GAP = 0.5            # 碰撞判定：距前车最小间距 (m)
BACKWARD_V = -0.5         # 低于此速度 = 倒车（正油门应为正向前进）
OFFROAD_MARGIN = 1.0      # 超出路沿的容忍 (m)
JERK_MAX = 12.0           # 最大 jerk m/s³（运动学可接受上限）

# 逐场景阈值
SCENARIO_GATES = {
    "cruise": {
        "duration": 30.0,
        "start_v": 0.0,
        "start_lane": 2,
        "min_progress": 30.0,     # 至少前进 30m（不应倒车/原地）
        "min_final_v": 2.0,       # 不应卡死在 0
        "max_lane_dev": 2.0,      # 车道保持（WARN 级）
    },
    "lead": {
        "duration": 30.0,
        "start_v": 0.0,
        "start_lane": 2,
        "lead_v": 6.0,
        "lead_dist": 40.0,
        "min_progress": 20.0,
        "speed_converge_tol": 4.0,  # 末速应接近前车速度
        "max_lane_dev": 2.0,
    },
    "emergency": {
        "duration": 30.0,
        "start_v": 0.0,
        "start_lane": 2,
        "lead_v": 0.0,
        "lead_dist": 60.0,
        "min_progress": 20.0,
        "max_final_v": 2.0,         # 急停场景末速应接近 0
        "max_lane_dev": 2.0,
    },
}


class Recorder:
    """Per-step trajectory recorder + derived safety metrics."""

    def __init__(self, start_lane: int):
        self.start_lane = start_lane
        self.lane_ref = lane_center_y(start_lane)
        self.t, self.x, self.y, self.v, self.heading = [], [], [], [], []
        self.steer, self.throttle, self.brake = [], [], []
        self.min_gap = 1e9
        self.collision = False
        self.off_road = False
        self.any_nan = False
        # derived
        self.max_speed = 0.0
        self.min_speed = 0.0
        self.max_lane_dev = 0.0
        self.max_jerk = 0.0

    def record(self, t, x, y, v, heading, steer, throttle, brake):
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.v.append(v)
        self.heading.append(heading)
        self.steer.append(steer)
        self.throttle.append(throttle)
        self.brake.append(brake)
        self.max_speed = max(self.max_speed, v)
        self.min_speed = min(self.min_speed, v)
        self.max_lane_dev = max(self.max_lane_dev, abs(y - self.lane_ref))
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(v):
            self.any_nan = True

    def finalize(self):
        """Compute jerk from the velocity trace."""
        if len(self.v) >= 2:
            jerk = [abs((self.v[i] - self.v[i - 1]) / DT) for i in range(1, len(self.v))]
            self.max_jerk = max(jerk)
        return self.max_jerk

    def progress(self, start_x: float) -> float:
        return self.x[-1] - start_x if self.x else 0.0


def run_scenario(model_path: str, name: str) -> tuple[Recorder, dict]:
    """Run one closed-loop scenario, return (recorder, metrics)."""
    cfg = SCENARIO_GATES[name]
    runner = ModelRunner(model_path)
    start_y = lane_center_y(cfg["start_lane"])
    ego = VehicleState(x0=0.0, y0=start_y, v0=cfg["start_v"], heading0=0.0)
    rec = Recorder(cfg["start_lane"])

    # scenario lead vehicle (absolute frame)
    lead = None
    if "lead_dist" in cfg:
        lead = {"x": cfg["lead_dist"], "y": start_y, "vx": cfg.get("lead_v", 0.0),
                "vy": 0.0, "type": 1.0, "confidence": 1.0,
                "length": 4.5, "width": 1.9}

    n_steps = int(cfg["duration"] / DT)
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
        throttle = max(-1.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))
        steer = max(-0.96, min(0.96, steer))

        rec.record(t, ego.x, ego.y, ego.v, ego.heading, steer, throttle, brake)

        if lead:
            gap = lead["x"] - ego.x
            rec.min_gap = min(rec.min_gap, gap)
            if gap < SAFE_GAP:
                rec.collision = True
                break
        if abs(ego.y) > ROAD_HALF_WIDTH + OFFROAD_MARGIN:
            rec.off_road = True
            break

        ego.step(steer, throttle, brake, dt=DT, v_max=SPEED_LIMIT)

    rec.finalize()
    metrics = {
        "final_x": rec.x[-1] if rec.x else 0.0,
        "final_y": rec.y[-1] if rec.y else 0.0,
        "final_v": rec.v[-1] if rec.v else 0.0,
        "max_speed": rec.max_speed,
        "min_speed": rec.min_speed,
        "progress": round(rec.progress(0.0), 2),
        "max_lane_dev": round(rec.max_lane_dev, 3),
        "min_gap": round(rec.min_gap, 2) if lead else None,
        "max_jerk": round(rec.max_jerk, 2),
        "collision": rec.collision,
        "off_road": rec.off_road,
        "backward": rec.min_speed < BACKWARD_V,
        "any_nan": rec.any_nan,
    }
    return rec, metrics


def score_scenario(name: str, m: dict) -> tuple[bool, list[str]]:
    """Return (passed, gate_failure_reasons)."""
    cfg = SCENARIO_GATES[name]
    fails: list[str] = []

    if m["any_nan"]:
        fails.append("NaN/inf 出现在状态量中")
    if m["collision"]:
        fails.append(f"碰撞（min_gap={m['min_gap']:.1f}m < {SAFE_GAP}m）")
    if m["off_road"]:
        fails.append(f"冲出路面（|y|>{ROAD_HALF_WIDTH + OFFROAD_MARGIN}m）")
    if m["backward"]:
        fails.append(f"倒车（min_speed={m['min_speed']:.1f}m/s < {BACKWARD_V}）")
    if m["progress"] < cfg["min_progress"]:
        fails.append(f"前进不足（progress={m['progress']:.1f}m < {cfg['min_progress']}m）")
    if m["max_jerk"] > JERK_MAX:
        fails.append(f"jerk 过大（{m['max_jerk']:.1f} > {JERK_MAX}）")
    if name == "cruise" and m["final_v"] < cfg["min_final_v"]:
        fails.append(f"巡航末速卡死（final_v={m['final_v']:.1f} < {cfg['min_final_v']}）")
    if name == "emergency" and m["final_v"] > cfg["max_final_v"]:
        fails.append(f"急停未刹停（final_v={m['final_v']:.1f} > {cfg['max_final_v']}）")
    if name == "lead" and m["final_v"] is not None:
        if abs(m["final_v"] - cfg["lead_v"]) > cfg["speed_converge_tol"]:
            fails.append(f"跟车未收敛（final_v={m['final_v']:.1f} vs lead={cfg['lead_v']}）")

    return (len(fails) == 0, fails)


def print_report(model_path: str, results: dict) -> None:
    print(f"\n✦ 模型闭环评估: {model_path}")
    print(f"{'=' * 74}")
    header = f"{'场景':<14}{'结果':<8}{'progress':>10}{'末速':>8}{'max|y|':>9}{'min_gap':>10}{'jerk':>8}"
    print(header)
    print('-' * 74)
    for name, r in results.items():
        m = r["metrics"]
        mg = '—' if m["min_gap"] is None else f'{m["min_gap"]:.1f}'
        print(f"{name:<14}{r['result']:<8}{m['progress']:>10.1f}{m['final_v']:>8.1f}"
              f"{m['max_lane_dev']:>9.2f}{mg:>10}{m['max_jerk']:>8.1f}")
        for gate in r["gates"]:
            print(f"    ✗ {gate}")
    print('-' * 74)


def main() -> int:
    parser = argparse.ArgumentParser(description="Closed-loop safety evaluator for E2E model")
    parser.add_argument("--model", default="models/loop_retry_20260801/model.txt")
    parser.add_argument("--scenarios", default="cruise,lead,emergency",
                        help="comma-separated scenario names")
    parser.add_argument("--output", default=None,
                        help="write closed_loop_eval.json to this path")
    parser.add_argument("--json", action="store_true",
                        help="print machine-readable JSON to stdout")
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"model not found: {model_path}")

    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    for s in scenarios:
        if s not in SCENARIO_GATES:
            raise SystemExit(f"unknown scenario: {s} (known: {list(SCENARIO_GATES)})")

    results = {}
    all_pass = True
    for name in scenarios:
        rec, metrics = run_scenario(str(model_path), name)
        passed, gates = score_scenario(name, metrics)
        results[name] = {
            "result": "PASS" if passed else "FAIL",
            "metrics": metrics,
            "gates": gates,
        }
        if not passed:
            all_pass = False

    print_report(str(model_path), results)
    overall = "PASS" if all_pass else "FAIL"
    print(f"  总体: {overall}")

    summary = {
        "schema_version": "flowengine.e2e_closed_loop.v1",
        "model": str(model_path),
        "created_unix_ms": int(time.time() * 1000),
        "evaluator_result": overall,
        "scenarios": results,
    }
    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"  已写入: {out}")
    if args.json:
        print(json.dumps(summary, ensure_ascii=False))

    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())