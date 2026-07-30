#!/usr/bin/env python3
"""trace_incident.py — 事故逐层追溯工具

读 /tmp/flow_topology.json，对指定时间点或最近一次事故，
dump 每一层在那一刻的输入/输出，快速定位哪个模块出了问题。

用法:
    python3 tools/trace_incident.py                    # 最近一次事故（road_departure / collision）
    python3 tools/trace_incident.py --at 17.6          # 指定时间戳
    python3 tools/trace_incident.py --json /tmp/snap.json  # 指定快照
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_JSON = Path("/tmp/flow_topology.json")


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def lane_idx_from_y(y: float, lc: int, lw: float) -> int:
    off = -y / lw + (lc - 1) * 0.5
    return max(0, min(lc - 1, int(round(off))))


def lane_center_y(idx: int, lc: int, lw: float) -> float:
    return -(idx - (lc - 1) * 0.5) * lw


def find_incident(samples: list[dict], data: dict) -> dict | None:
    """从 samples 和 topology 中找最近一次事故。"""
    # Check collision in topic stats
    for t in data.get("metrics", {}).get("topics", []):
        if t["topic"] == "sim/collision" and t["pub"] > 0:
            return {"type": "collision", "topic_pub": t["pub"], "at_s": "unknown (log only)"}

    # Check road departure from samples
    scene = data.get("metrics", {}).get("scene", {})
    lane = scene.get("lane", {})
    lc = int(lane.get("count", 4))
    lw = float(lane.get("width", 3.5))
    half_road = lc * lw * 0.5

    if len(samples) >= 5:
        ys = [s.get("y", 0) for s in samples]
        xs = [s.get("x", 0) for s in samples]
        ts = [s.get("t", 0) for s in samples]

        # Find when ego left the road
        for i in range(len(ys)):
            if abs(ys[i]) > half_road:
                return {
                    "type": "road_departure",
                    "at_s": ts[i] - ts[0] if ts else 0,
                    "y": ys[i],
                    "speed": samples[i].get("speed", 0),
                    "index": i,
                }
    return None


def print_layer(label: str, items: list[tuple[str, str]], indent: int = 2) -> None:
    pad = " " * indent
    print(f"\n{pad}── {label} ──")
    for key, val in items:
        print(f"{pad}  {key}: {val}")


def trace(data: dict, at_s: float | None = None, verbose: bool = False) -> int:
    metrics = data.get("metrics", {})
    scene = metrics.get("scene", {})
    beh = metrics.get("behavior", {})
    samples = data.get("samples", [])

    # Find incident
    if at_s is None:
        incident = find_incident(samples, data)
    else:
        # Find sample closest to at_s
        best = None
        best_dt = float("inf")
        for s in samples:
            dt = abs(s.get("t", 0) - at_s)
            if dt < best_dt:
                best_dt = dt
                best = s
        if best:
            incident = {"type": "manual", "at_s": at_s, "sample": best}
        else:
            incident = None

    if not incident:
        print("未发现事故")
        print("提示: 用 --at <时间> 指定追溯点，或跑完 demo 后有碰撞/出路沿数据再来")
        return 0

    at_val = incident.get("at_s", "?")
    if isinstance(at_val, (int, float)):
        at_str = f"~{at_val:.1f}s"
    else:
        at_str = str(at_val)
    print(f"\n{'='*60}")
    print(f"  事故类型: {incident['type']}")
    print(f"  发生时间: {at_str}")
    print(f"{'='*60}")

    # Find the sample closest to incident time
    incident_sample = None
    target_t = incident.get("at_s")
    if isinstance(target_t, (int, float)):
        for s in samples:
            if abs(s.get("t", 0) - data.get("timestamp", 0) - target_t) < 0.5:
                incident_sample = s
                break
    if incident_sample is None and samples:
        incident_sample = samples[-1]  # fallback to last

    # ── Layer 1: localzation ──
    ego = scene.get("ego", {})
    lane = scene.get("lane", {})
    lc = int(lane.get("count", 4))
    lw = float(lane.get("width", 3.5))
    half_road = lc * lw * 0.5
    if incident_sample:
        sx = incident_sample.get("x", 0)
        sy = incident_sample.get("y", 0)
        sh = incident_sample.get("heading", 0)
        ss = incident_sample.get("speed", 0)
    else:
        sx, sy, sh, ss = ego.get("x", 0), ego.get("y", 0), ego.get("heading", 0), ego.get("speed", 0)

    li = lane_idx_from_y(sy, lc, lw)
    lcy = lane_center_y(li, lc, lw)
    off_road = abs(sy) > half_road
    print_layer("1. 定位 (Localization)", [
        (f"x={sx:.1f}  y={sy:.2f}  heading={math.degrees(sh):.0f}°", ""),
        (f"speed", f"{ss:.1f} m/s ({ss*3.6:.0f} km/h)"),
        (f"lane", f"idx={li}  center_y={lcy:.2f}  cte={sy-lcy:.2f}m"),
        (f"off_road?", "🚨 YES" if off_road else "OK"),
    ])

    # ── Layer 2: Perception (真值 vs 感知) ──
    obstacles = scene.get("obstacles", [])       # 真值（flowsim 透传）
    entities = scene.get("entities", [])
    beh_obs_count = beh.get("obs_count", 0)       # 感知（perception/obstacles 过来的）
    same_lane_ahead = []
    for o in obstacles:
        wy = sy + o.get("y", 0)
        if o.get("x", 0) > 0 and abs(wy - lcy) < lw * 0.5 + 0.6:
            o["_wy"] = wy
            same_lane_ahead.append(o)

    print_layer("2. 感知 (真值 vs 感知)", [
        (f"truth obstacles", f"{len(obstacles)} (same-lane ahead: {len(same_lane_ahead)})"),
        (f"perceived (behavior.obs_count)", f"{beh_obs_count}"),
        (f"⚠️ 不匹配!" if beh_obs_count < len(obstacles) * 0.5 else "✅ 基本一致", ""),
    ])
    if same_lane_ahead:
        closest = min(same_lane_ahead, key=lambda o: o["x"])
        print(f"    truth closest: id={closest['id']} dx={closest['x']:.0f}m vx={closest.get('vx',0):.1f} type={closest['type']}")
        if verbose:
            for o in sorted(same_lane_ahead, key=lambda o: o["x"])[:5]:
                print(f"      id={o['id']:2d} dx={o['x']:7.1f} vx={o.get('vx',0):.1f} type={o['type']}")
    print(f"    behavior obs_count={beh_obs_count} — 如果 < 真值数量说明 perception 没送出来")

    # ── Layer 3: Behavior planner ──
    print_layer("3. 行为规划 (Behavior)", [
        (f"state", f"{beh.get('state', '?')}"),
        (f"obs_count", f"{beh.get('obs_count', 0)}"),
        (f"best_gap", f"{beh.get('best_gap', -1):.1f} m"),
        (f"blocked", f"{beh.get('blocked', '?')}"),
        (f"worthwhile", f"{beh.get('worthwhile', '?')}"),
        (f"committed_lane", f"{beh.get('committed_lane', '?')}"),
        (f"target_lane", f"{beh.get('target_lane', '?')}"),
        (f"lead_speed", f"{beh.get('lead_speed', 0):.1f} m/s"),
        (f"target_speed", f"{beh.get('target_speed', 0):.1f} m/s"),
        (f"follow_speed", f"{beh.get('follow_speed', 0):.1f} m/s"),
        (f"desired_gap", f"{beh.get('desired_gap', 0):.1f} m"),
        (f"left_ok/right_ok", f"{beh.get('left_ok', '?')} / {beh.get('right_ok', '?')}"),
        (f"left_gap/right_gap", f"{beh.get('left_gap', -1):.0f} / {beh.get('right_gap', -1):.0f}"),
    ])

    # ── Layer 4: Planning trajectory ──
    traj = scene.get("trajectory_path", [])
    print_layer("4. 规划轨迹 (Planning)", [
        (f"points", f"{len(traj)}"),
    ])
    if traj:
        speeds = [p[2] for p in traj]
        print(f"    speed profile: {[f'{s:.1f}' for s in speeds]}")
        print(f"    target_speed (末点): {speeds[-1]:.1f} m/s")

    # ── Layer 5: Control command ──
    vehicle = metrics.get("vehicle", {})
    print_layer("5. 控制输出 (Control)", [
        (f"speed / target", f"{vehicle.get('speed', 0):.1f} / {vehicle.get('target_speed', 0):.1f} m/s"),
        (f"throttle / brake", f"{vehicle.get('throttle', 0):.2f} / {vehicle.get('brake', 0):.2f}"),
        (f"error", f"{vehicle.get('error', 0):.1f} m/s"),
    ])

    # ── Summary: who failed? ──
    print()
    verdicts = []
    if not same_lane_ahead and not off_road:
        verdicts.append("❌ 感知: 同车道无前车（但场景明明有）→ 盲开")
    elif same_lane_ahead and beh.get("state") in ("CRUISE",):
        closest = min(same_lane_ahead, key=lambda o: o["x"])
        if closest["x"] < 100:
            verdicts.append(f"❌ 行为: 前车 dx={closest['x']:.0f}m 但 state=CRUISE → 没进 FOLLOW")
    if beh.get("best_gap", -1) > 0 and beh.get("state") in ("FOLLOW", "LEFT_CHANGE", "RIGHT_CHANGE"):
        if traj and traj[-1][2] > beh.get("target_speed", 0) + 2:
            verdicts.append(f"❌ 规划: target_speed={beh.get('target_speed',0):.1f} 但轨迹末点={traj[-1][2]:.1f} → 不减速")
    if off_road:
        if vehicle.get("speed", 0) > 5:
            verdicts.append(f"🚨 出路沿时速度 {vehicle.get('speed',0):.1f} m/s → 恢复了但太慢")
        else:
            verdicts.append("⚠️ 出路沿但速度低 → 恢复中")

    if not verdicts:
        verdicts.append("✅ 所有模块行为一致，事故因外部因素（场景/初始化）")

    print("  >>> 结论 <<<")
    for v in verdicts:
        print(f"    {v}")
    print()

    return 1 if any("❌" in v for v in verdicts) else 0


def main() -> int:
    ap = argparse.ArgumentParser(description="FlowEngine 事故逐层追溯")
    ap.add_argument("--at", type=float, default=None, help="指定事故时间点 (s)")
    ap.add_argument("--json", type=Path, default=DEFAULT_JSON, help="拓扑JSON路径")
    ap.add_argument("--verbose", "-v", action="store_true", help="显示详细感知数据")
    args = ap.parse_args()

    data = load(args.json)
    return trace(data, at_s=args.at, verbose=args.verbose)


if __name__ == "__main__":
    sys.exit(main())
