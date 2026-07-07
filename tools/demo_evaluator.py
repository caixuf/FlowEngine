#!/usr/bin/env python3
"""Run and score the FlowEngine demo from observable runtime data.

The evaluator samples /tmp/flow_topology.json while scripts/demo.sh is running
and turns visual complaints such as collision, lane departure, stuck vehicle,
and missing topic data into repeatable PASS/FAIL checks.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_JSON = Path("/tmp/flow_topology.json")
LAUNCHER_STDERR = Path("/tmp/flow_launcher_stderr.txt")

REQUIRED_EDGES = [
    ("sim_world", "vehicle/state", "perception"),
    ("perception", "sensor/lidar", "fusion"),
    ("perception", "sensor/gps", "fusion"),
    ("fusion", "fusion/localization", "planning"),
    ("planning", "planning/trajectory", "control"),
    ("control", "control/raw_cmd", "safety_control"),
    ("safety_control", "control/cmd", "sim_world"),
]

TOPIC_MIN_FREQ = {
    "vehicle/state": 15.0,
    "sensor/lidar": 15.0,
    "sensor/gps": 7.0,
    "fusion/localization": 20.0,
    "planning/trajectory": 5.0,
    "control/raw_cmd": 10.0,
    "control/cmd": 10.0,
}


def load_json(path: Path) -> dict | None:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def topic_map(sample: dict) -> dict:
    topics = sample.get("metrics", {}).get("topics", [])
    return {t.get("topic"): t for t in topics if t.get("topic")}


def node_topic_roles(sample: dict) -> tuple[set[tuple[str, str]], set[tuple[str, str]]]:
    pubs: set[tuple[str, str]] = set()
    subs: set[tuple[str, str]] = set()
    for node in sample.get("nodes", []):
        name = node.get("name")
        for topic in node.get("topics", []):
            topic_name = topic.get("topic")
            role = topic.get("role")
            if not name or not topic_name:
                continue
            if role == "pub":
                pubs.add((name, topic_name))
            elif role == "sub":
                subs.add((name, topic_name))
    return pubs, subs


def nearest_lane_error(y: float, lane_width: float = 3.5) -> float:
    lane_centers = [-lane_width * 0.5, lane_width * 0.5]
    return min(abs(y - center) for center in lane_centers)


def sample_metrics(sample: dict) -> dict:
    metrics = sample.get("metrics", {})
    vehicle = metrics.get("vehicle", {})
    scene = metrics.get("scene", {})
    ego = scene.get("ego", {})
    obstacles = scene.get("obstacles", [])
    lane = scene.get("lane", {})

    speed = float(vehicle.get("speed", ego.get("speed", 0.0)) or 0.0)
    x = float(vehicle.get("x", ego.get("x", 0.0)) or 0.0)
    y = float(ego.get("y", 0.0) or 0.0)
    steer = abs(float(ego.get("steer", 0.0) or 0.0))
    lane_width = float(lane.get("width", 3.5) or 3.5)
    lane_count = int(lane.get("count", 2) or 2)

    min_forward_gap = math.inf
    min_abs_gap = math.inf
    for obs in obstacles:
        rel_x = float(obs.get("x", math.inf) or math.inf)
        rel_y = abs(float(obs.get("y", math.inf) or math.inf))
        length = float(obs.get("len", 4.6) or 4.6)
        width = float(obs.get("wid", 2.0) or 2.0)
        gap_x = abs(rel_x) - (4.6 + length) * 0.5
        gap_y = rel_y - (2.0 + width) * 0.5
        min_abs_gap = min(min_abs_gap, max(gap_x, gap_y))
        if rel_x > 0 and rel_y < 2.5:
            min_forward_gap = min(min_forward_gap, rel_x - (4.6 + length) * 0.5)

    return {
        "speed": speed,
        "x": x,
        "y": y,
        "steer": steer,
        "lane_error": nearest_lane_error(y, lane_width),
        "road_edge_margin": lane_width * lane_count * 0.5 - abs(y) - 1.0,
        "min_forward_gap": min_forward_gap,
        "min_abs_gap": min_abs_gap,
    }


def collect_samples(duration: int, json_file: Path, interval: float) -> tuple[list[dict], int]:
    try:
        json_file.unlink()
    except FileNotFoundError:
        pass

    started_wall = time.time()
    cmd = [str(ROOT / "scripts" / "demo.sh"), "--no-browser", str(duration)]
    proc = subprocess.Popen(
        cmd,
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    samples: list[dict] = []
    started = time.monotonic()
    deadline = started + duration + 12.0
    while proc.poll() is None and time.monotonic() < deadline:
        try:
            if json_file.stat().st_mtime < started_wall:
                time.sleep(interval)
                continue
        except FileNotFoundError:
            time.sleep(interval)
            continue
        sample = load_json(json_file)
        if sample:
            samples.append(sample)
        time.sleep(interval)

    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)
    output = proc.stdout.read() if proc.stdout else ""
    if output:
        print(output.rstrip())
    return samples, proc.returncode or 0


def score(samples: list[dict], launcher_log: Path) -> tuple[list[str], list[str], dict]:
    failures: list[str] = []
    warnings: list[str] = []
    if not samples:
        return ["no topology samples collected"], warnings, {}

    last = samples[-1]
    series = [sample_metrics(s) for s in samples]
    speeds = [m["speed"] for m in series]
    xs = [m["x"] for m in series]
    lane_errors = [m["lane_error"] for m in series]
    road_margins = [m["road_edge_margin"] for m in series]
    steer_values = [m["steer"] for m in series]

    topics = topic_map(last)
    pubs, subs = node_topic_roles(last)
    for pub_node, topic, sub_node in REQUIRED_EDGES:
        if (pub_node, topic) not in pubs or (sub_node, topic) not in subs:
            failures.append(f"missing topology edge {pub_node} --{topic}--> {sub_node}")

    for topic, min_freq in TOPIC_MIN_FREQ.items():
        actual = float(topics.get(topic, {}).get("freq", 0.0) or 0.0)
        if actual < min_freq:
            failures.append(f"topic {topic} freq too low: {actual:.1f} Hz < {min_freq:.1f} Hz")

    collision_pub = int(topics.get("sim/collision", {}).get("pub", 0) or 0)
    log_text = launcher_log.read_text(encoding="utf-8", errors="ignore") if launcher_log.exists() else ""
    collision_log_count = len(re.findall(r"COLLISION", log_text))
    if collision_pub > 0 or collision_log_count > 0:
        failures.append(f"collision detected: topic_pub={collision_pub}, log_count={collision_log_count}")

    max_lane_index = max(range(len(series)), key=lambda i: lane_errors[i])
    max_lane_error = lane_errors[max_lane_index]
    min_road_margin_index = min(range(len(series)), key=lambda i: road_margins[i])
    min_road_margin = road_margins[min_road_margin_index]
    if min_road_margin < 0.0:
        failures.append(f"road departure: ego body exceeded road edge by {-min_road_margin:.2f} m")
    if max_lane_error > 1.35:
        warnings.append(f"large lane-center deviation during maneuver: {max_lane_error:.2f} m")

    progress = xs[-1] - xs[0] if len(xs) >= 2 else 0.0
    if progress < 10.0:
        failures.append(f"vehicle stuck or no progress: x delta {progress:.1f} m")

    avg_speed = statistics.fmean(speeds) if speeds else 0.0
    if avg_speed < 1.0:
        failures.append(f"average speed too low: {avg_speed:.1f} m/s")
    if max(speeds) > 25.0:
        failures.append(f"unrealistic speed spike: max speed {max(speeds):.1f} m/s")

    steer_saturation_ratio = sum(1 for s in steer_values if s > 0.219) / max(1, len(steer_values))
    if steer_saturation_ratio > 0.45:
        warnings.append(f"steer saturated often: {steer_saturation_ratio * 100:.0f}% samples")

    drops = sum(int(t.get("drop", 0) or 0) for t in topics.values())
    if drops > 0:
        failures.append(f"message drops detected: {drops}")

    summary = {
        "samples": len(samples),
        "duration_s": max(0.0, samples[-1].get("timestamp", 0) - samples[0].get("timestamp", 0)),
        "x_delta_m": progress,
        "avg_speed_mps": avg_speed,
        "max_speed_mps": max(speeds),
        "max_lane_error_m": max_lane_error,
        "max_lane_error_at_s": max(0.0, samples[max_lane_index].get("timestamp", 0) - samples[0].get("timestamp", 0)),
        "max_lane_error_y": series[max_lane_index]["y"],
        "max_lane_error_speed_mps": series[max_lane_index]["speed"],
        "min_road_margin_m": min_road_margin,
        "min_road_margin_at_s": max(0.0, samples[min_road_margin_index].get("timestamp", 0) - samples[0].get("timestamp", 0)),
        "steer_saturation_ratio": steer_saturation_ratio,
        "collision_topic_pub": collision_pub,
        "topic_freq_hz": {topic: float(topics.get(topic, {}).get("freq", 0.0) or 0.0) for topic in TOPIC_MIN_FREQ},
    }
    return failures, warnings, summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate FlowEngine demo behavior.")
    parser.add_argument("--duration", type=int, default=15, help="demo run duration in seconds")
    parser.add_argument("--interval", type=float, default=0.25, help="JSON sample interval in seconds")
    parser.add_argument("--json-file", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--no-run", action="store_true", help="evaluate current JSON/logs without starting demo.sh")
    args = parser.parse_args()

    if args.no_run:
        sample = load_json(args.json_file)
        samples = [sample] if sample else []
        returncode = 0
    else:
        samples, returncode = collect_samples(args.duration, args.json_file, args.interval)
        if returncode != 0:
            print(f"demo.sh exited with code {returncode}")

    failures, warnings, summary = score(samples, LAUNCHER_STDERR)

    print("\n=== FlowEngine Demo Evaluation ===")
    for key, value in summary.items():
        if key == "topic_freq_hz":
            print("topic_freq_hz:")
            for topic, freq in value.items():
                print(f"  {topic}: {freq:.1f}")
        elif isinstance(value, float):
            print(f"{key}: {value:.3f}")
        else:
            print(f"{key}: {value}")

    if warnings:
        print("\nWARN:")
        for warning in warnings:
            print(f"  - {warning}")

    if failures:
        print("\nFAIL:")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    print("\nPASS: demo behavior is within the current regression envelope")
    return 0


if __name__ == "__main__":
    sys.exit(main())