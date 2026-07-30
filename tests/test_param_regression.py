#!/usr/bin/env python3
"""test_param_regression.py — 调参回归检查。

对当前参数运行 N 秒的评估，计算 CTE RMS / yaw_rate RMS / steer flip rate
等核心指标，与保存的 baseline 对比，判断参数更改是改善还是退化。

用法:
    # 保存当前参数为 baseline（在 demo 运行时）
    python3 tests/test_param_regression.py --save-baseline

    # 改参数后，检查退化
    python3 tests/test_param_regression.py

    # 指定采样时长
    python3 tests/test_param_regression.py --duration 30

    # 指定退化阈值（百分比）
    python3 tests/test_param_regression.py --threshold 20

退出码: 0=PASS(无退化), 1=WARN(轻微退化), 2=FAIL(显著退化)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
BASELINE_FILE = ROOT / ".param_baseline.json"
JSON_FILE = Path("/tmp/flow_topology.json")
FLOWCTL = ROOT / "build" / "bin" / "flowctl"

# 默认退化阈值（百分比）：当前值劣于 baseline 的此比例时告警/失败
DEFAULT_WARN_THRESHOLD = 15.0   # 退化 ≥15% → WARN
DEFAULT_FAIL_THRESHOLD = 30.0   # 退化 ≥30% → FAIL


def read_metrics() -> dict | None:
    """从拓扑 JSON 读取当前帧 ego 数据。"""
    try:
        if not JSON_FILE.exists():
            return None
        with open(JSON_FILE) as f:
            d = json.load(f)
        scene = d.get("metrics", {}).get("scene", {})
        ego = scene.get("ego", {})
        lane = scene.get("lane", {})
        metrics = d.get("metrics", {})
        veh = metrics.get("vehicle", {})
        if not ego:
            return None
        return {
            "x": float(ego.get("x", 0)),
            "y": float(ego.get("y", 0)),
            "heading": float(ego.get("heading", 0)),
            "speed": float(ego.get("speed", 0)),
            "steer": float(ego.get("steer", 0)),
            "t": float(d.get("timestamp", 0)),
            "lane_width": float(lane.get("width", 3.5)),
            "lane_count": int(lane.get("count", 4)),
            "behavior_state": metrics.get("behavior", {}).get("state", ""),
        }
    except (json.JSONDecodeError, OSError, KeyError):
        return None


def compute_cte(y: float, lane_count: int, lane_width: float) -> float:
    """计算到最近车道中心的横向偏差。"""
    offset = -y / lane_width + (lane_count - 1) * 0.5
    idx = max(0, min(lane_count - 1, int(round(offset))))
    lcy = -(idx - (lane_count - 1) * 0.5) * lane_width
    return y - lcy


def collect_samples(duration_sec: float) -> list[dict]:
    """采样 N 秒的 ego 数据。"""
    samples: list[dict] = []
    t0 = time.monotonic()
    deadline = t0 + duration_sec + 10.0
    while time.monotonic() - t0 < duration_sec and time.monotonic() < deadline:
        m = read_metrics()
        if m:
            samples.append(m)
        time.sleep(0.1)  # 10Hz 采样

    if len(samples) < 5:
        print(f"   采样不足: {len(samples)} 帧 (需要 ≥5)")
        return []

    return samples


def analyze(samples: list[dict]) -> dict:
    """从采样序列计算统计指标。"""
    speeds = [s["speed"] for s in samples]
    ys = [s["y"] for s in samples]
    headings = [s["heading"] for s in samples]
    steers = [s["steer"] for s in samples]
    timestamps = [s["t"] for s in samples]

    lane_count = samples[0].get("lane_count", 4)
    lane_width = samples[0].get("lane_width", 3.5)

    # CTE
    ctes = [compute_cte(s["y"], s.get("lane_count", lane_count),
                        s.get("lane_width", lane_width)) for s in samples]
    cte_rms = math.sqrt(statistics.fmean(c * c for c in ctes)) if ctes else 999

    # 横向摆幅
    if len(ys) >= 3:
        y_range = max(ys) - min(ys)
    else:
        y_range = 0.0

    # Yaw rate
    yaw_rates = [0.0]
    for i in range(1, len(headings)):
        dt = timestamps[i] - timestamps[i - 1]
        if dt > 0 and dt < 1.0:
            dh = headings[i] - headings[i - 1]
            while dh > math.pi:
                dh -= 2.0 * math.pi
            while dh < -math.pi:
                dh += 2.0 * math.pi
            yaw_rates.append(abs(dh) / dt)
        else:
            yaw_rates.append(0.0)
    yaw_rate_rms = math.sqrt(statistics.fmean(
        y * y for y in yaw_rates)) if yaw_rates else 999

    # Steer flip rate
    flip_count = 0
    prev_steer = steers[0] if steers else 0
    for s in steers[1:]:
        if prev_steer * s < 0 and abs(s - prev_steer) > 0.01:
            flip_count += 1
        prev_steer = s
    duration = timestamps[-1] - timestamps[0] if len(timestamps) >= 2 else 1.0
    flip_rate = flip_count / max(duration, 0.01)

    # Steer rate
    steer_rates = [0.0]
    for i in range(1, len(steers)):
        dt = timestamps[i] - timestamps[i - 1]
        if dt > 0 and dt < 1.0:
            steer_rates.append(abs(steers[i] - steers[i - 1]) / dt)
        else:
            steer_rates.append(0.0)
    steer_rate_rms = math.sqrt(statistics.fmean(
        s * s for s in steer_rates)) if steer_rates else 999

    # Avg speed
    avg_speed = statistics.fmean(speeds) if speeds else 0.0

    # 行为状态分布
    state_counts: dict[str, int] = {}
    for s in samples:
        st = s.get("behavior_state", "")
        if st:
            state_counts[st] = state_counts.get(st, 0) + 1

    return {
        "cte_rms": cte_rms,
        "y_range": y_range,
        "yaw_rate_rms": yaw_rate_rms,
        "steer_rate_rms": steer_rate_rms,
        "flip_rate": flip_rate,
        "avg_speed": avg_speed,
        "n_samples": len(samples),
        "duration": duration,
        "state_counts": state_counts,
    }


def save_baseline(results: dict) -> None:
    """保存当前结果为 baseline。"""
    baseline = {k: v for k, v in results.items()
                if isinstance(v, (int, float))}
    baseline["timestamp"] = time.time()
    with open(BASELINE_FILE, "w") as f:
        json.dump(baseline, f, indent=2)
    print(f"\n  Baseline saved to {BASELINE_FILE}")
    print(f"  cte_rms={results['cte_rms']:.4f}  "
          f"yaw_rate_rms={results['yaw_rate_rms']:.4f}  "
          f"flip_rate={results['flip_rate']:.2f}")


def load_baseline() -> dict | None:
    """读取已保存的 baseline。"""
    if not BASELINE_FILE.exists():
        return None
    try:
        return json.loads(BASELINE_FILE.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def wait_for_flowctl(timeout: float = 30) -> bool:
    """等待 flow_launcher 的 param IPC 就绪。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if JSON_FILE.exists():
            m = read_metrics()
            if m and abs(m["x"]) > 0.1:
                return True
        time.sleep(0.5)
    return False


def run_demo(duration: int) -> subprocess.Popen | None:
    """启动 demo，返回进程对象。"""
    cmd = [str(ROOT / "scripts" / "demo.sh"), "--no-browser", str(duration)]
    try:
        proc = subprocess.Popen(
            cmd, cwd=ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        return proc
    except FileNotFoundError:
        print("demo.sh 未找到")
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="FlowEngine 调参回归检查")
    ap.add_argument("--duration", type=int, default=20,
                    help="采样时长 (秒, default=20)")
    ap.add_argument("--save-baseline", action="store_true",
                    help="保存当前参数为 baseline（不检查退化）")
    ap.add_argument("--threshold", type=float, default=None,
                    help=f"退化告警阈值 %% (default WARN>{DEFAULT_WARN_THRESHOLD} FAIL>{DEFAULT_FAIL_THRESHOLD})")
    ap.add_argument("--no-run", action="store_true",
                    help="连接到已在运行的 demo，不启动新实例")
    args = ap.parse_args()

    warn_thresh = args.threshold or DEFAULT_WARN_THRESHOLD
    fail_thresh = args.threshold or DEFAULT_FAIL_THRESHOLD

    # 启动或连接 demo
    demo_proc = None
    if not args.no_run:
        print(f"启动 demo ({args.duration}s)...")
        # 先杀残留
        subprocess.run(
            "pkill -9 -f flow_launcher 2>/dev/null; pkill -9 -f flowmond 2>/dev/null; "
            "rm -f /tmp/flow_topology.json /tmp/flow_param.sock 2>/dev/null",
            shell=True, capture_output=True, timeout=10)
        demo_proc = run_demo(args.duration + 10)
        if not demo_proc:
            return 2
        print("等待仿真就绪...")
        if not wait_for_flowctl():
            print("仿真未就绪")
            return 2

    if not JSON_FILE.exists():
        print(f"拓扑 JSON 不存在: {JSON_FILE}")
        print("提示: 需要先启动 demo (或连接已有实例)")
        return 2

    # 采样
    print(f"采样 {args.duration}s ...")
    samples = collect_samples(args.duration)
    if not samples or len(samples) < 5:
        print("采样失败")
        return 2

    # 分析
    results = analyze(samples)

    print(f"\n=== 调参回归检查 ({results['n_samples']} 帧 / {results['duration']:.1f}s) ===")
    print(f"  CTE RMS:        {results['cte_rms']:.4f} m")
    print(f"  Y range:        {results['y_range']:.3f} m")
    print(f"  Yaw rate RMS:   {results['yaw_rate_rms']:.4f} rad/s")
    print(f"  Steer rate RMS: {results['steer_rate_rms']:.4f} rad/s")
    print(f"  Steer flip:     {results['flip_rate']:.2f} Hz")
    print(f"  Avg speed:      {results['avg_speed']:.1f} m/s")

    if results["state_counts"]:
        total = sum(results["state_counts"].values())
        states = ", ".join(f"{s}={c//max(total//100,1):.0f}%"
                          for s, c in sorted(results["state_counts"].items()))
        print(f"  Behavior:       {states}")

    # Baseline 保存模式
    if args.save_baseline:
        save_baseline(results)
        if demo_proc:
            subprocess.run("pkill -9 -f flow_launcher 2>/dev/null",
                           shell=True, capture_output=True, timeout=5)
        return 0

    # 对比 baseline
    baseline = load_baseline()
    if not baseline:
        print(f"\n  没有 baseline 可对比。先用 --save-baseline 保存。")
        if demo_proc:
            subprocess.run("pkill -9 -f flow_launcher 2>/dev/null",
                           shell=True, capture_output=True, timeout=5)
        return 1

    # 比对各项指标
    compare_fields = ["cte_rms", "yaw_rate_rms", "steer_rate_rms", "flip_rate"]
    regression_fields = []

    print(f"\n  Baseline 对比 (退化阈值: WARN≥{warn_thresh:.0f}% FAIL≥{fail_thresh:.0f}%):")
    for field in compare_fields:
        cur = results.get(field, 0)
        ref = baseline.get(field, 0)
        if ref > 0 and cur > 0:
            # 对 negative 指标（越小越好）计算恶化比例
            change_pct = (cur - ref) / ref * 100.0
            direction = "↑" if change_pct > 0 else "↓" if change_pct < 0 else "="
            if change_pct > fail_thresh:
                regression_fields.append(f"  ✗ {field}: {cur:.4f} vs {ref:.4f} ({direction} {abs(change_pct):.0f}%) — FAIL")
            elif change_pct > warn_thresh:
                regression_fields.append(f"  ⚠ {field}: {cur:.4f} vs {ref:.4f} ({direction} {abs(change_pct):.0f}%) — WARN")
            else:
                print(f"  ✓ {field}: {cur:.4f} vs {ref:.4f} ({direction} {abs(change_pct):.0f}%)")
        elif ref == 0 and cur > 0:
            regression_fields.append(f"  ⚠ {field}: {cur:.4f} (baseline was 0)")
        elif ref > 0 and cur == 0:
            regression_fields.append(f"  ✓ {field}: 0.0 (improved from {ref})")

    if regression_fields:
        for msg in regression_fields:
            print(msg)

    # 清理
    if demo_proc:
        subprocess.run("pkill -9 -f flow_launcher 2>/dev/null",
                       shell=True, capture_output=True, timeout=5)

    # 判定
    fails = [f for f in regression_fields if "FAIL" in f]
    warns = [f for f in regression_fields if "WARN" in f and "FAIL" not in f]

    if fails:
        print(f"\n结果: FAIL ({len(fails)} 项显著退化)")
        return 2
    if warns:
        print(f"\n结果: WARN ({len(warns)} 项轻微退化)")
        return 1
    print(f"\n结果: PASS (无退化)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
