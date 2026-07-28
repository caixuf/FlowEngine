#!/usr/bin/env python3
"""Plant 修复效果评估脚本 — 直线稳定性 + 变道品质。

跑一次完整仿真，从 /tmp/flow_topology.json 提取 ego 轨迹，计算：
  直线段: RMS cte, heading_err RMS, steer_flip_rate, yaw_rate RMS
  变道:   持续时间、heading_err 峰值、overshoot、steer 平滑度

用法:
    python3 tools/plant_eval.py                          # 跑 25s 并输出报告
    python3 tools/plant_eval.py --duration 40            # 跑 40s
    python3 tools/plant_eval.py --no-run                 # 只分析已有的 JSON
    python3 tools/plant_eval.py --csv /tmp/plant.csv     # 导出时间序列
"""

import argparse
import csv
import json
import math
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCENARIO_FILE = ROOT / "scenarios" / "straight_road.json"
JSON_FILE = Path("/tmp/flow_topology.json")
STDERR_LOG = Path("/tmp/flow_launcher_stderr.txt")


# ── 道路几何（与 include/road_geometry.h 同步）──────────────────

def load_scenario_road(scenario_path: str = None) -> dict:
    """从场景 JSON 加载道路参数。"""
    path = Path(scenario_path or SCENARIO_FILE)
    if not path.exists():
        return {"lane_count": 2, "lane_width": 3.5}
    with open(path) as f:
        sc = json.load(f)
    road = sc.get("road", {})
    lane_cfg = sc.get("lane", {})
    result = {
        "lane_count": int(road.get("lane_count", lane_cfg.get("count", 2))),
        "lane_width": float(road.get("lane_width", lane_cfg.get("width", 3.5))),
        "curve_start_x": float(road.get("curve_start_x", 0.0)),
        "curve_length_m": float(road.get("curve_length_m", 0.0)),
        "curve_offset_m": float(road.get("curve_offset_m", 0.0)),
        "side_offset": float(road.get("side_offset", 0.0)),
    }
    return result


def road_center_y(x: float, road: dict) -> float:
    """道路中心 y（直线/弯道）。与 road_geometry.h::road_center_y 同步。"""
    csx = road["curve_start_x"]
    clm = road["curve_length_m"]
    com = road["curve_offset_m"]
    if clm <= 0.0 or com == 0.0:
        return 0.0
    if x <= csx:
        return 0.0
    x_in_curve = x - csx
    if x_in_curve >= clm:
        return com
    return com * (1.0 - math.cos(math.pi * x_in_curve / clm)) * 0.5


def road_center_heading(x: float, road: dict) -> float:
    """道路切线航向角 (rad)。与 road_geometry.h::road_center_heading 同步。"""
    csx = road["curve_start_x"]
    clm = road["curve_length_m"]
    com = road["curve_offset_m"]
    if clm <= 0.0 or com == 0.0:
        return 0.0
    if x <= csx or x >= csx + clm:
        return 0.0
    xic = x - csx
    return math.asin(com * math.pi / clm * math.sin(math.pi * xic / clm) * 0.5)


def lane_center_y(lane_idx: int, lane_count: int, lane_width: float,
                  road_c: float = 0.0, side_offset: float = 0.0) -> float:
    """车道中心 y。lane_idx: 0=最左, N-1=最右。"""
    if lane_count <= 1:
        return road_c
    return road_c + side_offset - (lane_idx - (lane_count - 1) * 0.5) * lane_width


def lane_idx_from_y(y: float, lane_count: int, lane_width: float,
                    road_c: float = 0.0, side_offset: float = 0.0) -> int:
    if lane_count <= 1:
        return 0
    offset = (road_c + side_offset - y) / lane_width + (lane_count - 1) * 0.5
    idx = int(round(offset))
    return max(0, min(lane_count - 1, idx))


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


# ── 采样 ───────────────────────────────────────────────────────

def sample_json(path: Path) -> dict | None:
    try:
        if path.stat().st_size == 0:
            return None
        with open(path) as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return None


def collect_samples(duration: int) -> list[dict]:
    """起仿真，每 0.1s 采一次 JSON，返回样本列表。"""
    # 清残留
    subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond; sleep 0.5",
                   shell=True, capture_output=True, timeout=10)

    cmd = [str(ROOT / "scripts" / "demo.sh"), "--no-browser", str(duration)]
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)

    # 删旧 JSON 以免读到上一轮残留
    try:
        JSON_FILE.unlink()
    except FileNotFoundError:
        pass

    samples = []
    started = time.monotonic()
    deadline = started + duration + 60.0
    interval = 0.1

    # 等第一份 JSON
    first_seen = False
    wait_start = time.monotonic()
    while not first_seen and time.monotonic() - wait_start < 20:
        d = sample_json(JSON_FILE)
        if d:
            first_seen = True
            break
        time.sleep(0.5)

    while proc.poll() is None and time.monotonic() < deadline:
        d = sample_json(JSON_FILE)
        if d:
            samples.append(d)
        time.sleep(interval)

    # 等 demo 退出
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        subprocess.run("pkill -9 -f flow_launcher", shell=True, timeout=5)
    subprocess.run("pkill -9 -f flowmond; pkill -9 -f flow_node_host",
                   shell=True, capture_output=True, timeout=5)

    return samples


# ── 指标计算 ───────────────────────────────────────────────────

def extract_series(samples: list[dict], road: dict) -> dict:
    """从 samples 中提取 ego 时间序列和道路相关信息。"""
    xs, ys, headings, speeds, steers = [], [], [], [], []
    timestamps = []
    ego_x_start = None

    for s in samples:
        m = s.get("metrics", {})
        scene = m.get("scene", {})
        ego = scene.get("ego", {})
        ts = s.get("timestamp", 0)
        ex = ego.get("x", 0) if ego else 0
        if not ex:
            continue
        if ego_x_start is None:
            ego_x_start = ex

        xs.append(ex)
        ys.append(ego.get("y", 0))
        headings.append(ego.get("heading", 0))
        speeds.append(ego.get("speed", 0))
        steers.append(ego.get("steer", 0))
        timestamps.append(ts)

    if not xs:
        return {}

    # 道路参考
    road_cs = [road_center_y(x, road) for x in xs]
    road_hs = [road_center_heading(x, road) for x in xs]

    # cross-track error（相对最近车道中心）
    lc = road["lane_count"]
    lw = road["lane_width"]
    so = road.get("side_offset", 0.0)
    ctes = []
    for y, rc in zip(ys, road_cs):
        li = lane_idx_from_y(y, lc, lw, rc, so)
        lcy = lane_center_y(li, lc, lw, rc, so)
        ctes.append(y - lcy)

    # heading error（相对道路切线）
    hdg_errs = [angle_diff(h, rh) for h, rh in zip(headings, road_hs)]

    # yaw rate（数值微分）
    yaw_rates = [0.0]
    for i in range(1, len(headings)):
        dt = timestamps[i] - timestamps[i - 1]
        if dt > 0:
            yaw_rates.append(abs(angle_diff(headings[i], headings[i - 1])) / dt)
        else:
            yaw_rates.append(0.0)

    # steer rate
    steer_rates = [0.0]
    for i in range(1, len(steers)):
        dt = timestamps[i] - timestamps[i - 1]
        if dt > 0:
            steer_rates.append(abs(steers[i] - steers[i - 1]) / dt)
        else:
            steer_rates.append(0.0)

    # steer flip 计数
    flip_count = 0
    prev_s = steers[0]
    for s in steers[1:]:
        if prev_s * s < 0 and abs(s - prev_s) > 0.01:
            flip_count += 1
        prev_s = s
    duration = timestamps[-1] - timestamps[0] if timestamps else 1.0
    flip_rate = flip_count / max(duration, 1.0)

    return {
        "xs": xs,
        "ys": ys,
        "headings": headings,
        "speeds": speeds,
        "steers": steers,
        "ctes": ctes,
        "hdg_errs": hdg_errs,
        "yaw_rates": yaw_rates,
        "steer_rates": steer_rates,
        "road_cs": road_cs,
        "road_hs": road_hs,
        "timestamps": timestamps,
        "duration": duration,
        "final_x": xs[-1] if xs else 0,
        "avg_speed": statistics.mean(speeds) if speeds else 0,
        "flip_rate": flip_rate,
    }


def detect_lane_changes(series: dict, road: dict) -> list[dict]:
    """检测变道事件，返回事件列表。"""
    events = []
    lw = road["lane_width"]
    lc = road["lane_count"]
    so = road.get("side_offset", 0.0)

    if not series or not series.get("ctes"):
        return events

    ctes = series["ctes"]
    ts = series["timestamps"]
    last_lane = lane_idx_from_y(
        series["ys"][0], lc, lw,
        road_center_y(series["xs"][0], road), so)

    in_transition = False
    transition_start = None
    start_cte = None
    start_idx = 0

    for i in range(1, len(ctes)):
        rc = road_center_y(series["xs"][i], road)
        cur_lane = lane_idx_from_y(series["ys"][i], lc, lw, rc, so)

        if cur_lane != last_lane and not in_transition:
            in_transition = True
            transition_start = ts[i]
            start_cte = ctes[i]
            last_lane = cur_lane
            start_idx = i

        if in_transition:
            # 检测完成：稳定在新车道
            stable_count = 0
            for j in range(i, min(i + 5, len(ctes))):
                rj = road_center_y(series["xs"][j], road)
                if lane_idx_from_y(series["ys"][j], lc, lw, rj, so) == cur_lane:
                    stable_count += 1
            if stable_count >= 3:
                # 变道事件
                end_idx = i
                end_cte = ctes[i]
                seg_cte = ctes[start_idx:end_idx + 1]
                seg_hdg = series["hdg_errs"][start_idx:end_idx + 1]
                seg_steer = series["steers"][start_idx:end_idx + 1]

                events.append({
                    "start_time": transition_start,
                    "end_time": ts[i],
                    "duration": ts[i] - transition_start,
                    "cte_delta": abs(end_cte - start_cte),
                    "max_hdg_err": max(abs(h) for h in seg_hdg) if seg_hdg else 0,
                    "cte_rms": math.sqrt(statistics.fmean([c * c for c in seg_cte])) if seg_cte else 0,
                    "steer_max": max(abs(s) for s in seg_steer) if seg_steer else 0,
                    "steer_range": max(seg_steer) - min(seg_steer) if seg_steer else 0,
                    "direction": "left" if end_cte > start_cte else "right",
                    "overshoot": max(abs(c) for c in seg_cte) - abs(end_cte) if seg_cte else 0,
                })
                in_transition = False

    return events


def compute_straight_metrics(series: dict, road: dict) -> dict:
    """直线段指标（排除变道区间）。"""
    if not series or not series.get("ctes"):
        return {}

    # 检测变道区间
    lc = road["lane_count"]
    lw = road["lane_width"]
    so = road.get("side_offset", 0.0)

    # 标记哪些帧在变道中
    in_lc = [False] * len(series["ctes"])
    for i in range(1, len(series["ctes"])):
        rc = road_center_y(series["xs"][i], road)
        cur_lane = lane_idx_from_y(series["ys"][i], lc, lw, rc, so)
        prev_lane = lane_idx_from_y(series["ys"][i - 1], lc, lw,
                                     road_center_y(series["xs"][i - 1], road), so)
        if cur_lane != prev_lane:
            for j in range(max(0, i - 5), min(len(series["ctes"]), i + 8)):
                in_lc[j] = True

    straight_cte = [series["ctes"][i] for i in range(len(series["ctes"])) if not in_lc[i]]
    straight_hdg = [series["hdg_errs"][i] for i in range(len(series["hdg_errs"])) if not in_lc[i]]
    straight_yaw = [series["yaw_rates"][i] for i in range(len(series["yaw_rates"])) if not in_lc[i]]
    straight_steer_rate = [series["steer_rates"][i] for i in range(len(series["steer_rates"])) if not in_lc[i]]

    if not straight_cte:
        return {}

    return {
        "cte_rms": math.sqrt(statistics.fmean([c * c for c in straight_cte])),
        "cte_max": max(abs(c) for c in straight_cte),
        "hdg_err_rms": math.sqrt(statistics.fmean([h * h for h in straight_hdg])) if straight_hdg else 0,
        "hdg_err_max": max(abs(h) for h in straight_hdg) if straight_hdg else 0,
        "yaw_rate_rms": math.sqrt(statistics.fmean([y * y for y in straight_yaw])) if straight_yaw else 0,
        "steer_rate_rms": math.sqrt(statistics.fmean([s * s for s in straight_steer_rate])) if straight_steer_rate else 0,
        "flip_rate": series["flip_rate"],
        "straight_sample_count": len(straight_cte),
    }


# ── 评分 ───────────────────────────────────────────────────────

def evaluate(straight: dict, lc_events: list[dict], series: dict) -> tuple[list[str], list[str]]:
    """返回 (passes, fails) 列表。"""
    passes, fails = [], []
    lw = 3.5  # fallback

    if straight:
        # 直线段
        if straight["cte_rms"] < 0.3:
            passes.append(f"straight CTE RMS: {straight['cte_rms']:.3f} m (PASS < 0.3)")
        else:
            fails.append(f"straight CTE RMS: {straight['cte_rms']:.3f} m (FAIL ≥ 0.3)")

        if straight["hdg_err_rms"] < 0.05:
            passes.append(f"straight heading err RMS: {straight['hdg_err_rms']:.4f} rad (PASS < 0.05)")
        else:
            fails.append(f"straight heading err RMS: {straight['hdg_err_rms']:.4f} rad (FAIL ≥ 0.05)")

        if straight["flip_rate"] < 1.0:
            passes.append(f"steer flip rate: {straight['flip_rate']:.3f} Hz (PASS < 1.0)")
        else:
            fails.append(f"steer flip rate: {straight['flip_rate']:.3f} Hz (FAIL ≥ 1.0, oscillating)")

        if straight["yaw_rate_rms"] < 0.3:
            passes.append(f"yaw rate RMS: {straight['yaw_rate_rms']:.4f} rad/s (PASS < 0.3)")
        else:
            fails.append(f"yaw rate RMS: {straight['yaw_rate_rms']:.4f} rad/s (FAIL ≥ 0.3)")

    if lc_events:
        for i, ev in enumerate(lc_events):
            tag = f"LC#{i} ({ev['direction']})"
            if ev["duration"] < 5.0:
                passes.append(f"{tag}: duration {ev['duration']:.1f}s (PASS < 5.0)")
            else:
                fails.append(f"{tag}: duration {ev['duration']:.1f}s (FAIL ≥ 5.0, too slow)")

            if ev["max_hdg_err"] < 0.15:
                passes.append(f"{tag}: max heading err {ev['max_hdg_err']:.3f} rad (PASS < 0.15)")
            else:
                fails.append(f"{tag}: max heading err {ev['max_hdg_err']:.3f} rad (FAIL ≥ 0.15, overshoot)")

            if ev["overshoot"] < 0.3:
                passes.append(f"{tag}: overshoot {ev['overshoot']:.3f} m (PASS < 0.3)")
            else:
                fails.append(f"{tag}: overshoot {ev['overshoot']:.3f} m (FAIL ≥ 0.3)")
    else:
        fails.append("no lane changes detected in {:.0f}s run".format(series.get("duration", 0)))

    if series.get("avg_speed", 0) > 8.0:
        passes.append(f"avg speed: {series['avg_speed']:.1f} m/s (PASS > 8.0)")
    else:
        fails.append(f"avg speed: {series['avg_speed']:.1f} m/s (FAIL ≤ 8.0)")

    return passes, fails


def save_csv(series: dict, path: str):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "heading", "speed", "steer",
                     "cte", "hdg_err", "yaw_rate", "road_c", "road_h"])
        for i in range(len(series["timestamps"])):
            w.writerow([
                f"{series['timestamps'][i]:.3f}",
                f"{series['xs'][i]:.3f}",
                f"{series['ys'][i]:.3f}",
                f"{series['headings'][i]:.4f}",
                f"{series['speeds'][i]:.3f}",
                f"{series['steers'][i]:.4f}",
                f"{series['ctes'][i]:.4f}",
                f"{series['hdg_errs'][i]:.4f}",
                f"{series['yaw_rates'][i]:.4f}",
                f"{series['road_cs'][i]:.4f}",
                f"{series['road_hs'][i]:.4f}",
            ])


def main():
    ap = argparse.ArgumentParser(description="Plant 修复效果评估")
    ap.add_argument("--duration", type=int, default=25, help="仿真时长 (s)")
    ap.add_argument("--no-run", action="store_true", help="不启仿真，只分析已有 JSON")
    ap.add_argument("--csv", default="", help="保存时间序列 CSV 到路径")
    args = ap.parse_args()

    road = load_scenario_road()
    lane_count = road["lane_count"]
    lane_width = road["lane_width"]

    print(f"\n  ╔══════════════════════════════════════╗")
    print(f"  ║  Plant 修复效果评估                    ║")
    print(f"  ╚══════════════════════════════════════╝")
    print(f"  场景: {SCENARIO_FILE.name}")
    print(f"  车道: {lane_count}x{lane_width}m")
    print(f"  弯道: start={road['curve_start_x']}m  "
          f"len={road['curve_length_m']}m  "
          f"offset={road['curve_offset_m']}m")
    print()

    samples = []
    if args.no_run:
        d = json.loads(JSON_FILE.read_text()) if JSON_FILE.exists() else None
        if d:
            samples = [d]
    else:
        print(f"  起仿真 ({args.duration}s)...")
        samples = collect_samples(args.duration)
        print(f"  采集到 {len(samples)} 个样本")

    if not samples:
        print("  ✗ 无数据")
        return 1

    series = extract_series(samples, road)
    if not series:
        print("  ✗ 无法提取时间序列")
        return 1

    # 变道检测
    lc_events = detect_lane_changes(series, road)
    straight = compute_straight_metrics(series, road)

    # 评分
    passes, fails = evaluate(straight, lc_events, series)
    total = len(passes) + len(fails)
    score = len(passes) / max(total, 1) * 100

    # ── 输出报告 ──
    print(f"\n  ╔══════════════════════════════════════╗")
    print(f"  ║  报告 (score={score:.0f}%  {len(passes)}/{total})          ║")
    print(f"  ╚══════════════════════════════════════╝")
    print(f"\n  概要:")
    print(f"    行驶距离: {series['final_x']:.0f} m")
    print(f"    平均速度: {series['avg_speed']:.1f} m/s")
    print(f"    steer flip rate: {series['flip_rate']:.3f} Hz")
    if lc_events:
        print(f"    变道事件: {len(lc_events)} 次")
    print()

    if straight:
        print(f"  直线段指标:")
        print(f"    CTE RMS:       {straight['cte_rms']:.3f} m")
        print(f"    CTE max:       {straight['cte_max']:.3f} m")
        print(f"    heading err RMS: {straight['hdg_err_rms']:.4f} rad")
        print(f"    heading err max: {straight['hdg_err_max']:.4f} rad")
        print(f"    yaw rate RMS:  {straight['yaw_rate_rms']:.4f} rad/s")
        print(f"    steer rate RMS: {straight['steer_rate_rms']:.4f} rad/s")
        print(f"    样本数: {straight['straight_sample_count']}")
        print()

    if lc_events:
        print(f"  变道事件:")
        for i, ev in enumerate(lc_events):
            print(f"    #{i} {ev['direction']:>5s}: "
                  f"dur={ev['duration']:.1f}s  "
                  f"cte_delta={ev['cte_delta']:.2f}m  "
                  f"max_hdg_err={ev['max_hdg_err']:.3f}rad  "
                  f"overshoot={ev['overshoot']:.3f}m  "
                  f"steer_range={ev['steer_range']:.3f}rad")
        print()

    print(f"  检测项:")
    for p in passes:
        print(f"    ✅ {p}")
    for f in fails:
        print(f"    ❌ {f}")
    print()

    verdict = "PASS" if not fails else "FAIL"
    print(f"  Verdict: {verdict}")

    if args.csv:
        save_csv(series, args.csv)
        print(f"  CSV: {args.csv}")

    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
