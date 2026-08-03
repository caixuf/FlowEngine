#!/usr/bin/env python3
"""motion_analyzer.py — 车辆运动分析工具（通用）

从 topology JSON（实时 /tmp/flow_topology.json 或历史副本）提取 ego/NPC
运动序列，量化"横移 vs 转向"—— 判断车是"开过去的"（车头先转、沿弧线）
还是"拖过去的"（车没转就横着动 = 车屁股平移）。

核心指标（对每个运动段）：
  v_lat        = Δy/Δt              实际横向速度 (m/s)
  v_lat_head   = v·sin(heading)     车头方向应有的横向分量 (m/s)
  crab_index   = |v_lat| / |v_lat_head|
                ≈ 1.0  → 正常：横向运动来自车头转向（弧线变道）
                > 1.5  → 横移嫌疑：车没转（或转得不够）就横着动
                ≈ 0    → 纯转向（原地掉头/侧滑）

判定准则（物理）：
  自行车模型（后轴无侧滑）：v_lat = v·sin(δ_eff)，横向运动只能由转向产生。
  crab_index 显著 >1 说明横向运动来源不是转向 → 位置与姿态解耦（轨道拖动/
  死推算/offset 平移等层的问题）。

用法:
  # 实时监控（检测到变道段自动输出分析）
  python3 tools/motion_analyzer.py --live --watch

  # 分析历史副本（demo 结束后存的 /tmp/flow_topology_*.json）
  python3 tools/motion_analyzer.py --json /tmp/flow_topology_20260803_145824.json

  # 只分析 ego（默认 ego + NPC 都分析）
  python3 tools/motion_analyzer.py --live --ego-only

输出: 每个变道段（横向位移累计 > 1.5m）的逐帧表 + 段级 crab_index 汇总。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

# ── 变道段检测参数 ──
LANE_CHANGE_DY = 1.5      # 横向位移累计超过此值视为一个变道段 (m)
CRAB_INDEX_THRESHOLD = 1.5  # crab_index 超过此值判"横移嫌疑"


def load_entities(path: str) -> list[dict]:
    """从 topology JSON 提取 ego + entities（快照级）。"""
    try:
        d = json.load(open(path))
    except (json.JSONDecodeError, OSError) as e:
        print(f"✗ 无法读取 {path}: {e}", file=sys.stderr)
        return []
    out = []
    metrics = d.get("metrics", {})
    scene = metrics.get("scene", {})
    ego = scene.get("ego") or metrics.get("vehicle", {})
    if ego:
        out.append({"id": "ego", "x": ego.get("x", 0), "y": ego.get("y", 0),
                    "heading": ego.get("heading", 0), "speed": ego.get("speed", 0)})
    for e in scene.get("entities", []):
        if not e or e.get("type") == "ego":
            continue
        out.append({"id": str(e.get("id", "?")), "x": e.get("x", 0),
                    "y": e.get("y", 0), "heading": e.get("heading", 0),
                    "speed": e.get("speed", 0)})
    return out


def analyze_segment(seg: list[tuple[float, float, float, float]], dt: float) -> dict:
    """分析一段运动（x, y, heading, v 序列）→ 段级指标。"""
    if len(seg) < 3:
        return {}
    crab_ratios = []
    max_crab = 0.0
    max_crab_at = 0.0
    lat_total = 0.0
    heading_total = 0.0
    for i in range(1, len(seg)):
        x0, y0, h0, v0 = seg[i - 1]
        x1, y1, h1, v1 = seg[i]
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            continue
        v_lat = dy / dt                      # 实际横向速度（ENU y = 横向）
        v = max(v1, v0, 0.1)
        v_lat_head = v * math.sin(h1)        # 车头方向应有的横向分量
        denom = max(abs(v_lat_head), 0.1)
        crab = abs(v_lat) / denom
        crab_ratios.append(crab)
        if crab > max_crab:
            max_crab = crab
            max_crab_at = x1
        lat_total += abs(dy)
        heading_total += abs(h1 - h0)
    if not crab_ratios:
        return {}
    mean_crab = sum(crab_ratios) / len(crab_ratios)
    return {
        "seg_len": len(seg),
        "lat_total_m": lat_total,
        "heading_total_rad": heading_total,
        "mean_crab": mean_crab,
        "max_crab": max_crab,
        "max_crab_at_x": max_crab_at,
        "verdict": ("横移嫌疑" if mean_crab > CRAB_INDEX_THRESHOLD
                    else "正常弧线" if mean_crab > 0.8 else "转向为主"),
    }


def analyze_series(samples: list[dict], dt: float, ego_only: bool) -> None:
    """按实体分组的采样序列 → 检测变道段并分析。"""
    by_id: dict[str, list] = {}
    for s in samples:
        for e in s:
            if ego_only and e["id"] != "ego":
                continue
            by_id.setdefault(e["id"], []).append(e)

    for eid, series in by_id.items():
        if len(series) < 5:
            continue
        # 检测变道段：累计 |Δy| 超阈值
        segments = []
        cur = [series[0]]
        lat_accum = 0.0
        for i in range(1, len(series)):
            dy = series[i]["y"] - series[i - 1]["y"]
            lat_accum += abs(dy)
            cur.append(series[i])
            if lat_accum >= LANE_CHANGE_DY:
                segments.append(cur)
                cur = [series[i]]
                lat_accum = 0.0
        if not segments:
            continue
        print(f"\n── 实体 {eid}: {len(segments)} 个运动段 ──")
        for k, seg in enumerate(segments):
            r = analyze_segment(
                [(s["x"], s["y"], s["heading"], s["speed"]) for s in seg], dt)
            if not r:
                continue
            print(f"  段 {k}: 横向 {r['lat_total_m']:.1f}m / 朝向变化 "
                  f"{r['heading_total_rad']:.2f}rad / crab_mean={r['mean_crab']:.2f} "
                  f"max={r['max_crab']:.2f} @x={r['max_crab_at_x']:.0f} → {r['verdict']}")


def live_watch(dt: float, ego_only: bool, duration_s: float) -> None:
    """实时监控 /tmp/flow_topology.json，持续采样并分析。"""
    print(f"实时监控中 (dt={dt*1000:.0f}ms, {'ego only' if ego_only else 'ego+NPC'}, "
          f"{duration_s:.0f}s)… Ctrl+C 退出")
    samples: list[list[dict]] = []
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < duration_s:
        try:
            ents = load_entities("/tmp/flow_topology.json")
        except Exception:
            ents = []
        if ents:
            samples.append(ents)
        now = time.time()
        if samples and now - last_print >= 8.0:  # 每 8s 输出一次增量分析
            last_print = now
            analyze_series(samples[-200:], dt, ego_only)
            print(f"  [{now - t0:.0f}s] 已采样 {len(samples)} 帧", flush=True)
        time.sleep(dt)
    print("\n── 最终分析 ──")
    analyze_series(samples, dt, ego_only)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true", help="实时读 /tmp/flow_topology.json")
    ap.add_argument("--watch", action="store_true", help="--live 时持续监控（默认实时模式已持续）")
    ap.add_argument("--json", type=str, help="分析历史 topology JSON 文件")
    ap.add_argument("--ego-only", action="store_true", help="只分析 ego")
    ap.add_argument("--dt", type=float, default=0.1, help="采样间隔 (s)，默认 0.1")
    ap.add_argument("--duration", type=float, default=120.0, help="实时监控时长 (s)")
    args = ap.parse_args()

    if args.json:
        path = Path(args.json)
        if not path.exists():
            print(f"✗ 文件不存在: {path}", file=sys.stderr)
            return 1
        # 单快照文件：按 scene.entities + ego 分析当前帧（无历史则提示）
        ents = load_entities(str(path))
        print(f"文件 {path.name}: {len(ents)} 个实体（单帧快照，无运动历史）")
        print("  提示：用 --live 实时监控才能检测运动段；历史序列可用"
              " demo 运行期间 --live 采集。")
        return 0

    if args.live:
        live_watch(args.dt, args.ego_only, args.duration)
        return 0

    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
