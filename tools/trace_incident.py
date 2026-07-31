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
import re
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

    # ── Layer 4b: Planning debug (Frenet/横向规划变量) ──
    pd = metrics.get("planning_debug", {})
    if pd:
        print_layer("4b. 规划Debug (横向链路)", [
            (f"ego_x / ego_y", f"{pd.get('ego_x',0):.1f} / {pd.get('ego_y',0):.2f}"),
            (f"road_center_y (rc_y)", f"{pd.get('road_center_y',0):.3f} m"),
            (f"ego_d (ego_y - rc_y)", f"{pd.get('ego_d',0):.3f} m"),
            (f"target_lane_offset (d目标)", f"{pd.get('target_lane_offset',0):.3f} m"),
            (f"command_speed", f"{pd.get('command_speed',0):.2f} m/s"),
            (f"n_wp (轨迹点数)", f"{int(pd.get('n_wp',0))}"),
            (f"traj_valid", "✅" if pd.get("traj_valid",0) > 0.5 else "❌ INVALID"),
            (f"n_lanes / lane_width", f"{int(pd.get('n_lanes',0))} / {pd.get('lane_width',0):.1f}m"),
        ])
        if pd.get("beh_target_lane", -1) >= 0:
            print(f"    beh_cmd={int(pd.get('beh_cmd',-1))} target_lane={int(pd.get('beh_target_lane',-1))} "
                  f"d: start={pd.get('d_start',0):.3f} → lookahead={pd.get('d_lookahead',0):.3f} → end={pd.get('d_end',0):.3f}")

    # ── Layer 5: Control command ──
    vehicle = metrics.get("vehicle", {})
    cd = metrics.get("control_debug", {})
    print_layer("5. 控制输出 (Control)", [
        (f"speed / target", f"{vehicle.get('speed', 0):.1f} / {vehicle.get('target_speed', 0):.1f} m/s"),
        (f"throttle / brake", f"{vehicle.get('throttle', 0):.2f} / {vehicle.get('brake', 0):.2f}"),
        (f"error", f"{vehicle.get('error', 0):.1f} m/s"),
    ])

    # ── Layer 5b: Control debug (Stanley/横向控制变量) ──
    if cd:
        mpc_str = "MPC" if cd.get("mpc_used", 0) > 0.5 else "Stanley"
        hazard_str = "🚨 HAZARD" if cd.get("hazard", 0) > 0.5 else "ok"
        print_layer("5b. 控制Debug (横向链路)", [
            (f"controller", f"{mpc_str}"),
            (f"ego_y", f"{cd.get('ego_y',0):.3f} m"),
            (f"lane_d (来自traj)", f"{cd.get('lane_d',0):.3f} m"),
            (f"road_center_y (rc_y)", f"{cd.get('road_center_y',0):.3f} m"),
            (f"target_y (rc_y + lane_d)", f"{cd.get('target_y',0):.3f} m"),
            (f"lat_error (target - ego)", f"{cd.get('lat_error',0):.3f} m"),
            (f"steer (rad/deg)", f"{cd.get('steer',0):.4f} / {math.degrees(cd.get('steer',0)):.1f}°"),
            (f"y_from_target", f"{cd.get('y_from_target',0):.3f} m"),
            (f"hazard", f"{hazard_str}"),
            (f"mode", f"{cd.get('mode','?')}"),
            (f"has_planning", "✅" if cd.get("has_planning",0) > 0.5 else "❌ NO"),
            (f"lookahead_dist", f"{cd.get('lookahead_dist',0):.1f}m"),
        ])

    # ── 横向链路一致性检查 ──
    lateral_verdicts = []
    if pd and cd:
        # planning 输出的 target_lane_offset 应该等于 control 的 lane_d
        pd_d = pd.get("target_lane_offset", 0)
        cd_lane_d = cd.get("lane_d", 0)
        if abs(pd_d - cd_lane_d) > 0.1:
            lateral_verdicts.append(f"❌ planning.target_lane_offset={pd_d:.3f} ≠ control.lane_d={cd_lane_d:.3f} → 轨迹横向偏移没传到control")
        else:
            lateral_verdicts.append(f"✅ planning→control 横向偏移一致: d={pd_d:.3f}m")

        # control 的 target_y = road_center_y + lane_d
        expected_target = cd.get("road_center_y", 0) + cd_lane_d
        actual_target = cd.get("target_y", 0)
        if abs(expected_target - actual_target) > 0.05:
            lateral_verdicts.append(f"❌ target_y计算错误: expected rc_y+lane_d={expected_target:.3f}, got {actual_target:.3f}")

        # 变道中检查：ego_y 是否朝 target_y 方向移动
        beh_target = pd.get("beh_target_lane", -1)
        if beh_target >= 0 and cd.get("lat_error", 0) != 0:
            lat_err = cd.get("lat_error", 0)
            steer = cd.get("steer", 0)
            # steer方向应该与lat_error一致（正误差→负steer修正向左，反之向右，取决于坐标约定）
            # 这里只检查 steer!=0 且方向有意义
            if abs(steer) < 0.001 and abs(lat_err) > 0.3:
                lateral_verdicts.append(f"❌ lat_error={lat_err:.3f}m 但 steer≈0 → Stanley没有输出修正方向")

    # ── F: 碰撞复盘（碰撞前 5s 时间线） ──
    if incident and incident["type"] == "collision":
        print_layer("F. 碰撞复盘 (Collision Timeline)", [])
        # 从 launcher stderr 提取碰撞现场数据
        launcher_log = Path("/tmp/flow_launcher_stderr.txt")
        col_data = {}
        if launcher_log.exists():
            txt = launcher_log.read_text(encoding="utf-8", errors="ignore")
            # 匹配 COLLISION 行中的 JSON 片段
            col_match = re.search(r"COLLISION\s+ego.*?\{.*?overlap", txt, re.DOTALL)
            if col_match:
                # 尝试提取 JSON 尾部的数值字段
                for key in ("obs_x", "obs_y", "obs_vx", "obs_vy", "obs_heading", "obs_speed", "obs_id"):
                    vm = re.search(rf'"{key}"\s*:\s*([-\d.e+]+)', txt)
                    if vm:
                        col_data[key] = float(vm.group(1))

        if col_data:
            print(f"    碰撞对象: id={int(col_data.get('obs_id', -1))}")
            print(f"    NPC 位置:  x={col_data.get('obs_x',0):.1f}  y={col_data.get('obs_y',0):.2f}")
            print(f"    NPC 速度:  {col_data.get('obs_speed',0):.1f} m/s  vx={col_data.get('obs_vx',0):.1f}  vy={col_data.get('obs_vy',0):.1f}")
            print(f"    NPC 航向:  {math.degrees(col_data.get('obs_heading',0)):.0f}°")
            print()

        # 碰撞前 5s 时间线（从 samples 中提取 ego + NPC 轨迹）
        if len(samples) >= 3:
            # 找到碰撞时间点（最后一个样本的 t）
            col_t = max(s.get("t_demo", s.get("t", 0)) for s in samples)
            timeline_start = col_t - 5.0
            print(f"    ┌─ 碰撞前 5s 时间线 (t={timeline_start:.1f}s → {col_t:.1f}s) ──")
            print(f"    │  {'t(s)':>6}  {'ego_x':>7}  {'ego_y':>7}  {'speed':>6}  {'gap':>6}  {'lead':>6}  {'state':>10}")
            col_id = int(col_data.get("obs_id", -1)) if col_data else -1
            for s in samples:
                ts = s.get("t_demo", s.get("t", 0))
                if ts < timeline_start or ts > col_t:
                    continue
                sx = s.get("x", 0)
                sy = s.get("y", 0)
                ss = s.get("speed", 0)
                # 碰撞对象的相对位置
                npc_x = None
                gap = float("inf")
                for ent in s.get("metrics", {}).get("scene", {}).get("entities", []):
                    if int(ent.get("id", -1)) == col_id:
                        npc_x = float(ent.get("x", 0))
                        gap = npc_x - sx
                        break
                bm = s.get("metrics", {}).get("behavior", {})
                state = bm.get("state", "?")
                lead = bm.get("lead_speed", 0)
                gap_str = f"{gap:.1f}" if math.isfinite(gap) else "∞"
                print(f"    │  {ts:>6.1f}  {sx:>7.1f}  {sy:>7.2f}  {ss:>6.1f}  {gap_str:>6}  {lead:>6.1f}  {state:>10}")
            print(f"    └{'─'*65}")

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

    # 横向链路诊断
    verdicts.extend(lateral_verdicts)
    # 变道超时检测
    if beh.get("state") in ("LEFT_CHANGE", "RIGHT_CHANGE"):
        timer = beh.get("state_timer", 0)
        dist = beh.get("dist_to_target_lane", -1)
        verdicts.append(f"ℹ️ 变道中: state_timer={timer:.1f}s dist_to_target={dist:.2f}m "
                        f"committed→target: {int(beh.get('committed_lane',-1))}→{int(beh.get('target_lane',-1))}")

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
