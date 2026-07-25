#!/usr/bin/env python3
"""Check NPC y-drift over time during a demo run.

Samples /tmp/flow_topology.json at 5Hz while demo.sh runs the straight_road
scenario, then prints per-NPC y trajectory + lane-change detection.
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/workspace")
TOPO = Path("/tmp/flow_topology.json")

# Kill any stale demo
subprocess.run(["pkill", "-f", "flow_node_host"], capture_output=True)
subprocess.run(["pkill", "-f", "flow_launcher"], capture_output=True)
subprocess.run(["pkill", "-f", "demo.sh"], capture_output=True)
time.sleep(1)

try:
    TOPO.unlink()
except FileNotFoundError:
    pass

duration = 14
proc = subprocess.Popen(
    [str(ROOT / "scripts" / "demo.sh"), "--no-browser",
     "--scenario", "scenarios/straight_road.json", str(duration)],
    cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    start_new_session=True,
)

samples = []
deadline = time.time() + duration + 40
while proc.poll() is None and time.time() < deadline:
    try:
        if TOPO.exists():
            with open(TOPO) as f:
                s = json.load(f)
            m = s.get("metrics", {})
            ents = m.get("scene", {}).get("entities", [])
            if ents:
                samples.append({
                    "t": time.time(),
                    "entities": [{"id": e.get("id"), "x": e.get("x"), "y": e.get("y"),
                                  "type": e.get("type"), "vx": e.get("vx"),
                                  "vy": e.get("vy")}
                                 for e in ents if e.get("type") in ("car", "truck", "suv")],
                })
    except (json.JSONDecodeError, OSError):
        pass
    time.sleep(0.25)

try:
    os.killpg(os.getpgid(proc.pid), 15)
    proc.wait(timeout=10)
except Exception:
    try:
        os.killpg(os.getpgid(proc.pid), 9)
    except Exception:
        pass

print(f"=== Collected {len(samples)} samples ===\n")

# Group by NPC id
by_id = {}
for s in samples:
    for e in s["entities"]:
        eid = e["id"]
        if eid is None:
            continue
        by_id.setdefault(eid, []).append((s["t"], e))

print("=== Per-NPC y trajectory ===")
for eid in sorted(by_id.keys()):
    traj = by_id[eid]
    if not traj:
        continue
    t0 = traj[0][0]
    print(f"\nNPC id={eid} (n={len(traj)} samples):")
    print(f"  {'t(s)':>6}  {'x':>8}  {'y':>7}  {'vx':>6}  {'vy':>6}  d_y_from_start")
    start_y = traj[0][1]["y"]
    last_y = start_y
    max_dy = 0.0
    lane_changes = 0
    lane = round((start_y or 0) / 3.5)  # rough lane index
    for t, e in traj:
        y = e.get("y")
        if y is None:
            continue
        dy_from_start = y - start_y
        dy_from_prev = y - last_y
        if abs(dy_from_prev) > max_dy:
            max_dy = abs(dy_from_prev)
        # Detect lane change (crossed 3.5m boundary)
        new_lane = round(y / 3.5)
        if new_lane != lane:
            lane_changes += 1
            lane = new_lane
        last_y = y
        print(f"  {t-t0:6.2f}  {e.get('x',0):8.2f}  {y:7.2f}  "
              f"{e.get('vx',0):6.2f}  {e.get('vy',0):6.2f}  {dy_from_start:+.3f}")
    print(f"  --> lane_changes_detected: {lane_changes}, max_dy_per_step: {max_dy:.3f} m")
