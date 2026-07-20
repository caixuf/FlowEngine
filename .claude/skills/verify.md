# Verify — FlowEngine Demo Evaluator

Run the automated regression evaluator against the FlowEngine demo pipeline
and interpret results.

## When to use

- After any change to a node in the pipeline chain:
  `sim_world` → `sensor_model` → `perception` → `fusion` → `planning` → `control` → `safety_control`
- After tuning controller parameters (PID, lat_kp, lat_kd_heading, blocked_timeout_s, etc.)
- Before committing, per the project's /verify convention
- When investigating a "car stops driving" or "speed drops to zero" report

## How to run

```bash
# Default: 15 s run, 0.25 s sampling interval
python3 tools/demo_evaluator.py

# Longer run to catch slow drifts (road-edge creep takes >20 s to manifest)
python3 tools/demo_evaluator.py --duration 45 --interval 0.5

# Evaluate the last run's data without re-launching demo.sh
python3 tools/demo_evaluator.py --no-run
```

The evaluator:
1. Launches `scripts/demo.sh --no-browser <duration>`
2. Samples `/tmp/flow_topology.json` every `--interval` seconds
3. Scores the run against the scenario's `pass_criteria` from `config/pipeline.json`
4. Prints a summary table, warnings, and PASS/FAIL verdict

## What it checks

| Category | Check | Threshold |
|----------|-------|-----------|
| Topology | All 5 required edges present | — |
| Topology | At least one sensor edge pair (perception or sensor_model) | — |
| Frequency | Each topic meets minimum rate | vehicle/state ≥15Hz, sensor/lidar ≥15Hz, etc. |
| Collision | No `sim/collision` publishes or "COLLISION" in logs | 0 |
| Road departure | ego body stays within road edge | margin ≥ 0 |
| Progress | Vehicle covers minimum distance | x_delta ≥ min_distance_m (default 10m) |
| Stagnation | Low-speed ratio & longest run below threshold | <50% ratio, <5s streak (with scenario criteria) |
| Lane change | At least 1 lane change detected | ≥1 |
| Yaw wobble | RMS and max yaw rate | rms <0.35, max <1.2 rad/s |
| Steer oscillation | RMS and max steer rate, flip rate | rms <0.9, max <3.0 /s, flips <1.0 Hz |
| NPC motion | Max speed and lateral speed (catches teleport jumps) | speed <45, lateral <12 m/s |
| Message drops | Zero dropped messages | 0 |

## Interpreting results

### PASS
All checks passed. The demo pipeline is regression-clean.

### WARN (non-fatal)
- **"large lane-center deviation"** — normal during lane changes (up to ~1.75 m when crossing the centerline)
- **"npc respawn jump"** — known issue: obstacle recycling in `sim_world_node.c` teleports NPCs; not a regression
- **"steer saturated"** — may indicate underdamped lateral controller; if persistent, check `lat_kp` / `lat_kd_heading`
- **"ego yaw wobble"** — heading oscillations; check Stanley controller damping

### FAIL
- **"vehicle stuck or no progress"** → likely the ROAD_GUARD deadlock:
  car drifts to road edge, speed drops to 0, bicycle model can't move laterally.
  Check `control_node.c` ROAD_GUARD low-speed recovery condition (should be `>=` not `>`).
- **"road departure"** → lane-change overshoot. Check `lat_kd_heading` is actually
  used in the Stanley heading term (not hardcoded to 0.5).
- **"collision detected"** → AEB or ACC gap too aggressive. Check safety_control params.
- **"low-speed stagnation"** → car stuck behind slow lead vehicle, lane change blocked.

## Common failure patterns & fixes

### Pattern 1: ROAD_GUARD deadlock (car stops permanently at road edge)
**Symptom:** `x_delta` near 0, `min_road_margin` near 0, speed=0 in final samples.
**Root cause:** `control_node.c` line 534 — low-speed recovery uses `>` instead of `>=`,
and at exactly `|y| = road_center_limit` the escape throttle never fires.
**Fix:** Change `>` to `>=` in the ROAD_GUARD condition.

### Pattern 2: Lane-change overshoot (road departure)
**Symptom:** `min_road_margin < 0`, `max_lane_error > 2.0`.
**Root cause:** Stanley heading damping hardcoded to 0.5 instead of using config
`lat_kd_heading`. With `lat_kp=0.32, lat_kd_heading=1.35` (pipeline.json defaults)
the controller is properly damped; without it, overshoot is common.
**Fix:** Use `g.lat_kd_heading * g.ego_heading` not `0.5 * g.ego_heading` at line 548.

### Pattern 3: NPC teleport visual jumps
**Symptom:** `max_npc_speed > 45 m/s` in evaluator output.
**Root cause:** `sim_world_node.c` obstacle recycling places NPCs 100-120m ahead
instantly when they leave the visible range. This is a known simulation artifact, not a control bug.

### Pattern 4: EKF convergence period misinitializes lane side (2026-07)
**Symptom:** ego drifts from y=-1.75 (left lane) to y>0 (right lane) in the first 0.5s, then ROAD_GUARD triggers.
**Root cause:** `fusion_node.cpp` EKF initializes with `x0[5] = {0,0,5,0,0}` — y=0, but ego actually starts at y=-1.75.
Before EKF converges (~10 cycles = 0.5s), fusion reports ego_y≈0. control_node's
`committed_lane_side = (ego_y < road_c) ? -1 : 1` evaluates to +1 (right lane) at ego_y=0,
pulling ego toward y=+1.75.
**Fix:** `control_node.cpp` ~line 706 — only initialize `committed_lane_side` when
`|ego_y - road_c| > 1.0` (EKF converged). While uncommitted (==0), `cruise_lane_y = ego_y`
(hold lateral position). Same 1.0m threshold applied to DATA_TIMEOUT fallback target.

### Pattern 5: ref_path heading corruption at junctions (2026-07)
**Symptom:** ego suddenly turns hard toward a fork/ramp as it approaches a junction (e.g., x=500).
**Root cause:** esmini's `Route::sample_ahead()` near junctions samples points on a connecting
road (ramp) whose tangent heading can differ from ego's current heading by ~5 rad.
Stanley `heading_term = lat_kd_heading * (ego_h - ref_h)` explodes, saturating steer.
**Fix:** `control_node.cpp` ~line 1048 — normalize `(ref_h - ego_heading)` to [-π,π];
if `|dh| > 0.5 rad` (≈29°) treat ref_h as invalid and use `ego_heading` (heading_term=0).
Sharp curve following is handled by `ff_term` (curvature feedforward), not heading_term.

### Pattern 6: NPCs on non-route segments project onto main road (2026-07)
**Symptom:** NPC placed on a connecting road (e.g., left_ramp segment 10) appears on the main
road in ego's lane, triggering phantom lane changes or collisions.
**Root cause:** If NPC's road is not on the main `Route::build()` chain, `npc_init_route()`
sets `route_dir=0`, falling back to world-frame integration in `npc_ai.cpp` line 314:
`world_to_frenet → frenet_to_world`. esmini's nearest-road query near junctions can project
the NPC onto the main road, causing positional drift.
**Fix:** When designing scenarios, avoid placing NPCs that need to travel long distances on
connecting roads. Use `vx=0` static vehicles, or move them to a segment on the main route.

### Pattern 7: Negative `s` values in scenario JSON (2026-07)
**Symptom:** Multiple NPCs stack at the route start point (x≈0), colliding with each other or blocking ego.
**Root cause:** `npc_init_route()` line 346 — `if (npc.s < 0.0) { npc.route_dir = 0; return; }`
Negative-s NPCs fall through to world-frame fallback, and esmini may locate them all at the
nearest road start.
**Fix:** All `s` values in scenario JSON actors must be ≥ 0. Negative s is semantically invalid
in OpenDRIVE (no road surface exists before s=0).

## Diagnostic commands

```bash
# Scan for collision/road-guard/stuck events
grep -E "COLLISION|ROAD_GUARD|STUCK|INTERVENED" /tmp/flow_launcher_stderr.txt

# Lane change trigger reasons (cur_gap/adj_gap/lead_v/ego pos/target_y)
grep "LANE CHANGE" /tmp/flow_launcher_stderr.txt

# Ego position snapshots (flowsim prints every 100 cycles)
grep "flowsim.*ego(" /tmp/flow_launcher_stderr.txt

# Control loop detail (spd/err/thr/brk/st/target_y/lc state)
grep "control       \] #" /tmp/flow_launcher_stderr.txt | tail -50

# List NPCs currently in ego's lane (find phantom obstacles)
python3 -c "
import json
d = json.load(open('/tmp/flow_topology.json'))
entities = d['metrics']['scene']['entities']
ego = [e for e in entities if e['type']=='ego'][0]
for e in entities:
    if e.get('type') in ('car','truck'):
        dx = e['x'] - ego['x']
        if -10 < dx < 130 and abs(e['y']-ego['y']) < 3:
            print(f\"id={e['id']:2d} x={e['x']:7.1f} y={e['y']:6.2f} dx={dx:7.1f} spd={e.get('spd',0):.1f}\")
"
```
