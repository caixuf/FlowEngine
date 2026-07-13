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
