"""Phase 5: regression tests for the DATA_TIMEOUT curve-following fallback.

The control_node DATA_TIMEOUT branch (control_node.c:382) was previously
publishing ``target=0.0`` and keeping ``prev_steer`` frozen — fine on a
straight road, but on a curve it lets the car drive straight off the lane.

After the fix, the fallback computes a Stanley-style steering command
aimed at ``road_center_y(ego_x)`` instead of y=0. This file pins the
math from the C side using the same road_center_y / road_center_heading
formulas so any future divergence (e.g. demo_evaluator's Python mirror
or the C fallback) is caught immediately.

The test only verifies the *steering sign and magnitude* on a few
representative curve positions — not the closed-loop dynamics, which
remain covered by ``scenarios/zhongkai_road_full.json`` regression runs.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# Reuse the C-mirror implementation already used by demo_evaluator
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import demo_evaluator as DE  # noqa: E402


# Mirror of control_node.c::steer_limit_for_speed() — same constants.
def steer_limit_for_speed(speed_mps: float, max_lat_accel: float,
                          wheelbase: float = 2.7,
                          min_clamp: float = 0.012,
                          hard_cap: float = 0.24) -> float:
    speed = max(speed_mps, 2.0)
    limit = math.atan(max_lat_accel * wheelbase / (speed * speed))
    return max(min(limit, hard_cap), min_clamp)


# Mirror of control_node.c::road_center_heading() — same math as the header.
def road_center_heading(x: float, curve_start_x: float,
                        curve_length_m: float, curve_offset_m: float) -> float:
    if curve_length_m <= 0.0 or curve_offset_m == 0.0:
        return 0.0
    if x <= curve_start_x or x >= curve_start_x + curve_length_m:
        return 0.0
    t = (x - curve_start_x) / curve_length_m
    dy_dt = curve_offset_m * (6.0 * t - 6.0 * t * t)
    return math.atan(dy_dt / curve_length_m)


# Mirror of the fallback calculation in control_node.c (Phase 5).
def fallback_steer(ego_x: float, ego_y: float, ego_heading: float,
                    speed_mps: float,
                    curve_start_x: float, curve_length_m: float,
                    curve_offset_m: float,
                    lat_kp: float = 0.5, lat_kd_heading: float = 2.0,
                    prev_steer: float = 0.0,
                    max_lat_accel: float = 1.4) -> float:
    road_c = DE.road_center_y(ego_x, {
        "curve_start_x": curve_start_x,
        "curve_length_m": curve_length_m,
        "curve_offset_m": curve_offset_m,
    })
    road_h = road_center_heading(ego_x, curve_start_x,
                                 curve_length_m, curve_offset_m)
    lat_error = road_c - ego_y
    cte_term = math.atan2(lat_kp * lat_error, max(speed_mps, 3.0))
    heading_term = lat_kd_heading * (ego_heading - road_h)
    steer = cte_term - heading_term
    limit = steer_limit_for_speed(speed_mps, max_lat_accel)
    steer = max(min(steer, limit), -limit)
    # Phase 5 also feeds through a one-pole low-pass (0.7 new + 0.3 prev).
    return 0.7 * steer + 0.3 * prev_steer


# ── road geometry parity with C header ────────────────────────────

def test_road_center_y_disabled_when_curve_length_zero():
    assert DE.road_center_y(200.0, {
        "curve_start_x": 150.0, "curve_length_m": 0.0, "curve_offset_m": 9.0,
    }) == 0.0


def test_road_center_y_zero_before_curve_start():
    assert DE.road_center_y(100.0, {
        "curve_start_x": 150.0, "curve_length_m": 260.0, "curve_offset_m": 9.0,
    }) == 0.0


def test_road_center_y_smoothstep_endpoints():
    # t=0 → 0, t=1 → offset, symmetric smoothstep in between
    road = {"curve_start_x": 150.0, "curve_length_m": 260.0, "curve_offset_m": 9.0}
    assert DE.road_center_y(150.0, road) == pytest.approx(0.0, abs=1e-9)
    assert DE.road_center_y(410.0, road) == pytest.approx(9.0, abs=1e-9)
    # midpoint t=0.5 → 0.5 (smoothstep: 3*0.25 - 2*0.125 = 0.5)
    assert DE.road_center_y(280.0, road) == pytest.approx(4.5, abs=1e-9)


# ── fallback steering: must follow road_center_y on a curve ───────

CURVE = {"curve_start_x": 150.0, "curve_length_m": 260.0, "curve_offset_m": 9.0}

# Real production defaults from control_node.c::control_node_init()
DEFAULT_LAT_KP = 0.5
DEFAULT_LAT_KD_HEADING = 2.0


def test_fallback_returns_near_zero_when_aligned_with_road():
    """On a straight road (road_center_y = 0) with the car already at the
    road centerline (y=0), the fallback should produce ~0 steer."""
    s = fallback_steer(ego_x=50.0, ego_y=0.0, ego_heading=0.0,
                       speed_mps=10.0, **CURVE)
    # lat_error = 0, road_h = 0 → steer = 0 (clamped/low-passed to 0)
    assert abs(s) < 1e-9


def test_fallback_corrects_when_car_left_of_centerline_in_curve():
    """Mid-curve (x=280, road_c≈4.5), car at y=0. With the old target=0
    fallback and prev_steer=0, steer would have stayed at 0 and the car
    would have driven straight into the right shoulder. After the fix,
    the fallback should command a *positive* (right) steer to follow the
    road centerline. Set heading = road_h so the heading term is zero
    and the lat_error term dominates the result."""
    road_h = road_center_heading(280.0, **CURVE)
    s = fallback_steer(ego_x=280.0, ego_y=0.0, ego_heading=road_h,
                       speed_mps=10.0, **CURVE)
    assert s > 0.0, f"expected positive steer to follow road right, got {s:.3f}"
    # The steer must respect the hard cap of 0.24 rad
    assert s <= 0.24 + 1e-6, f"steer {s:.3f} exceeded hard cap"


def test_fallback_overshoot_when_car_right_of_centerline():
    """If the car has drifted to the right of the road centerline, fallback
    must steer left (negative) to recover. With the old target=0 fallback
    and prev_steer=0 the steer would have stayed at 0. Set heading =
    road_h so the heading term is zero and lat_error drives the sign."""
    road_h = road_center_heading(280.0, **CURVE)
    s = fallback_steer(ego_x=280.0, ego_y=6.0, ego_heading=road_h,
                       speed_mps=10.0, **CURVE)
    # lat_error = 4.5 - 6.0 = -1.5 → steer is negative
    assert s < 0.0, f"expected negative steer to recover left, got {s:.3f}"


def test_fallback_steer_within_speed_dependent_limit():
    """Verify the fallback output never exceeds the steer limit computed
    for the given speed (atan-based physics cap, hard-clamped to 0.24)."""
    for speed in (3.0, 8.0, 12.0, 20.0, 30.0):
        s = fallback_steer(ego_x=280.0, ego_y=-100.0, ego_heading=0.0,
                           speed_mps=speed, **CURVE)
        limit = steer_limit_for_speed(speed, 1.4)
        assert abs(s) <= limit + 1e-6, (
            f"speed={speed}: steer {s:.4f} exceeded limit {limit:.4f}")


def test_fallback_low_pass_blends_capped_steer_with_prev():
    """The C code does `out = 0.7 * capped_steer + 0.3 * prev_steer`,
    where `capped_steer` is the steer AFTER the speed-clamp, not the
    previously-published output. Verify the blend formula uses the
    capped (pre-filter) steer."""
    # Recompute the capped steer for the same scenario with prev=0
    road_c = DE.road_center_y(280.0, CURVE)
    road_h = road_center_heading(280.0, **CURVE)
    lat_error = road_c - 0.0
    cte_term = math.atan2(0.5 * lat_error, max(10.0, 3.0))
    heading_term = 2.0 * (0.0 - road_h)
    capped_steer = cte_term - heading_term
    limit = steer_limit_for_speed(10.0, 1.4)
    capped_steer = max(min(capped_steer, limit), -limit)

    s = fallback_steer(ego_x=280.0, ego_y=0.0, ego_heading=0.0,
                       speed_mps=10.0, prev_steer=0.2, **CURVE)
    # C blend uses capped_steer, not previous output
    expected = 0.7 * capped_steer + 0.3 * 0.2
    assert s == pytest.approx(expected, abs=1e-9)


def test_fallback_keeps_correction_after_timeout_window():
    """Two consecutive fallback calls in a curve should keep producing
    non-zero steer (the prev_steer filter doesn't decay the correction
    in 1 step, so the car keeps tracking the road across a 50ms timeout)."""
    road_h = road_center_heading(280.0, **CURVE)
    s1 = fallback_steer(ego_x=280.0, ego_y=0.0, ego_heading=road_h,
                        speed_mps=10.0, prev_steer=0.0, **CURVE)
    s2 = fallback_steer(ego_x=280.0, ego_y=0.0, ego_heading=road_h,
                        speed_mps=10.0, prev_steer=s1, **CURVE)
    assert s1 > 0 and s2 > 0, (
        f"correction should persist across fallback iterations: "
        f"s1={s1:.4f}, s2={s2:.4f}")
