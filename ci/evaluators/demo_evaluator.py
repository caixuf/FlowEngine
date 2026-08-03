#!/usr/bin/env python3
"""Run and score the FlowEngine demo from observable runtime data.

The evaluator samples /tmp/flow_topology.json while scripts/demo.sh is running
and turns visual complaints such as collision, lane departure, stuck vehicle,
and missing topic data into repeatable PASS/FAIL checks.
"""

from __future__ import annotations

import argparse
import collections
import contextlib
import json
import math
import os
import re
import select
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]  # ci/evaluators/ → 项目根
DEFAULT_JSON = Path("/tmp/flow_topology.json")
LAUNCHER_STDERR = Path("/tmp/flow_launcher_stderr.txt")
PIPELINE_JSON = ROOT / "config" / "pipeline.json"

TOPIC_MIN_FREQ = {
    "vehicle/state": 15.0,
    "sensor/lidar": 15.0,
    "sensor/gps": 7.0,
    "fusion/localization": 17.0,  # 名义 20Hz, FlowCoro 协程事件驱动有 ~7% OS 调度抖动,
                                  # 17Hz 给 15% soft real-time 容差（业界标准）。
                                  # 旧值 19Hz (95% 名义) 过严，实测 18.6Hz 即 FAIL。
    "planning/trajectory": 5.0,
    "control/raw_cmd": 10.0,
    "control/cmd": 10.0,
}

# Minimum expected publish frequency for inference/trajectory when inference_node is active.
INFERENCE_TOPIC_MIN_FREQ = 5.0

# Shadow inference output files written by torch_sidecar.py / tiny_sidecar.py.
SHADOW_INFERENCE_FILES = [
    Path("/tmp/flow_torch_inference.json"),
    Path("/tmp/flow_tiny_inference.json"),
]

# Thresholds for shadow_delta = inference_speed - planning_speed (m/s).
SHADOW_DELTA_WARN = 2.0   # |delta| > this → WARN
SHADOW_DELTA_FAIL = 5.0   # |delta| > this → FAIL

# ── Task 5：分层识别率 / 预警提前量阈值 ──
# 真值实体（flowsim scene.entities）与感知障碍（scene.obstacles）匹配距离阈值。
# 同帧同位置的 truth 与 perceived 距离 ≤ 此值视为"识别成功"。
PERCEPTION_MATCH_DIST_M = 3.0
# 单类型识别率低于此阈值 → WARN（提示该类型感知能力下降）。
PERCEPTION_RATE_WARN = 0.80
# 单类型识别率低于此阈值 → FAIL（明显漏检，影响安全）。
PERCEPTION_RATE_FAIL = 0.50
# TTC 临界阈值：forward gap / ego_speed 低于此值视为"碰撞风险临界事件"，
# 用于计算预警提前量（系统在临界事件发生前多久就检测到了该障碍）。
TTC_CRITICAL_S = 3.0
# 预警提前量低于此阈值 → WARN（感知/规划反应过晚）。
WARNING_LEAD_WARN_S = 1.0
# 预警提前量低于此阈值 → FAIL（系统几乎无预警，紧急刹车）。
WARNING_LEAD_FAIL_S = 0.3
# ACC 期望间距参数 —— 必须与 behavior_planner_node.cpp 的 acc_* 默认值一致。
# 门禁用它把"跟车间距够不够"变成随车速伸缩的判据，而不是固定 5m。
ACC_STANDOFF_M = 5.0
ACC_TIME_HEADWAY_S = 1.5

# ── 控制层量化门禁（Phase 1.3）：病理性抖动 FAIL 阻断 ──
# 抓 bang-bang 转向（MPC 每帧翻符号）与 1-2Hz 横向极限环。
# 健康实测（straight_road 30s，2026-08-01）：yaw_rms≈0.02 rad/s、
# steer_flip≈0.0/s、steer_rms≈0.02/s。FAIL 阈值取健康值 ~10×，
# 只拦"每帧翻符号 / 持续高频抖动"这种不可能靠运气过去的现象。
# 旧实现只有 WARN：bang-bang 时 steer_flip≈9-10/s 也照样 PASS，
# 于是"修好后没复发"从没人验证过 —— 门禁抓不住已知故障 = 它的 PASS 不可信。
# WARN 阈值命名成常量，避免与 FAIL 阈值同写一处时漂移。
YAW_RMS_WARN = 0.35         # rad/s
YAW_MAX_WARN = 1.2          # rad/s
HEADING_FLIP_WARN = 1.2     # /s
HEADING_FLIP_WARN_MIN_YAW = 0.1  # rad/s，flip 佐证下界
STEER_RATE_RMS_WARN = 0.9   # /s
STEER_MAX_RATE_WARN = 3.0   # /s
STEER_FLIP_WARN = 1.0       # /s
YAW_RMS_FAIL = 0.6          # rad/s
YAW_MAX_FAIL = 2.0          # rad/s
HEADING_FLIP_FAIL = 3.0     # /s，且需 yaw_rms>0.2 佐证（弯道上 heading 单调变化不算）
STEER_RATE_RMS_FAIL = 1.6   # /s
STEER_MAX_RATE_FAIL = 6.0   # /s
STEER_FLIP_FAIL = 2.5       # /s
# 实际最小间距低于期望间距的这个比例 → FAIL。取 0.5 是因为 ACC 有超调是
# 正常的，但掉到期望值一半以下意味着间距根本没被控制。
ACC_GAP_FAIL_RATIO = 0.5
# 真值实体 type → 分层类别映射（与 flowsim entity.h::EntityType 对齐）。
TRUTH_TYPE_VEHICLE = {"car", "truck", "suv"}
TRUTH_TYPE_VRU = {"pedestrian"}  # Vulnerable Road User
TRUTH_TYPE_INFRA = {"ego", "tl", "etc_gate"}  # 基础设施，不计入识别率

# 绿灯卡死检测的近邻窗口 (m)：只有 ego 在停止线前后这个范围内，才算"在等这盏灯"。
# 取 60m 与 planning 注入红绿灯虚拟墙的前瞻距离一致（planning_node.cpp dx_tl > 60 跳过）。
GREEN_STOP_NEAR_M = 60.0
TRUTH_LAYER_FOR_TYPE = {
    "car": "vehicle",
    "truck": "vehicle",
    "suv": "vehicle",
    "pedestrian": "vru",
}


def _load_shadow_delta() -> float | None:
    """Read the latest shadow_delta value from any active sidecar output file.

    Returns the most recently written shadow_delta, or None if no sidecar file exists.
    """
    best_path: Path | None = None
    best_mtime = -1.0
    for path in SHADOW_INFERENCE_FILES:
        try:
            mtime = path.stat().st_mtime
            if mtime > best_mtime:
                best_mtime = mtime
                best_path = path
        except FileNotFoundError:
            pass
    if best_path is None:
        return None
    try:
        with best_path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        delta = data.get("shadow_delta")
        return float(delta) if delta is not None else None
    except (json.JSONDecodeError, OSError, ValueError):
        return None


def _pipeline_nodes(pipeline: dict) -> list:
    """Return the node list of a launcher config.

    The launcher schema uses ``processes``; older/simpler configs use ``nodes``.
    Accept either so scenario tooling works across both.
    """
    if not isinstance(pipeline, dict):
        return []
    nodes = pipeline.get("processes")
    if not isinstance(nodes, list):
        nodes = pipeline.get("nodes")
    return nodes if isinstance(nodes, list) else []


def _topic_name(pub_entry) -> str | None:
    if isinstance(pub_entry, str):
        return pub_entry
    if isinstance(pub_entry, dict) and isinstance(pub_entry.get("topic"), str):
        return pub_entry["topic"]
    return None


def expected_edges_from_pipeline(pipeline: dict) -> list[tuple[str, str, str]]:
    publishers: dict[str, list[str]] = {}
    subscribers: list[tuple[str, str]] = []
    for node in _pipeline_nodes(pipeline):
        if not isinstance(node, dict):
            continue
        name = node.get("name")
        if not isinstance(name, str):
            continue
        for pub in node.get("publish", []) or []:
            topic = _topic_name(pub)
            if topic:
                publishers.setdefault(topic, []).append(name)
        for topic in node.get("subscribe", []) or []:
            if isinstance(topic, str):
                subscribers.append((name, topic))

    edges = []
    for sub_node, topic in subscribers:
        for pub_node in publishers.get(topic, []):
            if pub_node != sub_node:
                edges.append((pub_node, topic, sub_node))
    return edges


def load_scenario_criteria_from_pipeline() -> tuple[dict, str | None, bool, dict | None, list]:
    """Load pass_criteria from config/pipeline.json -> flowsim.params.scenario_file.

    Returns:
        (criteria_dict, scenario_name, has_noa_route, road, traffic_lights)

    ``has_noa_route`` is True when the scenario defines a non-empty ``route``
    list, meaning the driving-mode state machine is expected to reach NOA
    and actively change lanes per the navigation route (see docs/tutorials/08_state_machine.md).

    ``road`` is the scenario's optional "road" object (curve_start_x/
    curve_length_m/curve_offset_m), or None for straight-road scenarios —
    used by score()/sample_metrics() to compute lane_error/road_edge_margin
    relative to the (possibly curved) road centerline instead of a fixed y=0.

    ``traffic_lights`` is the scenario's optional "traffic_lights" list
    (each entry: x, y_lane, red_s, yellow_s, green_s, phase_offset_s),
    or [] for scenarios without signalized intersections — used by score()
    to check red-light violations (ego crossing stop line during red phase).
    """
    pipeline = load_json(PIPELINE_JSON) or {}
    nodes = _pipeline_nodes(pipeline)
    scenario_file = None
    for node in nodes:
        if not isinstance(node, dict):
            continue
        if node.get("name") != "flowsim":
            continue
        params = node.get("params", {})
        if isinstance(params, str):
            try:
                params = json.loads(params)
            except json.JSONDecodeError:
                print("warning: flowsim.params is not valid JSON; skipping scenario_file lookup",
                      file=sys.stderr)
                params = {}
        if isinstance(params, dict):
            scenario_file = params.get("scenario_file")
        break

    if not scenario_file:
        return {}, None, False, None, []

    scenario_path = Path(scenario_file)
    if not scenario_path.is_absolute():
        scenario_path = ROOT / scenario_path

    scenario = load_json(scenario_path)
    if not isinstance(scenario, dict):
        return {}, None, False, None, []

    criteria = scenario.get("pass_criteria", {})
    if not isinstance(criteria, dict):
        criteria = {}
    name = scenario.get("name") if isinstance(scenario.get("name"), str) else None
    has_route = bool(scenario.get("route"))
    road = scenario.get("road")
    if not isinstance(road, dict):
        road = None
    traffic_lights = scenario.get("traffic_lights", [])
    if not isinstance(traffic_lights, list):
        traffic_lights = []
    return criteria, name, has_route, road, traffic_lights


def load_pipeline_expected_edges() -> list[tuple[str, str, str]]:
    pipeline = load_json(PIPELINE_JSON) or {}
    return expected_edges_from_pipeline(pipeline)


def _pipeline_flowsim_scenario_file() -> str | None:
    """Return the scenario_file path configured in config/pipeline.json's
    flowsim node params (or None if not found). Used to pass the pipeline's
    default scenario to demo.sh --scenario, so that demo.sh does not override
    it with its own DEFAULT_SCENARIO (infinite_straight.json, no route)."""
    pipeline = load_json(PIPELINE_JSON) or {}
    for node in _pipeline_nodes(pipeline):
        if not isinstance(node, dict) or node.get("name") != "flowsim":
            continue
        params = node.get("params", {})
        if isinstance(params, str):
            try:
                params = json.loads(params)
            except json.JSONDecodeError:
                return None
        if isinstance(params, dict):
            return params.get("scenario_file")
        return None
    return None


def load_scenario_for_duration(scenario_override: str | None = None) -> dict:
    """Load scenario JSON to read duration_s for auto-detection."""
    scenario_path = scenario_override
    if not scenario_path:
        # Read default scenario from demo.sh
        demo_sh = ROOT / "scripts" / "demo.sh"
        if demo_sh.exists():
            text = demo_sh.read_text()
            for line in text.splitlines():
                if line.strip().startswith("DEFAULT_SCENARIO="):
                    scenario_path = line.split("=", 1)[1].strip().strip('"')
                    break
    if scenario_path:
        return load_json(ROOT / scenario_path) or {}
    return {}


def load_json(path: Path) -> dict | None:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


@contextlib.contextmanager
def pipeline_scenario_override(scenario_file: str | None):
    """Temporarily point config/pipeline.json's scenario-aware nodes at ``scenario_file``.

    Node ``params`` is a JSON-encoded string; we patch the embedded
    ``scenario_file`` key, yield, then restore the original file byte-for-byte.
    Passing ``None`` is a no-op so callers can use this unconditionally.
    planning and navigation also reading scenario_file lets route-driven
    lane changes / branch selection use the same scene as flowsim during
    evaluator runs, instead of mixing actor placement from one scenario
    with navigation steps from another.
    """
    if not scenario_file:
        yield None
        return

    original_text = PIPELINE_JSON.read_text(encoding="utf-8")
    pipeline = json.loads(original_text)
    patched_nodes = []
    for node in _pipeline_nodes(pipeline):
        if not isinstance(node, dict) or node.get("name") not in ("flowsim", "planning", "navigation", "control"):
            continue
        params = node.get("params")
        # params may be a JSON string (launcher format) or a plain dict.
        if isinstance(params, str):
            params_obj = json.loads(params)
            params_obj["scenario_file"] = scenario_file
            node["params"] = json.dumps(params_obj)
        elif isinstance(params, dict):
            params["scenario_file"] = scenario_file
        else:
            continue
        patched_nodes.append(node["name"])

    if "flowsim" not in patched_nodes:
        raise RuntimeError("flowsim node with params not found in config/pipeline.json")
    if "planning" not in patched_nodes:
        print("warning: planning node not patched with scenario_file (missing/malformed "
              "params?) — route-driven planning will not be exercised this run",
              file=sys.stderr)
    if "navigation" not in patched_nodes:
        print("warning: navigation node not patched with scenario_file (missing/malformed "
              "params?) — route steps may come from a different scenario this run",
              file=sys.stderr)

    try:
        PIPELINE_JSON.write_text(json.dumps(pipeline, indent=2, ensure_ascii=False) + "\n",
                                 encoding="utf-8")
        yield scenario_file
    finally:
        PIPELINE_JSON.write_text(original_text, encoding="utf-8")


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


def road_center_y(x: float, road: dict | None) -> float:
    """Mirror of include/road_geometry.h::road_center_y() — must stay in sync
    with the C implementation shared by flowsim/planning/control nodes.
    Returns 0.0 (straight road) when ``road`` is None/absent or the curve is
    disabled (curve_length_m <= 0 or curve_offset_m == 0), matching every
    existing scenario file that has no "road" section."""
    if not road:
        return 0.0
    curve_start_x = float(road.get("curve_start_x", 0.0) or 0.0)
    curve_length_m = float(road.get("curve_length_m", 0.0) or 0.0)
    curve_offset_m = float(road.get("curve_offset_m", 0.0) or 0.0)
    if curve_length_m <= 0.0 or curve_offset_m == 0.0:
        return 0.0
    if x <= curve_start_x:
        return 0.0
    t = (x - curve_start_x) / curve_length_m
    if t >= 1.0:
        return curve_offset_m
    return curve_offset_m * (3.0 * t * t - 2.0 * t * t * t)


def lane_center_y(lane_idx: int, lane_count: int, lane_width: float, road_c: float = 0.0,
                  side_offset: float = 0.0) -> float:
    """Mirror of include/road_geometry.h::lane_center_y() — 靠右行驶 v2。
    lane_idx: 0=最左, lane_count-1=最右；road_c=道路中心 y 坐标。
    side_offset: 车道组整体偏移（0=关于 road_c 对称，负值=向 -y 偏移）。"""
    if lane_count <= 1:
        return road_c
    return road_c + side_offset - (lane_idx - (lane_count - 1) * 0.5) * lane_width


def lane_idx_from_y(y: float, lane_count: int, lane_width: float, road_c: float = 0.0,
                    side_offset: float = 0.0) -> int:
    """Mirror of include/road_geometry.h::lane_idx_from_y() — 反推车道索引，
    clamp 到 [0, lane_count-1]。
    side_offset: 与 lane_center_y 相同的 side_offset 值。"""
    if lane_count <= 1:
        return 0
    offset = (road_c + side_offset - y) / lane_width + (lane_count - 1) * 0.5
    idx = int(round(offset))
    if idx < 0:
        idx = 0
    if idx >= lane_count:
        idx = lane_count - 1
    return idx


def nearest_lane_error(y: float, lane_width: float = 3.5, lane_count: int = 2, road_c: float = 0.0) -> float:
    """ego 横向位置 y（相对道路中心 road_c）到最近车道中心的最小距离。
    N 车道模型：lane_count=2 时退化为 [-lane_width*0.5, +lane_width*0.5]，与旧实现一致。"""
    if lane_count <= 1:
        return abs(y - road_c)
    lane_centers = [lane_center_y(i, lane_count, lane_width, road_c) for i in range(lane_count)]
    return min(abs(y - c) for c in lane_centers)


def _road_network_cross_track(scene: dict, x: float, y: float) -> tuple[float, float] | None:
    """Return (lane_error, road_edge_margin) from scene.road_network if present.

    The legacy evaluator only mirrored ``road_center_y(x)`` for single-curve
    scenarios. Multi-edge road-network scenes publish sampled polyline nodes in
    ``scene.road_network.edges[].nodes``; use the nearest segment's local normal
    to measure lateral offset against the actual routed road geometry.
    """
    road_network = scene.get("road_network")
    if not isinstance(road_network, dict):
        return None
    edges = road_network.get("edges")
    if not isinstance(edges, list):
        return None

    best: tuple[float, float, int, float, float] | None = None
    px = float(x)
    py = float(y)
    for edge in edges:
        if not isinstance(edge, dict):
            continue
        nodes = edge.get("nodes")
        if not isinstance(nodes, list) or len(nodes) < 2:
            continue
        lane_width = float(edge.get("lane_width", 3.5) or 3.5)
        lane_count = int(edge.get("lanes", 2) or 2)
        if lane_width <= 0.0 or lane_count <= 0:
            continue
        for i in range(len(nodes) - 1):
            a = nodes[i]
            b = nodes[i + 1]
            if not (isinstance(a, list) and isinstance(b, list) and len(a) >= 2 and len(b) >= 2):
                continue
            ax, ay = float(a[0]), float(a[1])
            bx, by = float(b[0]), float(b[1])
            dx = bx - ax
            dy = by - ay
            seg_len2 = dx * dx + dy * dy
            if seg_len2 <= 1e-9:
                continue
            t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            cx = ax + t * dx
            cy = ay + t * dy
            seg_len = math.sqrt(seg_len2)
            nx = -dy / seg_len
            ny = dx / seg_len
            signed_offset = (px - cx) * nx + (py - cy) * ny
            dist = math.hypot(px - cx, py - cy)
            margin = lane_width * lane_count * 0.5 - abs(signed_offset) - 1.0
            if (best is None or
                    margin > best[4] + 1e-9 or
                    (abs(margin - best[4]) <= 1e-9 and dist < best[0])):
                best = (dist, signed_offset, lane_count, lane_width, margin)

    if best is None:
        return None

    _, signed_offset, lane_count, lane_width, margin = best
    return (
        nearest_lane_error(signed_offset, lane_width, lane_count, 0.0),
        margin,
    )


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def sample_metrics(sample: dict, road: dict | None = None) -> dict:
    metrics = sample.get("metrics", {})
    vehicle = metrics.get("vehicle", {})
    scene = metrics.get("scene", {})
    ego = scene.get("ego", {})
    obstacles = scene.get("obstacles", [])
    lane = scene.get("lane", {})
    scn_entities = scene.get("entities", [])

    speed = float(vehicle.get("speed", ego.get("speed", 0.0)) or 0.0)
    x = float(vehicle.get("x", ego.get("x", 0.0)) or 0.0)
    y = float(ego.get("y", 0.0) or 0.0)
    heading = float(ego.get("heading", 0.0) or 0.0)
    steer = abs(float(ego.get("steer", 0.0) or 0.0))
    steer_signed = float(ego.get("steer", 0.0) or 0.0)
    lane_width = float(lane.get("width", 3.5) or 3.5)
    lane_count = int(lane.get("count", 2) or 2)

    # 优先按 scene.road_network 的真实 polyline 几何计算横向偏差；旧场景仍回退
    # 到 road_center_y(x) 的单段弯道镜像，保持既有行为不变。
    center = road_center_y(x, road)
    y_rel = y - center
    cross_track = _road_network_cross_track(scene, x, y)
    if cross_track is not None:
        lane_error, road_edge_margin = cross_track
    else:
        lane_error = nearest_lane_error(y_rel, lane_width, lane_count, 0.0)
        road_edge_margin = lane_width * lane_count * 0.5 - abs(y_rel) - 1.0

    min_forward_gap = math.inf
    min_abs_gap = math.inf
    obs_world = []
    for obs in obstacles:
        rel_x = float(obs.get("x", math.inf))
        rel_y_signed = float(obs.get("y", math.inf))
        if not math.isfinite(rel_x) or not math.isfinite(rel_y_signed):
            continue
        rel_y = abs(rel_y_signed)
        length = float(obs.get("len", 4.6) or 4.6)
        width = float(obs.get("wid", 2.0) or 2.0)
        obs_world.append({
            "id": int(obs.get("id", len(obs_world)) or len(obs_world)),
            "x": x + rel_x,
            "y": y + rel_y_signed,
        })
        gap_x = abs(rel_x) - (4.6 + length) * 0.5
        gap_y = rel_y - (2.0 + width) * 0.5
        min_abs_gap = min(min_abs_gap, max(gap_x, gap_y))
        if rel_x > 0 and rel_y < 2.5:
            min_forward_gap = min(min_forward_gap, rel_x - (4.6 + length) * 0.5)

    # P3: 提取每个 truth entity 的 tp_cycle（last_teleport_cycle），供 NPC teleport
    # 检查区分合法 recycle（设计内瞬移）vs id-collision pollution。entity id 与
    # obstacle id 都是 pool index（flowsim entity.id），可直接关联。
    tp_cycle_by_id: dict[int, int] = {}
    if isinstance(scn_entities, list):
        for ent in scn_entities:
            if not isinstance(ent, dict):
                continue
            eid = ent.get("id")
            if eid is None:
                continue
            try:
                tp_cycle_by_id[int(eid)] = int(ent.get("tp_cycle", 0) or 0)
            except (TypeError, ValueError):
                continue

    return {
        "speed": speed,
        "x": x,
        "y": y,
        "heading": heading,
        "steer": steer,
        "steer_signed": steer_signed,
        "lane_error": lane_error,
        "road_edge_margin": road_edge_margin,
        "lane_count": lane_count,
        "y_rel": y_rel,
        "min_forward_gap": min_forward_gap,
        "min_abs_gap": min_abs_gap,
        "obs_world": obs_world,
        "driver_mode": str(metrics.get("driver_mode", "") or ""),
        "route_lane": int(metrics.get("route_lane", 0) or 0),
        "entities": scn_entities if isinstance(scn_entities, list) else [],
        "tp_cycle_by_id": tp_cycle_by_id,
    }


def sign_flips(values: list[float], deadband: float) -> int:
    flips = 0
    prev = 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign == 0:
            continue
        if prev and sign != prev:
            flips += 1
        prev = sign
    return flips


def _compute_perception_metrics(series: list[dict], timestamps: list[float]) -> dict:
    """Task 5：分层识别率 + 预警提前量。

    分层识别率（layered recognition rate）：
        把 truth 实体（flowsim scene.entities）按 type 分两层 — vehicle
        (car/truck/suv) 与 vru (pedestrian)。对每帧每层每个 truth 实体，
        在同帧 perceived 障碍（scene.obstacles 转世界坐标后的 obs_world）
        中查找距离 ≤ PERCEPTION_MATCH_DIST_M 的最近一个；命中即视为"识别成功"。
        聚合所有帧得 vehicle / vru / overall 三档识别率。

    预警提前量（warning lead time）：
        对每个 perceived 障碍（按 id 跨帧跟踪），记录其首次被检测到的时刻
        first_detect_ts；同时按 forward gap / ego_speed 计算 TTC，记录其
        首次跌破 TTC_CRITICAL_S 的时刻 first_critical_ts。预警提前量 =
        first_critical_ts - first_detect_ts（值越大说明系统越早检测到危险）。
        对所有发生临界事件的障碍取平均与最小值。

    返回 dict，所有字段空数据时返回 0/空，不抛异常。
    """
    match_d2 = PERCEPTION_MATCH_DIST_M * PERCEPTION_MATCH_DIST_M

    # ── 分层识别率累积器 ──
    # key = 'vehicle' / 'vru' / 'overall'；value = [matched, total]
    layer_counts: dict[str, list[int]] = {
        "vehicle": [0, 0],
        "vru": [0, 0],
        "overall": [0, 0],
    }
    # 同时按细粒度 type 累积（car/truck/suv/pedestrian），用于诊断输出
    type_counts: dict[str, list[int]] = collections.defaultdict(lambda: [0, 0])

    # ── 预警提前量累积器 ──
    # obs_id → first detection ts（同 id 跨帧去重）
    first_detect_ts: dict[int, float] = {}
    # obs_id → first critical TTC ts（首次跌破 TTC_CRITICAL_S）
    first_critical_ts: dict[int, float] = {}
    # obs_id → 最小 TTC（用于报告 min_ttc_s）
    obs_min_ttc: dict[int, float] = {}
    # 已经记录过 first_critical 的 obs_id 集合，避免重复触发
    critical_recorded: set[int] = set()

    for i, m in enumerate(series):
        ts_i = timestamps[i] if i < len(timestamps) else 0.0
        ego_x = m["x"]
        # truth 实体：跳过 ego/tl/etc_gate 等基础设施
        truth = []
        for ent in m.get("entities", []):
            if not isinstance(ent, dict):
                continue
            etype = str(ent.get("type", "") or "")
            if etype in TRUTH_TYPE_INFRA or not etype:
                continue
            try:
                truth.append({
                    "id": ent.get("id"),
                    "type": etype,
                    "x": float(ent.get("x", 0.0) or 0.0),
                    "y": float(ent.get("y", 0.0) or 0.0),
                })
            except (TypeError, ValueError):
                continue
        # perceived 障碍（obs_world 已是世界坐标）
        perceived = m.get("obs_world", []) or []

        # 1) 分层识别率
        for t in truth:
            layer = TRUTH_LAYER_FOR_TYPE.get(t["type"], "vehicle")
            layer_counts[layer][1] += 1
            layer_counts["overall"][1] += 1
            type_counts[t["type"]][1] += 1
            best_d2 = match_d2
            for p in perceived:
                dx = p["x"] - t["x"]
                dy = p["y"] - t["y"]
                d2 = dx * dx + dy * dy
                if d2 <= best_d2:
                    best_d2 = d2
            if best_d2 < match_d2:
                layer_counts[layer][0] += 1
                layer_counts["overall"][0] += 1
                type_counts[t["type"]][0] += 1

        # 2) 预警提前量：对 perceived 障碍跨帧跟踪 + TTC 监测
        ego_speed = m["speed"]
        for p in perceived:
            pid = p.get("id")
            if pid is None:
                continue
            # 首次检测时间戳
            if pid not in first_detect_ts:
                first_detect_ts[pid] = ts_i
            # TTC = forward gap / ego_speed；forward gap 用世界坐标 dx
            # （perceived 在 ego 前方时 p.x > ego_x）
            rel_x = p["x"] - ego_x
            if rel_x <= 0:
                continue  # 仅前方障碍纳入 TTC
            if ego_speed > 0.5:
                ttc = rel_x / ego_speed
                prev_min = obs_min_ttc.get(pid, math.inf)
                if ttc < prev_min:
                    obs_min_ttc[pid] = ttc
                # 首次跌破临界阈值
                if ttc < TTC_CRITICAL_S and pid not in critical_recorded:
                    first_critical_ts[pid] = ts_i
                    critical_recorded.add(pid)

    # 汇总识别率
    def _rate(counts: list[int]) -> float:
        return counts[0] / counts[1] if counts[1] > 0 else 1.0  # 无样本视为 1.0（不影响判定）

    rate_vehicle = _rate(layer_counts["vehicle"])
    rate_vru = _rate(layer_counts["vru"])
    rate_overall = _rate(layer_counts["overall"])
    rate_by_type = {t: _rate(c) for t, c in type_counts.items()}

    # 汇总预警提前量（秒）
    lead_times: list[float] = []
    for pid, crit_ts in first_critical_ts.items():
        det_ts = first_detect_ts.get(pid, crit_ts)
        lead = crit_ts - det_ts
        if lead >= 0:  # 异常负值（感知先于真值出现）跳过
            lead_times.append(lead)
    avg_lead = statistics.fmean(lead_times) if lead_times else 0.0
    min_lead = min(lead_times) if lead_times else 0.0
    min_ttc_overall = min(obs_min_ttc.values()) if obs_min_ttc else math.inf
    crit_event_count = len(lead_times)

    return {
        "recognition_rate_vehicle": rate_vehicle,
        "recognition_rate_vru": rate_vru,
        "recognition_rate_overall": rate_overall,
        "recognition_rate_by_type": {t: round(r, 3) for t, r in rate_by_type.items()},
        "truth_count_vehicle": layer_counts["vehicle"][1],
        "truth_count_vru": layer_counts["vru"][1],
        "truth_count_overall": layer_counts["overall"][1],
        "warning_lead_avg_s": avg_lead,
        "warning_lead_min_s": min_lead,
        "critical_event_count": crit_event_count,
        "min_ttc_s": min_ttc_overall if math.isfinite(min_ttc_overall) else None,
        "perceived_track_count": len(first_detect_ts),
    }


def collect_samples(duration: int, json_file: Path, interval: float,
                    scenario: str | None = None) -> tuple[list[dict], int]:
    try:
        json_file.unlink()
    except FileNotFoundError:
        pass

    started_wall = time.time()
    # 传 --scenario 给 demo.sh，确保 demo.sh 不会用 DEFAULT_SCENARIO（infinite_straight，
    # 无 route）覆盖 pipeline.json 的 scenario_file。否则 planning 运行时加载的是
    # infinite_straight，route_count=0，NOA guard 永远拒绝，模式停在 NP。
    cmd = [str(ROOT / "scripts" / "demo.sh"), "--no-browser"]
    if scenario:
        cmd += ["--scenario", scenario]
    cmd += [str(duration)]
    proc = subprocess.Popen(
        cmd,
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )

    # 兜底：评估器被 kill（SIGTERM/SIGINT，如 CI 超时）时不留孤儿 demo.sh ——
    # demo.sh 是 start_new_session 的独立会话，默认信号传播不到它。
    # 处理完清理后恢复默认处置并重发，保证进程按原语义终止（如 SIGTERM→143）。
    _demo_pgid = proc.pid

    def _kill_demo_session(sig, _frm):
        try:
            os.killpg(_demo_pgid, 15)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        signal.signal(sig, signal.SIG_DFL)
        os.kill(os.getpid(), sig)

    _prev_sigterm = signal.signal(signal.SIGTERM, _kill_demo_session)
    _prev_sigint = signal.signal(signal.SIGINT, _kill_demo_session)

    samples: list[dict] = []
    started = time.monotonic()
    # 缓冲时间：demo.sh 自身总时长 = 构建(0-5s) + wait-for-JSON(最多 15s)
    # + sleep 1+2 + 监控循环(duration + 每帧 fork python3 ~10s) + cleanup(3-5s)。
    # 旧值 duration+30s 在冷启动/CI 上经常不够，导致 SIGTERM 截断 demo.sh
    # 且孤儿进程残留。改为 duration+60s 覆盖最坏情况。
    deadline = started + duration + 60.0
    first_sample_seen = False
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
            if not first_sample_seen:
                first_sample_seen = True
                # 收到首个有效样本后再给运行时长 + 30s 收尾缓冲，
                # 覆盖 demo.sh 监控循环的 python3 fork 开销 + cleanup。
                deadline = max(deadline, time.monotonic() + duration + 30.0)
        time.sleep(interval)

    if proc.poll() is None:
        print(f"warning: demo.sh still running after {time.monotonic() - started:.1f}s, terminating",
              file=sys.stderr)
        # 用进程组信号确保 demo.sh 的子孙进程（flow_launcher / flowmond /
        # foxglove_bridge / flow_node_host）也被一并清理，避免孤儿残留。
        try:
            os.killpg(os.getpgid(proc.pid), 15)  # SIGTERM
            proc.wait(timeout=10.0)
        except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
            try:
                os.killpg(os.getpgid(proc.pid), 9)  # SIGKILL
            except (ProcessLookupError, PermissionError):
                pass
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass
    # 读 demo.sh 输出仅用于展示（评估数据源是 /tmp/flow_topology.json 采样）。
    # 不能 read() 等 EOF：demo.sh 的后台 tail/grep 泄漏进程可能持有本管道写端
    # （stderr=STDOUT 继承），read() 将无限阻塞 → 评估挂死。用 select 限时读取，
    # 3s 无新数据即止，泄漏进程持管也不会阻塞评估。
    output = ""
    if proc.stdout:
        fd = proc.stdout.fileno()
        while True:
            try:
                ready, _, _ = select.select([fd], [], [], 3.0)
            except (OSError, ValueError):
                break  # 管道已关闭
            if not ready:
                break  # 3s 空闲 → 停止（可能仍有泄漏进程持有写端，不等待）
            try:
                chunk = os.read(fd, 65536)
            except OSError:
                break
            if not chunk:
                break  # 真 EOF
            output += chunk.decode("utf-8", errors="replace")
        proc.stdout.close()
    signal.signal(signal.SIGTERM, _prev_sigterm)
    signal.signal(signal.SIGINT, _prev_sigint)
    if output:
        print(output.rstrip())
    return samples, proc.returncode or 0


# P3 诊断辅助：dump 单帧 sample 到 /tmp，供分析 NPC 瞬移根因。
class _DbgDumped:
    done = False

_dbg_dumped = _DbgDumped()


def _dbg_dump_sample(filename: str, sample: dict) -> None:
    """把单帧 entities 写到 /tmp/<filename>，仅保留 NPC 车辆 + ego，按 id 排序。
    sample 可以是 sample_metrics() 输出（entities 在顶层）或原始 topology JSON
    （entities 在 metrics.scene.entities）。"""
    import json as _json
    ents = sample.get("entities") if isinstance(sample, dict) else None
    if ents is None:
        scn = sample.get("scene", {}) if isinstance(sample, dict) else {}
        ents = scn.get("entities", []) if isinstance(scn, dict) else []
    rows = []
    for e in ents:
        if not isinstance(e, dict):
            continue
        t = e.get("type")
        if t in TRUTH_TYPE_VEHICLE or t == "ego":
            rows.append({
                "id": e.get("id"), "type": t,
                "x": e.get("x"), "y": e.get("y"),
                "vx": e.get("vx"), "tp_cycle": e.get("tp_cycle", 0),
                "heading": e.get("heading"),
            })
    rows.sort(key=lambda r: (r.get("type") != "ego", r.get("id", 0)))
    with open(f"/tmp/{filename}", "w") as fh:
        _json.dump({"entity_count": len(ents), "vehicles": rows}, fh, indent=2)


"""── 门禁有效性层 (liveness gate) ──────────────────────────────

本项目的门禁被静默绕过至少六次，每次机制不同：
  1. 分母为 0 → 比率算出满分   (truth_count_vru=0 → recognition_rate 1.000)
  2. 被检查量恒为 0 → 上界永满足 (kappa≡0 → `|kappa|>0.25` 永假)
  3. 判据写进 warnings → 可忽略  (steer_flip_rate)
  4. 上游欠采样 → 目标频率混叠
  5. 场景不含该情形 → 判据空跑
  6. 指标与事实错配        (warning_lead=51s 却发生碰撞)

逐条补特例永远落后一个，因为它们共享同一个结构缺陷：
**门禁不知道自己有没有真的测到东西。**

下面两个函数把「测到了吗」变成一等判据：

- `liveness_report()` —— 统计每个进入判据的量在整个 run 里的取值分布。
  恒为初值/恒为单值 = 该量结构性死掉，依赖它的判据全部无效。
  这一条不需要枚举「哪些量可能为零」，覆盖上面 1/2/5 三种。
- `require()` —— 判据显式声明前置条件；不满足时记为 INCONCLUSIVE
  并计入 failures，而不是 `continue` 掉。无法判定 ≠ 通过。
"""

# 必须"活着"的量：run 结束时若恒为单一值，说明上游链路断了。
# (字段名, 人类可读名, 允许恒定?) —— 允许恒定的量不参与死值判定。
LIVENESS_FIELDS = [
    ("speed",           "ego speed",        False),
    ("x",               "ego x",            False),
    ("y",               "ego y",            False),
    ("heading",         "ego heading",      False),
    ("steer_signed",    "steer command",    False),
    ("lane_count",      "lane count",       True),   # 场景固定即合法
]


def liveness_report(series: list[dict]) -> dict:
    """统计每个受监控量的取值分布，识别结构性死值。

    返回 {field: {"unique": n, "min": .., "max": .., "dead": bool}}。
    dead=True 表示整个 run 只有一个取值 —— 依赖它的判据无意义。
    """
    report: dict[str, dict] = {}
    for field, label, may_be_const in LIVENESS_FIELDS:
        vals = [m[field] for m in series
                if field in m and isinstance(m[field], (int, float))
                and math.isfinite(m[field])]
        if not vals:
            report[field] = {"label": label, "unique": 0, "min": None,
                             "max": None, "dead": True, "reason": "no data"}
            continue
        uniq = len(set(round(v, 6) for v in vals))
        dead = (uniq <= 1) and not may_be_const
        report[field] = {
            "label": label, "unique": uniq,
            "min": min(vals), "max": max(vals),
            "dead": dead,
            "reason": "constant" if dead else "",
        }
    return report


def require(failures: list[str], gate_name: str, conditions: dict) -> bool:
    """判据前置条件检查。

    conditions: {人类可读的前置描述: bool}。任一为 False → 该判据无法判定，
    记为 INCONCLUSIVE 并计入 failures。**无法判定不等于通过** —— 这是与
    旧 `if n < MIN: continue` 的关键区别，后者把"没测到"渲染成 PASS。

    返回 True 表示前置齐备、调用方可以继续做实际判定。
    """
    unmet = [desc for desc, ok in conditions.items() if not ok]
    if unmet:
        failures.append(
            f"INCONCLUSIVE [{gate_name}]: cannot evaluate — " + "; ".join(unmet)
        )
        return False
    return True


def scenario_actor_layer_counts(scenario: dict | None) -> dict[str, int]:
    """场景 JSON 声明的各感知层 actor 数量。

    这是"本应被感知到什么"的权威来源。识别率判据拿它做前置：场景里有
    行人却没有 vru 真值样本 = 感知链路漏了整个类别，而不是"该层没数据、
    跳过判定"。旧实现缺这个对照，所以 truth_count_vru=0 被当成正常。
    """
    counts: dict[str, int] = {"vehicle": 0, "vru": 0}
    if not isinstance(scenario, dict):
        return counts
    for actor in scenario.get("actors", []) or []:
        if not isinstance(actor, dict):
            continue
        layer = TRUTH_LAYER_FOR_TYPE.get(str(actor.get("type", "")).lower())
        if layer in counts:
            counts[layer] += 1
    return counts


def score(samples: list[dict], launcher_log: Path, criteria: dict | None = None, scenario_name: str | None = None, expected_edges: list[tuple[str, str, str]] | None = None, has_noa_route: bool = False, road: dict | None = None, traffic_lights: list | None = None, scenario: dict | None = None, expected_duration_s: float | None = None) -> tuple[list[str], list[str], dict]:
    failures: list[str] = []
    warnings: list[str] = []
    criteria = criteria or {}
    if not samples:
        return ["no topology samples collected"], warnings, {}

    # ── E: 从场景配置读取限速（NPC 速度 spike 门禁联动） ──
    # speed_limit 来自场景的 road_network.edges[0].speed_limit（m/s）
    speed_limit = 22.0  # 默认限速（直路场景 typical 值）
    if scenario:
        edges = scenario.get("road_network", {}).get("edges", [])
        if edges:
            sl = edges[0].get("speed_limit", 0.0)
            if sl and sl > 0:
                speed_limit = float(sl)

    last = samples[-1]
    series = [sample_metrics(s, road) for s in samples]
    speeds = [m["speed"] for m in series]
    xs = [m["x"] for m in series]
    ys = [m["y"] for m in series]
    lane_errors = [m["lane_error"] for m in series]
    road_margins = [m["road_edge_margin"] for m in series]
    steer_values = [m["steer"] for m in series]
    steer_signed = [m["steer_signed"] for m in series]
    headings = [m["heading"] for m in series]
    timestamps = [float(s.get("timestamp", 0.0) or 0.0) for s in samples]
    scenario_layer_counts = scenario_actor_layer_counts(scenario)

    # ── run 完整性门禁：截断的 run 不许冒充 PASS ──
    # 请求跑 N 秒但样本跨度 < 50% → INCONCLUSIVE → FAIL。触发场景：monitor
    # 写 /tmp/flow_topology.json 中途静默停止（back-to-back demo 瞬态），
    # 评估器只采到开头几秒。没有这个门禁，截断 run 会：
    #   1) 行为指标"正常" → 误报 PASS（60s 只测了 8s）；
    #   2) x_delta 骤降 → 误报对 baseline 的数值回归。
    # 两者都让"PASS 可信"破产 —— 判定不了就该 FAIL，不是 PASS。
    # span 复用上面算好的 timestamps（同一时间戳语义，不重复解析）。
    if expected_duration_s is not None and expected_duration_s > 0 and len(timestamps) >= 5:
        span = timestamps[-1] - timestamps[0]
        if span < expected_duration_s * 0.5:
            failures.append(
                f"run truncated: sample span {span:.1f}s < 50% of requested "
                f"{expected_duration_s:.0f}s — demo did not produce data for the "
                f"full run, result is INCONCLUSIVE (likely monitor JSON writer "
                f"stopped mid-run)"
            )

    # ── 门禁有效性：量的活性检查 ──────────────────────────────
    # 在任何判据之前跑。一个恒为初值的量意味着上游链路断了，
    # 而所有依赖它的判据都会"通过"——这正是本项目反复翻车的机制
    # (ego_v 恒 0 / obs 恒空 / kappa 恒 0)。死值直接 FAIL，
    # 并且要先报出来，否则后面几十条 PASS 会掩盖它。
    liveness = liveness_report(series)
    for field, info in liveness.items():
        if info["dead"]:
            detail = (info["reason"] if info["reason"] != "constant"
                      else f"constant at {info['min']}")
            failures.append(
                f"DEAD SIGNAL [{info['label']}]: {detail} across all "
                f"{len(series)} samples — every check reading this quantity "
                f"is vacuous"
            )

    topics = topic_map(last)
    pubs, subs = node_topic_roles(last)
    expected_edges = expected_edges if expected_edges is not None else load_pipeline_expected_edges()
    for pub_node, topic, sub_node in expected_edges:
        if (pub_node, topic) not in pubs or (sub_node, topic) not in subs:
            failures.append(f"missing topology edge {pub_node} --{topic}--> {sub_node}")

    for topic, min_freq in TOPIC_MIN_FREQ.items():
        actual = float(topics.get(topic, {}).get("freq", 0.0) or 0.0)
        if actual < min_freq:
            failures.append(f"topic {topic} freq too low: {actual:.1f} Hz < {min_freq:.1f} Hz")

    collision_pub = int(topics.get("sim/collision", {}).get("pub", 0) or 0)
    log_text = launcher_log.read_text(encoding="utf-8", errors="ignore") if launcher_log.exists() else ""
    # 提取碰撞对象 ID（LOG_ERROR 格式 "COLLISION ego↔entityN"），便于定位是哪个 NPC
    collision_entity_ids = re.findall(r"COLLISION ego.*?entity(\d+)", log_text)
    collision_log_count = len(collision_entity_ids)
    no_collision_required = bool(criteria.get("no_collision", True))
    if no_collision_required and (collision_pub > 0 or collision_log_count > 0):
        entity_ids_str = ", ".join(f"entity{eid}" for eid in collision_entity_ids[:5]) if collision_entity_ids else "n/a"
        failures.append(f"collision detected: topic_pub={collision_pub}, log_count={collision_log_count}"
                        f", entities=[{entity_ids_str}]")

    # P2-7: flowsim invariant 失败升级为 FAIL。
    # flowsim_node.cpp 在 cleanup 时打印 [INV] summary total=N marker。
    # 兼容旧格式 [INVARIANT_FAILED] total=N。
    invariant_match = re.search(r"\[INV\]\s*summary\s*total=(\d+)", log_text)
    if not invariant_match:
        invariant_match = re.search(r"\[INVARIANT_FAILED\]\s*total=(\d+)", log_text)
    if invariant_match and int(invariant_match.group(1)) > 0:
        failures.append(
            f"flowsim invariant failed: total={invariant_match.group(1)} "
            f"(spatial+motion+temporal checks, see stderr for details)"
        )

    max_lane_index = max(range(len(series)), key=lambda i: lane_errors[i])
    max_lane_error = lane_errors[max_lane_index]
    min_road_margin_index = min(range(len(series)), key=lambda i: road_margins[i])
    min_road_margin = road_margins[min_road_margin_index]
    # road departure 检测：区分"持续偏出"与"短暂过渡"。
    # ego 从多车道进入单车道 ramp 时，横向位置需要从 ±1.6m 收敛到 0m，
    # 必然有短暂帧 |y| 超出单车道半宽（1.75m - 1.0m body = 0.75m）。
    # 单帧极值检测会把这种过渡误报为 road departure。
    # 修复：只检测以下情况为 road departure：
    #   1. 严重偏出：margin < -1.5m（ego body 超出路面 1.5m，绝非过渡）
    #   2. 持续偏出：连续 ≥10 帧（0.5s @20Hz）margin < 0
    road_departure_consecutive = 0
    road_departure_max_consecutive = 0
    for m in road_margins:
        if m < 0.0:
            road_departure_consecutive += 1
            if road_departure_consecutive > road_departure_max_consecutive:
                road_departure_max_consecutive = road_departure_consecutive
        else:
            road_departure_consecutive = 0
    if min_road_margin < -1.5 or road_departure_max_consecutive >= 10:
        failures.append(f"road departure: ego body exceeded road edge by {-min_road_margin:.2f} m"
                        f" (consecutive={road_departure_max_consecutive} frames)")
    elif min_road_margin < 0.0:
        warnings.append(f"brief road edge excursion during lane-count transition: {-min_road_margin:.2f} m"
                        f" (consecutive={road_departure_max_consecutive} frames, < 10 threshold)")
    if max_lane_error > 2.0:
        warnings.append(f"large lane-center deviation during maneuver: {max_lane_error:.2f} m")

    progress = xs[-1] - xs[0] if len(xs) >= 2 else 0.0
    min_distance = float(criteria.get("min_distance_m", 0.0) or 0.0)
    required_distance = min_distance if min_distance > 0.0 else 10.0
    if progress < required_distance:
        failures.append(f"vehicle stuck or no progress: x delta {progress:.1f} m < {required_distance:.1f} m")

    avg_speed = statistics.fmean(speeds) if speeds else 0.0
    min_avg_speed = float(criteria.get("min_avg_speed_mps", 0.0) or 0.0)
    required_avg_speed = min_avg_speed if min_avg_speed > 0.0 else 1.0
    if avg_speed < required_avg_speed:
        failures.append(f"average speed too low: {avg_speed:.1f} m/s < {required_avg_speed:.1f} m/s")
    if max(speeds) > 25.0:
        failures.append(f"unrealistic speed spike: max speed {max(speeds):.1f} m/s")

    # ── 低速停滞检测（龟速） ──
    low_speed_thresh = 6.0  # m/s, 低于此判为龟速
    low_speed_samples = sum(1 for s in speeds if s < low_speed_thresh)
    low_speed_ratio = low_speed_samples / max(1, len(speeds))
    # 最长连续龟速区间（按实际帧间 dt 累加，对非单调/异常 dt 跳过）
    longest_stagnation = 0.0
    current_stagnation = 0.0
    prev_ts = None
    for i, s in enumerate(speeds):
        ts = timestamps[i] if i < len(timestamps) else 0.0
        if s < low_speed_thresh:
            if prev_ts is not None:
                dt = ts - prev_ts
                if 0.0 < dt <= 2.0:
                    current_stagnation += dt
            if current_stagnation > longest_stagnation:
                longest_stagnation = current_stagnation
        else:
            current_stagnation = 0.0
        prev_ts = ts
    stagnation_duration_s = longest_stagnation

    # ── 变道次数统计（基于 y 量化到车道 idx） ──
    # N 车道模型：用 lane_idx_from_y 把每帧 y_rel 量化到车道索引，
    # 相邻帧 idx 变化即记一次变道。lane_count=2 时与旧实现等价。
    # 注：metrics["lane_count"] 取自 road/geometry topic（flowsim 按 ego road_id
    # 实时发布），中途若切换路段导致 lane_count 变化，按每帧各自的 lane_count 量化。
    lane_width_default = 3.5
    lane_change_count = 0
    prev_lane = None
    for m in series:
        lc = int(m.get("lane_count", 2) or 2)
        yr = float(m.get("y_rel", 0.0) or 0.0)
        lane_idx = lane_idx_from_y(yr, lc, lane_width_default, 0.0)
        if prev_lane is not None and lane_idx != prev_lane:
            lane_change_count += 1
        prev_lane = lane_idx

    # Legacy fallback only: if scenario criteria doesn't specify min_avg_speed_mps,
    # use hardcoded stagnation thresholds to catch deadlocks that pass_criteria
    # can't express. When min_avg_speed_mps IS set, low-speed is governed by that
    # check above (more precise than generic stagnation thresholds).
    if not min_avg_speed and low_speed_ratio > 0.50 and stagnation_duration_s > 5.0:
        failures.append(
            f"low-speed stagnation: {low_speed_ratio*100:.0f}% samples below {low_speed_thresh} m/s, "
            f"longest run {stagnation_duration_s:.1f}s"
        )
    required_lane_changes = int(criteria.get("required_lane_changes", 0) or 0)
    if lane_change_count < required_lane_changes:
        failures.append(f"lane changes too few: {lane_change_count} < {required_lane_changes}")

    # ── NOA (导航领航辅助) 功能校验 ──
    # 场景定义了 route[] 时，模式层状态机预期能升级到 NOA 并按导航路线主动变道
    # (见 docs/tutorials/08_state_machine.md)。仅靠拓扑/频率检查发现不了"模式没升级"或
    # "route_lane 从未被消费"这类功能性回归，因此单独校验驾驶模式序列。
    driver_modes_seen = sorted({m["driver_mode"].split(":")[0] for m in series if m["driver_mode"]})
    reached_noa = "NOA" in driver_modes_seen
    route_lane_active = any(m["route_lane"] != 0 for m in series)
    if has_noa_route:
        if not reached_noa:
            failures.append(
                f"NOA scenario defines a navigation route but driving mode never reached NOA "
                f"(modes observed: {driver_modes_seen or ['(none)']})"
            )
        if not route_lane_active:
            warnings.append("NOA route defined but route_lane target was never set by planning")
        if lane_change_count < 1:
            warnings.append("NOA route defined but no lane change was observed during the run")

    # ── 红绿灯违章检测 ──
    # 当场景定义了 traffic_lights 且 pass_criteria.no_red_light_violation 为真时：
    #   FAIL: ego 在红灯期间越过停止线（闯红灯）
    #   WARN: ego 在绿灯期间不必要地长时间停留（误判/过度保守，planning 可能在
    #         绿灯时仍注入了虚拟停止墙）
    # 红绿灯状态来自 monitor 透传的 scene.entities（flowsim 真值发布，世界坐标）。
    # 若某帧缺少状态数据，沿用上一已知状态（灯相位切换周期远大于采样间隔）。
    scenario_lights = traffic_lights if traffic_lights else []
    has_red_light_check = bool(criteria.get("no_red_light_violation", False))
    red_light_violation = False
    green_phase_max_stop_s = 0.0
    has_signal_data = False

    # 有灯就扫。闯红灯判定仍受 no_red_light_violation 控制，但"绿灯下卡死"是
    # 通用死锁检测，不该被无关开关关掉 —— straight_road.json 没声明该 flag，
    # 整块被跳过，ego 绿灯下静止 30s 仍报 0.000，放行了 planning 的闭锁。
    if scenario_lights:
        for tl_def in scenario_lights:
            if not isinstance(tl_def, dict):
                continue
            stop_x = float(tl_def.get("x", 0.0) or 0.0)

            prev_ego_x = None
            prev_state = "unknown"
            green_stop_start_ts = None

            for i, m in enumerate(series):
                ego_x_i = m["x"]
                ts_i = timestamps[i] if i < len(timestamps) else 0.0

                # 从样本中查找对应红绿灯的当前状态（按 x 匹配同一盏灯）
                curr_state = None
                for s_tl in m.get("entities", []):
                    if isinstance(s_tl, dict) and s_tl.get("type") == "tl":
                        s_x = float(s_tl.get("x", stop_x) or stop_x)
                        if abs(s_x - stop_x) < 2.0:
                            curr_state = str(s_tl.get("state", "") or "")
                            has_signal_data = True
                            break

                # 缺失帧沿用上一已知状态
                if curr_state is None:
                    curr_state = prev_state

                # 闯红灯检测：ego 从停止线前越到停止线后，且当前或上一帧灯为红
                if prev_ego_x is not None:
                    crossed = prev_ego_x < stop_x and ego_x_i >= stop_x
                    if crossed and "red" in (prev_state, curr_state):
                        red_light_violation = True

                # 绿灯期间不必要停车检测：灯为绿、ego 在停止线前的合理接近范围内、车速极低。
                #
                # 近邻约束不可省：本循环对每盏灯各跑一遍，而场景有 10 盏
                # （x=200..9200）。对远处的灯 "ego_x_i < stop_x" 恒为真，ego 在
                # x=177 等红灯的静止会被记到 1000m 外那盏绿灯头上，误报 5-6.8s。
                near_stop_line = -GREEN_STOP_NEAR_M < (stop_x - ego_x_i) <= GREEN_STOP_NEAR_M
                if curr_state == "green" and near_stop_line and m["speed"] < 0.5:
                    if green_stop_start_ts is None:
                        green_stop_start_ts = ts_i
                else:
                    if green_stop_start_ts is not None:
                        stop_dur = ts_i - green_stop_start_ts
                        if stop_dur > green_phase_max_stop_s:
                            green_phase_max_stop_s = stop_dur
                        green_stop_start_ts = None

                prev_ego_x = ego_x_i
                prev_state = curr_state

            # 样本序列结束时仍在绿灯期停车
            if green_stop_start_ts is not None and len(timestamps) > 0:
                stop_dur = timestamps[-1] - green_stop_start_ts
                if stop_dur > green_phase_max_stop_s:
                    green_phase_max_stop_s = stop_dur

        if not has_signal_data:
            # 措辞跟着 has_red_light_check 走：这段现在对任何有灯的场景都跑，
            # 对没声明 no_red_light_violation 的场景说"check enabled"是错的。
            what = "red-light check" if has_red_light_check else "green-stall check"
            warnings.append(
                f"{what} has no traffic_light state data in samples "
                "(scene.entities may not include tl type)"
            )
        elif red_light_violation and has_red_light_check:
            failures.append("red light violation: ego crossed stop line during red phase")
        if green_phase_max_stop_s > 5.0:
            # FAIL 而非 WARN：绿灯下长时间不动是功能性死锁。只报 WARN 时，
            # 45s 里静止 30s 的 run 照样 PASS，只靠 avg_speed 偶然兜底 ——
            # 而 avg_speed 在长 run 里会被摊薄。
            failures.append(
                f"stuck during green: ego stopped {green_phase_max_stop_s:.1f}s "
                f"while light was green (>5s — planning/control deadlock)"
            )

    # P2-7: steer 饱和阈值从 0.219 降到 0.17。
    # 旧值 0.219 按 max_steer=0.22 定，但实际生效限幅是 low_speed_steer=0.18
    # （pipeline.json:220 → safety_control_node.cpp:383，速度<3.0 分支）。
    # 差 0.039 完美漏检 0.18 饱和——98% 帧死贴 0.18 时旧门禁看不到。
    steer_saturation_ratio = sum(1 for s in steer_values if s > 0.17) / max(1, len(steer_values))
    if steer_saturation_ratio > 0.45:
        warnings.append(f"steer saturated often: {steer_saturation_ratio * 100:.0f}% samples")
    elif steer_saturation_ratio > 0.10:
        # P2-7: 10% 以上饱和即告警，>45% 升级为 warning（上方分支）。
        # 验收标准要求饱和帧占比 < 10%（P0-1 修复前 98%）。
        failures.append(f"steer saturation too high: {steer_saturation_ratio * 100:.0f}% samples > 10% threshold")

    # P2-7: heading 健全性断言 —— garbage-in 防御。
    # flowsim_node.cpp:1196 在 kinematic 模式下把 ego.heading 钉死为 wp.h（直道=0），
    # 旧门禁读 ego.heading 算 yaw_rate_rms / heading_flip_rate，恒为 0 → 永不触发。
    # 这类"字段恒定"应视为门禁自身失效。判据：heading 方差≈0 且 steer 非 0
    # → 必然 garbage-in（车在动方向却不变，物理不可能）。
    if len(headings) >= 10:
        heading_mean = statistics.fmean(headings)
        heading_var = statistics.fmean([(h - heading_mean) ** 2 for h in headings])
        steer_nonzero = sum(1 for s in steer_values if abs(s) > 0.01)
        if heading_var < 1e-12 and steer_nonzero > len(steer_values) * 0.1:
            failures.append(
                f"heading invariant garbage-in: heading variance={heading_var:.2e} ≈ 0 "
                f"while {steer_nonzero}/{len(steer_values)} frames have non-zero steer "
                f"(field pinned constant, gate invalid)"
            )

    # P2-7: 横向偏离真实幅度检查 —— 旧 max_lane_error 取最近车道中心距离，
    # 4m 蛇形跨车道时车总"接近某条车道中心"，该值反而不大，度量选择错误。
    # 补充直接检查 y 坐标的峰峰值，>4.5m 即横向大幅扫动跨越多条车道；
    # >4.0m 为 WARN（多车道道路变道 y 范围天然可达 3.5m，含 overshoot 约 4.0m）。
    if len(ys) >= 10:
        y_range = max(ys) - min(ys)
        if y_range > 4.5:
            failures.append(
                f"lateral excursion too large: y range={y_range:.2f} m > 4.5 m "
                f"(snaking across lanes, y_min={min(ys):.2f} y_max={max(ys):.2f})"
            )
        elif y_range > 4.0:
            warnings.append(f"lateral wobble: y range={y_range:.2f} m > 4.0 m")

    yaw_rates: list[float] = []
    steer_rates: list[float] = []
    npc_speed_spikes: list[float] = []
    npc_lateral_spikes: list[float] = []
    # P2-7: 记录超大位移（千米级瞬移），不再静音。
    # 旧逻辑 `if disp > 30.0: continue` 把最严重的瞬移直接跳过，
    # 1000m 级瞬移连样本都不进 → P0-2 漏检。
    npc_teleport_displacements: list[tuple[int, float]] = []
    # P3: NPC 跟踪改用 ground-truth entities（scene_frame，绝对坐标 + tp_cycle），
    # 不再用 scene.obstacles（vehicle_state，由 monitor 从多个独立缓存的 buffer
    # 非原子拼装，ego/obstacles 可能来自不同 sim cycle → 同一 id 出现 +200m 伪位移）。
    # entities 是 flowsim scene_frame 单帧快照，自洽且携带 tp_cycle，可直接判定
    # 合法瞬移（recycle / 碰撞分离）。
    for i in range(1, len(series)):
        dt = timestamps[i] - timestamps[i - 1]
        if dt <= 1e-3 or dt > 2.0:
            continue
        yaw_rates.append(abs(angle_diff(headings[i], headings[i - 1])) / dt)
        steer_rates.append(abs(steer_signed[i] - steer_signed[i - 1]) / dt)

        prev_ents = {
            e.get("id"): e for e in series[i - 1].get("entities", [])
            if e.get("type") in TRUTH_TYPE_VEHICLE and "x" in e
        }
        for e in series[i].get("entities", []):
            if e.get("type") not in TRUTH_TYPE_VEHICLE:
                continue
            eid = e.get("id")
            pe = prev_ents.get(eid)
            if not pe or "x" not in pe:
                continue
            dx = float(e["x"]) - float(pe["x"])
            dy = float(e["y"]) - float(pe["y"])
            disp = math.hypot(dx, dy)
            if disp > 30.0:
                # tp_cycle 增大 ⇒ 本区间内发生过 recycle/碰撞分离 ⇒ 合法瞬移
                curr_tp = int(e.get("tp_cycle", 0) or 0)
                prev_tp = int(pe.get("tp_cycle", 0) or 0)
                is_legitimate = curr_tp > prev_tp
                # DEBUG: 临时打印每个 >30m 位移的 tp_cycle 详情
                import os as _os
                if _os.environ.get("EVAL_DEBUG_TELEPORT"):
                    print(f"[DBG] teleport i={i} id={eid} disp={disp:.1f} "
                          f"curr_tp={curr_tp} prev_tp={prev_tp} legit={is_legitimate} "
                          f"x_prev={float(pe['x']):.1f} x_curr={float(e['x']):.1f} "
                          f"t_prev={timestamps[i-1]:.2f} t_curr={timestamps[i]:.2f}",
                          file=sys.stderr)
                    # P3 诊断：第一次出现 >200m 位移时，dump 前后两帧的全部 NPC 实体，
                    # 用于判断是"整帧坐标平移/ID 错位"还是"单车真实瞬移"。
                    if disp > 200.0 and not getattr(_dbg_dumped, "done", False):
                        _dbg_dumped.done = True
                        _dbg_dump_sample("frame_dump_prev.json", series[i - 1])
                        _dbg_dump_sample("frame_dump_curr.json", series[i])
                # 30-200m 且非合法: 可能是 respawn 残差，记录但不 FAIL。
                # >200m 且非合法: 几乎必然是 P0-2 类 id 撞车污染，直接 FAIL。
                npc_teleport_displacements.append((eid if isinstance(eid, int) else -1, disp))
                if disp > 200.0 and not is_legitimate:
                    failures.append(
                        f"npc teleport: id={eid} displaced {disp:.1f} m "
                        f"in one frame (>200m threshold, likely id-collision pollution)"
                    )
                continue
            speed = disp / dt
            npc_speed_spikes.append(speed)
            npc_lateral_spikes.append(abs(dy) / dt)

    yaw_rate_rms = math.sqrt(statistics.fmean([r * r for r in yaw_rates])) if yaw_rates else 0.0
    max_yaw_rate = max(yaw_rates) if yaw_rates else 0.0
    steer_rate_rms = math.sqrt(statistics.fmean([r * r for r in steer_rates])) if steer_rates else 0.0
    max_steer_rate = max(steer_rates) if steer_rates else 0.0
    steer_flip_rate = sign_flips(steer_signed, 0.03) / max(1e-6, (timestamps[-1] - timestamps[0]))
    heading_flip_rate = sign_flips([angle_diff(headings[i], headings[i - 1]) for i in range(1, len(headings))], 0.003) / max(1e-6, (timestamps[-1] - timestamps[0]))
    max_npc_speed = max(npc_speed_spikes) if npc_speed_spikes else 0.0
    max_npc_lateral_speed = max(npc_lateral_spikes) if npc_lateral_spikes else 0.0

    if (yaw_rate_rms > YAW_RMS_FAIL or max_yaw_rate > YAW_MAX_FAIL or
            (heading_flip_rate > HEADING_FLIP_FAIL and yaw_rate_rms > 0.2)):
        failures.append(
            f"ego yaw limit cycle: yaw_rms={yaw_rate_rms:.2f} rad/s, max={max_yaw_rate:.2f}, "
            f"flips={heading_flip_rate:.2f}/s (1-2Hz lateral limit cycle, FAIL > "
            f"{YAW_RMS_FAIL}/{YAW_MAX_FAIL})"
        )
    elif (yaw_rate_rms > YAW_RMS_WARN or max_yaw_rate > YAW_MAX_WARN or
            (heading_flip_rate > HEADING_FLIP_WARN and yaw_rate_rms > HEADING_FLIP_WARN_MIN_YAW)):
        warnings.append(
            f"ego yaw wobble: yaw_rms={yaw_rate_rms:.2f} rad/s, max={max_yaw_rate:.2f}, flips={heading_flip_rate:.2f}/s"
        )
    if (steer_rate_rms > STEER_RATE_RMS_FAIL or max_steer_rate > STEER_MAX_RATE_FAIL or
            steer_flip_rate > STEER_FLIP_FAIL):
        failures.append(
            f"steer bang-bang oscillation: steer_rate_rms={steer_rate_rms:.2f}/s, "
            f"max={max_steer_rate:.2f}/s, flips={steer_flip_rate:.2f}/s "
            f"(control dithering every frame, FAIL > {STEER_FLIP_FAIL}/s)"
        )
    elif (steer_rate_rms > STEER_RATE_RMS_WARN or max_steer_rate > STEER_MAX_RATE_WARN or
            steer_flip_rate > STEER_FLIP_WARN):
        warnings.append(
            f"steer oscillation: steer_rate_rms={steer_rate_rms:.2f}/s, max={max_steer_rate:.2f}/s, flips={steer_flip_rate:.2f}/s"
        )
    if max_npc_lateral_speed > 12.0:
        warnings.append(
            f"npc motion spike: max_speed={max_npc_speed:.1f} m/s, max_lateral={max_npc_lateral_speed:.1f} m/s"
        )
    elif max_npc_speed > speed_limit * 1.5:
        warnings.append(f"npc respawn jump: max_speed={max_npc_speed:.1f} m/s ({speed_limit*1.5:.0f}=1.5×speed_limit={speed_limit:.0f} m/s), max_lateral={max_npc_lateral_speed:.1f} m/s")

    drops = sum(int(t.get("drop", 0) or 0) for t in topics.values())
    total_pub = sum(int(t.get("pub", 0) or 0) for t in topics.values())
    drop_rate = drops / total_pub if total_pub > 0 else 0.0
    if drops > 50 or drop_rate > 0.01:
        failures.append(f"message drops detected: {drops} (rate {drop_rate*100:.2f}%)")
    elif drops > 0:
        warnings.append(f"message drops detected: {drops} (rate {drop_rate*100:.2f}%)")

    # ── min_forward_gap 近距/碰撞 FAIL ──
    # 前方有车但 gap <= 0 → 追尾/碰撞，直接 FAIL（无论是否触发 COLLISION 正则）
    #
    # 判据必须与 ACC 的期望间距挂钩，而不是一个固定 5m。理由：ego 以 12 m/s
    # 跟车时 desired_gap = 5 + 1.5*12 = 23m，此时 gap=6m 是"差点撞上"而非
    # "安全"。旧实现把 4.66m 记为 WARN（可忽略），于是"追尾是运气问题"这件事
    # 从未阻断过合并 —— 而 min_forward_gap 在 -0.69m 和 4.66m 之间随机漂移，
    # 两次 run 的差别只是运气。用 desired_gap 的比例做阈值，判据随车速伸缩。
    valid_gaps = [m["min_forward_gap"] for m in series if not math.isinf(m["min_forward_gap"])]
    if valid_gaps:
        min_gap_all = min(valid_gaps)
        # 取该帧车速估期望间距；用整段的中位速度避免个别低速帧放宽判据
        _speeds_sorted = sorted(speeds)
        v_med = _speeds_sorted[len(_speeds_sorted) // 2] if _speeds_sorted else 0.0
        desired_gap = ACC_STANDOFF_M + ACC_TIME_HEADWAY_S * v_med
        gap_fail_thresh = desired_gap * ACC_GAP_FAIL_RATIO
        if min_gap_all <= 0.0:
            failures.append(f"min_forward_gap <= 0 (min={min_gap_all:.2f}m): rear-end collision risk")
        elif min_gap_all < gap_fail_thresh:
            failures.append(
                f"min_forward_gap {min_gap_all:.2f}m < {gap_fail_thresh:.2f}m "
                f"({ACC_GAP_FAIL_RATIO:.0%} of desired {desired_gap:.1f}m at "
                f"v_med={v_med:.1f} m/s): ACC is not holding headway — "
                f"collision avoided by margin, not by control"
            )
        elif min_gap_all < desired_gap:
            warnings.append(
                f"min_forward_gap {min_gap_all:.2f}m below desired "
                f"{desired_gap:.1f}m (v_med={v_med:.1f} m/s)"
            )

    # ── 感知降频检测 ──
    # 场景有 entities 但 obstacles 长期为空 → 感知链路降频/掉线
    frames_with_obs = sum(1 for m in series if m["obs_world"])
    obs_ratio = frames_with_obs / len(series) if series else 0.0
    # 只有在 samples 中出现过至少一次障碍物时才做检测
    # （空场景无 NPC 时不应触发感知告警）
    has_seen_obs = frames_with_obs > 0
    if has_seen_obs and obs_ratio < 0.3:
        failures.append(f"perception dropout: obstacles present in only {obs_ratio*100:.1f}% frames")
    elif has_seen_obs and obs_ratio < 0.7:
        warnings.append(f"perception degradation: obstacles in {obs_ratio*100:.1f}% frames")

    # ── 上游 dead 专项断言 ──
    # DATA_TIMEOUT / EKF_NOT_CONVERGED 出现频率过高 → 上游定位/感知已死
    data_timeout_frames = sum(1 for m in series if m["driver_mode"] == "DATA_TIMEOUT")
    ekf_timeout_frames = sum(1 for m in series if m["driver_mode"] == "EKF_NOT_CONVERGED")
    total_frames = len(series)
    if total_frames > 0:
        dt_ratio = data_timeout_frames / total_frames
        ekf_ratio = ekf_timeout_frames / total_frames
        if dt_ratio > 0.1:
            failures.append(f"upstream DATA_TIMEOUT: {dt_ratio*100:.1f}% frames ({data_timeout_frames}/{total_frames})")
        elif dt_ratio > 0.02:
            warnings.append(f"upstream DATA_TIMEOUT: {dt_ratio*100:.1f}% frames")
        if ekf_ratio > 0.1:
            failures.append(f"EKF not converged: {ekf_ratio*100:.1f}% frames ({ekf_timeout_frames}/{total_frames})")
        elif ekf_ratio > 0.02:
            warnings.append(f"EKF not converged: {ekf_ratio*100:.1f}% frames")

    # ── inference/trajectory frequency check (only when topic is present in runtime data) ──
    inference_freq = float(topics.get("inference/trajectory", {}).get("freq", 0.0) or 0.0)
    inference_topic_active = "inference/trajectory" in topics
    if inference_topic_active and inference_freq < INFERENCE_TOPIC_MIN_FREQ:
        failures.append(
            f"topic inference/trajectory freq too low: {inference_freq:.1f} Hz "
            f"< {INFERENCE_TOPIC_MIN_FREQ:.1f} Hz"
        )

    # ── shadow delta check (read latest sidecar output for current-frame validation) ──
    shadow_delta = _load_shadow_delta()
    shadow_delta_abs = abs(shadow_delta) if shadow_delta is not None else None
    if shadow_delta_abs is not None:
        if shadow_delta_abs > SHADOW_DELTA_FAIL:
            failures.append(
                f"shadow_delta too large: {shadow_delta:+.2f} m/s (threshold {SHADOW_DELTA_FAIL:.1f} m/s)"
            )
        elif shadow_delta_abs > SHADOW_DELTA_WARN:
            warnings.append(
                f"shadow_delta elevated: {shadow_delta:+.2f} m/s (warn threshold {SHADOW_DELTA_WARN:.1f} m/s)"
            )

    # ── Task 5：分层识别率 / 预警提前量 ──
    # truth（flowsim scene.entities）vs perceived（scene.obstacles 转世界坐标）
    # 的匹配率，按 vehicle / vru 分层；预警提前量 = TTC 跌破临界时刻 - 首次检测时刻。
    perception = _compute_perception_metrics(series, timestamps)
    # 分层识别率 FAIL/WARN。
    #
    # 旧实现 `if n < REC_MIN_SAMPLES: continue` 是本项目"虚假满分"的来源：
    # 场景里有行人，但行人从未进入真值统计（truth_count_vru=0），于是
    # recognition_rate_vru 被算成 1.000 且判据被 continue 跳过 —— 报告打印
    # "感知 100%" 而实际那一层根本没测。现在改为：该层在场景中存在
    # (scenario_expects) 但真值样本不足 → INCONCLUSIVE（计 FAIL）。
    REC_MIN_SAMPLES = 5
    for layer_name, rate_key, count_key in [
        ("vehicle", "recognition_rate_vehicle", "truth_count_vehicle"),
        ("vru", "recognition_rate_vru", "truth_count_vru"),
    ]:
        rate = perception[rate_key]
        n = perception[count_key]
        expected_in_scene = scenario_layer_counts.get(layer_name, 0) > 0
        if not require(failures, f"recognition_rate_{layer_name}", {
            f"scenario declares {scenario_layer_counts.get(layer_name, 0)} "
            f"{layer_name} actor(s) but only {n} truth samples reached the "
            f"evaluator (>= {REC_MIN_SAMPLES} required) — "
            f"the {layer_name} sensing path is not being measured":
                (not expected_in_scene) or n >= REC_MIN_SAMPLES,
        }):
            continue
        if n < REC_MIN_SAMPLES:
            continue  # 场景本身没有该类 actor，跳过属正常
        if rate < PERCEPTION_RATE_FAIL:
            failures.append(
                f"{layer_name} recognition rate too low: {rate*100:.1f}% "
                f"({n} truth samples, FAIL < {PERCEPTION_RATE_FAIL*100:.0f}%)"
            )
        elif rate < PERCEPTION_RATE_WARN:
            warnings.append(
                f"{layer_name} recognition rate degraded: {rate*100:.1f}% "
                f"({n} truth samples, WARN < {PERCEPTION_RATE_WARN*100:.0f}%)"
            )
    # 预警提前量 FAIL/WARN（仅当发生过临界事件时才判定）
    if perception["critical_event_count"] > 0:
        min_lead = perception["warning_lead_min_s"]
        if min_lead < WARNING_LEAD_FAIL_S:
            failures.append(
                f"warning lead time too short: min={min_lead:.2f}s "
                f"({perception['critical_event_count']} critical events, "
                f"FAIL < {WARNING_LEAD_FAIL_S:.1f}s)"
            )
        elif min_lead < WARNING_LEAD_WARN_S:
            warnings.append(
                f"warning lead time short: min={min_lead:.2f}s "
                f"({perception['critical_event_count']} critical events, "
                f"WARN < {WARNING_LEAD_WARN_S:.1f}s)"
            )

    summary = {
        "scenario": scenario_name or "(unknown)",
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
        "yaw_rate_rms_radps": yaw_rate_rms,
        "max_yaw_rate_radps": max_yaw_rate,
        "heading_flip_rate_hz": heading_flip_rate,
        "low_speed_ratio": low_speed_ratio,
        "stagnation_duration_s": stagnation_duration_s,
        "lane_change_count": lane_change_count,
        "has_noa_route": has_noa_route,
        "driver_modes_seen": driver_modes_seen,
        "reached_noa": reached_noa,
        "route_lane_active": route_lane_active,
        "steer_rate_rms_per_s": steer_rate_rms,
        "max_steer_rate_per_s": max_steer_rate,
        "steer_flip_rate_hz": steer_flip_rate,
        "max_npc_speed_mps": max_npc_speed,
        "max_npc_lateral_speed_mps": max_npc_lateral_speed,
        "collision_topic_pub": collision_pub,
        "topic_freq_hz": {topic: float(topics.get(topic, {}).get("freq", 0.0) or 0.0) for topic in TOPIC_MIN_FREQ},
        "inference_topic_active": inference_topic_active,
        "inference_freq_hz": inference_freq,
        "shadow_delta_latest": shadow_delta,
        "has_traffic_lights": bool(scenario_lights),
        "red_light_violation": red_light_violation,
        "green_phase_max_stop_s": green_phase_max_stop_s,
        # Task 5：分层识别率 + 预警提前量
        "recognition_rate_vehicle": round(perception["recognition_rate_vehicle"], 3),
        "recognition_rate_vru": round(perception["recognition_rate_vru"], 3),
        "recognition_rate_overall": round(perception["recognition_rate_overall"], 3),
        "recognition_rate_by_type": perception["recognition_rate_by_type"],
        "truth_count_vehicle": perception["truth_count_vehicle"],
        "truth_count_vru": perception["truth_count_vru"],
        "truth_count_overall": perception["truth_count_overall"],
        "warning_lead_avg_s": round(perception["warning_lead_avg_s"], 3),
        "warning_lead_min_s": round(perception["warning_lead_min_s"], 3),
        "critical_event_count": perception["critical_event_count"],
        "min_ttc_s": perception["min_ttc_s"],
        "perceived_track_count": perception["perceived_track_count"],
        "liveness": {k: {"unique": v["unique"], "dead": v["dead"]}
                     for k, v in liveness.items()},
        "scenario_actor_counts": scenario_layer_counts,
        # behavior 指标（从最后一个 sample 的 metrics.behavior 提取，
        # 仅在 behavior/state 已发布时可用）
        "behavior_state": last.get("metrics", {}).get("behavior", {}).get("state", ""),
        "behavior_obs_count": int(last.get("metrics", {}).get("behavior", {}).get("obs_count", 0) or 0),
        "best_gap_m": float(last.get("metrics", {}).get("behavior", {}).get("best_gap", -1.0) or -1.0),
        "lead_speed_mps": float(last.get("metrics", {}).get("behavior", {}).get("lead_speed", 0.0) or 0.0),
        "desired_gap_m": float(last.get("metrics", {}).get("behavior", {}).get("desired_gap", 0.0) or 0.0),
        "committed_lane": int(last.get("metrics", {}).get("behavior", {}).get("committed_lane", -1) or -1),
        "target_lane": int(last.get("metrics", {}).get("behavior", {}).get("target_lane", -1) or -1),
        "blocked": bool(last.get("metrics", {}).get("behavior", {}).get("blocked", False)),
        "worthwhile": bool(last.get("metrics", {}).get("behavior", {}).get("worthwhile", False)),
    }

    # ── 门禁有效性：指标交叉一致性 ────────────────────────────
    # 发生了碰撞，却报出 51s 的预警提前量 —— 这两个数不可能同时成立，
    # 说明 warning_lead 算的是另一个目标（远处某辆车的首次检测），
    # 与真正撞上的那个无关。指标与事实错配时，漂亮数字比没有数字更糟：
    # 它让"感知 100% 却撞了"看起来像个谜，而不是一个 bug。
    if collision_pub > 0 or collision_log_count > 0:
        lead = perception["warning_lead_min_s"]
        if perception["critical_event_count"] <= 0:
            failures.append(
                "METRIC MISMATCH: collision occurred but critical_event_count=0 "
                "— the TTC/critical-event detector never saw the object that "
                "was actually hit"
            )
        elif math.isfinite(lead) and lead > 5.0:
            failures.append(
                f"METRIC MISMATCH: collision occurred yet warning_lead_min="
                f"{lead:.1f}s — the reported lead time is computed against a "
                f"different target than the collision partner"
            )

    # ── max_duration_s 超时检查 ──
    # 场景声明了 max_duration_s (>0) 时，实际运行时长不能超过它。
    # 这捕获"demo 卡住但 ego 仍在微小前进、碰撞数为 0"的退化场景——
    # 之前评估器从不检查此字段，所有场景的"超时即 FAIL"语义在 CI 中失效。
    max_duration = float(criteria.get("max_duration_s", 0.0) or 0.0)
    if max_duration > 0.0 and summary["duration_s"] > max_duration:
        failures.append(
            f"exceeded max duration: {summary['duration_s']:.1f}s > {max_duration:.1f}s"
        )

    return failures, warnings, summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate FlowEngine demo behavior.")
    parser.add_argument("--duration", type=int, default=0, help="demo run duration in seconds (0=auto-detect from scenario)")
    parser.add_argument("--interval", type=float, default=0.25, help="JSON sample interval in seconds")
    parser.add_argument("--json-file", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--no-run", action="store_true", help="evaluate current JSON/logs without starting demo.sh")
    parser.add_argument("--scenario", type=str, default=None,
                        help="scenario JSON path; temporarily overrides flowsim.scenario_file for this run")
    parser.add_argument("--json-out", type=Path, default=None,
                        help="write the machine-readable evaluation result to this JSON path")
    args = parser.parse_args()

    # Auto-detect duration from scenario if not explicitly given
    duration = args.duration
    if duration <= 0 and not args.no_run:
        scenario_cfg = load_scenario_for_duration(args.scenario)
        if scenario_cfg and scenario_cfg.get("duration_s", 0) > 0:
            duration = int(scenario_cfg["duration_s"])
        if duration <= 0:
            duration = 60  # fallback

    if args.no_run:
        data = load_json(args.json_file)
        if not data:
            samples = []
        else:
            # 如果能从 ring buffer（monitor_node.c samples 数组）读到多帧，
            # 用 ring buffer 重建时序 —— 否则只有 1 帧快照，所有时序检查都无效。
            ring = data.get("samples", [])
            if isinstance(ring, list) and len(ring) >= 3:
                samples = []
                base_scene = data.get("metrics", {}).get("scene", {})
                base_lane = base_scene.get("lane", {})
                base_ego = base_scene.get("ego", {})
                base_metrics = data.get("metrics", {})
                base_vehicle = base_metrics.get("vehicle", {})
                base_driver = base_metrics.get("driver_mode", "NA:READY")
                base_route = base_metrics.get("route_lane", 0)
                base_behavior = base_metrics.get("behavior", {})
                base_obstacles = base_scene.get("obstacles", [])
                base_entities = base_scene.get("entities", [])

                for r in ring:
                    rx = float(r.get("x", base_ego.get("x", 0)))
                    ry = float(r.get("y", base_ego.get("y", 0)))
                    rh = float(r.get("heading", base_ego.get("heading", 0)))
                    rs = float(r.get("speed", base_ego.get("speed", 0)))
                    rst = float(r.get("steer", base_ego.get("steer", 0)))

                    ego = dict(base_ego)
                    ego["x"] = rx
                    ego["y"] = ry
                    ego["heading"] = rh
                    ego["speed"] = rs
                    ego["steer"] = rst

                    samples.append({
                        "timestamp": float(r.get("t", data.get("timestamp", 0))),
                        "t_demo": float(r.get("t", 0)) / 1_000_000.0,  # C: 归一化到 demo 启动后秒
                        "metrics": {
                            "scene": {
                                "ego": ego,
                                "lane": base_lane,
                                "obstacles": base_obstacles,
                                "entities": base_entities,
                            },
                            "vehicle": base_vehicle,
                            "driver_mode": base_driver,
                            "route_lane": base_route,
                            "behavior": base_behavior,
                        },
                    })
            else:
                samples = [data]
        returncode = 0
        criteria, scenario_name, has_noa_route, road, traffic_lights = load_scenario_criteria_from_pipeline()
    else:
        # 默认场景：从 pipeline.json 的 flowsim.scenario_file 读取（即 city_to_highway_full）。
        # 旧实现不传 --scenario 给 demo.sh，demo.sh 用 DEFAULT_SCENARIO=infinite_straight
        # 覆盖 pipeline.json，导致 planning 运行时加载无 route 的场景，NOA 永远不升级。
        # 这里显式把 pipeline.json 里的 scenario_file 传给 demo.sh，确保 demo.sh 用
        # 该场景而非 DEFAULT_SCENARIO。args.scenario 优先级最高（用户显式指定）。
        effective_scenario = args.scenario
        if not effective_scenario:
            # 从 pipeline.json 读 flowsim.scenario_file 作为默认场景传给 demo.sh，
            # 避免 demo.sh 用 DEFAULT_SCENARIO（infinite_straight，无 route）覆盖。
            effective_scenario = _pipeline_flowsim_scenario_file()
        with pipeline_scenario_override(effective_scenario):
            samples, returncode = collect_samples(duration, args.json_file, args.interval,
                                                  scenario=effective_scenario)
            # Read pass_criteria/route while the override is still active, otherwise
            # the context manager's restore-on-exit would make this reflect the
            # pre-override (default) scenario instead of the one just run.
            criteria, scenario_name, has_noa_route, road, traffic_lights = load_scenario_criteria_from_pipeline()
        if returncode != 0:
            print(f"demo.sh exited with code {returncode}")

    # 场景 actor 清单：识别率判据的前置对照（"本应感知到什么"）。
    # 与 criteria/road 分开加载，避免改动 load_scenario_criteria_from_pipeline
    # 的返回元组形状（三处调用方）。
    _scn_file = args.scenario or _pipeline_flowsim_scenario_file()
    _scn_dict = None
    if _scn_file:
        _scn_path = Path(_scn_file)
        if not _scn_path.is_absolute():
            _scn_path = ROOT / _scn_path
        _scn_dict = load_json(_scn_path)

    failures, warnings, summary = score(samples, LAUNCHER_STDERR, criteria, scenario_name, has_noa_route=has_noa_route, road=road, traffic_lights=traffic_lights, scenario=_scn_dict, expected_duration_s=duration if not args.no_run else None)

    print("\n=== FlowEngine Demo Evaluation ===")
    for key, value in summary.items():
        if key == "topic_freq_hz":
            print("topic_freq_hz:")
            for topic, freq in value.items():
                print(f"  {topic}: {freq:.1f}")
        elif key == "recognition_rate_by_type":
            print("recognition_rate_by_type:")
            for t, r in value.items():
                print(f"  {t}: {r*100:.1f}%")
        elif isinstance(value, float):
            print(f"{key}: {value:.3f}")
        else:
            print(f"{key}: {value}")

    if warnings:
        print("\nWARN:")
        for warning in warnings:
            print(f"  - {warning}")

    result = "FAIL" if failures else "PASS"
    if args.json_out:
        # ── B: NPC 轨迹补全（每帧 NPC 列表，复盘追尾无需推演） ──
        # 从 samples 中提取每帧的 entities 真值（flowsim 发布，含所有活跃 NPC）
        npc_trajectories: dict[int, list[dict]] = {}
        for s in samples:
            ts = float(s.get("timestamp", 0.0) or 0.0)
            for ent in s.get("metrics", {}).get("scene", {}).get("entities", []):
                etype = str(ent.get("type", "")).lower()
                if etype in ("car", "suv", "truck", "pedestrian", "bicycle"):
                    eid = int(ent.get("id", -1))
                    if eid < 0:
                        continue
                    if eid not in npc_trajectories:
                        npc_trajectories[eid] = []
                    npc_trajectories[eid].append({
                        "t": ts,
                        "x": float(ent.get("x", 0.0) or 0.0),
                        "y": float(ent.get("y", 0.0) or 0.0),
                        "speed": float(ent.get("speed", 0.0) or 0.0),
                    })

        # ── C: 时间轴对齐 ──
        # 首帧采样时刻 t0，所有样本输出 t_demo = t - t0（归一化到 demo 启动后秒）
        if samples:
            t0 = float(samples[0].get("timestamp", 0.0) or 0.0)
            for s in samples:
                s["t_demo"] = (float(s.get("timestamp", 0.0) or 0.0) - t0) / 1_000_000.0 \
                    if t0 > 0 else 0.0

        payload = {
            "scenario": summary.get("scenario"),
            "result": result,
            "failures": failures,
            "warnings": warnings,
            "summary": summary,
            "samples": samples,  # 全量 ego 轨迹采样(t/x/y/heading/speed/steer)，供离线故事/事故分析
            "npc_trajectories": {str(eid): pts for eid, pts in npc_trajectories.items()},  # B: NPC 轨迹
        }
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                                 encoding="utf-8")
        print(f"\nwrote evaluation result to {args.json_out}")

    if failures:
        print("\nFAIL:")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    print("\nPASS: demo behavior is within the current regression envelope")
    return 0


if __name__ == "__main__":
    sys.exit(main())