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
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_JSON = Path("/tmp/flow_topology.json")
LAUNCHER_STDERR = Path("/tmp/flow_launcher_stderr.txt")
PIPELINE_JSON = ROOT / "config" / "pipeline.json"

TOPIC_MIN_FREQ = {
    "vehicle/state": 15.0,
    "sensor/lidar": 15.0,
    "sensor/gps": 7.0,
    "fusion/localization": 19.0,
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
# 真值实体 type → 分层类别映射（与 flowsim entity.h::EntityType 对齐）。
TRUTH_TYPE_VEHICLE = {"car", "truck", "suv"}
TRUTH_TYPE_VRU = {"pedestrian"}  # Vulnerable Road User
TRUTH_TYPE_INFRA = {"ego", "tl", "etc_gate"}  # 基础设施，不计入识别率
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
    and actively change lanes per the navigation route (see skills/08_state_machine.md).

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
    """Temporarily point config/pipeline.json's flowsim (and planning, if present)
    node(s) at ``scenario_file``.

    Node ``params`` is a JSON-encoded string; we patch the embedded
    ``scenario_file`` key, yield, then restore the original file byte-for-byte.
    Passing ``None`` is a no-op so callers can use this unconditionally.
    planning also reading scenario_file lets NOA route-driven lane changes
    (defined in the scenario's optional "route" array) take effect during
    evaluator runs, not just flowsim's actor/ego placement.
    """
    if not scenario_file:
        yield None
        return

    original_text = PIPELINE_JSON.read_text(encoding="utf-8")
    pipeline = json.loads(original_text)
    patched_nodes = []
    for node in _pipeline_nodes(pipeline):
        if not isinstance(node, dict) or node.get("name") not in ("flowsim", "planning", "control"):
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
    # planning is optional (older pipeline.json layouts may omit scenario_file support),
    # so only flowsim is required for the override to be considered successful.
    if "planning" not in patched_nodes:
        print("warning: planning node not patched with scenario_file (missing/malformed "
              "params?) — NOA route-driven lane changes will not be exercised this run",
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


def lane_center_y(lane_idx: int, lane_count: int, lane_width: float, road_c: float = 0.0) -> float:
    """Mirror of include/road_geometry.h::lane_center_y() — N 车道中心对称布置。
    lane_idx: 0=最左, lane_count-1=最右；road_c=道路中心 y 坐标。"""
    if lane_count <= 1:
        return road_c
    return road_c + (lane_idx - (lane_count - 1) * 0.5) * lane_width


def lane_idx_from_y(y: float, lane_count: int, lane_width: float, road_c: float = 0.0) -> int:
    """Mirror of include/road_geometry.h::lane_idx_from_y() — 反推车道索引，
    clamp 到 [0, lane_count-1]。"""
    if lane_count <= 1:
        return 0
    offset = (y - road_c) / lane_width + (lane_count - 1) * 0.5
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

    # 弯道时车道中心线偏移（road=None 或禁用弯道时恒为 0，与既有直道场景
    # 完全等价），lane_error/road_edge_margin 均相对该中心线计算。
    center = road_center_y(x, road)
    y_rel = y - center

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

    return {
        "speed": speed,
        "x": x,
        "y": y,
        "heading": heading,
        "steer": steer,
        "steer_signed": steer_signed,
        "lane_error": nearest_lane_error(y_rel, lane_width, lane_count, 0.0),
        "road_edge_margin": lane_width * lane_count * 0.5 - abs(y_rel) - 1.0,
        "lane_count": lane_count,
        "y_rel": y_rel,
        "min_forward_gap": min_forward_gap,
        "min_abs_gap": min_abs_gap,
        "obs_world": obs_world,
        "driver_mode": str(metrics.get("driver_mode", "") or ""),
        "route_lane": int(metrics.get("route_lane", 0) or 0),
        "entities": scn_entities if isinstance(scn_entities, list) else [],
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
    )

    samples: list[dict] = []
    started = time.monotonic()
    # 缓冲时间：给 demo.sh 构建 node plugins、等待首帧 topology 写入、
    # 以及正常收尾足够余量。首次运行或 CI 冷启动时 12s 经常不够。
    deadline = started + duration + 30.0
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
                # 收到首个有效样本后再给运行时长 + 15s 收尾缓冲，避免
                # 刚出数据就被 deadline 截断。
                deadline = max(deadline, time.monotonic() + duration + 15.0)
        time.sleep(interval)

    if proc.poll() is None:
        print(f"warning: demo.sh still running after {time.monotonic() - started:.1f}s, terminating",
              file=sys.stderr)
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)
    output = proc.stdout.read() if proc.stdout else ""
    if output:
        print(output.rstrip())
    return samples, proc.returncode or 0


def score(samples: list[dict], launcher_log: Path, criteria: dict | None = None, scenario_name: str | None = None, expected_edges: list[tuple[str, str, str]] | None = None, has_noa_route: bool = False, road: dict | None = None, traffic_lights: list | None = None) -> tuple[list[str], list[str], dict]:
    failures: list[str] = []
    warnings: list[str] = []
    criteria = criteria or {}
    if not samples:
        return ["no topology samples collected"], warnings, {}

    last = samples[-1]
    series = [sample_metrics(s, road) for s in samples]
    speeds = [m["speed"] for m in series]
    xs = [m["x"] for m in series]
    lane_errors = [m["lane_error"] for m in series]
    road_margins = [m["road_edge_margin"] for m in series]
    steer_values = [m["steer"] for m in series]
    steer_signed = [m["steer_signed"] for m in series]
    headings = [m["heading"] for m in series]
    timestamps = [float(s.get("timestamp", 0.0) or 0.0) for s in samples]

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
    collision_log_count = len(re.findall(r"COLLISION ego", log_text))
    no_collision_required = bool(criteria.get("no_collision", True))
    if no_collision_required and (collision_pub > 0 or collision_log_count > 0):
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
    # (见 skills/08_state_machine.md)。仅靠拓扑/频率检查发现不了"模式没升级"或
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

    if has_red_light_check and scenario_lights:
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

                # 绿灯期间不必要停车检测：灯为绿、ego 尚未过停止线、车速极低
                if curr_state == "green" and ego_x_i < stop_x and m["speed"] < 0.5:
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
            warnings.append(
                "red-light check enabled but no traffic_light state data in samples "
                "(scene.entities may not include tl type)"
            )
        elif red_light_violation:
            failures.append("red light violation: ego crossed stop line during red phase")
        if green_phase_max_stop_s > 5.0:
            warnings.append(
                f"unnecessary stop during green: ego stopped {green_phase_max_stop_s:.1f}s "
                f"while light was green (possible over-conservative planning)"
            )

    steer_saturation_ratio = sum(1 for s in steer_values if s > 0.219) / max(1, len(steer_values))
    if steer_saturation_ratio > 0.45:
        warnings.append(f"steer saturated often: {steer_saturation_ratio * 100:.0f}% samples")

    yaw_rates: list[float] = []
    steer_rates: list[float] = []
    npc_speed_spikes: list[float] = []
    npc_lateral_spikes: list[float] = []
    for i in range(1, len(series)):
        dt = timestamps[i] - timestamps[i - 1]
        if dt <= 1e-3 or dt > 2.0:
            continue
        yaw_rates.append(abs(angle_diff(headings[i], headings[i - 1])) / dt)
        steer_rates.append(abs(steer_signed[i] - steer_signed[i - 1]) / dt)

        prev_obs = {o["id"]: o for o in series[i - 1]["obs_world"]}
        for obs in series[i]["obs_world"]:
            prev = prev_obs.get(obs["id"])
            if not prev:
                continue
            dx = obs["x"] - prev["x"]
            dy = obs["y"] - prev["y"]
            # NPC respawn 时位置跳变（recycle 距离 50-138m）会反算出 25-500+ m/s
            # 的"假速度"。原先用 speed > 50.0 过滤会漏掉 25-50 m/s 区间的中等跳变
            # （50m recycle / 2s 采样 ≈ 25 m/s），这些跳变会触发 >45.0 的 respawn
            # jump 告警，导致 CI 误报。改为用位移阈值：连续两帧间位移 > 30m 几乎
            # 不可能是真实运动（即使 30 m/s × 1s 采样也只有 30m），视为 teleport。
            disp = math.hypot(dx, dy)
            if disp > 30.0:
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

    if yaw_rate_rms > 0.35 or max_yaw_rate > 1.2 or (heading_flip_rate > 1.2 and yaw_rate_rms > 0.1):
        warnings.append(
            f"ego yaw wobble: yaw_rms={yaw_rate_rms:.2f} rad/s, max={max_yaw_rate:.2f}, flips={heading_flip_rate:.2f}/s"
        )
    if steer_rate_rms > 0.9 or max_steer_rate > 3.0 or steer_flip_rate > 1.0:
        warnings.append(
            f"steer oscillation: steer_rate_rms={steer_rate_rms:.2f}/s, max={max_steer_rate:.2f}/s, flips={steer_flip_rate:.2f}/s"
        )
    if max_npc_lateral_speed > 12.0:
        warnings.append(
            f"npc motion spike: max_speed={max_npc_speed:.1f} m/s, max_lateral={max_npc_lateral_speed:.1f} m/s"
        )
    elif max_npc_speed > 45.0:
        warnings.append(f"npc respawn jump: max_speed={max_npc_speed:.1f} m/s, max_lateral={max_npc_lateral_speed:.1f} m/s")

    drops = sum(int(t.get("drop", 0) or 0) for t in topics.values())
    total_pub = sum(int(t.get("pub", 0) or 0) for t in topics.values())
    drop_rate = drops / total_pub if total_pub > 0 else 0.0
    # Tolerate transient drops during startup/scheduling jitter.
    # Visual upgrades do not affect backend transport; threshold raised to avoid
    # flaky CI failures on heavily-loaded runners.
    if drops > 50 or drop_rate > 0.01:
        failures.append(f"message drops detected: {drops} (rate {drop_rate*100:.2f}%)")
    elif drops > 0:
        warnings.append(f"message drops detected: {drops} (rate {drop_rate*100:.2f}%)")

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
    # 分层识别率 FAIL/WARN（仅当该层有足够样本 ≥ 5 帧时才判定，避免小样本噪声）
    REC_MIN_SAMPLES = 5
    for layer_name, rate_key, count_key in [
        ("vehicle", "recognition_rate_vehicle", "truth_count_vehicle"),
        ("vru", "recognition_rate_vru", "truth_count_vru"),
    ]:
        rate = perception[rate_key]
        n = perception[count_key]
        if n < REC_MIN_SAMPLES:
            continue  # 样本不足，跳过判定（仍写入 summary 供诊断）
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
    }

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
        sample = load_json(args.json_file)
        samples = [sample] if sample else []
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

    failures, warnings, summary = score(samples, LAUNCHER_STDERR, criteria, scenario_name, has_noa_route=has_noa_route, road=road, traffic_lights=traffic_lights)

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
        payload = {
            "scenario": summary.get("scenario"),
            "result": result,
            "failures": failures,
            "warnings": warnings,
            "summary": summary,
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