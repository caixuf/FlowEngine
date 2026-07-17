"""Shared feature contracts for FlowEngine E2E training tools (v3)."""

from __future__ import annotations

import math
import random

DATASET_SCHEMA_V1 = "flowengine.e2e_dataset.v1"
DATASET_SCHEMA_V2 = "flowengine.e2e_dataset.v2"
DATASET_SCHEMA_V3 = "flowengine.e2e_dataset.v3"
SAMPLE_SCHEMA_V2 = "flowengine.e2e_sample.v2"
SAMPLE_SCHEMA_V3 = "flowengine.e2e_sample.v3"

# ── 特征名定义 ────────────────────────────────────────────────

FEATURE_NAMES_V1 = ["ego_v", "ego_y", "ego_heading", "ego_yaw_rate"]

FEATURE_NAMES_V2 = [
    "ego_v", "ego_y", "ego_heading", "ego_yaw_rate",
    "front0_x", "front0_y", "front0_vx", "front0_type", "front0_confidence",
    "front1_x", "front1_y", "front1_vx", "front1_type", "front1_confidence",
    "control_brake", "control_emergency_stop",
]

FEATURE_NAMES_V3 = [
    # ── v2 基础（16 维，索引 0-15）──
    "ego_v", "ego_y", "ego_heading", "ego_yaw_rate",
    "front0_x", "front0_y", "front0_vx", "front0_type", "front0_confidence",
    "front1_x", "front1_y", "front1_vx", "front1_type", "front1_confidence",
    "control_brake", "control_emergency_stop",

    # ── 感知统计（6 维，索引 16-21）──
    "lidar_point_count",        # 点云总点数
    "lidar_front_density",      # 前方 30m 点密度
    "lidar_clutter_ratio",      # 杂波比例 (z<0.3m 的点 / 总点)
    "perception_obj_count",     # 检测到的障碍物数
    "perception_max_conf",      # 最高置信度
    "perception_mean_conf",     # 平均置信度

    # ── 场景上下文（7 维，索引 22-28）──
    "tl_state",                 # 0=green, 1=yellow, 2=red, -1=无灯
    "tl_distance",              # 距最近红灯距离 (m)
    "road_curvature",           # 当前道路曲率 (1/R, m⁻¹)
    "road_speed_limit",         # 限速 (m/s)
    "lane_count",               # 车道数
    "lane_width",               # 车道宽度 (m)
    "ego_lane_offset",          # 距车道中心偏移 (m)

    # ── 障碍物全貌统计（30 维，索引 29-58）──
    # 类型分布 (5): 车/卡车/行人/未知/总计
    "obs_count",
    "obs_car_count", "obs_truck_count", "obs_ped_count", "obs_unknown_count",
    # 距离分布 (5): min/p25/p50/p75/max
    "obs_dist_min", "obs_dist_p25", "obs_dist_p50", "obs_dist_p75", "obs_dist_max",
    # 速度分布 (5): min/p25/p50/p75/max
    "obs_speed_min", "obs_speed_p25", "obs_speed_p50", "obs_speed_p75", "obs_speed_max",
    # 前方最近间距 (5): 左/中/右 + 绝对值 + 相对速度
    "front_min_gap",
    "left_min_gap",
    "right_min_gap",
    "front_min_gap_rel_v",
    "front_min_gap_diff",
    # 后方来车情况 (5)
    "rear_min_gap",
    "rear_has_fast_approach",
    "rear_closest_vx",
    "rear_closest_length",
    "rear_closest_width",
    # 左右车道占用 (5)
    "left_lane_occupied",
    "right_lane_occupied",
    "left_lane_nearest_x",
    "right_lane_nearest_x",
    "left_lane_nearest_vx",
]

LABEL_NAMES = ["target_speed"]

# ── 维度常量 ──────────────────────────────────────────────────

V2_DIM = len(FEATURE_NAMES_V2)   # 16
V3_DIM = len(FEATURE_NAMES_V3)   # 59


def sample_feature_names(sample: dict) -> list[str]:
    if sample.get("schema_version") == SAMPLE_SCHEMA_V3 or "features_v3" in sample:
        return FEATURE_NAMES_V3
    if sample.get("schema_version") == SAMPLE_SCHEMA_V2 or "features_v2" in sample:
        return FEATURE_NAMES_V2
    return FEATURE_NAMES_V1


# ── 辅助函数 ──────────────────────────────────────────────────

def _float(value: object) -> float:
    try:
        return float(value or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _get(values: list, index: int) -> float:
    return values[index] if index < len(values) else 0.0


def _percentile(vals: list[float], pct: float) -> float:
    if not vals:
        return 0.0
    sorted_vals = sorted(vals)
    k = (len(sorted_vals) - 1) * pct / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[f] * (c - k) + sorted_vals[c] * (k - f)


# ── 标准化障碍物列表 ──────────────────────────────────────────

def normalize_obstacles(obstacles: object, limit: int = 2) -> list[dict]:
    if not isinstance(obstacles, list):
        return []
    normalized = [obs for obs in obstacles if isinstance(obs, dict)]
    normalized.sort(key=lambda obs: (float(obs.get("x", 1e9) or 1e9), abs(float(obs.get("y", 0.0) or 0.0))))
    return normalized[:limit]


def get_full_obstacles(obstacles: object) -> list[dict]:
    if not isinstance(obstacles, list):
        return []
    return [obs for obs in obstacles if isinstance(obs, dict)]


# ── v3 特征构建 ──────────────────────────────────────────────

V3_DIM_TOTAL = V3_DIM  # 59


def build_v2_features(ego: object, obstacles: object, control: object, fallback_features: object = None) -> list[float]:
    """v2 特征构建（16 维），保持向后兼容。"""
    ego_obj = ego if isinstance(ego, dict) else {}
    control_obj = control if isinstance(control, dict) else {}
    fallback = fallback_features if isinstance(fallback_features, list) else []

    ego_v = _float(ego_obj.get("v", ego_obj.get("speed", _get(fallback, 0))))
    ego_y = _float(ego_obj.get("y", _get(fallback, 1)))
    ego_heading = _float(ego_obj.get("heading", _get(fallback, 2)))
    ego_yaw_rate = _float(ego_obj.get("yaw_rate", _get(fallback, 3)))

    rows = [ego_v, ego_y, ego_heading, ego_yaw_rate]
    for obstacle in normalize_obstacles(obstacles, limit=2):
        rows.extend([
            _float(obstacle.get("x")),
            _float(obstacle.get("y")),
            _float(obstacle.get("vx")),
            _float(obstacle.get("type")),
            _float(obstacle.get("confidence")),
        ])
    while len(rows) < 14:
        rows.append(0.0)

    rows.append(_float(control_obj.get("brake")))
    rows.append(1.0 if bool(control_obj.get("emergency_stop", False)) else 0.0)
    return rows


def _obstacle_type_code(obs: dict) -> int:
    """障碍物类型编码: 0=unknown, 1=car, 2=truck, 3=pedestrian"""
    t = str(obs.get("type", "")).lower()
    if t in ("car", "vehicle", "1"):
        return 1
    if t in ("truck", "2"):
        return 2
    if t in ("pedestrian", "3", "ped"):
        return 3
    return 0


def _is_in_lane(obs: dict, lane_center_y: float, lane_width: float, margin: float = 1.5) -> bool:
    """判断障碍物是否在给定车道内"""
    oy = _float(obs.get("y"))
    return abs(oy - lane_center_y) < lane_width * 0.5 + margin


def build_v3_features(
    ego: object,
    obstacles: object,
    control: object,
    lidar_stats: object | None = None,
    scene_context: object | None = None,
    fallback_features: object | None = None,
) -> list[float]:
    """
    v3 特征构建（59 维）。

    参数：
        ego: {v, y, heading, yaw_rate, x, lane_offset, lane_center_y}
        obstacles: [{x, y, vx, vy, type, confidence, length, width}]
        control: {throttle, brake, steering, emergency_stop}
        lidar_stats: {point_count, front_density, clutter_ratio}
        scene_context: {tl_state, tl_distance, curvature, speed_limit,
                        lane_count, lane_width, ego_lane_offset}
    """
    ego_obj = ego if isinstance(ego, dict) else {}
    control_obj = control if isinstance(control, dict) else {}
    lidar = lidar_stats if isinstance(lidar_stats, dict) else {}
    ctx = scene_context if isinstance(scene_context, dict) else {}

    # ── v2 基础（16 维） ──
    features = build_v2_features(ego, obstacles, control, fallback_features)

    # ── 感知统计（6 维，索引 16-21） ──
    features.append(_float(lidar.get("point_count", 0.0)))
    features.append(_float(lidar.get("front_density", 0.0)))
    features.append(_float(lidar.get("clutter_ratio", 0.0)))

    all_obs = get_full_obstacles(obstacles)
    obj_count = len(all_obs)
    confs = [_float(o.get("confidence", 0.0)) for o in all_obs if _float(o.get("confidence", 0.0)) > 0]
    features.append(float(obj_count))
    features.append(max(confs) if confs else 0.0)
    features.append(sum(confs) / len(confs) if confs else 0.0)

    # ── 场景上下文（7 维，索引 22-28） ──
    features.append(_float(ctx.get("tl_state", -1.0)))
    features.append(_float(ctx.get("tl_distance", -1.0)))
    features.append(_float(ctx.get("curvature", 0.0)))
    features.append(_float(ctx.get("speed_limit", 30.0)))
    features.append(_float(ctx.get("lane_count", 2.0)))
    features.append(_float(ctx.get("lane_width", 3.5)))
    features.append(_float(ctx.get("ego_lane_offset", 0.0)))

    # ── 障碍物全貌统计（30 维，索引 29-58） ──
    # 类型分布
    type_counts = {1: 0, 2: 0, 3: 0, 0: 0}
    for o in all_obs:
        tc = _obstacle_type_code(o)
        type_counts[tc] = type_counts.get(tc, 0) + 1

    features.append(float(len(all_obs)))
    features.append(float(type_counts.get(1, 0)))  # car
    features.append(float(type_counts.get(2, 0)))  # truck
    features.append(float(type_counts.get(3, 0)))  # pedestrian
    features.append(float(type_counts.get(0, 0)))  # unknown

    # 距离分布
    dists = [math.hypot(_float(o.get("x", 0)), _float(o.get("y", 0))) for o in all_obs]
    if dists:
        ds = sorted(dists)
        features.extend([ds[0], _percentile(dists, 25), _percentile(dists, 50),
                         _percentile(dists, 75), ds[-1]])
    else:
        features.extend([0.0] * 5)

    # 速度分布
    speeds = [abs(_float(o.get("vx", 0))) for o in all_obs if abs(_float(o.get("vx", 0))) > 0.01]
    if speeds:
        ss = sorted(speeds)
        features.extend([ss[0], _percentile(speeds, 25), _percentile(speeds, 50),
                         _percentile(speeds, 75), ss[-1]])
    else:
        features.extend([0.0] * 5)

    # 前方/左/右最近间距
    ego_x = _float(ego_obj.get("x", 0))
    ego_y = _float(ego_obj.get("y", 0))
    lane_width = _float(ctx.get("lane_width", 3.5))
    lane_center_y = _float(ego_obj.get("lane_center_y", ego_y))

    front_gaps = []
    left_gaps = []
    right_gaps = []
    rear_obs = []
    front_obs = []

    for o in all_obs:
        ox = _float(o.get("x", 1e9))
        oy = _float(o.get("y", 0))
        dx = ox - ego_x
        dy = oy - ego_y
        dist = math.hypot(dx, dy)

        # 前后区分
        if dx > 0:
            front_obs.append(o)
            if abs(dy) < lane_width * 0.5 + 1.5:
                front_gaps.append(dist)
            elif dy < -lane_width * 0.5:
                left_gaps.append(dist)
            elif dy > lane_width * 0.5:
                right_gaps.append(dist)
        else:
            rear_obs.append(o)

    features.append(min(front_gaps) if front_gaps else 200.0)
    features.append(min(left_gaps) if left_gaps else 200.0)
    features.append(min(right_gaps) if right_gaps else 200.0)

    # 前车相对速度
    if front_gaps and front_obs:
        closest = min(front_obs, key=lambda o: math.hypot(_float(o.get("x", 0)) - ego_x,
                                                          _float(o.get("y", 0)) - ego_y))
        rel_v = _float(closest.get("vx", 0)) - _float(ego_obj.get("v", 0))
        features.append(rel_v)
        features.append(front_gaps[0] - (features[-3] if len(features) >= 3 else 3.0))
    else:
        features.extend([0.0, 0.0])

    # 后方来车
    if rear_obs:
        rear_dists = [math.hypot(_float(o.get("x", 0)) - ego_x, _float(o.get("y", 0)) - ego_y)
                      for o in rear_obs]
        rear_closest = min(rear_obs, key=lambda o: math.hypot(_float(o.get("x", 0)) - ego_x,
                                                               _float(o.get("y", 0)) - ego_y))
        features.append(min(rear_dists))
        rear_vx = _float(rear_closest.get("vx", 0))
        features.append(1.0 if rear_vx > _float(ego_obj.get("v", 0)) + 2.0 else 0.0)
        features.append(rear_vx)
        features.append(_float(rear_closest.get("length", 4.6)))
        features.append(_float(rear_closest.get("width", 2.0)))
    else:
        features.extend([200.0, 0.0, 0.0, 0.0, 0.0])

    # 左右车道占用
    left_occ = 0
    right_occ = 0
    left_nearest = 200.0
    right_nearest = 200.0
    left_vx = 0.0
    right_vx = 0.0
    for o in all_obs:
        oy = _float(o.get("y", 0))
        ox = _float(o.get("x", 0))
        if abs(oy - (lane_center_y - lane_width)) < lane_width * 0.7 and ox > 0:
            left_occ = 1
            d = math.hypot(ox - ego_x, oy - ego_y)
            if d < left_nearest:
                left_nearest = d
                left_vx = _float(o.get("vx", 0))
        if abs(oy - (lane_center_y + lane_width)) < lane_width * 0.7 and ox > 0:
            right_occ = 1
            d = math.hypot(ox - ego_x, oy - ego_y)
            if d < right_nearest:
                right_nearest = d
                right_vx = _float(o.get("vx", 0))
    features.append(float(left_occ))
    features.append(float(right_occ))
    features.append(left_nearest)
    features.append(right_nearest)
    features.append(left_vx)

    # 确保刚好 59 维
    while len(features) < V3_DIM_TOTAL:
        features.append(0.0)
    if len(features) > V3_DIM_TOTAL:
        features = features[:V3_DIM_TOTAL]

    return features


# ── 数据增强 ──────────────────────────────────────────────────

def augment_features(features: list[float], feature_names: list[str],
                     noise_scale: float = 0.05,
                     drop_prob: float = 0.15,
                     tl_flip_prob: float = 0.05) -> list[float]:
    """
    对特征向量注入传感器噪声增强（返回新副本）。

    - 障碍物位置: 高斯噪声
    - 障碍物速度: 高斯噪声
    - 随机丢弃障碍物特征: drop_prob
    - 红绿灯状态翻转: tl_flip_prob
    """
    if feature_names not in (FEATURE_NAMES_V2, FEATURE_NAMES_V3):
        return list(features)

    aug = list(features)
    n = len(feature_names)

    # 对障碍物位置/速度加噪声（v2: 4-13, v3: 4-13）
    for idx in range(4, min(14, n)):
        aug[idx] += random.gauss(0, abs(aug[idx]) * noise_scale + 0.01)

    # 随机丢弃障碍物（设为 0）
    for offset in [4, 9]:  # front0 and front1 start indices
        if random.random() < drop_prob:
            for di in range(5):
                if offset + di < n:
                    aug[offset + di] = 0.0

    # 红绿灯翻转
    if feature_names is FEATURE_NAMES_V3:
        tl_idx = feature_names.index("tl_state")
        if 0 <= tl_idx < n and random.random() < tl_flip_prob:
            s = aug[tl_idx]
            if s == 0.0:
                aug[tl_idx] = 2.0  # green → red
            elif s == 2.0:
                aug[tl_idx] = 0.0  # red → green
            elif s == 1.0:
                aug[tl_idx] = random.choice([0.0, 2.0])  # yellow → random

    return aug


# ── 从状态构建特征 ────────────────────────────────────────────

def features_from_state(state: dict, feature_names: list[str]) -> tuple[list[float], dict]:
    scene = state.get("scene", {}) if isinstance(state.get("scene"), dict) else {}
    scene_ego = scene.get("ego", {}) if isinstance(scene.get("ego"), dict) else {}
    metrics_vehicle = state.get("metrics", {}).get("vehicle", {}) if isinstance(state.get("metrics"), dict) else {}

    ego_v = _float(scene_ego.get("speed", metrics_vehicle.get("speed", 0.0)))
    ego_y = _float(scene_ego.get("y", metrics_vehicle.get("y", 0.0)))
    ego_heading = _float(scene_ego.get("heading", scene_ego.get("hdg", 0.0)))
    ego_yaw_rate = _float(scene_ego.get("yaw_rate", 0.0))
    ego_x = _float(scene_ego.get("x", metrics_vehicle.get("x", 0.0)))
    lane_center_y = _float(scene.get("lane", {}).get("center", 0.0)) if isinstance(scene.get("lane"), dict) else 0.0
    lane_width = _float(scene.get("lane", {}).get("width", 3.5)) if isinstance(scene.get("lane"), dict) else 3.5
    ego_lane_offset = ego_y - lane_center_y

    ego = {"x": ego_x, "y": ego_y, "v": ego_v, "heading": ego_heading,
           "yaw_rate": ego_yaw_rate, "lane_offset": ego_lane_offset, "lane_center_y": lane_center_y}

    fallback = [ego_v, ego_y, ego_heading, ego_yaw_rate]
    if feature_names == FEATURE_NAMES_V1:
        return fallback, ego
    if feature_names == FEATURE_NAMES_V2:
        control = state.get("control", {}) if isinstance(state.get("control"), dict) else {}
        return build_v2_features(ego, scene.get("obstacles", []), control, fallback), ego
    if feature_names == FEATURE_NAMES_V3:
        control = state.get("control", {}) if isinstance(state.get("control"), dict) else {}
        lidar_stats = state.get("lidar_stats", {}) if isinstance(state.get("lidar_stats"), dict) else {}
        road_data = state.get("road_network", {}) if isinstance(state.get("road_network"), dict) else {}
        scene_context = {
            "tl_state": _float(scene.get("tl_state", -1)),
            "tl_distance": _float(scene.get("tl_distance", -1)),
            "curvature": _float(scene.get("curvature", scene.get("road", {}).get("curve_offset_m", 0))),
            "speed_limit": _float(scene.get("speed_limit", 30)),
            "lane_count": _float(scene.get("lane", {}).get("count", 2)),
            "lane_width": _float(scene.get("lane", {}).get("width", 3.5)),
            "ego_lane_offset": ego_lane_offset,
        }
        return build_v3_features(ego, scene.get("obstacles", []), control,
                                 lidar_stats, scene_context, fallback), ego
    raise ValueError(f"unsupported feature schema: {feature_names!r}")


def features_from_sample(sample: dict, feature_names: list[str] | None = None) -> list[float]:
    names = feature_names or sample_feature_names(sample)
    if names == FEATURE_NAMES_V1:
        features = sample.get("features")
        if not isinstance(features, list) or len(features) != len(FEATURE_NAMES_V1):
            raise ValueError("sample does not contain v1 features")
        return [float(value) for value in features]

    if names == FEATURE_NAMES_V2:
        features_v2 = sample.get("features_v2")
        if isinstance(features_v2, list) and len(features_v2) == len(FEATURE_NAMES_V2):
            return [float(value) for value in features_v2]
        return build_v2_features(
            ego=sample.get("ego", {}),
            obstacles=sample.get("obstacles", []),
            control=sample.get("control", {}),
            fallback_features=sample.get("features"),
        )

    if names == FEATURE_NAMES_V3:
        features_v3 = sample.get("features_v3")
        if isinstance(features_v3, list) and len(features_v3) == len(FEATURE_NAMES_V3):
            return [float(value) for value in features_v3]
        return build_v3_features(
            ego=sample.get("ego", {}),
            obstacles=sample.get("obstacles", []),
            control=sample.get("control", {}),
            lidar_stats=sample.get("lidar_stats", {}),
            scene_context=sample.get("scene_context", {}),
            fallback_features=sample.get("features"),
        )

    raise ValueError(f"unsupported feature schema: {names!r}")
