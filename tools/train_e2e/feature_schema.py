"""Shared feature contracts for FlowEngine E2E training tools."""

from __future__ import annotations


DATASET_SCHEMA_V1 = "flowengine.e2e_dataset.v1"
DATASET_SCHEMA_V2 = "flowengine.e2e_dataset.v2"
SAMPLE_SCHEMA_V2 = "flowengine.e2e_sample.v2"

FEATURE_NAMES_V1 = ["ego_v", "ego_y", "ego_heading", "ego_yaw_rate"]
FEATURE_NAMES_V2 = [
    "ego_v",
    "ego_y",
    "ego_heading",
    "ego_yaw_rate",
    "front0_x",
    "front0_y",
    "front0_vx",
    "front0_type",
    "front0_confidence",
    "front1_x",
    "front1_y",
    "front1_vx",
    "front1_type",
    "front1_confidence",
    "control_brake",
    "control_emergency_stop",
]
LABEL_NAMES = ["target_speed"]


def sample_feature_names(sample: dict) -> list[str]:
    if sample.get("schema_version") == SAMPLE_SCHEMA_V2 or "features_v2" in sample:
        return FEATURE_NAMES_V2
    return FEATURE_NAMES_V1


def normalize_obstacles(obstacles: object, limit: int = 2) -> list[dict]:
    if not isinstance(obstacles, list):
        return []
    normalized = [obs for obs in obstacles if isinstance(obs, dict)]
    normalized.sort(key=lambda obs: (float(obs.get("x", 1e9) or 1e9), abs(float(obs.get("y", 0.0) or 0.0))))
    return normalized[:limit]


def features_from_sample(sample: dict, feature_names: list[str] | None = None) -> list[float]:
    names = feature_names or sample_feature_names(sample)
    if names == FEATURE_NAMES_V1:
        features = sample.get("features")
        if not isinstance(features, list) or len(features) != len(FEATURE_NAMES_V1):
            raise ValueError("sample does not contain v1 features")
        return [float(value) for value in features]

    if names != FEATURE_NAMES_V2:
        raise ValueError(f"unsupported feature schema: {names!r}")

    features_v2 = sample.get("features_v2")
    if isinstance(features_v2, list) and len(features_v2) == len(FEATURE_NAMES_V2):
        return [float(value) for value in features_v2]

    return build_v2_features(
        ego=sample.get("ego", {}),
        obstacles=sample.get("obstacles", []),
        control=sample.get("control", {}),
        fallback_features=sample.get("features"),
    )


def build_v2_features(ego: object, obstacles: object, control: object, fallback_features: object = None) -> list[float]:
    ego_obj = ego if isinstance(ego, dict) else {}
    control_obj = control if isinstance(control, dict) else {}
    fallback = fallback_features if isinstance(fallback_features, list) else []

    ego_v = _float(ego_obj.get("v", ego_obj.get("speed", _get(fallback, 0))))
    ego_y = _float(ego_obj.get("y", _get(fallback, 1)))
    ego_heading = _float(ego_obj.get("heading", _get(fallback, 2)))
    ego_yaw_rate = _float(ego_obj.get("yaw_rate", _get(fallback, 3)))

    rows = [ego_v, ego_y, ego_heading, ego_yaw_rate]
    for obstacle in normalize_obstacles(obstacles, limit=2):
        rows.extend(
            [
                _float(obstacle.get("x")),
                _float(obstacle.get("y")),
                _float(obstacle.get("vx")),
                _float(obstacle.get("type")),
                _float(obstacle.get("confidence")),
            ]
        )
    while len(rows) < 14:
        rows.append(0.0)

    rows.append(_float(control_obj.get("brake")))
    rows.append(1.0 if bool(control_obj.get("emergency_stop", False)) else 0.0)
    return rows


def features_from_state(state: dict, feature_names: list[str]) -> tuple[list[float], dict]:
    scene = state.get("scene", {}) if isinstance(state.get("scene"), dict) else {}
    scene_ego = scene.get("ego", {}) if isinstance(scene.get("ego"), dict) else {}
    metrics_vehicle = state.get("metrics", {}).get("vehicle", {}) if isinstance(state.get("metrics"), dict) else {}

    ego_v = _float(scene_ego.get("speed", metrics_vehicle.get("speed", 0.0)))
    ego_y = _float(scene_ego.get("y", metrics_vehicle.get("y", 0.0)))
    ego_heading = _float(scene_ego.get("heading", scene_ego.get("hdg", 0.0)))
    ego_yaw_rate = _float(scene_ego.get("yaw_rate", 0.0))
    ego_x = _float(scene_ego.get("x", metrics_vehicle.get("x", 0.0)))
    ego = {"x": ego_x, "y": ego_y, "v": ego_v, "heading": ego_heading, "yaw_rate": ego_yaw_rate}

    fallback = [ego_v, ego_y, ego_heading, ego_yaw_rate]
    if feature_names == FEATURE_NAMES_V1:
        return fallback, ego
    if feature_names == FEATURE_NAMES_V2:
        control = state.get("control", {}) if isinstance(state.get("control"), dict) else {}
        return build_v2_features(ego, scene.get("obstacles", []), control, fallback), ego
    raise ValueError(f"unsupported feature schema: {feature_names!r}")


def _get(values: list, index: int) -> float:
    return values[index] if index < len(values) else 0.0


def _float(value: object) -> float:
    try:
        return float(value or 0.0)
    except (TypeError, ValueError):
        return 0.0
