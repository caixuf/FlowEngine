#!/usr/bin/env python3
"""
FlowBoard 数据归一化 + 契约校验 — 纯函数模块

从 flowboard_server.py 抽出，便于单元测试（无副作用、无网络/线程/HTTP）。
把 monitor_server / discovery / NodePlugin 三种输入格式统一成 FlowBoard 模型。

数据契约见 tools/schema/flowboard.schema.json 与 docs/FLOWBOARD_CONTRACT.md。
"""

import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple

# ── Schema (可选 jsonschema，缺失时退化为轻量结构校验) ─────────────
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "schema", "flowboard.schema.json")

_SCHEMA_CACHE: Optional[Dict[str, Any]] = None


def load_schema() -> Optional[Dict[str, Any]]:
    """Load and cache the JSON schema. Returns None if unavailable."""
    global _SCHEMA_CACHE
    if _SCHEMA_CACHE is None:
        try:
            with open(SCHEMA_PATH) as f:
                _SCHEMA_CACHE = json.load(f)
        except Exception:
            _SCHEMA_CACHE = {}
    return _SCHEMA_CACHE or None


def validate_payload(raw: Any) -> Tuple[bool, str]:
    """Lightweight contract validation.

    Returns (ok, reason). Uses `jsonschema` when installed (dev/CI); otherwise
    falls back to a minimal structural check so production stays zero-dependency.
    Validation is advisory: callers should degrade gracefully, never crash.
    """
    if not isinstance(raw, dict):
        return False, f"top-level payload must be an object, got {type(raw).__name__}"

    # Structural sanity: fields, when present, must have the expected container type.
    for key, typ in (("nodes", list), ("endpoints", list),
                     ("topic_roles", list), ("metrics", dict),
                     ("registry", dict), ("scene", dict)):
        if key in raw and not isinstance(raw[key], typ):
            return False, f"field '{key}' must be {typ.__name__}, got {type(raw[key]).__name__}"

    schema = load_schema()
    if schema:
        try:
            import jsonschema  # type: ignore
            jsonschema.validate(instance=raw, schema=schema)
        except ImportError:
            pass  # jsonschema not installed → structural check above is enough
        except Exception as e:  # jsonschema.ValidationError
            return False, f"schema validation failed: {getattr(e, 'message', str(e))}"
    return True, "ok"


# ── 真实数据归一化 ────────────────────────────────────────

def _cap_role(caps: Any) -> str:
    try:
        caps = int(caps)
    except Exception:
        caps = 0
    is_pub = bool(caps & 1)
    is_sub = bool(caps & 2)
    if is_pub and is_sub:
        return "pubsub"
    if is_pub:
        return "pub"
    if is_sub:
        return "sub"
    return "unknown"


def _topic_type_map(registry: Optional[Dict[str, Any]]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for t in (registry or {}).get("topics", []) or []:
        name = t.get("name") or t.get("topic")
        if name:
            out[name] = t.get("type_id", "0x00000000")
    return out


def _stats_map(raw: Dict[str, Any]) -> Dict[str, Any]:
    stats: Dict[str, Any] = {}
    metric_topics: List[Any] = []
    if isinstance(raw.get("metrics"), dict):
        metric_topics.extend(raw["metrics"].get("topics", []) or [])
    metric_topics.extend(raw.get("topics", []) or [])
    for t in metric_topics:
        name = t.get("topic") or t.get("name")
        if name:
            stats[name] = t
    return stats


def _synth_nodes_from_registry(registry: Optional[Dict[str, Any]],
                               stats_by_topic: Dict[str, Any]) -> List[Dict[str, Any]]:
    nodes: List[Dict[str, Any]] = []
    type_map = _topic_type_map(registry)
    for i, task in enumerate((registry or {}).get("tasks", []) or []):
        name = task.get("name") or f"task_{i}"
        topics = []
        for topic in task.get("inputs", []) or []:
            s = stats_by_topic.get(topic, {})
            topics.append({
                "topic": topic,
                "role": "sub",
                "caps": 2,
                "type_id": type_map.get(topic, s.get("type_id", "0x00000000")),
                "freq": float(s.get("freq", 0) or 0),
            })
        for topic in task.get("outputs", []) or []:
            s = stats_by_topic.get(topic, {})
            topics.append({
                "topic": topic,
                "role": "pub",
                "caps": 1,
                "type_id": type_map.get(topic, s.get("type_id", "0x00000000")),
                "freq": float(s.get("freq", 0) or 0),
            })
        nodes.append({
            "name": name,
            "pid": task.get("pid", 0),
            "alive": task.get("alive", True),
            "caps": task.get("caps", 0),
            "kind": "task",
            "description": task.get("desc", ""),
            "plugin": task.get("plugin", ""),
            "topics": topics,
        })
    return nodes


def _empty_model() -> Dict[str, Any]:
    return {"nodes": [], "metrics": {}, "timestamp": time.time(), "endpoints": []}


def normalize_live_data(raw: Any) -> Dict[str, Any]:
    """把 flow_e2e 状态文件、monitor_server JSON、discovery JSON 统一成 FlowBoard 模型。

    契约校验失败或输入非 dict 时，返回明确的空模型（降级），绝不抛异常。
    """
    if not isinstance(raw, dict):
        return _empty_model()

    ok, _reason = validate_payload(raw)
    if not ok:
        # 脏数据不流进渲染：退化到空模型，reason 由调用方决定是否记录。
        model = _empty_model()
        model["invalid_reason"] = _reason
        return model

    data = dict(raw)

    # monitor_server.c 输出: {bus, topology:{nodes}, topics}
    if "topology" in data and isinstance(data.get("topology"), dict):
        topo = data.get("topology") or {}
        metrics = data.get("metrics") or {}
        metrics.setdefault("bus", data.get("bus", {}))
        metrics.setdefault("topics", data.get("topics", []))
        data = {
            "self": topo.get("self", data.get("self", "flowengine")),
            "nodes": topo.get("nodes", []),
            "metrics": metrics,
            "registry": data.get("registry", {}),
            "timestamp": data.get("timestamp", time.time()),
        }

    # Hoist scene/registry from metrics to top-level if not already there
    metrics_raw = data.get("metrics") or {}
    if "scene" not in data and "scene" in metrics_raw:
        data["scene"] = metrics_raw["scene"]
    if "registry" not in data and "registry" in metrics_raw:
        data["registry"] = metrics_raw["registry"]
    if "sysmon" not in data and "sysmon" in metrics_raw:
        data["sysmon"] = metrics_raw["sysmon"]

    registry = data.get("registry") or (data.get("metrics") or {}).get("registry") or {}
    if registry and "registry" not in data:
        data["registry"] = registry
    stats_by_topic = _stats_map(data)

    nodes = data.get("nodes") or []
    if (not nodes) and registry.get("tasks"):
        nodes = _synth_nodes_from_registry(registry, stats_by_topic)
    else:
        # 补齐 discovery topic 的 role/caps/type/freq 字段。
        # 同时兼容 NodePlugin 格式: inputs[]/outputs[] → topics[]
        type_map = _topic_type_map(registry)
        for n in nodes:
            if not n.get("topics") and (n.get("inputs") or n.get("outputs")):
                topics = []
                for tp in (n.get("inputs") or []):
                    topics.append({"topic": tp, "role": "sub", "caps": 2})
                for tp in (n.get("outputs") or []):
                    topics.append({"topic": tp, "role": "pub", "caps": 1})
                n["topics"] = topics
            for t in n.get("topics", []) or []:
                topic = t.get("topic") or t.get("name", "")
                caps = t.get("caps", t.get("capabilities", 0))
                role = t.get("role") or _cap_role(caps)
                if role == "unknown" and float(t.get("freq", 0) or 0) > 0:
                    role = "pub"
                t["role"] = role
                t["caps"] = caps
                if topic and not t.get("type_id"):
                    t["type_id"] = type_map.get(topic, "0x00000000")

    topic_roles: Dict[str, Any] = {}
    endpoints: List[Dict[str, Any]] = []
    for n in nodes:
        node_name = n.get("name", "unknown")
        for t in n.get("topics", []) or []:
            topic = t.get("topic") or t.get("name")
            if not topic:
                continue
            role = t.get("role") or _cap_role(t.get("caps", 0))
            s = stats_by_topic.get(topic, {})
            topic_roles.setdefault(topic, {"topic": topic, "publishers": [], "subscribers": []})
            if role in ("pub", "pubsub"):
                topic_roles[topic]["publishers"].append(node_name)
            if role in ("sub", "pubsub"):
                topic_roles[topic]["subscribers"].append(node_name)
            endpoints.append({
                "node": node_name,
                "topic": topic,
                "role": role,
                "type_id": t.get("type_id", s.get("type_id", "0x00000000")),
                "freq": float(t.get("freq", s.get("freq", 0)) or 0),
                "pub": s.get("pub", s.get("publish_count", 0)),
                "del": s.get("del", s.get("deliver_count", 0)),
                "drop": s.get("drop", s.get("drop_count", 0)),
                "lat_us": s.get("lat_us", 0),
                "subs": s.get("subs", s.get("subscriber_count", 0)),
            })

    metrics = data.get("metrics") or {}
    if "topics" not in metrics:
        metrics["topics"] = data.get("topics", []) or []

    data["nodes"] = nodes
    data["metrics"] = metrics
    data["topic_roles"] = list(topic_roles.values())
    data["endpoints"] = endpoints
    data["timestamp"] = time.time()
    data["source"] = "live"
    return data
