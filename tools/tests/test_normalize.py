"""Phase 3a: unit tests for flowboard_normalize pure functions.

Covers the three input formats (monitor_server / discovery / NodePlugin) plus
missing/dirty-field robustness. This is the core regression guard for the
data contract.
"""
import pytest

import flowboard_normalize as N


# ── pure helpers ──────────────────────────────────────────

@pytest.mark.parametrize("caps,expected", [
    (0, "unknown"),
    (1, "pub"),
    (2, "sub"),
    (3, "pubsub"),
    ("1", "pub"),
    ("bad", "unknown"),
    (None, "unknown"),
])
def test_cap_role(caps, expected):
    assert N._cap_role(caps) == expected


def test_topic_type_map():
    reg = {"topics": [{"name": "a", "type_id": "0x1"}, {"topic": "b", "type_id": "0x2"}]}
    m = N._topic_type_map(reg)
    assert m == {"a": "0x1", "b": "0x2"}


def test_topic_type_map_empty():
    assert N._topic_type_map(None) == {}
    assert N._topic_type_map({}) == {}


def test_stats_map_merges_metrics_and_toplevel():
    raw = {
        "metrics": {"topics": [{"topic": "a", "freq": 1}]},
        "topics": [{"name": "b", "freq": 2}],
    }
    s = N._stats_map(raw)
    assert set(s.keys()) == {"a", "b"}
    assert s["b"]["freq"] == 2


# ── validation ────────────────────────────────────────────

def test_validate_rejects_non_dict():
    ok, reason = N.validate_payload([1, 2, 3])
    assert not ok
    assert "object" in reason


def test_validate_rejects_wrong_container_type():
    ok, reason = N.validate_payload({"nodes": {"not": "a list"}})
    assert not ok
    assert "nodes" in reason


def test_validate_accepts_minimal():
    ok, _ = N.validate_payload({})
    assert ok
    ok, _ = N.validate_payload({"nodes": [], "metrics": {}})
    assert ok


def test_validate_hotpath_tolerates_offspec_field_types():
    # freq as a string is off-spec per schema but must NOT be rejected on the
    # hot path (would blank the dashboard). Structural check passes.
    ok, _ = N.validate_payload(
        {"nodes": [{"name": "n", "topics": [{"topic": "t", "freq": "3.5"}]}]}
    )
    assert ok


def test_validate_strict_rejects_offspec(schema_available):
    # In strict mode (tests/CI) the same off-spec freq is rejected.
    ok, reason = N.validate_payload(
        {"nodes": [{"name": "n", "topics": [{"topic": "t", "freq": "3.5"}]}]},
        strict=True,
    )
    assert not ok
    assert "schema" in reason


# ── normalize: degradation ────────────────────────────────

def test_normalize_non_dict_returns_empty_model():
    m = N.normalize_live_data("garbage")
    assert m["nodes"] == [] and m["endpoints"] == []


def test_normalize_invalid_payload_degrades_not_raises():
    m = N.normalize_live_data({"nodes": "oops"})
    assert m["nodes"] == []
    assert "invalid_reason" in m


# ── normalize: format 1 — discovery nodes ─────────────────

def test_normalize_discovery_format():
    raw = {
        "self": "flow",
        "nodes": [
            {"name": "perception", "caps": 1,
             "topics": [{"topic": "sensor/lidar", "caps": 1, "freq": 10.0}]},
            {"name": "fusion", "caps": 2,
             "topics": [{"topic": "sensor/lidar", "caps": 2, "freq": 0}]},
        ],
    }
    out = N.normalize_live_data(raw)
    assert out["source"] == "live"
    eps = {(e["node"], e["topic"]): e for e in out["endpoints"]}
    assert eps[("perception", "sensor/lidar")]["role"] == "pub"
    assert eps[("fusion", "sensor/lidar")]["role"] == "sub"
    roles = {r["topic"]: r for r in out["topic_roles"]}
    assert roles["sensor/lidar"]["publishers"] == ["perception"]
    assert roles["sensor/lidar"]["subscribers"] == ["fusion"]


# ── normalize: format 2 — NodePlugin inputs/outputs ───────

def test_normalize_nodeplugin_inputs_outputs():
    raw = {
        "nodes": [
            {"name": "planner", "inputs": ["fusion/obj"], "outputs": ["plan/traj"]},
        ],
    }
    out = N.normalize_live_data(raw)
    eps = {(e["node"], e["topic"]): e for e in out["endpoints"]}
    assert eps[("planner", "fusion/obj")]["role"] == "sub"
    assert eps[("planner", "plan/traj")]["role"] == "pub"


# ── normalize: format 3 — registry (synth nodes) ──────────

def test_normalize_registry_synthesizes_nodes():
    raw = {
        "nodes": [],
        "registry": {
            "tasks": [
                {"name": "control", "inputs": ["plan/traj"], "outputs": ["control/cmd"]},
            ],
            "topics": [{"name": "control/cmd", "type_id": "0xabc"}],
        },
    }
    out = N.normalize_live_data(raw)
    names = [n["name"] for n in out["nodes"]]
    assert "control" in names
    eps = {(e["node"], e["topic"]): e for e in out["endpoints"]}
    assert eps[("control", "control/cmd")]["role"] == "pub"
    assert eps[("control", "control/cmd")]["type_id"] == "0xabc"


# ── normalize: dirty / missing fields ─────────────────────

def test_normalize_node_without_topics():
    out = N.normalize_live_data({"nodes": [{"name": "lonely"}]})
    assert [n["name"] for n in out["nodes"]] == ["lonely"]
    assert out["endpoints"] == []


def test_normalize_topic_missing_name_is_skipped():
    out = N.normalize_live_data({"nodes": [{"name": "n", "topics": [{"freq": 5}]}]})
    # topic without topic/name key produces no endpoint
    assert out["endpoints"] == []


def test_normalize_freq_string_coerced():
    # freq-as-string is tolerated by the hot-path (structural) validation and
    # coerced to float during endpoint synthesis (role→pub via positive freq).
    out = N.normalize_live_data(
        {"nodes": [{"name": "n", "topics": [{"topic": "t", "freq": "3.5"}]}]}
    )
    assert out["endpoints"][0]["freq"] == 3.5
    assert out["endpoints"][0]["role"] == "pub"


def test_normalize_preserves_metrics_and_topics():
    raw = {"nodes": [], "metrics": {"bus": {"published": 10}}, "topics": [{"topic": "x"}]}
    out = N.normalize_live_data(raw)
    assert out["metrics"]["bus"]["published"] == 10
    assert out["metrics"]["topics"] == [{"topic": "x"}]


def test_normalize_monitor_server_topology_shape():
    # monitor_server.c style: {bus, topology:{nodes}, topics}
    raw = {
        "bus": {"published": 5, "delivered": 5, "dropped": 0},
        "topology": {"self": "m", "nodes": [
            {"name": "p", "caps": 1, "topics": [{"topic": "a", "freq": 1}]}]},
        "topics": [{"topic": "a", "freq": 1}],
    }
    out = N.normalize_live_data(raw)
    assert out["metrics"]["bus"]["published"] == 5
    assert [n["name"] for n in out["nodes"]] == ["p"]
