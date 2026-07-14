"""Phase 3c: contract consistency tests.

A single schema (tools/schema/flowboard.schema.json) must validate:
  - the Python server's built-in sample/demo data,
  - the frontend demo data embedded in flowboard/js/app.js,
  - the normalized /api/topology output.

This catches any single end silently changing the payload format.
"""
import json
import os
import re

import pytest

jsonschema = pytest.importorskip("jsonschema")

import flowboard_server as S
import flowboard_normalize as N

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SCHEMA_PATH = os.path.join(ROOT, "tools", "schema", "flowboard.schema.json")
APP_JS_PATH = os.path.join(ROOT, "tools", "flowboard", "js", "app.js")


@pytest.fixture(scope="module")
def schema():
    with open(SCHEMA_PATH) as f:
        return json.load(f)


def test_schema_itself_is_valid(schema):
    # Raises SchemaError if the schema is malformed.
    jsonschema.Draft7Validator.check_schema(schema)


def test_sample_data_matches_schema(schema):
    data = S.generate_sample_data()
    jsonschema.validate(instance=data, schema=schema)


def test_normalized_sample_matches_schema(schema):
    normalized = N.normalize_live_data(S.generate_sample_data())
    jsonschema.validate(instance=normalized, schema=schema)


def test_monitor_style_payload_normalizes_and_matches_schema(schema):
    raw = {
        "self": "flow_launcher",
        "nodes": [
            {"name": "perception", "caps": 1,
             "topics": [{"topic": "sensor/lidar", "freq": 10.0, "type_id": "0xd712aa51"}]},
            {"name": "fusion", "caps": 2,
             "topics": [{"topic": "sensor/lidar", "freq": 0}]},
        ],
        "metrics": {
            "bus": {"published": 100, "delivered": 200, "dropped": 0},
            "latency": {"avg_us": 145, "p50_us": 120, "p99_us": 450},
            "vehicle": {"speed": 8.0, "target_speed": 10.0, "throttle": 0.3,
                        "brake": 0.0, "x": 5.0, "error": 2.0},
            "sysmon": {"cpu_total_pct": 12.0, "mem_used_pct": 40.0, "cpu_count": 4,
                       "thread_count": 2, "threads": [
                           {"tid": 1, "name": "monitor", "cpu_pct": 1.0, "state": "R"}]},
        },
    }
    normalized = N.normalize_live_data(raw)
    jsonschema.validate(instance=normalized, schema=schema)


def _extract_frontend_demo(js_source):
    """Pull the doSimulate() topoData object literal out of flowboard/js/app.js and
    parse the endpoints array (a strict-JSON-compatible subset) to validate
    against the schema. We validate the endpoints array specifically because it
    is pure JSON (no JS expressions)."""
    m = re.search(r"endpoints:\s*(\[[\s\S]*?\])\s*\n\s*\};", js_source)
    assert m, "could not locate frontend demo endpoints array"
    raw = m.group(1)
    # keys are unquoted JS identifiers → quote them for json.loads
    raw = re.sub(r"([{,]\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:", r'\1"\2":', raw)
    return json.loads(raw)


def test_frontend_demo_endpoints_match_schema(schema):
    with open(APP_JS_PATH) as f:
        js_source = f.read()
    endpoints = _extract_frontend_demo(js_source)
    payload = {"endpoints": endpoints}
    jsonschema.validate(instance=payload, schema=schema)
    # each endpoint must carry the required contract fields
    for e in endpoints:
        assert {"node", "topic", "role"} <= set(e.keys())
        assert e["role"] in ("pub", "sub", "pubsub", "unknown")
