"""Phase 3a: integration tests for the FlowBoard HTTP server.

Starts the real ThreadingHTTPServer on an ephemeral port and exercises the
public endpoints, including the SSE stream (read one frame then disconnect)
and the static-file path-traversal guard.
"""
import json
import socket
import threading
import time
import http.client

import pytest

import flowboard_server as S


@pytest.fixture()
def demo_server():
    """Start the server in demo mode on a free port; yield the port."""
    # reset global state to a clean demo config
    S.g_simulate = True
    S.g_json_file = None
    S.g_read_failures = 0

    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()

    server = S.ThreadingHTTPServer(("127.0.0.1", port), S.FlowBoardHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    time.sleep(0.1)
    try:
        yield port
    finally:
        server.shutdown()
        server.server_close()


def _get(port, path):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", path)
    resp = conn.getresponse()
    body = resp.read()
    conn.close()
    return resp.status, body


def test_health_endpoint(demo_server):
    status, body = _get(demo_server, "/api/health")
    assert status == 200
    d = json.loads(body)
    assert d["status"] == "ok"
    assert d["source"] == "demo"
    assert "read_failures" in d


def test_topology_endpoint_demo_source(demo_server):
    status, body = _get(demo_server, "/api/topology")
    assert status == 200
    d = json.loads(body)
    assert d["source"] == "demo"
    assert isinstance(d.get("nodes"), list) and len(d["nodes"]) > 0


def test_index_served(demo_server):
    status, body = _get(demo_server, "/")
    assert status == 200
    assert b"FlowBoard" in body


def test_static_path_traversal_blocked(demo_server):
    # attempt to escape tools/ via ../
    status, _ = _get(demo_server, "/tools/../flowboard_server.py")
    assert status in (403, 404)


def test_static_serves_known_asset(demo_server):
    status, body = _get(demo_server, "/tools/d3.v7.min.js")
    assert status == 200
    assert len(body) > 0


def test_unknown_path_404(demo_server):
    status, _ = _get(demo_server, "/nope")
    assert status == 404


def test_sse_stream_one_frame(demo_server):
    conn = http.client.HTTPConnection("127.0.0.1", demo_server, timeout=5)
    conn.request("GET", "/api/stream")
    resp = conn.getresponse()
    assert resp.status == 200
    assert "text/event-stream" in resp.getheader("Content-Type", "")
    # read a small chunk containing at least one SSE 'data:' frame
    chunk = resp.read(512)
    conn.close()
    assert b"data:" in chunk


# ── _is_stale logic ───────────────────────────────────────

def test_is_stale_when_live_data_old(monkeypatch):
    S.g_simulate = False
    S.g_last_update = time.time() - (S.STALE_AFTER_SEC + 1)
    assert S._is_stale() is True


def test_is_not_stale_when_fresh():
    S.g_simulate = False
    S.g_last_update = time.time()
    assert S._is_stale() is False


def test_is_not_stale_in_demo():
    S.g_simulate = True
    S.g_last_update = 0
    assert S._is_stale() is False
