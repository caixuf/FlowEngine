#!/usr/bin/env python3
"""
FlowBoard Server — 实时监控桥接

连接 FlowEngine 进程 → 提供 HTTP/SSE 实时数据给浏览器仪表盘。

用法:
  python3 tools/flowboard_server.py [--port 8800] [--json-file /tmp/flow_topology.json]

FlowEngine 侧:
  在代码中定期调用 discovery_export_json() 并写入 json-file，
  或通过管道/信号触发更新。

无 FlowEngine 时：
  使用内置模拟数据演示仪表盘效果。
"""

import http.server
import socketserver
import json
import os
import sys
import time
import threading
import random
import math
import argparse
import logging
from pathlib import Path

# 归一化 + 契约校验从独立模块导入（便于单测；见 tools/flowboard_normalize.py）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flowboard_normalize import (  # noqa: E402
    normalize_live_data, validate_payload,
    _cap_role, _topic_type_map, _stats_map, _synth_nodes_from_registry,
)

log = logging.getLogger("flowboard")

# ── 全局状态 ──────────────────────────────────────────────
g_data = {"nodes": [], "metrics": {}, "timestamp": 0, "endpoints": []}
g_lock = threading.Lock()
g_json_file = None
g_simulate = True
g_last_update = 0.0          # wall-clock time the state file was last successfully read
g_read_failures = 0          # cumulative state-file read failures (corrupt/partial JSON, IO errors)
STALE_AFTER_SEC = 5.0        # data older than this is considered stale (e2e stopped/stuck)
DEFAULT_STATE_FILE = os.environ.get("FLOWENGINE_STATE_FILE", "/tmp/flow_topology.json")


def _is_stale():
    """True when live data hasn't refreshed within STALE_AFTER_SEC.

    NOTE: callers must already hold g_lock (this reads g_simulate/g_last_update
    without locking to avoid re-entrant deadlock on the non-reentrant g_lock).
    """
    return (not g_simulate) and (time.time() - g_last_update > STALE_AFTER_SEC)


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """One worker thread per connection.

    Without this, the single-threaded HTTPServer serves requests serially, so a
    long-lived /api/stream SSE connection blocks every other request (topology
    snapshot, reconnects, extra tabs) → the dashboard flips to '● offline' and
    silently falls back to fake data. This is the root cause of the recurring
    "visualization offline" problem.
    """
    daemon_threads = True   # don't block process exit on active SSE streams
    allow_reuse_address = True

# ── 模拟数据生成（无 FlowEngine 时演示用）─────────────────

def generate_sample_data():
    t = time.time()
    nodes = [
        {"name": "perception", "pid": 1001, "alive": True, "caps": 1,
         "topics": [{"topic": "sensor/lidar", "freq": 10.0, "type_id": "0xd712aa51"},
                    {"topic": "sensor/gps", "freq": 5.0, "type_id": "0x0596b0b7"}]},
        {"name": "radar", "pid": 1005, "alive": True, "caps": 1,
         "topics": [{"topic": "sensor/radar", "freq": 20.0, "type_id": "0xaabbccdd"}]},
        {"name": "fusion", "pid": 1002, "alive": True, "caps": 9,
         "topics": [{"topic": "sensor/lidar", "freq": 0},
                    {"topic": "sensor/gps", "freq": 0},
                    {"topic": "sensor/radar", "freq": 0},
                    {"topic": "fusion/objects", "freq": 10.0, "type_id": "0xf0ed10c0"}]},
        {"name": "control", "pid": 1003, "alive": True, "caps": 2,
         "topics": [{"topic": "fusion/objects", "freq": 0},
                    {"topic": "control/cmd", "freq": 0, "type_id": "0x2d95c6d2"}]},
        {"name": "planning", "pid": 1004, "alive": t % 20 > 15, "caps": 2,
         "topics": [{"topic": "fusion/objects", "freq": 0}]},
    ]

    # Simulate some metrics
    metrics = {
        "bus": {"published": int(1000 + t * 50) % 10000, "delivered": int(950 + t * 48) % 10000, "dropped": 0},
        "transport": {"local_pub": int(500 + t * 25) % 5000, "remote_pub": int(50 + t * 2) % 200},
        "scheduler": {"tasks": 5, "mode": "CHOREO"},
        "latency": {"avg_us": 145, "p50_us": 120, "p99_us": 450 + int(50 * math.sin(t * 0.1))},
        "driver_mode": "ACC" if t % 30 < 20 else "CP",
        "vehicle": {
            "speed": 8.0 + 2 * math.sin(t * 0.3),
            "target_speed": 10.0,
            "throttle": 0.3 + 0.1 * math.sin(t * 0.5),
            "brake": 0.0,
            "x": t * 5 % 200,
            "y": 2 * math.sin(t * 0.2),
            "error": 1.0 + math.sin(t * 0.3)
        },
    }

    return {"nodes": nodes, "metrics": metrics, "timestamp": t}

# ── 文件监视线程 ──────────────────────────────────────────

def file_watcher():
    global g_data, g_simulate, g_last_update, g_read_failures
    last_mtime = 0
    while True:
        try:
            if g_json_file and os.path.exists(g_json_file):
                mtime = os.path.getmtime(g_json_file)
                if mtime != last_mtime:
                    last_mtime = mtime
                    with open(g_json_file) as f:
                        raw = json.load(f)
                    # Contract check before it reaches the renderer. On failure we
                    # KEEP the previous good frame (do not clobber g_data) so a
                    # single malformed write can't blank the dashboard.
                    ok, reason = validate_payload(raw)
                    if not ok:
                        g_read_failures += 1
                        log.warning("state file failed contract check (%s) — keeping last frame", reason)
                        continue
                    normalized = normalize_live_data(raw)
                    with g_lock:
                        g_data = normalized
                        g_simulate = False
                        g_last_update = time.time()
        except json.JSONDecodeError:
            # File being written (non-atomic writer) — retain last frame, count it.
            g_read_failures += 1
            log.debug("state file partial/corrupt JSON — retrying next cycle")
        except Exception as e:
            g_read_failures += 1
            log.warning("watcher error: %s — keeping last frame", e)
        time.sleep(0.1)  # 10 Hz — matches monitor task write rate

# ── HTTP 请求处理器 ───────────────────────────────────────

class FlowBoardHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            self._handle()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _snapshot(self):
        """Return (data, source) under lock. source ∈ live|stale|demo."""
        with g_lock:
            if g_simulate:
                data = generate_sample_data()
                data["source"] = "demo"
                data["stale"] = False
                return data, "demo"
            data = dict(g_data)
            source = "stale" if _is_stale() else "live"
            data["source"] = source
            data["stale"] = (source == "stale")
            data["age_sec"] = round(time.time() - g_last_update, 1)
            return data, source

    def _handle(self):
        if self.path == '/api/health':
            with g_lock:
                source = "demo" if g_simulate else ("stale" if _is_stale() else "live")
                age = None if g_simulate else round(time.time() - g_last_update, 1)
                read_failures = g_read_failures
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "ok", "source": source,
                "age_sec": age, "read_failures": read_failures,
            }).encode())

        elif self.path == '/api/topology':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()

            data, _ = self._snapshot()
            self.wfile.write(json.dumps(data).encode())

        elif self.path == '/api/stream':
            # SSE (Server-Sent Events) for real-time updates.
            # NOTE: this handler blocks its worker thread for the whole stream;
            # the server MUST be threaded (ThreadingHTTPServer) so that other
            # requests (/api/topology, reconnects, extra tabs) are not starved.
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()

            try:
                deadline = time.time() + 300  # wall-clock cap: 5 minutes
                last_beat = 0.0
                while time.time() < deadline:
                    data, _ = self._snapshot()
                    self.wfile.write(f"data: {json.dumps(data)}\n\n".encode())
                    # Periodic comment frame keeps proxies/clients from timing out.
                    now = time.time()
                    if now - last_beat > 15:
                        self.wfile.write(b": keep-alive\n\n")
                        last_beat = now
                    self.wfile.flush()
                    time.sleep(0.1)  # 10 Hz — matches monitor task write rate
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected

        elif self.path == '/' or self.path == '/index.html':
            # Serve the dashboard HTML
            html_path = os.path.join(os.path.dirname(__file__), 'flowboard.html')
            if os.path.exists(html_path):
                with open(html_path) as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(content.encode())
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"flowboard.html not found")

        elif self.path.startswith('/tools/') or self.path.startswith('/static/'):
            # Serve static assets (Three.js, CSS, etc.) from the tools/ directory.
            # Strip leading /tools/ or /static/ to get the relative filename.
            rel = self.path.lstrip('/')
            # /tools/three.min.js → tools/three.min.js
            # /static/foo.js       → tools/foo.js (mapped to tools/ for simplicity)
            if rel.startswith('static/'):
                rel = 'tools/' + rel[len('static/'):]
            fpath = os.path.join(os.path.dirname(__file__), '..', rel)
            fpath = os.path.normpath(fpath)
            # Security: only serve files directly inside tools/
            tools_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'tools'))
            if not fpath.startswith(tools_dir):
                self.send_response(403)
                self.end_headers()
                self.wfile.write(b"Forbidden")
                return
            if not os.path.isfile(fpath):
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"Not found")
                return
            # Guess MIME type
            ct = "application/octet-stream"
            if fpath.endswith('.js'):   ct = 'application/javascript'
            elif fpath.endswith('.css'): ct = 'text/css'
            elif fpath.endswith('.html'): ct = 'text/html'
            elif fpath.endswith('.svg'): ct = 'image/svg+xml'
            with open(fpath, 'rb') as f:
                data = f.read()
            self.send_response(200)
            self.send_header('Content-Type', ct)
            self.send_header('Content-Length', str(len(data)))
            # Cache static assets for 1 hour (they don't change at runtime)
            self.send_header('Cache-Control', 'public, max-age=3600')
            self.end_headers()
            self.wfile.write(data)

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Route HTTP access logs through the logger at DEBUG so they are silent
        # by default but available with --log-level debug.
        log.debug("%s - %s", self.address_string(), format % args)

# ── Main ──────────────────────────────────────────────────

def main():
    global g_json_file, g_simulate, g_last_update, g_data

    parser = argparse.ArgumentParser(description='FlowBoard Server')
    parser.add_argument('--port', type=int, default=8800)
    parser.add_argument('--json-file', type=str, default=DEFAULT_STATE_FILE,
                        help=f'Path to live state JSON file (default: {DEFAULT_STATE_FILE})')
    parser.add_argument('--demo', action='store_true', help='Force built-in demo data instead of live state file')
    parser.add_argument('--log-level', default='info',
                        choices=['debug', 'info', 'warning', 'error'],
                        help='Logging verbosity (default: info)')
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format='%(asctime)s %(levelname)-7s [%(name)s] %(message)s',
        stream=sys.stderr,
    )

    g_json_file = args.json_file
    g_simulate = bool(args.demo)
    if g_json_file and not args.demo:
        # Load initial data if file already exists (avoids 1s cold-start gap)
        if os.path.exists(g_json_file):
            try:
                with open(g_json_file) as f:
                    raw = json.load(f)
                g_data = normalize_live_data(raw)
                g_simulate = False
                g_last_update = time.time()
            except Exception as e:
                log.warning("initial state load failed (%s) — starting in demo mode", e)
                g_simulate = True  # fallback to simulation on parse error
        else:
            g_simulate = True
        threading.Thread(target=file_watcher, daemon=True).start()

    # Threaded server: one worker thread per connection so a long-lived SSE
    # stream can never starve /api/topology, reconnects, or extra browser tabs.
    try:
        server = ThreadingHTTPServer(('0.0.0.0', args.port), FlowBoardHandler)
    except OSError as e:
        log.error("cannot bind port %d: %s", args.port, e)
        log.error("Is another FlowBoard server already running? "
                  "Try: `ss -tlnp 'sport = :%d'` and kill the stale PID.", args.port)
        sys.exit(1)

    log.info("FlowBoard server on http://localhost:%d  (API: /api/topology /api/stream /api/health)", args.port)
    if g_json_file and not args.demo:
        log.info("Watching state file: %s", g_json_file)
    else:
        log.info("Mode: simulation (demo data)")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down...")
        server.shutdown()

if __name__ == '__main__':
    main()
