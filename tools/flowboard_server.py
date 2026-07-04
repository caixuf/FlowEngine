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
import json
import os
import sys
import time
import threading
import random
import math
import argparse
from pathlib import Path

# ── 全局状态 ──────────────────────────────────────────────
g_data = {"nodes": [], "metrics": {}, "timestamp": 0}
g_lock = threading.Lock()
g_json_file = None
g_simulate = True

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
    }

    return {"nodes": nodes, "metrics": metrics, "timestamp": t}

# ── 文件监视线程 ──────────────────────────────────────────

def file_watcher():
    global g_data
    last_mtime = 0
    while True:
        try:
            if g_json_file and os.path.exists(g_json_file):
                mtime = os.path.getmtime(g_json_file)
                if mtime != last_mtime:
                    last_mtime = mtime
                    with open(g_json_file) as f:
                        raw = json.load(f)
                    with g_lock:
                        # If it has 'nodes', use as-is; otherwise wrap
                        if isinstance(raw, dict) and 'nodes' in raw:
                            g_data = raw
                        else:
                            g_data = {"nodes": raw if isinstance(raw, list) else [],
                                      "metrics": {}, "timestamp": time.time()}
                        g_data["timestamp"] = time.time()
                        g_simulate = False
        except Exception as e:
            pass
        time.sleep(1)

# ── HTTP 请求处理器 ───────────────────────────────────────

class FlowBoardHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/topology':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()

            with g_lock:
                if g_simulate:
                    data = generate_sample_data()
                else:
                    data = dict(g_data)

            self.wfile.write(json.dumps(data).encode())

        elif self.path == '/api/stream':
            # SSE (Server-Sent Events) for real-time updates
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()

            for _ in range(300):  # 5 minutes max
                with g_lock:
                    if g_simulate:
                        data = generate_sample_data()
                    else:
                        data = dict(g_data)
                self.wfile.write(f"data: {json.dumps(data)}\n\n".encode())
                self.wfile.flush()
                time.sleep(1)

        elif self.path == '/' or self.path == '/index.html':
            # Serve the dashboard HTML
            html_path = os.path.join(os.path.dirname(__file__), 'flowboard.html')
            if os.path.exists(html_path):
                with open(html_path) as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(content.encode())
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"flowboard.html not found")

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # quiet

# ── Main ──────────────────────────────────────────────────

def main():
    global g_json_file, g_simulate

    parser = argparse.ArgumentParser(description='FlowBoard Server')
    parser.add_argument('--port', type=int, default=8800)
    parser.add_argument('--json-file', type=str, help='Path to discovery JSON file (polled)')
    args = parser.parse_args()

    g_json_file = args.json_file
    if g_json_file:
        g_simulate = False
        threading.Thread(target=file_watcher, daemon=True).start()

    server = http.server.HTTPServer(('0.0.0.0', args.port), FlowBoardHandler)

    print(f"╔══════════════════════════════════════╗")
    print(f"║  FlowBoard Server                    ║")
    print(f"║  http://localhost:{args.port}           ║")
    print(f"║  API: /api/topology  /api/stream    ║")
    if g_json_file:
        print(f"║  Watching: {g_json_file}")
    else:
        print(f"║  Mode: simulation (demo data)")
    print(f"╚══════════════════════════════════════╝")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()

if __name__ == '__main__':
    main()
