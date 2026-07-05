#!/bin/bash
# =============================================================================
# FlowEngine Demo — 一键演示脚本
#
# 从零到全链路运行，自动打开可视化仪表盘。
#
# 用法:
#   bash scripts/demo.sh              # 默认 15 秒演示
#   bash scripts/demo.sh 30           # 30 秒演示
#   bash scripts/demo.sh --no-browser # 不打开浏览器
# =============================================================================
set -e

# Kill any stale processes from previous runs
# Aggressive cleanup
{ pkill -9 -f flowboard; pkill -9 -f flow_e2e; pkill -9 -f foxglove; } 2>/dev/null || true
sleep 1
# Force-free ports
for port in 8800 8765; do
  pid=$(ss -tlnp "sport = :$port" 2>/dev/null | grep -oP 'pid=\K\d+' | head -1)
  [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
done
sleep 0.5

DURATION=30
OPEN_BROWSER=true
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build"
E2E_BIN="$BUILD_DIR/bin/flow_e2e"
JSON_FILE="/tmp/flow_topology.json"

for arg in "$@"; do
  case "$arg" in
    --no-browser) OPEN_BROWSER=false ;;
    ''|*[!0-9]*) ;;
    *) DURATION="$arg" ;;
  esac
done

# ── Banner ──────────────────────────────────────────────────
clear
cat << 'BANNER'

  ╔══════════════════════════════════════════════════════════╗
  ║                                                          ║
  ║   ███████╗██╗      ██████╗ ██╗    ██╗                  ║
  ║   ██╔════╝██║     ██╔═══██╗██║    ██║                  ║
  ║   █████╗  ██║     ██║   ██║██║ █╗ ██║                  ║
  ║   ██╔══╝  ██║     ██║   ██║██║███╗██║                  ║
  ║   ██║     ███████╗╚██████╔╝╚███╔███╔╝                  ║
  ║   ╚═╝     ╚══════╝ ╚═════╝  ╚══╝╚══╝                   ║
  ║                                                          ║
  ║   E N G I N E                                           ║
  ║   Lightweight Middleware for Autonomous Driving          ║
  ║                                                          ║
  ╚══════════════════════════════════════════════════════════╝

BANNER

echo "   Demo Duration: ${DURATION}s"
echo ""

# ── Build ───────────────────────────────────────────────────
echo "───[1/5] Building..."
if [ ! -f "$E2E_BIN" ]; then
  echo "  First build, this may take a moment..."
  cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
fi
cmake --build "$BUILD_DIR" --target flow_e2e -j$(nproc) 2>/dev/null | tail -1
echo "  ✓ Build complete"

# ── Cleanup handler ─────────────────────────────────────────
cleanup() {
  echo ""
  echo "───[Cleanup] Shutting down..."
  kill $E2E_PID $SERVER_PID $BRIDGE_PID 2>/dev/null
  wait 2>/dev/null
  rm -f "$JSON_FILE"

  echo ""
  echo "  ╔══════════════════════════════════════╗"
  echo "  ║  Demo Complete — FlowEngine v1.0     ║"
  echo "  ║  github.com/caixuf/FlowEngine        ║"
  echo "  ╚══════════════════════════════════════╝"
  exit 0
}
trap cleanup EXIT INT TERM

# ── Start e2e pipeline ──────────────────────────────────────
echo "───[2/5] Starting E2E pipeline (perception→fusion→control)..."
rm -f "$JSON_FILE"
"$E2E_BIN" "$DURATION" > /tmp/flow_e2e_stdout.txt 2>/tmp/flow_e2e_stderr.txt &
E2E_PID=$!
sleep 1
echo "  ✓ Pipeline running (PID $E2E_PID)"

# ── Start dashboard server ──────────────────────────────────
echo "───[3/5] Starting dashboard server..."
python3 "$ROOT/tools/flowboard_server.py" --json-file "$JSON_FILE" --port 8800 > /tmp/flowboard.log 2>&1 &
SERVER_PID=$!
sleep 2
if kill -0 $SERVER_PID 2>/dev/null; then
    echo "  ✓ Dashboard at http://localhost:8800"
else
    echo "  ✗ Server failed! Check /tmp/flowboard.log"
    cat /tmp/flowboard.log
fi

# Start Foxglove WebSocket bridge for 3D live viz
python3 "$ROOT/tools/foxglove_bridge.py" --port 8765 --json-file "$JSON_FILE" > /tmp/foxglove_bridge.log 2>&1 &
BRIDGE_PID=$!
echo "  ✓ 3D Bridge at ws://localhost:8765 (Foxglove Studio)"

# ── Open browser ────────────────────────────────────────────
if $OPEN_BROWSER; then
  echo "───[4/5] Opening browser..."
  if command -v xdg-open &>/dev/null; then
    xdg-open http://localhost:8800 2>/dev/null || true
  elif command -v open &>/dev/null; then
    open http://localhost:8800 2>/dev/null || true
  fi
  echo "  ✓ Browser opened"
else
  echo "───[4/5] Skipping browser (--no-browser)"
fi

# ── Live monitor ────────────────────────────────────────────
echo "───[5/5] Live monitor (${DURATION}s)..."
echo ""
echo "  ┌─ Perception ──→  Fusion  ──→  Control  ──→  Monitor ─┐"
echo "  │  10Hz LiDAR      time-align    decisions     stats     │"
echo "  │  5Hz GPS         50ms window   CRUISE/BRAKE  1Hz      │"
echo "  └───────────────────────────────────────────────────────┘"
echo ""

ELAPSED=0
while [ $ELAPSED -lt $DURATION ]; do
  if [ -f "$JSON_FILE" ]; then
    STATS=$(python3 -c "
import json
with open('$JSON_FILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
print(f\"pub={b.get('published',0)} del={b.get('delivered',0)} lat={l.get('avg_us',0)}us p99={l.get('p99_us',0)}us\")
" 2>/dev/null)
    printf "\r  ⏱ %3ds  |  %s  " "$ELAPSED" "$STATS"
  else
    printf "\r  ⏱ %3ds  |  waiting for data..." "$ELAPSED"
  fi
  sleep 1
  ELAPSED=$((ELAPSED + 1))
done
echo ""
echo ""

# ── Print summary ───────────────────────────────────────────
echo "═══ Pipeline Summary ═══"

# Extract stats from e2e stderr
# The e2e logger writes with NEL line terminators that break plain sed/grep.
# Use grep -a (force binary/text mode) with a pipe to wc for robust counting.
FUSED=$(grep -a "fusion.*#" /tmp/flow_e2e_stderr.txt 2>/dev/null | wc -l)
CTRL=$(grep -a "control.*#" /tmp/flow_e2e_stderr.txt 2>/dev/null | wc -l)
PERCEPT=$(grep -a "stopped.*frames" /tmp/flow_e2e_stderr.txt 2>/dev/null | grep -oa "[0-9]* frames" || echo "0 frames")
LATENCY=$(grep -a "Fusion Lat" /tmp/flow_e2e_stderr.txt -A2 2>/dev/null | tail -1 | xargs || echo "N/A")
MODE=$(grep -a "driving mode" /tmp/flow_e2e_stderr.txt 2>/dev/null | tail -1 | grep -oa "ACC\|CP\|NP\|NOA\|NA" || echo "N/A")

echo "  Perception : $PERCEPT"
echo "  Fusion     : $FUSED aligned frames"
echo "  Control    : $CTRL decisions"
echo "  Drive Mode : $MODE"
echo "  Latency    : $LATENCY"

TESTS=$(ctest --test-dir "$BUILD_DIR" 2>/dev/null | tail -1 || echo "N/A")
echo "  Tests      : $TESTS"

echo ""
echo "  Dashboard  : http://localhost:8800"
echo "  Topology   : tools/topology_viewer.html (standalone)"
echo "  CI Status  : github.com/caixuf/FlowEngine/actions"
