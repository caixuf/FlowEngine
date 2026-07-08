#!/bin/bash
# =============================================================================
# FlowEngine Demo — 一键演示脚本 (v2 — 配置驱动插件架构)
#
# 使用 flow_launcher + pipeline.json 启动全链路节点插件。
# 自动打开可视化仪表盘。
#
# 用法:
#   bash scripts/demo.sh              # 默认 15 秒，dlopen 单进程模式
#   bash scripts/demo.sh 30           # 30 秒演示
#   bash scripts/demo.sh --multi      # fork+exec 多进程模式（各节点独立 PID）
#   bash scripts/demo.sh --no-browser # 不打开浏览器
# =============================================================================
set -e

# Kill any stale processes from previous runs
{ pkill -9 -f flowboard; pkill -9 -f flow_launcher; pkill -9 -f foxglove; } 2>/dev/null || true
sleep 1
for port in 8800 8765; do
  pid=$(ss -tlnp "sport = :$port" 2>/dev/null | grep -oP 'pid=\K\d+' | head -1)
  [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
done
sleep 0.5

DURATION=0  # 0 = 持续运行直到 Ctrl+C，设置正数 = 限时运行秒数
OPEN_BROWSER=true
MULTI_MODE=false
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build"
LAUNCHER_BIN="$BUILD_DIR/bin/flow_launcher"
PIPELINE="$ROOT/config/pipeline.json"
JSON_FILE="/tmp/flow_topology.json"

for arg in "$@"; do
  case "$arg" in
    --no-browser) OPEN_BROWSER=false ;;
    --multi) MULTI_MODE=true ;;
    ''|*[!0-9]*) ;;
    *) DURATION="$arg" ;;
  esac
  shift 2>/dev/null || true
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

   echo "Demo Duration: ${DURATION}s   Mode: $([ "$MULTI_MODE" = true ] && echo "Multi-Process" || echo "Single-Process (dlopen)")"
echo ""

# ── Build ───────────────────────────────────────────────────
echo "───[1/5] Building..."
if [ ! -f "$LAUNCHER_BIN" ]; then
  echo "  First build, this may take a moment..."
  cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc > /dev/null 2>&1
fi
cmake --build "$BUILD_DIR" --target flow_launcher -j$(nproc) 2>/dev/null | tail -1
# Also build node plugins. They live in a separate CMake project, so the main
# flow_launcher target does not automatically rebuild them after node source edits.
echo "  Building node plugins..."
cmake -B "$BUILD_DIR/modules/adas_nodes" -S "$ROOT/modules/adas_nodes" \
  -DFLOWENGINE_BUILD="$BUILD_DIR" > /dev/null 2>&1
cmake --build "$BUILD_DIR/modules/adas_nodes" -j$(nproc) 2>/dev/null | tail -1
echo "  ✓ Build complete"

# ── Cleanup handler ─────────────────────────────────────────
cleanup() {
  echo ""
  echo "───[Cleanup] Shutting down..."
  kill $LAUNCHER_PID $BRIDGE_PID $FLOWMOND_PID 2>/dev/null
  wait 2>/dev/null
  rm -f "$JSON_FILE"

  echo ""
  echo "  ╔══════════════════════════════════════╗"
  echo "  ║  Demo Complete — FlowEngine v2.0     ║"
  echo "  ║  Plugin Architecture                 ║"
  echo "  ║  github.com/caixuf/FlowEngine        ║"
  echo "  ╚══════════════════════════════════════╝"
  exit 0
}
trap cleanup EXIT INT TERM

# ── Start flow_launcher with pipeline ───────────────────────
echo "───[2/5] Starting pipeline (sim_world→sensor_model→perception→fusion→planning→control→monitor)..."
rm -f "$JSON_FILE"
cd "$ROOT"  # run from root so build/lib/ paths resolve
if [ "$MULTI_MODE" = true ]; then
  echo "  Multi-process mode: each node runs as a separate process"
  if [ "$DURATION" -gt 0 ] 2>/dev/null; then
    "$LAUNCHER_BIN" "$PIPELINE" --multi --duration "$DURATION" \
      > /tmp/flow_launcher_stdout.txt 2>/tmp/flow_launcher_stderr.txt &
  else
    "$LAUNCHER_BIN" "$PIPELINE" --multi \
      > /tmp/flow_launcher_stdout.txt 2>/tmp/flow_launcher_stderr.txt &
  fi
else
  if [ "$DURATION" -gt 0 ] 2>/dev/null; then
    "$LAUNCHER_BIN" "$PIPELINE" --duration "$DURATION" \
      > /tmp/flow_launcher_stdout.txt 2>/tmp/flow_launcher_stderr.txt &
  else
    "$LAUNCHER_BIN" "$PIPELINE" \
      > /tmp/flow_launcher_stdout.txt 2>/tmp/flow_launcher_stderr.txt &
  fi
fi
LAUNCHER_PID=$!
sleep 1
if ! kill -0 $LAUNCHER_PID 2>/dev/null; then
  echo "  ✗ Pipeline failed! Check /tmp/flow_launcher_stderr.txt"
  cat /tmp/flow_launcher_stderr.txt 2>/dev/null || true
  exit 1
fi
echo "  ✓ Pipeline running (PID $LAUNCHER_PID)"

# ── Start dashboard server (flowmond) ──────────────────────
echo "───[3/5] Starting dashboard server..."
"$BUILD_DIR/bin/flowmond" --port 8800 --html-path "$ROOT/tools/flowboard.html" \
  > /tmp/flowmond.log 2>&1 &
FLOWMOND_PID=$!
sleep 2
if kill -0 $FLOWMOND_PID 2>/dev/null; then
    echo "  ✓ Dashboard at http://localhost:8800"
else
    echo "  ✗ flowmond failed! Check /tmp/flowmond.log"
    cat /tmp/flowmond.log
fi

python3 "$ROOT/tools/foxglove_bridge.py" --port 8765 --json-file "$JSON_FILE" \
  > /tmp/foxglove_bridge.log 2>&1 &
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
echo "  ┌─ SimWorld ─→  Perception ─→  Fusion  ─→  Planning ─→  Control ┐"
echo "  │  dynamics      DBSCAN          EKF          Frenet       PID      │"
echo "  └───────────────────────────────────────────────────────────────────┘"
echo ""

ELAPSED=0
# 持续运行: DURATION=0 时无限循环，直到脚本被 Ctrl+C 终止
while true; do
  # 限时模式: 达到时长后退出循环
  if [ "$DURATION" -gt 0 ] 2>/dev/null && [ $ELAPSED -ge $DURATION ]; then break; fi
  if [ -f "$JSON_FILE" ]; then
    STATS=$(python3 -c "
import json
with open('$JSON_FILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
v=m.get('vehicle',{})
print(f\"pub={b.get('published',0)} del={b.get('delivered',0)} lat={l.get('avg_us',0)}us speed={v.get('speed',0):.1f}m/s\")
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

# Extract stats from launcher stderr
# Build node plugins if stderr log is available
FUSED=$(grep -a "EKF" /tmp/flow_launcher_stderr.txt 2>/dev/null | tail -1 | grep -oP "#\K\d+" | tail -1 || echo "0")
CTRL=$(grep -a "control.*#" /tmp/flow_launcher_stderr.txt 2>/dev/null | tail -1 | grep -oP "#\K\d+" | tail -1 || echo "0")
PLAN=$(grep -a "planning.*#" /tmp/flow_launcher_stderr.txt 2>/dev/null | tail -1 | grep -oP "#\K\d+" | tail -1 || echo "0")
SPEED=$(grep -a "sim_world.*stopped" /tmp/flow_launcher_stderr.txt 2>/dev/null | grep -oP "speed=\K[0-9.]+" || echo "?")

echo "  Simulation : $SPEED m/s final speed"
echo "  Fusion     : $FUSED EKF frames"
echo "  Planning   : $PLAN trajectories"
echo "  Control    : $CTRL control cycles"

echo ""
echo "  Dashboard  : http://localhost:8800"
echo "  Topology   : tools/topology_viewer.html (standalone)"
echo "  CI Status  : github.com/caixuf/FlowEngine/actions"
