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
#   bash scripts/demo.sh --multi      # fork+exec 多进程模式
#   bash scripts/demo.sh --no-browser # 不打开浏览器
#   bash scripts/demo.sh --scenario scenarios/highway_overtake.json  # 指定场景
# =============================================================================
set -e

# Kill any stale processes from previous runs (node hosts + servers + bridges)
{ pkill -9 -f flowboard; pkill -9 -f flow_launcher; pkill -9 -f flow_node_host; \
  pkill -9 -f flowmond; pkill -9 -f foxglove_bridge; } 2>/dev/null || true
sleep 1
for port in 8800 8765; do
  pid=$(ss -tlnp "sport = :$port" 2>/dev/null | grep -oP 'pid=\K\d+' | head -1)
  [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
done
sleep 0.5

DURATION=0  # 0 = 持续运行直到 Ctrl+C，设置正数 = 限时运行秒数
OPEN_BROWSER=true
MULTI_MODE=false
RECORD_MODE=false
REPLAY_FILE=""
SCENARIO=""
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build"
LAUNCHER_BIN="$BUILD_DIR/bin/flow_launcher"
PIPELINE="${FLOW_PIPELINE:-$ROOT/config/pipeline.json}"
JSON_FILE="/tmp/flow_topology.json"
BAG_FILE="/tmp/flow_demo_$(date +%Y%m%d_%H%M%S).bag"

while [ $# -gt 0 ]; do
  case "$1" in
    --no-browser) OPEN_BROWSER=false ;;
    --multi) MULTI_MODE=true ;;
    --record) RECORD_MODE=true ;;
    --replay) REPLAY_FILE="$2"; shift ;;
    --scenario) SCENARIO="$2"; shift ;;
    ''|*[!0-9]*) ;;
    *) DURATION="$1" ;;
  esac
  shift
done

# ── Scenario override: patch pipeline.json's scenario_file ──
PIPELINE_ORIG="$ROOT/config/pipeline.json"
PIPELINE_TMP=""
cleanup_pipeline_tmp() {
  [ -n "$PIPELINE_TMP" ] && rm -f "$PIPELINE_TMP"
}
if [ -n "$SCENARIO" ]; then
  PIPELINE_TMP=$(mktemp /tmp/pipeline_XXXX.json)
  trap 'cleanup_pipeline_tmp' EXIT
  SCENARIO_ABS="$([ -f "$SCENARIO" ] && echo "$(cd "$(dirname "$SCENARIO")" && pwd)/$(basename "$SCENARIO")" || echo "$SCENARIO")"
  sed 's|\\"scenario_file\\": \\"[^\\"]*\\"|\\"scenario_file\\": \\"'$SCENARIO_ABS'\\"|g' "$PIPELINE_ORIG" > "$PIPELINE_TMP"
  PIPELINE="$PIPELINE_TMP"
  echo "  Scenario: $SCENARIO_ABS"
fi

# ── Replay fast path: no pipeline, just flowmond + flow_launcher --replay ──
if [ -n "$REPLAY_FILE" ]; then
  echo "═══ Replay Mode: $REPLAY_FILE ═══"
  "$BUILD_DIR/bin/flowmond" --port 8800 --html-path "$ROOT/tools/flowboard/index.html" > /tmp/flowmond.log 2>&1 &
  SERVER_PID=$!
  sleep 1
  echo "  Dashboard: http://localhost:8800"
  if $OPEN_BROWSER; then
    xdg-open http://localhost:8800 2>/dev/null || open http://localhost:8800 2>/dev/null || true
  fi
  "$BUILD_DIR/bin/flow_bag" --replay "$REPLAY_FILE" 2>&1
  kill $SERVER_PID 2>/dev/null
  exit 0
fi

# ── Banner ──────────────────────────────────────────────────
if [ -t 1 ] && [ -n "${TERM:-}" ] && [ "${TERM}" != "dumb" ] && command -v clear >/dev/null 2>&1; then
  clear
fi
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

   SCENARIO_DISPLAY="${SCENARIO:-$(grep -o 'scenarios/[^\"]*' "$PIPELINE_ORIG" | head -1)}"
   echo "Demo Duration: ${DURATION}s   Mode: $([ "$MULTI_MODE" = true ] && echo "Multi-Process" || echo "Single-Process (dlopen)")   Scenario: ${SCENARIO_DISPLAY:-default}"
echo ""

# ── Build ───────────────────────────────────────────────────
echo "───[1/5] Building..."
if [ ! -f "$LAUNCHER_BIN" ]; then
  echo "  First build, this may take a moment..."
  cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc > /dev/null 2>&1
fi
set -o pipefail  # scoped to this build block only — restored via `set +o pipefail`
                 # below once both `cmake --build ... | tail -1` calls are done, so
                 # a build failure surfaces (pipeline exit = cmake's, not tail's)
                 # without changing pipe-failure semantics for the rest of the script.
if ! cmake --build "$BUILD_DIR" --target flow_launcher flow_node_host -j$(nproc) 2>&1 | tail -1; then
  echo "  ✗ Build failed for flow_launcher/flow_node_host — re-run without the trailing"
  echo "    'tail -1' filter (cmake --build \"$BUILD_DIR\" --target flow_launcher flow_node_host) to see the full error."
  exit 1
fi
# Also build node plugins. They live in a separate CMake project, so the main
# flow_launcher target does not automatically rebuild them after node source edits.
echo "  Building node plugins..."
ADAS_CFG_LOG=$(cmake -B "$BUILD_DIR/modules/adas_nodes" -S "$ROOT/modules/adas_nodes" \
  -DFLOWENGINE_BUILD="$BUILD_DIR" 2>&1)
ADAS_CFG_STATUS=$?
if [ $ADAS_CFG_STATUS -ne 0 ]; then
  echo "$ADAS_CFG_LOG"
  echo "  ✗ Node plugin CMake configuration failed (exit $ADAS_CFG_STATUS)"
  exit 1
fi
# Surface any other configure-time warnings/errors too (not just the Eigen/
# Frenet fallback case handled separately below) — this log used to be
# swallowed entirely (> /dev/null), which meant genuine misconfigurations
# were undebuggable from demo.sh output.
ADAS_CFG_EXTRA_WARN=$(echo "$ADAS_CFG_LOG" | grep -i "warning\|error" || true)
if [ -n "$ADAS_CFG_EXTRA_WARN" ]; then
  echo "  ⚠ Node plugin CMake configuration reported warnings/errors:"
  echo "$ADAS_CFG_EXTRA_WARN" | sed 's/^/    /'
fi
if ! cmake --build "$BUILD_DIR/modules/adas_nodes" -j$(nproc) 2>&1 | tail -1; then
  echo "  ✗ Build failed for node plugins — re-run without the trailing 'tail -1' filter"
  echo "    (cmake --build \"$BUILD_DIR/modules/adas_nodes\") to see the full error."
  exit 1
fi
set +o pipefail
echo "  ✓ Build complete"
# planning_node silently degrades to a lane-keep-only fallback (no overtaking /
# lane changes) when the Frenet planner isn't linked in (missing Eigen). Make
# this loud instead of a one-line cmake log nobody reads.
if echo "$ADAS_CFG_LOG" | grep -q "planning_node: building in fallback mode"; then
  echo ""
  echo "  ⚠ WARNING: Frenet planner NOT built (Eigen3 not found) — ego will"
  echo "    NEVER change lanes or overtake, even with a clear adjacent lane."
  echo "    Fix: sudo apt install libeigen3-dev, then re-run this script."
  echo ""
fi

# ── Cleanup handler ─────────────────────────────────────────
# Runs on normal exit AND on Ctrl+C (INT) / TERM. It must:
#   1. run at most once (INT → cleanup → exit → EXIT would otherwise re-enter),
#   2. never block forever (a plain `wait` hangs if a child ignores SIGTERM —
#      that is exactly why a "demo.sh" process was left lingering after Ctrl+C),
#   3. reap the whole process tree, including flow_node_host children that
#      flow_launcher fork+execs in --multi mode.
CLEANED_UP=false
cleanup() {
  $CLEANED_UP && return 0
  CLEANED_UP=true
  echo ""
  echo "───[Cleanup] Shutting down..."

  # 1) Ask our direct children to stop.
  for pid in $LAUNCHER_PID $BRIDGE_PID $SERVER_PID; do
    [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null || true
  done

  # 2) Give them up to ~3s to exit gracefully, without blocking indefinitely.
  #    (6 iterations × 0.5s sleep = 3s grace period.)
  for _ in 1 2 3 4 5 6; do
    still=""
    for pid in $LAUNCHER_PID $BRIDGE_PID $SERVER_PID; do
      [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && still="$still $pid"
    done
    [ -z "$still" ] && break
    sleep 0.5
  done

  # 3) Force-kill any survivors, then sweep stragglers (multi-process node
  #    hosts, an orphaned bridge, or a previous run's server) by name.
  for pid in $LAUNCHER_PID $BRIDGE_PID $SERVER_PID; do
    [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
  done
  { pkill -9 -f flow_node_host; pkill -9 -f flow_launcher; \
    pkill -9 -f flowmond; pkill -9 -f flowboard_server; pkill -9 -f foxglove_bridge; } 2>/dev/null || true

  # 保留拓扑文件供评估器/evaluator 事后分析（不删除）
  [ -f "$JSON_FILE" ] && cp "$JSON_FILE" "${JSON_FILE%.json}_$(date +%Y%m%d_%H%M%S).json" 2>/dev/null || true

  echo ""
  echo "  ╔══════════════════════════════════════╗"
  echo "  ║  Demo Complete — FlowEngine v2.0     ║"
  echo "  ║  Plugin Architecture                 ║"
  echo "  ║  github.com/caixuf/FlowEngine        ║"
  echo "  ╚══════════════════════════════════════╝"
  # Note: no explicit `exit` here — the INT/TERM traps below exit after us,
  # and the EXIT trap just unwinds (guarded so we only run once).
}
# On Ctrl+C / TERM we must actually terminate: without an explicit exit the
# interrupted `sleep` in the monitor loop would resume and keep the demo alive.
# Exit codes follow the shell convention 128+signal (SIGINT=2 → 130, SIGTERM=15 → 143).
trap 'cleanup' EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# ── Start flow_launcher with pipeline ───────────────────────
echo "───[2/5] Starting pipeline (flowsim→sensor_model→perception→fusion→planning→control→monitor)..."
rm -f "$JSON_FILE"
cd "$ROOT"  # run from root so build/lib/ paths resolve

LAUNCHER_ARGS=("$PIPELINE")
[ "$DURATION" -gt 0 ] 2>/dev/null && LAUNCHER_ARGS+=(--duration "$DURATION")
[ "$MULTI_MODE" = true ] && LAUNCHER_ARGS+=(--multi)
if [ "$RECORD_MODE" = true ]; then
  LAUNCHER_ARGS+=(--bag "$BAG_FILE")
  echo "  Recording to: $BAG_FILE"
fi
[ "$MULTI_MODE" = true ] && echo "  Multi-process mode: each node runs as a separate process"

"$LAUNCHER_BIN" "${LAUNCHER_ARGS[@]}" \
  > /tmp/flow_launcher_stdout.txt 2>/tmp/flow_launcher_stderr.txt &
LAUNCHER_PID=$!
sleep 1
if ! kill -0 $LAUNCHER_PID 2>/dev/null; then
  echo "  ✗ Pipeline failed! Check /tmp/flow_launcher_stderr.txt"
  cat /tmp/flow_launcher_stderr.txt 2>/dev/null || true
  exit 1
fi
echo "  ✓ Pipeline running (PID $LAUNCHER_PID)"

# ── Start dashboard server (file bridge: read /tmp/flow_topology.json) ──
# 依据 VISUALIZATION_ARCHITECTURE.md: flowmond 拥有独立 MessageBus,
# 看不到 flow_launcher 进程内的数据, launch 演示必须用文件桥接。
echo "───[3/5] Starting dashboard..."
# Wait for monitor node to write first snapshot
for _ in $(seq 1 30); do
  if [ -s "$JSON_FILE" ]; then break; fi
  sleep 0.5
done
if [ ! -s "$JSON_FILE" ]; then
  echo "  ✗ Timeout waiting for $JSON_FILE — monitor node may have failed"
fi
"$BUILD_DIR/bin/flowmond" --port 8800 --html-path "$ROOT/tools/flowboard/index.html" \
  > /tmp/flowmond.log 2>&1 &
SERVER_PID=$!
sleep 2
if kill -0 $SERVER_PID 2>/dev/null; then
    # Self-check: verify the server actually responds
    CHECK=$(curl -s --max-time 3 -o /dev/null -w '%{http_code}' http://127.0.0.1:8800/api/health 2>/dev/null || echo "000")
    if [ "$CHECK" = "200" ]; then
        echo "  ✓ Dashboard at http://localhost:8800"
    else
        echo "  ✗ Dashboard started but not responding (HTTP $CHECK)"
        echo "  Server log:"
        cat /tmp/flowmond.log
    fi
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
echo "  ┌─ FlowSim ─→  Perception ─→  Fusion  ─→  Planning ─→  Control ┐"
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
SPEED=$(grep -a "flowsim.*stopped" /tmp/flow_launcher_stderr.txt 2>/dev/null | grep -oP "speed=\K[0-9.]+" || echo "?")

echo "  Simulation : $SPEED m/s final speed"
echo "  Fusion     : $FUSED EKF frames"
echo "  Planning   : $PLAN trajectories"
echo "  Control    : $CTRL control cycles"

echo ""
echo "  Dashboard  : http://localhost:8800"
echo "  Topology   : tools/topology_viewer.html (standalone)"
if [ "$RECORD_MODE" = true ] && [ -f "$BAG_FILE" ]; then
  BAG_SIZE=$(du -h "$BAG_FILE" 2>/dev/null | cut -f1)
  echo "  Bag        : $BAG_FILE ($BAG_SIZE)"
fi
echo "  CI Status  : github.com/caixuf/FlowEngine/actions"
