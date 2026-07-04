#!/bin/bash
# =============================================================================
# FlowEngine Sim Demo — 仿真数据 + 全链路 + 实时监控
#
# 流程:
#   2D 运动学模拟器 → topic → Fusion → Control → FlowBoard Dashboard
#
# 用法:
#   bash scripts/demo_sim.sh              # 默认 30s
#   bash scripts/demo_sim.sh 60           # 60s
#   bash scripts/demo_sim.sh --carla      # Carla 模式 (需要 Carla 运行中)
# =============================================================================
set -e

DURATION=${1:-30}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build"
JSON_FILE="/tmp/flow_topology.json"
SIM_LOG="/tmp/sim_pipeline.log"

echo "╔══════════════════════════════════════════════════════════╗"
echo "║  FlowEngine Simulation Demo                              ║"
echo "║  2D Sim → Perception → Fusion → Control → FlowBoard      ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "  Duration: ${DURATION}s"
echo ""

# Build if needed
if [ ! -x "$BUILD_DIR/bin/flow_e2e" ]; then
    echo "[1/4] Building..."
    cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
    cmake --build "$BUILD_DIR" --target flow_e2e -j$(nproc) 2>/dev/null
fi

# Clean state
rm -f "$JSON_FILE" "$SIM_LOG"

echo "[1/4] Starting simulation pipeline..."
echo ""

# Run e2e with simulation data — this feeds lidar/gps topics
# which flow through fusion → control → monitor
"$BUILD_DIR/bin/flow_e2e" "$DURATION" > "$SIM_LOG" 2>&1 &
E2E_PID=$!
sleep 1

echo "[2/4] Pipeline running (PID $E2E_PID)"
echo ""

# Start dashboard server
echo "[3/4] Starting FlowBoard..."
python3 "$ROOT/tools/flowboard_server.py" --json-file "$JSON_FILE" --port 8800 > /dev/null 2>&1 &
SERVER_PID=$!
sleep 1
echo "  Dashboard: http://localhost:8800"
echo ""

# Open browser
if command -v xdg-open &>/dev/null; then
    xdg-open http://localhost:8800 2>/dev/null || true
elif command -v open &>/dev/null; then
    open http://localhost:8800 2>/dev/null || true
fi

echo "[4/4] Live monitoring..."
echo ""
echo "  ┌─────────────────────────────────────────────────────┐"
echo "  │  Pipeline:                                          │"
echo "  │                                                     │"
echo "  │  🚗 2D Vehicle Model                                │"
echo "  │   ├─ LiDAR frames  (10Hz)                           │"
echo "  │   ├─ GPS position  (5Hz)                            │"
echo "  │   └─ Obstacle list                                  │"
echo "  │        ↓                                            │"
echo "  │  🔄 Eigen EKF Fusion (50ms window)                  │"
echo "  │   └─ State estimate (x,y,vx,vy,ax,ay)               │"
echo "  │        ↓                                            │"
echo "  │  🎮 OSQP MPC Control                                │"
echo "  │   └─ throttle/brake/steer commands                  │"
echo "  │        ↓                                            │"
echo "  │  📊 FlowBoard Dashboard                             │"
echo "  │   ├─ Topology: nodes + topics                       │"
echo "  │   ├─ Frames:  pub/del/drop/latency                  │"
echo "  │   ├─ Charts:  rate + latency + throughput           │"
echo "  │   └─ QoS:     per-topic stats table                 │"
echo "  └─────────────────────────────────────────────────────┘"
echo ""

ELAPSED=0
while [ $ELAPSED -lt $DURATION ] && kill -0 $E2E_PID 2>/dev/null; do
    if [ -f "$JSON_FILE" ]; then
        STATS=$(python3 -c "
import json
with open('$JSON_FILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
print(f'pub={b.get(\"published\",0)} del={b.get(\"delivered\",0)} drop={b.get(\"dropped\",0)} lat={l.get(\"avg_us\",0)}us p99={l.get(\"p99_us\",0)}us')
" 2>/dev/null)
        printf "\r  ⏱ %3ds | %s" "$ELAPSED" "$STATS"
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done
echo ""
echo ""

# Summary
wait $E2E_PID 2>/dev/null || true

FUSED=$(grep -c "fusion #" "$SIM_LOG" 2>/dev/null || echo 0)
CTRL=$(grep -c "control #" "$SIM_LOG" 2>/dev/null || echo 0)
MODE=$(grep "driving mode" "$SIM_LOG" 2>/dev/null | tail -1 | grep -o "ACC\|CP\|NP\|NA" || echo "?")

echo "═══ Pipeline Results ═══"
echo "  Fused frames: $FUSED"
echo "  Control decisions: $CTRL"
echo "  Drive mode: $MODE"

if [ -f "$JSON_FILE" ]; then
    python3 -c "
import json
with open('$JSON_FILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
print(f'  Bus:  pub={b.get(\"published\",0)} del={b.get(\"delivered\",0)} drop={b.get(\"dropped\",0)}')
print(f'  Latency: avg={l.get(\"avg_us\",0)}us p50={l.get(\"p50_us\",0)}us p99={l.get(\"p99_us\",0)}us')
ts = d.get('metrics',{}).get('topics',[])
for t in ts:
    print(f'  Topic {t[\"topic\"]}: pub={t.get(\"pub\",0)} del={t.get(\"del\",0)} drop={t.get(\"drop\",0)} lat={t.get(\"lat_us\",0)}us')
" 2>/dev/null
fi

echo ""
echo "  Dashboard: http://localhost:8800 (browser already open)"
echo "  Topology:  paste JSON into tools/topology_viewer.html"
echo ""

# Keep dashboard running
echo "Dashboard server still running (PID $SERVER_PID). Press Ctrl+C to stop."
trap "kill $SERVER_PID 2>/dev/null; echo 'Stopped.'" EXIT
wait $SERVER_PID 2>/dev/null || true
