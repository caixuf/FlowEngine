#!/bin/bash
# =============================================================================
# FlowEngine Multi-Process Demo
#
# 每个节点作为独立进程运行，通过 UDP 发现 + TCP 传输通信。
#
# 用法:
#   bash scripts/multi_demo.sh [duration_sec=15]
#
# 架构:
#   perception ──sensor/lidar──→ fusion ──localization──→ control
#                                    └──localization──→ planning ──trajectory──→ control
#   monitor (独立监控)
# =============================================================================
set -e

DURATION=${1:-15}
BIN="$(cd "$(dirname "$0")/.." && pwd)/build/bin/flow_e2e"
JSON_FILE="/tmp/flow_topology.json"

if [ ! -x "$BIN" ]; then
    echo "Error: $BIN not found. Run './build.sh release' first."
    exit 1
fi

cleanup() {
    echo ""
    echo "Shutting down all nodes..."
    kill $PID_P $PID_F $PID_C $PID_PL $PID_M $PID_SRV 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

echo "╔══════════════════════════════════════════════════════════╗"
echo "║  FlowEngine Multi-Process Demo (${DURATION}s)               ║"
echo "║  5 nodes × independent processes                        ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# Clear state file
rm -f "$JSON_FILE"

echo "[launcher] Starting perception (sensor simulator, 10Hz LiDAR + 5Hz GPS)..."
"$BIN" --role perception "$DURATION" &
PID_P=$!
sleep 0.5

echo "[launcher] Starting fusion (time-aligned sensor fusion)..."
"$BIN" --role fusion "$DURATION" &
PID_F=$!
sleep 0.5

echo "[launcher] Starting control (driving decision maker)..."
"$BIN" --role control "$DURATION" &
PID_C=$!
sleep 0.5

echo "[launcher] Starting planning (trajectory planner)..."
"$BIN" --role planning "$DURATION" &
PID_PL=$!
sleep 0.5

echo "[launcher] Starting monitor (stats reporter → dashboard)..."
"$BIN" --role monitor "$DURATION" &
PID_M=$!
sleep 0.5

echo ""
echo "[launcher] Starting dashboard server..."
python3 "$(cd "$(dirname "$0")/.." && pwd)/tools/flowboard_server.py" \
    --json-file "$JSON_FILE" --port 8800 &
PID_SRV=$!
sleep 1

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  All 5 nodes running. Dashboard: http://localhost:8800   ║"
echo "║  Press Ctrl+C to stop.                                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

wait $PID_P $PID_F $PID_C $PID_PL $PID_M 2>/dev/null || true

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Multi-Process Demo Complete!                            ║"
echo "╚══════════════════════════════════════════════════════════╝"
