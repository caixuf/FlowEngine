#!/bin/bash
# =============================================================================
# FlowEngine 全栈多进程集成演示
#
#   感知进程 ──IPC──> 融合进程 ──bus──> 控制进程
#      ↑ UDP discovery + 状态机追踪 ↑
#
# 用法: bash scripts/fullstack_demo.sh [duration_sec=8]
# =============================================================================
set -e

DURATION=${1:-8}
BIN_DIR="$(cd "$(dirname "$0")/../build/bin" && pwd)"
FULLSTACK="$BIN_DIR/flow_fullstack"

if [ ! -x "$FULLSTACK" ]; then
    echo "Error: $FULLSTACK not found. Run './build.sh release' first."
    exit 1
fi

echo "╔══════════════════════════════════════════════════════════╗"
echo "║  FlowEngine Full-Stack Demo (${DURATION}s)               ║"
echo "║  感知 → 融合 → 控制  全链路                              ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $PID_PERCEPTION $PID_FUSION $PID_CONTROL 2>/dev/null
    wait 2>/dev/null
    echo "All processes stopped."
}

trap cleanup EXIT INT TERM

echo "[launcher] Starting perception node..."
"$FULLSTACK" --role perception --duration "$DURATION" &
PID_PERCEPTION=$!
sleep 0.5

echo "[launcher] Starting fusion node..."
"$FULLSTACK" --role fusion --duration "$DURATION" &
PID_FUSION=$!
sleep 0.5

echo "[launcher] Starting control node..."
"$FULLSTACK" --role control --duration "$DURATION" &
PID_CONTROL=$!

echo ""
echo "[launcher] All 3 nodes running. Wait ${DURATION}s..."
echo "[launcher] Press Ctrl+C to stop early."
echo ""

wait $PID_PERCEPTION $PID_FUSION $PID_CONTROL 2>/dev/null || true

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Demo Complete!                                         ║"
echo "╚══════════════════════════════════════════════════════════╝"
