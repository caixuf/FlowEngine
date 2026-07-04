#!/bin/bash
# FlowEngine Topology Viewer
# Starts the FlowBoard dashboard server and opens in browser
# Usage: bash scripts/view_topology.sh [port=8800]

PORT=${1:-8800}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FLOWBOARD="$ROOT/tools/flowboard.html"
SERVER="$ROOT/tools/flowboard_server.py"

echo "╔══════════════════════════════════════╗"
echo "║  FlowBoard Dashboard                 ║"
echo "║  http://localhost:$PORT               ║"
echo "╚══════════════════════════════════════╝"
echo ""
echo "Run flow_e2e in another terminal first:"
echo "  ./build/bin/flow_e2e 30"
echo ""

if [ -f "$SERVER" ]; then
    python3 "$SERVER" --port "$PORT"
else
    echo "Error: flowboard_server.py not found."
    echo "Serving static flowboard.html instead..."
    cd "$(dirname "$FLOWBOARD")"
    python3 -m http.server "$PORT"
fi
