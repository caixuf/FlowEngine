#!/bin/bash
# FlowEngine Topology Viewer
# Generates discovery JSON and opens in browser
# Usage: bash scripts/view_topology.sh [port=8800]

PORT=${1:-8800}
VIEWER="$(cd "$(dirname "$0")/.." && pwd)/tools/topology_viewer.html"

echo "╔══════════════════════════════════════╗"
echo "║  FlowEngine Topology Viewer          ║"
echo "║  Open http://localhost:$PORT          ║"
echo "╚══════════════════════════════════════╝"
echo ""
echo "Paste discovery JSON from your running nodes, or:"
echo "  - Run 'flow_e2e 15 &' to start the end-to-end pipeline"
echo "  - The discovery_export_json() output can be pasted directly"
echo ""
echo "Starting HTTP server on port $PORT..."

# Check if python3 is available
if command -v python3 &> /dev/null; then
    cd "$(dirname "$VIEWER")"
    python3 -m http.server "$PORT"
elif command -v python &> /dev/null; then
    cd "$(dirname "$VIEWER")"
    python -m SimpleHTTPServer "$PORT"
else
    echo "Error: Python not found. Just open tools/topology_viewer.html in your browser."
    exit 1
fi
