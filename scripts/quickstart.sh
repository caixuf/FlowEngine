#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -x "$ROOT/bin/flow_launcher" ] && [ -x "$ROOT/../../bin/flow_launcher" ]; then
    ROOT="$(cd "$ROOT/../.." && pwd)"
fi

DURATION="${1:-15}"
PIPELINE="${FLOWENGINE_PIPELINE:-$ROOT/share/flowengine/config/pipeline.json}"
FLOWBOARD="$ROOT/share/flowengine/flowboard/index.html"
JSON_FILE="/tmp/flow_topology.json"
LOG_DIR="$ROOT/logs"
mkdir -p "$LOG_DIR"

export FLOWENGINE_HOME="$ROOT"
export PATH="$ROOT/bin:$PATH"
export LD_LIBRARY_PATH="$ROOT/lib:$ROOT/lib/flowengine/plugins:${LD_LIBRARY_PATH:-}"

LAUNCHER_PID=""
SERVER_PID=""

cleanup() {
    for pid in "$LAUNCHER_PID" "$SERVER_PID"; do
        [ -n "$pid" ] && kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "$LAUNCHER_PID" "$SERVER_PID"; do
        [ -n "$pid" ] && kill -KILL "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

if [ ! -f "$PIPELINE" ]; then
    echo "Missing pipeline: $PIPELINE" >&2
    exit 1
fi
if [ ! -f "$FLOWBOARD" ]; then
    echo "Missing dashboard: $FLOWBOARD" >&2
    exit 1
fi

cd "$ROOT"
rm -f "$JSON_FILE"

echo "Starting FlowEngine demo for ${DURATION}s..."
"$ROOT/bin/flow_launcher" "$PIPELINE" --duration "$DURATION" \
    > "$LOG_DIR/flow_launcher.out" 2> "$LOG_DIR/flow_launcher.err" &
LAUNCHER_PID=$!

for _ in $(seq 1 40); do
    [ -s "$JSON_FILE" ] && break
    if ! kill -0 "$LAUNCHER_PID" 2>/dev/null; then
        echo "flow_launcher exited early; see $LOG_DIR/flow_launcher.err" >&2
        exit 1
    fi
    sleep 0.25
done

"$ROOT/bin/flowmond" --port 8800 --html-path "$FLOWBOARD" \
    > "$LOG_DIR/flowmond.out" 2> "$LOG_DIR/flowmond.err" &
SERVER_PID=$!

for _ in $(seq 1 40); do
    code="$(curl -s --max-time 2 -o /dev/null -w '%{http_code}' http://127.0.0.1:8800/api/health 2>/dev/null || echo 000)"
    [ "$code" = "200" ] && break
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "flowmond exited early; see $LOG_DIR/flowmond.err" >&2
        exit 1
    fi
    sleep 0.25
done

if [ "${code:-000}" != "200" ]; then
    echo "Dashboard did not become healthy (HTTP ${code:-000}); see $LOG_DIR/flowmond.err" >&2
    exit 1
fi

echo "Dashboard: http://localhost:8800"
echo "Topology:  $JSON_FILE"
echo "Logs:      $LOG_DIR"

wait "$LAUNCHER_PID" 2>/dev/null || true
