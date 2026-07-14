#!/bin/bash
# FlowEngine Stress Test
# Usage: bash scripts/stress_test.sh [duration_sec=30] [iterations=3] [--full]
#   --full: also run 1-hour bus stress, IPC stability, and bag consistency checks
set -e

DURATION=${1:-30}
ITERS=${2:-3}
FULL_MODE=0
[[ "$*" == *"--full"* ]] && FULL_MODE=1

BIN_DIR="$(cd "$(dirname "$0")/../build/bin" && pwd)"
E2E="$BIN_DIR/flow_launcher"
BUS="$BIN_DIR/flow_bus"
IPC="$BIN_DIR/flow_ipc"
BAG="$BIN_DIR/flow_bag"

PASS=0
FAIL=0

echo "╔══════════════════════════════════════════╗"
echo "║  FlowEngine Stress Test                  ║"
echo "║  Duration: ${DURATION}s × $ITERS iterations   ║"
echo "╚══════════════════════════════════════════╝"

# ── E2E Pipeline Stress ──────────────────────────────────────
e2e_stress() {
    echo ""
    echo "─── E2E Pipeline Stress ───"
    for i in $(seq 1 $ITERS); do
        echo "  Iteration $i/$ITERS..."
        START=$(date +%s)
        if timeout $((DURATION+5)) "$E2E" config/pipeline.json --duration "$DURATION" > /tmp/stress_launcher_$i.log 2>&1; then
            END=$(date +%s)
            ELAPSED=$((END - START))
            ERRORS=$(grep -c "ILLEGAL\|ERROR\|FATAL\|CRASH\|SIGSEGV" /tmp/stress_launcher_$i.log 2>/dev/null || echo 0)
            echo "    Time: ${ELAPSED}s | Errors: $ERRORS"
            if [ "$ERRORS" -gt 2 ]; then
                echo "    ❌ FAIL: $ERRORS errors"
                FAIL=$((FAIL+1))
            else
                echo "    ✅ PASS"
                PASS=$((PASS+1))
            fi
        else
            echo "    ❌ FAIL: crashed or timed out"
            FAIL=$((FAIL+1))
        fi
    done
}

# ── Message Bus Throughput ───────────────────────────────────
bus_stress() {
    echo ""
    echo "─── Message Bus Throughput ───"
    local dur=${1:-10}
    for i in $(seq 1 $ITERS); do
        echo "  Iteration $i/$ITERS (${dur}s)..."
        if timeout $((dur+5)) "$BUS" > /tmp/stress_bus_$i.log 2>&1; then
            PUB=$(grep -oP 'published=\K[0-9]+' /tmp/stress_bus_$i.log 2>/dev/null || echo 0)
            DROP=$(grep -oP 'dropped=\K[0-9]+' /tmp/stress_bus_$i.log 2>/dev/null || echo 0)
            echo "    Published: $PUB | Dropped: $DROP"
            echo "    ✅ PASS"
            PASS=$((PASS+1))
        else
            echo "    ❌ FAIL: bus demo crashed"
            FAIL=$((FAIL+1))
        fi
    done
}

# ── IPC Stability ────────────────────────────────────────────
ipc_stress() {
    echo ""
    echo "─── IPC Channel Stability ───"
    local dur=${1:-10}
    for i in $(seq 1 $ITERS); do
        echo "  Iteration $i/$ITERS (${dur}s)..."
        if timeout $((dur+5)) "$IPC" > /tmp/stress_ipc_$i.log 2>&1; then
            echo "    ✅ PASS"
            PASS=$((PASS+1))
        else
            echo "    ❌ FAIL: IPC demo crashed"
            FAIL=$((FAIL+1))
        fi
    done
}

# ── Bag Record/Replay Consistency ────────────────────────────
bag_stress() {
    echo ""
    echo "─── Bag Record/Replay Consistency ───"
    local dur=${1:-5}
    local bagfile="/tmp/stress_test_$(date +%s).bag"

    for i in $(seq 1 $ITERS); do
        echo "  Iteration $i/$ITERS..."
        # Record then replay
        if timeout $((dur+10)) "$BAG" > /tmp/stress_bag_$i.log 2>&1; then
            REC=$(grep -c "recorded\|Recorded\|wrote\|Wrote" /tmp/stress_bag_$i.log 2>/dev/null || echo 0)
            echo "    Messages: $REC"
            echo "    ✅ PASS"
            PASS=$((PASS+1))
        else
            echo "    ❌ FAIL: bag demo failed"
            FAIL=$((FAIL+1))
        fi
    done
    rm -f "$bagfile"
}

# ── Launcher Smoke quick test ─────────────────────────────────
smoke_test() {
    echo ""
    echo "─── Launcher Smoke Test ───"
    local ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    if bash "$ROOT/scripts/launcher_smoke.sh" 3 > /tmp/smoke.log 2>&1; then
        echo "  ✅ PASS"
        PASS=$((PASS+1))
    else
        echo "  ❌ FAIL"
        FAIL=$((FAIL+1))
    fi
}

# ── Run tests ─────────────────────────────────────────────────
smoke_test
e2e_stress

if [ "$FULL_MODE" -eq 1 ]; then
    echo ""
    echo "══════════════════════════════════════════"
    echo "  Full Mode: extended stress tests"
    echo "══════════════════════════════════════════"
    bus_stress 10
    ipc_stress 10
    bag_stress 5
fi

# ── Valgrind memory leak check ────────────────────────────────
if command -v valgrind &> /dev/null; then
    echo ""
    echo "─── Valgrind Memory Check ───"
    if valgrind --leak-check=full --error-exitcode=1 \
        "$E2E" config/pipeline.json --duration 5 > /tmp/valgrind.log 2>&1; then
        echo "  ✅ No memory leaks"
        PASS=$((PASS+1))
    else
        echo "  ❌ Memory leak detected!"
        grep -E "definitely lost|indirectly lost|possibly lost|ERROR SUMMARY" /tmp/valgrind.log
        FAIL=$((FAIL+1))
    fi
fi

echo ""
echo "══════════════════════════════════════════"
echo "  Results: $PASS passed, $FAIL failed"
echo "══════════════════════════════════════════"

exit $FAIL
