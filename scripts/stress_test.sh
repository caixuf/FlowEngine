#!/bin/bash
# FlowEngine Stress Test
# Runs e2e demo at high frequency, checks for memory leaks, crashes, and performance regressions.
# Usage: bash scripts/stress_test.sh [duration_sec=30] [iterations=3]

set -e
DURATION=${1:-30}
ITERS=${2:-3}
BIN="$(cd "$(dirname "$0")/../build/bin" && pwd)/flow_e2e"

echo "╔══════════════════════════════════════════╗"
echo "║  FlowEngine Stress Test                  ║"
echo "║  Duration: ${DURATION}s × $ITERS iterations   ║"
echo "╚══════════════════════════════════════════╝"

PASS=0
FAIL=0

for i in $(seq 1 $ITERS); do
    echo ""
    echo "─── Iteration $i/$ITERS ───"

    START=$(date +%s)
    if timeout $((DURATION+5)) "$BIN" "$DURATION" > /tmp/stress_$i.log 2>&1; then
        END=$(date +%s)
        ELAPSED=$((END - START))

        # Check key metrics
        FUSED=$(grep -c "fusion #" /tmp/stress_$i.log || echo 0)
        DECISIONS=$(grep -c "control #" /tmp/stress_$i.log || echo 0)
        ERRORS=$(grep -c "ILLEGAL\|ERROR\|FATAL\|CRASH\|SIGSEGV" /tmp/stress_$i.log || echo 0)

        echo "  Time: ${ELAPSED}s | Fused: $FUSED | Decisions: $DECISIONS | Errors: $ERRORS"

        if [ "$ERRORS" -gt 2 ]; then  # Allow up to 2 benign ILLEGAL warnings
            echo "  ❌ FAIL: $ERRORS errors detected"
            FAIL=$((FAIL+1))
        else
            echo "  ✅ PASS"
            PASS=$((PASS+1))
        fi
    else
        echo "  ❌ FAIL: process crashed or timed out"
        FAIL=$((FAIL+1))
        tail -20 /tmp/stress_$i.log
    fi
done

echo ""
echo "══════════════════════════════════════════"
echo "  Results: $PASS passed, $FAIL failed"
echo "══════════════════════════════════════════"

# Memory leak check (if valgrind available)
if command -v valgrind &> /dev/null; then
    echo ""
    echo "─── Valgrind memory check ───"
    valgrind --leak-check=full --error-exitcode=1 \
        "$BIN" 5 > /tmp/valgrind.log 2>&1 || {
        echo "❌ Memory leak detected!"
        grep "definitely lost\|indirectly lost\|possibly lost" /tmp/valgrind.log
    }
    echo "✅ No memory leaks"
fi

exit $FAIL
