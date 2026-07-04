#!/bin/bash
# =============================================================================
# FlowEngine Longevity Test — 长时稳定性验证
#
# 运行 e2e pipeline 持续指定时长，监控内存/CPU/消息统计，
# 检测崩溃、泄漏、性能退化。
#
# 用法:
#   bash scripts/longevity_test.sh 3600    # 1小时压测
#   bash scripts/longevity_test.sh 60      # 1分钟快速冒烟
#   bash scripts/longevity_test.sh 60 --valgrind  # valgrind 内存检测
# =============================================================================

set -e
DURATION=${1:-60}
USE_VALGRIND=false
[[ "$2" == "--valgrind" ]] && USE_VALGRIND=true

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/build/bin/flow_e2e"

echo "╔══════════════════════════════════════════╗"
echo "║  FlowEngine Longevity Test               ║"
echo "║  Duration: ${DURATION}s                    ║"
echo "║  $(date '+%Y-%m-%d %H:%M:%S')                           ║"
echo "╚══════════════════════════════════════════╝"

# Build if needed
if [ ! -x "$BIN" ]; then
    echo "[build] Building..."
    cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
    cmake --build "$ROOT/build" --target flow_e2e -j$(nproc) 2>/dev/null
fi

LOGFILE="/tmp/flowengine_longevity_$$.log"
JSONFILE="/tmp/flow_topology.json"
rm -f "$JSONFILE"

echo "[start] Launching e2e pipeline (${DURATION}s)..."

START_TS=$(date +%s)

if $USE_VALGRIND; then
    valgrind --leak-check=full --show-leak-kinds=all \
        --log-file=/tmp/valgrind_$$.log \
        "$BIN" "$DURATION" > "$LOGFILE" 2>&1 &
else
    "$BIN" "$DURATION" > "$LOGFILE" 2>&1 &
fi
PID=$!

# Monitor
LAST_PUB=0
while kill -0 $PID 2>/dev/null; do
    ELAPSED=$(($(date +%s) - START_TS))
    if [ $ELAPSED -ge $((DURATION + 10)) ]; then
        echo "[timeout] Killing stuck process..."
        kill -9 $PID 2>/dev/null
        break
    fi

    # Read stats from JSON
    if [ -f "$JSONFILE" ]; then
        STATS=$(python3 -c "
import json
with open('$JSONFILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
print(f\"pub={b.get('published',0)} del={b.get('delivered',0)} drop={b.get('dropped',0)} lat={l.get('avg_us',0)}us p99={l.get('p99_us',0)}us\")
" 2>/dev/null)
        PUB=$(echo "$STATS" | grep -o 'pub=[0-9]*' | cut -d= -f2)
        RATE=$(( (PUB - LAST_PUB) / 2 ))
        LAST_PUB=$PUB
    else
        RATE="?"
    fi

    # Memory
    if [ -f /proc/$PID/status ]; then
        MEM_KB=$(grep VmRSS /proc/$PID/status 2>/dev/null | awk '{print $2}')
        MEM_MB=$((MEM_KB / 1024))
    else
        MEM_MB="?"
    fi

    printf "\r  ⏱ %4ds | mem: %4s MB | rate: %4s msg/s | %s" \
        "$ELAPSED" "$MEM_MB" "$RATE" "$STATS"
    sleep 2
done

wait $PID 2>/dev/null || true
echo ""

# Results
END_TS=$(date +%s)
ACTUAL=$((END_TS - START_TS))
CRASHED=false
[ $ACTUAL -lt $((DURATION - 1)) ] && CRASHED=true

echo ""
echo "═══ Results ═══"
echo "  Duration:   ${DURATION}s requested, ${ACTUAL}s actual"
echo "  Crashed:    $CRASHED"

FUSED=$(grep -c "fusion #" "$LOGFILE" 2>/dev/null || echo 0)
ERRORS=$(grep -ci "ILLEGAL\|ERROR\|FATAL\|SIGSEGV\|abort\|assert" "$LOGFILE" 2>/dev/null || echo 0)
echo "  Fused:      $FUSED frames"
echo "  Errors:     $ERRORS"

# Final JSON check
if [ -f "$JSONFILE" ]; then
    python3 -c "
import json
with open('$JSONFILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
print(f'  Bus:        pub={b.get(\"published\",0)} del={b.get(\"delivered\",0)} drop={b.get(\"dropped\",0)}')
print(f'  Latency:    avg={l.get(\"avg_us\",0)}us p50={l.get(\"p50_us\",0)}us p99={l.get(\"p99_us\",0)}us')
" 2>/dev/null
fi

# Valgrind results
if $USE_VALGRIND && [ -f /tmp/valgrind_$$.log ]; then
    LEAKS=$(grep -c "definitely lost\|indirectly lost" /tmp/valgrind_$$.log 2>/dev/null || echo 0)
    echo "  Valgrind:   $LEAKS leak types"
    [ "$LEAKS" -gt 0 ] && grep "lost:" /tmp/valgrind_$$.log | head -5
fi

echo ""
if $CRASHED || [ "$ERRORS" -gt 2 ]; then
    echo "❌ FAILED"
    exit 1
else
    echo "✅ PASSED"
fi

rm -f "$LOGFILE" "$JSONFILE" /tmp/valgrind_$$.log
