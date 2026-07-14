#!/bin/bash
# FlowEngine Performance Comparison — 对比不同构建配置的性能
# Usage: bash scripts/perf_compare.sh [duration=10]

DURATION=${1:-10}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_BASE="$ROOT/build_perf"

echo "╔══════════════════════════════════════════╗"
echo "║  FlowEngine Performance Comparison       ║"
echo "║  Duration: ${DURATION}s per config         ║"
echo "╚══════════════════════════════════════════╝"
echo ""

declare -A RESULTS

run_config() {
    local name="$1"
    local build_type="$2"
    local extra_flags="$3"
    local build_dir="${BUILD_BASE}_${name}"

    echo "─── $name ($build_type) ───"

    cmake -S "$ROOT" -B "$build_dir" \
        -DCMAKE_BUILD_TYPE="$build_type" $extra_flags \
        > /dev/null 2>&1
    cmake --build "$build_dir" --target flow_launcher -j$(nproc) 2>/dev/null

    local log="/tmp/perf_${name}_$$.log"
    timeout $((DURATION + 5)) "$build_dir/bin/flow_launcher" config/pipeline.json --duration "$DURATION" > "$log" 2>&1 || true

    local fused=$(grep -c "fusion #" "$log" 2>/dev/null || echo 0)
    local frames=$(grep "stopped.*frames" "$log" 2>/dev/null | grep -o "[0-9]* frames" | head -1 | grep -o "[0-9]*" || echo 0)
    local lat_avg=$(grep "Fusion Lat" "$log" -A2 2>/dev/null | tail -1 | grep -o "avg=[0-9]*" | cut -d= -f2 || echo 0)

    RESULTS["${name}_fused"]=$fused
    RESULTS["${name}_frames"]=$frames
    RESULTS["${name}_latency"]=$lat_avg

    printf "    Fused: %4s  Frames: %4s  Latency: %4s µs\n" \
        "$fused" "$frames" "$lat_avg"

    rm -f "$log"
}

run_config "release" "Release" ""
run_config "debug"   "Debug"   ""

echo ""
echo "═══ Comparison ═══"
printf "  %-10s %8s %8s %8s\n" "Config" "Fused" "Frames" "Latency"
printf "  %-10s %8s %8s %8s\n" "──────" "─────" "──────" "───────"
for cfg in release debug; do
    printf "  %-10s %8s %8s %8s µs\n" \
        "$cfg" "${RESULTS[${cfg}_fused]}" "${RESULTS[${cfg}_frames]}" "${RESULTS[${cfg}_latency]}"
done

echo ""
FR=${RESULTS[release_fused]:-1}
FD=${RESULTS[debug_fused]:-1}
if [ "$FR" -gt 0 ] && [ "$FD" -gt 0 ]; then
    RATIO=$(echo "scale=1; $FR / $FD" | bc 2>/dev/null || echo "?")
    echo "  Release is ${RATIO}x faster than Debug (throughput)"
fi

rm -rf "$BUILD_BASE"_*
