#!/bin/bash
# FlowEngine Sanitizer CI Script
# Builds and tests with ASAN and TSAN enabled.
# Usage: bash scripts/ci_sanitizers.sh [asan|tsan|ubsan|all]
set -e

MODE="${1:-all}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

run_sanitizer() {
    local name="$1"
    local flag="$2"
    local build_dir="$PROJECT_DIR/build-$name"

    echo ""
    echo "══════════════════════════════════════════"
    echo "  $name Build & Test"
    echo "══════════════════════════════════════════"

    # Configure
    cmake -B "$build_dir" -S "$PROJECT_DIR" \
        "-D${flag}=ON" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_TESTING=ON 2>&1 | tail -3

    # Build
    cmake --build "$build_dir" -j$(nproc) 2>&1 | tail -5

    # Run tests (exclude benchmarks and long-running stability tests)
    if ctest --test-dir "$build_dir" --output-on-failure -j$(nproc) \
        -E "benchmark|manual|stability" 2>&1; then
        echo "  ✅ $name: all tests passed"
    else
        echo "  ❌ $name: test failures detected"
        return 1
    fi
}

case "$MODE" in
    asan)
        run_sanitizer "asan" "ENABLE_ASAN"
        ;;
    tsan)
        run_sanitizer "tsan" "ENABLE_TSAN"
        ;;
    ubsan)
        run_sanitizer "ubsan" "ENABLE_UBSAN"
        ;;
    all)
        echo "╔══════════════════════════════════════════╗"
        echo "║  FlowEngine Sanitizer CI                 ║"
        echo "╚══════════════════════════════════════════╝"
        FAILED=0
        run_sanitizer "asan" "ENABLE_ASAN" || FAILED=$((FAILED+1))
        run_sanitizer "ubsan" "ENABLE_UBSAN" || FAILED=$((FAILED+1))
        # TSAN last — it's the slowest
        run_sanitizer "tsan" "ENABLE_TSAN" || FAILED=$((FAILED+1))
        echo ""
        echo "══════════════════════════════════════════"
        echo "  Sanitizer Results: $FAILED failed"
        echo "══════════════════════════════════════════"
        exit $FAILED
        ;;
    *)
        echo "Usage: $0 [asan|tsan|ubsan|all]"
        exit 1
        ;;
esac
