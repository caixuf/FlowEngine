#!/bin/bash
#
# launcher_smoke.sh — flow_launcher 多节点集成冒烟测试
#
# 验证 dlopen 单进程模式下，flow_launcher 能按 config/pipeline.json 加载全部
# 节点插件、正常运行一小段时间并优雅退出。用于 CI 的多进程/多节点集成验证。
#
# 用法:  bash scripts/launcher_smoke.sh [duration]
#
# 退出码: 0 = 通过, 非 0 = 失败
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
DURATION="${1:-3}"
LOG="$(mktemp /tmp/launcher_smoke.XXXXXX.log)"

cd "$ROOT" || { echo "FAIL: cannot cd to repo root"; exit 1; }

# ── 1. 确保 flow_launcher 已构建 ──────────────────────────────
if [ ! -x "$BUILD_DIR/bin/flow_launcher" ]; then
    echo "FAIL: $BUILD_DIR/bin/flow_launcher not found — build the main project first"
    exit 1
fi

# ── 2. 确保节点插件已构建（独立 cmake 子项目） ────────────────
if [ ! -f "$BUILD_DIR/lib/libsim_world.so" ]; then
    echo "INFO: node plugins missing — building modules/adas_nodes ..."
    cmake -B "$BUILD_DIR/modules/adas_nodes" -S "$ROOT/modules/adas_nodes" \
          -DFLOWENGINE_BUILD="$BUILD_DIR" >/dev/null 2>&1 \
        && cmake --build "$BUILD_DIR/modules/adas_nodes" -j"$(nproc)" >/dev/null 2>&1
    if [ ! -f "$BUILD_DIR/lib/libsim_world.so" ]; then
        echo "FAIL: could not build node plugins"
        exit 1
    fi
fi

# ── 3. 运行 launcher 一小段时间 ───────────────────────────────
echo "INFO: running flow_launcher (duration=${DURATION}s) ..."
timeout 90 "$BUILD_DIR/bin/flow_launcher" config/pipeline.json --duration "$DURATION" \
    > "$LOG" 2>&1
rc=$?

# ── 4. 断言 ───────────────────────────────────────────────────
fail=0

if [ $rc -ne 0 ]; then
    echo "FAIL: flow_launcher exited with code $rc (expected clean exit 0)"
    fail=1
fi

# 至少加载的节点数（planning_node 依赖 Eigen，可能缺失，故用下限 6）。
loaded=$(grep -c "loaded  " "$LOG")
if [ "$loaded" -lt 6 ]; then
    echo "FAIL: only $loaded nodes loaded (expected >= 6)"
    fail=1
else
    echo "INFO: $loaded nodes loaded"
fi

if grep -q "init() failed" "$LOG"; then
    echo "FAIL: a node reported init() failure"
    fail=1
fi

if grep -qiE "incompatible plugin ABI" "$LOG"; then
    echo "FAIL: plugin ABI mismatch detected"
    fail=1
fi

if [ $fail -ne 0 ]; then
    echo "───── launcher log tail ─────"
    tail -30 "$LOG"
    rm -f "$LOG"
    exit 1
fi

echo "PASS: flow_launcher integration smoke test"
rm -f "$LOG"
exit 0
