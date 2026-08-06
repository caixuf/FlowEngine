#!/bin/bash
# =============================================================================
# FlowEngine Demo — 一键演示脚本 (v2 — 配置驱动插件架构)
#
# 使用 flow_launcher + pipeline.json 启动全链路节点插件。
# 主流程不再自动弹浏览器（WSL/无头环境）；仅 --replay 模式会自动打开仪表盘。
#
# 用法:
#   bash scripts/demo.sh              # 默认 15 秒，dlopen 单进程模式
#   bash scripts/demo.sh 30           # 30 秒演示
#   bash scripts/demo.sh --multi      # fork+exec 多进程模式
#   bash scripts/demo.sh --manual     # 游戏模式：终端 WASD 键盘直接驾驶 ego
#   bash scripts/demo.sh --scenario scenarios/straight_road.json  # 指定场景
#   bash scripts/demo.sh --no-browser # 不自动打开浏览器（仅对 --replay 模式生效）
# =============================================================================
set -e

# ── 跨平台可移植性 ────────────────────────────────────────────
# CPU 核数：Linux 用 nproc，macOS 无 nproc 用 sysctl，都缺则退回 4。
NPROC="$( (command -v nproc >/dev/null 2>&1 && nproc) \
          || sysctl -n hw.ncpu 2>/dev/null || echo 4 )"

# 默认场景：200m 高架+国道复合场景，单 ego 无 NPC（基础设施验证）
# 可用 --scenario 覆盖；不指定时 patch pipeline.json 指向此场景。
DEFAULT_SCENARIO="${FLOWENGINE_SCENARIO:-scenarios/straight_road.json}"

LOG_DIR="${FLOW_LOG_DIR:-/tmp/flow_logs}"
PID_FILE="${FLOWENGINE_DEMO_PID_FILE:-/tmp/flowengine_demo.pids}"

terminate_pids() {
  local pids=()
  local pid
  for pid in "$@"; do
    case "$pid" in
      ''|*[!0-9]*) continue ;;
      "$$") continue ;;
    esac
    kill -0 "$pid" 2>/dev/null && pids+=("$pid")
  done
  [ ${#pids[@]} -eq 0 ] && return 0

  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for _ in 1 2 3 4 5 6; do
    local alive=()
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive+=("$pid"); done
    [ ${#alive[@]} -eq 0 ] && return 0
    pids=("${alive[@]}")
    sleep 0.5
  done
  for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
}

record_pid() {
  case "${1:-}" in
    ''|*[!0-9]*) return 0 ;;
  esac
  printf '%s %s\n' "$1" "${2:-unknown}" >> "$PID_FILE"
}

pid_matches_label() {
  local pid="$1"
  local label="$2"
  local args
  args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  [ -z "$args" ] && return 1
  case "$label" in
    flow_launcher) [[ "$args" == *flow_launcher* ]] ;;
    flowmond) [[ "$args" == *flowmond* ]] ;;
    foxglove_bridge) [[ "$args" == *foxglove_bridge.py* ]] ;;
    beh_log_watcher) [[ "$args" == *flow_beh_monitor.txt* ]] ;;
    *) return 1 ;;
  esac
}

terminate_recorded_pids() {
  [ -f "$PID_FILE" ] || return 0
  local pid label
  local pids=()
  while read -r pid label _; do
    case "$pid" in
      ''|*[!0-9]*) continue ;;
    esac
    if pid_matches_label "$pid" "$label"; then
      pids+=("$pid")
    fi
  done < "$PID_FILE"
  terminate_pids "${pids[@]}"
}

# Kill stale children recorded by the previous demo run. Avoid process-name
# sweeps: this environment may be shared, so only PIDs created by demo.sh are
# eligible for cleanup.
if [ -f "$PID_FILE" ]; then
  terminate_recorded_pids
  rm -f "$PID_FILE"
fi
sleep 0.5
# Clean up old per-module log files from previous run
rm -rf "$LOG_DIR" 2>/dev/null || true
mkdir -p "$LOG_DIR"
for port in 8800 8765; do
  # Linux 优先 ss；macOS 无 ss（也无 /proc）用 lsof 找监听该端口的进程。
  if command -v ss >/dev/null 2>&1; then
    pid=$(ss -tlnp "sport = :$port" 2>/dev/null | grep -oP 'pid=\K\d+' | head -1)
  else
    pid=$(lsof -ti "tcp:$port" -sTCP:LISTEN 2>/dev/null | head -1)
  fi
  [ -n "$pid" ] && terminate_pids "$pid"
done
sleep 0.5

DURATION=0  # 0 = 自动从场景 duration_s 读取，设置正数 = 覆盖
# 默认无限时模式：场景可能跑超过 duration_s（NPC/事件还在继续），
# 退出由用户 Ctrl+C 触发。需要限时演示时显式传数字：bash demo.sh 60。
AUTO_DURATION_DEFAULT=0  # 0 = 不限时；改非零则恢复"无字段时默认 60s"旧行为
OPEN_BROWSER=true
MULTI_MODE=false
RECORD_MODE=false
GAME_MODE=false
REPLAY_FILE=""
SCENARIO=""  # 留空则用 DEFAULT_SCENARIO（旗舰场景）
MANUAL_MODE=false
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build"
LAUNCHER_BIN="$BUILD_DIR/bin/flow_launcher"
PIPELINE="${FLOW_PIPELINE:-$ROOT/config/pipeline.json}"
JSON_FILE="/tmp/flow_topology.json"
BAG_FILE="/tmp/flow_demo_$(date +%Y%m%d_%H%M%S).bag"

while [ $# -gt 0 ]; do
  case "$1" in
    --no-browser) OPEN_BROWSER=false ;;
    --multi) MULTI_MODE=true ;;
    --manual) MANUAL_MODE=true ;;
    --record) RECORD_MODE=true ;;
    --replay) REPLAY_FILE="$2"; shift ;;
    --game) GAME_MODE=true ;;
    --scenario) SCENARIO="$2"; shift ;;
    ''|*[!0-9]*) ;;
    *) DURATION="$1" ;;
  esac
  shift
done

# ── Manual mode: 用 pipeline_manual.json 替代默认 pipeline.json ──
# 游戏模式：删除 planning/control/safety_control/inference，新增 manual_drive
# 节点（终端 WASD 键盘 → control/cmd）。flowsim 无感知地消费 manual_drive
# 发布的 ControlCmd 驱动 ego。--manual 与 --scenario 可组合使用。
if [ "$MANUAL_MODE" = true ]; then
  PIPELINE="$ROOT/config/pipeline_manual.json"
fi

# ── Scenario override: patch pipeline.json's scenario_file ──
# 未指定 --scenario 时用 DEFAULT_SCENARIO（旗舰场景），patch 进临时 pipeline.json。
# --manual 模式下 patch 基底换成 pipeline_manual.json（flowsim 仍读 scenario_file）。
if [ "$MANUAL_MODE" = true ]; then
  PIPELINE_ORIG="$ROOT/config/pipeline_manual.json"
else
  # FLOW_PIPELINE 允许外部注入自定义 pipeline（如 learning_loop.py 换 inference
  # model_path 做影子评估），scenario patch 以它为基底而不是覆盖它。
  PIPELINE_ORIG="${FLOW_PIPELINE:-$ROOT/config/pipeline.json}"
fi
PIPELINE_TMP=""
cleanup_pipeline_tmp() {
  [ -n "$PIPELINE_TMP" ] && rm -f "$PIPELINE_TMP"
}
if [ -z "$SCENARIO" ]; then
  SCENARIO="$DEFAULT_SCENARIO"
fi
if [ -n "$SCENARIO" ]; then
  # mktemp 模板差异:GNU 接受 XXXX 后带后缀,BSD(macOS)只认结尾的 X(否则
  # 原样建出 "pipeline_XXXX.json" 并残留、阻塞下次 mkstemp)。用 -t + 结尾 X 的
  # 可移植写法先建无后缀临时文件,再改名加 .json —— 两平台一致。
  PIPELINE_TMP="$(mktemp /tmp/pipeline_XXXXXX)" || { echo "  ✗ mktemp failed"; exit 1; }
  mv "$PIPELINE_TMP" "$PIPELINE_TMP.json"
  PIPELINE_TMP="$PIPELINE_TMP.json"
  trap 'cleanup_pipeline_tmp' EXIT
  SCENARIO_ABS="$([ -f "$SCENARIO" ] && echo "$(cd "$(dirname "$SCENARIO")" && pwd)/$(basename "$SCENARIO")" || echo "$SCENARIO")"
  sed 's|\\"scenario_file\\": \\"[^\\"]*\\"|\\"scenario_file\\": \\"'$SCENARIO_ABS'\\"|g' "$PIPELINE_ORIG" > "$PIPELINE_TMP"
  PIPELINE="$PIPELINE_TMP"
  echo "  Scenario: $SCENARIO_ABS"
fi

# ── Replay fast path: no pipeline, just flowmond + flow_launcher --replay ──
if [ -n "$REPLAY_FILE" ]; then
  echo "═══ Replay Mode: $REPLAY_FILE ═══"
  "$BUILD_DIR/bin/flowmond" --port 8800 --html-path "$ROOT/tools/flowboard/index.html" > /tmp/flowmond.log 2>&1 &
  SERVER_PID=$!
  sleep 1
  echo "  Dashboard: http://localhost:8800"
  if $OPEN_BROWSER; then
    # 等服务器就绪再开浏览器（复用 main path 的 health check 写法）
    for _ in $(seq 1 20); do
      HEALTH_CODE=$(curl -s --max-time 2 -o /dev/null -w '%{http_code}' http://127.0.0.1:8800/api/health 2>/dev/null || echo "000")
      if [ "$HEALTH_CODE" = "200" ]; then break; fi
      sleep 0.25
    done
    xdg-open http://localhost:8800 2>/dev/null || open http://localhost:8800 2>/dev/null || true
  fi
  "$BUILD_DIR/bin/flow_bag" --replay "$REPLAY_FILE" 2>&1
  kill $SERVER_PID 2>/dev/null
  exit 0
fi

# ── Banner ──────────────────────────────────────────────────
if [ -t 1 ] && [ -n "${TERM:-}" ] && [ "${TERM}" != "dumb" ] && command -v clear >/dev/null 2>&1; then
  clear
fi
cat << 'BANNER'

  ╔══════════════════════════════════════════════════════════╗
  ║                                                          ║
  ║   ███████╗██╗      ██████╗ ██╗    ██╗                  ║
  ║   ██╔════╝██║     ██╔═══██╗██║    ██║                  ║
  ║   █████╗  ██║     ██║   ██║██║ █╗ ██║                  ║
  ║   ██╔══╝  ██║     ██║   ██║██║███╗██║                  ║
  ║   ██║     ███████╗╚██████╔╝╚███╔███╔╝                  ║
  ║   ╚═╝     ╚══════╝ ╚═════╝  ╚══╝╚══╝                   ║
  ║                                                          ║
  ║   E N G I N E                                           ║
  ║   Lightweight Middleware for Autonomous Driving          ║
  ║                                                          ║
  ╚══════════════════════════════════════════════════════════╝

BANNER

   SCENARIO_DISPLAY="${SCENARIO:-$(grep -o 'scenarios/[^\"]*' "$PIPELINE_ORIG" | head -1)}"
   # Auto-detect duration from scenario if not explicitly given
   if [ "$DURATION" -le 0 ] 2>/dev/null; then
     DURATION=$(python3 -c "import json; d=json.load(open('$SCENARIO_ABS')); print(int(d.get('duration_s',0) or 0))" 2>/dev/null || echo 0)
     # DURATION 仍 ≤0 时不再默认 60s 限时：场景里的事件/NPC 在 duration_s 之后
     # 还会继续跑（回收车流、ETC 重复触发等），限时退出会让用户错过后续。
     # 默认 0 = 不限时，由 Ctrl+C 退出。需要限时显式传数字：bash demo.sh 60。
     [ "$DURATION" -le 0 ] 2>/dev/null && DURATION=$AUTO_DURATION_DEFAULT
   fi
   if [ "$DURATION" -gt 0 ] 2>/dev/null; then
     echo "Demo Duration: ${DURATION}s   Mode: $([ "$MULTI_MODE" = true ] && echo "Multi-Process" || echo "Single-Process (dlopen)")   Scenario: ${SCENARIO_DISPLAY:-default}"
   else
     echo "Demo Duration: ∞ (Ctrl+C 退出)   Mode: $([ "$MULTI_MODE" = true ] && echo "Multi-Process" || echo "Single-Process (dlopen)")   Scenario: ${SCENARIO_DISPLAY:-default}"
   fi
   if [ "$MANUAL_MODE" = true ]; then
     echo "  🎮 Manual Drive Mode: WASD=油门/刹车/转向  空格=手刹  q=退出  (pipeline_manual.json)"
   fi
echo ""

# ── Build ───────────────────────────────────────────────────
echo "───[1/4] Building..."
if [ ! -f "$LAUNCHER_BIN" ]; then
  echo "  First build, this may take a moment..."
  # 仅 Linux 强制 gcc（项目在 Ubuntu/CI 上以 gcc 为准）；macOS 无 gcc，用系统
  # 默认 Apple clang（C++20 协程 -std=c++20 原生支持，无需 -fcoroutines）。
  CC_ARG=""
  [ "$(uname -s)" = "Linux" ] && CC_ARG="-DCMAKE_C_COMPILER=gcc"
  cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release $CC_ARG > /dev/null 2>&1
fi
set -o pipefail  # scoped to this build block only — restored via `set +o pipefail`
                 # below once both `cmake --build ... | tail -1` calls are done, so
                 # a build failure surfaces (pipeline exit = cmake's, not tail's)
                 # without changing pipe-failure semantics for the rest of the script.
if ! cmake --build "$BUILD_DIR" --target flow_launcher flow_node_host flowmond -j"$NPROC" 2>&1 | tail -1; then
  echo "  ✗ Build failed for flow_launcher/flow_node_host/flowmond — re-run without the trailing"
  echo "    'tail -1' filter (cmake --build \"$BUILD_DIR\" --target flow_launcher flow_node_host flowmond) to see the full error."
  exit 1
fi
# Also build node plugins. They live in a separate CMake project, so the main
# flow_launcher target does not automatically rebuild them after node source edits.
echo "  Building node plugins..."
# 自动检测 ONNX Runtime：头文件存在则启用 ONNX（可选，缺失时回退 tiny-MLP，不影响 demo）
ONNX_FLAG=""
if [ -f /usr/local/include/onnxruntime/onnxruntime_cxx_api.h ] || \
   [ -f /usr/include/onnxruntime/onnxruntime_cxx_api.h ]; then
    ONNX_FLAG="-DENABLE_ONNX=ON"
    echo "    ONNX Runtime detected — enabling HAVE_ONNXRUNTIME"
fi
ADAS_CFG_LOG=$(cmake -B "$BUILD_DIR/modules/adas_nodes" -S "$ROOT/modules/adas_nodes" \
  -DFLOWENGINE_BUILD="$BUILD_DIR" $ONNX_FLAG 2>&1)
ADAS_CFG_STATUS=$?
if [ $ADAS_CFG_STATUS -ne 0 ]; then
  echo "$ADAS_CFG_LOG"
  echo "  ✗ Node plugin CMake configuration failed (exit $ADAS_CFG_STATUS)"
  exit 1
fi
# Surface any other configure-time warnings/errors too (not just the Eigen/
# Frenet fallback case handled separately below) — this log used to be
# swallowed entirely (> /dev/null), which meant genuine misconfigurations
# were undebuggable from demo.sh output.
ADAS_CFG_EXTRA_WARN=$(echo "$ADAS_CFG_LOG" | grep -i "warning\|error" || true)
if [ -n "$ADAS_CFG_EXTRA_WARN" ]; then
  echo "  ⚠ Node plugin CMake configuration reported warnings/errors:"
  echo "$ADAS_CFG_EXTRA_WARN" | sed 's/^/    /'
fi
if ! cmake --build "$BUILD_DIR/modules/adas_nodes" -j"$NPROC" 2>&1 | tail -1; then
  echo "  ✗ Build failed for node plugins — re-run without the trailing 'tail -1' filter"
  echo "    (cmake --build \"$BUILD_DIR/modules/adas_nodes\") to see the full error."
  exit 1
fi
set +o pipefail
echo "  ✓ Build complete"
# planning_node silently degrades to a lane-keep-only fallback (no overtaking /
# lane changes) when the Frenet planner isn't linked in (missing Eigen). Make
# this loud instead of a one-line cmake log nobody reads.
if echo "$ADAS_CFG_LOG" | grep -q "planning_node: building in fallback mode"; then
  echo ""
  echo "  ⚠ WARNING: Frenet planner NOT built (Eigen3 not found) — ego will"
  echo "    NEVER change lanes or overtake, even with a clear adjacent lane."
  echo "    Fix: sudo apt install libeigen3-dev, then re-run this script."
  echo ""
fi

# ── Cleanup handler ─────────────────────────────────────────
# Runs on normal exit AND on Ctrl+C (INT) / TERM. It must:
#   1. run at most once (INT → cleanup → exit → EXIT would otherwise re-enter),
#   2. never block forever (a plain `wait` hangs if a child ignores SIGTERM —
#      that is exactly why a "demo.sh" process was left lingering after Ctrl+C),
#   3. reap the whole process tree, including flow_node_host children that
#      flow_launcher fork+execs in --multi mode.
CLEANED_UP=false
cleanup() {
  $CLEANED_UP && return 0
  CLEANED_UP=true
  echo ""
  echo "───[Cleanup] Shutting down..."

  terminate_pids "${TAIL_BEH_PID:-}" "${LAUNCHER_PID:-}" "${BRIDGE_PID:-}" "${SERVER_PID:-}"
  if [ -f "$PID_FILE" ]; then
    terminate_recorded_pids
    rm -f "$PID_FILE"
  fi

  # 保留拓扑文件供评估器/evaluator 事后分析（不删除）
  [ -f "$JSON_FILE" ] && cp "$JSON_FILE" "${JSON_FILE%.json}_$(date +%Y%m%d_%H%M%S).json" 2>/dev/null || true
  # 退出游戏模式（flowsim 恢复正常 control_node 驱动）
  rm -f /tmp/game_mode /tmp/game_input.json 2>/dev/null || true
  cleanup_pipeline_tmp

  echo ""
  echo "  ╔══════════════════════════════════════╗"
  echo "  ║  Demo Complete — FlowEngine v2.0     ║"
  echo "  ║  Plugin Architecture                 ║"
  echo "  ║  github.com/caixuf/FlowEngine        ║"
  echo "  ╚══════════════════════════════════════╝"
  # Note: no explicit `exit` here — the INT/TERM traps below exit after us,
  # and the EXIT trap just unwinds (guarded so we only run once).
}
# On Ctrl+C / TERM we must actually terminate: without an explicit exit the
# interrupted `sleep` in the monitor loop would resume and keep the demo alive.
# Exit codes follow the shell convention 128+signal (SIGINT=2 → 130, SIGTERM=15 → 143).
trap 'cleanup' EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# ── Start flow_launcher with pipeline ───────────────────────
echo "───[2/4] Starting pipeline (flowsim→sensor_model→perception→fusion→planning→control→monitor)..."
rm -f "$JSON_FILE"
cd "$ROOT"  # run from root so build/lib/ paths resolve

LAUNCHER_ARGS=("$PIPELINE")
[ "$DURATION" -gt 0 ] 2>/dev/null && LAUNCHER_ARGS+=(--duration "$DURATION")
[ "$MULTI_MODE" = true ] && LAUNCHER_ARGS+=(--multi)
if [ "$RECORD_MODE" = true ]; then
  LAUNCHER_ARGS+=(--bag "$BAG_FILE")
  echo "  Recording to: $BAG_FILE"
fi
if [ "$GAME_MODE" = true ]; then
  # 游戏模式：flowsim 主循环读 /tmp/game_input.json 当控制指令（玩家键盘）
  touch /tmp/game_mode
  echo "  🎮 GAME MODE: 浏览器打开 http://localhost:8800/?game=1"
  echo "     方向键/WASD 开车（↑油门 ↓刹车 ←→转向，空格手刹）"
fi
[ "$MULTI_MODE" = true ] && echo "  Multi-process mode: each node runs as a separate process"

"$LAUNCHER_BIN" "${LAUNCHER_ARGS[@]}" \
  > /tmp/flow_launcher_stdout.txt 2>/tmp/flow_launcher_stderr.txt &
LAUNCHER_PID=$!
record_pid "$LAUNCHER_PID" flow_launcher
sleep 1
if ! kill -0 $LAUNCHER_PID 2>/dev/null; then
  echo "  ✗ Pipeline failed! Check /tmp/flow_launcher_stderr.txt"
  cat /tmp/flow_launcher_stderr.txt 2>/dev/null || true
  exit 1
fi
echo "  ✓ Pipeline running (PID $LAUNCHER_PID)"

# ── Start dashboard server (file bridge: read /tmp/flow_topology.json) ──
# 依据 VISUALIZATION_ARCHITECTURE.md: flowmond 拥有独立 MessageBus,
# 看不到 flow_launcher 进程内的数据, launch 演示必须用文件桥接。
echo "───[3/4] Starting dashboard..."
# Wait for monitor node to write first snapshot
for _ in $(seq 1 30); do
  if [ -s "$JSON_FILE" ]; then break; fi
  sleep 0.5
done
if [ ! -s "$JSON_FILE" ]; then
  echo "  ✗ Timeout waiting for $JSON_FILE — monitor node may have failed"
fi
"$BUILD_DIR/bin/flowmond" --port 8800 --html-path "$ROOT/tools/flowboard/index.html" \
  > /tmp/flowmond.log 2>&1 &
SERVER_PID=$!
record_pid "$SERVER_PID" flowmond
sleep 2
if kill -0 $SERVER_PID 2>/dev/null; then
    # Self-check: verify the server actually responds
    CHECK=$(curl -s --max-time 3 -o /dev/null -w '%{http_code}' http://127.0.0.1:8800/api/health 2>/dev/null || echo "000")
    if [ "$CHECK" = "200" ]; then
        echo "  ✓ Dashboard at http://localhost:8800"
    else
        echo "  ✗ Dashboard started but not responding (HTTP $CHECK)"
        echo "  Server log:"
        cat /tmp/flowmond.log
    fi
else
    echo "  ✗ flowmond failed! Check /tmp/flowmond.log"
    cat /tmp/flowmond.log
fi

python3 "$ROOT/tools/foxglove_bridge.py" --port 8765 --json-file "$JSON_FILE" \
  > /tmp/foxglove_bridge.log 2>&1 &
BRIDGE_PID=$!
record_pid "$BRIDGE_PID" foxglove_bridge
echo "  ✓ 3D Bridge at ws://localhost:8765 (Foxglove Studio)"

# ── Live monitor ────────────────────────────────────────────
echo "───[4/4] Live monitor (${DURATION}s)..."
echo ""

# 准备行为日志过滤（筛选 [BEH]、[SM] 和 [INV] 日志行）
BEH_LOG="/tmp/flow_beh_monitor.txt"
: > "$BEH_LOG"  # 清空
sleep 1  # 等日志文件就绪
# 旧实现是 `tail -F | grep &`：EXIT trap 只能杀到管道最后一个进程，
# macOS 又没有 GNU tail --pid，tail 孙进程会泄漏并持有评估器 stdout 管道。
# 用一个 Python watcher 轮询文件，cleanup 只需按 PID 杀一个进程。
python3 - "$LOG_DIR/launcher.log" /tmp/flow_launcher_stderr.txt "$BEH_LOG" <<'PY' &
import os
import re
import sys
import time

primary, fallback, out_path = sys.argv[1:4]
pat = re.compile(r"\[(BEH|SM|INV)\]")
path = None
fp = None
pos = 0

def select_path():
    return primary if os.path.exists(primary) else fallback

while True:
    wanted = select_path()
    try:
        st = os.stat(wanted)
    except OSError:
        time.sleep(0.2)
        continue
    if wanted != path or fp is None:
        if fp:
            fp.close()
        path = wanted
        fp = open(path, "r", encoding="utf-8", errors="replace")
        pos = 0
    if st.st_size < pos:
        fp.seek(0)
    for line in fp:
        if pat.search(line):
            with open(out_path, "a", encoding="utf-8") as out:
                out.write(line)
                out.flush()
    pos = fp.tell()
    time.sleep(0.2)
PY
TAIL_BEH_PID=$!
record_pid "$TAIL_BEH_PID" beh_log_watcher

echo "  ┌─ FlowSim ─→ Perception ─→ Behavior ─→ Planning ─→ Control ┐"
echo "  │  dynamics      DBSCAN        SM        Frenet       PID      │"
echo "  └──────────────────────────────────────────────────────────────┘"
echo ""

ELAPSED=0
BEH_LINECOUNT=0
# 持续运行: DURATION=0 时无限循环，直到脚本被 Ctrl+C 终止
while true; do
  # 限时模式: 达到时长后退出循环
  if [ "$DURATION" -gt 0 ] 2>/dev/null && [ $ELAPSED -ge $DURATION ]; then break; fi

  # ── 显示新增的 [BEH] / [SM] 日志行 ──
  if [ -f "$BEH_LOG" ]; then
    CURRENT_LC=$(wc -l < "$BEH_LOG" 2>/dev/null || echo 0)
    if [ "$CURRENT_LC" -gt "$BEH_LINECOUNT" ]; then
      # 显示新增的行（最多 5 行，避免刷屏）
      DISPLAY_LINES=$((CURRENT_LC - BEH_LINECOUNT))
      [ "$DISPLAY_LINES" -gt 5 ] && DISPLAY_LINES=5
      START_LINE=$((CURRENT_LC - DISPLAY_LINES + 1))
      [ "$START_LINE" -le 0 ] && START_LINE=1
      sed -n "${START_LINE},${CURRENT_LC}p" "$BEH_LOG" 2>/dev/null | while IFS= read -r line; do
        # 高亮：[BEH] 用青色，[SM] 用黄色
        if echo "$line" | grep -q '\[BEH\]'; then
          echo -e "\n  \033[36m$(echo "$line" | grep -oE '\[BEH\].*' || echo "$line")\033[0m"
        elif echo "$line" | grep -q '\[SM\]'; then
          echo -e "\n  \033[33m$(echo "$line" | grep -oE '\[SM\].*' || echo "$line")\033[0m"
        elif echo "$line" | grep -q '\[INV\]'; then
          echo -e "\n  \033[31m$(echo "$line" | grep -oE '\[INV\].*' || echo "$line")\033[0m"
        fi
      done
      BEH_LINECOUNT=$CURRENT_LC
    fi
  fi

  # ── 主状态行 ──
  if [ -f "$JSON_FILE" ]; then
    STATS=$(python3 -c "
import json
with open('$JSON_FILE') as f:
    d=json.load(f)
m=d.get('metrics',{})
b=m.get('bus',{})
l=m.get('latency',{})
v=m.get('vehicle',{})
beh=m.get('behavior',{})
state=beh.get('state','?')
lane=beh.get('committed_lane',-1)
print(f\"pub={b.get('published',0)} del={b.get('delivered',0)} lat={l.get('avg_us',0)}us speed={v.get('speed',0):.1f}m/s beh={state} lane={lane}\")
" 2>/dev/null)
    printf "\r  ⏱ %3ds  |  %s  " "$ELAPSED" "$STATS"
  else
    printf "\r  ⏱ %3ds  |  waiting for data..." "$ELAPSED"
  fi
  sleep 1
  ELAPSED=$((ELAPSED + 1))
done
echo ""
echo ""

# ── Print summary ───────────────────────────────────────────
echo "═══ Pipeline Summary ═══"

# Extract stats from per-module log files (fallback to old stderr file)
LAUNCHER_LOG="$LOG_DIR/launcher.log"
STDERR_LOG="/tmp/flow_launcher_stderr.txt"
STAT_SRC="${LAUNCHER_LOG}"
[ -f "$STAT_SRC" ] || STAT_SRC="$STDERR_LOG"

FUSED=$(grep -a "EKF" "$STAT_SRC" 2>/dev/null | tail -1 | grep -oE "#[0-9]+" | tail -1 | tr -d '#' || echo "0")
CTRL=$(grep -a "control.*#" "$STAT_SRC" 2>/dev/null | tail -1 | grep -oE "#[0-9]+" | tail -1 | tr -d '#' || echo "0")
PLAN=$(grep -a "planning.*#" "$STAT_SRC" 2>/dev/null | tail -1 | grep -oE "#[0-9]+" | tail -1 | tr -d '#' || echo "0")
SPEED=$(grep -a "flowsim.*stopped" "$STAT_SRC" 2>/dev/null | grep -oE "speed=[0-9.]+" | head -1 | cut -d= -f2 || echo "?")

echo "  Simulation : $SPEED m/s final speed"
echo "  Fusion     : $FUSED EKF frames"
echo "  Planning   : $PLAN trajectories"
echo "  Control    : $CTRL control cycles"
echo ""
echo "  Logs       : $LOG_DIR/{launcher,flowsim,planning,control,...}.log"

echo ""
echo "  Dashboard  : http://localhost:8800"
echo "  Topology   : tools/topology_viewer.html (standalone)"
if [ "$RECORD_MODE" = true ] && [ -f "$BAG_FILE" ]; then
  BAG_SIZE=$(du -h "$BAG_FILE" 2>/dev/null | cut -f1)
  echo "  Bag        : $BAG_FILE ($BAG_SIZE)"
fi
echo "  CI Status  : github.com/caixuf/FlowEngine/actions"
