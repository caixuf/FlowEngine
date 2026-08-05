#!/usr/bin/env bash
# smoke_closed_loop.sh — 学习闭环快速验证门禁（每次数据链路改动必跑）
#
# 一条命令验证「训练 → 三场景闭环」是否达标：
#   1. 用最近累积数据集训练 300 epochs（~2min）
#   2. 三场景闭环评估（cruise/lead/emergency，~1min）
#   3. 断言：cruise PASS + lead/emergency progress>0（会开）
#
# 防反复修：任何改动（互斥阈值/样本格式/刹车提前量）跑一次，绿了再提交。
#
# 用法:
#   bash tools/train_e2e/smoke_closed_loop.sh [dataset.jsonl] [model_out]
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

DATASET="${1:-runs/auto_train_1785913802/dataset.jsonl}"
OUT="${2:-/tmp/smoke_model}"
echo "=== smoke: $DATASET → $OUT ==="

# 1. 准备 ds 目录（滑动窗口 700，与 auto_train 一致）
rm -rf "$OUT"
mkdir -p "$OUT/ds"
tail -700 "$DATASET" > "$OUT/ds/samples.jsonl"
echo '{"feature_names": "v3", "schema_version": "flowengine.e2e_sample.v2"}' > "$OUT/ds/metadata.json"

# 2. 训练
echo "[train]"
python3 tools/train_e2e/train.py --dataset "$OUT/ds" --output "$OUT/model" \
    --hidden "64 32" --epochs 300 --early-stop 30 2>&1 | tail -2

# 3. 闭环评估
echo "[closed_loop]"
python3 tools/train_e2e/eval_closed_loop.py --model "$OUT/model/model.txt" 2>&1 | \
    grep -E "cruise|lead|emergency|总体" | head -6

# 4. 断言
echo "[gate]"
python3 - "$OUT/model/model.txt" <<'PYEOF'
import sys, json, subprocess
r = subprocess.run(["python3", "tools/train_e2e/eval_closed_loop.py", "--model", sys.argv[1]],
                   capture_output=True, text=True)
# 从 stdout 抓结果
import re
scenes = {}
for line in r.stdout.splitlines():
    m = re.match(r'(\w+)\s+(PASS|FAIL)\s+([\d.]+)\s+([\d.]+)', line)
    if m:
        scenes[m.group(1)] = {"result": m.group(2), "progress": float(m.group(3)),
                              "final_v": float(m.group(4))}
cruise_ok = scenes.get("cruise", {}).get("result") == "PASS"
lead_moves = scenes.get("lead", {}).get("progress", 0) > 20
emg_moves = scenes.get("emergency", {}).get("progress", 0) > 20
print(f"  cruise PASS: {cruise_ok}")
print(f"  lead 会开: {lead_moves} (progress={scenes.get('lead',{}).get('progress',0):.0f})")
print(f"  emergency 会开: {emg_moves} (progress={scenes.get('emergency',{}).get('progress',0):.0f})")
ok = cruise_ok and lead_moves and emg_moves
print(f"  {'✅ GATE PASS' if ok else '❌ GATE FAIL'}")
sys.exit(0 if ok else 1)
PYEOF
