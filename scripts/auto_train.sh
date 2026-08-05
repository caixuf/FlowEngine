#!/usr/bin/env bash
# auto_train.sh — 学习闭环后台持续训练启动脚本
#
# 用法:
#   bash scripts/auto_train.sh [轮数] [每轮采集秒数]
#   bash scripts/auto_train.sh 20 60          # 20 轮, 每轮采集 60s
#   bash scripts/auto_train.sh 10 45 straight_road,multi_light
#
# 每轮: 采集(场景轮换) → 合成补样本 → 训练 → 闭环评估 → (达标)影子评估+promote
# 输出: runs/auto_train_<ts>/  (summary.jsonl 每轮记录)
# 最佳模型: models/auto_train_best/  (闭环三场景全 PASS 才更新)
#
# 查看进度:
#   tail -f /tmp/auto_train.log
#   cat runs/auto_train_*/summary.jsonl
#
# 挂后台示例:
#   nohup bash scripts/auto_train.sh 20 60 > /tmp/auto_train.log 2>&1 &
#   disown

set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUNDS="${1:-10}"
DURATION="${2:-60}"
SCENARIOS="${3:-straight_road,multi_light,lane_change_traffic}"

cd "$ROOT"
echo "[auto_train] rounds=$ROUNDS duration=${DURATION}s scenarios=$SCENARIOS"
python3 tools/train_e2e/auto_train_loop.py \
    --rounds "$ROUNDS" \
    --collect-duration "$DURATION" \
    --scenarios "$SCENARIOS"
