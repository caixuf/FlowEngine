#!/usr/bin/env python3
"""synth_data.py — 合成训练样本生成器（IDM 规则，补真实采集的边界覆盖）

真实采集(planning 日常行为)天然缺少「高速逼近慢车/静止障碍」的急刹样本
（60s 采集 0 条近灯+强刹），模型学到的是「planning 日常」而非「安全底线」。
本生成器按 IDM 安全间距规则合成 (特征 → throttle/brake) 对，补进训练集。

关键约定（与 eval_closed_loop 执行端一致）：
  - throttle/brake 严格互斥（危险→刹车 0 油门；安全→油门 0 刹车）
  - front0_x 用世界坐标（与 data_recorder 一致：ego 在 x=0，障碍在 front_x）
  - 远处占位 = ego+500（无车时）

用法:
  python3 tools/train_e2e/synth_data.py --output /tmp/synth.jsonl --count 250
  # 网格密度: 速度×距离×前车速度
  python3 tools/train_e2e/synth_data.py --dense
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from feature_schema import build_v3_features  # noqa: E402


def make_sample(v: float, front_x: float, front_vx: float,
                ego_y: float = -1.75) -> dict:
    """构造一条 (特征, control) 样本: ego 在 x=0, 障碍在 front_x(世界坐标)。"""
    ego = {"x": 0.0, "y": ego_y, "v": v, "heading": 0.0, "yaw_rate": 0.0}
    obstacles = [{"x": front_x, "y": 0.0, "vx": front_vx, "vy": 0.0,
                  "type": 1.0, "confidence": 1.0, "length": 4.5, "width": 1.9}]
    gap = front_x - 2.25  # 相对距离（车头到障碍）
    dv = v - front_vx
    safe = 5.0 + v * 1.5  # IDM 安全间距
    # 严格互斥：危险→刹车(0 油门)；否则→油门(0 刹车)
    # 2026-08-05 提前减速：中距离(≤2×safe)就开始轻刹/收油 —— 模型只学
    # 「近距才刹」→ 40m 前车从 15.7m/s 刹不住（实测 lead 碰撞）。提前量
    # 按 TTC：gap/v_rel < 4s 就开始收油，< 2.5s 轻刹。
    ttc = gap / max(dv, 0.1)
    if gap < safe and dv > 0:
        brake = min(1.0, 0.4 + (safe - gap) * 0.15 + dv * 0.08)
        throttle = 0.0
    elif gap < 8:
        brake = min(1.0, 0.6 + (8 - gap) * 0.1)
        throttle = 0.0
    elif ttc < 2.5:  # 提前减速：TTC<2.5s 轻刹（40m@15.7→12m/s 撞前车前刹住）
        brake = min(1.0, 0.25 + (2.5 - ttc) * 0.15)
        throttle = 0.0
    elif ttc < 4.0:  # 收油不刹（预减速，planning 的 TTC 提前压速）
        throttle = min(0.5, max(0.0, 0.15 + (20 - v) * 0.02))
        brake = 0.0
    else:
        throttle = min(1.0, max(0.0, 0.25 + (20 - v) * 0.04))
        brake = 0.0
    control = {"throttle": throttle, "brake": brake, "steering": 0.0,
               "emergency_stop": False}
    scene = {"tl_state": -1.0, "tl_distance": -1.0, "curvature": 0.0,
             "speed_limit": 30.0, "lane_count": 4, "lane_width": 3.5,
             "ego_lane_offset": ego_y}
    feats = build_v3_features(ego, obstacles, control, None, scene, [])
    # 2026-08-05 样本同构：补 ego 字段（真实采集格式）。
    # 旧合成样本无 ego → build_windows 停滞过滤 ego_v=0 全被跳过 →
    # 训练集 throttle 非零仅 5% → 模型学「不踩油门」→ 闭环永不动。
    return {"features_v3": feats, "control": control, "label": 0.0,
            "ego": {"x": 0.0, "y": ego_y, "v": v, "heading": 0.0,
                    "yaw_rate": 0.0},
            "synthetic": True, "synth_v": v, "synth_gap": gap}


def gen_grid() -> list[dict]:
    """网格: 速度(5) × 距离(10) × 前车速度(5) = 250 条"""
    out = []
    # 2026-08-05 加 v=0 起步瞬态：模型没学「起步+前车该加速接近」→
    # lead/emergency 起步就刹死（progress=0 实测）。v=0 起步样本教
    # 「远前车(≥25m) → 油门起步，近前车 → 刹」。
    for v in [0, 3, 5, 8, 12, 16, 20]:
        for dist in [5, 8, 12, 18, 25, 35, 50, 80, 150, 300]:
            for fv in [0, 4, 8, 12, 16]:
                out.append(make_sample(v, dist, fv))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="合成训练样本生成器")
    ap.add_argument("--output", default="/tmp/synth.jsonl")
    ap.add_argument("--dense", action="store_true",
                    help="更密网格: 速度(9)×距离(14)×前车速(7)=882 条")
    args = ap.parse_args()

    if args.dense:
        samples = []
        for v in [3, 5, 8, 10, 12, 16, 18, 20, 24]:
            for dist in [3, 5, 8, 10, 12, 15, 18, 22, 28, 35, 45, 60, 100, 300]:
                for fv in [-2, 0, 3, 6, 10, 14, 18]:
                    samples.append(make_sample(v, dist, fv))
    else:
        samples = gen_grid()

    with open(args.output, "w", encoding="utf-8") as f:
        for s in samples:
            f.write(json.dumps(s) + "\n")
    print(f"合成样本: {len(samples)} 条 → {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
