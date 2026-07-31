#!/usr/bin/env python3
"""闭环学习版自动调参 — 支持 Stanley/PID 和 MPC 双模调优。

Stanley 模式 (--mode stanley, 默认):
  调参对象: lat_kp, lat_kd_heading, yaw_damping, k_v_lat
  闭环方式: flowctl param set 直接生效，无需 model.txt
  流程:
    1. 起一次长 demo
    2. flowctl param set 逐组注入参数
    3. 采样 /tmp/flow_topology.json 算 CTE/speed/slew
    4. 选最优参数，flowctl param set 固化为新 baseline

MPC 模式 (--mode mpc):
  调参对象: mpc_q_y, mpc_q_theta, mpc_r_a, mpc_r_ddelta
  闭环方式: 训练 model.txt → inference_node OTA 热重载
  流程:
    1. 起一次长 demo（不需要反复重启）
    2. flowctl param set 逐组注入参数
    3. 采样 /tmp/flow_topology.json 算 CTE/speed/slew
    4. 训练 model.txt（将最优参数 delta 固化到 tiny-MLP 权重）
    5. 复制到 learner save_path → inference_node OTA 热重载 → 闭环生效

用法:
    # Stanley 模式：扫描 lat_kd_heading
    python3 tools/auto_tune_mpc.py --mode stanley --param lat_kd_heading

    # Stanley 模式：指定取值
    python3 tools/auto_tune_mpc.py --mode stanley --param yaw_damping --values 0.1 0.2 0.3 0.4 0.5

    # MPC 模式：扫描 mpc_q_y
    python3 tools/auto_tune_mpc.py --mode mpc --param mpc_q_y --values 5 10 20 40 80

    # 多轮 zoom-in（自动逼近最优）
    python3 tools/auto_tune_mpc.py --mode stanley --param lat_kd_heading --rounds 3

    # 全自动自标定：多参数联合优化，零人工（真正的数据驱动）
    python3 tools/auto_tune_mpc.py --mode stanley --auto

    # 只评估当前参数
    python3 tools/auto_tune_mpc.py --eval-only

依赖: flowctl（需先编译好）、demo.sh（仿真）、/tmp/flow_topology.json（拓扑数据）
"""
import argparse
import json
import math
import os
import random
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PIPELINE = ROOT / "config" / "pipeline.json"
FLOWCTL = ROOT / "build" / "bin" / "flowctl"
JSON_FILE = Path("/tmp/flow_topology.json")
SOCK_FILE = Path("/tmp/flow_param.sock")

# ── 学习闭环参数（与 model.txt out_dim=9 中的索引 5-8 对应）───────
# (param_name, baseline_value, delta_scale, min_clamp, max_clamp, model_idx)
# MPC 权重（q_y/q_theta/r_a/r_ddelta），通过 inference_node → model.txt 闭环
MPC_PARAMS = [
    ("mpc_q_y",          10.0,  5.0,   0.1,   100.0, 5),
    ("mpc_q_theta",      100.0, 20.0,  1.0,   500.0, 6),
    ("mpc_r_a",           0.5,  0.2,   0.01,   10.0, 7),
    ("mpc_r_ddelta",      0.5,  0.2,   0.01,   10.0, 8),
]

# Stanley/PID 横向控制参数，直接通过 flowctl param set 闭环
# (param_name, baseline_value, delta_scale, min_clamp, max_clamp, _unused)
STANLEY_PARAMS = [
    ("lat_kp",            0.5,  0.1,   0.15,   1.0,   0),
    ("lat_kd_heading",    3.5,  0.5,   1.0,    6.0,   0),
    ("yaw_damping",       0.3,  0.05,  0.05,   0.6,   0),
    ("k_v_lat",           0.4,  0.1,   0.1,    1.0,   0),
]

MODEL_PATH = ROOT / "tools" / "train" / "model.txt"
LEARNER_SAVE_PATH = Path("/tmp/flow_learner_model.txt")


# ── 工具函数 ────────────────────────────────────────────────────

def flowctl(*args: str) -> tuple[bool, str]:
    """调用 flowctl，返回 (成功?, 输出文本)。"""
    try:
        r = subprocess.run([str(FLOWCTL)] + list(args),
                           capture_output=True, text=True, timeout=10)
        ok = r.returncode == 0
        return ok, (r.stdout + r.stderr).strip()
    except Exception as e:
        return False, str(e)


def sample_json() -> dict | None:
    """读取 /tmp/flow_topology.json 的 metrics.scene.ego。"""
    try:
        if JSON_FILE.stat().st_size == 0:
            return None
        with open(JSON_FILE) as f:
            d = json.load(f)
        ego = d.get("metrics", {}).get("scene", {}).get("ego", {})
        if not ego or not ego.get("x"):
            return None
        return {
            "t": d.get("timestamp", 0),
            "x": ego.get("x", 0),
            "y": ego.get("y", 0),
            "heading": ego.get("heading", 0),
            "speed": ego.get("speed", 0),
            "steer": ego.get("steer", 0),
            "vx": ego.get("vx", 0),
            "vy": ego.get("vy", 0),
        }
    except (json.JSONDecodeError, OSError, KeyError):
        return None


def compute_cte(y: float, lane_count: int = 4,
                lane_width: float = 3.5, road_c: float = 0.0) -> float:
    """cross-track error 相对最近车道中心。

    默认 4 车道（与 straight_road.json 一致）。2 车道假设已过时，
    会导致 lane 3（y=-5.25）的 CTE 算成 3.5m 而非 ~0m。
    """
    offset = (road_c - y) / lane_width + (lane_count - 1) * 0.5
    idx = max(0, min(lane_count - 1, int(round(offset))))
    lcy = road_c - (idx - (lane_count - 1) * 0.5) * lane_width
    return y - lcy


def sample_series(duration_s: float, interval: float = 0.2) -> list[dict]:
    """采样 duration_s 秒的 JSON 数据。"""
    samples = []
    t0 = time.monotonic()
    deadline = t0 + duration_s + 2.0
    while time.monotonic() < deadline:
        s = sample_json()
        if s:
            samples.append(s)
        time.sleep(interval)
    return samples


def compute_reward(samples: list[dict]) -> dict:
    """从采样数据算 reward 指标。

    改进（2026-07 P3）：
      - 使用 CTE 95% 分位数（抗离群点）
      - 使用 steer slew rate 替代零交叉率（更公平的振荡指标）
      - 增加速度标准差检测振荡
      - 连续评分（无硬阈值跳变）
    """
    if not samples:
        return {"score": -999, "cte_rms": 999, "cte_95": 999, "max_cte": 999,
                "avg_speed": 0, "speed_std": 999, "slew_rate": 999,
                "flip_rate": 999, "n": 0}

    ctes = [compute_cte(s["y"]) for s in samples]
    speeds = [s["speed"] for s in samples]
    steers = [s["steer"] for s in samples]
    n = len(samples)
    if n < 3:
        return {"score": -999, "cte_rms": 999, "cte_95": 999, "max_cte": 999,
                "avg_speed": 0, "speed_std": 999, "slew_rate": 999,
                "flip_rate": 999, "n": n}

    # --- CTE 指标 ---
    cte_abs = [abs(c) for c in ctes]
    cte_abs.sort()
    cte_rms  = math.sqrt(statistics.fmean([c * c for c in ctes]))
    cte_95   = cte_abs[int(n * 0.95)]        # 95% 分位，抗单次离群点
    max_cte  = cte_abs[-1]

    # --- 速度指标 ---
    avg_speed  = statistics.mean(speeds)
    speed_std  = statistics.stdev(speeds) if n > 1 else 0.0

    # --- steer jitter：slew rate（平均转向角速率），比零交叉率更鲁棒 ---
    # 累计所有相邻帧的 |Δsteer| / Δt，平均
    deltas = [abs(steers[i] - steers[i-1]) for i in range(1, n)]
    duration = (samples[-1]["t"] - samples[0]["t"]) if n > 1 else 1.0
    slew_rate = sum(deltas) / max(duration, 0.1) if deltas else 0.0

    # 零交叉率仍保留作为辅助指标
    flips = sum(1 for i in range(1, len(steers))
                if steers[i-1] * steers[i] < 0 and abs(steers[i] - steers[i-1]) > 0.01)
    flip_rate = flips / max(duration, 0.1)

    # --- 综合评分（连续函数，越大越好）---
    score = 0.0

    # 1) 速度评分：目标 12 m/s，低于 6 重罚，高于 14 不奖励（超速）
    target_speed = 12.0
    if avg_speed > 6.0:
        speed_score = (1.0 - (avg_speed - target_speed)**2 / target_speed**2) * 50
        speed_score = max(0, speed_score)
    else:
        speed_score = -30.0  # 低于 6m/s 严重倒扣
    score += speed_score

    # 2) CTE 评分：指数衰减，0.2m → 接近满分，1.0m → ~-20
    #    score = 40 * exp(-cte_rms / 0.3) - 10 * exp(cte_95 / 1.0) ...
    cte_score = 40.0 * math.exp(-cte_rms / 0.3) - 15.0 * (cte_95 / 1.0)
    score += cte_score

    # max_cte 不能出车道：>3m 硬罚
    if max_cte > 3.0:
        score -= 80.0 * min((max_cte / 3.0), 3.0)

    # 3) 振荡惩罚：slew_rate > 0.3 rad/s 开始扣分
    slew_threshold = 0.3
    if slew_rate > slew_threshold:
        score -= (slew_rate - slew_threshold) * 40.0
    elif slew_rate < 0.15:
        score += 5.0  # 平滑转向奖励

    # 速度振荡（speed_std > 1.5 m/s 惩罚）
    if speed_std > 1.5:
        score -= (speed_std - 1.5) * 10.0

    return {
        "score": score,
        "cte_rms": cte_rms,
        "cte_95": cte_95,
        "max_cte": max_cte,
        "avg_speed": avg_speed,
        "speed_std": speed_std,
        "slew_rate": slew_rate,
        "flip_rate": flip_rate,
        "n": n,
    }


def wait_for_flowctl(timeout: float = 30) -> bool:
    """等 flow_launcher 启动，flow_param.sock 就绪。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if SOCK_FILE.exists():
            ok, out = flowctl("param", "get", "control.k_v_lat")
            if ok:
                return True
        time.sleep(0.5)
    return False


# ── 模型训练 ────────────────────────────────────────────────────

def read_model_template() -> list[str]:
    """读取当前 model.txt 的头部模板（保留网络结构）。"""
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"{MODEL_PATH} not found")
    with open(MODEL_PATH) as f:
        return f.readlines()


def write_model(lines: list[str], out_dir: Path, name: str):
    """写 model.txt（保留模板，只改权重）。"""
    path = out_dir / name
    with open(path, "w") as f:
        f.writelines(lines)
    return path


def train_model(results: list[dict], out_path: Path, learn_params: list):
    """用扫参结果训练 model.txt。

    策略：找到每组参数的 reward，选最优参数组合的 delta 作为训练目标。
    model.txt 的 w2/b2（输出层）被训练成输出固定的最佳 delta。
    这相当于一个「常值策略」——后续 learner_node 会在此基础上做状态相关微调。
    """
    if not results:
        print("  ✗ 无扫参数据，跳过训练")
        return False

    # 找最优
    best = max(results, key=lambda r: r["reward"]["score"])
    best_params = best["params"]

    # 计算相对 baseline 的 delta
    deltas = {}
    for name, baseline, scale, _, _, _ in learn_params:
        if name in best_params:
            delta = (best_params[name] - baseline) / scale  # 归一化到 scale
            delta = max(-2.0, min(2.0, delta))  # 钳位
            deltas[name] = delta
        else:
            deltas[name] = 0.0

    print(f"\n  训练目标 delta: {deltas}")

    # 读取模板
    lines = read_model_template()
    # 保留结构行，替换 w2/b2（输出层）
    # w2: out_dim × hid_dim = 9×16 = 144 个浮点
    # b2: out_dim = 9 个浮点
    # 只改索引 5-8（control delta），其余保持 0

    new_lines = []
    in_w2 = False
    in_b2 = False
    w2_row = 0
    b2_idx = 0

    # 构建新的 w2 行（9 行 × 16 列）
    w2_rows = []
    for i in range(9):
        row = [0.0] * 16
        if 5 <= i <= 8:
            param_idx = i - 5
            name = learn_params[param_idx][0]
            row[0] = deltas.get(name, 0.0)  # 第一列存 delta
        w2_rows.append(row)

    # 构建新的 b2（9 个值，只改 5-8）
    b2_vals = [0.0] * 9
    for i in range(5, 9):
        param_idx = i - 5
        name = learn_params[param_idx][0]
        b2_vals[i] = deltas.get(name, 0.0) * 0.1  # bias 小一点

    # 逐行替换
    for line in lines:
        stripped = line.strip()

        if stripped == "w2":
            in_w2 = True
            in_b2 = False
            new_lines.append(line)
            # 追加新的 w2 行
            for row in w2_rows:
                new_lines.append(" " + " ".join(f"{v:.6f}" for v in row) + "\n")
            continue

        if stripped == "b2":
            in_w2 = False
            in_b2 = True
            new_lines.append(line)
            new_lines.append(" " + " ".join(f"{v:.6f}" for v in b2_vals) + "\n")
            continue

        # 跳过 w2/b2 的旧值行（已被替换）
        if in_w2:
            w2_row += 1
            if w2_row >= 9:
                in_w2 = False
                w2_row = 0
            continue
        if in_b2:
            b2_idx += 1
            if b2_idx >= 1:  # b2 只有一行
                in_b2 = False
                b2_idx = 0
            continue

        new_lines.append(line)

    # 写文件
    with open(out_path, "w") as f:
        f.writelines(new_lines)

    print(f"  ✓ 模型已训练: {out_path}")
    print(f"    输出层 w2[5..8][0] = {[w2_rows[i][0] for i in range(5,9)]}")
    print(f"    输出层 b2[5..8]    = {b2_vals[5:9]}")
    return True


# ── 全自动自标定 ────────────────────────────────────────────────

def auto_tune_stanley(args, log_dir: Path):
    """多参数联合随机搜索 + 自适应收缩。

    算法：
      1. 以 baseline 为中心，初始化各参数搜索范围 [center - half, center + half]
      2. 每轮随机采样一组参数，全部通过 flowctl 注入
      3. 评估 reward，如果更好则更新 best
      4. 将搜索中心向 best 移动，搜索半径减半
      5. 重复 args.auto_iter 次

    特点：
      - 4 个参数联合优化，捕获参数间耦合
      - 搜索空间自动收缩，从粗到细
      - 零人工干预：只需指定 --auto
    """
    # 参数定义: (name, center, lo, hi)
    param_defs = [
        ("lat_kp",          0.5,  0.15, 1.0),
        ("lat_kd_heading",  3.5,  1.0,  6.0),
        ("yaw_damping",     0.3,  0.05, 0.6),
        ("k_v_lat",         0.4,  0.1,  1.0),
    ]
    # 全局硬边界快速查找
    hard_bounds = {name: (lo, hi) for name, _, lo, hi in param_defs}

    # 初始搜索半径 = 全局范围的 30%
    bounds = {}
    for name, center, lo, hi in param_defs:
        half = (hi - lo) * 0.3
        bounds[name] = {
            "lo": max(lo, center - half),
            "hi": min(hi, center + half),
            "best": center,
            "center": center,
        }

    best_score = -float("inf")
    best_params = {name: center for name, center, _, _ in param_defs}

    print(f"\n  ╔══════════════════════════════════════════════════════════╗")
    print(f"  ║  全自动自标定  {args.auto_iter} 轮迭代                      ║")
    print(f"  ║  参数: lat_kp, lat_kd_heading, yaw_damping, k_v_lat      ║")
    print(f"  ╚══════════════════════════════════════════════════════════╝")

    all_results = []

    for iteration in range(args.auto_iter):
        # 随机采样一组参数
        trial = {}
        desc_parts = []
        for name, center, lo, hi in param_defs:
            b = bounds[name]
            # 搜索范围随迭代收缩
            shrink = max(0.05, 1.0 - iteration / args.auto_iter)
            half = (b["hi"] - b["lo"]) * 0.5 * shrink
            mid = b["center"]
            lo_val = max(b["lo"], mid - half)
            hi_val = min(b["hi"], mid + half)
            val = random.uniform(lo_val, hi_val)
            trial[name] = val
            desc_parts.append(f"{name}={val:.3f}")

        desc = " ".join(desc_parts)
        print(f"\n  [iter {iteration+1:2d}/{args.auto_iter}] 采样: {desc}")

        # 注入所有参数
        for name, val in trial.items():
            ok, _ = flowctl("param", "set", f"control.{name}", str(val))
            if not ok:
                print(f"    ✗ flowctl set {name} 失败")
                continue

        # 稳定 + 评估
        time.sleep(args.stabilize)
        samples = sample_series(args.eval_s)
        reward = compute_reward(samples)

        result = {
            "params": trial,
            "reward": reward,
            "iteration": iteration,
        }
        all_results.append(result)

        print(f"    score={reward['score']:>7.1f}  "
              f"cte_rms={reward['cte_rms']:.3f}m  "
              f"cte95={reward['cte_95']:.3f}m  "
              f"speed={reward['avg_speed']:.1f}m/s  "
              f"slew={reward['slew_rate']:.3f}rad/s  "
              f"n={reward['n']}")

        # 更新最优
        if reward["score"] > best_score:
            best_score = reward["score"]
            best_params = trial.copy()
            print(f"    ★ 新最优! score={best_score:.1f}")

        # 自适应收缩：向 best 方向移动搜索中心
        for name, _, _, _ in param_defs:
            b = bounds[name]
            lo, hi = hard_bounds[name]
            # 朝 best 移动 30%
            b["center"] = b["center"] * 0.7 + best_params[name] * 0.3
            # 搜索半径缩减
            current_half = (b["hi"] - b["lo"]) * 0.5
            new_half = current_half * 0.85
            b["lo"] = max(lo, b["center"] - new_half)
            b["hi"] = min(hi, b["center"] + new_half)
            bounds[name] = b

    # 写入最优参数
    print(f"\n  ═══ 自标定完成 ═══")
    print(f"  最优参数: ", end="")
    for name in [p[0] for p in param_defs]:
        print(f"{name}={best_params[name]:.4g}  ", end="")
    print(f"\n  最优 score={best_score:.1f}")

    # 应用最优参数（固化到系统）
    for name, val in best_params.items():
        flowctl("param", "set", f"control.{name}", str(val))
    print(f"  ✓ 最优参数已固化到 flowctl param set")

    # 写汇总
    summary_path = log_dir / "auto_tune_summary.json"
    summary = {
        "mode": "stanley_auto",
        "iterations": args.auto_iter,
        "best_params": best_params,
        "best_score": best_score,
        "runs": [
            {"iter": r["iteration"], "params": r["params"], **r["reward"]}
            for r in all_results
        ],
    }
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"  汇总: {summary_path}")

    return best_params, best_score


# ── 主循环 ──────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="闭环学习自动调参（MPC / Stanley 双模）")
    ap.add_argument("--mode", default="stanley", choices=["stanley", "mpc"],
                    help="调参模式: stanley (PID+Stanley 参数) 或 mpc (MPC 权重)")
    ap.add_argument("--param", default="lat_kd_heading",
                    help="扫描的参数名 (default: lat_kd_heading)")
    ap.add_argument("--values", nargs="+", type=float,
                    default=None,
                    help="单轮扫描参数取值列表 (default: 按模式自动选择)")
    ap.add_argument("--rounds", type=int, default=1,
                    help="迭代轮数 (1=单次, >1=逐轮zoom-in逼近最优)")
    ap.add_argument("--n-values", type=int, default=5,
                    help="每轮扫描取值个数（rounds>1 时自动生成）")
    ap.add_argument("--duration", type=int, default=0,
                    help="demo 总时长 (s, 0=自动按 rounds*n_values 计算)")
    ap.add_argument("--stabilize", type=float, default=10.0,
                    help="每次调参后稳定等待 (s)")
    ap.add_argument("--eval-s", type=float, default=30.0,
                    help="每组合评估采样时长 (s)")
    ap.add_argument("--eval-only", action="store_true",
                    help="只评估当前参数，不训练模型")
    ap.add_argument("--auto", action="store_true",
                    help="全自动自标定：多参数联合随机搜索 + 自适应收缩")
    ap.add_argument("--auto-iter", type=int, default=20,
                    help="--auto 模式下的迭代次数 (default: 20)")
    ap.add_argument("--sweep", type=int, default=0,
                    help="多参数网格采样轮数 (0=单参数扫描)")
    ap.add_argument("--log-dir", default="/tmp/auto_tune_mpc",
                    help="日志目录")
    args = ap.parse_args()

    # 选择参数集
    if args.mode == "stanley":
        LEARN_PARAMS = STANLEY_PARAMS
    else:
        LEARN_PARAMS = MPC_PARAMS

    # 默认 values 按模式选择
    if args.values is None:
        if args.mode == "stanley":
            # 根据参数名给出合理的默认扫描范围
            defaults = {
                "lat_kp":         [0.25, 0.35, 0.5, 0.65, 0.8],
                "lat_kd_heading": [1.5, 2.5, 3.5, 4.5, 5.5],
                "yaw_damping":    [0.1, 0.2, 0.3, 0.4, 0.5],
                "k_v_lat":        [0.15, 0.3, 0.4, 0.6, 0.8],
            }
            args.values = defaults.get(args.param, [0.5, 1.0, 2.0, 4.0, 8.0])
        else:
            args.values = [5.0, 10.0, 20.0, 40.0, 80.0]

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    # ── 全自动自标定模式 ──────────────────────────────────────
    if args.auto:
        if args.mode != "stanley":
            print("  --auto 目前仅支持 --mode stanley")
            return 1

        # 杀残留
        subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond; "
                       "rm -f /tmp/flow_param.sock; sleep 1",
                       shell=True, capture_output=True, timeout=10)

        # 自动计算 demo 时长
        auto_duration = int(args.auto_iter * (args.stabilize + args.eval_s) * 1.3 + 30)
        if args.duration == 0:
            args.duration = auto_duration
            print(f"  自动计算 demo 时长: {args.duration}s ({args.auto_iter} 轮评估)")

        print(f"\n  起仿真 ({args.duration}s)...")
        subprocess.Popen(
            ["setsid", "bash", str(ROOT / "scripts" / "demo.sh"),
             "--no-browser", str(args.duration)],
            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if not wait_for_flowctl(45):
            print("  ✗ flow_launcher 未就绪")
            return 1

        # Stanley 模式：禁用 MPC
        ok, _ = flowctl("param", "set", "control.mpc_horizon", "0")
        print(f"  MPC 禁用 (horizon=0) {'✓' if ok else '✗'}")
        time.sleep(1.0)

        best_params, best_score = auto_tune_stanley(args, log_dir)

        # cleanup
        subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond",
                       shell=True, capture_output=True, timeout=5)

        return 0 if best_score > -100 else 1

    # ── 单参数扫描 / 评估模式 ─────────────────────────────────

    # 杀残留
    subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond; "
                   "rm -f /tmp/flow_param.sock; sleep 1",
                   shell=True, capture_output=True, timeout=10)

    # 起 demo
    print(f"\n  起仿真 ({args.duration}s)...")
    subprocess.Popen(
        ["setsid", "bash", str(ROOT / "scripts" / "demo.sh"),
         "--no-browser", str(args.duration)],
        cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not wait_for_flowctl(45):
        print("  ✗ flow_launcher 未就绪")
        return 1

    # 按模式设置 MPC 开关
    if args.mode == "stanley":
        ok, out = flowctl("param", "set", "control.mpc_horizon", "0")
        print(f"  Stanley 模式: MPC 禁用 (horizon=0) {'✓' if ok else '✗'}")
    else:
        ok, out = flowctl("param", "set", "control.mpc_horizon", "20")
        print(f"  MPC 模式: horizon=20 {'✓' if ok else '✗'}")
        ok, out = flowctl("param", "set", "control.mpc_dt", "0.1")
        print(f"  MPC dt=0.1 {'✓' if ok else '✗'}")
    time.sleep(1.0)

    # 读 baseline
    ok, baseline_str = flowctl("param", "get", f"control.{args.param}")
    baseline = float(baseline_str.split("=")[-1].strip()) if ok else 10.0
    print(f"  baseline {args.param} = {baseline}")

    # ── 构建各轮取值列表 ──────────────────────────────────────
    # 策略: round 0 用 --values 全范围扫描, 后续逐轮 zoom-in
    round_values = {}  # round_idx -> sorted list of values
    if args.rounds == 1:
        round_values[0] = sorted(args.values)
    else:
        init_vals = sorted(args.values)
        half_range = (init_vals[-1] - init_vals[0]) / 2.0
        center = init_vals[len(init_vals) // 2]  # 取中间值作为初始中心
        round_values[0] = init_vals
        for r in range(1, args.rounds):
            half_range *= 0.5
            # 在 [center - half_range, center + half_range] 均匀采样 args.n_values 个点
            vals = []
            for i in range(args.n_values):
                t = 2.0 * i / (args.n_values - 1) - 1.0  # [-1, 1]
                vals.append(center + half_range * t)
            round_values[r] = sorted(vals)

    # 自动计算 demo 时长
    n_total = sum(len(v) for v in round_values.values())
    total_eval_time = n_total * (args.stabilize + args.eval_s) * 1.3 + 30
    if args.duration == 0:
        args.duration = int(total_eval_time)
        print(f"  自动计算 demo 时长: {args.duration}s ({n_total} 组评估)")

    # ── 多轮迭代扫描 ──────────────────────────────────────────
    all_results = []
    current_best_val = baseline

    for round_idx in range(args.rounds):
        vals = round_values[round_idx]
        print(f"\n  ╔══════════════════════════════════════════════════╗")
        print(f"  ║  Round {round_idx+1}/{args.rounds}  扫描 {len(vals)} 个值  [{vals[0]:.3g} ~ {vals[-1]:.3g}]  ║")
        print(f"  ╚══════════════════════════════════════════════════╝")

        for i, val in enumerate(vals):
            ok, _ = flowctl("param", "set", f"control.{args.param}", str(val))
            if not ok:
                print(f"  [R{round_idx+1}/{i+1}] flowctl set 失败, 跳过")
                continue

            time.sleep(args.stabilize)
            samples = sample_series(args.eval_s)
            reward = compute_reward(samples)

            result = {
                "params": {args.param: val},
                "reward": reward,
                "samples": samples,
                "round": round_idx,
            }
            all_results.append(result)

            print(f"  [R{round_idx+1}/{i+1}/{len(vals)}] {args.param}={val:.4g}  "
                  f"score={reward['score']:>7.1f}  "
                  f"cte_rms={reward['cte_rms']:.3f}m  "
                  f"cte95={reward['cte_95']:.3f}m  "
                  f"speed={reward['avg_speed']:.1f}m/s  "
                  f"slew={reward['slew_rate']:.3f}rad/s  "
                  f"n={reward['n']}")

        # 本轮排序 + 最优
        round_results = [r for r in all_results if r["round"] == round_idx]
        round_results.sort(key=lambda r: r["reward"]["score"], reverse=True)
        round_best = round_results[0] if round_results else None

        if round_best:
            bv = round_best["params"][args.param]
            print(f"  Round {round_idx+1} 最优: {args.param}={bv:.4g}  "
                  f"score={round_best['reward']['score']:.1f}  "
                  f"cte_rms={round_best['reward']['cte_rms']:.3f}m  "
                  f"slew={round_best['reward']['slew_rate']:.3f}rad/s")
            current_best_val = bv

        # 最后一轮不调参（后面直接训练）
        if round_idx < args.rounds - 1:
            # 把参数设为当前最优作为下一轮的 baseline/中心
            flowctl("param", "set", f"control.{args.param}", str(current_best_val))
            print(f"  → 设置中心为 {args.param}={current_best_val:.4g}，准备下一轮 zoom-in")
            time.sleep(args.stabilize)

    # ── 全局最优 ──────────────────────────────────────────────
    all_results.sort(key=lambda r: r["reward"]["score"], reverse=True)
    best = all_results[0] if all_results else None
    if best:
        print(f"\n  ═══ 全局最优 (跨越 {args.rounds} 轮) ═══")
        print(f"  {args.param}={best['params'][args.param]:.4g}  "
              f"score={best['reward']['score']:.1f}  "
              f"cte_rms={best['reward']['cte_rms']:.3f}m  "
              f"cte95={best['reward']['cte_95']:.3f}m  "
              f"speed={best['reward']['avg_speed']:.1f}m/s  "
              f"slew={best['reward']['slew_rate']:.3f}rad/s")

    # 写汇总
    summary_path = log_dir / "live_summary.json"
    summary = {
        "param": args.param,
        "baseline": baseline,
        "rounds": args.rounds,
        "best_value": best["params"][args.param] if best else None,
        "best_score": best["reward"]["score"] if best else None,
        "runs": [
            {"round": r.get("round", 0), "value": r["params"][args.param], **r["reward"]}
            for r in all_results
        ],
    }
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"  汇总: {summary_path}")

    # ── Phase 2: 训练/应用 ────────────────────────────────────
    if not args.eval_only and best:
        if args.mode == "stanley":
            # Stanley 模式：直接 flowctl param set 应用最优参数，无需 model.txt
            best_val = best["params"][args.param]
            ok, _ = flowctl("param", "set", f"control.{args.param}", str(best_val))
            if ok:
                print(f"\n  ── Stanley 闭环就绪 ──")
                print(f"  ✓ 已应用最优参数: {args.param} = {best_val:.4g}")
                print(f"  baseline={baseline:.4g} → best={best_val:.4g}  "
                      f"(delta={best_val - baseline:+.4g})")
                print(f"  score={best['reward']['score']:.1f}  "
                      f"cte_rms={best['reward']['cte_rms']:.3f}m  "
                      f"slew={best['reward']['slew_rate']:.3f}rad/s")
                print(f"  (下次启动 demo 自动生效，或已实时生效)")
            else:
                print(f"  ✗ flowctl set 失败")
        else:
            # MPC 模式：训练 model.txt → OTA 热重载（原有逻辑）
            train_results = []
            for r in all_results:
                params = {}
                for name, bl, _, _, _, _ in LEARN_PARAMS:
                    if name == args.param:
                        params[name] = r["params"][args.param]
                    else:
                        params[name] = bl
                train_results.append({
                    "params": params,
                    "reward": r["reward"],
                })

            if train_model(train_results, MODEL_PATH, LEARN_PARAMS):
                if LEARNER_SAVE_PATH.exists():
                    LEARNER_SAVE_PATH.unlink()
                shutil.copy(MODEL_PATH, LEARNER_SAVE_PATH)
                print(f"  ✓ 模型已复制到 {LEARNER_SAVE_PATH}（OTA 热重载就绪）")

                ok, out = flowctl("param", "set", "inference.model_path",
                                  str(MODEL_PATH))
                if ok:
                    print(f"  ✓ OTA reload 触发")

                delta = best["params"][args.param] - baseline
                print(f"\n  ── MPC 闭环就绪 ──")
                print(f"  模型输出 delta = {delta:.4f}")
                print(f"  应用后 {args.param} = {baseline} + {delta:.4f} = "
                      f"{baseline + delta:.4f}")

    # cleanup
    subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond",
                   shell=True, capture_output=True, timeout=5)

    return 0 if best and best["reward"]["score"] > -100 else 1


if __name__ == "__main__":
    sys.exit(main())
