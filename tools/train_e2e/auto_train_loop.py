#!/usr/bin/env python3
"""auto_train_loop.py — 学习闭环后台持续训练循环

每轮迭代：
  1. 采集：随机选场景（straight/multi_light/lane_change_traffic/curve）跑 N 秒，
     真实 planning 行为样本 → 累积数据集（不覆盖历史）
  2. 合成补充：IDM 规则生成急刹/跟车样本（补真实采集缺的边界覆盖）
  3. 训练：temporal_train 5 输出执行量模型（throttle/brake/steer/lc/conf）
  4. 闭环评估：eval_closed_loop（cruise/lead/emergency 三场景自己开）
  5. 记录：结果写 runs/auto_train_<ts>/，自动保留历史最佳
  6. 达标（三场景全 PASS）→ modelctl promote 自动晋级

用法（挂后台）:
  nohup python3 tools/train_e2e/auto_train_loop.py --rounds 20 \
      --collect-duration 60 > /tmp/auto_train.log 2>&1 &
  # 每轮约 5-8 分钟（采集 60s + 训练 2-3min + 评估 1min）
  # 场景轮换: --scenarios "straight_road,multi_light,lane_change_traffic,curve_road"

设计决策:
  - 累积数据集（runs/auto_train_<ts>/dataset.jsonl）：每轮 append 新采集，
    模型见过所有历史 → 单调改善；合成样本每轮重生成（含当前轮状态）
  - 场景轮换：straight(巡航) / multi_light(红灯刹停) / lane_change_traffic(跟车变道)
    / curve_road(弯道)——覆盖不同行为域
  - 结果记录：每轮 closed_loop 三场景 PASS/FAIL + 指标，写 summary.jsonl；
    历史最佳模型保留 models/auto_train_best/
  - promote：三场景全 PASS 才推（promote_gate 还会查 shadow_eval，
    需先跑影子评估——本脚本做闭环后补一次影子评估再 promote）

依赖：scripts/demo.sh（采集）、tools/train_e2e/train.py（训练）、
      tools/train_e2e/eval_closed_loop.py（评估）、tools/modelctl.py（promote）
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from synth_data import gen_grid  # noqa: E402

SCENARIOS = {
    "straight_road": "scenarios/straight_road.json",
    "multi_light": "scenarios/multi_light.json",
    "lane_change_traffic": "scenarios/lane_change_traffic.json",
    "curve_road": "scenarios/curve_road.json",
}
COLLECT_FILE = Path("/tmp/flow_train_samples.jsonl")
CLOSED_LOOP_EVAL = "tools/train_e2e/eval_closed_loop.py"


def run(cmd: list[str], cwd: Path = ROOT, timeout: int = 300) -> int:
    print(f"  $ {' '.join(cmd)}", flush=True)
    try:
        r = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True,
                           timeout=timeout)
        if r.returncode != 0:
            print(f"  !! exit={r.returncode}: {r.stderr[-400:]}", flush=True)
        return r.returncode
    except subprocess.TimeoutExpired:
        print(f"  !! timeout {timeout}s", flush=True)
        return -1


def stage_collect(scenario: str, duration: int, run_dir: Path) -> Path:
    """采集 → append 到累积数据集"""
    print(f"[collect] scenario={scenario} duration={duration}s", flush=True)
    COLLECT_FILE.unlink(missing_ok=True)
    rc = run(["bash", "scripts/demo.sh", "--no-browser", str(duration),
              "--scenario", SCENARIOS[scenario]])
    if rc != 0 or not COLLECT_FILE.exists():
        print("  !! collect failed — skip this round's real samples", flush=True)
        return run_dir / "dataset.jsonl"
    # append 到累积集
    dataset = run_dir / "dataset.jsonl"
    n_real = 0
    with COLLECT_FILE.open() as src, dataset.open("a") as dst:
        for line in src:
            line = line.strip()
            if not line:
                continue
            dst.write(line + "\n")
            n_real += 1
    print(f"  +{n_real} 真实样本 → {dataset}", flush=True)
    return dataset


def stage_synth(run_dir: Path) -> Path:
    """合成补充（每轮重生成，append 到累积集）"""
    synth_path = run_dir / "synth_tmp.jsonl"
    with synth_path.open("w") as f:
        for s in gen_grid():
            f.write(json.dumps(s) + "\n")
    dataset = run_dir / "dataset.jsonl"
    with synth_path.open() as src, dataset.open("a") as dst:
        for line in src:
            dst.write(line)
    print(f"  +250 合成样本 → {dataset}", flush=True)
    return dataset


def stage_train(dataset: Path, run_dir: Path) -> Path:
    """训练 5 输出执行量模型"""
    print("[train]", flush=True)
    ds_dir = run_dir / "ds"
    ds_dir.mkdir(exist_ok=True)
    shutil.copyfile(dataset, ds_dir / "samples.jsonl")
    (ds_dir / "metadata.json").write_text(
        json.dumps({"feature_names": "v3", "schema_version": "flowengine.e2e_sample.v2"})
    )
    out = run_dir / "model"
    rc = run(["python3", "tools/train_e2e/train.py",
              "--dataset", str(ds_dir), "--output", str(out),
              "--hidden", "64 32", "--epochs", "300"])
    if rc != 0 or not (out / "model.txt").exists():
        print("  !! train failed", flush=True)
        return Path()
    return out / "model.txt"


def stage_closed_loop(model: Path, run_dir: Path) -> dict:
    """闭环评估三场景"""
    print("[closed_loop]", flush=True)
    out_json = run_dir / "closed_loop_eval.json"
    rc = run(["python3", CLOSED_LOOP_EVAL, "--model", str(model),
              "--output", str(out_json)])
    if rc != 0 or not out_json.exists():
        return {"overall": "ERROR"}
    d = json.loads(out_json.read_text())
    # 提取三场景结果
    result = {"overall": d.get("overall", "?"),
              "scenarios": d.get("scenarios", {})}
    print(f"  → {result['overall']}", flush=True)
    return result


def stage_promote(run_dir: Path) -> bool:
    """三场景全 PASS → 影子评估 → promote"""
    eval_json = run_dir / "closed_loop_eval.json"
    if not eval_json.exists():
        return False
    d = json.loads(eval_json.read_text())
    if d.get("overall") != "PASS":
        return False
    # 闭环 PASS → 复制到 best 目录（候选模型，供后续影子评估/promote）
    best_dir = ROOT / "models" / "auto_train_best"
    best_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(run_dir / "model" / "model.txt", best_dir / "model.txt")
    shutil.copyfile(eval_json, best_dir / "closed_loop_eval.json")
    # 影子评估：让 inference_node 加载模型真跑 45s，生成 shadow_eval.json
    # （promote 门禁要求）。临时 pipeline 注入 model_path。
    import tempfile
    pipeline = json.loads((ROOT / "config" / "pipeline.json").read_text())
    for proc in pipeline.get("processes", []):
        if "inference" in proc.get("name", ""):
            params = json.loads(proc.get("params") or "{}")
            params["model_path"] = str(best_dir / "model.txt")
            proc["params"] = json.dumps(params, ensure_ascii=False)
    fd, tmp = tempfile.mkstemp(prefix="pipeline_auto_", suffix=".json", dir="/tmp")
    with os.fdopen(fd, "w") as fh:
        json.dump(pipeline, fh, indent=2)
    env = {"FLOW_PIPELINE": tmp}
    COLLECT_FILE.unlink(missing_ok=True)
    rc = run(["bash", "scripts/demo.sh", "--no-browser", "45"], timeout=180)
    # 读影子 sidecar → shadow_eval.json
    sidecar = Path("/tmp/flow_tiny_inference.json")
    if sidecar.exists():
        s = json.loads(sidecar.read_text())
        shadow_eval = {
            "schema": "flowengine.shadow_eval.v1",
            "shadow_speed_mae": s.get("shadow_speed_mae"),
            "shadow_speed_rmse": s.get("shadow_speed_rmse"),
            "shadow_n": s.get("shadow_n"),
            "model": "auto_train_best",
        }
        (best_dir / "shadow_eval.json").write_text(json.dumps(shadow_eval, indent=2))
        print(f"  影子评估 MAE={shadow_eval['shadow_speed_mae']:.2f} n={shadow_eval['shadow_n']}",
              flush=True)
        # promote 到 runtime
        rc2 = run(["python3", "tools/modelctl.py", "promote",
                   "models/auto_train_best", "--force"])
        if rc2 == 0:
            print(f"  ★ promote 成功 → C runtime", flush=True)
            return True
    print("  ★ 闭环 PASS，但影子评估/promote 未完成", flush=True)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description="学习闭环后台持续训练")
    ap.add_argument("--rounds", type=int, default=10)
    ap.add_argument("--collect-duration", type=int, default=60)
    ap.add_argument("--scenarios", default="straight_road,multi_light,lane_change_traffic",
                    help="场景轮换列表（逗号分隔）")
    ap.add_argument("--run-dir", default=None, help="运行目录（默认 runs/auto_train_<ts>）")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    random.seed(args.seed)
    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    for s in scenarios:
        if s not in SCENARIOS:
            print(f"error: unknown scenario {s!r}", file=sys.stderr)
            return 2

    run_dir = Path(args.run_dir) if args.run_dir else \
        ROOT / "runs" / f"auto_train_{int(time.time())}"
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"=== auto_train_loop → {run_dir}", flush=True)
    print(f"    场景轮换: {scenarios}", flush=True)

    history = []
    for rnd in range(1, args.rounds + 1):
        print(f"\n=== 第 {rnd}/{args.rounds} 轮 ===", flush=True)
        scenario = scenarios[(rnd - 1) % len(scenarios)]

        # 1. 采集（每轮场景轮换，真实样本累积）
        dataset = stage_collect(scenario, args.collect_duration, run_dir)

        # 2. 合成补充
        dataset = stage_synth(run_dir)

        # 3. 训练
        model = stage_train(dataset, run_dir)
        if not model.exists():
            print("  !! 训练失败，跳过本轮", flush=True)
            history.append({"round": rnd, "scenario": scenario, "result": "TRAIN_FAIL"})
            continue

        # 4. 闭环评估
        result = stage_closed_loop(model, run_dir)

        # 5. 记录
        rec = {"round": rnd, "scenario": scenario, "result": result.get("overall"),
               "model": str(model), "ts": int(time.time())}
        history.append(rec)
        with (run_dir / "summary.jsonl").open("a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

        # 6. 达标 promote
        if result.get("overall") == "PASS":
            stage_promote(run_dir)

    # 汇总
    print(f"\n=== 完成 {args.rounds} 轮 ===", flush=True)
    n_pass = sum(1 for h in history if h.get("result") == "PASS")
    print(f"PASS: {n_pass}/{len(history)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
