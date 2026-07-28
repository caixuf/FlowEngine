#!/usr/bin/env python3
"""MPC 权重扫参：改 pipeline.json 的 control params → 跑 demo_evaluator → 收指标。

用法：
    python3 tools/mpc_sweep.py --key mpc_r_ddelta --values 5.0 2.0 1.0 0.5 --duration 45

对每个取值：把 key 写进 pipeline.json 的 control 节点 params（JSON-in-string），
跑一次 demo_evaluator，抓 avg_speed / steer_flip_rate_hz / lane_changes / max_x。
结束后恢复 pipeline.json 原样并打印对比表。

这是**离线 A/B 扫参**，与 `flowctl param set`（边跑边调）分工不同，不是它的
替代品：本脚本抓的是整段 run 的聚合指标（avg_speed / flip_rate / x_delta），
要可比就必须每个取值都从 x=0 起跑一次干净的 run，因此重启进程是必需的而非
将就。想在一次 run 内实时试值，用 `flowctl param set control.mpc_r_ddelta 2.0`。
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PIPELINE = ROOT / "config" / "pipeline.json"

# 从 demo_evaluator 输出里抓的指标（正则 → 表头）。键名与 demo_evaluator.py
# 的 metrics dict 一致（avg_speed_mps / steer_flip_rate_hz / ...）。
METRICS = [
    ("avg_speed_mps", r"avg_speed_mps[\"']?\s*[:=]\s*([-\d.]+)"),
    ("x_delta_m", r"x_delta_m[\"']?\s*[:=]\s*([-\d.]+)"),
    ("lane_change_count", r"lane_change_count[\"']?\s*[:=]\s*([-\d.]+)"),
    ("steer_flip_rate_hz", r"steer_flip_rate_hz[\"']?\s*[:=]\s*([-\d.]+)"),
]


def load_control_params(doc):
    """返回 (proc_dict, params_dict)。control 进程的 params 是 JSON-in-string。"""
    for proc in doc.get("processes", []):
        if proc.get("name") == "control":
            return proc, json.loads(proc.get("params", "{}"))
    raise SystemExit("pipeline.json 里没找到 control 进程")


def set_param(key, value):
    doc = json.loads(PIPELINE.read_text())
    proc, params = load_control_params(doc)
    params[key] = value
    proc["params"] = json.dumps(params, ensure_ascii=False)
    PIPELINE.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")


def run_eval(duration):
    cmd = [
        sys.executable, str(ROOT / "tools" / "demo_evaluator.py"),
        "--duration", str(duration), "--interval", "0.5",
    ]
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                          timeout=duration + 120)
    out = proc.stdout + proc.stderr
    row = {}
    for name, pattern in METRICS:
        m = re.search(pattern, out)
        row[name] = float(m.group(1)) if m else float("nan")
    row["verdict"] = "FAIL" if "FAIL" in out else "PASS"
    return row, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", required=True, help="params_json 里的键名，如 mpc_r_ddelta")
    ap.add_argument("--values", required=True, nargs="+", type=float)
    ap.add_argument("--duration", type=int, default=45)
    ap.add_argument("--repeat", type=int, default=1,
                    help="每个取值重复跑几次（demo 有显著 run-to-run 方差，"
                         "单次结果不足以区分参数效应与噪声）")
    ap.add_argument("--log-dir", default="/tmp/mpc_sweep")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    backup = log_dir / "pipeline.json.bak"
    shutil.copy(PIPELINE, backup)

    rows = []
    try:
        for v in args.values:
            set_param(args.key, v)
            for rep in range(args.repeat):
                tag = f"{v}" if args.repeat == 1 else f"{v} (#{rep + 1})"
                print(f"\n=== {args.key} = {tag} ===", flush=True)
                row, out = run_eval(args.duration)
                (log_dir / f"{args.key}_{v}_r{rep}.log").write_text(out)
                row[args.key] = v
                rows.append(row)
                print("  " + "  ".join(f"{k}={row[k]}" for k, _ in METRICS)
                      + f"  {row['verdict']}", flush=True)
    finally:
        shutil.copy(backup, PIPELINE)
        print(f"\npipeline.json 已恢复（备份在 {backup}）")

    hdr = [args.key] + [k for k, _ in METRICS] + ["verdict"]
    print("\n" + " | ".join(f"{h:>14}" for h in hdr))
    print("-" * (17 * len(hdr)))
    for row in rows:
        print(" | ".join(f"{row.get(h, ''):>14}" for h in hdr))


if __name__ == "__main__":
    main()
