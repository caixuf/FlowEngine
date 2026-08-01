#!/usr/bin/env python3
"""learning_loop.py — 车端学习闭环一键流水线（PLAN v0.2 Phase 2.1）

一条命令跑完 采集 → 训练 → 影子评估 → （可选）晋级：

    python3 tools/learning_loop.py                        # collect 60s + train tiny + shadow eval 45s
    python3 tools/learning_loop.py --collect 90 --backend temporal
    python3 tools/learning_loop.py --skip-collect --input /tmp/flow_train_samples.jsonl
    python3 tools/learning_loop.py --promote              # 影子评估达标后自动 modelctl promote

只做编排，不写新训练代码：
  Stage 0 采集   → scripts/demo.sh（data_recorder_node 写 /tmp/flow_train_samples.jsonl）
  Stage 1 训练   → tools/train_demo_model.py（tiny / torch / temporal backend）
  Stage 2 影子   → 临时 pipeline 注入候选 model_path（FLOW_PIPELINE），
                   ci/evaluators/demo_evaluator.py 真跑评分，
                   inference_node 落盘 /tmp/flow_tiny_inference.json（含累计 MAE/RMSE）
  Stage 3 晋级   → tools/modelctl.py promote（门禁读 shadow_eval.json，见 modelctl）

产物目录（runs/ 每次一个 run，模型 artifact 仍在 models/<name>/）：
  runs/loop_<ts>/
    ├── samples.jsonl        采集样本快照
    ├── eval.json            demo_evaluator --json-out 完整报告
    ├── shadow_sidecar.json  运行结束时的 sidecar 快照
    └── loop_summary.json    全流程结论（阶段状态 + 指标 + promote 结果）
  models/<name>/shadow_eval.json   promote 门禁消费的影子评估结论
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SAMPLES = Path("/tmp/flow_train_samples.jsonl")
TINY_SIDECAR = Path("/tmp/flow_tiny_inference.json")
DEFAULT_RUNS_DIR = ROOT / "runs"
PIPELINE_JSON = ROOT / "config" / "pipeline.json"


def log(stage: str, msg: str) -> None:
    print(f"[learning_loop] {stage}: {msg}", flush=True)


def run(cmd: list[str], env: dict | None = None) -> int:
    log("exec", " ".join(str(c) for c in cmd))
    merged = dict(os.environ)
    if env:
        merged.update(env)
    return subprocess.run(cmd, cwd=ROOT, env=merged).returncode


# ── Stage 0: 采集 ─────────────────────────────────────────────

def stage_collect(seconds: int, input_path: Path) -> int:
    rc = run(["bash", "scripts/demo.sh", "--no-browser", str(seconds)])
    if rc != 0:
        log("collect", f"demo.sh exit={rc}")
        return rc
    if not input_path.exists() or input_path.stat().st_size == 0:
        log("collect", f"no samples at {input_path} — data_recorder 没在 pipeline 里？")
        return 1
    n = sum(1 for _ in input_path.open())
    log("collect", f"{n} samples in {input_path}")
    return 0


# ── Stage 1: 训练 ─────────────────────────────────────────────

def stage_train(backend: str, name: str, input_path: Path,
                epochs: int | None, hidden: int | None) -> int:
    cmd = [sys.executable, "tools/train_demo_model.py",
           "--backend", backend, "--name", name, "--input", str(input_path)]
    if epochs is not None:
        cmd += ["--epochs", str(epochs)]
    if hidden is not None:
        cmd += ["--hidden", str(hidden)]
    return run(cmd)


# ── Stage 2: 影子评估 ─────────────────────────────────────────

def build_shadow_pipeline(model_file: Path) -> Path:
    """复制 config/pipeline.json，把 inference 节点 model_path 换成候选模型。

    注意 launcher 的 processes[].params 是「字符串形式的 JSON」，需二次解析。
    """
    pipeline = json.loads(PIPELINE_JSON.read_text(encoding="utf-8"))
    patched = False
    for proc in pipeline.get("processes", []):
        if not isinstance(proc, dict) or "inference" not in proc.get("name", ""):
            continue
        params = json.loads(proc.get("params") or "{}")
        params["model_path"] = str(model_file)
        proc["params"] = json.dumps(params, ensure_ascii=False)
        patched = True
    if not patched:
        raise SystemExit("error: config/pipeline.json 中找不到 inference 节点")
    fd, tmp = tempfile.mkstemp(prefix="pipeline_shadow_", suffix=".json", dir="/tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump(pipeline, fh, indent=2, ensure_ascii=False)
    return Path(tmp)


def stage_shadow_eval(model_dir: Path, duration: int, run_dir: Path,
                      scenario: str | None) -> tuple[int, dict]:
    manifest = json.loads((model_dir / "manifest.json").read_text(encoding="utf-8"))
    model_file = model_dir / manifest.get("model_path", "model.txt")
    if not model_file.exists():
        log("shadow", f"model file missing: {model_file}")
        return 1, {}
    if manifest.get("backend") != "tiny_mlp":
        log("shadow", f"backend={manifest.get('backend')!r} 不能进 C runtime 影子评估，"
                      "torch 模型请用 tools/train_e2e/torch_sidecar.py")
        return 1, {}

    TINY_SIDECAR.unlink(missing_ok=True)
    shadow_pipeline = build_shadow_pipeline(model_file)
    eval_json = run_dir / "eval.json"
    cmd = [sys.executable, "ci/evaluators/demo_evaluator.py",
           "--duration", str(duration), "--interval", "0.5",
           "--json-out", str(eval_json)]
    if scenario:
        cmd += ["--scenario", scenario]
    try:
        rc = run(cmd, env={"FLOW_PIPELINE": str(shadow_pipeline)})
    finally:
        shadow_pipeline.unlink(missing_ok=True)

    result: dict = {"evaluator_exit": rc}
    if eval_json.exists():
        report = json.loads(eval_json.read_text(encoding="utf-8"))
        result["evaluator_result"] = report.get("result")
        result["failures"] = report.get("failures", [])
    if TINY_SIDECAR.exists():
        sidecar = json.loads(TINY_SIDECAR.read_text(encoding="utf-8"))
        shutil.copyfile(TINY_SIDECAR, run_dir / "shadow_sidecar.json")
        result["shadow_speed_mae"] = sidecar.get("shadow_speed_mae")
        result["shadow_speed_rmse"] = sidecar.get("shadow_speed_rmse")
        result["shadow_n"] = sidecar.get("shadow_n")
        result["model"] = sidecar.get("model")
    else:
        log("shadow", f"sidecar 未生成（{TINY_SIDECAR}）——模型没加载成功？")
        result["shadow_speed_mae"] = None

    # 写进 artifact，promote 门禁消费
    shadow_eval = {
        "schema": "flowengine.shadow_eval.v1",
        "created_unix_ms": int(time.time() * 1000),
        "eval_duration_s": duration,
        "scenario": scenario or "default",
        **result,
    }
    (model_dir / "shadow_eval.json").write_text(
        json.dumps(shadow_eval, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    log("shadow", f"shadow_eval.json → {model_dir / 'shadow_eval.json'}")
    return (0 if rc == 0 else 1), result


# ── Stage 3: 晋级 ─────────────────────────────────────────────

def stage_promote(model_dir: Path) -> int:
    return run([sys.executable, "tools/modelctl.py", "promote", str(model_dir)])


# ── main ─────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine 学习闭环一键流水线")
    parser.add_argument("--collect", type=int, default=60, metavar="SECONDS",
                        help="采集阶段 demo 时长（默认 60）")
    parser.add_argument("--skip-collect", action="store_true", help="跳过采集，直接用 --input")
    parser.add_argument("--input", default=str(DEFAULT_SAMPLES), help="训练样本 JSONL")
    parser.add_argument("--backend", choices=["tiny", "temporal"], default="tiny",
                        help="训练 backend（torch 走 torch_sidecar，不在本闭环内）")
    parser.add_argument("--name", default=None, help="模型名（默认 loop_<ts>）")
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--hidden", type=int, default=None)
    parser.add_argument("--eval-duration", type=int, default=45, help="影子评估时长（默认 45s）")
    parser.add_argument("--scenario", default=None, help="影子评估场景 JSON（默认 demo.sh 默认场景）")
    parser.add_argument("--promote", action="store_true", help="影子评估后自动 modelctl promote（走门禁）")
    parser.add_argument("--eval-only", metavar="NAME_OR_DIR", default=None,
                        help="跳过采集+训练，只对已有 models/<NAME> 重跑影子评估（刷新 shadow_eval.json）")
    parser.add_argument("--runs-dir", default=str(DEFAULT_RUNS_DIR))
    args = parser.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = args.name or f"loop_{stamp}"
    input_path = Path(args.input)
    run_dir = Path(args.runs_dir) / f"loop_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    model_dir = ROOT / "models" / name

    summary: dict = {"schema": "flowengine.loop_summary.v1", "name": name,
                     "backend": args.backend, "run_dir": str(run_dir), "stages": {}}

    def finish(code: int) -> int:
        (run_dir / "loop_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        log("done", f"summary → {run_dir / 'loop_summary.json'} (exit={code})")
        return code

    # ── --eval-only: 只刷新已有 artifact 的影子评估 ──
    if args.eval_only:
        model_dir = Path(args.eval_only)
        if not model_dir.is_dir():
            model_dir = ROOT / "models" / args.eval_only
        if not (model_dir / "manifest.json").exists():
            log("eval-only", f"artifact 不存在: {args.eval_only}")
            return finish(1)
        summary["name"] = model_dir.name
        summary["model_dir"] = str(model_dir)
        summary["stages"]["collect"] = summary["stages"]["train"] = "skipped(eval-only)"
        rc, shadow = stage_shadow_eval(model_dir, args.eval_duration, run_dir, args.scenario)
        summary["stages"]["shadow_eval"] = "ok" if rc == 0 else "failed"
        summary["shadow"] = shadow
        if rc == 0 and args.promote:
            prc = stage_promote(model_dir)
            summary["stages"]["promote"] = "ok" if prc == 0 else "rejected"
            return finish(0 if prc == 0 else 3)
        return finish(0 if rc == 0 else 2)

    # Stage 0
    if args.skip_collect:
        summary["stages"]["collect"] = "skipped"
        if not input_path.exists():
            log("collect", f"--skip-collect 但样本不存在: {input_path}")
            return finish(1)
    else:
        if stage_collect(args.collect, input_path) != 0:
            summary["stages"]["collect"] = "failed"
            return finish(1)
        summary["stages"]["collect"] = "ok"
    shutil.copyfile(input_path, run_dir / "samples.jsonl")

    # Stage 1
    if stage_train(args.backend, name, input_path, args.epochs, args.hidden) != 0:
        summary["stages"]["train"] = "failed"
        return finish(1)
    summary["stages"]["train"] = "ok"
    summary["model_dir"] = str(model_dir)

    # Stage 2
    rc, shadow = stage_shadow_eval(model_dir, args.eval_duration, run_dir, args.scenario)
    summary["stages"]["shadow_eval"] = "ok" if rc == 0 else "failed"
    summary["shadow"] = shadow
    if rc != 0:
        log("shadow", "影子评估未通过 —— 模型留在 models/ 供 inspect，不晋级")
        return finish(2)

    # Stage 3
    if args.promote:
        prc = stage_promote(model_dir)
        summary["stages"]["promote"] = "ok" if prc == 0 else "rejected"
        if prc != 0:
            return finish(3)
    else:
        summary["stages"]["promote"] = "skipped"
        log("promote", f"手动晋级: python3 tools/modelctl.py promote {model_dir}")

    return finish(0)


if __name__ == "__main__":
    raise SystemExit(main())
