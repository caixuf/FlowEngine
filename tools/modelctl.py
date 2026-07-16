#!/usr/bin/env python3
"""Inspect and promote FlowEngine learning-loop model artifacts."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODELS_DIR = ROOT / "models"
DEFAULT_RUNTIME_MODEL = ROOT / "tools" / "train" / "model.txt"
TRAINING_STATE_DIR = DEFAULT_MODELS_DIR / ".training"
TRAINING_STATUS_FILE = TRAINING_STATE_DIR / "status.json"
TRAINING_PIDFILE = TRAINING_STATE_DIR / "job.pid"
TRAINING_LOGFILE = TRAINING_STATE_DIR / "job.log"


def load_manifest(artifact_dir: Path) -> dict:
    manifest_path = artifact_dir / "manifest.json"
    if not manifest_path.exists():
        return {}
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {"_error": "invalid manifest.json"}


def load_metrics(artifact_dir: Path) -> dict | None:
    metrics_path = artifact_dir / "metrics.json"
    if not metrics_path.exists():
        return None
    try:
        return json.loads(metrics_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {"_error": "invalid metrics.json"}


def iter_artifacts(models_dir: Path) -> list[tuple[Path, dict]]:
    if not models_dir.exists():
        return []
    artifacts = []
    for path in sorted(models_dir.iterdir()):
        if path.is_dir() and (path / "manifest.json").exists():
            artifacts.append((path, load_manifest(path)))
    return artifacts


def model_file_for_artifact(artifact_dir: Path, manifest: dict) -> Path:
    return artifact_dir / manifest.get("model_path", "model.txt")


def describe_artifact(artifact_dir: Path, manifest: dict) -> str:
    backend = manifest.get("backend", "unknown")
    model_format = manifest.get("model_format", "unknown")
    model_path = model_file_for_artifact(artifact_dir, manifest)
    dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
    sample_count = dataset.get("sample_count", "?")
    scenario = dataset.get("scenario", "unknown")
    runtime_note = "runtime-promotable" if backend == "tiny_mlp" else "sidecar/eval only"
    exists = "ok" if model_path.exists() else "missing-model"
    return (
        f"  {artifact_dir.name:<28} backend={backend:<9} format={model_format:<22} "
        f"samples={sample_count!s:<5} scenario={scenario:<12} {runtime_note} {exists}"
    )


def cmd_list(args: argparse.Namespace) -> int:
    runtime_model = Path(args.runtime_model)
    models_dir = Path(args.models_dir)

    print("FlowEngine models")
    print(f"  runtime tiny : {runtime_model}")
    print(f"  runtime state: {'present' if runtime_model.exists() else 'missing'}")
    print(f"  artifact dir : {models_dir}")

    artifacts = iter_artifacts(models_dir)
    if not artifacts:
        print("  artifacts    : none")
        return 0

    print("  artifacts:")
    for artifact_dir, manifest in artifacts:
        print(describe_artifact(artifact_dir, manifest))
    return 0


def cmd_inspect(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact)
    if not artifact_dir.is_dir():
        raise SystemExit(f"error: artifact directory not found: {artifact_dir}")

    manifest = load_manifest(artifact_dir)
    metrics = load_metrics(artifact_dir)

    print(f"artifact : {artifact_dir}")
    print(f"backend  : {manifest.get('backend', 'unknown')}")
    print(f"format   : {manifest.get('model_format', 'unknown')}")

    dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
    if dataset:
        dataset_desc = (
            f"{dataset.get('path', '?')} "
            f"({dataset.get('sample_count', '?')} samples, "
            f"scenario={dataset.get('scenario', '?')}, "
            f"schema={dataset.get('schema_version', '?')})"
        )
        print(f"dataset  : {dataset_desc}")

    training = manifest.get("training", {}) if isinstance(manifest.get("training"), dict) else {}
    if training:
        parts = [f"epochs={training.get('epochs', '?')}", f"lr={training.get('lr', '?')}",
                 f"hidden={training.get('hidden', '?')}"]
        if training.get("init_from"):
            parts.append(f"init_from={training['init_from']}")
        if training.get("final_mse_norm") is not None:
            parts.append(f"final_mse_norm={training['final_mse_norm']:.6f}")
        print(f"training : {', '.join(parts)}")

    in_schema = manifest.get("input_schema", {})
    out_schema = manifest.get("output_schema", {})
    if in_schema:
        features = in_schema.get("features", [])
        print(f"features : {len(features)} dims — {features}")
    if out_schema:
        print(f"labels   : {out_schema.get('labels', [])}")

    if metrics:
        print("\nmetrics:")
        skip = {"schema_version", "model", "dataset", "sample_count"}
        for key, value in metrics.items():
            if key in skip:
                continue
            print(f"  {key:<28} {value:.6f}" if isinstance(value, float) else f"  {key:<28} {value}")
    else:
        print("\nmetrics : not available (run eval_model.py to generate)")

    return 0


# Epsilon for floating-point improvement comparison in diff output.
_METRIC_EPSILON = 1e-6
_DIFF_METRICS = [
    "mae_target_speed",
    "rmse_target_speed",
    "std_error",
    "mean_error",
    "max_abs_error",
    "p50_abs_error",
    "p90_abs_error",
    "p95_abs_error",
    "p99_abs_error",
]


def cmd_diff(args: argparse.Namespace) -> int:
    dir_a = Path(args.artifact_a)
    dir_b = Path(args.artifact_b)
    for path in (dir_a, dir_b):
        if not path.is_dir():
            raise SystemExit(f"error: artifact directory not found: {path}")

    manifest_a = load_manifest(dir_a)
    manifest_b = load_manifest(dir_b)
    metrics_a = load_metrics(dir_a)
    metrics_b = load_metrics(dir_b)

    def _fmt_backend(m: dict) -> str:
        return f"{m.get('backend', '?')} / {m.get('model_format', '?')}"

    def _fmt_samples(m: dict) -> str:
        d = m.get("dataset", {}) if isinstance(m.get("dataset"), dict) else {}
        return f"{d.get('sample_count', '?')} samples, scenario={d.get('scenario', '?')}"

    col = 32
    print(f"{'':>{col}}  {'A':>14}  {'B':>14}  {'delta':>14}")
    print(f"  {'artifact':<{col-2}}  {dir_a.name:>14}  {dir_b.name:>14}")
    print(f"  {'backend':<{col-2}}  {_fmt_backend(manifest_a):>14}  {_fmt_backend(manifest_b):>14}")
    print(f"  {'dataset':<{col-2}}  {_fmt_samples(manifest_a):>14}  {_fmt_samples(manifest_b):>14}")

    if metrics_a is None and metrics_b is None:
        print("\n  (no metrics.json in either artifact — run eval_model.py)")
        return 0

    print()
    for key in _DIFF_METRICS:
        va = metrics_a.get(key) if metrics_a else None
        vb = metrics_b.get(key) if metrics_b else None
        if va is None and vb is None:
            continue
        sa = f"{va:.4f}" if isinstance(va, float) else "n/a"
        sb = f"{vb:.4f}" if isinstance(vb, float) else "n/a"
        if isinstance(va, float) and isinstance(vb, float):
            delta = vb - va
            sd = f"{delta:+.4f}"
            # Highlight improvement (lower error = better)
            marker = " ✓" if delta < -_METRIC_EPSILON else (" ✗" if delta > _METRIC_EPSILON else "")
            print(f"  {key:<{col-2}}  {sa:>14}  {sb:>14}  {sd:>14}{marker}")
        else:
            print(f"  {key:<{col-2}}  {sa:>14}  {sb:>14}")

    return 0


def cmd_promote(args: argparse.Namespace) -> int:
    # --json: read params from stdin (HTTP bridge path)
    if args.json:
        try:
            payload = json.loads(sys.stdin.read())
        except json.JSONDecodeError as e:
            print(json.dumps({"ok": False, "error": f"invalid JSON: {e}"}))
            return 1
        name = str(payload.get("name", "")).strip()
        if not name or not all(c.isalnum() or c in "._-" for c in name):
            print(json.dumps({"ok": False, "error": "invalid model name"}))
            return 1
        args.artifact = name
        if payload.get("runtime_model"):
            args.runtime_model = str(payload["runtime_model"])

    artifact_path = Path(args.artifact)
    runtime_model = Path(args.runtime_model)

    artifact_dir = artifact_path
    if not artifact_dir.is_dir():
        artifact_dir = DEFAULT_MODELS_DIR / args.artifact
        if not artifact_dir.is_dir():
            raise SystemExit(f"error: artifact not found: {args.artifact}")

    manifest = load_manifest(artifact_dir)
    backend = manifest.get("backend")
    if backend != "tiny_mlp":
        raise SystemExit(
            f"error: only backend=tiny_mlp artifacts can be promoted to C runtime; got {backend!r}. "
            "Use torch_sidecar.py for PyTorch artifacts."
        )

    source = model_file_for_artifact(artifact_dir, manifest)
    if not source.exists():
        raise SystemExit(f"error: model file not found: {source}")

    runtime_model.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, runtime_model)
    print(f"promoted {source} -> {runtime_model}")
    return 0


# ─────────────────────────────────────────────────────────────
#  Training 子命令 (Stage 0: 后台训练协调)
# ─────────────────────────────────────────────────────────────

def _write_status(status: dict) -> None:
    TRAINING_STATE_DIR.mkdir(parents=True, exist_ok=True)
    tmp = TRAINING_STATUS_FILE.with_name("status.json.tmp")
    tmp.write_text(json.dumps(status, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(TRAINING_STATUS_FILE)


def _load_status() -> dict:
    if TRAINING_STATUS_FILE.exists():
        try:
            return json.loads(TRAINING_STATUS_FILE.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            return {}
    return {}


def _is_pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except (OSError, ProcessLookupError):
        return False


def _tail_logfile(lines: int = 50) -> str:
    if not TRAINING_LOGFILE.exists():
        return ""
    try:
        content = TRAINING_LOGFILE.read_text(encoding="utf-8", errors="replace")
        return "\n".join(content.split("\n")[-lines:])
    except Exception:
        return ""


def _supervisor_process(command: list[str], model_name: str, backend: str) -> None:
    TRAINING_STATE_DIR.mkdir(parents=True, exist_ok=True)
    logfile = TRAINING_LOGFILE.open("w", encoding="utf-8", buffering=1)
    try:
        proc = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, cwd=ROOT, text=True)
        TRAINING_PIDFILE.write_text(str(proc.pid) + "\n", encoding="utf-8")
        returncode = proc.wait()
        status = {
            "running": False,
            "command": command,
            "model_name": model_name,
            "backend": backend,
            "started_at": _load_status().get("started_at"),
            "pid": proc.pid,
            "returncode": returncode,
            "log_tail": _tail_logfile(100),
            "error": None if returncode == 0 else f"training exited with code {returncode}",
        }
    except Exception as e:
        status = {
            "running": False,
            "command": command,
            "model_name": model_name,
            "backend": backend,
            "started_at": _load_status().get("started_at"),
            "pid": None,
            "returncode": None,
            "log_tail": _tail_logfile(100),
            "error": str(e),
        }
    finally:
        logfile.close()
    _write_status(status)
    TRAINING_PIDFILE.unlink(missing_ok=True)


def _validate_model_name(name: str) -> str:
    name = name.strip()
    if not name or not all(c.isalnum() or c in "._-" for c in name):
        raise ValueError(f"invalid model name: {name!r}")
    return name


def _safe_int(value, param_name: str, min_val: int, max_val: int) -> int | None:
    if value is None:
        return None
    try:
        v = int(value)
        if not min_val <= v <= max_val:
            raise ValueError(f"{param_name} must be {min_val}–{max_val}, got {v}")
        return v
    except (ValueError, TypeError) as e:
        raise ValueError(f"invalid {param_name}: {e}") from e


def cmd_train_start(args: argparse.Namespace) -> int:
    # --json: read params from stdin (HTTP bridge path)
    if args.json:
        try:
            payload = json.loads(sys.stdin.read())
        except json.JSONDecodeError as e:
            print(json.dumps({"ok": False, "error": f"invalid JSON: {e}"}))
            return 1
        args.backend = str(payload.get("backend", "torch"))
        name = str(payload.get("name", "")).strip()
        if not name or not all(c.isalnum() or c in "._-" for c in name):
            print(json.dumps({"ok": False, "error": "invalid model name"}))
            return 1
        args.name = name
        if payload.get("epochs") is not None:
            args.epochs = int(payload["epochs"])
        if payload.get("hidden") is not None:
            args.hidden = int(payload["hidden"])
        if payload.get("run_demo_seconds") is not None:
            args.run_demo_seconds = int(payload["run_demo_seconds"])
        if payload.get("init_from"):
            args.init_from = str(payload["init_from"])

    backend = args.backend
    if backend not in ("torch", "tiny"):
        raise ValueError("backend must be torch or tiny")
    model_name = _validate_model_name(args.name)
    epochs = _safe_int(args.epochs, "epochs", 1, 100000)
    hidden = _safe_int(args.hidden, "hidden", 1, 4096)
    run_demo = _safe_int(args.run_demo_seconds, "run_demo_seconds", 1, 3600)
    input_file = args.input or str(Path("/tmp/flow_train_samples.jsonl"))

    status = _load_status()
    if status.get("running"):
        raise SystemExit("error: training job is already running")

    command = [sys.executable, "tools/train_demo_model.py", "--backend", backend, "--name", model_name]
    if epochs is not None:
        command.extend(["--epochs", str(epochs)])
    if hidden is not None:
        command.extend(["--hidden", str(hidden)])
    if run_demo is not None:
        command.extend(["--run-demo", str(run_demo)])
    if args.init_from:
        command.extend(["--init-from", args.init_from])

    status = {
        "running": True,
        "command": command,
        "model_name": model_name,
        "backend": backend,
        "started_at": int(time.time()),
        "pid": None,
        "returncode": None,
        "log_tail": "",
        "error": None,
    }
    _write_status(status)

    pid = os.fork()
    if pid == 0:
        os.setsid()
        child_pid = os.fork()
        if child_pid == 0:
            _supervisor_process(command, model_name, backend)
            sys.exit(0)
        else:
            sys.exit(0)
    else:
        os.waitpid(pid, 0)

    print(json.dumps({"ok": True, "job": status}, indent=2))
    return 0


def cmd_train_status(args: argparse.Namespace) -> int:
    models_dir = Path(args.models_dir)
    status = _load_status()

    if status.get("running") and status.get("pid"):
        if not _is_pid_alive(status["pid"]):
            status["running"] = False
            _write_status(status)

    artifacts = []
    if models_dir.exists():
        for path in sorted(models_dir.iterdir()):
            if path.is_dir() and path.name != ".training" and (path / "manifest.json").exists():
                manifest = load_manifest(path)
                backend = manifest.get("backend", "unknown")
                dataset = manifest.get("dataset", {}) if isinstance(manifest.get("dataset"), dict) else {}
                metrics_path = path / "metrics.json"
                metrics = None
                if metrics_path.exists():
                    try:
                        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
                    except Exception:
                        metrics = None
                artifacts.append({
                    "name": path.name,
                    "backend": backend,
                    "promotable": backend == "tiny_mlp",
                    "manifest": manifest,
                    "metrics": metrics,
                })

    result = {
        "job": status,
        "models": artifacts,
    }
    print(json.dumps(result, indent=2))
    return 0


# ─────────────────────────────────────────────────────────────
#  OTA 子命令 (Stage 4: model_ota_node 交互)
# ─────────────────────────────────────────────────────────────

OTA_CMD_FILE    = Path("/tmp/flow_ota_cmd.json")
OTA_STATUS_FILE = Path("/tmp/flow_ota_status.json")


def _ota_write_cmd(cmd_json: str) -> None:
    """将 OTA 命令写入命令文件，model_ota_node 会在下一个轮询周期处理。"""
    OTA_CMD_FILE.write_text(cmd_json + "\n", encoding="utf-8")
    print(f"OTA command written to {OTA_CMD_FILE}")
    print("model_ota_node will process it within poll_interval_ms (default 500 ms)")


def cmd_ota_push(args: argparse.Namespace) -> int:
    """加载并激活一个新模型版本（通过 model_ota_node）。"""
    path = Path(args.path)
    if not path.exists():
        raise SystemExit(f"error: model file not found: {path}")
    version_id = args.id or path.stem
    cmd = f'{{"cmd":"load","id":"{version_id}","path":"{path}"}}'
    _ota_write_cmd(cmd)
    return 0


def cmd_ota_rollback(args: argparse.Namespace) -> int:
    """回滚到上一个模型版本（通过 model_ota_node）。"""
    del args  # no arguments needed
    _ota_write_cmd('{"cmd":"rollback"}')
    return 0


def cmd_ota_ab_test(args: argparse.Namespace) -> int:
    """开启或关闭 A-B 测试模式（通过 model_ota_node）。"""
    enable = not args.disable
    cmd_parts = [f'"cmd":"ab_test"', f'"enable":{str(enable).lower()}']
    if args.ratio is not None:
        cmd_parts.append(f'"ratio":{args.ratio}')
    if args.model_b:
        mb = Path(args.model_b)
        if not mb.exists():
            raise SystemExit(f"error: model_b file not found: {mb}")
        cmd_parts.append(f'"model_b_path":"{mb}"')
    _ota_write_cmd("{" + ",".join(cmd_parts) + "}")
    return 0


def cmd_ota_status(args: argparse.Namespace) -> int:
    """显示 model_ota_node 当前状态（读取 /tmp/flow_ota_status.json）。"""
    del args  # no arguments needed
    if not OTA_STATUS_FILE.exists():
        print("OTA status file not found. Is model_ota_node running?")
        print(f"  Expected: {OTA_STATUS_FILE}")
        return 1

    try:
        status = json.loads(OTA_STATUS_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise SystemExit(f"error: invalid status file: {e}") from e

    print("model_ota_node status")
    for key, value in status.items():
        print(f"  {key:<24} {value}")

    # 如果注册表存在，也显示
    registry_path = ROOT / "models" / "registry.json"
    if registry_path.exists():
        try:
            registry = json.loads(registry_path.read_text(encoding="utf-8"))
            versions = registry.get("versions", [])
            if versions:
                print(f"\n  version registry ({len(versions)} entries):")
                for v in versions:
                    active_mark = " ← active" if v.get("active") else ""
                    print(f"    {v.get('id','?'):<20} {v.get('path','?')}{active_mark}")
        except json.JSONDecodeError:
            pass
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine model artifact manager")
    sub = parser.add_subparsers(dest="command", required=True)

    list_parser = sub.add_parser("list", help="Show runtime model and known artifacts")
    list_parser.add_argument("--runtime-model", default=str(DEFAULT_RUNTIME_MODEL))
    list_parser.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    list_parser.set_defaults(func=cmd_list)

    train_start_parser = sub.add_parser("train-start", help="Start a background training job")
    train_start_parser.add_argument("--backend", choices=["torch", "tiny"], default="torch")
    train_start_parser.add_argument("--name", help="Model name (required unless --json)")
    train_start_parser.add_argument("--epochs", type=int, default=None)
    train_start_parser.add_argument("--hidden", type=int, default=None)
    train_start_parser.add_argument("--run-demo-seconds", type=int, default=None)
    train_start_parser.add_argument("--init-from", default=None)
    train_start_parser.add_argument("--input", default=None)
    train_start_parser.add_argument("--json", action="store_true",
                                     help="Read params from stdin as JSON (for HTTP bridge)")
    train_start_parser.set_defaults(func=cmd_train_start)

    train_status_parser = sub.add_parser("train-status", help="Query training job status and artifacts")
    train_status_parser.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    train_status_parser.set_defaults(func=cmd_train_status)

    inspect_parser = sub.add_parser("inspect", help="Show detailed manifest and metrics for an artifact")
    inspect_parser.add_argument("artifact", help="Artifact directory containing manifest.json")
    inspect_parser.set_defaults(func=cmd_inspect)

    diff_parser = sub.add_parser("diff", help="Compare metrics between two artifacts")
    diff_parser.add_argument("artifact_a", help="Baseline artifact directory")
    diff_parser.add_argument("artifact_b", help="New artifact directory to compare against baseline")
    diff_parser.set_defaults(func=cmd_diff)

    promote_parser = sub.add_parser("promote", help="Promote a tiny-MLP artifact to the C runtime model")
    promote_parser.add_argument("artifact", nargs="?", help="Artifact name or directory (required unless --json)")
    promote_parser.add_argument("--runtime-model", default=str(DEFAULT_RUNTIME_MODEL))
    promote_parser.add_argument("--json", action="store_true",
                                help="Read params from stdin as JSON (for HTTP bridge)")
    promote_parser.set_defaults(func=cmd_promote)

    # ── OTA 子命令 ────────────────────────────────────────────────
    ota_sub = sub.add_parser("ota", help="Model OTA operations (requires model_ota_node running)")
    ota_cmds = ota_sub.add_subparsers(dest="ota_command", required=True)

    ota_push = ota_cmds.add_parser("push", help="Load and activate a new model version via OTA")
    ota_push.add_argument("path", help="Path to the model file (.txt for tiny-MLP)")
    ota_push.add_argument("--id", default=None, help="Version ID (defaults to filename stem)")
    ota_push.set_defaults(func=cmd_ota_push)

    ota_rollback = ota_cmds.add_parser("rollback", help="Rollback to the previous model version")
    ota_rollback.set_defaults(func=cmd_ota_rollback)

    ota_ab = ota_cmds.add_parser("ab-test", help="Enable or disable A-B model comparison")
    ota_ab.add_argument("--disable", action="store_true", help="Disable A-B test (default: enable)")
    ota_ab.add_argument("--ratio", type=float, default=None,
                        help="Fraction of inferences using model A (0.0–1.0, default 0.5)")
    ota_ab.add_argument("--model-b", dest="model_b", default=None,
                        help="Path to model B for comparison")
    ota_ab.set_defaults(func=cmd_ota_ab_test)

    ota_status = ota_cmds.add_parser("status", help="Show current OTA node status")
    ota_status.set_defaults(func=cmd_ota_status)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
