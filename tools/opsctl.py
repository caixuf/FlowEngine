#!/usr/bin/env python3
"""opsctl.py — FlowBoard 运维操作桥接（bag 回灌 + 学习闭环）"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "runs" / ".ops"
STATE_FILE = STATE_DIR / "state.json"
PYTHON = sys.executable or "python3"


def _ensure_state_dir() -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)


def _load_state() -> dict:
    if not STATE_FILE.exists():
        return {"jobs": {}}
    try:
        return json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {"jobs": {}}


def _save_state(state: dict) -> None:
    _ensure_state_dir()
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(STATE_FILE)


def _ok(**kwargs) -> int:
    payload = {"ok": True}
    payload.update(kwargs)
    print(json.dumps(payload, ensure_ascii=False))
    return 0


def _err(msg: str, **kwargs) -> int:
    payload = {"ok": False, "error": msg}
    payload.update(kwargs)
    print(json.dumps(payload, ensure_ascii=False))
    return 1


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _tail(path: Path, lines: int = 80) -> list[str]:
    if not path.exists():
        return []
    out: deque[str] = deque(maxlen=lines)
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            out.append(line.rstrip("\n"))
    return list(out)


def _job_status(entry: dict | None) -> dict:
    if not isinstance(entry, dict):
        return {"running": False}
    pid = int(entry.get("pid", 0) or 0)
    log_path = Path(str(entry.get("log_path", ""))) if entry.get("log_path") else None
    return {
        "pid": pid,
        "running": _pid_alive(pid),
        "started_unix": int(entry.get("started_unix", 0) or 0),
        "cmd": entry.get("cmd", []),
        "log_path": str(log_path) if log_path else "",
        "log_tail": _tail(log_path) if log_path else [],
    }


def _start_job(state: dict, job_name: str, cmd: list[str], log_name: str) -> tuple[bool, str]:
    jobs = state.setdefault("jobs", {})
    st = _job_status(jobs.get(job_name))
    if st.get("running"):
        return False, f"{job_name} 已在运行 (pid={st.get('pid')})"

    _ensure_state_dir()
    log_path = STATE_DIR / log_name
    with log_path.open("a", encoding="utf-8") as logf:
        logf.write(f"\n[{time.strftime('%F %T')}] start: {' '.join(cmd)}\n")
        logf.flush()
        proc = subprocess.Popen(
            cmd,
            cwd=ROOT,
            stdout=logf,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
    jobs[job_name] = {
        "pid": proc.pid,
        "started_unix": int(time.time()),
        "cmd": cmd,
        "log_path": str(log_path),
    }
    _save_state(state)
    return True, f"{job_name} 已启动 (pid={proc.pid})"


def _stop_job(state: dict, job_name: str) -> tuple[bool, str]:
    jobs = state.setdefault("jobs", {})
    entry = jobs.get(job_name)
    st = _job_status(entry)
    if not st.get("running"):
        return False, f"{job_name} 未运行"
    pid = int(st.get("pid") or 0)
    try:
        os.killpg(os.getpgid(pid), signal.SIGTERM)
    except OSError as e:
        return False, f"停止失败: {e}"
    time.sleep(0.2)
    return True, f"{job_name} 已停止"


def _normalize_file(path_raw: str) -> Path:
    if not path_raw:
        raise ValueError("路径不能为空")
    p = Path(path_raw.strip()).expanduser()
    if not p.is_absolute():
        p = (ROOT / p).resolve()
    return p


def _require_exists(path: Path) -> None:
    if not path.exists():
        raise ValueError(f"文件不存在: {path}")


def _run_bag_info(path: Path) -> tuple[bool, dict]:
    flowctl = ROOT / "build" / "bin" / "flowctl"
    if not flowctl.exists():
        return False, {"error": f"未找到 {flowctl}"}
    cmd = [str(flowctl), "bag", "info", str(path)]
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=20)
    out = (p.stdout or "") + (p.stderr or "")
    return p.returncode == 0, {"returncode": p.returncode, "output": out.strip()}


def cmd_status(_: argparse.Namespace) -> int:
    state = _load_state()
    jobs = state.get("jobs", {})
    return _ok(
        jobs={
            "bag_replay": _job_status(jobs.get("bag_replay")),
            "learning_loop": _job_status(jobs.get("learning_loop")),
        }
    )


def cmd_run(args: argparse.Namespace) -> int:
    if not args.json:
        return _err("run 仅支持 --json")
    try:
        payload = json.loads(sys.stdin.read() or "{}")
    except json.JSONDecodeError as e:
        return _err(f"JSON 解析失败: {e}")

    action = str(payload.get("action", "")).strip()
    state = _load_state()

    try:
        if action == "bag_info":
            path = _normalize_file(str(payload.get("path", "")))
            _require_exists(path)
            ok, info = _run_bag_info(path)
            return _ok(action=action, **info) if ok else _err(info.get("error", "bag info 失败"), **info)

        if action == "bag_replay_start":
            bag = ROOT / "build" / "bin" / "flow_bag"
            if not bag.exists():
                return _err(f"未找到 {bag}")
            path = _normalize_file(str(payload.get("path", "")))
            _require_exists(path)
            ok, msg = _start_job(
                state,
                "bag_replay",
                [str(bag), "--replay", str(path)],
                "bag_replay.log",
            )
            return _ok(action=action, message=msg) if ok else _err(msg)

        if action == "bag_replay_stop":
            ok, msg = _stop_job(state, "bag_replay")
            _save_state(state)
            return _ok(action=action, message=msg) if ok else _err(msg)

        if action == "learning_eval_start":
            model = str(payload.get("model", "")).strip()
            if not model:
                return _err("model 不能为空")
            duration = int(payload.get("duration", 45) or 45)
            duration = max(10, min(duration, 600))
            cmd = [PYTHON, "tools/learning_loop.py", "--eval-only", model, "--eval-duration", str(duration)]
            if bool(payload.get("promote", False)):
                cmd.append("--promote")
            ok, msg = _start_job(state, "learning_loop", cmd, "learning_loop.log")
            return _ok(action=action, message=msg) if ok else _err(msg)

        if action == "learning_full_start":
            backend = str(payload.get("backend", "tiny")).strip()
            if backend not in ("tiny", "temporal"):
                return _err("backend 仅支持 tiny / temporal")
            collect = int(payload.get("collect", 60) or 60)
            collect = max(15, min(collect, 600))
            cmd = [PYTHON, "tools/learning_loop.py", "--backend", backend, "--collect", str(collect)]
            name = str(payload.get("name", "")).strip()
            if name:
                cmd += ["--name", name]
            if bool(payload.get("promote", False)):
                cmd.append("--promote")
            ok, msg = _start_job(state, "learning_loop", cmd, "learning_loop.log")
            return _ok(action=action, message=msg) if ok else _err(msg)

        if action == "learning_stop":
            ok, msg = _stop_job(state, "learning_loop")
            _save_state(state)
            return _ok(action=action, message=msg) if ok else _err(msg)

        return _err(f"未知 action: {action}")
    except (ValueError, OSError, subprocess.SubprocessError) as e:
        return _err(str(e))


def main() -> int:
    parser = argparse.ArgumentParser(description="FlowEngine 运维操作桥接")
    sub = parser.add_subparsers(dest="cmd", required=True)

    status_p = sub.add_parser("status", help="查询 bag 回灌 / 学习闭环任务状态")
    status_p.add_argument("--json", action="store_true", help="兼容 HTTP bridge（忽略）")
    status_p.set_defaults(func=cmd_status)

    run_p = sub.add_parser("run", help="按 JSON action 执行操作")
    run_p.add_argument("--json", action="store_true", help="从 stdin 读取 JSON")
    run_p.set_defaults(func=cmd_run)

    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
