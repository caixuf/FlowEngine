#!/usr/bin/env python3
"""Training API bridge for C monitor_server.

Reads JSON from stdin, validates it, and calls modelctl.py subcommands.
Used by monitor_server.c POST handlers to execute training operations.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    if len(sys.argv) < 2:
        print(json.dumps({"ok": False, "error": "missing cmd argument"}))
        return 1

    cmd = sys.argv[1]
    try:
        body = sys.stdin.read()
        payload = json.loads(body) if body else {}
    except json.JSONDecodeError as e:
        print(json.dumps({"ok": False, "error": f"invalid JSON: {e}"}))
        return 1

    if cmd == "train-start":
        backend = str(payload.get("backend", "torch"))
        if backend not in ("torch", "tiny"):
            print(json.dumps({"ok": False, "error": "backend must be torch or tiny"}))
            return 1
        model_name = str(payload.get("name", "")).strip()
        if not model_name or not all(c.isalnum() or c in "._-" for c in model_name):
            print(json.dumps({"ok": False, "error": "invalid model name"}))
            return 1

        cmd_args = ["python3", "tools/modelctl.py", "train-start", "--backend", backend, "--name", model_name]
        if payload.get("epochs") is not None:
            cmd_args.extend(["--epochs", str(int(payload["epochs"]))])
        if payload.get("hidden") is not None:
            cmd_args.extend(["--hidden", str(int(payload["hidden"]))])
        if payload.get("run_demo_seconds") is not None:
            cmd_args.extend(["--run-demo-seconds", str(int(payload["run_demo_seconds"]))])
        if payload.get("init_from"):
            cmd_args.extend(["--init-from", str(payload["init_from"])])

        result = subprocess.run(cmd_args, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
        if result.returncode == 0:
            try:
                print(result.stdout)
            except Exception:
                print(json.dumps({"ok": True, "job": {}}))
        else:
            print(json.dumps({"ok": False, "error": result.stdout}))
        return result.returncode

    elif cmd == "promote":
        model_name = str(payload.get("name", "")).strip()
        if not model_name or not all(c.isalnum() or c in "._-" for c in model_name):
            print(json.dumps({"ok": False, "error": "invalid model name"}))
            return 1

        result = subprocess.run(
            ["python3", "tools/modelctl.py", "promote", model_name],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        print(json.dumps({"ok": result.returncode == 0, "returncode": result.returncode, "output": result.stdout}))
        return result.returncode

    else:
        print(json.dumps({"ok": False, "error": f"unknown command: {cmd}"}))
        return 1


if __name__ == "__main__":
    sys.exit(main())
