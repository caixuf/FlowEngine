#!/usr/bin/env python3
"""FlowEngine scenario-matrix regression harness.

This is the "simulation-as-testing" driver (plan item B2). It runs the whole
scenario suite through ``demo_evaluator.py`` — one full simulation per scenario —
collects each PASS/FAIL result plus its numeric summary, and (optionally) diffs
those numbers against a saved *baseline* so that a code change that quietly
degrades behaviour shows up as a regression.

    # Run the full suite and print a matrix report
    python3 tools/scenario_regression.py

    # Record the current results as the regression baseline
    python3 tools/scenario_regression.py --update-baseline

    # Run and compare against the saved baseline (fail on regression)
    python3 tools/scenario_regression.py --baseline

Exit code is 0 only when every scenario PASSes and (when --baseline is given)
no numeric regression exceeds the suite tolerances.

------------------------------------------------------------------------------
Extending this harness (for follow-up implementers):
  * Add scenarios by editing ``scenarios/suite.json`` — no code change needed.
  * Tighten/loosen numeric regression gates via ``baseline_tolerances`` there.
  * The only two functions worth touching are ``run_scenario`` (how one scenario
    is executed/scored) and ``compare_summary`` (how a regression is decided).
------------------------------------------------------------------------------
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVALUATOR = ROOT / "tools" / "demo_evaluator.py"
DEFAULT_SUITE = ROOT / "scenarios" / "suite.json"
DEFAULT_RESULTS_DIR = Path("/tmp/flow_regression")
DEFAULT_BASELINE_DIR = ROOT / "tests" / "baseline"


def load_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def load_suite(path: Path) -> dict:
    suite = load_json(path)
    if not isinstance(suite, dict) or not isinstance(suite.get("scenarios"), list):
        raise SystemExit(f"invalid suite manifest: {path}")
    return suite


def enabled_scenarios(suite: dict) -> list[dict]:
    return [s for s in suite["scenarios"]
            if isinstance(s, dict) and s.get("file") and s.get("enabled", True)]


def scenario_key(entry: dict) -> str:
    """Stable identifier for a scenario (its file stem)."""
    return Path(entry["file"]).stem


def run_scenario(entry: dict, default_duration: int, interval: float,
                 results_dir: Path) -> dict:
    """Run one scenario through demo_evaluator and return its result payload.

    Returns the dict written by ``demo_evaluator --json-out``:
        {scenario, result, failures, warnings, summary}
    On launch failure a synthetic FAIL payload is returned so the matrix stays
    complete rather than aborting the whole suite.
    """
    key = scenario_key(entry)
    out_path = results_dir / f"{key}.json"
    duration = int(entry.get("duration_s", default_duration))
    cmd = [
        sys.executable, str(EVALUATOR),
        "--scenario", entry["file"],
        "--duration", str(duration),
        "--interval", str(interval),
        "--json-out", str(out_path),
    ]
    print(f"\n─── running scenario '{key}' ({duration}s) ───")
    proc = subprocess.run(cmd, cwd=ROOT)
    payload = load_json(out_path)
    if payload is None:
        payload = {
            "scenario": key,
            "result": "FAIL",
            "failures": [f"evaluator produced no result (exit={proc.returncode})"],
            "warnings": [],
            "summary": {},
        }
    return payload


def compare_summary(baseline: dict, current: dict, tolerances: dict) -> list[str]:
    """Return a list of regression messages (empty == no regression).

    Only keys present in ``tolerances`` are gated. Supported gate types:
      * ``min_ratio``          — current >= baseline * ratio
      * ``max_abs_increase``   — current <= baseline + delta
    Missing/non-numeric values are skipped (they cannot regress meaningfully).
    """
    regressions: list[str] = []
    for metric, rule in tolerances.items():
        if metric.startswith("_") or not isinstance(rule, dict):
            continue
        base = baseline.get(metric)
        cur = current.get(metric)
        if not isinstance(base, (int, float)) or not isinstance(cur, (int, float)):
            continue
        if "min_ratio" in rule:
            threshold = base * rule["min_ratio"]
            if cur < threshold:
                regressions.append(
                    f"{metric}: {cur:.3f} < {threshold:.3f} "
                    f"(baseline {base:.3f} x {rule['min_ratio']})"
                )
        if "max_abs_increase" in rule:
            threshold = base + rule["max_abs_increase"]
            if cur > threshold:
                regressions.append(
                    f"{metric}: {cur:.3f} > {threshold:.3f} "
                    f"(baseline {base:.3f} + {rule['max_abs_increase']})"
                )
    return regressions


def print_report(rows: list[dict]) -> None:
    print("\n==================== Regression Matrix ====================")
    header = f"{'scenario':<26} {'result':<6} {'regress':<8} notes"
    print(header)
    print("-" * len(header))
    for row in rows:
        note = ""
        if row["failures"]:
            note = row["failures"][0]
        elif row["regressions"]:
            note = row["regressions"][0]
        print(f"{row['scenario']:<26} {row['result']:<6} "
              f"{('YES' if row['regressions'] else '-'):<8} {note}")
    print("=" * len(header))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--suite", type=Path, default=DEFAULT_SUITE,
                        help="scenario suite manifest (default: scenarios/suite.json)")
    parser.add_argument("--interval", type=float, default=0.25,
                        help="sample interval passed to demo_evaluator")
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS_DIR,
                        help="where per-scenario result JSON is written")
    parser.add_argument("--baseline-dir", type=Path, default=DEFAULT_BASELINE_DIR,
                        help="directory holding baseline result JSON files")
    parser.add_argument("--baseline", action="store_true",
                        help="compare results against the saved baseline and fail on regression")
    parser.add_argument("--update-baseline", action="store_true",
                        help="write the current results into the baseline directory and exit 0")
    parser.add_argument("--only", type=str, default=None,
                        help="run only the scenario whose file stem matches this value")
    parser.add_argument("--dry-run", action="store_true",
                        help="list the scenarios that would run without executing the demo")
    args = parser.parse_args()

    suite = load_suite(args.suite)
    scenarios = enabled_scenarios(suite)
    if args.only:
        scenarios = [s for s in scenarios if scenario_key(s) == args.only]
        if not scenarios:
            raise SystemExit(f"no enabled scenario matches --only {args.only!r}")
    default_duration = int(suite.get("default_duration_s", 30))
    raw_tolerances = suite.get("baseline_tolerances")
    tolerances = raw_tolerances if isinstance(raw_tolerances, dict) else {}

    if args.dry_run:
        print(f"suite: {suite.get('name')} ({len(scenarios)} scenarios)")
        for entry in scenarios:
            print(f"  - {scenario_key(entry):<26} "
                  f"{entry.get('duration_s', default_duration)}s  {entry['file']}")
        return 0

    args.results_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    for entry in scenarios:
        key = scenario_key(entry)
        payload = run_scenario(entry, default_duration, args.interval, args.results_dir)
        summary = payload.get("summary", {}) if isinstance(payload.get("summary"), dict) else {}
        failures = payload.get("failures", []) or []
        regressions: list[str] = []

        if args.baseline:
            baseline_payload = load_json(args.baseline_dir / f"{key}.json")
            if baseline_payload is None:
                regressions.append("no baseline recorded for this scenario")
            else:
                base_summary = baseline_payload.get("summary", {})
                regressions = compare_summary(base_summary, summary, tolerances)

        rows.append({
            "scenario": key,
            "result": payload.get("result", "FAIL"),
            "failures": failures,
            "warnings": payload.get("warnings", []) or [],
            "regressions": regressions,
        })

    if args.update_baseline:
        args.baseline_dir.mkdir(parents=True, exist_ok=True)
        for entry in scenarios:
            key = scenario_key(entry)
            src = args.results_dir / f"{key}.json"
            if src.exists():
                (args.baseline_dir / f"{key}.json").write_text(
                    src.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"\nupdated baseline in {args.baseline_dir} "
              f"({len(scenarios)} scenarios)")
        return 0

    print_report(rows)

    failed = [r for r in rows if r["result"] != "PASS"]
    regressed = [r for r in rows if r["regressions"]]
    if failed:
        print(f"\n{len(failed)} scenario(s) FAILED behavioral checks.")
    if args.baseline and regressed:
        print(f"{len(regressed)} scenario(s) REGRESSED vs baseline.")
    if failed or (args.baseline and regressed):
        return 2
    print("\nALL scenarios PASS within the regression envelope.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
