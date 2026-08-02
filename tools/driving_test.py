#!/usr/bin/env python3
"""FlowEngine 驾校统一考试框架（plan item 第二期）。

与 ci/evaluators/scenario_regression.py 并列的另一档门禁：把"驾校考试"
（科目一~科目四）作为常设质量门禁。本期只挂科目二（tools/subject2_sim.py），
科目一/三/四暂标记为 NOT_IMPLEMENTED，不阻断 overall_pass。

科目二通过 subprocess 调用 subject2_sim.py，假设其接口为：
    python3 tools/subject2_sim.py --all --json-out /tmp/s2_results.json
输出 JSON 记分卡：
    {"items": [{"name","score","pass","deductions"}, ...],
     "total_score": 462, "all_pass": true}

用法:
    # 跑单科
    python3 tools/driving_test.py --subject 2

    # 跑全科（已实现的科目跑，未实现的标 SKIP）
    python3 tools/driving_test.py --all

    # CI 模式：有不及格项则退出码非 0
    python3 tools/driving_test.py --all --ci

    # 把当前成绩写为 baseline
    python3 tools/driving_test.py --update-baseline

    # 与 baseline 对比，单项分数下降 >5 即 FAIL（防退步）
    python3 tools/driving_test.py --baseline

退出码:
    - 默认（无 --ci/--baseline）: 始终 0（本地迭代，"跑通"即可）
    - --ci        : 有已实现科目不及格（任一项目 <90）→ 非 0
    - --baseline  : 任一项目分数较 baseline 下降 >5    → 非 0
    - --update-baseline: 写完即 0
    --ci 与 --baseline 可叠加，任一触发即非 0。

NOT_IMPLEMENTED 科目不参与 overall_pass 判定，不阻断。
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import unicodedata
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # tools/ → 项目根
SUBJECT2_SIM = ROOT / "tools" / "subject2_sim.py"
OUT_DIR = ROOT / "out"
REPORT_PATH = OUT_DIR / "driving_test_report.json"
BASELINE_DIR = ROOT / "tests" / "baseline"
BASELINE_PATH = BASELINE_DIR / "driving_test_baseline.json"

# 单项满分与及格线（真实驾考标准：每项 100，≥90 及格）
ITEM_MAX_SCORE = 100
PASS_THRESHOLD = 90
# baseline 回归阈值：单项分数下降超过此值视为退步
BASELINE_DROP_THRESHOLD = 5

# 科目二五个项目英文名 → 中文显示名
S2_ITEM_NAMES = {
    "reverse_parking": "倒车入库",
    "parallel_parking": "侧方停车",
    "hill_start": "坡道定点停车与起步",
    "s_curve": "曲线行驶",
    "right_angle_turn": "直角转弯",
}

# 全部科目（含未实现，用于全科报告展示）
ALL_SUBJECTS = ("S1", "S2", "S3", "S4")
SUBJECT_LABELS = {
    "S1": "科目一（交规理论）",
    "S2": "科目二（场地五项）",
    "S3": "科目三（道路驾驶）",
    "S4": "科目四（安全文明）",
}


def load_json(path: Path):
    """安全读 JSON；不存在/解析失败返回 None。"""
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                    encoding="utf-8")


def item_display_name(name: str) -> str:
    """项目英文名转中文显示名；未知则原样返回。"""
    return S2_ITEM_NAMES.get(name, name)


def _disp_width(s: str) -> int:
    """字符串显示宽度（CJK 全角算 2）。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def _pad(s: str, width: int) -> str:
    """按显示宽度左对齐补空格。"""
    return s + " " * max(0, width - _disp_width(s))


def _synthetic_fail(reason: str) -> dict:
    """合成一个 FAIL 记分卡（subject2 跑不起来时用）。"""
    return {
        "status": "FAIL",
        "items": [],
        "total_score": 0,
        "max_score": 0,
        "all_pass": False,
        "error": reason,
    }


def not_implemented_subject(subject_id: str) -> dict:
    """未实现科目的占位记分卡：不阻断 overall_pass。"""
    return {
        "status": "NOT_IMPLEMENTED",
        "items": [],
        "total_score": 0,
        "max_score": 0,
        "all_pass": True,  # 不阻断
        "note": f"{SUBJECT_LABELS.get(subject_id, subject_id)} 尚未实现，跳过",
    }


def run_subject2() -> dict:
    """调用 subject2_sim.py --all，解析 --json-out 输出的记分卡。

    subject2_sim.py 不存在/崩溃/无输出时，合成 FAIL 记分卡，
    保证框架自身可跑通（不抛异常）。
    """
    if not SUBJECT2_SIM.exists():
        return _synthetic_fail(f"subject2_sim.py 不存在: {SUBJECT2_SIM}")

    # 用临时文件接收 subject2_sim 的 JSON 记分卡
    fd, tmp_name = tempfile.mkstemp(prefix="s2_results_", suffix=".json")
    os.close(fd)
    tmp_path = Path(tmp_name)
    cmd = [sys.executable, str(SUBJECT2_SIM),
           "--all", "--json-out", str(tmp_path)]
    print(f"\n─── 运行科目二: subject2_sim.py --all ───")
    try:
        try:
            proc = subprocess.run(cmd, cwd=ROOT)
        except FileNotFoundError:
            return _synthetic_fail(f"无法启动: {sys.executable}")
        payload = load_json(tmp_path)
        if payload is None:
            return _synthetic_fail(
                f"subject2_sim.py 未产生有效 JSON（exit={proc.returncode}）")

        items = payload.get("items")
        if not isinstance(items, list) or not items:
            return _synthetic_fail("subject2_sim.py 输出缺少 items 列表")

        # 规范化每项：补全 score/pass 字段，缺 pass 则按分数推断
        norm_items = []
        for it in items:
            if not isinstance(it, dict):
                continue
            name = str(it.get("name", "unknown"))
            score = it.get("score")
            if not isinstance(score, (int, float)):
                score = 0
            passed = it.get("pass")
            if not isinstance(passed, bool):
                passed = score >= PASS_THRESHOLD
            deductions = it.get("deductions", [])
            if not isinstance(deductions, list):
                deductions = []
            norm_items.append({
                "name": name,
                "score": score,
                "pass": passed,
                "deductions": deductions,
            })

        total_score = payload.get("total_score")
        if not isinstance(total_score, (int, float)):
            total_score = sum(it["score"] for it in norm_items)
        all_pass = payload.get("all_pass")
        if not isinstance(all_pass, bool):
            all_pass = all(it["pass"] for it in norm_items) if norm_items else False

        return {
            "status": "OK",
            "items": norm_items,
            "total_score": total_score,
            "max_score": len(norm_items) * ITEM_MAX_SCORE,
            "all_pass": all_pass,
        }
    finally:
        try:
            tmp_path.unlink()
        except OSError:
            pass


def run_subject(subject_id: str) -> dict:
    """按科目分发到对应 runner。"""
    if subject_id == "S2":
        return run_subject2()
    # S1/S3/S4 暂未实现
    return not_implemented_subject(subject_id)


def compare_baseline(report: dict, baseline: dict) -> list[str]:
    """逐项对比 baseline，返回回归说明列表（空=无回归）。

    单项分数较 baseline 下降 > BASELINE_DROP_THRESHOLD 视为回归。
    """
    regressions: list[str] = []
    subjects = report.get("subjects", {})
    for sid, base_items in baseline.items():
        if not isinstance(base_items, dict):
            continue
        subj = subjects.get(sid, {})
        if subj.get("status") != "OK":
            continue
        cur_map = {it["name"]: it["score"] for it in subj.get("items", [])}
        for item_name, base_score in base_items.items():
            if not isinstance(base_score, (int, float)):
                continue
            cur_score = cur_map.get(item_name)
            if not isinstance(cur_score, (int, float)):
                regressions.append(
                    f"{sid}/{item_name}: 当前缺失（baseline={base_score}）")
                continue
            drop = base_score - cur_score
            if drop > BASELINE_DROP_THRESHOLD:
                regressions.append(
                    f"{sid}/{item_name}: {cur_score} < {base_score} "
                    f"(下降 {drop} > {BASELINE_DROP_THRESHOLD})")
    return regressions


def build_baseline_from_report(report: dict) -> dict:
    """从当前报告投影出 baseline（仅已实现科目 + 各项分数）。"""
    baseline: dict = {}
    for sid, subj in report.get("subjects", {}).items():
        if subj.get("status") != "OK":
            continue
        baseline[sid] = {it["name"]: it["score"]
                         for it in subj.get("items", [])}
    return baseline


def print_table(report: dict, regressions: list[str] | None) -> None:
    """打印终端记分卡表格。"""
    print("\n==================== 驾校考试报告 ====================")
    print(f"{_pad('科目', 6)} {_pad('项目', 22)} {_pad('分数', 6)} 结果")
    print("-" * 50)
    grand_score = 0
    grand_max = 0
    for sid in ALL_SUBJECTS:
        subj = report.get("subjects", {}).get(sid)
        if subj is None:
            continue
        status = subj.get("status", "OK")
        if status == "NOT_IMPLEMENTED":
            print(f"{_pad(sid, 6)} {_pad('(未实现)', 22)} {_pad('-', 6)} SKIP")
            continue
        items = subj.get("items", [])
        if not items:
            # FAIL（跑不起来）：单行展示错误（截断）
            err = subj.get("error", "无项目")[:20]
            print(f"{_pad(sid, 6)} {_pad(err, 22)} {_pad('-', 6)} FAIL")
            grand_max += subj.get("max_score", 0)
            continue
        for it in items:
            score = it["score"]
            result = "PASS" if it["pass"] else "FAIL"
            print(f"{_pad(sid, 6)} {_pad(item_display_name(it['name']), 22)} "
                  f"{_pad(str(score), 6)} {result}")
        grand_score += subj.get("total_score", 0)
        grand_max += subj.get("max_score", 0)
    print("=" * 50)
    overall = report.get("overall_pass", False)
    print(f"总分: {grand_score}/{grand_max}  全科及格: {'YES' if overall else 'NO'}")

    if regressions:
        print("\n── baseline 回归 ──")
        for r in regressions:
            print(f"  REGRESS  {r}")


def build_report(subjects_to_run: list[str]) -> dict:
    """跑指定科目，组装报告。"""
    subjects: dict[str, dict] = {}
    for sid in subjects_to_run:
        print(f"\n>>> 考试科目: {sid} ({SUBJECT_LABELS.get(sid, '')})")
        subjects[sid] = run_subject(sid)

    # overall_pass：只看已实现科目（status==OK）；FAIL 也算不过；NOT_IMPLEMENTED 不阻断
    overall_pass = True
    for subj in subjects.values():
        status = subj.get("status", "OK")
        if status == "OK":
            overall_pass = overall_pass and subj.get("all_pass", False)
        elif status == "FAIL":
            overall_pass = False

    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "subjects": subjects,
        "overall_pass": overall_pass,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = parser.add_mutually_exclusive_group()
    g.add_argument("--subject", type=str,
                   help="只跑某一科（1/2/3/4）")
    g.add_argument("--all", action="store_true",
                   help="跑全科（已实现的科目跑，未实现的标 SKIP）")
    parser.add_argument("--ci", action="store_true",
                        help="CI 模式：有不及格项则退出码非 0")
    parser.add_argument("--update-baseline", action="store_true",
                        help="把当前成绩写入 baseline 并退出")
    parser.add_argument("--baseline", action="store_true",
                        help="与 baseline 对比，单项分数下降 >5 即 FAIL")
    args = parser.parse_args()

    # 决定要跑哪些科目
    if args.subject:
        sid = f"S{args.subject}"
        if sid not in ALL_SUBJECTS:
            raise SystemExit(f"未知科目: {args.subject}（可选 1/2/3/4）")
        subjects_to_run = [sid]
    else:
        # 默认 / --all / --baseline / --update-baseline 均跑全科
        subjects_to_run = list(ALL_SUBJECTS)

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    report = build_report(subjects_to_run)

    # baseline 对比
    regressions: list[str] = []
    if args.baseline:
        baseline = load_json(BASELINE_PATH)
        if baseline is None:
            print(f"\n⚠ 未找到 baseline: {BASELINE_PATH}"
                  f"（建议先运行 --update-baseline）")
        else:
            regressions = compare_baseline(report, baseline)
        report["baseline_regressions"] = regressions

    # --update-baseline：投影并写盘后即退出（仍保存报告）
    if args.update_baseline:
        BASELINE_DIR.mkdir(parents=True, exist_ok=True)
        baseline = build_baseline_from_report(report)
        write_json(BASELINE_PATH, baseline)
        write_json(REPORT_PATH, report)
        print(f"\n已更新 baseline: {BASELINE_PATH}")
        print(f"报告已保存: {REPORT_PATH}")
        return 0

    # 保存报告
    write_json(REPORT_PATH, report)

    # 打印终端表格
    print_table(report, regressions if args.baseline else None)
    print(f"\n报告已保存: {REPORT_PATH}")

    # 退出码判定
    exit_code = 0
    if args.ci:
        # 有已实现科目不及格 / 跑失败 → 非 0
        for subj in report["subjects"].values():
            status = subj.get("status", "OK")
            if status == "FAIL":
                exit_code = 2
                break
            if status == "OK" and not subj.get("all_pass", False):
                exit_code = 2
                break
    if args.baseline and regressions:
        exit_code = 2

    if exit_code != 0:
        reason = []
        if args.ci:
            reason.append("有科目不及格")
        if args.baseline and regressions:
            reason.append(f"{len(regressions)} 项 baseline 回归")
        print("\n❌ 考试未通过：" + "；".join(reason))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
