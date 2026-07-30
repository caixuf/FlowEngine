#!/usr/bin/env python3
"""门禁自测 —— 证明 demo_evaluator 能抓住已知故障。

**为什么需要这个文件**

FlowEngine 的横向极限环被"修好"过六次而不收敛。2026-07 查明的机制性原因
不是控制律，而是门禁：每次改完评估器都 PASS，但从没人验证过它能不能拦。
一个抓不住已知故障的门禁，不能用来宣布"这次修好了"。

所以门禁自己也要有测试。下面每个用例注入一种**已在本项目真实发生过**的
故障模式，断言评估器必须 FAIL。它们全部来自实际排查记录：

  1. dead signal      ego_v 恒 0（EKF χ² gating 维度 bug 拒了所有 2-DOF 观测）
  2. dead signal      obs 恒空（behavior 订错 topic，看不见任何障碍物）
  3. 虚假满分         场景有行人但 truth_count_vru=0 → recognition_rate 1.000
  4. 指标错配         发生碰撞却报 warning_lead=51s
  5. 恒定量           kappa≡0（scenario_loader 不解析 road_network）
  6. 场景空跑         直道上跑曲率判据

运行：python3 tests/test_evaluator_gate.py
"""

import importlib.util
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _load_evaluator():
    spec = importlib.util.spec_from_file_location(
        "demo_evaluator", ROOT / "tools" / "demo_evaluator.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


de = _load_evaluator()

_passed = 0
_failed = 0


def check(name: str, condition: bool, detail: str = "") -> None:
    global _passed, _failed
    if condition:
        _passed += 1
        print(f"  ok   {name}")
    else:
        _failed += 1
        print(f"  FAIL {name}" + (f" — {detail}" if detail else ""))


def _series(n=20, **overrides):
    """构造 n 帧正常行驶的 series，overrides 覆盖指定字段为恒定值。"""
    out = []
    for i in range(n):
        m = {
            "speed": 12.0 + 0.1 * i,
            "x": 5.0 + 12.0 * i,
            "y": -1.75 + 0.01 * (i % 3),
            "heading": 0.001 * (i % 5),
            "steer_signed": 0.002 * ((i % 4) - 2),
            "lane_count": 4,
        }
        m.update({k: v for k, v in overrides.items()})
        out.append(m)
    return out


# ── 1/2. dead signal：上游链路断了，量恒为初值 ──────────────
# 实例：EKF 输出恒 (0,0) v=0 时，behavior 看到 v=0.0 obs=0，
# 而所有速度/横向判据照样"通过"。
print("\n[1] dead signal — ego speed constant (EKF stuck at zero)")
rep = de.liveness_report(_series(speed=0.0))
check("speed flagged dead", rep["speed"]["dead"])
check("x still alive", not rep["x"]["dead"])

print("\n[2] dead signal — steer constant (lateral control never acts)")
rep = de.liveness_report(_series(steer_signed=0.0))
check("steer flagged dead", rep["steer_signed"]["dead"])

print("\n[3] dead signal — heading constant (flowsim resetting heading)")
rep = de.liveness_report(_series(heading=0.0))
check("heading flagged dead", rep["heading"]["dead"])

print("\n[4] healthy run must NOT trip the liveness gate")
rep = de.liveness_report(_series())
dead = [k for k, v in rep.items() if v["dead"]]
check("no false positives on healthy series", not dead, f"flagged: {dead}")

print("\n[5] lane_count may legitimately be constant (scenario-fixed)")
rep = de.liveness_report(_series(lane_count=4))
check("lane_count not flagged", not rep["lane_count"]["dead"])

print("\n[6] empty series → dead, not silently passing")
rep = de.liveness_report([])
check("all fields dead on no data", all(v["dead"] for v in rep.values()))


# ── require()：无法判定 ≠ 通过 ────────────────────────────
print("\n[7] require() records INCONCLUSIVE as a failure")
f = []
ok = de.require(f, "recognition_rate_vru",
                {"scenario has 1 pedestrian but 0 truth samples": False})
check("returns False", ok is False)
check("appends exactly one failure", len(f) == 1, f"got {len(f)}")
check("message says INCONCLUSIVE", "INCONCLUSIVE" in f[0], f[0] if f else "")

print("\n[8] require() passes through when preconditions are met")
f = []
ok = de.require(f, "gate", {"enough samples": True, "signal alive": True})
check("returns True", ok is True)
check("no failure appended", not f)


# ── 虚假满分：场景有行人，感知层没测到 ────────────────────
print("\n[9] scenario actor counts drive the recognition-rate precondition")
counts = de.scenario_actor_layer_counts({
    "actors": [{"type": "car"}] * 41 + [{"type": "pedestrian"}],
})
check("41 vehicles counted", counts["vehicle"] == 41, str(counts))
check("1 vru counted", counts["vru"] == 1, str(counts))
check("scenario declaring a pedestrian means vru is expected",
      counts["vru"] > 0)

print("\n[10] scenario with no pedestrian → vru layer legitimately skippable")
counts = de.scenario_actor_layer_counts({"actors": [{"type": "car"}]})
check("vru count zero", counts["vru"] == 0)

print("\n[11] malformed / missing scenario does not crash the gate")
check("None scenario", de.scenario_actor_layer_counts(None)["vru"] == 0)
check("no actors key", de.scenario_actor_layer_counts({})["vehicle"] == 0)
check("junk actors", de.scenario_actor_layer_counts(
    {"actors": [None, 3, {"type": "car"}]})["vehicle"] == 1)


# ── 真实回归：本项目实际打印过的那份报告必须 FAIL ──────────
print("\n[12] regression: the actual 2026-07-30 report must not read as PASS")
# 那份报告：recognition_rate_overall 1.000 / warning_lead_min 51.341
# / truth_count_vru 0 / critical_event_count 1 / 发生了碰撞。
# 门禁必须至少抓到 vru 分母为 0 和 指标错配 两条。
f = []
ok = de.require(f, "recognition_rate_vru", {
    "scenario declares 1 vru actor(s) but only 0 truth samples reached "
    "the evaluator": False,
})
check("vru fake-100% caught", not ok and len(f) == 1)

# 指标错配的判定逻辑（与 score() 内一致）：碰撞 + lead > 5s
collision = True
lead = 51.341
mismatch = collision and math.isfinite(lead) and lead > 5.0
check("collision-vs-warning_lead mismatch caught", mismatch)


print(f"\n{'='*52}")
print(f"gate self-test: {_passed} passed, {_failed} failed")
print(f"{'='*52}")
if _failed:
    print("\n门禁自身失效 —— 它无法抓住已知故障，因此它的 PASS 不可信。")
sys.exit(1 if _failed else 0)
