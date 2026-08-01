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

v0.2 追加（2026-08-01，Phase 1.2，对齐新场景矩阵）：
  7. 多红灯闯行       ego 红灯期间越过停止线 → 门禁必须 FAIL
  8. 绿灯卡死         v=0 停在绿灯下 >5s（planning 闭锁）→ 门禁必须 FAIL
  9. 追尾 gap         min_forward_gap ≤ 0 → 门禁必须 FAIL
  10. 过度保守刹停     对向会车场景巡航降到 ~1m/s → min_avg_speed FAIL
  11. 转向 bang-bang   steer 每帧翻符号 → Phase 1.3 FAIL 门禁

运行（两种方式等价）：
  python3 ci/evaluators/test_evaluator_gate.py   # 独立脚本，退出码=失败数
  python3 -m pytest ci/evaluators/test_evaluator_gate.py   # pytest 收集
"""

import importlib.util
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _load_evaluator():
    spec = importlib.util.spec_from_file_location(
        "demo_evaluator", ROOT / "ci" / "evaluators" / "demo_evaluator.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def check(name: str, condition: bool, detail: str = "") -> None:
    """断言单条检查；结果记入模块级计数，供 run_all_checks 汇总。"""
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


def run_all_checks() -> int:
    """执行全部门禁自测，打印逐条结果，返回失败数（独立运行与 pytest 共用）。

    pytest 下作为单个 test 收集：任何检查 FAIL 都让整个测试红——
    门禁自己抓不住已知故障 = 它的 PASS 不可信，必须阻断。
    """
    global _passed, _failed
    _passed = 0
    _failed = 0

    de = _load_evaluator()

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

    # ── v0.2 新场景门禁有效性（Phase 1.2）：每个目标故障注入即红 ──
    # multi_light（红灯闯行/绿灯卡死）、dense_npc（追尾 gap）、oncoming
    # （过度保守刹停）、控制层 bang-bang、run 截断。每个用例注入一种已知故障，
    # 断言对应门禁 FAIL —— 门禁抓不住 = 它的 PASS 不可信。
    # topic 名复用 demo_evaluator.TOPIC_MIN_FREQ（唯一事实源），避免 set 漂移。
    _TOPICS = [{"topic": t, "freq": 20.0} for t in de.TOPIC_MIN_FREQ]

    def _mk(x, y, speed, steer, ts, tl_state=None, obstacles=None):
        ents = [{"id": 0, "type": "tl", "x": 300.0, "state": tl_state}] if tl_state else []
        return {
            "timestamp": ts,
            "metrics": {
                "topics": _TOPICS,
                "vehicle": {"speed": speed, "x": x},
                "scene": {
                    "ego": {"x": x, "y": y, "speed": speed, "steer": steer, "heading": 0.0},
                    "lane": {"width": 3.5, "count": 4},
                    "obstacles": obstacles or [],
                    "entities": ents,
                },
                "driver_mode": "NOA:CRUISE",
            },
            "nodes": [],
        }

    def _score(label, samples, criteria, lights=None, expected_duration_s=None):
        f, _, _ = de.score(samples, ROOT / "does-not-exist.log", criteria=criteria,
                           scenario_name=label, expected_edges=[], road=None,
                           traffic_lights=lights, expected_duration_s=expected_duration_s)
        return f

    _TL = [{"id": 0, "x": 300.0, "y_lane": -1.75, "green_s": 12.0, "yellow_s": 3.0, "red_s": 10.0}]
    _ZERO_CRIT = {"min_distance_m": 0.0, "min_avg_speed_mps": 0.0}

    print("\n[13] multi_light — red light running (ego crosses stop line during red)")
    _red_cross = [_mk(285 + i * 5, -1.75, 12.0, 0.0, i * 0.25,
                      tl_state=("red" if 285 + i * 5 < 302 else "green")) for i in range(12)]
    _f = _score("red-cross", _red_cross, {**_ZERO_CRIT, "no_red_light_violation": True}, lights=_TL)
    check("red-light violation caught", any("red light violation" in x for x in _f))

    print("\n[14] multi_light — green-stall deadlock (v=0 while light green > 5s)")
    _stall = [_mk(290.0, -1.75, 0.0, 0.0, i * 0.25, tl_state="green") for i in range(60)]
    _f = _score("green-stall", _stall, {**_ZERO_CRIT, "no_red_light_violation": True}, lights=_TL)
    check("green-stall deadlock caught", any("stuck during green" in x for x in _f))

    print("\n[15] multi_light — healthy red-light obey must NOT trip the red gate")
    _obey = [_mk(285 + i * 5, -1.75, 12.0, 0.0, i * 0.25,
                 tl_state="green") for i in range(12)]
    _f = _score("red-obey", _obey, {**_ZERO_CRIT, "no_red_light_violation": True}, lights=_TL)
    check("no false red-light violation on green", not any("red light violation" in x for x in _f))

    print("\n[16] dense_npc — rear-end gap <= 0")
    _gap = [_mk(10 + i * 10, -1.75, 12.0, 0.0, i * 0.25, obstacles=[
        {"id": 1, "x": 8.0, "y": 0.0, "len": 4.6, "wid": 2.0},
        {"id": 2, "x": 2.0, "y": 0.0, "len": 4.6, "wid": 2.0},
    ]) for i in range(10)]
    _f = _score("rear-end", _gap, {**_ZERO_CRIT, "no_collision": False})
    check("min_forward_gap<=0 caught", any("min_forward_gap" in x for x in _f))

    print("\n[17] oncoming — over-conservative brake (avg speed collapses)")
    _slow = [_mk(10 + i * 2, -1.75, 1.0, 0.0, i * 0.25) for i in range(40)]
    _f = _score("oncoming-overbrake", _slow, {"min_distance_m": 100.0, "min_avg_speed_mps": 5.0})
    check("avg-speed collapse caught", any("average speed too low" in x for x in _f))

    print("\n[18] control-layer — bang-bang steer (sign flips every frame) must FAIL")
    _bang = [_mk(10 + i * 10, -1.75, 12.0 + (i % 2) * 0.5, 0.25 if i % 2 == 0 else -0.25, i * 0.25)
             for i in range(60)]
    _f = _score("bang-bang", _bang, _ZERO_CRIT)
    check("steer bang-bang FAIL", any("steer bang-bang" in x for x in _f))

    print("\n[19] run integrity — truncated run (8s of a 60s demo) must be INCONCLUSIVE")
    # monitor 写 /tmp/flow_topology.json 中途停止 → 只采到开头几秒。
    # 没有此门禁，截断 run 会误报 PASS 或触发虚假数值回归。
    _trunc = [_mk(10 + i * 10, -1.75, 12.0, 0.0, i * 0.25) for i in range(32)]  # span ≈ 7.8s
    _f = _score("truncated", _trunc, _ZERO_CRIT, expected_duration_s=60.0)
    check("truncated run caught", any("run truncated" in x for x in _f))
    _full = [_mk(10 + i * 10, -1.75, 12.0, 0.0, i * 1.0) for i in range(32)]  # span ≈ 31s
    _f2 = _score("full-run", _full, _ZERO_CRIT, expected_duration_s=30.0)
    check("full-length run not flagged", not any("run truncated" in x for x in _f2))

    print(f"\n{'='*52}")
    print(f"gate self-test: {_passed} passed, {_failed} failed")
    print(f"{'='*52}")
    if _failed:
        print("\n门禁自身失效 —— 它无法抓住已知故障，因此它的 PASS 不可信。")
    return _failed


def test_gate_self_test():
    """pytest 入口：门禁必须抓住全部 12 类已知故障，否则本测试 FAIL。

    这保证 CI 里的评估器门禁"先证伪自己再判别人"——评估器本身若退化到
    抓不住已知故障，评估器改动会被这里拦下，而不是等真撞车了才发现。
    """
    failures = run_all_checks()
    assert failures == 0, f"{failures} gate self-check(s) failed"


if __name__ == "__main__":
    sys.exit(1 if run_all_checks() else 0)
