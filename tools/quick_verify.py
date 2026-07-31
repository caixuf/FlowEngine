#!/usr/bin/env python3
"""quick_verify.py — 交互式算法快速验证工具（无需重编译）。

核心思路：启动一次 demo 后，通过 flowctl param set 热改参数，
实时观察仪表盘上的关键指标，快速验证算法效果。类似 auto_tune_mpc.py
但更通用：覆盖 control/behavior/planning 全部可热调参数，支持交互式
调参 + 实时仪表盘 + 快速评估，不需要反复重编译和重启。

用法:
    python3 tools/quick_verify.py [--duration 120] [--no-browser] [--scenario FILE]
    python3 tools/quick_verify.py --eval-only    # 不启动demo，连接已有实例

内置测试场景:
    urban_challenge  — 前车急刹+行人横穿+红绿灯（综合鲁棒性测试）
    straight_road    — 默认直路场景
    lane_change_traffic — 密集车流变道

交互命令（运行中输入）:
    set <param> <value>    热改参数，例如: set behavior.acc_time_headway 2.0
    get <param>            查看当前参数值
    eval [seconds]         采样N秒计算CTE/速度/跟车/变道指标
    reset                  重置所有参数到baseline
    baseline               保存当前参数为新baseline
    params [prefix]        列出所有可热调参数（按前缀过滤）
    log                    开关实时BEH-DBG日志显示
    quit / q               退出（停止demo）

Examples:
    # 启动验证（自动起demo，打开浏览器）
    python3 tools/quick_verify.py

    # 只调跟车时距，看CTE和跟车间距变化
    > set behavior.acc_time_headway 2.0
    > eval 30
    > set behavior.acc_k_gap 0.6
    > eval 30
    > reset
"""
import argparse
import csv
import json
import math
import os
import statistics
import subprocess
import sys
import time
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FLOWCTL = ROOT / "build" / "bin" / "flowctl"
JSON_FILE = Path("/tmp/flow_topology.json")
SOCK_FILE = Path("/tmp/flow_param.sock")

# 已知可热调参数列表（用于 params 命令和 reset 时恢复 baseline）
TUNABLE_PARAMS = {
    # ── Behavior（行为决策——速度决策唯一来源） ──
    "behavior.cruise_speed":        (15.0,  "巡航目标速度 m/s（Apollo原则：behavior决策速度）"),
    # ── Control（横向控制——纯轨迹跟随器，不做速度决策） ──
    "control.pid_kp":               (0.5,   "纵向PID P增益"),
    "control.pid_ki":               (0.05,  "纵向PID I增益"),
    "control.pid_kd":               (0.2,   "纵向PID D增益"),
    "control.lat_kp":               (0.5,   "横向P增益（Stanley）"),
    "control.lat_kd_heading":       (3.5,   "航向阻尼增益"),
    "control.yaw_damping":          (0.3,   "横摆角速度阻尼"),
    "control.lat_lookahead_gain":   (1.0,   "前瞻时间增益(s)"),
    "control.k_v_lat":              (0.22,  "横向速度阻尼增益（防超调）"),
    "control.k_vy":                 (0.3,   "v_y_des位置增益"),
    "control.k_vy_damp":            (0.6,   "v_y_des速度阻尼"),
    "control.mpc_horizon":          (0,     "MPC预测时域(0=禁用MPC,用Stanley)"),
    # ── Behavior（跟车/变道决策） ──
    "behavior.acc_standoff":        (5.0,   "ACC静止安全余量(m)"),
    "behavior.acc_time_headway":    (1.5,   "ACC时距(s): desired_gap=standoff+headway*v"),
    "behavior.acc_k_gap":           (0.4,   "ACC间距误差增益(1/s)"),
    "behavior.acc_gap_err_clamp":   (8.0,   "ACC间距误差修正上限(m/s)"),
    "behavior.blocked_range_mult":  (4.0,   "blocked检测距离倍数"),
    "behavior.blocked_range_min":   (30.0,  "blocked检测最小距离(m)"),
    "behavior.follow_hysteresis":   (1.3,   "FOLLOW退出滞环倍数"),
    "behavior.lane_change_timeout_s":(8.0,  "变道超时(s)"),
    "behavior.lane_change_cooldown_s":(3.0, "变道冷却(s)"),
    "behavior.lc_gap_mult":         (1.5,   "目标车道间距阈值倍数"),
    "behavior.rear_safe_min_m":     (15.0,  "后向安全最小距离(m)"),
    "behavior.rear_safe_time_s":    (3.0,   "后向安全时距(s)"),
    "behavior.same_lane_tol_offset":(0.6,   "车道横向容差偏移(m)"),
}

# ANSI colors
class C:
    R = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GRAY = '\033[90m'


def flowctl(*args: str) -> tuple[bool, str]:
    try:
        r = subprocess.run([str(FLOWCTL)] + list(args),
                           capture_output=True, text=True, timeout=10)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except Exception as e:
        return False, str(e)


def read_metrics() -> dict | None:
    """读 /tmp/flow_topology.json，提取 ego + behavior + entities + obstacles。"""
    try:
        if not JSON_FILE.exists() or JSON_FILE.stat().st_size == 0:
            return None
        with open(JSON_FILE) as f:
            d = json.load(f)
        metrics = d.get("metrics", {})
        scene = metrics.get("scene", {})
        ego = scene.get("ego", {})
        beh = metrics.get("behavior", {})
        entities = scene.get("entities", [])
        obstacles = scene.get("obstacles", [])
        lane = scene.get("lane", {})
        if not ego:
            return None
        return {
            "ego": ego,
            "beh": beh,
            "entities": entities if isinstance(entities, list) else [],
            "obstacles": obstacles if isinstance(obstacles, list) else [],
            "lane": lane,
            "t": d.get("timestamp", 0),
        }
    except (json.JSONDecodeError, OSError, KeyError):
        return None


def compute_cte(y: float, lane_count: int = 4, lane_width: float = 3.5) -> float:
    """计算横向偏差（相对最近车道中心）。"""
    offset = -y / lane_width + (lane_count - 1) * 0.5
    idx = max(0, min(lane_count - 1, int(round(offset))))
    lcy = -(idx - (lane_count - 1) * 0.5) * lane_width
    return y - lcy


# ── 几何与变道分析（移植自 plant_eval.py） ───────────────────────

def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def load_scenario_road(scenario_path: str = None) -> dict:
    """从场景 JSON 加载道路参数。"""
    if not scenario_path:
        scenario_path = str(ROOT / "scenarios" / "straight_road.json")
    path = Path(scenario_path)
    if not path.exists():
        return {"lane_count": 4, "lane_width": 3.5}
    with open(path) as f:
        sc = json.load(f)
    rn = sc.get("road_network", {})
    edges = rn.get("edges", [])
    if edges:
        e = edges[0]
        return {
            "lane_count": int(e.get("lanes", 4)),
            "lane_width": float(e.get("lane_width", 3.5)),
        }
    road = sc.get("road", {})
    lane_cfg = sc.get("lane", {})
    return {
        "lane_count": int(road.get("lane_count", lane_cfg.get("count", 4))),
        "lane_width": float(road.get("lane_width", lane_cfg.get("width", 3.5))),
    }


def extract_series(samples: list[dict]) -> dict:
    """从 samples 中提取 ego 时间序列。"""
    xs, ys, headings, speeds, steers = [], [], [], [], []
    timestamps = []
    for s in samples:
        ex = s.get("x", 0)
        if not ex:
            continue
        xs.append(ex)
        ys.append(s.get("y", 0))
        headings.append(s.get("heading", 0))
        speeds.append(s.get("speed", 0))
        steers.append(s.get("steer", 0))
        timestamps.append(s.get("t", 0))
    if not xs:
        return {}
    # Steer flip count
    flip_count = 0
    prev_s = steers[0] if steers else 0
    for s in steers[1:]:
        if prev_s * s < 0 and abs(s - prev_s) > 0.01:
            flip_count += 1
        prev_s = s
    duration = timestamps[-1] - timestamps[0] if timestamps else 1.0
    flip_rate = flip_count / max(duration, 1.0)
    # Yaw rate
    yaw_rates = [0.0]
    for i in range(1, len(headings)):
        dt = timestamps[i] - timestamps[i-1]
        yaw_rates.append(abs(angle_diff(headings[i], headings[i-1])) / max(dt, 0.01))
    # Steer rate
    steer_rates = [0.0]
    for i in range(1, len(steers)):
        dt = timestamps[i] - timestamps[i-1]
        steer_rates.append(abs(steers[i] - steers[i-1]) / max(dt, 0.01))
    return {
        "xs": xs, "ys": ys, "headings": headings, "speeds": speeds, "steers": steers,
        "timestamps": timestamps, "duration": duration,
        "final_x": xs[-1], "avg_speed": statistics.fmean(speeds) if speeds else 0,
        "flip_rate": flip_rate, "yaw_rates": yaw_rates, "steer_rates": steer_rates,
    }


def lane_idx_from_y(y: float, lane_count: int, lane_width: float) -> int:
    offset = -y / lane_width + (lane_count - 1) * 0.5
    return max(0, min(lane_count - 1, int(round(offset))))


def detect_lane_changes(xs, ys, timestamps, lane_count: int, lane_width: float) -> list[dict]:
    """检测变道事件，返回事件列表。"""
    events = []
    if not xs:
        return events
    last_lane = lane_idx_from_y(ys[0], lane_count, lane_width)
    in_transition = False
    transition_start = None
    start_cte = None
    start_idx = 0
    for i in range(1, len(ys)):
        cur_lane = lane_idx_from_y(ys[i], lane_count, lane_width)
        if cur_lane != last_lane and not in_transition:
            in_transition = True
            transition_start = timestamps[i]
            start_cte = compute_cte(ys[i], lane_count, lane_width)
            last_lane = cur_lane
            start_idx = i
        if in_transition:
            stable_count = 0
            for j in range(i, min(i + 5, len(ys))):
                if lane_idx_from_y(ys[j], lane_count, lane_width) == cur_lane:
                    stable_count += 1
            if stable_count >= 3:
                end_cte = compute_cte(ys[i], lane_count, lane_width)
                seg_cte = [compute_cte(ys[j], lane_count, lane_width) for j in range(start_idx, i + 1)]
                events.append({
                    "start_time": transition_start,
                    "end_time": timestamps[i],
                    "duration": timestamps[i] - transition_start,
                    "cte_delta": abs(end_cte - start_cte),
                    "cte_rms": math.sqrt(statistics.fmean(c*c for c in seg_cte)) if seg_cte else 0,
                    "overshoot": max(abs(c) for c in seg_cte) - abs(end_cte) if seg_cte else 0,
                    "direction": "left" if end_cte > start_cte else "right",
                })
                in_transition = False
    return events


def compute_straight_metrics(series: dict, lane_count: int, lane_width: float) -> dict:
    """直线段指标（排除变道区间）。"""
    ys = series.get("ys", [])
    if not ys:
        return {}
    in_lc = [False] * len(ys)
    for i in range(1, len(ys)):
        cur = lane_idx_from_y(ys[i], lane_count, lane_width)
        prev = lane_idx_from_y(ys[i-1], lane_count, lane_width)
        if cur != prev:
            for j in range(max(0, i-5), min(len(ys), i+8)):
                in_lc[j] = True
    ctes = [compute_cte(ys[i], lane_count, lane_width) for i in range(len(ys)) if not in_lc[i]]
    yaw = [series["yaw_rates"][i] for i in range(len(series["yaw_rates"])) if not in_lc[i]] if "yaw_rates" in series else []
    sr = [series["steer_rates"][i] for i in range(len(series["steer_rates"])) if not in_lc[i]] if "steer_rates" in series else []
    if not ctes:
        return {}
    return {
        "cte_rms": math.sqrt(statistics.fmean(c*c for c in ctes)),
        "cte_max": max(abs(c) for c in ctes),
        "yaw_rate_rms": math.sqrt(statistics.fmean(y*y for y in yaw)) if yaw else 0,
        "steer_rate_rms": math.sqrt(statistics.fmean(s*s for s in sr)) if sr else 0,
        "flip_rate": series.get("flip_rate", 0),
        "straight_sample_count": len(ctes),
    }


def format_state(state: str) -> str:
    colors = {
        "CRUISE": C.GREEN,
        "FOLLOW": C.YELLOW,
        "LEFT_CHANGE": C.CYAN,
        "RIGHT_CHANGE": C.BLUE,
        "STOP": C.RED,
        "EMERGENCY": f"{C.BOLD}{C.RED}",
    }
    c = colors.get(state, "")
    return f"{c}{state}{C.R}" if c else state


def format_gap(gap: float) -> str:
    if gap < 0:
        return f"{C.GRAY}--{C.R}"
    if gap < 10:
        return f"{C.RED}{gap:.0f}m{C.R}"
    if gap < 30:
        return f"{C.YELLOW}{gap:.0f}m{C.R}"
    return f"{C.GREEN}{gap:.0f}m{C.R}"


def format_speed(v: float, target: float) -> str:
    diff = v - target
    if abs(diff) < 1.0:
        return f"{C.GREEN}{v:.1f}{C.R}"
    if diff > 2.0:
        return f"{C.RED}{v:.1f}{C.R}"
    if diff < -2.0:
        return f"{C.YELLOW}{v:.1f}{C.R}"
    return f"{C.CYAN}{v:.1f}{C.R}"


class Dashboard:
    def __init__(self, scenario_path: str = None):
        self.running = True
        self.show_log = False
        self.baseline = {}
        self.scenario_path = scenario_path
        self._save_baseline()

    def _save_baseline(self):
        """读取当前所有参数作为baseline。"""
        for name in TUNABLE_PARAMS:
            ok, out = flowctl("param", "get", name)
            if ok:
                try:
                    val = float(out.split("=")[-1].strip())
                    self.baseline[name] = val
                except (ValueError, IndexError):
                    self.baseline[name] = TUNABLE_PARAMS[name][0]
            else:
                self.baseline[name] = TUNABLE_PARAMS[name][0]

    def reset_params(self):
        """重置到baseline。"""
        for name, val in self.baseline.items():
            flowctl("param", "set", name, str(val))

    def render(self, m: dict | None):
        if not m:
            sys.stdout.write(f"\r{C.GRAY}等待仿真数据...{C.R}" + " " * 20)
            sys.stdout.flush()
            return
        ego = m["ego"]
        beh = m.get("beh", {})

        x = ego.get("x", 0)
        y = ego.get("y", 0)
        speed = ego.get("speed", 0)
        heading = ego.get("heading", 0)
        steer = ego.get("steer", 0)
        cte = compute_cte(y)

        state = beh.get("state", "?")
        tgt_speed = beh.get("target_speed", 20.0)
        best_gap = beh.get("best_gap", -1)
        lead_v = beh.get("lead_speed", 0)
        desired_gap = beh.get("desired_gap", 0)
        follow_v = beh.get("follow_speed", 0)
        blocked = beh.get("blocked", False)
        worthwhile = beh.get("worthwhile", False)
        left_gap = beh.get("left_gap", -1)
        right_gap = beh.get("right_gap", -1)
        left_ok = beh.get("left_ok", False)
        right_ok = beh.get("right_ok", False)
        adj_idx = beh.get("adj_idx", -1)
        cl = beh.get("committed_lane", -1)
        tl = beh.get("target_lane", -1)
        obs_count = beh.get("obs_count", 0)

        # 清屏 + 重绘
        lines = []
        lines.append(f"{C.BOLD}╔════ FlowEngine Quick Verify ═══════════════════════════════════════════╗{C.R}")
        lines.append(f"{C.BOLD}║{C.R}  {C.BOLD}Ego{C.R}  x={x:.0f}m  y={y:.2f}m  hdg={math.degrees(heading):.0f}°  "
                     f"steer={steer:.3f}  CTE={C.RED if abs(cte)>0.5 else C.GREEN}{cte:+.2f}m{C.R}")
        lines.append(f"{C.BOLD}║{C.R}  {C.BOLD}Speed{C.R}  v={format_speed(speed, tgt_speed)} m/s  "
                     f"target={C.CYAN}{tgt_speed:.1f}{C.R} m/s  "
                     f"obs={obs_count}")
        lines.append(f"{C.BOLD}║{C.R}  {C.BOLD}Behavior{C.R}  {format_state(state)}  "
                     f"lane={C.CYAN}{cl}{C.R}→{C.YELLOW}{tl}{C.R}  "
                     f"blocked={C.RED+'Y'+C.R if blocked else C.GREEN+'N'+C.R}  "
                     f"worth={C.YELLOW+'Y'+C.R if worthwhile else C.GREEN+'N'+C.R}")
        lines.append(f"{C.BOLD}║{C.R}  {C.BOLD}Lead{C.R}  gap={format_gap(best_gap)}  "
                     f"desired={desired_gap:.0f}m  lead_v={lead_v:.1f} m/s  "
                     f"follow_v={follow_v:.1f} m/s")
        lines.append(f"{C.BOLD}║{C.R}  {C.BOLD}Lanes{C.R}  "
                     f"[L] gap={format_gap(left_gap)} ok={C.GREEN+'Y'+C.R if left_ok else C.RED+'N'+C.R}  "
                     f"[R] gap={format_gap(right_gap)} ok={C.GREEN+'Y'+C.R if right_ok else C.RED+'N'+C.R}  "
                     f"→ adj={C.CYAN}{'L' if adj_idx >= 0 and adj_idx < cl else ('R' if adj_idx > cl else '-')}{C.R}")
        lines.append(f"{C.BOLD}╠══════════════════════════════════════════════════════════════════════════╣{C.R}")
        lines.append(f"{C.GRAY}  set <param> <val>  热改参数  |  eval [s] 评估N秒  |  reset 重置  |  params 列表{C.R}")
        lines.append(f"{C.GRAY}  log 开关BEH日志    |  quit/q 退出{C.R}")
        lines.append(f"{C.BOLD}╚══════════════════════════════════════════════════════════════════════════╝{C.R}")

        sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n> ")
        sys.stdout.flush()

    def do_eval(self, seconds: float = 20.0, csv_path: str = "") -> dict:
        """采样N秒计算评估指标。含直线段品质 + 变道品质分析。"""
        print(f"\n{C.CYAN}采样 {seconds}s ...{C.R}")
        samples = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds + 5:
            m = read_metrics()
            if m:
                ego = m["ego"]
                beh = m.get("beh", {})
                samples.append({
                    "t": time.monotonic() - t0,
                    "speed": ego.get("speed", 0),
                    "x": ego.get("x", 0),
                    "y": ego.get("y", 0),
                    "heading": ego.get("heading", 0),
                    "steer": ego.get("steer", 0),
                    "state": beh.get("state", "?"),
                    "target_speed": beh.get("target_speed", 20),
                    "best_gap": beh.get("best_gap", -1),
                    "follow_speed": beh.get("follow_speed", 0),
                    "blocked": beh.get("blocked", False),
                })
            time.sleep(0.2)

        if len(samples) < 5:
            n_samples = len(samples)
            print(f"{C.RED}采样数据不足 n={n_samples}{C.R}")
            return {}

        # 道路参数
        road = load_scenario_road(self.scenario_path if hasattr(self, 'scenario_path') else None)
        lc = road["lane_count"]
        lw = road["lane_width"]

        # 时间序列 + 直线段 + 变道分析
        series = extract_series(samples)
        straight = compute_straight_metrics(series, lc, lw)
        lc_events = detect_lane_changes(series["xs"], series["ys"], series["timestamps"], lc, lw)

        avg_speed = statistics.fmean(s["speed"] for s in samples)
        ctes = [compute_cte(s["y"], lc, lw) for s in samples]
        cte_rms = math.sqrt(statistics.fmean(c*c for c in ctes))
        cte_95 = sorted(abs(c) for c in ctes)[int(len(ctes)*0.95)] if len(ctes) > 5 else 999
        gaps = [s["best_gap"] for s in samples if s["best_gap"] > 0]
        follow_states = sum(1 for s in samples if s["state"] == "FOLLOW")
        lc_states = sum(1 for s in samples if "CHANGE" in s["state"])
        cruise_states = sum(1 for s in samples if s["state"] == "CRUISE")
        blocked_count = sum(1 for s in samples if s["blocked"])

        # 综合评分（使用直线段指标更准确）
        score = 0
        score += max(0, (1.0 - (avg_speed - 15)**2 / 400)) * 35
        rms_cte = straight.get("cte_rms", cte_rms)
        score += max(0, 35 * math.exp(-rms_cte / 0.3) - 10 * cte_95)
        flip_rate = straight.get("flip_rate", series.get("flip_rate", 0))
        if flip_rate > 1.0:
            score -= (flip_rate - 1.0) * 20
        if series.get("final_x", 0) < 20:
            score -= 40

        result = {
            "score": score,
            "avg_speed": avg_speed,
            "cte_rms": cte_rms,
            "cte_95": cte_95,
            "flip_rate": flip_rate,
            "yaw_rate_rms": straight.get("yaw_rate_rms", 0),
            "steer_rate_rms": straight.get("steer_rate_rms", 0),
            "avg_gap": statistics.fmean(gaps) if gaps else -1,
            "follow_pct": 100 * follow_states / len(samples),
            "lc_pct": 100 * lc_states / len(samples),
            "cruise_pct": 100 * cruise_states / len(samples),
            "blocked_pct": 100 * blocked_count / len(samples),
            "final_x": series.get("final_x", 0),
            "n_lc": len(lc_events),
            "n": len(samples),
        }

        # 打印报告
        print(f"\n{C.BOLD}═══ Eval Report ({seconds}s, n={result['n']}) ═══{C.R}")
        print(f"  {C.BOLD}Score{C.R}: {C.GREEN if score>50 else C.RED}{score:.1f}{C.R}")
        print(f"  {C.BOLD}Speed{C.R}: avg={avg_speed:.1f} m/s  dist={series.get('final_x',0):.0f}m")
        print(f"  {C.BOLD}CTE{C.R}:   rms={C.GREEN if cte_rms<0.3 else C.RED}{cte_rms:.3f}m{C.R}  "
              f"95%={cte_95:.3f}m  "
              f"flip={C.GREEN if flip_rate<1 else C.RED}{flip_rate:.2f}Hz{C.R}")
        if straight:
            print(f"  {C.BOLD}Straight{C.R}: CTE_rms={straight['cte_rms']:.3f}m  "
                  f"yaw_rate_rms={straight['yaw_rate_rms']:.3f}rad/s  "
                  f"steer_rate_rms={straight['steer_rate_rms']:.3f}rad/s")
        if lc_events:
            print(f"  {C.BOLD}Lane Changes{C.R}: {len(lc_events)} events")
            for i, ev in enumerate(lc_events):
                dur = ev["duration"]
                overshoot = ev["overshoot"]
                print(f"    #{i} {ev['direction']:>5s}  dur={dur:.1f}s  "
                      f"d_cte={ev['cte_delta']:.2f}m  "
                      f"overshoot={C.GREEN if overshoot<0.3 else C.RED}{overshoot:.3f}m{C.R}")
        else:
            print(f"  {C.BOLD}Lane Changes{C.R}: 0 events")
        if gaps:
            print(f"  {C.BOLD}Follow{C.R}: avg_gap={statistics.fmean(gaps):.1f}m  "
                  f"follow={result['follow_pct']:.0f}%  blocked={result['blocked_pct']:.0f}%")
        print(f"  {C.BOLD}States{C.R}: CRUISE={result['cruise_pct']:.0f}%  "
              f"FOLLOW={result['follow_pct']:.0f}%  LANE_CHG={result['lc_pct']:.0f}%")

        # CSV 导出
        if csv_path:
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t","x","y","heading","speed","steer","cte"])
                for s in samples:
                    w.writerow([
                        f"{s['t']:.3f}", f"{s['x']:.3f}", f"{s['y']:.3f}",
                        f"{s['heading']:.4f}", f"{s['speed']:.3f}", f"{s['steer']:.4f}",
                        f"{compute_cte(s['y'], lc, lw):.4f}",
                    ])
            print(f"  CSV: {csv_path}")
        return result

    def cmd_loop(self):
        """交互命令循环。"""
        while self.running:
            try:
                cmd = input().strip()
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

            if not cmd:
                continue
            parts = cmd.split()
            op = parts[0].lower()

            if op in ("quit", "q", "exit"):
                self.running = False
                break
            elif op == "set" and len(parts) >= 3:
                name = parts[1]
                try:
                    val = float(parts[2])
                    ok, out = flowctl("param", "set", name, str(val))
                    if ok:
                        print(f"  {C.GREEN}✓{C.R} {name} = {val}")
                    else:
                        print(f"  {C.RED}✗{C.R} 设置失败: {out}")
                except ValueError:
                    print(f"  {C.RED}参数值需为数字{C.R}")
            elif op == "get" and len(parts) >= 2:
                name = parts[1]
                ok, out = flowctl("param", "get", name)
                print(f"  {out if ok else C.RED+'查询失败'+C.R}")
            elif op == "eval":
                secs = float(parts[1]) if len(parts) > 1 else 20.0
                self.do_eval(secs)
            elif op == "reset":
                self.reset_params()
                print(f"  {C.GREEN}✓ 已重置到baseline{C.R}")
            elif op == "baseline":
                self._save_baseline()
                print(f"  {C.GREEN}✓ 当前参数已保存为新baseline{C.R}")
            elif op == "params":
                prefix = parts[1] if len(parts) > 1 else ""
                print(f"\n{C.BOLD}可热调参数 (prefix='{prefix}'):{C.R}")
                for name, (default, desc) in sorted(TUNABLE_PARAMS.items()):
                    if prefix and not name.startswith(prefix):
                        continue
                    ok, out = flowctl("param", "get", name)
                    cur = "?"
                    if ok:
                        try:
                            cur = out.split("=")[-1].strip()
                        except IndexError:
                            pass
                    print(f"  {C.CYAN}{name}{C.R} = {cur}  {C.GRAY}({desc}){C.R}")
                print()
            elif op == "log":
                self.show_log = not self.show_log
                print(f"  BEH-DBG日志: {'ON' if self.show_log else 'OFF'}")
            elif op == "help" or op == "?":
                print(__doc__)
            else:
                print(f"  {C.GRAY}未知命令: {cmd}  (输入 help 查看帮助){C.R}")


def wait_for_flowctl(timeout: float = 45) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if SOCK_FILE.exists():
            ok, _ = flowctl("param", "get", "behavior.cruise_speed")
            if ok:
                return True
        time.sleep(0.5)
    return False


def main():
    ap = argparse.ArgumentParser(description="FlowEngine 交互式算法快速验证工具")
    ap.add_argument("--duration", type=int, default=180, help="demo运行时长(s), default=180")
    ap.add_argument("--no-browser", action="store_true", help="不打开浏览器")
    ap.add_argument("--eval-only", action="store_true", help="不启动demo,连接已有实例")
    ap.add_argument("--scenario", type=str, default=None,
                    help="场景文件路径或内置场景名(urban_challenge/straight_road/lane_change_traffic)")
    args = ap.parse_args()

    # 解析场景路径
    scenario_path = None
    if args.scenario:
        builtin = {
            "urban_challenge": "scenarios/urban_challenge.json",
            "straight_road": "scenarios/straight_road.json",
            "lane_change_traffic": "scenarios/lane_change_traffic.json",
        }
        if args.scenario in builtin:
            scenario_path = str(ROOT / builtin[args.scenario])
        else:
            scenario_path = args.scenario if os.path.isabs(args.scenario) else str(ROOT / args.scenario)

    demo_proc = None
    if not args.eval_only:
        # 杀残留
        subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond; "
                       "rm -f /tmp/flow_param.sock /tmp/flow_topology.json; sleep 1",
                       shell=True, capture_output=True, timeout=10)

        print(f"{C.CYAN}启动 demo ({args.duration}s)...{C.R}")
        demo_args = ["setsid", "bash", str(ROOT / "scripts" / "demo.sh")]
        if scenario_path:
            demo_args += ["--scenario", scenario_path]
        if args.no_browser:
            demo_args += ["--no-browser", str(args.duration)]
        else:
            demo_args.append(str(args.duration))
        demo_proc = subprocess.Popen(demo_args, cwd=ROOT,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print(f"{C.CYAN}等待仿真就绪...{C.R}")
    if not wait_for_flowctl(45):
        print(f"{C.RED}flow_launcher 未就绪，请检查demo是否正常启动{C.R}")
        if demo_proc:
            demo_proc.terminate()
        return 1

    # 默认禁用MPC（用Stanley+PID）
    flowctl("param", "set", "control.mpc_horizon", "0")
    print(f"{C.GREEN}✓ 仿真就绪！{C.R}")
    time.sleep(1.0)

    # 初始化终端
    os.system("clear")

    dashboard = Dashboard(scenario_path=scenario_path)

    # 仪表盘刷新线程
    stop_refresh = threading.Event()

    def refresh_loop():
        while not stop_refresh.is_set() and dashboard.running:
            m = read_metrics()
            if dashboard.running:
                dashboard.render(m)
            time.sleep(0.5)

    t = threading.Thread(target=refresh_loop, daemon=True)
    t.start()

    try:
        dashboard.cmd_loop()
    finally:
        stop_refresh.set()
        time.sleep(0.3)
        os.system("clear")
        if demo_proc:
            print(f"{C.CYAN}停止demo...{C.R}")
            subprocess.run("pkill -9 -f flow_launcher; pkill -9 -f flowmond",
                           shell=True, capture_output=True, timeout=5)
        print(f"{C.GREEN}已退出{C.R}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
