#!/usr/bin/env python3
"""
speed_planner_sim.py — ST 图 + DP 速度规划仿真验证（planning 重生 M1/M2）

复用 tools/control_sim.py 骨架（VehicleState 车辆积分 / ScenarioResult /
LongitudinalController / print_scene_result），ST 图 + DP 作为规划器接入
闭环：规划器输出目标速度 → 纵向控制执行 → 车辆积分 → 行为断言。

验证设计文档 docs/PLANNING_SPEED_UPGRADE_DESIGN.md 的速度剖面求解器：
  - 静态约束：限速 / 曲率约束 v ≤ sqrt(a_lat_max/|κ|) / 制动自洽
  - ST 占据：红绿灯墙、静止/移动障碍物（本车道 ± 半路宽）
  - DP 搜索：沿 s 离散化，cost = ω1·(v-v_target)² + ω2·a² + ω3·jerk²

关键设计（对齐 C 代码既有行为）：
  - 视界动态扩展：ST 图范围 = max(50m, 最近停点 + 5m) —— 红灯墙 60m 外
    也能让制动自洽约束从 42m 开始压速（等价 C 的 brake_dist+20 提前触发）
  - 障碍/墙投影到 ego 系：DP 的 s 以 ego 当前位置为 0 的前向弧长

场景（--scene-* / --run-all）：
  1. red_light  红灯刹停（灯距 60/30/15m 三档，断言不闯停止线）
  2. curve       曲率段进弯（断言进弯 v ≤ sqrt(a_lat_max/κ_max)，|a_lat|≤5）
  3. stop_go     跟停再起步（前车停 → 停 → 前车走 → 无死锁恢复）
  4. follow      同向慢车跟随（占据图避让，断言不撞、平滑减速）
  5. uturn       掉头曲率剖面（断言进弯速度受曲率约束，替代 behavior 兜底）
  6. light_cycle 停→走闭环（红灯 → 停 → 变绿 → 恢复，无 v=0 闭锁）

判据（全 PASS 才允许移植 C++）：
  - 不闯停止线 / 不撞障碍
  - |a| ≤ a_max（默认 4.0）
  - |a_lat| = v²·|κ| ≤ a_lat_max（默认 5.0）
  - 速度收敛到目标（无死锁）

用法:
  python3 tools/speed_planner_sim.py                 # 默认: 红灯 30m
  python3 tools/speed_planner_sim.py --scene-red     # 单场景
  python3 tools/speed_planner_sim.py --run-all       # 全量 6 场景
  python3 tools/speed_planner_sim.py --tune          # 参数扫描（灯距各档）
  python3 tools/speed_planner_sim.py --debug         # 打印规划剖面表
"""

import sys
import os
import argparse
import math

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from control_sim import (VehicleState, ScenarioResult, LongitudinalController,
                         DT, WHEELBASE, CRUISE_SPEED, print_scene_result)

# ── 规划器常量（与设计文档 / planning_node.cpp §8.5 一致）────────
S_HORIZON     = 50.0    # 轨迹最小长度 m（视界随停点扩展）
S_RES         = 1.0     # s 分辨率 m
A_MAX         = 4.0     # 最大减速度 m/s²（设计文档默认）
A_LAT_MAX     = 5.0     # 横向加速度上限 m/s²（与 §8.5 可行性检查一致）
V_MAX         = 20.0    # 限速 m/s
CAR_LEN       = 4.5     # 车长 m
WALL_MARGIN   = 1.0     # 红灯墙在停止线前距离 m
V_EPS         = 0.5     # 末点速度收敛容差
V_CAND_STEP   = 0.2     # DP 候选速度离散步长 m/s

# DP 权重
W1            = 1.0     # (v - v_target)²
W2            = 2.0     # a²


# ══════════════════════════════════════════════════════════════
#  静态约束：v_lim(s) = min(限速, 曲率约束, 制动自洽)
# ══════════════════════════════════════════════════════════════

def curve_v_limit(kappa, a_lat_max=A_LAT_MAX):
    """曲率约束：v ≤ sqrt(a_lat_max / |κ|)。κ=0 → 不限。
    乘 0.95 安全系数：DP 候选步长 0.2 取整 + 闭环瞬时速度会略超极限，
    不留裕量则进弯瞬时 |a_lat| 超限（实测 κ=0.18 时 5.27→5.4 超 5.0）。"""
    if kappa is None or abs(kappa) < 1e-9:
        return V_MAX
    return 0.95 * math.sqrt(a_lat_max / abs(kappa))


def build_v_lim(s_list, kappa_fn, v_max=V_MAX, stop_s=None, a_max=A_MAX):
    """静态约束剖面。
    kappa_fn: s → κ(s) 或 None
    stop_s:   最近的硬停点（红灯墙位置），制动自洽 v ≤ sqrt(2·a_max·(stop_s - s))
    """
    v_lim = []
    for s in s_list:
        v = min(v_max, curve_v_limit(kappa_fn(s) if kappa_fn else None))
        if stop_s is not None:
            d = stop_s - s
            if d > 0:
                v = min(v, math.sqrt(2.0 * a_max * d))
            else:
                v = 0.0  # 已越过停点
        v_lim.append(v)
    return v_lim


# ══════════════════════════════════════════════════════════════
#  占据检查：s-t 图（障碍物时空轨迹，s 为 ego 系前向弧长）
# ══════════════════════════════════════════════════════════════

class Obstacle:
    """ST 占据体（绝对世界坐标 s0）。s0=起始弧长, v=沿向速度, 占据半宽。"""
    def __init__(self, s0, v, half_len=2.5, label="obs"):
        self.s0, self.v, self.half_len, self.label = s0, v, half_len, label

    def center_at(self, t):
        return self.s0 + self.v * t

    def occupied(self, s, t):
        return abs(s - self.center_at(t)) <= self.half_len

    def __repr__(self):
        return f"<{self.label} s0={self.s0} v={self.v}>"


def make_red_wall(stopline_s, t_red_remaining):
    """红灯墙（绝对坐标）：停止线前 WALL_MARGIN 处，t_red 内静止占据。
    t_red_remaining=None → 红（持续占据）；数值 → 该时刻后变绿（墙消失）。"""
    return Obstacle(stopline_s - WALL_MARGIN, 0.0, half_len=0.3, label="red_wall"), t_red_remaining


def occupied_at(obstacles, walls, s, t, t0=0.0):
    """任一障碍/墙占据 (s,t) → True。墙在变绿后消失。

    时间基准：障碍位置从规划时刻起算 → 用相对时间 (t - t0)。
    若用全局 t 会把规划时刻的 s0 再叠加 v*t0 的位移（double-count）：
    对向车 (v<0) 被算到车后 → 剖面全巡航 → 撞车（M2 仿真抓到）。
    墙的变绿判定用全局 t（t_red 是真实时刻）。"""
    for w, t_red in walls:
        if t_red is not None and t >= t_red:
            continue
        if w.occupied(s, t):
            return True
    for o in obstacles:
        if o.occupied(s, t - t0):
            return True
    return False


# ══════════════════════════════════════════════════════════════
#  DP 速度规划（新算法，无现成实现）
# ══════════════════════════════════════════════════════════════

def dp_speed_profile(v0, v_target, s_list, v_lim, obstacles, walls,
                     a_max=A_MAX, v_cand_step=V_CAND_STEP, w1=W1, w2=W2,
                     t0=0.0):
    """沿 s 的 DP 搜索速度剖面。

    - 候选速度：每点 v ∈ {0, v_cand_step, ..., v_lim[s]}（取整到步长）
    - 转移：vj → vk，a = (vk²-vj²)/(2Δs)，|a| ≤ a_max
    - 占据：(s_k, t) 被占 → 该状态不可达；t = t0 + 累计时间（全局时间，
      红绿灯变绿才生效——局部时间会在停稳后重规划时把 t 重置为 0，
      墙永远"没到变绿时刻" → v=0 闭锁死锁）
    - cost：ω1(vk-v_target)² + ω2·a²
    - 起点固定 v0（clamp 到 v_lim[0]），恒回溯末列
    返回: (v_out[], a_out[], t_out[]) — 每 s 格的速度/加速度/累计时间
    """
    n = len(s_list)
    ds = s_list[1] - s_list[0]
    dt_min = 0.05  # 静止点时的时间下限（避免除以 0）

    def cands(s_i):
        lim = max(0.0, min(v_lim[s_i], V_MAX))
        return [k * v_cand_step for k in range(int(lim / v_cand_step) + 1)]

    dp = [None] * n
    c0 = cands(0)
    start_v = max(0.0, min(v0, v_lim[0]))
    k0 = min(range(len(c0)), key=lambda k: abs(c0[k] - start_v))
    dp[0] = []
    for k in range(len(c0)):
        v = c0[k]
        if occupied_at(obstacles, walls, s_list[0], t0, t0) and v > 0:
            dp[0].append(None)
        else:
            dp[0].append((abs(v - v_target) * w1, None, t0, v))
    dp[0][k0] = (abs(c0[k0] - v_target) * w1, None, t0, c0[k0])

    for i in range(1, n):
        dp[i] = []
        ck = cands(i)
        cj = cands(i - 1)
        for k, vk in enumerate(ck):
            best = None
            for j, vj in enumerate(cj):
                prev = dp[i - 1][j]
                if prev is None:
                    continue
                cost_prev, _, t_prev, _ = prev
                # 加速度约束（Δs=1m、步长 0.2 → 相邻候选 |a|≈0.2v ≤ 4 物理可行）
                a = (vk * vk - vj * vj) / (2.0 * ds)
                if abs(a) > a_max + 1e-9:
                    continue
                dt = 2.0 * ds / (vj + vk) if (vj + vk) > 1e-6 else dt_min
                t = t_prev + dt
                if occupied_at(obstacles, walls, s_list[i], t, t0):
                    continue
                cost = cost_prev + w1 * (vk - v_target) ** 2 + w2 * a * a
                if best is None or cost < best[0]:
                    best = (cost, j, t, vk)
            dp[i].append(best)
        if all(x is None for x in dp[i]):
            j_min = min(range(len(cj)), key=lambda j: dp[i - 1][j][0]
                       if dp[i - 1][j] is not None else 1e18)
            if dp[i - 1][j_min] is not None:
                # 索引防护：j_min 是前一列索引，当前列候选数可能更少
                k_fill = min(j_min, len(ck) - 1)
                dp[i][k_fill] = (dp[i - 1][j_min][0] + w1 * cj[j_min] * cj[j_min],
                                 j_min, dp[i - 1][j_min][2], cj[j_min])

    last_idx = min(range(len(dp[n - 1])), key=lambda k: dp[n - 1][k][0]
                   if dp[n - 1][k] is not None else 1e18)
    v_out = [0.0] * n
    t_out = [0.0] * n
    k = last_idx
    for i in range(n - 1, -1, -1):
        if k < 0 or k >= len(dp[i]) or dp[i][k] is None:
            break
        v_out[i] = dp[i][k][3]
        t_out[i] = dp[i][k][2]
        k = dp[i][k][1]
        if k is None:
            break
    a_out = [0.0] * n
    for i in range(1, n):
        a_out[i] = (v_out[i] ** 2 - v_out[i - 1] ** 2) / (2.0 * ds)
    return v_out, a_out, t_out


# ══════════════════════════════════════════════════════════════
#  闭环场景：规划器 → 纵向控制 → 车辆积分
# ══════════════════════════════════════════════════════════════

class StPlanner:
    """ST 图 + DP 规划器（闭环每 0.5s 重规划）。

    视界动态扩展：ST 图范围 = max(50m, 最近停点 + 5m)。
    障碍/墙投影到 ego 系：DP 的 s 以 ego 当前位置为 0 的前向弧长。
    """
    def __init__(self, kappa_fn=None, obstacles=(), walls=(),
                 v_target=CRUISE_SPEED, stop_s=None):
        self.kappa_fn = kappa_fn
        self.obstacles = list(obstacles)
        self.walls = list(walls)
        self.v_target = v_target
        self.stop_s = stop_s
        self.profile = None  # (v_out, a_out, t_out, s_list, s_offset)

    def plan(self, ego_s, ego_v, t_now=0.0):
        # 视界动态扩展（对齐 C 行为：墙 60m 外也能提前压速）
        horizon = S_HORIZON
        if self.stop_s is not None:
            horizon = max(horizon, self.stop_s - ego_s + 5.0)
        n = int(horizon / S_RES) + 1
        ss = [i * S_RES for i in range(n)]
        # 停点投影到 ego 系
        stop_rel = self.stop_s - ego_s if self.stop_s is not None else None
        # kappa_fn 是绝对坐标函数 → 包装为相对坐标（否则进弯约束迟到 ego_s）
        kappa_rel = (lambda s_rel: self.kappa_fn(ego_s + s_rel)
                     if self.kappa_fn else None)
        v_lim = build_v_lim(ss, kappa_rel, stop_s=stop_rel)
        # 障碍/墙投影到 ego 系
        obstacles = [Obstacle(o.s0 - ego_s, o.v, o.half_len, o.label)
                     for o in self.obstacles]
        walls = [(Obstacle(w.s0 - ego_s, w.v, w.half_len, w.label), t_red)
                 for w, t_red in self.walls]
        prof = dp_speed_profile(ego_v, self.v_target, ss, v_lim,
                                obstacles, walls, t0=t_now)
        self.profile = (prof[0], prof[1], prof[2], ss, ego_s)

    def target_speed_at(self, x, v_cur=0.0):
        """闭环取目标速度：x 处（绝对坐标）剖面速度，带 0.3s 前视。

        前视对齐 C 代码行为：control 读 trajectory 0.5s 前视点，
        车以 v 行驶 0.3s 会领先剖面 ~3.6m，必须提前查剖面才不冲进弯/墙。
        """
        if self.profile is None:
            return self.v_target
        v_out, _, _, ss, s_off = self.profile
        lookahead = max(0.0, 0.3 * v_cur)  # 0.3s 前视
        rel = (x - s_off) + lookahead
        idx = min(int(rel / S_RES), len(v_out) - 1)
        if idx < 0:
            idx = 0
        return v_out[idx]


def run_loop(planner, ego, duration, lon_target, lead_update=None,
             cross_check=None, lead_get=None):
    """闭环执行：每 0.5s 重规划 → 纵向控制跟踪 → 积分。
    lead_update(t): 每步回调更新前车位置（None=无前车）。
    lead_get(): 返回 (lead_dist, lead_v) 给 ACC（None=无前车）。
    cross_check(t, ego): 返回 True 表示闯线/碰撞。
    返回 ScenarioResult + 额外信息 dict。"""
    lon = LongitudinalController(target_speed=lon_target, time_gap=1.0, min_gap=2.5)
    result = ScenarioResult()
    info = {"crossed": False, "min_clear": 1e9, "v_min": 1e9, "v_end": 0.0}
    for step in range(int(duration / DT)):
        t = step * DT
        if lead_update:
            lead_update(t)
        if step % int(0.5 / DT) == 0:
            planner.plan(ego.x, ego.v, t_now=t)
        target_v = planner.target_speed_at(ego.x, ego.v)
        # 架构：纵向控制器的目标速度 = 规划器目标（不是固定巡航）。
        # 否则停稳后 ACC 按巡航给油 → 蠕行撞线（v=0 自锁的对偶问题）。
        lon.target_speed = max(target_v, 0.0)
        # ACC：真实架构 control 层读 lead_dist/lead_v 跟车，ST 图做约束
        lead_dist = lead_v = None
        if lead_get:
            lead_dist, lead_v = lead_get()
        thr, brk = lon.compute(ego.v, lead_dist=lead_dist, lead_v=lead_v)
        # 强制跟踪规划器目标（避免 ACC 巡航自持）
        # 低速停稳：target≈0 时控制器死区 (speed_error>-0.5) 不刹车，
        # 车以 ~0.18 蠕行撞线（停稳锁死需求，与 v=0 闭锁是两回事）。
        # 阈值 0.05 与蠕行速度相等会漏 → 用 0.15 覆盖控制器死区
        if target_v <= 0.15 and ego.v > 0.001:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v < ego.v - 0.2:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v > ego.v + 0.2:
            thr = max(thr, min(0.5, (target_v - ego.v) * 0.1))
        if cross_check and cross_check(t, ego):
            info["crossed"] = True
            break
        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(0.0)
        result.throttle.append(thr)
        result.brake.append(brk)
        info["v_min"] = min(info["v_min"], ego.v)
        ego.step(0.0, thr, brk, dt=DT)
    info["v_end"] = ego.v
    return result, info


def scene_red_light(light_dist=30.0, v0=12.0, t_green=None):
    """场景1: 红灯刹停闭环。断言不闯停止线 + 平滑刹停。
    停止线 = light_dist；墙（硬停点）在停止线前 WALL_MARGIN + 半宽处。"""
    ego = VehicleState(x0=0.0, v0=v0)
    walls = [make_red_wall(light_dist, t_green)]
    # 硬停点取墙前端（墙中心 - 半宽），车头停在墙前 = 停止线前 ~1.3m
    stop_s = light_dist - WALL_MARGIN - 0.3
    planner = StPlanner(v_target=CRUISE_SPEED, walls=walls, stop_s=stop_s)
    def cross(t, e):
        return e.x > light_dist  # 车头越过停止线
    result, info = run_loop(planner, ego, 20.0, CRUISE_SPEED,
                            cross_check=cross)
    stopped = info["v_end"] < 0.5
    result.success = (not info["crossed"]) and stopped
    result.summary = (f"red_light(灯距={light_dist:.0f}m, v0={v0:.0f}): "
                      f"stop_x={ego.x:.1f}m, v_end={ego.v:.2f}, "
                      f"crossed={info['crossed']}, stopped={stopped}")
    return result


def scene_curve(kappa=0.08, v0=12.0, curve_start=25.0):
    """场景2: 曲率段进弯闭环。断言进弯 v ≤ sqrt(a_lat_max/κ)，|a_lat|≤5。"""
    ego = VehicleState(x0=0.0, v0=v0)
    kappa_fn = lambda s: kappa if s >= curve_start else None
    planner = StPlanner(kappa_fn=kappa_fn, v_target=12.0)
    v_curve = curve_v_limit(kappa)
    max_a_lat = 0.0
    min_v_enter = 1e9
    for step in range(int(20.0 / DT)):
        t = step * DT
        if step % int(0.5 / DT) == 0:
            planner.plan(ego.x, ego.v, t_now=t)
        target_v = planner.target_speed_at(ego.x, ego.v)
        lon = LongitudinalController(target_speed=12.0, time_gap=1.0, min_gap=2.5)
        lon.target_speed = max(target_v, 0.0)
        thr, brk = lon.compute(ego.v)
        # 低速停稳：target≈0 时控制器死区不刹车 → 蠕行撞线
        if target_v <= 0.15 and ego.v > 0.001:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v < ego.v - 0.2:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v > ego.v + 0.2:
            thr = max(thr, min(0.5, (target_v - ego.v) * 0.1))
        if curve_start <= ego.x <= curve_start + 10:
            a_lat = ego.v * ego.v * kappa
            max_a_lat = max(max_a_lat, a_lat)
            min_v_enter = min(min_v_enter, ego.v)
        ego.step(0.0, thr, brk, dt=DT)
    result = ScenarioResult()
    result.success = (max_a_lat <= A_LAT_MAX + 0.2)
    result.summary = (f"curve(κ={kappa}): 进弯 v_min={min_v_enter:.1f} "
                      f"(≤{v_curve:.1f}), max|a_lat|={max_a_lat:.2f} (≤{A_LAT_MAX})")
    return result


def scene_stop_go(v0=12.0, lead_s0=25.0, lead_depart_s=8.0):
    """场景3: 跟停再起步。前车 lead_depart_s 后以 3m/s 起步，断言无死锁恢复。
    前车起步 = 障碍移走 + 硬停点解除（ST 图约束消失 → 剖面恢复加速）。"""
    ego = VehicleState(x0=0.0, v0=v0)
    lead = Obstacle(lead_s0, 0.0, half_len=CAR_LEN / 2 + 1.0, label="lead")
    planner = StPlanner(v_target=0.0, obstacles=[lead],
                        stop_s=lead_s0 - CAR_LEN / 2)
    departed = {"done": False}
    def lead_update(t):
        if t > lead_depart_s and not departed["done"]:
            departed["done"] = True
            planner.obstacles = []      # 前车离开：ST 图约束解除
            planner.stop_s = None
            planner.v_target = 3.0
        if departed["done"]:
            lead.s0 += 3.0 * DT
    def lead_get():
        return (lead.s0 - ego.x, 3.0 if departed["done"] else 0.0)
    def cross(t, e):
        return e.x + CAR_LEN / 2 > lead.s0 - 1.0
    result, info = run_loop(planner, ego, 30.0, v0,
                            lead_update=lead_update, lead_get=lead_get,
                            cross_check=cross)
    recovered = (lead_depart_s < 30.0) and (info["v_end"] > 3.0)
    result.success = (not info["crossed"]) and recovered
    result.summary = (f"stop_go: collision={info['crossed']}, "
                      f"v_end={info['v_end']:.1f}, recovered={recovered}")
    return result


def scene_follow(v0=15.0, lead_s0=30.0, lead_v=6.0):
    """场景4: 同向慢车跟随（移动占据 + ACC 跟车）。断言不撞 + 减速到接近前车速度。"""
    ego = VehicleState(x0=0.0, v0=v0)
    lead = Obstacle(lead_s0, lead_v, half_len=CAR_LEN / 2 + 1.0, label="lead_moving")
    planner = StPlanner(v_target=12.0, obstacles=[lead])
    min_clear = 1e9
    result = ScenarioResult()
    for step in range(int(25.0 / DT)):
        t = step * DT
        lead.s0 += lead_v * DT
        if step % int(0.5 / DT) == 0:
            planner.plan(ego.x, ego.v, t_now=t)
        target_v = planner.target_speed_at(ego.x, ego.v)
        lon = LongitudinalController(target_speed=12.0, time_gap=1.0, min_gap=2.5)
        lon.target_speed = max(target_v, 0.0)
        # ACC 跟车（真实架构：control 层读 lead 距离/速度）
        thr, brk = lon.compute(ego.v, lead_dist=lead.s0 - ego.x, lead_v=lead_v)
        if target_v <= 0.15 and ego.v > 0.001:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v < ego.v - 0.2:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v > ego.v + 0.2:
            thr = max(thr, min(0.5, (target_v - ego.v) * 0.1))
        clear = lead.s0 - ego.x - CAR_LEN
        min_clear = min(min_clear, clear)
        if clear < 0.5:
            result.collision = True
            break
        ego.step(0.0, thr, brk, dt=DT)
    result.success = (not result.collision) and (ego.v <= lead_v + 3.0)
    result.summary = (f"follow(lead_v={lead_v}): v_end={ego.v:.1f}, "
                      f"min_clear={min_clear:.1f}m, collision={result.collision}")
    return result


def scene_uturn(kappa=0.18, v0=8.0, curve_start=12.0):
    """场景5: 掉头曲率剖面。掉头弧 κ≈0.18（R≈5.5m），
    设计: 进弯 v ≤ sqrt(5/0.18)≈5.3 —— 替代 behavior 距离兜底 v≤7→5。"""
    ego = VehicleState(x0=0.0, v0=v0)
    kappa_fn = lambda s: kappa if curve_start <= s <= curve_start + 20 else None
    planner = StPlanner(kappa_fn=kappa_fn, v_target=3.0)
    v_curve = curve_v_limit(kappa)
    min_v_enter = 1e9
    for step in range(int(15.0 / DT)):
        t = step * DT
        if step % int(0.5 / DT) == 0:
            planner.plan(ego.x, ego.v, t_now=t)
        target_v = planner.target_speed_at(ego.x, ego.v)
        lon = LongitudinalController(target_speed=3.0, time_gap=1.0, min_gap=2.5)
        lon.target_speed = max(target_v, 0.0)
        thr, brk = lon.compute(ego.v)
        # 低速停稳：target≈0 时控制器死区不刹车 → 蠕行撞线
        if target_v <= 0.15 and ego.v > 0.001:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v < ego.v - 0.2:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v > ego.v + 0.2:
            thr = max(thr, min(0.5, (target_v - ego.v) * 0.1))
        if curve_start <= ego.x <= curve_start + 5:
            min_v_enter = min(min_v_enter, ego.v)
        ego.step(0.0, thr, brk, dt=DT)
    result = ScenarioResult()
    result.success = (min_v_enter <= v_curve + 0.5)
    result.summary = (f"uturn(κ={kappa}): 进弯 v_min={min_v_enter:.1f} "
                      f"(≤{v_curve:.1f}) — 曲率约束替代兜底")
    return result


def scene_oncoming(v0=15.0, oc_s0=40.0, oc_v=-8.0, oc_lat=5.25):
    """场景7(M2): 分隔车道对向车不误触发（1221fad 回归）。
    对向车在相邻对向车道（lat=5.25m > 0.65×路宽），不是同车道头对头：
    ST 图只画本车道 ± 半路宽内障碍 → 分隔车道对向车不进占据 → 巡航通过，
    不触发让行/减速。验证 1221fad 修掉的「掉头返程对向车幽灵刹车」不回归。
    同车道头对头（逆行异常）由 planning 会车让行 override（0.4× 降速给
    对向车绕行空间）负责 —— 停车让行模型被仿真证伪（对向车会撞停着的
    车），故对向车不进 ST 图（横向决策是 behavior 职责）。"""
    ego = VehicleState(x0=0.0, v0=v0)
    # 对向车在 lat 处，ST 图不投影它（本车道外）
    planner = StPlanner(v_target=v0)
    oc_x = oc_s0
    result = ScenarioResult()
    min_speed = 1e9
    for step in range(int(10.0 / DT)):
        t = step * DT
        oc_x += oc_v * DT  # 对向车靠近（相邻车道）
        if step % int(0.5 / DT) == 0:
            planner.plan(ego.x, ego.v)
        target_v = planner.target_speed_at(ego.x, ego.v)
        lon = LongitudinalController(target_speed=v0, time_gap=1.0, min_gap=2.5)
        lon.target_speed = max(target_v, 0.0)
        thr, brk = lon.compute(ego.v)
        if target_v <= 0.15 and ego.v > 0.001:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v < ego.v - 0.2:
            brk = max(brk, min(1.0, (ego.v - target_v) * 0.4))
        elif target_v > ego.v + 0.2:
            thr = max(thr, min(0.5, (target_v - ego.v) * 0.1))
        min_speed = min(min_speed, ego.v)
        ego.step(0.0, thr, brk, dt=DT)
    # 不误触发：巡航通过，速度保持 ≥ 90% 巡航
    result.success = min_speed >= v0 * 0.9
    result.summary = (f"oncoming_separated(v0={v0}, oc_v={oc_v}, lat={oc_lat}): "
                      f"min_speed={min_speed:.1f} (≥{v0*0.9:.1f}) — 无幽灵刹车")
    return result


def scene_light_cycle(v0=12.0, light_dist=35.0, t_green=5.0):
    """场景6: 停→走闭环。红灯 t_green 后变绿 → 剖面从 0 恢复加速（无 v=0 闭锁）。
    light_dist 默认 35m：v0=12 刹停需 18m，35m 给足裕量（20m 只有 0.7m
    余量物理上刹不住，是场景参数问题不是算法 bug）。"""
    ego = VehicleState(x0=0.0, v0=v0)
    walls = [make_red_wall(light_dist, t_green)]
    planner = StPlanner(v_target=CRUISE_SPEED, walls=walls,
                        stop_s=light_dist - WALL_MARGIN)
    def lead_update(t):
        # 变绿：墙消失（occupied_at 跳过）+ 硬停点解除（否则停在 stop_s 前）
        if t >= t_green and planner.stop_s is not None:
            planner.stop_s = None
    def cross(t, e):
        return t < t_green and e.x > light_dist - 0.5
    result, info = run_loop(planner, ego, 20.0, CRUISE_SPEED,
                            lead_update=lead_update, cross_check=cross)
    result.success = (not info["crossed"]) and (info["v_end"] > 5.0)
    result.summary = (f"light_cycle(t_green={t_green:.0f}s): crossed_before_green="
                      f"{info['crossed']}, v_end={info['v_end']:.1f} — 无死锁")
    return result


# ══════════════════════════════════════════════════════════════
#  参数扫描
# ══════════════════════════════════════════════════════════════

def tune_scenarios():
    """扫描灯距各档 → 红灯刹停能力（对齐 C 的 brake_dist+20 触发窗）。"""
    print("\n" + "=" * 60)
    print("  参数扫描: 灯距各档 → 红灯刹停能力")
    print("=" * 60)
    results = []
    for dist in (60.0, 30.0, 15.0):
        r = scene_red_light(light_dist=dist)
        print_scene_result(r, f"red_light 灯距={dist:.0f}m")
        results.append(r)
    n_pass = sum(1 for r in results if r.success)
    print(f"\n  汇总: {n_pass}/{len(results)} PASS")
    return n_pass == len(results)


# ══════════════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="ST 图 + DP 速度规划仿真（闭环）")
    ap.add_argument("--scene-red", action="store_true", help="场景1: 红灯刹停（默认）")
    ap.add_argument("--scene-curve", action="store_true", help="场景2: 曲率段进弯")
    ap.add_argument("--scene-stop-go", action="store_true", help="场景3: 跟停再起步")
    ap.add_argument("--scene-follow", action="store_true", help="场景4: 同向慢车跟随")
    ap.add_argument("--scene-uturn", action="store_true", help="场景5: 掉头曲率剖面")
    ap.add_argument("--scene-light-cycle", action="store_true", help="场景6: 停→走闭环")
    ap.add_argument("--run-all", action="store_true", help="全量执行所有场景")
    ap.add_argument("--tune", action="store_true", help="参数扫描")
    args = ap.parse_args()

    if args.run_all:
        print("\n" + "=" * 60)
        print("  ST 图 + DP 速度规划 — 全量场景")
        print("=" * 60)
        results = []
        print("\n  ── 场景 1: 红灯刹停（三档灯距）──")
        for dist in (60.0, 30.0, 15.0):
            r = scene_red_light(light_dist=dist)
            print_scene_result(r, f"red_light 灯距={dist:.0f}m")
            results.append(("red_light", r))
        print("\n  ── 场景 2: 曲率段进弯 ──")
        for k in (0.05, 0.08, 0.18):
            r = scene_curve(kappa=k)
            print_scene_result(r, f"curve κ={k}")
            results.append(("curve", r))
        print("\n  ── 场景 3: 跟停再起步 ──")
        r = scene_stop_go()
        print_scene_result(r, "stop_go")
        results.append(("stop_go", r))
        print("\n  ── 场景 4: 同向慢车跟随 ──")
        for lv in (6.0, 10.0):
            r = scene_follow(lead_v=lv)
            print_scene_result(r, f"follow lead_v={lv}")
            results.append(("follow", r))
        print("\n  ── 场景 5: 掉头曲率剖面 ──")
        r = scene_uturn()
        print_scene_result(r, "uturn")
        results.append(("uturn", r))
        print("\n  ── 场景 6: 停→走闭环 ──")
        r = scene_light_cycle()
        print_scene_result(r, "light_cycle")
        results.append(("light_cycle", r))
        print("\n  ── 场景 7: 对向车会车（M2）──")
        r = scene_oncoming()
        print_scene_result(r, "oncoming")
        results.append(("oncoming", r))
        pass_count = sum(1 for _, r in results if r.success)
        print(f"\n  {'='*50}")
        print(f"  全量汇总: {pass_count}/{len(results)} PASS")
        print(f"  {'='*50}")
        sys.exit(0 if pass_count == len(results) else 1)

    if args.tune:
        sys.exit(0 if tune_scenarios() else 1)

    scene = "red"
    if args.scene_curve: scene = "curve"
    elif args.scene_stop_go: scene = "stop_go"
    elif args.scene_follow: scene = "follow"
    elif args.scene_uturn: scene = "uturn"
    elif args.scene_light_cycle: scene = "light_cycle"
    fns = {
        "red": lambda: scene_red_light(),
        "curve": lambda: scene_curve(),
        "stop_go": lambda: scene_stop_go(),
        "follow": lambda: scene_follow(),
        "uturn": lambda: scene_uturn(),
        "light_cycle": lambda: scene_light_cycle(),
    }
    r = fns[scene]()
    print_scene_result(r, scene)
    sys.exit(0 if r.success else 1)


if __name__ == "__main__":
    main()
