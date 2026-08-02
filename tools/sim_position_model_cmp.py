#!/usr/bin/env python3
"""对比两种 ego 位置模型（py-sim-first：验证"斜着直行"根因）。

  Model B — 物理真值/沿车头（step_bicycle + world_to_frenet，u_turn_active 返程在用）:
      x += v·cos(h)·dt,  y += v·sin(h)·dt
  Model A — 贴路推进/当前 C++ 正常模式（road_pos.advance + set_offset(sin(dh)))：
      沿道路推进满 v·dt，横向 sin(h)·v·dt（直路沿 +x 时）

  结论指标：
    1) crab_angle = |车头 heading − 实际运动方向|（>0 即"斜着直行"）
    2) 掉头/变道轨迹上两种模型的位置发散
"""

import math

WHEELBASE = 2.7
DT = 0.05


def step_velocity(v, steer, throttle, brake, dt):
    """纵向加速（与 flowsim physics.cpp 一致的简化映射）。"""
    if throttle < 0:
        v += throttle * 3.33 * dt
        if v < -4.0: v = -4.0
    elif brake > 0:
        v -= brake * 8.0 * dt
        if v < 0: v = 0.0
    elif throttle > 0:
        v += throttle * 3.33 * dt
        if v > 22.0: v = 22.0
    return v


def norm(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def step(px, py, h, v, steer, dt, model):
    """一步位置+航向积分。
       model: 'A'(贴路·现状) / 'B'(沿车头·物理真值) / 'C'(cos 投影·修复方案)。"""
    h = norm(h + (v / WHEELBASE) * math.tan(steer) * dt)
    if model == 'B':                      # 物理真值：沿车头
        nx = px + v * math.cos(h) * dt
        ny = py + v * math.sin(h) * dt
    elif model == 'C':                    # 修复：advance(v·dt·cos h) + lateral(v·dt·sin h)
        nx = px + v * math.cos(h) * dt    # 沿道路推进 v·dt·cos(h)（不再满 v·dt）
        ny = py + v * dt * math.sin(h)    # 横向 sin(h)·v·dt（不变）
    else:                                 # A：贴路推进（直路沿 +x）
        nx = px + v * dt                  # 沿道路推进满 v·dt
        ny = py + v * dt * math.sin(h)    # 横向 sin(h)·v·dt
    return nx, ny, h


def heading_offset_table():
    """恒定车头偏角 h 下，两种模型的实际运动方向 + 斜走角。"""
    print("=" * 92)
    print("① 恒定车头偏角 h（不转向，直线开）：运动方向 vs 车头")
    print("=" * 92)
    print(f"{'车头 h(°)':>10} | {'B沿车头 方向(°)':>14} {'斜走角(°)':>9} | "
          f"{'A贴路 方向(°)':>14} {'斜走角(°)':>9} | {'C修复 方向(°)':>14} {'斜走角(°)':>9}")
    print("-" * 92)
    for deg in [0, 5, 10, 15, 20, 30, 45, 60, 75, 80, 89]:
        h = math.radians(deg)
        v = 3.5
        mb_dir = math.degrees(h)                                  # B: 运动方向 = h
        mb_crab = 0.0
        ma_dir = math.degrees(math.atan2(v * math.sin(h), v))     # A: atan(sin h)
        ma_crab = abs(deg - ma_dir)
        mc_dir = math.degrees(math.atan2(v * math.sin(h), v * math.cos(h)))  # C
        mc_crab = abs(deg - mc_dir)
        print(f"{deg:>10} | {mb_dir:>14.2f} {mb_crab:>9.2f} | "
              f"{ma_dir:>14.2f} {ma_crab:>9.2f} | {mc_dir:>14.2f} {mc_crab:>9.2f}")
    print()


def run_uturn(steer_profile, label, n_steps, v0=20.0):
    """跑一段机动（掉头/变道），对比两模型轨迹 + 全程斜走角。"""
    print("=" * 108)
    print(f"② {label}：A(现状贴路) vs C(cos 投影修复) vs B(沿车头真值)")
    print("=" * 108)
    traj = {}
    for model in ('A', 'B', 'C'):
        x = y = h = 0.0
        v = v0
        pts = []
        for i in range(n_steps):
            steer = steer_profile(i)
            v = step_velocity(v, steer, 0.0, 0.0, DT)
            x, y, h = step(x, y, h, v, steer, DT, model)
            pts.append((x, y, h, v))
        traj[model] = pts
    stride = max(1, n_steps // 12)
    print(f"{'t(s)':>5} | {'A_x':>8} {'A_y':>8} | {'C_x':>8} {'C_y':>8} | "
          f"{'B_x':>8} {'B_y':>8} | {'|C−B|':>8} | {'C运动(°)':>8} {'C车头(°)':>8}")
    print("-" * 108)
    for i in range(0, n_steps, stride):
        ax, ay, ah, _ = traj['A'][i]
        cx, cy, ch, _ = traj['C'][i]
        bx, by, bh, _ = traj['B'][i]
        if i + 1 < n_steps:
            ncx, ncy = traj['C'][i + 1][0], traj['C'][i + 1][1]
            c_dir = math.degrees(math.atan2(ncy - cy, ncx - cx))
        else:
            c_dir = math.degrees(ch)
        dcb = math.hypot(cx - bx, cy - by)
        print(f"{i * DT:>5.2f} | {ax:>8.2f} {ay:>8.2f} | {cx:>8.2f} {cy:>8.2f} | "
              f"{bx:>8.2f} {by:>8.2f} | {dcb:>8.3f} | {c_dir:>8.1f} {math.degrees(ch):>8.1f}")
    print()


def lane_change_profile(i):
    """变道机动：先右打 0.10，再回 0，模拟 Stanley 变道。"""
    t = i * DT
    if t < 2.0:
        return 0.0
    elif t < 4.0:
        return 0.12
    elif t < 5.0:
        return -0.08
    else:
        return 0.0


def uturn_forward_profile(i):
    """掉头 phase1：满舵左打 0.55。"""
    return 0.55 if i * DT < 3.0 else 0.0


if __name__ == '__main__':
    heading_offset_table()
    run_uturn(uturn_forward_profile, "掉头前进一把（steer=0.55, 3s）", n_steps=120, v0=3.5)
    run_uturn(lane_change_profile, "变道机动（steer 0.12 → 回正）", n_steps=150, v0=20.0)
