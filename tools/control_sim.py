#!/usr/bin/env python3
"""
control_sim.py — 车辆横向控制仿真验证工具

纯 Python 标准库实现，无需 numpy/scipy。100% 对齐 C 代码逻辑，包含：
  - 运动学自行车模型（同 flowsim kinematic 模式）
  - 直道/变道场景
  - 改进版 Stanley 控制器（C代码当前默认，带v_y_des前馈）
  - LTV-MPC 控制器（可选，完全移植ltv_mpc.c的Riccati求解器）
  - CSV 输出 + 终端统计 + 参数扫描

符号约定（与C代码严格一致）：
  - lat_error = target_y - ego_y  (Stanley约定，车在目标右侧为正)
  - LTV-MPC e_y = -lat_error = ego_y - target_y
  - LTV-MPC e_psi = ref_heading - ego_heading

用法:
  python3 tools/control_sim.py                          # 默认: Stanley, 直道
  python3 tools/control_sim.py --lc                      # 变道场景
  python3 tools/control_sim.py --mpc                     # 启用LTV-MPC
  python3 tools/control_sim.py --tune                    # Stanley参数扫描
  python3 tools/control_sim.py --tune-mpc                # LTV-MPC参数扫描
"""

import math
import csv
import sys
import os
import argparse

# ── 常量（与C代码严格一致）──────────────────────────────────
DT               = 0.05    # 控制周期 20Hz
WHEELBASE        = 2.7     # 轴距 m
CRUISE_SPEED     = 12.0    # 默认巡航速度 m/s
SIM_DURATION     = 20.0    # 仿真时长 s
STEER_FILTER_ALPHA = 0.5   # 低通滤波新值权重
STEER_DEADBAND   = 0.005   # 转向死区 rad
MAX_LATERAL_ACCEL = 1.4    # 转向限幅最大横向加速度 m/s²
STEER_LIMIT_MIN  = 0.016   # 最小转向限幅 rad
STEER_LIMIT_MAX  = 0.16    # 最大转向限幅 rad
ROAD_GUARD_Y     = 4.5     # 路沿保护阈值 m

# 变道参数
LANE_WIDTH       = 3.75    # 车道宽度 m
LC_TRIGGER_TIME  = 3.0     # 变道触发时间 s


# ══════════════════════════════════════════════════════════════
#  LTV MPC 求解器（完全移植 ltv_mpc.c）
# ══════════════════════════════════════════════════════════════

LTV_MPC_MAX_HORIZON = 20
LTV_MPC_OK          = 0
LTV_MPC_ERR_SINGULAR = -2

class LtvMpcConfig:
    def __init__(self):
        self.q_y       = 10.0
        self.q_psi     = 20.0
        self.q_delta   = 2.0
        self.r_ddelta  = 0.5
        self.qf_y      = 20.0
        self.qf_psi    = 40.0
        self.horizon   = 20
        self.dt        = 0.05
        self.wheelbase = WHEELBASE
        self.max_steer  = 0.35
        self.max_dsteer = 0.5

class LtvMpcSolver:
    def __init__(self, cfg=None):
        self.cfg = cfg if cfg else LtvMpcConfig()
        self.v_ref = [0.0]*LTV_MPC_MAX_HORIZON
        self.kappa_ref = [0.0]*LTV_MPC_MAX_HORIZON
        self.ref_n = 0
        self.e_y = self.e_psi = self.delta = self.v = 0.0
        self.K = [[0.0]*3 for _ in range(LTV_MPC_MAX_HORIZON)]
        self.kff = [0.0]*LTV_MPC_MAX_HORIZON

    def set_reference(self, v_ref, kappa_ref, N):
        n = min(N, LTV_MPC_MAX_HORIZON)
        for i in range(n):
            self.v_ref[i] = v_ref[i]
            self.kappa_ref[i] = kappa_ref[i]
        self.ref_n = n

    def set_state(self, e_y, e_psi, delta, v):
        self.e_y = e_y
        self.e_psi = e_psi
        self.delta = delta
        self.v = v

    def solve(self):
        cfg = self.cfg
        N = cfg.horizon
        dt = cfg.dt
        L = cfg.wheelbase
        max_steer = cfg.max_steer
        max_dsteer = cfg.max_dsteer

        if N < 1 or N > LTV_MPC_MAX_HORIZON:
            return None, -1

        P = [[0.0]*3 for _ in range(3)]
        P[0][0] = cfg.qf_y
        P[1][1] = cfg.qf_psi
        P[2][2] = 0.0
        p = [0.0, 0.0, 0.0]

        Q = [[0.0]*3 for _ in range(3)]
        Q[0][0] = cfg.q_y
        Q[1][1] = cfg.q_psi
        Q[2][2] = cfg.q_delta
        R = cfg.r_ddelta

        for k in range(N-1, -1, -1):
            vk = self.v_ref[k] if k < self.ref_n else self.v
            kk = self.kappa_ref[k] if k < self.ref_n else 0.0
            v_safe = vk if vk >= 0.01 else 0.01

            A = [[0.0]*3 for _ in range(3)]
            A[0][0] = 1.0;  A[0][1] = v_safe * dt;  A[0][2] = 0.5 * v_safe * dt
            A[1][1] = 1.0;  A[1][2] = 0.5 * v_safe / L * dt
            A[2][2] = 1.0
            B = [0.0, 0.0, dt]
            c = [0.0, -kk * v_safe * dt, 0.0]

            BPB = 0.0
            for i in range(3):
                for j in range(3):
                    BPB += B[i] * P[i][j] * B[j]
            Quu = R + BPB
            if abs(Quu) < 1e-12:
                return None, LTV_MPC_ERR_SINGULAR
            Quu_inv = 1.0 / Quu

            BtP = [0.0]*3
            for j in range(3):
                for i in range(3):
                    BtP[j] += B[i] * P[i][j]

            for j in range(3):
                s = 0.0
                for i in range(3):
                    s += BtP[i] * A[i][j]
                self.K[k][j] = -Quu_inv * s

            Pc_p = [0.0]*3
            for i in range(3):
                s = p[i]
                for j in range(3):
                    s += P[i][j] * c[j]
                Pc_p[i] = s

            BtPc_p = 0.0
            for i in range(3):
                BtPc_p += B[i] * Pc_p[i]
            self.kff[k] = -Quu_inv * BtPc_p

            A_cl = [[0.0]*3 for _ in range(3)]
            for i in range(3):
                for j in range(3):
                    A_cl[i][j] = A[i][j] + B[i] * self.K[k][j]

            AtPA = [[0.0]*3 for _ in range(3)]
            for i in range(3):
                for j in range(3):
                    s = 0.0
                    for ii in range(3):
                        for jj in range(3):
                            s += A_cl[ii][i] * P[ii][jj] * A_cl[jj][j]
                    AtPA[i][j] = s

            KtRK = [[0.0]*3 for _ in range(3)]
            for i in range(3):
                for j in range(3):
                    KtRK[i][j] = self.K[k][i] * R * self.K[k][j]

            for i in range(3):
                for j in range(3):
                    P[i][j] = Q[i][j] + AtPA[i][j] + KtRK[i][j]

            p = [0.0, 0.0, 0.0]

        x = [self.e_y, self.e_psi, self.delta]
        best_u = 0.0
        for k in range(N):
            u = self.K[k][0]*x[0] + self.K[k][1]*x[1] + self.K[k][2]*x[2] + self.kff[k]
            if u >  max_dsteer: u =  max_dsteer
            if u < -max_dsteer: u = -max_dsteer
            if k == 0:
                best_u = u

            vk = self.v_ref[k] if k < self.ref_n else self.v
            kk = self.kappa_ref[k] if k < self.ref_n else 0.0
            v_safe = vk if vk >= 0.01 else 0.01

            x_next = [0.0]*3
            x_next[0] = x[0] + dt * (v_safe * x[1] + 0.5 * v_safe * x[2])
            x_next[1] = x[1] + dt * (0.5 * v_safe / L * x[2] - kk * v_safe)
            x_next[2] = x[2] + dt * u
            if x_next[2] >  max_steer: x_next[2] =  max_steer
            if x_next[2] < -max_steer: x_next[2] = -max_steer
            x = x_next

            if math.isnan(x[0]) or math.isnan(x[1]) or math.isnan(x[2]):
                return None, -4

        return best_u, LTV_MPC_OK


# ══════════════════════════════════════════════════════════════
#  改进版 Stanley 控制器（C代码默认）
# ══════════════════════════════════════════════════════════════

class StanleyParams:
    def __init__(self):
        self.lat_kp         = 0.5
        self.lat_kd_heading = 2.0
        self.yaw_damping    = 0.28
        self.k_vy           = 0.35
        self.k_vy_damp      = 0.6
        self.curve_ff_boost_radius = 60.0
        self.curve_ff_boost_factor = 1.5

def steer_limit_for_speed(speed_mps, max_lat_accel=MAX_LATERAL_ACCEL):
    speed = speed_mps if speed_mps >= 2.0 else 2.0
    limit = math.atan(max_lat_accel * WHEELBASE / (speed * speed))
    if limit < STEER_LIMIT_MIN: limit = STEER_LIMIT_MIN
    if limit > STEER_LIMIT_MAX: limit = STEER_LIMIT_MAX
    return limit

def stanley_control(lat_error, heading_error, yaw_rate, speed, ref_kappa, prev_steer, params):
    """
    改进版 Stanley 控制（完全对齐 control_node.cpp 562-605 行）
    输入符号约定（C代码）：
      lat_error = target_y - ego_y  (右侧为正)
      heading_error = ego_heading - ref_heading  (左侧为正)
    """
    speed_eff = max(speed, 3.0)
    v_lat_actual = speed * math.sin(heading_error)

    v_y_des = params.k_vy * lat_error - params.k_vy_damp * v_lat_actual

    psi_des = 0.0
    vy_ratio = v_y_des / speed_eff
    if vy_ratio > 0.5: vy_ratio = 0.5
    if vy_ratio < -0.5: vy_ratio = -0.5
    psi_des = math.asin(vy_ratio)  # ref_heading = 0, 所以psi_des直接是修正量

    delta_ff = math.atan(WHEELBASE * v_y_des / (speed_eff * speed_eff + 1e-6))

    ref_h_eff = psi_des
    if abs(ref_h_eff - 0.0) > 0.5:  # 相对ref_heading=0
        ref_h_eff = 0.0

    cte_term     = math.atan2(params.lat_kp * lat_error, speed_eff)
    heading_term = params.lat_kd_heading * heading_error  # ego_heading - ref_h_eff = heading_error - psi_des? 不，C代码这里是ego_heading - ref_h_eff
    # 注意：C代码中heading_term是 g.lat_kd_heading * (g.ego_heading - ref_h_eff)
    # ref_h_eff = psi_des（相对road heading的修正），而road heading是ref_heading，
    # 在我们仿真里ref_heading=0，所以heading输入就是ego_heading
    heading_term = params.lat_kd_heading * (heading_error - ref_h_eff)

    yaw_damp_term = params.yaw_damping * yaw_rate
    kappa = ref_kappa
    ff_weight = 1.0
    if abs(kappa) > 1e-9:
        R = 1.0 / abs(kappa)
        if R <= params.curve_ff_boost_radius:
            ff_weight = params.curve_ff_boost_factor
    ff_term = WHEELBASE * kappa * ff_weight

    steer = cte_term - heading_term - yaw_damp_term + ff_term + delta_ff

    steer_limit = steer_limit_for_speed(speed)
    if steer >  steer_limit: steer =  steer_limit
    if steer < -steer_limit: steer = -steer_limit

    steer = STEER_FILTER_ALPHA * steer + (1.0 - STEER_FILTER_ALPHA) * prev_steer
    if abs(steer) < STEER_DEADBAND: steer = 0.0

    return steer


# ══════════════════════════════════════════════════════════════
#  运动学自行车模型
# ══════════════════════════════════════════════════════════════

class VehicleState:
    def __init__(self, x0=0.0, y0=0.0, v0=0.0, heading0=0.0):
        self.x = x0
        self.y = y0
        self.v = v0
        self.heading = heading0
        self.yaw_rate = 0.0
        self.steer = 0.0

    def step(self, steer_cmd, throttle, brake, dt=DT):
        """运动学自行车模型积分（与flowsim一致）"""
        if brake > 0:
            self.v -= brake * 8.0 * dt
            if self.v < 0: self.v = 0
        elif throttle > 0:
            self.v += throttle * 3.0 * dt
            if self.v > CRUISE_SPEED + 2: self.v = CRUISE_SPEED + 2

        self.steer = steer_cmd
        self.yaw_rate = self.v / WHEELBASE * math.tan(steer_cmd)

        self.x += self.v * math.cos(self.heading) * dt
        self.y += self.v * math.sin(self.heading) * dt
        self.heading += self.yaw_rate * dt

        while self.heading >  math.pi: self.heading -= 2*math.pi
        while self.heading < -math.pi: self.heading += 2*math.pi


# ══════════════════════════════════════════════════════════════
#  仿真运行
# ══════════════════════════════════════════════════════════════

class SimResult:
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.v = []
        self.heading = []
        self.steer = []
        self.lat_error = []
        self.target_y = []
        self.settling_time = None
        self.overshoot = None
        self.max_steer = 0.0
        self.steady_state_error = None
        self.collided = False
        self.stable = False

def run_simulation(use_mpc=False, do_lane_change=False,
                   stanley_params=None, mpc_config=None,
                   target_speed=CRUISE_SPEED, duration=SIM_DURATION,
                   initial_y=0.0, initial_heading=0.0):
    if stanley_params is None:
        stanley_params = StanleyParams()
    if mpc_config is None:
        mpc_config = LtvMpcConfig()

    ego = VehicleState(x0=0.0, y0=initial_y, v0=5.0, heading0=initial_heading)
    mpc = LtvMpcSolver(mpc_config) if use_mpc else None
    prev_steer = 0.0

    result = SimResult()
    lane_change_done = False
    target_y = 0.0
    lc_time = None

    n_steps = int(duration / DT)
    for step in range(n_steps):
        t = step * DT

        # 变道逻辑：t>LC_TRIGGER_TIME后阶跃目标y到左车道
        if do_lane_change and t >= LC_TRIGGER_TIME and not lane_change_done:
            target_y = -LANE_WIDTH  # 左变道：y减小（ENU坐标系y北，这里道路沿x东，y侧向）
            lane_change_done = True
            lc_time = t

        # 当前速度控制：简单PID加速到目标速度
        speed_error = target_speed - ego.v
        throttle = 0.0
        brake = 0.0
        if speed_error > 0:
            throttle = min(speed_error / 10.0, 0.5)
        else:
            brake = min(-speed_error / 10.0, 0.3)

        # 误差计算
        lat_error = target_y - ego.y  # Stanley约定
        heading_error = ego.heading - 0.0  # 直道ref_heading=0
        e_y = ego.y - target_y  # MPC约定
        e_psi = 0.0 - ego.heading  # MPC约定：ref_h - ego_h

        steer_cmd = 0.0
        mpc_ok = False

        if use_mpc:
            v_ref = [target_speed] * LTV_MPC_MAX_HORIZON
            kappa_ref = [0.0] * LTV_MPC_MAX_HORIZON
            mpc.set_state(e_y, e_psi, prev_steer, ego.v)
            mpc.set_reference(v_ref, kappa_ref, LTV_MPC_MAX_HORIZON)
            ddelta, rc = mpc.solve()
            if rc == LTV_MPC_OK and ddelta is not None:
                # C代码直接加，不乘dt（保持一致行为）
                steer_cmd = prev_steer + ddelta
                mpc_ok = True

        if not mpc_ok:
            steer_cmd = stanley_control(lat_error, heading_error, ego.yaw_rate,
                                        ego.v, 0.0, prev_steer, stanley_params)

        # 路沿保护
        if abs(ego.y - target_y) > ROAD_GUARD_Y - 0.5:
            limit = steer_limit_for_speed(ego.v, 2.4)
            steer_cmd = limit if lat_error > 0 else -limit

        prev_steer = steer_cmd

        # 记录
        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.lat_error.append(lat_error)
        result.target_y.append(target_y)
        if abs(steer_cmd) > result.max_steer:
            result.max_steer = abs(steer_cmd)

        # 碰撞检测（飞出路面）
        if abs(ego.y) > 6.0:
            result.collided = True
            break

        # 积分
        ego.step(steer_cmd, throttle, brake)

    # 统计指标
    if not result.collided:
        # 稳态误差（最后2秒）
        tail_start = max(0, len(result.lat_error) - int(2.0/DT))
        tail_err = [abs(e) for e in result.lat_error[tail_start:]]
        result.steady_state_error = sum(tail_err) / len(tail_err) if tail_err else 999

        # 变道调节时间（进入±0.3m带宽不再离开）
        if do_lane_change and lc_time is not None:
            settle_threshold = 0.3
            settled_idx = None
            for i in range(len(result.t)):
                if result.t[i] < lc_time: continue
                if all(abs(result.lat_error[j]) < settle_threshold for j in range(i, len(result.lat_error))):
                    settled_idx = i
                    break
            if settled_idx is not None:
                result.settling_time = result.t[settled_idx] - lc_time

            # 超调
            min_y = min(result.y[settled_idx:]) if settled_idx else min(result.y)
            result.overshoot = abs(min_y - target_y)

        # 稳定性：最后2秒steer振荡<0.02rad
        tail_steer = result.steer[tail_start:]
        steer_amp = max(tail_steer) - min(tail_steer) if tail_steer else 999
        result.stable = steer_amp < 0.02 and result.steady_state_error < 0.1

    return result


# ══════════════════════════════════════════════════════════════
#  输出与评估
# ══════════════════════════════════════════════════════════════

def print_result(result, label=""):
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    if result.collided:
        print("  ❌ COLLIDED / FLEW OFF ROAD")
        return
    print(f"  最终位置:         x={result.x[-1]:.1f}m, y={result.y[-1]:.2f}m")
    print(f"  最终速度:         {result.v[-1]:.1f} m/s")
    print(f"  稳态横向误差:     {result.steady_state_error:.3f} m")
    print(f"  最大转向角:       {math.degrees(result.max_steer):.1f}°")
    if result.settling_time is not None:
        print(f"  变道调节时间:     {result.settling_time:.2f} s")
    if result.overshoot is not None:
        print(f"  变道超调:         {result.overshoot:.2f} m")
    print(f"  直道稳定性:       {'✅ 稳定' if result.stable else '⚠️  振荡'}")

def write_csv(result, filename):
    with open(filename, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'v', 'heading', 'steer', 'lat_error', 'target_y'])
        for i in range(len(result.t)):
            w.writerow([f"{result.t[i]:.3f}",
                        f"{result.x[i]:.3f}",
                        f"{result.y[i]:.3f}",
                        f"{result.v[i]:.2f}",
                        f"{result.heading[i]:.4f}",
                        f"{result.steer[i]:.4f}",
                        f"{result.lat_error[i]:.3f}",
                        f"{result.target_y[i]:.3f}"])
    print(f"\n  CSV written to: {filename}")


# ══════════════════════════════════════════════════════════════
#  参数扫描
# ══════════════════════════════════════════════════════════════

def tune_straight():
    """扫描Stanley直道保持参数"""
    print("\n" + "="*60)
    print("  Stanley 直道保持参数扫描")
    print("="*60)
    best = None
    best_score = 1e9

    for k_vy in [0.2, 0.3, 0.35, 0.4, 0.5]:
        for k_vy_damp in [0.4, 0.6, 0.8, 1.0, 1.2]:
            for yaw_damp in [0.15, 0.2, 0.28, 0.35, 0.45]:
                for lat_kp in [0.3, 0.4, 0.5, 0.6, 0.8]:
                    p = StanleyParams()
                    p.k_vy = k_vy
                    p.k_vy_damp = k_vy_damp
                    p.yaw_damping = yaw_damp
                    p.lat_kp = lat_kp
                    # 从1m初始偏移开始测试调节性能
                    r = run_simulation(use_mpc=False, do_lane_change=False,
                                      stanley_params=p, duration=10.0,
                                      initial_y=1.0)
                    r.steady_state_error = abs(r.y[-1])
                    r.stable = abs(max(r.steer[-40:]) - min(r.steer[-40:])) < 0.02
                    if r.stable and not r.collided:
                        score = r.steady_state_error + abs(max(r.steer) - min(r.steer))*10
                        if score < best_score:
                            best_score = score
                            best = (k_vy, k_vy_damp, yaw_damp, lat_kp, r)

    if best:
        k_vy, k_vy_damp, yaw_damp, lat_kp, r = best
        print(f"\n  ✅ 最优直道参数:")
        print(f"     lat_kp={lat_kp}, k_vy={k_vy}, k_vy_damp={k_vy_damp}, yaw_damping={yaw_damp}")
        print(f"     稳态误差={r.steady_state_error:.3f}m, 转向振幅={math.degrees(abs(max(r.steer)-min(r.steer))):.2f}°")
    else:
        print("  ⚠️  未找到稳定参数")

def tune_lane_change():
    """扫描Stanley变道参数"""
    print("\n" + "="*60)
    print("  Stanley 变道参数扫描")
    print("="*60)
    best = None
    best_score = 1e9

    for k_vy in [0.2, 0.3, 0.35, 0.4]:
        for k_vy_damp in [0.4, 0.6, 0.8, 1.0]:
            for yaw_damp in [0.2, 0.28, 0.35]:
                for lat_kd_h in [1.0, 1.5, 2.0, 2.5, 3.0]:
                    p = StanleyParams()
                    p.k_vy = k_vy
                    p.k_vy_damp = k_vy_damp
                    p.yaw_damping = yaw_damp
                    p.lat_kd_heading = lat_kd_h
                    r = run_simulation(use_mpc=False, do_lane_change=True,
                                      stanley_params=p, duration=15.0)
                    if r.stable and not r.collided and r.settling_time is not None:
                        score = r.settling_time + r.overshoot*5
                        if score < best_score:
                            best_score = score
                            best = (k_vy, k_vy_damp, yaw_damp, lat_kd_h, r)

    if best:
        k_vy, k_vy_damp, yaw_damp, lat_kd_h, r = best
        print(f"\n  ✅ 最优变道参数:")
        print(f"     lat_kd_heading={lat_kd_h}, k_vy={k_vy}, k_vy_damp={k_vy_damp}, yaw_damping={yaw_damp}")
        print(f"     调节时间={r.settling_time:.2f}s, 超调={r.overshoot:.2f}m, 最大转向={math.degrees(r.max_steer):.1f}°")
    else:
        print("  ⚠️  未找到能完成变道的稳定参数")


# ══════════════════════════════════════════════════════════════
#  主入口
# ══════════════════════════════════════════════════════════════

def main():
    global CRUISE_SPEED
    parser = argparse.ArgumentParser(description="FlowEngine 横向控制仿真验证工具")
    parser.add_argument("--lc", action="store_true", help="变道场景")
    parser.add_argument("--mpc", action="store_true", help="启用LTV-MPC（默认Stanley）")
    parser.add_argument("--tune", action="store_true", help="扫描Stanley最优参数")
    parser.add_argument("--tune-mpc", action="store_true", help="扫描LTV-MPC最优参数")
    parser.add_argument("--speed", type=float, default=CRUISE_SPEED, help=f"巡航速度 (默认{CRUISE_SPEED}m/s)")
    parser.add_argument("--duration", type=float, default=SIM_DURATION, help="仿真时长")
    parser.add_argument("--csv", type=str, default=None, help="输出CSV文件路径")
    args = parser.parse_args()

    CRUISE_SPEED = args.speed

    if args.tune:
        tune_straight()
        tune_lane_change()
        return

    if args.tune_mpc:
        print("MPC参数扫描功能待实现（当前默认参数q_y=10,q_psi=20,r=0.5因C代码直接加未乘dt有单位问题）")
        return

    # 默认单次仿真
    mode_str = []
    mode_str.append("LTV-MPC" if args.mpc else "改进版Stanley")
    mode_str.append("变道" if args.lc else "直道保持")
    label = f"{mode_str[0]} @ {args.speed:.0f}m/s — {mode_str[1]}"

    result = run_simulation(use_mpc=args.mpc, do_lane_change=args.lc,
                           target_speed=args.speed, duration=args.duration)
    print_result(result, label)

    if args.csv:
        write_csv(result, args.csv)
    else:
        outdir = "/tmp/flow_control_sim"
        os.makedirs(outdir, exist_ok=True)
        csvname = f"{'mpc' if args.mpc else 'stanley'}_{'lc' if args.lc else 'straight'}_{int(args.speed)}ms.csv"
        write_csv(result, os.path.join(outdir, csvname))

    # 简要总结
    if not result.collided and result.stable:
        if args.lc and result.settling_time:
            print(f"\n  🚗 变道成功！耗时{result.settling_time:.1f}s，超调{result.overshoot:.2f}m")
        else:
            print(f"\n  🚗 直道保持稳定！稳态误差{result.steady_state_error:.3f}m")
    elif not result.collided:
        print(f"\n  ⚠️  存在持续振荡，需要调参")
    else:
        print(f"\n  ❌ 控制失败，车辆飞出路面")

if __name__ == "__main__":
    main()
