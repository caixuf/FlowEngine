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
LANE_WIDTH       = 3.5     # 车道宽度 m（与 C 代码 4 车道场景一致）
N_LANES          = 4       # 车道数
LC_TRIGGER_TIME  = 3.0     # 变道触发时间 s
# ROAD_GUARD：偏离目标车道中心超过此值 → 强制回正（匹配修复后的 C 代码）
ROAD_GUARD_THRESHOLD = 3.0  # m


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
#  规划层仿真（planning_node → trajectory）
# ══════════════════════════════════════════════════════════════

def lane_center_y(lane_idx, n_lanes=N_LANES, lane_w=LANE_WIDTH):
    """车道中心 y（与 C 代码 lane_center_y 公式一致）。
    0=最左, N-1=最右。4 车道: lane 0→+5.25, 1→+1.75, 2→-1.75, 3→-5.25"""
    return -(lane_idx - (n_lanes - 1) / 2.0) * lane_w

class TrajectoryPoint:
    __slots__ = ('x', 'y', 's', 'l', 'heading', 'kappa', 'v')
    def __init__(self, x=0, y=0, s=0, l=0, heading=0, kappa=0, v=0):
        self.x, self.y = x, y
        self.s, self.l = s, l       # Frenet (s=纵向, l=横向偏移)
        self.heading, self.kappa, self.v = heading, kappa, v

class PlanningLayer:
    """模拟 planning_node 的轨迹生成逻辑。

    核心逻辑（与 C 代码一致）：
    1. ego_d = ego_y - road_center_y (当前横向偏移)
    2. d_out[i] = ego_d * (1-t) + target_lane_offset * t (线性插值到目标车道)
    3. frenet_to_cartesian: x=s, y=road_center + d*cos(theta), heading=theta, kappa=0
    4. 前视点(0.5s)的 l 字段供 control 的 lane_d 使用

    road_center_y = 0（直道），target_lane_offset = 目标车道 y - road_center_y
    """
    def __init__(self, n_points=10, horizon_m=50.0):
        self.n_points = n_points
        self.horizon = horizon_m

    def generate(self, ego_x, ego_y, ego_v, target_lane_offset, command_speed):
        """生成轨迹点列表。"""
        ego_d = ego_y  # road_center = 0
        pts = []
        for i in range(self.n_points):
            t = i / (self.n_points - 1) if self.n_points > 1 else 0.0
            s = ego_x + self.horizon * t
            d = ego_d * (1.0 - t) + target_lane_offset * t
            # frenet_to_cartesian（直道：theta=0, cos=1, sin=0）
            x = s
            y = d  # road_center(=0) + d * cos(0)
            heading = 0.0
            kappa = 0.0
            v = command_speed
            pts.append(TrajectoryPoint(x, y, s, d, heading, kappa, v))
        return pts


# ══════════════════════════════════════════════════════════════
#  控制层仿真（control_node — 修复后的 road_center 逻辑）
# ══════════════════════════════════════════════════════════════

class ControlLayer:
    """模拟 control_node 的轨迹消费 + 横向控制。

    修复点（与 C 代码同步）：
    1. road_center_y = trajectory_y - l * cos(heading)（减去横向偏移，得到真正道路中心）
    2. target_lane_center = road_center_y + lane_d（目标车道中心）
    3. ROAD_GUARD 检查 |ego_y - target_lane_center| > 阈值（非 |ego_y - road_center|）
    4. lat_error = ego_y - target_lane_center
    """
    def __init__(self, params=None):
        self.params = params or StanleyParams()
        self.ref_path = []
        self.prev_steer = 0.0
        self.lane_d = 0.0          # 前视点横向偏移（从 trajectory 提取）
        self.road_center_y = 0.0   # 道路中心 y（修复后 = traj_y - l*cos(h)）
        self.target_speed = 0.0

    def on_trajectory(self, traj_points):
        """存储轨迹并提取前视点信息（匹配 on_trajectory 回调）。"""
        self.ref_path = traj_points
        if not traj_points:
            return
        # 前视点：0.5s 后的位置（索引 = 0.5s / DT = 10 → 取第 10 个点或最后一个）
        lookahead_idx = min(int(0.5 / DT), len(traj_points) - 1)
        self.lane_d = traj_points[lookahead_idx].l
        self.target_speed = traj_points[-1].v

    def compute_steer(self, ego_x, ego_y, ego_v, ego_heading, ego_yaw_rate):
        """计算转向角（匹配修复后的 control_node 逻辑）。"""
        if not self.ref_path:
            return 0.0

        # 找最近轨迹点
        best_d2 = 1e9
        best = None
        for p in self.ref_path:
            d2 = (p.x - ego_x)**2 + (p.y - ego_y)**2
            if d2 < best_d2:
                best_d2 = d2
                best = p
        if not best or best_d2 > 25.0:  # >5m 偏离
            return 0.0

        # 修复核心：road_center = traj_y - l * cos(heading)
        # 旧 bug：直接用 best.y（含偏移）→ cruise_lane_y 双重计算 → 发散
        self.road_center_y = best.y - best.l * math.cos(best.heading)

        # 目标车道中心 = 道路中心 + 前视点横向偏移
        target_lane_center = self.road_center_y + self.lane_d

        # 横向误差（修复后：相对目标车道中心，非道路中心）
        lat_error = target_lane_center - ego_y  # Stanley 约定
        heading_error = ego_heading - best.heading

        # Stanley 控制
        steer = stanley_control(lat_error, heading_error, ego_yaw_rate,
                                ego_v, best.kappa, self.prev_steer, self.params)

        # ROAD_GUARD（修复后：检查偏离目标车道，非道路中心）
        y_from_target = abs(ego_y - target_lane_center)
        if y_from_target > ROAD_GUARD_THRESHOLD:
            # 强制回正
            limit = steer_limit_for_speed(ego_v, 2.4)
            steer = limit if lat_error > 0 else -limit

        self.prev_steer = steer
        return steer


# ══════════════════════════════════════════════════════════════
#  闭环仿真（planning → control → vehicle dynamics）
# ══════════════════════════════════════════════════════════════

def run_closed_loop(stanley_params=None, target_lane=2, do_lane_change=False,
                    target_speed=CRUISE_SPEED, duration=SIM_DURATION,
                    initial_y=None, initial_v=5.0, start_lane=2):
    """planning→control 闭环仿真。

    与 run_simulation 的区别：
    - planning 层生成完整轨迹（d_out blending），非直接设 target_y
    - control 层从轨迹提取 road_center/lane_d（模拟修复后的逻辑）
    - 测试完整 planning→control 数据通路

    变道场景：t < LC_TRIGGER_TIME 保持在 start_lane，之后切到 target_lane。
    """
    if stanley_params is None:
        stanley_params = StanleyParams()

    final_offset = lane_center_y(target_lane)
    start_offset = lane_center_y(start_lane)
    if initial_y is None:
        initial_y = start_offset  # 从起始车道出发

    ego = VehicleState(x0=0.0, y0=initial_y, v0=initial_v, heading0=0.0)
    planner = PlanningLayer()
    controller = ControlLayer(stanley_params)

    result = SimResult()
    lc_time = None
    current_target = start_offset  # 当前目标车道偏移

    n_steps = int(duration / DT)
    for step in range(n_steps):
        t = step * DT

        # 变道触发：t >= LC_TRIGGER_TIME 后切换目标车道
        if do_lane_change and t >= LC_TRIGGER_TIME and lc_time is None:
            lc_time = t
            current_target = final_offset

        # 速度控制
        speed_error = target_speed - ego.v
        throttle = min(speed_error / 10.0, 0.5) if speed_error > 0 else 0.0
        brake = min(-speed_error / 10.0, 0.3) if speed_error < 0 else 0.0

        # planning 生成轨迹（目标 = current_target）
        traj = planner.generate(ego.x, ego.y, ego.v, current_target, target_speed)

        # control 消费轨迹
        controller.on_trajectory(traj)
        steer_cmd = controller.compute_steer(ego.x, ego.y, ego.v, ego.heading, ego.yaw_rate)

        # 记录
        target_y = current_target
        lat_error = target_y - ego.y
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

        # 飞出路面检测（4 车道半宽 = 7m）
        if abs(ego.y) > N_LANES * LANE_WIDTH / 2 + 1.0:
            result.collided = True
            break

        # 积分
        ego.step(steer_cmd, throttle, brake)

    # 统计
    if not result.collided:
        tail_start = max(0, len(result.lat_error) - int(2.0/DT))
        tail_err = [abs(e) for e in result.lat_error[tail_start:]]
        result.steady_state_error = sum(tail_err) / len(tail_err) if tail_err else 999

        if do_lane_change and lc_time is not None:
            settle_threshold = 0.3
            settled_idx = None
            for i in range(len(result.t)):
                if result.t[i] < lc_time: continue
                if all(abs(result.lat_error[j]) < settle_threshold
                       for j in range(i, len(result.lat_error))):
                    settled_idx = i
                    break
            if settled_idx is not None:
                result.settling_time = result.t[settled_idx] - lc_time
            min_y = min(result.y[settled_idx:]) if settled_idx else min(result.y)
            result.overshoot = abs(min_y - final_offset)

        tail_steer = result.steer[tail_start:]
        steer_amp = max(tail_steer) - min(tail_steer) if tail_steer else 999
        result.stable = steer_amp < 0.02 and result.steady_state_error < 0.1

    return result


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


def tune_joint():
    """联合调参：planning→control 闭环，直道(lane 2) + 变道(lane 2→3)，15 m/s。

    评分 = 直道稳态误差*3 + 变道调节时间 + 变道超调*5 + 转向振幅*10
    只保留两个场景都稳定且不碰撞的参数。
    """
    print("\n" + "="*60)
    print("  联合调参: planning→control 闭环 (直道+变道, 15 m/s, 4车道)")
    print("="*60)
    best = None
    best_score = 1e9
    n_tried = 0
    n_valid = 0

    for lat_kp in [0.3, 0.4, 0.5, 0.6]:
        for lat_kd_h in [1.5, 2.0, 2.5, 3.0]:
            for yaw_damp in [0.2, 0.28, 0.35]:
                for k_vy in [0.2, 0.3, 0.4]:
                    for k_vy_damp in [0.4, 0.6, 0.8]:
                        n_tried += 1
                        p = StanleyParams()
                        p.lat_kp = lat_kp
                        p.lat_kd_heading = lat_kd_h
                        p.yaw_damping = yaw_damp
                        p.k_vy = k_vy
                        p.k_vy_damp = k_vy_damp

                        # 场景1: 直道 lane 2 (y=-1.75), 带初始偏移
                        r1 = run_closed_loop(p, target_lane=2, do_lane_change=False,
                                             target_speed=15.0, duration=10.0,
                                             initial_y=-1.0)  # 从 y=-1 开始（偏离 lane 2）
                        if r1.collided or not r1.stable:
                            continue

                        # 场景2: 变道 lane 2→3 (y=-1.75→-5.25)
                        r2 = run_closed_loop(p, target_lane=3, do_lane_change=True,
                                             target_speed=15.0, duration=15.0,
                                             initial_y=lane_center_y(2))  # 从 lane 2 出发
                        if r2.collided or not r2.stable or r2.settling_time is None:
                            continue

                        n_valid += 1
                        steer_amp = max(r1.steer[-40:]) - min(r1.steer[-40:])
                        score = (r1.steady_state_error * 3 +
                                 r2.settling_time +
                                 r2.overshoot * 5 +
                                 steer_amp * 10)
                        if score < best_score:
                            best_score = score
                            best = (lat_kp, lat_kd_h, yaw_damp, k_vy, k_vy_damp, r1, r2)

    print(f"\n  尝试参数组合: {n_tried}, 有效(两场景均稳定): {n_valid}")
    if best:
        lat_kp, lat_kd_h, yaw_damp, k_vy, k_vy_damp, r1, r2 = best
        print(f"\n  ✅ 最优联合参数:")
        print(f"     lat_kp={lat_kp}, lat_kd_heading={lat_kd_h}, yaw_damping={yaw_damp}")
        print(f"     k_vy={k_vy}, k_vy_damp={k_vy_damp}")
        print(f"\n  直道 (lane 2, 15 m/s):")
        print(f"     稳态误差={r1.steady_state_error:.4f}m, 最大转向={math.degrees(r1.max_steer):.1f}°")
        print(f"\n  变道 (lane 2→3, 15 m/s):")
        print(f"     调节时间={r2.settling_time:.2f}s, 超调={r2.overshoot:.2f}m, 最大转向={math.degrees(r2.max_steer):.1f}°")
        print(f"\n  综合评分: {best_score:.3f} (越低越好)")
        print(f"\n  flowctl 固化命令:")
        print(f"     flowctl param set control.lat_kp {lat_kp}")
        print(f"     flowctl param set control.lat_kd_heading {lat_kd_h}")
        print(f"     flowctl param set control.yaw_damping {yaw_damp}")
        print(f"     flowctl param set control.k_vy {k_vy}")
        print(f"     flowctl param set control.k_vy_damp {k_vy_damp}")
    else:
        print("  ⚠️  未找到两场景均稳定的参数")


# ══════════════════════════════════════════════════════════════
#  主入口
# ══════════════════════════════════════════════════════════════

def main():
    global CRUISE_SPEED
    parser = argparse.ArgumentParser(description="FlowEngine 横向控制仿真验证工具")
    parser.add_argument("--lc", action="store_true", help="变道场景")
    parser.add_argument("--mpc", action="store_true", help="启用LTV-MPC（默认Stanley）")
    parser.add_argument("--tune", action="store_true", help="扫描Stanley最优参数（纯控制层）")
    parser.add_argument("--tune-joint", action="store_true",
                        help="联合调参: planning→control 闭环（直道+变道）")
    parser.add_argument("--tune-mpc", action="store_true", help="扫描LTV-MPC最优参数")
    parser.add_argument("--plan", action="store_true",
                        help="使用 planning→control 闭环仿真（非直接设 target_y）")
    parser.add_argument("--lane", type=int, default=2,
                        help=f"目标车道 0~{N_LANES-1}（默认2）")
    parser.add_argument("--speed", type=float, default=CRUISE_SPEED, help=f"巡航速度 (默认{CRUISE_SPEED}m/s)")
    parser.add_argument("--duration", type=float, default=SIM_DURATION, help="仿真时长")
    parser.add_argument("--csv", type=str, default=None, help="输出CSV文件路径")
    args = parser.parse_args()

    CRUISE_SPEED = args.speed

    if args.tune:
        tune_straight()
        tune_lane_change()
        return

    if args.tune_joint:
        tune_joint()
        return

    if args.tune_mpc:
        print("MPC参数扫描功能待实现（当前默认参数q_y=10,q_psi=20,r=0.5因C代码直接加未乘dt有单位问题）")
        return

    # 单次仿真
    if args.plan:
        # planning→control 闭环模式
        mode_str = f"planning→control 闭环 @ {args.speed:.0f}m/s"
        mode_str += f" — 变道→lane{args.lane}" if args.lc else " — 直道保持"
        result = run_closed_loop(stanley_params=StanleyParams(),
                                 target_lane=args.lane,
                                 do_lane_change=args.lc,
                                 target_speed=args.speed,
                                 duration=args.duration)
    else:
        # 纯控制层模式（原逻辑）
        mode_str = []
        mode_str.append("LTV-MPC" if args.mpc else "改进版Stanley")
        mode_str.append("变道" if args.lc else "直道保持")
        mode_str = f"{mode_str[0]} @ {args.speed:.0f}m/s — {mode_str[1]}"
        result = run_simulation(use_mpc=args.mpc, do_lane_change=args.lc,
                               target_speed=args.speed, duration=args.duration)

    print_result(result, mode_str)

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
