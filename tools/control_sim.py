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
STEER_FULL_LOCK  = 0.60    # 物理满舵角 rad（entity.h steer_override=true 的限幅）
ROAD_GUARD_Y     = 4.5     # 路沿保护阈值 m

# 变道参数
LANE_WIDTH       = 3.5     # 车道宽度 m（与 C 代码 4 车道场景一致）
N_LANES          = 4       # 车道数
LC_TRIGGER_TIME  = 3.0     # 变道触发时间 s
# ROAD_GUARD：偏离目标车道中心超过此值 → 强制回正（匹配修复后的 C 代码）
ROAD_GUARD_THRESHOLD = 3.0  # m


def _rect_max_abs_y(y, h, half_len=2.3, half_wid=1.0):
    """整车矩形 4 角点 |y| 最大值（2026-08-07 Fix A 占据空间）。

    与 C++ generate_uturn_trajectory 的 rect_max_abs_y 一致：校验车身全部
    4 角点的 |y| 都落在路沿内，任意角越界即收（人手式"不碰路边"），
    替代旧实现只看单个"外侧角点"。
    """
    sh, ch = math.sin(h), math.cos(h)
    c = [y + half_len * sh + half_wid * ch,
         y + half_len * sh - half_wid * ch,
         y - half_len * sh + half_wid * ch,
         y - half_len * sh - half_wid * ch]
    return max(abs(v) for v in c)



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

    def step(self, steer_cmd, throttle, brake, dt=DT, grade=0.0, v_max=None):
        """运动学自行车模型积分（与physics.cpp一致，车辆中心参考点，支持倒车）

        参数扩展（默认值不变，向后兼容）：
          grade — 坡度角(rad)，上坡为正；加重力分量 v += -g·sin(grade)·dt
          v_max — 速度上限（默认 CRUISE_SPEED+2，科目二可设低速上限）
        """
        # 注意：UTurn 时 steer 可能超过 0.25（C 代码限制），
        # 因为三把方向掉头/单把 U-turn 需要更大的转向角才能在有限路宽内完成
        # C 代码 physics.cpp 的 0.25 限幅在 U-turn 场景下需要临时放宽

        if throttle < 0:
            # 倒车：负油门（三把方向掉头用）
            # 匹配 C 代码 physics.cpp: drive_force = throttle * 5000, mass=1500
            # accel = throttle * 5000 / 1500 = throttle * 3.33
            self.v += throttle * 3.33 * dt
            if self.v < -4.0: self.v = -4.0  # 匹配 C 代码物理限制
        elif brake > 0:
            self.v -= brake * 8.0 * dt
            if self.v < 0: self.v = 0
        elif throttle > 0:
            self.v += throttle * 3.33 * dt
            _vmax = v_max if v_max is not None else CRUISE_SPEED + 2
            if self.v > _vmax: self.v = _vmax

        # 坡度重力分量（上坡减速、下坡加速）
        if grade != 0.0:
            self.v += -9.81 * math.sin(grade) * dt

        self.steer = steer_cmd
        self.yaw_rate = self.v / WHEELBASE * math.tan(steer_cmd)

        # 车辆中心参考点（与physics.cpp step_bicycle一致）
        # 后轴不可横向移动，车辆中心绕后轴转动
        # dx_c = v·cos(θ)·dt - half_wb·sin(θ)·yaw_rate·dt
        # dy_c = v·sin(θ)·dt + half_wb·cos(θ)·yaw_rate·dt
        half_wb = WHEELBASE * 0.5
        self.x += (self.v * math.cos(self.heading)
                   - half_wb * math.sin(self.heading) * self.yaw_rate) * dt
        self.y += (self.v * math.sin(self.heading)
                   + half_wb * math.cos(self.heading) * self.yaw_rate) * dt
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
#  三把方向掉头仿真（纯 Python 验证，与 C 代码 flowsim_node.cpp 一致）
# ══════════════════════════════════════════════════════════════

class UturnParams:
    """掉头可调参数

    mode='single': 单把匀速 U-turn（仅 phase0 生效，走恒定 steer 的圆弧）
    mode='three_point': 三把方向掉头（所有 phase 生效）
    mode='multi': 多把方向掉头（N-point，角点约束退出，2026-08-04 新增）
    """
    def __init__(self, mode='single'):
        self.mode = mode
        # 通用参数
        self.init_speed = 5.0         # 掉头前速度 m/s

        # Phase 0: 前进转向（单把 U-turn 用这个就够了）
        self.phase0_steer = 0.60      # rad (单把 U-turn 需要 0.6+)
        self.phase0_throttle = 0.0    # 0=匀速
        self.phase0_duration = 2.5    # s

        # 三把方向专用参数
        # Phase 1: 先刹车再倒车反打方向
        self.phase1_brake = 1.0       # 刹车力度(0~1)
        self.phase1_steer = -0.60     # rad (倒车方向)
        self.phase1_reverse_throttle = -0.7
        self.phase1_reverse_duration = 0.0  # 0=关闭三把方向模式

        # Phase 2: 前进回正
        self.phase2_steer = 0.0
        self.phase2_throttle = 0.0
        self.phase2_duration = 0.0

        # ── multi 模式（多把方向，角点约束退出）──
        # 2026-08-07 Fix A：路沿从真实车道布局推导（不再硬编码 4 车道 7.0），
        # 角点上限 = 路沿 − 护栏余量，与 C++ generate_uturn_trajectory 一致。
        self.road_half = (N_LANES / 2.0) * LANE_WIDTH   # 路沿 y（4 车道 = 7.0）
        self.guardrail_margin = 1.0                     # 护栏安全余量 m
        self.corner_limit = self.road_half - self.guardrail_margin
        self.reverse_corridor = 0.5       # 倒车回撤走廊（中心回到 |y|≤0.5 即收）
        self.max_strokes = 5              # 最多前进弧把数
        self.stroke_steer = 0.60          # 前弧满舵（掉头规范"向左打死"，去/返程同号）
        self.reverse_steer = -0.60        # 倒车满舵（反打）
        self.cruise_speed = 3.5           # 前弧速度 m/s
        self.reverse_speed = -2.5         # 倒车速度 m/s（负值）
        self.finish_heading_tol = 0.10    # 完成 heading 容差（rad）
        self.finish_lane_tol = 1.75       # 完成 y 在目标车道半幅内
        self.stroke_timeout_s = 4.0       # 每把超时
        self.reverse_timeout_s = 3.0      # 倒车超时
        self.reverse_h_guard = 0.30       # 倒车 heading 护栏（离目标 0.3 rad 内即收，
                                          #   防止倒车把 heading 转过目标）


class UturnResult:
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.v = []
        self.heading = []
        self.steer = []
        self.final_heading = 0.0
        self.final_y = 0.0
        self.final_x = 0.0
        self.heading_error = 999.0  # 目标 heading 偏差（目标 π）
        self.lane_error = 999.0     # 目标车道偏差（目标 opposite lane y）
        self.success = False
        self.min_dist_to_center = 999.0  # 过程中离道路中心最近距离（评估是否压线）
        self.corner_max = 0.0            # 全程车身角点 |y| 最大值（multi 模式，出路沿门禁）
        self.strokes_used = 0            # 实际使用的把数（multi 模式）

    def print_phases(self, params):
        if params.mode == 'multi':
            print(f"  多把方向: corner_limit={params.corner_limit:.1f}m, "
                  f"corridor={params.reverse_corridor:.1f}m, "
                  f"max_strokes={params.max_strokes}, "
                  f"v_fwd={params.cruise_speed:.1f} v_rev={params.reverse_speed:.1f}")
            return
        print(f"  Phase 0: steer={params.phase0_steer:.2f}rad, "
              f"throttle={params.phase0_throttle:.2f}, duration={params.phase0_duration:.2f}s")
        print(f"  Phase 1: brake={params.phase1_brake:.1f} to stop, "
              f"then steer={params.phase1_steer:.2f}rad, "
              f"reverse_throttle={params.phase1_reverse_throttle:.2f}, "
              f"reverse_duration={params.phase1_reverse_duration:.2f}s")
        print(f"  Phase 2: steer={params.phase2_steer:.2f}rad, "
              f"throttle={params.phase2_throttle:.2f}, duration={params.phase2_duration:.2f}s")


def run_uturn_simulation(params=None, start_lane_y=-1.75, start_heading=0.0):
    """纯 Python 掉头仿真

    mode='single'（默认）: 单把匀速 U-turn
      只用 phase0，走恒定 steer 的圆弧。适合参数扫描找最优转向角。

    mode='three_point': 三把方向掉头
      Phase 0: 前进左打 → 车头摆入对向车道
      Phase 1: 先刹车停稳，再倒车右打 → 车尾摆正完成掉头
      Phase 2: 前进回正，进入对向车道

    mode='multi': 多把方向掉头（N-point，2026-08-04 新增）
      前进弧满舵（±0.6）以「车身角点」到达 corner_limit 为界（人手式不碰路边），
      倒车反打回撤到 reverse_corridor 走廊，末把 heading 对准目标车道即收。
      替代固定角度（130°/170°）退出 —— 固定角度几何本身就把角点甩出路沿 0.12m，
      实测再叠加跟踪偏差出沿 2.1m（2026-08-04 长跑）。

    输入：
      start_lane_y: 起始车道 y（默认 -1.75 = 前进车道）
      start_heading: 起始 heading（默认 0 = 前进端；返程传 π）

    输出：
      UturnResult：含轨迹 + 最终 heading/y 偏差 + corner_max（全程角点门禁）
    """
    if params is None:
        params = UturnParams()

    ego = VehicleState(x0=0.0, y0=start_lane_y, v0=params.init_speed, heading0=start_heading)
    result = UturnResult()

    target_y = -start_lane_y  # 对向车道中心 y

    if params.mode == 'single':
        # ── 单把匀速 U-turn ──
        n_steps = int(params.phase0_duration / DT)
        for _ in range(n_steps):
            result.t.append(len(result.t) * DT)
            result.x.append(ego.x)
            result.y.append(ego.y)
            result.v.append(ego.v)
            result.heading.append(ego.heading)
            result.steer.append(params.phase0_steer)
            ego.step(params.phase0_steer, params.phase0_throttle, 0.0, dt=DT)
    elif params.mode == 'three_point':
        # ── 三把方向掉头 ──
        # Phase 0: 前进左打
        n_steps0 = int(params.phase0_duration / DT)
        for _ in range(n_steps0):
            result.t.append(len(result.t) * DT)
            result.x.append(ego.x)
            result.y.append(ego.y)
            result.v.append(ego.v)
            result.heading.append(ego.heading)
            result.steer.append(params.phase0_steer)
            ego.step(params.phase0_steer, params.phase0_throttle, 0.0, dt=DT)

        # Phase 1a: 刹车停稳（steer 保持）
        while ego.v > 0.1:
            result.t.append(len(result.t) * DT)
            result.x.append(ego.x)
            result.y.append(ego.y)
            result.v.append(ego.v)
            result.heading.append(ego.heading)
            result.steer.append(params.phase1_steer)
            ego.step(params.phase1_steer, 0.0, params.phase1_brake, dt=DT)

        # Phase 1b: 倒车反打方向
        if params.phase1_reverse_duration > 0:
            n_steps1b = int(params.phase1_reverse_duration / DT)
            for _ in range(n_steps1b):
                result.t.append(len(result.t) * DT)
                result.x.append(ego.x)
                result.y.append(ego.y)
                result.v.append(ego.v)
                result.heading.append(ego.heading)
                result.steer.append(params.phase1_steer)
                ego.step(params.phase1_steer, params.phase1_reverse_throttle, 0.0, dt=DT)

        # Phase 2: 前进回正
        if params.phase2_duration > 0:
            n_steps2 = int(params.phase2_duration / DT)
            for _ in range(n_steps2):
                result.t.append(len(result.t) * DT)
                result.x.append(ego.x)
                result.y.append(ego.y)
                result.v.append(ego.v)
                result.heading.append(ego.heading)
                result.steer.append(params.phase2_steer)
                ego.step(params.phase2_steer, params.phase2_throttle, 0.0, dt=DT)

    elif params.mode == 'multi':
        # ── 多把方向掉头（N-point，角点约束退出）──
        # 与 C++ generate_uturn_trajectory stroke 循环完全一致：
        #   * 前进弧 steer=+0.6（掉头规范"向左打死"，去/返程同号），1.2s 渐进 ramp
        #   * 退出 = 车身角点达 corner_limit（方向相关的外侧角点）| 对准 | 超时
        #   * 倒车 steer=-0.6（反打，heading 继续向目标转），回撤到 |y|≤corridor
        #   * 末把对准（heading±0.1 且 y 在目标车道半幅）即收
        # 车身角点公式（len=4.6, wid=2.0，与 C++/flowsim 一致）：
        #   去程北角点: y + 2.3*sin(h) + 1.0*|cos(h)|
        #   返程南角点: y + 2.3*sin(h) - 1.0*|cos(h)|
        def _norm(h):
            while h >  math.pi: h -= 2.0 * math.pi
            while h < -math.pi: h += 2.0 * math.pi
            return h
        target_h = 0.0 if abs(_norm(start_heading)) > math.pi * 0.5 else math.pi
        stroke_dir = -1.0 if target_h < 0.5 else 1.0   # 去程 +1（北侧约束）/ 返程 -1（南侧）
        hb = WHEELBASE * 0.5  # half_wb 切向项
        # 返程进入 heading 用 -π 表示（与 C 代码 norm 后积分一致）
        if abs(_norm(start_heading)) > math.pi * 0.5:
            ego.heading = -math.pi

        def _record(steer, v):
            result.t.append(len(result.t) * DT)
            result.x.append(ego.x); result.y.append(ego.y)
            result.v.append(ego.v); result.heading.append(ego.heading)
            result.steer.append(steer)
            corner = _rect_max_abs_y(ego.y, ego.heading)   # Fix A：整车 4 角点扫掠
            result.corner_max = max(result.corner_max, corner)

        done = False
        for stroke in range(params.max_strokes):
            if done: break
            result.strokes_used = stroke + 1
            # ── 前进弧 ──
            t_stroke = 0.0
            while True:
                ramp = min(1.0, t_stroke / 1.2)              # 1.2s 渐进打满（与 C++ 一致）
                steer = params.stroke_steer * ramp
                yaw_rate = (params.cruise_speed / WHEELBASE) * math.tan(steer)
                ego.heading += yaw_rate * DT
                ego.x += params.cruise_speed * math.cos(ego.heading) * DT - hb * math.sin(ego.heading) * yaw_rate * DT
                ego.y += params.cruise_speed * math.sin(ego.heading) * DT + hb * math.cos(ego.heading) * yaw_rate * DT
                ego.v = params.cruise_speed
                _record(steer, ego.v)
                corner = _rect_max_abs_y(ego.y, ego.heading)   # Fix A：整车 4 角点扫掠
                corner_ok = corner <= params.corner_limit      # 双向统一按 |y| 判越界
                aligned = (abs(_norm(ego.heading) - target_h) < params.finish_heading_tol
                           and abs(ego.y - target_y) < params.finish_lane_tol)
                t_stroke += DT
                if (not corner_ok) or aligned or (t_stroke >= params.stroke_timeout_s):
                    done = aligned
                    break
            if done: break
            # ── 刹停点（换挡 D→R）──
            ego.v = 0.0
            _record(0.0, 0.0)
            # ── 倒车（反打；outbound 继续向目标转 / return 回摆 heading）──
            t_rev = 0.0
            while True:
                steer = params.reverse_steer
                yaw_rate = (params.reverse_speed / WHEELBASE) * math.tan(steer)
                ego.heading += yaw_rate * DT
                ego.x += params.reverse_speed * math.cos(ego.heading) * DT - hb * math.sin(ego.heading) * yaw_rate * DT
                ego.y += params.reverse_speed * math.sin(ego.heading) * DT + hb * math.cos(ego.heading) * yaw_rate * DT
                ego.v = params.reverse_speed
                _record(steer, ego.v)
                t_rev += DT
                # 回撤走廊（outbound: y≤+0.5 / return: y≥-0.5）或 heading 护栏或超时
                corridor = (ego.y <= params.reverse_corridor) if stroke_dir > 0 \
                           else (ego.y >= -params.reverse_corridor)
                h_guard = ((ego.heading >= target_h - params.reverse_h_guard)
                           if stroke_dir > 0 else (ego.heading <= -math.pi + params.reverse_h_guard))
                if corridor or h_guard or (t_rev >= params.reverse_timeout_s):
                    break
            # ── 刹停点（换挡 R→D）──
            ego.v = 0.0
            _record(0.0, 0.0)

    # 评估
    result.final_heading = ego.heading
    result.final_y = ego.y
    result.final_x = ego.x
    # 目标 heading：前进端掉头 π / 返程掉头 0（2026-08-04 返程掉头修复同款语义）
    target_heading = 0.0 if abs(start_heading) > math.pi * 0.5 else math.pi
    # 正确归一化到 [-π, π] 再取 abs
    he = ego.heading - target_heading
    while he >  math.pi: he -= 2.0 * math.pi
    while he < -math.pi: he += 2.0 * math.pi
    result.heading_error = abs(he)
    result.lane_error = abs(ego.y - target_y)
    # 判据：heading 偏差 < 1.0 rad + 车在路面上（未飞出）
    # 注意：lane_error 是到对向车道中心 y=1.75 的距离，但车只要在路面上就算成功
    if params.mode == 'multi':
        # 多把方向：严格判据（角点全程 ≤ corner_limit + 对准 + 车道）
        result.success = (result.corner_max <= params.corner_limit + 0.2
                          and result.heading_error < 0.15
                          and result.lane_error < params.finish_lane_tol)
    else:
        result.success = (result.heading_error < 1.0 and abs(ego.y) < 7.0)

    return result


def print_uturn_result(result, params, label="", start_lane_y=-1.75, start_heading=0.0):
    target_y = -start_lane_y
    target_h = 0.0 if abs(start_heading) > math.pi * 0.5 else math.pi
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"{'='*60}")
    result.print_phases(params)
    print(f"  最终位置:         x={result.final_x:.1f}m, y={result.final_y:.2f}m")
    print(f"  最终 heading:     {result.final_heading:.2f} rad ({math.degrees(result.final_heading):.1f}°)")
    print(f"  heading 偏差:     {result.heading_error:.2f} rad (目标 {target_h:.2f})")
    print(f"  车道偏差:         {result.lane_error:.2f} m (目标 y={target_y:.2f})")
    if params.mode == 'multi':
        print(f"  全程角点 max|y|:  {result.corner_max:.2f} m (路沿 7.0, 上限 {params.corner_limit:.1f})")
        print(f"  实际把数:         {result.strokes_used}")
    status = "PASS" if result.success else "FAIL"
    print(f"  状态:             {status}")


def tune_uturn():
    """扫描单把 U-turn 参数，找最优组合"""
    print("\n" + "=" * 60)
    print("  单把 U-turn 参数扫描")
    print("=" * 60)
    print("  运动学自行车模型: yaw_rate = v/L * tan(steer)")
    print(f"  轴距 L={WHEELBASE}m, 路宽 14m (4车道)")
    print("  目标: heading=π(180°), 对向车道 y≈1.75")
    print("=" * 60)

    best = None
    best_practical = None
    best_score = 1e9
    best_practical_score = 1e9
    n_tried = 0
    n_valid = 0

    # steer 需要足够大才能在路宽内完成 U-turn
    # R = L/tan(steer), y_final = -1.75 + 2*R
    for steer_deg in range(25, 56, 1):  # 25°~55°
        steer = math.radians(steer_deg)
        for init_v in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0]:
            yaw_rate = init_v / WHEELBASE * math.tan(steer)
            if yaw_rate < 0.01: continue
            t_needed = math.pi / yaw_rate
            duration = t_needed * 1.1

            for throttle in [0.0]:
                n_tried += 1
                p = UturnParams(mode='single')
                p.phase0_steer = steer
                p.phase0_duration = duration
                p.phase0_throttle = throttle
                p.init_speed = init_v
                r = run_uturn_simulation(p, start_lane_y=-1.75)
                if not r.success: continue
                n_valid += 1

                R = WHEELBASE / math.tan(steer)
                lat_accel = init_v * init_v / R
                # 综合评分：heading精度 + 车道偏差 + 横向加速度(尽量小)
                score = (r.heading_error * 8.0
                         + r.lane_error * 2.0
                         + (lat_accel / 9.81) * 2.0)  # 以g为单位惩罚

                # 找最佳整体
                if score < best_score:
                    best_score = score
                    best = (steer_deg, init_v, lat_accel, duration, r)

                # 找最佳实用参数 (lat_accel < 0.6g)
                if lat_accel < 0.6 * 9.81:
                    practical_score = (r.heading_error * 8.0 + r.lane_error * 2.0 + (lat_accel / 9.81) * 1.0)
                    if practical_score < best_practical_score:
                        best_practical_score = practical_score
                        best_practical = (steer_deg, init_v, lat_accel, duration, r)

    print(f"\n  尝试: {n_tried}, 有效: {n_valid}")

    # 显示最佳整体
    if best:
        sd, v, la, dur, r = best
        R = WHEELBASE / math.tan(math.radians(sd))
        print(f"\n  {'='*50}")
        print(f"  🏆 最佳整体 (评分={best_score:.2f})")
        print(f"  {'='*50}")
        print(f"  转向角:  {sd}° ({math.radians(sd):.2f}rad)")
        print(f"  速度:    {v:.0f} m/s, 持续时间: {dur:.2f}s")
        print(f"  半径:    {R:.2f}m, 横向加速度: {la/9.81:.2f}g")
        print(f"  heading: {r.final_heading:.2f}rad ({math.degrees(r.final_heading):.0f}°)")
        print(f"  h_err={r.heading_error:.2f}rad, l_err={r.lane_error:.2f}m, dx={r.final_x:.1f}m")

    # 显示最佳实用参数
    if best_practical:
        sd, v, la, dur, r = best_practical
        R = WHEELBASE / math.tan(math.radians(sd))
        print(f"\n  {'='*50}")
        print(f"  ✅ 最佳实用参数 (lat<0.6g, 评分={best_practical_score:.2f})")
        print(f"  {'='*50}")
        print(f"  转向角:      {sd}° ({math.radians(sd):.2f}rad)")
        print(f"  初始速度:    {v:.0f} m/s")
        print(f"  持续时间:    {dur:.2f}s")
        print(f"  转向半径:    {R:.2f}m")
        print(f"  横向加速度:  {la/9.81:.2f}g ({la:.1f} m/s²)")
        print(f"  ─────────────────────────────")
        print(f"  最终位置:    x={r.final_x:.1f}m, y={r.final_y:.2f}m")
        print(f"  最终 heading:{r.final_heading:.2f}rad ({math.degrees(r.final_heading):.0f}°)")
        print(f"  heading 偏差:{r.heading_error:.2f}rad (目标 π={math.pi:.2f})")
        print(f"  车道偏差:    {r.lane_error:.2f}m (目标 y=1.75)")
        print(f"  ─────────────────────────────")
        print(f"  推荐 C 代码配置:")
        print(f"     steer={math.radians(sd):.2f} (需临时放宽物理限幅)")
        print(f"     init_speed={v:.0f}")
        print(f"     duration={dur:.2f}s")
        print(f"     throttle=0.0 (匀速)")
    else:
        if best:
            print("\n  ⚠️  无 lat<0.6g 的实用参数，建议增大 steer 上限")
        else:
            print("\n  ⚠️  未找到任何有效参数")


def tune_uturn_three_point():
    """扫描三把方向 U-turn 参数，找最优组合"""
    print("\n" + "=" * 60)
    print("  三把方向 U-turn 参数扫描")
    print("=" * 60)
    print("  运动学自行车模型: yaw_rate = v/L * tan(steer)")
    print(f"  轴距 L={WHEELBASE}m, 路宽 14m (4车道)")
    print("  目标: heading=π(180°), 对向车道 y≈1.75")
    print("  策略: 左打死前进 → 右打死倒车 → 左打前进对齐")
    print("=" * 60)

    best = None
    best_three = None
    best_score = 1e9
    best_three_score = 1e9
    n_tried = 0

    # Phase 0: 前进左打转向（第一阶段）
    for p0_steer in [0.40, 0.45, 0.50, 0.55, 0.60]:
        for p0_dur in [1.5, 2.0, 2.5, 3.0]:
            for p0_thr in [0.18, 0.22, 0.26, 0.30]:
                # Phase 1: 倒车右打反方向
                for p1_steer in [-0.50, -0.55, -0.60]:
                    for p1_dur in [1.5, 2.0, 2.5, 3.0, 3.5]:
                        for p1_thr in [-0.30, -0.35, -0.40]:
                            # Phase 2: 前进对齐
                            for p2_steer in [0.05, 0.10, 0.15]:
                                for p2_dur in [0.3, 0.5, 0.8, 1.0]:
                                    n_tried += 1
                                    p = UturnParams(mode='three_point')
                                    p.init_speed = 4.0
                                    p.phase0_steer = p0_steer
                                    p.phase0_throttle = p0_thr
                                    p.phase0_duration = p0_dur
                                    p.phase1_steer = p1_steer
                                    p.phase1_reverse_throttle = p1_thr
                                    p.phase1_reverse_duration = p1_dur
                                    p.phase2_steer = p2_steer
                                    p.phase2_throttle = 0.24
                                    p.phase2_duration = p2_dur
                                    r = run_uturn_simulation(p, start_lane_y=-1.75)

                                    # 综合评分：heading精度 + 车道偏差 + 转向惩罚(越小越好)
                                    avg_steer = (abs(p0_steer) + abs(p1_steer) + abs(p2_steer)) / 3.0
                                    score = (r.heading_error * 10.0
                                             + r.lane_error * 5.0
                                             + avg_steer * 1.0)

                                    if r.success and score < best_score:
                                        best_score = score
                                        best = (p, r, score)

                                    # 三把方向专用：要求 heading 转过 ≥150° 且不压线
                                    dh = abs(r.final_heading - math.pi)
                                    if (r.success and dh < 0.5
                                            and r.min_dist_to_center > 0.5
                                            and score < best_three_score):
                                        best_three_score = score
                                        best_three = (p, r, score)

    print(f"\n  尝试: {n_tried}")

    if best:
        params, r, score = best
        print(f"\n  {'='*50}")
        print(f"  🏆 最佳整体 (评分={score:.2f})")
        print(f"  {'='*50}")
        print(f"  Phase 0: steer={params.phase0_steer:.2f}rad, throttle={params.phase0_throttle:.2f}, duration={params.phase0_duration:.2f}s")
        print(f"  Phase 1: steer={params.phase1_steer:.2f}rad, reverse_throttle={params.phase1_reverse_throttle:.2f}, reverse_duration={params.phase1_reverse_duration:.2f}s")
        print(f"  Phase 2: steer={params.phase2_steer:.2f}rad, throttle={params.phase2_throttle:.2f}, duration={params.phase2_duration:.2f}s")
        print(f"  最终位置:      x={r.final_x:.1f}m, y={r.final_y:.2f}m")
        print(f"  最终 heading:  {r.final_heading:.2f}rad ({math.degrees(r.final_heading):.0f}°)")
        print(f"  heading 偏差:  {r.heading_error:.2f}rad (目标 π={math.pi:.2f})")
        print(f"  车道偏差:      {r.lane_error:.2f}m (目标 y=1.75)")
        print(f"  推荐 C 代码配置:")
        print(f"    p0_steer={params.phase0_steer:.2f}  p0_thr={params.phase0_throttle:.2f}  p0_dur={params.phase0_duration:.2f}")
        print(f"    p1_steer={params.phase1_steer:.2f}  p1_thr={params.phase1_reverse_throttle:.2f}  p1_dur={params.phase1_reverse_duration:.2f}")
        print(f"    p2_steer={params.phase2_steer:.2f}  p2_dur={params.phase2_duration:.2f}")

    if best_three:
        params, r, score = best_three
        print(f"\n  {'='*50}")
        print(f"  ✅ 最佳三把方向 (dh<0.5rad, 不压线, 评分={score:.2f})")
        print(f"  {'='*50}")
        print(f"  Phase 0: steer={params.phase0_steer:.2f}rad, throttle={params.phase0_throttle:.2f}, duration={params.phase0_duration:.2f}s")
        print(f"  Phase 1: steer={params.phase1_steer:.2f}rad, reverse_throttle={params.phase1_reverse_throttle:.2f}, reverse_duration={params.phase1_reverse_duration:.2f}s")
        print(f"  Phase 2: steer={params.phase2_steer:.2f}rad, throttle={params.phase2_throttle:.2f}, duration={params.phase2_duration:.2f}s")
        print(f"  最终位置:      x={r.final_x:.1f}m, y={r.final_y:.2f}m")
        print(f"  最终 heading:  {r.final_heading:.2f}rad ({math.degrees(r.final_heading):.0f}°)")
        print(f"  heading 偏差:  {r.heading_error:.2f}rad (目标 π={math.pi:.2f})")
        print(f"  车道偏差:      {r.lane_error:.2f}m (目标 y=1.75)")
        print(f"  距中线最小距离: {r.min_dist_to_center:.2f}m")
        print(f"  推荐 C 代码配置:")
        print(f"    p0_steer={params.phase0_steer:.2f}  p0_thr={params.phase0_throttle:.2f}  p0_dur={params.phase0_duration:.2f}")
        print(f"    p1_steer={params.phase1_steer:.2f}  p1_thr={params.phase1_reverse_throttle:.2f}  p1_dur={params.phase1_reverse_duration:.2f}")
        print(f"    p2_steer={params.phase2_steer:.2f}  p2_dur={params.phase2_duration:.2f}")
    else:
        print("\n  ⚠️  未找到满足三把方向条件的参数")


# ══════════════════════════════════════════════════════════════
#  场景全集 — 6 种上路操作（纯 Python 验证，给 C++ 上能力做支撑）
# ══════════════════════════════════════════════════════════════
#  场景 1: curve     — 曲线跟随（弯道保持）
#  场景 2: emergency — 紧急制动（前方障碍物急刹）
#  场景 3: stop_go   — 跟停再起步（前车停→走，ACC 跟车）
#  场景 4: obstacle  — 障碍物避让（变道绕行）
#  场景 5: merge     — 匝道汇入（加速车道汇入主路）
#  场景 6: cutin     — 加塞处理（旁车突然切入本车道）
# ══════════════════════════════════════════════════════════════

class ScenarioResult:
    """场景仿真结果"""
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.v = []
        self.heading = []
        self.steer = []
        self.throttle = []
        self.brake = []
        self.success = False
        self.collision = False
        self.off_road = False
        self.score = 0.0
        self.summary = ""

    def print_trajectory(self, stride=10):
        print(f"  轨迹 (每{stride*DT:.1f}s):")
        for i in range(0, len(self.t), stride):
            yaw_deg = math.degrees(self.heading[i]) if i < len(self.heading) else 0
            print(f"    t={self.t[i]:.1f}s: x={self.x[i]:.1f}, y={self.y[i]:.2f}, "
                  f"h={yaw_deg:.0f}°, v={self.v[i]:.2f}m/s, steer={self.steer[i]:.2f}rad")


# ── 弯道模型 ─────────────────────────────────────────────────

class CurvedRoad:
    """弯道道路模型

    支持常量曲率弯道。道路中心线参数方程：
      heading(s) = s * kappa
      x(s) = integral(cos(heading(s))) ds ≈ sin(s*kappa)/kappa
      y(s) = integral(sin(heading(s))) ds ≈ (1 - cos(s*kappa))/kappa

    对于 kappa=0（直道），退化到直道模型。
    """
    def __init__(self, kappa=0.0, length=500.0):
        self.kappa = kappa       # 曲率 (1/m)，正=右弯
        self.length = length     # 道路长度 m

    def center_heading_at(self, s):
        """道路中心线在 s 处的航向角"""
        return s * self.kappa

    def center_pos_at(self, s):
        """道路中心线在 s 处的 (x, y)"""
        if abs(self.kappa) < 1e-9:
            return (s, 0.0)
        return (math.sin(s * self.kappa) / self.kappa,
                (1.0 - math.cos(s * self.kappa)) / self.kappa)

    def lane_center_y(self, lane_idx, n_lanes=N_LANES, lane_w=LANE_WIDTH):
        """车道中心 y（垂直道路方向），与 lane_center_y 函数一致"""
        return -(lane_idx - (n_lanes - 1) / 2.0) * lane_w


# ── 纵向控制器（ACC + 紧急制动） ─────────────────────────────

class LongitudinalController:
    """纵向控制器：ACC 跟车 + 紧急制动

    模式：
      - cruise: 保持目标速度
      - follow: 保持安全跟车距离（IDM 风格）
      - brake: 紧急制动
    """
    def __init__(self, target_speed=CRUISE_SPEED, time_gap=1.5, min_gap=5.0):
        self.target_speed = target_speed
        self.time_gap = time_gap       # 时距 s
        self.min_gap = min_gap         # 最小间距 m
        self.max_accel = 2.0           # 最大加速度 m/s²
        self.max_brake = 5.0           # 最大制动减速度 m/s²

    def compute(self, ego_v, lead_dist=None, lead_v=None, lead_accel=None):
        """计算 throttle/brake

        输入：
          ego_v: 自车速度 m/s
          lead_dist: 与前车距离 m（None=无前车）
          lead_v: 前车速度 m/s
        输出：
          (throttle, brake)
        """
        if lead_dist is not None and lead_dist < self.min_gap:
            # 跟车距离过小 → 紧急制动
            brake = min(self.max_brake, (self.min_gap - lead_dist) * 2.0 + 0.3)
            return (0.0, min(brake, 1.0))

        if lead_dist is not None and lead_v is not None:
            # ACC 跟车模式：保持安全间距
            desired_gap = self.min_gap + ego_v * self.time_gap
            gap_error = lead_dist - desired_gap  # 正=间距太大

            if gap_error < -1.0:
                # 间距太小 → 减速
                brake = min(self.max_brake, -gap_error * 0.5 + max(0, (ego_v - lead_v)) * 0.3)
                return (0.0, min(brake, 1.0))
            elif gap_error > 2.0 and ego_v < self.target_speed:
                # 间距大且没到目标速度 → 加速
                throttle = min(self.max_accel, gap_error * 0.2 + (self.target_speed - ego_v) * 0.05)
                return (min(throttle, 0.5), 0.0)

        # 巡航模式：保持目标速度
        speed_error = self.target_speed - ego_v
        if speed_error > 0.5:
            throttle = min(self.max_accel, speed_error * 0.1)
            return (min(throttle, 0.5), 0.0)
        elif speed_error < -0.5:
            brake = min(self.max_brake, -speed_error * 0.1)
            return (0.0, min(brake, 0.3))
        return (0.0, 0.0)


def tune_uturn_multi():
    """扫描多把方向 U-turn 参数（角点约束退出），找最优组合

    网格: corner_limit × init_speed × max_strokes，去程+返程各跑一遍。
    判据（全 PASS）:
      1. corner_max ≤ corner_limit + 0.2（全程车身角点不出沿 7.0）
      2. 完成位姿: heading_error < 0.15 且 lane_error < 1.75
      3. 总时长 ≤ 30s（behavior uturn_timeout_s=40 预算）
    """
    print("\n" + "=" * 60)
    print("  多把方向 U-turn 参数扫描（角点约束退出）")
    print("=" * 60)
    print(f"  轴距 L={WHEELBASE}m, 路宽 14m (4车道, 路沿 ±7.0)")
    print("  策略: 满舵前进弧（角点达限即收）→ 反打倒车回撤 → 末把对准")
    print("  去程（y=-1.75→+1.75, h: 0→π）+ 返程（y=+1.75→-1.75, h: π→0）双跑")
    print("=" * 60)

    results = []
    n_tried = 0
    for corner_limit in [6.0, 6.3, 6.5, 6.7]:
        for init_speed in [3.5, 5.0, 8.0]:
            for max_strokes in [3, 5]:
                for v_rev in [-2.0, -2.5, -3.0]:
                    n_tried += 1
                    p = UturnParams(mode='multi')
                    p.corner_limit = corner_limit
                    p.init_speed = init_speed
                    p.max_strokes = max_strokes
                    p.reverse_speed = v_rev
                    r_fwd = run_uturn_simulation(p, start_lane_y=-1.75, start_heading=0.0)
                    r_ret = run_uturn_simulation(p, start_lane_y=+1.75, start_heading=math.pi)
                    both_pass = (r_fwd.success and r_ret.success)
                    # 总时长 = 模拟步数 * DT（单边）
                    dur_fwd = len(r_fwd.t) * DT
                    dur_ret = len(r_ret.t) * DT
                    ok = both_pass and max(dur_fwd, dur_ret) <= 30.0
                    score = (r_fwd.corner_max + r_ret.corner_max) / 2.0 \
                        + 0.1 * max(r_fwd.heading_error, r_ret.heading_error) \
                        + 0.05 * max(dur_fwd, dur_ret)
                    results.append((score, ok, corner_limit, init_speed, max_strokes, v_rev,
                                    r_fwd, r_ret, dur_fwd, dur_ret))
                    tag = "PASS" if ok else "fail"
                    print(f"  corner={corner_limit:.1f} v_in={init_speed:.1f} "
                          f"strokes={max_strokes} v_rev={v_rev:.1f} "
                          f"→ {tag}  corner_max=({r_fwd.corner_max:.2f}/{r_ret.corner_max:.2f}) "
                          f"h_err=({r_fwd.heading_error:.2f}/{r_ret.heading_error:.2f}) "
                          f"lane=({r_fwd.lane_error:.2f}/{r_ret.lane_error:.2f}) "
                          f"n=({r_fwd.strokes_used}/{r_ret.strokes_used}) "
                          f"t=({dur_fwd:.1f}/{dur_ret:.1f}s)")

    passed = [r for r in results if r[1]]
    print(f"\n  尝试: {n_tried}, 通过: {len(passed)}")
    if passed:
        best = min(passed, key=lambda r: r[0])
        (score, ok, cl, iv, ms, vr, rf, rr, tf, tr) = best
        print(f"\n  {'='*50}")
        print(f"  🏆 最优（评分={score:.2f}）: corner_limit={cl} init_speed={iv} "
              f"max_strokes={ms} reverse_speed={vr}")
        print(f"  去程: corner_max={rf.corner_max:.2f}m h_err={rf.heading_error:.2f} "
              f"lane_err={rf.lane_error:.2f} 把数={rf.strokes_used} 时长={tf:.1f}s")
        print(f"  返程: corner_max={rr.corner_max:.2f}m h_err={rr.heading_error:.2f} "
              f"lane_err={rr.lane_error:.2f} 把数={rr.strokes_used} 时长={tr:.1f}s")
    else:
        print("  ❌ 网格内无通过组合 — 需调整策略/参数")


# ── 场景 1: 曲线跟随（Curve following） ─────────────────────

def run_curve_scenario(kappa=0.005, target_speed=12.0, duration=15.0, use_mpc=False,
                       stanley_params=None):
    """弯道保持仿真

    道路常曲率弯道（kappa=0.005 ≈ R=200m），自车沿车道中心行驶。
    测试 Stanley/MPC 在弯道上的 heading 跟踪精度和横向偏差。

    横向偏移计算：对于常曲率弯道，道路中心线是半径为 R=1/kappa 的圆弧。
    自车到道路中心的距离 = |自车到圆心距离 - R|，此为正确横向偏移量。

    输入：
      kappa: 道路曲率 (1/m)，正=右弯
      target_speed: 目标速度 m/s
      duration: 仿真时长 s
      use_mpc: 使用 MPC 而非 Stanley
      stanley_params: 自定义 Stanley 参数（None=默认）

    输出：
      ScenarioResult
    """
    road = CurvedRoad(kappa=kappa, length=target_speed * duration)
    ego = VehicleState(x0=0.0, y0=0.0, v0=target_speed, heading0=0.0)
    result = ScenarioResult()

    lane_ctrl = stanley_params if stanley_params else StanleyParams()
    lon_ctrl = LongitudinalController(target_speed=target_speed)

    n_steps = int(duration / DT)
    prev_steer = 0.0

    # 弯道圆心（右弯时圆心在 x=0, y=R）
    R = 1.0 / kappa if abs(kappa) > 1e-9 else float('inf')
    circle_cy = R if abs(kappa) > 1e-9 else 0.0  # 右弯圆心 y=R

    for step in range(n_steps):
        t = step * DT
        s = ego.x  # 近似纵向距离（仅用于计算参考 heading）

        # 道路参考线
        ref_heading = road.center_heading_at(s)
        cx, cy = road.center_pos_at(s)

        # 横向误差：自车到道路中心线的正确距离
        # 对于常曲率弯道，用圆心距离法
        if abs(kappa) > 1e-9:
            # 自车到圆心的距离
            dc = math.sqrt(ego.x * ego.x + (ego.y - circle_cy) * (ego.y - circle_cy))
            lateral_offset = dc - R  # 正=在圆弧外侧
            # 道路中心 y 在最近点处的 y
            # 最近点在圆心到自车的方向上
            if dc > 1e-9:
                nearest_cy = circle_cy + R * (ego.y - circle_cy) / dc
            else:
                nearest_cy = circle_cy
            lat_error = nearest_cy - ego.y  # Stanley 约定：目标在右侧为正
            # 正确参考 heading：最近点处的切线方向（右弯时切线方向向下）
            # 圆上点 (x, y) 的切线方向向量 = (-(y-cy), x) 归一化
            if dc > 1e-9:
                nx = ego.x / dc
                ny = (ego.y - circle_cy) / dc
                # 切线方向（右弯顺时针）：(-ny, nx)
                nearest_heading = math.atan2(nx, -ny)  # atan2(tangent_y, tangent_x)
            else:
                nearest_heading = 0.0
        else:
            lat_error = cy - ego.y
            nearest_heading = ref_heading

        heading_error = ego.heading - nearest_heading

        # 控制
        if use_mpc:
            mpc = LtvMpcSolver()
            mpc.set_state(lat_error, heading_error, prev_steer, ego.v)
            N = 20
            v_ref = [target_speed] * N
            k_ref = [kappa] * N
            mpc.set_reference(v_ref, k_ref, N)
            steer_cmd, _ = mpc.solve()
            if steer_cmd is None:
                steer_cmd = 0.0
        else:
            steer_cmd = stanley_control(lat_error, heading_error, ego.yaw_rate,
                                        ego.v, kappa, prev_steer, lane_ctrl)

        prev_steer = steer_cmd

        # 纵向控制
        throttle, brake = lon_ctrl.compute(ego.v)

        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.throttle.append(throttle)
        result.brake.append(brake)

        # 积分
        ego.step(steer_cmd, throttle, brake)

        # 飞出路面检测（用圆心距离法计算正确横向偏移）
        if abs(kappa) > 1e-9:
            dc = math.sqrt(ego.x * ego.x + (ego.y - circle_cy) * (ego.y - circle_cy))
            lateral_offset = abs(dc - R)
        else:
            lateral_offset = abs(ego.y)
        if lateral_offset > N_LANES * LANE_WIDTH / 2 + 1.0:
            result.off_road = True
            break

    # 评估（用圆心距离法计算正确横向偏移）
    tail_n = max(1, n_steps // 2)
    tail_h_list = []
    tail_y_list = []
    for i in range(tail_n, len(result.t)):
        # 正确计算 heading 误差
        if abs(kappa) > 1e-9:
            dc = math.sqrt(result.x[i] * result.x[i] + (result.y[i] - circle_cy) * (result.y[i] - circle_cy))
            if dc > 1e-9:
                nx = result.x[i] / dc
                ny = (result.y[i] - circle_cy) / dc
                nearest_heading = math.atan2(nx, -ny)
            else:
                nearest_heading = 0.0
            # 横向偏移
            lo = abs(dc - R)
        else:
            nearest_heading = 0.0
            lo = abs(result.y[i])
        he = abs(result.heading[i] - nearest_heading)
        while he > math.pi: he -= 2.0 * math.pi
        while he < -math.pi: he += 2.0 * math.pi
        tail_h_list.append(abs(he))
        tail_y_list.append(lo)

    avg_heading_err = sum(tail_h_list) / len(tail_h_list) if tail_h_list else 999
    avg_lat_err = sum(tail_y_list) / len(tail_y_list) if tail_y_list else 999

    result.success = (avg_heading_err < 0.08 and avg_lat_err < 0.5 and not result.off_road)
    result.score = avg_heading_err * 10 + avg_lat_err * 2
    result.summary = (f"curve(kappa={kappa:.4f}, R={R:.0f}m) @ {target_speed:.0f}m/s: "
                      f"avg_heading_err={avg_heading_err:.4f}rad, "
                      f"avg_lat_err={avg_lat_err:.2f}m")
    return result


# ── 场景 2: 紧急制动（Emergency braking） ───────────────────

def run_emergency_brake_scenario(init_speed=15.0, obstacle_dist=50.0, duration=8.0):
    """紧急制动仿真

    自车巡航中，前方 obstacle_dist 处出现静止障碍物。
    自车必须紧急制动以避免碰撞。

    输入：
      init_speed: 初始速度 m/s
      obstacle_dist: 障碍物初始距离 m
      duration: 仿真时长 s

    输出：
      ScenarioResult
    """
    ego = VehicleState(x0=0.0, y0=0.0, v0=init_speed, heading0=0.0)
    obs_x = obstacle_dist  # 障碍物 x 位置
    result = ScenarioResult()

    lon_ctrl = LongitudinalController(target_speed=init_speed, time_gap=0.5, min_gap=1.0)

    n_steps = int(duration / DT)
    prev_steer = 0.0

    for step in range(n_steps):
        t = step * DT
        dist_to_obs = obs_x - ego.x

        if dist_to_obs < 0:
            result.collision = True
            break

        # 紧急制动：当 TTC < 3s 或距离 < 20m 时全力刹车
        if dist_to_obs < 20.0 or (ego.v > 0.5 and dist_to_obs / ego.v < 3.0):
            brake = min(1.0, 0.3 + (20.0 - dist_to_obs) / 20.0 * 0.7)
            throttle = 0.0
        else:
            throttle, brake = lon_ctrl.compute(ego.v)

        # 直道保持（无横向控制需求）
        steer_cmd = 0.0
        prev_steer = steer_cmd

        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.throttle.append(throttle)
        result.brake.append(brake)

        ego.step(steer_cmd, throttle, brake)

    # 评估
    stop_dist = result.x[-1] if result.x else 0
    min_dist = max(0, obstacle_dist - stop_dist)
    stop_time = result.t[-1] if result.t else duration

    result.success = (not result.collision and min_dist > 0.1)
    result.score = min_dist * 0.5 + stop_time * 0.1
    result.summary = (f"emergency_brake(v0={init_speed:.0f}m/s, obs={obstacle_dist:.0f}m): "
                      f"stop_dist={stop_dist:.1f}m, clearance={min_dist:.1f}m, "
                      f"stop_time={stop_time:.1f}s, speed_final={result.v[-1]:.1f}m/s")
    return result


# ── 场景 3: 跟停再起步（Stop-and-go） ───────────────────────

class LeadVehicle:
    """前车模型（可配置速度曲线）"""
    def __init__(self, x0=50.0, v0=10.0):
        self.x = x0
        self.v = v0
        self.initial_x = x0

    def step(self, dt, t):
        """按时间步进速度曲线

        速度曲线：
          t < 3s:   v = 10 m/s (巡航)
          3s ≤ t < 8s:  v = 10 → 0 (刹车到停)
          8s ≤ t < 13s: v = 0 (停)
          13s ≤ t:  v = 0 → 10 (加速)
        """
        if t < 3.0:
            self.v = 10.0
        elif t < 8.0:
            self.v = 10.0 * (1.0 - (t - 3.0) / 5.0)
        elif t < 13.0:
            self.v = 0.0
        else:
            self.v = min(10.0, (t - 13.0) * 2.0)
        self.x += self.v * dt


def run_stop_go_scenario(init_speed=10.0, initial_gap=30.0, duration=20.0):
    """跟停再起步仿真

    前车经历：巡航 → 刹车停车 → 静止 → 加速起步。
    自车 ACC 跟车，不能追尾，停车后能自动起步。

    输入：
      init_speed: 初始速度 m/s
      initial_gap: 初始跟车距离 m
      duration: 仿真时长 s

    输出：
      ScenarioResult
    """
    ego = VehicleState(x0=0.0, y0=0.0, v0=init_speed, heading0=0.0)
    lead = LeadVehicle(x0=initial_gap, v0=init_speed)
    result = ScenarioResult()

    lon_ctrl = LongitudinalController(target_speed=init_speed)
    prev_steer = 0.0

    n_steps = int(duration / DT)

    for step in range(n_steps):
        t = step * DT

        # 前车步进
        lead.step(DT, t)
        lead_dist = lead.x - ego.x

        if lead_dist < 0:
            result.collision = True
            result.summary = f"COLLISION at t={t:.1f}s (gap={lead_dist:.2f}m)"
            break

        # 纵向控制（ACC 跟车）
        throttle, brake = lon_ctrl.compute(ego.v, lead_dist, lead.v)

        # 横向控制（直道保持）
        steer_cmd = 0.0
        prev_steer = steer_cmd

        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.throttle.append(throttle)
        result.brake.append(brake)

        ego.step(steer_cmd, throttle, brake)

    # 评估
    if not result.collision:
        gaps = [lead.x - result.x[i] for i in range(len(result.t))]
        min_gap = min(gaps) if gaps else 999
        # 最后 3s 是否在跟车
        tail_gaps = gaps[-int(3.0 / DT):] if len(gaps) > int(3.0 / DT) else gaps
        final_gap = sum(tail_gaps) / len(tail_gaps) if tail_gaps else 999

        result.success = (min_gap > 1.0 and final_gap < 60.0)
        result.score = min_gap * 0.5 + abs(final_gap - 30.0) * 0.2
        result.summary = (f"stop_go(v0={init_speed:.0f}m/s, gap0={initial_gap:.0f}m): "
                          f"min_gap={min_gap:.1f}m, final_gap={final_gap:.1f}m, "
                          f"final_speed={result.v[-1]:.1f}m/s")
    return result


# ── 场景 4: 障碍物避让（Obstacle avoidance） ─────────────────

def run_obstacle_avoid_scenario(init_speed=12.0, obs_x=60.0, obs_width=2.0,
                                 duration=10.0, target_lane=1):
    """障碍物避让仿真

    自车巡航中，前方车道内有静止障碍物。
    自车需要变道绕行，然后回到原车道。

    输入：
      init_speed: 初始速度 m/s
      obs_x: 障碍物 x 位置 m
      obs_width: 障碍物宽度 m
      duration: 仿真时长 s
      target_lane: 避让目标车道（0=最左）

    输出：
      ScenarioResult
    """
    start_lane = 2  # 自车起始车道（索引 2 = y=-1.75）
    start_y = lane_center_y(start_lane)
    target_y = lane_center_y(target_lane)

    ego = VehicleState(x0=0.0, y0=start_y, v0=init_speed, heading0=0.0)
    result = ScenarioResult()

    planner = PlanningLayer()
    controller = ControlLayer(StanleyParams())
    lon_ctrl = LongitudinalController(target_speed=init_speed)
    prev_steer = 0.0

    n_steps = int(duration / DT)
    lc_triggered = False
    lc_return_triggered = False
    current_target = start_y

    for step in range(n_steps):
        t = step * DT
        s = ego.x

        # 决策：接近障碍物时变道绕行
        if not lc_triggered and s > obs_x - 30.0:
            lc_triggered = True
            current_target = target_y  # 切换到目标车道

        # 决策：绕过障碍物后回到原车道
        if lc_triggered and not lc_return_triggered and s > obs_x + 20.0:
            if abs(ego.y - target_y) < 0.5:  # 确认已在目标车道
                lc_return_triggered = True
                current_target = start_y

        # 障碍物碰撞检测
        if abs(ego.y - lane_center_y(start_lane)) < obs_width / 2 + 1.0:
            if obs_x - 2.0 < ego.x < obs_x + 2.0:
                result.collision = True
                result.summary = f"COLLISION with obstacle at t={t:.1f}s"
                break

        # 纵向控制
        throttle, brake = lon_ctrl.compute(ego.v)

        # Planning → Control
        traj = planner.generate(ego.x, ego.y, ego.v, current_target, init_speed)
        controller.on_trajectory(traj)
        steer_cmd = controller.compute_steer(ego.x, ego.y, ego.v, ego.heading, ego.yaw_rate)
        prev_steer = steer_cmd

        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.throttle.append(throttle)
        result.brake.append(brake)

        ego.step(steer_cmd, throttle, brake)

        if abs(ego.y) > N_LANES * LANE_WIDTH / 2 + 1.0:
            result.off_road = True
            break

    # 评估
    if not result.collision:
        # 是否成功绕过障碍物
        passed_obs = result.x[-1] > obs_x + 10.0
        # 最终是否回到原车道附近
        final_y_err = abs(ego.y - start_y)
        max_lat = max(abs(y) for y in result.y) if result.y else 0

        result.success = (passed_obs and final_y_err < 1.5 and not result.off_road)
        result.score = final_y_err * 0.5 + max_lat * 0.1
        result.summary = (f"obstacle_avoid(v0={init_speed:.0f}m/s, obs_x={obs_x:.0f}m): "
                          f"passed={passed_obs}, final_y_err={final_y_err:.2f}m, "
                          f"max|y|={max_lat:.2f}m")
    return result


# ── 场景 5: 匝道汇入（Merge） ────────────────────────────────

def run_merge_scenario(init_speed=8.0, target_speed=15.0, merge_x=50.0,
                        merge_lane=1, duration=15.0):
    """匝道汇入仿真

    自车在加速车道（起始 y=-5.25m = lane 3），
    需要在 merge_x 之前加速到 target_speed 并汇入主路。

    输入：
      init_speed: 初始速度 m/s（匝道速度）
      target_speed: 目标速度 m/s（主路速度）
      merge_x: 汇入点 x 位置 m
      merge_lane: 汇入目标车道索引
      duration: 仿真时长 s

    输出：
      ScenarioResult
    """
    start_y = lane_center_y(3)  # 加速车道
    target_y = lane_center_y(merge_lane)

    ego = VehicleState(x0=0.0, y0=start_y, v0=init_speed, heading0=0.0)
    result = ScenarioResult()

    planner = PlanningLayer()
    controller = ControlLayer(StanleyParams())
    lon_ctrl = LongitudinalController(target_speed=target_speed)

    n_steps = int(duration / DT)
    merge_started = False
    current_target = start_y

    for step in range(n_steps):
        t = step * DT
        s = ego.x

        # 决策：在汇入点附近开始变道汇入
        if not merge_started and s > merge_x - 20.0:
            merge_started = True
            current_target = target_y

        # 纵向控制（加速到目标速度）
        throttle, brake = lon_ctrl.compute(ego.v)

        # Planning → Control
        traj = planner.generate(ego.x, ego.y, ego.v, current_target, target_speed)
        controller.on_trajectory(traj)
        steer_cmd = controller.compute_steer(ego.x, ego.y, ego.v, ego.heading, ego.yaw_rate)

        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.throttle.append(throttle)
        result.brake.append(brake)

        ego.step(steer_cmd, throttle, brake)

        if abs(ego.y) > N_LANES * LANE_WIDTH / 2 + 1.0:
            result.off_road = True
            break

    # 评估
    speed_at_merge = result.v[min(len(result.v)-1, int((merge_x / max(ego.x, 1)) * len(result.v)))] if result.v else 0
    final_y_err = abs(ego.y - target_y)
    speed_at_end = result.v[-1] if result.v else 0

    result.success = (final_y_err < 1.0 and speed_at_end >= target_speed * 0.8
                      and not result.off_road)
    result.score = final_y_err * 0.5 + max(0, target_speed - speed_at_end) * 0.3
    result.summary = (f"merge(v0={init_speed:.0f}→{target_speed:.0f}m/s, merge_x={merge_x:.0f}m): "
                      f"speed_at_merge={speed_at_merge:.1f}m/s, "
                      f"final_y_err={final_y_err:.2f}m, "
                      f"final_speed={speed_at_end:.1f}m/s")
    return result


# ── 场景 6: 加塞处理（Cut-in） ──────────────────────────────

def run_cutin_scenario(init_speed=12.0, cutin_speed=10.0, cutin_x=30.0,
                        duration=12.0):
    """加塞仿真

    自车巡航中，右前方旁车突然向左变道切入本车道。
    自车需要减速让行，保持安全距离。

    输入：
      init_speed: 自车初始速度 m/s
      cutin_speed: 加塞车速度 m/s
      cutin_x: 加塞发生的 x 位置 m
      duration: 仿真时长 s

    输出：
      ScenarioResult
    """
    ego_y = lane_center_y(2)    # 自车在 lane 2 (y=-1.75)
    cutin_y0 = lane_center_y(3) # 加塞车从 lane 3 (y=-5.25) 切过来

    ego = VehicleState(x0=0.0, y0=ego_y, v0=init_speed, heading0=0.0)
    # 加塞车：初始在右侧车道，前方
    cutin = VehicleState(x0=cutin_x, y0=cutin_y0, v0=cutin_speed, heading0=0.0)
    result = ScenarioResult()

    planner = PlanningLayer()
    controller = ControlLayer(StanleyParams())
    lon_ctrl = LongitudinalController(target_speed=init_speed)

    n_steps = int(duration / DT)
    cutin_active = False
    cutin_lat_progress = 0.0
    cutin_duration = 2.0  # 加塞过程持续 2s
    prev_steer = 0.0

    for step in range(n_steps):
        t = step * DT

        # 加塞车运动：t=2s 时开始切过来
        if t >= 2.0 and not cutin_active:
            cutin_active = True
            cutin_lat_progress = 0.0

        if cutin_active:
            cutin_lat_progress += DT / cutin_duration
            if cutin_lat_progress > 1.0:
                cutin_lat_progress = 1.0
            # 横向插值：从 lane 3 到 lane 2
            cutin.y = cutin_y0 + (ego_y - cutin_y0) * cutin_lat_progress
            # 纵向：保持速度
            cutin.x += cutin.v * DT

        # 自车：检测加塞车是否在本车道前方
        same_lane = abs(ego.y - cutin.y) < 1.0
        lead_dist = cutin.x - ego.x if same_lane else None
        lead_v = cutin.v if same_lane else None

        # 纵向控制：加塞时让行
        if same_lane and lead_dist is not None and lead_dist < 25.0:
            throttle, brake = lon_ctrl.compute(ego.v, lead_dist, lead_v)
        else:
            throttle, brake = lon_ctrl.compute(ego.v)

        # 横向控制：直道保持
        traj = planner.generate(ego.x, ego.y, ego.v, ego_y, init_speed)
        controller.on_trajectory(traj)
        steer_cmd = controller.compute_steer(ego.x, ego.y, ego.v, ego.heading, ego.yaw_rate)
        prev_steer = steer_cmd

        result.t.append(t)
        result.x.append(ego.x)
        result.y.append(ego.y)
        result.v.append(ego.v)
        result.heading.append(ego.heading)
        result.steer.append(steer_cmd)
        result.throttle.append(throttle)
        result.brake.append(brake)

        ego.step(steer_cmd, throttle, brake)

        # 碰撞检测
        if same_lane and lead_dist is not None and lead_dist < 0:
            result.collision = True
            result.summary = f"COLLISION with cutin vehicle at t={t:.1f}s"
            break

        if abs(ego.y) > N_LANES * LANE_WIDTH / 2 + 1.0:
            result.off_road = True
            break

    # 评估
    if not result.collision:
        final_gap = cutin.x - ego.x if cutin_active else 999
        min_gap = 999
        for i in range(len(result.t)):
            if cutin_active:
                g = cutin.x - result.x[i]
                if g < min_gap: min_gap = g
        min_gap = max(min_gap, 0)

        result.success = (not result.collision and min_gap > 1.0 and not result.off_road)
        result.score = min_gap * 0.5
        result.summary = (f"cutin(v0={init_speed:.0f}m/s, cutin_v={cutin_speed:.0f}m/s): "
                          f"min_gap={min_gap:.1f}m, final_gap={final_gap:.1f}m, "
                          f"final_speed={result.v[-1]:.1f}m/s")
    return result


# ── 场景调参扫描 ─────────────────────────────────────────────

def tune_curve():
    """扫描曲线跟随最优参数"""
    print("\n" + "=" * 60)
    print("  曲线跟随参数扫描 (kappa=0.005, R=200m)")
    print("=" * 60)

    best_score = 1e9
    best = None
    n_valid = 0

    for kp in [0.3, 0.5, 0.8, 1.0, 1.2, 1.5]:
        for kd_h in [1.0, 2.0, 3.0, 4.0, 5.0]:
            for yd in [0.2, 0.28, 0.35, 0.5]:
                for speed in [8.0, 12.0, 15.0]:
                    p = StanleyParams()
                    p.lat_kp = kp
                    p.lat_kd_heading = kd_h
                    p.yaw_damping = yd
                    r = run_curve_scenario(kappa=0.005, target_speed=speed,
                                           duration=12.0, stanley_params=p)
                    if not r.success: continue
                    n_valid += 1
                    score = r.score
                    if score < best_score:
                        best_score = score
                        best = (kp, kd_h, yd, speed, r)

    print(f"  有效: {n_valid}")
    if best:
        kp, kd_h, yd, sp, r = best
        print(f"  最佳: kp={kp}, kd_heading={kd_h}, yaw_damp={yd} @ {sp:.0f}m/s")
        print(f"  score={best_score:.2f}, {r.summary}")


def tune_emergency():
    """扫描紧急制动参数"""
    print("\n" + "=" * 60)
    print("  紧急制动参数扫描")
    print("=" * 60)

    for v0 in [10.0, 15.0, 20.0]:
        for obs_d in [30.0, 50.0, 80.0]:
            r = run_emergency_brake_scenario(init_speed=v0, obstacle_dist=obs_d)
            status = "PASS" if r.success else "FAIL"
            print(f"  v0={v0:.0f}m/s, obs={obs_d:.0f}m → {status}: {r.summary}")


def tune_stop_go():
    """扫描跟停再起步参数"""
    print("\n" + "=" * 60)
    print("  跟停再起步参数扫描")
    print("=" * 60)
    for gap in [20.0, 30.0, 40.0]:
        for v0 in [8.0, 10.0, 12.0]:
            r = run_stop_go_scenario(init_speed=v0, initial_gap=gap)
            status = "PASS" if r.success else "FAIL"
            print(f"  gap={gap:.0f}m, v0={v0:.0f}m/s → {status}: {r.summary}")


def tune_merge():
    """扫描匝道汇入参数"""
    print("\n" + "=" * 60)
    print("  匝道汇入参数扫描")
    print("=" * 60)
    for init_v in [5.0, 8.0, 10.0]:
        for target_v in [12.0, 15.0, 18.0]:
            for merge_x in [30.0, 50.0, 80.0]:
                r = run_merge_scenario(init_speed=init_v, target_speed=target_v,
                                       merge_x=merge_x)
                status = "PASS" if r.success else "FAIL"
                print(f"  v0={init_v:.0f}→{target_v:.0f}m/s, merge_x={merge_x:.0f}m → {status}: {r.summary}")


def tune_cutin():
    """扫描加塞处理参数"""
    print("\n" + "=" * 60)
    print("  加塞处理参数扫描")
    print("=" * 60)
    for init_v in [10.0, 12.0, 15.0]:
        for cutin_v in [8.0, 10.0, 12.0]:
            for cutin_x in [20.0, 30.0, 50.0]:
                r = run_cutin_scenario(init_speed=init_v, cutin_speed=cutin_v,
                                       cutin_x=cutin_x)
                status = "PASS" if r.success else "FAIL"
                print(f"  v0={init_v:.0f}m/s, cutin_v={cutin_v:.0f}m/s, cutin_x={cutin_x:.0f}m → {status}: {r.summary}")


# ══════════════════════════════════════════════════════════════
#  场景打印辅助
# ══════════════════════════════════════════════════════════════

def print_scene_result(result, label):
    """打印场景结果"""
    print(f"\n  {'='*50}")
    status_icon = "PASS" if result.success else "FAIL"
    print(f"  [{status_icon}] {label}")
    print(f"  {'='*50}")
    if result.collision:
        print(f"  ❌ 碰撞!")
    elif result.off_road:
        print(f"  ❌ 冲出路面!")
    print(f"  {result.summary}")
    result.print_trajectory(stride=100)


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
    parser.add_argument("--uturn", action="store_true", help="三把方向掉头仿真（纯 Python 验证）")
    parser.add_argument("--tune-uturn", action="store_true", help="扫描三把方向掉头最优参数")
    parser.add_argument("--csv", type=str, default=None, help="输出CSV文件路径")

    # ── 6 种上路操作场景 ──
    parser.add_argument("--scene-curve", action="store_true", help="场景1: 曲线跟随（弯道保持）")
    parser.add_argument("--scene-emergency", action="store_true", help="场景2: 紧急制动")
    parser.add_argument("--scene-stop-go", action="store_true", help="场景3: 跟停再起步")
    parser.add_argument("--scene-obstacle", action="store_true", help="场景4: 障碍物避让")
    parser.add_argument("--scene-merge", action="store_true", help="场景5: 匝道汇入")
    parser.add_argument("--scene-cutin", action="store_true", help="场景6: 加塞处理")
    parser.add_argument("--run-all", action="store_true", help="全量执行所有6个场景")
    # 场景调参
    parser.add_argument("--tune-curve", action="store_true", help="扫描曲线跟随最优参数")
    parser.add_argument("--tune-emergency", action="store_true", help="扫描紧急制动参数")
    parser.add_argument("--tune-stop-go", action="store_true", help="扫描跟停再起步参数")
    parser.add_argument("--tune-merge", action="store_true", help="扫描匝道汇入参数")
    parser.add_argument("--tune-cutin", action="store_true", help="扫描加塞处理参数")
    args = parser.parse_args()

    CRUISE_SPEED = args.speed

    # ── 场景调参入口 ──
    if args.tune_curve:
        tune_curve()
        return
    if args.tune_emergency:
        tune_emergency()
        return
    if args.tune_stop_go:
        tune_stop_go()
        return
    if args.tune_merge:
        tune_merge()
        return
    if args.tune_cutin:
        tune_cutin()
        return

    # ── 全量场景执行 ──
    if args.run_all:
        print("\n" + "=" * 60)
        print("  场景全集 — 6 种上路操作全量验证")
        print("=" * 60)
        results = []

        print("\n  ── 场景 1: 曲线跟随 ──")
        r1 = run_curve_scenario(kappa=0.005, target_speed=12.0, duration=15.0)
        print_scene_result(r1, "曲线跟随 (kappa=0.005, R=200m, 12m/s)")
        results.append(("curve", r1))

        print("\n  ── 场景 2: 紧急制动 ──")
        r2 = run_emergency_brake_scenario(init_speed=15.0, obstacle_dist=50.0)
        print_scene_result(r2, "紧急制动 (v0=15m/s, 障碍物=50m)")
        results.append(("emergency", r2))

        print("\n  ── 场景 3: 跟停再起步 ──")
        r3 = run_stop_go_scenario(init_speed=10.0, initial_gap=30.0, duration=20.0)
        print_scene_result(r3, "跟停再起步 (v0=10m/s, gap=30m)")
        results.append(("stop_go", r3))

        print("\n  ── 场景 4: 障碍物避让 ──")
        r4 = run_obstacle_avoid_scenario(init_speed=12.0, obs_x=60.0)
        print_scene_result(r4, "障碍物避让 (v0=12m/s, obs_x=60m)")
        results.append(("obstacle", r4))

        print("\n  ── 场景 5: 匝道汇入 ──")
        r5 = run_merge_scenario(init_speed=8.0, target_speed=15.0, merge_x=50.0)
        print_scene_result(r5, "匝道汇入 (8→15m/s, merge_x=50m)")
        results.append(("merge", r5))

        print("\n  ── 场景 6: 加塞处理 ──")
        r6 = run_cutin_scenario(init_speed=12.0, cutin_speed=10.0, cutin_x=30.0)
        print_scene_result(r6, "加塞处理 (v0=12m/s, cutin_v=10m/s, cutin_x=30m)")
        results.append(("cutin", r6))

        # 汇总
        pass_count = sum(1 for _, r in results if r.success)
        fail_count = sum(1 for _, r in results if not r.success)
        print(f"\n  {'='*50}")
        print(f"  场景全集汇总: {pass_count}/{len(results)} PASS, {fail_count}/{len(results)} FAIL")
        print(f"  {'='*50}")
        for name, r in results:
            icon = "PASS" if r.success else "FAIL"
            print(f"    [{icon}] {name}: {r.summary}")
        return

    # ── 单场景执行 ──
    if args.scene_curve:
        r = run_curve_scenario(kappa=0.005, target_speed=args.speed, duration=args.duration)
        print_scene_result(r, f"曲线跟随 @ {args.speed:.0f}m/s")
        return
    if args.scene_emergency:
        r = run_emergency_brake_scenario(init_speed=args.speed, obstacle_dist=50.0, duration=args.duration)
        print_scene_result(r, f"紧急制动 @ {args.speed:.0f}m/s")
        return
    if args.scene_stop_go:
        r = run_stop_go_scenario(init_speed=min(args.speed, 10.0), initial_gap=30.0, duration=args.duration)
        print_scene_result(r, f"跟停再起步 @ {args.speed:.0f}m/s")
        return
    if args.scene_obstacle:
        r = run_obstacle_avoid_scenario(init_speed=args.speed)
        print_scene_result(r, f"障碍物避让 @ {args.speed:.0f}m/s")
        return
    if args.scene_merge:
        r = run_merge_scenario(init_speed=min(args.speed, 8.0), target_speed=args.speed)
        print_scene_result(r, f"匝道汇入 → {args.speed:.0f}m/s")
        return
    if args.scene_cutin:
        r = run_cutin_scenario(init_speed=args.speed)
        print_scene_result(r, f"加塞处理 @ {args.speed:.0f}m/s")
        return

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

    if args.tune_uturn:
        tune_uturn()
        tune_uturn_three_point()
        tune_uturn_multi()
        return

    if args.uturn:
        # 默认多把方向掉头（角点约束退出，2026-08-04 新增），去程+返程双跑
        p = UturnParams(mode='multi')
        r = run_uturn_simulation(p, start_lane_y=-1.75, start_heading=0.0)
        print_uturn_result(r, p, "多把方向掉头·去程（y=-1.75, h=0 → +1.75, h=π）",
                           start_lane_y=-1.75, start_heading=0.0)
        r2 = run_uturn_simulation(p, start_lane_y=+1.75, start_heading=math.pi)
        print_uturn_result(r2, p, "多把方向掉头·返程（y=+1.75, h=π → -1.75, h=0）",
                           start_lane_y=+1.75, start_heading=math.pi)
        if args.csv:
            with open(args.csv, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'v', 'heading', 'steer'])
                for i in range(len(r.t)):
                    w.writerow([f"{r.t[i]:.3f}", f"{r.x[i]:.3f}", f"{r.y[i]:.3f}",
                                f"{r.v[i]:.2f}", f"{r.heading[i]:.4f}", f"{r.steer[i]:.4f}"])
            print(f"\n  CSV written to: {args.csv}")
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
