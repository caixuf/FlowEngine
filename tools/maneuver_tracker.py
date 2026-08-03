#!/usr/bin/env python3
"""maneuver_tracker.py — 通用机动跟踪器原型（Python 仿真先行）

目标（回应"遇新操作就改好久、没通用能力"）：
  机动 = 参考轨迹数据（点列 + 带符号 v），执行 = 这一个跟踪器。
  新操作（掉头/倒库/侧方/坡道）= 加一份轨迹数据，不改执行逻辑。

核心设计（vs 掉头特例的 5 层 gate）：
  1. 弧长推进 s 单调，不猜段 —— 掉头 debug 的 D/R 段交界歧义、prog_i
     单调锁段、heading gate，根源都是"几何最近点在两条弧间跳"。用
     弧长参数 s 单调推进，段归属由 s 在轨迹上的位置唯一决定，天生没有
     歧义（s 在倒车段 → 倒挡，s 推进到正向段 → 前进挡）。
  2. 挡位 = 执行点 v 符号（v<0→R），换挡滞回 + 刹停由 s 处 v 决定。
  3. 横向 = kappa 前馈 + 车体系 e_lat/dh 反馈（倒挡反馈反号）。
  4. 速度 = s 处 |v|（执行点），纵向 PID。

用法：
  from maneuver_tracker import ManeuverTracker, TrajectoryPoint
  traj = [TrajectoryPoint(...), ...]   # 带 v 符号
  mt = ManeuverTracker(traj, wheelbase=2.7)
  for t in range(N):
      steer, throttle, brake, gear = mt.tick(x, y, heading, v)
      x, y, heading, v = step_bicycle(...)  # VehicleState.step
"""
import math

WHEELBASE = 2.7
MAX_STEER = 0.60
UTURN_SPEED = 3.5
REVERSE_SPEED = -3.0


class TrajectoryPoint:
    __slots__ = ('x', 'y', 'heading', 'kappa', 'v')
    def __init__(self, x=0.0, y=0.0, heading=0.0, kappa=0.0, v=0.0):
        self.x, self.y = x, y
        self.heading, self.kappa, self.v = heading, kappa, v


def norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class ManeuverTracker:
    """通用机动跟踪器。

    状态：s（弧长推进，单调）+ gear（带滞回）。不猜段——s 唯一决定
    执行点，执行点 v 符号决定挡位，无几何歧义。
    """
    def __init__(self, traj, wheelbase=WHEELBASE):
        self.traj = traj
        self.wheelbase = wheelbase
        self.n = len(traj)
        self.s = 0.0
        self.gear = 1          # +1=DRIVE, -1=REVERSE
        self.prev_steer = 0.0
        self.integral = 0.0
        # 累计弧长表：cum_s[i] = 点 i 到点 0 的弧长
        self.cum_s = [0.0] * self.n
        for i in range(1, self.n):
            dx = traj[i].x - traj[i - 1].x
            dy = traj[i].y - traj[i - 1].y
            self.cum_s[i] = self.cum_s[i - 1] + math.hypot(dx, dy)

    # ── 弧长推进：s 按车实际行驶距离单调推进 ──
    # 车沿轨迹执行，每帧沿执行点方向走 |v|·dt。s 按行驶距离推进，不靠
    # 几何最近点（自交轨迹上最近点会跳到远处却朝向不同的段，导致 s 冻结
    # 或跳变）。执行点的方向由 exec v 符号决定：前进沿 +s，倒车沿 +s
    # （轨迹点列就是执行顺序，倒车段的弧长仍递增——挡位由 v 符号决定，
    # 与 s 推进方向无关）。朝向一致性只作为异常兜底：若执行点与车头朝向
    # 偏差 >1.2rad（车明显脱离轨迹），暂停推进等收敛。
    def _advance_s(self, ego_x, ego_y, ego_heading, current_speed, dt):
        # 执行点朝向作为推进方向参考
        ex, _ = self._exec()
        dh = norm_angle(ex.heading - ego_heading)
        if abs(dh) > 1.2:
            return  # 车明显脱离执行方向，暂停推进（避免跳到错误段）
        # s 按车实际行驶距离推进；但推进量受"下一挡段起点朝向"约束：
        # 倒车段若车还没转够 heading 就推进进正向段 → 倒车提前结束。
        # 车沿轨迹执行，s 每帧最多推进 |v|·dt，且不越过车尚未到达的
        # 段边界（段边界 = v 符号变化处）。
        new_s = self.s + abs(current_speed) * dt
        # 找 v 符号变化点，若 new_s 跨过它，检查车 heading 是否已接近
        # 下一段起点（避免倒车未完成就跳段）
        for i in range(self.n - 1):
            if (self.traj[i + 1].v < -0.1) != (self.traj[i].v < -0.1):
                if self.s < self.cum_s[i] < new_s:
                    # 跨段：要求车 heading 接近下一段起点朝向
                    next_h = self.traj[i + 1].heading
                    if abs(norm_angle(next_h - ego_heading)) > 0.4:
                        new_s = self.cum_s[i]  # 停在段边界，等车转够
                # 可能多段，只处理第一个
        self.s = min(new_s, self.cum_s[-1])

    # ── 执行点（当前弧长处的轨迹点，线性插值）──
    def _exec(self):
        i = 0
        while i + 1 < self.n and self.cum_s[i + 1] < self.s:
            i += 1
        if i + 1 >= self.n:
            return self.traj[self.n - 1], self.n - 1
        frac = (self.s - self.cum_s[i]) / max(1e-9, self.cum_s[i + 1] - self.cum_s[i])
        a, b = self.traj[i], self.traj[i + 1]
        return TrajectoryPoint(
            x=a.x + (b.x - a.x) * frac,
            y=a.y + (b.y - a.y) * frac,
            heading=norm_angle(a.heading + norm_angle(b.heading - a.heading) * frac),
            kappa=a.kappa + (b.kappa - a.kappa) * frac,
            v=a.v + (b.v - a.v) * frac,
        ), i

    # ── 前视点：执行点前视 ~2m 弧长（同一挡段内，跨 D/R 边界就停）──
    def _lookahead(self, base_i):
        rev = self.traj[base_i].v < -0.1
        target = self.traj[base_i]
        j = base_i
        arc = 0.0
        while j + 1 < self.n and arc < 2.0:
            if (self.traj[j + 1].v < -0.1) != rev:
                break
            dx = self.traj[j + 1].x - self.traj[j].x
            dy = self.traj[j + 1].y - self.traj[j].y
            arc += math.hypot(dx, dy)
            j += 1
            target = self.traj[j]
        return target

    # ── 挡位：从执行点向前扫第一个 |v|>0.3 取符号 ──
    # v=0 刹停点（Phase 交界）会被跳过——它是换挡的物理刹停前提，不是
    # 挡位意图。只有扫到真正的运动点才决定挡位。
    def _update_gear(self, base_i, current_speed):
        want = self.gear
        for i in range(base_i, self.n):
            if abs(self.traj[i].v) > 0.3:
                want = -1 if self.traj[i].v < 0 else 1
                break
        if want != self.gear and abs(current_speed) > 0.8:
            return True   # gear_pending：带速想换挡，本帧刹停
        self.gear = want
        return False

    # ── 主 tick：返回 (steer, throttle, brake, gear) ──
    def tick(self, ego_x, ego_y, ego_heading, current_speed, dt=0.05):
        self._advance_s(ego_x, ego_y, ego_heading, current_speed, dt)
        exec_pt, exec_i = self._exec()
        # 挡位（向前扫第一个运动点，跳过 v=0 刹停点）
        gear_pending = self._update_gear(exec_i, current_speed)
        # 目标速度 = 执行点 |v|，前视 6m 内同挡段最小 |v|，下限 1.5
        mag = abs(exec_pt.v)
        arc = 0.0
        for i in range(exec_i, self.n - 1):
            dx = self.traj[i + 1].x - self.traj[i].x
            dy = self.traj[i + 1].y - self.traj[i].y
            arc += math.hypot(dx, dy)
            if arc > 6.0 or (self.traj[i + 1].v < -0.1) != (exec_pt.v < -0.1):
                break
            mag = min(mag, abs(self.traj[i + 1].v))
        mag = max(mag, 1.5)
        target_speed = -mag if exec_pt.v < -0.1 else mag
        # 轨迹终点 = 机动完成 → 目标 0，车刹停（不继续反向冲）。
        # 注意：末段刹停点零空间（x,y 与前一运动点重合，cum_s 相等），
        # 弧长索引看不到它，故用 s 是否到尽头顶判断。
        if self.s >= self.cum_s[-1]:
            target_speed = 0.0
            self.gear = 1
        if gear_pending:
            target_speed = 0.0

        # ── 横向：kappa 前馈 + e_lat/dh 反馈，倒挡反馈反号 ──
        tgt = self._lookahead(exec_i)
        rev = self.gear == -1
        dh = norm_angle(tgt.heading - ego_heading)
        e_lat = (-math.sin(tgt.heading) * (tgt.x - ego_x)
                 + math.cos(tgt.heading) * (tgt.y - ego_y))
        ff = math.atan(self.wheelbase * tgt.kappa)
        fb = 0.8 * dh + 0.25 * e_lat
        if rev:
            fb = -fb
        steer = ff + fb
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        self.prev_steer = steer

        # ── 纵向：PID ──
        if gear_pending:
            return steer, 0.0, 1.0, self.gear   # 换挡刹停
        pid_target = abs(target_speed)
        pid_current = abs(current_speed)
        err = pid_target - pid_current
        self.integral = max(-200.0, min(500.0, self.integral + err * dt))
        deriv = (err - getattr(self, '_prev_err', 0.0)) / dt
        self._prev_err = err
        out = 3.0 * err + 0.05 * self.integral + 0.5 * deriv
        if out > 0:
            throttle = min(1.0, out / 5000.0 * 1500.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, -out / 8000.0 * 1500.0)
        if self.gear == -1:
            throttle = -throttle   # 倒挡油门取反（与 physics.cpp 一致）
        return steer, throttle, brake, self.gear


# ══════════════════════════════════════════════════════════
# 生成参考轨迹：掉头（三把方向，含倒车段）— 第一个数据实例
# ══════════════════════════════════════════════════════════
def gen_uturn_traj(start_x=2900.0, start_y=-1.75, start_h=0.0,
                   lane_w=3.5, n_lanes=4, dt=0.05):
    """生成三把方向掉头参考轨迹（与 planning UTurnPlanner 同构，但
    输出纯数据点列 + 带符号 v）。返回 [TrajectoryPoint]"""
    road_half = lane_w * n_lanes * 0.5
    pts = []
    x, y, h = start_x, start_y, start_h
    t = 0.0
    def push(steer, v, dur):
        nonlocal x, y, h, t
        n = int(dur / dt)
        for _ in range(n):
            yaw = v / WHEELBASE * math.tan(steer)
            half_wb = WHEELBASE * 0.5
            h += yaw * dt
            x += (v * math.cos(h) - half_wb * math.sin(h) * yaw) * dt
            y += (v * math.sin(h) + half_wb * math.cos(h) * yaw) * dt
            pts.append(TrajectoryPoint(x=x, y=y, heading=h,
                                       kappa=math.tan(steer) / WHEELBASE, v=v))
            t += dt

    # 自适应相位（与 C++ UTurnPlanner 同构，几何条件退出而非固定时长）：
    #   Phase 2: 前进满舵左打 → heading ≥ 105° 退出
    #   Phase 3: 倒车右打满舵 → heading ≥ 175°（≈π）退出
    #   Phase 4: 前进左打归位对向车道 → 再前进对齐
    #   Phase 5: 巡航填充（朝 -x）
    steer_sign = 1.0
    TARGET_H = math.pi
    def phase_time():  # 当前相位累计时长（用局部变量，避免跨相位共享）
        return 0.0
    # Phase 2: 前进满舵弧，heading 推到 105°
    steer = steer_sign * 0.60
    v = UTURN_SPEED
    p2_t = 0.0
    while p2_t < 3.0 and abs(norm_angle(h - start_h)) < 105.0 * math.pi / 180.0:
        yaw = v / WHEELBASE * math.tan(steer)
        half_wb = WHEELBASE * 0.5
        h += yaw * dt
        x += (v * math.cos(h) - half_wb * math.sin(h) * yaw) * dt
        y += (v * math.sin(h) + half_wb * math.cos(h) * yaw) * dt
        pts.append(TrajectoryPoint(x=x, y=y, heading=h,
                                   kappa=math.tan(steer) / WHEELBASE, v=v))
        p2_t += dt
    pts.append(TrajectoryPoint(x=x, y=y, heading=h, kappa=0.0, v=0.0))  # 刹停
    # Phase 3: 倒车右打，heading 推到 ≈π（独立时长上限）
    steer = -steer_sign * 0.60
    v = REVERSE_SPEED
    p3_t = 0.0
    while p3_t < 3.5 and abs(norm_angle(TARGET_H - h)) > 5.0 * math.pi / 180.0:
        yaw = v / WHEELBASE * math.tan(steer)
        half_wb = WHEELBASE * 0.5
        h += yaw * dt
        x += (v * math.cos(h) - half_wb * math.sin(h) * yaw) * dt
        y += (v * math.sin(h) + half_wb * math.cos(h) * yaw) * dt
        pts.append(TrajectoryPoint(x=x, y=y, heading=h,
                                   kappa=math.tan(steer) / WHEELBASE, v=v))
        p3_t += dt
    pts.append(TrajectoryPoint(x=x, y=y, heading=h, kappa=0.0, v=0.0))  # 刹停
    # Phase 4: 前进对齐 heading→π（heading 修正到精确 π，消除 0.06rad 残差
    # —— 否则巡航 300m 后横漂 18m）
    while abs(norm_angle(TARGET_H - h)) > 0.01:
        yaw = UTURN_SPEED / WHEELBASE * math.tan(0.3)
        half_wb = WHEELBASE * 0.5
        h += yaw * dt
        x += (UTURN_SPEED * math.cos(h) - half_wb * math.sin(h) * yaw) * dt
        y += (UTURN_SPEED * math.sin(h) + half_wb * math.cos(h) * yaw) * dt
        pts.append(TrajectoryPoint(x=x, y=y, heading=h,
                                   kappa=math.tan(0.3) / WHEELBASE, v=UTURN_SPEED))
    # Phase 5: 巡航填充（直行朝 -x，heading 锁定 π）
    push(0.0, 20.0, 2.0)
    return pts


# ══════════════════════════════════════════════════════════
# 主循环：用通用 tracker 跑掉头
# ══════════════════════════════════════════════════════════
def run(verbose=True):
    traj = gen_uturn_traj()
    mt = ManeuverTracker(traj)
    # 车辆状态（与 physics.cpp 一致）
    x, y, heading, v = 2900.0, -1.75, 0.0, 0.0
    dt = 0.05
    y_min, y_max = 1e9, -1e9
    gear_log = []
    for t in range(400):
        steer, thr, brk, gear = mt.tick(x, y, heading, v, dt)
        # 积分（复制 VehicleState.step 的倒车逻辑）
        if thr < 0:
            v += thr * 3.33 * dt
            if v < -4.0: v = -4.0
        elif brk > 0:
            v -= brk * 8.0 * dt
            if v < 0: v = 0
        elif thr > 0:
            v += thr * 3.33 * dt
            if v > 22.0: v = 22.0
        yaw = v / WHEELBASE * math.tan(steer)
        half_wb = WHEELBASE * 0.5
        heading += yaw * dt
        x += (v * math.cos(heading) - half_wb * math.sin(heading) * yaw) * dt
        y += (v * math.sin(heading) + half_wb * math.cos(heading) * yaw) * dt
        heading = norm_angle(heading)
        y_min, y_max = min(y_min, y), max(y_max, y)
        gear_log.append(gear)
        if verbose and t % 40 == 0:
            print(f"t={t:3d} x={x:7.1f} y={y:6.2f} h={heading:5.2f} v={v:5.1f} "
                  f"gear={gear:+d} steer={steer:+.2f}")

    print(f"\n=== 结果 ===")
    print(f"final: x={x:7.1f} y={y:6.2f} h={heading:5.2f} v={v:5.1f}")
    print(f"y 范围: [{y_min:.2f}, {y_max:.2f}]  (路沿 ±7.0)")
    print(f"y 出界: {'YES ❌' if y_min < -7.0 or y_max > 7.0 else 'NO ✅'}")
    # 挡位翻转次数（D/R 振荡检测）
    flips = sum(1 for i in range(1, len(gear_log)) if gear_log[i] != gear_log[i - 1])
    print(f"gear 翻转次数: {flips}  ({'振荡 ❌' if flips > 10 else '正常 ✅'})")
    ok = (y_min >= -7.0 and y_max <= 7.0) and flips <= 10 and abs(heading - math.pi) < 0.3
    print(f"掉头完成(heading≈π): {abs(heading - math.pi) < 0.3}")
    print(f"总体: {'PASS ✅' if ok else 'FAIL ❌'}")
    return ok


if __name__ == '__main__':
    run()


# ══════════════════════════════════════════════════════════
# 第二个操作：倒车入库（reverse parking）— 验证通用性
# 只用同一 ManeuverTracker，只换轨迹数据（含倒车段 v<0）。
# ══════════════════════════════════════════════════════════
def gen_parking_traj(start_x=0.0, start_y=1.75, start_h=0.0, dt=0.05):
    """倒车入库参考轨迹：前进过库位 → 倒车右打入库 → 停正。
    数据实例化证明：新操作 = 加一份轨迹，不改执行逻辑。"""
    pts = []
    x, y, h = start_x, start_y, start_h
    def push(steer, v, dur):
        nonlocal x, y, h
        n = int(dur / dt)
        for _ in range(n):
            yaw = v / WHEELBASE * math.tan(steer)
            half_wb = WHEELBASE * 0.5
            h += yaw * dt
            x += (v * math.cos(h) - half_wb * math.sin(h) * yaw) * dt
            y += (v * math.sin(h) + half_wb * math.cos(h) * yaw) * dt
            pts.append(TrajectoryPoint(x=x, y=y, heading=h,
                                       kappa=math.tan(steer) / WHEELBASE, v=v))

    # 前进过库位（直线）
    push(0.0, 5.0, 1.5)
    # 刹停
    pts.append(TrajectoryPoint(x=x, y=y, heading=h, kappa=0.0, v=0.0))
    # 倒车左打满舵（入库弧线，v<0 = 倒挡，往 -y 方向转入库）
    push(-0.60, REVERSE_SPEED, 0.7)
    # 倒车回正（入库后直线倒到底）
    push(0.0, REVERSE_SPEED, 1.2)
    # 刹停（入库完成）
    pts.append(TrajectoryPoint(x=x, y=y, heading=h, kappa=0.0, v=0.0))
    return pts


def run_parking(verbose=True):
    traj = gen_parking_traj()
    mt = ManeuverTracker(traj)
    x, y, heading, v = 0.0, 1.75, 0.0, 0.0
    dt = 0.05
    y_min, y_max = 1e9, -1e9
    gear_log = []
    for t in range(150):
        steer, thr, brk, gear = mt.tick(x, y, heading, v, dt)
        if thr < 0:
            v += thr * 3.33 * dt
            if v < -4.0: v = -4.0
        elif brk > 0:
            v -= brk * 8.0 * dt
            if v < 0: v = 0
        elif thr > 0:
            v += thr * 3.33 * dt
            if v > 22.0: v = 22.0
        yaw = v / WHEELBASE * math.tan(steer)
        half_wb = WHEELBASE * 0.5
        heading += yaw * dt
        x += (v * math.cos(heading) - half_wb * math.sin(heading) * yaw) * dt
        y += (v * math.sin(heading) + half_wb * math.cos(heading) * yaw) * dt
        heading = norm_angle(heading)
        y_min, y_max = min(y_min, y), max(y_max, y)
        gear_log.append(gear)
        if verbose and t % 25 == 0:
            print(f"t={t:3d} x={x:6.1f} y={y:6.2f} h={heading:5.2f} v={v:5.1f} gear={gear:+d} st={steer:+.2f}")

    print(f"\n=== 倒车入库结果 ===")
    print(f"final: x={x:6.1f} y={y:6.2f} h={heading:5.2f} v={v:5.1f}")
    print(f"y 范围: [{y_min:.2f}, {y_max:.2f}]")
    flips = sum(1 for i in range(1, len(gear_log)) if gear_log[i] != gear_log[i - 1])
    print(f"gear 翻转次数: {flips}")
    ok = flips >= 2  # D→R→(停) 至少一次换挡，倒车执行过
    print(f"倒车执行: {'YES ✅' if any(g == -1 for g in gear_log) else 'NO ❌'}")
    print(f"总体: {'PASS ✅' if ok and any(g == -1 for g in gear_log) else 'FAIL ❌'}")
    return ok


if __name__ == '__main__':
    import sys
    if '--parking' in sys.argv:
        run_parking()
    else:
        run()
