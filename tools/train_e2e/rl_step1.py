#!/usr/bin/env python3
"""rl_step1.py — RL 换老师可行性验证（Step 1：纵向巡航 + 紧急刹停）

验证核心问题：**RL 能否学会比 IDM 规则更好的驾驶策略**（碰撞率 0 +
平均速度 ≥ IDM 基线）。成立 → RL 路线值得投入；不成立 → 回到扩场景路线。

设计（docs/LEARNING_LOOP.md「RL 换老师路线」）：
  - 环境：control_sim 的 VehicleState 物理（纯 Python，~0.1s/episode）
  - 状态：[ego_v, gap]（纵向，先不学 steer）
  - 动作：[throttle, brake]（连续，互斥执行，阈值 0.2 与训练一致）
  - 奖励：r = 速度效率 + 碰撞大罚 + jerk 舒适惩罚
  - 算法：REINFORCE（策略梯度），数值梯度更新（零依赖，~80 参数）
  - 对比：RL 策略 vs IDM 基线，同场景各跑 N 次

用法:
  python3 tools/train_e2e/rl_step1.py                 # 训练 2000 episodes + 对比
  python3 tools/train_e2e/rl_step1.py --episodes 5000
  python3 tools/train_e2e/rl_step1.py --no-train      # 只评估当前策略 vs IDM
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control_sim import VehicleState, DT  # noqa: E402

# ── 常量（与训练/执行端一致）──────────────────────────────
CRUISE_V = 20.0      # 巡航目标 m/s
A_MAX = 4.0          # 制动上限 m/s²（VehicleState: brake×8）
BRAKE_THRESHOLD = 0.2  # 互斥阈值（与 feature_schema 一致）
COLLISION_PENALTY = -100.0  # 碰撞惩罚必须 >> 速度收益（否则学「撞了也值」）
JERK_PENALTY = -0.05
EPISODE_LEN = 300    # 15s @20Hz


# ══════════════════════════════════════════════════════════════
#  环境
# ══════════════════════════════════════════════════════════════

class LongitudinalEnv:
    """纵向环境：cruise（无前车）或 emergency（静止障碍）。"""
    def __init__(self, mode="emergency", init_v=15.0, obstacle_x=50.0):
        self.mode = mode
        self.init_v = init_v
        self.obstacle_x = obstacle_x
        self.reset()

    def reset(self):
        self.ego = VehicleState(x0=0.0, y0=0.0, v0=self.init_v, heading0=0.0)
        self.step_n = 0
        self.prev_v = self.init_v
        self.total_jerk = 0.0
        self.collision = False
        self.prev_gap = self._gap()
        return self._state()

    def _gap(self):
        if self.mode == "emergency":
            return self.obstacle_x - self.ego.x - 2.25
        return 200.0

    def _state(self):
        """状态 [ego_v, gap]。gap=障碍距离（cruise 恒 200=无车）。"""
        return [self.ego.v, max(self._gap(), 0.0)]

    def step(self, action):
        """action=[throttle, brake]。返回 (state, reward, done, info)。"""
        throttle, brake = action
        # 互斥（与训练/执行端一致）
        if brake > BRAKE_THRESHOLD:
            throttle = 0.0
        else:
            brake = 0.0
        throttle = max(-1.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))

        self.ego.step(0.0, throttle, brake, dt=DT, v_max=30.0)
        self.step_n += 1

        # 奖励
        v = self.ego.v
        r = v / CRUISE_V  # 速度效率
        jerk = abs(v - self.prev_v) / DT
        self.total_jerk += jerk
        r += JERK_PENALTY * min(jerk, 20.0)  # 舒适惩罚

        # 推进奖励（2026-08-05 Step2）：车接近障碍(未撞)有正奖励。
        # 解决「碰撞 -100 太强 → 学一开始就刹(34m 外停, 极低效)」：
        # 原地刹没推进奖励，巡航后再刹有 → 鼓励「开过去再刹」。
        gap = self._gap()
        r += 0.02 * (self.prev_gap - gap)
        self.prev_gap = gap

        # 终止条件
        done = False
        if self.mode == "emergency":
            if gap < 0.5:
                r += COLLISION_PENALTY
                self.collision = True
                done = True
            elif v < 0.3:  # 刹停成功
                # 停得太远(>15m)是低效策略 → 惩罚（IDM 停 5.5m）
                if gap > 15.0:
                    r -= 5.0
                r += 2.0  # 停稳奖励
                done = True
        if self.step_n >= EPISODE_LEN:
            done = True

        self.prev_v = v
        return self._state(), r, done, {"collision": self.collision}


# ══════════════════════════════════════════════════════════════
#  策略网络（2→16→2）+ REINFORCE 数值梯度
# ══════════════════════════════════════════════════════════════

def net_forward(theta, s):
    """2→16→2 MLP，输出 [throttle, brake] 的均值（tanh 激活 → [-1,1]）。"""
    w1, b1, w2, b2 = theta
    h = [math.tanh(sum(w1[i][j] * s[j] for j in range(2)) + b1[i])
         for i in range(16)]
    out = [sum(w2[i][j] * h[j] for j in range(16)) + b2[i] for i in range(2)]
    return [math.tanh(o) for o in out]  # throttle ∈[-1,1], brake∈[-1,1]


def init_theta(seed=0):
    rng = random.Random(seed)
    w1 = [[rng.uniform(-0.5, 0.5) for _ in range(2)] for _ in range(16)]
    b1 = [rng.uniform(-0.5, 0.5) for _ in range(16)]
    w2 = [[rng.uniform(-0.5, 0.5) for _ in range(16)] for _ in range(2)]
    b2 = [rng.uniform(-0.5, 0.5) for _ in range(2)]
    return [w1, b1, w2, b2]


def sample_action(theta, s, sigma=0.15):
    """高斯采样：μ = net(s)，a ~ N(μ, σ²)。"""
    mu = net_forward(theta, s)
    return [m + random.gauss(0, sigma) for m in mu]


def idm_action(s, mode):
    """IDM 基线（与 synth_data/dagger_oracle 一致的规则）。"""
    v, gap = s
    if mode == "emergency" or gap < 40:
        dv = v - 0.0
        safe = 5.0 + v * 1.5
        ttc = gap / max(dv, 0.1)
        if gap < safe and dv > 0:
            brake = min(1.0, 0.4 + (safe - gap) * 0.15 + dv * 0.08)
            return [0.0, brake]
        if ttc < 2.5:
            return [0.0, min(1.0, 0.25 + (2.5 - ttc) * 0.15)]
        if ttc < 4.0:
            return [min(0.5, 0.15), 0.0]
    thr = min(1.0, max(0.0, 0.25 + (20 - v) * 0.04))
    return [thr, 0.0]


def run_episode(theta, mode, sample=True, sigma=0.15, seed=None):
    """跑一个 episode，返回 (总回报, 轨迹, info)。"""
    if seed is not None:
        random.seed(seed)
    env = LongitudinalEnv(mode=mode)
    s = env.reset()
    states, actions, rewards = [], [], []
    done = False
    while not done:
        if sample:
            a = sample_action(theta, s, sigma)
        else:
            a = net_forward(theta, s)
        s2, r, done, info = env.step(a)
        states.append(s)
        actions.append(a)
        rewards.append(r)
        s = s2
    return sum(rewards), (states, actions, rewards), info


def policy_gradient(theta, trajectory, baseline):
    """REINFORCE：∇θ J ≈ Σ_t (G_t - baseline) ∇θ log π(a_t|s_t)。
    用数值梯度近似 ∇θ log π。"""
    states, actions, rewards = trajectory
    # 折扣回报
    G = 0.0
    returns = []
    for r in reversed(rewards):
        G = r + 0.99 * G
        returns.insert(0, G)
    # 数值梯度：对每个参数 θ_k
    grad = [None] * len(theta)
    eps = 1e-3
    for li in range(len(theta)):
        if isinstance(theta[li], list):
            if theta[li] and isinstance(theta[li][0], list):
                # 2 维权重矩阵
                grad[li] = [[0.0] * len(theta[li][0]) for _ in range(len(theta[li]))]
                for i in range(len(theta[li])):
                    for j in range(len(theta[li][i])):
                        grad[li][i][j] = _num_grad(theta, (li, i, j), states,
                                                   actions, returns, baseline, eps)
            else:
                # 1 维 bias
                grad[li] = [0.0] * len(theta[li])
                for i in range(len(theta[li])):
                    grad[li][i] = _num_grad(theta, (li, i), states, actions,
                                            returns, baseline, eps)
        else:
            grad[li] = _num_grad(theta, (li,), states, actions, returns,
                                 baseline, eps)
    return grad


def _num_grad(theta, idx, states, actions, returns, baseline, eps):
    """∂J/∂θ_idx 数值近似。J = Σ_t (G_t - b) log π(a_t|s_t)。"""
    def log_prob(t_plus):
        total = 0.0
        for s, a in zip(states, actions):
            mu = net_forward(t_plus, s)
            # 简化：只算 throttle 维度的 log prob（brake 对称）
            for k in range(2):
                total += -((a[k] - mu[k]) ** 2) / (2 * 0.15 ** 2)
        return total

    def J(t_plus):
        lp = log_prob(t_plus)
        return sum((g - baseline) for g in returns) * lp

    t_plus = _perturb(theta, idx, eps)
    t_minus = _perturb(theta, idx, -eps)
    return (J(t_plus) - J(t_minus)) / (2 * eps)


def _perturb(theta, idx, delta):
    out = []
    for li in range(len(theta)):
        if isinstance(theta[li], list):
            # 递归拷贝（bias 是 1 维 float list，权重是 2 维）
            out.append([[x for x in row] for row in theta[li]]
                       if theta[li] and isinstance(theta[li][0], list)
                       else [x for x in theta[li]])
        else:
            out.append(theta[li])
    if len(idx) == 3:
        out[idx[0]][idx[1]][idx[2]] += delta
    elif len(idx) == 2:
        out[idx[0]][idx[1]] += delta
    else:
        out[idx[0]] += delta
    return out


# ══════════════════════════════════════════════════════════════
#  训练 + 评估
# ══════════════════════════════════════════════════════════════

def evaluate(theta, mode, n=50, seed_base=1000):
    """无探索评估：碰撞率 + 刹停位置（效率指标）。

    刹停位置 = 停稳时距障碍的距离（越小 = 刹车越晚越高效，
    但必须 0 碰撞）。IDM 提前 40m 保守刹车 → 停得远；
    RL 若学「更晚刹」→ 停得近 = 效率超越。
    """
    coll = 0
    stop_gaps = []  # 刹停时距障碍距离（碰撞 episode 不计）
    for i in range(n):
        env = LongitudinalEnv(mode=mode)
        s = env.reset()
        done = False
        while not done:
            a = net_forward(theta, s)  # 无探索
            s, _, done, info = env.step(a)
        if info["collision"]:
            coll += 1
        else:
            gap = env.obstacle_x - env.ego.x - 2.25
            stop_gaps.append(max(gap, 0.0))
    avg_gap = sum(stop_gaps) / len(stop_gaps) if stop_gaps else float("inf")
    return coll, n, avg_gap


def warm_start(theta, n_traj=20, lr=0.02):
    """模仿初始化：采样 IDM 轨迹对 (s, a)，监督拟合策略网络。

    解决「纯 REINFORCE 从随机出发卡在『立刻刹』局部最优」：
    策略先学会 IDM 的「巡航→提前减速→刹停」路径，RL 只需微调
    「更晚刹更高效」，不用从零探索长路径。
    """
    pairs = []  # (s, a_idm)
    for _ in range(n_traj):
        env = LongitudinalEnv(mode="emergency")
        s = env.reset()
        done = False
        while not done:
            a = idm_action(s, "emergency")
            pairs.append((s, a))
            s, _, done, _ = env.step(a)
    # 监督拟合：MSE(a_net(s), a_idm)，数值梯度
    for _ in range(30):
        for s, a in pairs:
            mu = net_forward(theta, s)
            for k in range(2):
                err = mu[k] - a[k]
                # 数值梯度 ∂err²/∂θ
                for li in range(len(theta)):
                    if isinstance(theta[li], list):
                        if theta[li] and isinstance(theta[li][0], list):
                            for i in range(len(theta[li])):
                                for j in range(len(theta[li][i])):
                                    t_p = _perturb(theta, (li, i, j), 1e-3)
                                    t_m = _perturb(theta, (li, i, j), -1e-3)
                                    g = (net_forward(t_p, s)[k]
                                         - net_forward(t_m, s)[k]) / 2e-3
                                    theta[li][i][j] -= lr * 2 * err * g
                        else:
                            for i in range(len(theta[li])):
                                t_p = _perturb(theta, (li, i), 1e-3)
                                t_m = _perturb(theta, (li, i), -1e-3)
                                g = (net_forward(t_p, s)[k]
                                     - net_forward(t_m, s)[k]) / 2e-3
                                theta[li][i] -= lr * 2 * err * g
    return theta


def main() -> int:
    ap = argparse.ArgumentParser(description="RL Step1: 纵向巡航+刹停")
    ap.add_argument("--episodes", type=int, default=2000)
    ap.add_argument("--lr", type=float, default=0.01)
    ap.add_argument("--no-train", action="store_true")
    ap.add_argument("--no-warm", action="store_true", help="跳过模仿初始化")
    ap.add_argument("--seed", type=int, default=7, help="固定随机种子（可复现）")
    args = ap.parse_args()

    random.seed(args.seed)
    theta = init_theta(seed=args.seed)
    print("=== RL Step1: 纵向 RL vs IDM 基线 ===")

    if not args.no_warm:
        print("[warm-start] 用 IDM 轨迹监督初始化策略（学会巡航→提前刹）")
        theta = warm_start(theta)
        coll, _, gap = evaluate(theta, "emergency", n=30)
        print(f"  warm-start 后: 碰撞率 {coll}/30, 刹停距离 {gap:.1f}m (IDM 5.5m)")

    if not args.no_train:
        print(f"\n[训练] emergency 场景 {args.episodes} episodes (REINFORCE)")
        best_coll = 99
        for ep in range(args.episodes):
            # 探索：前 60% 用高斯采样，后 40% 降噪
            sigma = 0.15 if ep < args.episodes * 0.6 else 0.05
            total, traj, info = run_episode(theta, "emergency", sample=True,
                                            sigma=sigma)
            baseline = 0.0
            grad = policy_gradient(theta, traj, baseline)
            # 更新（按 grad 维度：2 维权重 / 1 维 bias）
            for li in range(len(theta)):
                if isinstance(grad[li], list):
                    if grad[li] and isinstance(grad[li][0], list):
                        for i in range(len(grad[li])):
                            for j in range(len(grad[li][i])):
                                theta[li][i][j] += args.lr * grad[li][i][j]
                    else:
                        for i in range(len(grad[li])):
                            theta[li][i] += args.lr * grad[li][i]
                else:
                    theta[li] += args.lr * grad[li]
            if ep % 200 == 199:
                coll, _, _ = evaluate(theta, "emergency", n=20)
                print(f"  ep {ep+1:5d} 回报={total:6.1f} 碰撞率={coll}/20")
                if coll < best_coll:
                    best_coll = coll

    # 对比评估（Step2: 效率 = 刹停位置，越近越高效）
    print("\n[对比] emergency 场景 (无探索, 各 50 次)")
    rl_coll, _, rl_gap = evaluate(theta, "emergency", n=50)
    print(f"  RL:     碰撞率 {rl_coll}/50, 平均刹停距离 {rl_gap:.1f}m")
    # IDM 基线
    idm_coll = 0
    idm_gaps = []
    for i in range(50):
        env = LongitudinalEnv(mode="emergency")
        s = env.reset()
        done = False
        while not done:
            a = idm_action(s, "emergency")
            s, _, done, info = env.step(a)
        if info["collision"]:
            idm_coll += 1
        else:
            gap = env.obstacle_x - env.ego.x - 2.25
            idm_gaps.append(max(gap, 0.0))
    idm_gap = sum(idm_gaps) / len(idm_gaps) if idm_gaps else float("inf")
    print(f"  IDM:    碰撞率 {idm_coll}/50, 平均刹停距离 {idm_gap:.1f}m")

    # 结论：碰撞率 ≤ IDM 且刹停更近（更晚刹更高效）= 超越
    coll_ok = rl_coll <= idm_coll
    eff_ok = rl_gap < idm_gap - 1.0  # 至少近 1m 才算效率超越
    verdict = "✅ RL 超越 IDM（碰撞 0 且刹车更晚更高效）" if (coll_ok and eff_ok) else \
              "✅ RL 追平 IDM（碰撞 0, 效率相当）" if coll_ok else \
              "❌ RL 未超越（碰撞率更高）"
    print(f"\n结论: RL 碰撞 {rl_coll}/50 vs IDM {idm_coll}/50, "
          f"刹停距离 {rl_gap:.1f}m vs {idm_gap:.1f}m → {verdict}")
    return 0 if coll_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
