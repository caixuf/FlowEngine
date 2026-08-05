#!/usr/bin/env python3
"""rl_ppo.py — PPO 版 RL（Step2 效率超越，替代朴素 REINFORCE）

rl_step1.py 的结论：REINFORCE 高方差 + 数值梯度，卡「立刻刹」局部最优
（34.1m 外停，无法学「更晚刹更高效」）。PPO 用裁剪目标 + GAE 优势 +
价值函数，步长稳定，能微调出「巡航→接近→晚刹」策略。

复用 rl_step1 的：LongitudinalEnv / IDM 基线 / 刹停位置评估 / warm-start。

依赖：torch（训练侧已装，GPU 可用）。纯 Python 数值梯度跑不动 PPO
（多 epoch 反向传播）。

用法:
  python3 tools/train_e2e/rl_ppo.py --episodes 3000        # 训练+对比
  python3 tools/train_e2e/rl_ppo.py --no-train             # 只评估当前策略
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rl_step1 import (  # noqa: E402
    LongitudinalEnv, idm_action, evaluate as evaluate_numpy,
)

import torch  # noqa: E402
import torch.nn as nn  # noqa: E402
import torch.optim as optim  # noqa: E402

# PPO 超参
LR = 1e-3
GAMMA = 0.99
GAE_LAMBDA = 0.95
CLIP_EPS = 0.2
K_EPOCHS = 4          # 每次更新内 epoch
BATCH_EPISODES = 128  # 每次更新收集的 episode 数
COLLISION_PENALTY = -100.0


# ══════════════════════════════════════════════════════════════
#  策略网络：Actor(μ) + Critic(V)
# ══════════════════════════════════════════════════════════════

class ActorCritic(nn.Module):
    """状态 [ego_v, gap] → 动作均值 [throttle, brake] + 价值 V。"""
    def __init__(self):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(2, 32), nn.Tanh(),
            nn.Linear(32, 32), nn.Tanh(),
        )
        self.mu = nn.Linear(32, 2)      # 动作均值
        self.v = nn.Linear(32, 1)       # 价值
        self.log_sigma = nn.Parameter(torch.zeros(2))  # 动作方差(可学)

    def forward(self, s):
        h = self.shared(s)
        mu = torch.tanh(self.mu(h))     # throttle/brake ∈ [-1,1]
        v = self.v(h).squeeze(-1)
        return mu, v

    def sample(self, s):
        """采样动作 + log_prob + 价值（用于轨迹收集）。"""
        mu, v = self(s)
        sigma = self.log_sigma.exp().clamp(0.05, 0.5)
        dist = torch.distributions.Normal(mu, sigma)
        a = dist.sample().clamp(-1.0, 1.0)
        return a, dist.log_prob(a).sum(-1), v


# ══════════════════════════════════════════════════════════════
#  PPO 训练
# ══════════════════════════════════════════════════════════════

def collect_episodes(model, n_episodes, mode="emergency", seed=0):
    """收集 n 个 episode 的轨迹（state/action/logp/reward/done/value）。"""
    rng = random.Random(seed)
    S, A, LOGP, R, D, V = [], [], [], [], [], []
    for _ in range(n_episodes):
        env = LongitudinalEnv(mode=mode)
        s = env.reset()
        done = False
        ep_s, ep_a, ep_lp, ep_r, ep_d, ep_v = [], [], [], [], [], []
        while not done:
            s_t = torch.tensor([s], dtype=torch.float32)
            a, lp, v = model.sample(s_t)
            s2, r, done, _ = env.step(a.detach().numpy()[0])
            ep_s.append(s_t)
            ep_a.append(a.detach())
            ep_lp.append(lp.detach())
            ep_r.append(r)
            ep_d.append(done)
            ep_v.append(v.detach())
            s = s2
        S.extend(ep_s); A.extend(ep_a); LOGP.extend(ep_lp)
        R.extend(ep_r); D.extend(ep_d); V.extend(ep_v)
    return S, A, LOGP, R, D, V


def compute_gae(R, D, V, gamma=GAMMA, lam=GAE_LAMBDA):
    """GAE 优势估计。"""
    adv = []
    gae = 0.0
    for t in reversed(range(len(R))):
        next_v = 0.0 if D[t] else V[t + 1] if t + 1 < len(V) else 0.0
        delta = R[t] + gamma * next_v - V[t]
        gae = delta + gamma * lam * (0.0 if D[t] else gae)
        adv.insert(0, gae)
    return torch.tensor(adv, dtype=torch.float32)


def ppo_update(model, opt, S, A, LOGP, ADV, V_old, clip_eps=CLIP_EPS,
               k_epochs=K_EPOCHS, ent_coef=0.01):
    """PPO 裁剪目标更新（多 epoch 小步长）。

    ent_coef: 熵奖励权重（2026-08-05 可调——后期漂移根因之一：
    熵奖励 0.01 恒定，训练后期还在探索 → 策略从最优滑开）。
    """
    S = torch.cat(S)
    A = torch.cat(A)
    LOGP = torch.cat(LOGP)
    V_old = torch.cat(V_old)
    ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)  # 归一化优势

    for _ in range(k_epochs):
        mu, v = model(S)
        sigma = model.log_sigma.exp().clamp(0.05, 0.5)
        dist = torch.distributions.Normal(mu, sigma)
        logp_new = dist.log_prob(A).sum(-1)

        ratio = (logp_new - LOGP).exp()
        adv = ADV.detach()
        surr1 = ratio * adv
        surr2 = torch.clamp(ratio, 1 - clip_eps, 1 + clip_eps) * adv
        actor_loss = -torch.min(surr1, surr2).mean()

        # 价值损失（裁剪价值目标，标准 PPO）
        v_clip = V_old + (v - V_old).clamp(-clip_eps, clip_eps)
        v_loss = torch.max((v - ADV.detach() - V_old) ** 2,
                           (v_clip - ADV.detach() - V_old) ** 2).mean()

        # 熵奖励(鼓励探索，权重可调/衰减)
        entropy = dist.entropy().mean()

        loss = actor_loss + 0.5 * v_loss - ent_coef * entropy
        opt.zero_grad()
        loss.backward()
        opt.step()


# ══════════════════════════════════════════════════════════════
#  评估（复用 rl_step1 的刹停位置指标）
# ══════════════════════════════════════════════════════════════

def evaluate_torch(model, n=50, seed_base=2000):
    """无探索评估：碰撞率 + 平均刹停距离。"""
    coll = 0
    gaps = []
    for i in range(n):
        env = LongitudinalEnv(mode="emergency")
        s = env.reset()
        done = False
        while not done:
            s_t = torch.tensor([s], dtype=torch.float32)
            mu, _ = model(s_t)
            a = mu.detach().numpy()[0]
            s, _, done, info = env.step(a)
        if info["collision"]:
            coll += 1
        else:
            gaps.append(max(env.obstacle_x - env.ego.x - 2.25, 0.0))
    avg = sum(gaps) / len(gaps) if gaps else float("inf")
    return coll, n, avg


# ══════════════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════════════

def main() -> int:
    ap = argparse.ArgumentParser(description="PPO 纵向 RL（Step2 效率超越）")
    ap.add_argument("--episodes", type=int, default=3000)
    ap.add_argument("--no-train", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    random.seed(args.seed)
    model = ActorCritic()
    opt = optim.Adam(model.parameters(), lr=LR)
    print("=== PPO: 纵向 RL vs IDM（目标: 刹停距离 < IDM 5.5m）===")

    if not args.no_train:
        n_updates = max(1, args.episodes // BATCH_EPISODES)
        print(f"[train] {args.episodes} episodes / {n_updates} updates "
              f"(batch={BATCH_EPISODES})")
        # 2026-08-05 稳定性调参：
        # ① lr 线性衰减(后期小步长, 防破坏已学策略)
        # ② 熵权重衰减(前期 0.01 探索 → 后期 0.001 收敛, 防漂移)
        # ③ 早停最优: 每 upd 评估, 保存「碰撞 0 且刹停最近」权重
        best_coll, best_gap = 99, 1e9
        for upd in range(n_updates):
            frac = upd / max(n_updates - 1, 1)
            lr_now = LR * (1.0 - 0.9 * frac)          # lr 衰减 10 倍
            ent_now = 0.01 - 0.009 * frac             # 熵 0.01→0.001
            for pg in opt.param_groups:
                pg["lr"] = lr_now
            S, A, LOGP, R, D, V = collect_episodes(
                model, BATCH_EPISODES, seed=args.seed + upd)
            ADV = compute_gae(R, D, V)
            ppo_update(model, opt, S, A, LOGP, ADV, V, ent_coef=ent_now)
            if upd % 5 == 0 or upd == n_updates - 1:
                coll, _, gap = evaluate_torch(model, n=20, seed_base=3000 + upd)
                print(f"  upd {upd+1:4d}/{n_updates}: 碰撞率 {coll}/20, "
                      f"刹停距离 {gap:.1f}m (lr={lr_now:.5f} ent={ent_now:.4f})")
                # 早停最优：碰撞 0 且刹停更近（效率优先）
                if coll == 0 and gap < best_gap:
                    best_coll, best_gap = coll, gap
                    torch.save(model.state_dict(), "/tmp/rl_ppo_best.pt")
                    print(f"    ★ 新最优: 刹停 {gap:.1f}m → /tmp/rl_ppo_best.pt")
        # 载入最优权重（防后期漂移）
        import os
        if os.path.exists("/tmp/rl_ppo_best.pt"):
            model.load_state_dict(torch.load("/tmp/rl_ppo_best.pt"))
            print(f"  载入最优权重 (刹停 {best_gap:.1f}m, 碰撞 {best_coll}/20)")

    # 对比
    print("\n[对比] emergency 场景 (无探索, 各 50 次)")
    rl_coll, _, rl_gap = evaluate_torch(model, n=50)
    print(f"  RL(PPO): 碰撞率 {rl_coll}/50, 刹停距离 {rl_gap:.1f}m")
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
            idm_gaps.append(max(env.obstacle_x - env.ego.x - 2.25, 0.0))
    idm_gap = sum(idm_gaps) / len(idm_gaps) if idm_gaps else float("inf")
    print(f"  IDM:    碰撞率 {idm_coll}/50, 刹停距离 {idm_gap:.1f}m")

    coll_ok = rl_coll <= idm_coll
    eff_ok = rl_gap < idm_gap - 1.0
    verdict = ("✅ PPO 超越 IDM（碰撞 0 且刹车更晚更高效）"
               if (coll_ok and eff_ok) else
               "✅ PPO 追平 IDM" if coll_ok else "❌ PPO 未超越")
    print(f"\n结论: 碰撞 {rl_coll}/50 vs {idm_coll}/50, "
          f"刹停 {rl_gap:.1f}m vs {idm_gap:.1f}m → {verdict}")
    return 0 if coll_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
