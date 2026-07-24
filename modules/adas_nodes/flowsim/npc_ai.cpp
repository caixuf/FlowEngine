/**
 * npc_ai.cpp — NPC AI 实现
 *
 * IDM 跟车模型（简化版，来自设计文档 §4.2）：
 *   safe_gap = base + v * time
 *   gap_error = gap - safe_gap
 *   if gap_error > 0:  v_desired = min(v + accel_rate*dt, target_v)
 *   else:              v_desired = max(0, v - brake*exp(-gap_error/2)*dt)
 *
 * 横向（位置）：NPC 不再用世界系直线积分（那样道路一拐弯车就飞出路网），
 * 改为沿 **中央 route** 的累计 s 推进，每步 frenet_to_world 反算世界坐标，
 * 严格贴道路几何行驶；到 route 末端回收到 ego 附近形成持续车流。
 * esmini/route 缺失时退回旧的直线积分，保证不退化（见 step_npc_vehicle 第 5 步）。
 *
 * 前车搜索：同车道（route 模式看横向 offset + 方向；旧模式看 lane_id/Δy），
 * 前方（route 模式沿 route_s，旧模式沿 Δx）、最近。
 */

#include "npc_ai.h"
#include "physics.h"
#include "road_network.h"
#include "route.h"

#include <algorithm>
#include <cmath>

namespace flowsim {

/* ── 判断两车是否同车道 ── */
static bool same_lane(const Entity& a, const Entity& b, const NpcAiConfig& cfg) {
    // route 模式：同方向 + 横向偏移接近（沿车道行驶，offset 就是车道横向位置）
    if (a.route_dir != 0 && b.route_dir != 0) {
        return a.route_dir == b.route_dir &&
               std::fabs(a.offset - b.offset) < cfg.same_lane_tol;
    }
    // 旧格式：road_id/lane_id 严格匹配
    if (a.road_id > 0 && b.road_id > 0 && a.lane_id != 0 && b.lane_id != 0) {
        return a.road_id == b.road_id && a.lane_id == b.lane_id;
    }
    // 更旧：横向距离 < 容差
    return std::fabs(a.y - b.y) < cfg.same_lane_tol;
}

/* ── 找同车道最近前车 ── */
static EntityId find_lead(const Entity& npc, const EntityPool& pool,
                          const NpcAiConfig& cfg) {
    EntityId   best     = INVALID_ENTITY;
    double     best_gap = 1e9;
    const bool on_route = (npc.route_dir != 0);
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& o = pool[i];
        if (!o.active || &o == &npc) continue;
        if (!o.is_vehicle()) continue;
        if (!same_lane(npc, o, cfg)) continue;

        // 前车相对本车的纵向前方距离（>0 表示在前）
        double ahead;
        if (on_route && o.route_dir != 0) {
            // 沿 route 纵向：顺行看 route_s 更大者，对向(dir=-1)看更小者
            ahead = (o.route_s - npc.route_s) * (double)npc.route_dir;
        } else {
            ahead = o.x - npc.x;  // 旧世界系：沿 x
        }
        if (ahead <= 0) continue;
        if (ahead > cfg.look_ahead) continue;

        double gap = ahead - (o.length * 0.5 + npc.length * 0.5);
        if (gap < best_gap) {
            best_gap = gap;
            best = i;
        }
    }
    return best;
}

/* ── IDM 期望速度 ── */
static double idm_desired_speed(double v, double gap, double target_v,
                                const NpcAiConfig& cfg, double dt) {
    double safe_gap = cfg.idm_safe_gap_base + v * cfg.idm_safe_gap_time;
    double gap_error = gap - safe_gap;
    if (gap_error > 0) {
        // 间距充足：平稳加速到 target_v
        return std::min(v + cfg.accel_rate * dt, target_v);
    }
    // 间距不足：减速，gap_error 越负刹车越猛
    double brake = cfg.follow_decel_factor * std::exp(-gap_error / 2.0);
    return std::max(0.0, v - brake * dt);
}

/* ── MOBIL 辅助：在指定车道找前车（leader） ──
 * 返回实体索引，未找到返回 INVALID_ENTITY。 */
static EntityId find_leader_in_lane(const Entity& npc, double lane_offset,
                                    const EntityPool& pool, const NpcAiConfig& cfg,
                                    double& out_gap) {
    EntityId best = INVALID_ENTITY;
    double best_gap = 1e9;
    out_gap = 1e9;
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& o = pool[i];
        if (!o.active || &o == &npc) continue;
        if (!o.is_vehicle()) continue;
        if (o.route_dir != npc.route_dir) continue;
        if (std::fabs(o.offset - lane_offset) >= cfg.same_lane_tol) continue;
        double ahead = (o.route_s - npc.route_s) * (double)npc.route_dir;
        if (ahead <= 0) continue;
        double gap = ahead - (o.length * 0.5 + npc.length * 0.5);
        if (gap < best_gap && gap < cfg.look_ahead) {
            best_gap = gap;
            best = i;
        }
    }
    if (best != INVALID_ENTITY) out_gap = best_gap;
    return best;
}

/* ── MOBIL 辅助：在指定车道找后车（follower） ──
 * 返回实体索引，未找到返回 INVALID_ENTITY。 */
static EntityId find_follower_in_lane(const Entity& npc, double lane_offset,
                                      const EntityPool& pool, const NpcAiConfig& cfg,
                                      double& out_gap) {
    EntityId best = INVALID_ENTITY;
    double best_gap = 1e9;
    out_gap = 1e9;
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& o = pool[i];
        if (!o.active || &o == &npc) continue;
        if (!o.is_vehicle()) continue;
        if (o.route_dir != npc.route_dir) continue;
        if (std::fabs(o.offset - lane_offset) >= cfg.same_lane_tol) continue;
        double behind = (npc.route_s - o.route_s) * (double)npc.route_dir;
        if (behind <= 0) continue;
        double gap = behind - (o.length * 0.5 + npc.length * 0.5);
        if (gap < best_gap && gap < cfg.mobil_back_look) {
            best_gap = gap;
            best = i;
        }
    }
    if (best != INVALID_ENTITY) out_gap = best_gap;
    return best;
}

/* ── MOBIL 辅助：计算 IDM 加速度（给定前车和间距） ──
 * 无前车时返回 target_vx 对应的自由巡航加速度。 */
static double mobil_idm_accel(const Entity& vehicle, double gap, double target_v,
                              const Entity* lead, const NpcAiConfig& cfg) {
    double v = vehicle.speed;
    if (lead) {
        double safe_gap = cfg.idm_safe_gap_base + v * cfg.idm_safe_gap_time;
        double gap_err = gap - safe_gap;
        if (gap_err > 0) {
            return std::min(cfg.accel_rate, (target_v - v) / 0.5);  // 巡航加速
        }
        return -cfg.follow_decel_factor * std::exp(-gap_err / 2.0);
    }
    // 自由巡航：向 target_v 加速
    double dv = target_v - v;
    if (dv > 0) return std::min(cfg.accel_rate, dv / 0.5);
    return 0.0;
}

/* ── MOBIL 变道代价函数 ──
 * gain = a'_c - a_c + p * (a'_n - a_n + a'_o - a_o)
 * 返回值 > mobil_gain_threshold 表示变道有益。
 * 安全约束：a'_n > -mobil_safe_brake（新跟随者不会被迫急刹）。
 *
 * @param npc         变道主体
 * @param target_offset 目标车道横向位置
 * @param pool        实体池
 * @param cfg         AI 配置
 * @param out_safety  [out] 安全约束是否满足
 * @return MOBIL gain 值 */
static double mobil_gain(const Entity& npc, double target_offset,
                         const EntityPool& pool, const NpcAiConfig& cfg,
                         bool& out_safety) {
    out_safety = true;

    // ── 当前车道：前车 + 后车 ──
    double cur_leader_gap, cur_follower_gap;
    EntityId cur_leader = find_leader_in_lane(npc, npc.offset, pool, cfg, cur_leader_gap);
    EntityId cur_follower = find_follower_in_lane(npc, npc.offset, pool, cfg, cur_follower_gap);

    const Entity* cur_lead_ptr = (cur_leader != INVALID_ENTITY) ? &pool[cur_leader] : nullptr;
    double a_c = mobil_idm_accel(npc, cur_leader_gap, npc.target_vx, cur_lead_ptr, cfg);

    // ── 目标车道：前车 + 后车 ──
    double tgt_leader_gap, tgt_follower_gap;
    EntityId tgt_leader = find_leader_in_lane(npc, target_offset, pool, cfg, tgt_leader_gap);
    EntityId tgt_follower = find_follower_in_lane(npc, target_offset, pool, cfg, tgt_follower_gap);

    const Entity* tgt_lead_ptr = (tgt_leader != INVALID_ENTITY) ? &pool[tgt_leader] : nullptr;
    double a_c_prime = mobil_idm_accel(npc, tgt_leader_gap, npc.target_vx, tgt_lead_ptr, cfg);

    // ── 安全约束：新跟随者（目标车道后车）不会被迫急刹 ──
    double a_n = 0.0, a_n_prime = 0.0;
    if (tgt_follower != INVALID_ENTITY) {
        const Entity& nf = pool[tgt_follower];
        // 变道前：新跟随者跟它原来的前车
        double nf_old_gap;
        EntityId nf_old_leader = find_leader_in_lane(nf, nf.offset, pool, cfg, nf_old_gap);
        const Entity* nf_old_ptr = (nf_old_leader != INVALID_ENTITY) ? &pool[nf_old_leader] : nullptr;
        a_n = mobil_idm_accel(nf, nf_old_gap, nf.target_vx, nf_old_ptr, cfg);
        // 变道后：新跟随者跟 npc（npc 插入到它前面）
        double new_gap = tgt_follower_gap;
        a_n_prime = mobil_idm_accel(nf, new_gap, nf.target_vx, &npc, cfg);
        // 安全约束
        if (a_n_prime < -cfg.mobil_safe_brake) out_safety = false;
    }

    // ── 旧跟随者（当前车道后车）──
    double a_o = 0.0, a_o_prime = 0.0;
    if (cur_follower != INVALID_ENTITY) {
        const Entity& of = pool[cur_follower];
        // 变道前：旧跟随者跟 npc
        a_o = mobil_idm_accel(of, cur_follower_gap, of.target_vx, &npc, cfg);
        // 变道后：旧跟随者跟 npc 原来的前车
        const Entity* of_new_ptr = cur_lead_ptr;  // npc 离开后，of 跟原来的前车
        double of_new_gap = 1e9;
        if (cur_lead_ptr) {
            // 旧跟随者到原前车的间距 = cur_follower_gap + cur_leader_gap + npc.length
            of_new_gap = cur_follower_gap + cur_leader_gap + npc.length;
        }
        a_o_prime = mobil_idm_accel(of, of_new_gap, of.target_vx, of_new_ptr, cfg);
    }

    double gain = (a_c_prime - a_c)
                + cfg.mobil_politeness * ((a_n_prime - a_n) + (a_o_prime - a_o));
    return gain;
}

/* ── 边界权限门：检查当前车道边界是否允许变道 ──
 * 判断逻辑：
 *   1. 中心线保护：双向道路参考线 (offset=0) 是对向分界（双黄/实线），严禁跨越。
 *      同向 NPC (route_dir>0) 只能在 offset<0 侧；对向 NPC (route_dir<0) 只能在 offset>0 侧。
 *   2. 路面范围：目标 offset 不得超出同向可行驶半宽。
 *   3. 跨度限制：单次变道最多跨 2 车道。
 * CutIn 状态机：跨实线变道，bypass 此检查。 */
static bool boundary_permissive(const Entity& npc, double target_offset,
                                FlowRoadNetwork* roads) {
    // CutIn 状态机：跨实线变道，直接放行
    if (npc.ai_state == AIState::CutIn) return true;
    if (!roads || !roads->loaded()) return true;  // 无路网时保守放行

    // 中心线保护：offset=0 是对向分界线，严禁跨越。留 0.5m 余量防越线。
    if (npc.route_dir > 0 && target_offset >= -0.5) return false;
    if (npc.route_dir < 0 && target_offset <=  0.5) return false;

    int total_lanes = roads->drivable_lane_count(npc.road_id, npc.s);
    if (total_lanes <= 0) return true;  // 无法查询 → 保守放行

    // 同向半宽估算：双向道路同向车道数 ≈ total_lanes/2；
    // total_lanes 为奇数（单向道路）时同向车道数 = total_lanes。
    int same_dir_lanes = total_lanes / 2;
    if (same_dir_lanes < 1) same_dir_lanes = total_lanes;
    double same_dir_half = same_dir_lanes * 3.5;

    // 目标 offset 必须在同向路面范围内（+1m 余量）
    if (npc.route_dir > 0) {
        if (target_offset < -same_dir_half - 1.0) return false;
    } else if (npc.route_dir < 0) {
        if (target_offset >  same_dir_half + 1.0) return false;
    }

    // 单次变道最多跨 2 车道
    if (std::fabs(target_offset - npc.offset) > 7.0) return false;

    return true;
}

/* ── 回收：跑到 route 末端的 NPC 放回 ego 附近，形成持续车流 ──
 * B4: 检查目标点附近是否已有同方向 NPC，被占则再后退一段，避免多车叠在同一点。
 * Phase 2: 若 npc.road_pos.ok() 且 roads 可用，回收后用 road_pos.init 重新定位
 * 到新 (road_id, s_local, offset)，使后续 step5 走 road_pos.advance 分支而非旧
 * route_s+frenet_to_world。 */
static void recycle_npc(Entity& npc, const Route& route, double ego_route_s,
                        const EntityPool& pool, FlowRoadNetwork* roads) {
    const double total = route.total_length();
    // 按 id 错开回收距离（50..138m），避免所有车叠在同一点
    const double back = 50.0 + (double)(npc.id % 5) * 22.0;
    double target;
    if (ego_route_s > 1.0) {
        target = (npc.route_dir > 0) ? (ego_route_s - back)   // 顺行：回 ego 后方
                                     : (ego_route_s + back);  // 对向：回 ego 前方(朝 ego 开来)
    } else {
        target = (npc.route_dir > 0) ? 0.0 : total;           // ego 未定位：回起/末端
    }
    // B4: 防叠车 — 目标点 8m 内若已有同方向 NPC，再后退 12m，最多重试 5 次。
    // 解决 ego_route_s=0 时所有 NPC 都被回收到 route 起点（target=0）的叠车问题。
    for (int attempt = 0; attempt < 5; ++attempt) {
        bool occupied = false;
        for (int i = 0; i < pool.size(); ++i) {
            const Entity& o = pool[i];
            if (!o.active || &o == &npc) continue;
            if (!o.is_npc_vehicle()) continue;
            if (o.route_dir != npc.route_dir) continue;
            if (std::fabs(o.route_s - target) < 8.0) { occupied = true; break; }
        }
        if (!occupied) break;
        target += (npc.route_dir > 0) ? -12.0 : 12.0;
    }
    if (target < 0.0)   target = 0.0;
    if (target > total) target = total;
    npc.route_s = target;
    // 重置动态状态：之前 recycle 只改 route_s，speed/ai_state/lead_id/crash_cooldown
    // 残留旧值——刚刹停的车回收后 speed=0 顶在新位置不动；刚撞车冻结的车回收后
    // crash_cooldown>0 继续冻结；follow 状态的车回收后还在追一辆已不存在的 lead。
    // 重置后 NPC 在新位置以 Cruise 状态、原 target_vx 起步。
    npc.speed = 0.0;
    npc.vx = 0.0; npc.vy = 0.0;
    npc.throttle = 0.0; npc.brake = 0.0;
    npc.ai_state = AIState::Cruise;
    npc.lateral_control = LateralControl::None;
    npc.lead_id = INVALID_ENTITY;
    npc.follow_gap = 1e9;
    npc.crash_cooldown = 0.0;
    npc.route_fail_count = 0;
    npc.lane_change_timer = 0.0;
    npc.target_offset = npc.offset;  /* recycle 后保持当前车道，不残留变道目标 */

    /* Phase 2: road_pos 重新 init 到回收点。
     * route.locate 把 route_s 转成 (road_id, s_local)，再用 npc.offset（保持
     * 横向车道）init road_pos。失败则 road_pos 失效，下一帧 step5 走旧 route 逻辑。
     * 注意：road_pos.init 内部会 RM_DeletePosition 旧 handle 再 RM_CreatePosition，
     * 不需要显式 destroy；失败时 RoadPosition::init 已 fprintf stderr 告警。 */
    if (npc.road_pos.ok() && roads && roads->loaded()) {
        int rid = 0, ridx = -1;
        double s_local = 0.0;
        route.locate(npc.route_s, rid, s_local, ridx);
        if (npc.road_pos.init(*roads, rid, 0, s_local, npc.offset)) {
            /* 立即同步世界坐标 — 旧实现只 init road_pos 不更新 npc.x/y，
             * 下一帧 road_pos.world() 才把 npc.x/y 跳到新位置，evaluator
             * 在两次采样间反算出 45 m/s 的"伪速度"触发 respawn jump 告警。
             * 这里在 recycle 当帧立即用 road_pos.world() 同步 x/y/heading，
             * 让 NPC 在新位置以 speed=0 出现，dx/dy 跨帧无跳变。 */
            WorldPos wp;
            if (npc.road_pos.world(wp)) {
                npc.x = wp.x;
                npc.y = wp.y;
                double h = wp.h + (npc.route_dir < 0 ? M_PI : 0.0);
                while (h >  M_PI) h -= 2.0 * M_PI;
                while (h < -M_PI) h += 2.0 * M_PI;
                npc.heading = h;
                /* vx/vy 已在上面清零，保持 0 即可（speed=0） */
            }
        }
    }
}

void step_npc_vehicle(Entity& npc, const EntityPool& pool,
                      double dt, const NpcAiConfig& cfg,
                      FlowRoadNetwork* roads, const Route* route,
                      double ego_route_s) {
    bool in_crash_cooldown = (npc.crash_cooldown > 0.0);
    if (in_crash_cooldown) {
        npc.crash_cooldown -= dt;
        if (npc.crash_cooldown < 0.0) npc.crash_cooldown = 0.0;
        npc.speed = 0.0;
        npc.throttle = 0.0;
        npc.brake = 1.0;
        /* E3: 不 return！crash_cooldown 期间仍执行 route 位置刷新（offset 平滑
         * + frenet_to_world），让被碰撞弹偏的车自动回到车道里。之前直接 return
         * 导致 crash_cooldown 2s 内车停在被推到的错误位置/朝向，冷却结束后
         * 在路外 world_to_frenet 失败 → 飞出路面。 */
    }

    // ── 横向控制仲裁 ──
    // NPC 横向由以下系统之一控制，优先级从高到低：
    //   Choreo/Script > CutIn > Mobil > None（E2 保持当前车道）
    // lateral_control 由写入 target_offset 的系统设置，CutIn 完成时清除。
    // 脚本/编舞 (lateral_control=Script/Choreo) 期间 MOBIL 跳过评估，
    // 防止两套系统互相覆盖。

    // ── E2: offset → target_offset 平滑插值（换道不瞬移，1.5s 完成 3.5m 变道）──
    // 变道速率 ≈ 2.3 m/s（3.5m / 1.5s），视觉上是平滑横移，不会突然跳。
    // 未在变道时 target_offset == offset，插值无变化。
    //
    // CutIn 状态机（ai_state==CutIn）走专属 PID 横向控制，跳过 E2 固定速率：
    //   u = Kp*e + Ki*∫e + Kd*de/dt   （e = target_offset - offset）
    //   offset += clamp(u, ±max_lateral_speed) * dt
    // PID 比 2.3 m/s 固定插值更"激进"且可控（超调后回拉），符合加塞场景；
    // 同时 bypass lane_change_safe（跨实线变道本就要"硬挤"）。
    if (!in_crash_cooldown && npc.ai_state == AIState::CutIn) {
        double err = npc.target_offset - npc.offset;
        npc.cutin_pid_integral += err * dt;
        // 积分项防饱和（误差大时积分不持续累积，避免过冲后回拉过度）
        double int_lim = cfg.cutin_max_lateral_speed / std::max(0.01, cfg.cutin_pid_ki);
        if (npc.cutin_pid_integral >  int_lim) npc.cutin_pid_integral =  int_lim;
        if (npc.cutin_pid_integral < -int_lim) npc.cutin_pid_integral = -int_lim;
        double deriv = (err - npc.cutin_pid_prev) / std::max(1e-4, dt);
        npc.cutin_pid_prev = err;
        double u = cfg.cutin_pid_kp * err
                 + cfg.cutin_pid_ki * npc.cutin_pid_integral
                 + cfg.cutin_pid_kd * deriv;
        // 横向速度限幅，防止初帧大误差引起突变
        if (u >  cfg.cutin_max_lateral_speed) u =  cfg.cutin_max_lateral_speed;
        if (u < -cfg.cutin_max_lateral_speed) u = -cfg.cutin_max_lateral_speed;
        double step = u * dt;
        // 防过冲：剩余距离不足一个 step 时直接收敛到目标
        double remain = npc.target_offset - npc.offset;
        if (std::fabs(step) > std::fabs(remain)) step = remain;
        npc.offset += step;
        /* 中心线硬 clamp（同 E2 分支）：CutIn 跨实线变道允许跨车道线，
         * 但严禁跨过道路中心线进入对向。 */
        if (npc.route_dir > 0 && npc.offset > -0.3) npc.offset = -0.3;
        if (npc.route_dir < 0 && npc.offset <  0.3) npc.offset =  0.3;
        npc.cutin_active = true;
        // 到达目标通道 → 完成，回 Cruise（清积分项防残留影响下次）
        // 释放横向控制权：lateral_control 恢复为 None，后续 MOBIL/IDM 可接管
        if (std::fabs(npc.target_offset - npc.offset) < cfg.cutin_completion_threshold) {
            npc.offset = npc.target_offset;
            npc.ai_state = AIState::Cruise;
            npc.lateral_control = LateralControl::None;
            npc.cutin_pid_integral = 0.0;
            npc.cutin_pid_prev = 0.0;
            npc.cutin_active = false;
        }
    } else if (!in_crash_cooldown && std::fabs(npc.offset - npc.target_offset) > 0.01) {
        double dir = (npc.target_offset > npc.offset) ? 1.0 : -1.0;
        double step = 2.3 * dt;  // ≈2.3 m/s 横移速率
        double remain = std::fabs(npc.target_offset - npc.offset);
        if (step > remain) step = remain;
        npc.offset += dir * step;
        /* 中心线硬 clamp：同向 NPC(route_dir>0) 的 offset 严禁跨过 0 进入对向。
         * 即使 target_offset 异常为正（cutin 时 ego.y>0、或 Frenet 同步错误），
         * 也强制卡在 -0.3m（留余量防越线）。对向 NPC(route_dir<0) 对称处理。
         * 这是防止"NPC 频繁向逆向车道变来变去"的最后防线。 */
        if (npc.route_dir > 0 && npc.offset > -0.3) npc.offset = -0.3;
        if (npc.route_dir < 0 && npc.offset <  0.3) npc.offset =  0.3;
    }

    // 1. 找前车（碰撞冷却期间跳过，保持 speed=0）
    EntityId lead = INVALID_ENTITY;
    double gap = 1e9;
    if (!in_crash_cooldown) {
        lead = find_lead(npc, pool, cfg);
        npc.lead_id = lead;
        if (lead != INVALID_ENTITY) {
            const Entity& lead_e = pool[lead];
            if (npc.route_dir != 0 && lead_e.route_dir != 0) {
                gap = (lead_e.route_s - npc.route_s) * (double)npc.route_dir
                      - (lead_e.length * 0.5 + npc.length * 0.5);
            } else {
                gap = (lead_e.x - npc.x) - (lead_e.length * 0.5 + npc.length * 0.5);
            }
            npc.follow_gap = gap;
        } else {
            npc.follow_gap = 1e9;
        }
    }

    // 1.5 MOBIL 变道决策：用代价函数评估变道收益 + 边界权限门 + 安全约束
    //    候选车道：当前 offset ± lane_width（3.5m），不超过路面半宽。
    //    每个候选评估：boundary_permissive(是否虚线可跨越) + mobil_gain(收益>阈值)
    //    + safety(新跟随者不会被迫急刹)。同时保留避障触发：前车太慢时主动评估。
    //    仲裁检查：lateral_control 不为 None（脚本/编舞/CutIn 活跃中）时跳过，
    //    防止 MOBIL 覆盖场景导演的横向指令。
    //    红绿灯/让行期间不评估变道（StopForTL/Yield 状态下保持当前车道）
    if (false) {
        // 仲裁门禁：存在更高优先级的横向控制时跳过
        if (npc.lateral_control == LateralControl::Script ||
            npc.lateral_control == LateralControl::Choreo ||
            npc.ai_state == AIState::CutIn) { goto mobil_done; }
        if (npc.lane_change_timer > 0.0) {
            npc.lane_change_timer -= dt;
            if (npc.lane_change_timer < 0.0) npc.lane_change_timer = 0.0;
        } else if (npc.route_dir > 0 && npc.speed > 2.0) {
            // 避障触发条件：前车太慢且间距不足
            bool blocked = false;
            if (lead != INVALID_ENTITY) {
                const Entity& lead_e = pool[lead];
                double trigger_gap = 10.0 + npc.speed * 1.5;
                if (gap < trigger_gap && lead_e.speed < npc.speed * 0.7) {
                    blocked = true;
                }
            }
            // 候选车道：右移（更负）和左移（向 0 靠近）。
            // 中心线保护：同向 NPC (route_dir>0) 在 offset<0 侧行驶，严禁跨过
            // offset=0 进入对向车道。留 0.5m 余量防 E2 平滑插值期间越线。
            // 原 `> 0.0` 判断允许 cand_left==0.0 → NPC 变到中心线，下一帧又变回，
            // 表现为"向逆向车道来回变道"。
            double cand_right = npc.target_offset - 3.5;
            double cand_left  = npc.target_offset + 3.5;
            if (npc.route_dir > 0 && cand_left >= -0.5) cand_left = -999;

            double best_gain = -1e9;
            double chosen = -999;
            for (double cand : {cand_right, cand_left}) {
                if (cand < -900) continue;
                if (std::fabs(cand) > 20.0) continue;  // 不超过路面范围

                // 边界权限门：虚线才可变道
                if (!boundary_permissive(npc, cand, roads)) continue;

                // MOBIL 代价函数
                bool safe = false;
                double gain = mobil_gain(npc, cand, pool, cfg, safe);
                if (!safe) continue;  // 安全约束不满足（新跟随者会急刹）

                if (gain > best_gain && gain > cfg.mobil_gain_threshold) {
                    best_gain = gain;
                    chosen = cand;
                }
            }

            // 如果 MOBIL 没找到好车道，但被前车阻挡，仍尝试基础安全换道（不跨对向）
            if (chosen < -900 && blocked) {
                for (double cand : {cand_right, cand_left}) {
                    if (cand < -900) continue;
                    if (std::fabs(cand) > 20.0) continue;
                    if (!boundary_permissive(npc, cand, roads)) continue;
                    // 基础安全：检查目标车道前后 30m/8m 无车
                    bool base_safe = true;
                    for (int i = 0; i < pool.size(); ++i) {
                        const Entity& o = pool[i];
                        if (!o.active || &o == &npc) continue;
                        if (!o.is_vehicle()) continue;
                        if (o.route_dir != npc.route_dir) continue;
                        if (std::fabs(o.offset - cand) >= cfg.same_lane_tol) continue;
                        double ahead = (o.route_s - npc.route_s) * (double)npc.route_dir;
                        if (ahead > 0.0 && ahead < 30.0 + npc.length) { base_safe = false; break; }
                        if (ahead < 0.0 && -ahead < 8.0 + npc.length)  { base_safe = false; break; }
                    }
                    if (base_safe) { chosen = cand; break; }
                }
            }

            if (chosen > -900) {
                npc.target_offset = chosen;
                npc.lateral_control = LateralControl::Mobil;
                npc.lane_change_timer = cfg.mobil_lane_change_cooldown;
            }
        }
mobil_done: ;
    }

    // 2. 计算 v_desired（碰撞冷却期间 v_desired=0）
    double v = npc.speed;
    double v_desired = 0.0;
    if (!in_crash_cooldown) {
        if (npc.ai_state == AIState::Stop || npc.ai_state == AIState::StopForTL) {
            v_desired = 0.0;
        } else if (npc.ai_state == AIState::CutIn) {
            // CutIn 期间：纵向减速 cfg.cutin_longitudinal_decel m/s（避免变道时冲撞侧后车），
            // 仍按 IDM 跟前车（gap 不足时进一步减速），但上限为 target_vx - decel。
            double cap = std::max(0.0, npc.target_vx - cfg.cutin_longitudinal_decel);
            if (lead != INVALID_ENTITY) {
                v_desired = idm_desired_speed(v, gap, cap, cfg, dt);
            } else {
                v_desired = cap;
            }
            // 保持 ai_state==CutIn（不要被 lead 分支覆盖成 Follow）
            npc.ai_state = AIState::CutIn;
        } else if (lead != INVALID_ENTITY) {
            npc.ai_state = AIState::Follow;
            v_desired = idm_desired_speed(v, gap, npc.target_vx, cfg, dt);
        } else {
            npc.ai_state = AIState::Cruise;
            v_desired = npc.target_vx;
        }
    }

    // 3. v_desired → throttle/brake（碰撞冷却期间保持刹车）
    double throttle = 0.0, brake = in_crash_cooldown ? 1.0 : 0.0;
    if (!in_crash_cooldown) {
        double dv = v_desired - v;
        if (dv > 0.01) {
            throttle = std::min(1.0, dv / (cfg.accel_rate * dt + 0.01));
            throttle = std::max(0.2, throttle);
        } else if (dv < -0.01) {
            brake = std::min(1.0, -dv / (cfg.brake_rate * dt + 0.01));
        }
    }
    npc.throttle = throttle;
    npc.brake = brake;

    // 4. 纵向积分：step_bicycle 只用来更新 speed（steer=0），位置随后覆盖
    step_bicycle(npc, dt, throttle, brake, 0.0);

    // 5. 位置更新
    // Phase 2: npc.road_pos.ok() 时优先用 RoadPosition 推进——沿真实 OpenDRIVE
    // 拓扑 RM_PositionMoveForward，过路口按 junction_angle 选支路，杜绝单链 Route
    // 在 fork/merge/toll 多通道处丢支路。否则走旧 route/世界系兜底逻辑。
    //
    // 对向 NPC (route_dir < 0) 不能用 road_pos.advance：该 API 只能沿道路 +s 方向
    // 推进，对向车需要 -s 方向。交给 road_pos 会导致位置前进但朝向翻转（车头朝后
    // 却向前移动），route_s 只增不减永远不触发回收，最终在路网末端被回收后反复
    // 出现在 ego 前方——与 ego 同向行驶但 OBB 朝向相反，在窄路段易发生碰撞。
    // 改走旧 route 分支（route_s += route_dir * speed * dt）正确处理对向后退，
    // 到达 route 起点时直接停用（不 recycle，见下方说明）。
    if (npc.road_pos.ok() && roads && roads->loaded() && npc.route_dir >= 0) {
        // ── RoadPosition 推进分支 ──
        // junction_angle 暂用 M_PI（直行）；BranchSel 状态可后续从 route step
        // type 映射左/右转。advance 失败（路网边界）→ recycle。
        double dist = npc.speed * dt;
        double junc_angle = M_PI;
        if (npc.ai_state == AIState::BranchSel) {
            // 路口选支路：暂用直行（后续可从 target_vx 或 route step 决定）
            junc_angle = M_PI;
        }
        bool adv_ok = true;
        if (dist > 0.0) {
            adv_ok = npc.road_pos.advance(dist, junc_angle);
        }
        if (!adv_ok) {
            // 推进失败（路网边界）→ recycle 到 ego 附近。
            // 无 route 可 recycle 时（route.build 失败但 roads 加载成功）停车防飞出：
            // advance 失败时 esmini position 停在最后有效点，speed=0 让 NPC 不再尝试推进。
            if (route && route->ok() && npc.route_dir != 0) {
                recycle_npc(npc, *route, ego_route_s, pool, roads);
            } else {
                npc.speed = 0.0;
                npc.vx = 0.0; npc.vy = 0.0;
            }
        } else {
            // sync 横向 offset 到 road_pos（E2/CutIn 平滑插值后的 npc.offset）
            // 然后用 road_pos.world() 取路网对齐世界坐标
            npc.road_pos.set_offset(npc.offset);
            WorldPos wp;
            if (npc.road_pos.world(wp)) {
                npc.x = wp.x;
                npc.y = wp.y;
                double h = wp.h + (npc.route_dir < 0 ? M_PI : 0.0);
                while (h >  M_PI) h -= 2.0 * M_PI;
                while (h < -M_PI) h += 2.0 * M_PI;
                npc.heading = h;
                npc.vx = npc.speed * std::cos(h);
                npc.vy = npc.speed * std::sin(h);
            }
            // 同步 Frenet 字段（same_lane/lane_change_safe 等用 npc.offset 比较，
            // 但 npc.road_id/lane_id/s 也需更新供下游逻辑/调试使用）
            FrenetPos fp;
            if (npc.road_pos.frenet(fp)) {
                npc.road_id = fp.road_id;
                npc.lane_id = fp.lane_id;
                npc.s = fp.s;
                // 注意：fp.offset 是 lane 内 offset（lane_id=0 时 = npc.offset）。
                // npc.offset 保持由 E2/CutIn 插值驱动的值，不覆盖。
            }
            // route_s 同步：用 route.index_of + to_route_s 把 road_pos 的 (road,s)
            // 映射回 route 累计 s，让 recycle_npc / find_lead 的 route_s 比较仍有效
            if (route && route->ok() && npc.route_dir != 0) {
                int ei = route->index_of(npc.road_id);
                if (ei >= 0) {
                    npc.route_s = route->to_route_s(ei, npc.s);
                    // 越界 → recycle
                    if ((npc.route_dir > 0 && npc.route_s > route->total_length()) ||
                        (npc.route_dir < 0 && npc.route_s < 0.0)) {
                        recycle_npc(npc, *route, ego_route_s, pool, roads);
                    }
                }
            }
            npc.route_fail_count = 0;  // 成功 → 清零
        }
    } else if (route && route->ok() && npc.route_dir != 0 && roads) {
        // ── 中央 Frenet 车道跟随：沿 route 推进 s，反算世界坐标 ──
        // 对向 NPC (route_dir<0) 走此分支（road_pos.advance 不支持 -s 方向）。
        // B1: 保存推进前的 route_s，frenet_to_world 失败时回滚。
        //     原实现失败时 route_s 已推进但 npc.x/y 未更新，下一帧 step_bicycle
        //     会用旧 heading 做世界系直线积分把 NPC 沿切线推离路网 → 飞出。
        double old_route_s = npc.route_s;
        npc.route_s += (double)npc.route_dir * npc.speed * dt;
        // 顺行 NPC (route_dir>0) 到达 route 末端 → recycle 到 ego 后方形成持续车流。
        // 对向 NPC (route_dir<0) 到达 route 起点 → 直接停用，不 recycle。
        //   原因：recycle_npc 把对向车放到 ego 前方 (ego_route_s + back)，但 ego
        //   可能在高速公路（单向 3 车道，半宽 5.25m），对向车 offset=+5.5/+8.0
        //   超出高速路半宽 → 被回收到路外/对向不存在车道，与同向 NPC 冲撞并把
        //   ego 推出路缘（实测 d4ac6b0 commit 引入 entity21/23/24 三次碰撞 +
        //   2.12m road departure）。对向车是单次事件（迎面驶过后不再相关），
        //   停用比错误回收更安全。
        if (npc.route_dir > 0 && npc.route_s > route->total_length()) {
            recycle_npc(npc, *route, ego_route_s, pool, roads);
        } else if (npc.route_dir < 0 && npc.route_s < 0.0) {
            npc.active = false;
            return;
        }
        int rid = 0, ridx = -1;
        double s_local = 0.0;
        route->locate(npc.route_s, rid, s_local, ridx);
        WorldPos wp;
        if (roads->frenet_to_world(rid, 0, s_local, npc.offset, wp)) {
            npc.x = wp.x;
            npc.y = wp.y;
            double h = wp.h + (npc.route_dir < 0 ? M_PI : 0.0);
            while (h >  M_PI) h -= 2.0 * M_PI;
            while (h < -M_PI) h += 2.0 * M_PI;
            npc.heading = h;
            npc.vx = npc.speed * std::cos(h);
            npc.vy = npc.speed * std::sin(h);
            npc.road_id = rid;
            npc.s = s_local;
            npc.route_fail_count = 0;  // 成功 → 清零
        } else {
            // B1: frenet_to_world 失败。回滚 route_s，速度归零防切线飞出；
            //     顺行 NPC 连续失败 ≥5 帧（250ms）强制 recycle；
            //     对向 NPC 连续失败 ≥5 帧直接停用（同样避免错误回收）。
            npc.route_s = old_route_s;
            npc.speed = 0.0;
            npc.vx = 0.0; npc.vy = 0.0;
            npc.route_fail_count++;
            if (npc.route_fail_count >= 5) {
                if (npc.route_dir < 0) {
                    npc.active = false;
                    return;
                }
                recycle_npc(npc, *route, ego_route_s, pool, roads);
            }
        }
    } else if (roads && roads->loaded()) {
        // ── 兜底（route/esmini 缺失或该 NPC 不在主 route 上）──
        FrenetPos f;
        if (roads->world_to_frenet(npc.x, npc.y, f)) {
            npc.road_id = f.road_id;
            npc.lane_id = f.lane_id;
            npc.s = f.s;
            npc.offset = f.offset;
            // B2: world→frenet→world 回投闭环。原实现只用 world_to_frenet
            //     更新 Frenet 字段，npc.x/y 仍是 step_bicycle 世界系直线积分结果
            //     —— 弯道/匝道上车会沿切线飞出。frenet_to_world 把 NPC 投回路面。
            WorldPos wp;
            if (roads->frenet_to_world(f.road_id, 0, f.s, f.offset, wp)) {
                npc.x = wp.x;
                npc.y = wp.y;
                npc.heading = wp.h;
                npc.vx = npc.speed * std::cos(wp.h);
                npc.vy = npc.speed * std::sin(wp.h);
            }
        }
    }
}

void npc_init_route(Entity& npc, const Route& route, int dir) {
    int idx = route.index_of(npc.road_id);
    if (idx < 0) {
        npc.route_dir = 0;   // 不在主 route → 走旧逻辑兜底
        return;
    }
    // B5: 负 s（actor 放在 road 起点之前，或对向车 road 链上的 s 反向）
    // 会让 to_route_s 算出负 route_s，多个负 s NPC 被 locate() clamp 到 0
    // 后全部叠在 route 起点。直接置 route_dir=0 让其走世界系兜底分支。
    if (npc.s < 0.0) {
        npc.route_dir = 0;
        return;
    }
    npc.route_dir = (dir < 0) ? -1 : 1;
    npc.route_s   = route.to_route_s(idx, npc.s);
}

void step_npc_pedestrian(Entity& ped, double dt, const NpcAiConfig& cfg) {
    // 完全静止的行人（vx=0 && vy=0）保持原地
    if (ped.vx == 0.0 && ped.vy == 0.0) {
        ped.speed = 0.0;
        return;
    }

    // 已停在路边：累计等待计时器，到时反向
    if (ped.ped_parked) {
        ped.ped_wait_timer += dt;
        if (ped.ped_wait_timer >= cfg.ped_wait_time) {
            ped.ped_wait_timer = 0.0;
            ped.ped_parked = 0;
            ped.vy = -ped.vy;  // 反向横穿
        }
        return;
    }

    // 移动
    step_pedestrian(ped, dt);

    // 横穿行人（vy != 0）：到达边界后停车等待
    if (std::fabs(ped.vy) > 0.01) {
        if (ped.vy > 0 && ped.y >= cfg.ped_boundary) {
            ped.y = cfg.ped_boundary;
            ped.ped_parked = 1;
        } else if (ped.vy < 0 && ped.y <= -cfg.ped_boundary) {
            ped.y = -cfg.ped_boundary;
            ped.ped_parked = 1;
        }
    }
}

}  // namespace flowsim
