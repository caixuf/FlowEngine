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
    // 旧格式：road_id/lane_id 严格匹配。
    // P2 修复：原 `road_id > 0` 漏掉 road_id == 0 的合法道路（OpenDRIVE 允许
    // road id 从 0 起，straight_road.json 的 edges[0].id 即为 0；road_network.cpp
    // 的 world_to_frenet 也以 road_id >= 0 判定合法）。改为 `>= 0` 与
    // flowsim_node.cpp::spawn_npc / road_network.cpp::world_to_frenet 一致。
    if (a.road_id >= 0 && b.road_id >= 0 && a.lane_id != 0 && b.lane_id != 0) {
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
    if (npc.state == NpcState::CutIn) return true;
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
                        const EntityPool& pool, FlowRoadNetwork* roads,
                        uint32_t cycle, const NpcAiConfig& cfg) {
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
    /* 标记本次显式传送，供 temporal invariant 跳过 Δpos 检查 */
    npc.last_teleport_cycle = cycle;
    // 重置动态状态：之前 recycle 只改 route_s，speed/ai_state/lead_id/crash_cooldown
    // 残留旧值——刚刹停的车回收后 speed=0 顶在新位置不动；刚撞车冻结的车回收后
    // crash_cooldown>0 继续冻结；follow 状态的车回收后还在追一辆已不存在的 lead。
    // 重置后 NPC 在新位置以 Cruise 状态、原 target_vx 起步。
    npc.speed = 0.0;
    npc.vx = 0.0; npc.vy = 0.0;
    npc.throttle = 0.0; npc.brake = 0.0;
    /* P2 修复：状态转移改走统一入口 npc_request_state（与 apply_actor_override
     * P1.3 修复同模式）。
     *
     * 原实现直接 `npc.state = NpcState::Cruise` 绕过状态机，导致 cutin_active
     * 孤儿——若回收前 NPC 处于 CutIn 状态，cutin_active 保持 true 不被清除
     * （state==CutIn 时 boundary_permissive 直接放行跨实线变道、MOBIL 跳过
     * 评估），后续帧逻辑误判。改用 NpcEvent::Recycle 走统一入口：
     *   - CutIn→Cruise：npc_request_state 内部"离开 CutIn"块清 cutin_active
     *     + cutin_pid_integral/prev；
     *   - 任意→Cruise：Recycle 块无条件清 lead_id/follow_gap/lane_change_timer
     *     /crash_cooldown。
     * speed/vx/vy/throttle/brake/route_fail_count/target_offset 不在状态机内
     * 管理，仍手动清零。 */
    NpcTransitionRequest req;
    req.event = NpcEvent::Recycle;
    npc_request_state(npc, req, cfg);
    npc.route_fail_count = 0;
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
                      double ego_route_s, uint32_t cycle) {
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
    // 横向控制权由写入 target_offset 的系统隐式持有，CutIn 完成时释放。
    // 脚本/编舞（状态=CutIn/LaneChange）期间 MOBIL 跳过评估，
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
    if (!in_crash_cooldown && npc.state == NpcState::CutIn) {
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
        // 释放横向控制权，后续 MOBIL/IDM 可接管
        if (std::fabs(npc.target_offset - npc.offset) < cfg.cutin_completion_threshold) {
            npc.offset = npc.target_offset;
            /* ── P1.1 修复：状态转移改走统一入口 npc_request_state ──
             *
             * 原 4 行手动副作用（state=Cruise + 清 PID 积分 + 清 cutin_active）
             * 已在 npc_request_state 内部统一处理（line 882-886）。
             *
             * 用 LeadLost 事件而非新增 CutInCompleted 的原因：CutIn 完成后下一帧
             * step_npc_vehicle 会自动评估 lead（line 600-604）：有前车 → Follow，
             * 无前车 → Cruise。所以 CutIn 完成等价于"释放横向控制权，让状态机
             * 重新评估纵向行为"——LeadLost 语义最贴近。
             *
             * 这同时把"CutIn 退出时清理 PID 状态"集中到 npc_request_state，
             * 避免 apply_actor_override / recycle_npc 等其他写 state 的地方
             * 重复实现清理逻辑（之前就有遗漏 case，见 P1.3）。 */
            npc_request_state(npc, {NpcEvent::LeadLost}, cfg);
        }
    } else if (!in_crash_cooldown && npc.state == NpcState::LaneChange &&
               std::fabs(npc.offset - npc.target_offset) > 0.01) {
        /* ── E2: 平滑横移到 target_offset ──
         *
         * 用户需求：演示场景中 NPC 各守其道不变道，仅 ego 变道超车。
         * 此分支原为 MOBIL 自主变道服务——MOBIL 把 state 置为 LaneChange 后，
         * E2 在 1.5s 内把 offset 平滑插值到 target_offset。
         *
         * 现在 MOBIL 已 #if 0 禁用，state==LaneChange 永远不会被设置，
         * 此分支事实上是死代码。但保留它（带 state 门禁）有两个好处：
         *   1. 未来重新启用 MOBIL 时无需改这里，把 #if 0 改回 #if 1 即可联动恢复。
         *   2. 防御性：即使有 bug 让 target_offset != offset（如 Frenet 投影漂移、
         *      外部代码误写），Cruise/Follow/StopForTL 状态下也不会触发横移，
         *      NPC 严格按当前 offset 直行。
         *
         * 关键改动：原条件 `std::fabs(offset - target_offset) > 0.01` 不带 state 门禁，
         * 任何状态下只要 target_offset != offset 就横移——这是"NPC 禁用后仍变来变去"
         * 的根因（target_offset 可能被 recycle/外部代码/choreography 残留值污染，
         * 与 offset 不同步）。加 state==LaneChange 门禁后，仅 MOBIL 显式变道时触发。 */
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

    /* ── NPC 横向控制权归零：非变道状态下强制 target_offset = offset ──
     *
     * 用户需求：演示场景中 NPC 各守其道不变道。MOBIL 已 #if 0 禁用，
     * LaneChange 状态永不被设置；CutIn 完成后状态切回 Cruise/Follow。
     * 此时若 target_offset != offset（残留值/Frenet 投影漂移/外部代码误写），
     * 会让下一帧的 E2 分支（虽已加 state 门禁）或 CutIn PID 误动作。
     *
     * 此处兜底：非 CutIn 且非 LaneChange 状态下，强制 target_offset = offset，
     * 彻底断绝"target_offset 残留导致 NPC 漂移"的可能。
     * 不影响 CutIn PID（PID 分支在上面已执行完毕）和 LaneChange E2（上面已执行）。 */
    if (npc.state != NpcState::CutIn && npc.state != NpcState::LaneChange) {
        npc.target_offset = npc.offset;
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
    //    仲裁检查：CutIn/LaneChange（脚本/编舞/CutIn 活跃中）时跳过，
    //    防止 MOBIL 覆盖场景导演的横向指令。
    //    红绿灯/让行期间不评估变道（StopForTL/Yield 状态下保持当前车道）
    /* ──────────────────────────────────────────────────────────────
     * MOBIL 自主变道已被有意禁用。
     *
     * 用户需求：演示场景中 NPC 各守其道不变道，仅 ego 变道超车。
     * 启用 MOBIL 后 NPC 会自主换道避堵/超车，与"NPC 不变道"需求冲突。
     * 场景编导（choreography）才是横向指令的唯一来源。
     *
     * 保留完整 MOBIL 实现作为参考，便于未来重新启用（如 NOA 高密度
     * 车流场景）。重新启用步骤：
     *   1. 把下方 #if 0 改回 #if 1
     *   2. 修复 mobil_idm_accel 的简化 bug（std::min 在 target_v<v 时
     *      返回负加速度，应改用标准 IDM (1-(v/v0)^4) 项）
     *   3. 验证 lane_change_timer 冷却期间 target_offset 平滑插值无抖动
     * ────────────────────────────────────────────────────────────── */
#if 0
    {
        // 仲裁门禁：存在更高优先级的横向控制时跳过
        if (npc.state == NpcState::CutIn ||
            npc.state == NpcState::LaneChange) { goto mobil_done; }
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
                npc.state = NpcState::LaneChange;
                npc.lane_change_timer = cfg.mobil_lane_change_cooldown;
            }
        }
mobil_done: ;
    }
#endif  /* MOBIL 自主变道禁用 */

    // 2. 计算 v_desired（碰撞冷却期间 v_desired=0）
    double v = npc.speed;
    double v_desired = 0.0;
    if (!in_crash_cooldown) {
        if (npc.state == NpcState::Stopped || npc.state == NpcState::StopForTL) {
            v_desired = 0.0;
        } else if (npc.state == NpcState::CutIn) {
            // CutIn 期间：纵向减速 cfg.cutin_longitudinal_decel m/s（避免变道时冲撞侧后车），
            // 仍按 IDM 跟前车（gap 不足时进一步减速），但上限为 target_vx - decel。
            double cap = std::max(0.0, npc.target_vx - cfg.cutin_longitudinal_decel);
            if (lead != INVALID_ENTITY) {
                v_desired = idm_desired_speed(v, gap, cap, cfg, dt);
            } else {
                v_desired = cap;
            }
            // 保持 ai_state==CutIn（不要被 lead 分支覆盖成 Follow）
            npc.state = NpcState::CutIn;
        } else if (lead != INVALID_ENTITY) {
            npc.state = NpcState::Follow;
            v_desired = idm_desired_speed(v, gap, npc.target_vx, cfg, dt);
        } else {
            npc.state = NpcState::Cruise;
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
        if (npc.state == NpcState::Cruise) {
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
                recycle_npc(npc, *route, ego_route_s, pool, roads, cycle, cfg);
            } else {
                npc.speed = 0.0;
                npc.vx = 0.0; npc.vy = 0.0;
            }
        } else {
            // sync 横向 offset 到 road_pos（E2/CutIn 平滑插值后的 npc.offset）
            // 然后用 road_pos.world() 取路网对齐世界坐标
            //
            // ── Bug 修复（"NPC 骑线行驶" / "车晃来晃去"）──
            // npc.offset 语义是「相对道路参考线（lane_id=0）的横向偏移」，
            // 与 init 时 (flowsim_node.cpp:554) `e.offset = lane_center_t + fp.offset`
            // 一致。但 RoadPosition::set_offset 内部调用
            //   RM_SetLanePosition(handle, roadId, laneId, offset, s, false)
            // 当 laneId != 0 时 offset 语义为「相对车道中心的偏移」(lane-internal)。
            //
            // 旧格式 NPC init road_pos 时用真实 lane_id (flowsim_node.cpp:602，
            // 如 -1/-2/+1)，直接传 npc.offset(road ref) 会让车被推到车道边界
            // （实测 NPC1 在 y=-3.50 骑线行驶、motion_direction invariant 失败）。
            // 新格式 NPC 用 lane_id=0 init，offset 语义匹配，不报此 bug。
            //
            // 修复：先取 road_pos 刚 advance 后的 lane_id（可能与上一帧 npc.lane_id
            // 不同，过路口时 esmini 可能切车道），把 npc.offset 转换为
            //   lane_internal = npc.offset - lane_center_t
            // 其中 lane_center_t = sign(lane_id) * (|lane_id| - 0.5) * lane_width
            // （与 flowsim_node.cpp:552-553 / 本文件 line 834-835 公式一致）。
            // lane_id == 0 时 offset 即 road ref，无需转换。
            double lane_internal = npc.offset;
            int rp_lane = npc.road_pos.lane_id();
            if (rp_lane != 0 && roads && roads->loaded()) {
                int    rp_road = npc.road_pos.road_id();
                double rp_s    = npc.road_pos.s();
                double lw = roads->lane_width(rp_road, rp_lane, rp_s);
                if (lw <= 0.0) lw = 3.5;
                double lane_center_t = (rp_lane > 0 ? 1.0 : -1.0)
                                     * (std::abs(rp_lane) - 0.5) * lw;
                lane_internal = npc.offset - lane_center_t;
            }
            npc.road_pos.set_offset(lane_internal);
            WorldPos wp;
            if (npc.road_pos.world(wp)) {
                /* DEBUG: trace esmini heading（按场景业务 id=1 的 NPC） */
                if (npc.scenario_id == 1 && (cycle % 50) == 1) {
                    fprintf(stderr, "[DBG npc1] road_pos.world wp=(%.2f,%.2f,%.2f) h=%.3f route_dir=%d\n",
                            wp.x, wp.y, wp.z, wp.h, npc.route_dir);
                }
                /* ── P1.2 修复：crash_cooldown 期间不覆写 npc.x/y ──
                 *
                 * 原实现无条件 `npc.x = wp.x; npc.y = wp.y;` —— 但 crash_cooldown
                 * 期间 collision.cpp::apply_collision_response 已把车纵向分离
                 * （route_dir!=0 改 route_s；route_dir==0 改 npc.x/y）。road_pos
                 * 内部 s 未同步 → world() 取回碰撞前位置 → 覆盖 collision 写入
                 * 的分离位置 → 两车再次重叠 → cooldown 重置 → 永久卡死。
                 *
                 * 修复：crash_cooldown 期间保留 collision 写入的 npc.x/y/heading，
                 * 仅同步 Frenet 字段（road_id/lane_id/s/route_s）。同时用
                 * world_to_frenet 反向 sync road_pos 到分离后位置，让 cooldown
                 * 结束后第一帧 advance 用正确 s，不会跳回碰撞前位置。 */
                if (!in_crash_cooldown) {
                    npc.x = wp.x;
                    npc.y = wp.y;
                    double h = wp.h + (npc.route_dir < 0 ? M_PI : 0.0);
                    while (h >  M_PI) h -= 2.0 * M_PI;
                    while (h < -M_PI) h += 2.0 * M_PI;
                    npc.heading = h;
                    npc.vx = npc.speed * std::cos(h);
                    npc.vy = npc.speed * std::sin(h);
                } else {
                    /* crash_cooldown：用 npc 当前 (x,y) 反向 sync road_pos
                     * 让 road_pos 内部 s 与 collision 分离后位置一致。
                     * 注意 speed=0，vx/vy 已被 collision response 清零，无需重算。 */
                    FrenetPos fp_sep;
                    if (roads->world_to_frenet(npc.x, npc.y, fp_sep)) {
                        npc.road_pos.init(*roads, fp_sep.road_id, 0,
                                          fp_sep.s, npc.offset);
                    }
                }
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
                        recycle_npc(npc, *route, ego_route_s, pool, roads, cycle, cfg);
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
            recycle_npc(npc, *route, ego_route_s, pool, roads, cycle, cfg);
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
                recycle_npc(npc, *route, ego_route_s, pool, roads, cycle, cfg);
            }
        }
    } else if (roads && roads->loaded()) {
        // ── 兜底（route/esmini 缺失或该 NPC 不在主 route 上）──
        FrenetPos f;
        if (roads->world_to_frenet(npc.x, npc.y, f)) {
            npc.road_id = f.road_id;
            npc.lane_id = f.lane_id;
            npc.s = f.s;
            /* ── Bug 修复：原 `npc.offset = f.offset` 把「车道内偏移」当成
             * 「相对参考线偏移」直接覆盖，是"NPC 在车道间变来变去"的根因。
             *
             * f.offset 是 esmini 报告的 lane 内偏移（车相对 lane 中心的横向偏移），
             * 范围约 [-lw/2, +lw/2]，对在车道中心的 NPC ≈ 0。而 npc.offset 语义是
             * 「相对道路参考线（lane_id=0）的横向偏移」，与 init 时
             * (flowsim_node.cpp:540) `e.offset = lane_center_t + fp.offset` 一致：
             *   lane_center_t = sign(lane_id) * (|lane_id| - 0.5) * lane_width
             *
             * 原代码用 f.offset（≈0）覆盖 npc.offset（如 -1.75）→ 下一帧
             * frenet_to_world(f.road_id, 0, f.s, npc.offset=0, wp) 把 NPC 投到
             * 参考线（y=0）而非原车道（y=-1.75）→ world_to_frenet 在车道边界
             * 抖动把 NPC 反复 snap 到不同 lane_id → npc.y 在 -1.75 / 0 / +1.75
             * 间"变来变去"。
             *
             * 修复：与 init 一致，把 f.offset 转换为 lane_center + f.offset 后
             * 再写入 npc.offset，保持横向位置语义一致。 */
            double lw = roads->lane_width(f.road_id, f.lane_id, f.s);
            if (lw <= 0.0) lw = 3.5;
            double lane_center_t = (f.lane_id > 0 ? 1.0 : -1.0)
                                 * (std::abs(f.lane_id) - 0.5) * lw;
            npc.offset = lane_center_t + f.offset;
            /* NPC 横向控制权归零：兜底 NPC 通常 route_dir=0（不在主 route 上），
             * 不参与变道，强制 target_offset = offset 让它沿当前车道直行。
             * 注意此处 offset 已是正确的 lane_center + f.offset。 */
            npc.target_offset = npc.offset;
            // B2: world→frenet→world 回投闭环。原实现只用 world_to_frenet
            //     更新 Frenet 字段，npc.x/y 仍是 step_bicycle 世界系直线积分结果
            //     —— 弯道/匝道上车会沿切线飞出。frenet_to_world 把 NPC 投回路面。
            //     注意第四个参数用 npc.offset（lane_center + f.offset）而非 f.offset，
            //     与上面的转换保持一致，否则 wp.y 仍会落到参考线而非车道中心。
            WorldPos wp;
            if (roads->frenet_to_world(f.road_id, 0, f.s, npc.offset, wp)) {
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

/* ── 统一状态转移入口 ──
 * 唯一合法修改 npc.state 的函数。
 * 按 NpcEvent 映射到 NpcState，并重置状态相关字段。 */
bool npc_request_state(Entity& npc, const NpcTransitionRequest& req,
                       const NpcAiConfig& cfg) {
    (void)cfg;
    NpcState old = npc.state;
    NpcState target = old;

    switch (req.event) {
        case NpcEvent::LeadFound:       target = NpcState::Follow;     break;
        case NpcEvent::LeadLost:        target = NpcState::Cruise;     break;
        case NpcEvent::TL_Red:          target = NpcState::StopForTL;  break;
        case NpcEvent::TL_Green:        target = NpcState::Cruise;     break;
        case NpcEvent::MobilChange:     target = NpcState::LaneChange; break;
        case NpcEvent::ChoreoCutIn:     target = NpcState::CutIn;      break;
        case NpcEvent::ChoreoOvertake:  target = NpcState::Cruise;     break;
        case NpcEvent::ScriptOverride:  target = NpcState::CutIn;     break;
        case NpcEvent::ScriptSet:        target = req.target_state;    break;
        case NpcEvent::Collision:       target = NpcState::Stopped;    break;
        case NpcEvent::Recycle:         target = NpcState::Cruise;     break;
        case NpcEvent::None:            return false;
    }

    /* 状态切换时重置状态相关字段 */
    if (target != old) {
        npc.state = target;
        /* 离开 CutIn → 释放横向控制权 */
        if (old == NpcState::CutIn && target != NpcState::CutIn) {
            npc.cutin_active = false;
            npc.cutin_pid_integral = 0.0;
            npc.cutin_pid_prev = 0.0;
        }
        /* 进入 CutIn → 初始化 PID 状态 */
        if (target == NpcState::CutIn) {
            npc.cutin_active = true;
            npc.cutin_pid_integral = 0.0;
            npc.cutin_pid_prev = 0.0;
        }
        /* 进入 LaneChange → 设冷却 */
        if (target == NpcState::LaneChange) {
            npc.lane_change_timer = cfg.mobil_lane_change_cooldown;
        }
        /* 进入 StopForTL/Stopped → 立即减速 */
        if (target == NpcState::StopForTL || target == NpcState::Stopped) {
            npc.brake = 1.0;
            npc.throttle = 0.0;
        }
    }
    /* Recycle → 全量重置（无论状态是否变化）。
     * P2 修复：原本在 `if (target != old)` 内，导致已处于 Cruise 的 NPC
     * 回收时 lead_id/follow_gap/lane_change_timer/crash_cooldown 不被清零
     * （Cruise→Cruise 时 target==old 跳过整块），残留旧值（如已不存在的
     * lead_id）。移出到状态切换块外，使 Recycle 事件无条件重置这些字段。
     * cutin_active 不在此清——它只可能在 state==CutIn 时为 true，而
     * CutIn→Cruise 必有 target!=old，上面的"离开 CutIn"块已覆盖。 */
    if (req.event == NpcEvent::Recycle) {
        npc.lead_id = INVALID_ENTITY;
        npc.follow_gap = 1e9;
        npc.lane_change_timer = 0.0;
        npc.crash_cooldown = 0.0;
    }

    /* 应用请求覆盖字段（无论是否发生状态切换） */
    if (req.target_offset != NPC_REQ_UNSET) {
        npc.target_offset = req.target_offset;
    }
    if (req.target_vx != NPC_REQ_UNSET) {
        npc.target_vx = req.target_vx;
    }
    if (req.vx != NPC_REQ_UNSET) {
        npc.vx = req.vx;
        npc.speed = std::fabs(req.vx);
    }
    if (req.vy != NPC_REQ_UNSET) {
        npc.vy = req.vy;
    }
    if (req.x != NPC_REQ_UNSET) {
        npc.x = req.x;
    }
    if (req.y != NPC_REQ_UNSET) {
        npc.y = req.y;
    }
    if (req.heading != NPC_REQ_UNSET) {
        npc.heading = req.heading;
    }

    return true;
}

}  // namespace flowsim
