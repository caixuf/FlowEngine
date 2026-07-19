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

/* ── 回收：跑到 route 末端的 NPC 放回 ego 附近，形成持续车流 ──
 * B4: 检查目标点附近是否已有同方向 NPC，被占则再后退一段，避免多车叠在同一点。 */
static void recycle_npc(Entity& npc, const Route& route, double ego_route_s,
                        const EntityPool& pool) {
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
    npc.lead_id = INVALID_ENTITY;
    npc.follow_gap = 1e9;
    npc.crash_cooldown = 0.0;
    npc.route_fail_count = 0;
}

void step_npc_vehicle(Entity& npc, const EntityPool& pool,
                      double dt, const NpcAiConfig& cfg,
                      FlowRoadNetwork* roads, const Route* route,
                      double ego_route_s) {
    // 0. 碰撞冷却：crash_cooldown > 0 期间速度强制归零，跳过 AI。
    //    冷却到期后释放，NPC 重新走 IDM + 路径跟随。原实现永久冻结导致
    //    任何一次接触都把两辆车钉死成永久路障，长场景越积越多把路堵死。
    if (npc.crash_cooldown > 0.0) {
        npc.crash_cooldown -= dt;
        if (npc.crash_cooldown < 0.0) npc.crash_cooldown = 0.0;
        npc.speed = 0.0;
        npc.vx = 0.0; npc.vy = 0.0;
        npc.throttle = 0.0; npc.brake = 1.0;
        return;
    }

    // 1. 找前车
    EntityId lead = find_lead(npc, pool, cfg);
    npc.lead_id = lead;

    double gap = 1e9;
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

    // 2. 计算 v_desired
    // 死区修复：原 `gap < 60.0` 才进 Follow，但 find_lead 的 look_ahead=80m，
    // 60-80m 内 lead 已写入但 FSM 走 Cruise 全速前进——30 m/s 时这 0.67s 死区
    // 叠加 IDM 制动距离不足必然追尾。改为只要 find_lead 命中就启用 IDM 跟车，
    // 远距时 idm_desired_speed 自然返回 target_vx（safe_gap 充足走加速分支）。
    double v = npc.speed;
    double v_desired;
    if (npc.ai_state == AIState::Stop || npc.ai_state == AIState::StopForTL) {
        v_desired = 0.0;
    } else if (lead != INVALID_ENTITY) {
        npc.ai_state = AIState::Follow;
        v_desired = idm_desired_speed(v, gap, npc.target_vx, cfg, dt);
    } else {
        npc.ai_state = AIState::Cruise;
        v_desired = npc.target_vx;
    }

    // 3. v_desired → throttle/brake
    double dv = v_desired - v;
    double throttle = 0.0, brake = 0.0;
    if (dv > 0.01) {
        throttle = std::min(1.0, dv / (cfg.accel_rate * dt + 0.01));
        throttle = std::max(0.2, throttle);  // 起步最低油门
    } else if (dv < -0.01) {
        brake = std::min(1.0, -dv / (cfg.brake_rate * dt + 0.01));
    }

    // 4. 纵向积分：step_bicycle 只用来更新 speed（steer=0），位置随后覆盖
    step_bicycle(npc, dt, throttle, brake, 0.0);

    // 5. 位置更新
    if (route && route->ok() && npc.route_dir != 0 && roads) {
        // ── 中央 Frenet 车道跟随：沿 route 推进 s，反算世界坐标 ──
        // B1: 保存推进前的 route_s，frenet_to_world 失败时回滚。
        //     原实现失败时 route_s 已推进但 npc.x/y 未更新，下一帧 step_bicycle
        //     会用旧 heading 做世界系直线积分把 NPC 沿切线推离路网 → 飞出。
        double old_route_s = npc.route_s;
        npc.route_s += (double)npc.route_dir * npc.speed * dt;
        if ((npc.route_dir > 0 && npc.route_s > route->total_length()) ||
            (npc.route_dir < 0 && npc.route_s < 0.0)) {
            recycle_npc(npc, *route, ego_route_s, pool);
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
            //     连续失败 ≥5 帧（250ms）强制 recycle。
            npc.route_s = old_route_s;
            npc.speed = 0.0;
            npc.vx = 0.0; npc.vy = 0.0;
            npc.route_fail_count++;
            if (npc.route_fail_count >= 5) {
                recycle_npc(npc, *route, ego_route_s, pool);
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
