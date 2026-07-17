/**
 * npc_ai.cpp — NPC AI 实现
 *
 * IDM 跟车模型（简化版，来自设计文档 §4.2）：
 *   safe_gap = base + v * time
 *   gap_error = gap - safe_gap
 *   if gap_error > 0:  v_desired = min(v + accel_rate*dt, target_v)
 *   else:              v_desired = max(0, v - brake*exp(-gap_error/2)*dt)
 *
 * 前车搜索：同车道（lane_id 匹配 或 |Δy| < tol）、前方（Δx > 0）、最近。
 * 兼容新旧坐标系：lane_id==0 时回退到横向距离判断。
 */

#include "npc_ai.h"
#include "physics.h"
#include "road_network.h"

#include <algorithm>
#include <cmath>

namespace flowsim {

/* ── 判断两车是否同车道 ── */
static bool same_lane(const Entity& a, const Entity& b, const NpcAiConfig& cfg) {
    // 新格式：都有 road_id > 0 且 lane_id != 0 → 严格匹配
    if (a.road_id > 0 && b.road_id > 0 && a.lane_id != 0 && b.lane_id != 0) {
        return a.road_id == b.road_id && a.lane_id == b.lane_id;
    }
    // 旧格式：横向距离 < 车道宽度/2
    return std::fabs(a.y - b.y) < cfg.same_lane_tol;
}

/* ── 找同车道最近前车 ── */
static EntityId find_lead(const Entity& npc, const EntityPool& pool,
                          const NpcAiConfig& cfg) {
    EntityId best = INVALID_ENTITY;
    double best_dx = 1e9;
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& o = pool[i];
        if (!o.active || &o == &npc) continue;
        if (!o.is_vehicle()) continue;
        // 只看前车（Δx > 0）
        double dx = o.x - npc.x;
        if (dx <= 0) continue;
        if (dx > cfg.look_ahead) continue;
        if (!same_lane(npc, o, cfg)) continue;
        // 算到前车后沿的净间距
        double gap = dx - (o.length * 0.5 + npc.length * 0.5);
        if (gap < best_dx) {
            best_dx = gap;
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

void step_npc_vehicle(Entity& npc, const EntityPool& pool,
                      double dt, const NpcAiConfig& cfg,
                      FlowRoadNetwork* roads) {
    // 1. 找前车
    EntityId lead = find_lead(npc, pool, cfg);
    npc.lead_id = lead;

    double gap = 1e9;
    if (lead != INVALID_ENTITY) {
        const Entity& lead_e = pool[lead];
        gap = (lead_e.x - npc.x) - (lead_e.length * 0.5 + npc.length * 0.5);
        npc.follow_gap = gap;
    } else {
        npc.follow_gap = 1e9;
    }

    // 2. 计算 v_desired
    double v = npc.speed;
    double v_desired;
    if (npc.ai_state == AIState::Stop || npc.ai_state == AIState::StopForTL) {
        // 强制停车状态：目标速度 0
        v_desired = 0.0;
    } else if (lead != INVALID_ENTITY && gap < 60.0) {
        // 有前车且较近：IDM 跟车
        npc.ai_state = AIState::Follow;
        v_desired = idm_desired_speed(v, gap, npc.target_vx, cfg, dt);
    } else {
        // 无前车或前车很远：巡航
        npc.ai_state = AIState::Cruise;
        v_desired = npc.target_vx;
    }

    // 3. v_desired → throttle/brake
    double dv = v_desired - v;
    double throttle = 0.0, brake = 0.0;
    if (dv > 0.01) {
        // 需要加速：throttle 按 dv 比例，上限 1.0
        throttle = std::min(1.0, dv / (cfg.accel_rate * dt + 0.01));
        throttle = std::max(0.2, throttle);  // 起步最低油门
    } else if (dv < -0.01) {
        // 需要减速：brake 按 |dv| 比例
        brake = std::min(1.0, -dv / (cfg.brake_rate * dt + 0.01));
    }

    // 4. 自行车模型积分（NPC 直行，steer=0；横向控制后续可加车道保持）
    step_bicycle(npc, dt, throttle, brake, 0.0);

    // 5. 更新 Frenet 坐标（若提供路网）
    if (roads && roads->loaded()) {
        FrenetPos f;
        if (roads->world_to_frenet(npc.x, npc.y, f)) {
            npc.road_id = f.road_id;
            npc.lane_id = f.lane_id;
            npc.s = f.s;
            npc.offset = f.offset;
        }
    }
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
