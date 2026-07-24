/**
 * scene_events.cpp — 场景事件调度实现
 *
 * 红绿灯相位推进：基于仿真时间 fmod 计算当前相位。
 *   T = green + yellow + red
 *   tp = fmod(sim_time + offset, T)
 *   tp < green → Green
 *   tp < green+yellow → Yellow
 *   else → Red
 *
 * ETC 闸门：根据 ego 距离切换状态，抬杆进度线性插值。
 *
 * NPC 响应：前方有红灯/关闭闸门时，按制动距离减速到 0。
 */

#include "scene_events.h"
#include "npc_ai.h"
#include "physics.h"
#include "road_network.h"
#include "route.h"
#include "logger.h"
#include "scenario_loader.h"

#include <cmath>
#include <algorithm>

namespace flowsim {

void tick_traffic_lights(EntityPool& pool, double sim_time_s) {
    for (int i = 0; i < pool.size(); ++i) {
        Entity& tl = pool[i];
        if (!tl.active || tl.type != EntityType::TrafficLight) continue;

        // 相位时长复用 throttle/brake/steer 字段
        double green_s  = tl.throttle;
        double yellow_s = tl.brake;
        double red_s    = tl.steer;
        double offset   = tl.target_vx;  // 相位偏移

        if (green_s <= 0 && yellow_s <= 0 && red_s <= 0) {
            // 未配置相位时长，跳过
            continue;
        }

        double T = green_s + yellow_s + red_s;
        if (T <= 0) continue;

        double tp = std::fmod(sim_time_s + offset, T);
        if (tp < 0) tp += T;  // fmod 可能返回负

        if (tp < green_s) {
            tl.phase_state = (int)TLPhase::Green;
        } else if (tp < green_s + yellow_s) {
            tl.phase_state = (int)TLPhase::Yellow;
        } else {
            tl.phase_state = (int)TLPhase::Red;
        }
        tl.phase_timer = T - tp;  // 当前相位剩余时间
    }

    // 单路口多信号灯相位互斥校验（潜在设计缺口检测）。
    // 当前场景只有 1 盏灯未触发，但任何新场景在同一路口加第二盏灯就可能
    // 两个方向同时绿灯。此处只做检测+告警，不实现完整互斥机制。
    // 路口分组：Entity 上无 junction_id 字段，用相同 x 位置（±1m 容差）近似。
    for (int i = 0; i < pool.size(); ++i) {
        const Entity& tl_i = pool[i];
        if (!tl_i.active || tl_i.type != EntityType::TrafficLight) continue;
        if (tl_i.phase_state != (int)TLPhase::Green) continue;
        for (int j = i + 1; j < pool.size(); ++j) {
            const Entity& tl_j = pool[j];
            if (!tl_j.active || tl_j.type != EntityType::TrafficLight) continue;
            if (tl_j.phase_state != (int)TLPhase::Green) continue;
            if (std::fabs(tl_i.x - tl_j.x) <= 1.0) {
                LOG_WARN("flowsim",
                         "tick_traffic_lights: two traffic lights at same junction "
                         "(x=%.2f) both Green - phase mutex violated",
                         tl_i.x);
            }
        }
    }
}

void tick_etc_gates(EntityPool& pool, const Entity& ego, double dt) {
    for (int i = 0; i < pool.size(); ++i) {
        Entity& gate = pool[i];
        if (!gate.active || gate.type != EntityType::ETCGate) continue;

        // 用 x 差作为距离（闸门沿 x 轴布置）
        double dist = gate.x - ego.x;

        /* ETC 闸门状态用 phase_state：0=关闭 1=抬杆中 2=全开。
         * 原复用 ai_state（Stop/Yield/Cruise），改为独立字段避免 NPC 状态语义冲突。 */
        if (dist > 50.0) {
            gate.phase_state = 0;   /* closed */
            gate.phase_timer = 0.0;
        } else if (dist > 10.0) {
            gate.phase_state = 1;   /* opening */
            gate.phase_timer = std::min(1.0, gate.phase_timer + dt * 0.5);
        } else if (dist > 0) {
            gate.phase_state = 2;   /* open */
            gate.phase_timer = 1.0;
        } else {
            gate.phase_state = 0;   /* closed (passed) */
            gate.phase_timer = std::max(0.0, gate.phase_timer - dt * 0.5);
        }
    }
}

void check_npc_scene_events(EntityPool& pool, double look_ahead,
                             const NpcAiConfig& cfg) {
    for (int i = 0; i < pool.size(); ++i) {
        Entity& npc = pool[i];
        if (!npc.active || !npc.is_npc_vehicle()) continue;

        // 找前方最近的红灯/关闭闸门
        double nearest_block_s = 1e9;
        bool  should_stop = false;

        for (int j = 0; j < pool.size(); ++j) {
            const Entity& ev = pool[j];
            if (!ev.active) continue;
            if (ev.type != EntityType::TrafficLight && ev.type != EntityType::ETCGate) continue;

            // 只看前方
            double dx = ev.x - npc.x;
            if (dx <= 0 || dx > look_ahead) continue;

            double tl_lane_y = ev.width;  /* e.width = 车道中心 y */
            if (tl_lane_y * npc.y < 0.0) continue;
            if (std::fabs(ev.y - npc.y) > 10.0) continue;

            // 红灯/黄灯：需要减速
            if (ev.type == EntityType::TrafficLight) {
                if (ev.phase_state == (int)TLPhase::Red ||
                    ev.phase_state == (int)TLPhase::Yellow) {
                    if (dx < nearest_block_s) {
                        nearest_block_s = dx;
                        should_stop = (ev.phase_state == (int)TLPhase::Red ||
                                       ev.phase_state == (int)TLPhase::Yellow);
                    }
                }
            }
            // ETC 闸门：NPC 不响应闸栏杆
        }

        if (should_stop) {
            double brake_dist = npc.speed * npc.speed / (2.0 * npc.max_brake);
            if (nearest_block_s < brake_dist + 5.0) {
                NpcTransitionRequest req;
                req.event = NpcEvent::TL_Red;
                npc_request_state(npc, req, cfg);
            }
        } else if (npc.state == NpcState::StopForTL) {
            // 前方无红灯/黄灯 → 恢复巡航
            NpcTransitionRequest req;
            req.event = NpcEvent::TL_Green;
            npc_request_state(npc, req, cfg);
        }
    }
}

/* ── 编舞循环：每 loop_period_s 重置 actor 到 ego 附近 ──
 *
 * 复用 scene_events.cpp:44 的 fmod(sim_time+offset, T) 循环范式。
 * 用 last_triggered_loop[] 跟踪每个 beat 在当前 loop 周期内是否已触发，
 * 避免每 tick 重复触发同一个 beat。
 *
 * 红绿灯 beat：actor="tl" 时直接设置所有红绿灯的 phase_state。
 */
void tick_choreography(EntityPool& pool, const Entity& ego,
                       double sim_time_s, double dt,
                       const Choreography* choreo,
                       FlowRoadNetwork* roads,
                       const Route* route) {
    (void)dt;
    if (!choreo || !choreo->enabled || choreo->beat_count <= 0) return;
    if (choreo->loop_period_s <= 0.0) return;

    double T = choreo->loop_period_s;
    double phase = std::fmod(sim_time_s, T);
    if (phase < 0.0) phase += T;
    int current_loop = (int)(sim_time_s / T);

    /* 静态 last_triggered_loop 数组，跟踪每个 beat 上次触发的 loop 编号 */
    static int last_triggered[SCENARIO_MAX_CHOREO_BEATS];
    static bool init_done = false;
    if (!init_done) {
        for (int i = 0; i < SCENARIO_MAX_CHOREO_BEATS; ++i)
            last_triggered[i] = -1;
        init_done = true;
    }

    for (int i = 0; i < choreo->beat_count; ++i) {
        if (last_triggered[i] >= current_loop) continue;  /* 本轮已触发 */

        const ChoreoBeat* b = &choreo->beats[i];
        if (phase < b->t) continue;  /* 还没到节拍时间 */

        last_triggered[i] = current_loop;

        /* ── 红绿灯 beat ── */
        if (strcmp(b->actor, "tl") == 0) {
            int new_phase = (int)TLPhase::Green;  /* 默认 */
            if (b->phase[0]) {
                if (strcmp(b->phase, "red") == 0)    new_phase = (int)TLPhase::Red;
                else if (strcmp(b->phase, "yellow") == 0) new_phase = (int)TLPhase::Yellow;
                else if (strcmp(b->phase, "green") == 0)  new_phase = (int)TLPhase::Green;
            }
            for (int j = 0; j < pool.size(); ++j) {
                Entity& tl = pool[j];
                if (!tl.active || tl.type != EntityType::TrafficLight) continue;
                tl.phase_state = new_phase;
                /* 也重置相位计时器，让 tick_traffic_lights 后续继续推进 */
                if (new_phase == (int)TLPhase::Red) {
                    double red_s = tl.steer;
                    tl.phase_timer = red_s;  /* 红灯剩余时长 */
                    /* 将 fmod 偏移量设为使 phase 落在红灯段 */
                    tl.target_vx = -sim_time_s + T * (int)(sim_time_s / T);
                }
            }
            continue;
        }

        /* ── actor beat：解析 actor id ── */
        int actor_id = atoi(b->actor);
        if (actor_id <= 0 && b->actor[0] != '0') continue;

        /* 查找 pool 中 actor_id 匹配的实体 */
        Entity* target = nullptr;
        for (int j = 0; j < pool.size(); ++j) {
            Entity& e = pool[j];
            if (e.active && e.id == actor_id) { target = &e; break; }
        }
        if (!target) continue;

        /* ── 重置到 ego 前方 ──
         * dl 语义：相对 ego.y 的横向偏移。负值 = 向右（同向更外侧车道），
         * 正值 = 向左（靠近中心线/对向）。cutin 场景 NPC 应从右车道（dl<0）
         * 切入 ego 车道，而非从对向车道（dl>0）切入。
         *
         * 路面 clamp：ego 变道后 ego.y 可能已不在右车道（如 -5.25），
         * dl=-3.5 会把 NPC 放到 -8.75（路外）。clamp 到同向路面范围
         * [-same_dir_half, -0.3] 防止 NPC 被传送到路外或对向。 */
        double new_x = ego.x + b->ds;
        double new_y = ego.y + b->dl;
        if (roads && roads->loaded()) {
            FrenetPos fp_ego;
            if (roads->world_to_frenet(ego.x, ego.y, fp_ego)) {
                int total = roads->drivable_lane_count(fp_ego.road_id, fp_ego.s);
                if (total >= 2) {
                    int same_dir = total / 2;
                    if (same_dir < 1) same_dir = total;
                    double same_dir_half = same_dir * 3.5;
                    if (ego.y < 0.0) {
                        if (new_y < -same_dir_half + 0.5) new_y = -same_dir_half + 0.5;
                        if (new_y > -0.3) new_y = -0.3;
                    } else {
                        if (new_y >  same_dir_half - 0.5) new_y =  same_dir_half - 0.5;
                        if (new_y <  0.3) new_y =  0.3;
                    }
                }
            }
        }
        target->x = new_x;
        target->y = new_y;
        target->vx = b->vx;
        target->vy = 0.0;
        target->speed = std::fabs(b->vx);
        target->heading = 0.0;  /* 沿 +x 直行 */

        /* ── 同步 Frenet 状态 ──
         * choreography 传送 NPC 到新世界坐标后，必须同步其 Frenet 状态
         * (offset/route_s/road_pos)，否则下一帧 step_npc_vehicle 会用旧
         * Frenet 状态把 NPC 拉回原位（road_pos.world() 覆盖 x/y）。
         * 用 world_to_frenet 重新计算 (road_id, s, offset)，然后 reinit
         * road_pos 和 route_s。 */
        if (roads && roads->loaded()) {
            FrenetPos fp;
            if (roads->world_to_frenet(new_x, new_y, fp)) {
                target->road_id = fp.road_id;
                target->s       = fp.s;
                /* fp.offset 是车道内偏移，需换算成相对参考线的横向偏移 */
                double lw = roads->lane_width(fp.road_id, fp.lane_id, fp.s);
                if (lw <= 0.0) lw = 3.5;
                double lane_center_t = (fp.lane_id > 0 ? 1.0 : -1.0)
                                     * (std::abs(fp.lane_id) - 0.5) * lw;
                target->offset = lane_center_t + fp.offset;
                /* 非 cutin 时 target_offset = offset（保持当前车道） */
                /* cutin 的 target_offset 在下方动作块设置 */
                /* reinit road_pos 到新位置 */
                if (target->road_pos.ok()) {
                    target->road_pos.init(*roads, fp.road_id, 0, fp.s, target->offset);
                }
                /* 同步 route_s */
                if (route && route->ok()) {
                    int ei = route->index_of(fp.road_id);
                    if (ei >= 0) {
                        target->route_s = route->to_route_s(ei, fp.s);
                    }
                }
            }
        }

        /* ── 动作：统一状态转移 ── */
        NpcTransitionRequest req;
        if (b->act[0]) {
            if (strcmp(b->act, "overtake") == 0) {
                req.event = NpcEvent::ChoreoOvertake;
                req.target_vx = b->vx;
                req.target_offset = target->offset;
            } else if (strcmp(b->act, "cutin") == 0) {
                req.event = NpcEvent::ChoreoCutIn;
                double cutin_target = ego.y;
                if (target->route_dir > 0 && cutin_target > -0.3) cutin_target = -0.3;
                if (target->route_dir < 0 && cutin_target <  0.3) cutin_target =  0.3;
                req.target_offset = cutin_target;
                req.target_vx = b->vx;
            }
        } else {
            /* 无 act：仅重置位置速度 */
            req.event = NpcEvent::ChoreoOvertake;
            req.target_vx = b->vx;
            req.target_offset = target->offset;
        }
        /* 统一入口转移 NPC 状态 */
        {
            NpcAiConfig choreo_cfg;  /* 使用默认配置 */
            npc_request_state(*target, req, choreo_cfg);
        }

        LOG_INFO("flowsim", "choreo beat t=%.1f actor=%d pos=(%.1f,%.1f) offset=%.2f vx=%.1f act='%s' (loop=%d)",
                 b->t, actor_id, new_x, new_y, target->offset, b->vx, b->act, current_loop);
    }
}

}  // namespace flowsim
