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
#include "physics.h"
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
            tl.ai_state = AIState::Cruise;  // 绿灯：通行
        } else if (tp < green_s + yellow_s) {
            tl.phase_state = (int)TLPhase::Yellow;
            tl.ai_state = AIState::Yield;   // 黄灯：注意
        } else {
            tl.phase_state = (int)TLPhase::Red;
            tl.ai_state = AIState::Stop;    // 红灯：停
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

        if (dist > 50.0) {
            // 远距：关闭
            gate.ai_state = AIState::Stop;
            gate.phase_timer = 0.0;  // 抬杆进度 0
        } else if (dist > 10.0) {
            // 中距：开始抬杆
            gate.ai_state = AIState::Yield;
            gate.phase_timer = std::min(1.0, gate.phase_timer + dt * 0.5);  // 2s 抬完
        } else if (dist > 0) {
            // 近距：全开
            gate.ai_state = AIState::Cruise;
            gate.phase_timer = 1.0;
        } else {
            // 已通过：放下（但 ego 已过，不影响）
            gate.ai_state = AIState::Stop;
            gate.phase_timer = std::max(0.0, gate.phase_timer - dt * 0.5);
        }
    }
}

void check_npc_scene_events(EntityPool& pool, double look_ahead) {
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

            // 同向车流判定：TL.e.width 存的是车道中心 y（由 flowsim_node 从 y_lane +
            // road_center_y 计算），NPC.y 是世界坐标车道中心。两者 sign 一致表示
            // 同侧车流，才需要响应本侧红绿灯；异号是对向车流的灯，跳过。
            // 横向距离阈值放宽到 6.0m 覆盖杆位（5m）与最远同向车道中心（1.75m）
            // 的最大差。原 2.0m 阈值会过滤掉所有 TL（NPC 看不到红灯 → 闯灯撞 ego）。
            //
            // 修复：用 e.width（车道中心 y）替代 e.y（杆位 y）做同侧判断。
            // 杆位 y 在路缘外侧 ±(road_half_width+1.5)m，与 NPC.y（车道中心 ±1.75m）
            // 相距太远，sign 乘积可能因精度/坐标偏移出错，导致 NPC 看不到本侧红灯。
            double tl_lane_y = ev.width;  /* e.width = 车道中心 y */
            if (tl_lane_y * npc.y < 0.0) continue;
            if (std::fabs(ev.y - npc.y) > 10.0) continue;  /* 放宽到 10m，覆盖杆位到车道中心距离 */

            // 红灯/黄灯：需要减速
            if (ev.type == EntityType::TrafficLight) {
                if (ev.phase_state == (int)TLPhase::Red ||
                    ev.phase_state == (int)TLPhase::Yellow) {
                    if (dx < nearest_block_s) {
                        nearest_block_s = dx;
                        // 黄灯也减速停车（之前只 Red 置 true，整个 yellow_s=3s
                        // 阶段 NPC 零减速 → 接近路口时闯黄灯 → 红灯刚亮就冲
                        // 出去）。简单起见统一 should_stop=true；如需更精细控制
                        // 可再加 should_slow_down 标志把目标速度降到一半。
                        should_stop = (ev.phase_state == (int)TLPhase::Red ||
                                       ev.phase_state == (int)TLPhase::Yellow);
                    }
                }
            }
            // ETC 闸门：NPC 不响应闸栏杆，避免在路口远处排队把 ego 堵住。
            // ego 自己的 planning/safety 会在关闭的闸门前停车。
            // if (ev.type == EntityType::ETCGate) { ... }
        }

        if (should_stop) {
            // 按制动距离判断：v²/2a，提前减速
            // 不覆盖 target_vx——NPC AI 的 StopForTL 状态已使 v_desired=0，
            // 保留原始 target_vx 供绿灯后恢复巡航。
            double brake_dist = npc.speed * npc.speed / (2.0 * npc.max_brake);
            if (nearest_block_s < brake_dist + 5.0) {
                npc.ai_state = AIState::StopForTL;
            }
        } else if (npc.ai_state == AIState::StopForTL) {
            // 前方无红灯/黄灯 → 恢复巡航（绿灯已亮或已通过路口）
            npc.ai_state = AIState::Cruise;
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
                       const Choreography* choreo) {
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
                    double green_s = tl.throttle;
                    double yellow_s = tl.brake;
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

        /* ── 重置到 ego 前方 ── */
        double new_x = ego.x + b->ds;
        double new_y = ego.y + b->dl;
        target->x = new_x;
        target->y = new_y;
        target->vx = b->vx;
        target->vy = 0.0;
        target->speed = std::fabs(b->vx);
        target->heading = 0.0;  /* 沿 +x 直行 */

        /* ── 动作 ── */
        if (b->act[0]) {
            if (strcmp(b->act, "overtake") == 0) {
                target->ai_state = AIState::Cruise;
                target->target_vx = b->vx;
            } else if (strcmp(b->act, "cutin") == 0) {
                target->ai_state = AIState::CutIn;
                target->target_offset = ego.y;  /* cutin 到 ego 车道 */
                target->target_vx = b->vx;
                target->cutin_pid_integral = 0.0;
                target->cutin_pid_prev = 0.0;
                target->cutin_active = true;
            }
        } else {
            /* 无 act：仅重置位置速度，保持当前 AI 状态 */
            target->ai_state = AIState::Cruise;
            target->target_vx = b->vx;
        }

        LOG_INFO("flowsim", "choreo beat t=%.1f actor=%d pos=(%.1f,%.1f) vx=%.1f act='%s' (loop=%d)",
                 b->t, actor_id, new_x, new_y, b->vx, b->act, current_loop);
    }
}

}  // namespace flowsim
