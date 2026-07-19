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

            // 同向车流判定：TL.entity.y 已被改为路缘外立柱位置（±5m 左右），
            // 而 NPC.y 在车道中心（±1.75m）。两者 sign 一致表示同侧路缘 /
            // 同向车流，才需要响应本侧红绿灯；异号是对向车流的灯，跳过。
            // 横向距离阈值放宽到 6.0m 覆盖杆位（5m）与最远同向车道中心（1.75m）
            // 的最大差。原 2.0m 阈值会过滤掉所有 TL（NPC 看不到红灯 → 闯灯撞 ego）。
            if (ev.y * npc.y < 0.0) continue;
            if (std::fabs(ev.y - npc.y) > 6.0) continue;

            // 红灯/黄灯：需要减速
            if (ev.type == EntityType::TrafficLight) {
                if (ev.phase_state == (int)TLPhase::Red ||
                    ev.phase_state == (int)TLPhase::Yellow) {
                    if (dx < nearest_block_s) {
                        nearest_block_s = dx;
                        should_stop = (ev.phase_state == (int)TLPhase::Red);
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

}  // namespace flowsim
