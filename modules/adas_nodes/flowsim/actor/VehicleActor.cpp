/**
 * VehicleActor.cpp — 车辆行为 Actor 实现
 *
 * 从 Entity 状态派生车灯信号。纯函数式，无副作用（除写 Entity.lights）。
 */

#include "VehicleActor.h"
#include <cmath>

namespace flowsim {

/* ── 转向灯阈值 ── */
static constexpr double STEER_TURN_THRESHOLD = 0.1;  // |steer| > 0.1 rad ≈ 5.7° 触发转向灯
static constexpr double SPEED_REVERSE_THRESHOLD = 0.5;  // m/s，低于此速度判为近乎停止

/* ── 昼夜判断（简化版）──
 * sim_time_s 是仿真秒数，取模 86400 得到当天时间。
 * 18:00-06:00（64800s - 21600s 跨午夜）开近光灯。
 * 场景无真实昼夜循环时，这个逻辑可被 config 覆盖。 */
static bool is_night(double sim_time_s) {
    double time_of_day = std::fmod(sim_time_s, 86400.0);
    if (time_of_day < 0) time_of_day += 86400.0;
    return time_of_day >= 64800.0 || time_of_day < 21600.0;  // 18:00~06:00
}

void VehicleActor::update_ego_lights(Entity& ego, double sim_time_s) {
    /* 注意：ego.lights 的 turn_signal / hazard 已在 flowsim_node 主循环中
     * 从 ControlCmd（决策下发）设置，此处不覆盖、不清除。VehicleActor 只补充
     * 非决策类灯光：夜间近光灯、倒车灯等。 */

    /* 近光灯：夜间自动开启 */
    if (is_night(sim_time_s)) {
        ego.lights.set_low_beam(true);
    }

    /* 倒车灯：暂不启用（需要挡位 R 信号），保留接口。
     * 刹车灯不占 lights 位：VehicleView 直接读 ego.brake > 0.1 判断。 */
}

void VehicleActor::update_npc_lights(Entity& npc) {
    /* NPC 车灯根据 AI 状态派生。先清空再按状态设置。
     * 刹车灯同样由 VehicleView 读 npc.brake 字段直接驱动，不占 lights 位。 */
    npc.lights.clear();

    switch (npc.state) {
        case NpcState::CutIn:
        case NpcState::LaneChange:
            /* 变道：根据 target_offset 方向打转向灯 */
            if (npc.target_offset > npc.offset + 0.1) {
                npc.lights.set_turn_left(true);
            } else if (npc.target_offset < npc.offset - 0.1) {
                npc.lights.set_turn_right(true);
            }
            break;

        case NpcState::Yield:
            /* 让行：开双闪 */
            npc.lights.set_hazard(true);
            break;

        case NpcState::Stopped:
            /* 停止且近乎静止：开双闪 */
            if (npc.speed < SPEED_REVERSE_THRESHOLD) {
                npc.lights.set_hazard(true);
            }
            break;

        case NpcState::StopForTL:
            /* 红灯停车：不开双闪 */
            break;

        case NpcState::Cruise:
        case NpcState::Follow:
        default:
            /* 巡航/跟车：根据 steer 打转向灯 */
            if (npc.steer > STEER_TURN_THRESHOLD) {
                npc.lights.set_turn_right(true);
            } else if (npc.steer < -STEER_TURN_THRESHOLD) {
                npc.lights.set_turn_left(true);
            }
            break;
    }

    /* 夜间近光灯（NPC 也开）—— 用 ego 的 sim_time 判断不适用此处，
     * update_all_lights 会传 sim_time。简化：NPC 暂不判昼夜，只 ego 判。 */
}

void VehicleActor::update_all_lights(EntityPool& pool, double sim_time_s) {
    for (int i = 0; i < pool.size(); ++i) {
        Entity& e = pool[i];
        if (!e.active || !e.is_vehicle()) continue;

        if (e.type == EntityType::Ego) {
            update_ego_lights(e, sim_time_s);
        } else {
            update_npc_lights(e);
        }
    }
}

}  // namespace flowsim
