/**
 * VehicleActor.h — 车辆行为 Actor（Layer 3）
 *
 * 从 Entity 已有状态（steer/brake/speed/ai_state）派生车灯信号，
 * 写入 Entity.lights（VehicleLights 位掩码）。
 *
 * 设计决策（方案 D）：
 *   不改 ControlRaw / control_node / actuator 的 cmd 链路。
 *   ControlRaw 是 msg_codegen.py 自动生成的二进制消息（DO NOT EDIT），
 *   改它需要重新 codegen + 改所有订阅节点。而车灯信号完全可以从 ego
 *   已有的 steer/brake/speed 派生，不需要额外的传输字段。
 *
 *   control_node 的灯信号决策逻辑下沉到此处，flowsim_node tick 末尾调用。
 *   NPC 的灯信号根据 ai_state（CutIn→转向灯，Stop→双闪，etc.）派生。
 *
 * 调用时机：flowsim_node tick 里 step_bicycle 之后、scene_pub 之前。
 */

#ifndef FLOWSIM_VEHICLE_ACTOR_H
#define FLOWSIM_VEHICLE_ACTOR_H

#include "../entity.h"

namespace flowsim {

/**
 * 车辆行为 Actor。纯静态方法，无实例状态。
 * 灯信号从 Entity 状态实时派生，不需要持久化计时器（转向灯随 steer 实时亮灭）。
 */
class VehicleActor {
public:
    /**
     * 更新 ego 车灯。在 step_bicycle 之后调用。
     * 规则（spec §5.3）：
     *   - steer > 0.1  → 右转向灯
     *   - steer < -0.1 → 左转向灯
     *   - brake > 0.1  → 刹车灯（由 VehicleView 直接读 brake 字段，不占 lights 位）
     *   - speed < 0.5 且倒车 → 倒车灯
     *   - sim_time 在夜间（18:00-06:00）→ 近光灯
     * @param ego         ego Entity（index 0）
     * @param sim_time_s  仿真时间（秒），用于判断昼夜
     */
    static void update_ego_lights(Entity& ego, double sim_time_s);

    /**
     * 更新 NPC 车灯。在 npc_ai tick 之后调用。
     * 规则：
     *   - ai_state==CutIn  → 根据 target_offset 方向打转向灯
     *   - ai_state==Merge  → 汇入方向打转向灯
     *   - ai_state==Yield  → 双闪（让行警示）
     *   - ai_state==Stop 且 speed<0.5 → 双闪（紧急停车）
     *   - 其余状态 → 转向灯灭（刹车灯仍由 brake 字段驱动）
     * @param npc  NPC Entity
     */
    static void update_npc_lights(Entity& npc);

    /**
     * 更新所有车辆实体的车灯。遍历 EntityPool，对每个 is_vehicle() 的实体
     * 调用对应的 update 方法。flowsim_node tick 末尾调用一次。
     * @param pool        实体池
     * @param sim_time_s  仿真时间（秒）
     */
    static void update_all_lights(EntityPool& pool, double sim_time_s);
};

}  // namespace flowsim

#endif  // FLOWSIM_VEHICLE_ACTOR_H
