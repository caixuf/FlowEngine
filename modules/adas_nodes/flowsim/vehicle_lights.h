/**
 * vehicle_lights.h — 车灯状态位掩码定义
 *
 * 用 1 字节位掩码表示车辆所有灯的状态，紧凑传输。
 * scene_pub 序列化为 JSON 整数，VehicleView 按位与判断亮灭。
 *
 * 位定义（bit7 → bit0）：
 *   bit 7  雾灯         fog_light        (保留位，未实现)
 *   bit 6  倒车灯       reverse_light    (保留位，未实现)
 *   bit 5  高位刹车灯   high_mount_brake (保留位，未实现)
 *   bit 4  近光灯       low_beam
 *   bit 3  远光灯       high_beam        (保留位，未实现)
 *   bit 2  双闪         hazard_warning
 *   bit 1  右转向灯     turn_right
 *   bit 0  左转向灯     turn_left
 *
 * 刹车灯不占位：由 Entity.brake 字段直接驱动（brake > 0.1 即亮），
 * 避免与转向灯位冲突，减少传输。
 *
 * P2 清理：移除从未被调用过的查询方法（turn_left/right/hazard/high_beam/
 * low_beam/high_mount/reverse/fog/headlight_on）和未被调用过的 setter
 * （set_high_beam/set_reverse/set_fog）。alive 路径：mask 直接由
 * scene_pub 序列化为整数传给前端；setter 仅留 set_turn_left/right、
 * set_hazard、set_low_beam；aggregate 查询仅留 any_turn。高位刹车/倒车/
 * 雾灯的位常量保留作为协议文档（前端 LIGHT_REVERSE 仍按 0x40 测试）。
 */

#ifndef FLOWSIM_VEHICLE_LIGHTS_H
#define FLOWSIM_VEHICLE_LIGHTS_H

#include <cstdint>

namespace flowsim {

// 车灯位掩码常量（位常量保留作为协议文档，未实现的位不会被任何 setter 写入）
namespace lights {
    constexpr uint8_t TURN_LEFT      = 0x01;  // bit 0
    constexpr uint8_t TURN_RIGHT     = 0x02;  // bit 1
    constexpr uint8_t HAZARD         = 0x04;  // bit 2
    constexpr uint8_t HIGH_BEAM      = 0x08;  // bit 3  (保留位)
    constexpr uint8_t LOW_BEAM       = 0x10;  // bit 4
    // bit 5 (HIGH_MOUNT 0x20) / bit 6 (REVERSE 0x40) / bit 7 (FOG 0x80)
    // 未实现，位值不删以保持协议位编号稳定（前端 LIGHT_REVERSE 仍按 0x40 测试）。
}

/**
 * 车灯状态。封装位掩码 + 便捷查询方法。
 * Entity 持有一个实例，control_node/actor 写入，scene_pub 读取序列化。
 */
struct VehicleLights {
    uint8_t mask{0};  ///< 位掩码，默认全灭

    /// 任一转向灯亮（左/右/双闪）—— VehicleActor 用于决定是否清零转向灯
    bool any_turn() const {
        return mask & (lights::TURN_LEFT | lights::TURN_RIGHT | lights::HAZARD);
    }

    // ── 设置（按 spec §5.3 规则由 control_node / actor 调用）──
    void set_turn_left(bool on)  { set_bit(lights::TURN_LEFT, on); }
    void set_turn_right(bool on) { set_bit(lights::TURN_RIGHT, on); }
    void set_hazard(bool on)     { set_bit(lights::HAZARD, on); }
    void set_low_beam(bool on)   { set_bit(lights::LOW_BEAM, on); }

    void clear() { mask = 0; }

private:
    void set_bit(uint8_t bit, bool on) {
        if (on) mask |= bit;
        else    mask &= ~bit;
    }
};

}  // namespace flowsim

#endif  // FLOWSIM_VEHICLE_LIGHTS_H
