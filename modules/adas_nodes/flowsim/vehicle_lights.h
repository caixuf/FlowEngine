/**
 * vehicle_lights.h — 车灯状态位掩码定义
 *
 * 用 1 字节位掩码表示车辆所有灯的状态，紧凑传输。
 * scene_pub 序列化为 JSON 整数，VehicleView 按位与判断亮灭。
 *
 * 位定义（bit7 → bit0）：
 *   bit 7  雾灯         fog_light
 *   bit 6  倒车灯       reverse_light
 *   bit 5  高位刹车灯   high_mount_brake
 *   bit 4  近光灯       low_beam
 *   bit 3  远光灯       high_beam
 *   bit 2  双闪         hazard_warning
 *   bit 1  右转向灯     turn_right
 *   bit 0  左转向灯     turn_left
 *
 * 刹车灯不占位：由 Entity.brake 字段直接驱动（brake > 0.1 即亮），
 * 避免与转向灯位冲突，减少传输。
 */

#ifndef FLOWSIM_VEHICLE_LIGHTS_H
#define FLOWSIM_VEHICLE_LIGHTS_H

#include <cstdint>

namespace flowsim {

// 车灯位掩码常量
namespace lights {
    constexpr uint8_t TURN_LEFT      = 0x01;  // bit 0
    constexpr uint8_t TURN_RIGHT     = 0x02;  // bit 1
    constexpr uint8_t HAZARD         = 0x04;  // bit 2
    constexpr uint8_t HIGH_BEAM      = 0x08;  // bit 3
    constexpr uint8_t LOW_BEAM       = 0x10;  // bit 4
    constexpr uint8_t HIGH_MOUNT     = 0x20;  // bit 5
    constexpr uint8_t REVERSE        = 0x40;  // bit 6
    constexpr uint8_t FOG            = 0x80;  // bit 7
}  // namespace lights

/**
 * 车灯状态。封装位掩码 + 便捷查询方法。
 * Entity 持有一个实例，control_node/actor 写入，scene_pub 读取序列化。
 */
struct VehicleLights {
    uint8_t mask{0};  ///< 位掩码，默认全灭

    // ── 查询 ──
    bool turn_left() const      { return mask & lights::TURN_LEFT; }
    bool turn_right() const     { return mask & lights::TURN_RIGHT; }
    bool hazard() const         { return mask & lights::HAZARD; }
    bool high_beam() const      { return mask & lights::HIGH_BEAM; }
    bool low_beam() const       { return mask & lights::LOW_BEAM; }
    bool high_mount() const     { return mask & lights::HIGH_MOUNT; }
    bool reverse() const        { return mask & lights::REVERSE; }
    bool fog() const            { return mask & lights::FOG; }

    /// 任一转向灯亮（左/右/双闪）
    bool any_turn() const {
        return mask & (lights::TURN_LEFT | lights::TURN_RIGHT | lights::HAZARD);
    }

    /// 前灯亮（近光或远光）
    bool headlight_on() const {
        return mask & (lights::LOW_BEAM | lights::HIGH_BEAM);
    }

    // ── 设置（按 spec §5.3 规则由 control_node / actor 调用）──
    void set_turn_left(bool on)  { set_bit(lights::TURN_LEFT, on); }
    void set_turn_right(bool on) { set_bit(lights::TURN_RIGHT, on); }
    void set_hazard(bool on)     { set_bit(lights::HAZARD, on); }
    void set_low_beam(bool on)   { set_bit(lights::LOW_BEAM, on); }
    void set_high_beam(bool on)  { set_bit(lights::HIGH_BEAM, on); }
    void set_reverse(bool on)    { set_bit(lights::REVERSE, on); }
    void set_fog(bool on)        { set_bit(lights::FOG, on); }

    void clear() { mask = 0; }

private:
    void set_bit(uint8_t bit, bool on) {
        if (on) mask |= bit;
        else    mask &= ~bit;
    }
};

}  // namespace flowsim

#endif  // FLOWSIM_VEHICLE_LIGHTS_H
