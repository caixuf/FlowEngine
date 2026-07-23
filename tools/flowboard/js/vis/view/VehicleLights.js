/**
 * VehicleLights.js — 车灯位掩码 + 纯状态推导（零 THREE 依赖，便于单元测试）
 *
 * Step 5 重构：从 VehicleView.js 抽出。
 *  - deriveLightState: 纯函数，无 THREE 依赖，便于单元测试
 *  - createVehicleLights: 在 VehicleView.js 中实现（需 THREE）
 *
 * 车灯位掩码（与 flowsim/vehicle_lights.h 一致）：
 *   bit0=左转 0x01, bit1=右转 0x02, bit2=双闪 0x04,
 *   bit3=远光 0x08, bit4=近光 0x10, bit6=倒车 0x40, bit7=雾灯 0x80
 * 刹车灯由 brake 字段驱动（不在 lights 位掩码里）。
 */

export const LIGHT_TURN_LEFT  = 0x01;
export const LIGHT_TURN_RIGHT = 0x02;
export const LIGHT_HAZARD     = 0x04;
export const LIGHT_HIGH_BEAM  = 0x08;
export const LIGHT_LOW_BEAM   = 0x10;
export const LIGHT_REVERSE    = 0x40;

/** 车灯位掩码 → 渲染 state 对象（纯函数，无 THREE 依赖）
 *  - brake 由 brake 字段驱动（不在 lights 位掩码里）
 *  - hazard（双闪）让左右转向灯同时亮
 *  - head 由 low_beam 或 high_beam 触发
 *  @param {number} mask  vehicle_lights.h 的位掩码
 *  @param {number} brake  brake 字段（>0.05 触发刹车灯）
 *  @returns {{brake:boolean, turnL:boolean, turnR:boolean, head:boolean}} */
export function deriveLightState(mask, brake) {
  return {
    brake: brake > 0.05,
    turnL: !!(mask & (LIGHT_TURN_LEFT | LIGHT_HAZARD)),
    turnR: !!(mask & (LIGHT_TURN_RIGHT | LIGHT_HAZARD)),
    head: !!(mask & (LIGHT_LOW_BEAM | LIGHT_HIGH_BEAM)),
  };
}
