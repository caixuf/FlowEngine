/**
 * VehicleLights.js — 车灯位掩码 + 纯状态推导 + THREE 灯光网格
 *
 * Step 5 重构：从 VehicleView.js 抽出。
 *  - deriveLightState: 纯函数，无 THREE 依赖，便于单元测试
 *  - createVehicleLights: THREE 灯光网格工厂（供 VehicleView 用）
 *
 * 车灯位掩码（与 flowsim/vehicle_lights.h 一致）：
 *   bit0=左转 0x01, bit1=右转 0x02, bit2=双闪 0x04,
 *   bit3=远光 0x08, bit4=近光 0x10, bit6=倒车 0x40, bit7=雾灯 0x80
 * 刹车灯由 brake 字段驱动（不在 lights 位掩码里）。
 */

import * as THREE from 'three';

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

// ═══════════════════════════════════════════════════════════
// createVehicleLights — THREE 灯光网格工厂
// ═══════════════════════════════════════════════════════════

const LIGHT_OFF = new THREE.Color(0x111111);
const LIGHT_BRAKE_ON = new THREE.Color(0xff0000);
const LIGHT_TURN_ON = new THREE.Color(0xff8800);
const LIGHT_HEAD_ON = new THREE.Color(0xffffcc);

const BRAKE_Y = 0.55;      // 尾灯高度
const BRAKE_Z = -2.0;      // 车尾
const BRAKE_X = 0.65;      // 左右间距
const TURN_X = 0.75;
const TURN_Z = -1.95;
const HEAD_Y = 0.45;       // 前灯高度
const HEAD_Z = 2.05;       // 车头
const HEAD_X = 0.60;

const GEO_RECT = new THREE.PlaneGeometry(0.18, 0.10);

function _makeRectMesh(color, x, y, z) {
  const mat = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.9 });
  const m = new THREE.Mesh(GEO_RECT, mat);
  m.position.set(x, y, z);
  return m;
}

/**
 * 为车辆模型创建灯光网格组。
 * @param {THREE.Group} vehicleGroup 车辆模型根节点
 * @returns {{group: THREE.Group, update: (v: object) => void}}
 */
export function createVehicleLights(vehicleGroup) {
  const group = new THREE.Group();

  const brakeL = _makeRectMesh(LIGHT_OFF, -BRAKE_X, BRAKE_Y, BRAKE_Z);
  const brakeR = _makeRectMesh(LIGHT_OFF,  BRAKE_X, BRAKE_Y, BRAKE_Z);
  const turnL  = _makeRectMesh(LIGHT_OFF, -TURN_X,  BRAKE_Y, TURN_Z);
  const turnR  = _makeRectMesh(LIGHT_OFF,  TURN_X,  BRAKE_Y, TURN_Z);
  const headL  = _makeRectMesh(LIGHT_OFF, -HEAD_X,  HEAD_Y,  HEAD_Z);
  const headR  = _makeRectMesh(LIGHT_OFF,  HEAD_X,  HEAD_Y,  HEAD_Z);

  group.add(brakeL, brakeR, turnL, turnR, headL, headR);

  return {
    group,
    update(v) {
      const s = deriveLightState(v.lights || 0, v.brake || 0);
      brakeL.material.color.copy(s.brake ? LIGHT_BRAKE_ON : LIGHT_OFF);
      brakeR.material.color.copy(s.brake ? LIGHT_BRAKE_ON : LIGHT_OFF);
      turnL.material.color.copy(s.turnL ? LIGHT_TURN_ON : LIGHT_OFF);
      turnR.material.color.copy(s.turnR ? LIGHT_TURN_ON : LIGHT_OFF);
      headL.material.color.copy(s.head ? LIGHT_HEAD_ON : LIGHT_OFF);
      headR.material.color.copy(s.head ? LIGHT_HEAD_ON : LIGHT_OFF);
    }
  };
}
