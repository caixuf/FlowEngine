/**
 * vis_vehicle_lights.test.mjs — 车灯位掩码冒烟测试（纯 Node，无浏览器）
 *
 * 验证 VehicleView.deriveLightState(mask, brake) 的位掩码语义：
 *   bit0=左转 0x01, bit1=右转 0x02, bit2=双闪 0x04,
 *   bit3=远光 0x08, bit4=近光 0x10, bit6=倒车 0x40
 * 刹车灯由 brake 字段驱动（不在 lights 位掩码里）。
 *
 * 跑法：node tests/vis_vehicle_lights.test.mjs
 */
import {
  deriveLightState,
  LIGHT_TURN_LEFT,
  LIGHT_TURN_RIGHT,
  LIGHT_HAZARD,
  LIGHT_HIGH_BEAM,
  LIGHT_LOW_BEAM,
  LIGHT_REVERSE,
} from '../tools/flowboard/js/vis/view/VehicleLights.js';

let pass = 0, fail = 0;
function check(name, actual, expected) {
  const ok = actual === expected;
  if (ok) { pass++; console.log('  PASS  ' + name); }
  else { fail++; console.log('  FAIL  ' + name + '  actual=' + JSON.stringify(actual) + '  expected=' + JSON.stringify(expected)); }
}
function checkDeep(name, actual, expected) {
  const ok = JSON.stringify(actual) === JSON.stringify(expected);
  if (ok) { pass++; console.log('  PASS  ' + name); }
  else { fail++; console.log('  FAIL  ' + name + '  actual=' + JSON.stringify(actual) + '  expected=' + JSON.stringify(expected)); }
}

console.log('--- bitmask constants（与 flowsim/vehicle_lights.h 一致） ---');
check('LIGHT_TURN_LEFT  = 0x01', LIGHT_TURN_LEFT,  0x01);
check('LIGHT_TURN_RIGHT = 0x02', LIGHT_TURN_RIGHT, 0x02);
check('LIGHT_HAZARD     = 0x04', LIGHT_HAZARD,     0x04);
check('LIGHT_HIGH_BEAM  = 0x08', LIGHT_HIGH_BEAM,  0x08);
check('LIGHT_LOW_BEAM   = 0x10', LIGHT_LOW_BEAM,   0x10);
check('LIGHT_REVERSE    = 0x40', LIGHT_REVERSE,    0x40);

console.log('\n--- deriveLightState 全暗 ---');
checkDeep('mask=0 brake=0 → 全 false',
  deriveLightState(0, 0),
  { brake: false, turnL: false, turnR: false, head: false });

console.log('\n--- brake 阈值（>0.05）---');
checkDeep('brake=0.05 边界不触发',
  deriveLightState(0, 0.05),
  { brake: false, turnL: false, turnR: false, head: false });
checkDeep('brake=0.06 触发',
  deriveLightState(0, 0.06),
  { brake: true, turnL: false, turnR: false, head: false });

console.log('\n--- 单独位 ---');
checkDeep('LIGHT_TURN_LEFT → turnL only',
  deriveLightState(LIGHT_TURN_LEFT, 0),
  { brake: false, turnL: true, turnR: false, head: false });
checkDeep('LIGHT_TURN_RIGHT → turnR only',
  deriveLightState(LIGHT_TURN_RIGHT, 0),
  { brake: false, turnL: false, turnR: true, head: false });
checkDeep('LIGHT_HAZARD → turnL AND turnR',
  deriveLightState(LIGHT_HAZARD, 0),
  { brake: false, turnL: true, turnR: true, head: false });
checkDeep('LIGHT_LOW_BEAM → head',
  deriveLightState(LIGHT_LOW_BEAM, 0),
  { brake: false, turnL: false, turnR: false, head: true });
checkDeep('LIGHT_HIGH_BEAM → head',
  deriveLightState(LIGHT_HIGH_BEAM, 0),
  { brake: false, turnL: false, turnR: false, head: true });

console.log('\n--- LIGHT_REVERSE 不映射到 state（state.head 不含 reverse）---');
checkDeep('LIGHT_REVERSE → state 全 false（reverse 由 procedural 路径单独处理）',
  deriveLightState(LIGHT_REVERSE, 0),
  { brake: false, turnL: false, turnR: false, head: false });

console.log('\n--- 组合位 ---');
checkDeep('TURN_LEFT | LOW_BEAM → turnL + head',
  deriveLightState(LIGHT_TURN_LEFT | LIGHT_LOW_BEAM, 0),
  { brake: false, turnL: true, turnR: false, head: true });
checkDeep('HAZARD | HIGH_BEAM | brake → turnL+turnR+head+brake',
  deriveLightState(LIGHT_HAZARD | LIGHT_HIGH_BEAM, 0.5),
  { brake: true, turnL: true, turnR: true, head: true });

console.log('\n--- 全位 OR ---');
checkDeep('all bits OR → brake off, turnL+turnR+head on（reverse 仍不进 state）',
  deriveLightState(
    LIGHT_TURN_LEFT | LIGHT_TURN_RIGHT | LIGHT_HAZARD |
    LIGHT_HIGH_BEAM | LIGHT_LOW_BEAM | LIGHT_REVERSE, 0),
  { brake: false, turnL: true, turnR: true, head: true });

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
process.exit(fail > 0 ? 1 : 0);
