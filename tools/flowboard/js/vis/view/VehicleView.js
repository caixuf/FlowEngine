/**
 * VehicleView.js — 车辆渲染（gltf 优先 + 程序化 fallback）
 *
 * 车灯位掩码（vehicle_lights.h）：
 *   bit0=左转 0x01, bit1=右转 0x02, bit2=双闪 0x04,
 *   bit3=远光 0x08, bit4=近光 0x10, bit6=倒车 0x40, bit7=雾灯 0x80
 * 刹车灯由 brake 字段驱动（不在 lights 位掩码里）。
 *
 * 资产来源：
 *   - gltf 模型（models.js 加载 su7/sedan/suv/truck/pedestrian）
 *   - 程序化几何体（AssetFactory BoxGeometry/CylinderGeometry）
 *
 * gltf 模型节点约定（gen_models.py 生成）：
 *   wheel_FL/FR/RL/RR + axle_front/rear — 车轮 + 前轴转向
 *   brakelight_L/R + brakelight_bar — 刹车灯 + 贯穿式尾灯（SU7）
 *   turnsignal_FL/FR/RL/RR — 转向灯
 *   headlight_L/R — 大灯
 *   ads_indicator_L/R — 自动驾驶小蓝灯（常亮）
 */

import { getBox, getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { worldToThree, headingToRotationY } from '../math/Coord.js';
import { initModelCache, getModel, _setVehicleLights } from '../../models.js';
import { _buildContactShadow } from '../../utils.js';
// Step 5 重构：纯逻辑（LIGHT_* + deriveLightState）抽到 VehicleLights.js，
// 零 THREE 依赖，便于 tests/vis_vehicle_lights.test.mjs 直接 import。
import {
  LIGHT_TURN_LEFT, LIGHT_TURN_RIGHT, LIGHT_HAZARD,
  LIGHT_HIGH_BEAM, LIGHT_LOW_BEAM, LIGHT_REVERSE,
  deriveLightState,
} from './VehicleLights.js';

// 向后兼容：原调用方从 VehicleView import 这些符号，保持不变。
export {
  LIGHT_TURN_LEFT, LIGHT_TURN_RIGHT, LIGHT_HAZARD,
  LIGHT_HIGH_BEAM, LIGHT_LOW_BEAM, LIGHT_REVERSE,
  deriveLightState,
};

// 程序化车灯颜色
const COLOR_HEAD_LOW  = 0xfff4d6;
const COLOR_HEAD_HIGH = 0xffffff;
const COLOR_TAIL      = 0xff2020;
const COLOR_TURN      = 0xffaa20;
const COLOR_REVERSE   = 0xffffff;

// 程序化车身颜色（按 type）
const BODY_COLORS = {
  ego: 0x1A528C,       // ego：海湾蓝（与 SU7 一致）
  car: 0x58a6ff,
  suv: 0xbc8cff,
  truck: 0xd29922,
  default: 0x8b949e,
};

// type → gltf 模型名映射
const GLTF_TYPE_MAP = {
  ego: 'su7',        // ego 用 SU7
  car: 'sedan',
  suv: 'suv',
  truck: 'truck',
};

let _gltfReady = false;
let _gltfInitStarted = false;

/** 异步预加载 gltf 模型（main.js init3DScene 时调一次） */
export function initModels() {
  if (_gltfInitStarted) return;
  _gltfInitStarted = true;
  initModelCache().then(() => {
    _gltfReady = true;
    console.log('[vis] gltf models ready');
  }).catch(err => {
    console.warn('[vis] gltf models load failed, fallback to procedural:', err.message);
  });
}

export function createVehicleView(scene) {
  const pool = new Map();
  let _upgradedToGltf = false;   // gltf 异步就绪后是否已把程序化车升级过一次

  // ═══════════════════════════════════════════════════════
  // gltf 车辆构建
  // ═══════════════════════════════════════════════════════

  function _createGltfVehicle(ent) {
    const type = ent.type || 'car';
    const modelName = GLTF_TYPE_MAP[type] || 'sedan';
    const model = getModel(modelName);
    if (!model) return null;

    return model;
  }

  function _updateGltfVehicle(entry, ent, simTime) {
    const { group } = entry;

    // 位姿（Step 3 重构：用 Coord.worldToThree 统一 ENU→THREE 映射，
    // 不再内联 position.set(ent.x, ent.z||0, ent.y)）
    group.position.set(...worldToThree(ent.x, ent.y, ent.z || 0));
    group.rotation.y = headingToRotationY(ent.heading || 0);

    // ── 车轮自转 + 转向平滑 ──
    const speed = ent.speed || 0;
    const WHEEL_RADIUS = 0.35;  // glTF 模型车轮半径 ≈ 0.35m
    const dt = 0.05;

    if (entry._wheelSpin === undefined) entry._wheelSpin = 0;
    entry._wheelSpin += speed * dt / WHEEL_RADIUS;

    // 转向平滑
    const steer = ent.steer || 0;
    const maxSteerRad = 0.35;
    const targetSteer = Math.max(-maxSteerRad, Math.min(maxSteerRad, steer * 1.5));
    if (entry._smoothSteer === undefined) entry._smoothSteer = 0;
    const STEER_LERP = 0.3;
    entry._smoothSteer += (targetSteer - entry._smoothSteer) * STEER_LERP;

    // 前轮转向：旋转 axle_front 节点
    if (group.userData.frontAxle && group.userData.frontAxle.rotation) {
      group.userData.frontAxle.rotation.y = entry._smoothSteer;
    }

    // 车轮自转：遍历 wheel_* 节点，绕其本地 Z 轴旋转（glTF cylinder axis=Z）
    if (entry._wheelNodes === undefined) {
      entry._wheelNodes = [];
      group.traverse(function(c) {
        if (c.name && c.name.indexOf('wheel_') === 0) {
          entry._wheelNodes.push(c);
        }
      });
    }
    for (let w = 0; w < entry._wheelNodes.length; w++) {
      entry._wheelNodes[w].rotation.z = entry._wheelSpin;
    }

    // 车灯状态（Step 5 重构：调 deriveLightState 纯函数，避免重复逻辑）
    const state = deriveLightState(ent.lights || 0, ent.brake || 0);
    // 闪烁相位（1.5Hz，与 _setVehicleLights 一致）
    const blinkPhase = simTime / 1000;
    _setVehicleLights(group, state, blinkPhase);
  }

  // ═══════════════════════════════════════════════════════
  // 程序化车辆构建（fallback）
  // ═══════════════════════════════════════════════════════

  function _createProceduralVehicle(ent) {
    const group = new THREE.Group();
    const len = ent.length || 4.6;
    const wid = ent.width || 2.0;
    const bodyH = 1.4;
    const type = ent.type || 'car';
    const bodyColor = BODY_COLORS[type] || BODY_COLORS.default;

    // 车身
    const bodyGeo = getBox(len, bodyH, wid);
    const bodyMat = getStdMaterial(bodyColor, 0.6, 0.1);
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.position.y = 0.7 + 0.10;
    body.castShadow = true;
    body.receiveShadow = true;
    group.add(body);

    // 车窗
    const cabinLen = len * 0.55;
    const cabinGeo = getBox(cabinLen, 0.7, wid * 0.95);
    const cabinMat = getStdMaterial(0x1a1a2a, 0.2, 0.5);
    const cabin = new THREE.Mesh(cabinGeo, cabinMat);
    cabin.position.set(-len * 0.05, 0.7 + bodyH * 0.5 + 0.35, 0);
    cabin.castShadow = true;
    group.add(cabin);

    // 4 车轮（InstancedMesh，1 draw call）
    const wheelGeo = new THREE.CylinderGeometry(0.32, 0.32, 0.25, 12);
    const wheelMat = getStdMaterial(0x0a0a0a, 0.85, 0.05);
    const wheelPositions = [
      [ len * 0.32, 0.32 + 0.10,  wid * 0.5],
      [ len * 0.32, 0.32 + 0.10, -wid * 0.5],
      [-len * 0.32, 0.32 + 0.10,  wid * 0.5],
      [-len * 0.32, 0.32 + 0.10, -wid * 0.5],
    ];
    const wheelInst = new THREE.InstancedMesh(wheelGeo, wheelMat, 4);
    wheelInst.rotation.x = Math.PI / 2;
    const dummy = new THREE.Object3D();
    for (let i = 0; i < 4; i++) {
      dummy.position.set(wheelPositions[i][0], wheelPositions[i][1], wheelPositions[i][2]);
      dummy.updateMatrix();
      wheelInst.setMatrixAt(i, dummy.matrix);
    }
    group.add(wheelInst);
    const wheels = [wheelInst, wheelInst, wheelInst, wheelInst];

    // 车灯 mesh
    const lights = {};
    const lampSize = 0.18;
    const lampGeo = getBox(lampSize, lampSize, lampSize * 1.5);

    const headLMat = createEmissiveMaterial(COLOR_HEAD_LOW, 1.0);
    const headRMat = createEmissiveMaterial(COLOR_HEAD_LOW, 1.0);
    const headL = new THREE.Mesh(lampGeo, headLMat);
    const headR = new THREE.Mesh(lampGeo, headRMat);
    headL.position.set(len * 0.49, 0.7 + 0.1,  wid * 0.4);
    headR.position.set(len * 0.49, 0.7 + 0.1, -wid * 0.4);
    group.add(headL, headR);
    lights.headLMat = headLMat; lights.headRMat = headRMat;

    const tailLMat = createEmissiveMaterial(COLOR_TAIL, 0.5);
    const tailRMat = createEmissiveMaterial(COLOR_TAIL, 0.5);
    const tailL = new THREE.Mesh(lampGeo, tailLMat);
    const tailR = new THREE.Mesh(lampGeo, tailRMat);
    tailL.position.set(-len * 0.49, 0.7 + 0.1,  wid * 0.4);
    tailR.position.set(-len * 0.49, 0.7 + 0.1, -wid * 0.4);
    group.add(tailL, tailR);
    lights.tailLMat = tailLMat; lights.tailRMat = tailRMat;

    const turnGeo = getBox(0.15, 0.12, 0.2);
    const turnFLMat = createEmissiveMaterial(COLOR_TURN, 0.0);
    const turnFRMat = createEmissiveMaterial(COLOR_TURN, 0.0);
    const turnRLMat = createEmissiveMaterial(COLOR_TURN, 0.0);
    const turnRRMat = createEmissiveMaterial(COLOR_TURN, 0.0);
    const turnFL = new THREE.Mesh(turnGeo, turnFLMat);
    const turnFR = new THREE.Mesh(turnGeo, turnFRMat);
    const turnRL = new THREE.Mesh(turnGeo, turnRLMat);
    const turnRR = new THREE.Mesh(turnGeo, turnRRMat);
    turnFL.position.set( len * 0.49, 0.7 + 0.35,  wid * 0.45);
    turnFR.position.set( len * 0.49, 0.7 + 0.35, -wid * 0.45);
    turnRL.position.set(-len * 0.49, 0.7 + 0.35,  wid * 0.45);
    turnRR.position.set(-len * 0.49, 0.7 + 0.35, -wid * 0.45);
    group.add(turnFL, turnFR, turnRL, turnRR);
    lights.turnFLMat = turnFLMat; lights.turnFRMat = turnFRMat;
    lights.turnRLMat = turnRLMat; lights.turnRRMat = turnRRMat;

    const revMat = createEmissiveMaterial(COLOR_REVERSE, 0.0);
    const rev = new THREE.Mesh(lampGeo, revMat);
    rev.position.set(-len * 0.49, 0.7 + 0.3, 0);
    group.add(rev);
    lights.revMat = revMat;

    return { group, wheels, lights, useGltf: false };
  }

  function _updateProceduralVehicle(entry, ent, simTime) {
    const { group, wheels, lights } = entry;

    // 位姿（Step 3 重构：用 Coord.worldToThree 统一 ENU→THREE 映射，
    // 不再内联 position.set(ent.x, ent.z||0, ent.y)）
    group.position.set(...worldToThree(ent.x, ent.y, ent.z || 0));
    group.rotation.y = headingToRotationY(ent.heading || 0);

    // ── 车轮自转 + 转向平滑 ──
    const speed = ent.speed || 0;
    const WHEEL_RADIUS = 0.32;
    const dt = 0.05;  // 20Hz 仿真 ≈ 50ms 帧间隔，用固定步长避免帧率耦合

    // 车轮累计旋转角度（rad）
    if (entry._wheelSpin === undefined) entry._wheelSpin = 0;
    entry._wheelSpin += speed * dt / WHEEL_RADIUS;

    // 转向平滑：lerp 向目标值，避免跳变
    const steer = ent.steer || 0;
    const maxSteerRad = 0.35;
    const targetSteer = Math.max(-maxSteerRad, Math.min(maxSteerRad, steer * 1.5));
    if (entry._smoothSteer === undefined) entry._smoothSteer = 0;
    const STEER_LERP = 0.3;  // 每帧逼近 30%
    entry._smoothSteer += (targetSteer - entry._smoothSteer) * STEER_LERP;

    const len = ent.length || 4.6;
    const wid = ent.width || 2.0;
    const wheelInst = wheels[0];
    if (wheelInst) {
      const wheelPositions = [
        [ len * 0.32, 0.32 + 0.10,  wid * 0.5],   // FL
        [ len * 0.32, 0.32 + 0.10, -wid * 0.5],   // FR
        [-len * 0.32, 0.32 + 0.10,  wid * 0.5],   // RL
        [-len * 0.32, 0.32 + 0.10, -wid * 0.5],   // RR
      ];
      const dummy = new THREE.Object3D();
      for (let i = 0; i < 4; i++) {
        dummy.position.set(wheelPositions[i][0], wheelPositions[i][1], wheelPositions[i][2]);
        // CylinderGeometry 默认轴=Y，wheelInst.rotation.x=PI/2 让轴=Z。
        // 所以：rotation.y=steer(转向), rotation.z=spin(自转)
        dummy.rotation.set(0, (i < 2) ? entry._smoothSteer : 0, entry._wheelSpin);
        dummy.updateMatrix();
        wheelInst.setMatrixAt(i, dummy.matrix);
      }
      wheelInst.instanceMatrix.needsUpdate = true;
    }

    // ── 车灯（统一用 deriveLightState + _setVehicleLights 逻辑）──
    const state = deriveLightState(ent.lights || 0, ent.brake || 0);
    const blinkPhase = simTime / 1000;

    // 刹车灯强度渐变
    if (entry._brakeIntensity === undefined) entry._brakeIntensity = 0;
    const targetBrake = state.brake ? 2.5 : 0.4;
    const BRAKE_FADE = 0.4;  // 刹车灯渐变速度
    entry._brakeIntensity += (targetBrake - entry._brakeIntensity) * BRAKE_FADE;
    lights.tailLMat.emissiveIntensity = entry._brakeIntensity;
    lights.tailRMat.emissiveIntensity = entry._brakeIntensity;

    // 转向灯：1.5Hz 正弦闪烁，与 glTF _setVehicleLights 一致
    const blinkOn = Math.sin(blinkPhase * Math.PI * 2 * 1.5) > 0;
    const turnIntensity = blinkOn ? 1.5 : 0.0;

    // 大灯
    const mask = ent.lights || 0;
    const lowBeam   = mask & LIGHT_LOW_BEAM;
    const highBeam  = mask & LIGHT_HIGH_BEAM;
    if (highBeam) {
      lights.headLMat.emissive.setHex(COLOR_HEAD_HIGH);
      lights.headRMat.emissive.setHex(COLOR_HEAD_HIGH);
      lights.headLMat.emissiveIntensity = 2.0;
      lights.headRMat.emissiveIntensity = 2.0;
    } else if (lowBeam) {
      lights.headLMat.emissive.setHex(COLOR_HEAD_LOW);
      lights.headRMat.emissive.setHex(COLOR_HEAD_LOW);
      lights.headLMat.emissiveIntensity = 1.2;
      lights.headRMat.emissiveIntensity = 1.2;
    } else {
      lights.headLMat.emissiveIntensity = 0.0;
      lights.headRMat.emissiveIntensity = 0.0;
    }

    const turnLeft  = state.turnL;
    const turnRight = state.turnR;
    lights.turnFLMat.emissiveIntensity = turnLeft  ? turnIntensity : 0.0;
    lights.turnRLMat.emissiveIntensity = turnLeft  ? turnIntensity : 0.0;
    lights.turnFRMat.emissiveIntensity = turnRight ? turnIntensity : 0.0;
    lights.turnRRMat.emissiveIntensity = turnRight ? turnIntensity : 0.0;

    const reverse = mask & LIGHT_REVERSE;
    lights.revMat.emissiveIntensity = reverse ? 1.5 : 0.0;
  }

  // ═══════════════════════════════════════════════════════
  // 统一入口
  // ═══════════════════════════════════════════════════════

  function _createVehicle(ent) {
    let entry;
    // gltf 优先（ego 用 SU7，其他用 sedan/suv/truck）
    if (_gltfReady) {
      const gltfModel = _createGltfVehicle(ent);
      if (gltfModel) {
        scene.add(gltfModel);
        entry = { group: gltfModel, wheels: [], lights: null, useGltf: true };
      }
    }
    if (!entry) {
      // fallback 程序化
      entry = _createProceduralVehicle(ent);
      scene.add(entry.group);
    }

    // 车底接触阴影（gltf + 程序化都加）
    // 尺寸按车身长宽，沿用 utils.js 默认 opacity 0.75
    const len = ent.length || 4.6;
    const wid = ent.width || 2.0;
    const shadow = _buildContactShadow(len * 1.05, wid * 1.15);
    entry.group.add(shadow);  // 作为车的子节点，跟随位姿
    entry.shadow = shadow;
    return entry;
  }

  function _updateVehicle(entry, ent, simTime) {
    if (entry.useGltf) {
      _updateGltfVehicle(entry, ent, simTime);
    } else {
      _updateProceduralVehicle(entry, ent, simTime);
    }
    // shadow 是 group 子节点，position/rotation 自动跟随，无需手动同步
  }

  /** 主更新入口：diff 同步 entity 池 */
  function update(store, simTime) {
    // gltf 车辆模型异步加载：首帧建车时 _gltfReady 多半还是 false，ego 被建成
    // 程序化 box。等 gltf 就绪后清空一次池，让所有车重建为 gltf（su7 等）。
    // 一次性，之后不再重建，避免每帧抖动。
    if (_gltfReady && !_upgradedToGltf) {
      _upgradedToGltf = true;
      clear();
    }

    const all = [];
    if (store.ego) {
      all.push({ id: 'ego', type: 'ego', ...store.ego });
    }
    if (store.entities) {
      for (const e of store.entities) {
        const t = e.type;
        if (t === 'ego' || t === 'car' || t === 'suv' || t === 'truck') {
          all.push(e);
        }
      }
    }

    // 删除消失的
    const aliveIds = new Set(all.map(e => e.id));
    for (const [id, entry] of pool.entries()) {
      if (!aliveIds.has(id)) {
        scene.remove(entry.group);
        pool.delete(id);
      }
    }

    // 创建/更新。
    // 注意：gltf 模型异步加载，加载完成前用程序化 fallback。
    //       加载完成后，已存在的程序化车不会自动升级为 gltf（避免抖动），
    //       新创建的车才会用 gltf。场景刷新（hash 变）时 vehicleView 不重建，
    //       所以如需强制升级 gltf，需要 clear() 重建。
    for (const ent of all) {
      let entry = pool.get(ent.id);
      if (!entry) {
        entry = _createVehicle(ent);
        pool.set(ent.id, entry);
      }
      _updateVehicle(entry, ent, simTime);
    }
  }

  /** 清空所有车辆（强制升级 gltf 时调） */
  function clear() {
    for (const [, entry] of pool) {
      scene.remove(entry.group);
    }
    pool.clear();
  }

  function getVehicleCount() { return pool.size; }

  return { update, clear, getVehicleCount };
}
