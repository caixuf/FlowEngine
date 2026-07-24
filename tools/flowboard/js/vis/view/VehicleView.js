/**
 * VehicleView.js — 车辆 3D 渲染 + 动画
 *
 * P2 画质升级：
 *   - glTF 车辆：遍历 mesh 把车身材质升级为 MeshPhysicalMaterial（envMap 反射 + clearcoat）
 *   - 程序化 fallback：MeshPhysicalMaterial 清漆替代 MeshStandardMaterial
 *   - PMREMGenerator 生成环境反射贴图
 *
 * 两套渲染路径：
 *   1. glTF 模型（models/*.gltf）—— 优先，加载失败回退到 2
 *   2. 程序化 fallback（utils._buildSedan 等）—— 兼容 /api/topology 无 model 字段
 *
 * 不动现有的 wheelSpin / window slide / steering 动画逻辑。
 */

import { deriveLightState, LIGHT_TURN_LEFT, LIGHT_TURN_RIGHT, LIGHT_HAZARD, LIGHT_HIGH_BEAM, LIGHT_LOW_BEAM } from './VehicleLights.js';
import { getStdMaterial } from '../core/AssetFactory.js';
import { initModelCache, getModel } from '../../models.js';
import { worldToThree, headingToRotationY } from '../math/Coord.js';

// ═══════════════════════════════════════════════════════════
// createVehicleLights — THREE 灯光网格工厂（原在 VehicleLights.js，
// 移到此处以保持 VehicleLights.js 零 THREE 依赖，供 Node 单元测试）
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

/** 为车辆模型创建灯光网格组。
 *  @param {THREE.Group} vehicleGroup 车辆模型根节点
 *  @returns {{group: THREE.Group, update: (v: object) => void}} */
function createVehicleLights(vehicleGroup) {
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

// ═══════════════════════════════════════════════════════════
// 车漆参数（MeshPhysicalMaterial）
// ═══════════════════════════════════════════════════════════

const PAINT_CLEARCOAT = 0.9;
const PAINT_CLEARCOAT_ROUGHNESS = 0.15;
const PAINT_ROUGHNESS = 0.35;
const PAINT_METALNESS = 0.15;
const ENVMAP_INTENSITY = 0.9;

const BODY_KEYWORDS = ['body', 'car', 'paint', 'chassis', 'body_', 'Body', 'Chassis', 'CAR', 'Paint', 'Car', 'mesh_'];

// ═══════════════════════════════════════════════════════════
// 全局 envMap 缓存（PMREMGenerator）
// ═══════════════════════════════════════════════════════════

let _pmrem = null;
let _envMap = null;

/** 确保 envMap 已生成，返回共享的 envMap texture */
function _ensureEnvMap(renderer, scene) {
  if (_envMap) return _envMap;
  if (!renderer || !scene) return null;
  try {
    _pmrem = new THREE.PMREMGenerator(renderer);
    // 从场景渲染环境贴图（天空 + 光照）
    _envMap = _pmrem.fromScene(scene, 0.04).texture;
    _envMap.colorSpace = THREE.SRGBColorSpace;
  } catch (e) {
    console.warn('[VehicleView] PMREMGenerator failed, envMap unavailable:', e.message);
  }
  return _envMap;
}

/** 判断 mesh 是否是车身（需要车漆升级） */
function _isBodyMesh(mesh) {
  const name = mesh.name || '';
  return BODY_KEYWORDS.some(kw => name.includes(kw));
}

/** 把 mesh 的材质升级为车漆 MeshPhysicalMaterial */
function _upgradeToCarPaint(mesh, envMap) {
  if (!mesh.material) return;

  // 如果已经是 MeshPhysicalMaterial 且已设 clearcoat，跳过
  if (mesh.material.isMeshPhysicalMaterial && mesh.material.clearcoat > 0.5) return;

  // 保留原色
  const oldColor = mesh.material.color ? mesh.material.color.getHex() : 0xcccccc;
  const oldMetalness = mesh.material.metalness !== undefined ? mesh.material.metalness : PAINT_METALNESS;
  const oldRoughness = mesh.material.roughness !== undefined ? mesh.material.roughness : PAINT_ROUGHNESS;

  const mat = new THREE.MeshPhysicalMaterial({
    color: oldColor,
    metalness: oldMetalness,
    roughness: Math.min(oldRoughness, PAINT_ROUGHNESS),
    clearcoat: PAINT_CLEARCOAT,
    clearcoatRoughness: PAINT_CLEARCOAT_ROUGHNESS,
    envMap: envMap || null,
    envMapIntensity: ENVMAP_INTENSITY,
    depthWrite: true,
  });

  mesh.material.dispose();
  mesh.material = mat;
}

/** 遍历 glTF scene，给车身 mesh 上清漆 */
function _applyCarPaintToScene(gltfScene, envMap) {
  gltfScene.traverse((child) => {
    if (!child.isMesh) return;
    if (_isBodyMesh(child)) {
      _upgradeToCarPaint(child, envMap);
    }
  });
}

// ═══════════════════════════════════════════════════════════
// 主工厂
// ═══════════════════════════════════════════════════════════

/** 异步预加载 glTF 车辆模型（SU7/sedan/suv/truck）。
 *  加载完成前用程序化 fallback，完成后新车自动用 glTF。
 *  main.js 在 init3DScene 中调用，无需 await。 */
export function initModels() {
  initModelCache();
}

export function createVehicleView(scene, renderer, modelCache) {
  const vehicleMap = new Map();  // id → { group, lights, modelData, ... }
  let vehicleGroup = new THREE.Group();
  scene.add(vehicleGroup);

  // ── 辅助工具 ──

  /** 获取车辆的可视 group（兼容 glTF 包裹层） */
  function _getVisGroup(entry) {
    if (!entry) return null;
    if (entry.visGroup) return entry.visGroup;
    // glTF 路径：scene 对象本身可能就是个 group
    if (entry.modelData && entry.modelData.scene) {
      return entry.modelData.scene;
    }
    return entry.group;
  }

  /** 创建程序化 fallback（MeshPhysicalMaterial 车漆 + X-forward 朝向） */
  function _createFallbackVehicle(type, id) {
    const group = new THREE.Group();

    // 车型 → 车身颜色映射（与 gen_models.py 材质一致）
    const COLOR_MAP = {
      su7: 0x1a5288,    // 海湾蓝
      sedan: 0x2a6fc4,   // 深蓝
      truck: 0x4a4a4a,   // 深灰
      suv: 0x2d6b3a,     // 深绿
      car: 0x2a6fc4,
    };
    const bodyColor = COLOR_MAP[type] || 0xcccccc;

    const bodyMat = new THREE.MeshPhysicalMaterial({
      color: bodyColor,
      metalness: PAINT_METALNESS,
      roughness: PAINT_ROUGHNESS,
      clearcoat: PAINT_CLEARCOAT,
      clearcoatRoughness: PAINT_CLEARCOAT_ROUGHNESS,
      envMap: _envMap || null,
      envMapIntensity: ENVMAP_INTENSITY,
      depthWrite: true,
    });

    // 车身：长轴沿 X（forward），宽沿 Z（与 glTF 模型一致）
    const bodyGeo = new THREE.BoxGeometry(4.5, 0.6, 1.8);
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.castShadow = true;
    body.receiveShadow = true;
    body.position.y = 0.65;
    group.add(body);

    // 挡风玻璃（半透明）
    const glassGeo = new THREE.BoxGeometry(0.1, 0.35, 1.6);
    const glassMat = new THREE.MeshPhysicalMaterial({
      color: 0x88ccff,
      roughness: 0.1,
      metalness: 0.1,
      opacity: 0.4,
      transparent: true,
      depthWrite: false,
    });
    const glass = new THREE.Mesh(glassGeo, glassMat);
    glass.position.set(0.8, 0.95, 0);
    group.add(glass);

    // 后窗
    const rearGlass = new THREE.Mesh(glassGeo.clone(), glassMat);
    rearGlass.position.set(-0.8, 0.95, 0);
    group.add(rearGlass);

    // 轮毂（4个圆柱，轴沿 Z 与 glTF 一致）
    const wheelGeo = new THREE.CylinderGeometry(0.3, 0.3, 0.2, 16);
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.6, metalness: 0.7 });
    const wheelPositions = [
      [1.3, 0.3, -0.8], [1.3, 0.3, 0.8],
      [-1.3, 0.3, -0.8], [-1.3, 0.3, 0.8],
    ];
    wheelPositions.forEach(([x, y, z]) => {
      const w = new THREE.Mesh(wheelGeo, wheelMat);
      w.position.set(x, y, z);
      w.rotation.x = Math.PI / 2; // 圆柱轴从 Y → Z（横向，与 glTF cylinder axis=Z 一致）
      w.castShadow = true;
      group.add(w);
    });

    group.name = 'fallback_' + type + '_' + id;
    return group;
  }

  /** 从 glTF 创建车辆。
   *  getModel() 返回的已经是 THREE.Group（非 loader 原始 {scene}），
   *  直接对其 clone，不要走 .scene 否则抛 TypeError 导致永远回退 fallback。 */
  function _createGltfVehicle(gltf, id) {
    const scene = gltf.clone(true);
    scene.name = 'gltf_' + id;
    // 应用车漆升级
    _applyCarPaintToScene(scene, _envMap);
    return scene;
  }

  /** 每帧更新 glTF 车辆：车轮旋转 + 方向盘 */
  function _updateGltfVehicle(entry, speed_mps) {
    const vis = _getVisGroup(entry);
    if (!vis) return;

    vis.traverse((child) => {
      if (!child.isMesh) return;
      const name = (child.name || '').toLowerCase();

      // 车轮旋转（沿 X 轴）
      if (name.includes('wheel') || name.includes('tire') || name.includes('tyre')) {
        if (speed_mps !== undefined) {
          // 假设半径 0.35m
          const angularSpeed = speed_mps / 0.35;
          child.rotation.x += angularSpeed * 0.016;
        }
      }

      // 方向盘（Z 轴缓慢回旋）
      if (name.includes('steering') || name.includes('steer') || name.includes('handle')) {
        if (entry.steerAngle !== undefined) {
          child.rotation.z = entry.steerAngle * 0.02;
        }
      }
    });
  }

  // ── 公共 API ──

  /** 添加或更新一辆车 */
  function updateVehicle(id, vehicleData, type) {
    // 确保 envMap 已生成
    _ensureEnvMap(renderer, scene);

    let entry = vehicleMap.get(id);
    if (!entry) {
      entry = { group: null, lights: null, modelData: null, steerAngle: 0 };
      vehicleMap.set(id, entry);
    }

    // 尝试加载 glTF 模型
    const gltf = getModel(type);
    if (gltf && !entry.modelData) {
      // 清除旧 fallback group（避免双重模型叠加）
      if (entry.group) {
        vehicleGroup.remove(entry.group);
        entry.group = null;
      }
      entry.modelData = gltf;
      entry.group = _createGltfVehicle(gltf, id);
      vehicleGroup.add(entry.group);

      // 灯光
      entry.lights = createVehicleLights(entry.group);
      if (entry.lights && entry.lights.group) {
        entry.group.add(entry.lights.group);
      }
    }

    // 如果没有 group（glTF 加载失败或未就绪），创建 fallback
    if (!entry.group) {
      entry.group = _createFallbackVehicle(type, id);
      vehicleGroup.add(entry.group);
      entry.lights = createVehicleLights(entry.group);
      if (entry.lights && entry.lights.group) {
        entry.group.add(entry.lights.group);
      }
    }

    // 更新位姿（ENU → THREE 坐标映射）
    if (vehicleData) {
      const [tx, ty, tz] = worldToThree(
        vehicleData.x || vehicleData.px || 0,
        vehicleData.y || vehicleData.py || 0,
        vehicleData.z || vehicleData.pz || 0
      );
      entry.group.position.set(tx, ty, tz);
      entry.group.rotation.set(0, headingToRotationY(vehicleData.heading || vehicleData.yaw || 0), 0);
      entry.steerAngle = vehicleData.steer || vehicleData.steerAngle || 0;
    }

    return entry;
  }

  /** 每帧 tick */
  function tick(vehicles, dt) {
    if (!vehicles) return;
    for (const v of vehicles) {
      const entry = vehicleMap.get(v.id);
      if (!entry) continue;

      // 车轮动画
      if (entry.modelData) {
        _updateGltfVehicle(entry, v.speed_mps);
      }

      // 灯光
      if (entry.lights) {
        entry.lights.update(v);
      }
    }
  }

  /** 移除车辆 */
  function removeVehicle(id) {
    const entry = vehicleMap.get(id);
    if (!entry) return;
    if (entry.group) {
      vehicleGroup.remove(entry.group);
      entry.group.traverse((child) => {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
          if (Array.isArray(child.material)) {
            child.material.forEach(m => m.dispose());
          } else {
            child.material.dispose();
          }
        }
      });
    }
    vehicleMap.delete(id);
  }

  /** 清理 */
  function dispose() {
    vehicleMap.forEach((entry, id) => removeVehicle(id));
    vehicleMap.clear();
    scene.remove(vehicleGroup);
    if (_pmrem) {
      _pmrem.dispose();
      _pmrem = null;
    }
    if (_envMap) {
      _envMap.dispose();
      _envMap = null;
    }
  }

  function getVehicleGroup() { return vehicleGroup; }
  function getVehicleMap() { return vehicleMap; }

  /** Layer 树每帧调用：从 store 同步 ego + entities → 创建/更新/删除车辆 */
  function update(store, now) {
    if (!store) return;

    // 收集所有需要渲染的实体（ego + 其他车辆/NPC）
    const activeIds = new Set();

    // 1. ego 车辆（type 用于选模型：'ego' → su7）
    if (store.ego) {
      const egoId = 'ego';
      activeIds.add(egoId);
      updateVehicle(egoId, store.ego, 'su7');
    }

    // 2. 其他实体（car/truck/suv/pedestrian 等）
    const entities = store.entities || [];
    for (const ent of entities) {
      if (!ent || !ent.id) continue;
      activeIds.add(ent.id);
      updateVehicle(ent.id, ent, ent.type || 'car');
    }

    // 3. 删除消失的车辆
    for (const id of Array.from(vehicleMap.keys())) {
      if (!activeIds.has(id)) {
        removeVehicle(id);
      }
    }
  }

  return { update, updateVehicle, tick, removeVehicle, dispose, getVehicleGroup, getVehicleMap };
}