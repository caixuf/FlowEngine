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

import { createVehicleLights } from './VehicleLights.js';
import { getStdMaterial } from '../core/AssetFactory.js';

// ═══════════════════════════════════════════════════════════
// 车漆参数（MeshPhysicalMaterial）
// ═══════════════════════════════════════════════════════════

const PAINT_CLEARCOAT = 0.9;
const PAINT_CLEARCOAT_ROUGHNESS = 0.15;
const PAINT_ROUGHNESS = 0.35;
const PAINT_METALNESS = 0.85;
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

  /** 创建程序化 fallback（保留原 getStdMaterial 但升级为 MeshPhysicalMaterial） */
  function _createFallbackVehicle(type, id) {
    const group = new THREE.Group();
    let bodyMat;

    // 尝试用 MeshPhysicalMaterial 替代 MeshStandardMaterial
    const stdMat = getStdMaterial(type);
    if (stdMat && stdMat.color) {
      bodyMat = new THREE.MeshPhysicalMaterial({
        color: stdMat.color.getHex(),
        metalness: PAINT_METALNESS,
        roughness: PAINT_ROUGHNESS,
        clearcoat: PAINT_CLEARCOAT,
        clearcoatRoughness: PAINT_CLEARCOAT_ROUGHNESS,
        envMap: _envMap || null,
        envMapIntensity: ENVMAP_INTENSITY,
        depthWrite: true,
      });
    } else {
      bodyMat = new THREE.MeshPhysicalMaterial({
        color: 0xcccccc,
        metalness: PAINT_METALNESS,
        roughness: PAINT_ROUGHNESS,
        clearcoat: PAINT_CLEARCOAT,
        clearcoatRoughness: PAINT_CLEARCOAT_ROUGHNESS,
        depthWrite: true,
      });
    }

    // 简易盒子车身（保留原逻辑，换材质）
    const bodyGeo = new THREE.BoxGeometry(1.8, 0.6, 4.5);
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.castShadow = true;
    body.receiveShadow = true;
    body.position.y = 0.65;
    group.add(body);

    // 挡风玻璃（半透明）
    const glassGeo = new THREE.BoxGeometry(1.6, 0.35, 0.1);
    const glassMat = new THREE.MeshPhysicalMaterial({
      color: 0x88ccff,
      roughness: 0.1,
      metalness: 0.1,
      opacity: 0.4,
      transparent: true,
      depthWrite: false,
    });
    const glass = new THREE.Mesh(glassGeo, glassMat);
    glass.position.set(0, 0.95, 0.8);
    group.add(glass);

    // 后窗
    const rearGlass = new THREE.Mesh(glassGeo.clone(), glassMat);
    rearGlass.position.set(0, 0.95, -0.8);
    group.add(rearGlass);

    // 轮毂（4个圆柱）
    const wheelGeo = new THREE.CylinderGeometry(0.3, 0.3, 0.2, 16);
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.6, metalness: 0.7 });
    const wheelPositions = [
      [-0.8, 0.3, 1.3], [0.8, 0.3, 1.3],
      [-0.8, 0.3, -1.3], [0.8, 0.3, -1.3],
    ];
    wheelPositions.forEach(([x, y, z]) => {
      const w = new THREE.Mesh(wheelGeo, wheelMat);
      w.position.set(x, y, z);
      w.rotation.z = Math.PI / 2;
      w.castShadow = true;
      group.add(w);
    });

    group.name = 'fallback_' + type + '_' + id;
    return group;
  }

  /** 从 glTF 创建车辆 */
  function _createGltfVehicle(gltf, id) {
    const scene = gltf.scene.clone(true);
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
    const gltf = (modelCache && modelCache.getModel) ? modelCache.getModel(type) : null;
    if (gltf && !entry.modelData) {
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

    // 更新位姿
    if (vehicleData) {
      entry.group.position.set(
        vehicleData.x || vehicleData.px || 0,
        vehicleData.y || vehicleData.py || 0,
        vehicleData.z || vehicleData.pz || 0
      );
      entry.group.rotation.set(0, vehicleData.heading || vehicleData.yaw || 0, 0);
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

  return { updateVehicle, tick, removeVehicle, dispose, getVehicleGroup, getVehicleMap };
}