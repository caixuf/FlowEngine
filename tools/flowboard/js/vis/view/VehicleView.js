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
 * Layer 树驱动：rootLayer.update(store, now) → 创建/更新车辆 + 每帧动画
 * （车轮转向/滚动、方向盘、刹车灯/转向灯/大灯 emissive 控制）
 */

import { deriveLightState, LIGHT_TURN_LEFT, LIGHT_TURN_RIGHT, LIGHT_HAZARD, LIGHT_HIGH_BEAM, LIGHT_LOW_BEAM } from './VehicleLights.js';
import { getStdMaterial } from '../core/AssetFactory.js';
import { initModelCache, getModel, _setVehicleLights, _relinkWheelUserData } from '../../models.js';
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
const GEO_BRAKE = new THREE.PlaneGeometry(0.10, 0.06);  // 刹车灯：窄条不占视觉
const GEO_TURN  = new THREE.PlaneGeometry(0.22, 0.12);  // 转向灯：更大更醒目
const GEO_HEAD  = new THREE.PlaneGeometry(0.26, 0.14);  // 近光灯：宽大明显

function _makeRectMesh(geo, color, x, y, z) {
  const mat = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.9 });
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  return m;
}

/** 为车辆模型创建灯光网格组。
 *  @param {THREE.Group} vehicleGroup 车辆模型根节点
 *  @returns {{group: THREE.Group, update: (v: object) => void}} */
function createVehicleLights(vehicleGroup) {
  const group = new THREE.Group();

  const brakeL = _makeRectMesh(GEO_BRAKE, LIGHT_OFF, -BRAKE_X, BRAKE_Y, BRAKE_Z);
  const brakeR = _makeRectMesh(GEO_BRAKE, LIGHT_OFF,  BRAKE_X, BRAKE_Y, BRAKE_Z);
  const turnL  = _makeRectMesh(GEO_TURN, LIGHT_OFF, -TURN_X,  BRAKE_Y, TURN_Z);
  const turnR  = _makeRectMesh(GEO_TURN, LIGHT_OFF,  TURN_X,  BRAKE_Y, TURN_Z);
  const headL  = _makeRectMesh(GEO_HEAD, LIGHT_OFF, -HEAD_X,  HEAD_Y,  HEAD_Z);
  const headR  = _makeRectMesh(GEO_HEAD, LIGHT_OFF,  HEAD_X,  HEAD_Y,  HEAD_Z);

  // 熄灯时隐藏，避免暗色灯片显示为车身边缘的黑方块
  brakeL.visible = brakeR.visible = false;
  turnL.visible = turnR.visible = false;
  headL.visible = headR.visible = false;

  group.add(brakeL, brakeR, turnL, turnR, headL, headR);

  return {
    group,
    update(v) {
      const s = deriveLightState(v.lights || 0, v.brake || 0);
      brakeL.visible = s.brake;
      brakeR.visible = s.brake;
      brakeL.material.color.copy(LIGHT_BRAKE_ON);
      brakeR.material.color.copy(LIGHT_BRAKE_ON);
      turnL.visible = s.turnL;
      turnR.visible = s.turnR;
      turnL.material.color.copy(LIGHT_TURN_ON);
      turnR.material.color.copy(LIGHT_TURN_ON);
      headL.visible = s.head;
      headR.visible = s.head;
      headL.material.color.copy(LIGHT_HEAD_ON);
      headR.material.color.copy(LIGHT_HEAD_ON);
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
  // 不信 gltf 烘焙的 metalness（gen_models.py 里偏高，导致金属漆缺环境反射时发黑）。
  // 强制 clamp 到 PAINT_METALNESS，保证有漫反射基色，不纯靠反射成像。
  const oldRoughness = mesh.material.roughness !== undefined ? mesh.material.roughness : PAINT_ROUGHNESS;

  const mat = new THREE.MeshPhysicalMaterial({
    color: oldColor,
    metalness: PAINT_METALNESS,
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
// 方向盘程序化注入
// ═══════════════════════════════════════════════════════════

/* glTF 模型（sedan/suv/truck/su7）均无 "steering"/"steer"/"handle" 命名节点，
 * 因此程序化注入一个名为 "steering_wheel" 的 torus mesh。
 * 旋转方向：steer > 0 = 右转，THREE rotation.z 正向逆时针，故取 -10 倍。 */
function _createSteeringWheel() {
  const pos = new THREE.Group();
  pos.position.set(0.5, 1.0, 0.4);

  const yaw = new THREE.Group();
  yaw.rotation.y = -Math.PI / 2;  // column: +Z → -X

  const tilt = new THREE.Group();
  tilt.rotation.x = 0.45;  // column 顶部向后上方 25°

  // 方向盘圆环（TorusGeometry：XY 平面，法线沿 Z）
  const torusGeo = new THREE.TorusGeometry(0.16, 0.018, 8, 24);
  const torusMat = new THREE.MeshStandardMaterial({
    color: 0x111111, roughness: 0.45, metalness: 0.7,  // exempt: steering wheel rim (non-body metal)
  });
  const torus = new THREE.Mesh(torusGeo, torusMat);
  torus.name = 'steering_wheel';  // 让 _updateGltfVehicle 的 traverse 选中
  torus.castShadow = false;       // 方向盘不投影，避免车内饰阴影噪音

  // 中心 hub（child of torus，跟随旋转）
  const hubGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.05, 16);
  const hubMat = new THREE.MeshStandardMaterial({
    color: 0x222222, roughness: 0.4, metalness: 0.8,  // exempt: steering wheel hub (non-body metal)
  });
  const hub = new THREE.Mesh(hubGeo, hubMat);
  hub.rotation.x = Math.PI / 2;  // 圆柱轴 Y → Z (沿 column)
  torus.add(hub);

  // 3 条辐条（child of torus，跟随旋转；各 120° 间隔）
  const spokeGeo = new THREE.BoxGeometry(0.022, 0.30, 0.010);
  const spokeMat = new THREE.MeshStandardMaterial({
    color: 0x333333, roughness: 0.5, metalness: 0.6,  // exempt: steering wheel spokes (non-body metal)
  });
  for (let i = 0; i < 3; i++) {
    const spoke = new THREE.Mesh(spokeGeo, spokeMat);
    spoke.rotation.z = (i * 120) * Math.PI / 180;
    torus.add(spoke);
  }

  tilt.add(torus);
  yaw.add(tilt);
  pos.add(yaw);
  return pos;
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

    // 注入方向盘（fallback 车辆同样需要可见的转向反馈，行人跳过）
    if (type !== 'pedestrian') {
      group.add(_createSteeringWheel());
    }

    group.name = 'fallback_' + type + '_' + id;
    return group;
  }

  /** 从 glTF 创建车辆/行人。
   *  getModel() 返回的已经是 THREE.Group（非 loader 原始 {scene}），
   *  直接对其 clone，不要走 .scene 否则抛 TypeError 导致永远回退 fallback。
   *  行人只显示躯干+头+腿，不注入方向盘/车漆/车轮动画。 */
  function _createGltfVehicle(gltf, id, type) {
    const scene = gltf.clone(true);
    scene.name = 'gltf_' + id;
    // clone() 不深拷贝 userData 引用，重建 wheel/light 引用
    _relinkWheelUserData(scene);
    if (type === 'pedestrian') {
      // 行人：无车漆、无方向盘、无灯光节点
      return scene;
    }
    // 应用车漆升级
    _applyCarPaintToScene(scene, _envMap);
    // 注入方向盘（glTF 车辆模型未自带 steering_wheel mesh）
    scene.add(_createSteeringWheel());
    return scene;
  }

  /** 每帧更新 glTF 车辆/行人：前轴转向 + 车轮旋转 + 灯光 */
  function _updateGltfVehicle(entry, v, dt, type) {
    const vis = _getVisGroup(entry);
    if (!vis) return;
    // 行人：无方向盘/车轮/灯光动画
    if (type === 'pedestrian') return;
    const speed_mps = v ? v.speed_mps : 0;

    // 前轴转向：glTF 模型 axle_front（含 wheel_FL/wheel_FR）整体绕 Y 旋转。
    // steer > 0 = 右转，THREE rotation.y 正向 = 逆时针，故取负。
    // 1:1 映射（2026-08 修复）：后端 step_bicycle 的 steer 就是前轮转角 δ，
    // 直接显示 —— 变道 0.05-0.1 rad ≈ 3-6°（可见），掉头满舵 0.45-0.60 rad
    // ≈ 26-34°（真实前轮最大转角）。旧乘子 0.6 把前轮角又缩小 40%，
    // 变道时前轮只转 2-3.5° 肉眼不可见 → "前轮不会动"。
    // 死区滤波：|steer| < 0.005 时视为 0，避免直路巡航时方向盘和车轮微动。
    const rawSteer = (entry && entry.steerAngle) || 0;
    const steerFiltered = (Math.abs(rawSteer) < 0.005) ? 0 : rawSteer;
    if (vis.userData && vis.userData.frontAxle) {
      vis.userData.frontAxle.rotation.y = -steerFiltered;
    }

    vis.traverse((child) => {
      const name = (child.name || '').toLowerCase();

      // 方向盘优先处理（允许非 Mesh 的 Group，但 _createSteeringWheel 里的 torus
      // 是 Mesh 且 name 含 'steering'，所以 isMesh 早退仍能命中）。
      // 乘子 10（不是原 0.02）：让 0.1 rad 的 steer 视觉上转 ~57°，max 0.25 → ~143°，
      // 肉眼可见。负号：THREE.js rotation.z 正向是逆时针，驾驶员视角下右转（steer>0）
      // 应顺时针，所以取反符合直觉。
      if ((name.includes('steering') || name.includes('steer') || name.includes('handle')) &&
          steerFiltered !== undefined) {
        child.rotation.z = steerFiltered * -10;
        // 命中方向盘后不再处理车轮逻辑（避免方向盘被误判为 wheel）
        return;
      }

      if (!child.isMesh) return;

      // 车轮旋转（沿 X 轴）
      if (name.includes('wheel') || name.includes('tire') || name.includes('tyre')) {
        if (speed_mps !== undefined) {
          // 假设半径 0.35m
          const angularSpeed = speed_mps / 0.35;
          child.rotation.x += angularSpeed * 0.016;
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
      entry.group = _createGltfVehicle(gltf, id, type);
      vehicleGroup.add(entry.group);

      // glTF 模型自带灯节点（turnsignal_/brakelight_/headlight_），
      // 由 _setVehicleLights 控制 emissive，不需要 PlaneGeometry 方块。
      entry.lights = null;
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

  /* ── tick() 已删除 ──
   *
   * MVC 重构后 Layer 树只调 view.update()，不调 view.tick()。
   * 车轮转向/滚动 + 灯光控制等每帧动画已全部合并到 update() 中。
   * 若需单独驱动动画，直接调 update(store, now) 即可。 */


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

  /** Layer 树每帧调用：从 store 同步 ego + entities → 创建/更新/删除车辆 + 动画 */
  function update(store, now) {
    if (!store) return;
    const dt = 1 / 60;  // 默认帧间隔

    // 收集所有需要渲染的实体（ego + 其他车辆/NPC）
    const activeIds = new Set();

    // 1. ego 车辆（type 用于选模型：'ego' → su7）
    if (store.ego) {
      const egoId = 'ego';
      activeIds.add(egoId);
      updateVehicle(egoId, store.ego, 'su7');
    }

    // 2. 其他实体（car/truck/suv/pedestrian — 排除红绿灯/ETC/停止线等非车辆）
    const VEHICLE_TYPES = new Set(['car', 'suv', 'truck', 'pedestrian']);
    const entities = store.entities || [];
    for (const ent of entities) {
      if (!ent || !ent.id) continue;
      if (!VEHICLE_TYPES.has(ent.type)) continue;
      activeIds.add(ent.id);
      updateVehicle(ent.id, ent, ent.type || 'car');
    }

    // 3. 删除消失的车辆
    for (const id of Array.from(vehicleMap.keys())) {
      if (!activeIds.has(id)) {
        removeVehicle(id);
      }
    }

    // 4. 每帧动画：前轴转向 + 车轮旋转 + 灯光
    const animateVehicle = (id, type) => {
      const entry = vehicleMap.get(id);
      if (!entry) return;
      const v = (id === 'ego') ? store.ego
               : entities.find(e => e && e.id === id);
      if (!v) return;

      // glTF 动画（行人跳过方向盘/车轮/灯光）
      if (entry.modelData) {
        _updateGltfVehicle(entry, v, dt, type);
        // 行人灯光不处理
        if (type === 'pedestrian') return;
      } else {
        // 程序化 fallback 前轴转向（1:1 显示前轮角，见上方 glTF 路径注释；
        // 加死区滤波，避免直路巡航时微动）
        const vis = _getVisGroup(entry);
        if (vis && vis.userData && vis.userData.frontAxle) {
          const raw = (entry && entry.steerAngle) || 0;
          const sf = (Math.abs(raw) < 0.005) ? 0 : raw;
          vis.userData.frontAxle.rotation.y = -sf;
        }
      }

      // 灯光：glTF 模型用 emissive 材质自带灯节点，不叠加 PlaneGeometry 方块
      // 只有程序化 fallback 车辆（无 modelData）才需要 PlaneGeometry 灯光
      if (!entry.modelData && entry.lights) {
        entry.lights.update(v);
      }

      // glTF 模型 emissive 灯光（含 ads_indicator 小蓝灯）
      const vis = _getVisGroup(entry);
      if (vis && vis.userData &&
          (vis.userData.brakeLights || vis.userData.turnSignals || vis.userData.headlights || vis.userData.adsIndicators)) {
        const ls = deriveLightState(v.lights || 0, v.brake || 0);
        _setVehicleLights(vis, ls, now !== undefined ? now / 1000 : undefined);
      }
    };

    animateVehicle('ego', 'su7');
    for (const ent of entities) {
      if (ent && ent.id) animateVehicle(ent.id, ent.type);
    }
  }

  return { update, updateVehicle, removeVehicle, dispose, getVehicleGroup, getVehicleMap };
}