/**
 * models.js — glTF model cache for FlowBoard 3D scene
 *
 * Loads vehicle/pedestrian .gltf files via Three.js GLTFLoader.
 * Falls back to programmatic geometry when GLTFLoader is unavailable
 * or a model file fails to load.
 *
 * glTF 节点约定（gen_models.py 生成）：
 *   wheel_FL / wheel_FR / wheel_RL / wheel_RR  — 四轮节点
 *   body / cabin / cab / cargo / torso / head  — 车身/驾驶舱等
 * 本模块从命名节点建立 userData.frontAxle / rearAxle / wheels，使 glTF 车辆
 * 与程序化 _buildSedan 一样支持前轴转向 + 全轮滚动动画。
 *
 * Usage:
 *   import { initModelCache, getModel } from './models.js';
 *   await initModelCache();
 *   const sedan = getModel('sedan').clone();
 *   scene.add(sedan);
 */

import { _buildSedan, _buildObstacle, _buildContactShadow } from './utils.js';

const THREE = window.THREE;

/** Model registry: { name: THREE.Group or null (fallback) } */
const _cache = {};

/** Loading state */
let _ready = false;
let _loadError = null;

const MODEL_NAMES = ['sedan', 'truck', 'suv', 'pedestrian'];

/**
 * 从 glTF 场景建立带 userData 的车辆 Group。
 * - 保留节点层级与 name（便于按名查找车轮）
 * - 克隆 material 使每个实例独立可改色
 * - 扫描 wheel_FL/FR/RL/RR 节点，建立 frontAxle / rearAxle Group + wheels 数组。
 *   frontAxle Group 位于前轴中心，rearAxle 位于后轴中心，车轮改为相对位置后挂入，
 *   这样 _renderFrame 的 frontAxle.rotation.y（转向绕轴心）和 wheels[].rotation.x（滚动）能生效
 */
function _buildVehicleFromGltf(name, gltf) {
  var group = new THREE.Group();
  // 保留层级：把 gltf.scene 的顶层子节点克隆进来（保留 name）
  gltf.scene.children.forEach(function(child) {
    group.add(child.clone());
  });
  // 克隆 material（每个实例独立）
  group.traverse(function(c) {
    if (c.isMesh && c.material) {
      c.material = c.material.clone();
    }
  });

  group.userData.isVehicle = true;
  group.userData.modelType = name;

  // 车辆类型才需要建立 wheel userData（行人无轮）
  if (name !== 'pedestrian') {
    var fl = null, fr = null, rl = null, rr = null;
    group.traverse(function(c) {
      switch (c.name) {
        case 'wheel_FL': fl = c; break;
        case 'wheel_FR': fr = c; break;
        case 'wheel_RL': rl = c; break;
        case 'wheel_RR': rr = c; break;
      }
    });
    var wheels = [];
    if (fl && fr) {
      // 建立前轴 Group：位于 FL/FR 几何中心。注意不能用 getWorldPosition() ——
      // gen_models.py 生成的 glTF 节点变换是单位矩阵，车轮的真实位置烘在顶点
      // 数据里，getWorldPosition 恒返回 (0,0,0)，会把转向支点错误地放在车身
      // 原点，导致转向时车轮绕错误支点画弧线。改用几何包围盒中心（世界空间）。
      var flBox = new THREE.Box3().setFromObject(fl);
      var flCenter = flBox.getCenter(new THREE.Vector3());
      var frCenter = new THREE.Box3().setFromObject(fr).getCenter(new THREE.Vector3());
      // 真实轮半径（车轮直径沿 Y 轴）：滚动动画角速度 = v·dt / r，
      // 缺省 0.33（与 _buildSedan 一致）。
      group.userData.wheelRadius = (flBox.max.y - flBox.min.y) / 2 || 0.33;
      var fwCenter = flCenter.clone().add(frCenter).multiplyScalar(0.5);
      var fwGroup = new THREE.Group();
      fwGroup.name = '_axle_front';
      fwGroup.position.copy(fwCenter);
      // 将 FL/FR 从原 parent 移入 fwGroup，调整为相对坐标
      [[fl, flCenter], [fr, frCenter]].forEach(function(pair) {
        var w = pair[0];
        // 相对 fwGroup 的位置（提前算好，避免 reparent 后 matrixWorld 失效）
        var wp = pair[1].clone().sub(fwCenter);
        if (w.parent) w.parent.remove(w);
        w.position.copy(wp);
        w.rotation.set(0, 0, 0);  // 重置旋转，滚动动画由 rotation.x 驱动
        fwGroup.add(w);
        wheels.push(w);
      });
      group.add(fwGroup);
      group.userData.frontAxle = fwGroup;
      // 后轴 Group：pivot 在后轴中心，后轮相对轴定位
      if (rl || rr) {
        var rlCenter = rl ? new THREE.Box3().setFromObject(rl).getCenter(new THREE.Vector3()) : null;
        var rrCenter = rr ? new THREE.Box3().setFromObject(rr).getCenter(new THREE.Vector3()) : null;
        var rwCenter = new THREE.Vector3();
        if (rlCenter && rrCenter) {
          rwCenter.copy(rlCenter).add(rrCenter).multiplyScalar(0.5);
        } else if (rlCenter) { rwCenter.copy(rlCenter); }
        else { rwCenter.copy(rrCenter); }
        var rwGroup = new THREE.Group();
        rwGroup.name = '_axle_rear';
        rwGroup.position.copy(rwCenter);
        [[rl, rlCenter], [rr, rrCenter]].forEach(function(pair) {
          var w = pair[0], wc = pair[1];
          if (!w || !wc) return;
          var wp = wc.clone().sub(rwCenter);
          if (w.parent) w.parent.remove(w);
          w.position.copy(wp);
          w.rotation.set(0, 0, 0);
          rwGroup.add(w);
          wheels.push(w);
        });
        group.add(rwGroup);
        group.userData.rearAxle = rwGroup;
      }
    } else {
      // 未找到命名前轮：收集所有 wheel_* 作为普通轮
      [fl, fr, rl, rr].forEach(function(w) { if (w) wheels.push(w); });
    }
    if (wheels.length) group.userData.wheels = wheels;
  }
  // 灯节点扫描：brakelight_L/R, turnsignal_FL/FR/RL/RR, headlight_L/R。
  // scene3d.js 通过 material.emissiveIntensity 切换亮灭（接感知/规划链路）。
  var brakeLights = [], turnSignals = {}, headlights = [];
  group.traverse(function(c) {
    if (!c.isMesh) return;
    var n = c.name || '';
    if (n.indexOf('brakelight_') === 0) brakeLights.push(c);
    else if (n.indexOf('turnsignal_') === 0) {
      turnSignals[n.substring('turnsignal_'.length)] = c;  // FL/FR/RL/RR
    }
    else if (n.indexOf('headlight_') === 0) headlights.push(c);
  });
  if (brakeLights.length) group.userData.brakeLights = brakeLights;
  if (Object.keys(turnSignals).length) group.userData.turnSignals = turnSignals;
  if (headlights.length) group.userData.headlights = headlights;
  return group;
}

/**
 * Preload all glTF models from /tools/flowboard/models/<name>.gltf.
 * If GLTFLoader is unavailable, all models stay null and getModel() returns fallback.
 * Returns a promise that resolves when all models are loaded or failed.
 */
export function initModelCache() {
  if (_ready) return Promise.resolve();
  if (window._gltfLoaderUnavailable) {
    _ready = true;
    return Promise.resolve();
  }
  if (!THREE || !THREE.GLTFLoader) {
    window._gltfLoaderUnavailable = true;
    _ready = true;
    return Promise.resolve();
  }

  return new Promise(function(resolve) {
    var loader = new THREE.GLTFLoader();
    var pending = MODEL_NAMES.length;

    function onModelLoaded(name, gltf) {
      _cache[name] = _buildVehicleFromGltf(name, gltf);
      pending--;
      if (pending <= 0) { _ready = true; resolve(); }
    }

    function onModelError(name, err) {
      console.warn('[models] ' + name + ' load failed: ' + (err.message || err) + ' — using programmatic fallback');
      _cache[name] = null;
      pending--;
      if (pending <= 0) { _ready = true; resolve(); }
    }

    for (var i = 0; i < MODEL_NAMES.length; i++) {
      var name = MODEL_NAMES[i];
      _cache[name] = null;
      loader.load(
        '/tools/flowboard/models/' + name + '.gltf',
        function(n) { return function(g) { onModelLoaded(n, g); }; }(name),
        undefined,
        function(n) { return function(e) { onModelError(n, e); }; }(name)
      );
    }
  });
}

/**
 * Get a cached model group by type name.
 * Returns a THREE.Group (clone before adding to scene).
 * If glTF model is unavailable, returns null (caller uses programmatic fallback).
 *
 * @param {string} type  'sedan', 'truck', 'suv', 'pedestrian', or undefined
 * @returns {THREE.Group|null}
 */
export function getModel(type) {
  var name = type || 'car';
  // Map type names to model names
  switch (name) {
    case 'car':    name = 'sedan'; break;
    case 'truck':  name = 'truck'; break;
    case 'suv':    name = 'suv'; break;
    case 'pedestrian': name = 'pedestrian'; break;
    default:       name = 'sedan';
  }
  var model = _cache[name];
  if (model) {
    var clone = model.clone();
    // Scale: glTF models are built in meters (1:1 with scene)
    // Reset any pre-applied transforms
    clone.scale.set(1, 1, 1);
    // clone() 不深拷贝 userData 中的 Group 引用，重建 frontAxle/rearAxle/wheels
    if (model.userData.frontAxle || model.userData.wheels) {
      _relinkWheelUserData(clone);
    }
    return clone;
  }
  return null;
}

/** clone() 后 userData.frontAxle / rearAxle / wheels 引用失效。
 *  clone 已保留完整层级（_axle_front / _axle_rear 含车轮），
 *  仅需按名查找并重建 userData 引用，无需重建 Group。 */
function _relinkWheelUserData(clone) {
  var fwGroup = null, rwGroup = null;
  var fl = null, fr = null, rl = null, rr = null;
  clone.traverse(function(c) {
    var n = c.name;
    if (n === '_axle_front') fwGroup = c;
    else if (n === '_axle_rear') rwGroup = c;
    else if (n === 'wheel_FL') fl = c;
    else if (n === 'wheel_FR') fr = c;
    else if (n === 'wheel_RL') rl = c;
    else if (n === 'wheel_RR') rr = c;
  });
  var wheels = [];
  if (fwGroup) { clone.userData.frontAxle = fwGroup; if (fl) wheels.push(fl); if (fr) wheels.push(fr); }
  if (rwGroup) { clone.userData.rearAxle = rwGroup; if (rl) wheels.push(rl); if (rr) wheels.push(rr); }
  if (!fwGroup && !rwGroup) { [fl, fr, rl, rr].forEach(function(w) { if (w) wheels.push(w); }); }
  if (wheels.length) clone.userData.wheels = wheels;
  // 灯节点引用也需重建
  var brakeLights = [], turnSignals = {}, headlights = [];
  clone.traverse(function(c) {
    if (!c.isMesh) return;
    var n = c.name || '';
    if (n.indexOf('brakelight_') === 0) brakeLights.push(c);
    else if (n.indexOf('turnsignal_') === 0) turnSignals[n.substring('turnsignal_'.length)] = c;
    else if (n.indexOf('headlight_') === 0) headlights.push(c);
  });
  if (brakeLights.length) clone.userData.brakeLights = brakeLights;
  if (Object.keys(turnSignals).length) clone.userData.turnSignals = turnSignals;
  if (headlights.length) clone.userData.headlights = headlights;
}

/**
 * 将 glTF 车身的 PBR 材质升级为 MeshPhysicalMaterial（clearcoat 车漆），
 * 并整体涂色。仅改 body/cabin/cab/cargo/hood/trunklid/door_* / wiper_* 等
 * 车身件；跳过灯节点（brakelight_* / turnsignal_* / headlight_* 保留发光材质）、
 * 车轮（wheel_* 保持轮胎黑）和玻璃（windshield/rear_window 保持透明）。
 */
function _upgradeCarPaint(model, color) {
  var bodyNames = { body: 1, cabin: 1, cab: 1, cargo: 1, rear: 1, hood: 1, trunklid: 1,
                    door_FL: 1, door_FR: 1, door_RL: 1, door_RR: 1,
                    chargeport_cover: 1, wiper_L: 1, wiper_R: 1 };
  var SKIP_PREFIXES = ['brakelight_', 'turnsignal_', 'headlight_', 'wheel_', 'ads_indicator'];
  var SKIP_NAMES = { windshield: 1, rear_window: 1 };
  function shouldSkip(name) {
    if (!name) return false;
    if (SKIP_NAMES[name]) return true;
    for (var i = 0; i < SKIP_PREFIXES.length; i++) {
      if (name.indexOf(SKIP_PREFIXES[i]) === 0) return true;
    }
    return false;
  }
  model.traverse(function(c) {
    if (!c.isMesh || !c.material) return;
    if (shouldSkip(c.name)) return;  // 灯/轮/玻璃：保留原材质
    if (c.name && bodyNames[c.name]) {
      // 升级为 MeshPhysicalMaterial 清漆车漆
      var oldMat = c.material;
      var newMat = new THREE.MeshPhysicalMaterial({
        color: color || (oldMat.color ? oldMat.color.getHex() : 0x4488dd),
        metalness: oldMat.metalness !== undefined ? oldMat.metalness : 0.5,
        roughness: oldMat.roughness !== undefined ? oldMat.roughness : 0.25,
        envMapIntensity: 1.1,
        clearcoat: 1.0, clearcoatRoughness: 0.07,
        sheen: new THREE.Color(0.4, 0.4, 0.4)
      });
      c.material = newMat;
    } else if (c.material && c.material.color) {
      // 其他未命名车身件：保留原材质，仅改色
      c.material.color.setHex(color);
    }
  });
}

/**
 * _setVehicleLights — 切换车辆灯光状态（接感知 / 规划链路）。
 * 通过修改 material.emissiveIntensity 控制亮灭，不重建材质。
 *
 * @param {THREE.Group} group   车辆 group（含 userData.brakeLights/turnSignals）
 * @param {object} state        { brake: bool, turnL: bool, turnR: bool, head: bool }
 * @param {number} blinkPhase   闪烁相位 0..1（转向灯 1.5Hz 闪烁，scene3d.js 传入 _animT）
 */
export function _setVehicleLights(group, state, blinkPhase) {
  if (!group || !group.userData) return;
  var ud = group.userData;
  var blinkOn = (blinkPhase !== undefined) ? (Math.sin(blinkPhase * Math.PI * 2 * 1.5) > 0) : true;
  // 刹车灯：on=2.0, off=0.15
  if (ud.brakeLights) {
    var bi = state.brake ? 2.0 : 0.15;
    for (var i = 0; i < ud.brakeLights.length; i++) {
      if (ud.brakeLights[i].material) ud.brakeLights[i].material.emissiveIntensity = bi;
    }
  }
  // 转向灯：on 时按 blinkPhase 闪烁，off=0.1
  if (ud.turnSignals) {
    var ts = ud.turnSignals;
    var setSide = function(on, keys) {
      var intensity = on ? (blinkOn ? 2.2 : 0.1) : 0.1;
      for (var k = 0; k < keys.length; k++) {
        if (ts[keys[k]] && ts[keys[k]].material) {
          ts[keys[k]].material.emissiveIntensity = intensity;
        }
      }
    };
    setSide(state.turnL, ['FL', 'RL']);
    setSide(state.turnR, ['FR', 'RR']);
  }
  // 大灯：常亮（白天低亮，可扩展为夜间高亮）
  if (ud.headlights && state.head !== undefined) {
    var hi = state.head ? 1.5 : 0.4;
    for (var h = 0; h < ud.headlights.length; h++) {
      if (ud.headlights[h].material) ud.headlights[h].material.emissiveIntensity = hi;
    }
  }
}

/**
 * Build a vehicle group for use as ego car.
 * 优先使用 glTF（gen_models.py 生成，含命名灯节点 + PBR 材质），
 * GLTFLoader 未就绪时回退到程序化 _buildSedan。
 */
export function buildEgoCar(color) {
  var model = getModel('sedan');
  if (model) {
    _upgradeCarPaint(model, color || 0x4488dd);
    model.add(_buildContactShadow(4.6, 2.0));
    return model;
  }
  return _buildSedan(color || 0x4488dd, 0x3377bb);
}

/**
 * Build an obstacle group for NPC vehicles/pedestrians.
 * 优先使用 glTF，GLTFLoader 未就绪时回退到程序化 _buildObstacle。
 */
export function buildObstacleGroup(type, color) {
  var model = getModel(type);
  if (model) {
    var c = color || 0xff9944;
    _upgradeCarPaint(model, c);
    if (type !== 'pedestrian') {
      model.add(_buildContactShadow(4.6, 2.0));
    }
    model.userData.realScale = true;
    return model;
  }
  return _buildObstacle(type || 'car', color || 0xff9944);
}
