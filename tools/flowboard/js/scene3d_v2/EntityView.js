/**
 * EntityView.js — v2 障碍物/NPC 渲染
 *
 * 简化策略：
 *   - 复用 v1 models.js 的 buildObstacleGroup（按 type 返回车/卡车/行人/锥桶）
 *   - mesh 池：避免每帧 alloc，复用隐藏的 mesh
 *   - 不做 NPC 详情面板、不做车灯
 *
 * 数据源：scene/frame.entities[]
 *   每个实体：{ id, type, x, y, h, length, width, speed }
 *   - type: 'ego' / 'vehicle' / 'pedestrian' / 'bicycle' / 'cone' / ...
 *   - (x, y) 是 world 坐标 → THREE.Vector3(x, elevation, y)
 */
const THREE = window.THREE;
import { buildObstacleGroup } from '../models.js';
import { getRoadElevationAt } from './RoadBuilder.js';

// ── mesh 池：type → mesh[]
// 复用同类型 mesh，避免每帧 alloc/dispose
const _pool = new Map();  // type → array of { mesh, inUse }

function getFromPool(type) {
  var arr = _pool.get(type);
  if (arr) {
    for (var i = 0; i < arr.length; i++) {
      if (!arr[i].inUse) {
        arr[i].inUse = true;
        arr[i].mesh.visible = true;
        return arr[i].mesh;
      }
    }
  }
  return null;
}

function createForType(type) {
  var colorMap = {
    car: 0xff9944,
    truck: 0x886644,
    pedestrian: 0xddaa66,
    bicycle: 0x66aa66,
    cone: 0xff6600
  };
  var color = colorMap[type] || 0xff9944;
  var mesh = buildObstacleGroup(type, color);
  if (!mesh) {
    // 兜底：简单 box
    mesh = new THREE.Group();
    var g = new THREE.BoxGeometry(4.6, 1.4, 2.0);
    var m = new THREE.MeshStandardMaterial({ color: color, roughness: 0.5, metalness: 0.3 });
    var body = new THREE.Mesh(g, m);
    body.position.y = 0.7;
    mesh.add(body);
  }
  mesh.userData.entityType = type;
  return mesh;
}

function getOrCreate(type, parent) {
  var mesh = getFromPool(type);
  if (mesh) return mesh;
  // 池里没有可用的，新建
  mesh = createForType(type);
  parent.add(mesh);
  var arr = _pool.get(type) || [];
  arr.push({ mesh: mesh, inUse: true });
  _pool.set(type, arr);
  return mesh;
}

function releaseAll() {
  _pool.forEach(function(arr) {
    arr.forEach(function(entry) {
      entry.inUse = false;
      entry.mesh.visible = false;
    });
  });
}

/**
 * 每帧更新所有 entities。
 * @param {Array} entities  scene/frame.entities[]
 * @param {THREE.Group} parent  挂载目标
 * @param {Array} curves  道路曲线（算 elevation）
 */
export function updateEntities(entities, parent, curves) {
  // 先全部标记为未使用
  releaseAll();
  if (!entities || !entities.length) return;

  for (var i = 0; i < entities.length; i++) {
    var ent = entities[i];
    if (!ent || ent.type === 'ego') continue;  // ego 由 EgoView 单独处理
    var mesh = getOrCreate(ent.type || 'car', parent);
    var elev = getRoadElevationAt(ent.x || 0, ent.y || 0, curves);
    mesh.position.set(ent.x || 0, elev, ent.y || 0);
    mesh.rotation.y = -(ent.h || 0);  // 与 v1 一致
    // 长宽覆盖（如果实体带尺寸，缩放 mesh）
    if (ent.length && ent.width) {
      // models.js 默认 sedan 4.6×2.0，按比例缩放
      mesh.scale.set(ent.length / 4.6, 1.0, ent.width / 2.0);
    } else {
      mesh.scale.set(1, 1, 1);
    }
  }
}

/**
 * 场景销毁时清理池（dispose 所有 mesh 资源）。
 */
export function disposePool() {
  _pool.forEach(function(arr) {
    arr.forEach(function(entry) {
      entry.mesh.traverse(function(child) {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
          if (Array.isArray(child.material)) child.material.forEach(function(m) { m.dispose(); });
          else child.material.dispose();
        }
      });
    });
  });
  _pool.clear();
}
