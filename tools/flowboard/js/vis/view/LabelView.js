/**
 * LabelView.js — 实体标签（NPC 头顶 speed + ai_state 文字）
 *
 * 使用 THREE.Sprite + CanvasTextureFactory 渲染文字标签，始终面向相机。
 * 通过 Layer 树每帧 update(store) 驱动，与 VehicleView 共享 agent 层。
 *
 * 每帧 diff store.entities 与内部 Map，自动创建/更新/删除标签。
 * 标签文字格式：speed 在前，ai_state 在后（若 ai_state 非空）。
 */

import { worldToThree } from '../math/Coord.js';
import { makeLabelTexture } from '../utils/CanvasTextureFactory.js';

const LABEL_SCALE = 2.0;       // 标签世界大小
const LABEL_Y_OFFSET = 2.8;    // 头顶上方偏移
const CANVAS_W = 256;          // 标签纹理分辨率
const CANVAS_H = 64;

/** 从纹理创建 Sprite 标签 */
function _createLabelSprite(texture) {
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
    sizeAttenuation: true,
  });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(LABEL_SCALE, LABEL_SCALE * (CANVAS_H / CANVAS_W), 1);
  return sprite;
}

export function createLabelView(scene) {
  const labelGroup = new THREE.Group();
  labelGroup.name = 'labelGroup';
  scene.add(labelGroup);

  /** 内部 Map: entityId → { sprite, lastSpeed, lastState } */
  const _labelMap = new Map();

  /** 获取或创建标签 Sprite */
  function _ensureLabel(id, speed, aiState) {
    let entry = _labelMap.get(id);
    if (!entry) {
      const tex = makeLabelTexture(speed, aiState, { width: CANVAS_W, height: CANVAS_H });
      const sprite = _createLabelSprite(tex);
      labelGroup.add(sprite);
      entry = { sprite, lastSpeed: speed, lastState: aiState };
      _labelMap.set(id, entry);
    } else if (entry.lastSpeed !== speed || entry.lastState !== aiState) {
      // 文本变了才换纹理（避免每帧 GC）。
      // 车速每帧都在变（20.01→20.02…）→ 这里几乎每帧都换 → 每次 makeLabelTexture
      // 新建 CanvasTexture。**旧纹理必须 dispose**，否则 GPU/JS 内存无界泄漏
      // （浏览器 5GB，2026-08-04 实测）；CanvasTextureFactory 的缓存也因 speed key
      // 每帧新增条目而膨胀，故 label 纹理走 noCache 路径（见工厂）。
      const oldMap = entry.sprite.material.map;
      const newMap = makeLabelTexture(speed, aiState, { width: CANVAS_W, height: CANVAS_H, noCache: true });
      if (oldMap && oldMap !== newMap) oldMap.dispose();
      entry.sprite.material.map = newMap;
      entry.sprite.material.needsUpdate = true;
      entry.lastSpeed = speed;
      entry.lastState = aiState;
    }
    return entry.sprite;
  }

  /** 移除标签 */
  function _removeLabel(id) {
    const entry = _labelMap.get(id);
    if (!entry) return;
    labelGroup.remove(entry.sprite);
    entry.sprite.material.map?.dispose();
    entry.sprite.material.dispose();
    _labelMap.delete(id);
  }

  /** Layer 树每帧调用 */
  function update(store) {
    if (!store) return;

    const VEHICLE_TYPES = new Set(['car', 'suv', 'truck', 'pedestrian']);
    const activeIds = new Set();

    // 1. ego 标签（显示 speed + "ego"）
    if (store.ego) {
      activeIds.add('ego');
      const sprite = _ensureLabel('ego', store.ego.speed, 'ego');
      const [tx, ty, tz] = worldToThree(store.ego.x, store.ego.y, store.ego.z);
      sprite.position.set(tx, ty + LABEL_Y_OFFSET, tz);
    }

    // 2. NPC 标签
    const entities = store.entities || [];
    for (const ent of entities) {
      if (!ent || !ent.id) continue;
      if (!VEHICLE_TYPES.has(ent.type)) continue;
      activeIds.add(ent.id);
      const sprite = _ensureLabel(ent.id, ent.speed, ent.ai_state);
      const [tx, ty, tz] = worldToThree(ent.x, ent.y, ent.z);
      sprite.position.set(tx, ty + LABEL_Y_OFFSET, tz);
    }

    // 3. 删除消失的标签
    for (const id of Array.from(_labelMap.keys())) {
      if (!activeIds.has(id)) {
        _removeLabel(id);
      }
    }
  }

  function dispose() {
    for (const id of Array.from(_labelMap.keys())) {
      _removeLabel(id);
    }
    _labelMap.clear();
    scene.remove(labelGroup);
  }

  return { update, dispose };
}