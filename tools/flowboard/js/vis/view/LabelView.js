/**
 * LabelView.js — 实体标签（NPC 头顶 speed + ai_state 文字）
 *
 * 使用 THREE.Sprite + CanvasTexture 渲染文字标签，始终面向相机。
 * 通过 Layer 树每帧 update(store) 驱动，与 VehicleView 共享 agent 层。
 *
 * 每帧 diff store.entities 与内部 Map，自动创建/更新/删除标签。
 * 标签文字格式：speed 在前，ai_state 在后（若 ai_state 非空）。
 */

import { worldToThree } from '../math/Coord.js';

const LABEL_SCALE = 2.0;       // 标签世界大小
const LABEL_Y_OFFSET = 2.8;    // 头顶上方偏移
const FONT_SIZE = 28;          // Canvas 字体大小
const CANVAS_W = 256;          // 标签纹理分辨率
const CANVAS_H = 64;

/** 创建文字 canvas（每帧重建，因为文本变化） */
function _makeLabelCanvas(speed, aiState) {
  const canvas = document.createElement('canvas');
  canvas.width = CANVAS_W;
  canvas.height = CANVAS_H;
  const ctx = canvas.getContext('2d');

  // 透明背景 + 圆角矩形底色
  ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);

  // 速度文本（大号，白色）
  const speedText = (speed != null ? speed.toFixed(1) : '?') + ' m/s';
  ctx.font = `bold ${FONT_SIZE}px 'JetBrains Mono', 'Inter', monospace, sans-serif`;
  const speedMetrics = ctx.measureText(speedText);
  const speedW = speedMetrics.width;

  // ai_state 文本（小号，灰色）
  let stateText = '';
  let stateW = 0;
  if (aiState && aiState !== '') {
    stateText = aiState;
    ctx.font = `${FONT_SIZE * 0.7}px 'Inter', sans-serif`;
    stateW = ctx.measureText(stateText).width;
  }

  // 总宽度 = 速度 + 间隔 + 状态
  const gap = stateText ? 16 : 0;
  const totalW = speedW + gap + stateW;
  const startX = (CANVAS_W - totalW) / 2;

  // 绘制背景圆角矩形容器
  const padX = 14, padY = 8;
  const bgX = startX - padX;
  const bgW = totalW + padX * 2;
  const bgY = (CANVAS_H - FONT_SIZE) / 2 - padY;
  const bgH = FONT_SIZE + padY * 2;

  ctx.beginPath();
  ctx.roundRect(bgX, bgY, bgW, bgH, 8);
  ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
  ctx.lineWidth = 1;
  ctx.stroke();

  // 速度文字
  ctx.font = `bold ${FONT_SIZE}px 'JetBrains Mono', 'Inter', monospace, sans-serif`;
  ctx.textBaseline = 'middle';
  ctx.fillStyle = '#ffffff';
  ctx.fillText(speedText, startX, CANVAS_H / 2);

  // ai_state 文字
  if (stateText) {
    ctx.font = `${FONT_SIZE * 0.7}px 'Inter', sans-serif`;
    ctx.fillStyle = '#8b949e';
    ctx.fillText(stateText, startX + speedW + gap, CANVAS_H / 2);
  }

  return canvas;
}

/** 从 canvas 创建 Sprite 标签 */
function _createLabelSprite(canvas) {
  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
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
      const canvas = _makeLabelCanvas(speed, aiState);
      const sprite = _createLabelSprite(canvas);
      labelGroup.add(sprite);
      entry = { sprite, lastSpeed: speed, lastState: aiState };
      _labelMap.set(id, entry);
    } else if (entry.lastSpeed !== speed || entry.lastState !== aiState) {
      // 文本变了才重建 canvas（避免每帧 GC）
      const canvas = _makeLabelCanvas(speed, aiState);
      entry.sprite.material.map = new THREE.CanvasTexture(canvas);
      entry.sprite.material.map.needsUpdate = true;
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