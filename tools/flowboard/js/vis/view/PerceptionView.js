/**
 * PerceptionView.js — 3D 感知标注覆盖层
 *
 * 在 3D 场景中叠加感知风格的可视化元素：
 * - 绿色角标框（corner brackets）包围检测到的实体
 * - 检测射线（detection rays）从 ego 指向每个实体
 * - 雷达扫描扇形（radar sweep）动画
 *
 * 所有坐标使用 worldToThree 映射，零魔法数。
 */

import { worldToThree } from '../math/Coord.js';

// ── 角标框几何体（共享，每帧更新位置） ──
const CORNER_LEN = 0.8;  // 角标臂长（米）
const BOX_COLOR = 0x00ff88;
const SWEEP_SEGMENTS = 32;
const SWEEP_RADIUS = 80;  // 米

export function createPerceptionView(scene) {
  // ── 角标框 ──
  // 每实体 4 个 L 形角，每角 2 个线段 → 每实体 8 顶点对
  // 最大同时跟踪 32 实体
  const MAX_ENTITIES = 32;
  const FLOATS_PER_BOX = 8 * 3 * 2;  // 8 线段 × 2 端点 × 3 分量
  const boxPositions = new Float32Array(MAX_ENTITIES * FLOATS_PER_BOX);
  const boxGeo = new THREE.BufferGeometry();
  boxGeo.setAttribute('position', new THREE.BufferAttribute(boxPositions, 3));
  boxGeo.setDrawRange(0, 0);
  const boxMat = new THREE.LineBasicMaterial({
    color: BOX_COLOR,
    transparent: true,
    opacity: 0.85,
    depthTest: true,
    depthWrite: false,
  });
  const boxMesh = new THREE.LineSegments(boxGeo, boxMat);
  boxMesh.frustumCulled = false;
  scene.add(boxMesh);

  // ── 检测射线 ──
  // 每实体 1 条线段 × 2 端点
  const rayPositions = new Float32Array(MAX_ENTITIES * 2 * 3);
  const rayColors = new Float32Array(MAX_ENTITIES * 2 * 3);
  const rayGeo = new THREE.BufferGeometry();
  rayGeo.setAttribute('position', new THREE.BufferAttribute(rayPositions, 3));
  rayGeo.setAttribute('color', new THREE.BufferAttribute(rayColors, 3));
  rayGeo.setDrawRange(0, 0);
  const rayMat = new THREE.LineBasicMaterial({
    vertexColors: true,
    transparent: true,
    opacity: 0.3,
    depthTest: false,
    depthWrite: false,
  });
  const rayMesh = new THREE.LineSegments(rayGeo, rayMat);
  rayMesh.frustumCulled = false;
  scene.add(rayMesh);

  // ── 雷达扫描扇形 ──
  const sweepPositions = new Float32Array((SWEEP_SEGMENTS + 2) * 3);
  const sweepGeo = new THREE.BufferGeometry();
  sweepGeo.setAttribute('position', new THREE.BufferAttribute(sweepPositions, 3));
  const sweepMat = new THREE.MeshBasicMaterial({
    color: 0x00ff88,
    transparent: true,
    opacity: 0.06,
    side: THREE.DoubleSide,
    depthWrite: false,
    depthTest: false,
  });
  const sweepMesh = new THREE.Mesh(sweepGeo, sweepMat);
  sweepMesh.frustumCulled = false;
  scene.add(sweepMesh);

  // 扫描前沿线
  const sweepEdgePositions = new Float32Array(2 * 3);
  const sweepEdgeGeo = new THREE.BufferGeometry();
  sweepEdgeGeo.setAttribute('position', new THREE.BufferAttribute(sweepEdgePositions, 3));
  const sweepEdgeMat = new THREE.LineBasicMaterial({
    color: 0x00ff88,
    transparent: true,
    opacity: 0.25,
    depthWrite: false,
    depthTest: false,
  });
  const sweepEdgeMesh = new THREE.Line(sweepEdgeGeo, sweepEdgeMat);
  sweepEdgeMesh.frustumCulled = false;
  scene.add(sweepEdgeMesh);

  // ── 内部状态 ──
  let _entityCount = 0;
  let _frame = 0;

  function update(store, now) {
    _frame++;
    const ego = store.ego;
    if (!ego) {
      boxMesh.visible = false;
      rayMesh.visible = false;
      sweepMesh.visible = false;
      sweepEdgeMesh.visible = false;
      return;
    }
    boxMesh.visible = true;
    rayMesh.visible = true;
    sweepMesh.visible = true;
    sweepEdgeMesh.visible = true;

    const entities = store.entities || [];
    const count = Math.min(entities.length, MAX_ENTITIES);

    // ── 更新角标框 ──
    _updateBrackets(ego, entities, count, boxPositions);
    boxGeo.attributes.position.needsUpdate = true;
    boxGeo.setDrawRange(0, count * 8 * 2);

    // ── 更新检测射线 ──
    _updateRays(ego, entities, count, rayPositions, rayColors);
    rayGeo.attributes.position.needsUpdate = true;
    rayGeo.attributes.color.needsUpdate = true;
    rayGeo.setDrawRange(0, count * 2);

    // ── 更新雷达扫描 ──
    _updateSweep(ego, _frame, sweepPositions, sweepEdgePositions);
    sweepGeo.attributes.position.needsUpdate = true;
    sweepEdgeGeo.attributes.position.needsUpdate = true;

    _entityCount = count;
  }

  function clear() {
    _entityCount = 0;
    boxGeo.setDrawRange(0, 0);
    rayGeo.setDrawRange(0, 0);
    boxMesh.visible = false;
    rayMesh.visible = false;
    sweepMesh.visible = false;
    sweepEdgeMesh.visible = false;
  }

  function dispose() {
    scene.remove(boxMesh);
    scene.remove(rayMesh);
    scene.remove(sweepMesh);
    scene.remove(sweepEdgeMesh);
    boxGeo.dispose();
    boxMat.dispose();
    rayGeo.dispose();
    rayMat.dispose();
    sweepGeo.dispose();
    sweepMat.dispose();
    sweepEdgeGeo.dispose();
    sweepEdgeMat.dispose();
  }

  return { update, clear, dispose };
}

// ── 内部函数 ──

function _updateBrackets(ego, entities, count, positions) {
  for (let i = 0; i < count; i++) {
    const ent = entities[i];
    if (!ent) continue;
    const dx = ent.x - ego.x;
    const dy = ent.y - ego.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const bi = i * 8 * 2 * 3;
    if (dist > 100 || dist < 2) {
      for (let j = 0; j < 8 * 2 * 3; j++) positions[bi + j] = 0;
      continue;
    }

    const halfL = (ent.length || 4.6) / 2;
    const halfW = (ent.width || 2.0) / 2;
    const H = 1.5;
    const cl = Math.min(0.8, Math.min(halfL, halfW) * 0.3);
    const rotY = -ent.heading;
    const cosA = Math.cos(rotY);
    const sinA = Math.sin(rotY);
    const [cx, cy, cz] = worldToThree(ent.x, ent.y, (ent.z || 0) + H / 2);

    // 局部 4 个上角点（车体朝前 +X）
    const localTops = [
      [-halfL,  halfW,  H],  // 后左
      [ halfL,  halfW,  H],  // 前左
      [ halfL, -halfW,  H],  // 前右
      [-halfL, -halfW,  H],  // 后右
    ];
    // 每个角：垂直向下 cl，水平向里 cl → 共 2 条线段
    let idx = 0;
    for (const [lx, ly, lz] of localTops) {
      // 角点 THREE 世界坐标
      const wx = cx + lx * cosA - ly * sinA;
      const wy = cy + lz;
      const wz = cz + lx * sinA + ly * cosA;
      // 垂直向下
      positions[bi + idx++] = wx; positions[bi + idx++] = wy; positions[bi + idx++] = wz;
      positions[bi + idx++] = wx; positions[bi + idx++] = wy - cl; positions[bi + idx++] = wz;
      // 水平向里（沿 -lx, -ly 方向）
      const inDir = Math.sqrt(lx * lx + ly * ly) || 1;
      const inX = -lx / inDir * cl;
      const inY = -ly / inDir * cl;
      const hwx = wx + inX * cosA - inY * sinA;
      const hwy = wy;
      const hwz = wz + inX * sinA + inY * cosA;
      positions[bi + idx++] = wx; positions[bi + idx++] = wy; positions[bi + idx++] = wz;
      positions[bi + idx++] = hwx; positions[bi + idx++] = hwy; positions[bi + idx++] = hwz;
    }
  }
}

function _updateRays(ego, entities, count, positions, colors) {
  const [ex, ey, ez] = worldToThree(ego.x, ego.y, (ego.z || 0) + 0.5);

  for (let i = 0; i < count; i++) {
    const ent = entities[i];
    if (!ent) continue;
    const dx = ent.x - ego.x;
    const dy = ent.y - ego.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const baseIdx = i * 2 * 3;

    // 近端（ego 位置）
    positions[baseIdx + 0] = ex;
    positions[baseIdx + 1] = ey;
    positions[baseIdx + 2] = ez;
    colors[baseIdx + 0] = 0;
    colors[baseIdx + 1] = 0.2;
    colors[baseIdx + 2] = 0;

    if (dist > 100) {
      // 远端缩到近端（不可见）
      positions[baseIdx + 3] = ex;
      positions[baseIdx + 4] = ey;
      positions[baseIdx + 5] = ez;
      colors[baseIdx + 3] = 0;
      colors[baseIdx + 4] = 0.2;
      colors[baseIdx + 5] = 0;
      continue;
    }

    const [tx, ty, tz] = worldToThree(ent.x, ent.y, (ent.z || 0) + 0.5);
    positions[baseIdx + 3] = tx;
    positions[baseIdx + 4] = ty;
    positions[baseIdx + 5] = tz;
    colors[baseIdx + 3] = 0;
    colors[baseIdx + 4] = 1;
    colors[baseIdx + 5] = 0.5;
  }
}

function _updateSweep(ego, frame, positions, edgePositions) {
  // 扫描角度随帧变化
  const sweepAngle = (frame * 0.02) % (Math.PI * 2);
  const sweepStart = -Math.PI * 0.75 + sweepAngle * 0.3;
  const sweepEnd = sweepStart + Math.PI * 0.5;

  const [ex, ey, ez] = worldToThree(ego.x, ego.y, (ego.z || 0) + 0.2);
  const R = SWEEP_RADIUS;

  // 扇形顶点：原点 + 弧顶点
  positions[0] = ex;
  positions[1] = ey;
  positions[2] = ez;
  for (let i = 0; i <= SWEEP_SEGMENTS; i++) {
    const t = sweepStart + (sweepEnd - sweepStart) * (i / SWEEP_SEGMENTS);
    positions[(i + 1) * 3 + 0] = ex + Math.cos(t) * R;
    positions[(i + 1) * 3 + 1] = ey;
    positions[(i + 1) * 3 + 2] = ez + Math.sin(t) * R;
  }

  // 前沿线（原点 → 弧终点）
  edgePositions[0] = ex;
  edgePositions[1] = ey;
  edgePositions[2] = ez;
  edgePositions[3] = ex + Math.cos(sweepEnd) * R;
  edgePositions[4] = ey;
  edgePositions[5] = ez + Math.sin(sweepEnd) * R;
}

// 不再使用模块级帧计数器——使用闭包内 _frame