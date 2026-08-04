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

import { worldToThree, forwardENU } from '../math/Coord.js';

// ── 角标框几何体常量（模块级，供 _updateBrackets 使用） ──
const CORNER_LEN = 0.6;  // 角标臂长（米），稍短更精致
const BOX_COLOR = 0x00ff88;  // 科技青绿
const SWEEP_SEGMENTS = 32;
const SWEEP_RADIUS = 80;  // 米
// 每实体几何数据大小：每个角（top/bottom）3 segments（1 vertical + 2 horizontal）
const SEGMENTS_PER_CORNER = 3;
const VERTS_PER_SEGMENT = 2;
const COMPS_PER_VERT = 3;
const FLOATS_PER_CORNER = SEGMENTS_PER_CORNER * VERTS_PER_SEGMENT * COMPS_PER_VERT;  // 18
const FLOATS_PER_VEDGE = 2 * 3;       // 6
const CORNERS = 8;                    // 4 top + 4 bottom
const VEDGES = 4;
const FLOATS_PER_BOX = CORNERS * FLOATS_PER_CORNER + VEDGES * FLOATS_PER_VEDGE;  // 8*18+4*6 = 168
const VERTICES_PER_BOX = CORNERS * SEGMENTS_PER_CORNER * VERTS_PER_SEGMENT + VEDGES * 2; // 8*6+8 = 56
const MAX_ENTITIES = 32;

export function createPerceptionView(scene) {
  // ── 角标框 ──
  const boxPositions = new Float32Array(MAX_ENTITIES * FLOATS_PER_BOX);
  const boxGeo = new THREE.BufferGeometry();
  boxGeo.setAttribute('position', new THREE.BufferAttribute(boxPositions, 3));
  boxGeo.setDrawRange(0, 0);
  const boxMat = new THREE.LineBasicMaterial({
    color: BOX_COLOR,
    transparent: true,
    opacity: 0.9,
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
    color: BOX_COLOR,
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
    color: BOX_COLOR,
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
    boxGeo.setDrawRange(0, count * VERTICES_PER_BOX);

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
    const bi = i * FLOATS_PER_BOX;
    if (dist > 100 || dist < 2) {
      for (let j = 0; j < FLOATS_PER_BOX; j++) positions[bi + j] = 0;
      continue;
    }

    /* ── 按实体类型确定尺寸/高度（使用 ent.length/width 优先，兜底值按类型） ── */
    const etype = ent.type || 'car';
    let defLen, defWid, defH;
    switch (etype) {
      case 'pedestrian':
        defLen = 0.5; defWid = 0.5; defH = 1.75; break;
      case 'bicycle': case 'cyclist': case 'motorcycle':
        defLen = 1.8; defWid = 0.6; defH = 1.6; break;
      case 'truck': case 'bus':
        defLen = 8.0; defWid = 2.5; defH = 4.0; break;
      case 'suv':
        defLen = 4.7; defWid = 2.0; defH = 1.85; break;
      case 'car': default:
        defLen = 4.6; defWid = 2.0; defH = 1.5; break;
    }
    const halfL = (ent.length || defLen) / 2;
    const halfW = (ent.width  || defWid) / 2;
    const vehH  = ent.height || defH;

    /* 角标臂长：实体最小维度的 25%，但不超过 CORNER_LEN，也不小于 0.08m */
    const minDim = Math.min(halfL * 2, halfW * 2);
    const cl = Math.max(0.08, Math.min(CORNER_LEN, minDim * 0.25));

    // Entity ground Z (ENU up)
    const entZ = ent.z || 0;
    const [cx, , cz] = worldToThree(ent.x, ent.y, entZ);
    const cy = entZ;                          // THREE Y = ENU Z（地面高度）
    const roofY = cy + vehH;                  // THREE Y（顶部高度）

    // Coord 纯函数：heading → ENU 前向 [cos(h), sin(h)]
    const [fx, fz] = forwardENU(ent.heading || 0);
    const fwdX = fx, fwdZ = -fz;              // ENU → THREE 前向
    const sideX = -fwdZ, sideZ = fwdX;        // 左向（逆时针 90°）

    // 4 角：[前/后偏移, 左/右偏移]
    const cornerOffsets = [
      [ halfL,  halfW],  // 0: front-left
      [ halfL, -halfW],  // 1: front-right
      [-halfL, -halfW],  // 2: back-right
      [-halfL,  halfW],  // 3: back-left
    ];

    let idx = 0;
    function addCorner(base3X, base3Y, base3Z, isTop) {
      for (const [foff, soff] of cornerOffsets) {
        const cx3 = base3X + foff * fwdX + soff * sideX;
        const cz3 = base3Z + foff * fwdZ + soff * sideZ;

        const inFwd = -Math.sign(foff);
        const inSide = -Math.sign(soff);

        // 垂直短臂
        const vDir = isTop ? -1 : 1;
        positions[bi + idx++] = cx3; positions[bi + idx++] = base3Y;          positions[bi + idx++] = cz3;
        positions[bi + idx++] = cx3; positions[bi + idx++] = base3Y + vDir * cl; positions[bi + idx++] = cz3;

        // 沿长度方向向内
        positions[bi + idx++] = cx3; positions[bi + idx++] = base3Y; positions[bi + idx++] = cz3;
        positions[bi + idx++] = cx3 + inFwd * cl * fwdX; positions[bi + idx++] = base3Y; positions[bi + idx++] = cz3 + inFwd * cl * fwdZ;

        // 沿宽度方向向内
        positions[bi + idx++] = cx3; positions[bi + idx++] = base3Y; positions[bi + idx++] = cz3;
        positions[bi + idx++] = cx3 + inSide * cl * sideX; positions[bi + idx++] = base3Y; positions[bi + idx++] = cz3 + inSide * cl * sideZ;
      }
    }

    addCorner(cx, roofY, cz, true);   // 顶角（车顶）
    addCorner(cx, cy, cz, false);    // 底角（地面）

    // 4 条垂直连接棱
    for (const [foff, soff] of cornerOffsets) {
      const topX = cx + foff * fwdX + soff * sideX;
      const topZ = cz + foff * fwdZ + soff * sideZ;
      positions[bi + idx++] = topX; positions[bi + idx++] = roofY; positions[bi + idx++] = topZ;
      positions[bi + idx++] = topX; positions[bi + idx++] = cy;    positions[bi + idx++] = topZ;
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

    // 近端（ego 位置）—— 2026-08 提亮：旧值 (0, 0.2, 0) 暗绿 ≈ 黑线
    positions[baseIdx + 0] = ex;
    positions[baseIdx + 1] = ey;
    positions[baseIdx + 2] = ez;
    colors[baseIdx + 0] = 0.2;
    colors[baseIdx + 1] = 0.8;
    colors[baseIdx + 2] = 0.3;

    if (dist > 100) {
      // 远端缩到近端（不可见）
      positions[baseIdx + 3] = ex;
      positions[baseIdx + 4] = ey;
      positions[baseIdx + 5] = ez;
      colors[baseIdx + 3] = 0.2;
      colors[baseIdx + 4] = 0.8;
      colors[baseIdx + 5] = 0.3;
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
  // 雷达扫描扇形（2026-08 修复）：
  //  1. FOV 120°（匹配 sensor_model lidar_fov_deg=120）—— 旧值 90° 显示
  //     与实际传感器不符（用户反馈"雷达难道只有 90 度"）
  //  2. 车头前向 ±60°（随 ego heading 旋转）—— 旧值世界系固定角度
  //     （-135°~-45°）不随车头转，掉头/转向时扇形指错方向
  //  3. 轻微摆动（±8°）模拟扫描动画
  const heading = ego.heading || 0;
  const halfFov = Math.PI * 60 / 180;
  // 扫描摆动：三角波，约 1 秒一个完整来回（60 帧 @60fps），幅度 ±0.2（约 ±11°）。
  // 旧实现 ((sweepAngle*10)%0.6)-0.3 是高频锯齿，每 3 帧跳变一次 ±17°，导致线条"抽搐/扫太快"。
  const cycle = 60;
  const wobble = (Math.abs((frame % cycle) - cycle / 2) / (cycle / 2)) * 0.4 - 0.2;
  const sweepStart = heading - halfFov + wobble;
  const sweepEnd = heading + halfFov + wobble;

  const [ex, ey, ez] = worldToThree(ego.x, ego.y, (ego.z || 0) + 0.2);
  const R = SWEEP_RADIUS;

  // 扇形顶点：原点 + 弧顶点
  // Coord 纯函数：角度 t → 前向单位向量 [fx, fz]，然后乘以半径 R
  positions[0] = ex;
  positions[1] = ey;
  positions[2] = ez;
  for (let i = 0; i <= SWEEP_SEGMENTS; i++) {
    const t = sweepStart + (sweepEnd - sweepStart) * (i / SWEEP_SEGMENTS);
    const [fx, fz] = forwardENU(t);  // ENU 前向向量 [east, north]
    // ENU(east,north) → THREE(east, -north)，所以 Z 分量 = -fz*R
    positions[(i + 1) * 3 + 0] = ex + fx * R;
    positions[(i + 1) * 3 + 1] = ey;
    positions[(i + 1) * 3 + 2] = ez - fz * R;
  }

  // 前沿线（原点 → 弧终点）
  edgePositions[0] = ex;
  edgePositions[1] = ey;
  edgePositions[2] = ez;
  const [fxEnd, fzEnd] = forwardENU(sweepEnd);
  edgePositions[3] = ex + fxEnd * R;
  edgePositions[4] = ey;
  edgePositions[5] = ez - fzEnd * R;
}