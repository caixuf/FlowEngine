/**
 * StreetlightView.js — 路灯（普通道路场景）
 *
 * Step 4 重构：新增模块。Phase 3 View 清单 9 个中补的第 8 个。
 *
 * 仿真侧 scene_pub.cpp 只发道路几何（edges/nodes），不发路灯数据 —— 必须
 * 3D 层从 roadNetwork.edges 自动布局。
 *
 * 设计：
 *   - 4 个 InstancedMesh：pole / arm / head / glow（共 4 draw call）
 *   - LAMP_SPACING = 40m，道路两侧交替放置
 *   - offset = halfWidth + 1.5m（路灯在路肩外）
 *   - 跳过 type='viaduct_highway' 的 edge（高架场景路灯由 ViaductView 内置）
 *
 * 坐标约定：
 *   edge.nodes 经 sampleEdgeNodes 采样后输出 [x, y_up, z_north, ...]
 *   （ENU→THREE 交换由 sampleEdgeNodes 内部完成，详见 Curve.js）
 *   所以这里直接用 p.x / p.z，不要再调 worldToThree。
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { LANE_WIDTH, DEFAULT_LANES, EDGE_TYPE } from '../core/Constants.js';
import { tangentToNormal, directionToRotationY } from '../math/Coord.js';

const LAMP_SPACING = 40;   // 路灯间距（米）
const LAMP_OFFSET  = 1.5;  // 路灯距路缘外距离（米）
const POLE_H       = 7.0;  // 杆高
const ARM_LEN      = 1.8;  // 悬臂长度
const HEAD_SIZE    = 0.4;  // 灯头尺寸

const COLOR_POLE  = 0x3a3a3a;
const COLOR_HEAD  = 0xfff4d6;
const COLOR_GLOW  = 0xfff4d6;

export function createStreetlightView(scene) {
  let group = new THREE.Group();
  scene.add(group);

  /** InstancedMesh 池（build 时一次性创建，clear 时 dispose） */
  let poleMesh, armMesh, headMesh, glowMesh;

  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) {
        if (Array.isArray(c.material)) c.material.forEach(m => m.dispose());
        else c.material.dispose();
      }
    }
    poleMesh = armMesh = headMesh = glowMesh = null;
  }

  function build(roadNetwork) {
    clear();
    if (!roadNetwork || !roadNetwork.edges || roadNetwork.edges.length === 0) return;

    // ── 第一遍：收集所有放置点 ──
    const slots = [];  // [{x, z, nx, nz, side: +1|-1}]
    for (const edge of roadNetwork.edges) {
      // 高架场景路灯由 ViaductView 内置，这里跳过
      if (edge.type === EDGE_TYPE.VIADUCT_HIGHWAY || edge.name === EDGE_TYPE.VIADUCT_HIGHWAY) continue;

      let nodes = edge.nodes;
      if (!nodes || nodes.length < 2) continue;
      if (nodes[0] && typeof nodes[0] === 'object' && !Array.isArray(nodes[0])) {
        nodes = nodes.map(n => [n.x || 0, n.y || 0, n.z || 0]);
      }

      const points = sampleEdgeNodes(nodes, 24);
      const lanes = edge.lanes || 2;
      const laneWidth = edge.lane_width || LANE_WIDTH;
      const halfWidth = (lanes * laneWidth) / 2;

      // 中心线 spine + 沿弧长 march，每 LAMP_SPACING 米放一盏
      const spine = [];
      for (let i = 0; i < points.length; i += 3) {
        const px = points[i], py = points[i + 1], pz = points[i + 2];
        let tx = 1, tz = 0;
        if (i + 6 < points.length) { tx = points[i + 3] - px; tz = points[i + 5] - pz; }
        else if (i >= 3) { tx = px - points[i - 3]; tz = pz - points[i - 2]; }
        const [nx, nz] = tangentToNormal(tx, tz);
        spine.push({ px, py, pz, nx, nz, cum: 0 });
      }
      if (spine.length < 2) continue;

      // 沿弧长累计
      for (let i = 1; i < spine.length; i++) {
        const dx = spine[i].px - spine[i - 1].px;
        const dz = spine[i].pz - spine[i - 1].pz;
        spine[i].cum = spine[i - 1].cum + Math.sqrt(dx * dx + dz * dz);
      }
      const totalLen = spine[spine.length - 1].cum;
      const count = Math.floor(totalLen / LAMP_SPACING);
      if (count === 0) continue;

      for (let i = 0; i < count; i++) {
        const targetArc = (i + 0.5) * LAMP_SPACING;
        // 找到 spine 中 cum >= targetArc 的点
        let j = 1;
        while (j < spine.length && spine[j].cum < targetArc) j++;
        if (j >= spine.length) j = spine.length - 1;
        const s = spine[j];
        // 交替两侧：偶数 i 放左侧（+1），奇数 i 放右侧（-1）
        const side = (i % 2 === 0) ? 1 : -1;
        // 路灯位置：中心线 + 法线 × (halfWidth + LAMP_OFFSET) × side
        const x = s.px + s.nx * (halfWidth + LAMP_OFFSET) * side;
        const z = s.pz + s.nz * (halfWidth + LAMP_OFFSET) * side;
        slots.push({ x, z, nx: s.nx, nz: s.nz, side });
      }
    }

    if (slots.length === 0) return;

    // ── 第二遍：构建 4 个 InstancedMesh ──
    const N = slots.length;
    const poleGeo = new THREE.CylinderGeometry(0.08, 0.12, POLE_H, 8);
    const armGeo  = new THREE.BoxGeometry(ARM_LEN, 0.08, 0.08);
    const headGeo = new THREE.BoxGeometry(HEAD_SIZE, HEAD_SIZE * 0.5, HEAD_SIZE * 1.5);
    const glowGeo = new THREE.SphereGeometry(HEAD_SIZE * 0.6, 8, 6);

    const poleMat = getStdMaterial(COLOR_POLE, 0.6, 0.3);
    const armMat  = getStdMaterial(COLOR_POLE, 0.6, 0.3);
    const headMat = getStdMaterial(COLOR_HEAD, 0.3, 0.6);
    const glowMat = createEmissiveMaterial(COLOR_GLOW, 1.2);

    poleMesh = new THREE.InstancedMesh(poleGeo, poleMat, N);
    armMesh  = new THREE.InstancedMesh(armGeo, armMat, N);
    headMesh = new THREE.InstancedMesh(headGeo, headMat, N);
    glowMesh = new THREE.InstancedMesh(glowGeo, glowMat, N);

    const dummy = new THREE.Object3D();
    for (let i = 0; i < N; i++) {
      const s = slots[i];
      // ── 杆：竖直立在 (x, 0, z) ──
      // 注意：sampleEdgeNodes 已做 ENU→THREE 交换，p.x / p.z 已是 THREE 坐标，
      // 不要再调 worldToThree（会双交换让灯钻地底）
      dummy.position.set(s.x, POLE_H / 2, s.z);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      poleMesh.setMatrixAt(i, dummy.matrix);

      // ── 悬臂：从杆顶向道路方向延伸 ──
      // 朝向道路中心 = -法线方向（路灯在路肩外，悬臂伸回路上方）
      const dirX = -s.nx * s.side;
      const dirZ = -s.nz * s.side;
      const armRotY = directionToRotationY(dirX, dirZ);  // 旋转使 BoxGeometry 的 +X 朝向 dir
      dummy.position.set(s.x + dirX * ARM_LEN / 2, POLE_H, s.z + dirZ * ARM_LEN / 2);
      dummy.rotation.set(0, armRotY, 0);
      dummy.updateMatrix();
      armMesh.setMatrixAt(i, dummy.matrix);

      // ── 灯头：在悬臂末端，朝下照路 ──
      dummy.position.set(s.x + dirX * ARM_LEN, POLE_H - 0.15, s.z + dirZ * ARM_LEN);
      dummy.rotation.set(0, armRotY, 0);
      dummy.updateMatrix();
      headMesh.setMatrixAt(i, dummy.matrix);

      // ── 灯光辉光：在灯头位置 ──
      dummy.position.set(s.x + dirX * ARM_LEN, POLE_H - 0.2, s.z + dirZ * ARM_LEN);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      glowMesh.setMatrixAt(i, dummy.matrix);
    }
    poleMesh.instanceMatrix.needsUpdate = true;
    armMesh.instanceMatrix.needsUpdate = true;
    headMesh.instanceMatrix.needsUpdate = true;
    glowMesh.instanceMatrix.needsUpdate = true;

    group.add(poleMesh, armMesh, headMesh, glowMesh);
  }

  function getGroup() { return group; }

  return { build, clear, getGroup };
}
