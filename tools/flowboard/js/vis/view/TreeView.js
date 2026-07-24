/**
 * TreeView.js — 行道树（沿路两侧低模树木）
 *
 * 设计：
 *   - 2 个 InstancedMesh：树干（圆柱）+ 树冠（圆锥），共 2 draw call
 *   - TREE_SPACING = 40m，道路两侧交替放置
 *   - offset = halfWidth + 2.0m（树在路肩外）
 *   - 位置走 Coord.js placeOnRoad 的横向偏移，不裸 position.set
 *   - 纳入 vis:check（SceneDirector 注册）
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { getStdMaterial } from '../core/AssetFactory.js';
import { LANE_WIDTH, EDGE_TYPE } from '../core/Constants.js';
import { tangentToNormal } from '../math/Coord.js';

const TREE_SPACING = 40;   // 树间距（米）
const TREE_OFFSET  = 2.0;  // 树距路缘外距离（米）
const TRUNK_H      = 2.5;  // 树干高度
const TRUNK_R      = 0.12; // 树干半径
const CANOPY_H     = 2.0;  // 树冠高度
const CANOPY_R     = 1.5;  // 树冠半径

const COLOR_TRUNK  = 0x5c3a1e;
const COLOR_CANOPY = 0x2d6a1e;

export function createTreeView(scene) {
  let group = new THREE.Group();
  scene.add(group);

  let trunkMesh, canopyMesh;

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
    trunkMesh = canopyMesh = null;
  }

  function build(roadNetwork) {
    clear();
    if (!roadNetwork || !roadNetwork.edges || roadNetwork.edges.length === 0) return;

    // ── 第一遍：收集所有放置点 ──
    const slots = [];
    for (const edge of roadNetwork.edges) {
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

      for (let i = 1; i < spine.length; i++) {
        const dx = spine[i].px - spine[i - 1].px;
        const dz = spine[i].pz - spine[i - 1].pz;
        spine[i].cum = spine[i - 1].cum + Math.sqrt(dx * dx + dz * dz);
      }
      const totalLen = spine[spine.length - 1].cum;
      const count = Math.floor(totalLen / TREE_SPACING);
      if (count === 0) continue;

      for (let i = 0; i < count; i++) {
        const targetArc = (i + 0.5) * TREE_SPACING;
        let j = 1;
        while (j < spine.length && spine[j].cum < targetArc) j++;
        if (j >= spine.length) j = spine.length - 1;
        const s = spine[j];
        const side = (i % 2 === 0) ? 1 : -1;
        // 树位置：中心线 + 法线 × (halfWidth + TREE_OFFSET) × side
        const x = s.px + s.nx * (halfWidth + TREE_OFFSET) * side;
        const z = s.pz + s.nz * (halfWidth + TREE_OFFSET) * side;
        slots.push({ x, z, py: s.py });
      }
    }

    if (slots.length === 0) return;

    // ── 第二遍：构建 2 个 InstancedMesh ──
    const N = slots.length;
    const trunkGeo = new THREE.CylinderGeometry(TRUNK_R, TRUNK_R * 1.3, TRUNK_H, 8);
    const canopyGeo = new THREE.ConeGeometry(CANOPY_R, CANOPY_H, 8);

    const trunkMat = getStdMaterial(COLOR_TRUNK, 0.8, 0.5);
    const canopyMat = getStdMaterial(COLOR_CANOPY, 0.6, 0.7);

    trunkMesh = new THREE.InstancedMesh(trunkGeo, trunkMat, N);
    canopyMesh = new THREE.InstancedMesh(canopyGeo, canopyMat, N);

    const dummy = new THREE.Object3D();
    for (let i = 0; i < N; i++) {
      const s = slots[i];
      // 树干：竖直立在 (x, 0, z)，底部贴地
      dummy.position.set(s.x, TRUNK_H / 2, s.z);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      trunkMesh.setMatrixAt(i, dummy.matrix);

      // 树冠：在树干顶部
      dummy.position.set(s.x, TRUNK_H + CANOPY_H / 2, s.z);
      dummy.rotation.set(0, 0, 0);
      dummy.updateMatrix();
      canopyMesh.setMatrixAt(i, dummy.matrix);
    }
    trunkMesh.instanceMatrix.needsUpdate = true;
    canopyMesh.instanceMatrix.needsUpdate = true;

    group.add(trunkMesh, canopyMesh);
  }

  function getGroup() { return group; }

  return { build, clear, getGroup };
}