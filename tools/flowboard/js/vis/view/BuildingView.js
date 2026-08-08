/**
 * BuildingView.js — 道路两侧城市楼群
 *
 * 2 个 InstancedMesh（楼体 + 屋顶），共享一张程序生成窗格贴图。
 * 无外网资产；性能档只改变实例密度，不改变 draw call。
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { directionToRotationY, tangentToNormal } from '../math/Coord.js';
import { EDGE_TYPE, LANE_WIDTH } from '../core/Constants.js';

const SPACING = { high: 55, medium: 80, low: 120 };
const SETBACK = 14;

function buildingTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 64;
  canvas.height = 128;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#273446';
  ctx.fillRect(0, 0, 64, 128);
  for (let y = 7; y < 124; y += 14) {
    for (let x = 6; x < 62; x += 14) {
      const lit = ((x * 7 + y * 13) % 5) !== 0;
      ctx.fillStyle = lit ? '#9ec7dc' : '#182433';
      ctx.fillRect(x, y, 8, 7);
      ctx.fillStyle = lit ? '#d7eef6' : '#223142';
      ctx.fillRect(x + 1, y + 1, 6, 2);
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.anisotropy = 2;
  return texture;
}

export function createBuildingView(scene) {
  const group = new THREE.Group();
  scene.add(group);

  function clear() {
    while (group.children.length) {
      const child = group.children[0];
      group.remove(child);
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (child.material.map) child.material.map.dispose();
        child.material.dispose();
      }
    }
  }

  function build(roadNetwork, store = {}) {
    clear();
    if (!roadNetwork || !Array.isArray(roadNetwork.edges)) return;
    const tier = store.perfTier || 'high';
    const spacing = SPACING[tier] || SPACING.high;
    const slots = [];

    for (const edge of roadNetwork.edges) {
      if (edge.type === EDGE_TYPE.VIADUCT_HIGHWAY ||
          edge.name === EDGE_TYPE.VIADUCT_HIGHWAY) continue;
      const raw = Array.isArray(edge.nodes) ? edge.nodes : [];
      if (raw.length < 2) continue;
      const nodes = raw.map((n) => Array.isArray(n) ? n : [n.x || 0, n.y || 0, n.z || 0]);
      const points = sampleEdgeNodes(nodes, 18);
      const lanes = edge.lanes || 2;
      const laneWidth = edge.lane_width || LANE_WIDTH;
      const offset = lanes * laneWidth / 2 + SETBACK;

      let distance = 0;
      for (let i = 3; i < points.length; i += 3) {
        const px = points[i], pz = points[i + 2];
        const prevX = points[i - 3], prevZ = points[i - 1];
        const segment = Math.hypot(px - prevX, pz - prevZ);
        distance += segment;
        if (distance < spacing) continue;
        distance = 0;
        const [nx, nz] = tangentToNormal(px - prevX, pz - prevZ);
        const hash = Math.abs(Math.trunc(px * 17 + pz * 31)) % 997;
        const seed = hash / 997;
        const height = 18 + seed * 42;
        const width = 12 + seed * 8;
        const depth = 10 + (1 - seed) * 8;
        for (const side of [-1, 1]) {
          slots.push({
            x: px + nx * offset * side,
            z: pz + nz * offset * side,
            height, width, depth,
            rotation: directionToRotationY(nx, nz),
          });
        }
      }
    }
    if (!slots.length) return;

    const facade = new THREE.MeshStandardMaterial({
      color: 0x8998a8,
      roughness: 0.78,
      metalness: 0.04,
      map: buildingTexture(),
    });
    const roof = new THREE.MeshStandardMaterial({
      color: 0x4a5664,
      roughness: 0.9,
    });
    const bodyGeo = new THREE.BoxGeometry(1, 1, 1);
    const roofGeo = new THREE.BoxGeometry(1, 1, 1);
    const bodies = new THREE.InstancedMesh(bodyGeo, facade, slots.length);
    const roofs = new THREE.InstancedMesh(roofGeo, roof, slots.length);
    const dummy = new THREE.Object3D();

    slots.forEach((slot, i) => {
      dummy.position.set(slot.x, slot.height / 2, slot.z);
      dummy.rotation.set(0, slot.rotation, 0);
      dummy.scale.set(slot.depth, slot.height, slot.width);
      dummy.updateMatrix();
      bodies.setMatrixAt(i, dummy.matrix);

      dummy.position.set(slot.x, slot.height + 0.5, slot.z);
      dummy.scale.set(slot.depth * 0.72, 1, slot.width * 0.72);
      dummy.updateMatrix();
      roofs.setMatrixAt(i, dummy.matrix);
    });
    bodies.instanceMatrix.needsUpdate = true;
    roofs.instanceMatrix.needsUpdate = true;
    bodies.castShadow = tier === 'high';
    bodies.receiveShadow = true;
    group.add(bodies, roofs);
  }

  return { build, clear, getGroup: () => group };
}
