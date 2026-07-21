/**
 * BarrierView.js — 护栏（普通道路场景）
 *
 * Step 4 重构：新增模块。Phase 3 View 清单 9 个中补的第 9 个。
 *
 * 仿真侧 scene_pub.cpp 只发道路几何，不发护栏数据 —— 必须 3D 层
 * 从 roadNetwork.edges 自动布局。
 *
 * 设计：
 *   - 3 个 InstancedMesh：1 个 post + 2 个 beam 层（上下横梁）
 *   - POST_SPACING = 3m，道路两侧对称放置
 *   - offset = halfWidth + 0.5m（紧贴路肩）
 *   - 跳过 type='viaduct_highway'（高架护栏由 ViaductView 内置）
 *
 * 坐标约定：
 *   edge.nodes 经 sampleEdgeNodes 采样后输出 [x, y_up, z_north, ...]
 *   （ENU→THREE 交换由 sampleEdgeNodes 内部完成，详见 Curve.js）
 *   所以这里直接用 p.x / p.z，不要再调 worldToThree。
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { getStdMaterial } from '../core/AssetFactory.js';

const POST_SPACING   = 3.0;  // 立柱间距（米）
const POST_OFFSET    = 0.5;  // 护栏距路缘外距离（米）
const POST_H         = 0.9;  // 立柱高度
const BEAM_THICKNESS = 0.08; // 横梁厚度
const BEAM_W         = 0.18; // 横梁宽度
const BEAM_UPPER_Y   = 0.75; // 上横梁高度
const BEAM_LOWER_Y   = 0.30; // 下横梁高度
const SEGMENT_LEN    = 3.0;  // 横梁分段长度（与立柱间距对齐）

const COLOR_POST = 0xc0c0c0;
const COLOR_BEAM = 0xc0c0c0;

export function createBarrierView(scene) {
  let group = new THREE.Group();
  scene.add(group);

  let postMesh, upperBeamMesh, lowerBeamMesh;

  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
    postMesh = upperBeamMesh = lowerBeamMesh = null;
  }

  function build(roadNetwork) {
    clear();
    if (!roadNetwork || !roadNetwork.edges || roadNetwork.edges.length === 0) return;

    // ── 第一遍：收集所有立柱位置 + 横梁段位置 ──
    const posts = [];        // [{x, y, z, rotY}]
    const beamSegs = [];     // [{x1, z1, x2, z2, y}] 横梁段（用 box + 旋转表达）
    for (const edge of roadNetwork.edges) {
      if (edge.type === 'viaduct_highway' || edge.name === 'viaduct_highway') continue;

      let nodes = edge.nodes;
      if (!nodes || nodes.length < 2) continue;
      if (nodes[0] && typeof nodes[0] === 'object' && !Array.isArray(nodes[0])) {
        nodes = nodes.map(n => [n.x || 0, n.y || 0, n.z || 0]);
      }

      const points = sampleEdgeNodes(nodes, 24);
      const lanes = edge.lanes || 2;
      const laneWidth = edge.lane_width || 3.5;
      const halfWidth = (lanes * laneWidth) / 2;

      // 中心线 spine + 沿弧长 march
      const spine = [];
      for (let i = 0; i < points.length; i += 3) {
        const px = points[i], py = points[i + 1], pz = points[i + 2];
        let tx = 1, tz = 0;
        if (i + 6 < points.length) { tx = points[i + 3] - px; tz = points[i + 5] - pz; }
        else if (i >= 3) { tx = px - points[i - 3]; tz = pz - points[i - 2]; }
        const l = Math.sqrt(tx * tx + tz * tz) || 1;
        tx /= l; tz /= l;
        spine.push({ px, py, pz, nx: -tz, nz: tx, cum: 0 });
      }
      if (spine.length < 2) continue;
      for (let i = 1; i < spine.length; i++) {
        const dx = spine[i].px - spine[i - 1].px;
        const dz = spine[i].pz - spine[i - 1].pz;
        spine[i].cum = spine[i - 1].cum + Math.sqrt(dx * dx + dz * dz);
      }
      const totalLen = spine[spine.length - 1].cum;
      const postCount = Math.floor(totalLen / POST_SPACING);
      if (postCount === 0) continue;

      // 两侧对称放置
      for (const side of [+1, -1]) {
        const sideOffset = (halfWidth + POST_OFFSET) * side;
        const prevPost = { x: null, z: null };
        for (let i = 0; i <= postCount; i++) {
          const targetArc = i * POST_SPACING;
          let j = 1;
          while (j < spine.length && spine[j].cum < targetArc) j++;
          if (j >= spine.length) j = spine.length - 1;
          const s = spine[j];
          const x = s.px + s.nx * sideOffset;
          const z = s.pz + s.nz * sideOffset;
          // 立柱旋转：让 box 沿道路切向
          const rotY = Math.atan2(s.nz, s.nx);  // 切向 = (tz, tx)，法向 = (-tz, tx) = (nz, nx)
          posts.push({ x, y: POST_H / 2, z, rotY });

          // 横梁段（除第一根外，每根立柱连一段到前一根）
          if (prevPost.x !== null) {
            const midX = (prevPost.x + x) / 2;
            const midZ = (prevPost.z + z) / 2;
            const dx = x - prevPost.x;
            const dz = z - prevPost.z;
            const len = Math.sqrt(dx * dx + dz * dz);
            if (len > 0.01) {
              const beamRotY = Math.atan2(dx, dz);
              beamSegs.push({ x: midX, z: midZ, len, rotY: beamRotY, y: BEAM_UPPER_Y });
              beamSegs.push({ x: midX, z: midZ, len, rotY: beamRotY, y: BEAM_LOWER_Y });
            }
          }
          prevPost.x = x;
          prevPost.z = z;
        }
      }
    }

    if (posts.length === 0) return;

    // ── 第二遍：构建 InstancedMesh ──
    const N = posts.length;
    const postGeo = new THREE.BoxGeometry(BEAM_THICKNESS, POST_H, BEAM_THICKNESS);
    const postMat = getStdMaterial(COLOR_POST, 0.6, 0.3);
    postMesh = new THREE.InstancedMesh(postGeo, postMat, N);

    const M = beamSegs.length;
    const beamGeo = new THREE.BoxGeometry(SEGMENT_LEN, BEAM_W, BEAM_THICKNESS);
    const beamMat = getStdMaterial(COLOR_BEAM, 0.6, 0.3);
    upperBeamMesh = new THREE.InstancedMesh(beamGeo, beamMat, M);
    lowerBeamMesh = new THREE.InstancedMesh(beamGeo, beamMat, M);

    const dummy = new THREE.Object3D();
    for (let i = 0; i < N; i++) {
      const p = posts[i];
      dummy.position.set(p.x, p.y, p.z);
      dummy.rotation.set(0, p.rotY, 0);
      dummy.scale.set(1, 1, 1);
      dummy.updateMatrix();
      postMesh.setMatrixAt(i, dummy.matrix);
    }
    for (let i = 0; i < M; i++) {
      const b = beamSegs[i];
      // 按 SEGMENT_LEN 长度切，scale 调整真实长度
      const scale = b.len / SEGMENT_LEN;
      dummy.position.set(b.x, b.y, b.z);
      dummy.rotation.set(0, b.rotY, 0);
      dummy.scale.set(scale, 1, 1);
      dummy.updateMatrix();
      if (b.y > 0.5) {
        upperBeamMesh.setMatrixAt(i, dummy.matrix);
      } else {
        lowerBeamMesh.setMatrixAt(i, dummy.matrix);
      }
    }
    postMesh.instanceMatrix.needsUpdate = true;
    upperBeamMesh.instanceMatrix.needsUpdate = true;
    lowerBeamMesh.instanceMatrix.needsUpdate = true;

    group.add(postMesh, upperBeamMesh, lowerBeamMesh);
  }

  function getGroup() { return group; }

  return { build, clear, getGroup };
}
