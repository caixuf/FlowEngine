/**
 * RoadView.js — 路面 ribbon + 车道线
 * 沥青 #2a2a2a 路面 + 白色边线/虚线车道分隔 + 黄色中心线。
 * 车道线手法移植自 docs/scene.html（materials + polygonOffset 防 z-fight），
 * 但几何由 road_network 数据驱动（沿采样中心线按横向偏移铺设）。
 * road_network 变化时重建，ego 位姿变化不重建。
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { mergeGeometries } from '../math/GeometryMerge.js';

const ASPHALT_COLOR = 0x2a2a2a;
const LINE_WHITE = 0xffffff;
const LINE_YELLOW = 0xffd700;

const LINE_W = 0.15;      // 车道线宽度 (m)
const EDGE_INSET = 0.25;  // 边线相对路缘内缩 (m)
const Y_ROAD = 0.10;      // 路面高度
const Y_MARK = 0.13;      // 车道线高度（略高于路面防 z-fight）
const DASH = 3.0;         // 虚线段长 (m)
const GAP = 6.0;          // 虚线间隔 (m)

export function createRoadView(scene) {
  let roadGroup = new THREE.Group();
  scene.add(roadGroup);
  let built = false;

  // ── 几何辅助 ──

  /** 从中心线样点（含法线）+ 半宽构建朝上的 ribbon 几何体。
   *  centers: [{px,py,pz,nx,nz}]，法线在 XZ 平面。yOff: 抬高量。 */
  function ribbonGeo(centers, halfW, yOff) {
    if (centers.length < 2) return null;
    const positions = [], indices = [], uvs = [];
    for (let k = 0; k < centers.length; k++) {
      const c = centers[k];
      positions.push(c.px + c.nx * halfW, c.py + yOff, c.pz + c.nz * halfW); // 左
      positions.push(c.px - c.nx * halfW, c.py + yOff, c.pz - c.nz * halfW); // 右
      uvs.push(0, k); uvs.push(1, k);
    }
    // 缠绕：法线朝 +Y（与主路面一致，见 tests/vis_geometry.test.mjs）
    const vertCount = positions.length / 3;
    for (let i = 0; i < vertCount - 2; i += 2) {
      indices.push(i, i + 2, i + 1);
      indices.push(i + 1, i + 2, i + 3);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  }

  /** 把中心线整体横向偏移 d（沿各点法线方向） */
  function offsetSpine(spine, d) {
    return spine.map(c => ({
      px: c.px + c.nx * d, py: c.py, pz: c.pz + c.nz * d, nx: c.nx, nz: c.nz,
    }));
  }

  /** 实线：沿偏移中心线铺一条连续窄 ribbon */
  function solidLine(spine, d) {
    return ribbonGeo(offsetSpine(spine, d), LINE_W / 2, Y_MARK);
  }

  /** 虚线：沿偏移中心线按弧长 march，每 (DASH+GAP) 铺一段 */
  function dashedLine(spine, d) {
    const centers = offsetSpine(spine, d);
    const cum = [0];
    for (let i = 1; i < centers.length; i++) {
      const a = centers[i - 1], b = centers[i];
      cum.push(cum[i - 1] + Math.hypot(b.px - a.px, b.pz - a.pz));
    }
    const total = cum[cum.length - 1];
    const sampleAt = (s) => {
      if (s <= 0) return centers[0];
      if (s >= total) return centers[centers.length - 1];
      let i = 1; while (i < cum.length && cum[i] < s) i++;
      const t = (s - cum[i - 1]) / ((cum[i] - cum[i - 1]) || 1);
      const a = centers[i - 1], b = centers[i];
      return {
        px: a.px + (b.px - a.px) * t, py: a.py + (b.py - a.py) * t,
        pz: a.pz + (b.pz - a.pz) * t, nx: a.nx + (b.nx - a.nx) * t,
        nz: a.nz + (b.nz - a.nz) * t,
      };
    };
    const geos = [];
    for (let s = 0; s < total; s += DASH + GAP) {
      const g = ribbonGeo([sampleAt(s), sampleAt(Math.min(s + DASH, total))], LINE_W / 2, 0);
      if (g) geos.push(g);
    }
    return geos;
  }

  /** 从 edge nodes 构建路面 + 车道线 */
  function build(roadNetwork) {
    while (roadGroup.children.length) {
      const c = roadGroup.children[0];
      roadGroup.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
    built = false;

    if (!roadNetwork || !roadNetwork.edges || roadNetwork.edges.length === 0) return;

    const roadGeos = [];
    const whiteGeos = [];
    const yellowGeos = [];

    for (const edge of roadNetwork.edges) {
      // 兼容三种 edge 格式（见 scene_pub 输出）
      let nodes = edge.nodes;
      if (!nodes || nodes.length < 2) {
        const len = edge.length_m || 100;
        const h = edge.heading || 0;
        const sx = edge.start_x || 0, sz = edge.start_z || 0;
        nodes = [[sx, sz, 0], [sx + Math.cos(h) * len, sz + Math.sin(h) * len, 0]];
      } else if (nodes[0] && typeof nodes[0] === 'object' && !Array.isArray(nodes[0])) {
        // 对象 {x,y,z}(ENU) → 数组 [x,y,z](ENU)；轴交换交给 sampleEdgeNodes
        nodes = nodes.map(n => [n.x || 0, n.y || 0, n.z || 0]);
      }

      const points = sampleEdgeNodes(nodes, 24);
      const lanes = edge.lanes || 2;
      const laneWidth = edge.lane_width || 3.5;
      const hw = (lanes * laneWidth) / 2;

      // ── 中心线 spine：每个样点的位置 + XZ 平面法线 ──
      const spine = [];
      for (let i = 0; i < points.length; i += 3) {
        const px = points[i], py = points[i + 1], pz = points[i + 2];
        let tx = 1, tz = 0;
        if (i + 6 < points.length) { tx = points[i + 3] - px; tz = points[i + 5] - pz; }
        else if (i >= 3) { tx = px - points[i - 3]; tz = pz - points[i - 2]; }
        const l = Math.sqrt(tx * tx + tz * tz) || 1;
        tx /= l; tz /= l;
        spine.push({ px, py, pz, nx: -tz, nz: tx });
      }
      if (spine.length < 2) continue;

      // ── 路面 ──
      const road = ribbonGeo(spine, hw, Y_ROAD);
      if (road) roadGeos.push(road);

      // ── 边线（实线白）：路缘内缩 ──
      const eL = solidLine(spine, hw - EDGE_INSET);
      const eR = solidLine(spine, -(hw - EDGE_INSET));
      if (eL) whiteGeos.push(eL);
      if (eR) whiteGeos.push(eR);

      // ── 车道分隔 ──
      // 内部分隔线 k=1..lanes-1，偏移 = -hw + k*laneWidth。
      // 偶数车道的正中（k=lanes/2）为对向分界 → 黄实线；其余白虚线。
      for (let k = 1; k < lanes; k++) {
        const d = -hw + k * laneWidth;
        if (lanes % 2 === 0 && k === lanes / 2) {
          const g = solidLine(spine, d);
          if (g) yellowGeos.push(g);
        } else {
          for (const g of dashedLine(spine, d)) whiteGeos.push(g);
        }
      }
    }

    // ── 合并 + 上材质 ──
    if (roadGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: ASPHALT_COLOR, roughness: 0.92, metalness: 0.0,
        side: THREE.DoubleSide,   // 兜底：弯道端点切线估计可能瞬时翻转缠绕
      });
      const mesh = new THREE.Mesh(mergeGeometries(roadGeos), mat);
      mesh.receiveShadow = true;
      roadGroup.add(mesh);
    }
    if (whiteGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: LINE_WHITE, roughness: 0.6, metalness: 0.0, side: THREE.DoubleSide,
        polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
      });
      roadGroup.add(new THREE.Mesh(mergeGeometries(whiteGeos), mat));
    }
    if (yellowGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: LINE_YELLOW, roughness: 0.6, metalness: 0.0, side: THREE.DoubleSide,
        polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
      });
      roadGroup.add(new THREE.Mesh(mergeGeometries(yellowGeos), mat));
    }

    built = true;
  }

  function getRoadGroup() { return roadGroup; }
  function isBuilt() { return built; }

  return { build, getRoadGroup, isBuilt };
}
