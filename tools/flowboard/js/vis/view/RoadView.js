/**
 * RoadView.js — 路面 ribbon + 车道线 + 路肩
 *
 * P1 画质升级：
 *   - 沥青 PBR：程序化 CanvasTexture 生成 albedo + normal map（颗粒感）
 *   - 车道线：轻微 emissive 反光 + 虚线 bug 修复
 *   - 路肩/路缘石：道路两侧各一条浅色窄带
 *
 * 车道线手法移植自 docs/scene.html（materials + polygonOffset 防 z-fight），
 * 但几何由 road_network 数据驱动（沿采样中心线按横向偏移铺设）。
 * road_network 变化时重建，ego 位姿变化不重建。
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { mergeGeometries } from '../math/GeometryMerge.js';
import { LANE_WIDTH, DEFAULT_LANES } from '../core/Constants.js';

const ASPHALT_COLOR = 0x2a2a2a;
const SHOULDER_COLOR = 0x5a5a55;
const LINE_WHITE = 0xffffff;
const LINE_YELLOW = 0xffd700;

const LINE_W = 0.15;      // 车道线宽度 (m)
const EDGE_INSET = 0.25;  // 边线相对路缘内缩 (m)
const Y_ROAD = 0.10;      // 路面高度
const Y_MARK = 0.13;      // 车道线高度（略高于路面防 z-fight）
const Y_SHOULDER = 0.08;  // 路肩高度（略低于路面）
const SHOULDER_W = 0.6;   // 路肩宽度 (m)
const DASH = 3.0;         // 虚线段长 (m)
const GAP = 6.0;          // 虚线间隔 (m)

// ═══════════════════════════════════════════════════════════
// 程序化沥青纹理（CanvasTexture，零外部资源）
// ═══════════════════════════════════════════════════════════

let _asphaltTex = null;
let _asphaltNormal = null;

function _buildAsphaltTextures() {
  if (_asphaltTex) return;

  const SIZE = 512;
  const canvas = document.createElement('canvas');
  canvas.width = SIZE; canvas.height = SIZE;
  const ctx = canvas.getContext('2d');

  // 基底：深灰沥青色
  ctx.fillStyle = '#2a2a2a';
  ctx.fillRect(0, 0, SIZE, SIZE);

  // 随机噪声颗粒（模拟沥青骨料）
  const imageData = ctx.getImageData(0, 0, SIZE, SIZE);
  const data = imageData.data;
  for (let i = 0; i < data.length; i += 4) {
    const noise = (Math.random() - 0.5) * 28;
    data[i]     = Math.max(0, Math.min(255, data[i] + noise));
    data[i + 1] = Math.max(0, Math.min(255, data[i + 1] + noise));
    data[i + 2] = Math.max(0, Math.min(255, data[i + 2] + noise));
  }
  ctx.putImageData(imageData, 0, 0);

  // 细纹裂缝（随机短线，模拟沥青路面微裂纹）
  ctx.strokeStyle = 'rgba(20,20,20,0.15)';
  ctx.lineWidth = 0.5;
  for (let i = 0; i < 80; i++) {
    const x = Math.random() * SIZE, y = Math.random() * SIZE;
    const len = 10 + Math.random() * 40;
    const angle = Math.random() * Math.PI;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + Math.cos(angle) * len, y + Math.sin(angle) * len);
    ctx.stroke();
  }

  _asphaltTex = new THREE.CanvasTexture(canvas);
  _asphaltTex.wrapS = THREE.RepeatWrapping;
  _asphaltTex.wrapT = THREE.RepeatWrapping;
  _asphaltTex.repeat.set(8, 8);  // 8m 重复，匹配路面尺度
  _asphaltTex.colorSpace = THREE.SRGBColorSpace;

  // 法线贴图：从灰度高度图生成
  const normalCanvas = document.createElement('canvas');
  normalCanvas.width = SIZE; normalCanvas.height = SIZE;
  const nctx = normalCanvas.getContext('2d');
  nctx.drawImage(canvas, 0, 0);  // 复制噪声图
  const nImgData = nctx.getImageData(0, 0, SIZE, SIZE);
  const nd = nImgData.data;

  // Sobel 算子转法线
  const heightMap = new Float32Array(SIZE * SIZE);
  for (let i = 0; i < SIZE * SIZE; i++) {
    heightMap[i] = nd[i * 4] / 255;  // 灰度值
  }

  const out = new Uint8ClampedArray(SIZE * SIZE * 4);
  for (let y = 1; y < SIZE - 1; y++) {
    for (let x = 1; x < SIZE - 1; x++) {
      const idx = (y * SIZE + x);
      const tl = heightMap[(y - 1) * SIZE + (x - 1)];
      const t  = heightMap[(y - 1) * SIZE + x];
      const tr = heightMap[(y - 1) * SIZE + (x + 1)];
      const l  = heightMap[y * SIZE + (x - 1)];
      const r  = heightMap[y * SIZE + (x + 1)];
      const bl = heightMap[(y + 1) * SIZE + (x - 1)];
      const b  = heightMap[(y + 1) * SIZE + x];
      const br = heightMap[(y + 1) * SIZE + (x + 1)];

      const gx = (tr + 2 * r + br) - (tl + 2 * l + bl);
      const gy = (bl + 2 * b + br) - (tl + 2 * t + tr);
      const strength = 2.5;

      const nx = -gx * strength;
      const ny = -gy * strength;
      const nz = 1.0;
      const len = Math.sqrt(nx * nx + ny * ny + nz * nz);

      const pi = idx * 4;
      out[pi]     = Math.round(((nx / len) * 0.5 + 0.5) * 255);
      out[pi + 1] = Math.round(((ny / len) * 0.5 + 0.5) * 255);
      out[pi + 2] = Math.round(((nz / len) * 0.5 + 0.5) * 255);
      out[pi + 3] = 255;
    }
  }
  const normalImgData = new ImageData(out, SIZE, SIZE);
  nctx.putImageData(normalImgData, 0, 0);

  _asphaltNormal = new THREE.CanvasTexture(normalCanvas);
  _asphaltNormal.wrapS = THREE.RepeatWrapping;
  _asphaltNormal.wrapT = THREE.RepeatWrapping;
  _asphaltNormal.repeat.set(8, 8);
  _asphaltNormal.colorSpace = THREE.LinearSRGBColorSpace;
}

export function createRoadView(scene) {
  let roadGroup = new THREE.Group();
  scene.add(roadGroup);
  let built = false;

  // 确保纹理已生成
  _buildAsphaltTextures();

  // ── 几何辅助 ──

  /** 从中心线样点（含法线）+ 半宽构建朝上的 ribbon 几何体。
   *  centers: [{px,py,pz,nx,nz}]，法线在 XZ 平面。yOff: 抬高量。 */
  function ribbonGeo(centers, halfW, yOff) {
    if (centers.length < 2) return null;
    const positions = [], indices = [], uvs = [];
    for (let k = 0; k < centers.length; k++) {
      const c = centers[k];
      positions.push(c.px + c.nx * halfW, c.py + yOff, c.pz + c.nz * halfW);
      positions.push(c.px - c.nx * halfW, c.py + yOff, c.pz - c.nz * halfW);
      uvs.push(0, k); uvs.push(1, k);
    }
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
    // 修复虚线 bug：从 0 开始按 DASH+GAP 步进，最后一段用 sampleAt 采样而不是直接套 centers
    for (let s = 0; s < total; s += DASH + GAP) {
      const end = Math.min(s + DASH, total);
      if (end - s < 0.1) continue;  // 跳过过短的残段
      // 每段虚线用起点+终点两个样点构建 ribbon（保持直线段）
      const g = ribbonGeo([sampleAt(s), sampleAt(end)], LINE_W / 2, Y_MARK);
      if (g) geos.push(g);
    }
    return geos;
  }

  /** 从 edge nodes 构建路面 + 车道线 + 路肩 */
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
    const shoulderGeos = [];
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
        nodes = nodes.map(n => [n.x || 0, n.y || 0, n.z || 0]);
      }

      const points = sampleEdgeNodes(nodes, 24);
      const lanes = edge.lanes || 2;
      const laneWidth = edge.lane_width || LANE_WIDTH;
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

      // ── 路面（沥青 PBR）──
      const road = ribbonGeo(spine, hw, Y_ROAD);
      if (road) roadGeos.push(road);

      // ── 路肩（浅灰窄带，略低于路面）──
      const shoulderFull = hw + SHOULDER_W;
      // 左路肩：从 hw 到 shoulderFull 的窄带
      const shoulderL = ribbonGeo(spine.map(c => ({
        px: c.px + c.nx * (hw + SHOULDER_W * 0.5),
        py: c.py, pz: c.pz + c.nz * (hw + SHOULDER_W * 0.5),
        nx: c.nx, nz: c.nz,
      })), SHOULDER_W * 0.5, Y_SHOULDER);
      if (shoulderL) shoulderGeos.push(shoulderL);
      // 右路肩
      const shoulderR = ribbonGeo(spine.map(c => ({
        px: c.px - c.nx * (hw + SHOULDER_W * 0.5),
        py: c.py, pz: c.pz - c.nz * (hw + SHOULDER_W * 0.5),
        nx: c.nx, nz: c.nz,
      })), SHOULDER_W * 0.5, Y_SHOULDER);
      if (shoulderR) shoulderGeos.push(shoulderR);

      // ── 边线（白虚线）：路缘内缩 ──
      // B4 fix: 自车道边界改白虚线，不再用黄实线
      for (const g of dashedLine(spine, hw - EDGE_INSET)) whiteGeos.push(g);
      for (const g of dashedLine(spine, -(hw - EDGE_INSET))) whiteGeos.push(g);

      // ── 车道分隔 ──
      // B4 fix: 黄实线只留真对向分界（4+车道），2车道全程白虚线
      for (let k = 1; k < lanes; k++) {
        const d = -hw + k * laneWidth;
        if (lanes >= 4 && k === Math.floor(lanes / 2)) {
          // 4+ 车道：中央对向分界 → 黄实线
          const g = solidLine(spine, d);
          if (g) yellowGeos.push(g);
        } else {
          // 其他车道分隔 → 白虚线
          for (const g of dashedLine(spine, d)) whiteGeos.push(g);
        }
      }
    }

    // ── 合并 + 上材质 ──

    // 路面：沥青 PBR（带程序化纹理 + 法线贴图）
    if (roadGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: ASPHALT_COLOR,
        map: _asphaltTex,
        normalMap: _asphaltNormal,
        normalScale: new THREE.Vector2(0.4, 0.4),
        roughness: 0.88,
        metalness: 0.02,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(mergeGeometries(roadGeos), mat);
      mesh.receiveShadow = true;
      roadGroup.add(mesh);
    }

    // 路肩：浅灰，略粗糙
    if (shoulderGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: SHOULDER_COLOR,
        roughness: 0.85,
        metalness: 0.05,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(mergeGeometries(shoulderGeos), mat);
      mesh.receiveShadow = true;
      roadGroup.add(mesh);
    }

    // 白色车道线：轻微 emissive 反光
    if (whiteGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: LINE_WHITE,
        emissive: LINE_WHITE,
        emissiveIntensity: 0.15,
        roughness: 0.5,
        metalness: 0.05,
        side: THREE.DoubleSide,
        polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
      });
      roadGroup.add(new THREE.Mesh(mergeGeometries(whiteGeos), mat));
    }

    // 黄色中心线：轻微 emissive 反光
    if (yellowGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: LINE_YELLOW,
        emissive: LINE_YELLOW,
        emissiveIntensity: 0.12,
        roughness: 0.5,
        metalness: 0.05,
        side: THREE.DoubleSide,
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