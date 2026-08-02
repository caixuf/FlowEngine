/**
 * RoadView.js — 路面 ribbon + 车道线 + 路肩 + 弯道/匝道支持
 *
 * P1 画质升级：
 *   - 沥青 PBR：程序化 CanvasTexture 生成 albedo + normal map（颗粒感）
 *   - 车道线：轻微 emissive 反光 + 虚线 bug 修复
 *   - 路肩/路缘石：道路两侧各一条浅色窄带
 *
 * P2 弯道/匝道（2026-08）：
 *   - 边缘类型感知：road/ramp_curve/viaduct_highway/urban
 *   - 弯道：CatmullRomCurve3 多点采样，平滑法线
 *   - 匝道：单行道、窄路面、无黄色中心线、汇入区虚线标识
 *   - 汇入区：匝道汇入主路时渲染渐变虚线
 *
 * 车道线手法移植自 docs/scene.html（materials + polygonOffset 防 z-fight），
 * 但几何由 road_network 数据驱动（沿采样中心线按横向偏移铺设）。
 * road_network 变化时重建，ego 位姿变化不重建。
 */

import { sampleEdgeNodes } from '../math/Curve.js';
import { mergeGeometries } from '../math/GeometryMerge.js';
import { LANE_WIDTH, DEFAULT_LANES, EDGE_TYPE } from '../core/Constants.js';
import { tangentToNormal, offsetAlongNormal, forwardENU } from '../math/Coord.js';

const ASPHALT_COLOR = 0x2a2a2a;
const SHOULDER_COLOR = 0x5a5a55;
const LINE_WHITE = 0xcccccc;
const LINE_YELLOW = 0xffd700;
const RAMP_COLOR = 0x353535;    // 匝道路面比主路略浅

const LINE_W = 0.15;      // 车道线宽度 (m)
const EDGE_INSET = 0.25;  // 边线相对路缘内缩 (m)
const Y_ROAD = 0.10;      // 路面高度
const Y_MARK = 0.13;      // 车道线高度（略高于路面防 z-fight）
const Y_SHOULDER = 0.08;  // 路肩高度（略低于路面）
const SHOULDER_W = 0.6;   // 路肩宽度 (m)
const DASH = 3.0;         // 虚线段长 (m)
const GAP = 6.0;          // 虚线间隔 (m)

/* 弯道/匝道采样密度：主路多点曲线用 32，匝道用 24，直道 2 点用 24 */
const SAMPLES_CURVE = 32;
const SAMPLES_RAMP = 24;
const SAMPLES_STRAIGHT = 24;

/* 匝道渲染参数 */
const RAMP_LANE_W = 3.0;   // 匝道车道宽度略窄 (m)
const RAMP_SHOULDER_W = 0.4; // 匝道路肩略窄

// ═══════════════════════════════════════════════════════════
// 程序化沥青纹理（CanvasTexture，零外部资源）
// ═══════════════════════════════════════════════════════════

let _asphaltTex = null;
let _asphaltNormal = null;

function _buildAsphaltTextures() {
  if (_asphaltTex) return;
  // 在 Node.js 无头测试环境中，document 不可用，跳过纹理生成
  if (typeof document === 'undefined') return;

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
    return spine.map(c => {
      const [opx, , opz] = offsetAlongNormal(c.px, c.pz, c.nx, c.nz, d);
      return { px: opx, py: c.py, pz: opz, nx: c.nx, nz: c.nz };
    });
  }

  /** 构建 spine 的累计弧长数组 */
  function buildCumulative(spine) {
    const cum = [0];
    for (let i = 1; i < spine.length; i++) {
      cum.push(cum[i - 1] + Math.hypot(spine[i].px - spine[i - 1].px, spine[i].pz - spine[i - 1].pz));
    }
    return cum;
  }

  /** 沿 spine 按弧长比例插值样点 */
  function sampleSpineAt(spine, cum, s) {
    const total = cum[cum.length - 1];
    if (s <= 0) return spine[0];
    if (s >= total) return spine[spine.length - 1];
    let i = 1; while (i < cum.length && cum[i] < s) i++;
    const t = (s - cum[i - 1]) / ((cum[i] - cum[i - 1]) || 1);
    const a = spine[i - 1], b = spine[i];
    return {
      px: a.px + (b.px - a.px) * t, py: a.py + (b.py - a.py) * t,
      pz: a.pz + (b.pz - a.pz) * t, nx: a.nx + (b.nx - a.nx) * t,
      nz: a.nz + (b.nz - a.nz) * t,
    };
  }

  /** 实线：沿偏移中心线铺一条连续窄 ribbon */
  function solidLine(spine, d) {
    return ribbonGeo(offsetSpine(spine, d), LINE_W / 2, Y_MARK);
  }

  /** 虚线：沿偏移中心线按弧长 march，每 (DASH+GAP) 铺一段 */
  function dashedLine(spine, d) {
    const centers = offsetSpine(spine, d);
    const cum = buildCumulative(centers);
    const total = cum[cum.length - 1];
    const geos = [];
    for (let s = 0; s < total; s += DASH + GAP) {
      const end = Math.min(s + DASH, total);
      if (end - s < 0.1) continue;
      const g = ribbonGeo([sampleSpineAt(centers, cum, s), sampleSpineAt(centers, cum, end)], LINE_W / 2, Y_MARK);
      if (g) geos.push(g);
    }
    return geos;
  }

  /** 在指定弧长范围 [rangeStart, rangeEnd) 内生成虚线 */
  function dashedLineInRange(spine, d, rangeStart, rangeEnd) {
    const centers = offsetSpine(spine, d);
    const cum = buildCumulative(centers);
    const total = cum[cum.length - 1];
    const rs = Math.max(0, rangeStart);
    const re = Math.min(total, rangeEnd);
    if (rs >= re) return [];

    const geos = [];
    const pattern = DASH + GAP;
    let s = Math.floor(rs / pattern) * pattern;
    for (; s < re; s += pattern) {
      const end = Math.min(s + DASH, re);
      if (end - s < 0.1 || end <= rs) continue;
      const g = ribbonGeo([sampleSpineAt(centers, cum, Math.max(s, rs)), sampleSpineAt(centers, cum, end)], LINE_W / 2, Y_MARK);
      if (g) geos.push(g);
    }
    return geos;
  }

  /** 从 edge nodes 构建 spine（中心线样点 + XZ 法线） */
  function buildSpine(points) {
    const spine = [];
    for (let i = 0; i < points.length; i += 3) {
      const px = points[i], py = points[i + 1], pz = points[i + 2];
      let tx = 1, tz = 0;
      if (i + 6 < points.length) { tx = points[i + 3] - px; tz = points[i + 5] - pz; }
      else if (i >= 3) { tx = px - points[i - 3]; tz = pz - points[i - 2]; }
      const [nx, nz] = tangentToNormal(tx, tz);
      spine.push({ px, py, pz, nx, nz });
    }
    return spine;
  }

  /** 确定 edge 的采样密度 */
  function edgeSampleCount(nodes, edgeType) {
    if (edgeType === EDGE_TYPE.RAMP_CURVE) return SAMPLES_RAMP;
    if (nodes.length > 2) return SAMPLES_CURVE;  // 弯道
    return SAMPLES_STRAIGHT;
  }

  // ── 匝道渲染 ──

  /** 渲染匝道：窄路面、无黄色中心线、白虚线边线 */
  function buildRamp(edge, nodes) {
    const points = sampleEdgeNodes(nodes, SAMPLES_RAMP);
    const spine = buildSpine(points);
    if (spine.length < 2) return { roadGeos: [], whiteGeos: [] };

    const laneWidth = edge.lane_width || RAMP_LANE_W;
    const rampLanes = Math.max(1, edge.lanes || 1);
    const hw = (rampLanes * laneWidth) / 2;

    // 匝道路面（略浅色）
    const road = ribbonGeo(spine, hw, Y_ROAD);

    // 路肩
    const shoulderW = RAMP_SHOULDER_W;
    const shoulderL = ribbonGeo(spine.map(c => ({
      px: c.px + c.nx * (hw + shoulderW * 0.5), py: c.py, pz: c.pz + c.nz * (hw + shoulderW * 0.5),
      nx: c.nx, nz: c.nz,
    })), shoulderW * 0.5, Y_SHOULDER);
    const shoulderR = ribbonGeo(spine.map(c => ({
      px: c.px - c.nx * (hw + shoulderW * 0.5), py: c.py, pz: c.pz - c.nz * (hw + shoulderW * 0.5),
      nx: c.nx, nz: c.nz,
    })), shoulderW * 0.5, Y_SHOULDER);

    const whiteGeos = [];
    // 匝道两侧路缘边线（白实线，与静态 invariant「外沿=实线」一致）
    const rampEdgeL = solidLine(spine, hw - EDGE_INSET);
    if (rampEdgeL) whiteGeos.push(rampEdgeL);
    const rampEdgeR = solidLine(spine, -(hw - EDGE_INSET));
    if (rampEdgeR) whiteGeos.push(rampEdgeR);

    // 多车道匝道内部车道分隔（白虚线）
    if (rampLanes > 1) {
      for (let k = 1; k < rampLanes; k++) {
        const d = -hw + k * laneWidth;
        for (const g of dashedLine(spine, d)) whiteGeos.push(g);
      }
    }

    const roadGeos = [road];
    if (shoulderL) roadGeos.push(shoulderL);
    if (shoulderR) roadGeos.push(shoulderR);
    return { roadGeos, whiteGeos, spine, cum: buildCumulative(spine) };
  }

  /** 在匝道汇入主路的位置渲染汇入区标识（渐变虚线标记） */
  function buildMergeZone(rampEdge, rampSpine, rampCum, mainSpine, mainCum) {
    if (!rampSpine || rampSpine.length < 2 || !mainSpine || mainSpine.length < 2) return [];

    // 匝道末端（汇入点）
    const rampEnd = rampSpine[rampSpine.length - 1];
    const rampTotal = rampCum[rampCum.length - 1];

    // 在主路上找距离匝道末端最近的点
    let bestDist = 1e9, bestIdx = 0;
    for (let i = 0; i < mainSpine.length; i++) {
      const dx = mainSpine[i].px - rampEnd.px;
      const dz = mainSpine[i].pz - rampEnd.pz;
      const d = dx * dx + dz * dz;
      if (d < bestDist) { bestDist = d; bestIdx = i; }
    }

    const mergeDist = Math.sqrt(bestDist);
    // 汇入区距离 > 30m 不渲染（可能是远距离另一条路，不是汇入）
    if (mergeDist > 30) return [];

    // 在主路上标记汇入区：从汇入点往前 30m 的渐变虚线
    const mainCumI = mainCum[bestIdx];
    const mergeStart = Math.max(0, mainCumI - 30);
    const mergeEnd = mainCumI + 10;

    // 匝道末端到主路汇入点的渐变虚线束
    const geos = [];
    // 汇入区在主路上的横向位置：匝道 1 车道宽，放在主路右车道侧
    // 匝道从右车道汇入，汇入区标记在主路 rightmost lane 位置
    // 此处用渐变虚线表示"请观察主路车辆"
    const laneWidth = 3.5;
    // 主路右车道中心线位置（假设单向 2 车道，主路右车道中心在 -hw + 0.75*laneWidth）
    // 简化为在主路右路肩附近画一段汇入标识
    const mergeZone = dashedLineInRange(mainSpine, -(laneWidth * 1.5), mergeStart, mergeEnd);
    for (const g of mergeZone) geos.push(g);
    return geos;
  }

  /** 从 edge nodes 构建主路路面 + 车道线 + 路肩 */
  function buildStandardRoad(edge, nodes) {
    const sampleCount = edgeSampleCount(nodes, edge.type);
    const points = sampleEdgeNodes(nodes, sampleCount);
    const spine = buildSpine(points);
    if (spine.length < 2) return { roadGeos: [], shoulderGeos: [], whiteGeos: [], yellowGeos: [], spine, cum: [] };

    const lanes = edge.lanes || DEFAULT_LANES;
    const laneWidth = edge.lane_width || LANE_WIDTH;
    const hw = (lanes * laneWidth) / 2;

    // 路面
    const road = ribbonGeo(spine, hw, Y_ROAD);

    // 路肩
    const shoulderL = ribbonGeo(spine.map(c => ({
      px: c.px + c.nx * (hw + SHOULDER_W * 0.5), py: c.py, pz: c.pz + c.nz * (hw + SHOULDER_W * 0.5),
      nx: c.nx, nz: c.nz,
    })), SHOULDER_W * 0.5, Y_SHOULDER);
    const shoulderR = ribbonGeo(spine.map(c => ({
      px: c.px - c.nx * (hw + SHOULDER_W * 0.5), py: c.py, pz: c.pz - c.nz * (hw + SHOULDER_W * 0.5),
      nx: c.nx, nz: c.nz,
    })), SHOULDER_W * 0.5, Y_SHOULDER);

    const whiteGeos = [];
    const yellowGeos = [];

    // 路缘边线（白实线）：路缘内缩。外沿=实线（真路约定 + 静态 invariant
    // 「外沿=实线」一致）；虚线只用于同向车道分隔。旧版把路缘画成虚线，
    // 双向四车道上视觉上像"整条路都是虚线"。
    const edgeL = solidLine(spine, hw - EDGE_INSET);
    if (edgeL) whiteGeos.push(edgeL);
    const edgeR = solidLine(spine, -(hw - EDGE_INSET));
    if (edgeR) whiteGeos.push(edgeR);

    // 车道分隔
    for (let k = 1; k < lanes; k++) {
      const d = -hw + k * laneWidth;
      if (lanes >= 4 && k === Math.floor(lanes / 2)) {
        // 中央对向分界 → 黄实线为主 + 两端黄虚线（掉头区域）
        const DASH_ZONE = 50.0;
        const cum = buildCumulative(spine);
        const totalLen = cum[cum.length - 1];
        if (totalLen > 2 * DASH_ZONE) {
          const solidSpine = [];
          for (let i = 0; i < spine.length; i++) {
            if (cum[i] >= DASH_ZONE && cum[i] <= totalLen - DASH_ZONE) {
              solidSpine.push({ ...spine[i] });
            }
          }
          if (solidSpine.length >= 2) {
            const g = solidLine(solidSpine, d);
            if (g) yellowGeos.push(g);
          }
        }
        for (const g of dashedLineInRange(spine, d, 0, DASH_ZONE)) yellowGeos.push(g);
        for (const g of dashedLineInRange(spine, d, totalLen - DASH_ZONE, totalLen)) yellowGeos.push(g);
      } else {
        for (const g of dashedLine(spine, d)) whiteGeos.push(g);
      }
    }

    const roadGeos = [road];
    if (shoulderL) roadGeos.push(shoulderL);
    if (shoulderR) roadGeos.push(shoulderR);
    return { roadGeos, shoulderGeos: [], whiteGeos, yellowGeos, spine, cum: buildCumulative(spine) };
  }

  /** 从 edge nodes 构建（兼容三种 edge 格式） */
  function parseNodes(edge) {
    let nodes = edge.nodes;
    if (!nodes || nodes.length < 2) {
      const len = edge.length_m || 100;
      const h = edge.heading || 0;
      const sx = edge.start_x || 0, sz = edge.start_z || 0;
      const [fex, fey] = forwardENU(h);
      nodes = [[sx, sz, 0], [sx + fex * len, sz + fey * len, 0]];
    } else if (nodes[0] && typeof nodes[0] === 'object' && !Array.isArray(nodes[0])) {
      nodes = nodes.map(n => [n.x || 0, n.y || 0, n.z || 0]);
    }
    return nodes;
  }

  /** 检测 edge 是否为匝道 */
  function isRampEdge(edge) {
    return edge.type === EDGE_TYPE.RAMP_CURVE
      || edge.oneway === true
      || (edge.name && edge.name.indexOf('ramp') !== -1);
  }

  /** 从 road_network 构建全部路面 + 车道线 + 路肩 */
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
    const whiteLineGeos = [];
    const yellowLineGeos = [];
    const rampGeos = [];

    // 收集所有主路 edge 的 spine（用于汇入区检测）
    const mainRoadSpines = [];

    // 第一遍：主路渲染
    for (const edge of roadNetwork.edges) {
      if (isRampEdge(edge)) continue;
      const nodes = parseNodes(edge);
      const result = buildStandardRoad(edge, nodes);
      for (const g of result.roadGeos) roadGeos.push(g);
      for (const g of result.shoulderGeos) shoulderGeos.push(g);
      for (const g of result.whiteGeos) whiteLineGeos.push(g);
      for (const g of result.yellowGeos) yellowLineGeos.push(g);
      if (result.spine.length >= 2) {
        mainRoadSpines.push({ spine: result.spine, cum: result.cum });
      }
    }

    // 第二遍：匝道渲染
    for (const edge of roadNetwork.edges) {
      if (!isRampEdge(edge)) continue;
      const nodes = parseNodes(edge);
      const result = buildRamp(edge, nodes);
      for (const g of result.roadGeos) rampGeos.push(g);
      for (const g of result.whiteGeos) whiteLineGeos.push(g);

      // 汇入区标识：在匝道和主路之间画汇入虚线
      if (result.spine && result.spine.length >= 2) {
        for (const ms of mainRoadSpines) {
          for (const g of buildMergeZone(edge, result.spine, result.cum, ms.spine, ms.cum)) {
            whiteLineGeos.push(g);
          }
        }
      }
    }

    // ── 合并 + 上材质 ──

    // 主路路面：沥青 PBR
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

    // 匝道路面（略浅色）
    if (rampGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: RAMP_COLOR,
        map: _asphaltTex,
        normalMap: _asphaltNormal,
        normalScale: new THREE.Vector2(0.4, 0.4),
        roughness: 0.85,
        metalness: 0.02,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(mergeGeometries(rampGeos), mat);
      mesh.receiveShadow = true;
      roadGroup.add(mesh);
    }

    // 路肩
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

    // 白色车道线
    if (whiteLineGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: LINE_WHITE,
        roughness: 0.6,
        metalness: 0.05,
        side: THREE.DoubleSide,
        polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
      });
      roadGroup.add(new THREE.Mesh(mergeGeometries(whiteLineGeos), mat));
    }

    // 黄色中心线
    if (yellowLineGeos.length) {
      const mat = new THREE.MeshStandardMaterial({
        color: LINE_YELLOW,
        roughness: 0.6,
        metalness: 0.05,
        side: THREE.DoubleSide,
        polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
      });
      roadGroup.add(new THREE.Mesh(mergeGeometries(yellowLineGeos), mat));
    }

    built = true;
  }

  function getRoadGroup() { return roadGroup; }
  function isBuilt() { return built; }

  return { build, getRoadGroup, isBuilt };
}