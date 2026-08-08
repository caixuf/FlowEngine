/**
 * Curve.js — Catmull-Rom 曲线采样
 * 从控制点采样等距点，用于路面 ribbon 和车道线生成。
 */

const DEFAULT_SAMPLE_SPACING_M = 3.0;
const MIN_EDGE_SAMPLES = 16;
const MAX_EDGE_SAMPLES = 2048;

/** 按控制点折线弧长估算 edge 采样数，保证长弯道不会被少量直线弦切穿。 */
export function edgeSampleCount(nodes, spacingM = DEFAULT_SAMPLE_SPACING_M) {
  if (!nodes || nodes.length < 2) return 0;
  let length = 0;
  for (let i = 1; i < nodes.length; i++) {
    const a = nodes[i - 1], b = nodes[i];
    length += Math.hypot(
      (b[0] || 0) - (a[0] || 0),
      (b[1] || 0) - (a[1] || 0),
      (b[2] || 0) - (a[2] || 0)
    );
  }
  const spacing = Math.max(0.5, spacingM);
  return Math.max(MIN_EDGE_SAMPLES,
    Math.min(MAX_EDGE_SAMPLES, Math.ceil(length / spacing) + 1));
}

/** 从控制点数组采样 n 个点，返回 [x,y,z,...] 扁平数组 */
export function sampleCatmullRom(points, n) {
  if (!points || points.length < 2) return [];
  const curve = new THREE.CatmullRomCurve3(
    points.map(p => new THREE.Vector3(p[0], p[2] || 0, -p[1]))
  );
  const out = [];
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    const v = curve.getPoint(t);
    out.push(v.x, v.y, v.z);
  }
  return out;
}

/** 从 edge 的 nodes 数组采样路面点 */
export function sampleEdgeNodes(nodes, samplesPerEdge = edgeSampleCount(nodes)) {
  if (!nodes || nodes.length < 2) return [];
  if (nodes.length === 2) {
    // 直道：线性插值
    const out = [];
    for (let i = 0; i < samplesPerEdge; i++) {
      const t = i / (samplesPerEdge - 1);
      const a = nodes[0], b = nodes[1];
      out.push(
        a[0] + (b[0] - a[0]) * t,
        (a[2] || 0) + ((b[2] || 0) - (a[2] || 0)) * t,
        -(a[1] + (b[1] - a[1]) * t)
      );
    }
    return out;
  }
  // 多点：用 CatmullRom
  return sampleCatmullRom(nodes, samplesPerEdge);
}

/** 沿 edge 按弧长 s (0~length) 采样世界坐标，返回 {x,y,z} (Three.js 坐标系) */
export function getEdgePointAtS(nodes, s, edgeLength) {
  if (!nodes || nodes.length < 2 || edgeLength <= 0) return null;
  const t = Math.max(0, Math.min(1, s / edgeLength));
  if (nodes.length === 2) {
    const a = nodes[0], b = nodes[1];
    return {
      x: a[0] + (b[0] - a[0]) * t,
      y: (a[2] || 0) + ((b[2] || 0) - (a[2] || 0)) * t,
      z: -(a[1] + (b[1] - a[1]) * t)
    };
  }
  const curve = new THREE.CatmullRomCurve3(
    nodes.map(p => new THREE.Vector3(p[0], p[2] || 0, -p[1]))
  );
  const v = curve.getPointAt(t);
  return { x: v.x, y: v.y, z: v.z };
}
