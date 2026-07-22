/**
 * Curve.js — Catmull-Rom 曲线采样
 * 从控制点采样等距点，用于路面 ribbon 和车道线生成。
 */

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
export function sampleEdgeNodes(nodes, samplesPerEdge = 16) {
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
