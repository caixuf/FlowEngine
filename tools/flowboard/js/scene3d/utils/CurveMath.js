// ═══════════════════════════════════════════════════════════════
// CurveMath.js — 道路曲线数学工具
// ═══════════════════════════════════════════════════════════════
// 职责：smoothstep 弯道偏移/切线角计算、路网最近切线查询。
// 设计：纯函数（_getRoadTangentAt 接受 curves 数组参数，不读全局）。
// ═══════════════════════════════════════════════════════════════

/**
 * smoothstep 计算弯道横向偏移量。
 * 镜像 C 端 road_center_shift() 逻辑。
 */
export function curveShiftAt(x, sx, len, off) {
  if (len <= 0 || Math.abs(off) < 0.01) return 0;
  if (x <= sx) return 0;
  if (x >= sx + len) return off;
  var t = (x - sx) / len;
  return off * (3 * t * t - 2 * t * t * t);
}

/**
 * smoothstep 导数计算弯道切线方向角（radians）。
 * 镜像 C 端 road_center_heading() 逻辑。
 */
export function curveHeadingAt(x, sx, len, off) {
  if (len <= 0 || Math.abs(off) < 0.01 || x <= sx) return 0;
  if (x >= sx + len) return 0;
  var t = (x - sx) / len;
  // smoothstep derivative: d/dt (3t² - 2t³) = 6t - 6t²
  var dy = off * (6 * t - 6 * t * t) / len;
  return Math.atan(dy);
}

/**
 * 在路网曲线数组中查找 (x,z) 最近点的切线方向。
 * @param {THREE.CatmullRomCurve3[]} curves - 路网曲线数组
 * @returns {{found: boolean, heading: number}}
 */
export function getRoadTangentAt(curves, x, z) {
  if (!curves || !curves.length) return { found: false, heading: 0 };
  var bestT = null, bestD2 = Infinity;
  for (var ci = 0; ci < curves.length; ci++) {
    var curve = curves[ci];
    var pts = curve.getSpacedPoints(20);
    for (var pi = 0; pi < pts.length; pi++) {
      var dx = pts[pi].x - x, dz = pts[pi].z - z;
      var d2 = dx * dx + dz * dz;
      if (d2 < bestD2) { bestD2 = d2; bestT = { curve: curve, t: pi / (pts.length - 1) }; }
    }
  }
  if (!bestT) return { found: false, heading: 0 };
  var tan = bestT.curve.getTangentAt(Math.max(0, Math.min(1, bestT.t)));
  return { found: true, heading: Math.atan2(tan.z, tan.x) };
}
