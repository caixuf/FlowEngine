// ═══════════════════════════════════════════════════════════════
// RoadNetworkModel.js — 路网曲线/邻接表/长度缓存
// ═══════════════════════════════════════════════════════════════
// 职责：管理多段道路网络的曲线几何数据，提供查询接口。
// 设计：数据 + 查询方法，不引用 THREE（曲线对象由 builder 注入）。
//       这是路网几何的"单一数据源"，builder 写入、updater/trajectory 读取。
// ═══════════════════════════════════════════════════════════════

/** @type {THREE.CatmullRomCurve3[]} 每条 edge 的曲线 */
let _curves = [];
/** @type {number[]} 每条 edge 的长度（与 _curves 一一对应） */
let _lens = [];
/** @type {number[]} edge 邻接表：_next[i] = 下一连接 edge 索引，无则 -1 */
let _next = [];

/** 重置路网数据（重建前调用） */
export function resetRoadNetwork() {
  _curves = [];
  _lens = [];
  _next = [];
}

/**
 * 追加一条 edge 的曲线数据。
 * @param {THREE.CatmullRomCurve3} curve
 * @param {number} length
 */
export function addRoadCurve(curve, length) {
  _curves.push(curve);
  _lens.push(length);
}

/**
 * 构建邻接表：按首尾端点重合（距离 < 1m）判定 edge 连接关系。
 * 必须在所有 edge 追加完后调用。
 * @param {Array} edgeEnds - [{start:{x,z}, end:{x,z}}, ...]
 */
export function buildAdjacency(edgeEnds) {
  _next = new Array(_curves.length).fill(-1);
  for (var i = 0; i < edgeEnds.length; i++) {
    for (var j = 0; j < edgeEnds.length; j++) {
      if (i === j) continue;
      var ei = edgeEnds[i].end, sj = edgeEnds[j].start;
      var dx = ei.x - sj.x, dz = ei.z - sj.z;
      if (dx * dx + dz * dz < 1.0) {
        _next[i] = j;
        break;
      }
    }
  }
}

/** @returns {THREE.CatmullRomCurve3[]} */
export function getCurves() { return _curves; }

/** @returns {number[]} */
export function getLens() { return _lens; }

/** @returns {number[]} */
export function getNext() { return _next; }

/** @returns {boolean} */
export function hasCurves() { return _curves.length > 0; }

/**
 * 获取指定 edge 的长度（兜底用 curve.getLength()）。
 * @param {number} idx
 */
export function getCurveLength(idx) {
  if (idx < 0 || idx >= _curves.length) return 0;
  return _lens[idx] || _curves[idx].getLength();
}

/**
 * 查找 (x,z) 在所有曲线中最近点的 edge 索引与 t 参数。
 * @returns {{idx: number, t: number}} idx=-1 表示未找到
 */
export function findNearestCurve(x, z) {
  if (!hasCurves()) return { idx: -1, t: 0 };
  var bestIdx = -1, bestT = 0, bestD2 = Infinity;
  for (var ci = 0; ci < _curves.length; ci++) {
    var pts = _curves[ci].getSpacedPoints(30);
    for (var pi = 0; pi < pts.length; pi++) {
      var dx = pts[pi].x - x, dz = pts[pi].z - z;
      var d2 = dx * dx + dz * dz;
      if (d2 < bestD2) { bestD2 = d2; bestIdx = ci; bestT = pi / (pts.length - 1); }
    }
  }
  return { idx: bestIdx, t: bestT };
}

/**
 * 在指定 edge 上查找 (x,z) 最近点的 t 参数。
 * @param {number} idx - edge 索引
 */
export function findNearestTOnCurve(idx, x, z) {
  if (idx < 0 || idx >= _curves.length) return 0;
  var pts = _curves[idx].getSpacedPoints(30);
  var bestT = 0, bestD2 = Infinity;
  for (var pi = 0; pi < pts.length; pi++) {
    var dx = pts[pi].x - x, dz = pts[pi].z - z;
    var d2 = dx * dx + dz * dz;
    if (d2 < bestD2) { bestD2 = d2; bestT = pi / (pts.length - 1); }
  }
  return bestT;
}
