// ═══════════════════════════════════════════════════════════════
// GeometryMerge.js — 几何体合并工具（无 BufferGeometryUtils 依赖）
// ═══════════════════════════════════════════════════════════════
// 职责：把多个 BufferGeometry 合并为单个 indexed BufferGeometry，
//       或用 transform matrix 复制并变换模板几何体。
// 设计：纯函数，不持有状态，不依赖 scene3d 内部变量。
// ═══════════════════════════════════════════════════════════════

const THREE = window.THREE;

/**
 * 合并多个 geometry 为单个 indexed BufferGeometry。
 * 顶点/法线/索引全部拼接，draw call 从 O(N) 降到 O(1)。
 */
export function mergeGeometries(geos) {
  var totalVerts = 0, totalIdx = 0;
  for (var i = 0; i < geos.length; i++) {
    var g = geos[i];
    totalVerts += g.attributes.position.count;
    totalIdx += (g.index ? g.index.count : g.attributes.position.count);
  }
  var pos = new Float32Array(totalVerts * 3);
  var norm = new Float32Array(totalVerts * 3);
  var idx = new (totalIdx > 65535 ? Uint32Array : Uint16Array)(totalIdx);
  var vOff = 0, iOff = 0;
  for (var j = 0; j < geos.length; j++) {
    var gg = geos[j];
    var pc = gg.attributes.position.count;
    var pa = gg.attributes.position.array;
    var na = gg.attributes.normal ? gg.attributes.normal.array : null;
    for (var k = 0; k < pc; k++) {
      pos[(vOff + k) * 3]     = pa[k * 3];
      pos[(vOff + k) * 3 + 1] = pa[k * 3 + 1];
      pos[(vOff + k) * 3 + 2] = pa[k * 3 + 2];
      if (na) {
        norm[(vOff + k) * 3]     = na[k * 3];
        norm[(vOff + k) * 3 + 1] = na[k * 3 + 1];
        norm[(vOff + k) * 3 + 2] = na[k * 3 + 2];
      }
    }
    if (gg.index) {
      var ia = gg.index.array;
      for (var m = 0; m < ia.length; m++) {
        idx[iOff + m] = ia[m] + vOff;
      }
      iOff += ia.length;
    } else {
      for (var m2 = 0; m2 < pc; m2++) idx[iOff + m2] = vOff + m2;
      iOff += pc;
    }
    vOff += pc;
  }
  var merged = new THREE.BufferGeometry();
  merged.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  merged.setAttribute('normal', new THREE.BufferAttribute(norm, 3));
  if (totalIdx > 0) merged.setIndex(new THREE.BufferAttribute(idx, 1));
  return merged;
}

/**
 * 用 transform matrix 复制一份 geometry 并变换顶点，返回新 geometry。
 * 用于把模板几何体摆到世界不同位置后合并。
 */
export function transformGeometry(geo, matrix) {
  var g = geo.clone();
  var pos = g.attributes.position.array;
  var norm = g.attributes.normal ? g.attributes.normal.array : null;
  var e = matrix.elements;
  for (var i = 0; i < g.attributes.position.count; i++) {
    var x = pos[i * 3], y = pos[i * 3 + 1], z = pos[i * 3 + 2];
    var tx = e[0] * x + e[4] * y + e[8] * z + e[12];
    var ty = e[1] * x + e[5] * y + e[9] * z + e[13];
    var tz = e[2] * x + e[6] * y + e[10] * z + e[14];
    pos[i * 3] = tx; pos[i * 3 + 1] = ty; pos[i * 3 + 2] = tz;
    if (norm) {
      var nx = norm[i * 3], ny = norm[i * 3 + 1], nz = norm[i * 3 + 2];
      var tnx = e[0] * nx + e[4] * ny + e[8] * nz;
      var tny = e[1] * nx + e[5] * ny + e[9] * nz;
      var tnz = e[2] * nx + e[6] * ny + e[10] * nz;
      norm[i * 3] = tnx; norm[i * 3 + 1] = tny; norm[i * 3 + 2] = tnz;
    }
  }
  return g;
}
