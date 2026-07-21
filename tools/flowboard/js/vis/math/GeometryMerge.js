/**
 * GeometryMerge.js — BufferGeometry 合并工具
 * 把多个小几何体合并成一个大 BufferGeometry，减少 draw call。
 */

export function mergeGeometries(geometries) {
  const valid = geometries.filter(g => g && g.attributes.position && g.attributes.position.count > 0);
  if (valid.length === 0) return new THREE.BufferGeometry();
  if (valid.length === 1) return valid[0].clone();

  let totalVerts = 0, totalIdx = 0;
  let hasIndex = false;
  for (const g of valid) {
    totalVerts += g.attributes.position.count;
    if (g.index) { hasIndex = true; totalIdx += g.index.count; }
    else { totalIdx += g.attributes.position.count; }
  }

  const positions = new Float32Array(totalVerts * 3);
  const normals = new Float32Array(totalVerts * 3);
  const uvs = new Float32Array(totalVerts * 2);
  const indices = new Uint32Array(totalIdx);

  let vOff = 0, iOff = 0, idxBase = 0;
  for (const g of valid) {
    const p = g.attributes.position.array;
    positions.set(p, vOff * 3);
    if (g.attributes.normal) normals.set(g.attributes.normal.array, vOff * 3);
    if (g.attributes.uv) uvs.set(g.attributes.uv.array, vOff * 2);

    if (g.index) {
      const idx = g.index.array;
      for (let i = 0; i < idx.length; i++) indices[iOff + i] = idx[i] + idxBase;
      iOff += idx.length;
    } else {
      const count = g.attributes.position.count;
      for (let i = 0; i < count; i++) indices[iOff + i] = i;
      iOff += count;
    }
    idxBase += g.attributes.position.count;
    vOff += g.attributes.position.count;
  }

  const merged = new THREE.BufferGeometry();
  merged.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  merged.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
  merged.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
  merged.setIndex(new THREE.BufferAttribute(indices, 1));
  return merged;
}

/** 创建一个平移后的 geometry 副本（用于护栏/路灯等重复元素） */
export function translateGeo(geo, x, y, z) {
  const g = geo.clone();
  g.translate(x, y, z);
  return g;
}
