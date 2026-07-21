/**
 * AssetFactory.js — 几何体/材质工厂，实例化缓存
 * 所有 View 共享，避免重复创建相同几何体。
 */

const _geoCache = new Map();
const _matCache = new Map();

export function getBox(w, h, d) {
  const key = `box_${w}_${h}_${d}`;
  if (!_geoCache.has(key)) _geoCache.set(key, new THREE.BoxGeometry(w, h, d));
  return _geoCache.get(key);
}

export function getCylinder(rTop, rBot, h, seg = 8) {
  const key = `cyl_${rTop}_${rBot}_${h}_${seg}`;
  if (!_geoCache.has(key)) _geoCache.set(key, new THREE.CylinderGeometry(rTop, rBot, h, seg));
  return _geoCache.get(key);
}

export function getPlane(w, h) {
  const key = `plane_${w}_${h}`;
  if (!_geoCache.has(key)) _geoCache.set(key, new THREE.PlaneGeometry(w, h));
  return _geoCache.get(key);
}

export function getStdMaterial(color, roughness = 0.7, metalness = 0.0) {
  const key = `mat_${color}_${roughness}_${metalness}`;
  if (!_matCache.has(key)) {
    _matCache.set(key, new THREE.MeshStandardMaterial({ color, roughness, metalness }));
  }
  return _matCache.get(key);
}

/** 发光材质（车灯用，缓存版）。
 *  警告：缓存返回同一实例，多对象共享会导致状态串扰。
 *  需要独立状态的（如车灯每帧切换 emissiveIntensity）用 createEmissiveMaterial。 */
export function getEmissiveMaterial(color, intensity = 1) {
  const key = `emi_${color}_${intensity}`;
  if (!_matCache.has(key)) {
    _matCache.set(key, new THREE.MeshStandardMaterial({
      color: 0x000000, emissive: color, emissiveIntensity: intensity, roughness: 0.4
    }));
  }
  return _matCache.get(key);
}

/** 创建独立发光材质（不缓存，每辆车/每个灯独立实例，可自由改 emissiveIntensity） */
export function createEmissiveMaterial(color, intensity = 1) {
  return new THREE.MeshStandardMaterial({
    color: 0x000000, emissive: color, emissiveIntensity: intensity, roughness: 0.4
  });
}

/** 清空缓存（场景重建时调） */
export function clearCache() {
  _geoCache.clear();
  _matCache.clear();
}
