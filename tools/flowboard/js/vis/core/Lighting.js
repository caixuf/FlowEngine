/**
 * Lighting.js — 三光照系统：环境光 + 方向光 + 半球光
 * 营造真实白天光照。夜间模式降低 ambient + direction 强度。
 */

export function createLighting(scene) {
  // 半球光：天空蓝 → 地面绿的渐变环境光。
  // 强度 0.55 → 0.7：稍亮一点让地面/树有自然色，不至于死黑。
  const hemi = new THREE.HemisphereLight(0xb0d0ff, 0x6b7a55, 0.7);
  scene.add(hemi);

  // 环境光：基础填充
  const ambient = new THREE.AmbientLight(0xffffff, 0.35);
  scene.add(ambient);

  // 方向光：太阳光，斜射而非正顶（参考 scene.html 50,70,30 风格）。
  // 强度 0.9 → 1.2：金属漆高光更亮，路面反光更强。
  const sun = new THREE.DirectionalLight(0xfff5e0, 1.2);
  sun.position.set(50, 80, 30);
  sun.castShadow = true;
  /* 阴影相机跟随 ego（见 updateSunShadow），小 frustum 保证贴图锐利。
   * ±90 范围在 1024 贴图下 = 11.7px/m，足够锐。 */
  sun.shadow.mapSize.set(2048, 2048);  // 1024 → 2048：阴影更锐，车边缘锯齿少
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 220;          // sun 高度 80 + 地表 = 80m，加冗余到 220
  sun.shadow.camera.left = -90;
  sun.shadow.camera.right = 90;
  sun.shadow.camera.top = 90;
  sun.shadow.camera.bottom = -90;
  sun.shadow.bias = -0.0005;
  sun.shadow.normalBias = 0.02;
  scene.add(sun);
  scene.add(sun.target);

  return { hemi, ambient, sun };
}

/** 让太阳阴影相机跟随 ego，使小 frustum 始终罩住主车周围。
 *  每帧调用（开销极小）。ego 缺省时不动。
 *  注意：sun.position 是相对 target 的偏移（方向光无位置概念，但
 *  shadow.camera frustum 沿 target→position 方向投影），所以保持
 *  固定偏移 (50,80,30) 平移。 */
export function updateSunShadow(lights, ego) {
  if (!lights || !ego) return;
  const sun = lights.sun;
  const tx = ego.x, tz = ego.y;   // three: x=worldX, z=worldY
  sun.target.position.set(tx, 0, tz);
  sun.position.set(tx + 50, 80, tz + 30);
  sun.target.updateMatrixWorld();
  sun.shadow.camera.updateProjectionMatrix();
}

/** 切换白天/夜间光照 */
export function setNightMode(lights, isNight) {
  if (!lights) return;
  lights.ambient.intensity = isNight ? 0.15 : 0.35;
  lights.hemi.intensity = isNight ? 0.2 : 0.55;
  lights.sun.intensity = isNight ? 0.2 : 0.9;
  lights.sun.color.setHex(isNight ? 0x8090b0 : 0xfff5e0);
}
