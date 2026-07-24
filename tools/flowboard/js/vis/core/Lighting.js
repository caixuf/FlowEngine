/**
 * Lighting.js — 电影感光照系统
 *
 * r160 迁移：
 *   - 构造器不再传 intensity 参数（r155 弃用三参数构造）
 *   - 砍掉纯白 AmbientLight（HDRI 接管环境光）
 *   - 半球光 0.7 → 0.3（只补底色，不洗白）
 *   - 太阳强度 1.2 → 3.0（physically correct，r155+ 单位变化）
 *   - 太阳色暖化（0xfff5e0 → 0xffe4b0）
 *   - 阴影 2048 → 4096（超锐）
 */

export function createLighting(scene) {
  // 环境光：兜底亮度，保证阴影面不发灰。离线环境 PMREM fromScene 偏暗，
  // 没有 HDRI 时车身底面/侧面会发黑。补底光后任何角度都有基础可见度。
  const ambient = new THREE.AmbientLight(0x8899bb);
  ambient.intensity = 0.35;
  scene.add(ambient);

  // 半球光：天空蓝 → 地面绿，低强度补底色。
  // HDRI 接管环境光后，半球光只做"地面反弹"的色调暗示。
  const hemi = new THREE.HemisphereLight(0xb0d0ff, 0x6b7a55);
  hemi.intensity = 0.3;
  scene.add(hemi);

  // 方向光：太阳光，暖色斜射，强主光 + 深阴影的电影感。
  // r155+ physically correct：DirectionalLight 强度 ≈ irradiance，
  // 3.0 ≈ 明亮正午阳光（旧 1.2 在新单位下偏暗）。
  const sun = new THREE.DirectionalLight(0xffe4b0);
  sun.intensity = 3.0;
  sun.position.set(50, 80, 30);
  sun.castShadow = true;
  /* 阴影 4096：演示画质优先，车边缘锯齿几乎不可见。
   * ±90 范围在 4096 贴图下 = 22.8px/m，极锐。 */
  sun.shadow.mapSize.set(4096, 4096);
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 220;
  sun.shadow.camera.left = -90;
  sun.shadow.camera.right = 90;
  sun.shadow.camera.top = 90;
  sun.shadow.camera.bottom = -90;
  sun.shadow.bias = -0.0005;
  sun.shadow.normalBias = 0.02;
  scene.add(sun);
  scene.add(sun.target);

  return { hemi, sun };
}

/** 让太阳阴影相机跟随 ego，使小 frustum 始终罩住主车。
 *  每帧调用（开销极小）。ego 缺省时不动。 */
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
  lights.hemi.intensity = isNight ? 0.1 : 0.3;
  lights.sun.intensity = isNight ? 0.3 : 3.0;
  lights.sun.color.setHex(isNight ? 0x8090b0 : 0xffe4b0);
}