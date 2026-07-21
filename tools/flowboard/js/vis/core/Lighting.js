/**
 * Lighting.js — 三光照系统：环境光 + 方向光 + 半球光
 * 营造真实白天光照。夜间模式降低 ambient + direction 强度。
 */

export function createLighting(scene) {
  // 半球光：天空蓝 → 地面绿的渐变环境光
  const hemi = new THREE.HemisphereLight(0xb0d0ff, 0x6b7a55, 0.55);
  scene.add(hemi);

  // 环境光：基础填充
  const ambient = new THREE.AmbientLight(0xffffff, 0.35);
  scene.add(ambient);

  // 方向光：太阳光，投射阴影
  const sun = new THREE.DirectionalLight(0xfff5e0, 0.9);
  sun.position.set(300, 400, 200);
  sun.castShadow = true;
  // 阴影相机跟随 ego（见 updateSunShadow），故用小 frustum：又清晰又省 fill。
  // 1024 贴图在 ±90m 范围内已经很锐，比原来 2048 铺 ±500m 省一半以上。
  sun.shadow.mapSize.set(1024, 1024);
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 500;
  sun.shadow.camera.left = -90;
  sun.shadow.camera.right = 90;
  sun.shadow.camera.top = 90;
  sun.shadow.camera.bottom = -90;
  sun.shadow.bias = -0.0005;
  sun.shadow.normalBias = 0.02;   // 配合紧 frustum 抑制 shadow acne
  scene.add(sun);
  scene.add(sun.target);          // target 必须在场景图内，updateSunShadow 改其位置才生效

  return { hemi, ambient, sun };
}

/** 让太阳阴影相机跟随 ego，使小 frustum 始终罩住主车周围。
 *  每帧调用（开销极小）。ego 缺省时不动。 */
export function updateSunShadow(lights, ego) {
  if (!lights || !ego) return;
  const sun = lights.sun;
  const tx = ego.x, tz = ego.y;   // three: x=worldX, z=worldY
  sun.target.position.set(tx, 0, tz);
  // 保持与原光照方向一致的偏移（大致 (300,400,200) 方向），只是跟着 ego 平移
  sun.position.set(tx + 110, 150, tz + 73);
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
