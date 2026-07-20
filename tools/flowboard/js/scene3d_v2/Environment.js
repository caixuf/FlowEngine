/**
 * Environment.js — v2 地面 + 基础光照
 *
 * 简化策略：
 *   - 地面：单色草地 PlaneGeometry，按路网 bbox 铺满
 *   - 光照：AmbientLight + HemisphereLight + DirectionalLight（带阴影但 frustum 跟随 ego）
 *   - 砍掉：天空盒 shader、PMREMGenerator、城市天际线、雨/水面、夜景模式
 *
 * 背景色用纯色（skyHorizon 浅蓝），不用 SphereGeometry shader。
 */
const THREE = window.THREE;

/**
 * 创建地面 + 光照并挂到 scene。
 */
export function buildEnvironment(scene) {
  // ── 背景色 ──
  scene.background = new THREE.Color(0xd6eafa);  // 浅蓝地平线色
  scene.fog = new THREE.Fog(0xd6eafa, 400, 1500);  // v1 修复后的雾参数

  // ── 地面（草地）──
  // 初始 1km × 1km，路网构建后会按 bbox 重新调整位置/大小
  var groundGeo = new THREE.PlaneGeometry(2000, 2000);
  var groundMat = new THREE.MeshStandardMaterial({
    color: 0x4a7a3a,  // 中绿草地
    roughness: 1.0,
    metalness: 0.0
  });
  var ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;  // 水平铺开
  ground.position.y = -0.02;  // 略低于路面避免 z-fighting
  ground.name = 'ground';
  ground.userData.fixed = false;  // 路网加载后标记为 true 锁定
  scene.add(ground);

  // ── 光照 ──
  var ambient = new THREE.AmbientLight(0xffffff, 0.55);
  scene.add(ambient);

  var hemi = new THREE.HemisphereLight(0xb0d8ff, 0x4a6a3a, 0.5);
  scene.add(hemi);

  var sun = new THREE.DirectionalLight(0xfff4d8, 1.0);
  sun.position.set(30, 40, 15);
  sun.castShadow = true;
  // 阴影 frustum：覆盖 ego 周围 60m × 60m
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 120;
  sun.shadow.camera.left = -40;
  sun.shadow.camera.right = 40;
  sun.shadow.camera.top = 40;
  sun.shadow.camera.bottom = -40;
  sun.shadow.bias = -0.0005;
  scene.add(sun);
  scene.add(sun.target);

  return { ground, ambient, hemi, sun };
}

/**
 * 路网构建完成后，调整地面位置/大小覆盖整个路网 bbox。
 */
export function fitGroundToBBox(ground, bbox) {
  if (!ground || !bbox || bbox.min.x === Infinity) return;
  var cx = (bbox.min.x + bbox.max.x) / 2;
  var cz = (bbox.min.z + bbox.max.z) / 2;
  var w = (bbox.max.x - bbox.min.x) + 400;  // 两侧外扩 200m
  var h = (bbox.max.z - bbox.min.z) + 400;
  // 不 dispose 旧 geometry，直接 scale（性能优先）
  var scaleX = w / 2000;
  var scaleZ = h / 2000;
  ground.scale.set(scaleX, 1, scaleZ);
  ground.position.set(cx, -0.02, cz);
  ground.userData.fixed = true;
}

/**
 * 每帧更新：阴影光源跟随 ego，让阴影 frustum 始终覆盖 ego 周围。
 */
export function updateSunFollowEgo(sun, egoX, egoZ) {
  if (!sun) return;
  sun.position.set(egoX + 30, 40, egoZ + 15);
  sun.target.position.set(egoX, 0, egoZ);
  sun.target.updateMatrixWorld();
}

/**
 * 切换昼夜模式（v2 仅支持 day/night，无 dusk）。
 * v2 简化：夜间只降低光照强度 + 改背景色，不做路灯。
 */
export function applyLightingMode(mode, ambient, hemi, sun, scene) {
  if (mode === 'night') {
    if (ambient) ambient.intensity = 0.2;
    if (hemi) hemi.intensity = 0.15;
    if (sun) sun.intensity = 0.3;
    if (scene) scene.background = new THREE.Color(0x1a2540);
    if (scene) scene.fog = new THREE.Fog(0x1a2540, 200, 800);
  } else {
    // day（默认）
    if (ambient) ambient.intensity = 0.55;
    if (hemi) hemi.intensity = 0.5;
    if (sun) sun.intensity = 1.0;
    if (scene) scene.background = new THREE.Color(0xd6eafa);
    if (scene) scene.fog = new THREE.Fog(0xd6eafa, 400, 1500);
  }
}
