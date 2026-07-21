/**
 * SkyEnv.js — 天空背景 + 雾
 * 纯色天空（不生成 canvas 纹理），fog 远距离淡出。
 */

const DAY_COLOR = 0xb0d0f0;    // 白天浅蓝
const NIGHT_COLOR = 0x1a1a30;  // 夜间深蓝

export function createSkyEnv(scene, isNight = false) {
  const color = isNight ? NIGHT_COLOR : DAY_COLOR;
  scene.background = new THREE.Color(color);
  // fog：远距离淡出，避免远处空地突兀。near=400 让近处完全清晰。
  scene.fog = new THREE.Fog(color, 400, 1500);
  return { color, isNight };
}

/** 切换白天/夜间天空 */
export function setSkyNightMode(scene, skyEnv, isNight) {
  const color = isNight ? NIGHT_COLOR : DAY_COLOR;
  scene.background.setHex(color);
  if (scene.fog) scene.fog.color.setHex(color);
  if (skyEnv) { skyEnv.color = color; skyEnv.isNight = isNight; }
}
