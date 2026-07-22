/**
 * SkyEnv.js — 天空背景 + 雾
 *
 * 用 ShaderMaterial 球状渐变天空（顶深蓝→底浅蓝→地平线暖白），
 * 远处雾色与天空地平线一致，自然融入；不依赖 HDR 文件，
 * 烘到 PMREM 也能给金属漆提供反射环境。
 *
 * 之前用纯色 scene.background，看起来像"贴了张蓝色硬纸板"，
 * 远处雾和天空接不上。现在换成渐变球，远景平滑消失到天际。
 */

import * as THREE from 'three';

const DAY_TOP    = new THREE.Color(0x1e90ff);  // 顶蓝（道奇蓝）
const DAY_BOTTOM = new THREE.Color(0xbfe4ff);  // 地平线浅蓝
const DAY_HORIZON = new THREE.Color(0xfff0d0); // 地平线暖白（大气散射）
const NIGHT_TOP    = new THREE.Color(0x0a0a2e);
const NIGHT_BOTTOM = new THREE.Color(0x1a1a3e);
const NIGHT_HORIZON = new THREE.Color(0x2a2a4a);

/* ShaderMaterial：球状渐变天空
 * - vWorld.y 决定颜色高度比例
 * - pow(max(h,0), exp) 模拟大气散射的指数曲线（参考 page 185 行）*/
const skyVertex = /* glsl */`
  varying vec3 vWorld;
  void main() {
    vec4 wp = modelMatrix * vec4(position, 1.0);
    vWorld = wp.xyz;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;
const skyFragment = /* glsl */`
  uniform vec3 topColor;
  uniform vec3 horizonColor;
  uniform vec3 bottomColor;
  uniform float exponent;
  uniform float offset;
  varying vec3 vWorld;
  void main() {
    float h = normalize(vWorld + vec3(0.0, offset, 0.0)).y;
    /* h>0: 上半部，顶→地平线渐变；h<=0: 下半部，horizon→底色 */
    float t = pow(max(h, 0.0), exponent);
    vec3 upper = mix(horizonColor, topColor, t);
    float b = pow(max(-h, 0.0), 0.6);
    vec3 lower = mix(horizonColor, bottomColor, b);
    gl_FragColor = vec4(h >= 0.0 ? upper : lower, 1.0);
  }
`;

function buildSkyDome(top, horizon, bottom) {
  const geo = new THREE.SphereGeometry(2000, 32, 16);
  const mat = new THREE.ShaderMaterial({
    uniforms: {
      topColor:     { value: top.clone() },
      horizonColor: { value: horizon.clone() },
      bottomColor:  { value: bottom.clone() },
      exponent:     { value: 0.6 },
      offset:       { value: 33.0 },
    },
    vertexShader: skyVertex,
    fragmentShader: skyFragment,
    side: THREE.BackSide,
    depthWrite: false,   // 不写深度，避免遮挡场景物体
  });
  const dome = new THREE.Mesh(geo, mat);
  dome.frustumCulled = false;
  return { dome, mat };
}

export function createSkyEnv(scene, isNight = false) {
  const top     = isNight ? NIGHT_TOP     : DAY_TOP;
  const horizon = isNight ? NIGHT_HORIZON : DAY_HORIZON;
  const bottom  = isNight ? NIGHT_BOTTOM  : DAY_BOTTOM;
  /* 背景 fallback：PMREMGenerator.fromScene 在天空之前创建会丢失颜色，
   * 此处先用 horizon 色做 background 兜底，建好球状天空后保留 background
   * 给 fog 用的底色（防渲染前 blank 一帧）。*/
  scene.background = horizon.clone();
  const { dome, mat } = buildSkyDome(top, horizon, bottom);
  scene.add(dome);
  /* 雾色 = 地平线色：远处路/树自然融入天空，远景不"断"
   * FogExp2：指数衰减比线性 Fog 更自然，远景不会突然"断"在某个距离。
   * density 0.0008：~500m 开始明显，~1000m 接近天空色，匹配天空球半径。 */
  scene.fog = new THREE.FogExp2(horizon, 0.0008);
  return { dome, mat, color: horizon, isNight };
}

/** 切换白天/夜间天空（用原地 uniform 改色避免重建球体） */
export function setSkyNightMode(scene, skyEnv, isNight) {
  const top     = isNight ? NIGHT_TOP     : DAY_TOP;
  const horizon = isNight ? NIGHT_HORIZON : DAY_HORIZON;
  const bottom  = isNight ? NIGHT_BOTTOM  : DAY_BOTTOM;
  if (skyEnv && skyEnv.mat) {
    skyEnv.mat.uniforms.topColor.value.copy(top);
    skyEnv.mat.uniforms.horizonColor.value.copy(horizon);
    skyEnv.mat.uniforms.bottomColor.value.copy(bottom);
  }
  if (scene.background && scene.background.isColor) scene.background.copy(horizon);
  if (scene.fog) scene.fog.color.copy(horizon);
  if (skyEnv) { skyEnv.color = horizon; skyEnv.isNight = isNight; }
}
