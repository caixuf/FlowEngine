/**
 * Renderer.js — WebGLRenderer + 渲染循环 + 后处理
 * 管理渲染管线，支持性能档位（high/medium/low）。
 */

import { createLighting, setNightMode } from './Lighting.js';
import { createSkyEnv, setSkyNightMode } from './SkyEnv.js';

export function createRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({
    canvas, antialias: true, powerPreference: 'high-performance'
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;

  return renderer;
}

/** 创建后处理 Composer。
 *  Phase 3 简化：禁用 Composer（WebGLRenderer antialias:true 已提供 MSAA，
 *  SMAAPass 依赖链不稳定且收益有限）。如需 bloom/后处理，再启用。
 *  返回 null 表示走 renderer.render() 直接渲染。 */
export function createComposer(renderer, scene, camera) {
  return null;
}

/** 渲染一帧 */
export function renderFrame(renderer, composer, scene, camera) {
  if (composer) composer.render();
  else renderer.render(scene, camera);
}

/** 调整大小 */
export function resize(renderer, composer, camera, width, height) {
  renderer.setSize(width, height);
  if (composer) composer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}
