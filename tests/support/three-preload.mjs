/**
 * three-preload.mjs — 预加载脚本
 *
 * 在 Node.js 测试启动前设置全局 THREE mock + 注册 loader hook。
 *
 * 用法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_module_load.test.mjs
 */

import { register } from 'node:module';

// 注册自定义 loader hook，拦截 import 'three' → three-shim.mjs
register('./three-loader.mjs', import.meta.url);

// 设置 global.THREE（vis/ 模块通过 window.THREE 或 globalThis.THREE 访问）
import * as THREE from './three-shim.mjs';
globalThis.THREE = THREE;
globalThis.window = globalThis.window || { THREE, document: undefined, location: { search: '' } };

// 浏览器 API shim（vis/ 模块可能用到的）
if (!globalThis.performance) {
  globalThis.performance = { now: () => Date.now() };
}
if (!globalThis.requestAnimationFrame) {
  globalThis.requestAnimationFrame = (cb) => setTimeout(cb, 16);
}
if (!globalThis.cancelAnimationFrame) {
  globalThis.cancelAnimationFrame = (id) => clearTimeout(id);
}
if (!globalThis.ResizeObserver) {
  globalThis.ResizeObserver = class { observe() {} unobserve() {} disconnect() {} };
}