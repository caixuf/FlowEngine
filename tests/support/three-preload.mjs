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

// ── 最小 DOM shim ──
// 仅覆盖 vis/ 视图实际用到的 document.createElement('canvas')（LabelView /
// ConstructionView 的纹理工厂），让 build/update 在 Node 下真正执行——否则抛
// ReferenceError 被 Layer 吞掉 → 门禁假绿（新 view 代码从未被测试到）。
// getElementById 等返回 null / 空，与"无 DOM 时跳过浏览器专用分支"行为一致。
function _makeCanvas() {
  // 2D ctx 桩：覆盖 vis/ 全部纹理生成路径（Road/Ground/Barrier/Label/Sign）用到
  // 的方法。除 measureText / getImageData 返回可观察对象外，其余全部 no-op。
  const ctx = {
    fillStyle: '', strokeStyle: '', lineWidth: 1, font: '',
    textAlign: 'left', textBaseline: 'alphabetic',
    globalAlpha: 1, lineDashOffset: 0, lineJoin: 'miter',
    shadowBlur: 0, shadowColor: 'rgba(0,0,0,0)',
    clearRect() {}, fillRect() {}, strokeRect() {}, fillText() {}, strokeText() {},
    beginPath() {}, closePath() {}, moveTo() {}, lineTo() {},
    stroke() {}, fill() {}, arc() {}, roundRect() {},
    quadraticCurveTo() {}, save() {}, restore() {},
    translate() {}, rotate() {}, scale() {},
    drawImage() {}, setLineDash() {}, putImageData() {},
    measureText: () => ({ width: 0 }),
    getImageData: () => ({ data: new Uint8ClampedArray(0), width: 0, height: 0 }),
    createLinearGradient() { return { addColorStop() {} }; },
    createRadialGradient() { return { addColorStop() {} }; },
  };
  return { width: 0, height: 0, getContext: () => ctx };
}
const _document = {
  createElement: (tag) => (tag === 'canvas' ? _makeCanvas() : {}),
  getElementById: () => null,
  querySelector: () => null,
  querySelectorAll: () => [],
  body: { appendChild() {}, style: {} },
  addEventListener() {}, hidden: false,
};
globalThis.document = _document;
globalThis.window = globalThis.window || { THREE, document: _document, location: { search: '' } };

// RoadView 法线贴图生成用 new ImageData(data, w, h) 重建像素缓冲。
class _ImageData {
  constructor(data, width = 0, height = 0) {
    this.data = data || new Uint8ClampedArray(0);
    this.width = width;
    this.height = height;
  }
}
globalThis.ImageData = _ImageData;

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