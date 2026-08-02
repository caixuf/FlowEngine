/**
 * StatsView.js — 性能监控面板（FPS / drawcall / triangle / 纹理缓存）
 *
 * 挂在 DOM 层（非 3D 场景），每帧读取 renderer.info 统计渲染指标。
 * 默认显示在右上角，可通过 CSS 调整位置。
 *
 * 使用方式：
 *   const stats = createStatsView();
 *   document.body.appendChild(stats.dom);
 *   // 每帧渲染循环中
 *   stats.update(renderer);
 *
 * 显示内容：
 *   FPS      — 帧率（绿色 >=50，黄色 >=30，红色 <30）
 *   ms       — 帧耗时
 *   Draw     — drawcall 数量
 *   Tri      — 三角形数量
 *   Geo      — geometry 内存占用
 *   Tex      — texture 内存占用（含 CanvasTextureFactory 缓存）
 *   Cache    — CanvasTextureFactory 缓存条目数
 */

import { getTextureCacheSize } from '../utils/CanvasTextureFactory.js';

const UPDATE_INTERVAL = 500;  // 每 500ms 更新一次显示（避免每帧 DOM 操作）

export function createStatsView() {
  const container = document.createElement('div');
  container.id = 'stats-view';
  container.style.cssText = `
    position: fixed;
    top: 8px;
    right: 8px;
    z-index: 9999;
    background: rgba(0, 0, 0, 0.75);
    color: #0f0;
    font-family: 'JetBrains Mono', 'Consolas', monospace;
    font-size: 11px;
    line-height: 1.4;
    padding: 8px 10px;
    border-radius: 4px;
    border: 1px solid rgba(0, 255, 0, 0.3);
    pointer-events: none;
    user-select: none;
    min-width: 120px;
  `;

  const rows = {
    fps:   _createRow(container, 'FPS', '#0f0'),
    ms:    _createRow(container, 'ms', '#0f0'),
    draw:  _createRow(container, 'Draw', '#8af'),
    tri:   _createRow(container, 'Tri', '#8af'),
    geo:   _createRow(container, 'Geo', '#fa0'),
    tex:   _createRow(container, 'Tex', '#fa0'),
    cache: _createRow(container, 'Cache', '#f0f'),
  };

  let lastTime = performance.now();
  let frameCount = 0;
  let fps = 0;
  let lastUpdate = 0;

  function _createRow(parent, label, color) {
    const row = document.createElement('div');
    row.style.cssText = 'display:flex;justify-content:space-between;';
    const labelEl = document.createElement('span');
    labelEl.textContent = label;
    labelEl.style.color = '#888';
    const valueEl = document.createElement('span');
    valueEl.style.color = color;
    valueEl.textContent = '-';
    row.appendChild(labelEl);
    row.appendChild(valueEl);
    parent.appendChild(row);
    return valueEl;
  }

  function _formatNumber(n) {
    if (n >= 1000000) return (n / 1000000).toFixed(1) + 'M';
    if (n >= 1000) return (n / 1000).toFixed(1) + 'K';
    return String(n);
  }

  function _formatBytes(bytes) {
    if (bytes >= 1048576) return (bytes / 1048576).toFixed(1) + 'MB';
    if (bytes >= 1024) return (bytes / 1024).toFixed(1) + 'KB';
    return bytes + 'B';
  }

  /**
   * 每帧调用（在 renderer.render 之后）
   * @param {THREE.WebGLRenderer} renderer
   */
  function update(renderer) {
    frameCount++;
    const now = performance.now();

    // 每 500ms 更新一次 DOM
    if (now - lastUpdate < UPDATE_INTERVAL) return;

    fps = Math.round(frameCount * 1000 / (now - lastTime));
    const ms = ((now - lastTime) / frameCount).toFixed(1);
    frameCount = 0;
    lastTime = now;
    lastUpdate = now;

    // FPS 颜色：绿 >=50，黄 >=30，红 <30
    const fpsColor = fps >= 50 ? '#0f0' : fps >= 30 ? '#ff0' : '#f00';
    rows.fps.style.color = fpsColor;
    rows.fps.textContent = fps;
    rows.ms.textContent = ms;

    if (renderer && renderer.info) {
      const info = renderer.info;
      rows.draw.textContent = _formatNumber(info.render.calls);
      rows.tri.textContent = _formatNumber(info.render.triangles);
      rows.geo.textContent = _formatBytes(info.memory.geometries * 1000);  // 估算
      rows.tex.textContent = _formatBytes(info.memory.textures * 1000);    // 估算
    }

    rows.cache.textContent = getTextureCacheSize();
  }

  function dispose() {
    if (container.parentNode) {
      container.parentNode.removeChild(container);
    }
  }

  return { dom: container, update, dispose };
}
