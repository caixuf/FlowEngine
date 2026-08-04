/**
 * CanvasTextureFactory.js — 统一 Canvas 纹理工厂
 *
 * 所有视图的 Canvas 纹理创建唯一入口，消除重复代码。
 * 统一字体栈、抗锯齿、padding、颜色规范。
 *
 * 提供三类纹理：
 *   1. makeLabelTexture   — 实体标签（speed + ai_state，圆角背景）
 *   2. makeSignTexture    — 交通指示牌（文字 + 背景色 + 边框）
 *   3. makeStripeTexture  — 条纹纹理（施工围栏、警示带等）
 *
 * 所有纹理自动缓存，相同参数返回同一纹理对象。
 */

// ── 统一字体栈 ──
const FONT_MONO = `'JetBrains Mono', 'Inter', monospace, sans-serif`;
const FONT_SANS = `'Inter', 'Microsoft YaHei', 'PingFang SC', 'Noto Sans SC', sans-serif`;
const FONT_SIGN = `'Microsoft YaHei', 'SimHei', 'Noto Sans SC', 'PingFang SC', sans-serif`;

// ── 缓存 ──
const _cache = new Map();
/* 缓存上界：静态纹理（指示牌/条纹/低变化 label）数量有限，防止意外路径
 * 把缓存撑到无界（label 的 speed 是典型高变 key，已走 noCache）。 */
const _CACHE_MAX = 300;

function _cacheKey(...args) {
  return args.map(a => (typeof a === 'object' ? JSON.stringify(a) : String(a))).join('|');
}

/** 通用 canvas 创建 */
function _createCanvas(w, h) {
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  return c;
}

/** 通用 CanvasTexture 创建 */
function _toTexture(canvas, wrapS = THREE.ClampToEdgeWrapping, wrapT = THREE.ClampToEdgeWrapping) {
  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = wrapS;
  tex.wrapT = wrapT;
  return tex;
}

// ═══════════════════════════════════════════════════════════
// 1. 实体标签纹理（LabelView 用）
// ═══════════════════════════════════════════════════════════

/**
 * 创建实体标签纹理（speed + ai_state，圆角半透明背景）
 * @param {number|null} speed  速度 (m/s)，null 显示 '?'
 * @param {string} aiState     AI 状态文本，空字符串不显示
 * @param {object} [opts]      可选配置
 * @param {number} [opts.width=256]   画布宽度
 * @param {number} [opts.height=64]   画布高度
 * @param {number} [opts.fontSize=28] 主文字大小
 * @returns {THREE.CanvasTexture}
 */
export function makeLabelTexture(speed, aiState, opts = {}) {
  const { width = 256, height = 64, fontSize = 28, noCache = false } = opts;
  // 车速每帧在变（20.01→20.02…），若按 speed 进缓存则每帧新增一个 key →
  // 缓存无界膨胀（浏览器 5GB 泄漏，2026-08-04 实测）。LabelView 换纹理时
  // 会 dispose 旧纹理，所以 ephemeral label 直接 noCache，不进缓存。
  if (!noCache) {
    const key = _cacheKey('label', speed, aiState, width, height, fontSize);
    if (_cache.has(key)) return _cache.get(key);
    const tex = _buildLabelTexture(speed, aiState, width, height, fontSize);
    // 缓存有界保护：只缓存静态 label（speed 不变才复用）；超限丢最旧。
    if (_cache.size >= _CACHE_MAX) {
      const oldest = _cache.keys().next().value;
      _cache.delete(oldest);
    }
    _cache.set(key, tex);
    return tex;
  }
  return _buildLabelTexture(speed, aiState, width, height, fontSize);
}

/** 实际构建 label 纹理（供 noCache 路径与缓存路径共用） */
function _buildLabelTexture(speed, aiState, width, height, fontSize) {
  const c = _createCanvas(width, height);
  const ctx = c.getContext('2d');

  // 速度文本
  const speedText = (speed != null ? speed.toFixed(1) : '?') + ' m/s';
  ctx.font = `bold ${fontSize}px ${FONT_MONO}`;
  const speedW = ctx.measureText(speedText).width;

  // ai_state 文本
  let stateText = '';
  let stateW = 0;
  if (aiState && aiState !== '') {
    stateText = aiState;
    ctx.font = `${fontSize * 0.7}px ${FONT_SANS}`;
    stateW = ctx.measureText(stateText).width;
  }

  // 布局计算
  const gap = stateText ? 16 : 0;
  const totalW = speedW + gap + stateW;
  const startX = (width - totalW) / 2;
  const padX = 14, padY = 8;
  const bgX = startX - padX;
  const bgW = totalW + padX * 2;
  const bgY = (height - fontSize) / 2 - padY;
  const bgH = fontSize + padY * 2;

  // 背景圆角矩形（2026-08 淡化：0.65 太黑，观感像"一块黑的长方形"；
  // 0.4 半透明 + 细边框，文字为主背景为辅）
  ctx.beginPath();
  ctx.roundRect(bgX, bgY, bgW, bgH, 8);
  ctx.fillStyle = 'rgba(10, 16, 24, 0.40)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.18)';
  ctx.lineWidth = 1;
  ctx.stroke();

  // 速度文字（加细描边，浅背景上仍可读）
  ctx.font = `bold ${fontSize}px ${FONT_MONO}`;
  ctx.textBaseline = 'middle';
  ctx.lineWidth = 3;
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.7)';
  ctx.strokeText(speedText, startX, height / 2);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(speedText, startX, height / 2);

  // ai_state 文字
  if (stateText) {
    ctx.font = `${fontSize * 0.7}px ${FONT_SANS}`;
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.strokeText(stateText, startX + speedW + gap, height / 2);
    ctx.fillStyle = '#c9d4e0';
    ctx.fillText(stateText, startX + speedW + gap, height / 2);
  }

  return _toTexture(c);
}

// ═══════════════════════════════════════════════════════════
// 2. 交通指示牌纹理（ConstructionView / ETCGateView 用）
// ═══════════════════════════════════════════════════════════

/**
 * 创建交通指示牌纹理（文字 + 背景色 + 边框）
 * @param {string} text     指示牌文字
 * @param {string} bg       背景色（CSS 颜色值）
 * @param {string} fg       文字颜色（CSS 颜色值）
 * @param {string} border   边框颜色（CSS 颜色值）
 * @param {object} [opts]   可选配置
 * @param {number} [opts.width=256]   画布宽度
 * @param {number} [opts.height=192]  画布高度
 * @param {number} [opts.fontSize=54] 文字大小
 * @param {number} [opts.borderWidth=5] 边框宽度
 * @returns {THREE.CanvasTexture}
 */
export function makeSignTexture(text, bg, fg, border, opts = {}) {
  const { width = 256, height = 192, fontSize = 54, borderWidth = 5 } = opts;
  const key = _cacheKey('sign', text, bg, fg, border, width, height, fontSize, borderWidth);
  if (_cache.has(key)) return _cache.get(key);

  const c = _createCanvas(width, height);
  const ctx = c.getContext('2d');

  // 背景
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, width, height);

  // 边框
  ctx.strokeStyle = border;
  ctx.lineWidth = borderWidth;
  ctx.strokeRect(borderWidth / 2, borderWidth / 2, width - borderWidth, height - borderWidth);

  // 文字
  ctx.fillStyle = fg;
  ctx.font = `bold ${fontSize}px ${FONT_SIGN}`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, width / 2, height / 2);

  const tex = _toTexture(c);
  _cache.set(key, tex);
  return tex;
}

// ═══════════════════════════════════════════════════════════
// 3. 条纹纹理（ConstructionView 施工围栏用）
// ═══════════════════════════════════════════════════════════

/**
 * 创建条纹纹理（横向交替色带 + 反光渐变）
 * @param {string} color1   颜色1（奇数条）
 * @param {string} color2   颜色2（偶数条）
 * @param {object} [opts]   可选配置
 * @param {number} [opts.width=64]    画布宽度
 * @param {number} [opts.height=256]  画布高度
 * @param {number} [opts.stripeCount=8] 条纹数量
 * @param {boolean} [opts.reflective=true] 是否加反光渐变
 * @returns {THREE.CanvasTexture}
 */
export function makeStripeTexture(color1, color2, opts = {}) {
  const { width = 64, height = 256, stripeCount = 8, reflective = true } = opts;
  const key = _cacheKey('stripe', color1, color2, width, height, stripeCount, reflective);
  if (_cache.has(key)) return _cache.get(key);

  const c = _createCanvas(width, height);
  const ctx = c.getContext('2d');
  const stripeH = height / stripeCount;

  for (let i = 0; i < stripeCount; i++) {
    ctx.fillStyle = i % 2 === 0 ? color1 : color2;
    ctx.fillRect(0, i * stripeH, width, stripeH);
  }

  // 反光渐变覆盖
  if (reflective) {
    const g = ctx.createLinearGradient(0, 0, width, 0);
    g.addColorStop(0, 'rgba(255,255,255,0.15)');
    g.addColorStop(0.5, 'rgba(255,255,255,0.0)');
    g.addColorStop(1, 'rgba(255,255,255,0.25)');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, width, height);
  }

  const tex = _toTexture(c, THREE.RepeatWrapping, THREE.RepeatWrapping);
  _cache.set(key, tex);
  return tex;
}

// ═══════════════════════════════════════════════════════════
// 工具函数
// ═══════════════════════════════════════════════════════════

/** 清空缓存（测试/切场景时调用） */
export function clearTextureCache() {
  for (const [, tex] of _cache) {
    tex.dispose();
  }
  _cache.clear();
}

/** 获取缓存大小（调试用） */
export function getTextureCacheSize() {
  return _cache.size;
}
