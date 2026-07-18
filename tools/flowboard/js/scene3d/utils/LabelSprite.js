// ═══════════════════════════════════════════════════════════════
// LabelSprite.js — NPC AI 状态标签 Sprite 工厂
// ═══════════════════════════════════════════════════════════════
// 职责：创建/绘制浮在车顶上方的 AI 状态标签 Sprite（CanvasTexture）。
// 设计：纯工厂函数，不持有状态。ai 字符串变化时才重建纹理。
// ═══════════════════════════════════════════════════════════════

const THREE = window.THREE;

/**
 * 创建一个 NPC AI 状态标签 Sprite（128×32 CanvasTexture）。
 * 世界坐标 4m 宽 1m 高，默认不可见。
 */
export function makeLabelSprite() {
  var canvas = document.createElement('canvas');
  canvas.width = 128; canvas.height = 32;
  var tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.LinearFilter;
  var mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
  var sp = new THREE.Sprite(mat);
  sp.scale.set(4, 1, 1);   // 世界坐标 4m 宽 1m 高
  sp.visible = false;
  return sp;
}

/**
 * 在 Sprite canvas 上绘制圆角背景 + AI 状态文字。
 * @param {THREE.Sprite} sp - makeLabelSprite 返回的 sprite
 * @param {string} text - 要显示的文字（空字符串则清空）
 * @param {number} colorHex - 文字颜色（0xRRGGBB）
 */
export function setLabelSprite(sp, text, colorHex) {
  if (!sp || !sp.material || !sp.material.map) return;
  var canvas = sp.material.map.image;
  var ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (text) {
    // 圆角背景
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    var r = 6, w = canvas.width, h = canvas.height;
    ctx.beginPath();
    ctx.moveTo(r, 0); ctx.lineTo(w - r, 0);
    ctx.quadraticCurveTo(w, 0, w, r); ctx.lineTo(w, h - r);
    ctx.quadraticCurveTo(w, h, w - r, h); ctx.lineTo(r, h);
    ctx.quadraticCurveTo(0, h, 0, h - r); ctx.lineTo(0, r);
    ctx.quadraticCurveTo(0, 0, r, 0); ctx.closePath(); ctx.fill();
    // 文字
    var hex = ('000000' + (colorHex & 0xffffff).toString(16)).slice(-6);
    ctx.fillStyle = '#' + hex;
    ctx.font = 'bold 18px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, w / 2, h / 2 + 1);
  }
  sp.material.map.needsUpdate = true;
}
