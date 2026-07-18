// ═══════════════════════════════════════════════════════════════
// PerfModel.js — 性能档位 + FPS 统计
// ═══════════════════════════════════════════════════════════════
// 职责：管理性能档位状态（high/medium/low），统计 FPS 自动升降档。
// 设计：状态 + 统计逻辑分离。setTier 供手动覆盖，autoUpdate 供渲染循环调用。
// ═══════════════════════════════════════════════════════════════

/** @type {'high'|'medium'|'low'} */
let _tier = 'high';
let _frameCount = 0;
let _lastTime = 0;
let _checkInterval = 60; // 每 60 帧评估一次

/** @returns {'high'|'medium'|'low'} */
export function getTier() { return _tier; }

/** 手动设置档位 */
export function setTier(tier) {
  if (tier === 'high' || tier === 'medium' || tier === 'low') {
    _tier = tier;
  }
}

/**
 * 初始档位选择：URL 参数 ?perf=xxx 或屏幕尺寸。
 * @returns {'high'|'medium'|'low'}
 */
export function pickInitialTier() {
  var urlParams = new URLSearchParams(window.location.search);
  var initPerf = urlParams.get('perf');
  if (initPerf === 'low' || initPerf === 'medium' || initPerf === 'high') {
    _tier = initPerf;
  } else if (window.innerWidth < 700) {
    _tier = 'medium';
  }
  return _tier;
}

/**
 * 每帧调用，统计 FPS 并自动调整档位。
 * @returns {{changed: boolean, tier: string}} 本次是否发生档位变更
 */
export function autoUpdate() {
  _frameCount++;
  if (_frameCount < _checkInterval) {
    return { changed: false, tier: _tier };
  }
  var now = performance.now();
  if (_lastTime === 0) {
    _lastTime = now;
    _frameCount = 0;
    return { changed: false, tier: _tier };
  }
  var elapsed = (now - _lastTime) / 1000;
  var fps = _frameCount / elapsed;
  _lastTime = now;
  _frameCount = 0;

  var old = _tier;
  if (fps < 30 && _tier === 'high') _tier = 'medium';
  else if (fps < 20 && _tier === 'medium') _tier = 'low';
  else if (fps > 50 && _tier === 'low') _tier = 'medium';
  else if (fps > 55 && _tier === 'medium') _tier = 'high';

  return { changed: _tier !== old, tier: _tier };
}

/** 距离裁剪阈值（按档位返回，单位 m） */
export function getCullDistance() {
  if (_tier === 'low') return 120;
  if (_tier === 'medium') return 180;
  return 250; // high
}
