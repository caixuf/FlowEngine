/**
 * ViewRegistry.js — View 插件注册中心 + 错误隔离
 *
 * 借鉴 Qt 对象树 + 单向依赖：
 * - View 通过 register(name, factory) 注册，factory(scene) 返回 view 实例
 * - view 实例的任何方法调用经 safeCall 包 try/catch
 * - 一个 View 抛错只 log + 标记 _failed，后续跳过，兄弟 View 继续渲染
 *
 * 数据层契约：SceneStore schema 是 IDL，View 是消费方（插件）。
 * 一个 View 坏了不影响其他 View —— 这就是"插件化"的核心收益，
 * 直接对应"一个模块坏了整个 3D 就坏了"的痛点。
 *
 * 设计权衡：
 * - 不做 Layer 中间层（Qt 对象树后续可加，先做最小可行）
 * - _failed 采用指数退避而非永久标记：瞬时错误（一次 NaN payload / 临时
 *   缓冲不足）不应永久禁用一个 view；首次失败立即跳过避免 60fps 刷屏，
 *   之后按 backoff 周期重试，连续失败时退避指数增长（×1.5，上限 30s）
 *   但永不放弃——数据流恢复后 view 自动复活。
 * - resetFailures() 供测试/切场景后强制清零退避
 */

const _factories = new Map();   // name → factory(scene) → view
const _instances = new Map();   // name → view 实例
/* _failed: name → { sinceMs, backoffMs, retries }
 *   sinceMs   首次失败时间戳（用于 resetFailures 调试）
 *   backoffMs 当前退避时长（指数增长，上限 MAX_BACKOFF_MS）
 *   retries   连续失败次数（仅用于诊断） */
const _failed = new Map();

/** 退避参数。可通过 setBackoffPolicy() 在运行时覆盖（如降低测试延迟）。 */
let _policy = {
  initialMs: 2000,   // 首次失败后 2s 才重试
  growth:    1.5,     // 每次连续失败 ×1.5
  maxMs:     30000,   // 最多 30s 退避
};

export function setBackoffPolicy(p) {
  if (!p || typeof p !== 'object') return;
  if (typeof p.initialMs === 'number' && p.initialMs > 0)   _policy.initialMs = p.initialMs;
  if (typeof p.growth    === 'number' && p.growth    >= 1)  _policy.growth    = p.growth;
  if (typeof p.maxMs     === 'number' && p.maxMs     > 0)   _policy.maxMs     = p.maxMs;
}

function _markFailed(name) {
  const prev = _failed.get(name);
  const now = Date.now();
  if (!prev) {
    _failed.set(name, { sinceMs: now, backoffMs: _policy.initialMs, retries: 1 });
  } else {
    const next = Math.min(prev.backoffMs * _policy.growth, _policy.maxMs);
    _failed.set(name, { sinceMs: prev.sinceMs, backoffMs: next, retries: prev.retries + 1 });
  }
}

/** 退避已到期 → 允许重试。返回 true 表示"现在该重试"。 */
function _backoffExpired(name) {
  const f = _failed.get(name);
  if (!f) return true;
  return (Date.now() - f.sinceMs) >= f.backoffMs;
}

/** 重试成功 → 清除失败标记。失败期间 view 自动复活。 */
function _clearIfFailed(name) {
  _failed.delete(name);
}

export function register(name, factory) {
  if (typeof name !== 'string' || typeof factory !== 'function') {
    throw new TypeError('register(name, factory): name string, factory function');
  }
  _factories.set(name, factory);
}

export function unregister(name) {
  /* 同时清掉已实例化的实例（调 clear()）+ failed 标记，
   * 否则 unregister 后 has() 仍 true 会让人困惑。 */
  const view = _instances.get(name);
  if (view && typeof view.clear === 'function') {
    try { view.clear(); } catch (err) {
      console.error('[ViewRegistry] ' + name + '.clear() on unregister threw:', err);
    }
  }
  _instances.delete(name);
  _failed.delete(name);
  _factories.delete(name);
}

/** 实例化指定 view。factory 抛错只 log，不抛出。
 * 退避期内跳过实例化（避免每帧重建抛错对象）。退避到期会重试。 */
export function instantiate(name, scene) {
  const factory = _factories.get(name);
  if (!factory) return null;
  if (_instances.has(name)) return _instances.get(name);
  // 退避期内不重试实例化（避免 60fps 刷错）。退避到期才允许重建。
  if (_failed.has(name) && !_backoffExpired(name)) return null;
  try {
    const view = factory(scene);
    _instances.set(name, view);
    _clearIfFailed(name);   // 成功 → 清失败标记
    return view;
  } catch (err) {
    console.error('[ViewRegistry] instantiate "' + name + '" failed:', err);
    _markFailed(name);
    return null;
  }
}

/** 实例化所有已注册 view。 */
export function instantiateAll(scene) {
  for (const name of _factories.keys()) {
    if (!_instances.has(name)) instantiate(name, scene);
  }
  return _instances;
}

export function get(name) {
  return _instances.get(name) || null;
}

export function has(name) {
  return _instances.has(name);
}

export function names() {
  return Array.from(_instances.keys());
}

export function isFailed(name) {
  return _failed.has(name);
}

/** 重置失败标记（测试/切场景后调，让坏 view 重试一次）。 */
export function resetFailures() {
  _failed.clear();
}

/**
 * 安全调用 view 方法。失败只 log + 标记 _failed，不抛。
 * 退避期内跳过该 view（避免 60fps 刷屏）；退避到期后自动重试一次。
 * 重试成功 → 清除失败标记，view 复活。
 * @returns {any|undefined} 方法返回值；失败/跳过返回 undefined
 */
export function safeCall(name, method, ...args) {
  // 退避期内直接跳过。退避到期则 fall through 让下面 try 真的调用一次。
  if (_failed.has(name) && !_backoffExpired(name)) return undefined;
  const view = _instances.get(name);
  if (!view || typeof view[method] !== 'function') return undefined;
  try {
    const ret = view[method](...args);
    _clearIfFailed(name);   // 调用成功 → 清失败标记（瞬时错误自愈）
    return ret;
  } catch (err) {
    console.error('[ViewRegistry] ' + name + '.' + method + '() threw:', err);
    _markFailed(name);
    return undefined;
  }
}

/** 销毁所有 view 实例（调 view.clear()），清空注册表。 */
export function clear() {
  for (const [name, view] of _instances) {
    if (view && typeof view.clear === 'function') {
      try { view.clear(); }
      catch (err) { console.error('[ViewRegistry] ' + name + '.clear() threw:', err); }
    }
  }
  _instances.clear();
  _failed.clear();
}

/** 清空所有状态（实例 + failed 标记 + factory 注册），测试间隔离用。 */
export function reset() {
  _instances.clear();
  _failed.clear();
  _factories.clear();
}
