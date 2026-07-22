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
 * - _failed 集合避免坏 view 在 60fps 循环里每帧刷屏
 * - resetFailures() 供测试/切场景后重试
 */

const _factories = new Map();   // name → factory(scene) → view
const _instances = new Map();   // name → view 实例
const _failed   = new Set();    // 抛过错的 view name，跳过后续调用

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

/** 实例化指定 view。factory 抛错只 log，不抛出。 */
export function instantiate(name, scene) {
  const factory = _factories.get(name);
  if (!factory) return null;
  if (_instances.has(name)) return _instances.get(name);
  try {
    const view = factory(scene);
    _instances.set(name, view);
    return view;
  } catch (err) {
    console.error('[ViewRegistry] instantiate "' + name + '" failed:', err);
    _failed.add(name);
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
 * 已知坏的 view 直接跳过，避免 60fps 刷屏。
 * @returns {any|undefined} 方法返回值；失败/跳过返回 undefined
 */
export function safeCall(name, method, ...args) {
  if (_failed.has(name)) return undefined;
  const view = _instances.get(name);
  if (!view || typeof view[method] !== 'function') return undefined;
  try {
    return view[method](...args);
  } catch (err) {
    console.error('[ViewRegistry] ' + name + '.' + method + '() threw:', err);
    _failed.add(name);
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
