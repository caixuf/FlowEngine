/**
 * Layer.js — Qt 对象树风格的 View 容器节点
 *
 * 借鉴 Qt QObject 的核心三件事：
 *  1. 父子关系 — Layer 持有 children + parent，构造时挂到父
 *  2. 单向依赖 — 子只读 store + 持有 parent 引用（拿共享资源），
 *               绝不反向调父方法（Qt 信号槽默认子→父方向）
 *  3. 递归 dispose — 父 dispose 时自动遍历 children，每个 child 调
 *               dispose()，子再递归到孙。避免手动管生命周期导致 leak
 *
 * 错误隔离：每个 child 的 build/update 抛错只 log + 跳过，不传染兄弟。
 * 这正是"一个模块坏了整个 3D 就坏了"的解药。
 *
 * 与 ViewRegistry 的分工：
 *  - ViewRegistry 管"插件注册 + 工厂实例化"
 *  - Layer 管"运行时父子树 + 递归调用 + dispose"
 *  一个 View 既被 registry 实例化，又被挂到某个 Layer 下作为 child。
 *
 * 用法：
 *   const root = createLayer('root', scene);
 *   const roadLayer = root.addChild(createLayer('road', scene));
 *   roadLayer.addView(ViewRegistry.get('road'));        // view 作为 child
 *   roadLayer.addView(ViewRegistry.get('streetlight'));
 *   root.build({ rn });        // 递归 build 所有 child
 *   root.update(store);        // 递归 update 所有 child
 *   root.dispose();            // 递归销毁所有 child + 清 scene
 */

let _layerId = 0;

export function createLayer(name, scene, parent = null) {
  const id = ++_layerId;
  const children = [];          // Layer[]  子层
  const views = [];             // View 实例[] 直接挂的本层 view
  let _disposed = false;
  let _failed = false;

  const layer = {
    id,
    name,
    scene,
    parent,

    /* ── 树操作 ── */

    /** 添加子层，自动设 parent。返回子层本身便于链式。 */
    addChild(child) {
      if (!child || child.parent === layer) return child;
      if (child.parent) child.parent.removeChild(child);
      child.parent = layer;
      children.push(child);
      return child;
    },

    /** 添加 view 实例到本层。view 必须有 build/update/clear 方法（可选 dispose）。 */
    addView(view) {
      if (!view) return;
      views.push(view);
      return view;
    },

    removeChild(child) {
      const i = children.indexOf(child);
      if (i >= 0) {
        children.splice(i, 1);
        if (child.parent === layer) child.parent = null;
      }
    },

    /** 按名字找子层（不递归孙）。 */
    findChild(name) {
      return children.find(c => c.name === name) || null;
    },

    /** 递归按名字找后代（深度优先）。 */
    findDescendant(name) {
      for (const c of children) {
        if (c.name === name) return c;
        const d = c.findDescendant(name);
        if (d) return d;
      }
      return null;
    },

    getChildren() { return children.slice(); },
    getViews()    { return views.slice(); },
    isFailed()    { return _failed; },
    isDisposed()  { return _disposed; },

    /* ── 递归调用（核心）── */

    /** 递归 build：先本层 views，再子层。每个调用包 try/catch。 */
    build(ctx) {
      if (_disposed || _failed) return;
      for (const v of views) {
        if (typeof v.build !== 'function') continue;
        try { v.build(ctx); }
        catch (err) { console.error('[Layer ' + name + '] view.build threw:', err); }
      }
      for (const c of children) {
        try { c.build(ctx); }
        catch (err) { console.error('[Layer ' + name + '] child ' + c.name + '.build threw:', err); }
      }
    },

    /** 递归 update：先本层 views，再子层。每帧调用。 */
    update(store, now) {
      if (_disposed || _failed) return;
      for (const v of views) {
        if (typeof v.update !== 'function') continue;
        try { v.update(store, now); }
        catch (err) {
          console.error('[Layer ' + name + '] view.update threw:', err);
          /* update 抛错不标记 _failed（build 才标记）—— 60fps 偶发错可能下次自愈 */
        }
      }
      for (const c of children) {
        try { c.update(store, now); }
        catch (err) { console.error('[Layer ' + name + '] child ' + c.name + '.update threw:', err); }
      }
    },

    /** 递归 clear：清掉所有 view 的资源 + 从 scene 移除。
     * clear 不销毁 layer 本身（与 dispose 不同），用于切场景时重置。 */
    clear() {
      for (const v of views) {
        if (typeof v.clear !== 'function') continue;
        try { v.clear(); }
        catch (err) { console.error('[Layer ' + name + '] view.clear threw:', err); }
      }
      for (const c of children) {
        try { c.clear(); }
        catch (err) { console.error('[Layer ' + name + '] child ' + c.name + '.clear threw:', err); }
      }
    },

    /** 标记本层失败（build 抛错后调用，跳过后续 build/update）。 */
    markFailed() { _failed = true; },

    /** 重置失败标记（切场景 / 测试用）。 */
    resetFailure() {
      _failed = false;
      for (const c of children) c.resetFailure();
    },

    /** 递归销毁：先 dispose 子层，再清本层 views（调 view.clear/dispose），
     * 从 parent 移除自己。view 若有 dispose() 方法则调，否则退化为 clear()。
     * 幂等：重复调不报错。 */
    dispose() {
      if (_disposed) return;
      _disposed = true;
      /* 先递归子层（深度优先，确保孙先销毁） */
      while (children.length) {
        const c = children.pop();
        try { c.dispose(); }
        catch (err) { console.error('[Layer ' + name + '] child ' + c.name + '.dispose threw:', err); }
      }
      /* 再清本层 views */
      for (const v of views) {
        try {
          if (typeof v.dispose === 'function') v.dispose();
          else if (typeof v.clear === 'function') v.clear();
        } catch (err) {
          console.error('[Layer ' + name + '] view dispose/clear threw:', err);
        }
      }
      views.length = 0;
      /* 从 parent 移除 */
      if (parent) parent.removeChild(layer);
      parent = null;
    },
  };

  return layer;
}
