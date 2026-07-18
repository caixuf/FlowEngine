// ═══════════════════════════════════════════════════════════════
// TopologyStore.js — 拓扑数据存储（Model 层入口）
// ═══════════════════════════════════════════════════════════════
// 职责：接收 app.js setTopoData() 推送的数据，提供只读访问器。
// 设计：单一数据源，View/Controller 只读不写。不引用 THREE。
// ═══════════════════════════════════════════════════════════════

/** @type {{nodes: Array, metrics: object}} */
let _topo = { nodes: [], metrics: {} };

/** 写入最新拓扑数据（由 app.js updateAll → setTopoData 调用） */
export function setTopoData(d) {
  _topo = d || _topo;
}

/** 读取完整拓扑数据 */
export function getTopoData() {
  return _topo;
}

/** 读取 metrics 子对象（空对象兜底） */
export function getMetrics() {
  return _topo.metrics || {};
}

/** 读取 scene 子对象（metrics.scene） */
export function getScene() {
  return (_topo.metrics || {}).scene || null;
}

/** 读取 vehicle 子对象（metrics.vehicle） */
export function getVehicle() {
  return (_topo.metrics || {}).vehicle || {};
}

/** 读取 nodes 列表 */
export function getNodes() {
  return _topo.nodes || [];
}
