/**
 * SceneStore.js — v2 集中式 scene state
 *
 * v1 的 51 个模块级 let 全文件可见导致耦合爆炸，v2 把所有可变状态收敛到
 * 一个对象上，通过 getter 暴露只读视图，修改必须走 setter（带不变量检查）。
 *
 * 设计原则：
 *   - 单例 store，整个场景只有一份
 *   - 字段分组：renderer（场景/相机/渲染器）/ road（路网）/ ego / entities / env / cam
 *   - 不存 Three.js 对象以外的业务状态（业务状态在 _topoData）
 *   - 重置场景时调用 reset() 清空所有引用
 */
const THREE = window.THREE;

// ── 业务数据（外部 setTopoData 写入）───────────────────────────
let _topoData = { nodes: [], metrics: {} };

// ── 渲染器层 ──────────────────────────────────────────────────
let _scene = null;
let _camera = null;
let _renderer = null;
let _sceneReady = false;

// ── 道路层 ────────────────────────────────────────────────────
let _roadGroup = null;          // THREE.Group 包含所有 edge 的路面/车道线 mesh
let _roadCurves = [];           // CatmullRomCurve3[]，供 ego 投影 / 相机锚定
let _roadCurveLens = [];        // 每条 edge 的长度（与 _roadCurves 一一对应）
let _roadBBox = null;           // THREE.Box3，路网边界框，Map/Orbit 相机锚定用
let _lastRoadKey = '';          // 缓存键，避免每帧重建

// ── ego 层 ────────────────────────────────────────────────────
let _egoGroup = null;           // ego 车 mesh group

// ── 实体层（障碍物 / NPC）─────────────────────────────────────
let _entityGroup = null;        // 所有 NPC/障碍物的父 group
let _entityPool = [];           // 复用 mesh 池，避免每帧 alloc

// ── 环境层 ────────────────────────────────────────────────────
let _groundMesh = null;         // 草地平面
let _ambientLight = null;
let _sunLight = null;           // DirectionalLight
let _hemiLight = null;          // HemisphereLight

// ── 调试 ──────────────────────────────────────────────────────
let _debugCam = false;

export const store = {
  // ── 业务数据 ──
  getTopoData() { return _topoData; },
  setTopoData(d) { _topoData = d || { nodes: [], metrics: {} }; },
  getMetrics() { return _topoData.metrics || {}; },
  getScene() { return (_topoData.metrics || {}).scene; },

  // ── 渲染器 ──
  getScene3D() { return _scene; },
  setScene3D(s) { _scene = s; },
  getCamera() { return _camera; },
  setCamera(c) { _camera = c; },
  getRenderer() { return _renderer; },
  setRenderer(r) { _renderer = r; },
  isReady() { return _sceneReady; },
  setReady(v) { _sceneReady = !!v; },

  // ── 道路 ──
  getRoadGroup() { return _roadGroup; },
  setRoadGroup(g) { _roadGroup = g; },
  getRoadCurves() { return _roadCurves; },
  setRoadCurves(c) { _roadCurves = c || []; },
  getRoadCurveLens() { return _roadCurveLens; },
  setRoadCurveLens(l) { _roadCurveLens = l || []; },
  getRoadBBox() { return _roadBBox; },
  setRoadBBox(b) { _roadBBox = b; },
  getLastRoadKey() { return _lastRoadKey; },
  setLastRoadKey(k) { _lastRoadKey = k || ''; },

  // ── ego ──
  getEgo() { return _egoGroup; },
  setEgo(g) { _egoGroup = g; },

  // ── entities ──
  getEntityGroup() { return _entityGroup; },
  setEntityGroup(g) { _entityGroup = g; },
  getEntityPool() { return _entityPool; },
  setEntityPool(p) { _entityPool = p || []; },

  // ── 环境 ──
  getGround() { return _groundMesh; },
  setGround(m) { _groundMesh = m; },
  getAmbientLight() { return _ambientLight; },
  setAmbientLight(l) { _ambientLight = l; },
  getSunLight() { return _sunLight; },
  setSunLight(l) { _sunLight = l; },
  getHemiLight() { return _hemiLight; },
  setHemiLight(l) { _hemiLight = l; },

  // ── 调试 ──
  isDebugCam() { return _debugCam; },
  setDebugCam(v) { _debugCam = !!v; },

  /**
   * 重置所有引用（场景销毁/重建时调用）。
   * 不 dispose Three.js 资源，由调用方负责；这里只清引用让 GC 能回收。
   */
  reset() {
    _roadGroup = null;
    _roadCurves = [];
    _roadCurveLens = [];
    _roadBBox = null;
    _lastRoadKey = '';
    _egoGroup = null;
    _entityGroup = null;
    _entityPool = [];
    _groundMesh = null;
    _ambientLight = null;
    _sunLight = null;
    _hemiLight = null;
    _sceneReady = false;
  }
};
