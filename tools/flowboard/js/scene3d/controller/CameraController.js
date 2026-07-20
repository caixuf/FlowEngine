/**
 * CameraController.js — MVC Controller 层：相机模式 + 轨道交互
 *
 * 管理：
 * - 5 种相机模式（chase/top/orbit/driver/front）
 * - orbit 模式的鼠标拖拽旋转、滚轮缩放
 * - NPC 点击 raycast 的鼠标坐标采集（命中后的处理通过回调注入）
 *
 * 不直接访问 scene3d 的渲染状态；_renderFrame 通过 getMode() / getOrbitState()
 * 读取当前配置，再计算具体相机位置。
 */

const THREE = window.THREE;

const VALID_MODES = ['chase', 'top', 'orbit', 'driver', 'front', 'map'];

let _camMode = 'chase';
let _orbitState = null;   // { azimuth, polar, distance, target }
let _orbitDragging = false;
let _orbitLast = { x: 0, y: 0 };
// map 模式状态：用户在鸟瞰图里拖拽平移 + 滚轮缩放。
//   offset.x/z  = 相机相对 ego 中心的平移（世界米）
//   height      = 相机离地高度（决定缩放级别，越大视野越广）
// 切到 map 模式时按 ego 位置初始化 offset=0；用户拖拽时累积 offset；
// 切回其它模式不重置，下次回 map 仍保留上次位置（用户体验更好）。
let _mapState = { offset: { x: 0, z: 0 }, height: 120, followEgo: true };
let _mapDragging = false;
let _mapLast = { x: 0, y: 0 };
let _raycaster = null;
let _mouse = null;

/**
 * 切换相机模式。
 * @param {string} mode
 * @param {Object} opts  { onResetSmoothCamera(sx, sz) } 可选回调，用于切回 chase 时重置平滑相机
 */
export function setCameraMode(mode, opts) {
  if (VALID_MODES.indexOf(mode) < 0) return;
  _camMode = mode;
  if (mode === 'orbit' && !_orbitState) {
    _orbitState = {
      azimuth: Math.PI * 0.15,
      polar: Math.PI / 3,
      distance: 28,
      target: new THREE.Vector3()
    };
  }
  // 切回 chase 时通知 View 重置平滑相机，避免旧位置突兀过渡
  if (mode === 'chase' && opts && typeof opts.onResetSmoothCamera === 'function') {
    opts.onResetSmoothCamera();
  }
}

/** 重置为默认 chase 视角 */
export function resetCamera(opts) {
  setCameraMode('chase', opts);
}

export function getMode() { return _camMode; }
export function getOrbitState() { return _orbitState; }
export function getMapState() { return _mapState; }

/**
 * 重置 map 模式到默认状态：相机重新对准 ego、高度归 220m。
 * 用户拖远后想找回 ego 时调这个（按钮/快捷键）。
 */
export function resetMapView() {
  _mapState.offset.x = 0;
  _mapState.offset.z = 0;
  _mapState.height = 120;
  _mapState.followEgo = true;
}

/**
 * 初始化鼠标轨道控制 + NPC 点击检测。
 * @param {HTMLCanvasElement} canvas
 * @param {Object} callbacks
 *   - pickNPC(mouse: Vector2, raycaster: Raycaster) : void
 */
export function initCameraControls(canvas, callbacks) {
  if (!canvas) return;
  _raycaster = new THREE.Raycaster();
  _mouse = new THREE.Vector2();
  var dragMoved = false, downPos = { x: 0, y: 0 };

  canvas.addEventListener('mousedown', function(e) {
    dragMoved = false;
    downPos.x = e.clientX; downPos.y = e.clientY;
    if (_camMode === 'orbit') {
      _orbitDragging = true;
      _orbitLast.x = e.clientX; _orbitLast.y = e.clientY;
    } else if (_camMode === 'map') {
      // map 模式：左键拖拽平移相机（世界 X-Z 平面）。
      // 拖拽即解除 followEgo，避免 ego 跑着把相机又拽回去。
      _mapDragging = true;
      _mapLast.x = e.clientX; _mapLast.y = e.clientY;
      _mapState.followEgo = false;
    }
  });
  window.addEventListener('mousemove', function(e) {
    if (Math.abs(e.clientX - downPos.x) > 3 || Math.abs(e.clientY - downPos.y) > 3) dragMoved = true;
    if (_orbitDragging) {
      var dx = e.clientX - _orbitLast.x;
      var dy = e.clientY - _orbitLast.y;
      _orbitLast.x = e.clientX; _orbitLast.y = e.clientY;
      _orbitState.azimuth -= dx * 0.008;
      _orbitState.polar = Math.max(0.12, Math.min(Math.PI / 2 - 0.08, _orbitState.polar + dy * 0.008));
    } else if (_mapDragging) {
      // 鸟瞰图正下方俯视：屏幕右移 → 世界 +X；屏幕下移 → 世界 -Z（相机看 -Y 方向）。
      // 拖拽比例：每像素 = height/800 米（高度越大，每像素代表的世界距离越大，
      // 高空拖拽不会龟速，低空拖拽不会跳过快）。
      var dx = e.clientX - _mapLast.x;
      var dy = e.clientY - _mapLast.y;
      _mapLast.x = e.clientX; _mapLast.y = e.clientY;
      var scale = _mapState.height / 800;
      _mapState.offset.x += dx * scale;
      _mapState.offset.z -= dy * scale;
    }
  });
  window.addEventListener('mouseup', function() {
    _orbitDragging = false;
    _mapDragging = false;
  });
  canvas.addEventListener('wheel', function(e) {
    if (_camMode === 'orbit') {
      e.preventDefault();
      _orbitState.distance *= (e.deltaY > 0) ? 1.08 : 0.92;
      _orbitState.distance = Math.max(4, Math.min(120, _orbitState.distance));
    } else if (_camMode === 'map') {
      // map 模式滚轮缩放 = 调整相机高度（30-500m 范围）。
      // deltaY>0 = 滚轮向下 = 拉远（高度增加），与 orbit 语义一致。
      e.preventDefault();
      _mapState.height *= (e.deltaY > 0) ? 1.12 : 0.89;
      _mapState.height = Math.max(30, Math.min(800, _mapState.height));
    }
  }, { passive: false });

  // C.2: 点击 NPC 显示信息面板（任何模式下都可用）
  canvas.addEventListener('click', function(e) {
    if (dragMoved) return;
    var rect = canvas.getBoundingClientRect();
    _mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    _mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    if (callbacks && typeof callbacks.pickNPC === 'function') {
      callbacks.pickNPC(_mouse, _raycaster);
    }
  });
}
