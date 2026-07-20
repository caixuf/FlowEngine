/**
 * scene3d_v2.js — v2 3D 场景入口
 *
 * 目标：保持 app.js 的 7 个 export API 完全兼容，替换 scene3d.js。
 *
 *   import { init3DScene, resize3D, update3D, sceneReady, scene3d,
 *            setCameraMode, resetCamera, resetMapView,
 *            setTopoData, setDebugCam, setPerfTier, closeNPCDetail } from './scene3d_v2.js';
 *
 * 砍掉的部件（v2 第一版不做）：
 *   - 红绿灯 / ETC 门架 / 路面箭头 / 停止线 / 路口补丁
 *   - 护栏 / 路灯 / 高架桥墩 / 桥栏矮墙
 *   - 雨天 / 水面 / 城市天际线 / 天空盒 shader
 *   - LiDAR 点云 / NPC 详情面板
 *   - PMREMGenerator 环境贴图 / SMAA 后处理
 *   - Driver / Front 相机模式
 *   - 车轮滚动 / 车身侧倾 / 车灯动画
 *
 * 保留的核心：
 *   - Three.js 场景 / 相机 / 渲染器 / WebGL context restore
 *   - 道路网络构建（路面 + 车道线，合并 1 draw call）
 *   - ego 车（复用 models.js buildEgoCar）
 *   - 障碍物/NPC（复用 models.js buildObstacleGroup，mesh 池复用）
 *   - 基础光照（Ambient + Hemi + Directional 阴影）
 *   - 地面草地（按路网 bbox 铺满）
 *   - Chase / Map / Orbit 三模式相机
 *
 * 模块结构：
 *   - SceneStore.js    集中式 state（替代 v1 的 51 个模块级 let）
 *   - RoadBuilder.js   道路网络构建
 *   - EgoView.js       ego 车构建 + 每帧更新
 *   - EntityView.js    障碍物/NPC 渲染（mesh 池）
 *   - Environment.js   地面 + 光照
 *   - CameraRig.js     相机控制（复用 v1 CameraController）
 */
const THREE = window.THREE;

import { store } from './scene3d_v2/SceneStore.js';
import { buildRoadNetwork } from './scene3d_v2/RoadBuilder.js';
import { buildEgo, updateEgo } from './scene3d_v2/EgoView.js';
import { updateEntities, disposePool } from './scene3d_v2/EntityView.js';
import { buildEnvironment, fitGroundToBBox, updateSunFollowEgo, applyLightingMode } from './scene3d_v2/Environment.js';
import * as CameraRig from './scene3d_v2/CameraRig.js';
import { initDeadReckon, tickDeadReckon, _dr } from './deadreckon.js';
import { safeCall, reportDiag } from './utils.js';

// ── 对外导出（保持 app.js API 兼容）─────────────────────────
// scene3d / sceneReady 在 v1 是 export let（live binding），app.js 把 scene3d
// 传给 _auditSceneMaterials。v2 用 export let 同步 store 值保持兼容。
export let scene3d = null;
export let sceneReady = false;
function _syncExports() {
  scene3d = store.getScene3D();
  sceneReady = store.isReady();
}

export function setTopoData(d) { store.setTopoData(d); }
export function setDebugCam(v) { store.setDebugCam(v); }
export function setCameraMode(mode) { CameraRig.setMode(mode); }
export function resetCamera() { CameraRig.resetCamera(); }
export function resetMapView() { CameraRig.resetMapView(); }
export function setPerfTier(_tier) { /* v2 暂不做性能分级 */ }
export function closeNPCDetail() { /* v2 无 NPC 面板 */ }

// v1 export 了 _renderFrame / _applyRoadCurve 给 debug 用，v2 用空函数占位
// 避免 app.js 或控制台调试代码报 undefined。
export function _renderFrame() { /* v2 由内部 RAF 循环驱动，外部无需调用 */ }
export function _applyRoadCurve(_road) { /* v2 不支持单段弯道路径，只认 road_network */ }

// ── init3DScene ──────────────────────────────────────────────
export function init3DScene() {
  var el = document.getElementById('scene3d');
  if (!el) return;
  if (typeof THREE === 'undefined' || !THREE.WebGLRenderer) {
    _show3DError('WebGL not available');
    return;
  }
  var msgEl = document.getElementById('scene3d-msg');
  if (msgEl) msgEl.style.display = 'none';

  try {
    var w = el.clientWidth || (el.parentElement && el.parentElement.clientWidth) || 800;
    var h = el.clientHeight || 400;

    // 清理旧 renderer（webglcontextrestored 重入路径）
    var oldRenderer = store.getRenderer();
    if (oldRenderer) {
      try { oldRenderer.dispose(); } catch (e) { /* 旧 context 已死 */ }
      if (oldRenderer.domElement && oldRenderer.domElement.parentNode) {
        oldRenderer.domElement.parentNode.removeChild(oldRenderer.domElement);
      }
    }
    Array.from(el.querySelectorAll('canvas')).forEach(function(c) {
      if (c.parentNode) c.parentNode.removeChild(c);
    });

    // Scene
    var scene = new THREE.Scene();
    scene.userData._v2store = store;  // RoadBuilder 用 scene.userData._v2store 访问 store
    store.reset();  // 先清空旧引用（不清 scene，下面单独设）
    store.setScene3D(scene);

    // Camera
    var camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 3000);
    camera.position.set(-6, 3, 0);
    camera.lookAt(10, 1, 0);
    store.setCamera(camera);

    // Renderer
    var renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    el.appendChild(renderer.domElement);
    store.setRenderer(renderer);

    // 环境（地面 + 光照）
    var env = buildEnvironment(scene);
    store.setGround(env.ground);
    store.setAmbientLight(env.ambient);
    store.setHemiLight(env.hemi);
    store.setSunLight(env.sun);

    // ego 车
    var ego = buildEgo(scene);
    store.setEgo(ego);

    // 实体 group
    var entityGroup = new THREE.Group();
    entityGroup.name = 'entities';
    scene.add(entityGroup);
    store.setEntityGroup(entityGroup);

    // 相机交互（鼠标拖拽 / 滚轮）
    CameraRig.initInteraction(renderer.domElement);

    // WebGL context loss/restore
    renderer.domElement.addEventListener('webglcontextlost', function(e) {
      e.preventDefault();
      reportDiag('scene3d', 'WebGL context lost');
      store.setReady(false);
      _syncExports();
    }, false);
    renderer.domElement.addEventListener('webglcontextrestored', function() {
      reportDiag('scene3d', 'WebGL context restored — rebuilding scene');
      safeCall('scene3d.restore', function() { init3DScene(); });
    }, false);

    initDeadReckon();
    store.setReady(true);
    _syncExports();  // 同步 scene3d/sceneReady 到 export let

    // 启动渲染循环
    _startRenderLoop();
  } catch (err) {
    _show3DError('init3DScene failed: ' + (err && err.message || err));
    console.error('[scene3d_v2.init]', err);
  }
}

// ── resize3D ─────────────────────────────────────────────────
export function resize3D() {
  var el = document.getElementById('scene3d');
  var renderer = store.getRenderer();
  var camera = store.getCamera();
  if (!el || !renderer || !camera) return;
  var w = el.clientWidth || (el.parentElement && el.parentElement.clientWidth) || 800;
  var h = el.clientHeight || 400;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

// ── update3D（每帧由 app.js 调用，读 _topoData.metrics）─────
export function update3D() {
  // v2 的实际渲染由 _renderLoop 驱动（requestAnimationFrame）；
  // update3D 只负责把 _topoData 里的 scene/frame 数据应用到 store。
  // 这样 deadReckon + 道路重建 + 实体更新 都在 RAF 循环里跑，
  // 与 app.js 的 update3D 调用频率解耦（app.js 可能 60Hz 或更低）。
  var scn = store.getScene();
  if (scn && scn.road_network && scn.road_network.edges) {
    buildRoadNetwork(scn.road_network.edges, store.getScene3D());
    // 路网构建后调整地面覆盖整个 bbox
    var bbox = store.getRoadBBox();
    if (bbox) fitGroundToBBox(store.getGround(), bbox);
  }
}

// ── 渲染循环 ─────────────────────────────────────────────────
let _rafId = 0;
function _startRenderLoop() {
  if (_rafId) cancelAnimationFrame(_rafId);
  _rafId = requestAnimationFrame(_renderFrame);
}

function _renderFrame() {
  _rafId = requestAnimationFrame(_renderFrame);
  if (!store.isReady()) return;

  var scene = store.getScene3D();
  var camera = store.getCamera();
  var renderer = store.getRenderer();
  var ego = store.getEgo();
  if (!scene || !camera || !renderer || !ego) return;

  // Dead reckoning：平滑 ego 位置/朝向
  tickDeadReckon();
  var sx = _dr.smoothX, sz = _dr.smoothZ;
  var sheading = _dr.smoothHeading;

  // 更新 ego
  updateEgo(ego, sx, sz, sheading, store.getRoadCurves());

  // 更新 entities
  var scn = store.getScene();
  if (scn && scn.entities) {
    updateEntities(scn.entities, store.getEntityGroup(), store.getRoadCurves());
  }

  // 光照模式切换（day/night）
  var lightingMode = 'day';
  if (scn && scn.lighting) lightingMode = scn.lighting;
  // 缓存上次模式避免每帧重设
  if (lightingMode !== _lastLightingMode) {
    applyLightingMode(lightingMode, store.getAmbientLight(), store.getHemiLight(),
                      store.getSunLight(), scene);
    _lastLightingMode = lightingMode;
  }

  // 阴影光源跟随 ego
  updateSunFollowEgo(store.getSunLight(), sx, sz);

  // 相机更新
  CameraRig.updateCamera(camera, sx, sz, sheading, store.getRoadBBox());

  // 渲染
  renderer.render(scene, camera);
}

let _lastLightingMode = 'day';

// ── 错误显示 ─────────────────────────────────────────────────
function _show3DError(msg) {
  var el = document.getElementById('scene3d');
  if (!el) return;
  var div = document.createElement('div');
  div.style.cssText = 'position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);' +
                      'background:#fee;color:#900;padding:12px 16px;border-radius:4px;' +
                      'font-family:monospace;font-size:13px;max-width:80%;text-align:center;';
  div.textContent = msg;
  el.appendChild(div);
  reportDiag('scene3d_v2', msg);
}
