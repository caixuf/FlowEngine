/**
 * main.js — vis/ 架构入口
 * 组装所有模块，导出 app.js 期望的接口。
 * 取代旧 scene3d_v2.js（3000 行 God Object）。
 *
 * app.js import:
 *   import { init3DScene, resize3D, update3D, sceneReady, scene3d,
 *            setTopoData, setCameraMode, resetCamera, resetMapView,
 *            closeNPCDetail, setPerfTier } from './vis/main.js';
 */

import { createRenderer, createComposer, renderFrame, resize, getRendererInfo, resetRendererInfo } from './core/Renderer.js';
import { createCameraRig } from './core/CameraRig.js';
import { createLighting, updateSunShadow } from './core/Lighting.js';
import { createSkyEnv } from './core/SkyEnv.js';
import { createSceneDirector } from './director/SceneDirector.js';
import { clearCache } from './core/AssetFactory.js';
import { initModels } from './view/VehicleView.js';

// ── 模块级状态（只此一处，取代旧架构 51 个 let）──
let _scene = null;
let _renderer = null;
let _composer = null;
let _cameraRig = null;
let _lights = null;
let _skyEnv = null;
let _director = null;
let _ready = false;
let _lastTopoData = null;

/** 暴露 scene 对象（app.js 直接 import scene3d）*/
export let scene3d = null;

// ── 导出给 app.js 的接口 ──

/** 初始化 3D 场景 */
export function init3DScene(canvas) {
  // app.js 调 init3DScene() 不传参数，自己找/建 canvas
  if (!canvas) {
    canvas = document.getElementById('scene3d-canvas');
    if (!canvas) {
      // 找 #scene3d 容器 div，在里面建 canvas
      const container = document.getElementById('scene3d') || document.body;
      canvas = document.createElement('canvas');
      canvas.id = 'scene3d-canvas';
      canvas.style.width = '100%';
      canvas.style.height = '100%';
      canvas.style.display = 'block';
      container.appendChild(canvas);
    }
  }

  _scene = new THREE.Scene();
  scene3d = _scene;  // 暴露给 app.js
  _renderer = createRenderer(canvas);

  _cameraRig = createCameraRig(canvas);

  _lights = createLighting(_scene);
  _skyEnv = createSkyEnv(_scene);
  _director = createSceneDirector(_scene);
  _director.init();

  _composer = createComposer(_renderer, _scene, _cameraRig.camera);

  _ready = true;
  _startRenderLoop();

  // 异步预加载 gltf 车辆模型（SU7/sedan/suv/truck）
  // 加载完成前用程序化 fallback，完成后新车自动用 gltf
  initModels();

  // 立即设置初始尺寸（否则 renderer 默认 300x150，canvas 看起来空白）
  // 用 ResizeObserver 监听容器变化，比 setTimeout 更可靠
  _initResizeObserver(canvas);
  resize3D();

  return _scene;
}

/** 监听 canvas 容器尺寸变化，自动 resize */
function _initResizeObserver(canvas) {
  const container = canvas.parentElement || canvas;
  if (typeof ResizeObserver === 'undefined') return;
  const ro = new ResizeObserver(() => { resize3D(); });
  ro.observe(container);
}

/** 渲染循环 */
let _frameCount = 0;
let _lastRenderErr = null;
function _startRenderLoop() {
  function loop() {
    requestAnimationFrame(loop);
    if (!_ready) return;

    try {
      const now = performance.now();
      const store = _director.getStore();
      const roadGroup = _director.getRoadView().getRoadGroup();

      // 更新车辆（含车灯闪烁需要 simTime）
      _director.getVehicleView().update(store, now);

      // 太阳阴影相机跟随 ego（小 frustum 罩住主车周围）
      updateSunShadow(_lights, store.ego);

      _cameraRig.update(store.ego, roadGroup, now);

      renderFrame(_renderer, _composer, _scene, _cameraRig.camera);
      _frameCount++;
    } catch (err) {
      console.error('[vis] render loop error:', err);
      _lastRenderErr = err;
    }
  }
  loop();
}

// 调试接口（挂到 window 方便控制台诊断）
if (typeof window !== 'undefined') {
  window.__vis = {
    get scene() { return _scene; },
    get renderer() { return _renderer; },
    get camera() { return _cameraRig ? _cameraRig.camera : null; },
    get director() { return _director; },
    get ready() { return _ready; },
    get frameCount() { return _frameCount; },
    get lastError() { return _lastRenderErr; },
    /** 获取渲染性能统计：calls(Draw Call), triangles, geometries, textures */
    get perf() { return getRendererInfo(_renderer); },
    /** 重置渲染统计 */
    resetPerf() { resetRendererInfo(_renderer); },
    /** 手动渲染一帧（调试用） */
    debugRender() {
      if (!_renderer || !_scene) return 'no renderer/scene';
      try {
        _renderer.render(_scene, _cameraRig.camera);
        return 'rendered ok, frame=' + _frameCount;
      } catch (e) {
        return 'render failed: ' + e.message;
      }
    }
  };
}

/** 调整大小。不传参数时自动从 canvas 父容器读取尺寸。 */
export function resize3D(width, height) {
  if (!_renderer || !_cameraRig) return;
  if (width === undefined || height === undefined) {
    const canvas = _renderer.domElement;
    const container = canvas ? canvas.parentElement : null;
    if (container) {
      const rect = container.getBoundingClientRect();
      width = rect.width || container.clientWidth || window.innerWidth;
      height = rect.height || container.clientHeight || window.innerHeight;
    } else {
      width = window.innerWidth;
      height = window.innerHeight;
    }
  }
  if (width <= 0 || height <= 0) return;  // 容器还没渲染出来
  resize(_renderer, _composer, _cameraRig.camera, width, height);
}

/** 每帧更新（app.js 调用） */
export function update3D(topoData) {
  if (!_director) return;
  _lastTopoData = topoData;
  _director.update(topoData);
}

/** 设置 topology 数据（app.js setTopoData fan-out） */
export function setTopoData(data) {
  update3D(data);
}

/** 场景是否就绪 */
export function sceneReady() {
  return _ready;
}

// scene3d 已在模块顶部 export let 声明，init3DScene 里赋值

/** 切换视角 */
export function setCameraMode(mode) {
  if (_cameraRig) _cameraRig.setMode(mode);
}

/** 重置相机 */
export function resetCamera() {
  if (_cameraRig && _director) {
    _cameraRig.reset(_director.getRoadView().getRoadGroup());
  }
}

/** 重置 Map 视角 */
export function resetMapView() {
  resetCamera();
}

/** 设置性能档位 */
export function setPerfTier(tier) {
  if (!_director) return;
  _director.getStore().perfTier = tier;
  // 性能档位影响：low=关阴影, medium=降像素比, high=全开
  if (_renderer) {
    if (tier === 'low') {
      _renderer.shadowMap.enabled = false;
      _renderer.setPixelRatio(1);
    } else if (tier === 'medium') {
      _renderer.shadowMap.enabled = true;
      _renderer.setPixelRatio(1.5);
    } else {
      _renderer.shadowMap.enabled = true;
      _renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    }
  }
}

/** 调试相机（占位，Phase 3 实现） */
export function setDebugCam(mode) {
  setCameraMode(mode);
}

/** 关闭 NPC 详情面板（占位，Phase 3 实现） */
export function closeNPCDetail() {
  // Phase 3 VehicleView 实现 NPC 详情面板后补全
}
