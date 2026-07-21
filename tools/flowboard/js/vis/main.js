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
import { tickDeadReckon, _dr } from '../deadreckon.js';

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

      // ── 死推算：每帧 advance 平滑位置，弥补 SSE 5Hz 离散数据 ──
      tickDeadReckon();

      // 用死推算平滑值覆盖 ego 位置（原始数据保留在 store.ego._simX 等字段）
      if (store.ego && _dr.init) {
        store.ego.x = _dr.smoothX;
        store.ego.y = _dr.smoothZ;
        store.ego.heading = _dr.smoothHeading;
        store.ego.speed = _dr.smoothSpeed;
      }

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
  let _wireframeMode = false;
  let _fpsTimes = [];

  window.__vis = {
    get scene() { return _scene; },
    get renderer() { return _renderer; },
    get camera() { return _cameraRig ? _cameraRig.camera : null; },
    get director() { return _director; },
    get ready() { return _ready; },
    get frameCount() { return _frameCount; },
    get lastError() { return _lastRenderErr; },
    get wireframe() { return _wireframeMode; },
    /** 获取渲染性能统计：calls(Draw Call), triangles, geometries, textures */
    get perf() {
      const info = getRendererInfo(_renderer);
      if (!info) return null;
      const now = performance.now();
      _fpsTimes.push(now);
      _fpsTimes = _fpsTimes.filter(t => now - t < 1000);
      return { ...info, fps: _fpsTimes.length };
    },
    /** 获取当前场景状态 */
    get store() {
      return _director ? _director.getStore() : null;
    },
    /** 获取道路组 */
    get roadGroup() {
      return _director ? _director.getRoadView().getRoadGroup() : null;
    },
    /** 获取相机位置和朝向 */
    get cameraInfo() {
      const cam = _cameraRig ? _cameraRig.camera : null;
      if (!cam) return null;
      return {
        pos: { x: cam.position.x, y: cam.position.y, z: cam.position.z },
        fov: cam.fov,
        near: cam.near,
        far: cam.far
      };
    },
    /** 获取实体列表 */
    get entities() {
      const store = _director ? _director.getStore() : null;
      return store ? store.entities : [];
    },
    /** 获取 ego 状态 */
    get ego() {
      const store = _director ? _director.getStore() : null;
      return store ? store.ego : null;
    },
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
    },
    /** 切换 wireframe 模式 */
    toggleWireframe() {
      _wireframeMode = !_wireframeMode;
      _scene.traverse(o => {
        if (o.material && !o.material.isSpriteMaterial) {
          o.material.wireframe = _wireframeMode;
        }
      });
      return 'wireframe ' + (_wireframeMode ? 'ON' : 'OFF');
    },
    /** 打印场景层级 */
    printHierarchy(root, depth = 0) {
      const obj = root || _scene;
      if (!obj) return;
      const prefix = '  '.repeat(depth);
      const type = obj.type || 'Object3D';
      const name = obj.name || '(unnamed)';
      console.log(prefix + type + (name ? ' "' + name + '"' : ''));
      if (obj.children) {
        obj.children.forEach(c => this.printHierarchy(c, depth + 1));
      }
    },
    /** 显示/隐藏道路 */
    toggleRoad(show) {
      const rg = this.roadGroup;
      if (!rg) return 'no road group';
      rg.visible = show !== false;
      return 'road ' + (rg.visible ? 'SHOWN' : 'HIDDEN');
    },
    /** 显示/隐藏所有 NPC */
    toggleNPCs(show) {
      const store = this.store;
      if (!store) return 'no store';
      const vv = _director ? _director.getVehicleView() : null;
      if (vv) {
        const npcPool = vv.getNPCPool ? vv.getNPCPool() : null;
        if (npcPool) {
          npcPool.forEach(npc => { npc.visible = show !== false; });
          return 'NPCs ' + (show !== false ? 'SHOWN' : 'HIDDEN');
        }
      }
      return 'no NPC pool';
    },
    /** 重置相机位置 */
    resetCamera() {
      if (_cameraRig) _cameraRig.reset(this.roadGroup);
      return 'camera reset';
    },
    /** 设置相机模式 */
    setCameraMode(mode) {
      if (_cameraRig) _cameraRig.setMode(mode);
      return 'camera mode: ' + mode;
    },
    /** 显示当前渲染器配置 */
    get rendererConfig() {
      if (!_renderer) return null;
      return {
        pixelRatio: _renderer.getPixelRatio(),
        shadowMapEnabled: _renderer.shadowMap.enabled,
        shadowMapType: _renderer.shadowMap.type === THREE.PCFShadowMap ? 'PCF' : 'basic',
        antialias: true,
        toneMapping: 'ACESFilmic',
        outputEncoding: 'sRGB'
      };
    },
    /** 强制重渲染 */
    forceResize() { resize3D(); },
    /** 清除所有错误 */
    clearErrors() { _lastRenderErr = null; }
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