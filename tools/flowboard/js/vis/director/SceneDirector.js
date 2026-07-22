/**
 * SceneDirector.js — 场景导演
 * 接 scene/frame JSON → 写 SceneStore → 驱动各 View 更新。
 * diff 检测：road_network hash 变了才重建路。
 *
 * 数据校验（Step 1 重构）：对 frame / road_network / ego / entities 做轻量
 * schema 校验。校验失败不抛错（保持向后兼容），只 console.warn 一次，避免
 * 后端发坏 JSON 时前端静默继续渲染、用户看不到原因。
 */

import { createSceneStore, roadNetworkHash } from '../store/SceneStore.js';
import { createRoadView } from '../view/RoadView.js';
import { createGroundView } from '../view/GroundView.js';
import { createVehicleView } from '../view/VehicleView.js';
import { createConnectorView } from '../view/ConnectorView.js';
import { createTrafficLightView } from '../view/TrafficLightView.js';
import { createETCGateView } from '../view/ETCGateView.js';
import { createViaductView } from '../view/ViaductView.js';
import { createStreetlightView } from '../view/StreetlightView.js';
import { createBarrierView } from '../view/BarrierView.js';
import { tickDeadReckon, _dr } from '../core/DeadReckon.js';
import * as ViewRegistry from '../core/ViewRegistry.js';
import { createLayer } from '../core/Layer.js';
// Step 5 重构：纯函数 validateFrame 抽到 FrameValidator.js，
// 零 THREE 依赖，便于 tests/vis_director_validation.test.mjs 直接 import。
import { validateFrame } from './FrameValidator.js';

// 向后兼容：原调用方从 SceneDirector import validateFrame。
export { validateFrame };

/* 架构升级：View 插件注册 + 错误隔离（Qt 对象树 + 单向依赖思路）。
 * 所有 View 工厂在模块加载时注册一次，createSceneDirector 实例化时
 * 走 ViewRegistry.instantiateAll(scene)，update() 里所有 view 方法调用
 * 走 ViewRegistry.safeCall —— 单个 View 抛错只 log + 跳过，兄弟继续渲染。
 * 对应"一个模块坏了整个 3D 就坏了"的痛点。 */
ViewRegistry.register('road',         createRoadView);
ViewRegistry.register('ground',       createGroundView);
ViewRegistry.register('vehicle',      createVehicleView);
ViewRegistry.register('connector',    createConnectorView);
ViewRegistry.register('trafficLight', createTrafficLightView);
ViewRegistry.register('etcGate',      createETCGateView);
ViewRegistry.register('viaduct',     createViaductView);
ViewRegistry.register('streetlight',  createStreetlightView);
ViewRegistry.register('barrier',      createBarrierView);

export function createSceneDirector(scene) {
  const store = createSceneStore();
  /* 实例化所有已注册 View。instantiateAll 内部已有 try/catch，
   * 工厂本身抛错不会炸整个 director。 */
  ViewRegistry.instantiateAll(scene);
  const roadView         = ViewRegistry.get('road');
  const groundView       = ViewRegistry.get('ground');
  const vehicleView      = ViewRegistry.get('vehicle');
  const connectorView    = ViewRegistry.get('connector');
  const trafficLightView = ViewRegistry.get('trafficLight');
  const etcGateView      = ViewRegistry.get('etcGate');
  const viaductView      = ViewRegistry.get('viaduct');
  const streetlightView  = ViewRegistry.get('streetlight');
  const barrierView      = ViewRegistry.get('barrier');
  let lastRoadHash = '';

  /* ── Layer 树（Qt 对象树 + 单向依赖）──────────────────────────
   * 4 个语义层，每帧递归 update。view 抛错只 log + 跳过，不传染兄弟。
   * 静态布局 View（road/ground/connector/streetlight/barrier/viaduct）
   *   仍走 ViewRegistry.safeCall 的 build 路径，不挂 Layer 树（无需每帧调）。
   * 动态更新 View（vehicle/trafficLight/etcGate）挂到对应 Layer，每帧
   *   rootLayer.update(store, now) 递归调用 —— 取代 main.js 单独调
   *   vehicle.update + SceneDirector 末尾 safeCall trafficLight/etcGate。
   *
   * 层级：
   *   root
   *   ├── agent    (vehicle)             — 智能体层
   *   └── infra    (trafficLight, etcGate) — 路侧设施层
   * 后续 RoadLayer/EnvLayer 可继续挂（Step C）。 */
  const rootLayer  = createLayer('root', scene);
  const agentLayer = rootLayer.addChild(createLayer('agent', scene));
  const infraLayer = rootLayer.addChild(createLayer('infra', scene));
  if (vehicleView)      agentLayer.addView(vehicleView);
  if (trafficLightView) infraLayer.addView(trafficLightView);
  if (etcGateView)      infraLayer.addView(etcGateView);

  /* 已 warn 过的字段 key 集合，避免 20Hz × N 字段刷屏。
   * key 形如 'ego.x' / 'entities[3].heading' / 'road_network.edges' */
  const _warned = new Set();
  function _warnOnce(key, msg) {
    if (_warned.has(key)) return;
    _warned.add(key);
    console.warn('[SceneDirector] ' + msg + ' (后续同类问题不再重复打印)');
  }

  /* 重置 warn 集合（测试 / 切场景时调用，避免漏掉新场景的问题） */
  function resetWarnings() { _warned.clear(); }

  const VIADUCT_VIS_LENGTH = 500;

  function init() {
    ViewRegistry.safeCall('ground', 'build', 20000);
    const defaultRN = {
      edges: [{
        id: 0,
        type: 'viaduct_highway',
        lanes: 4,
        lane_width: 3.5,
        length: 500,
        nodes: [[0, 0, 0], [500, 0, 0]]
      }]
    };
    update({ metrics: { scene: { road_network: defaultRN, ego: { x: 50, y: 0, heading: 0, speed: 0, z: 7 }, entities: [] } } });
  }

  function update(topoData) {
    /* Step 5 重构：校验逻辑下沉到 validateFrame 纯函数，
     * update() 只负责把校验结果 emit 到 _warnOnce + 实际构建。 */
    const v = validateFrame(topoData);
    if (!v.ok) {
      for (const w of v.warnings) _warnOnce(w.key, w.msg);
      return;
    }
    for (const w of v.warnings) _warnOnce(w.key, w.msg);

    const { frame, rn, skipRoad, skipEgo, skipEntities } = v;

    if (rn && !skipRoad) {
      const hash = roadNetworkHash(rn);
      if (hash !== lastRoadHash) {
        const edgesArr = Array.isArray(rn.edges) ? rn.edges : [];
        const urlViaduct = typeof window !== 'undefined' &&
                           new URLSearchParams(window.location.search).get('viaduct') === '1';
        const isViaduct = urlViaduct || edgesArr.some(e =>
          e && (e.name === 'viaduct_highway' || e.type === 'viaduct_highway')
        );

        if (isViaduct) {
          const edge0 = edgesArr[0] || {};
          const laneCount = edge0.lanes || 4;
          const laneWidth = edge0.lane_width || 3.5;
          const actualLength = edge0.length || edge0.length_m || VIADUCT_VIS_LENGTH;
          ViewRegistry.safeCall('ground', 'build', 20000);
          ViewRegistry.safeCall('viaduct', 'build', { laneCount, laneWidth, length: actualLength, withEnvironment: true });
          ViewRegistry.safeCall('road', 'build', { edges: [] });
          /* Step 4：高架场景路灯/护栏由 ViaductView 内置，StreetlightView /
           * BarrierView 在普通 edge 上不放置（内部跳过 viaduct_highway），
           * 此处显式 clear 一遍保证切换场景时不残留。 */
          ViewRegistry.safeCall('streetlight', 'build', { edges: [] });
          ViewRegistry.safeCall('barrier', 'build', { edges: [] });
          store.isViaduct = true;
        } else {
          ViewRegistry.safeCall('road', 'build', rn);
          ViewRegistry.safeCall('ground', 'build', 20000);
          /* Step 4：普通道路场景，路灯 + 护栏从 roadNetwork 自动布局。 */
          ViewRegistry.safeCall('streetlight', 'build', rn);
          ViewRegistry.safeCall('barrier', 'build', rn);
          store.isViaduct = false;
        }

        ViewRegistry.safeCall('connector', 'build', rn);
        lastRoadHash = hash;
        store.roadNetwork = rn;
        store.roadHash = hash;
      }
    }

    if (frame.ego !== undefined && !skipEgo) {
      const e = frame.ego;
      const newX = e.x || 0;
      const viaductOffset = store.isViaduct ? 7.0 : 0;
      const simX = newX;
      const visualX = simX % VIADUCT_VIS_LENGTH;
      const wrapOffset = simX - visualX;

      store.ego = {
        x: simX,
        y: e.y || 0,
        z: viaductOffset,
        heading: e.heading != null ? e.heading : (e.h || 0),
        speed: e.speed != null ? e.speed : (e.spd || 0),
        steer: e.steer || 0,
        brake: e.brake || 0,
        throttle: e.throttle || 0,
        lights: e.lights || 0,
        vx: e.vx || 0,
        vy: e.vy || 0,
        length: e.length != null ? e.length : (e.len || 4.6),
        width: e.width != null ? e.width : (e.wid || 2.0),
        ai_state: e.ai_state || e.ai || '',
        _simX: newX,
        _visualX: visualX,
        _wrapOffset: wrapOffset,
      };

      if (store.isViaduct) {
        ViewRegistry.safeCall('viaduct', 'followEgo', wrapOffset + VIADUCT_VIS_LENGTH / 2);
      }
    }

    if (frame.entities !== undefined && !skipEntities) {
      const viaductOffset = store.isViaduct ? 7.0 : 0;
      store.entities = frame.entities.filter(e => e && e.type !== 'ego').map((e) => {
        /* 校验 8 由本函数上面完成，这里只构建。type 缺失的 entity 仍会被
         * 各 View 静默丢弃（与原行为一致）。 */
        return {
          id: e.id,
          type: e.type,
          x: e.x || 0,
          y: e.y || 0,
          z: viaductOffset,
          heading: e.heading != null ? e.heading : (e.h || 0),
          speed: e.speed != null ? e.speed : (e.spd || 0),
          steer: e.steer || 0,
          length: e.length != null ? e.length : (e.len || 4.6),
          width: e.width != null ? e.width : (e.wid || 2.0),
          ai_state: e.ai_state || e.ai || '',
          lights: e.lights || 0,
          vx: e.vx || 0,
          vy: e.vy || 0,
          throttle: e.throttle || 0,
          brake: e.brake || 0,
          state: e.state || '',
          progress: e.progress || 0,
          remain_s: e.remain_s || 0,
          parked: e.parked || false,
        };
      });
    }

    /* 动态 View 不在 update() 里调，统一由 tickAnimation(now) 每帧走 Layer 树
     * 递归 update（agent/infra 层）。update() 只负责写 store 数据 + 触发
     * 静态 View 的 build（roadHash 变了才重建）。 */
  }

  /* ── tickAnimation(now) — 每帧推进死推算 + Layer 树递归 update ──
   *
   * Step 2 重构：原 main.js 行 104-112 直接 import deadreckon + 覆盖
   * store.ego.x/y/heading/speed，违反"Director → Store → View 单向流"。
   * 现在此逻辑下沉到 SceneDirector。
   *
   * 架构升级：每帧 view update 走 rootLayer.update(store, now) 递归 ——
   * 取代 main.js 单独调 _director.getVehicleView().update(store, now) +
   * SceneDirector 末尾逐个 safeCall trafficLight/etcGate。
   * 一个 view 抛错只 log + 跳过，不传染兄弟（Layer 错误隔离）。
   *
   * 数据流：
   *   app.js sync2DTarget (SSE tick) → updateDeadReckon(x,z,speed,heading) → _dr
   *   main.js 渲染帧 → director.tickAnimation(now)
   *     → tickDeadReckon() 推进 _dr.smooth*
   *     → 把 smooth* 写入 store.ego（覆盖 x/y/heading/speed，保留 _simX 等原始字段）
   *     → rootLayer.update(store, now) 递归 update agent/infra 层所有 view
   *
   * 2D（scene2d.js）仍直接调 tickDeadReckon + 读 _dr.smooth*，与 3D 共享同一
   * _dr 全局单例，保证 3D/2D 视图 ego 位置完全同步。
   *
   * @param {number} now 当前 performance.now() 毫秒（传给 view.update 需要 simTime）
   */
  function tickAnimation(now) {
    tickDeadReckon();
    if (store.ego && _dr.init) {
      store.ego.x = _dr.smoothX;
      store.ego.y = _dr.smoothZ;
      store.ego.heading = _dr.smoothHeading;
      store.ego.speed = _dr.smoothSpeed;
    }
    /* Layer 树递归 update：agent(vehicle) + infra(trafficLight, etcGate) */
    rootLayer.update(store, now);
  }

  function getStore() { return store; }
  function getRoadView() { return roadView; }
  function getGroundView() { return groundView; }
  function getVehicleView() { return vehicleView; }
  function getConnectorView() { return connectorView; }
  function getTrafficLightView() { return trafficLightView; }
  function getETCGateView() { return etcGateView; }
  function getViaductView() { return viaductView; }
  function getStreetlightView() { return streetlightView; }
  function getBarrierView() { return barrierView; }

  return { init, update, tickAnimation, getStore, getRoadView, getGroundView, getVehicleView,
           getConnectorView, getTrafficLightView, getETCGateView, getViaductView,
           getStreetlightView, getBarrierView, resetWarnings };
}
