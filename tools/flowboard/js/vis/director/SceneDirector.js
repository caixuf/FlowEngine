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
import { createTreeView } from '../view/TreeView.js';
import {
  tickDeadReckon, _dr,
  updateEntityDeadReckon, tickEntityDeadReckon, getEntitySmooth,
  pruneEntities, resetEntities,
} from '../core/DeadReckon.js';
import * as ViewRegistry from '../core/ViewRegistry.js';
import { createLayer } from '../core/Layer.js';
// Step 5 重构：纯函数 validateFrame 抽到 FrameValidator.js，
// 零 THREE 依赖，便于 tests/vis_director_validation.test.mjs 直接 import。
import { validateFrame } from './FrameValidator.js';
import { LANE_WIDTH, EDGE_TYPE, VIADUCT_VIS_LENGTH } from '../core/Constants.js';

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
ViewRegistry.register('tree',         createTreeView);

export function createSceneDirector(scene) {
  const store = createSceneStore();
  /* 实例化所有已注册 View。instantiateAll 内部已有 try/catch，
   * 工厂本身抛错不会炸整个 director。 */
  ViewRegistry.instantiateAll(scene);
  let lastRoadHash = '';

  /* ── Layer 树（Qt 对象树 + 单向依赖）──────────────────────────
   * 4 个语义层，构成完整对象树。所有 View（静态 + 动态）都挂到 Layer，
   * dispose 时递归清理 geometry/material，不再需要手动管理生命周期。
   *
   * 层级：
   *   root
   *   ├── env     (ground, viaduct)              — 环境层
   *   ├── road     (road, streetlight, barrier, connector)  — 道路层
   *   ├── agent    (vehicle)                      — 智能体层
   *   └── infra    (trafficLight, etcGate)        — 路侧设施层
   *
   * 静态布局 View（road/ground/...）的 build 仍由 update() 内部条件性调用
   * （高架 vs 普通道路分支不同），不走 Layer.build（Layer 树只管递归 update
   * 和 dispose）。
   * 动态 View（vehicle/trafficLight/etcGate）每帧由 tickAnimation →
   * rootLayer.update(store, now) 递归调用，单个抛错只 log + 跳过。 */
  const rootLayer  = createLayer('root', scene);
  const envLayer   = rootLayer.addChild(createLayer('env', scene));
  const roadLayer  = rootLayer.addChild(createLayer('road', scene));
  const agentLayer = rootLayer.addChild(createLayer('agent', scene));
  const infraLayer = rootLayer.addChild(createLayer('infra', scene));
  /* View 实例从 ViewRegistry 取（register 后 instantiateAll 已建好），
   * 不再保留 9 个顶层 const —— ViewRegistry 是单一事实来源，避免双份引用。 */
  for (const [layerName, viewNames] of [
    ['env',   ['ground', 'viaduct']],
    ['road',  ['road', 'streetlight', 'barrier', 'connector', 'tree']],
    ['agent', ['vehicle']],
    ['infra', ['trafficLight', 'etcGate']],
  ]) {
    const layer = rootLayer.findDescendant(layerName);
    for (const vn of viewNames) {
      const v = ViewRegistry.get(vn);
      if (v) layer.addView(v);
    }
  }

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

  function init() {
    ViewRegistry.safeCall('ground', 'build', 20000);
    const defaultRN = {
      edges: [{
        id: 0,
        type: 'highway',
        lanes: 4,
        lane_width: LANE_WIDTH,
        length: 10000,
        nodes: [[0, 0, 0], [10000, 0, 0]]
      }]
    };
    update({ metrics: { scene: { road_network: defaultRN, ego: { x: 50, y: 0, heading: 0, speed: 0, z: 0 }, entities: [] } } });
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
          e && (e.name === EDGE_TYPE.VIADUCT_HIGHWAY || e.type === EDGE_TYPE.VIADUCT_HIGHWAY)
        );

        ViewRegistry.safeCall('road', 'build', rn);
        ViewRegistry.safeCall('ground', 'build', 20000);

        if (isViaduct) {
          const edge0 = edgesArr[0] || {};
          const laneCount = edge0.lanes || 4;
          const laneWidth = edge0.lane_width || 3.5;
          const actualLength = edge0.length || edge0.length_m || VIADUCT_VIS_LENGTH;
          ViewRegistry.safeCall('viaduct', 'build', { laneCount, laneWidth, length: actualLength, withEnvironment: true });
          ViewRegistry.safeCall('streetlight', 'build', { edges: [] });
          ViewRegistry.safeCall('barrier', 'build', { edges: [] });
          ViewRegistry.safeCall('tree', 'build', { edges: [] });
          store.isViaduct = true;
          store.viaductVisLength = actualLength;
        } else {
          ViewRegistry.safeCall('streetlight', 'build', rn);
          ViewRegistry.safeCall('barrier', 'build', rn);
          ViewRegistry.safeCall('tree', 'build', rn);
          ViewRegistry.safeCall('viaduct', 'build', { edges: [] });
          store.isViaduct = false;
          store.viaductVisLength = VIADUCT_VIS_LENGTH;
        }

        ViewRegistry.safeCall('connector', 'build', rn);
        lastRoadHash = hash;
        store.roadNetwork = rn;
        store.roadHash = hash;
        resetEntities();
      }
    }

    if (frame.ego !== undefined && !skipEgo) {
      const e = frame.ego;
      const newX = e.x || 0;

      store.ego = {
        x: newX,
        y: e.y || 0,
        z: e.z || 0,
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
        _visualX: newX,
        _wrapOffset: 0,
      };
    }

    if (frame.entities !== undefined && !skipEntities) {
      store.entities = frame.entities.filter(e => e && e.type !== 'ego').map((e) => {
        const ent = {
          id: e.id,
          type: e.type,
          x: e.x || 0,
          y: e.y || 0,
          z: e.z || 0,
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
        /* 流畅专题：把真值喂进多实体 dead reckon Map。tickAnimation 每帧
         * 会用平滑后的 x/y/heading/speed 覆盖上面的真值，让 NPC 在 SSE
         * 5Hz 离散帧之间平滑插值，不再每帧 snap 跳变。 */
        updateEntityDeadReckon(ent.id, ent.x, ent.y, ent.speed, ent.heading);
        return ent;
      });
      /* 清理消失的 NPC，防止 Map 无限增长。 */
      pruneEntities(new Set(store.entities.map(e => e.id)));
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
      const egoZ = store.ego.z;
      store.ego.x = _dr.smoothX;
      store.ego.y = _dr.smoothZ;
      store.ego.heading = _dr.smoothHeading;
      store.ego.speed = _dr.smoothSpeed;
      store.ego.z = egoZ;
    }
    /* 流畅专题：NPC 与 ego 同构的 dead reckon。
     * now 是 performance.now() 毫秒，转秒喂给 entity tick（与 ego 路径
     * 的 performance.now()/1000 一致）。tick 后用平滑值覆盖 store.entities
     * 的 x/y/heading/speed，VehicleView 读到的就是插值后位姿。
     * 首帧（entity 刚 init）smooth 已 snap 到真值，不会从原点 lerp。 */
    if (store.entities && store.entities.length) {
      tickEntityDeadReckon(now / 1000);
      for (let i = 0; i < store.entities.length; i++) {
        const ent = store.entities[i];
        const sm = getEntitySmooth(ent.id);
        if (sm) {
          ent.x = sm.x;
          ent.y = sm.y;
          ent.heading = sm.heading;
          ent.speed = sm.speed;
        }
      }
    }
    /* Layer 树递归 update：agent(vehicle) + infra(trafficLight, etcGate) */
    rootLayer.update(store, now);

    // B2 fix: 高架 deck 跟随 ego 移动，消除"钉死在世界原点"的视觉 bug
    if (store.isViaduct && store.ego) {
      const viaductView = ViewRegistry.get('viaduct');
      if (viaductView && viaductView.followEgo) {
        const visLen = store.viaductVisLength || VIADUCT_VIS_LENGTH;
        viaductView.followEgo(store.ego.x + visLen / 2);
      }
    }
  }

  function getStore() { return store; }
  /* getter 代理到 ViewRegistry —— ViewRegistry 是单一事实来源，
   * 避免顶层 const + getter 两份引用。删掉实际无人调用的
   * getGroundView/getConnectorView/getTrafficLightView/getETCGateView/
   * getStreetlightView/getBarrierView（API 表面更诚实）。
   * 保留 main.js 实际还在调的 4 个：getRoadView/getViaductView/
   * getVehicleView + getStore。 */
  const getRoadView    = () => ViewRegistry.get('road');
  const getViaductView = () => ViewRegistry.get('viaduct');
  const getVehicleView = () => ViewRegistry.get('vehicle');

  /* ── Layer 树访问（调试 + dispose 用）── */
  function getRootLayer() { return rootLayer; }
  function getLayer(name) { return rootLayer.findDescendant(name); }

  /* ── dispose() — 切场景 / 卸载时调用，递归清理所有 View 资源 ──
   * 调用后 director 不可再用（store/layer 都已 dispose）。
   * 当前 main.js 没用到（页面生命周期内不卸载），但留出口便于
   * 未来动态切场景、HMR、单测隔离用。 */
  function dispose() {
    rootLayer.dispose();
  }

  return { init, update, tickAnimation, dispose,
           getStore, getRoadView, getViaductView, getVehicleView,
           getRootLayer, getLayer, resetWarnings };
}
