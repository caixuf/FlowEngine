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

/* ── 数据校验辅助 ──────────────────────────────────────────────
 * _warnOnce(key, msg)：同一 key 只 warn 一次，避免 20Hz 刷屏。
 * 校验失败时调用方自行决定回退值（默认仍走原 || 0 逻辑）。 */
function _typeOf(v) {
  if (v === null) return 'null';
  if (Array.isArray(v)) return 'array';
  return typeof v;
}

export function createSceneDirector(scene) {
  const store = createSceneStore();
  const roadView = createRoadView(scene);
  const groundView = createGroundView(scene);
  const vehicleView = createVehicleView(scene);
  const connectorView = createConnectorView(scene);
  const trafficLightView = createTrafficLightView(scene);
  const etcGateView = createETCGateView(scene);
  const viaductView = createViaductView(scene);
  const streetlightView = createStreetlightView(scene);
  const barrierView = createBarrierView(scene);
  let lastRoadHash = '';

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
    groundView.build(20000);
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
    /* ── 校验 1：topoData 必须是 object ── */
    if (!topoData) return;
    if (typeof topoData !== 'object') {
      _warnOnce('topoData.type', 'update() 收到非对象 topoData: ' + _typeOf(topoData) + '，跳过本帧');
      return;
    }

    let frame = topoData;
    if (topoData.metrics && topoData.metrics.scene) frame = topoData.metrics.scene;
    else if (topoData.scene) frame = topoData.scene;

    /* ── 校验 2：frame 解包后必须是 object ── */
    if (typeof frame !== 'object' || frame === null) {
      _warnOnce('frame.type', 'frame 解包后非对象: ' + _typeOf(frame) + '，跳过本帧');
      return;
    }

    const rn = frame.road_network || frame.roads;
    let skipRoad = false;
    if (rn) {
      /* ── 校验 3：road_network 必须是 object ── */
      if (typeof rn !== 'object' || rn === null) {
        _warnOnce('road_network.type', 'road_network 非 object: ' + _typeOf(rn) + '，跳过道路重建');
        skipRoad = true;
      } else if (rn.edges !== undefined && !Array.isArray(rn.edges)) {
        /* ── 校验 3b：edges 若发则必须是 array ── */
        _warnOnce('road_network.edges', 'road_network.edges 非数组: ' + _typeOf(rn.edges) + '，跳过道路重建');
        skipRoad = true;
      }
      /* 校验 4：edges 是数组但为空时，仍允许走 viaduct 分支（edge0={} 兜底）。 */
    }

    let skipEgo = false;
    if (frame.ego !== undefined) {
      const e = frame.ego;
      /* ── 校验 5：ego 必须是 object ── */
      if (typeof e !== 'object' || e === null) {
        _warnOnce('ego.type', 'frame.ego 非 object: ' + _typeOf(e) + '，跳过 ego 更新');
        skipEgo = true;
      } else {
        /* ── 校验 6：ego.x 必须是 finite number；heading / lights 若发则必须是 number ── */
        if (typeof e.x !== 'number' || !isFinite(e.x)) {
          _warnOnce('ego.x', 'ego.x 非 finite number: ' + _typeOf(e.x) + '，回退 0（ego 会卡在原点，检查 scene_pub ego JSON）');
        }
        if (e.heading !== undefined && typeof e.heading !== 'number') {
          _warnOnce('ego.heading', 'ego.heading 非 number: ' + _typeOf(e.heading));
        }
        if (e.lights !== undefined && typeof e.lights !== 'number') {
          _warnOnce('ego.lights', 'ego.lights 非 number: ' + _typeOf(e.lights) + '，回退 0（车灯不亮）');
        }
      }
    }

    let skipEntities = false;
    if (frame.entities !== undefined) {
      /* ── 校验 7：entities 必须是 array ── */
      if (!Array.isArray(frame.entities)) {
        _warnOnce('entities.type', 'frame.entities 非数组: ' + _typeOf(frame.entities) + '，跳过实体更新');
        skipEntities = true;
      } else {
        /* ── 校验 8：每个 entity 必须有 type 字段（仅检查非 ego 实体，与 build 过滤一致）── */
        let nonEgoIdx = 0;
        for (const e of frame.entities) {
          if (!e || e.type === 'ego') continue;
          if (!e.type) {
            _warnOnce('entities[' + nonEgoIdx + '].type', 'entity 缺 type 字段，会被各 View 静默丢弃');
          }
          nonEgoIdx++;
        }
      }
    }

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
          groundView.build(20000);
          viaductView.build({ laneCount, laneWidth, length: actualLength, withEnvironment: true });
          roadView.build({ edges: [] });
          /* Step 4：高架场景路灯/护栏由 ViaductView 内置，StreetlightView /
           * BarrierView 在普通 edge 上不放置（内部跳过 viaduct_highway），
           * 此处显式 clear 一遍保证切换场景时不残留。 */
          streetlightView.build({ edges: [] });
          barrierView.build({ edges: [] });
          store.isViaduct = true;
        } else {
          roadView.build(rn);
          groundView.build(20000);
          /* Step 4：普通道路场景，路灯 + 护栏从 roadNetwork 自动布局。 */
          streetlightView.build(rn);
          barrierView.build(rn);
          store.isViaduct = false;
        }

        connectorView.build(rn);
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
        viaductView.followEgo(wrapOffset + VIADUCT_VIS_LENGTH / 2);
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

    trafficLightView.update(store);
    etcGateView.update(store);
  }

  /* ── tickAnimation(now) — 每帧推进死推算 + 把平滑结果写回 store.ego ──
   *
   * Step 2 重构：原 main.js 行 104-112 直接 import deadreckon + 覆盖
   * store.ego.x/y/heading/speed，违反"数据单向流：Director → Store → View"。
   * 现在此逻辑下沉到 SceneDirector，main.js 只调 director.tickAnimation(now)。
   *
   * 数据流：
   *   app.js sync2DTarget (SSE tick) → updateDeadReckon(x,z,speed,heading) → _dr
   *   main.js 渲染帧 → director.tickAnimation(now)
   *     → tickDeadReckon() 推进 _dr.smooth*
   *     → 把 smooth* 写入 store.ego（覆盖 x/y/heading/speed，保留 _simX 等原始字段）
   *
   * 2D（scene2d.js）仍直接调 tickDeadReckon + 读 _dr.smooth*，与 3D 共享同一
   * _dr 全局单例，保证 3D/2D 视图 ego 位置完全同步。
   *
   * @param {number} now 当前 performance.now() 毫秒（保留参数兼容，实际 tickDeadReckon 内部自取）
   */
  function tickAnimation(now) {
    tickDeadReckon();
    if (store.ego && _dr.init) {
      store.ego.x = _dr.smoothX;
      store.ego.y = _dr.smoothZ;
      store.ego.heading = _dr.smoothHeading;
      store.ego.speed = _dr.smoothSpeed;
    }
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
