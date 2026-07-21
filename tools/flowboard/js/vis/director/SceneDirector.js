/**
 * SceneDirector.js — 场景导演
 * 接 scene/frame JSON → 写 SceneStore → 驱动各 View 更新。
 * diff 检测：road_network hash 变了才重建路。
 */

import { createSceneStore, roadNetworkHash } from '../store/SceneStore.js';
import { createRoadView } from '../view/RoadView.js';
import { createGroundView } from '../view/GroundView.js';
import { createVehicleView } from '../view/VehicleView.js';
import { createConnectorView } from '../view/ConnectorView.js';
import { createTrafficLightView } from '../view/TrafficLightView.js';
import { createETCGateView } from '../view/ETCGateView.js';
import { createViaductView } from '../view/ViaductView.js';

export function createSceneDirector(scene) {
  const store = createSceneStore();
  const roadView = createRoadView(scene);
  const groundView = createGroundView(scene);
  const vehicleView = createVehicleView(scene);
  const connectorView = createConnectorView(scene);
  const trafficLightView = createTrafficLightView(scene);
  const etcGateView = createETCGateView(scene);
  const viaductView = createViaductView(scene);
  let lastRoadHash = '';

  /** 初始化：不建草地（等首次 update 拿到 road_network 后决定） */
  function init() {
    // groundView 延迟到 update 阶段根据 edge 类型决定
  }

  /** 从 topoData / frame JSON 更新场景 */
  function update(topoData) {
    if (!topoData) return;

    // ── 仿真坐标 → 视觉坐标 ──
    // ViaductView 是固定 200m 几何（高架+国道+树）。仿真 ego 跑出 200m 时：
    //   - 视觉 ego 位置 = egoX % 200（车永远在视觉场景中心）
    //   - ego._offset 累积 wrap 次数，让 simulation 真值（速度/规划）保持
    //   - ViaductView.followEgo() 把 group 整体平移到 (egoX - egoX%200)
    //     → 护栏/路面相对 ego 永远在 ±100m 范围内，看起来"无限路"
    //   远景树/灌木跟着平移看起来静止（race game 套路，但车灯/护栏相对运动 OK）
    const VIADUCT_VIS_LENGTH = 200;

    // 兼容三种数据格式：
    // 1. monitor_node topology: { metrics: { scene: { road_network, ego, entities } } }
    // 2. scene/frame JSON: { scene: { road_network, ego, entities } }
    // 3. 直接的 frame: { road_network, ego, entities }
    let frame = topoData;
    if (topoData.metrics && topoData.metrics.scene) frame = topoData.metrics.scene;
    else if (topoData.scene) frame = topoData.scene;

    // ── road_network diff 检测 ──
    const rn = frame.road_network || frame.roads;
    if (rn) {
      const hash = roadNetworkHash(rn);
      if (hash !== lastRoadHash) {
        // 检测是否启用 viaduct 场景：三种信号按优先级
        // 1) URL query param `?viaduct=1`（强制启用，不依赖后端）
        // 2) edge.name === 'viaduct_highway'（理想情况，但 esmini RM_GetRoadIdString
        //    返回 xodr 的 id 属性不是 name，所以这条不工作）
        // 3) edge.type === 'viaduct_highway'（legacy 模式时有效）
        const urlViaduct = typeof window !== 'undefined' &&
                           new URLSearchParams(window.location.search).get('viaduct') === '1';
        const isViaduct = urlViaduct || (rn.edges && rn.edges.some(e =>
          e.name === 'viaduct_highway' || e.type === 'viaduct_highway'
        ));

        if (isViaduct) {
          // 取 edge 的车道数和长度
          // - laneCount: scene_pub.cpp 已修，输出单方向车道数（drivable_lanes/2）
          // - length: 仿真道路长度（1km 让 ego 跑很久不 wrap），
          //           但 ViaductView 视觉只渲染 200m（树/护栏密度合理），
          //           SceneDirector 把 ego 视觉位置 mod 200 + ViaductView.followEgo 平移
          const edge0 = rn.edges[0] || {};
          const laneCount = edge0.lanes || 4;
          const simLength = edge0.length || edge0.length_m || 1000;
          // viaduct 自带 grass，禁用默认 groundView
          groundView.build(0);
          // 视觉：固定 200m（树/护栏密度合理），相机跟着 ego 在 200m 范围内 clamp
          viaductView.build({ laneCount, length: 200, withEnvironment: true });
          // viaduct 自带 grass 区域（createGround 内嵌），不另建 RoadView 路面
          roadView.build({ edges: [] });
        } else {
          // 通用道路：原 RoadView + 默认 50000 草地
          roadView.build(rn);
          groundView.build(50000);
        }

        connectorView.build(rn);   // 连接件（依赖 edge 位置）
        lastRoadHash = hash;
        store.roadNetwork = rn;
        store.roadHash = hash;
      }
    }

    // ── ego ──
    // 字段兼容：
    //   monitor_node metrics.scene.ego: {x, y, heading, speed, steer}
    //   scene_pub scene/frame.ego:      {x, y, h, spd, steer, ...}
    if (frame.ego) {
      const e = frame.ego;
      const newX = e.x || 0;
      // 临时简化：禁用 ego%200 和 followEgo，让 ego 静止在固定位置
      // 等路面能看到后再逐步恢复
      const isViaductActive = typeof window !== 'undefined' &&
        new URLSearchParams(window.location.search).get('viaduct') === '1';
      const simX = newX;  // 临时：不用 mod 200
      store.ego = {
        x: simX,
        y: e.y || 0,
        heading: e.heading != null ? e.heading : (e.h || 0),
        speed: e.speed != null ? e.speed : (e.spd || 0),
        steer: e.steer || 0,
        brake: e.brake || 0,
        throttle: e.throttle || 0,
        lights: e.lights || 0,
        vx: e.vx || 0,
        vy: e.vy || 0,
        length: e.len || 4.6,
        width: e.wid || 2.0,
        _simX: newX,
      };
      // 临时：禁用 followEgo（让路面固定在 group.position=0）
      // if (isViaductActive) {
      //   viaductView.followEgo(simX);
      // }
    }

    // ── entities ──
    // 字段兼容（同 ego）：
    //   monitor_node metrics.scene.entities: {x, y, heading, speed, ...}
    //   scene_pub scene/frame.entities:      {x, y, h, spd, ...}
    if (frame.entities) {
      store.entities = frame.entities.map(e => ({
        id: e.id,
        type: e.type,
        x: e.x || 0,
        y: e.y || 0,
        heading: e.heading != null ? e.heading : (e.h || 0),
        speed: e.speed != null ? e.speed : (e.spd || 0),
        steer: e.steer || 0,
        length: e.len || 4.6,
        width: e.wid || 2.0,
        ai_state: e.ai || '',
        lights: e.lights || 0,      // NPC 车灯位掩码
        vx: e.vx || 0,
        vy: e.vy || 0,
        throttle: e.throttle || 0,
        brake: e.brake || 0,
        // 红绿灯/ETC 专用
        state: e.state || '',
        progress: e.progress || 0,
        remain_s: e.remain_s || 0,
        parked: e.parked || false,
      }));
    }

    // ── 交通灯 / ETC 门架更新 ──
    // 它们扫描 entities 里 type='tl'/'etc_gate' 的实体；
    // 也会读 roadNetwork.edges[*].traffic_lights（TrafficLightView 内部）
    trafficLightView.update(store);
    etcGateView.update(store);
  }

  function getStore() { return store; }
  function getRoadView() { return roadView; }
  function getGroundView() { return groundView; }
  function getVehicleView() { return vehicleView; }
  function getConnectorView() { return connectorView; }
  function getTrafficLightView() { return trafficLightView; }
  function getETCGateView() { return etcGateView; }
  function getViaductView() { return viaductView; }

  return { init, update, getStore, getRoadView, getGroundView, getVehicleView,
           getConnectorView, getTrafficLightView, getETCGateView, getViaductView };
}
