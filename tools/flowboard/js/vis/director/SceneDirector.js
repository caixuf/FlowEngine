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
    if (!topoData) return;

    let frame = topoData;
    if (topoData.metrics && topoData.metrics.scene) frame = topoData.metrics.scene;
    else if (topoData.scene) frame = topoData.scene;

    const rn = frame.road_network || frame.roads;
    if (rn) {
      const hash = roadNetworkHash(rn);
      if (hash !== lastRoadHash) {
        const urlViaduct = typeof window !== 'undefined' &&
                           new URLSearchParams(window.location.search).get('viaduct') === '1';
        const isViaduct = urlViaduct || (rn.edges && rn.edges.some(e =>
          e.name === 'viaduct_highway' || e.type === 'viaduct_highway'
        ));

        if (isViaduct) {
          const edge0 = rn.edges[0] || {};
          const laneCount = edge0.lanes || 4;
          const laneWidth = edge0.lane_width || 3.5;
          const actualLength = edge0.length || edge0.length_m || VIADUCT_VIS_LENGTH;
          groundView.build(20000);
          viaductView.build({ laneCount, laneWidth, length: actualLength, withEnvironment: true });
          roadView.build({ edges: [] });
          store.isViaduct = true;
        } else {
          roadView.build(rn);
          groundView.build(20000);
          store.isViaduct = false;
        }

        connectorView.build(rn);
        lastRoadHash = hash;
        store.roadNetwork = rn;
        store.roadHash = hash;
      }
    }

    if (frame.ego) {
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

    if (frame.entities) {
      const viaductOffset = store.isViaduct ? 7.0 : 0;
      store.entities = frame.entities.filter(e => e.type !== 'ego').map(e => ({
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
      }));
    }

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