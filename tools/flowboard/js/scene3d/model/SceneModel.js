// ═══════════════════════════════════════════════════════════════
// SceneModel.js — 场景数据快照（ego/obstacles/lidar/trajectory）
// ═══════════════════════════════════════════════════════════════
// 职责：从 TopologyStore 派生场景实体数据快照，提供结构化访问器。
// 设计：纯读，每次调用返回最新快照。不引用 THREE，不持有 mesh。
// ═══════════════════════════════════════════════════════════════

import { getScene, getVehicle, getMetrics } from './TopologyStore.js';

/**
 * 读取 ego 场景数据快照。
 * @returns {{x:number, z:number, heading:number, speed:number, steer:number, brake:number}|null}
 */
export function getEgoSnapshot() {
  var scn = getScene();
  if (!scn || !scn.ego) return null;
  var v = getVehicle();
  var e = scn.ego || {};
  return {
    x: e.x || 0,
    z: e.z || 0,
    heading: e.heading || 0,
    speed: v.speed || 0,
    targetSpeed: v.target_speed || 0,
    steer: v.steer || 0,
    brake: v.brake || 0,
    throttle: v.throttle || 0,
    error: v.error || 0
  };
}

/**
 * 读取障碍物列表（scene.obstacles）。
 * @returns {Array}
 */
export function getObstacles() {
  var scn = getScene();
  return (scn && scn.obstacles) || [];
}

/**
 * 读取交通灯列表（scene.entities 中 type=traffic_light）。
 * @returns {Array}
 */
export function getTrafficLights() {
  var scn = getScene();
  return (scn && scn.entities) ? scn.entities.filter(function(e) {
    return e.type === 'traffic_light';
  }) : [];
}

/**
 * 读取 ETC 门架列表（scene.entities 中 type=etc_gate）。
 * @returns {Array}
 */
export function getETCGates() {
  var scn = getScene();
  return (scn && scn.entities) ? scn.entities.filter(function(e) {
    return e.type === 'etc_gate';
  }) : [];
}

/**
 * 读取 LiDAR 点云数据。
 * @returns {object|null}
 */
export function getLidar() {
  var scn = getScene();
  return (scn && scn.lidar) || null;
}

/**
 * 读取规划轨迹路径（Frenet [[s,d,spd],...]）。
 * @returns {Array}
 */
export function getTrajectoryPath() {
  var scn = getScene();
  return (scn && scn.trajectory_path) || [];
}

/** 读取轨迹所在 edge_id（monitor 透传 ego.road_id） */
export function getTrajectoryEdgeId() {
  var scn = getScene();
  return (scn && scn.trajectory_edge_id != null) ? scn.trajectory_edge_id : -1;
}

/** 读取 route_lane（转向灯判断用） */
export function getRouteLane() {
  return getMetrics().route_lane || 0;
}

/**
 * 读取道路网络数据。
 * @returns {{edges: Array, road: object}|null}
 */
export function getRoadNetwork() {
  var scn = getScene();
  if (!scn) return null;
  return {
    edges: (scn.road_network && scn.road_network.edges) || null,
    road: scn.road || null
  };
}
