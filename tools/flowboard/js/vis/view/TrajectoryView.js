/**
 * TrajectoryView.js — 预测轨迹线渲染
 *
 * 根据 ego 当前状态（steer, speed, heading）用运动学自行车模型
 * 预测未来 3 秒的行驶轨迹，以半透明管线显示。
 *
 * 颜色编码：
 *   绿色 (#00ff88) → 巡航直行
 *   黄色 (#ffaa00) → 转向/变道
 *   红色 (#ff2200) → 紧急制动
 *
 * 管线在每帧 update() 中重建，clear() 释放所有几何/材质。
 */

import { worldToThree } from '../math/Coord.js';

const PREDICT_DURATION_S = 3.0;    // 预测时长 (s)
const PREDICT_STEPS = 30;          // 轨迹点数
const TRAJECTORY_RADIUS = 0.12;    // 管线半径 (m)
const TURN_STEER_THRESHOLD = 0.05; // 转向检测阈值 (rad)
const WHEELBASE = 2.7;             // 轴距 (m)，与 flowsim 一致

/* 颜色常量 */
const COLOR_CRUISE = 0x00ff88;
const COLOR_TURN   = 0xffaa00;
const COLOR_BRAKE  = 0xff2200;

export function createTrajectoryView(scene) {
  const group = new THREE.Group();
  scene.add(group);

  let lineMesh = null;

  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
    lineMesh = null;
  }

  /** 用运动学自行车模型预测轨迹
   *  @param {object} ego  store.ego 对象
   *  @returns {{points: number[][], color: number}}
   *    points: [[x,y,z],...] in THREE 坐标
   *    color: 轨迹颜色
   */
  function _predictPath(ego) {
    if (!ego || ego.speed < 0.1) {
      return { points: [], color: COLOR_CRUISE };
    }

    const dt = PREDICT_DURATION_S / PREDICT_STEPS;
    const steer = ego.steer || 0;
    const speed = ego.speed || 0;
    const heading = ego.heading || 0;
    const brake = ego.brake || 0;

    // 运动学自行车模型积分（ENU 坐标系）
    let x = 0, y = 0, h = heading;
    const points = [];
    const [wx, wy, wz] = worldToThree(ego.x, ego.y, ego.z);

    for (let i = 0; i < PREDICT_STEPS; i++) {
      // 当前速度（刹车时减速）
      const v = brake > 0.3 ? speed * (1.0 - 0.5 * brake * i / PREDICT_STEPS) : speed;

      // yaw_rate = v / L * tan(steer)
      const yawRate = v / WHEELBASE * Math.tan(steer);
      h += yawRate * dt;

      x += v * Math.cos(h) * dt;
      y += v * Math.sin(h) * dt;

      // 转换为 THREE 坐标
      const [tx, ty, tz] = worldToThree(wx + x, wy + y, wz);
      points.push([tx, ty + 0.1, tz]);  // 略高于路面
    }

    // 确定颜色
    let color = COLOR_CRUISE;
    if (brake > 0.5) {
      color = COLOR_BRAKE;
    } else if (Math.abs(steer) > TURN_STEER_THRESHOLD) {
      color = COLOR_TURN;
    }

    return { points, color };
  }

  /** 从轨迹点数组构建 TubeGeometry
   *  @param {number[][]} points  [[x,y,z],...] 至少 2 点
   *  @param {number} color  THREE.Color hex
   *  @returns {THREE.Mesh|null}
   */
  function _buildTrajectoryMesh(points, color) {
    if (points.length < 2) return null;

    // 构建 CatmullRomCurve3
    const vecs = points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    const curve = new THREE.CatmullRomCurve3(vecs);

    const geo = new THREE.TubeGeometry(curve, points.length * 2, TRAJECTORY_RADIUS, 6, false);
    const mat = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.5,
      depthWrite: false,
    });
    return new THREE.Mesh(geo, mat);
  }

  function update(store, now) {
    if (!store.ego) return;

    // 移除旧轨迹线
    if (lineMesh) {
      group.remove(lineMesh);
      if (lineMesh.geometry) lineMesh.geometry.dispose();
      if (lineMesh.material) lineMesh.material.dispose();
      lineMesh = null;
    }

    const { points, color } = _predictPath(store.ego);
    if (points.length < 2) return;

    const mesh = _buildTrajectoryMesh(points, color);
    if (mesh) {
      lineMesh = mesh;
      group.add(mesh);
    }
  }

  return { update, clear };
}