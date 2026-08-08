/**
 * TrafficLightView.js — 交通信号灯
 *
 * 渲染逻辑唯一来源：scene/frame 的 entities 数组里 type='tl' 的实体。
 * 不再从 road_network.edges[*].traffic_lights 二次构造，避免"同个红绿灯
 * 多处实现"导致重复或错位渲染。
 *
 * 坐标/朝向约定：
 *   - 后端 heading 表示灯杆臂的指向（垂直于道路切线，指向道路中心）。
 *   - 前端本地模型：臂默认沿 -z，灯壳/灯泡默认朝 +x（向来车方向）。
 *   - 组旋转：headingToRotationY(heading - PI/2)，使臂与后端 heading 对齐。
 */

import { getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { worldToThree, headingToRotationY } from '../math/Coord.js';
import { roadHeightAt } from '../math/RoadHeight.js';

const RED = 0xff0000, YELLOW = 0xffaa00, GREEN = 0x00ff00;
const LAMP_Y = [4.6, 4.3, 4.0];  // 红/黄/绿的 Y 坐标

export function createTrafficLightView(scene) {
  // entity id → { group, lamps: [red, yellow, green] }
  const pool = new Map();

  function _createTrafficLight() {
    const g = new THREE.Group();

    const poleMat = getStdMaterial(0x555555, 0.4, 0.6);

    // 灯杆（5m 圆柱）
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.15, 5.0, 12), poleMat);
    pole.position.y = 2.5;
    pole.castShadow = true;
    g.add(pole);

    // 臂架：4m 横向 Box，默认沿 -z（从杆顶向道路中心延伸）
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 4.0), poleMat);
    arm.position.set(0, 4.8, -2.0);
    g.add(arm);

    // 灯壳：在臂架末端，默认朝 +x（向来车方向）
    const housingMat = getStdMaterial(0x222222, 0.7, 0.3);
    const housing = new THREE.Mesh(
      new THREE.BoxGeometry(0.3, 0.9, 0.35),
      housingMat
    );
    housing.position.set(0, 4.3, -4.0);
    // 默认 BoxGeometry 的 +x 面作为正面；无需额外旋转即可朝 +x
    housing.castShadow = true;
    g.add(housing);

    // 3 灯泡（红/黄/绿，默认暗），装在灯壳 +x 面外侧
    const lamps = [];
    const colors = [RED, YELLOW, GREEN];
    for (let i = 0; i < 3; i++) {
      const lamp = new THREE.Mesh(
        new THREE.SphereGeometry(0.11, 16, 12),
        createEmissiveMaterial(colors[i], 0.05)
      );
      // 灯泡排成一列，x 略微突出（-x 侧）以便面向来车
      lamp.position.set(-0.16, LAMP_Y[i], -4.0);
      g.add(lamp);
      lamps.push(lamp);
    }

    g.userData.lamps = lamps;
    g.userData.lampColors = colors;
    scene.add(g);
    return { group: g, lamps };
  }

  /** 切换灯状态；闪绿由仿真时间驱动，2Hz 闪烁。 */
  function _setLight(entry, state) {
    const idx = typeof state === 'number' ? state
              : state === 'red' ? 0
              : state === 'yellow' ? 1
              : state === 'green' || state === 'flashing_green' ? 2
              : -1;
    const flashingOff = state === 'flashing_green' &&
      Math.floor(performance.now() / 250) % 2 === 0;
    entry.lamps.forEach((lamp, i) => {
      lamp.material.emissiveIntensity =
        (i === idx && !flashingOff) ? 2.0 : 0.05;
    });
  }

  /** 主更新入口：只认 scene/frame entities 里的 type='tl' */
  function update(store) {
    const all = (store.entities || []).filter(e => e && e.type === 'tl');

    // 删除消失的
    const aliveIds = new Set(all.map(e => e.id));
    for (const [id, entry] of pool.entries()) {
      if (!aliveIds.has(id)) {
        scene.remove(entry.group);
        pool.delete(id);
      }
    }

    // 创建/更新
    for (const ent of all) {
      let entry = pool.get(ent.id);
      if (!entry) {
        entry = _createTrafficLight();
        pool.set(ent.id, entry);
      }
      const z = roadHeightAt(store, ent.x, ent.y);
      entry.group.position.set(...worldToThree(ent.x, ent.y, z));
      // 后端 heading 是臂指向；本地臂默认沿 -z。
      // 用 headingToRotationY(h - PI/2) = PI/2 - h 对齐。
      entry.group.rotation.y = headingToRotationY((ent.heading || 0) - Math.PI / 2);
      _setLight(entry, ent.state);
    }
  }

  function clear() {
    for (const [, entry] of pool) scene.remove(entry.group);
    pool.clear();
  }

  return { update, clear };
}
