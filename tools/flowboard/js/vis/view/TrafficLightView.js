/**
 * TrafficLightView.js — 交通信号灯
 *
 * 从 utils.js _buildTrafficLight 搬迁：灯杆 5m + 臂架 4m + 灯壳 + 3 灯泡（红黄绿）。
 * update(store) 扫描 entities 里 type='tl' 的实体，根据 state 切换灯亮：
 *   state='red'/'yellow'/'green'（或 0/1/2）→ 对应灯 emissiveIntensity 拉高
 *
 * 灯位置：路口 edge 的 traffic_lights 字段（s, l 横向偏移）。
 */

import { getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { worldToThree } from '../math/Coord.js';
import { roadHeightAt } from '../math/RoadHeight.js';
import { getEdgePointAtS } from '../math/Curve.js';

const RED = 0xff0000, YELLOW = 0xffaa00, GREEN = 0x00ff00;
const LAMP_Y = [4.6, 4.3, 4.0];  // 红/黄/绿的 Y 坐标

export function createTrafficLightView(scene) {
  // entity id → { group, lamps: [red, yellow, green] }
  const pool = new Map();

  function _createTrafficLight() {
    const g = new THREE.Group();

    // 灯杆（5m 圆柱）
    const poleMat = getStdMaterial(0x555555, 0.4, 0.6);
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.15, 5.0, 12), poleMat);
    pole.position.y = 2.5;
    pole.castShadow = true;
    g.add(pole);

    // 臂架（4m 横向 Box，高度 4.8m）
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.12, 4.0), poleMat);
    arm.position.set(0, 4.8, 2.0);
    g.add(arm);

    // 灯壳
    const housing = new THREE.Mesh(
      new THREE.BoxGeometry(0.35, 0.9, 0.3),
      getStdMaterial(0x222222, 0.7, 0.3)
    );
    housing.position.set(0, 4.3, 4.0);
    housing.castShadow = true;
    g.add(housing);

    // 3 灯泡（红/黄/绿，默认暗）
    const lamps = [];
    const colors = [RED, YELLOW, GREEN];
    for (let i = 0; i < 3; i++) {
      const lamp = new THREE.Mesh(
        new THREE.SphereGeometry(0.11, 16, 12),
        createEmissiveMaterial(colors[i], 0.05)
      );
      lamp.position.set(0, LAMP_Y[i], 4.16);
      g.add(lamp);
      lamps.push(lamp);
    }

    g.userData.lamps = lamps;
    g.userData.lampColors = colors;
    scene.add(g);
    return { group: g, lamps };
  }

  /** 切换灯状态：state='red'/'yellow'/'green' 或 0/1/2 */
  function _setLight(entry, state) {
    const idx = typeof state === 'number' ? state
              : state === 'red' ? 0
              : state === 'yellow' ? 1
              : state === 'green' ? 2
              : -1;
    entry.lamps.forEach((lamp, i) => {
      lamp.material.emissiveIntensity = (i === idx) ? 2.0 : 0.05;
    });
  }

  /** 主更新入口：扫描 entities 找 type='tl' */
  function update(store) {
    const all = (store.entities || []).filter(e => e.type === 'tl');
    // 也支持 frame.traffic_lights 数组（场景文件格式）
    const rn = store.roadNetwork;
    if (rn && rn.edges) {
      for (const edge of rn.edges) {
        const tls = edge.traffic_lights || [];
        for (let i = 0; i < tls.length; i++) {
          const tl = tls[i];
          /* getEdgePointAtS 返回 THREE 坐标 (x=ENU_x, y=elevation, z=-ENU_y)，
           * 但 entity 需要的是 ENU 坐标。直接从 nodes 算 ENU 位置 + 切线方向，
           * 再把 tl.l 作为横向偏移沿法线方向叠加，并推算垂直于道路的 heading。
           * 之前把 tl.l 当 ENU y、pt.z 当 elevation 是双重错位，导致灯杆
           * 落到错误世界坐标 + roadHeightAt 查询错位置返回 0 高度。 */
          const nodes = edge.nodes;
          const s = tl.s || 0;
          const edgeLen = edge.length || 1;
          const t = Math.max(0, Math.min(1, s / edgeLen));
          let enuX = 0, enuY = 0, tangentH = 0;
          if (nodes && nodes.length >= 2) {
            const a = nodes[0], b = nodes[nodes.length - 1];
            enuX = a[0] + (b[0] - a[0]) * t;
            enuY = a[1] + (b[1] - a[1]) * t;
            tangentH = Math.atan2(b[1] - a[1], b[0] - a[0]);
          }
          /* tl.l 是横向偏移：正值=法线左侧（北侧），负值=右侧（南侧）。
           * 法线方向 = 切线左侧 90° = (cos(h+π/2), sin(h+π/2)) = (-sin h, cos h)。 */
          const lateral = tl.l || 0;
          const nx = -Math.sin(tangentH);
          const ny = Math.cos(tangentH);
          const px = enuX + nx * lateral;
          const py = enuY + ny * lateral;
          /* 灯杆臂应指向道路中心 = 横向偏移的反方向。
           * 北侧杆 (lateral>0)：臂朝南 = tangentH - π/2
           * 南侧杆 (lateral<0)：臂朝北 = tangentH + π/2 */
          const sign = (lateral >= 0) ? 1.0 : -1.0;
          const armHeading = tangentH + (sign > 0 ? -Math.PI / 2 : Math.PI / 2);
          all.push({
            id: `tl_${edge.id}_${i}`,
            type: 'tl',
            x: px,
            y: py,
            heading: armHeading,
            state: tl.state || 'red',
          });
        }
      }
    }

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
      // 灯臂沿 local +z，需要旋转使臂指向道路中心。
      // ENU heading + π/2 映射为 THREE rotation.y（与车辆 headingToRotationY 不同）。
      entry.group.rotation.y = (ent.heading || 0) + Math.PI / 2;
      _setLight(entry, ent.state);
    }
  }

  function clear() {
    for (const [, entry] of pool) scene.remove(entry.group);
    pool.clear();
  }

  return { update, clear };
}
