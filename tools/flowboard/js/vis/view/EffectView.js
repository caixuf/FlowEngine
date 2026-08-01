/**
 * EffectView.js — 动态视觉效果（紧急制动/障碍物标记/切入预警）
 *
 * 渲染非车辆实体类视觉效果：
 *   - 紧急制动光晕（brake > 0.5 → 车后红色锥形光晕）
 *   - 障碍物标记（ego 附近 NPC → 橙色半透明框 + 距离标签）
 *   - 紧急闪光（hazard 亮起 → 红/黄交替闪烁）
 *   - 切入预警（旁车切入 → 黄色箭头指向切入方向）
 *
 * 所有效果在每帧 update() 中重建（位置/颜色随状态变化），
 * 几何/材质在 clear() 释放。
 */

import { worldToThree } from '../math/Coord.js';

const BRAKE_THRESHOLD = 0.5;
const OBSTACLE_RADIUS_M = 40.0;    // 障碍物标记可视范围 (m)
const CUTIN_LATERAL_SPEED = 2.0;   // 切入预警横向速度阈值 (m/s)
const FLASH_INTERVAL_MS = 500;     // 双闪周期 (ms)

/* 材质缓存：避免每帧新建 */
let _hazardMat = null;

export function createEffectView(scene) {
  const group = new THREE.Group();
  scene.add(group);

  /* ── 内部状态 ── */
  let brakeGlow = null;      // THREE.Mesh
  let hazardFlash = null;    // THREE.Mesh

  /** 清除所有视觉效果 */
  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
    brakeGlow = null;
    hazardFlash = null;
    _hazardMat = null;
  }

  /** 更新紧急制动光晕（红色锥形，在 ego 车后） */
  function _updateBrakeGlow(ego, now) {
    const isBraking = ego.brake > BRAKE_THRESHOLD;
    if (!isBraking) {
      if (brakeGlow) {
        group.remove(brakeGlow);
        if (brakeGlow.geometry) brakeGlow.geometry.dispose();
        if (brakeGlow.material) brakeGlow.material.dispose();
        brakeGlow = null;
      }
      return;
    }

    // 创建或更新刹车光晕
    if (!brakeGlow) {
      const geo = new THREE.ConeGeometry(0.8, 1.5, 12);
      const mat = new THREE.MeshBasicMaterial({
        color: 0xff2200,
        transparent: true,
        opacity: 0.35,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      brakeGlow = new THREE.Mesh(geo, mat);
      group.add(brakeGlow);
    }

    // 位置：车后 1.5m，略低于车底
    const [tx, ty, tz] = worldToThree(ego.x, ego.y, ego.z);
    const heading = ego.heading || 0;
    // 车后方向 = heading 反方向，THREE: heading 绕 Y 轴
    const bx = -Math.sin(heading) * 1.5;
    const bz = -Math.cos(heading) * 1.5;
    brakeGlow.position.set(tx + bx, 0.05, tz + bz);
    brakeGlow.rotation.x = Math.PI / 2;  // 锥体朝上
    brakeGlow.rotation.z = heading;       // 朝向车尾方向

    // 脉冲亮度：根据 brake 力度
    const intensity = Math.min(1.0, ego.brake / 1.5);
    brakeGlow.material.opacity = 0.15 + intensity * 0.4;
    brakeGlow.scale.setScalar(0.5 + intensity * 0.8);
  }

  /** 更新紧急双闪（红/黄交替闪烁） */
  function _updateHazardFlash(ego, now) {
    const isHazard = !!(ego.lights & 0x04);  // LIGHT_HAZARD
    if (!isHazard) {
      if (hazardFlash) {
        group.remove(hazardFlash);
        if (hazardFlash.geometry) hazardFlash.geometry.dispose();
        if (hazardFlash.material) hazardFlash.material.dispose();
        hazardFlash = null;
        _hazardMat = null;
      }
      return;
    }

    const flashOn = Math.floor(now / FLASH_INTERVAL_MS) % 2 === 0;

    if (!hazardFlash) {
      const geo = new THREE.SphereGeometry(0.3, 8, 8);
      _hazardMat = new THREE.MeshBasicMaterial({
        color: 0xff4400,
        transparent: true,
        opacity: 0.8,
        depthWrite: false,
      });
      hazardFlash = new THREE.Mesh(geo, _hazardMat);
      group.add(hazardFlash);
    }

    // 位置：车顶上方
    const [tx, ty, tz] = worldToThree(ego.x, ego.y, ego.z);
    hazardFlash.position.set(tx, ty + 1.5, tz);
    hazardFlash.scale.setScalar(flashOn ? 1.0 : 0.3);
    if (_hazardMat) {
      _hazardMat.color.setHex(flashOn ? 0xff4400 : 0xffff00);
      _hazardMat.opacity = flashOn ? 0.8 : 0.2;
    }
  }

  /** 更新障碍物标记（ego 附近 NPC → 橙色半透明框） */
  function _updateObstacleMarkers(store) {
    if (!store.ego || !store.entities) return;

    // 收集当前帧应显示的障碍物 id
    const activeIds = new Set();
    for (const ent of store.entities) {
      if (!ent || ent.type === 'ego') continue;
      const dx = (ent.x || 0) - store.ego.x;
      const dz = (ent.y || 0) - store.ego.y;  // entity.y 是 ENU y（北向）
      const dist = Math.sqrt(dx * dx + dz * dz);
      if (dist > OBSTACLE_RADIUS_M) continue;

      activeIds.add(ent.id);

      // 创建或复用标记
      let marker = obstacleMarkers.get(ent.id);
      if (!marker) {
        const geo = new THREE.BoxGeometry(0.5, 0.5, 0.02);
        const mat = new THREE.MeshBasicMaterial({
          color: 0xff8800,
          transparent: true,
          opacity: 0.5,
          side: THREE.DoubleSide,
          depthWrite: false,
        });
        marker = new THREE.Mesh(geo, mat);
        group.add(marker);
        obstacleMarkers.set(ent.id, marker);
      }

      // 位置：在 NPC 头顶上方
      const [tx, ty, tz] = worldToThree(ent.x || 0, ent.y || 0, ent.z || 0);
      marker.position.set(tx, ty + 2.5, tz);

      // 颜色根据距离渐变
      const intensity = 1.0 - Math.min(1.0, dist / OBSTACLE_RADIUS_M);
      const r = Math.round(0xff);
      const g = Math.round(0x88 + (1.0 - intensity) * 0x77);
      marker.material.color.setRGB(r / 255, g / 255, 0x22 / 255);
      marker.material.opacity = 0.2 + intensity * 0.5;
      // 始终面向相机（通过旋转朝上 + 让场景自动 billboard? 不，简单处理）
      marker.rotation.x = -Math.PI / 2;
    }

    // 移除消失的障碍物标记
    for (const [id, mesh] of obstacleMarkers) {
      if (!activeIds.has(id)) {
        group.remove(mesh);
        if (mesh.geometry) mesh.geometry.dispose();
        if (mesh.material) mesh.material.dispose();
        obstacleMarkers.delete(id);
      }
    }
  }

  const obstacleMarkers = new Map();

  /** 更新切入预警（旁车横向速度 > 阈值 → 黄色箭头） */
  function _updateCutinWarning(store) {
    if (!store.ego || !store.entities) return;

    const activeIds = new Set();
    for (const ent of store.entities) {
      if (!ent || ent.type === 'ego' || ent.parked) continue;
      if (ent.speed < 0.5) continue;  // 静止车不预警

      // 横向速度估计：从 heading 和 speed 分解
      // 现实世界应该用 vx/vy，但 store 里只有 heading + speed
      // 粗略估计：heading 与车道方向（ego heading）的偏差产生的横向速度
      const headingDelta = (ent.heading || 0) - (store.ego.heading || 0);
      const lateralSpeed = Math.abs(Math.sin(headingDelta) * (ent.speed || 0));
      if (lateralSpeed < CUTIN_LATERAL_SPEED) continue;

      // 相对距离：前后 20m 内
      const dx = (ent.x || 0) - store.ego.x;
      const dz = (ent.y || 0) - store.ego.y;
      const dist = Math.sqrt(dx * dx + dz * dz);
      if (dist > 25.0) continue;

      activeIds.add(ent.id);

      let arrow = cutinArrows.get(ent.id);
      if (!arrow) {
        // 角锥体作为箭头
        const geo = new THREE.ConeGeometry(0.3, 0.8, 8);
        const mat = new THREE.MeshBasicMaterial({
          color: 0xffcc00,
          transparent: true,
          opacity: 0.7,
          depthWrite: false,
        });
        arrow = new THREE.Mesh(geo, mat);
        group.add(arrow);
        cutinArrows.set(ent.id, arrow);
      }

      // 位置：在 NPC 侧方，指向切入方向
      const [tx, ty, tz] = worldToThree(ent.x || 0, ent.y || 0, ent.z || 0);
      const lateralDir = Math.sin(ent.heading || 0) > 0 ? 1 : -1;
      arrow.position.set(tx + lateralDir * 1.5, ty + 1.0, tz + lateralDir * 1.5);
      arrow.rotation.x = Math.PI / 2;
      arrow.rotation.z = -(ent.heading || 0);
    }

    // 清理消失的箭头
    for (const [id, mesh] of cutinArrows) {
      if (!activeIds.has(id)) {
        group.remove(mesh);
        if (mesh.geometry) mesh.geometry.dispose();
        if (mesh.material) mesh.material.dispose();
        cutinArrows.delete(id);
      }
    }
  }

  const cutinArrows = new Map();

  function update(store, now) {
    if (!store.ego) return;

    _updateBrakeGlow(store.ego, now);
    _updateHazardFlash(store.ego, now);
    _updateObstacleMarkers(store);
    _updateCutinWarning(store);
  }

  return { update, clear };
}