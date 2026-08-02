/**
 * TrajectoryView.js — 预测轨迹线渲染（科技蓝风格）
 *
 * 根据 ego 当前状态（steer, speed, heading）用运动学自行车模型
 * 预测未来 3 秒的行驶轨迹。
 *
 * 视觉效果：
 *   主管线：湛蓝色 (#00aaff) 半透明，宽半径 0.20m
 *   辉光层：外围更大半径淡蓝透明管（光晕效果）
 *   方向箭头：沿路径间隔放置的锥体箭头，指示行驶方向
 *   颜色微调：转向时浅蓝 (#00ccff)，制动时深蓝 (#0088cc)
 */

import { worldToThree, forwardENU } from '../math/Coord.js';

const PREDICT_DURATION_S = 3.0;    // 预测时长 (s)
const PREDICT_STEPS = 30;          // 轨迹点数
const TUBE_RADIUS = 0.20;          // 主管线半径 (m)
const GLOW_RADIUS = 0.40;          // 辉光层半径 (m)
const ARROW_INTERVAL = 5;          // 每 N 步放置一个箭头
const ARROW_LENGTH = 0.50;         // 箭头长度 (m)
const ARROW_RADIUS = 0.15;         // 箭头底部半径 (m)
const TURN_STEER_THRESHOLD = 0.05; // 转向检测阈值 (rad)
const WHEELBASE = 2.7;             // 轴距 (m)，与 flowsim 一致

/* 湛蓝色调色板 */
const COLOR_CRUISE = 0x00aaff;     // 巡航 - 科技湛蓝
const COLOR_TURN   = 0x00ccff;     // 转向 - 亮浅蓝
const COLOR_BRAKE  = 0x0088cc;     // 制动 - 深蓝
const COLOR_GLOW   = 0x0066ff;     // 辉光 - 淡蓝紫

export function createTrajectoryView(scene) {
  const group = new THREE.Group();
  scene.add(group);

  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
  }

  /** 用运动学自行车模型预测轨迹 */
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

      // ENU 坐标积分：用 Coord.forwardENU 取前向单位向量
      const [dx, dy] = forwardENU(h);
      x += v * dx * dt;
      y += v * dy * dt;

      // 转换为 THREE 坐标，略高于路面
      const [tx, ty, tz] = worldToThree(wx + x, wy + y, wz);
      points.push([tx, ty + 0.1, tz]);
    }

    // 确定颜色（保持湛蓝基调，微调色相/亮度）
    let color = COLOR_CRUISE;
    if (brake > 0.5) {
      color = COLOR_BRAKE;
    } else if (Math.abs(steer) > TURN_STEER_THRESHOLD) {
      color = COLOR_TURN;
    }

    return { points, color };
  }

  /** 构建主管线（CatmullRomCurve3 + TubeGeometry） */
  function _buildMainTube(points, color) {
    if (points.length < 2) return null;

    const vecs = points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    const curve = new THREE.CatmullRomCurve3(vecs);

    const geo = new THREE.TubeGeometry(curve, points.length * 2, TUBE_RADIUS, 8, false);
    const mat = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.75,
      depthWrite: false,
    });
    return new THREE.Mesh(geo, mat);
  }

  /** 构建辉光层（更大半径，低透明度，略在主管线外围） */
  function _buildGlowTube(points) {
    if (points.length < 2) return null;

    const vecs = points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    const curve = new THREE.CatmullRomCurve3(vecs);

    const geo = new THREE.TubeGeometry(curve, points.length * 2, GLOW_RADIUS, 8, false);
    const mat = new THREE.MeshBasicMaterial({
      color: COLOR_GLOW,
      transparent: true,
      opacity: 0.10,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    return new THREE.Mesh(geo, mat);
  }

  /** 沿路径放置方向箭头（ConeGeometry + 朝向切线方向） */
  function _buildArrows(points, color) {
    const meshes = [];
    if (points.length < 2) return meshes;

    for (let i = ARROW_INTERVAL; i < points.length; i += ARROW_INTERVAL) {
      const p = points[i];
      const pPrev = points[i - 1];
      if (!p || !pPrev) continue;

      // 该段路径方向（切线方向）
      const dx = p[0] - pPrev[0];
      const dy = p[1] - pPrev[1];
      const dz = p[2] - pPrev[2];
      const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (len < 0.01) continue;

      const dir = new THREE.Vector3(dx, dy, dz).normalize();
      const up = new THREE.Vector3(0, 1, 0);

      const geo = new THREE.ConeGeometry(ARROW_RADIUS, ARROW_LENGTH, 6);
      const mat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.85,
        depthWrite: false,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(p[0], p[1], p[2]);

      // 旋转锥体使之指向路径方向（ConeGeometry 默认朝 +Y）
      const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);
      mesh.quaternion.copy(quat);

      meshes.push(mesh);
    }
    return meshes;
  }

  function update(store, now) {
    if (!store.ego) return;

    // 移除旧元素（含几何/材质清理）
    clear();

    const { points, color } = _predictPath(store.ego);
    if (points.length < 2) return;

    // 1. 辉光层（先渲染，在底层）
    const glow = _buildGlowTube(points);
    if (glow) group.add(glow);

    // 2. 主管线（在辉光层之上）
    const tube = _buildMainTube(points, color);
    if (tube) group.add(tube);

    // 3. 方向箭头（在最上层）
    const arrows = _buildArrows(points, color);
    for (const a of arrows) group.add(a);
  }

  return { update, clear };
}