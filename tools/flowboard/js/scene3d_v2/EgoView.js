/**
 * EgoView.js — v2 ego 车构建与每帧更新
 *
 * 简化策略：
 *   - 复用 v1 models.js 的 buildEgoCar（SU7 glTF 或 sedan 程序化几何）
 *   - 不做车漆金属反射（PMREMGenerator 砍掉）、不做车轮滚动动画
 *   - 不做车身侧倾/俯仰（VisualPhysics 砍掉）
 *   - 不做车灯（_setVehicleLights 砍掉）
 *
 * 坐标系：ego.position = (x, elevation, z) = (forward, up, lateral)
 *   朝向：rotation.y = -heading（与 v1 一致，让车头朝 +X 前向）
 */
const THREE = window.THREE;
import { buildEgoCar } from '../models.js';
import { getRoadElevationAt } from './RoadBuilder.js';

/**
 * 构建 ego 车并挂到 scene。
 * @returns {THREE.Group} ego group
 */
export function buildEgo(scene) {
  var ego = buildEgoCar(0x4488dd);
  if (!ego) {
    // 兜底：简单 box
    ego = new THREE.Group();
    var bodyGeo = new THREE.BoxGeometry(4.6, 1.4, 2.0);
    var bodyMat = new THREE.MeshStandardMaterial({ color: 0x4488dd, roughness: 0.5, metalness: 0.3 });
    var body = new THREE.Mesh(bodyGeo, bodyMat);
    body.position.y = 0.7;
    ego.add(body);
  }
  ego.name = 'ego';
  scene.add(ego);
  return ego;
}

/**
 * 每帧更新 ego 位置/朝向。
 * @param {THREE.Group} ego
 * @param {number} x        平滑后 ego x（forward）
 * @param {number} z        平滑后 ego z（lateral）
 * @param {number} heading  平滑后 ego heading（弧度，0 = +X 方向）
 * @param {Array} curves    道路曲线数组（算 elevation 用）
 */
export function updateEgo(ego, x, z, heading, curves) {
  if (!ego) return;
  var elev = getRoadElevationAt(x, z, curves);
  ego.position.set(x, elev, z);
  ego.rotation.y = -heading;  // 与 v1 一致：车头朝 +X
}
