/**
 * Coord.js — ENU 世界坐标 ↔ THREE 世界系映射
 *
 * Step 3 重构：本文件是 ENU→THREE 转换的单一来源，所有 view 应改用
 * worldToThree(...) 而不是内联手写 [x, z || 0, y] 模式。
 *
 * FlowEngine 仿真用 ENU（x=East, y=North, z=Up elevation）。
 * THREE 默认 y=up，所以映射：world(x,y,z) → three(x, z, -y)。
 *
 * 注意：ENU 和 THREE 都是右手系，[x, z, y] 是反射（det=−1），
 * 会把整个三维世界左右镜像。修正为 [x, z, -y]（det=+1 旋转），
 * 确保中国右侧通行（ego.y<0 是右车道）渲染在屏幕右侧。
 *
 * 用法：
 *   import { worldToThree, headingToRotationY } from '../math/Coord.js';
 *   group.position.set(...worldToThree(ent.x, ent.y, ent.z || 0));
 *   group.rotation.y = headingToRotationY(ent.heading || 0);
 *
 * 注意：sampleEdgeNodes（Curve.js）内部已做 ENU→THREE 交换，所以
 * 经过 sampleEdgeNodes 输出的点已经是 THREE 坐标，不要再调 worldToThree。
 */

/** ENU 世界坐标 (x=East, y=North, z=Up) → THREE (x, z, -y)
 *  @param {number} x  ENU East（前向）
 *  @param {number} y  ENU North（侧向，正值=北=左，负值=南=右）
 *  @param {number} [z=0]  ENU Up（高度）
 *  @returns {[number, number, number]}  [three.x, three.y(up), three.z] */
export function worldToThree(x, y, z = 0) {
  return [x, z, -y];
}

/** THREE.Vector3 版本（少用，优先 worldToThree + spread） */
export function toVec3(x, y, z = 0) {
  return new THREE.Vector3(x, z, -y);
}

/** heading(rad) → THREE rotation.y（绕 Y 轴）。
 *  仿真 heading=0 朝 +X（车头朝 +X），THREE rotation.y=0 朝 +Z。
 *  推导：ENU 速度 (cosθ, sinθ) 经 [x, z, -y] → THREE (cosθ, 0, -sinθ)
 *  恰是 rotation.y = +θ（绕 +Y 轴转 θ 弧度）。
 *  验证：heading=0 → rotation=0 → 模型车头沿 +X（与 [gen_models.py:340] 一致）。 */
export function headingToRotationY(heading) {
  return heading;
}

/**
 * debugCoordMapping — 坐标映射调试工具，打印 ENU→THREE 转换结果
 * 帮助排查高度(z轴)对齐问题。
 * 
 * @param {number} x ENU x (前向)
 * @param {number} y ENU y (侧向)  
 * @param {number} z ENU z (高度)
 * @param {string} label 标签，便于区分不同对象
 */
export function debugCoordMapping(x, y, z, label = '') {
  const three = worldToThree(x, y, z);
  console.log(`[Coord] ${label || 'entity'}: ENU(${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) → THREE(${three[0].toFixed(2)}, ${three[1].toFixed(2)}, ${three[2].toFixed(2)})`);
}