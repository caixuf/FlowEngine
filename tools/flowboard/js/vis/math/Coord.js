/**
 * Coord.js — ENU 世界坐标 ↔ THREE 世界系映射
 *
 * Step 3 重构：本文件是 ENU→THREE 转换的单一来源，所有 view 应改用
 * worldToThree(...) 而不是内联手写 [x, z || 0, y] 模式。
 *
 * FlowEngine 仿真用 ENU（x=East, y=North, z=Up elevation）。
 * THREE 默认 y=up，所以映射：world(x,y,z) → three(x, z, y)。
 *
 * 用法：
 *   import { worldToThree, headingToRotationY } from '../math/Coord.js';
 *   group.position.set(...worldToThree(ent.x, ent.y, ent.z || 0));
 *   group.rotation.y = headingToRotationY(ent.heading || 0);
 *
 * 注意：sampleEdgeNodes（Curve.js）内部已做 ENU→THREE 交换，所以
 * 经过 sampleEdgeNodes 输出的点已经是 THREE 坐标，不要再调 worldToThree。
 */

/** ENU 世界坐标 (x=East, y=North, z=Up) → THREE (x, z, y)
 *  @param {number} x  ENU East（前向）
 *  @param {number} y  ENU North（侧向）
 *  @param {number} [z=0]  ENU Up（高度）
 *  @returns {[number, number, number]}  [three.x, three.y(up), three.z] */
export function worldToThree(x, y, z = 0) {
  return [x, z, y];
}

/** THREE.Vector3 版本（少用，优先 worldToThree + spread） */
export function toVec3(x, y, z = 0) {
  return new THREE.Vector3(x, z, y);
}

/** heading(rad) → THREE rotation.y（绕 Y 轴）。
 *  仿真 heading=0 朝 +X（车头朝 +X），THREE rotation.y=0 朝 +Z。
 *  旋转方向约定：右手系，绕 +Y 顺时针为负。
 *  验证：heading=0 → rotation=0 → 模型车头沿 +X（与 [gen_models.py:340] 一致）。
 *  之前 v1/v2 都用 `rotation.y = -heading`，重构时误加 `-Math.PI/2` 致车横在路中央。
 *  修复：恢复 `-heading` 单项。 */
export function headingToRotationY(heading) {
  return -heading;
}
