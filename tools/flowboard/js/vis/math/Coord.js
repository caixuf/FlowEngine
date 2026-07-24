/**
 * Coord.js — ENU 世界坐标 ↔ THREE 世界系映射（单一事实源）
 *
 * 本文件是项目中所有坐标/朝向/高度/尺度转换的**唯一合法入口**。
 * 禁止在任何 view 里手写裸 -y 翻转、裸 atan2 求朝向、裸 position.set 配魔法数。
 *
 * FlowEngine 仿真用 ENU（x=East, y=North, z=Up elevation）。
 * THREE 默认 y=up，映射：ENU(x,y,z) → THREE(x, z, -y)。
 *
 * 注意：ENU 和 THREE 都是右手系，[x, z, y] 是反射（det=−1），
 * 会把整个三维世界左右镜像。修正为 [x, z, -y]（det=+1 旋转），
 * 确保中国右侧通行（ego.y<0 是右车道）渲染在屏幕右侧。
 *
 * 约束（可 grep 强制）：
 *   grep -rnE "z:\s*-\(|\.position\.set\(.*Math\.(sin|cos)" js/vis/view/
 *   命中即违规（注释豁免）。所有 view 必须走以下纯函数。
 *
 * 用法：
 *   import { worldToThree, headingToRotationY, directionToRotationY,
 *            forwardENU, offsetAlongNormal, tangentToNormal, placeOnRoad }
 *     from '../math/Coord.js';
 *
 * 注意：sampleEdgeNodes（Curve.js）内部已做 ENU→THREE 交换，所以
 * 经过 sampleEdgeNodes 输出的点已经是 THREE 坐标，不要再调 worldToThree。
 */

// ═══════════════════════════════════════════════════════════
// 位置映射
// ═══════════════════════════════════════════════════════════

/** ENU 世界坐标 → THREE 坐标（唯一转换函数）
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

// ═══════════════════════════════════════════════════════════
// 朝向映射
// ═══════════════════════════════════════════════════════════

/** heading(rad) → THREE rotation.y（绕 Y 轴）。
 *  仿真 heading=0 朝 +X（车头朝 +X），THREE rotation.y=0 时 forward 朝 +X。
 *  推导：ENU 速度 (cosθ, sinθ) 经 [x, z, -y] → THREE (cosθ, 0, -sinθ)
 *  rotation.y = -θ 时 forward = (cos(-θ), 0, sin(-θ)) = (cosθ, 0, -sinθ) ✓
 *  验证：heading=0 → rotation=0 → 模型车头沿 +X（与 gen_models.py 一致）。
 *        heading=π/2(North) → rotation=-π/2 → 车头沿 -Z（ENU North→THREE -Z）✓ */
export function headingToRotationY(heading) {
  return -heading;
}

/** ENU heading → 单位前向向量（在 ENU 坐标系中）
 *  heading=0 → [1, 0]（朝东/+X），heading=π/2 → [0, 1]（朝北/+Y）
 *  @returns {[number, number]} [dx_ENU, dy_ENU] */
export function forwardENU(heading) {
  return [Math.cos(heading), Math.sin(heading)];
}

/** THREE 空间中的 2D 方向向量 → rotation.y
 *  替代裸 atan2(dx, dz)，统一计算旋转角。
 *  THREE 中 Y 轴向上，rotation.y=0 时物体朝向 +X，rotation.y=π/2 时朝向 +Z。
 *  推导：rotation.y = θ 时 forward 方向 = (cosθ, 0, sinθ)，故 θ = atan2(dz, dx)。
 *  @param {number} dx  THREE X 分量
 *  @param {number} dz  THREE Z 分量
 *  @returns {number} rotation.y（弧度） */
export function directionToRotationY(dx, dz) {
  return Math.atan2(dz, dx);
}

// ═══════════════════════════════════════════════════════════
// 道路相关辅助
// ═══════════════════════════════════════════════════════════

/** 沿法线方向偏移（THREE 空间）
 *  用于道路中心线 spine 的横向偏移（路灯、护栏、车道线等）。
 *  @param {number} px     中心线点 X（THREE 坐标）
 *  @param {number} pz     中心线点 Z（THREE 坐标）
 *  @param {number} nx     法线 X 分量（THREE 空间）
 *  @param {number} nz     法线 Z 分量（THREE 空间）
 *  @param {number} offset 偏移量（正值=法线方向，负值=反法线方向）
 *  @returns {[number, number, number]} [px+offset*nx, py(0), pz+offset*nz] */
export function offsetAlongNormal(px, pz, nx, nz, offset) {
  return [px + nx * offset, 0, pz + nz * offset];
}

/** 从道路样点切线计算法线（THREE 空间，XZ 平面法线）
 *  用于构建 spine 的 {nx, nz} 字段。
 *  @param {number} tx  切线 X 分量
 *  @param {number} tz  切线 Z 分量
 *  @returns {[number, number]} [nx, nz]（单位法线，指向切向右侧） */
export function tangentToNormal(tx, tz) {
  const l = Math.sqrt(tx * tx + tz * tz) || 1;
  return [-tz / l, tx / l];
}

/** 沿路参数 s + 横向偏移 → THREE 位置 + 朝向 + 高度
 *  从预建道路 spine 插值，统一"路上物体"的放置逻辑。
 *  @param {Array}  spine 道路中心线样点 [{px,py,pz,nx,nz,cum}]
 *  @param {number} s     沿路弧长
 *  @param {number} [lateralOffset=0] 横向偏移（正值=法线方向）
 *  @returns {{pos:[number,number,number], rotY:number, height:number}|null} */
export function placeOnRoad(spine, s, lateralOffset = 0) {
  if (!spine || spine.length < 2) return null;
  let j = 1;
  while (j < spine.length && spine[j].cum < s) j++;
  if (j >= spine.length) j = spine.length - 1;
  const a = spine[j - 1], b = spine[j];
  const segLen = b.cum - a.cum || 1;
  const t = (s - a.cum) / segLen;
  const px = a.px + (b.px - a.px) * t;
  const py = a.py + (b.py - a.py) * t;
  const pz = a.pz + (b.pz - a.pz) * t;
  const nx = a.nx + (b.nx - a.nx) * t;
  const nz = a.nz + (b.nz - a.nz) * t;
  const [ox, , oz] = offsetAlongNormal(px, pz, nx, nz, lateralOffset);
  const tx = b.px - a.px, tz = b.pz - a.pz;
  const rotY = directionToRotationY(tx, tz);
  return { pos: [ox, py, oz], rotY, height: py };
}

// ═══════════════════════════════════════════════════════════
// 调试
// ═══════════════════════════════════════════════════════════

/** 打印 ENU→THREE 转换结果，帮助排查高度对齐问题 */
export function debugCoordMapping(x, y, z, label = '') {
  const three = worldToThree(x, y, z);
  console.log(`[Coord] ${label || 'entity'}: ENU(${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}) → THREE(${three[0].toFixed(2)}, ${three[1].toFixed(2)}, ${three[2].toFixed(2)})`);
}