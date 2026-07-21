/**
 * Coord.js — ENU 世界坐标 ↔ THREE 世界系映射
 * FlowEngine 仿真用 ENU（x=East, y=North, z=Up elevation）。
 * THREE 默认 y=up，所以映射：world(x,y,z) → three(x, z, y)。
 */
/** ENU 世界坐标 (x=East, y=North, z=Up) → THREE (x, z, y)
 *  @deprecated 当前 view 模块各自内联手写映射，这个 helper 是死代码。
 *  下次重构时让所有 view 改用本函数，并删除内联的 [x, z || 0, y] 模式。
 *  不要在现有 view 里引用此函数。 */
export function worldToThree(x, y, z = 0) {
  return [x, z, y];
}

/** THREE.Vector3 版本
 *  @deprecated 同 worldToThree，未被任何 view 引用。 */
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
