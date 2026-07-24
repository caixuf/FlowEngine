/**
 * vis_coord_property.test.mjs — Coord.js 纯函数 property-test
 *
 * 验证所有坐标/朝向/高度转换函数的数学正确性。
 * 错一个符号 = 红，不给任何模糊空间。
 *
 * 跑法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_coord_property.test.mjs
 */

import { worldToThree, headingToRotationY, forwardENU,
         directionToRotationY, offsetAlongNormal, tangentToNormal,
         placeOnRoad }
  from '../tools/flowboard/js/vis/math/Coord.js';
import { ok, eq, done } from './test-utils.mjs';

console.log('=== Coord.js 纯函数 property-test ===\n');

// ═══════════════════════════════════════════════════════════
// 1. worldToThree — ENU→THREE 轴映射 golden 表
// ═══════════════════════════════════════════════════════════

console.log('--- 1. worldToThree 轴映射 golden ---');

// ENU +x (East) → THREE +x
{
  const [tx, ty, tz] = worldToThree(1, 0, 0);
  eq('ENU +x → THREE +x', tx, 1);
  eq('ENU +x → THREE y=0', ty, 0);
  eq('ENU +x → THREE -z=0', tz, 0);
}

// ENU +y (North) → THREE -z
{
  const [tx, ty, tz] = worldToThree(0, 1, 0);
  eq('ENU +y → THREE -z', tz, -1);
  eq('ENU +y → THREE x=0', tx, 0);
}

// ENU -y (South) → THREE +z
{
  const [tx, ty, tz] = worldToThree(0, -1, 0);
  eq('ENU -y → THREE +z', tz, 1);
  ok('ENU -y → THREE +z(正)', tz > 0);
}

// ENU +z (Up) → THREE +y
{
  const [tx, ty, tz] = worldToThree(0, 0, 7);
  eq('ENU +z → THREE +y', ty, 7);
  eq('ENU +z → THREE x=0', tx, 0);
  eq('ENU +z → THREE z=0', tz, 0);
}

// ENU -z (Down) → THREE -y
{
  const [tx, ty, tz] = worldToThree(0, 0, -3);
  eq('ENU -z → THREE -y', ty, -3);
}

// 零向量
{
  const [tx, ty, tz] = worldToThree(0, 0, 0);
  eq('零向量 x=0', tx, 0);
  eq('零向量 y=0', ty, 0);
  eq('零向量 z=0', tz, 0);
}

// 典型值：ego 在 (100, -3.5, 0) → 右车道
{
  const [tx, ty, tz] = worldToThree(100, -3.5, 0);
  eq('ego 右车道 x', tx, 100);
  eq('ego 右车道 y', ty, 0);
  ok('ego 右车道 z>0(右侧)', tz > 0);  // -(-3.5) = 3.5
}

// 高架：ego 在 (500, 0, 7.0)
{
  const [tx, ty, tz] = worldToThree(500, 0, 7.0);
  eq('高架 x', tx, 500);
  eq('高架 y(height)', ty, 7.0);
  eq('高架 z', tz, 0);
}

// ═══════════════════════════════════════════════════════════
// 2. headingToRotationY — 朝向映射
// ═══════════════════════════════════════════════════════════

console.log('--- 2. headingToRotationY ---');

// heading=0 → 车头朝 +X，rotationY=0
eq('heading=0 → rotY=0', headingToRotationY(0), 0);

// heading=π/2 → 车头朝 +Y(North)→THREE -Z，rotationY=-π/2
eq('heading=π/2 → rotY=-π/2', headingToRotationY(Math.PI / 2), -Math.PI / 2);

// heading=-π/2 → 车头朝 -Y(South)→THREE +Z，rotationY=+π/2
eq('heading=-π/2 → rotY=+π/2', headingToRotationY(-Math.PI / 2), Math.PI / 2);

// heading=π → 车头朝 -X，rotationY=-π（或 π，等价）
ok('heading=π → |rotY|=π', Math.abs(Math.abs(headingToRotationY(Math.PI)) - Math.PI) < 1e-10);

// ═══════════════════════════════════════════════════════════
// 3. forwardENU — heading → 单位向量
// ═══════════════════════════════════════════════════════════

console.log('--- 3. forwardENU ---');

// heading=0 → [1, 0]
{
  const [fx, fy] = forwardENU(0);
  eq('heading=0 → fx=1', fx, 1);
  ok('heading=0 → fy≈0', Math.abs(fy) < 1e-10);
}

// heading=π/2 → [0, 1]
{
  const [fx, fy] = forwardENU(Math.PI / 2);
  ok('heading=π/2 → fx≈0', Math.abs(fx) < 1e-10);
  ok('heading=π/2 → fy≈1', Math.abs(fy - 1) < 1e-10);
}

// heading=π → [-1, 0]
{
  const [fx, fy] = forwardENU(Math.PI);
  ok('heading=π → fx≈-1', Math.abs(fx + 1) < 1e-10);
  ok('heading=π → fy≈0', Math.abs(fy) < 1e-10);
}

// heading=-π/2 → [0, -1]
{
  const [fx, fy] = forwardENU(-Math.PI / 2);
  ok('heading=-π/2 → fx≈0', Math.abs(fx) < 1e-10);
  ok('heading=-π/2 → fy≈-1', Math.abs(fy + 1) < 1e-10);
}

// 单位向量长度=1
for (const h of [0, 0.5, 1.0, 1.5, Math.PI, -0.7, 2.3]) {
  const [fx, fy] = forwardENU(h);
  const len = Math.sqrt(fx * fx + fy * fy);
  ok(`forwardENU(${h.toFixed(1)}) 单位长度`, Math.abs(len - 1) < 1e-10);
}

// ═══════════════════════════════════════════════════════════
// 4. directionToRotationY — 2D 方向 → rotation.y
// ═══════════════════════════════════════════════════════════

console.log('--- 4. directionToRotationY ---');

// +X 方向 → rotationY=0
eq('+X dir → rotY=0', directionToRotationY(1, 0), 0);

// +Z 方向 → rotationY=π/2
ok('+Z dir → rotY≈π/2', Math.abs(directionToRotationY(0, 1) - Math.PI / 2) < 1e-10);

// -X 方向 → rotationY=π
ok('-X dir → rotY≈π', Math.abs(Math.abs(directionToRotationY(-1, 0)) - Math.PI) < 1e-10);

// -Z 方向 → rotationY=-π/2
ok('-Z dir → rotY≈-π/2', Math.abs(directionToRotationY(0, -1) + Math.PI / 2) < 1e-10);

// forwardENU 与 headingToRotationY 一致性：
// forwardENU 在 ENU 空间，但 directionToRotationY 在 THREE 空间。
// ENU heading → forwardENU → worldToThree → 方向在 THREE 中 →
// directionToRotationY 应该等于 headingToRotationY
for (const h of [0, 0.3, 0.7, 1.2, Math.PI / 2, -0.5, Math.PI]) {
  const [fex, fey] = forwardENU(h);
  const [tdx, _, tdz] = worldToThree(fex, fey, 0);
  const rotY = directionToRotationY(tdx, tdz);
  const expected = headingToRotationY(h);
  ok(`forwardENU→worldToThree→directionToRotationY 与 headingToRotationY 一致 (h=${h.toFixed(2)})`,
     Math.abs(rotY - expected) < 1e-10);
}

// ═══════════════════════════════════════════════════════════
// 5. offsetAlongNormal — 法线偏移
// ═══════════════════════════════════════════════════════════

console.log('--- 5. offsetAlongNormal ---');

// 零偏移
{
  const [ox, oy, oz] = offsetAlongNormal(10, 5, 0, 1, 0);
  eq('零偏移 x', ox, 10);
  eq('零偏移 z', oz, 5);
}

// 正偏移
{
  const [ox, oy, oz] = offsetAlongNormal(10, 5, 1, 0, 3);
  eq('+X法线偏移 x', ox, 13);
  eq('+X法线偏移 z', oz, 5);
}

// 负偏移
{
  const [ox, oy, oz] = offsetAlongNormal(10, 5, 0, 1, -2);
  eq('-Z法线偏移 x', ox, 10);
  eq('-Z法线偏移 z', oz, 3);
}

// 非单位法线
{
  const [ox, oy, oz] = offsetAlongNormal(0, 0, 0.6, 0.8, 5);
  ok('非单位法线偏移 x', Math.abs(ox - 3) < 1e-10);
  ok('非单位法线偏移 z', Math.abs(oz - 4) < 1e-10);
}

// ═══════════════════════════════════════════════════════════
// 6. tangentToNormal — 切线 → 法线
// ═══════════════════════════════════════════════════════════

console.log('--- 6. tangentToNormal ---');

// +X 切线 → 法线指向 +Z（右侧）
{
  const [nx, nz] = tangentToNormal(1, 0);
  ok('+X 切线 → nx≈0', Math.abs(nx) < 1e-10);
  ok('+X 切线 → nz≈1', Math.abs(nz - 1) < 1e-10);
}

// +Z 切线 → 法线指向 -X（右侧）
{
  const [nx, nz] = tangentToNormal(0, 1);
  ok('+Z 切线 → nx≈-1', Math.abs(nx + 1) < 1e-10);
  ok('+Z 切线 → nz≈0', Math.abs(nz) < 1e-10);
}

// 法线是单位向量
for (const [tx, tz] of [[1, 0], [0, 1], [3, 4], [-2, 5], [0.7, 0.7]]) {
  const [nx, nz] = tangentToNormal(tx, tz);
  const len = Math.sqrt(nx * nx + nz * nz);
  ok(`tangentToNormal(${tx}, ${tz}) 单位长度`, Math.abs(len - 1) < 1e-10);
}

// 法线与切线点积=0（正交）
for (const [tx, tz] of [[1, 0], [3, 4], [-2, 5], [0.7, 0.7]]) {
  const [nx, nz] = tangentToNormal(tx, tz);
  const dot = tx * nx + tz * nz;
  ok(`tangentToNormal(${tx}, ${tz}) 正交`, Math.abs(dot) < 1e-10);
}

// ═══════════════════════════════════════════════════════════
// 7. placeOnRoad — 沿路参数 s + 横向偏移 → 位置 + 朝向 + 高度
// ═══════════════════════════════════════════════════════════

console.log('--- 7. placeOnRoad ---');

// 直道 spine：沿 X 轴，从 x=0 到 x=100，无高度
{
  const spine = [
    { px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 },
    { px: 50, py: 0, pz: 0, nx: 0, nz: 1, cum: 50 },
    { px: 100, py: 0, pz: 0, nx: 0, nz: 1, cum: 100 },
  ];
  const r = placeOnRoad(spine, 50, 0);
  ok('placeOnRoad 直道中点', r !== null);
  if (r) {
    eq('直道中点 x', r.pos[0], 50);
    eq('直道中点 y', r.pos[1], 0);
    eq('直道中点 z', r.pos[2], 0);
    eq('直道中点 rotY≈0', r.rotY, 0);
    eq('直道中点 height', r.height, 0);
  }
}

// 直道 + 横向偏移
{
  const spine = [
    { px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 },
    { px: 100, py: 0, pz: 0, nx: 0, nz: 1, cum: 100 },
  ];
  const r = placeOnRoad(spine, 50, 3.5);
  ok('placeOnRoad 横向偏移', r !== null);
  if (r) {
    eq('横向偏移 x', r.pos[0], 50);
    ok('横向偏移 z≈3.5', Math.abs(r.pos[2] - 3.5) < 1e-10);
  }
}

// 直道 + 负横向偏移（左侧）
{
  const spine = [
    { px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 },
    { px: 100, py: 0, pz: 0, nx: 0, nz: 1, cum: 100 },
  ];
  const r = placeOnRoad(spine, 25, -3.5);
  ok('placeOnRoad 负横向偏移', r !== null);
  if (r) {
    eq('负横向偏移 x', r.pos[0], 25);
    ok('负横向偏移 z≈-3.5', Math.abs(r.pos[2] + 3.5) < 1e-10);
  }
}

// 边界 s=0
{
  const spine = [
    { px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 },
    { px: 100, py: 0, pz: 0, nx: 0, nz: 1, cum: 100 },
  ];
  const r = placeOnRoad(spine, 0, 0);
  ok('placeOnRoad s=0', r !== null);
  if (r) {
    eq('s=0 x', r.pos[0], 0);
    eq('s=0 z', r.pos[2], 0);
  }
}

// 边界 s=total
{
  const spine = [
    { px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 },
    { px: 100, py: 0, pz: 0, nx: 0, nz: 1, cum: 100 },
  ];
  const r = placeOnRoad(spine, 100, 0);
  ok('placeOnRoad s=total', r !== null);
  if (r) {
    eq('s=total x', r.pos[0], 100);
    eq('s=total z', r.pos[2], 0);
  }
}

// 空 spine
{
  const r = placeOnRoad([], 0, 0);
  eq('空 spine → null', r, null);
}

// 单点 spine
{
  const r = placeOnRoad([{ px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 }], 0, 0);
  eq('单点 spine → null', r, null);
}

// 插值：两点中间
{
  const spine = [
    { px: 0, py: 0, pz: 0, nx: 0, nz: 1, cum: 0 },
    { px: 100, py: 7, pz: 0, nx: 0, nz: 1, cum: 100 },
  ];
  const r = placeOnRoad(spine, 50, 0);
  ok('插值高度', r !== null);
  if (r) {
    ok('插值高度≈3.5', Math.abs(r.height - 3.5) < 1e-10);
    ok('插值位置 y≈3.5', Math.abs(r.pos[1] - 3.5) < 1e-10);
  }
}

done();