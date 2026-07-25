/**
 * vis_deadreckon_config.test.mjs — DeadReckon 运行时配置测试
 *
 * 验证 P1 修复：LAMBDA_POS / LAMBDA_HEADING 从顶层硬编码 var 改为
 * _cfg 对象，可通过 setDeadReckonConfig() 运行时调整，且立即生效于
 * 后续 tickDeadReckon() 调用。
 *
 * 默认值与原实现一致（lambdaPos=8, lambdaHeading=6），保证现有行为兼容。
 */
import {
  _dr,
  _cfg,
  initDeadReckon,
  updateDeadReckon,
  tickDeadReckon,
  setDeadReckonConfig,
  getDeadReckonConfig,
} from '../tools/flowboard/js/vis/core/DeadReckon.js';

let pass = 0, fail = 0;
function ok(name, cond) {
  if (cond) { pass++; console.log('  PASS  ' + name); }
  else      { fail++; console.log('  FAIL  ' + name); }
}

console.log('--- 默认配置（兼容原硬编码值）---');
initDeadReckon();
const def = getDeadReckonConfig();
ok('默认 lambdaPos=8', def.lambdaPos === 8);
ok('默认 lambdaHeading=6', def.lambdaHeading === 6);

console.log('\n--- setDeadReckonConfig：合法值生效 ---');
setDeadReckonConfig({ lambdaPos: 12.0, lambdaHeading: 9.0 });
const after = getDeadReckonConfig();
ok('lambdaPos=12 生效', after.lambdaPos === 12);
ok('lambdaHeading=9 生效', after.lambdaHeading === 9);

console.log('\n--- setDeadReckonConfig：非法值被忽略 ---');
setDeadReckonConfig({ lambdaPos: -1, lambdaHeading: 0 });
const afterBad = getDeadReckonConfig();
ok('lambdaPos=-1 被忽略，仍=12', afterBad.lambdaPos === 12);
ok('lambdaHeading=0 被忽略，仍=9', afterBad.lambdaHeading === 9);

setDeadReckonConfig(null);
setDeadReckonConfig('not-an-object');
ok('null / 非 object 不抛错、不改值', getDeadReckonConfig().lambdaPos === 12);

console.log('\n--- setDeadReckonConfig：单字段更新不覆盖另一字段 ---');
setDeadReckonConfig({ lambdaPos: 20 });
const partial = getDeadReckonConfig();
ok('lambdaPos=20 更新', partial.lambdaPos === 20);
ok('lambdaHeading 保持 9（未被 undefined 覆盖）', partial.lambdaHeading === 9);

console.log('\n--- 配置立即影响 tickDeadReckon 平滑速率 ---');
/* 思路：updateDeadReckon 首次采样会把 smooth snap 到真值，所以单纯喂一个新位置
 * 无法观察平滑。改用速度外推路径：
 *   - updateDeadReckon(x=0, z=0, speed=5, heading=0) → first sample snap smoothX=0
 *   - tickDeadReckon 推进 dt 后 targetX = 0 + 5*dt = 5*dt，smooth 向 target 逼近
 *
 * 大 lambda → smoothX 几乎追上 targetX；小 lambda → smoothX 远落后 targetX。
 * 比较的不是 smoothX 的绝对值，而是"是否追上 targetX"的差距。
 */
initDeadReckon();

// 大 lambda（≈瞬贴）
setDeadReckonConfig({ lambdaPos: 100, lambdaHeading: 100 });
updateDeadReckon(0, 0, 5, 0);   // 起始位置 (0,0)，速度 5m/s 沿 x
// 推进若干帧（busy-wait ~50ms 模拟 20Hz）
for (let i = 0; i < 5; i++) {
  const start = Date.now();
  while (Date.now() - start < 50) { /* busy-wait */ }
  tickDeadReckon();
}
// 大 lambda 下 smoothX 应接近 targetX（差距 < 0.5m）
const bigDiff = Math.abs(_dr.targetX - _dr.smoothX);
ok('大 lambda 下 smoothX 追上 targetX (|targetX-smoothX|<0.5)', bigDiff < 0.5);

// 重置 + 小 lambda（≈不动）
initDeadReckon();
setDeadReckonConfig({ lambdaPos: 0.1, lambdaHeading: 0.1 });
updateDeadReckon(0, 0, 5, 0);
for (let i = 0; i < 5; i++) {
  const start = Date.now();
  while (Date.now() - start < 50) { /* busy-wait */ }
  tickDeadReckon();
}
// 小 lambda 下 smoothX 应远未追上 targetX（落后 > 0.5m）
const smallDiff = Math.abs(_dr.targetX - _dr.smoothX);
ok('小 lambda 下 smoothX 远落后 targetX (|targetX-smoothX|>0.5)', smallDiff > 0.5);

console.log('\n--- 复位默认配置（避免污染其他测试）---');
setDeadReckonConfig({ lambdaPos: 8, lambdaHeading: 6 });
const restored = getDeadReckonConfig();
ok('恢复默认 lambdaPos=8', restored.lambdaPos === 8);
ok('恢复默认 lambdaHeading=6', restored.lambdaHeading === 6);

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
if (fail > 0) process.exit(1);
