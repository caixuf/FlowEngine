/**
 * vis_deadreckon_extrapolation.test.mjs — DeadReckon 外推数学测试
 *
 * 验证掉头"车屁股横移"修复：_advanceState 位置外推在提供世界系 vx/vy 时
 * 用世界速度（含绕后轴切向分量 half_wb·yaw_rate），而不是只有
 * speed·(cos h, sin h)。
 *
 * 确定性关键：heading=0 时旧公式 targetZ = 0 + sin(0)·speed·t = 0 恒成立；
 * 若喂入 vy≠0（掉头时切向速度），新公式 targetZ = vy·t > 0 —— 横移是否存在
 * 不依赖墙钟精度；再用 targetZ/targetX 比例 ≈ vy/vx 做定量断言。
 */
import {
  _dr,
  initDeadReckon,
  updateDeadReckon,
  tickDeadReckon,
  updateEntityDeadReckon,
  tickEntityDeadReckon,
  getEntitySmooth,
} from '../tools/flowboard/js/vis/core/DeadReckon.js';

let pass = 0, fail = 0;
function ok(name, cond) {
  if (cond) { pass++; console.log('  PASS  ' + name); }
  else      { fail++; console.log('  FAIL  ' + name); }
}

// busy-wait 让墙钟真正前进（tick 内部用 performance.now()）
function waitMs(ms) {
  const start = Date.now();
  while (Date.now() - start < ms) { /* busy-wait */ }
}

console.log('--- 掉头外推：vx/vy 含切向项 → 横向位移存在 ---');
initDeadReckon();
// 模拟掉头：speed=3.5 沿 heading=0（x 向），切向速度 vy=1.2（绕后轴旋转产生，
// 掉头满舵时切向速度可达车速的 ~34%）
updateDeadReckon(0, 0, 3.5, 0, 3.5, 1.2);
waitMs(80);
tickDeadReckon();          // 首次 tick 只校准帧时钟（no-op）
waitMs(80);
tickDeadReckon();          // 真实推进：elapsed ≈ 0.08s
ok('targetX 前向推进 (vx·t>0.1)', _dr.targetX > 0.1);
ok('targetZ 横向推进 (vy·t>0.04，旧公式恒为 0)', _dr.targetZ > 0.04);
const ratio = _dr.targetZ / _dr.targetX;
ok(`横向/前向比例 ≈ vy/vx=0.343 (实际 ${ratio.toFixed(3)})`,
   ratio > 0.25 && ratio < 0.45);
const oldFormulaZ = _dr.lastZ + Math.sin(_dr.lastHeading) * _dr.lastSpeed * 0.08;
ok('旧公式在 heading=0 时横向外推为 0（对比项）', Math.abs(oldFormulaZ) < 1e-9);

console.log('\n--- 回退路径：无 vx/vy → 旧 speed·(cos,sin) 外推 ---');
initDeadReckon();
updateDeadReckon(10, 0, 5, 0);   // 不传 vx/vy（旧 payload / vehicle-only 路径）
waitMs(80);
tickDeadReckon();
waitMs(80);
tickDeadReckon();
ok('无 vx/vy 时 hasVel=false', _dr.hasVel === false);
ok('回退外推 targetX 前向推进', _dr.targetX > 10.1);
ok('回退外推 targetZ 仍为 0（heading=0 直线）', Math.abs(_dr.targetZ - 0) < 1e-9);

console.log('\n--- 一致路径：vx/vy 与 heading 对齐（直线巡航）---');
initDeadReckon();
// 直线巡航：vx=speed·cos(h), vy=speed·sin(h)，h=0.5 rad
const h0 = 0.5, sp0 = 8.0;
updateDeadReckon(0, 0, sp0, h0, sp0 * Math.cos(h0), sp0 * Math.sin(h0));
waitMs(80);
tickDeadReckon();
waitMs(80);
tickDeadReckon();
const expX = sp0 * Math.cos(h0), expZ = sp0 * Math.sin(h0);
const actualRatio = _dr.targetZ / _dr.targetX;
ok(`直线巡航比例 ≈ tan(h) (实际 ${actualRatio.toFixed(3)} vs ${Math.tan(h0).toFixed(3)})`,
   Math.abs(actualRatio - Math.tan(h0)) < 0.1);

console.log('\n--- 去重：重复帧不刷新 lastTime（外推时钟连续）---');
initDeadReckon();
updateDeadReckon(0, 0, 3.5, 0, 3.5, 1.2);
const t0 = _dr.lastTime;
waitMs(30);
updateDeadReckon(0, 0, 3.5, 0, 3.5, 1.2);   // 完全相同 → 去重
ok('重复帧不刷新 lastTime', _dr.lastTime === t0);
waitMs(30);
updateDeadReckon(0, 0, 3.5, 0, 3.5, 1.3);   // vy 变化 → 接受
ok('vy 变化触发更新', _dr.lastVy === 1.3 && _dr.lastTime > t0);

console.log('\n--- NPC 实体路径：转弯外推有横向位移 ---');
initDeadReckon();
updateEntityDeadReckon('car:1', 100, 0, 3.5, 0, 3.5, 1.2);
waitMs(80);
tickEntityDeadReckon(performance.now() / 1000);  // 首帧校准
waitMs(80);
tickEntityDeadReckon(performance.now() / 1000);  // 真实推进
const e = getEntitySmooth('car:1');
ok('NPC 平滑状态可读', e !== null);
ok('NPC 横向位移出现（getEntitySmooth 的 y=横向，旧公式为 0）',
   e !== null && e.y > 0.03);

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
if (fail > 0) process.exit(1);
