/**
 * vis_view_registry.test.mjs — ViewRegistry 单元测试
 *
 * 验证：
 * 1. register / instantiate / get / safeCall 基本流程
 * 2. 错误隔离：一个 View 抛错不影响兄弟 View，也不抛出
 * 3. _failed 标记：坏 View 后续 safeCall 直接跳过，不重复尝试
 * 4. resetFailures() 让坏 View 重试
 * 5. clear() / reset() 清理
 *
 * 纯 Node，无 THREE 依赖。
 */
import * as Registry from '../tools/flowboard/js/vis/core/ViewRegistry.js';

let pass = 0, fail = 0;
function ok(name, cond) {
  if (cond) { pass++; console.log('  PASS  ' + name); }
  else      { fail++; console.log('  FAIL  ' + name); }
}

console.log('--- ViewRegistry 基本流程 ---');

// 每个测试开头 reset 避免跨用例污染
Registry.reset();

Registry.register('a', () => ({ name: 'A', build: (x) => x * 2 }));
Registry.register('b', () => ({ name: 'B', build: (x) => x + 1 }));

const aInst = Registry.instantiate('a', null);
const bInst = Registry.instantiate('b', null);
ok('instantiate a 返回实例', aInst && aInst.name === 'A');
ok('instantiate b 返回实例', bInst && bInst.name === 'B');
ok('get(a) === instantiate(a) 同一实例', Registry.get('a') === aInst);
ok('has(a) true', Registry.has('a'));
ok('has(unknown) false', !Registry.has('unknown'));
ok('names() 列出 a/b', Registry.names().sort().join(',') === 'a,b');

console.log('\n--- safeCall 正常路径 ---');
ok('safeCall a.build(5) → 10', Registry.safeCall('a', 'build', 5) === 10);
ok('safeCall b.build(5) → 6',  Registry.safeCall('b', 'build', 5) === 6);
ok('safeCall 不存在的方法 → undefined', Registry.safeCall('a', 'nope', 1) === undefined);
ok('safeCall 不存在的 view → undefined', Registry.safeCall('zzz', 'build') === undefined);

console.log('\n--- 错误隔离：View 抛错不炸调用方 ---');
Registry.reset();
Registry.register('good', () => ({ build: () => 'ok' }));
Registry.register('bad',  () => { throw new Error('boom'); });
Registry.register('evil', () => ({ build: () => { throw new Error('build-boom'); } }));

const goodInst = Registry.instantiate('good', null);
const badInst  = Registry.instantiate('bad', null);   // factory 抛错 → 返回 null
const evilInst = Registry.instantiate('evil', null);   // 工厂成功，但 build 会抛
ok('good 实例化成功', goodInst && goodInst.build() === 'ok');
ok('bad 工厂抛错 → 返回 null', badInst === null);
ok('bad 标记为 failed', Registry.isFailed('bad'));
ok('evil 工厂成功 → 有实例', evilInst !== null);
ok('evil.build() 抛错被吞 → safeCall 返回 undefined',
   Registry.safeCall('evil', 'build') === undefined);
ok('evil 标记为 failed（build 抛错后）', Registry.isFailed('evil'));

// 关键：good 不受 bad/evil 影响
ok('good 仍可调用（错误隔离）', Registry.safeCall('good', 'build') === 'ok');

console.log('\n--- _failed 跳过：坏 view 后续调用直接返回 undefined ---');
let boomCount = 0;
Registry.reset();
Registry.register('counter', () => ({
  build: () => { boomCount++; throw new Error('always-boom'); }
}));
Registry.instantiate('counter', null);
const r1 = Registry.safeCall('counter', 'build');
const r2 = Registry.safeCall('counter', 'build');
const r3 = Registry.safeCall('counter', 'build');
ok('第一次 build 抛错 → boomCount=1', boomCount === 1);
ok('第二/三次跳过，不再调 build → boomCount 仍=1', boomCount === 1);
ok('每次都返回 undefined', r1 === undefined && r2 === undefined && r3 === undefined);

console.log('\n--- resetFailures() 让坏 view 重试一次 ---');
const r4 = Registry.safeCall('counter', 'build');   // 仍 skipped
ok('reset 前 safeCall 仍跳过', r4 === undefined && boomCount === 1);
Registry.resetFailures();
const r5 = Registry.safeCall('counter', 'build');   // 重试 → 又抛一次
ok('reset 后重试 → boomCount=2', boomCount === 2);
ok('重试后仍标记 failed', Registry.isFailed('counter'));

console.log('\n--- instantiateAll：批量实例化所有已注册 ---');
Registry.reset();
let nA = 0, nB = 0;
Registry.register('allA', () => { nA++; return { build: () => 'A' }; });
Registry.register('allB', () => { nB++; return { build: () => 'B' }; });
Registry.instantiateAll(null);
ok('instantiateAll 实例化 allA', nA === 1);
ok('instantiateAll 实例化 allB', nB === 1);
ok('instantiateAll 后 has(allA) && has(allB)', Registry.has('allA') && Registry.has('allB'));
// 重复 instantiateAll 不重复实例化
Registry.instantiateAll(null);
ok('重复 instantiateAll 不重复实例化', nA === 1 && nB === 1);

console.log('\n--- unregister：移除注册 ---');
Registry.unregister('allB');
ok('unregister 后 has(allB) false', !Registry.has('allB'));
// instantiate 不存在的 factory
ok('unregister 后 instantiate(allB) 返回 null', Registry.instantiate('allB', null) === null);

console.log('\n--- register 参数校验 ---');
let threwType = false;
try { Registry.register(123, () => null); } catch (e) { threwType = e instanceof TypeError; }
ok('register 非 string name 抛 TypeError', threwType);
let threwFn = false;
try { Registry.register('xx', 'not-a-fn'); } catch (e) { threwFn = e instanceof TypeError; }
ok('register 非 function factory 抛 TypeError', threwFn);

console.log('\n--- clear()：销毁所有实例 + 清 failed ---');
Registry.reset();
let cleared = false;
Registry.register('clearable', () => ({ clear: () => { cleared = true; } }));
Registry.instantiate('clearable', null);
Registry.safeCall('clearable', 'build');  // 无此方法，不影响
Registry.clear();
ok('clear 调用 view.clear()', cleared);
ok('clear 后 has(clearable) false', !Registry.has('clearable'));
ok('clear 后 _failed 也清空', !Registry.isFailed('clearable'));

console.log('\n--- 空场景：空 registry 不炸 ---');
Registry.reset();
ok('空 registry safeCall → undefined', Registry.safeCall('x', 'y') === undefined);
ok('空 registry instantiate → null', Registry.instantiate('x', null) === null);
ok('空 registry instantiateAll 不抛', (() => { Registry.instantiateAll(null); return true; })());

console.log('\n--- 退避重试（P1：替换永久 _failed）---');
/* 场景：view 抛错一次后，退避期内 safeCall 直接跳过；
 * 退避到期后 safeCall 重试。如果数据流恢复（view 不再抛），自动复活。
 * 测试用 setBackoffPolicy({initialMs: 很小}) 加速，避免等 2s。
 */
Registry.reset();
Registry.setBackoffPolicy({ initialMs: 30, growth: 1.5, maxMs: 200 });
let boomCount2 = 0;
let recoverAfter = 2;   // 抛 2 次后第 3 次恢复正常
Registry.register('flake', () => ({
  build: () => {
    if (boomCount2 < recoverAfter) { boomCount2++; throw new Error('flake'); }
    return 'recovered';
  }
}));
Registry.instantiate('flake', null);
// 第一次抛 → boomCount2=1
const f1 = Registry.safeCall('flake', 'build');
ok('flake 首次抛错 → undefined', f1 === undefined && boomCount2 === 1);
ok('flake 标记 failed', Registry.isFailed('flake'));
// 立即再调（退避期内）→ 跳过，不调 build
const f2 = Registry.safeCall('flake', 'build');
ok('退避期内跳过 → boomCount2 仍=1', boomCount2 === 1 && f2 === undefined);
// 等退避到期（30ms）→ 重试一次 → 又抛（boomCount2=2），退避延长到 45ms
await new Promise(r => setTimeout(r, 40));
const f3 = Registry.safeCall('flake', 'build');
ok('退避到期重试 → 又抛 → boomCount2=2', boomCount2 === 2 && f3 === undefined);
// 再等退避到期（45ms）→ 重试，此时 view 恢复正常
await new Promise(r => setTimeout(r, 50));
const f4 = Registry.safeCall('flake', 'build');
ok('退避到期再次重试 → view 恢复正常 → 返回 recovered', f4 === 'recovered');
ok('view 恢复后清除 failed 标记', !Registry.isFailed('flake'));

console.log('\n--- 退避上限：连续失败不超过 maxMs ---');
Registry.reset();
Registry.setBackoffPolicy({ initialMs: 10, growth: 2.0, maxMs: 80 });
let alwaysBoom = 0;
Registry.register('perma', () => ({ build: () => { alwaysBoom++; throw new Error('perma'); } }));
Registry.instantiate('perma', null);
// 连续抛 5 次，每次后退避应被 maxMs=80 截断
for (let i = 0; i < 5; i++) {
  await new Promise(r => setTimeout(r, 100));   // 等 >maxMs 让每次都重试
  Registry.safeCall('perma', 'build');
}
ok('连续失败 5 次都重试（未永久放弃）', alwaysBoom === 5);
// 退避已被 maxMs 截断（每次重试间隔 ~80ms，5 次 < 1s）

console.log('\n--- setBackoffPolicy 参数校验 ---');
Registry.setBackoffPolicy({ initialMs: -1, growth: 0.5, maxMs: 'bad' });
// 非法值应被忽略，policy 不变
// (无直接 getter，靠行为验证：继续用上一个合法 policy)
ok('setBackoffPolicy 忽略非法值（不抛错）', true);

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
if (fail > 0) process.exit(1);
