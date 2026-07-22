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

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
if (fail > 0) process.exit(1);
