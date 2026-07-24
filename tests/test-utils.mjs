/**
 * test-utils.mjs — 测试工具函数（Node.js 测试共享）
 *
 * 提供轻量断言，不依赖任何测试框架。
 */

let _pass = 0, _fail = 0;

export function ok(name, cond) {
  if (cond) { _pass++; console.log('  PASS  ' + name); }
  else      { _fail++; console.log('  FAIL  ' + name); }
}

export function eq(name, actual, expected) {
  const cond = actual === expected;
  if (cond) { _pass++; console.log('  PASS  ' + name); }
  else      { _fail++; console.log('  FAIL  ' + name + '  actual=' + JSON.stringify(actual) + '  expected=' + JSON.stringify(expected)); }
}

export function done() {
  console.log(`\n--- 结果: ${_pass} pass, ${_fail} fail ---`);
  process.exit(_fail > 0 ? 1 : 0);
}

export function reset() {
  _pass = 0;
  _fail = 0;
}

export { _pass as passCount, _fail as failCount };