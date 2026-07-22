/**
 * vis_layer.test.mjs — Layer 对象树单元测试
 *
 * 验证 Qt 对象树核心三件事：
 * 1. 父子关系（addChild/removeChild/findChild/findDescendant）
 * 2. 递归调用（build/update/clear 递归到所有后代）
 * 3. 递归 dispose（子先销毁，幂等，从 parent 移除）
 *
 * 加错误隔离：单个 view/child 抛错不传染兄弟。
 */
import { createLayer } from '../tools/flowboard/js/vis/core/Layer.js';

let pass = 0, fail = 0;
function ok(name, cond) {
  if (cond) { pass++; console.log('  PASS  ' + name); }
  else      { fail++; console.log('  FAIL  ' + name); }
}

console.log('--- 父子关系 ---');
const root = createLayer('root', null);
const road = createLayer('road', null);
const agent = createLayer('agent', null);
const env = createLayer('env', null);
const infra = createLayer('infra', null);

root.addChild(road);
root.addChild(agent);
root.addChild(env);
root.addChild(infra);

ok('root 有 4 个子层', root.getChildren().length === 4);
ok('road.parent === root', road.parent === root);
ok('agent.parent === root', agent.parent === root);
ok('findChild(road) 返回 road', root.findChild('road') === road);
ok('findChild(unknown) 返回 null', root.findChild('unknown') === null);

// 孙层
const streetlight = createLayer('streetlight', null);
road.addChild(streetlight);
ok('road 有 1 个子层 streetlight', road.getChildren().length === 1);
ok('findDescendant(streetlight) 从 root 找到', root.findDescendant('streetlight') === streetlight);
ok('findDescendant(road) 从 root 找到', root.findDescendant('road') === road);
ok('findDescendant(root) 找不到自己（不搜 root 本身）', root.findDescendant('root') === null);

console.log('\n--- addChild 自动从原 parent 移除 ---');
const movedChild = createLayer('moved', null);
root.addChild(movedChild);
ok('movedChild 挂到 root', movedChild.parent === root && root.getChildren().includes(movedChild));
road.addChild(movedChild);
ok('movedChild 移到 road', movedChild.parent === road && road.getChildren().includes(movedChild));
ok('root 不再持有 movedChild', !root.getChildren().includes(movedChild));

console.log('\n--- addView + getViews ---');
let buildCalls = 0, updateCalls = 0, clearCalls = 0, disposeCalls = 0;
const fakeView = {
  build: () => { buildCalls++; },
  update: () => { updateCalls++; },
  clear:  () => { clearCalls++; },
  dispose: () => { disposeCalls++; },
};
road.addView(fakeView);
ok('road.getViews() 包含 fakeView', road.getViews().includes(fakeView));

console.log('\n--- 递归 build/update/clear ---');
buildCalls = 0; updateCalls = 0; clearCalls = 0;
// streetlight 也加个 view
let subBuild = 0, subUpdate = 0, subClear = 0;
streetlight.addView({
  build: () => { subBuild++; },
  update: () => { subUpdate++; },
  clear:  () => { subClear++; },
});
root.build({});
ok('root.build 调用了 road 的 fakeView.build', buildCalls === 1);
ok('root.build 递归到 streetlight 的 view', subBuild === 1);

root.update({}, 0);
ok('root.update 调用了 fakeView.update', updateCalls === 1);
ok('root.update 递归到 streetlight', subUpdate === 1);

root.clear();
ok('root.clear 调用了 fakeView.clear', clearCalls === 1);
ok('root.clear 递归到 streetlight', subClear === 1);

console.log('\n--- 错误隔离：单个 view 抛错不传染兄弟 ---');
let goodBuild = 0, evilBuild = 0, goodUpdate = 0, evilUpdate = 0;
agent.addView({
  build:  () => { goodBuild++; },
  update: () => { goodUpdate++; },
});
agent.addView({
  build:  () => { evilBuild++; throw new Error('boom-build'); },
  update: () => { evilUpdate++; throw new Error('boom-update'); },
});
root.build({});
root.update({}, 0);
ok('good view build 被调用', goodBuild === 1);
ok('evil view build 也被调用（throw 前）', evilBuild === 1);
ok('good view update 被调用（不受 evil 影响）', goodUpdate === 1);
ok('evil view update 抛错被吞', evilUpdate === 1);
// 再来一轮，确认 root 仍可调（没被传染）
root.update({}, 1);
ok('第二轮 update 仍正常调用', goodUpdate === 2);

console.log('\n--- 子 layer build 抛错不传染兄弟 layer ---');
let aBuild = 0, bBuild = 0, bUpdate = 0;
const testRoot = createLayer('testRoot', null);
const a = createLayer('a', null);
const b = createLayer('b', null);
testRoot.addChild(a);
testRoot.addChild(b);
a.addView({ build: () => { aBuild++; throw new Error('a-boom'); } });
b.addView({
  build:  () => { bBuild++; },
  update: () => { bUpdate++; },
});
testRoot.build({});
ok('a 抛错不影响 b.build', bBuild === 1);
testRoot.update({}, 0);
ok('a 抛错后仍能 update b', bUpdate === 1);

console.log('\n--- dispose 递归 + 幂等 ---');
let childDispose = 0, grandDispose = 0;
const dispRoot = createLayer('dispRoot', null);
const child = createLayer('child', null);
const grand = createLayer('grand', null);
dispRoot.addChild(child);
child.addChild(grand);
grand.addView({ dispose: () => { grandDispose++; } });
child.addView({ dispose: () => { childDispose++; } });
dispRoot.dispose();
ok('dispose 调用 child.dispose', childDispose === 1);
ok('dispose 递归到 grand', grandDispose === 1);
ok('dispRoot 已 disposed', dispRoot.isDisposed());
ok('child 已 disposed', child.isDisposed());
ok('grand 已 disposed', grand.isDisposed());
// 幂等
dispRoot.dispose();
ok('dispose 幂等（不重复调）', childDispose === 1 && grandDispose === 1);

console.log('\n--- dispose 无 dispose() 方法的 view 退化为 clear() ---');
let cleared = false;
const lr = createLayer('lr', null);
lr.addView({ clear: () => { cleared = true; } /* 无 dispose */ });
lr.dispose();
ok('view 无 dispose → 调 clear()', cleared);

console.log('\n--- markFailed 跳过后续 build/update ---');
let failBuild = 0, failUpdate = 0;
const fr = createLayer('fr', null);
fr.addView({
  build:  () => { failBuild++; },
  update: () => { failUpdate++; },
});
fr.markFailed();
fr.build({});
fr.update({}, 0);
ok('markFailed 后 build 被跳过', failBuild === 0);
ok('markFailed 后 update 被跳过', failUpdate === 0);
fr.resetFailure();
fr.build({});
fr.update({}, 0);
ok('resetFailure 后恢复调用', failBuild === 1 && failUpdate === 1);

console.log('\n--- removeChild ---');
const rc = createLayer('rc', null);
const c1 = createLayer('c1', null);
rc.addChild(c1);
rc.removeChild(c1);
ok('removeChild 后 rc 不再持有 c1', !rc.getChildren().includes(c1));
ok('removeChild 清掉 c1.parent', c1.parent === null);
// removeChild 不存在的 child 不报错
rc.removeChild(c1);
ok('removeChild 不存在的 child 不报错', true);

console.log('\n--- id 唯一递增 ---');
const l1 = createLayer('l1', null);
const l2 = createLayer('l2', null);
ok('layer id 递增', l2.id > l1.id && l1.id > 0);

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
if (fail > 0) process.exit(1);
