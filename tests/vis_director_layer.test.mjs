/**
 * vis_director_layer.test.mjs — SceneDirector × Layer 集成测试
 *
 * 真实 SceneDirector 依赖 THREE.js，无法直接 import。所以本测试直接
 * 验证 Layer 树结构（模拟 SceneDirector 内部挂载 9 个 View 到 4 个 Layer），
 * 验证：
 * 1. 9 个 View 都挂到对应 Layer
 * 2. tickAnimation 等价的 rootLayer.update(store, now) 调用所有动态 View
 * 3. dispose 递归清理所有 View（每个 view.dispose 被调一次）
 * 4. getLayer/getRootLayer 查找
 */
import { createLayer } from '../tools/flowboard/js/vis/core/Layer.js';

let pass = 0, fail = 0;
function ok(name, cond) {
  if (cond) { pass++; console.log('  PASS  ' + name); }
  else      { fail++; console.log('  FAIL  ' + name); }
}

/* 假 view：记录 build/update/clear/dispose 调用次数 */
function makeFakeView(name) {
  const calls = { build: 0, update: 0, clear: 0, dispose: 0 };
  return {
    name,
    build: () => { calls.build++; },
    update: (store, now) => { calls.update++; },
    clear:  () => { calls.clear++; },
    dispose: () => { calls.dispose++; },
    _calls: calls,
  };
}

console.log('--- 模拟 SceneDirector 的 Layer 树挂载 ---');
const root  = createLayer('root', null);
const env   = root.addChild(createLayer('env', null));
const road  = root.addChild(createLayer('road', null));
const agent = root.addChild(createLayer('agent', null));
const infra = root.addChild(createLayer('infra', null));

const groundV       = makeFakeView('ground');
const viaductV      = makeFakeView('viaduct');
const roadV         = makeFakeView('road');
const streetlightV  = makeFakeView('streetlight');
const barrierV      = makeFakeView('barrier');
const connectorV    = makeFakeView('connector');
const vehicleV      = makeFakeView('vehicle');
const trafficLightV = makeFakeView('trafficLight');
const etcGateV      = makeFakeView('etcGate');

env.addView(groundV);
env.addView(viaductV);
road.addView(roadV);
road.addView(streetlightV);
road.addView(barrierV);
road.addView(connectorV);
agent.addView(vehicleV);
infra.addView(trafficLightV);
infra.addView(etcGateV);

ok('env 层有 2 个 view',    env.getViews().length === 2);
ok('road 层有 4 个 view',   road.getViews().length === 4);
ok('agent 层有 1 个 view',  agent.getViews().length === 1);
ok('infra 层有 2 个 view',   infra.getViews().length === 2);
ok('root 直接子层有 4 个',  root.getChildren().length === 4);

console.log('\n--- getLayer 等价查找 ---');
ok('findDescendant(env) 找到 env',  root.findDescendant('env')  === env);
ok('findDescendant(road) 找到 road', root.findDescendant('road') === road);
ok('findDescendant(agent) 找到 agent', root.findDescendant('agent') === agent);
ok('findDescendant(infra) 找到 infra', root.findDescendant('infra') === infra);
ok('findDescendant(unknown) 返回 null', root.findDescendant('unknown') === null);

console.log('\n--- tickAnimation 等价：rootLayer.update(store, now) 递归所有动态 View ---');
// 初始 update 次数应为 0
ok('vehicle.update 调用前 0 次',   vehicleV._calls.update === 0);
ok('trafficLight.update 调用前 0 次', trafficLightV._calls.update === 0);
ok('etcGate.update 调用前 0 次',   etcGateV._calls.update === 0);
ok('ground.update 调用前 0 次',   groundV._calls.update === 0);
ok('road.update 调用前 0 次',     roadV._calls.update === 0);

// 模拟 SceneDirector.tickAnimation(now) 内部调 rootLayer.update(store, now)
root.update({ ego: { x: 1 } }, 12345);

ok('vehicle.update 被调一次',   vehicleV._calls.update === 1);
ok('trafficLight.update 被调一次', trafficLightV._calls.update === 1);
ok('etcGate.update 被调一次',   etcGateV._calls.update === 1);
// mock 给所有 view 都加了 update 方法（真实场景下静态 View 无 update 方法，
// Layer 会 typeof 检查跳过）。这里验证递归到达所有 view。
ok('groundV 也被调（mock 给了 update）', groundV._calls.update === 1);
ok('roadV 也被调（mock 给了 update）',   roadV._calls.update === 1);

// 多帧
root.update({ ego: { x: 2 } }, 12346);
root.update({ ego: { x: 3 } }, 12347);
ok('3 帧后 vehicle.update = 3', vehicleV._calls.update === 3);
ok('3 帧后 trafficLight.update = 3', trafficLightV._calls.update === 3);

console.log('\n--- dispose() 等价：rootLayer.dispose 递归清理所有 View ---');
ok('所有 view dispose 调用前 0 次',
   groundV._calls.dispose === 0 && vehicleV._calls.dispose === 0 && etcGateV._calls.dispose === 0);
root.dispose();
ok('groundV.dispose 被调',     groundV._calls.dispose === 1);
ok('viaductV.dispose 被调',    viaductV._calls.dispose === 1);
ok('roadV.dispose 被调',        roadV._calls.dispose === 1);
ok('streetlightV.dispose 被调', streetlightV._calls.dispose === 1);
ok('barrierV.dispose 被调',     barrierV._calls.dispose === 1);
ok('connectorV.dispose 被调',   connectorV._calls.dispose === 1);
ok('vehicleV.dispose 被调',     vehicleV._calls.dispose === 1);
ok('trafficLightV.dispose 被调', trafficLightV._calls.dispose === 1);
ok('etcGateV.dispose 被调',     etcGateV._calls.dispose === 1);

ok('dispose 后 root.isDisposed', root.isDisposed());
ok('dispose 后 env.isDisposed',  env.isDisposed());
ok('dispose 后 road.isDisposed', road.isDisposed());
ok('dispose 后 agent.isDisposed', agent.isDisposed());
ok('dispose 后 infra.isDisposed', infra.isDisposed());

console.log('\n--- dispose 幂等 ---');
root.dispose();
ok('二次 dispose 不重复调 view.dispose',
   groundV._calls.dispose === 1 && vehicleV._calls.dispose === 1);

console.log('\n--- 错误隔离：单个 View update 抛错不传染兄弟 ---');
const root2 = createLayer('root2', null);
const agent2 = root2.addChild(createLayer('agent2', null));
const infra2 = root2.addChild(createLayer('infra2', null));
let goodCount = 0;
const goodV = { update: () => { goodCount++; } };
const evilV = { update: () => { throw new Error('evil-boom'); } };
agent2.addView(goodV);
agent2.addView(evilV);
infra2.addView({ update: () => { goodCount++; } });
root2.update({}, 0);
ok('evil 抛错 → good agent 仍被调 (goodCount >= 1)', goodCount >= 1);
ok('infra 层 view 也仍被调 (goodCount === 2)',        goodCount === 2);
root2.update({}, 1);
root2.update({}, 2);
ok('多帧后仍正常（不传染，3 帧 × 2 view = 6）', goodCount === 6);
ok('root2 未 disposed（错误不致命）', !root2.isDisposed());

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
if (fail > 0) process.exit(1);
