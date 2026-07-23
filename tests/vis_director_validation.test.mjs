/**
 * vis_director_validation.test.mjs — SceneDirector frame schema 校验冒烟测试
 * （纯 Node，无浏览器）
 *
 * 验证 FrameValidator.validateFrame(topoData) 的契约：
 *   - 各种坏输入下 ok=false / skipXxx=true 而不抛错
 *   - warning 一次性，key 与字段对应
 *   - 正常 frame ok=true 无 warnings
 *
 * 跑法：node tests/vis_director_validation.test.mjs
 */
import { validateFrame } from '../tools/flowboard/js/vis/director/FrameValidator.js';

let pass = 0, fail = 0;
function check(name, actual, expected) {
  const ok = actual === expected;
  if (ok) { pass++; console.log('  PASS  ' + name); }
  else { fail++; console.log('  FAIL  ' + name + '  actual=' + JSON.stringify(actual) + '  expected=' + JSON.stringify(expected)); }
}
function checkDeep(name, actual, expected) {
  const ok = JSON.stringify(actual) === JSON.stringify(expected);
  if (ok) { pass++; console.log('  PASS  ' + name); }
  else { fail++; console.log('  FAIL  ' + name + '  actual=' + JSON.stringify(actual) + '  expected=' + JSON.stringify(expected)); }
}
function hasWarningKey(v, key) {
  return v.warnings.some(w => w.key === key);
}
function warningKeys(v) {
  return v.warnings.map(w => w.key);
}

console.log('--- topoData 顶层校验 ---');
checkDeep('null → ok=false, warnings=[]',
  validateFrame(null),
  { ok: false, warnings: [] });
checkDeep('undefined → ok=false, warnings=[]',
  validateFrame(undefined),
  { ok: false, warnings: [] });
check('字符串 → ok=false',
  validateFrame('garbage').ok, false);
check('字符串 → 1 warning (topoData.type)',
  validateFrame('garbage').warnings.length, 1);
check('字符串 → warning key 是 topoData.type',
  hasWarningKey(validateFrame('garbage'), 'topoData.type'), true);
check('数字 → ok=false',
  validateFrame(42).ok, false);
check('数字 → warning key 是 topoData.type',
  hasWarningKey(validateFrame(42), 'topoData.type'), true);

console.log('\n--- frame 解包 ---');
// 注意：wrapper 检测用 truthy 判断（topoData.scene 真值才解包），
// 所以 {scene: null} / {metrics: {scene: null}} 都等价于"未指定 scene"，
// frame 回退到 topoData 本身，ok=true。
// 这是现有契约（与 Step 1 一致），如需收紧到"显式 null 也算错"，再单独提改动。
check('{scene: null} → ok=true（topoData 自身作为 frame）',
  validateFrame({ scene: null }).ok, true);
check('{scene: null} → 无 frame.type warning',
  hasWarningKey(validateFrame({ scene: null }), 'frame.type'), false);
check('frame="str" → ok=false',
  validateFrame({ scene: 'oops' }).ok, false);
check('frame=42 → ok=false',
  validateFrame({ scene: 42 }).ok, false);
check('metrics.scene 解包路径 ok=true',
  validateFrame({ metrics: { scene: { ego: { x: 1 } } } }).ok, true);
check('topoData.scene 解包路径 ok=true',
  validateFrame({ scene: { ego: { x: 1 } } }).ok, true);

console.log('\n--- 正常最小 frame ---');
const ok0 = validateFrame({ ego: { x: 0 } });
check('minimal frame ok=true', ok0.ok, true);
check('minimal frame 无 warnings', ok0.warnings.length, 0);
check('minimal frame 解出 ego', ok0.frame.ego.x, 0);

console.log('\n--- road_network 校验 ---');
const okRn = validateFrame({ road_network: { edges: [{ id: 1, lanes: 2 }] } });
check('road_network array edges ok=true', okRn.ok, true);
check('road_network array edges skipRoad=false', okRn.skipRoad, false);
check('road_network.edges=[] ok=true', validateFrame({ road_network: { edges: [] } }).ok, true);
check('road_network.edges=[] skipRoad=false（空数组允许）',
  validateFrame({ road_network: { edges: [] } }).skipRoad, false);

const badRn1 = validateFrame({ road_network: 'oops' });
check('road_network 非 object → skipRoad=true', badRn1.skipRoad, true);
check('road_network 非 object → warning key=road_network.type',
  hasWarningKey(badRn1, 'road_network.type'), true);

const badRn2 = validateFrame({ road_network: { edges: 'oops' } });
check('road_network.edges 非数组 → skipRoad=true', badRn2.skipRoad, true);
check('road_network.edges 非数组 → warning key=road_network.edges',
  hasWarningKey(badRn2, 'road_network.edges'), true);

console.log('\n--- ego 校验 ---');
const badEgo1 = validateFrame({ ego: 'oops' });
check('ego 非 object → skipEgo=true', badEgo1.skipEgo, true);
check('ego 非 object → warning key=ego.type',
  hasWarningKey(badEgo1, 'ego.type'), true);

const badEgo2 = validateFrame({ ego: { x: 'oops' } });
check('ego.x 非数字 → ok=true（不丢整帧）', badEgo2.ok, true);
check('ego.x 非数字 → skipEgo=false（仍允许 ego 更新）', badEgo2.skipEgo, false);
check('ego.x 非数字 → warning key=ego.x', hasWarningKey(badEgo2, 'ego.x'), true);

const badEgo3 = validateFrame({ ego: { x: NaN } });
check('ego.x=NaN → warning key=ego.x', hasWarningKey(badEgo3, 'ego.x'), true);

const badEgo4 = validateFrame({ ego: { x: Infinity } });
check('ego.x=Infinity → warning key=ego.x', hasWarningKey(badEgo4, 'ego.x'), true);

const badEgo5 = validateFrame({ ego: { x: 0, heading: 'oops' } });
check('ego.heading 非数字 → warning key=ego.heading',
  hasWarningKey(badEgo5, 'ego.heading'), true);

const okEgoNoHeading = validateFrame({ ego: { x: 0 } });
check('ego.heading 缺省 → 无 ego.heading warning',
  hasWarningKey(okEgoNoHeading, 'ego.heading'), false);

const badEgo6 = validateFrame({ ego: { x: 0, lights: 'oops' } });
check('ego.lights 非数字 → warning key=ego.lights',
  hasWarningKey(badEgo6, 'ego.lights'), true);

const okEgoLightsMissing = validateFrame({ ego: { x: 0 } });
check('ego.lights 缺省 → 无 ego.lights warning',
  hasWarningKey(okEgoLightsMissing, 'ego.lights'), false);

console.log('\n--- entities 校验 ---');
const badEnt1 = validateFrame({ entities: 'oops' });
check('entities 非数组 → skipEntities=true', badEnt1.skipEntities, true);
check('entities 非数组 → warning key=entities.type',
  hasWarningKey(badEnt1, 'entities.type'), true);

const badEnt2 = validateFrame({ entities: [{ x: 1 }, { type: 'car' }] });
check('entities[0] 缺 type → warning key=entities[0].type',
  hasWarningKey(badEnt2, 'entities[0].type'), true);
check('entities[1] 有 type → 无 entities[1].type warning',
  hasWarningKey(badEnt2, 'entities[1].type'), false);

const entWithEgo = validateFrame({ entities: [{}, { type: 'ego' }, { type: 'car' }] });
check('entities 中 ego 元素被跳过，仅 [0] 缺 type 出 warning',
  warningKeys(entWithEgo).filter(k => k.startsWith('entities[')).length, 1);
check('entities[2] 不出 warning（ego 元素后顺位不变）',
  hasWarningKey(entWithEgo, 'entities[2].type'), false);

console.log('\n--- 组合：多 warning 一次返回 ---');
const combined = validateFrame({
  road_network: 'oops',
  ego: { x: 'oops' },
  entities: 'oops',
});
check('组合 ok=true（frame 仍可用）', combined.ok, true);
check('组合 skipRoad=true', combined.skipRoad, true);
check('组合 skipEgo=false（ego.x 仅 warn）', combined.skipEgo, false);
check('组合 skipEntities=true', combined.skipEntities, true);
check('组合 3 条 warnings',
  combined.warnings.length, 3);

console.log('\n--- 高度不变量校验 ---');
const heightOk = validateFrame({
  road_network: { edges: [{ id: 0, lanes: 4, lane_width: 3.5, nodes: [[0, 0, 0], [1000, 0, 0]] }] },
  ego: { x: 100, y: -1.75, z: 0 },
  entities: [{ type: 'car', x: 200, y: -1.75, z: 0 }, { type: 'tl', x: 300, y: 0, z: 0 }],
});
check('entity 在路面上 → 无 height warning', hasWarningKey(heightOk, 'height.ego'), false);
check('entity 在路面上 → warnings 数=0', heightOk.warnings.length, 0);

const heightBad = validateFrame({
  road_network: { edges: [{ id: 0, lanes: 4, lane_width: 3.5, nodes: [[0, 0, 0], [1000, 0, 0]] }] },
  ego: { x: 100, y: -1.75, z: 8 },
  entities: [{ type: 'car', x: 200, y: -1.75, z: -5 }],
});
check('ego 悬空 8m → height warning', hasWarningKey(heightBad, 'height.ego'), true);
check('entity 入地 5m → height warning', hasWarningKey(heightBad, 'height.entity[0](type=car)'), true);
check('悬空+入地 → 2 条 height warnings', heightBad.warnings.filter(w => w.key.startsWith('height.')).length, 2);

console.log('\n--- summary: ' + pass + ' pass, ' + fail + ' fail ---');
process.exit(fail > 0 ? 1 : 0);
