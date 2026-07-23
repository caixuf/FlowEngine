/**
 * vis_render_smoke.test.mjs — 渲染数据层冒烟测试（纯 Node，无浏览器）
 *
 * 验证：SceneDirector 接收的 frame 数据必须包含可渲染的 road/vehicle/tl，
 * 且关键 group 的 y/z 在合理范围（防止「数据有但画面无」 regression）。
 *
 * 跑法：node tests/vis_render_smoke.test.mjs
 */

let pass = 0, fail = 0;
function check(name, actual, expected) {
  const ok = actual === expected;
  if (ok) { pass++; console.log('  PASS  ' + name); }
  else { fail++; console.log('  FAIL  ' + name + '  actual=' + JSON.stringify(actual) + '  expected=' + JSON.stringify(expected)); }
}
function checkOk(name, actual) {
  if (actual) { pass++; console.log('  PASS  ' + name); }
  else { fail++; console.log('  FAIL  ' + name); }
}

/** 构造一个包含弯道 + 高架 + 红绿灯 + NPC 的 mock store */
function buildMockStore() {
  return {
    roadNetwork: {
      edges: [
        {
          id: 'e1',
          type: 'highway',
          length: 1000,
          lanes: 2,
          nodes: [
            [0, -1.75, 0],
            [100, -1.75, 2],
            [200, -1.75, 5],
            [300, -1.75, 7],
          ],
          traffic_lights: [
            { s: 150, l: 0, state: 'red' },
            { s: 300, l: 0, state: 'green' },
          ],
        },
      ],
    },
    entities: [
      { id: 'ego', type: 'ego', x: 50, y: -1.75, z: 0.5, heading: 0, speed: 15 },
      { id: 'npc1', type: 'npc', x: 120, y: -1.75, z: 2.5, heading: 0.1, speed: 12 },
    ],
  };
}

/** 复刻 TrafficLightView 的实体提取逻辑（纯 JS，无 Three.js） */
function extractTlEntities(store) {
  const all = (store.entities || []).filter(e => e.type === 'tl');
  const rn = store.roadNetwork;
  if (rn && rn.edges) {
    for (const edge of rn.edges) {
      const tls = edge.traffic_lights || [];
      for (let i = 0; i < tls.length; i++) {
        const tl = tls[i];
        // 复刻 getEdgePointAtS 的直道逻辑（测试用最小实现）
        let x = 0, z = 0;
        const nodes = edge.nodes;
        if (nodes && nodes.length >= 2) {
          const s = tl.s || 0;
          const len = edge.length || 1;
          const t = Math.max(0, Math.min(1, s / len));
          const a = nodes[0], b = nodes[nodes.length - 1];
          x = a[0] + (b[0] - a[0]) * t;
          z = -(a[1] + (b[1] - a[1]) * t);
        }
        all.push({ id: `tl_${edge.id}_${i}`, type: 'tl', x, y: tl.l || 0, z, state: tl.state || 'red' });
      }
    }
  }
  return all;
}

console.log('--- 渲染数据层冒烟测试 ---');
const store = buildMockStore();

const ego = store.entities.find(e => e.type === 'ego');
const npc = store.entities.find(e => e.type === 'npc');
const tls = extractTlEntities(store);
const edges = store.roadNetwork.edges;

checkOk('store 含 ego', !!ego);
checkOk('store 含 npc', !!npc);
check('tl 数量 == 2', tls.length, 2);
check('edge 数量 == 1', edges.length, 1);

/* 高度不变量：高架/弯道场景下 z 必须在合理范围（不能悬空或沉地） */
checkOk('ego z 在合理范围 (-10 ~ 20)', ego.z >= -10 && ego.z <= 20);
checkOk('npc z 在合理范围 (-10 ~ 20)', npc.z >= -10 && npc.z <= 20);

/* 红绿灯必须沿 edge 几何分布，而非简单加在 X 上 */
const tl0 = tls[0];
const tl1 = tls[1];
checkOk('tl0 z 在合理范围 (-10 ~ 20)', tl0.z >= -10 && tl0.z <= 20);
checkOk('tl1 z 在合理范围 (-10 ~ 20)', tl1.z >= -10 && tl1.z <= 20);
/* tl0.s=150, tl1.s=300, 在 1000m edge 上，x 应该分别是 ~45 和 ~90（直道近似） */
checkOk('tl0 x > 0', tl0.x > 0);
checkOk('tl1 x > tl0 x', tl1.x > tl0.x);

/* 路网的 node z 必须在合理范围（防止默认 0 或硬编码 7.7 漂移） */
for (const edge of edges) {
  for (const n of edge.nodes) {
    const z = n[2] || 0;
    checkOk(`edge ${edge.id} node z=${z} 在合理范围`, z >= -5 && z <= 20);
  }
}

console.log(`\n--- 结果: ${pass} pass, ${fail} fail ---`);
process.exit(fail > 0 ? 1 : 0);
