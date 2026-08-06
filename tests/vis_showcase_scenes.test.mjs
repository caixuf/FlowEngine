/**
 * vis_showcase_scenes.test.mjs — 综合展示门禁
 *
 * 目标：证明「其他场景的 3D 都建起来了」——把每个 scenarios/*.json 用
 * sceneAdapter 转成 scene 帧，喂进 SceneDirector 做 update + 连续 tick，
 * 任何一帧抛错即 FAIL。等价于 C 侧 demo_evaluator 的运行时回归，但只针对
 * 3D 渲染层，秒级完成。
 *
 * 另校验 tools/flowboard/showcase/scenes.json 未过期（raw 与磁盘一致），
 * 防止 build_showcase.py 忘跑导致展示页看到旧场景。
 *
 * 跑法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_showcase_scenes.test.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createSceneDirector } from '../tools/flowboard/js/vis/director/SceneDirector.js';
import * as ViewRegistry from '../tools/flowboard/js/vis/core/ViewRegistry.js';
import { scenarioToTopoData } from '../tools/flowboard/js/showcase/sceneAdapter.js';
import { ok, done } from './test-utils.mjs';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(__dirname, '..');
const SCEN_DIR = path.join(REPO, 'scenarios');
const SCENES_JSON = path.join(REPO, 'tools/flowboard/showcase/scenes.json');

console.log('=== vis/ 综合展示场景渲染门禁 ===\n');

function listScenarioFiles() {
  return fs.readdirSync(SCEN_DIR)
    .filter((f) => f.endsWith('.json') && f !== 'suite.json')
    .sort();
}

// ── 1. 每个场景都能 update + tick 不抛错 ──
{
  console.log('--- 1. 逐场景渲染 ---');
  const files = listScenarioFiles();
  ok('发现场景文件 >= 10 个', files.length >= 10);

  for (const f of files) {
    const raw = JSON.parse(fs.readFileSync(path.join(SCEN_DIR, f), 'utf-8'));
    const topo = scenarioToTopoData(raw);

    ViewRegistry.clear();
    const scene = globalThis.THREE.Scene();
    let pass = true;
    let errMsg = '';
    let store = null;
    try {
      const director = createSceneDirector(scene);
      director.update(topo);
      // 连续 3 帧 tick：抓「第二帧才崩」类回归。
      for (let i = 0; i < 3; i++) director.tickAnimation(1000 + i * 16);
      store = director.getStore();
    } catch (err) {
      pass = false;
      errMsg = err.message;
    }
    ok(`[${f}] update+tick 不抛错`, pass);
    if (!pass) console.log('        ' + errMsg);
    if (store) {
      ok(`[${f}] store.ego 已写入`, !!store.ego);
      ok(`[${f}] 路网 edges >= 1`,
        !!store.roadNetwork && Array.isArray(store.roadNetwork.edges) && store.roadNetwork.edges.length >= 1);
    }
  }
}

// ── 2. 场景实体转换正确性抽样 ──
{
  console.log('\n--- 2. 转换正确性抽样 (straight_road) ---');
  const raw = JSON.parse(fs.readFileSync(path.join(SCEN_DIR, 'straight_road.json'), 'utf-8'));
  const s = scenarioToTopoData(raw).metrics.scene;
  const actors = (raw.actors || []).length;
  const tls = (raw.traffic_lights || []).length;
  ok('entities 数 = actors + traffic_lights',
    s.entities.length === actors + tls);
  ok('红绿灯转为 type=tl 实体',
    s.entities.filter((e) => e.type === 'tl').length === tls);
  ok('ego 速度分量归零（静态快照防漂移）',
    (s.ego.vx || 0) === 0 && (s.ego.vy || 0) === 0);
  ok('actor 速度分量归零',
    s.entities.filter((e) => e.type !== 'tl').every((e) => (e.vx || 0) === 0 && (e.vy || 0) === 0));
  ok('edge.length 由 length_m 补全',
    s.road_network.edges.every((e) => typeof e.length === 'number' && e.length > 0));
}

// ── 3. showcase/scenes.json 未过期 ──
{
  console.log('\n--- 3. scenes.json 未过期 ---');
  if (!fs.existsSync(SCENES_JSON)) {
    ok('scenes.json 存在（跑 python3 tools/build_showcase.py 生成）', false);
  } else {
    const manifest = JSON.parse(fs.readFileSync(SCENES_JSON, 'utf-8'));
    const list = Array.isArray(manifest.scenarios) ? manifest.scenarios : [];
    ok('scenes.json 场景数 == 磁盘场景数', list.length === listScenarioFiles().length);
    let stale = 0;
    for (const entry of list) {
      const name = path.basename(entry.file || '');
      const p = path.join(SCEN_DIR, name);
      if (!fs.existsSync(p)) { stale++; continue; }
      const disk = JSON.parse(fs.readFileSync(p, 'utf-8'));
      if (JSON.stringify(disk) !== JSON.stringify(entry.raw)) stale++;
    }
    ok('scenes.json 内嵌 raw 与磁盘一致（过期请重跑 build_showcase.py）', stale === 0);
    if (stale > 0) console.log(`        ${stale} 个场景过期`);
  }
}

done();
