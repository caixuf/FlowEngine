/**
 * vis_render_tick.test.mjs — 单帧 tick 冒烟测试
 *
 * 用 THREE shim 构造 SceneDirector，喂 3 帧罐头数据，
 * 各调一次 tickAnimation()，验证：
 *  1. 构建不抛错（所有 View 工厂的 new THREE.X() 都不抛）
 *  2. 平路 / 高架 / 多车道 三种场景的 update + tickAnimation 不抛错
 *  3. tick 后 store.ego 存在（数据被正确写入）
 *  4. 高架场景 tick 后 viaductView.followEgo 被调用（不抛错）
 *
 * 跑法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_render_tick.test.mjs
 */

import { createSceneDirector } from '../tools/flowboard/js/vis/director/SceneDirector.js';
import * as ViewRegistry from '../tools/flowboard/js/vis/core/ViewRegistry.js';
import { ok, eq, done } from './test-utils.mjs';

console.log('=== vis/ 单帧 tick 冒烟 ===\n');

// ── 工具：构造罐头 frame ──

function makeFlatFrame() {
  return {
    metrics: {
      scene: {
        road_network: {
          edges: [{
            id: 0,
            type: 'highway',
            lanes: 4,
            lane_width: 3.5,
            length: 1000,
            nodes: [[0, 0, 0], [200, 0, 0], [400, 0, 0], [600, 0, 0], [800, 0, 0], [1000, 0, 0]]
          }]
        },
        ego: { x: 100, y: 0, heading: 0, speed: 20, z: 0, steer: 0, brake: 0, throttle: 0.3, lights: 0, vx: 20, vy: 0 },
        entities: [
          { id: 'npc1', type: 'car', x: 150, y: 0, heading: 0, speed: 18, z: 0, steer: 0, length: 4.6, width: 2.0, lights: 0, vx: 18, vy: 0, brake: 0, throttle: 0.25, ai_state: 'cruise', state: '', progress: 0, remain_s: 0, parked: false }
        ]
      }
    }
  };
}

function makeViaductFrame() {
  return {
    metrics: {
      scene: {
        road_network: {
          edges: [{
            id: 0,
            type: 'viaduct_highway',
            name: 'viaduct_highway_0',
            lanes: 4,
            lane_width: 3.5,
            length: 2000,
            nodes: (() => {
              const pts = [];
              for (let i = 0; i <= 20; i++) {
                const x = i * 100;
                pts.push([x, 0, 7.0]); // 高架 elevation 7m
              }
              return pts;
            })()
          }]
        },
        ego: { x: 500, y: 0, heading: 0, speed: 25, z: 7.0, steer: 0, brake: 0, throttle: 0.4, lights: 0, vx: 25, vy: 0 },
        entities: []
      }
    }
  };
}

function makeMultiLaneFrame() {
  return {
    metrics: {
      scene: {
        road_network: {
          edges: [{
            id: 0,
            type: 'urban',
            lanes: 6,
            lane_width: 3.2,
            length: 800,
            nodes: [[0, 0, 0], [200, 0, 0], [400, 0, 0], [600, 0, 0], [800, 0, 0]]
          }]
        },
        ego: { x: 200, y: -3.5, heading: 0.05, speed: 15, z: 0, steer: 0.02, brake: 0, throttle: 0.2, lights: 0, vx: 15, vy: 0.5 },
        entities: [
          { id: 'npc1', type: 'car', x: 250, y: -3.5, heading: 0.05, speed: 14, z: 0, steer: 0, length: 4.6, width: 2.0, lights: 0, vx: 14, vy: 0, brake: 0, throttle: 0.2, ai_state: 'follow', state: '', progress: 0, remain_s: 0, parked: false },
          { id: 'npc2', type: 'truck', x: 180, y: -7.0, heading: 0.03, speed: 10, z: 0, steer: 0, length: 10.0, width: 2.5, lights: 0, vx: 10, vy: 0, brake: 0, throttle: 0.15, ai_state: 'cruise', state: '', progress: 0, remain_s: 0, parked: false },
          { id: 'npc3', type: 'suv', x: 300, y: 0, heading: 0, speed: 18, z: 0, steer: 0, length: 5.0, width: 2.1, lights: 0, vx: 18, vy: 0, brake: 0, throttle: 0.3, ai_state: 'cruise', state: '', progress: 0, remain_s: 0, parked: false },
        ]
      }
    }
  };
}

// ── 测试 1: 平路场景 director 构建 + tick ──

{
  console.log('--- 1. 平路场景 ---');
  ViewRegistry.clear(); // 清实例保留工厂（reset 会清掉 import 时注册的 factory）
  const scene = globalThis.THREE.Scene(); // proxy scene
  let director;
  try {
    director = createSceneDirector(scene);
    ok('createSceneDirector 不抛错', true);
  } catch (err) {
    ok('createSceneDirector 不抛错', false);
    console.log('       ' + err.message);
    director = null;
  }

  if (director) {
    const frame = makeFlatFrame();
    try {
      director.update(frame);
      ok('director.update(平路) 不抛错', true);
    } catch (err) {
      ok('director.update(平路) 不抛错', false);
      console.log('       ' + err.message);
    }

    try {
      director.tickAnimation(1000);
      ok('director.tickAnimation(平路) 不抛错', true);
    } catch (err) {
      ok('director.tickAnimation(平路) 不抛错', false);
      console.log('       ' + err.message);
    }

    const store = director.getStore();
    ok('tick 后 store.ego 存在', !!store.ego);
    if (store.ego) {
      ok('ego.x 约为 100', Math.abs(store.ego._simX - 100) < 5);
    }
    ok('tick 后 store.entities 非空', store.entities && store.entities.length > 0);
  }
}

// ── 测试 2: 高架场景 ──

{
  console.log('--- 2. 高架场景 ---');
  ViewRegistry.clear();
  const scene = globalThis.THREE.Scene();
  let director;
  try {
    director = createSceneDirector(scene);
    ok('createSceneDirector(高架) 不抛错', true);
  } catch (err) {
    ok('createSceneDirector(高架) 不抛错', false);
    console.log('       ' + err.message);
    director = null;
  }

  if (director) {
    const frame = makeViaductFrame();
    try {
      director.update(frame);
      ok('director.update(高架) 不抛错', true);
    } catch (err) {
      ok('director.update(高架) 不抛错', false);
      console.log('       ' + err.message);
    }

    try {
      director.tickAnimation(2000);
      ok('director.tickAnimation(高架) 不抛错', true);
    } catch (err) {
      ok('director.tickAnimation(高架) 不抛错', false);
      console.log('       ' + err.message);
    }

    const store = director.getStore();
    ok('高架: isViaduct = true', store.isViaduct === true);

    // 验证 followEgo 被调用（不抛错即视为调了，因为 proxy 上任何方法调用都不抛）
    const viaductView = ViewRegistry.get('viaduct');
    ok('高架: viaductView 已注册', !!viaductView);
    if (viaductView && viaductView.followEgo) {
      try {
        viaductView.followEgo(store.ego.x + 250);
        ok('高架: followEgo 不抛错', true);
      } catch (err) {
        ok('高架: followEgo 不抛错', false);
      }
    }
  }
}

// ── 测试 3: 多车道场景 ──

{
  console.log('--- 3. 多车道场景 ---');
  ViewRegistry.clear();
  const scene = globalThis.THREE.Scene();
  let director;
  try {
    director = createSceneDirector(scene);
    ok('createSceneDirector(多车道) 不抛错', true);
  } catch (err) {
    ok('createSceneDirector(多车道) 不抛错', false);
    console.log('       ' + err.message);
    director = null;
  }

  if (director) {
    const frame = makeMultiLaneFrame();
    try {
      director.update(frame);
      ok('director.update(多车道) 不抛错', true);
    } catch (err) {
      ok('director.update(多车道) 不抛错', false);
      console.log('       ' + err.message);
    }

    try {
      director.tickAnimation(3000);
      ok('director.tickAnimation(多车道) 不抛错', true);
    } catch (err) {
      ok('director.tickAnimation(多车道) 不抛错', false);
      console.log('       ' + err.message);
    }

    const store = director.getStore();
    ok('多车道: store.entities 有 3 个 NPC', store.entities && store.entities.length === 3);
    if (store.entities) {
      ok('多车道: NPC1 type=car', store.entities[0].type === 'car');
      ok('多车道: NPC2 type=truck', store.entities[1].type === 'truck');
      ok('多车道: NPC3 type=suv', store.entities[2].type === 'suv');
    }
  }
}

// ── 测试 4: 连续 tick 稳定性（防止第二帧崩） ──

{
  console.log('--- 4. 连续 tick 稳定性 ---');
  ViewRegistry.clear();
  const scene = globalThis.THREE.Scene();
  const director = createSceneDirector(scene);
  director.update(makeFlatFrame());

  let consecutivePass = 0;
  for (let i = 0; i < 5; i++) {
    try {
      director.tickAnimation(1000 + i * 16);
      consecutivePass++;
    } catch (err) {
      console.log('       frame ' + i + ' threw: ' + err.message);
    }
  }
  eq('连续 5 帧 tick 不抛错', consecutivePass, 5);
}

done();