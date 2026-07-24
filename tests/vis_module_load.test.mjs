/**
 * vis_module_load.test.mjs — 全量加载冒烟测试
 *
 * 用 THREE shim 逐一 import 每个 js/vis/** 模块，验证：
 *  1. 模块语法正确（无语法错 → 抓漏括号/漏 export 等）
 *  2. 顶层 ReferenceError 不出现（抓未定义变量/未导入依赖）
 *  3. 所有 import 路径可解析（抓路径写错/文件不存在）
 *
 * 跑法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_module_load.test.mjs
 */

import { ok, done } from './test-utils.mjs';

console.log('=== vis/ 全量模块加载冒烟 ===\n');

// 按依赖顺序加载（先加载无依赖的 core，再加载 view）
const MODULES = [
  // ── core/ (无 THREE 依赖或导入 THREE) ──
  ['core/Constants',     '../tools/flowboard/js/vis/core/Constants.js'],
  ['core/Layer',         '../tools/flowboard/js/vis/core/Layer.js'],
  ['core/ViewRegistry',  '../tools/flowboard/js/vis/core/ViewRegistry.js'],
  ['core/DeadReckon',    '../tools/flowboard/js/vis/core/DeadReckon.js'],
  ['core/AssetFactory',  '../tools/flowboard/js/vis/core/AssetFactory.js'],
  ['core/Renderer',      '../tools/flowboard/js/vis/core/Renderer.js'],
  ['core/Lighting',      '../tools/flowboard/js/vis/core/Lighting.js'],
  ['core/SkyEnv',        '../tools/flowboard/js/vis/core/SkyEnv.js'],
  ['core/CameraRig',     '../tools/flowboard/js/vis/core/CameraRig.js'],
  ['store/SceneStore',   '../tools/flowboard/js/vis/store/SceneStore.js'],

  // ── math/ (纯函数) ──
  ['math/Coord',         '../tools/flowboard/js/vis/math/Coord.js'],
  ['math/Curve',         '../tools/flowboard/js/vis/math/Curve.js'],
  ['math/RoadHeight',    '../tools/flowboard/js/vis/math/RoadHeight.js'],
  ['math/GeometryMerge', '../tools/flowboard/js/vis/math/GeometryMerge.js'],

  // ── view/ (依赖 THREE 全局) ──
  ['view/VehicleLights',    '../tools/flowboard/js/vis/view/VehicleLights.js'],
  ['view/GroundView',       '../tools/flowboard/js/vis/view/GroundView.js'],
  ['view/RoadView',         '../tools/flowboard/js/vis/view/RoadView.js'],
  ['view/VehicleView',      '../tools/flowboard/js/vis/view/VehicleView.js'],
  ['view/ConnectorView',    '../tools/flowboard/js/vis/view/ConnectorView.js'],
  ['view/TrafficLightView', '../tools/flowboard/js/vis/view/TrafficLightView.js'],
  ['view/ETCGateView',      '../tools/flowboard/js/vis/view/ETCGateView.js'],
  ['view/ViaductView',      '../tools/flowboard/js/vis/view/ViaductView.js'],
  ['view/StreetlightView',  '../tools/flowboard/js/vis/view/StreetlightView.js'],
  ['view/BarrierView',      '../tools/flowboard/js/vis/view/BarrierView.js'],
  ['view/TreeView',         '../tools/flowboard/js/vis/view/TreeView.js'],

  // ── director/ (依赖所有 view) ──
  ['director/FrameValidator', '../tools/flowboard/js/vis/director/FrameValidator.js'],
  ['director/SceneDirector',  '../tools/flowboard/js/vis/director/SceneDirector.js'],
];

let pass = 0, fail = 0;

for (const [name, path] of MODULES) {
  try {
    await import(path);
    pass++;
    console.log(`  PASS  ${name}`);
  } catch (err) {
    fail++;
    console.log(`  FAIL  ${name}`);
    console.log(`        ${err.message}`);
    if (err.stack) {
      // 只打印第一帧（最相关的位置）
      const lines = err.stack.split('\n');
      for (const line of lines.slice(1, 4)) {
        console.log(`        ${line.trim()}`);
      }
    }
  }
}

console.log(`\n=== 结果: ${pass} pass, ${fail} fail ===`);
process.exit(fail > 0 ? 1 : 0);