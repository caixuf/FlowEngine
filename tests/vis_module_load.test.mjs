/**
 * vis_module_load.test.mjs — 全量加载冒烟测试
 *
 * A-4a 重构：用 readdirSync 递归扫描 js/vis/ 自动发现 .js 模块，
 * 取代硬编码模块列表（避免新增模块时漏改测试）。
 *
 * 用 THREE shim 逐一 import 每个 js/vis/** 模块，验证：
 *  1. 模块语法正确（无语法错 → 抓漏括号/漏 export 等）
 *  2. 顶层 ReferenceError 不出现（抓未定义变量/未导入依赖）
 *  3. 所有 import 路径可解析（抓路径写错/文件不存在）
 *
 * 跑法：
 *   node --import ./tests/support/three-preload.mjs tests/vis_module_load.test.mjs
 */

import { readdirSync } from 'fs';
import { resolve, relative, dirname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const VIS_DIR = resolve(__dirname, '../tools/flowboard/js/vis');

// A-4a: 递归扫描 vis/ 下所有 .js 文件
// 排除清单（带 DOM 副作用或仅浏览器入口的文件）：
//   main.js —— 入口模块，import 时会 init3DScene（DOM 依赖），不在 Node 测试范围
const EXCLUDE_FILES = new Set(['main.js']);

function discoverModules(dir) {
  const out = [];
  const entries = readdirSync(dir, { withFileTypes: true });
  for (const ent of entries) {
    const full = resolve(dir, ent.name);
    if (ent.isDirectory()) {
      out.push(...discoverModules(full));
    } else if (ent.isFile() && ent.name.endsWith('.js') && !EXCLUDE_FILES.has(ent.name)) {
      out.push(full);
    }
  }
  return out;
}

// 按路径排序，保证输出确定性（不依赖 fs 顺序）
const modulePaths = discoverModules(VIS_DIR).sort();

console.log('=== vis/ 全量模块加载冒烟（' + modulePaths.length + ' 个模块，readdirSync 自动发现）===\n');

let pass = 0, fail = 0;

for (const absPath of modulePaths) {
  // 显示相对 vis/ 的路径（如 core/CameraRig.js），更易读
  const name = relative(VIS_DIR, absPath).replace(/\\/g, '/');
  try {
    await import(absPath);
    pass++;
    console.log('  PASS  ' + name);
  } catch (err) {
    fail++;
    console.log('  FAIL  ' + name);
    console.log('        ' + err.message);
    if (err.stack) {
      // 只打印前 3 帧 stack（最相关的位置）
      const lines = err.stack.split('\n');
      for (const line of lines.slice(1, 4)) {
        console.log('        ' + line.trim());
      }
    }
  }
}

console.log('\n=== 结果: ' + pass + ' pass, ' + fail + ' fail ===');
process.exit(fail > 0 ? 1 : 0);
