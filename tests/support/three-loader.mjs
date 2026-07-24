/**
 * three-loader.mjs — Node.js 自定义 loader hook
 *
 * 拦截 import 'three' / import ... from 'three'，返回 three-shim.mjs。
 * 用于 Node 20+ 的 --loader 或 register() 机制。
 *
 * 用法（通过 three-preload.mjs 间接注册）：
 *   node --import ./tests/support/three-preload.mjs <test.mjs>
 */

import { resolve as _resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = _resolve(fileURLToPath(import.meta.url), '..');
const SHIM_URL = new URL('three-shim.mjs', import.meta.url).href;

export async function resolve(specifier, context, nextResolve) {
  // 拦截所有 'three' 导入（包括子路径如 'three/examples/...'）
  if (specifier === 'three' || specifier.startsWith('three/')) {
    return {
      url: SHIM_URL,
      format: 'module',
      shortCircuit: true,
    };
  }
  return nextResolve(specifier, context);
}

export async function load(url, context, nextLoad) {
  const result = await nextLoad(url, context);

  // vis/ 模块在浏览器中通过 <script> 标签获取全局 THREE，
  // 在 Node ESM 中 globalThis.THREE 不会自动暴露为模块作用域内的 THREE。
  // 对未显式 import THREE 的 vis/ 模块，在源码顶部注入 const THREE = globalThis.THREE;
  if (result.source && url.includes('/tools/flowboard/js/vis/')) {
    const src = result.source.toString();
    // 已有 import * as THREE 的模块（SceneStore.js, CameraRig.js）跳过
    if (!src.includes('import * as THREE from') && !src.includes("import * as THREE from")) {
      // 仅对确实使用了 THREE 的模块注入（避免无意义的污染）
      if (/\bTHREE\./.test(src)) {
        result.source = 'const THREE = globalThis.THREE;\n' + src;
      }
    }
  }

  return result;
}