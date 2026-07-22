/**
 * bootstrap.js — Three.js r160 ESM 引导模块
 *
 * r16x 删除了 UMD three.min.js，改用 ESM。本模块通过 import-map
 * 从 CDN 加载 three + addons，挂到 window.THREE，使所有 view
 * 里的 THREE.xxx 全局引用一行不用动。
 *
 * 加载顺序：index.html 先加载本模块（type="module"），完成后
 * 才加载 app.js（同为 type="module"，按文档顺序执行）。
 */

import * as THREE from 'three';
// ES module namespace 是只读的，Object.assign 拷贝为普通对象后再挂 addons
window.THREE = Object.assign({}, THREE);

// GLTFLoader — r16x 不再挂 THREE 全局，ESM 导入后手动挂
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
window.THREE.GLTFLoader = GLTFLoader;

// RGBELoader — HDRI 环境贴图加载
import { RGBELoader } from 'three/addons/loaders/RGBELoader.js';
window.THREE.RGBELoader = RGBELoader;

// 后处理管线 addons
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
window.THREE.EffectComposer = EffectComposer;

import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
window.THREE.RenderPass = RenderPass;

import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
window.THREE.GTAOPass = GTAOPass;

import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
window.THREE.UnrealBloomPass = UnrealBloomPass;

import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
window.THREE.OutputPass = OutputPass;

import { SMAAPass } from 'three/addons/postprocessing/SMAAPass.js';
window.THREE.SMAAPass = SMAAPass;

console.log('[bootstrap] Three.js r160 ESM loaded, THREE.GLTFLoader + post-processing ready');
