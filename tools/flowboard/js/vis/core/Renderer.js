/**
 * Renderer.js — WebGLRenderer + 渲染循环 + 后处理管线
 *
 * r160 迁移：
 *   - outputEncoding/sRGBEncoding → outputColorSpace/SRGBColorSpace
 *   - pixelRatio 封顶 1.5（初始；perfTier=high 时升到 min(dpr,2)）
 *   - EffectComposer: Bloom + OutputPass + SMAA（GTAO 仅 high 档，
 *     大场景下 GTAO 半分辨率 AO 噪点明显、性能开销大）
 */

export function createRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({
    canvas, antialias: true, powerPreference: 'high-performance'
  });
  /* 封顶 1.5：4K/Retina 全 dpr 跑全套后处理帧率崩，1.5 观感几乎无差 */
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  /* r152+：outputColorSpace 替代 outputEncoding */
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  /* exposure 1.0：配合 ACES tonemap，避免纯白车道线过曝辉光 */
  renderer.toneMappingExposure = 1.0;

  return renderer;
}

/** 创建后处理 Composer。
 *  管线：RenderPass → [GTAO(仅 opts.gtao)] → Bloom → OutputPass → SMAA
 *
 *  - GTAOPass：接触遮蔽。默认关——半分辨率 AO 在大开阔场景（路面/草地
 *    大平面 + 远景）下产生可见噪点涂抹，只在 perfTier=high 时启用
 *  - UnrealBloomPass：车灯/路灯辉光（threshold 0.85 只让 emissive 表面过阈）
 *  - OutputPass：r16x 新范式，把 ACES tonemap + colorSpace 收到管线末端
 *  - SMAAPass：开 Composer 会丢 MSAA，用 SMAA 补回抗锯齿
 *  @param {object} [opts] { gtao: boolean } */
export function createComposer(renderer, scene, camera, opts = {}) {
  if (!THREE.EffectComposer) {
    console.warn('[Renderer] EffectComposer unavailable, falling back to direct render');
    return null;
  }

  const composer = new THREE.EffectComposer(renderer);

  // 1. 基础渲染 Pass
  const renderPass = new THREE.RenderPass(scene, camera);
  composer.addPass(renderPass);

  // 2. GTAO — 仅 high 档（默认关，见函数头注释）
  if (opts.gtao && THREE.GTAOPass) {
    const gtao = new THREE.GTAOPass(scene, camera);
    gtao.output = THREE.GTAOPass.OUTPUT.Default;
    composer.addPass(gtao);
  }

  // 3. Bloom — 只让灯/发光体辉光
  //    strength 0.35（原 0.6 会让白车道线/天空边缘泛雾）
  //    threshold 0.85（原 1.0 实测车灯 emissive 大多压不过阈，Bloom 形同虚设）
  if (THREE.UnrealBloomPass) {
    const bloom = new THREE.UnrealBloomPass(
      new THREE.Vector2(window.innerWidth, window.innerHeight),
      0.35,  // strength
      0.3,   // radius
      0.85   // threshold
    );
    composer.addPass(bloom);
  }

  // 4. OutputPass — tonemap + colorSpace 统一在管线末端
  if (THREE.OutputPass) {
    composer.addPass(new THREE.OutputPass());
  }

  // 5. SMAA — 补回 Composer 丢掉的 MSAA
  if (THREE.SMAAPass) {
    composer.addPass(new THREE.SMAAPass(
      window.innerWidth * window.devicePixelRatio,
      window.innerHeight * window.devicePixelRatio
    ));
  }

  return composer;
}

/** 渲染一帧（支持 perfTier 跳过 Composer） */
export function renderFrame(renderer, composer, scene, camera) {
  if (composer) composer.render();
  else renderer.render(scene, camera);
}

/** 销毁 Composer 及其所有 Pass（perfTier=low 时释放 GPU 资源） */
export function disposeComposer(composer) {
  if (!composer) return;
  try {
    for (let i = composer.passes.length - 1; i >= 0; i--) {
      const pass = composer.passes[i];
      if (pass.dispose) pass.dispose();
    }
    composer.passes.length = 0;
    if (composer.renderTarget1) composer.renderTarget1.dispose();
    if (composer.renderTarget2) composer.renderTarget2.dispose();
  } catch (e) {
    console.warn('[Renderer] disposeComposer error:', e.message);
  }
}

/** 调整大小 */
export function resize(renderer, composer, camera, width, height) {
  renderer.setSize(width, height);
  if (composer) composer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

/** 获取渲染器性能统计（Draw Call 数、三角形数等）*/
export function getRendererInfo(renderer) {
  if (!renderer || !renderer.info) return null;
  const info = renderer.info;
  return {
    calls: info.render.calls,
    triangles: info.render.triangles,
    points: info.render.points,
    lines: info.render.lines,
    geometries: info.memory.geometries,
    textures: info.memory.textures,
  };
}

/** 重置渲染器统计 */
export function resetRendererInfo(renderer) {
  if (renderer && renderer.info) {
    renderer.info.reset();
  }
}
