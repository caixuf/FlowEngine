/**
 * Renderer.js — WebGLRenderer + 渲染循环 + 后处理管线
 *
 * r160 迁移：
 *   - outputEncoding/sRGBEncoding → outputColorSpace/SRGBColorSpace
 *   - 全 devicePixelRatio（演示画质优先）
 *   - 阴影 4096（超锐）
 *   - EffectComposer: GTAO + Bloom + OutputPass + SMAA
 */

export function createRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({
    canvas, antialias: true, powerPreference: 'high-performance'
  });
  /* 全分辨率：演示画质优先，不压 devicePixelRatio */
  renderer.setPixelRatio(window.devicePixelRatio);
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
 *  r160 后处理管线：RenderPass → GTAOPass → Bloom → OutputPass → SMAA
 *
 *  - GTAOPass：地面接触遮蔽 + 立体感（物体"落地"）
 *  - UnrealBloomPass：车灯/路灯/HDRI 高光辉光（阈值 0.8 只让灯发光）
 *  - OutputPass：r16x 新范式，把 ACES tonemap + colorSpace 收到管线末端
 *  - SMAAPass：开 Composer 会丢 MSAA，用 SMAA 补回抗锯齿 */
export function createComposer(renderer, scene, camera) {
  if (!THREE.EffectComposer) {
    console.warn('[Renderer] EffectComposer unavailable, falling back to direct render');
    return null;
  }

  const composer = new THREE.EffectComposer(renderer);

  // 1. 基础渲染 Pass
  const renderPass = new THREE.RenderPass(scene, camera);
  composer.addPass(renderPass);

  // 2. GTAO — 接触阴影 + 立体感
  if (THREE.GTAOPass) {
    const gtao = new THREE.GTAOPass(scene, camera);
    gtao.output = THREE.GTAOPass.OUTPUT.Default;
    composer.addPass(gtao);
  }

  // 3. Bloom — 只让灯/发光体辉光（阈值 0.8 滤掉普通表面）
  if (THREE.UnrealBloomPass) {
    const bloom = new THREE.UnrealBloomPass(
      new THREE.Vector2(window.innerWidth, window.innerHeight),
      0.6,   // strength：适中，不让非灯体发糊
      0.4,   // radius：柔和扩散
      1.0    // threshold：只让 emissive > 1.0 发光（真车灯），普通表面不过阈
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

/** 渲染一帧 */
export function renderFrame(renderer, composer, scene, camera) {
  if (composer) composer.render();
  else renderer.render(scene, camera);
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
