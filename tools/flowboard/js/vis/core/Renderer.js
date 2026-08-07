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
  /* 默认中档：压制 DPR 上限 1.5，避免高分屏（DPR=2-3）下全屏后处理
   * （GTAO/Bloom/SMAA）在 4-9 倍像素分辨率逐像素计算导致 GPU 卡死。
   * 高性能设备可通过 setPerfTier('high') 恢复全 DPR。 */
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
  // 注意：分辨率必须与 renderer 的受限 DPR 对齐（min(dpr,1.5)），否则高分屏下
  // SMAA 在器渲染目标之上更高的原始 DPR 分辨率跑，额外增加整屏开销。
  if (THREE.SMAAPass) {
    const cappedDpr = Math.min(window.devicePixelRatio || 1, 1.5);
    composer.addPass(new THREE.SMAAPass(
      Math.max(1, window.innerWidth * cappedDpr),
      Math.max(1, window.innerHeight * cappedDpr)
    ));
  }

  return composer;
}

/** 渲染一帧 */
export function renderFrame(renderer, composer, scene, camera) {
  if (composer) composer.render();
  else renderer.render(scene, camera);
}

/** 判断是否为软件渲染（无 GPU 的 WSL/云 VM/远程桌面）。
 * SwiftShader / llvmpipe / Softpipe / Mesa(无硬件) 是软件 WebGL 常见实现，
 * 跑 medium 档的 Bloom+SMAA+2048 阴影会卡成 PPT —— 必须直接落低档启动，
 * 不能等 6-9s 的自动降级（这段时间里用户已经在看 PPT 了）。 */
export function isSoftwareRenderer(renderer) {
  try {
    const gl = renderer.getContext();
    const ext = gl.getExtension('WEBGL_debug_renderer_info');
    if (!ext) return false;
    const name = String(gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) || '');
    return /swiftshader|llvmpipe|softpipe|software|angle \(software/i.test(name);
  } catch (_e) {
    return false;
  }
}

/** 判断 pass 是否为 GTAO（接触阴影）。GTAO 是全屏后处理里最贵的一趟，
 * low 档禁用整个 composer、medium 档单独关掉 GTAO 以明显降 GPU 压力。 */
function _isGTAOPass(p) {
  return p && (p.isGTAOPass || (p.constructor && p.constructor.name === 'GTAOPass'));
}

/** 启用/禁用 composer 中的 GTAO pass（medium 档用，保留 Bloom+SMAA） */
export function setComposerGTAOPassEnabled(composer, enabled) {
  if (!composer || !composer.passes || !composer.passes.length) return;
  for (const p of composer.passes) {
    if (_isGTAOPass(p)) p.enabled = !!enabled;
  }
}

/** 调整大小 */
export function resize(renderer, composer, camera, width, height) {
  /* ultra 档：渲染到缩放后的分辨率（如 0.5x），CSS 仍拉伸到全屏。
   * setSize 第三参 updateStyle=false 保证 canvas 由 CSS 100% 拉伸，
   * 避免覆盖 layout 的固定尺寸。 */
  const w = Math.max(1, Math.floor(width * _resScale));
  const h = Math.max(1, Math.floor(height * _resScale));
  renderer.setSize(w, h, false);
  if (composer) composer.setSize(w, h);
  /* setResolutionScale 会以 camera=null 调用（只重设分辨率），须判空 */
  if (camera) {
    camera.aspect = width / height;  // 用原始容器宽高比，避免拉伸变形
    camera.updateProjectionMatrix();
  }
}

/* 当前渲染分辨率缩放（默认 1，ultra 档降为 0.5） */
let _resScale = 1;

/** 设置渲染分辨率缩放（0.2~1）。ultra 档最后兜底：先降分辨率再让 CSS 放大，
 * 即使弱 GPU 也能把 2fps 提到可用的 15-30fps。 */
export function setResolutionScale(renderer, scale) {
  _resScale = Math.max(0.2, Math.min(1, scale));
  if (renderer && renderer.domElement) {
    const canvas = renderer.domElement;
    const container = canvas.parentElement;
    if (container) {
      const rect = container.getBoundingClientRect();
      resize(renderer, null, null, rect.width || window.innerWidth, rect.height || window.innerHeight);
      // 分辨率变化后需通知 camera 更新（resize 已处理 aspect）
    }
  }
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
