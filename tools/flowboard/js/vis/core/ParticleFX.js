/**
 * ParticleFX.js — 轻量粒子系统（尾气/尘土）
 *
 * 使用 THREE.Points + BufferGeometry，pool 管理粒子复用。
 * 零外部资产，运行时生成。
 *
 * Phase 2 升级：添加尾气/尘土粒子效果
 */

const MAX_PARTICLES = 500;

/** 创建粒子系统实例
 *  @param {THREE.Scene} scene
 *  @returns {{ tick: (dt: number) => void,
 *              emit: (x: number, y: number, z: number, opts: object) => void,
 *              dispose: () => void }} */
export function createParticleFX(scene) {
  // ── 粒子池 ──
  const pool = new Array(MAX_PARTICLES);
  for (let i = 0; i < MAX_PARTICLES; i++) {
    pool[i] = {
      alive: false,
      x: 0, y: 0, z: 0,
      vx: 0, vy: 0, vz: 0,
      size: 0.1,
      opacity: 1,
      life: 0,
      maxLife: 1,
      color: new THREE.Color(0.5, 0.5, 0.5),
    };
  }
  let head = 0;  // 下一个可复用的粒子索引

  // ── THREE 几何体 ──
  const positions = new Float32Array(MAX_PARTICLES * 3);
  const sizes = new Float32Array(MAX_PARTICLES);
  const opacities = new Float32Array(MAX_PARTICLES);
  const colors = new Float32Array(MAX_PARTICLES * 3);

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setAttribute('size', new THREE.BufferAttribute(sizes, 1));
  geo.setAttribute('opacity', new THREE.BufferAttribute(opacities, 1));
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  // ── 材质 ──
  // NormalBlending（非 Additive）：灰色尾气/尘土是遮光介质不是发光体，
  // 加法混合会让灰烟在亮背景下消失、暗处诡异发光。
  const mat = new THREE.PointsMaterial({
    size: 0.15,
    vertexColors: true,
    transparent: true,
    opacity: 0.35,
    blending: THREE.NormalBlending,
    depthWrite: false,
    sizeAttenuation: true,
  });

  const points = new THREE.Points(geo, mat);
  points.frustumCulled = true;
  points.visible = false;  // 无活跃粒子时隐藏
  scene.add(points);

  // ── 核心函数 ──

  /** 发射一个粒子
   *  @param {number} x,y,z  THREE 场景坐标（调用方负责 ENU→THREE 转换，
   *                         用 Coord.worldToThree，禁止直接传 ENU）
   *  @param {object} opts
   *    size, life, speed, color, spread, vy */
  function emit(x, y, z, opts = {}) {
    const p = pool[head];
    head = (head + 1) % MAX_PARTICLES;

    p.alive = true;
    p.x = x;
    p.y = y;
    p.z = z;
    p.size = opts.size || 0.1;
    p.maxLife = opts.life || 1.0;
    p.life = 0;
    p.opacity = 1;

    const spread = opts.spread || 0.3;
    p.vx = (Math.random() - 0.5) * spread;
    p.vy = opts.vy != null ? opts.vy : (Math.random() * 0.3 + 0.1);
    p.vz = (Math.random() - 0.5) * spread;

    const c = opts.color || [0.6, 0.6, 0.6];
    p.color.setRGB(c[0], c[1], c[2]);

    // 使 overhead 可见
    points.visible = true;
  }

  /** 每帧更新
   *  @param {number} dt 帧间隔（秒） */
  function tick(dt) {
    const dtClamped = Math.min(dt, 0.05);  // clamp 防大跳帧
    let anyAlive = false;

    for (let i = 0; i < MAX_PARTICLES; i++) {
      const p = pool[i];
      if (!p.alive) {
        // 死粒子归零（避免旧数据污染）
        positions[i * 3] = 0;
        positions[i * 3 + 1] = 0;
        positions[i * 3 + 2] = 0;
        sizes[i] = 0;
        opacities[i] = 0;
        colors[i * 3] = 0;
        colors[i * 3 + 1] = 0;
        colors[i * 3 + 2] = 0;
        continue;
      }

      p.life += dtClamped;
      if (p.life >= p.maxLife) {
        p.alive = false;
        continue;
      }

      const t = p.life / p.maxLife;

      // 位置更新
      p.x += p.vx * dtClamped;
      p.y += p.vy * dtClamped;
      p.z += p.vz * dtClamped;

      // 属性随生命周期衰减
      const fade = 1 - t;
      const sizeScale = 1 + t * 2;  // 随时间膨胀

      positions[i * 3] = p.x;
      positions[i * 3 + 1] = p.y;
      positions[i * 3 + 2] = p.z;
      sizes[i] = p.size * sizeScale;
      opacities[i] = fade * p.opacity;
      colors[i * 3] = p.color.r * fade;
      colors[i * 3 + 1] = p.color.g * fade;
      colors[i * 3 + 2] = p.color.b * fade;

      anyAlive = true;
    }

    // 更新 GPU 缓冲
    geo.attributes.position.needsUpdate = true;
    geo.attributes.size.needsUpdate = true;
    geo.attributes.opacity.needsUpdate = true;
    geo.attributes.color.needsUpdate = true;

    points.visible = anyAlive;
  }

  /** 根据车速和油门发射尾气粒子 */
  function emitExhaust(x, y, z, speed, throttle) {
    if (speed < 0.5 || throttle < 0.05) return;
    // 尾气浓度：随 throttle 增大
    const count = Math.min(3, Math.floor(throttle * 4));
    for (let i = 0; i < count; i++) {
      emit(x, y, z, {
        size: 0.08 + Math.random() * 0.07,
        life: 0.5 + Math.random() * 1.0,
        speed: 0.2 + Math.random() * 0.3,
        spread: 0.15,
        vy: 0.1 + Math.random() * 0.2,
        color: [0.4 + Math.random() * 0.2, 0.4 + Math.random() * 0.2, 0.4 + Math.random() * 0.1],
      });
    }
  }

  /** 急刹车时发射尘土粒子 */
  function emitDust(x, y, z, brake) {
    if (brake < 0.3) return;
    const count = Math.min(2, Math.floor(brake * 3));
    for (let i = 0; i < count; i++) {
      emit(x, y, z + (Math.random() - 0.5) * 1.5, {
        size: 0.05 + Math.random() * 0.05,
        life: 0.3 + Math.random() * 0.5,
        speed: 0.5 + Math.random() * 0.5,
        spread: 0.5,
        vy: 0.1,
        color: [0.5 + Math.random() * 0.2, 0.4 + Math.random() * 0.15, 0.2 + Math.random() * 0.1],
      });
    }
  }

  function dispose() {
    scene.remove(points);
    geo.dispose();
    mat.dispose();
  }

  return { tick, emit, emitExhaust, emitDust, dispose, getPoints: () => points };
}