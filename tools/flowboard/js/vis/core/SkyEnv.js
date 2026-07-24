/**
 * SkyEnv.js — 天空穹顶 + 环境光照 + 天气系统
 *
 * P3 画质升级：
 *   - 时段系统：日出/正午/黄昏/夜晚，动态太阳位置+色温
 *   - 动态雾：FogExp2 密度随时段变化
 *   - 天气模式：晴/雨/雪/阴天，可切换
 *   - 雨：instanced 粒子（高性能）
 *
 * 兼容原有 day/night toggle API，但内部已升级为连续时段。
 */

const TAU = Math.PI * 2;

// ═══════════════════════════════════════════════════════════
// 时段参数表
// ═══════════════════════════════════════════════════════════

const TIME_SLOTS = {
  dawn:    { angle: 0.15,  label: 'dawn',    skyTop: 0xf4a460, skyBot: 0xff6b35, sunColor: 0xffaa33, sunIntensity: 1.2,  fogDensity: 0.0035, ambIntensity: 0.5 },
  morning: { angle: 0.35,  label: 'morning',  skyTop: 0x87ceeb, skyBot: 0xb0c4de, sunColor: 0xfff8dc, sunIntensity: 2.0,  fogDensity: 0.0015, ambIntensity: 0.7 },
  noon:    { angle: 0.50,  label: 'noon',     skyTop: 0x4682b4, skyBot: 0x87ceeb, sunColor: 0xffffff, sunIntensity: 1.8,  fogDensity: 0.0008, ambIntensity: 0.6 },
  afternoon:{ angle: 0.65, label: 'afternoon',skyTop: 0x5b9bd5, skyBot: 0xc8dff5, sunColor: 0xfff5e0, sunIntensity: 2.0,  fogDensity: 0.0020, ambIntensity: 0.7 },
  dusk:    { angle: 0.80,  label: 'dusk',     skyTop: 0x6a5acd, skyBot: 0xff6347, sunColor: 0xff6600, sunIntensity: 1.0,  fogDensity: 0.0040, ambIntensity: 0.4 },
  night:   { angle: 0.95,  label: 'night',    skyTop: 0x0a0a2e, skyBot: 0x191970, sunColor: 0x334466, sunIntensity: 0.2,  fogDensity: 0.0060, ambIntensity: 0.2 },
};

// 天气模式
const WEATHER_MODES = {
  clear:   { rainRate: 0,     snowRate: 0,     skyTint: 0xffffff, fogDensityMul: 1.0,  sunIntensityMul: 1.0 },
  rain:    { rainRate: 8000,  snowRate: 0,     skyTint: 0x8899aa, fogDensityMul: 2.5,  sunIntensityMul: 0.4 },
  snow:    { rainRate: 0,     snowRate: 3000,  skyTint: 0xccddee, fogDensityMul: 2.0,  sunIntensityMul: 0.5 },
  overcast:{ rainRate: 0,     snowRate: 0,     skyTint: 0x999999, fogDensityMul: 3.0,  sunIntensityMul: 0.3 },
};

// ═══════════════════════════════════════════════════════════
// Sky dome shader
// ═══════════════════════════════════════════════════════════

const SKY_VERT = `
varying vec3 vWorldPos;
void main() {
  vec4 worldPos = modelMatrix * vec4(position, 1.0);
  vWorldPos = worldPos.xyz;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;

const SKY_FRAG = `
uniform vec3 uTopColor;
uniform vec3 uBotColor;
uniform float uOffset;
uniform float uExponent;
varying vec3 vWorldPos;
void main() {
  float h = normalize(vWorldPos).y;
  float t = max(pow(max(h + uOffset, 0.0), uExponent), 0.0);
  gl_FragColor = vec4(mix(uBotColor, uTopColor, t), 1.0);
}`;

// ═══════════════════════════════════════════════════════════
// 主工厂
// ═══════════════════════════════════════════════════════════

export function createSkyEnv(scene, sunLight, hemiLight) {
  // ── 天空穹顶 ──
  const skyGeo = new THREE.SphereGeometry(500, 32, 16);
  const skyMat = new THREE.ShaderMaterial({
    vertexShader: SKY_VERT,
    fragmentShader: SKY_FRAG,
    uniforms: {
      uTopColor:  { value: new THREE.Color(0x4682b4) },
      uBotColor:  { value: new THREE.Color(0x87ceeb) },
      uOffset:    { value: 0.1 },
      uExponent:  { value: 0.6 },
    },
    side: THREE.BackSide,
    depthWrite: false,
  });
  const sky = new THREE.Mesh(skyGeo, skyMat);
  sky.renderOrder = -1;
  sky.name = 'skyDome';
  scene.add(sky);

  // ── scene.background 兜底：即使穹顶被裁掉，背景也是天空色而非黑色 ──
  // 用 Color 对象，_applyTimeSlot 时同步更新。
  scene.background = new THREE.Color(0x87ceeb);

  // ── 雾 ──
  scene.fog = new THREE.FogExp2(0x87ceeb, 0.0015);

  // ── 接管 Lighting.js 的 sun + hemi；补充 AmbientLight ──
  const sun = sunLight;
  const hemi = hemiLight;
  const ambient = new THREE.AmbientLight(0xffffff, 0.8);
  scene.add(ambient);

  // ── 雨粒子系统 ──
  const RAIN_AREA = 80;       // 降雨覆盖范围 (m)
  const RAIN_HEIGHT = 40;     // 雨滴下落高度 (m)
  const MAX_DROPS = 10000;    // 最大粒子数

  let rainMesh = null;
  let rainCount = 0;

  function _buildRainMesh(count) {
    if (rainMesh) {
      scene.remove(rainMesh);
      rainMesh.geometry.dispose();
      rainMesh.material.dispose();
      rainMesh = null;
    }
    if (count <= 0) return;

    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(count * 3);
    const velocities = new Float32Array(count);  // 下落速度（存于 userData）

    for (let i = 0; i < count; i++) {
      positions[i * 3]     = (Math.random() - 0.5) * RAIN_AREA;
      positions[i * 3 + 1] = Math.random() * RAIN_HEIGHT;
      positions[i * 3 + 2] = (Math.random() - 0.5) * RAIN_AREA;
      velocities[i] = 15 + Math.random() * 10;  // 15-25 m/s
    }
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

    const mat = new THREE.PointsMaterial({
      color: 0xaaccff,
      size: 0.15,
      transparent: true,
      opacity: 0.5,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });

    rainMesh = new THREE.Points(geo, mat);
    rainMesh.userData.velocities = velocities;
    rainMesh.userData.area = RAIN_AREA;
    rainMesh.userData.height = RAIN_HEIGHT;
    scene.add(rainMesh);
  }

  // ── 状态 ──
  let _timeOfDay = 'noon';
  let _weather = 'clear';
  let _dayMode = true;  // 兼容旧 API
  let _camera = null;   // 相机引用，每帧让穹顶跟随相机

  // ── 更新函数 ──

  /** 根据时段计算太阳位置（沿圆弧从东到西） */
  function _setSunPosition(angle) {
    // angle: 0=东, 0.25=天顶, 0.5=西, 0.75=地底(夜), 1=东
    const a = angle * TAU;
    const r = 100;
    const x = Math.cos(a) * r;
    const y = Math.sin(a) * r;
    const z = 30;  // 偏南
    sun.position.set(x, Math.max(y, -10), z);
  }

  /** 应用时段参数 */
  function _applyTimeSlot(slot) {
    const t = TIME_SLOTS[slot] || TIME_SLOTS.noon;
    const w = WEATHER_MODES[_weather] || WEATHER_MODES.clear;

    // 天空颜色
    const skyTop = new THREE.Color(t.skyTop).lerp(new THREE.Color(w.skyTint), 0.5);
    const skyBot = new THREE.Color(t.skyBot).lerp(new THREE.Color(w.skyTint), 0.5);
    skyMat.uniforms.uTopColor.value.copy(skyTop);
    skyMat.uniforms.uBotColor.value.copy(skyBot);

    // 太阳
    _setSunPosition(t.angle);
    sun.color.set(t.sunColor);
    sun.intensity = t.sunIntensity * w.sunIntensityMul;

    // 环境光
    ambient.intensity = t.ambIntensity * w.sunIntensityMul;
    // 半球光（天空/地面环境）
    if (hemi) {
      hemi.intensity = t.ambIntensity * 0.4 * w.sunIntensityMul;
    }

    // 雾
    const fogDensity = t.fogDensity * w.fogDensityMul;
    scene.fog.density = fogDensity;
    scene.fog.color.copy(skyBot);

    // scene.background 同步为天空底色，保证穹顶被裁掉时背景不黑
    if (scene.background && scene.background.isColor) {
      scene.background.copy(skyBot);
    }

    // 雨
    const rainRate = w.rainRate;
    if (rainRate !== rainCount) {
      rainCount = rainRate;
      _buildRainMesh(rainRate);
    }
  }

  /** 设置时段（兼容旧 day/night bool） */
  function setTimeOfDay(time) {
    if (time === true)  { _timeOfDay = 'noon'; _dayMode = true; }
    else if (time === false) { _timeOfDay = 'night'; _dayMode = false; }
    else if (TIME_SLOTS[time]) { _timeOfDay = time; _dayMode = (time !== 'night'); }
    _applyTimeSlot(_timeOfDay);
  }

  /** 设置天气 */
  function setWeather(mode) {
    if (!WEATHER_MODES[mode]) mode = 'clear';
    _weather = mode;
    _applyTimeSlot(_timeOfDay);
  }

  /** 获取当前时段/天气 */
  function getTimeOfDay() { return _timeOfDay; }
  function getWeather() { return _weather; }
  function isDay() { return _dayMode; }

  /** 设置相机引用，穹顶每帧跟随相机位置（半径500的球罩住相机视野） */
  function setCamera(cam) { _camera = cam; }

  // ── 每帧 tick ──
  function tick(dt) {
    // 穹顶跟随相机：半径500的球必须以相机为中心，否则 ego 开远后穹顶
    // 被视锥裁掉，露出 renderer 默认黑 clear color。
    if (_camera) {
      sky.position.copy(_camera.position);
    }

    // 雨粒子动画
    if (rainMesh && rainCount > 0) {
      const pos = rainMesh.geometry.attributes.position;
      const vel = rainMesh.userData.velocities;
      const area = rainMesh.userData.area;
      const height = rainMesh.userData.height;
      const arr = pos.array;

      for (let i = 0; i < rainCount; i++) {
        const idx = i * 3;
        arr[idx + 1] -= vel[i] * (dt || 0.016);
        if (arr[idx + 1] < -2) {
          arr[idx + 1] = height;
          arr[idx] = (Math.random() - 0.5) * area;
          arr[idx + 2] = (Math.random() - 0.5) * area;
        }
      }
      pos.needsUpdate = true;
    }
  }

  /** 获取太阳引用（供外部调色温） */
  function getSun() { return sun; }
  function getAmbient() { return ambient; }

  /** 清理（不删除 sun/ambient，它们归 Lighting.js 管） */
  function dispose() {
    scene.remove(sky);
    skyGeo.dispose();
    skyMat.dispose();
    if (rainMesh) {
      scene.remove(rainMesh);
      rainMesh.geometry.dispose();
      rainMesh.material.dispose();
    }
  }

  // 初始应用
  _applyTimeSlot(_timeOfDay);

  return {
    setTimeOfDay, setWeather,
    getTimeOfDay, getWeather, isDay,
    setCamera, tick, dispose,
    getSun, getAmbient,
  };
}