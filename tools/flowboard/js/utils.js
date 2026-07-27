// ═════════════════════════════════════════════════════════════════════
// utils.js — Shared utility functions for FlowBoard dashboard
// ES module; no side effects on import.
// All globally-referenced functions are also set on window.* so inline
// HTML onclick handlers and the monolithic <script> block continue to work.
// ═════════════════════════════════════════════════════════════════════
import * as THREE from 'three';

// ── Diagnostic state ─────────────────────────────────────────────
const _diag = { msgs: [], seen: {} };

/**
 * reportDiag — append a diagnostic message to #diag-bar.
 * Deduplicates repeated messages and keeps at most 8 entries.
 */
export function reportDiag(where, err) {
  var msg = (err && err.message) ? err.message : String(err);
  var key = where + ':' + msg;
  if (_diag.seen[key]) {
    _diag.seen[key].n++;
  } else {
    _diag.seen[key] = { n: 1, where: where, msg: msg };
    _diag.msgs.push(key);
  }
  if (_diag.msgs.length > 8) _diag.msgs.shift();
  try {
    var bar = document.getElementById('diag-bar');
    if (!bar) return;
    bar.style.display = '';
    bar.innerHTML = _diag.msgs.map(function (k) {
      var e = _diag.seen[k];
      return '⚠ <b>' + e.where + '</b>: ' + e.msg + (e.n > 1 ? ' ×' + e.n : '');
    }).join('<br>');
  } catch (_) { /* silent */ }
}

/**
 * clearDiag — clear all diagnostics and hide the bar.
 */
export function clearDiag() {
  _diag.msgs = [];
  _diag.seen = {};
  var b = document.getElementById('diag-bar');
  if (b) { b.style.display = 'none'; b.innerHTML = ''; }
}

/**
 * safeCall — wrap fn in try/catch.  On throw, report via reportDiag
 * and return false.  Returns true on success.
 */
export function safeCall(where, fn) {
  try {
    fn();
    return true;
  } catch (err) {
    reportDiag(where, err);
    if (window.console && console.warn) console.warn('[' + where + ']', err);
    return false;
  }
}

// ── Three.js geometry helpers ────────────────────────────────────

// ── Contact (underbody) shadow ─────────────────────────────────
// 程序化软边径向渐变纹理，铺在车底地面模拟 AO 接触阴影，补充
// DirectionalLight 硬阴影（远处精度下降时尤其重要）。纹理全局共享。
let _contactShadowTex = null;
function _getContactShadowTex() {
  if (_contactShadowTex) return _contactShadowTex;
  var c = document.createElement('canvas');
  c.width = 128; c.height = 128;
  var ctx = c.getContext('2d');
  var grd = ctx.createRadialGradient(64, 64, 6, 64, 64, 62);
  grd.addColorStop(0, 'rgba(0,0,0,0.58)');
  grd.addColorStop(0.55, 'rgba(0,0,0,0.30)');
  grd.addColorStop(1, 'rgba(0,0,0,0)');
  ctx.fillStyle = grd;
  ctx.fillRect(0, 0, 128, 128);
  _contactShadowTex = new THREE.CanvasTexture(c);
  return _contactShadowTex;
}

/**
 * _buildContactShadow — 车底接触阴影 mesh（水平平面，软边径向渐变）。
 * @param {number} w  阴影纵向宽度（沿车身 X）
 * @param {number} d  阴影横向深度（沿车身 Z）
 * @returns {THREE.Mesh}  rotation.x = -π/2 的水平平面，y≈0.015
 */
export function _buildContactShadow(w, d) {
  var mat = new THREE.MeshBasicMaterial({
    map: _getContactShadowTex(),
    transparent: true, opacity: 0.75,
    depthWrite: false,
    polygonOffset: true, polygonOffsetFactor: -2
  });
  var m = new THREE.Mesh(new THREE.PlaneGeometry(w, d), mat);
  m.rotation.x = -Math.PI / 2;
  m.position.y = 0.015;
  m.renderOrder = -1;
  return m;
}

// ── Shared car geometry (sedan) ──────────────────────────────────
// Pre-built geometries reused by every _buildSedan call so we only
// allocate geometry once. Phase 6 画质升级：增加圆角、车窗、轮毂等细节。
const _carGeom = {};

/* 共享车轴横梁材质（避免每辆车重复创建 MeshStandardMaterial） */
let _axleBeamMat = null;
function _getAxleBeamMat(T) {
  if (!_axleBeamMat) _axleBeamMat = new T.MeshStandardMaterial({ color: 0x333333, metalness: 0.8, roughness: 0.3 });
  return _axleBeamMat;
}

/**
 * initCarMesh — create shared geometry objects used by _buildSedan.
 * Safe to call multiple times; only allocates on the first call.
 */
export function initCarMesh() {
  if (_carGeom.body) return; // already initialized
  const T = window.THREE;
  // ── 曲面车身（ExtrudeGeometry + Shape + bevel 倒角）──
  // 替代旧 BoxGeometry 拼装，做出极品飞车风格的平滑腰线/引擎盖/车顶曲线。
  // body：下半部分（底盘+引擎盖+后备箱），车侧轮廓 splineThru 平滑。
  // 坐标系：Shape 在 X-Y 平面，X=车长方向(+前/-后)，Y=高度(相对 body 原点)；
  //         ExtrudeGeometry 沿 +Z 拉伸出车宽，translate Z=-depth/2 居中。
  _carGeom.body = (function () {
    var s = new T.Shape();
    // 车侧轮廓点（逆时针）：前杠底→前杠上→引擎盖→后备箱→后杠→底盘
    var pts = [
      new T.Vector2(2.10, -0.36), new T.Vector2(2.22, -0.20),
      new T.Vector2(2.22, 0.08), new T.Vector2(2.08, 0.30),
      new T.Vector2(1.45, 0.36), new T.Vector2(-1.20, 0.36),
      new T.Vector2(-1.95, 0.33), new T.Vector2(-2.12, 0.26),
      new T.Vector2(-2.22, 0.08), new T.Vector2(-2.22, -0.20),
      new T.Vector2(-2.10, -0.36)
    ];
    s.moveTo(pts[0].x, pts[0].y);
    s.splineThru(pts.slice(1));
    s.closePath();
    var geo = new T.ExtrudeGeometry(s, {
      depth: 1.86, bevelEnabled: true, bevelThickness: 0.04, bevelSize: 0.04,
      bevelSegments: 3, curveSegments: 16, steps: 1
    });
    geo.translate(0, 0, -0.93);  // Z 居中（车宽 1.86 / 2）
    return geo;
  })();
  // cabin：上半部分（前挡风+车顶+后挡风），更窄(1.40m)形成车窗框
  _carGeom.cabin = (function () {
    var s = new T.Shape();
    var pts = [
      new T.Vector2(1.18, -0.27), new T.Vector2(1.05, 0.05),
      new T.Vector2(0.85, 0.23), new T.Vector2(-0.55, 0.27),
      new T.Vector2(-0.80, 0.18), new T.Vector2(-0.92, -0.13),
      new T.Vector2(-1.18, -0.27)
    ];
    s.moveTo(pts[0].x, pts[0].y);
    s.splineThru(pts.slice(1));
    s.closePath();
    var geo = new T.ExtrudeGeometry(s, {
      depth: 1.40, bevelEnabled: true, bevelThickness: 0.03, bevelSize: 0.03,
      bevelSegments: 2, curveSegments: 12, steps: 1
    });
    geo.translate(0, 0, -0.70);  // Z 居中（cabin 宽 1.40 / 2）
    return geo;
  })();
  _carGeom.windshield = new T.BoxGeometry(0.06, 0.50, 1.44);
  _carGeom.rearWindow = new T.BoxGeometry(0.06, 0.40, 1.34);
  _carGeom.sideWindow = new T.BoxGeometry(1.55, 0.32, 1.46);
  // 前/后保险杠：带弧度感的宽体
  _carGeom.frontBumper = new T.BoxGeometry(0.18, 0.35, 1.84, 1, 1, 4);
  _carGeom.rearBumper = new T.BoxGeometry(0.16, 0.35, 1.84, 1, 1, 4);
  // 侧裙：降低视觉重心
  _carGeom.sideSkirt = new T.BoxGeometry(2.6, 0.06, 1.92, 4, 1, 1);
  // 轮胎：加宽并带胎纹环
  _carGeom.wheel = new T.CylinderGeometry(0.33, 0.33, 0.26, 24);
  _carGeom.tread = new T.TorusGeometry(0.33, 0.018, 6, 24);
  _carGeom.hubcap = new T.CylinderGeometry(0.18, 0.18, 0.27, 16);
  // 车灯：椭圆透镜造型（前灯加大更醒目，尾灯维持窄条不占视觉）
  _carGeom.headlight = new T.BoxGeometry(0.10, 0.20, 0.50, 1, 1, 3);
  _carGeom.taillight = new T.BoxGeometry(0.06, 0.16, 0.42, 1, 1, 3);
  _carGeom.mirror = new T.BoxGeometry(0.14, 0.2, 0.1, 1, 1, 2);
  _carGeom.doorHandle = new T.BoxGeometry(0.12, 0.04, 0.05);
  _carGeom.grille = new T.BoxGeometry(0.05, 0.26, 1.15, 1, 1, 4);
  _carGeom.licensePlate = new T.BoxGeometry(0.04, 0.14, 0.42, 1, 1, 3);
  _carGeom.antenna = new T.CylinderGeometry(0.01, 0.01, 0.45, 6);
  // 车轴横梁（圆柱沿 Z 连接左右轮，radius 0.04，长度 1.9）
  _carGeom.axleBeam = new T.CylinderGeometry(0.04, 0.04, 1.9, 6);
  _carGeom.axleBeam.rotateX(Math.PI / 2);

  // ── SU7 标志性元素几何（贯穿式 LED 尾灯条 + 后扩散器 + 前唇 + 扰流板 + 内饰）──
  // 贯穿式 LED 尾灯条：长 1.70m，薄 0.08m 高，厚 0.04m，覆盖整个车尾
  _carGeom.ledTailBar = new T.BoxGeometry(0.04, 0.08, 1.70);
  // 后扩散器主体 + 前唇 splitter：薄长条
  _carGeom.diffuser = new T.BoxGeometry(0.15, 0.02, 1.60);
  _carGeom.splitter = new T.BoxGeometry(0.15, 0.02, 1.98);
  _carGeom.diffuserFin = new T.BoxGeometry(0.12, 0.06, 0.01);
  // 小扰流板（lip spoiler）+ 支架
  _carGeom.lipSpoiler = new T.BoxGeometry(0.08, 0.02, 1.60);
  _carGeom.spoilerSupport = new T.BoxGeometry(0.03, 0.04, 0.03);
  // 高位刹车灯（中央，贴后窗下沿）
  _carGeom.highBrake = new T.BoxGeometry(0.03, 0.025, 0.60);
  // 雾灯发光球（前杠两侧）
  _carGeom.fogSphere = new T.SphereGeometry(0.10, 16, 16);
  _carGeom.fogHousing = new T.BoxGeometry(0.04, 0.16, 0.22);
  // 内饰：方向盘（torus）+ 座椅 + 仪表盘
  _carGeom.intWheel = new T.TorusGeometry(0.14, 0.015, 8, 20);
  _carGeom.seat = new T.BoxGeometry(0.50, 0.45, 0.35);
  _carGeom.dash = new T.BoxGeometry(0.60, 0.15, 1.50);
  _carGeom.dashScreen = new T.BoxGeometry(0.02, 0.14, 0.80);
  // 腰线 / 肩线（车身侧特征线，subtle 高亮条）
  _carGeom.waistLine = new T.BoxGeometry(3.80, 0.01, 0.005);
  _carGeom.shoulderLine = new T.BoxGeometry(1.80, 0.008, 0.005);
}

/**
 * _buildWheel — helper 构建带轮胎+胎纹+轮毂+5辐条的车轮。
 */
function _buildWheel(T) {
  var wg = new T.Group();
  var rubberMat = new T.MeshStandardMaterial({ color: 0x111111, metalness: 0.05, roughness: 0.82 });
  var tire = new T.Mesh(_carGeom.wheel, rubberMat);
  tire.rotation.z = Math.PI / 2;
  tire.castShadow = true;
  wg.add(tire);
  // 胎纹：两条环增加侧面细节
  var treadMat = new T.MeshStandardMaterial({ color: 0x0a0a0a, metalness: 0.0, roughness: 0.95 });
  var tread1 = new T.Mesh(_carGeom.tread, treadMat);
  tread1.rotation.x = Math.PI / 2; tread1.position.x = 0.07;
  wg.add(tread1);
  var tread2 = tread1.clone(); tread2.position.x = -0.07;
  wg.add(tread2);
  // 轮毂
  var hubMat = new T.MeshStandardMaterial({ color: 0xaaaaaa, metalness: 0.7, roughness: 0.25 });
  var hub = new T.Mesh(_carGeom.hubcap, hubMat);
  hub.rotation.z = Math.PI / 2;
  wg.add(hub);
  // 5 辐条星形轮毂
  var spokeMat = new T.MeshStandardMaterial({ color: 0xbbbbbb, metalness: 0.6, roughness: 0.3 }); // exempt: wheel spoke
  for (var si = 0; si < 5; si++) {
    var spoke = new T.Mesh(new T.BoxGeometry(0.28, 0.035, 0.025), spokeMat);
    spoke.rotation.z = Math.PI / 2;
    spoke.rotation.x = (Math.PI * 2 / 5) * si;
    wg.add(spoke);
  }
  // 中心盖
  var cap = new T.Mesh(new T.CylinderGeometry(0.05, 0.05, 0.28, 12), hubMat);
  cap.rotation.z = Math.PI / 2;
  wg.add(cap);
  wg.userData.isWheel = true;
  return wg;
}

/**
 * _buildSedan — build a detailed sedan mesh group.
 * @param {number} color        body colour (e.g. 0x4488dd)
 * @param {number} secondaryColor  roof / cabin colour (e.g. 0x3377bb)
 * @param {boolean} addSpots    是否挂 2 个车头 SpotLight（默认 true）。
 *                              NPC 必须传 false：24 个 NPC 各 2 灯 = 48 个动态
 *                              光源会把 forward renderer 的 shader 撑爆。
 * @returns {THREE.Group}
 */
export function _buildSedan(color, secondaryColor, addSpots) {
  if (addSpots === undefined) addSpots = true;
  initCarMesh(); // ensure shared geometries exist
  const T = window.THREE;
  var g = new T.Group();
  // 真实车漆：MeshPhysicalMaterial + clearcoat（清漆层）+ sheen（绒光），
  // 配合 scene.environment 的 PMREM 环境贴图产生高光反射。
  var bodyMat = new T.MeshPhysicalMaterial({
    color: color, metalness: 0.15, roughness: 0.22, envMapIntensity: 1.1,
    clearcoat: 1.0, clearcoatRoughness: 0.06, sheen: new T.Color(0.4, 0.4, 0.4)
  });
  var cabinMat = new T.MeshPhysicalMaterial({
    color: secondaryColor, metalness: 0.4, roughness: 0.28, envMapIntensity: 0.9,
    clearcoat: 0.6, clearcoatRoughness: 0.12
  });
  var glassMat = new T.MeshPhysicalMaterial({
    color: 0x223344, metalness: 0.9, roughness: 0.04, envMapIntensity: 1.3, // exempt: glass
    clearcoat: 1.0, clearcoatRoughness: 0.02
  });
  var blackMat = new T.MeshStandardMaterial({ color: 0x111111, roughness: 0.7 });
  var chromeMat = new T.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.85, roughness: 0.14, envMapIntensity: 1.2 });
  var archMat = new T.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.85 });

  // ── 曲面车身（ExtrudeGeometry）：body 已含引擎盖/后备箱曲面，无需 hood/trunkDeck ──
  var body = new T.Mesh(_carGeom.body, bodyMat);
  body.position.y = 0.52; body.castShadow = true; body.receiveShadow = true; g.add(body);

  // cabin：车顶曲面（前挡风根→车顶→后挡风根）
  var cabin = new T.Mesh(_carGeom.cabin, cabinMat);
  cabin.position.set(0.05, 1.15, 0); cabin.castShadow = true; g.add(cabin);

  // Windshield / Rear window：玻璃薄方块，贴在 body 与 cabin 接缝处
  var ws = new T.Mesh(_carGeom.windshield, glassMat);
  ws.position.set(1.05, 1.05, 0); ws.rotation.z = -0.55; g.add(ws);
  var rw = new T.Mesh(_carGeom.rearWindow, glassMat);
  rw.position.set(-1.00, 1.00, 0); rw.rotation.z = 0.50; g.add(rw);
  // Side windows：贴在 cabin 侧面（cabin 宽 1.40，body 宽 1.86，玻璃 1.46 略宽于 cabin 露出）
  var sideWin = new T.Mesh(_carGeom.sideWindow, glassMat);
  sideWin.position.set(0.02, 1.18, 0); g.add(sideWin);

  // ── 轮拱改用 TorusGeometry 半圆弧（不再是 Box）──
  // TorusGeometry(radius=0.40, tube=0.06, arc=π) → 半圆，rotation.y=π/2 让圆环面对 X 轴
  var archGeo = new T.TorusGeometry(0.40, 0.06, 8, 18, Math.PI);
  var archPositions = [
    [1.35, 0.95], [1.35, -0.95], [-1.35, 0.95], [-1.35, -0.95]
  ];
  for (var ai = 0; ai < 4; ai++) {
    var arch = new T.Mesh(archGeo, archMat);
    arch.rotation.y = Math.PI / 2;  // 圆环从 X-Y 平面转到 Y-Z 平面，法线朝 X
    arch.position.set(archPositions[ai][0], 0.34, archPositions[ai][1]);
    g.add(arch);
  }

  // Side skirts
  var skirt = new T.Mesh(_carGeom.sideSkirt, bodyMat);
  skirt.position.set(0, 0.12, 0); g.add(skirt);

  // Bumpers
  var fb = new T.Mesh(_carGeom.frontBumper, chromeMat);
  fb.position.set(2.2, 0.35, 0); fb.castShadow = true; g.add(fb);
  var rb = new T.Mesh(_carGeom.rearBumper, chromeMat);
  rb.position.set(-2.2, 0.35, 0); rb.castShadow = true; g.add(rb);

  // Front grille + license plate
  var grille = new T.Mesh(_carGeom.grille, blackMat);
  grille.position.set(2.16, 0.56, 0); g.add(grille);
  var plateF = new T.Mesh(_carGeom.licensePlate, new T.MeshStandardMaterial({ color: 0xffffff, roughness: 0.4 }));
  plateF.position.set(2.19, 0.26, 0); g.add(plateF);
  var plateR = plateF.clone();
  plateR.position.set(-2.19, 0.26, 0); g.add(plateR);

  // Side mirrors
  var mL = new T.Mesh(_carGeom.mirror, bodyMat);
  mL.position.set(1.02, 0.92, 0.98); g.add(mL);
  var mR = new T.Mesh(_carGeom.mirror, bodyMat);
  mR.position.set(1.02, 0.92, -0.98); g.add(mR);

  // Door handles
  var dhL = new T.Mesh(_carGeom.doorHandle, chromeMat);
  dhL.position.set(0.35, 0.72, 0.95); g.add(dhL);
  var dhR = new T.Mesh(_carGeom.doorHandle, chromeMat);
  dhR.position.set(0.35, 0.72, -0.95); g.add(dhR);

  // Antenna
  var ant = new T.Mesh(_carGeom.antenna, chromeMat);
  ant.position.set(-0.9, 1.4, 0.55); g.add(ant);

  // 自动驾驶小蓝灯 ×2（车尾左右，量产 ADS 指示灯，加大提高穿透性）
  var adsLightGeo = new T.CylinderGeometry(0.10, 0.12, 0.12, 12);
  var adsLightMat = new T.MeshStandardMaterial({ color: 0x3388ff, emissive: 0x2277ee, emissiveIntensity: 2.5, roughness: 0.10, metalness: 0.2 });
  function addAdsLight(z, side) {
    var m = new T.Mesh(adsLightGeo, adsLightMat);
    m.name = 'ads_indicator_' + side;
    m.position.set(-1.75, 0.88, z);
    g.add(m);
  }
  addAdsLight(0.48, 'L'); addAdsLight(-0.48, 'R');

  // Wheels (4) — 前后轴分别建 Group，前轴支持转向动画。
  // 轴 Group pivot 在轴中心，车轮相对轴定位，rotation.y 转向时车轮绕轴心自转而非绕车身公转。
  var axleX = 1.35, rearAxleX = -1.35, wheelY = 0.34, wheelZ = 0.95;
  var wheels = [];
  var frontAxle = new T.Group();
  frontAxle.position.set(axleX, wheelY, 0);
  var rearAxle = new T.Group();
  rearAxle.position.set(rearAxleX, wheelY, 0);
  for (var wi = 0; wi < 4; wi++) {
    var wh = _buildWheel(T);
    var zSign = (wi === 0 || wi === 2) ? wheelZ : -wheelZ;
    wh.position.set(0, 0, zSign);
    if (wi < 2) frontAxle.add(wh); else rearAxle.add(wh);
    wheels.push(wh);
  }
  // 车轴可视化横梁（共享 geo + mat，避免每车重复创建）
  frontAxle.add(new T.Mesh(_carGeom.axleBeam, _getAxleBeamMat(T)));
  rearAxle.add(new T.Mesh(_carGeom.axleBeam, _getAxleBeamMat(T)));
  g.add(frontAxle);
  g.add(rearAxle);
  g.userData.frontAxle = frontAxle;
  g.userData.rearAxle = rearAxle;
  g.userData.wheels = wheels;
  // 真实轮半径（_carGeom.wheel 半径 0.33），scene3d 滚动动画角速度 = v·dt / r
  g.userData.wheelRadius = 0.33;

  // Headlights with chrome bezel（lens mesh 加 name 供 _setVehicleLights 查找）
  var hlMat = new T.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 1.7, roughness: 0.12 });
  var bezelMat = new T.MeshStandardMaterial({ color: 0xdddddd, metalness: 0.6, roughness: 0.25 });
  var headlightMeshes = [];
  function addHeadlight(z, side) {
    var bg = new T.Group();
    var lens = new T.Mesh(_carGeom.headlight, hlMat);
    lens.name = 'headlight_' + side;
    headlightMeshes.push(lens);
    var bezel = new T.Mesh(new T.BoxGeometry(0.09, 0.18, 0.46, 1, 1, 3), bezelMat);
    bezel.position.x = -0.02;
    bg.add(bezel); bg.add(lens);
    bg.position.set(2.16, 0.58, z);
    g.add(bg);
    return bg;
  }
  addHeadlight(0.58, 'L'); addHeadlight(-0.58, 'R');

  // Taillights（= 刹车灯，name brakelight_ 供 _setVehicleLights 查找）
  // SU7 标志性贯穿式 LED 尾灯条：长 1.70m 横贯车尾，左右各保留独立 brakelight_L/R 节点
  // 供 _setVehicleLights 按侧切换（实际物理是整条同时亮，但保留命名兼容现有灯光状态机）。
  var tlMat = new T.MeshStandardMaterial({ color: 0xff1a1a, emissive: 0xff1a1a, emissiveIntensity: 1.7, roughness: 0.12 });
  var tlBarMat = tlMat.clone();  // 贯穿条本体材质（与左右透镜同步切换）
  var brakeLightMeshes = [];
  // 贯穿 LED 条本体
  var ledBar = new T.Mesh(_carGeom.ledTailBar, tlBarMat);
  ledBar.position.set(-2.18, 0.58, 0);
  ledBar.name = 'brakelight_bar';
  g.add(ledBar);
  brakeLightMeshes.push(ledBar);
  // 左右透镜（与 LED 条同步切换，保留 brakelight_L/R 命名兼容 glTF 扫描逻辑）
  function addTaillight(z, side) {
    var lens = new T.Mesh(_carGeom.taillight, tlMat);
    lens.name = 'brakelight_' + side;
    lens.position.set(-2.18, 0.58, z);
    g.add(lens);
    brakeLightMeshes.push(lens);
  }
  addTaillight(0.58, 'L'); addTaillight(-0.58, 'R');
  // 中央高位刹车灯（贴后窗下沿，SU7 量产车配置）
  var highBrakeMat = new T.MeshStandardMaterial({ color: 0xff0000, emissive: 0xff0000, emissiveIntensity: 1.7, roughness: 0.15 });
  var highBrake = new T.Mesh(_carGeom.highBrake, highBrakeMat);
  highBrake.position.set(-1.95, 1.05, 0);
  highBrake.name = 'brakelight_high';
  g.add(highBrake);
  brakeLightMeshes.push(highBrake);

  // Turn signals（琥珀色，四角，name turnsignal_ 供 _setVehicleLights 查找）
  var turnMat = new T.MeshStandardMaterial({ color: 0xff8800, emissive: 0xff6600, emissiveIntensity: 2.0, roughness: 0.10 });
  var turnGeo = new T.BoxGeometry(0.10, 0.18, 0.36, 1, 1, 2);
  var turnSignals = {};
  function addTurnSignal(x, z, key) {
    var m = new T.Mesh(turnGeo, turnMat);
    m.name = 'turnsignal_' + key;
    m.position.set(x, 0.58, z);
    g.add(m);
    turnSignals[key] = m;
  }
  addTurnSignal(2.15, 0.82, 'FL');
  addTurnSignal(2.15, -0.82, 'FR');
  addTurnSignal(-2.16, 0.82, 'RL');
  addTurnSignal(-2.16, -0.82, 'RR');

  // userData 供 _setVehicleLights 控制 emissiveIntensity（替代 glTF 版按名扫描的结果）
  g.userData.headlights = headlightMeshes;
  g.userData.brakeLights = brakeLightMeshes;
  g.userData.turnSignals = turnSignals;

  // 车头灯真实照射（仅 ego 用，NPC 由 _setVehicleLights 切 emissive）
  var spotL = new T.SpotLight(0xffffee, 3.0, 60, 0.45, 0.4, 1.2);
  spotL.position.set(2.2, 0.54, 0.55);
  spotL.target.position.set(18, 0, 0.55);
  spotL.castShadow = false;
  g.add(spotL); g.add(spotL.target);
  var spotR = new T.SpotLight(0xffffee, 3.0, 60, 0.45, 0.4, 1.2);
  spotR.position.set(2.2, 0.54, -0.55);
  spotR.target.position.set(18, 0, -0.55);
  spotR.castShadow = false;
  g.add(spotR); g.add(spotR.target);

  // ── SU7 外观套件：前雾灯 + 前唇 splitter + 后扩散器 + 鳍片 + 小扰流板 ──
  // 前雾灯：球状发光体 + 深色外壳（不含 SpotLight，避免 NPC 24 个实例光源爆 shader）
  var fogMat = new T.MeshStandardMaterial({ color: 0xffdd88, emissive: 0xffdd88, emissiveIntensity: 0, roughness: 0.10, metalness: 0.2 });
  var fogHousingMat = new T.MeshStandardMaterial({ color: 0x111111, roughness: 0.7 });
  function addFogLight(z) {
    var housing = new T.Mesh(_carGeom.fogHousing, fogHousingMat);
    housing.position.set(2.16, 0.25, z);
    g.add(housing);
    var sphere = new T.Mesh(_carGeom.fogSphere, fogMat.clone());
    sphere.position.set(2.18, 0.25, z);
    g.add(sphere);
  }
  addFogLight(0.55); addFogLight(-0.55);

  // 前唇 splitter（降低视觉重心，sport package 标配）
  var splitterMat = new T.MeshStandardMaterial({ color: 0x0a0a15, roughness: 0.5, metalness: 0.7 });
  var splitter = new T.Mesh(_carGeom.splitter, splitterMat);
  splitter.position.set(2.18, 0.13, 0);
  g.add(splitter);

  // 后扩散器 + 5 片垂直鳍片（SU7 后保险杠标志性元素）
  var diffuser = new T.Mesh(_carGeom.diffuser, splitterMat);
  diffuser.position.set(-2.18, 0.13, 0);
  g.add(diffuser);
  for (var fi = -2; fi <= 2; fi++) {
    var fin = new T.Mesh(_carGeom.diffuserFin, splitterMat);
    fin.position.set(-2.12, 0.14, fi * 0.30);
    g.add(fin);
  }

  // 小扰流板（lip spoiler，后备箱边缘，SU7 量产车标配）
  var spoilerMat = new T.MeshStandardMaterial({ color: 0x0a0a15, roughness: 0.3, metalness: 0.8 });
  var spoiler = new T.Mesh(_carGeom.lipSpoiler, spoilerMat);
  spoiler.position.set(-1.85, 0.95, 0);
  g.add(spoiler);
  // 扰流板支架 ×2
  [-0.6, 0.6].forEach(function(z) {
    var support = new T.Mesh(_carGeom.spoilerSupport, spoilerMat);
    support.position.set(-1.85, 0.93, z);
    g.add(support);
  });

  // 车身特征线：腰线（waist line）+ 肩线（shoulder line），subtle 高亮条增加层次感
  var lineMat = new T.MeshStandardMaterial({ color: 0x88aacc, roughness: 0.1, metalness: 0.95 });
  var waistL = new T.Mesh(_carGeom.waistLine, lineMat);
  waistL.position.set(0, 0.58, 0.96); g.add(waistL);
  var waistR = new T.Mesh(_carGeom.waistLine, lineMat);
  waistR.position.set(0, 0.58, -0.96); g.add(waistR);
  var shoulderL = new T.Mesh(_carGeom.shoulderLine, lineMat);
  shoulderL.position.set(0.1, 1.10, 0.86); g.add(shoulderL);
  var shoulderR = new T.Mesh(_carGeom.shoulderLine, lineMat);
  shoulderR.position.set(0.1, 1.10, -0.86); g.add(shoulderR);

  // ── SU7 内饰（透过玻璃可见）：方向盘 + 座椅 + 仪表盘 + 屏幕 ──
  var interiorMat = new T.MeshStandardMaterial({ color: 0x0a0a12, roughness: 0.9 });
  var seatMat = new T.MeshStandardMaterial({ color: 0x151520, roughness: 0.9 });
  // 仪表盘
  var dash = new T.Mesh(_carGeom.dash, interiorMat);
  dash.position.set(0.80, 0.75, 0); g.add(dash);
  // 仪表屏（emissive 蓝色微亮）
  var screenMat = new T.MeshStandardMaterial({ color: 0x001122, emissive: 0x001133, emissiveIntensity: 0.5, roughness: 0.1, metalness: 0.5 });
  var screen = new T.Mesh(_carGeom.dashScreen, screenMat);
  screen.position.set(0.52, 0.80, 0);
  screen.rotation.y = 0.3;
  g.add(screen);
  var intWheelMat = new T.MeshStandardMaterial({ color: 0x222222, roughness: 0.8 });
  // 方向盘 — z=-0.35 = 左舵（中国靠右行驶，驾驶员在左侧）
  var intWheel = new T.Mesh(_carGeom.intWheel, intWheelMat);
  intWheel.position.set(0.70, 0.88, -0.35);
  intWheel.rotation.y = Math.PI / 2;
  g.add(intWheel);
  // 座椅 ×2
  [-0.35, 0.35].forEach(function(z) {
    var seat = new T.Mesh(_carGeom.seat, seatMat);
    seat.position.set(-0.05, 0.75, z);
    g.add(seat);
  });

  // 车底接触阴影：软边径向渐变平面，补充 AO 感
  g.add(_buildContactShadow(4.6, 2.0));

  // 真实尺寸 1:1 建模，标记 realScale 让 scene3d 跳过非均匀缩放
  g.userData.realScale = true;
  return g;
}

// ── Obstacle model ───────────────────────────────────────────────
/**
 * _buildObstacle — build an obstacle mesh shaped by type.
 * car/truck 使用真实尺寸 1:1 建模并标记 g.userData.realScale = true，
 * 让 scene3d 渲染时跳过非均匀缩放（避免圆柱车轮被压成椭圆）。
 * pedestrian/cone 仍是 unit 模型，由 scene3d 按各自尺寸缩放。
 * @param {string} type   obstacle type: 'car'/'truck' (sedan), 'pedestrian'
 *                        (capsule), 'cone' (traffic cone); defaults to sedan.
 * @param {number} color  body colour (e.g. 0xff9944). Legacy call signature
 *                        _buildObstacle(colorNumber) is accepted → treated as car.
 * @returns {THREE.Group}
 */
export function _buildObstacle(type, color) {
  const T = window.THREE;
  // Backward-compat: _buildObstacle(number) → sedan of that colour.
  if (typeof type === 'number') { color = type; type = 'car'; }
  color = color || 0xff9944;
  var g = new T.Group();

  if (type === 'pedestrian') {
    // 胶囊形行人：圆柱身体 + 球形头部（区别于车辆轿车外形）
    var pBody = new T.Mesh(
      new T.CylinderGeometry(0.18, 0.18, 0.7, 12),
      new T.MeshStandardMaterial({ color: color, metalness: 0.1, roughness: 0.6 })
    );
    pBody.position.y = 0.35; pBody.castShadow = true; g.add(pBody);
    var head = new T.Mesh(
      new T.SphereGeometry(0.18, 12, 12),
      new T.MeshStandardMaterial({ color: 0xffd9a0, metalness: 0.1, roughness: 0.5 })
    );
    head.position.y = 0.85; g.add(head);
    return g;
  }

  if (type === 'cone') {
    // 圆锥形路障 + 方形底座
    var cone = new T.Mesh(
      new T.ConeGeometry(0.25, 0.7, 16),
      new T.MeshStandardMaterial({ color: color, metalness: 0.1, roughness: 0.5 })
    );
    cone.position.y = 0.35; cone.castShadow = true; g.add(cone);
    var base = new T.Mesh(
      new T.BoxGeometry(0.5, 0.06, 0.5),
      new T.MeshStandardMaterial({ color: 0x222222, roughness: 0.8 })
    );
    base.position.y = 0.03; g.add(base);
    return g;
  }

  // ── car：直接复用 _buildSedan 的 ExtrudeGeometry 曲面车身（同款车漆/轮拱/车灯）──
  // 这样 NPC 轿车与 ego 同等画质，避免 NPC 是方块、ego 是曲面的割裂感。
  if (type === 'car' || type === 'suv') {
    return _buildSedan(color, color);
  }

  // ── truck：保留方块货箱建模（卡车货箱本身就是方块，曲面反而失真）──
  g.userData.realScale = true;

  var isTruck = true;
  var bodyMat = new T.MeshPhysicalMaterial({
    color: color, metalness: 0.15, roughness: 0.24, envMapIntensity: 1.0,
    clearcoat: 0.9, clearcoatRoughness: 0.08, sheen: new T.Color(0.3, 0.3, 0.3)
  });
  var glassMat = new T.MeshPhysicalMaterial({
    color: 0x223344, metalness: 0.85, roughness: 0.06, envMapIntensity: 1.1, // exempt: glass
    clearcoat: 1.0, clearcoatRoughness: 0.03
  });
  var blackMat = new T.MeshStandardMaterial({ color: 0x151515, roughness: 0.8 });
  var tireMat = new T.MeshStandardMaterial({ color: 0x111111, metalness: 0.05, roughness: 0.82 });
  var hubMat = new T.MeshStandardMaterial({ color: 0xaaaaaa, metalness: 0.6, roughness: 0.3 });

  // 卡车真实尺寸常量
  var BL = 7.0, BH = 1.0, BW = 2.2;
  var wheelR = 0.42, wheelW = 0.32;
  var wFrontX = 2.4, wRearX = -2.4, wZ = 0.95;

  // 车身主体
  var body = new T.Mesh(new T.BoxGeometry(BL, BH, BW, 4, 1, 3), bodyMat);
  body.position.y = 0.52; body.castShadow = true; body.receiveShadow = true; g.add(body);

  {
    // 卡车驾驶室（更高）
    var cab = new T.Mesh(new T.BoxGeometry(2.0, 1.4, BW, 2, 1, 2), bodyMat);
    cab.position.set(BL / 2 - 1.0, 1.2, 0); cab.castShadow = true; g.add(cab);
    // 货箱
    var cargo = new T.Mesh(new T.BoxGeometry(BL - 2.5, 1.6, BW - 0.1, 2, 1, 2), blackMat);
    cargo.position.set(-0.5, 1.3, 0); cargo.castShadow = true; g.add(cargo);
    // 驾驶室车窗
    var tWin = new T.Mesh(new T.BoxGeometry(0.06, 0.5, BW - 0.3), glassMat);
    tWin.position.set(BL / 2 - 0.2, 1.5, 0); g.add(tWin);
  }

  // 轮拱内衬
  var archGeo = new T.BoxGeometry(0.7, 0.38, 0.3, 2, 1, 1);
  var archPos = [[wFrontX, wZ], [wRearX, wZ]];
  for (var ai2 = 0; ai2 < 2; ai2++) {
    var aL = new T.Mesh(archGeo, blackMat);
    aL.position.set(archPos[ai2][0], 0.34, archPos[ai2][1]); g.add(aL);
    var aR = aL.clone(); aR.position.z = -archPos[ai2][1]; g.add(aR);
  }

  // 车轮 — 前后轴分别建 Group，前轴支持转向动画。
  // 轴 Group pivot 在轴中心，车轮相对轴定位（同 _buildSedan 模式）。
  var wheels = [];
  var wheelGeo = new T.CylinderGeometry(wheelR, wheelR, wheelW, 20);
  var hubGeo = new T.CylinderGeometry(wheelR * 0.55, wheelR * 0.55, wheelW + 0.01, 14);
  var spokeGeo = new T.BoxGeometry(wheelR * 1.6, 0.035, 0.025);
  var frontAxle = new T.Group();
  frontAxle.position.set(wFrontX, wheelR, 0);
  var rearAxle = new T.Group();
  rearAxle.position.set(wRearX, wheelR, 0);
  for (var wi = 0; wi < 4; wi++) {
    var wg = new T.Group();
    var tire = new T.Mesh(wheelGeo, tireMat);
    tire.rotation.z = Math.PI / 2; tire.castShadow = true; wg.add(tire);
    var hub = new T.Mesh(hubGeo, hubMat);
    hub.rotation.z = Math.PI / 2; wg.add(hub);
    for (var si = 0; si < 5; si++) {
      var spoke = new T.Mesh(spokeGeo, hubMat);
      spoke.rotation.z = Math.PI / 2;
      spoke.rotation.x = (Math.PI * 2 / 5) * si;
      wg.add(spoke);
    }
    var zSign = (wi === 0 || wi === 2) ? wZ : -wZ;
    wg.position.set(0, 0, zSign);
    wg.userData.isWheel = true;
    wheels.push(wg);
    if (wi < 2) frontAxle.add(wg); else rearAxle.add(wg);
  }
  // 车轴可视化横梁（共享 geo + mat）
  frontAxle.add(new T.Mesh(_carGeom.axleBeam, _getAxleBeamMat(T)));
  rearAxle.add(new T.Mesh(_carGeom.axleBeam, _getAxleBeamMat(T)));
  g.add(frontAxle);
  g.add(rearAxle);
  g.userData.frontAxle = frontAxle;
  g.userData.rearAxle = rearAxle;
  g.userData.wheels = wheels;
  // 卡车真实轮半径（wheelR = 0.42），scene3d 滚动动画角速度 = v·dt / r
  g.userData.wheelRadius = wheelR;

  // 车灯（简化 emissive，不加 SpotLight 避免 24 NPC × 2 灯 = 48 动态光源性能灾难）
  // 命名 + userData 引用与 _buildSedan 一致，_setVehicleLights 才能切换刹车/转向/大灯。
  var headMat = new T.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 1.2, roughness: 0.2 });
  var tailMat = new T.MeshStandardMaterial({ color: 0xff2222, emissive: 0xff1111, emissiveIntensity: 1.2, roughness: 0.2 });
  var hlGeo = new T.BoxGeometry(0.10, 0.20, 0.50, 1, 1, 2);
  var tlGeo = new T.BoxGeometry(0.06, 0.16, 0.42, 1, 1, 2);
  var lightX = BL / 2 - 0.04;
  var headlightMeshes = [], brakeLightMeshes = [];
  function makeObsLight(z, isHead, side) {
    var m = new T.Mesh(isHead ? hlGeo : tlGeo, isHead ? headMat : tailMat);
    m.name = (isHead ? 'headlight_' : 'brakelight_') + side;
    m.position.set(isHead ? lightX : -lightX, 0.58, z);
    g.add(m);
    if (isHead) headlightMeshes.push(m); else brakeLightMeshes.push(m);
  }
  makeObsLight(0.58, true, 'L'); makeObsLight(-0.58, true, 'R');
  makeObsLight(0.58, false, 'L'); makeObsLight(-0.58, false, 'R');

  // 转向灯（琥珀色，四角，name turnsignal_ 供 _setVehicleLights 查找）
  var turnMat = new T.MeshStandardMaterial({ color: 0xff8800, emissive: 0xff6600, emissiveIntensity: 2.0, roughness: 0.10 });
  var turnGeo = new T.BoxGeometry(0.10, 0.18, 0.36, 1, 1, 2);
  var turnSignals = {};
  function addTruckTurn(x, z, key) {
    var m = new T.Mesh(turnGeo, turnMat);
    m.name = 'turnsignal_' + key;
    m.position.set(x, 0.85, z);
    g.add(m);
    turnSignals[key] = m;
  }
  addTruckTurn(BL / 2 - 0.02, 1.02, 'FL');
  addTruckTurn(BL / 2 - 0.02, -1.02, 'FR');
  addTruckTurn(-BL / 2 + 0.02, 1.02, 'RL');
  addTruckTurn(-BL / 2 + 0.02, -1.02, 'RR');

  g.userData.headlights = headlightMeshes;
  g.userData.brakeLights = brakeLightMeshes;
  g.userData.turnSignals = turnSignals;

  // 车底接触阴影（真实尺寸）
  g.add(_buildContactShadow(BL * 1.1, BW * 1.1));

  return g;
}

// ── Material sanity audit ────────────────────────────────────────
/**
 * _auditSceneMaterials — scan a THREE.Scene for materials whose Color-typed
 * property holds something other than a real THREE.Color (e.g. a raw number
 * mistakenly passed where a Color is required, as with the `sheen: 0.4`
 * MeshPhysicalMaterial bug in _buildSedan/_buildObstacle/models.js). Pure
 * read — safe to call from devtools or from an error handler.
 *
 * @param {THREE.Scene} scene
 * @returns {Array} up to 20 findings: { objectName, objectType, materialType, materialUuid, prop, value, valueType }
 */
export function _auditSceneMaterials(scene) {
  var COLOR_PROPS = ['color', 'emissive', 'sheen', 'specular', 'sheenColor'];
  var findings = [];
  if (!scene || !scene.traverse) return findings;
  scene.traverse(function(obj) {
    if (findings.length >= 20) return;
    if (!(obj.isMesh || obj.isLine || obj.isPoints || obj.isSprite) || !obj.material) return;
    var mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    for (var i = 0; i < mats.length; i++) {
      var m = mats[i];
      if (!m) continue;
      for (var j = 0; j < COLOR_PROPS.length; j++) {
        var prop = COLOR_PROPS[j];
        var v = m[prop];
        if (v === null || v === undefined || v.isColor === true) continue;
        findings.push({
          objectName: obj.name || '(unnamed)',
          objectType: obj.type,
          materialType: m.type,
          materialUuid: m.uuid,
          prop: prop,
          value: typeof v === 'object' ? JSON.stringify(v) : v,
          valueType: typeof v
        });
        if (findings.length >= 20) return;
      }
    }
  });
  return findings;
}

// ── Colour map ───────────────────────────────────────────────────
const _obsColors = {
  car: 0xff9944,
  truck: 0xff4422,
  pedestrian: 0x33ff88,
  cyclist: 0x33ddff,
  cone: 0xff6600
};

// ═════════════════════════════════════════════════════════════════
// Phase 4.9: removed all `window.X = X` assignments here.
// app.js re-publishes the names used by inline-onclick handlers under
// a single `window.flowboard` namespace object. Internal modules
// communicate via ES module imports only.
// ═════════════════════════════════════════════════════════════════
