// ═════════════════════════════════════════════════════════════════════
// utils.js — Shared utility functions for FlowBoard dashboard
// ES module; no side effects on import.
// All globally-referenced functions are also set on window.* so inline
// HTML onclick handlers and the monolithic <script> block continue to work.
// ═════════════════════════════════════════════════════════════════════
const THREE = window.THREE;

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

// ── Card collapse ────────────────────────────────────────────────
/**
 * toggleCard — collapse / expand a card by toggling the "collapsed"
 * class on its parent element (card).
 */
export function toggleCard(hdr) {
  hdr.parentElement.classList.toggle('collapsed');
  // Phase 4.9: call saveState via the flowboard namespace (no global pollute)
  if (window.flowboard && window.flowboard.saveState) window.flowboard.saveState();
  // Let the 3D renderer know it may need to resize after expanding
  setTimeout(function () {
    if (!hdr.parentElement.classList.contains('collapsed')) {
      if (window.flowboard && window.flowboard.resize3D) window.flowboard.resize3D();
    }
  }, 50);
}

// ── Three.js geometry helpers ────────────────────────────────────
/**
 * _makeBox — create a THREE.Mesh with BoxGeometry and
 * MeshStandardMaterial.  Returns the mesh.
 */
export function _makeBox(w, h, d, color, emissive, opacity) {
  return new THREE.Mesh(
    new THREE.BoxGeometry(w, h, d),
    new THREE.MeshStandardMaterial({
      color: color,
      emissive: emissive !== undefined ? emissive : 0x000000,
      opacity: opacity !== undefined ? opacity : 1,
      transparent: opacity !== undefined && opacity < 1
    })
  );
}

/**
 * _makeRect — create a THREE.Mesh with PlaneGeometry.
 */
export function _makeRect(w, h, color) {
  return new THREE.Mesh(
    new THREE.PlaneGeometry(w, h),
    new THREE.MeshStandardMaterial({ color: color, side: THREE.DoubleSide })
  );
}

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
  // 车灯：椭圆透镜造型
  _carGeom.headlight = new T.BoxGeometry(0.08, 0.16, 0.42, 1, 1, 3);
  _carGeom.taillight = new T.BoxGeometry(0.06, 0.16, 0.42, 1, 1, 3);
  _carGeom.mirror = new T.BoxGeometry(0.14, 0.2, 0.1, 1, 1, 2);
  _carGeom.doorHandle = new T.BoxGeometry(0.12, 0.04, 0.05);
  _carGeom.grille = new T.BoxGeometry(0.05, 0.26, 1.15, 1, 1, 4);
  _carGeom.licensePlate = new T.BoxGeometry(0.04, 0.14, 0.42, 1, 1, 3);
  _carGeom.antenna = new T.CylinderGeometry(0.01, 0.01, 0.45, 6);
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
  var spokeMat = new T.MeshStandardMaterial({ color: 0xbbbbbb, metalness: 0.6, roughness: 0.3 });
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
 * @returns {THREE.Group}
 */
export function _buildSedan(color, secondaryColor) {
  initCarMesh(); // ensure shared geometries exist
  const T = window.THREE;
  var g = new T.Group();
  // 真实车漆：MeshPhysicalMaterial + clearcoat（清漆层）+ sheen（绒光），
  // 配合 scene.environment 的 PMREM 环境贴图产生高光反射。
  var bodyMat = new T.MeshPhysicalMaterial({
    color: color, metalness: 0.55, roughness: 0.22, envMapIntensity: 1.1,
    clearcoat: 1.0, clearcoatRoughness: 0.06, sheen: new T.Color(0.4, 0.4, 0.4)
  });
  var cabinMat = new T.MeshPhysicalMaterial({
    color: secondaryColor, metalness: 0.4, roughness: 0.28, envMapIntensity: 0.9,
    clearcoat: 0.6, clearcoatRoughness: 0.12
  });
  var glassMat = new T.MeshPhysicalMaterial({
    color: 0x223344, metalness: 0.9, roughness: 0.04, envMapIntensity: 1.3,
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

  // Wheels (4) — 前轮抽到 frontWheels 子 Group 以支持转向动画
  var wheelPos = [
    [1.35, 0.34, 0.95], [1.35, 0.34, -0.95],
    [-1.35, 0.34, 0.95], [-1.35, 0.34, -0.95]
  ];
  var wheels = [], frontWheels = new T.Group();
  for (var wi = 0; wi < 4; wi++) {
    var wh = _buildWheel(T);
    wh.position.set(wheelPos[wi][0], wheelPos[wi][1], wheelPos[wi][2]);
    wheels.push(wh);
    if (wi < 2) {
      frontWheels.add(wh);
    } else {
      g.add(wh);
    }
  }
  g.add(frontWheels);
  g.userData.frontWheels = frontWheels;
  g.userData.wheels = wheels;

  // Headlights with chrome bezel
  var hlMat = new T.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 1.7, roughness: 0.12 });
  var bezelMat = new T.MeshStandardMaterial({ color: 0xdddddd, metalness: 0.6, roughness: 0.25 });
  function addHeadlight(z) {
    var bg = new T.Group();
    var lens = new T.Mesh(_carGeom.headlight, hlMat);
    var bezel = new T.Mesh(new T.BoxGeometry(0.09, 0.18, 0.46, 1, 1, 3), bezelMat);
    bezel.position.x = -0.02;
    bg.add(bezel); bg.add(lens);
    bg.position.set(2.16, 0.58, z);
    g.add(bg);
    return bg;
  }
  addHeadlight(0.58); addHeadlight(-0.58);

  // Taillights
  var tlMat = new T.MeshStandardMaterial({ color: 0xff2222, emissive: 0xff1111, emissiveIntensity: 1.7, roughness: 0.12 });
  function addTaillight(z) {
    var bg = new T.Group();
    var lens = new T.Mesh(_carGeom.taillight, tlMat);
    var bezel = new T.Mesh(new T.BoxGeometry(0.07, 0.18, 0.46, 1, 1, 3), bezelMat);
    bezel.position.x = 0.02;
    bg.add(bezel); bg.add(lens);
    bg.position.set(-2.18, 0.58, z);
    g.add(bg);
    return bg;
  }
  addTaillight(0.58); addTaillight(-0.58);

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
  g.userData.headlights = [spotL, spotR];

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
    color: color, metalness: 0.5, roughness: 0.24, envMapIntensity: 1.0,
    clearcoat: 0.9, clearcoatRoughness: 0.08, sheen: new T.Color(0.3, 0.3, 0.3)
  });
  var glassMat = new T.MeshPhysicalMaterial({
    color: 0x223344, metalness: 0.85, roughness: 0.06, envMapIntensity: 1.1,
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

  // 车轮（真实尺寸，圆柱不会被非均匀缩放压扁）
  var wheels = [];
  var wheelGeo = new T.CylinderGeometry(wheelR, wheelR, wheelW, 20);
  var hubGeo = new T.CylinderGeometry(wheelR * 0.55, wheelR * 0.55, wheelW + 0.01, 14);
  var spokeGeo = new T.BoxGeometry(wheelR * 1.6, 0.035, 0.025);
  var wheelPositions = [
    [wFrontX, wheelR, wZ], [wFrontX, wheelR, -wZ],
    [wRearX, wheelR, wZ], [wRearX, wheelR, -wZ]
  ];
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
    wg.position.set(wheelPositions[wi][0], wheelPositions[wi][1], wheelPositions[wi][2]);
    wg.userData.isWheel = true;
    wheels.push(wg); g.add(wg);
  }
  g.userData.wheels = wheels;

  // 车灯（简化 emissive，不加 SpotLight 避免 24 NPC × 2 灯 = 48 动态光源性能灾难）
  var headMat = new T.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 1.2, roughness: 0.2 });
  var tailMat = new T.MeshStandardMaterial({ color: 0xff2222, emissive: 0xff1111, emissiveIntensity: 1.2, roughness: 0.2 });
  var hlGeo = new T.BoxGeometry(0.08, 0.16, 0.42, 1, 1, 2);
  var tlGeo = new T.BoxGeometry(0.06, 0.16, 0.42, 1, 1, 2);
  var lightX = BL / 2 - 0.04;
  function makeObsLight(z, isHead) {
    var m = new T.Mesh(isHead ? hlGeo : tlGeo, isHead ? headMat : tailMat);
    m.position.set(isHead ? lightX : -lightX, 0.58, z);
    g.add(m);
  }
  makeObsLight(0.58, true); makeObsLight(-0.58, true);
  makeObsLight(0.58, false); makeObsLight(-0.58, false);

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

/**
 * getColor — look up an obstacle colour by type name.
 */
export function getColor(name) {
  return _obsColors[name];
}

// ── Traffic light model ──────────────────────────────────────────
/**
 * _buildTrafficLight — build a traffic light mesh (pole + arm + 3-lamp housing).
 *
 * Real-world scale (NOT unit-normalised like obstacles — traffic lights are
 * fixed-size infrastructure, not scaled at render time).
 *
 * Coordinate system: X=forward (along road), Y=up, Z=lateral (across road).
 * The pole is placed at the roadside; the arm reaches over the road.
 *
 * Lamp meshes are stored in userData.lamps = [red, yellow, green] so the
 * renderer can toggle emissive intensity based on the current phase.
 * Inactive lamps are dim (emissive 0.05), active lamp is bright (emissive 1.0).
 *
 * @returns {THREE.Group} with userData.lamps[3] = {mesh, color}
 */
export function _buildTrafficLight() {
  const T = window.THREE;
  var g = new T.Group();

  // Pole: 5m tall cylinder at roadside
  var pole = new T.Mesh(
    new T.CylinderGeometry(0.12, 0.15, 5.0, 12),
    new T.MeshStandardMaterial({ color: 0x555555, metalness: 0.6, roughness: 0.4 })
  );
  pole.position.y = 2.5;
  pole.castShadow = true;
  g.add(pole);

  // Arm: horizontal box extending over road (length 4m, at height 4.8m)
  var arm = new T.Mesh(
    new T.BoxGeometry(0.12, 0.12, 4.0),
    new T.MeshStandardMaterial({ color: 0x555555, metalness: 0.6, roughness: 0.4 })
  );
  arm.position.set(0, 4.8, 2.0);
  g.add(arm);

  // Lamp housing: box at end of arm (0.35 wide × 0.9 tall × 0.3 deep)
  var housing = new T.Mesh(
    new T.BoxGeometry(0.35, 0.9, 0.3),
    new T.MeshStandardMaterial({ color: 0x222222, metalness: 0.3, roughness: 0.7 })
  );
  housing.position.set(0, 4.3, 4.0);
  housing.castShadow = true;
  g.add(housing);

  // Three lamps: red (top), yellow (mid), green (bottom)
  // Each is a small sphere with emissive material; renderer toggles intensity.
  var lampColors = [0xff0000, 0xffaa00, 0x00ff00];
  var lampY = [4.6, 4.3, 4.0];
  var lamps = [];
  for (var li = 0; li < 3; li++) {
    var lamp = new T.Mesh(
      new T.SphereGeometry(0.11, 16, 12),
      new T.MeshStandardMaterial({
        color: lampColors[li],
        emissive: lampColors[li],
        emissiveIntensity: 0.05  // dim by default
      })
    );
    lamp.position.set(0, lampY[li], 4.16);
    g.add(lamp);
    lamps.push(lamp);
  }

  g.userData.lamps = lamps;
  g.userData.lampColors = lampColors;
  return g;
}

// ── ETC gate model (Phase 3) ────────────────────────────────────
/**
 * _buildETCGate — 多车道 ETC 收费广场（4 个并列收费口 + 入口大标志门架）。
 *
 * 真实高速收费广场不是单个 9m 门架横跨整条路，而是把道路拓宽成 4-5 个
 * 收费岛并列、每车道独立 ETC 栏杆，入口处还有大型标志门架。本函数按
 * 1:1 真实尺度建模，替代旧的单门架。
 *
 * 坐标系：模型默认朝向与旧版一致——
 *   X = 前向（沿道路，ego 行驶方向）
 *   Y = 上
 *   Z = 横向（横跨道路，4 个收费口沿 Z 等距排列）
 * scene3d 仍用 `gm.rotation.y = -(h + π/2)` 把模型转到与道路切线对齐。
 *
 * 4 个栏杆统一收集到 userData.booms 数组，scene3d 同步驱动抬起角度。
 * 兼容字段 userData.boom 保留第一个栏杆，未升级的调用点仍可用。
 *
 * @returns {THREE.Group} with userData.booms = [boom×4], userData.boom = booms[0]
 */
export function _buildETCGate() {
  const T = window.THREE;
  var g = new T.Group();

  // ── 广场参数：4 车道并列收费口 ──
  var N_BOOTHS = 4;
  var LANE_W = 3.5;                       // 单车道宽（与 toll 段 3.2m 略宽，符合广场拓宽惯例）
  var totalWidth = N_BOOTHS * LANE_W;     // 14m
  var halfW = totalWidth / 2;

  // ── 共享材质 ──
  var plazaMat    = new T.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.92, metalness: 0.0 });
  var islandMat   = new T.MeshStandardMaterial({ color: 0xffcc00, roughness: 0.55, emissive: 0x332200, emissiveIntensity: 0.18 });
  var boothMat    = new T.MeshStandardMaterial({ color: 0xe8e8ec, roughness: 0.55, metalness: 0.1 });
  var boothRoofMat= new T.MeshStandardMaterial({ color: 0x2255aa, roughness: 0.35, metalness: 0.3, emissive: 0x113355, emissiveIntensity: 0.18 });
  var glassMat    = new T.MeshStandardMaterial({ color: 0x0a1a2a, metalness: 0.7, roughness: 0.15, emissive: 0x081020, emissiveIntensity: 0.12 });
  var poleMat     = new T.MeshStandardMaterial({ color: 0x445566, metalness: 0.6, roughness: 0.35 });
  var signMat     = new T.MeshStandardMaterial({ color: 0x2266aa, emissive: 0x1166aa, emissiveIntensity: 0.55, roughness: 0.3 });
  var boomMat     = new T.MeshStandardMaterial({ color: 0xffcc00, emissive: 0x442200, emissiveIntensity: 0.35, roughness: 0.4 });
  // 收费亭 ETC 显示屏（绿色发光小条）
  var screenMat   = new T.MeshStandardMaterial({ color: 0x00ff66, emissive: 0x00ff66, emissiveIntensity: 0.9, roughness: 0.3 });

  // ── 广场地面（深沥青底，标识收费区域与正常路面区分）──
  var plaza = new T.Mesh(
    new T.PlaneGeometry(10.0, totalWidth + 2.5),
    plazaMat
  );
  plaza.rotation.x = -Math.PI / 2;
  plaza.position.y = 0.02;
  plaza.receiveShadow = true;
  g.add(plaza);

  // ── 入口大型标志门架（广场前 3.5m，跨越全部车道）──
  // 真实收费广场前方总有一块大牌「ETC 收费」预告，强化"广场感"
  var megaPoleGeo = new T.CylinderGeometry(0.16, 0.20, 7.0, 12);
  var megaPole1 = new T.Mesh(megaPoleGeo, poleMat);
  megaPole1.position.set(-3.5, 3.5, -halfW - 0.7);
  megaPole1.castShadow = true;
  g.add(megaPole1);
  var megaPole2 = new T.Mesh(megaPoleGeo, poleMat);
  megaPole2.position.set(-3.5, 3.5, halfW + 0.7);
  megaPole2.castShadow = true;
  g.add(megaPole2);
  var megaBeam = new T.Mesh(
    new T.BoxGeometry(0.22, 0.22, totalWidth + 1.6),
    poleMat
  );
  megaBeam.position.set(-3.5, 6.85, 0);
  megaBeam.castShadow = true;
  g.add(megaBeam);
  // 大型 ETC 蓝底标志板（横跨广场顶部）
  var megaSign = new T.Mesh(
    new T.BoxGeometry(0.10, 0.85, totalWidth * 0.7),
    signMat
  );
  megaSign.position.set(-3.38, 6.85, 0);
  g.add(megaSign);
  // 标志板下方绿色「ETC」发光字条（简化为长条 emissive 绿）
  var etcText = new T.Mesh(
    new T.BoxGeometry(0.04, 0.18, totalWidth * 0.45),
    screenMat
  );
  etcText.position.set(-3.30, 6.40, 0);
  g.add(etcText);

  // ── 共享几何体模板（每车道实例化复用）──
  var poleGeo   = new T.CylinderGeometry(0.08, 0.10, 5.0, 10);
  var beamGeo   = new T.BoxGeometry(0.12, 0.12, LANE_W - 0.15);
  var signGeo   = new T.BoxGeometry(0.06, 0.32, (LANE_W - 0.15) * 0.75);
  var islandGeo = new T.BoxGeometry(2.0, 0.20, 0.45);
  var boothGeo  = new T.BoxGeometry(1.7, 2.3, 1.05);
  var roofGeo   = new T.BoxGeometry(1.85, 0.12, 1.2);
  var winGeo    = new T.BoxGeometry(0.04, 0.7, 0.85);
  var screenGeo = new T.BoxGeometry(0.04, 0.18, 0.45);

  var booms = [];  // 4 个栏杆，scene3d 同步驱动抬起

  // ── 4 个并列收费口 ──
  for (var i = 0; i < N_BOOTHS; i++) {
    // 第 i 个车道中心 Z（广场中心对称）
    var zCenter = (i - (N_BOOTHS - 1) / 2) * LANE_W;  // -5.25, -1.75, 1.75, 5.25
    var poleZ1 = zCenter - LANE_W / 2 + 0.15;  // 左立柱
    var poleZ2 = zCenter + LANE_W / 2 - 0.15;  // 右立柱

    // (a) 收费岛（车道右侧黄黑警示岛，长 2m × 高 0.2m × 宽 0.45m）
    var islandZ = zCenter + LANE_W / 2 - 0.25;
    var island = new T.Mesh(islandGeo, islandMat);
    island.position.set(0, 0.10, islandZ);
    island.castShadow = true;
    island.receiveShadow = true;
    g.add(island);

    // (b) 收费亭（岛上方小屋 1.7 × 2.3 × 1.05m）
    var booth = new T.Mesh(boothGeo, boothMat);
    booth.position.set(0, 1.35, islandZ);
    booth.castShadow = true;
    g.add(booth);
    // 屋顶（蓝色扁壳）
    var roof = new T.Mesh(roofGeo, boothRoofMat);
    roof.position.set(0, 2.56, islandZ);
    g.add(roof);
    // 窗户（深色玻璃，朝向前方车辆）
    var win = new T.Mesh(winGeo, glassMat);
    win.position.set(-0.86, 1.5, islandZ);
    g.add(win);
    // ETC 显示屏（绿色发光小条，提示"余额"）
    var screen = new T.Mesh(screenGeo, screenMat);
    screen.position.set(-0.88, 1.05, islandZ);
    g.add(screen);

    // (c) ETC 门架（2 立柱 + 横梁 + 标志板 + 栏杆）
    var pole1 = new T.Mesh(poleGeo, poleMat);
    pole1.position.set(0, 2.5, poleZ1);
    pole1.castShadow = true;
    g.add(pole1);
    var pole2 = new T.Mesh(poleGeo, poleMat);
    pole2.position.set(0, 2.5, poleZ2);
    pole2.castShadow = true;
    g.add(pole2);

    // 横梁连接两立柱顶部（高度 4.9m，跨该车道）
    var beam = new T.Mesh(beamGeo, poleMat);
    beam.position.set(0, 4.85, zCenter);
    beam.castShadow = true;
    g.add(beam);

    // ETC 蓝色标志板（横梁下方）
    var laneSign = new T.Mesh(signGeo, signMat);
    laneSign.position.set(0.09, 4.85, zCenter);
    g.add(laneSign);

    // 栏杆 boom（黄黑警示色，pivot 在 pole1 端，绕 Y 旋转抬起）
    // 几何体沿 +Z 伸出 boomLen，translate 把 pivot 移到原点
    var boomLen = LANE_W - 0.35;
    var boomGeo = new T.BoxGeometry(0.07, 0.07, boomLen);
    boomGeo.translate(0, 0, boomLen / 2);
    var boom = new T.Mesh(boomGeo, boomMat);
    boom.position.set(0, 4.45, poleZ1);
    boom.castShadow = true;
    g.add(boom);
    booms.push(boom);

    // 栏杆末端红色警示灯（小 Sphere emissive 红）
    var tipLight = new T.Mesh(
      new T.SphereGeometry(0.06, 8, 6),
      new T.MeshStandardMaterial({ color: 0xff2222, emissive: 0xff2222, emissiveIntensity: 0.9 })
    );
    tipLight.position.set(0, 4.45, poleZ1 + boomLen);
    g.add(tipLight);
  }

  // ── 兼容字段：userData.boom 保留第一个栏杆（旧 scene3d 调用点兼容）──
  if (booms.length) {
    g.userData.boom = booms[0];
    g.userData.booms = booms;  // 新字段：全部 4 个栏杆，scene3d 同步驱动
  }
  // 真实尺寸 1:1 建模（不会被 scene3d 非均匀缩放）
  g.userData.realScale = true;

  return g;
}

// ── Stub: export menu ───────────────────────────────────────────
export function toggleExportMenu() {
  var m = document.getElementById('export-menu');
  if (m) m.classList.toggle('show');
}

export function exportPNG() {
  // stub — will import topology SVG export logic from main
  console.log('exportPNG stub');
}

export function exportCSV() {
  // stub — will import QoS CSV export logic from main
  console.log('exportCSV stub');
}

// ── Stub: connection ────────────────────────────────────────────
export function doConnect() {
  // stub — will import connection logic from main
  console.log('doConnect stub');
}

export function doSimulate() {
  // stub — will import demo simulation logic from main
  console.log('doSimulate stub');
}

// ═════════════════════════════════════════════════════════════════
// Phase 4.9: removed all `window.X = X` assignments here.
// app.js re-publishes the names used by inline-onclick handlers under
// a single `window.flowboard` namespace object. Internal modules
// communicate via ES module imports only.
// ═════════════════════════════════════════════════════════════════
