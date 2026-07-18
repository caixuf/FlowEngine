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
  // Phase 6 画质再升级：车身不再是简单方块，而是带腰线的“胶囊”长方体
  // 更多分段给光影更多变化，模拟引擎盖/腰线的曲面过渡。
  _carGeom.body = new T.BoxGeometry(4.2, 0.72, 1.86, 6, 1, 4);
  _carGeom.hood = new T.BoxGeometry(1.25, 0.08, 1.52, 3, 1, 3);
  _carGeom.trunkDeck = new T.BoxGeometry(1.05, 0.06, 1.52, 3, 1, 3);
  _carGeom.cabin = new T.BoxGeometry(2.15, 0.52, 1.52, 3, 1, 3);
  _carGeom.windshield = new T.BoxGeometry(0.06, 0.46, 1.48);
  _carGeom.rearWindow = new T.BoxGeometry(0.06, 0.36, 1.38);
  _carGeom.sideWindow = new T.BoxGeometry(1.55, 0.36, 1.56);
  // 轮拱：黑色弧形遮挡，让车轮不像贴上去的
  _carGeom.wheelArch = new T.BoxGeometry(0.55, 0.38, 0.28, 2, 1, 1);
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
  var bodyMat = new T.MeshStandardMaterial({ color: color, metalness: 0.55, roughness: 0.24, envMapIntensity: 1.0 });
  var cabinMat = new T.MeshStandardMaterial({ color: secondaryColor, metalness: 0.4, roughness: 0.3, envMapIntensity: 0.8 });
  var glassMat = new T.MeshStandardMaterial({ color: 0x223344, metalness: 0.9, roughness: 0.04, envMapIntensity: 1.3 });
  var blackMat = new T.MeshStandardMaterial({ color: 0x111111, roughness: 0.7 });
  var chromeMat = new T.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.75, roughness: 0.18 });
  var archMat = new T.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.85 });

  // Lower body — 主车身带腰线曲面感，离地间隙约 0.16m
  var body = new T.Mesh(_carGeom.body, bodyMat);
  body.position.y = 0.52; body.castShadow = true; body.receiveShadow = true; g.add(body);

  // Hood / trunk deck 让侧面有层次
  var hood = new T.Mesh(_carGeom.hood, bodyMat);
  hood.position.set(1.48, 0.9, 0); hood.castShadow = true; g.add(hood);
  var deck = new T.Mesh(_carGeom.trunkDeck, bodyMat);
  deck.position.set(-1.58, 0.87, 0); deck.castShadow = true; g.add(deck);

  // Cabin / roof
  var cabin = new T.Mesh(_carGeom.cabin, cabinMat);
  cabin.position.set(0.05, 1.15, 0); cabin.castShadow = true; g.add(cabin);

  // Windshield
  var ws = new T.Mesh(_carGeom.windshield, glassMat);
  ws.position.set(1.1, 1.08, 0); ws.rotation.z = -0.45; g.add(ws);
  // Rear window
  var rw = new T.Mesh(_carGeom.rearWindow, glassMat);
  rw.position.set(-1.05, 1.02, 0); rw.rotation.z = 0.42; g.add(rw);
  // Side windows
  var sideWin = new T.Mesh(_carGeom.sideWindow, glassMat);
  sideWin.position.set(0.02, 1.13, 0); g.add(sideWin);

  // Wheel arches — 轮拱黑色内衬，避免车轮像贴图
  var archPos = [[1.35, 0.95], [-1.35, 0.95]];
  for (var ai = 0; ai < 2; ai++) {
    var aL = new T.Mesh(_carGeom.wheelArch, archMat);
    aL.position.set(archPos[ai][0], 0.34, archPos[ai][1]);
    g.add(aL);
    var aR = aL.clone(); aR.position.z = -archPos[ai][1]; g.add(aR);
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

  // 车头灯真实照射
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

  return g;
}

// ── Obstacle model ───────────────────────────────────────────────
/**
 * _buildObstacle — build a unit-normalised obstacle mesh shaped by type
 * (bounding box ≈ 1×1×1; scale with .set(L, H, W) at render time).
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

  // 默认：轿车形（car/truck/cyclist），unit-normalized，scale.set(L,H,W) 映射到真实尺寸
  var bodyMat = new T.MeshStandardMaterial({ color: color, metalness: 0.45, roughness: 0.28, envMapIntensity: 0.8 });
  var glassMat = new T.MeshStandardMaterial({ color: 0x223344, metalness: 0.85, roughness: 0.06, envMapIntensity: 1.0 });
  var chromeMat = new T.MeshStandardMaterial({ color: 0xbbbbbb, metalness: 0.7, roughness: 0.22 });
  var blackMat = new T.MeshStandardMaterial({ color: 0x151515, roughness: 0.8 });

  // 车身主体（更多分段，光影更柔和）
  var body = new T.Mesh(new T.BoxGeometry(1, 0.55, 0.96, 4, 1, 3), bodyMat);
  body.position.y = 0.31; body.castShadow = true; body.receiveShadow = true; g.add(body);

  // 引擎盖/后备箱盖
  var hood = new T.Mesh(new T.BoxGeometry(0.32, 0.07, 0.84, 2, 1, 2), bodyMat);
  hood.position.set(0.36, 0.57, 0); g.add(hood);
  var deck = new T.Mesh(new T.BoxGeometry(0.26, 0.05, 0.84, 2, 1, 2), bodyMat);
  deck.position.set(-0.36, 0.56, 0); g.add(deck);

  // 驾驶舱
  var cabin = new T.Mesh(new T.BoxGeometry(0.52, 0.34, 0.82, 2, 1, 2), bodyMat);
  cabin.position.set(0.02, 0.74, 0); cabin.castShadow = true; g.add(cabin);

  // 车窗
  var sideWin = new T.Mesh(new T.BoxGeometry(0.48, 0.25, 0.86, 1, 1, 1), glassMat);
  sideWin.position.set(0.02, 0.75, 0); g.add(sideWin);
  var fWin = new T.Mesh(new T.BoxGeometry(0.05, 0.27, 0.74), glassMat);
  fWin.position.set(0.28, 0.74, 0); fWin.rotation.z = -0.38; g.add(fWin);
  var rWin = new T.Mesh(new T.BoxGeometry(0.04, 0.23, 0.68), glassMat);
  rWin.position.set(-0.24, 0.72, 0); rWin.rotation.z = 0.32; g.add(rWin);

  // 轮拱内衬
  var archGeo = new T.BoxGeometry(0.16, 0.24, 0.12, 1, 1, 1);
  var archPos = [[0.34, 0.96], [-0.34, 0.96]];
  for (var ai2 = 0; ai2 < 2; ai2++) {
    var aL = new T.Mesh(archGeo, blackMat); aL.position.set(archPos[ai2][0], 0.32, archPos[ai2][1]); g.add(aL);
    var aR = aL.clone(); aR.position.z = -archPos[ai2][1]; g.add(aR);
  }

  // 侧裙
  var skirt = new T.Mesh(new T.BoxGeometry(0.65, 0.04, 0.99, 3, 1, 1), bodyMat);
  skirt.position.set(0, 0.11, 0); g.add(skirt);

  // 前/后保险杠
  var fBumper = new T.Mesh(new T.BoxGeometry(0.06, 0.16, 0.98, 1, 1, 3), chromeMat);
  fBumper.position.set(0.52, 0.26, 0); g.add(fBumper);
  var rBumper = new T.Mesh(new T.BoxGeometry(0.05, 0.16, 0.98, 1, 1, 3), chromeMat);
  rBumper.position.set(-0.52, 0.26, 0); g.add(rBumper);
  // 进气格栅
  var grille = new T.Mesh(new T.BoxGeometry(0.03, 0.18, 0.55, 1, 1, 2), blackMat);
  grille.position.set(0.51, 0.35, 0); g.add(grille);

  // 车轮：带胎纹 + 5 辐条 + 中心盖
  var wheels = [];
  var wheelGeo = new T.CylinderGeometry(0.17, 0.17, 0.09, 18);
  var treadGeo = new T.TorusGeometry(0.17, 0.01, 4, 18);
  var tireMat = new T.MeshStandardMaterial({ color: 0x121212, metalness: 0.05, roughness: 0.85 });
  var hubGeo = new T.CylinderGeometry(0.095, 0.095, 0.1, 12);
  var hubMat = new T.MeshStandardMaterial({ color: 0x999999, metalness: 0.55, roughness: 0.35 });
  var spokeMat = new T.MeshStandardMaterial({ color: 0xaaaaaa, metalness: 0.5, roughness: 0.4 });
  var wheelPos = [[0.34, 0.18, 0.41], [0.34, 0.18, -0.41], [-0.34, 0.18, 0.41], [-0.34, 0.18, -0.41]];
  for (var wi = 0; wi < 4; wi++) {
    var wg = new T.Group();
    var tire = new T.Mesh(wheelGeo, tireMat); tire.rotation.z = Math.PI / 2; tire.castShadow = true; wg.add(tire);
    var t1 = new T.Mesh(treadGeo, tireMat); t1.rotation.x = Math.PI / 2; t1.position.x = 0.025; wg.add(t1);
    var t2 = t1.clone(); t2.position.x = -0.025; wg.add(t2);
    var hub = new T.Mesh(hubGeo, hubMat); hub.rotation.z = Math.PI / 2; wg.add(hub);
    for (var si = 0; si < 5; si++) {
      var spoke = new T.Mesh(new T.BoxGeometry(0.14, 0.025, 0.02), spokeMat);
      spoke.rotation.z = Math.PI / 2; spoke.rotation.x = (Math.PI * 2 / 5) * si;
      wg.add(spoke);
    }
    var cap = new T.Mesh(new T.CylinderGeometry(0.03, 0.03, 0.11, 8), hubMat);
    cap.rotation.z = Math.PI / 2; wg.add(cap);
    wg.position.set(wheelPos[wi][0], wheelPos[wi][1], wheelPos[wi][2]);
    wg.userData.isWheel = true;
    wheels.push(wg); g.add(wg);
  }
  g.userData.wheels = wheels;

  // 前灯/尾灯（带边框）
  var headMat = new T.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 1.5, roughness: 0.12 });
  var tailMat = new T.MeshStandardMaterial({ color: 0xff2222, emissive: 0xff1111, emissiveIntensity: 1.6, roughness: 0.12 });
  var bezelMat2 = new T.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.5, roughness: 0.3 });
  var hlGeo = new T.BoxGeometry(0.04, 0.1, 0.18, 1, 1, 2);
  var tlGeo = new T.BoxGeometry(0.035, 0.1, 0.18, 1, 1, 2);
  function makeObsLight(z, isHead) {
    var bg = new T.Group();
    var lens = new T.Mesh(isHead ? hlGeo : tlGeo, isHead ? headMat : tailMat);
    var bezel = new T.Mesh(new T.BoxGeometry(0.05, 0.11, 0.2, 1, 1, 2), bezelMat2);
    bezel.position.x = isHead ? -0.01 : 0.01;
    bg.add(bezel); bg.add(lens);
    bg.position.set(isHead ? 0.51 : -0.51, 0.38, z);
    g.add(bg);
  }
  makeObsLight(0.3, true); makeObsLight(-0.3, true);
  makeObsLight(0.3, false); makeObsLight(-0.3, false);

  return g;
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
 * _buildETCGate — build an ETC toll gate mesh (2 poles + crossbar + boom arm).
 *
 * Real-world scale. The boom arm is stored in userData.boom so the renderer
 * can rotate it based on the gate's progress (0=closed horizontal, 1=open ~75°).
 *
 * Coordinate system: X=forward (along road), Y=up, Z=lateral (across road).
 * The gate spans the road laterally: poles at Z=±roadHalf, boom rotates
 * from one pole.
 *
 * @returns {THREE.Group} with userData.boom = boom arm mesh
 */
export function _buildETCGate() {
  const T = window.THREE;
  var g = new T.Group();

  // Two poles at Z = ±4m (covering a 2-lane road of 7m width + margin)
  var poleGeo = new T.CylinderGeometry(0.15, 0.18, 5.5, 12);
  var poleMat = new T.MeshStandardMaterial({ color: 0x445566, metalness: 0.5, roughness: 0.4 });
  for (var pi = 0; pi < 2; pi++) {
    var pole = new T.Mesh(poleGeo, poleMat);
    pole.position.set(0, 2.75, pi === 0 ? -4.5 : 4.5);
    pole.castShadow = true;
    g.add(pole);
  }

  // Crossbar: horizontal box connecting pole tops at height 5m
  var bar = new T.Mesh(
    new T.BoxGeometry(0.15, 0.15, 9.0),
    new T.MeshStandardMaterial({ color: 0x445566, metalness: 0.5, roughness: 0.4 })
  );
  bar.position.set(0, 5.2, 0);
  bar.castShadow = true;
  g.add(bar);

  // Sign panel on crossbar (rectangular plate)
  var sign = new T.Mesh(
    new T.BoxGeometry(0.08, 0.6, 3.0),
    new T.MeshStandardMaterial({ color: 0x2266aa, emissive: 0x113355, roughness: 0.3 })
  );
  sign.position.set(0.1, 5.2, 0);
  g.add(sign);

  // Boom arm: long thin box, pivot at one pole (Z=-4.5)
  // Rotates around Y axis: 0° = horizontal (closed), 75° = raised (open)
  var boom = new T.Mesh(
    new T.BoxGeometry(0.1, 0.1, 8.5),
    new T.MeshStandardMaterial({ color: 0xffcc00, emissive: 0x332200, roughness: 0.3 })
  );
  // Position so one end is at the pivot pole, extends across road to other pole
  boom.position.set(0, 4.8, 0);
  // Move geometry so pivot is at one end (Z=-4.5), arm extends to Z=+4
  boom.geometry.translate(0, 0, 4.25);
  boom.position.z = -4.5;
  g.add(boom);
  g.userData.boom = boom;

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
