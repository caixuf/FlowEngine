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
// allocate BoxGeometry / CylinderGeometry once.
const _carGeom = {};

/**
 * initCarMesh — create shared geometry objects used by _buildSedan.
 * Safe to call multiple times; only allocates on the first call.
 */
export function initCarMesh() {
  if (_carGeom.body) return; // already initialized
  const T = window.THREE;
  _carGeom.body = new T.BoxGeometry(4.2, 0.85, 1.85);
  _carGeom.cabin = new T.BoxGeometry(2.2, 0.55, 1.7);
  _carGeom.windshield = new T.BoxGeometry(0.15, 0.45, 1.6);
  _carGeom.rearWindow = new T.BoxGeometry(0.12, 0.35, 1.5);
  _carGeom.frontBumper = new T.BoxGeometry(0.3, 0.35, 1.75);
  _carGeom.rearBumper = new T.BoxGeometry(0.25, 0.35, 1.75);
  _carGeom.wheel = new T.CylinderGeometry(0.32, 0.32, 0.28, 16);
  _carGeom.headlight = new T.BoxGeometry(0.2, 0.15, 0.3);
  _carGeom.taillight = new T.BoxGeometry(0.15, 0.15, 0.3);
}

/**
 * _buildSedan — build a simple car mesh group from shared geometry.
 * @param {number} color        body colour (e.g. 0x4488dd)
 * @param {number} secondaryColor  roof / cabin colour (e.g. 0x3377bb)
 * @returns {THREE.Group}
 */
export function _buildSedan(color, secondaryColor) {
  initCarMesh(); // ensure shared geometries exist
  const T = window.THREE;
  var g = new T.Group();

  // Lower body
  var body = new T.Mesh(
    _carGeom.body,
    new T.MeshStandardMaterial({ color: color, metalness: 0.4, roughness: 0.25 })
  );
  body.position.y = 0.85;
  body.castShadow = true;
  g.add(body);

  // Cabin / roof
  var cabin = new T.Mesh(
    _carGeom.cabin,
    new T.MeshStandardMaterial({ color: secondaryColor, metalness: 0.3, roughness: 0.35 })
  );
  cabin.position.set(0.5, 1.45, 0);
  cabin.castShadow = true;
  g.add(cabin);

  // Windshield — 深色高反射玻璃，配合 PMREM 环境贴图反射天空/地面
  var ws = new T.Mesh(
    _carGeom.windshield,
    new T.MeshStandardMaterial({ color: 0x223344, metalness: 0.9, roughness: 0.05 })
  );
  ws.position.set(1.45, 1.3, 0);
  ws.rotation.z = -0.45;
  g.add(ws);

  // Rear window
  var rw = new T.Mesh(
    _carGeom.rearWindow,
    new T.MeshStandardMaterial({ color: 0x223344, metalness: 0.9, roughness: 0.05 })
  );
  rw.position.set(-0.55, 1.25, 0);
  rw.rotation.z = 0.45;
  g.add(rw);

  // Front bumper
  var fb = new T.Mesh(
    _carGeom.frontBumper,
    new T.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.3, roughness: 0.4 })
  );
  fb.position.set(2.25, 0.55, 0);
  g.add(fb);

  // Rear bumper
  var rb = new T.Mesh(
    _carGeom.rearBumper,
    new T.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.3, roughness: 0.4 })
  );
  rb.position.set(-2.25, 0.55, 0);
  g.add(rb);

  // Wheels (4) — 前轮抽到 frontWheels 子 Group 以支持转向动画
  var wheelPos = [
    [1.3, 0.35, 1.0], [1.3, 0.35, -1.0],
    [-1.3, 0.35, 1.0], [-1.3, 0.35, -1.0]
  ];
  var frontWheels = new T.Group();
  for (var wi = 0; wi < 4; wi++) {
    var wh = new T.Mesh(
      _carGeom.wheel,
      new T.MeshStandardMaterial({ color: 0x111111, metalness: 0.3, roughness: 0.6 })
    );
    wh.rotation.z = Math.PI / 2;
    wh.position.set(wheelPos[wi][0], wheelPos[wi][1], wheelPos[wi][2]);
    wh.castShadow = true;
    // 前轮（wi=0,1）加入 frontWheels 子 Group，后轮直接挂车体
    if (wi < 2) {
      frontWheels.add(wh);
    } else {
      g.add(wh);
    }
  }
  g.add(frontWheels);
  g.userData.frontWheels = frontWheels;

  // Headlights — emissive 触发 Bloom 光晕（threshold=0.85，亮区才发光）
  var hl = new T.Mesh(
    _carGeom.headlight,
    new T.MeshStandardMaterial({ color: 0xffffcc, emissive: 0xffffee, emissiveIntensity: 1.5, roughness: 0.2 })
  );
  hl.position.set(2.25, 0.65, 0.5);
  g.add(hl);
  var hl2 = hl.clone();
  hl2.position.z = -0.5;
  g.add(hl2);

  // Taillights — 红色 emissive，Bloom 会产生红色光晕
  var tl = new T.Mesh(
    _carGeom.taillight,
    new T.MeshStandardMaterial({ color: 0xff2222, emissive: 0xff1111, emissiveIntensity: 1.2, roughness: 0.2 })
  );
  tl.position.set(-2.25, 0.65, 0.5);
  g.add(tl);
  var tl2 = tl.clone();
  tl2.position.z = -0.5;
  g.add(tl2);

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

  // 默认：轿车形（car/truck/cyclist）
  var body = new T.Mesh(
    new T.BoxGeometry(1, 0.6, 1),
    new T.MeshStandardMaterial({ color: color, metalness: 0.35, roughness: 0.3 })
  );
  body.position.y = 0.3;
  body.castShadow = true;
  g.add(body);

  var cabin = new T.Mesh(
    new T.BoxGeometry(0.55, 0.4, 0.88),
    new T.MeshStandardMaterial({ color: 0x1a2233, metalness: 0.3, roughness: 0.4 })
  );
  cabin.position.set(0.05, 0.8, 0);
  g.add(cabin);

  var ws = new T.Mesh(
    new T.BoxGeometry(0.08, 0.28, 0.82),
    new T.MeshStandardMaterial({ color: 0x334455, metalness: 0.5, roughness: 0.15 })
  );
  ws.position.set(0.32, 0.78, 0);
  g.add(ws);

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
