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
  if (window.saveState) window.saveState();
  // Let the 3D renderer know it may need to resize after expanding
  setTimeout(function () {
    if (!hdr.parentElement.classList.contains('collapsed')) {
      if (window.resize3D) window.resize3D();
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

  // Windshield
  var ws = new T.Mesh(
    _carGeom.windshield,
    new T.MeshStandardMaterial({ color: 0x334455, metalness: 0.6, roughness: 0.1 })
  );
  ws.position.set(1.45, 1.3, 0);
  ws.rotation.z = -0.45;
  g.add(ws);

  // Rear window
  var rw = new T.Mesh(
    _carGeom.rearWindow,
    new T.MeshStandardMaterial({ color: 0x334455, metalness: 0.6, roughness: 0.1 })
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

  // Wheels (4)
  var wheelPos = [
    [1.3, 0.35, 1.0], [1.3, 0.35, -1.0],
    [-1.3, 0.35, 1.0], [-1.3, 0.35, -1.0]
  ];
  for (var wi = 0; wi < 4; wi++) {
    var wh = new T.Mesh(
      _carGeom.wheel,
      new T.MeshStandardMaterial({ color: 0x111111, metalness: 0.3, roughness: 0.6 })
    );
    wh.rotation.z = Math.PI / 2;
    wh.position.set(wheelPos[wi][0], wheelPos[wi][1], wheelPos[wi][2]);
    wh.castShadow = true;
    g.add(wh);
  }

  // Headlights
  var hl = new T.Mesh(
    _carGeom.headlight,
    new T.MeshStandardMaterial({ color: 0xffffcc, roughness: 0.2 })
  );
  hl.position.set(2.25, 0.65, 0.5);
  g.add(hl);
  var hl2 = hl.clone();
  hl2.position.z = -0.5;
  g.add(hl2);

  // Taillights
  var tl = new T.Mesh(
    _carGeom.taillight,
    new T.MeshStandardMaterial({ color: 0xff2222, roughness: 0.2 })
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
 * _buildObstacle — build a simple unit-normalised obstacle mesh
 * (bounding box ≈ 1×1×1; scale with .set(L, H, W) at render time).
 */
export function _buildObstacle(color) {
  const T = window.THREE;
  var g = new T.Group();

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
  cyclist: 0x33ddff
};

/**
 * getColor — look up an obstacle colour by type name.
 */
export function getColor(name) {
  return _obsColors[name];
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
// Global references — so inline HTML onclick handlers and the
// monolithic <script> block can still resolve these names.
// ═════════════════════════════════════════════════════════════════
window.safeCall = safeCall;
window.reportDiag = reportDiag;
window.clearDiag = clearDiag;
window.toggleCard = toggleCard;
window._makeBox = _makeBox;
window._makeRect = _makeRect;
window._buildSedan = _buildSedan;
window._buildObstacle = _buildObstacle;
window.getColor = getColor;
window.initCarMesh = initCarMesh;
window.toggleExportMenu = toggleExportMenu;
window.exportPNG = exportPNG;
window.exportCSV = exportCSV;
window.doConnect = doConnect;
window.doSimulate = doSimulate;
