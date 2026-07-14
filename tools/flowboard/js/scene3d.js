/* global THREE, window, topoData */

// ══════════════════════════════════════════════════════════════════════════════
// scene3d.js — Three.js 3D scene module for ADAS visualization
// ══════════════════════════════════════════════════════════════════════════════

import { safeCall, reportDiag, _makeBox, _makeRect, _buildSedan, _buildObstacle, getColor } from './utils.js';
import { initDeadReckon, updateDeadReckon, getDeadReckonState, _dr, _2d } from './deadreckon.js';

const THREE = window.THREE;

// ══════════════════════════════════════════════════════════════════════════════
// Module-level state (replaces former window globals)
// ══════════════════════════════════════════════════════════════════════════════

/** Main scene, camera, renderer */
let scene3d = null, camera3d = null, renderer3d = null;
let sceneReady = false;

/** Camera chase-cam state vectors */
let _cam = null, _camLook = null, _camTarget = null, _camLookTarget = null;

/** Scene object references — road, ground, environment, car */
let _lidarCloud = null;
let _lidarWorld = [];
let _obsPool = [], _obsWorld = [];
let _roadGroup = null, _groundMesh = null, _envGroup = null, _carGroup = null;

/** Obstacle type → colour lookup (defined once, shared) */
const _obsColors = { car: 0xff9944, truck: 0xff4422, pedestrian: 0x33ff88, cyclist: 0x33ddff };

/** Road curve state */
let _curveActive = false;
let _lastCurveKey = "";

/** WebGL context-loss flag — render loop skips while true */
let _glLost = false;

/** Pre-allocated vector/scale objects (avoids per-frame GC pressure) */
let _tmpV3 = null, _tmpScale = null;

/** Animation time counter (incremented per frame) */
let _animT = 0;

/** Obstacle height lookup (defined once, shared across all _renderFrame calls) */
const _OBS_H = { truck: 2.8, pedestrian: 1.8, cyclist: 1.7 };

// ══════════════════════════════════════════════════════════════════════════════
// 3D ADAS scene builder — road, cars, environment
// Coordinate system: X=forward, Y=up, Z=right
// ══════════════════════════════════════════════════════════════════════════════

// ── Cylinder helper (used only in _buildEnvironment) ──
function _makeCyl(rTop, rBot, h, segs, color) {
  return new THREE.Mesh(
    new THREE.CylinderGeometry(rTop, rBot, h, segs || 16),
    new THREE.MeshStandardMaterial({ color: color, metalness: 0.3, roughness: 0.6 })
  );
}

// ── Road builder (4 lanes, 3.5m each) ──
function _buildRoad(scene) {
  var roadGroup = new THREE.Group();
  var ROAD_LEN = 3000;     // total road length along X (3000m covers full demo)
  var ROAD_HALF = 7.5;     // half-width (4 lanes x 3.5m + shoulders)
  var ASPHALT = 0x333844;
  var MARKING = 0xddeeff;
  var YELLOW = 0xddbb55;

  // Asphalt base
  var asphalt = _makeBox(ROAD_LEN, 0.08, ROAD_HALF * 2, ASPHALT, 0, 0.9);
  asphalt.position.set(0, 0.01, 0);
  asphalt.receiveShadow = true;
  roadGroup.add(asphalt);

  // Shoulders (slightly lighter)
  var shld = _makeBox(ROAD_LEN, 0.04, 1.5, 0x556655, 0, 1);
  shld.position.set(0, 0.02, ROAD_HALF + 0.75);
  shld.receiveShadow = true;
  shld.userData.isEdge = true; shld.userData.baseZ = ROAD_HALF + 0.75;
  roadGroup.add(shld);
  var shld2 = shld.clone();
  shld2.position.z = -(ROAD_HALF + 0.75);
  shld2.userData.isEdge = true; shld2.userData.baseZ = -(ROAD_HALF + 0.75);
  roadGroup.add(shld2);

  // Dashed lane dividers at Z = -3.5, 0 (yellow), 3.5
  // Lane markings tagged with userData for curve reshaping by type
  var dashLen = 4.0, gap = 6.0, segLen = dashLen + gap;   // fewer segments for perf
  var nSeg = Math.floor(ROAD_LEN / segLen);
  for (var zi = 0; zi < 3; zi++) {
    var zPos = -3.5 + zi * 3.5;
    var isCenter = (zi === 1);
    for (var s = -Math.floor(nSeg / 2); s < Math.floor(nSeg / 2); s++) {
      var dash = _makeBox(dashLen, 0.01, 0.12, isCenter ? YELLOW : MARKING, 0, 0.3);
      dash.position.set(s * segLen + dashLen / 2, 0.07, zPos);
      dash.userData.isLaneMark = true;
      dash.userData.baseZ = zPos;
      roadGroup.add(dash);
    }
    if (isCenter) {
      for (var s2 = -Math.floor(nSeg / 2); s2 < Math.floor(nSeg / 2); s2++) {
        var d2 = _makeBox(dashLen, 0.01, 0.12, YELLOW, 0, 0.3);
        d2.position.set(s2 * segLen + dashLen / 2, 0.07, zPos + 0.22);
        d2.userData.isLaneMark = true;
        d2.userData.baseZ = zPos + 0.22;
        roadGroup.add(d2);
      }
    }
  }

  // Road shoulder edge markings
  for (var side = -1; side <= 1; side += 2) {
    var edge = _makeBox(ROAD_LEN, 0.01, 0.15, MARKING, 0, 0.3);
    edge.position.set(0, 0.07, side * 7.0);
    edge.userData.isEdge = true;
    edge.userData.baseZ = side * 7.0;
    roadGroup.add(edge);
  }

  scene.add(roadGroup);
  _roadGroup = roadGroup;
}

// ── Curve road using scene road geometry data ──
// Matches road_center_y() in include/road_geometry.h:
//   0 (x < sx), off*(3t²-2t³) (smoothstep), off (x >= sx+len)
// Road surface + long shoulders/edges -> deform vertices;
// short lane dashes -> translate position.z (single offset fine for 4m segments).
// sim_world Y = Three.js Z: positive curve_offset = right turn = positive Z shift.
// When curve is active, disable road chunking, otherwise chunk movement after deformation
// destroys the shape.
function _curveShiftAt(x, sx, len, off) {
  if (len <= 0 || Math.abs(off) < 0.01) return 0;
  if (x <= sx) return 0;
  if (x >= sx + len) return off;
  var t = (x - sx) / len;
  return off * (3 * t * t - 2 * t * t * t);
}

function _applyRoadCurve(roadData) {
  if (!roadData) return;
  var sx = roadData.curve_start_x || 0, len = roadData.curve_length_m || 0;
  var off = roadData.curve_offset_m || 0;
  var key = sx + "," + len + "," + off;
  if (key === _lastCurveKey) return;
  _lastCurveKey = key;
  if (len <= 0 || Math.abs(off) < 0.01) {
    _curveActive = false;
    return;
  }
  _curveActive = true;
  console.log('[scene3d] Applying road curve: start=' + sx + 'm, len=' + len + 'm, off=' + off + 'm');
  reportDiag('scene3d.curve', 'Applying curve: offset=' + off + 'm over X=' + sx + '..' + (sx + len));
  var group = _roadGroup;
  if (!group) return;
  group.position.x = 0;  // reset chunk displacement
  group.traverse(function(child) {
    if (!child.isMesh || !child.geometry) return;
    // Short lane dashes (4m): single Z offset by center X is accurate enough
    if (child.userData && child.userData.isLaneMark) {
      var baseZ = child.userData.baseZ || child.position.z;
      var meshX = child.position.x;
      child.position.z = baseZ + _curveShiftAt(meshX, sx, len, off);
      return;
    }
    // Road surface, shoulders, edge markings (long meshes): deform vertex Z
    var pos = child.geometry.attributes.position;
    if (!pos) return;
    var arr = pos.array;
    for (var i = 0; i < arr.length; i += 3) {
      var vx = arr[i];
      arr[i + 2] = arr[i + 2] + _curveShiftAt(vx, sx, len, off);
    }
    pos.needsUpdate = true;
    child.geometry.computeVertexNormals();
  });
}

// ── Environment: buildings + trees ──
function _buildEnvironment(scene) {
  var env = new THREE.Group();
  // Buildings (reduced for performance)
  // NOTE: lateral (Z) placement must keep the building's near face clear of
  // the chase camera's operating envelope (ego lane offset + camera "side"
  // lerp target, up to ~9m from the road centerline — see the `side`
  // chase-camera offset in _renderFrame()).
  // Previously buildings could spawn with an
  // inner edge as close as 7m from centerline, which is *inside* the
  // camera's reachable zone: the chase camera would end up embedded in the
  // building mesh, making the ego car (and everything else) invisible.
  var bldColors = [0x445566, 0x556677, 0x4a5a6a, 0x3d4d5d, 0x5a6a7a];
  for (var b = 0; b < 12; b++) {
    var side = (b % 2 === 0) ? 1 : -1;
    var bx = (b - 6) * 200 + Math.random() * 80;
    var bw = 4 + Math.random() * 6;
    // Minimum offset keeps the near face (bz - bw/2) at least ~18m from the
    // road centerline, safely outside the camera's reach.
    var bz = side * (23 + Math.random() * 25);
    var bh = 10 + Math.random() * 35;
    var bld = _makeBox(bw, bh, bw, bldColors[b % bldColors.length], 0.1, 0.85);
    bld.position.set(bx, bh / 2, bz);
    bld.castShadow = true; bld.receiveShadow = true;
    env.add(bld);
  }
  // Trees (reduced)
  for (var t = 0; t < 20; t++) {
    var tx = (t - 10) * 140 + Math.random() * 60;
    // Same clearance rationale as buildings above (canopy radius up to
    // ~3.5m, so a base offset of 16 keeps the near edge >= ~12.5m clear).
    var tz = (t % 2 === 0 ? 1 : -1) * (16 + Math.random() * 35);
    // Trunk
    var trunk = _makeCyl(0.2, 0.3, 2 + Math.random() * 3, 8, 0x6b4423);
    trunk.position.set(tx, 1, tz);
    trunk.castShadow = true;
    env.add(trunk);
    // Canopy layers
    for (var cl = 0; cl < 3; cl++) {
      var cr = 1.5 + Math.random() * 2;
      var canopy = _makeCyl(0, cr, 2 + Math.random() * 2, 8, 0x2d5a27 + cl * 0x101010);
      canopy.position.set(tx, 2.5 + cl * 1.5, tz);
      canopy.castShadow = true;
      env.add(canopy);
    }
  }
  scene.add(env);
  _envGroup = env;
}

// ══════════════════════════════════════════════════════════════════════════════
// init3DScene — create the full ADAS visualization
// ══════════════════════════════════════════════════════════════════════════════

function resize3D() {
  if (!renderer3d || !camera3d) return;
  var el = document.getElementById("scene3d");
  if (!el) return;
  var w = el.clientWidth, h = el.clientHeight;
  if (w < 10 || h < 10) return;
  renderer3d.setSize(w, h);
  camera3d.aspect = w / h;
  camera3d.updateProjectionMatrix();
}

function init3DScene() {
  var el = document.getElementById("scene3d"); if (!el || typeof THREE === "undefined") return;
  // Use fallback size when card is collapsed (clientWidth=0)
  var w = el.clientWidth || el.parentElement && el.parentElement.clientWidth || 800;
  var h = el.clientHeight || 400;

  // Tear down any previous renderer/canvas first. Without this, re-entering
  // init3DScene() (e.g. on webglcontextrestored) leaves the old dead <canvas>
  // stacked in the DOM alongside the new one — the stale element can end up
  // covering the freshly-rendered scene, making the ego car (and everything
  // else) appear to vanish even though rendering is actually working fine.
  var staleCanvasCount = 0;
  if (renderer3d) {
    // dispose() only frees GPU resources for a context that's still alive;
    // it's a no-op (and may throw) once the context is already lost — safe
    // to ignore since the goal here is just removing the stale DOM element.
    try { renderer3d.dispose(); } catch (disposeErr) {
      console.warn('[scene3d.init] renderer dispose failed (expected if context already lost):', disposeErr);
    }
    if (renderer3d.domElement && renderer3d.domElement.parentNode) {
      renderer3d.domElement.parentNode.removeChild(renderer3d.domElement);
      staleCanvasCount++;
    }
  }
  // Defensive: remove any other leftover <canvas> children (e.g. from a
  // context-loss race where cleanup above didn't run before a new canvas
  // was appended).
  Array.from(el.querySelectorAll('canvas')).forEach(function(c) {
    if (c.parentNode) { c.parentNode.removeChild(c); staleCanvasCount++; }
  });
  // Surface this in the diagnostics bar / console — a repeated context
  // loss/restore cycle (GPU reset, tab backgrounded then foregrounded,
  // driver hiccup) is otherwise invisible and shows up to users only as
  // "the car disappeared", with no trace to debug from.
  if (staleCanvasCount > 0) {
    reportDiag('scene3d.init', 'removed ' + staleCanvasCount + ' stale <canvas> element(s) before rebuilding 3D scene (likely WebGL context restore)');
  }

  // Scene
  scene3d = new THREE.Scene();
  scene3d.background = new THREE.Color(0x334466);
  scene3d.fog = new THREE.Fog(0x334466, 80, 350);

  // Camera — chase cam. Wider FOV keeps the whole car visible on small screens.
  camera3d = new THREE.PerspectiveCamera(60, w / h, 0.2, 500);

  // Renderer
  renderer3d = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer3d.setSize(w, h);
  renderer3d.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer3d.toneMapping = THREE.ACESFilmicToneMapping;
  renderer3d.toneMappingExposure = 1.1;
  el.appendChild(renderer3d.domElement);
  document.getElementById("scene3d-msg").style.display = "none";

  // WebGL context loss (GPU reset, tab backgrounded, driver hiccup) must not
  // crash the render loop. Suspend cleanly on loss, rebuild on restore; fall
  // back to the 2D canvas while the context is gone.
  renderer3d.domElement.addEventListener("webglcontextlost", function(ev) {
    ev.preventDefault();               // required so 'restored' can fire later
    reportDiag('scene3d', 'WebGL context lost — using 2D fallback');
    _glLost = true;
    try { window.init2DFallback(); } catch (_) { }
  }, false);
  renderer3d.domElement.addEventListener("webglcontextrestored", function() {
    _glLost = false;
    safeCall('scene3d.restore', function() { init3DScene(); });
  }, false);

  // Lighting
  scene3d.add(new THREE.AmbientLight(0x8899cc, 0.7));
  var sun = new THREE.DirectionalLight(0xfff8ee, 2.0);
  sun.position.set(30, 40, 15);
  scene3d.add(sun);
  // Hemisphere (sky/ground ambient)
  scene3d.add(new THREE.HemisphereLight(0x8899cc, 0x443322, 0.4));

  // ── Ground (large flat plane) ──
  var gndGeo = new THREE.PlaneGeometry(400, 400);
  var gndMat = new THREE.MeshStandardMaterial({ color: 0x3a5a30, roughness: 0.95 });
  var gnd = new THREE.Mesh(gndGeo, gndMat);
  gnd.rotation.x = -Math.PI / 2; gnd.position.y = -0.05;
  gnd.receiveShadow = true;
  scene3d.add(gnd);
  _groundMesh = gnd;

  // ── Road ──
  _buildRoad(scene3d);
  _applyRoadCurve(null);  // prime curve key (no data yet, will apply when scene data arrives)

  // ── Environment ──
  _buildEnvironment(scene3d);

  // ── Ego vehicle ──
  var egoCar = _buildSedan(0x4488dd, 0x3377bb);
  egoCar.position.set(0, 0, 0);
  egoCar.castShadow = true;
  scene3d.add(egoCar);
  _carGroup = egoCar;

  // ── Obstacle pool ──
  _obsPool = [];
  for (var oi = 0; oi < 8; oi++) {
    var obs = _buildObstacle(0xff9944);
    obs.visible = false;
    scene3d.add(obs);
    _obsPool.push(obs);
  }

  // ── LiDAR point cloud ──
  var lgeo = new THREE.BufferGeometry();
  var lpts = new Float32Array(900);
  for (var i = 0; i < 300; i++) {
    lpts[i * 3] = Math.random() * 80 - 10; lpts[i * 3 + 1] = Math.random() * 3;
    lpts[i * 3 + 2] = Math.random() * 30 - 15;
  }
  lgeo.setAttribute("position", new THREE.BufferAttribute(lpts, 3));
  var lmat = new THREE.PointsMaterial({ color: 0x44ff44, size: 0.22, transparent: true, opacity: 0.7 });
  var lcloud = new THREE.Points(lgeo, lmat);
  scene3d.add(lcloud);
  _lidarCloud = lcloud;

  // ── Camera state for smooth follow ──
  _cam = new THREE.Vector3(-22, 11, 6);
  _camLook = new THREE.Vector3(4, 1.2, -1.75);
  _camTarget = new THREE.Vector3(-22, 11, 6);
  _camLookTarget = new THREE.Vector3(4, 1.2, -1.75);

  // ── Initialize dead reckoning state ──
  initDeadReckon();

  // ── Pre-allocate temp vectors (after THREE is loaded) ──
  _tmpV3 = new THREE.Vector3();
  _tmpScale = new THREE.Vector3(1, 1, 1);

  // ── Animation loop ──
  function anim3D() {
    requestAnimationFrame(anim3D);
    if (!sceneReady) return;
    if (_glLost) return;               // skip while WebGL context is lost
    if (!renderer3d || !scene3d || !camera3d) return;
    safeCall('scene3d.frame', function() {
      _renderFrame();
      renderer3d.render(scene3d, camera3d);
    });
  }
  anim3D();
  sceneReady = true;
  window.scene3d = scene3d;
  window.sceneReady = true;
}

// ══════════════════════════════════════════════════════════════════════════════
// Per-frame render: world-space scene, chase camera
// Road is STATIC at world origin. Ego moves through the world.
// Camera follows ego. Other vehicles move independently.
// ══════════════════════════════════════════════════════════════════════════════

function _renderFrame() {
  _animT += 0.016;
  var ego = _carGroup; if (!ego) return;
  var now = performance.now() / 1000;

  // ── Dead reckoning ──
  if (_dr.init && _dr.lastTime > 0) {
    var dt = Math.min(now - _dr.lastTime, 2.0);
    _dr.targetX = _dr.lastX + _dr.lastSpeed * dt;
    _dr.targetZ = _dr.lastZ;
  }
  var lerpF = 0.15;
  _dr.smoothX += (_dr.targetX - _dr.smoothX) * lerpF;
  _dr.smoothZ += (_dr.targetZ - _dr.smoothZ) * lerpF;
  _dr.smoothHeading += (_dr.lastHeading - _dr.smoothHeading) * lerpF;

  var sx = _dr.smoothX, sz = _dr.smoothZ;

  // ── Ego car: world-space position (road is STATIC at origin) ──
  if (ego) { ego.position.set(sx, ego.position.y, sz); ego.rotation.y = -_dr.smoothHeading; }

  // Keep the static road/ground near ego for long-running demos. This avoids
  // driving beyond the initial mesh and ending up with an empty/dark 3D view.
  // When curve is active, disable chunking: the deformed road mesh vertices
  // are bound to local coordinates, and moving the chunk would break the shape.
  if (!_curveActive) {
    var chunkX = Math.round(sx / 1000) * 1000;
    if (_roadGroup) _roadGroup.position.x = chunkX;
    if (_groundMesh) _groundMesh.position.x = chunkX;
    if (_envGroup) _envGroup.position.x = chunkX;
  }

  // ── Chase camera: behind + above ego in world space ──
  var narrow = (renderer3d && renderer3d.domElement && renderer3d.domElement.clientWidth < 700);
  var back = narrow ? 26 : 22;
  var height = narrow ? 12 : 11;
  var side = narrow ? 7 : 6;
  _camTarget.set(sx - back, height, sz + side);
  _camLookTarget.set(sx + 4, 1.2, sz);
  _cam.lerp(_camTarget, 0.08);
  _camLook.lerp(_camLookTarget, 0.08);
  camera3d.position.copy(_cam);
  camera3d.lookAt(_camLook);

  // Road + environment are STATIC — no scrolling. The camera follows
  // the ego through the world so the road appears to flow past naturally.

  // ── Obstacles: use stored WORLD positions (not ego-relative) ──
  // World positions are set by update3D() when fresh data arrives.
  // Here we just smooth-lerp toward those fixed world targets so
  // obstacles stay anchored in their real lanes regardless of ego motion.
  if (_obsPool && _obsWorld) {
    // Half-length of ego vehicle for overlap-safe occlusion threshold
    var EGO_HALF_LEN = 2.3;
    // Default obstacle length when not specified in data
    var DEFAULT_OBS_LEN = 4;
    for (var oi = 0; oi < _obsPool.length; oi++) {
      var om = _obsPool[oi];
      var ow = _obsWorld[oi];
      if (ow) {
        var L = ow.len || DEFAULT_OBS_LEN, W = ow.wid || 2;
        // Type-based real height: trucks tall, pedestrians slim.
        var H = _OBS_H[ow.type] || 1.5;
        // Extrapolate by world speed to current time → 60fps smooth,
        // no longer jitters at 10Hz data ticks.
        var dtObs = Math.min(now - (ow.t0 || now), 1.0);
        var tx = ow.x + (ow.vx || 0) * dtObs, tz = ow.z + (ow.vz || 0) * dtObs;
        var distToEgo = Math.sqrt((tx - sx) * (tx - sx) + (tz - sz) * (tz - sz));
        // Bounding-box-aware occlusion: hide only when truly overlapping ego body
        if (distToEgo < L * 0.5 + EGO_HALF_LEN) { om.visible = false; continue; }
        // Unit-normalised model: group origin at road surface (y=0.05).
        // Children sit at fractions 0-1 so scale.set maps directly to metres.
        _tmpV3.set(tx, 0.05, tz);
        if (!om.visible || om.position.distanceTo(_tmpV3) > 35) {
          om.position.copy(_tmpV3);
        } else {
          om.position.lerp(_tmpV3, 0.18);
        }
        _tmpScale.set(L, H, W); om.scale.lerp(_tmpScale, 0.18);
        var c = (_obsColors[ow.type]) || 0xff9944;
        if (om.children.length > 0 && om.children[0].material && om.children[0].material.color) {
          om.children[0].material.color.setHex(c);
        }
        om.visible = true;
      } else { om.visible = false; }
    }
  }

  // ── LiDAR: use stored world points ──
  if (_lidarCloud && _lidarWorld && _lidarWorld.length) {
    var wl = _lidarWorld, pos = _lidarCloud.geometry.attributes.position.array;
    for (var i = 0; i < 300 && i * 3 < pos.length && i < wl.length; i++) {
      pos[i * 3] = wl[i].x; pos[i * 3 + 1] = wl[i].y; pos[i * 3 + 2] = wl[i].z;
    }
    _lidarCloud.geometry.attributes.position.needsUpdate = true;
  }

  // ── Car bounce ──
  var v = (window.topoData.metrics || {}).vehicle || {};
  if (ego && v.speed > 0.5) {
    ego.position.y = 0.05 + Math.sin(_animT * 6.5) * 0.008 * Math.min(1, v.speed * 0.12);
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// update3D — called from updateAll() on every SSE data tick.
// Feeds fresh ground-truth into the dead-reckoning state.
// ══════════════════════════════════════════════════════════════════════════════

function update3D() {
  if (!sceneReady) return;
  var m = window.topoData.metrics || {}, scn = m.scene, v = m.vehicle || {};
  var now = performance.now() / 1000;

  // Road curve geometry: apply once when scene road data arrives
  if (scn && scn.road) _applyRoadCurve(scn.road);

  // Ego position from scene data
  if (scn && scn.ego) {
    var newX = scn.ego.x || 0;
    var newZ = (scn.ego.y || 0);   // ego.y (m): -1.75=left lane center, +1.75=right lane center
    var newSpeed = scn.ego.speed || v.speed || 0;
    var newHeading = scn.ego.heading || 0;
    if (!_dr.init || Math.abs(newX - _dr.lastX) > 0.01 || Math.abs(newZ - _dr.lastZ) > 0.01 || Math.abs(newSpeed - _dr.lastSpeed) > 0.1) {
      _dr.lastX = newX;
      _dr.lastZ = newZ;
      _dr.lastSpeed = newSpeed;
      _dr.lastHeading = newHeading;
      _dr.lastTime = now;
      _dr.init = true;
    }
  } else if (v) {
    var nx = v.x || 0, ns = v.speed || 0, nz = (v.y || 0);   // v.y (m)
    if (!_dr.init || Math.abs(nx - _dr.lastX) > 0.01 || Math.abs(nz - _dr.lastZ) > 0.01) {
      _dr.lastX = nx;
      _dr.lastZ = nz;
      _dr.lastSpeed = ns;
      _dr.lastTime = now;
      _dr.init = true;
    }
  }

  // Obstacles: convert ego-relative → WORLD coords once, store as targets.
  // This prevents obstacles from drifting when ego moves laterally between
  // data ticks — they stay anchored in their real-world lanes.
  if (_obsPool && scn && scn.obstacles) {
    var obs = scn.obstacles;
    for (var oi = 0; oi < _obsPool.length; oi++) {
      var om = _obsPool[oi];
      if (oi < obs.length) {
        var o = obs[oi];
        var wx = _dr.lastX + (o.x || 0), wz = _dr.lastZ + (o.y || 0);
        // Store base position + world velocity + base timestamp →
        // _renderFrame extrapolates by velocity to avoid jerky stepping
        // for fast obstacles (oncoming car -9m/s) at 10Hz data ticks.
        _obsWorld[oi] = { x: wx, z: wz, vx: (o.vx || 0), vz: (o.vy || 0),
          t0: now, len: o.len || 4, wid: o.wid || 2, type: o.type };
        om.visible = true;
      } else { om.visible = false; _obsWorld[oi] = null; }
    }
  }
  // LiDAR: convert ego-relative → world once
  if (_lidarCloud && scn && scn.lidar && scn.lidar.length) {
    var pos = _lidarCloud.geometry.attributes.position.array;
    var n = scn.lidar.length;
    for (var i = 0; i < 300 && i * 3 < pos.length; i++) {
      if (i < n) {
        var pt = scn.lidar[i];
        var lx = _dr.lastX + (pt[0] || 0), ly = (pt[2] || 0) + 0.15, lz = _dr.lastZ + (pt[1] || 0);
        pos[i * 3] = lx; pos[i * 3 + 1] = ly; pos[i * 3 + 2] = lz;
        if (i < _lidarWorld.length) { _lidarWorld[i] = { x: lx, y: ly, z: lz }; }
        else { _lidarWorld.push({ x: lx, y: ly, z: lz }); }
      } else { pos[i * 3] = _dr.lastX; pos[i * 3 + 1] = -2; pos[i * 3 + 2] = _dr.lastZ; }
    }
    _lidarCloud.geometry.attributes.position.needsUpdate = true;
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// Exports
// ══════════════════════════════════════════════════════════════════════════════

export { init3DScene, resize3D, update3D, sceneReady, scene3d, _renderFrame, _applyRoadCurve };

// Expose init3DScene globally so inline onclick handlers work
window.init3DScene = init3DScene;
