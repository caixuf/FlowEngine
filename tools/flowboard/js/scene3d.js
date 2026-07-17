/* global THREE, window, topoData */

// ══════════════════════════════════════════════════════════════════════════════
// scene3d.js — Three.js 3D scene module for ADAS visualization
// ══════════════════════════════════════════════════════════════════════════════

import { safeCall, reportDiag, _makeBox, _makeRect, _buildSedan, _buildObstacle, _buildTrafficLight, _buildETCGate } from './utils.js';
import { initDeadReckon, tickDeadReckon, _dr } from './deadreckon.js';
import { init2DFallback } from './scene2d.js';
import { initModelCache, buildEgoCar, buildObstacleGroup } from './models.js';

const THREE = window.THREE;

// ══════════════════════════════════════════════════════════════════════════════
// Module-level state (replaces former window globals)
// ══════════════════════════════════════════════════════════════════════════════

/** Main scene, camera, renderer */
let scene3d = null, camera3d = null, renderer3d = null;
let sceneReady = false;
// Phase 4.9: debug-cam state lives module-scoped, debug3d.html sets it
// via the exposed setDebugCam() function (see bottom of file).
let _debugCam = null;
export function setDebugCam(v) { _debugCam = v; }

// Live topology data (Phase 4.9: no longer read from window.topoData).
// app.js calls setTopoData() from updateAll() / sync2DTarget().
let _topoData = { nodes: [], metrics: {} };
export function setTopoData(d) { _topoData = d || _topoData; }

/** Camera chase-cam state vectors */
let _cam = null, _camLook = null, _camTarget = null, _camLookTarget = null;

/** Scene object references — road, ground, environment, car */
let _lidarCloud = null;
let _lidarWorld = [];
let _obsPool = [], _obsWorld = [];
let _trafficLightPool = [], _trafficLightWorld = [];
let _roadGroup = null, _groundMesh = null, _envGroup = null, _carGroup = null;
let _sunLight = null;  /* DirectionalLight 引用，供 _renderFrame 跟随 ego 更新阴影 */
let _composer = null;  /* EffectComposer 引用（Bloom 后处理），为 null 时直渲 */

/** Obstacle type → colour lookup (defined once, shared) */
const _obsColors = { car: 0xff9944, truck: 0xff4422, pedestrian: 0x33ff88, cyclist: 0x33ddff, cone: 0xff6600 };

/** NOA Phase 5: NPC AI 状态 → 显示色（与场景实体的 ai 字段对齐，见 scene_pub.cpp）。
 *  cruise/follow/stop/stop_for_tl/etc_approach/branch_sel/merge/yield。 */
const _aiLabelColors = {
  cruise: 0x88aacc, follow: 0xaacc88, stop: 0xff5555, stop_for_tl: 0xffaa55,
  etc_approach: 0xffcc44, branch_sel: 0x44ffcc, merge: 0xcc88ff, yield: 0xff88aa
};
let _obsLabelPool = [];   // per-obstacle Sprite (NPC AI state label, Phase 5)
let _obsLabelLast = [];   // 上次显示的 ai 字符串，避免每帧重建 CanvasTexture

/** Road curve state */
let _curveActive = false;
let _lastCurveKey = "";

/** Phase 3: Road network group (multi-segment road from scene/frame).
 *  当 scene.road_network 存在时，_roadNetworkGroup 取代旧的 _roadGroup。 */
let _roadNetworkGroup = null;
let _lastRoadNetworkKey = "";

/** Phase 3: ETC gate pool (抬杆门架，从 scene/frame entities 读取) */
let _etcGatePool = [], _etcGateWorld = [];

/** WebGL context-loss flag — render loop skips while true */
let _glLost = false;

/** Animation time counter (incremented per frame) */
let _animT = 0;

/** Pre-allocated vector/scale objects (avoids per-frame GC pressure) */
let _tmpV3 = null, _tmpScale = null;

/** Obstacle height lookup (defined once, shared across all _renderFrame calls) */
const _OBS_H = { truck: 2.8, pedestrian: 1.8, cyclist: 1.7, cone: 0.8 };

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

// ── NOA Phase 5: NPC AI 状态标签 sprite（CanvasTexture） ──
// 每个障碍物槽位配一个 Sprite，浮在车顶上方显示当前 ai 状态。
// ai 字符串变化时才重建 CanvasTexture，避免每帧 GC 压力。
function _makeLabelSprite() {
  var canvas = document.createElement('canvas');
  canvas.width = 128; canvas.height = 32;
  var tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.LinearFilter;
  var mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
  var sp = new THREE.Sprite(mat);
  sp.scale.set(4, 1, 1);   // 世界坐标 4m 宽 1m 高
  sp.visible = false;
  return sp;
}

function _setLabelSprite(sp, text, colorHex) {
  if (!sp || !sp.material || !sp.material.map) return;
  var canvas = sp.material.map.image;
  var ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (text) {
    // 圆角背景
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    var r = 6, w = canvas.width, h = canvas.height;
    ctx.beginPath();
    ctx.moveTo(r, 0); ctx.lineTo(w - r, 0);
    ctx.quadraticCurveTo(w, 0, w, r); ctx.lineTo(w, h - r);
    ctx.quadraticCurveTo(w, h, w - r, h); ctx.lineTo(r, h);
    ctx.quadraticCurveTo(0, h, 0, h - r); ctx.lineTo(0, r);
    ctx.quadraticCurveTo(0, 0, r, 0); ctx.closePath(); ctx.fill();
    // 文字
    var hex = ('000000' + (colorHex & 0xffffff).toString(16)).slice(-6);
    ctx.fillStyle = '#' + hex;
    ctx.font = 'bold 18px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, w / 2, h / 2 + 1);
  }
  sp.material.map.needsUpdate = true;
}

// ════════════════════════════════════════════════════════════════════
// Road builder — 2 lanes × 3.5m, split into 2m segment groups.
// Each group bundles asphalt + 2 edges + 2 shoulders.
// Curve shifts group.position.z — NO vertex deformation.
// SEG_LEN=2m → Z step < 7cm across a 260m/9m curve → smooth.
// ════════════════════════════════════════════════════════════════════
function _buildRoad(scene) {
  var roadGroup = new THREE.Group();
  var ROAD_LEN   = 3000;
  var LANE_W     = 3.5;
  var LANE_N     = 2;
  var ROAD_HALF  = LANE_W * LANE_N / 2;
  var MARGIN     = 0.5;
  var ASPHALT_W  = (ROAD_HALF + MARGIN) * 2;
  var SHLD_W     = 1.5;
  var shldZ      = ROAD_HALF + MARGIN + SHLD_W / 2;
  var SEG_LEN    = 2;
  var nSeg       = Math.floor(ROAD_LEN / SEG_LEN);

  var aGeo = new THREE.BoxGeometry(SEG_LEN, 0.08, ASPHALT_W, 1, 1, 1);
  var aMat = new THREE.MeshStandardMaterial({ color: 0x3a3f4a, roughness: 0.9 });
  var eGeo = new THREE.BoxGeometry(SEG_LEN, 0.02, 0.28, 1, 1, 1);
  var eMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x333333, roughness: 0.3 });
  var sGeo = new THREE.BoxGeometry(SEG_LEN, 0.04, SHLD_W, 1, 1, 1);
  var sMat = new THREE.MeshStandardMaterial({ color: 0x556655, roughness: 1.0 });

  for (var si = -Math.floor(nSeg / 2); si < Math.floor(nSeg / 2); si++) {
    var cx = si * SEG_LEN + SEG_LEN / 2;
    var seg = new THREE.Group();
    seg.position.set(cx, 0, 0);
    seg.userData.isRoadSeg = true;
    seg.userData.baseX = cx;

    var a = new THREE.Mesh(aGeo, aMat);
    a.position.y = 0.01; a.receiveShadow = true;
    seg.add(a);

    for (var ei = -1; ei <= 1; ei += 2) {
      var e = new THREE.Mesh(eGeo, eMat);
      e.position.set(0, 0.048, ei * ROAD_HALF);
      seg.add(e);
    }
    for (var si2 = -1; si2 <= 1; si2 += 2) {
      var s = new THREE.Mesh(sGeo, sMat);
      s.position.set(0, 0.02, si2 * shldZ);
      s.receiveShadow = true;
      seg.add(s);
    }
    roadGroup.add(seg);
  }

  // Centre double yellow dashes
  var DASH = 4.5, GAP = 5.5, STEP = DASH + GAP;
  var nDash = Math.floor(ROAD_LEN / STEP);
  var dGeo = new THREE.BoxGeometry(DASH, 0.02, 0.22, 1, 1, 1);
  var dMat = new THREE.MeshStandardMaterial({ color: 0xffcc44, emissive: 0x333333, roughness: 0.3 });
  for (var d = -Math.floor(nDash / 2); d < Math.floor(nDash / 2); d++) {
    var dx = d * STEP + DASH / 2;
    for (var dy = -1; dy <= 1; dy += 2) {
      var dash = new THREE.Mesh(dGeo, dMat);
      dash.position.set(dx, 0.043, dy * 0.12);
      dash.userData.isLaneMark = true;
      dash.userData.baseZ = dy * 0.12;
      roadGroup.add(dash);
    }
  }

  scene.add(roadGroup);
  _roadGroup = roadGroup;
}

function _curveShiftAt(x, sx, len, off) {
  if (len <= 0 || Math.abs(off) < 0.01) return 0;
  if (x <= sx) return 0;
  if (x >= sx + len) return off;
  var t = (x - sx) / len;
  return off * (3 * t * t - 2 * t * t * t);
}

/** Road centerline tangent heading at x (radians), mirror of road_center_heading() in C. */
function _curveHeadingAt(x, sx, len, off) {
  if (len <= 0 || Math.abs(off) < 0.01 || x <= sx) return 0;
  if (x >= sx + len) return 0;
  var t = (x - sx) / len;
  // smoothstep derivative: d/dt (3t² - 2t³) = 6t - 6t²
  var dy = off * (6 * t - 6 * t * t) / len;
  return Math.atan(dy);
}

function _applyRoadCurve(roadData) {
  if (!roadData) return;
  var sx = roadData.curve_start_x || 0, len = roadData.curve_length_m || 0;
  var off = roadData.curve_offset_m || 0;
  var key = sx + "," + len + "," + off;
  if (key === _lastCurveKey) return;
  _lastCurveKey = key;
  if (len <= 0 || Math.abs(off) < 0.01) { _curveActive = false; return; }
  _curveActive = true;
  var group = _roadGroup;
  if (!group) return;
  group.traverse(function(child) {
    if (child.userData && child.userData.isRoadSeg) {
      var bx = child.userData.baseX;
      child.position.z = _curveShiftAt(bx, sx, len, off);
      // 绕 Y 轴旋转使 segment 跟随道路切线方向，消除折线感
      child.rotation.y = _curveHeadingAt(bx, sx, len, off);
      return;
    }
    if (child.userData && child.userData.isLaneMark) {
      child.position.z = child.userData.baseZ + _curveShiftAt(child.position.x, sx, len, off);
      return;
    }
  });
}

// ══════════════════════════════════════════════════════════════════════════════
// Phase 3: 多段道路网络渲染 — 从 scene/frame 的 road_network.edges 构建
// 使用 CatmullRomCurve3 + 自定义 ribbon mesh 生成沿曲线的平坦路面。
// 每条 edge 的 nodes [[x,y],...] 是道路参考线控制点，x=纵向 y=横向。
// ══════════════════════════════════════════════════════════════════════════════

/**
 * _buildRoadNetwork — 从 road_network.edges 构建多段道路网格。
 *
 * 每条 edge 用 CatmullRomCurve3 平滑插值 nodes 控制点，然后沿曲线
 * 采样 N 段，每段生成一个梯形面片（左右边缘点 × 当前+下一点），
 * 拼成 triangle strip 形成平坦路面。车道线沿曲线虚线绘制。
 *
 * 坐标系：scene/frame 的 [x, y] → THREE.Vector3(x, 0, y)
 *   x=纵向(forward), y=横向(lateral/Z轴), Y=up
 *
 * @param {Array} edges  road_network.edges 数组
 */
function _buildRoadNetwork(edges) {
  if (!edges || !edges.length || !scene3d) return;

  // 缓存键：用 edges 的 id+nodes 数量拼接，避免每帧重建
  var key = '';
  for (var i = 0; i < edges.length; i++) {
    key += (edges[i].id || i) + ':' + (edges[i].nodes ? edges[i].nodes.length : 0) + ',';
  }
  if (key === _lastRoadNetworkKey) return;  // 未变化，跳过重建
  _lastRoadNetworkKey = key;

  // 清除旧的道路网络
  if (_roadNetworkGroup) {
    scene3d.remove(_roadNetworkGroup);
    _roadNetworkGroup.traverse(function(child) {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
    _roadNetworkGroup = null;
  }

  var group = new THREE.Group();
  var SEG_LEN = 3;  // 3m 一段，平衡精度与性能

  for (var ei = 0; ei < edges.length; ei++) {
    var edge = edges[ei];
    var nodes = edge.nodes;
    if (!nodes || nodes.length < 2) continue;

    // 从 nodes 构建 CatmullRomCurve3 控制点
    var points = [];
    for (var ni = 0; ni < nodes.length; ni++) {
      points.push(new THREE.Vector3(nodes[ni][0], 0, nodes[ni][1]));
    }
    var curve = new THREE.CatmullRomCurve3(points);

    var length = edge.length || curve.getLength();
    var lanes = edge.lanes || 2;
    var laneWidth = edge.lane_width || 3.5;
    var roadWidth = lanes * laneWidth;
    var roadHalf = roadWidth / 2;

    // ── 路面 ribbon mesh ──
    var nSeg = Math.max(4, Math.floor(length / SEG_LEN));
    var positions = [];
    var indices = [];
    for (var si = 0; si <= nSeg; si++) {
      var t = si / nSeg;
      var pos = curve.getPointAt(t);
      var tangent = curve.getTangentAt(t);
      // 法线（切线在 XZ 平面内旋转 90°）
      var nx = -tangent.z, nz = tangent.x;

      // 左右边缘点
      positions.push(pos.x - nx * roadHalf, 0.01, pos.z - nz * roadHalf);
      positions.push(pos.x + nx * roadHalf, 0.01, pos.z + nz * roadHalf);

      if (si < nSeg) {
        var base = si * 2;
        indices.push(base, base + 1, base + 2);
        indices.push(base + 1, base + 3, base + 2);
      }
    }

    var roadGeo = new THREE.BufferGeometry();
    roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    roadGeo.setIndex(indices);
    roadGeo.computeVertexNormals();
    // 沥青路面：深灰近黑，高粗糙度（无镜面反射）
    var roadMat = new THREE.MeshStandardMaterial({ color: 0x2a2d30, roughness: 0.95, metalness: 0.02 });
    var roadMesh = new THREE.Mesh(roadGeo, roadMat);
    roadMesh.receiveShadow = true;
    group.add(roadMesh);

    // ── 路肩（道路两侧的浅灰边缘带，比沥青略亮）──
    var shldW = 1.0;
    var shldHalf = roadHalf + shldW / 2;
    var sPos = [], sIdx = [];
    for (var si2 = 0; si2 <= nSeg; si2++) {
      var t2 = si2 / nSeg;
      var p2 = curve.getPointAt(t2);
      var tan2 = curve.getTangentAt(t2);
      var nx2 = -tan2.z, nz2 = tan2.x;
      // 左路肩外缘 + 右路肩外缘
      sPos.push(p2.x - nx2 * shldHalf, 0.02, p2.z - nz2 * shldHalf);
      sPos.push(p2.x + nx2 * shldHalf, 0.02, p2.z + nz2 * shldHalf);
      if (si2 < nSeg) {
        var b2 = si2 * 2;
        sIdx.push(b2, b2 + 1, b2 + 2);
        sIdx.push(b2 + 1, b2 + 3, b2 + 2);
      }
    }
    var shldGeo = new THREE.BufferGeometry();
    shldGeo.setAttribute('position', new THREE.Float32BufferAttribute(sPos, 3));
    shldGeo.setIndex(sIdx);
    shldGeo.computeVertexNormals();
    var shldMesh = new THREE.Mesh(shldGeo,
      new THREE.MeshStandardMaterial({ color: 0x4a4d50, roughness: 0.9, metalness: 0.05 }));
    shldMesh.receiveShadow = true;
    group.add(shldMesh);

    // ── 车道线：用 ribbon mesh（非 THREE.Line）保证 3D 可见宽度 ──
    // 中心双黄线（实线，宽 0.15m，间距 0.3m）
    _addLaneMarkRibbon(group, curve, nSeg, -0.15, 0.15, 0xffcc44, 0.043);
    _addLaneMarkRibbon(group, curve, nSeg,  0.15, 0.15, 0xffcc44, 0.043);
    // 车道分隔虚线（每条车道线，宽 0.12m）
    for (var li = 1; li < lanes; li++) {
      var offset = -roadHalf + li * laneWidth;
      _addLaneMarkRibbon(group, curve, nSeg, offset, 0.12, 0xffffff, 0.043, true);
    }
    // 道路边缘白实线（宽 0.15m）
    _addLaneMarkRibbon(group, curve, nSeg,  roadHalf - 0.06, 0.15, 0xffffff, 0.043);
    _addLaneMarkRibbon(group, curve, nSeg, -roadHalf + 0.06, 0.15, 0xffffff, 0.043);
  }

  scene3d.add(group);
  _roadNetworkGroup = group;

  // 隐藏旧的静态道路
  if (_roadGroup) _roadGroup.visible = false;
}

/**
 * 沿曲线创建车道标线 ribbon mesh（替代 THREE.Line，后者线宽恒为 1px）。
 *
 * 与 road surface ribbon 同技术：等距采样 → 左右顶点 → triangle strip。
 * 宽度 0.12–0.15m，在追逐摄像机距离下清晰可见。
 *
 * @param {THREE.Group} group  父组
 * @param {THREE.CatmullRomCurve3} curve  道路曲线
 * @param {number} nSeg  采样段数
 * @param {number} lateralOffset  横向偏移（相对道路中心线，Z 方向，标线中心）
 * @param {number} width  标线宽度（m）
 * @param {number} color  颜色
 * @param {number} y  Y 高度
 * @param {boolean} dashed  true=虚线（4.5m dash + 5.5m gap）
 */
function _addLaneMarkRibbon(group, curve, nSeg, lateralOffset, width, color, y, dashed) {
  var halfW = width / 2;
  var length = curve.getLength();

  if (dashed) {
    // 虚线：每段 dash 独立 ribbon，不连成整体
    var DASH = 4.5, GAP = 5.5, STEP = DASH + GAP;
    var nDash = Math.floor(length / STEP);
    for (var d = 0; d <= nDash; d++) {
      var s0 = d * STEP;
      var s1 = s0 + DASH;
      if (s1 > length) s1 = length;
      if (s0 >= length) break;
      var dashLen = s1 - s0;
      var dSeg = Math.max(2, Math.ceil(dashLen / 1.5));
      _emitRibbonSegment(group, curve, s0, s1, dSeg, length, lateralOffset, halfW, color, y);
    }
  } else {
    _emitRibbonSegment(group, curve, 0, length, nSeg, length, lateralOffset, halfW, color, y);
  }
}

/** 沿曲线 [s0, s1] 段生成一条 ribbon 并加入 group */
function _emitRibbonSegment(group, curve, s0, s1, nSeg, totalLen, lateralOffset, halfW, color, y) {
  var positions = [];
  var indices = [];
  var m = Math.max(2, nSeg);
  for (var si = 0; si <= m; si++) {
    var t = (s0 + (s1 - s0) * si / m) / totalLen;
    if (t > 1) t = 1;
    var pos = curve.getPointAt(t);
    var tan = curve.getTangentAt(t);
    var nx = -tan.z, nz = tan.x;
    // 左右边缘：沿法线偏移
    positions.push(pos.x + nx * (lateralOffset - halfW), y, pos.z + nz * (lateralOffset - halfW));
    positions.push(pos.x + nx * (lateralOffset + halfW), y, pos.z + nz * (lateralOffset + halfW));
    if (si < m) {
      var base = si * 2;
      indices.push(base, base + 1, base + 2);
      indices.push(base + 1, base + 3, base + 2);
    }
  }
  var geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  var mat = new THREE.MeshStandardMaterial({ color: color, roughness: 0.4, emissive: color, emissiveIntensity: 0.25 });
  var mesh = new THREE.Mesh(geo, mat);
  mesh.receiveShadow = true;
  group.add(mesh);
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
  // Bloom 后处理 composer 需同步尺寸，否则 resize 后渲染分辨率与 canvas 不匹配，
  // 导致 Bloom pass 按旧尺寸绘制被缩放、画面糊或边缘错位。
  if (_composer) _composer.setSize(w, h);
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
  scene3d.background = new THREE.Color(0x5588aa);
  scene3d.fog = new THREE.Fog(0x5588aa, 120, 500);

  // Camera — chase cam. Wider FOV keeps the whole car visible on small screens.
  camera3d = new THREE.PerspectiveCamera(60, w / h, 0.2, 500);

  // Renderer
  renderer3d = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer3d.setSize(w, h);
  renderer3d.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer3d.toneMapping = THREE.ACESFilmicToneMapping;
  renderer3d.toneMappingExposure = 1.1;
  // 开启阴影渲染：物体 castShadow/receiveShadow 才会真正产生阴影，
  // 之前 sun.castShadow=true 但 renderer 未启用 shadowMap，阴影被静默丢弃。
  renderer3d.shadowMap.enabled = true;
  renderer3d.shadowMap.type = THREE.PCFSoftShadowMap;
  el.appendChild(renderer3d.domElement);
  document.getElementById("scene3d-msg").style.display = "none";

  // ── 程序化环境贴图：PMREMGenerator 从自建简单场景生成 ──
  // MeshStandardMaterial 需 envMap 才能让 metalness/roughness 产生反射。
  // RoomEnvironment 在 examples（核心库无），这里自建几个发光面片模拟
  // 环境光照，PMREM 预滤波后作为 scene.environment，车漆立刻有反射质感。
  try {
    var pmrem = new THREE.PMREMGenerator(renderer3d);
    var envScene = new THREE.Scene();
    // 模拟天窗/侧窗的发光面片，给金属表面提供反射参考
    var envFaces = [
      { pos: [0, 20, 0], col: 0xffffff, opa: 1.0 },   // 顶光（天窗）
      { pos: [20, 5, 0], col: 0x88aaff, opa: 0.6 },   // 右侧冷光
      { pos: [-20, 5, 0], col: 0xffddaa, opa: 0.5 },  // 左侧暖光
      { pos: [0, 5, 20], col: 0xaaccff, opa: 0.4 },   // 前方天光
      { pos: [0, 5, -20], col: 0x556677, opa: 0.3 }   // 后方阴影
    ];
    for (var fi = 0; fi < envFaces.length; fi++) {
      var ef = envFaces[fi];
      var em = new THREE.Mesh(
        new THREE.PlaneGeometry(30, 30),
        new THREE.MeshBasicMaterial({ color: ef.col, transparent: ef.opa < 1, opacity: ef.opa, side: THREE.DoubleSide })
      );
      em.position.set(ef.pos[0], ef.pos[1], ef.pos[2]);
      em.lookAt(0, 0, 0);
      envScene.add(em);
    }
    var envTex = pmrem.fromScene(envScene, 0.04).texture;
    scene3d.environment = envTex;
    pmrem.dispose();
  } catch (envErr) {
    console.warn('[scene3d] PMREM environment setup failed, falling back to no env:', envErr);
  }

  // WebGL context loss (GPU reset, tab backgrounded, driver hiccup) must not
  // crash the render loop. Suspend cleanly on loss, rebuild on restore; fall
  // back to the 2D canvas while the context is gone.
  renderer3d.domElement.addEventListener("webglcontextlost", function(ev) {
    ev.preventDefault();               // required so 'restored' can fire later
    reportDiag('scene3d', 'WebGL context lost — using 2D fallback');
    _glLost = true;
    try { init2DFallback(); } catch (_) { }
  }, false);
  renderer3d.domElement.addEventListener("webglcontextrestored", function() {
    _glLost = false;
    safeCall('scene3d.restore', function() { init3DScene(); });
  }, false);

  // Lighting — brighter scene so lane markings are clearly visible
  // 环境贴图已提供基础反射，降低 ambient 避免过曝，让阴影更立体
  scene3d.add(new THREE.AmbientLight(0xaabbdd, 0.45));
  var sun = new THREE.DirectionalLight(0xfff8ee, 1.8);
  sun.position.set(30, 40, 15);
  sun.castShadow = true;
  // 阴影相机参数：默认 frustum 太大导致阴影分辨率低（糊），收窄到跟随区域
  sun.shadow.mapSize.width = 2048;
  sun.shadow.mapSize.height = 2048;
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 120;
  sun.shadow.camera.left = -40;
  sun.shadow.camera.right = 40;
  sun.shadow.camera.top = 40;
  sun.shadow.camera.bottom = -40;
  sun.shadow.bias = -0.0005;  // 消除 peter-panning
  scene3d.add(sun);
  _sunLight = sun;  /* 供 _renderFrame 跟随 ego 更新 */
  // sun 目标点跟随 ego（在 _renderFrame 里更新 sun.target.position）
  scene3d.add(sun.target);
  // Hemisphere (sky/ground ambient)
  scene3d.add(new THREE.HemisphereLight(0xaabbdd, 0x554433, 0.4));

  // ── Ground (large flat plane, matches road length) ──
  var gndGeo = new THREE.PlaneGeometry(800, 400);
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

  // ── Ego vehicle (glTF with programmatic fallback) ──
  initModelCache().then(function() {
    if (_carGroup) scene3d.remove(_carGroup);
    var ec = buildEgoCar(0x4488dd);
    if (ec) {
      ec.position.set(0, 0, 0);
      ec.castShadow = true;
      scene3d.add(ec);
      _carGroup = ec;
    }
  });
  var egoCar = buildEgoCar(0x4488dd) || _buildSedan(0x4488dd, 0x3377bb);
  egoCar.position.set(0, 0, 0);
  egoCar.castShadow = true;
  scene3d.add(egoCar);
  _carGroup = egoCar;

  // ── Obstacle pool (glTF with programmatic fallback, 24 slots) ──
  // NOA Phase 2.3: 池容量 8 → 24，匹配 NOA 24-NPC 场景。
  // 优先使用 glTF 模型（PBR 材质），GLTFLoader 不可用时降级为 BoxGeometry。
  _obsPool = [];
  _obsLabelPool = [];
  _obsLabelLast = [];
  for (var oi = 0; oi < 24; oi++) {
    var obs = buildObstacleGroup('car', 0xff9944) || _buildObstacle('car', 0xff9944);
    obs.userData.obsType = 'car';  /* track current mesh type for rebuild-on-change */
    obs.visible = false;
    scene3d.add(obs);
    _obsPool.push(obs);
    // NOA Phase 5: 配套的 AI 状态标签 sprite
    var lbl = _makeLabelSprite();
    scene3d.add(lbl);
    _obsLabelPool.push(lbl);
    _obsLabelLast.push(null);
  }

  // ── Traffic light pool ──
  // Pre-allocate up to 4 traffic lights (matches SCENARIO_MAX_TRAFFIC_LIGHTS).
  // Each is a fixed-size mesh (pole+arm+housing+3 lamps); renderer toggles
  // lamp emissive based on state {green,yellow,red} from scene data.
  _trafficLightPool = [];
  _trafficLightWorld = [];
  for (var ti = 0; ti < 4; ti++) {
    var tl = _buildTrafficLight();
    tl.visible = false;
    scene3d.add(tl);
    _trafficLightPool.push(tl);
    _trafficLightWorld.push(null);
  }

  // ── Phase 3: ETC gate pool (高速 ETC 抬杆门架) ──
  // 每个门架 = 2 立柱 + 横梁 + 可旋转抬杆。progress ∈ [0,1] 控制抬杆角度。
  _etcGatePool = [];
  _etcGateWorld = [];
  for (var gi = 0; gi < 4; gi++) {
    var gate = _buildETCGate();
    gate.visible = false;
    scene3d.add(gate);
    _etcGatePool.push(gate);
    _etcGateWorld.push(null);
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
  _cam = new THREE.Vector3(-15, 5.0, 0);
  _camLook = new THREE.Vector3(8, 0.5, 0);
  _camTarget = new THREE.Vector3(-15, 5.0, 0);
  _camLookTarget = new THREE.Vector3(8, 0.5, 0);

  // ── Initialize dead reckoning state ──
  initDeadReckon();

  // ── Pre-allocate temp vectors (after THREE is loaded) ──
  _tmpV3 = new THREE.Vector3();
  _tmpScale = new THREE.Vector3(1, 1, 1);

  // ── Bloom 后处理：EffectComposer + UnrealBloomPass ──
  // 依赖 examples/js 后处理脚本（index.html CDN 引入）。加载失败或版本不匹配
  // 时 window._bloomUnavailable=true，降级为 renderer 直接渲染。
  _composer = null;
  if (!window._bloomUnavailable && THREE.EffectComposer && THREE.RenderPass &&
      THREE.UnrealBloomPass && THREE.ShaderPass) {
    try {
      _composer = new THREE.EffectComposer(renderer3d);
      _composer.addPass(new THREE.RenderPass(scene3d, camera3d));
      var bloomPass = new THREE.UnrealBloomPass(
        new THREE.Vector2(w, h),
        0.6,   // strength：车灯/反光的光晕强度
        0.4,   // radius：光晕扩散半径
        0.85   // threshold：只让高亮区域（车灯/emissive）发光，避免整屏泛白
      );
      _composer.addPass(bloomPass);
      var copyPass = new THREE.ShaderPass(THREE.CopyShader);
      copyPass.renderToScreen = true;
      _composer.addPass(copyPass);
    } catch (bloomErr) {
      console.warn('[scene3d] Bloom setup failed, falling back to direct render:', bloomErr);
      _composer = null;
    }
  } else if (!window._bloomUnavailable) {
    // 脚本可能异步未加载完，但通常同步 script 标签已执行；兜底降级
    window._bloomUnavailable = true;
  }

  // ── Animation loop ──
  function anim3D() {
    requestAnimationFrame(anim3D);
    if (!sceneReady) return;
    if (_glLost) return;               // skip while WebGL context is lost
    if (!renderer3d || !scene3d || !camera3d) return;
    safeCall('scene3d.frame', function() {
      _renderFrame();
      if (_composer) {
        _composer.render();
      } else {
        renderer3d.render(scene3d, camera3d);
      }
    });
  }
  anim3D();
  sceneReady = true;
  // Phase 4.9: scene3d / camera3d / renderer3d stay module-scoped;
  // debug access flows through app.js -> window.flowboard._scene3d etc.
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

  // ── Dead reckoning: advance the central smoothing engine ──
  // tickDeadReckon() performs speed-based extrapolation +
  // frame-rate-independent exponential lerp + heading wrap-around.
  // The renderer only reads the smoothed result below.
  tickDeadReckon();
  var sx = _dr.smoothX, sz = _dr.smoothZ;

  // ── Ego car: world-space position (road is STATIC at origin) ──
  if (ego) {
    ego.position.set(sx, ego.position.y, sz);
    ego.rotation.y = -_dr.smoothHeading;
    // 前轮转向动画：从 vehicle.steer 读取转向值旋转前轮组
    var v = (_topoData.metrics || {}).vehicle || {};
    var fw = ego.userData.frontWheels;
    if (fw && typeof v.steer === 'number') {
      fw.rotation.y = v.steer * 0.5;
    }
  }

  // Keep ground + environment near ego (road segments are at fixed world X).
  var chunkX = Math.round(sx / 200) * 200;

  // ── 阴影光源跟随 ego：sun 相对 ego 保持固定偏移，阴影 frustum 始终覆盖 ego 周围 ──
  if (_sunLight) {
    _sunLight.position.set(sx + 30, 40, 15);
    _sunLight.target.position.set(sx, 0, 0);
    _sunLight.target.updateMatrixWorld();
  }
  if (_groundMesh && Math.abs(_groundMesh.position.x - chunkX) > 100) _groundMesh.position.x = chunkX;
  if (_envGroup && Math.abs(_envGroup.position.x - chunkX) > 100) _envGroup.position.x = chunkX;

  // ── Chase camera: behind ego, balanced pitch ──
  // lookX=8m ahead keeps the road visible stretching into distance
  // while minimising empty sky above.
  var dc = _debugCam;
  var narrow = (renderer3d && renderer3d.domElement && renderer3d.domElement.clientWidth < 700);
  var back   = dc ? dc.back   : (narrow ? 20 : 15);
  var height = dc ? dc.height : (narrow ? 6.5 : 5.0);
  var side   = dc ? (dc.side || 0) : 0;
  var camLerp= dc ? dc.lerp   : 0.08;
  _camTarget.set(sx - back, height, sz + side);
  _camLookTarget.set(sx + (narrow ? 10 : 8), 0.5, sz);
  _cam.lerp(_camTarget, camLerp);
  _camLook.lerp(_camLookTarget, camLerp);
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
      var lbl = _obsLabelPool[oi];
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
        if (distToEgo < L * 0.5 + EGO_HALF_LEN) {
          om.visible = false;
          if (lbl) lbl.visible = false;
          continue;
        }
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
        // Defect 5: 障碍物类型变化时重建外形（轿车 ↔ 胶囊行人 ↔ 圆锥路障），
        // 复用同一 group 槽位，只替换 children，保留 position/scale/visible。
        if (om.userData.obsType !== ow.type) {
          var nm = buildObstacleGroup(ow.type, c) || _buildObstacle(ow.type, c);
          while (om.children.length) om.remove(om.children[0]);
          while (nm.children.length) om.add(nm.children[0]);
          om.userData.obsType = ow.type;
        }
        if (om.children.length > 0 && om.children[0].material && om.children[0].material.color) {
          om.children[0].material.color.setHex(c);
        }
        om.visible = true;

        // NOA Phase 5: NPC AI 状态标签（仅车辆类型显示，行人不显示）。
        // 标签浮在车顶上方 2.2m，ai 字符串变化时才重建 CanvasTexture。
        if (lbl) {
          var isVehicle = (ow.type === 'car' || ow.type === 'suv' || ow.type === 'truck');
          if (isVehicle && ow.ai) {
            // 标签位置：障碍物位置 + 车顶上方
            lbl.position.set(om.position.x, H + 1.2, om.position.z);
            // 仅当 ai 状态变化时重建纹理
            if (_obsLabelLast[oi] !== ow.ai) {
              var lblColor = _aiLabelColors[ow.ai] || 0xffffff;
              _setLabelSprite(lbl, ow.ai, lblColor);
              _obsLabelLast[oi] = ow.ai;
            }
            lbl.visible = true;
          } else {
            lbl.visible = false;
            _obsLabelLast[oi] = null;
          }
        }
      } else {
        om.visible = false;
        if (lbl) { lbl.visible = false; _obsLabelLast[oi] = null; }
      }
    }
  }

  // ── Traffic lights: position from stored world coords, toggle lamp emissive ──
  // state: 'green'→lamp[2] bright, 'yellow'→lamp[1] bright, 'red'→lamp[0] bright.
  // Inactive lamps are dim (emissiveIntensity 0.05), active lamp bright (1.2).
  if (_trafficLightPool && _trafficLightWorld) {
    for (var tli2 = 0; tli2 < _trafficLightPool.length; tli2++) {
      var tlm = _trafficLightPool[tli2];
      var tlw = _trafficLightWorld[tli2];
      if (tlw) {
        tlm.position.set(tlw.x, 0, tlw.z);
        tlm.visible = true;
        // Map state → active lamp index: red=0, yellow=1, green=2
        var activeIdx = 2;  // default green
        if (tlw.state === 'red') activeIdx = 0;
        else if (tlw.state === 'yellow') activeIdx = 1;
        var lamps = tlm.userData.lamps;
        if (lamps) {
          for (var li2 = 0; li2 < lamps.length; li2++) {
            if (lamps[li2] && lamps[li2].material) {
              lamps[li2].material.emissiveIntensity = (li2 === activeIdx) ? 1.2 : 0.05;
            }
          }
        }
      } else {
        tlm.visible = false;
      }
    }
  }

  // ── Phase 3: ETC gates — 抬杆动画 ──
  // state: 'closed' → 抬杆水平 (progress=0)
  //        'opening' → 抬杆旋转中 (progress 0→1)
  //        'open'    → 抬杆全开 (progress=1, 旋转 ~75°)
  if (_etcGatePool && _etcGateWorld) {
    for (var gi2 = 0; gi2 < _etcGatePool.length; gi2++) {
      var gm = _etcGatePool[gi2];
      var gw = _etcGateWorld[gi2];
      if (gw) {
        gm.position.set(gw.x, 0, gw.z);
        gm.visible = true;
        // 抬杆角度：progress 0→1 映射到 0→75° (1.31 rad)
        var boom = gm.userData.boom;
        if (boom) {
          var targetAngle = (gw.progress || 0) * 1.31;
          boom.rotation.y += (targetAngle - boom.rotation.y) * 0.15;
        }
      } else {
        gm.visible = false;
      }
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

  // ── Car bounce ── (v 已在上方 ego 段声明)
  if (ego && v.speed > 0.5) {
    ego.position.y = 0.05 + Math.sin(_animT * 6.5) * 0.008 * Math.min(1, v.speed * 0.12);
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// update3D — called from updateAll() on every SSE data tick.
// World-anchors obstacles & LiDAR using _dr.lastX/Z (fed by sync2DTarget).
// ══════════════════════════════════════════════════════════════════════════════

function update3D() {
  if (!sceneReady) return;
  var scn = (_topoData.metrics || {}).scene;
  var now = performance.now() / 1000;

  // Phase 3: 多段道路网络渲染 — 优先使用 scene/frame 的 road_network
  // （flowsim_node 发布，monitor_node 透传）。无 road_network 时 fallback
  // 到旧的 scene.road 单段弯道几何（sim_world_node 向后兼容）。
  if (scn && scn.road_network && scn.road_network.edges) {
    _buildRoadNetwork(scn.road_network.edges);
  } else if (scn && scn.road) {
    // 旧路径：单段弯道，隐藏 road_network 如果存在（场景切换时）
    if (_roadNetworkGroup) { _roadNetworkGroup.visible = false; _lastRoadNetworkKey = ''; }
    if (_roadGroup) _roadGroup.visible = true;
    _applyRoadCurve(scn.road);
  }

  // Phase 3: ETC 门架 — 从 scene/frame entities 读取（如果有）
  if (_etcGatePool && scn && scn.entities) {
    var ents = scn.entities;
    var gateIdx = 0;
    for (var ei = 0; ei < ents.length && gateIdx < _etcGatePool.length; ei++) {
      if (ents[ei].type === 'etc_gate') {
        var eg = ents[ei];
        // entities 已是世界坐标（flowsim_node 发布），无需叠加 ego 偏移。
        _etcGateWorld[gateIdx] = {
          x: (eg.x || 0), z: (eg.y || 0),
          state: eg.state || 'closed', progress: eg.progress || 0
        };
        gateIdx++;
      }
    }
    for (; gateIdx < _etcGatePool.length; gateIdx++) _etcGateWorld[gateIdx] = null;
  }

  // Ego ground-truth is fed into the dead-reckoning engine by app.js
  // sync2DTarget() (called just before update3D in updateAll). Here we
  // only read _dr.lastX / lastZ to world-anchor obstacles & LiDAR.

  // NOA Phase 2.3: 优先消费 scn.entities（flowsim_node 发布，世界坐标，
  // 含全部 24 NPC + 行人）。无 entities 时 fallback 到 scn.obstacles
  // （vehicle/state，ego-relative，前 16 个）。
  //
  // entities 字段映射 (scene_pub.cpp)：
  //   car/suv/truck: {type,id,x,y,h,spd,len,wid,ai,vx,vy}
  //   pedestrian:    {type,id,x,y,spd,vx,vy,parked}
  //   ego/tl/etc_gate/stop_line 跳过（由专门 pool 处理或不渲染）。
  if (_obsPool && scn) {
    var usedSlots = 0;
    if (scn.entities && scn.entities.length) {
      var ents2 = scn.entities;
      for (var ei2 = 0; ei2 < ents2.length && usedSlots < _obsPool.length; ei2++) {
        var ent = ents2[ei2];
        if (!ent || !ent.type) continue;
        // 跳过非障碍物实体：ego / 红绿灯 / ETC 门架 / 停止线由专门渲染路径处理。
        if (ent.type === 'ego' || ent.type === 'tl' ||
            ent.type === 'etc_gate' || ent.type === 'stop_line') continue;
        // entities 已是世界坐标，直接用 ent.x/ent.y 作为世界 (x, z)。
        _obsWorld[usedSlots] = {
          x: (ent.x || 0), z: (ent.y || 0),
          vx: (ent.vx || 0), vz: (ent.vy || 0),
          t0: now,
          len: ent.len || 4, wid: ent.wid || 2,
          type: ent.type,
          ai: ent.ai || null,   // Phase 5 NPC AI 状态标签用
          heading: ent.h || 0
        };
        _obsPool[usedSlots].visible = true;
        usedSlots++;
      }
    } else if (scn.obstacles) {
      // Fallback: vehicle/state obstacles（ego-relative，最多 16 个）。
      var obs = scn.obstacles;
      for (var oi = 0; oi < _obsPool.length && oi < obs.length; oi++) {
        var o = obs[oi];
        var wx = _dr.lastX + (o.x || 0), wz = _dr.lastZ + (o.y || 0);
        _obsWorld[oi] = { x: wx, z: wz, vx: (o.vx || 0), vz: (o.vy || 0),
          t0: now, len: o.len || 4, wid: o.wid || 2, type: o.type,
          ai: null, heading: 0 };
        _obsPool[oi].visible = true;
        usedSlots++;
      }
    }
    // 清空未使用的槽位
    for (var ci = usedSlots; ci < _obsPool.length; ci++) {
      _obsPool[ci].visible = false;
      _obsWorld[ci] = null;
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

  // Traffic lights: read state from scene data (published by sim_world via
  // monitor's scene.traffic_lights). Each entry: {id, x, y_lane, state, remain_s}.
  // Convert ego-relative x → world x (same anchoring as obstacles).
  if (_trafficLightPool && scn && scn.traffic_lights) {
    var lights = scn.traffic_lights;
    for (var tli = 0; tli < _trafficLightPool.length; tli++) {
      if (tli < lights.length) {
        var tl = lights[tli];
        var wlx = _dr.lastX + (tl.x || 0);
        var wlz = _dr.lastZ + (tl.y_lane || 0);
        _trafficLightWorld[tli] = { x: wlx, z: wlz, state: tl.state || 'green' };
      } else {
        _trafficLightWorld[tli] = null;
      }
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// Exports
// ══════════════════════════════════════════════════════════════════════════════

export { init3DScene, resize3D, update3D, sceneReady, scene3d, _renderFrame, _applyRoadCurve };
