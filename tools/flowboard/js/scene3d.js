/* global THREE, window, topoData */

// ══════════════════════════════════════════════════════════════════════════════
// scene3d.js — Three.js 3D scene module for ADAS visualization
// ══════════════════════════════════════════════════════════════════════════════

import { safeCall, reportDiag, _makeBox, _makeRect, _buildSedan, _buildObstacle, _buildTrafficLight, _buildETCGate } from './utils.js';
import { initDeadReckon, tickDeadReckon, _dr } from './deadreckon.js';
import { init2DFallback } from './scene2d.js';
import { initModelCache, buildEgoCar, buildObstacleGroup, _setVehicleLights } from './models.js';

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
/** 当前道路网络的曲线数组，供交通灯/ETC门架/轨迹线查询最近切线方向。 */
let _roadCurves = [];
/** 每条 edge 的长度（与 _roadCurves 一一对应），供轨迹跨 edge 投影按 s 累计切换。 */
let _roadCurveLens = [];
/** edge 邻接表：_roadCurveNext[i] = 下一连接 edge 的索引（按首尾端点重合判定），无则 -1。
 *  规划轨迹 s 超出当前 edge 剩余长度时，按此跳到下一条 edge 继续投影，避免弯道末端 clamp 堆积。 */
let _roadCurveNext = [];

/** NOA Phase 6: 规划轨迹线（planning/trajectory 的 path 数组渲染）。
 *  从 ego 当前位置出发，沿 heading 方向延伸 s、横向偏移 d 近似转世界坐标。
 *  用 Line2 风格的粗线（TubeGeometry 或 fat line）保证 3D 可见。这里用
 *  BufferGeometry + LineBasicMaterial（简单可靠，后续可升级 fat line）。 */
let _trajLine = null;
let _trajLastKey = "";

/** Phase 3: ETC gate pool (抬杆门架，从 scene/frame entities 读取) */
let _etcGatePool = [], _etcGateWorld = [];

/** 动态胎痕 trail：ego 后轮在世界坐标留下的渐隐深色带。
 *  用固定容量 ring buffer + BufferGeometry triangle strip，每帧重写顶点。
 *  vertexColors 从深黑渐变到沥青色，模拟胎痕衰减。 */
let _tireTrailMesh = null;
let _trailSamples = [];               // [{lx,lz,rx,rz}, ...]， newest 在前
const _TRAIL_MAX = 48;                // 最多 48 段采样
const _TRAIL_GAP = 1.2;               // 采样最小间距（m）
let _trailLastX = 9999, _trailLastZ = 9999;

/** 远处城市天际线：低多边形建筑剪影环，始终跟随 ego 居中，
 *  配合 fog 产生大气透视效果。 */
let _skylineGroup = null;

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

  // Centre double yellow lines (solid, not dashed — highway standard)
  var clGeo = new THREE.BoxGeometry(ROAD_LEN, 0.02, 0.15, 1, 1, 1);
  var clMat = new THREE.MeshBasicMaterial({ color: 0xffcc44 });
  for (var dy = -1; dy <= 1; dy += 2) {
    var cl = new THREE.Mesh(clGeo, clMat);
    cl.position.set(0, 0.043, dy * 0.15);
    cl.userData.isLaneMark = true;
    roadGroup.add(cl);
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

/** 程序化沥青纹理：深灰底 + 多层噪点 + 随机补丁/裂缝，模拟真实路面。 */
function _makeAsphaltTexture() {
  var canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 512;
  var ctx = canvas.getContext('2d');
  // 基底：不均匀的深灰
  ctx.fillStyle = '#26282c'; ctx.fillRect(0, 0, 512, 512);
  // 大色块补丁（模拟沥青压实不均）
  for (var p = 0; p < 40; p++) {
    var px = Math.random() * 512, py = Math.random() * 512;
    var pr = 30 + Math.random() * 80;
    var grd = ctx.createRadialGradient(px, py, 0, px, py, pr);
    var base = 30 + Math.floor(Math.random() * 25);
    grd.addColorStop(0, 'rgba(' + base + ',' + (base + 2) + ',' + (base + 4) + ',0.35)');
    grd.addColorStop(1, 'rgba(' + base + ',' + (base + 2) + ',' + (base + 4) + ',0)');
    ctx.fillStyle = grd;
    ctx.beginPath(); ctx.arc(px, py, pr, 0, Math.PI * 2); ctx.fill();
  }
  // 深色颗粒
  for (var i = 0; i < 6000; i++) {
    var x = Math.random() * 512, y = Math.random() * 512;
    var shade = Math.floor(Math.random() * 35);
    ctx.fillStyle = 'rgba(' + (26 + shade) + ',' + (28 + shade) + ',' + (32 + shade) + ',0.55)';
    ctx.fillRect(x, y, 2, 2);
  }
  // 浅色颗粒/反光碎石
  for (var j = 0; j < 2500; j++) {
    var x2 = Math.random() * 512, y2 = Math.random() * 512;
    var sh2 = Math.floor(Math.random() * 45);
    ctx.fillStyle = 'rgba(' + (70 + sh2) + ',' + (74 + sh2) + ',' + (80 + sh2) + ',0.3)';
    ctx.fillRect(x2, y2, 2, 2);
  }
  // 随机细裂缝
  ctx.strokeStyle = 'rgba(10,10,12,0.35)';
  ctx.lineWidth = 1;
  for (var k = 0; k < 12; k++) {
    ctx.beginPath();
    var cx = Math.random() * 512, cy = Math.random() * 512;
    ctx.moveTo(cx, cy);
    for (var s = 0; s < 4; s++) {
      cx += (Math.random() - 0.5) * 80; cy += (Math.random() - 0.5) * 80;
      ctx.lineTo(cx, cy);
    }
    ctx.stroke();
  }
  var tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = THREE.RepeatWrapping; tex.wrapT = THREE.RepeatWrapping;
  return tex;
}

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
  _roadCurves = [];
  _roadCurveLens = [];
  _roadCurveNext = [];

  var group = new THREE.Group();
  var SEG_LEN = 3;  // 3m 一段，平衡精度与性能

  // NOA Phase 6: 收集每条 edge 的首尾世界坐标，渲染后扫描分叉/汇入点。
  // 两条 edge 的首/尾节点重合（距离 < 1m）即判定为 junction 连接点。
  var edgeEnds = [];  // [{start:{x,z}, end:{x,z}, lanes, length}, ...]

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
    _roadCurves.push(curve);

    var length = edge.length || curve.getLength();
    _roadCurveLens.push(length);
    var lanes = edge.lanes || 2;
    var laneWidth = edge.lane_width || 3.5;
    /* 道路关于参考线对称：总宽度 = 车道数 × 单车道宽。
     * 参考线两侧均有车道（ OpenDRIVE lane 0 居中作为分隔基准）。
     * 旧实现把全部车道放在参考线一侧，导致 ego/车辆经常"悬空"在道路外。 */
    var roadWidth = lanes * laneWidth;
    var halfWidth = roadWidth / 2;

    // 记录首尾坐标供 junction 检测
    edgeEnds.push({
      start: { x: nodes[0][0], z: nodes[0][1] },
      end:   { x: nodes[nodes.length - 1][0], z: nodes[nodes.length - 1][1] },
      lanes: lanes, length: length, curve: curve
    });

    // ── 路面 ribbon mesh ──
    // 左边缘 = +halfWidth，右边缘 = -halfWidth（对称）
    var nSeg = Math.max(4, Math.floor(length / SEG_LEN));
    var positions = [];
    var indices = [];
    for (var si = 0; si <= nSeg; si++) {
      var t = si / nSeg;
      var pos = curve.getPointAt(t);
      var tangent = curve.getTangentAt(t);
      // 法线（切线在 XZ 平面内旋转 90°，指向"右"侧，即 d<0）
      var nx = -tangent.z, nz = tangent.x;

      positions.push(pos.x + nx * halfWidth, 0.01, pos.z + nz * halfWidth);  // 左边缘
      positions.push(pos.x - nx * halfWidth, 0.01, pos.z - nz * halfWidth);  // 右边缘

      if (si < nSeg) {
        var base = si * 2;
        indices.push(base, base + 1, base + 2);
        indices.push(base + 1, base + 3, base + 2);
      }
    }

    // 路面 UV：u 沿道路长度，v 沿横向（左=0，右=1）
    var uvs = [];
    for (var si = 0; si <= nSeg; si++) {
      var u = si / nSeg;
      uvs.push(u, 0);
      uvs.push(u, 1);
    }

    var roadGeo = new THREE.BufferGeometry();
    roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    roadGeo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
    roadGeo.setIndex(indices);
    roadGeo.computeVertexNormals();

    // 沥青路面：使用程序化纹理 + bump 感，强光下仍保持沥青黑灰质感。
    var asphaltTex = _makeAsphaltTexture();
    asphaltTex.repeat.set(length / 4, roadWidth / 4);
    asphaltTex.anisotropy = 8;
    var roadMat = new THREE.MeshStandardMaterial({
      map: asphaltTex,
      color: 0x888888, roughness: 0.92, metalness: 0.0,
      bumpMap: asphaltTex, bumpScale: 0.02
    });
    var roadMesh = new THREE.Mesh(roadGeo, roadMat);
    roadMesh.receiveShadow = true;
    group.add(roadMesh);

    // ── 路肩（路面两侧外扩 1.2m，外侧略低模拟排水横坡）──
    var shldW = 1.2;
    var sPos = [], sIdx = [];
    for (var si2 = 0; si2 <= nSeg; si2++) {
      var t2 = si2 / nSeg;
      var p2 = curve.getPointAt(t2);
      var tan2 = curve.getTangentAt(t2);
      var nx2 = -tan2.z, nz2 = tan2.x;
      // 内侧与路面同高，外侧降低 0.06m
      sPos.push(p2.x + nx2 * halfWidth, 0.015, p2.z + nz2 * halfWidth);
      sPos.push(p2.x + nx2 * (halfWidth + shldW), -0.045, p2.z + nz2 * (halfWidth + shldW));
      sPos.push(p2.x - nx2 * halfWidth, 0.015, p2.z - nz2 * halfWidth);
      sPos.push(p2.x - nx2 * (halfWidth + shldW), -0.045, p2.z - nz2 * (halfWidth + shldW));
      if (si2 < nSeg) {
        var b2 = si2 * 4;
        sIdx.push(b2, b2 + 1, b2 + 4,  b2 + 1, b2 + 5, b2 + 4);
        sIdx.push(b2 + 2, b2 + 6, b2 + 3,  b2 + 3, b2 + 6, b2 + 7);
      }
    }
    var shldGeo = new THREE.BufferGeometry();
    shldGeo.setAttribute('position', new THREE.Float32BufferAttribute(sPos, 3));
    shldGeo.setIndex(sIdx);
    shldGeo.computeVertexNormals();
    var shldTex = _makeAsphaltTexture();
    shldTex.repeat.set(length / 6, 1);
    shldTex.anisotropy = 4;
    var shldMesh = new THREE.Mesh(shldGeo,
      new THREE.MeshStandardMaterial({ map: shldTex, color: 0x999999, roughness: 0.95, metalness: 0.0, bumpMap: shldTex, bumpScale: 0.015 }));
    shldMesh.receiveShadow = true;
    group.add(shldMesh);

    // ── 车道线：中心双黄线 + 多车道时分隔虚线 + 道路边缘白实线 ──
    // 中心双黄线（实线）：只对双向道路（lanes ≥ 2）绘制，单车道为单向路无需中心线
    if (lanes >= 2) {
      _addLaneMarkRibbon(group, curve, nSeg, -0.15, 0.15, 0xffcc44, 0.045);
      _addLaneMarkRibbon(group, curve, nSeg,  0.15, 0.15, 0xffcc44, 0.045);
    }
    // 多车道分隔虚线：每方向 ≥2 车道时才需要内侧分隔线。
    // lanes 是双向总车道数，floor(lanes/2) 是单向车道数，避免 3 车道不对称路误画。
    var perSide = Math.floor(lanes / 2);
    for (var li = 1; li < perSide; li++) {
      var off = li * laneWidth;
      _addLaneMarkRibbon(group, curve, nSeg,  off, 0.12, 0xffffff, 0.045, true);
      _addLaneMarkRibbon(group, curve, nSeg, -off, 0.12, 0xffffff, 0.045, true);
    }
    // 道路边缘白实线
    _addLaneMarkRibbon(group, curve, nSeg,  halfWidth - 0.06, 0.15, 0xffffff, 0.045);
    _addLaneMarkRibbon(group, curve, nSeg, -halfWidth + 0.06, 0.15, 0xffffff, 0.045);

    // ── 路面导向箭头（每条 edge 中段画一个直行箭头）──
    _addRoadArrow(group, curve, length * 0.5, laneWidth, lanes);
    // ── 停止线（起点处，城市/路口道路）──
    if (length < 120) {
      _addStopLine(group, curve, 0, laneWidth, lanes);
    }

    // ── 道路护栏（两侧）──
    // 波形护栏：圆形立柱 + 上下双横梁 + 反光片，比单一方块真实。
    var guardMat = new THREE.MeshStandardMaterial({ color: 0x7799aa, metalness: 0.55, roughness: 0.35 });
    var guardPostGeo = new THREE.CylinderGeometry(0.055, 0.07, 0.9, 12);
    var railGeo = new THREE.BoxGeometry(1, 0.05, 0.08, 1, 1, 1);
    var reflectorMat = new THREE.MeshStandardMaterial({ color: 0xffeebb, emissive: 0xffeebb, emissiveIntensity: 0.35, roughness: 0.4 });
    var guardSpacing = 3.0;
    var guardCount = Math.max(2, Math.floor(length / guardSpacing));
    for (var gi2 = 0; gi2 <= guardCount; gi2++) {
      var gt = gi2 / guardCount;
      var gp = curve.getPointAt(gt);
      var gtan = curve.getTangentAt(gt);
      var gnx = -gtan.z, gnz = gtan.x;
      var gLeft = halfWidth + 0.45, gRight = -(halfWidth + 0.45);
      for (var gside = 0; gside < 2; gside++) {
        var lateral = gside === 0 ? gLeft : gRight;
        var gpP = new THREE.Vector3(gp.x + gnx * lateral, 0.45, gp.z + gnz * lateral);
        var post = new THREE.Mesh(guardPostGeo, guardMat);
        post.position.copy(gpP); post.castShadow = true; group.add(post);
        // 立柱顶部盖帽
        var cap = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.055, 0.04, 12), guardMat);
        cap.position.copy(gpP); cap.position.y += 0.47; group.add(cap);
        // 每隔一根立柱加反光片
        if (gi2 % 2 === 0) {
          var ref = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.18, 0.12), reflectorMat);
          ref.position.copy(gpP); ref.position.y += 0.12; ref.position.x -= gnx * 0.04; ref.position.z -= gnz * 0.04;
          group.add(ref);
        }
      }
      // 立柱间横梁（上下各一根）
      if (gi2 < guardCount) {
        var gt2 = (gi2 + 1) / guardCount;
        var gp2 = curve.getPointAt(gt2);
        var gtan2 = curve.getTangentAt(gt2);
        var gnx2 = -gtan2.z, gnz2 = gtan2.x;
        var railLen = gp.distanceTo(gp2);
        for (var gside = 0; gside < 2; gside++) {
          var lateral = gside === 0 ? gLeft : gRight;
          var gpS = new THREE.Vector3(gp.x + gnx * lateral, 0.62, gp.z + gnz * lateral);
          var gpE = new THREE.Vector3(gp2.x + gnx2 * lateral, 0.62, gp2.z + gnz2 * lateral);
          var railTop = new THREE.Mesh(new THREE.BoxGeometry(railLen, 0.05, 0.07), guardMat);
          railTop.position.copy(gpS).lerp(gpE, 0.5); railTop.lookAt(gpE); group.add(railTop);
          var gpS2 = gpS.clone(); gpS2.y = 0.32;
          var gpE2 = gpE.clone(); gpE2.y = 0.32;
          var railBot = new THREE.Mesh(new THREE.BoxGeometry(railLen, 0.05, 0.07), guardMat);
          railBot.position.copy(gpS2).lerp(gpE2, 0.5); railBot.lookAt(gpE2); group.add(railBot);
        }
      }
    }

    // ── 路灯（两侧交错，每隔 40m）──
    // 静态灯杆 + 顶部微弱 PointLight，增强夜间/隧道氛围；数量有限避免性能问题。
    var poleGeo = new THREE.CylinderGeometry(0.08, 0.12, 6.0, 10);
    var poleMat = new THREE.MeshStandardMaterial({ color: 0x555555, metalness: 0.5, roughness: 0.5 });
    var lampGeo = new THREE.BoxGeometry(0.5, 0.12, 0.25);
    var lampMat = new THREE.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 0.8 });
    var lampSpacing = 40.0;
    var lampCount = Math.max(1, Math.floor(length / lampSpacing));
    for (var li3 = 1; li3 <= lampCount; li3++) {
      var lt = Math.min(1.0, li3 * lampSpacing / length);
      var lp = curve.getPointAt(lt);
      var ltan = curve.getTangentAt(lt);
      var lnx = -ltan.z, lnz = ltan.x;
      var side = (li3 % 2 === 0) ? 1 : -1;  // 交错
      var lateral = side * (halfWidth + 2.5);
      var poleX = lp.x + lnx * lateral, poleZ = lp.z + lnz * lateral;
      var pole = new THREE.Mesh(poleGeo, poleMat);
      pole.position.set(poleX, 3.0, poleZ);
      pole.castShadow = true;
      group.add(pole);
      var lamp = new THREE.Mesh(lampGeo, lampMat);
      lamp.position.set(poleX - lnx * side * 0.8, 5.9, poleZ - lnz * side * 0.8);
      lamp.lookAt(poleX - lnx * side * 3.0, 0, poleZ - lnz * side * 3.0);
      group.add(lamp);
      // 微弱路面照明，距离衰减避免远处过亮
      var plight = new THREE.PointLight(0xffffee, 0.6, 28, 1.8);
      plight.position.set(lamp.position.x, 5.5, lamp.position.z);
      group.add(plight);
    }
  }

  // ── 构建 edge 邻接表：_roadCurveNext[i] = end 最接近某 edge j.start 的 j ──
  // 用于规划轨迹跨 edge 投影：s 超出当前 edge 剩余长度时跳到下一条 edge。
  // 容差与 junction 检测一致（端点重合 < 1.5m）。分叉点可能有多条候选，
  // 取距离最近的一条（主干方向优先）。
  var NEXT_TOL = 1.5;
  for (var ai = 0; ai < edgeEnds.length; ai++) {
    var bestJ = -1, bestD2 = NEXT_TOL * NEXT_TOL;
    for (var aj = 0; aj < edgeEnds.length; aj++) {
      if (aj === ai) continue;
      var ddx = edgeEnds[ai].end.x - edgeEnds[aj].start.x;
      var ddz = edgeEnds[ai].end.z - edgeEnds[aj].start.z;
      var d2 = ddx * ddx + ddz * ddz;
      if (d2 < bestD2) { bestD2 = d2; bestJ = aj; }
    }
    _roadCurveNext.push(bestJ);
  }

  // ── NOA Phase 6: 分叉/汇入点检测与标记 ──────────────────────
  // 一个端点被 ≥3 条 edge 共享时判定为 junction（分叉/汇入点）。
  // 阈值=3：顺序拼接处一个端点恰好被 2 条 edge 共享（前一条 end + 后一条 start），
  // 而 junction 分叉点会被 3 条以上 edge 共享（incoming + 2 条 connecting，或
  // merge 的 connecting + target + incoming）。
  var JUNCTION_TOL = 1.5;  // 端点重合判定容差(m)
  // 收集所有端点，聚类成"连接点"（距离 < TOL 的端点归为同一个连接点）
  var allEnds = [];
  for (var i = 0; i < edgeEnds.length; i++) {
    allEnds.push({ x: edgeEnds[i].start.x, z: edgeEnds[i].start.z, lanes: edgeEnds[i].lanes });
    allEnds.push({ x: edgeEnds[i].end.x,   z: edgeEnds[i].end.z,   lanes: edgeEnds[i].lanes });
  }
  // 聚类：每个端点找最近的已有簇，<TOL 则归入，否则新建簇
  var clusters = [];  // [{x, z, count, minLanes}]
  for (var e = 0; e < allEnds.length; e++) {
    var found = false;
    for (var cl = 0; cl < clusters.length; cl++) {
      var ddx = allEnds[e].x - clusters[cl].x;
      var ddz = allEnds[e].z - clusters[cl].z;
      if (ddx * ddx + ddz * ddz < JUNCTION_TOL * JUNCTION_TOL) {
        clusters[cl].count++;
        if (allEnds[e].lanes < clusters[cl].minLanes) clusters[cl].minLanes = allEnds[e].lanes;
        found = true; break;
      }
    }
    if (!found) clusters.push({ x: allEnds[e].x, z: allEnds[e].z, count: 1, minLanes: allEnds[e].lanes });
  }
  // count >= 3 的簇 = junction 连接点；含单车道（匝道）= fork，否则 = merge
  // 简化标记：只保留贴地圆环 + 方向箭头，去掉高立柱和浮动文字，避免卡通感。
  for (var ci = 0; ci < clusters.length; ci++) {
    if (clusters[ci].count < 3) continue;
    var kind = clusters[ci].minLanes <= 1 ? 'fork' : 'merge';
    var color = kind === 'fork' ? 0xff8800 : 0x44cc44;
    var jx = clusters[ci].x, jz = clusters[ci].z;

    // 贴地圆环
    var ringGeo = new THREE.RingGeometry(2.0, 2.6, 24);
    var ringMat = new THREE.MeshBasicMaterial({ color: color, side: THREE.DoubleSide, transparent: true, opacity: 0.75 });
    var ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(jx, 0.07, jz);
    group.add(ring);

    // 贴地箭头：fork 朝前分叉（Cone 压扁），merge 朝后汇合
    var arrowGeo = new THREE.ConeGeometry(0.6, 1.2, 3);
    var arrowMat = new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.7, side: THREE.DoubleSide });
    var arrow = new THREE.Mesh(arrowGeo, arrowMat);
    arrow.scale.y = 0.25;
    arrow.rotation.x = -Math.PI / 2;
    arrow.rotation.z = kind === 'fork' ? 0 : Math.PI;
    arrow.position.set(jx, 0.08, jz);
    group.add(arrow);
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

/**
 * 根据当前道路网络曲线，查询 (x,z) 处最近的道路切线方向。
 * 返回 { heading: 切线与 +X 轴夹角(rad), found: bool }。
 * 用于交通灯/ETC门架/轨迹线对齐道路方向。
 */
function _getRoadTangentAt(x, z) {
  if (!_roadCurves || !_roadCurves.length) return { found: false, heading: 0 };
  var bestT = null, bestD2 = Infinity;
  for (var ci = 0; ci < _roadCurves.length; ci++) {
    var curve = _roadCurves[ci];
    var pts = curve.getSpacedPoints(20);
    for (var pi = 0; pi < pts.length; pi++) {
      var dx = pts[pi].x - x, dz = pts[pi].z - z;
      var d2 = dx * dx + dz * dz;
      if (d2 < bestD2) { bestD2 = d2; bestT = { curve: curve, t: pi / (pts.length - 1) }; }
    }
  }
  if (!bestT) return { found: false, heading: 0 };
  var tan = bestT.curve.getTangentAt(Math.max(0, Math.min(1, bestT.t)));
  return { found: true, heading: Math.atan2(tan.z, tan.x) };
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
  // 车道线：白线略带粗糙反光，黄线模拟热熔标线微反光。
  var isYellow = (color === 0xffcc44 || color === 0xffaa00);
  var markMat = new THREE.MeshStandardMaterial({
    color: color,
    roughness: isYellow ? 0.55 : 0.45,
    metalness: 0.0,
    emissive: color,
    emissiveIntensity: isYellow ? 0.15 : 0.08
  });
  var mesh = new THREE.Mesh(geo, markMat);
  mesh.receiveShadow = false;
  group.add(mesh);
}

/** 在道路指定 s 位置画一个贴地直行箭头（多个三角形拼接成箭头形状） */
function _addRoadArrow(group, curve, s, laneWidth, lanes) {
  var totalLen = curve.getLength();
  if (s >= totalLen) s = totalLen * 0.5;
  var t = s / totalLen;
  var pos = curve.getPointAt(t);
  var tan = curve.getTangentAt(t);
  var nx = -tan.z, nz = tan.x;
  var y = 0.046;
  var arrowMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.45, emissive: 0xffffff, emissiveIntensity: 0.08 });
  // 每条车道画一个箭头
  var laneCenters = [];
  for (var li = 0; li < lanes; li++) {
    laneCenters.push((li + 0.5 - lanes / 2) * laneWidth);
  }
  for (var ci = 0; ci < laneCenters.length; ci++) {
    var c = laneCenters[ci];
    // 箭头由 3 个菱形/三角形片组成：尾杆 + 两个斜翼
    var shapes = [
      { dx: -0.9, dz: 0, w: 1.0, l: 0.22 }, // 箭杆
      { dx: 0.1, dz: 0.35, w: 0.7, l: 0.22 }, // 左翼
      { dx: 0.1, dz: -0.35, w: 0.7, l: 0.22 } // 右翼
    ];
    for (var si = 0; si < shapes.length; si++) {
      var sh = shapes[si];
      var cx = pos.x + nx * (c + sh.dz) + tan.x * sh.dx;
      var cz = pos.z + nz * (c + sh.dz) + tan.z * sh.dx;
      var arrow = new THREE.Mesh(new THREE.PlaneGeometry(sh.w, sh.l), arrowMat);
      arrow.rotation.x = -Math.PI / 2;
      arrow.rotation.z = -Math.atan2(tan.z, tan.x);
      arrow.position.set(cx, y, cz);
      group.add(arrow);
    }
  }
}

/** 在道路指定 s 位置画一条横跨所有车道的停止线 */
function _addStopLine(group, curve, s, laneWidth, lanes) {
  var totalLen = curve.getLength();
  if (s >= totalLen) s = 0;
  var t = s / totalLen;
  var pos = curve.getPointAt(t);
  var tan = curve.getTangentAt(t);
  var nx = -tan.z, nz = tan.x;
  var halfW = lanes * laneWidth / 2;
  var lineMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.45, emissive: 0xffffff, emissiveIntensity: 0.08 });
  var lineGeo = new THREE.PlaneGeometry(0.35, lanes * laneWidth);
  var line = new THREE.Mesh(lineGeo, lineMat);
  line.rotation.x = -Math.PI / 2;
  line.rotation.z = -Math.atan2(tan.z, tan.x);
  line.position.set(pos.x, 0.047, pos.z);
  group.add(line);
}

// ── Environment: buildings + trees + street props ──
// ══════════════════════════════════════════════════════════════════════════════
// 远处城市天际线 — 低多边形建筑剪影环，~450m 半径，跟随 ego 居中
// 配合 scene.fog(140, 520) 产生大气透视，营造远景城市感
// ══════════════════════════════════════════════════════════════════════════════
function _buildSkyline(scene) {
  var grp = new THREE.Group();
  // 冷蓝灰剪影色，fog 会进一步淡化到背景色
  var skyMat = new THREE.MeshBasicMaterial({ color: 0x4a5868, fog: true });
  var N = 72;
  for (var i = 0; i < N; i++) {
    var ang = (i / N) * Math.PI * 2 + (Math.random() - 0.5) * 0.04;
    var dist = 430 + Math.random() * 90;
    var bx = Math.cos(ang) * dist;
    var bz = Math.sin(ang) * dist;
    var bw = 8 + Math.random() * 22;
    var bd = 8 + Math.random() * 22;
    var bh = 18 + Math.random() * 85;
    var b = new THREE.Mesh(new THREE.BoxGeometry(bw, bh, bd), skyMat);
    b.position.set(bx, bh / 2, bz);
    grp.add(b);
    // 30% 概率叠加一个稍高的塔楼，丰富轮廓
    if (Math.random() > 0.7) {
      var th = bh * (1.1 + Math.random() * 0.5);
      var tower = new THREE.Mesh(new THREE.BoxGeometry(bw * 0.5, th, bd * 0.5), skyMat);
      tower.position.set(bx, th / 2, bz);
      grp.add(tower);
    }
  }
  scene.add(grp);
  _skylineGroup = grp;
}

// ══════════════════════════════════════════════════════════════════════════════
// 动态胎痕 trail — ego 后轮在世界坐标留下的渐隐深色带
// 每帧采样后轮世界位置，维护 ring buffer，重写 BufferGeometry
// ══════════════════════════════════════════════════════════════════════════════
function _initTireTrail(scene) {
  var geo = new THREE.BufferGeometry();
  // (MAX+1) * 2 顶点：每段 2 端点（左轮/右轮），相邻段共享端点
  var n = (_TRAIL_MAX + 1) * 2;
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
  geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
  // triangle strip 索引：每段 2 三角形（6 索引）
  var idx = [];
  for (var i = 0; i < _TRAIL_MAX; i++) {
    idx.push(i * 2, i * 2 + 1, (i + 1) * 2,
             i * 2 + 1, (i + 1) * 2 + 1, (i + 1) * 2);
  }
  geo.setIndex(idx);
  var mat = new THREE.MeshBasicMaterial({
    vertexColors: true, transparent: true, opacity: 0.6,
    depthWrite: false, polygonOffset: true, polygonOffsetFactor: -4
  });
  _tireTrailMesh = new THREE.Mesh(geo, mat);
  _tireTrailMesh.renderOrder = -2;
  _tireTrailMesh.visible = false;
  scene.add(_tireTrailMesh);
}

/** 每帧更新胎痕：采样 ego 后轮世界坐标，推入 ring buffer，重写顶点/颜色。
 *  颜色从深黑（新）渐变到沥青底色（旧），模拟胎痕衰减。 */
function _updateTireTrail(sx, sz, heading, speed) {
  if (!_tireTrailMesh) return;
  // 后轮世界坐标：local (-1.35, 0, ±0.92)，heading 旋转后投影
  // scene3d 中 ego.rotation.y = -heading，故世界旋转 = -heading
  var cosH = Math.cos(-heading), sinH = Math.sin(-heading);
  var rx = -1.35, hw = 0.92;
  var lx = sx + rx * cosH - hw * sinH;
  var lz = sz + rx * sinH + hw * cosH;
  var rwx = sx + rx * cosH + hw * sinH;
  var rwz = sz + rx * sinH - hw * cosH;

  if (speed > 0.5) {
    var dist = Math.hypot(lx - _trailLastX, lz - _trailLastZ);
    if (dist > _TRAIL_GAP || _trailSamples.length === 0) {
      _trailSamples.unshift({ lx: lx, lz: lz, rx: rwx, rz: rwz });
      _trailLastX = lx; _trailLastZ = lz;
      if (_trailSamples.length > _TRAIL_MAX) _trailSamples.pop();
    }
  }

  var n = _trailSamples.length;
  _tireTrailMesh.visible = n > 1;
  if (n < 2) return;

  var posAttr = _tireTrailMesh.geometry.attributes.position.array;
  var colAttr = _tireTrailMesh.geometry.attributes.color.array;
  // 沥青底色（与 _buildRoad 的 0x3a3f4a 接近），胎痕从此色渐变到深黑
  var roadR = 0.227, roadG = 0.247, roadB = 0.290;
  var darkR = 0.02, darkG = 0.02, darkB = 0.025;

  for (var i = 0; i <= n; i++) {
    var s = (i < n) ? _trailSamples[i] : _trailSamples[n - 1];
    // 顶点 2i = 左轮, 2i+1 = 右轮
    posAttr[(i * 2) * 3]     = s.lx; posAttr[(i * 2) * 3 + 1]     = 0.02; posAttr[(i * 2) * 3 + 2]     = s.lz;
    posAttr[(i * 2 + 1) * 3] = s.rx; posAttr[(i * 2 + 1) * 3 + 1] = 0.02; posAttr[(i * 2 + 1) * 3 + 2] = s.rz;
    // 衰减：i=0 最新（最深），i=MAX 最旧（接近沥青色）
    var fade = Math.min(1, i / _TRAIL_MAX);
    var cr = roadR + (darkR - roadR) * (1 - fade);
    var cg = roadG + (darkG - roadG) * (1 - fade);
    var cb = roadB + (darkB - roadB) * (1 - fade);
    colAttr[(i * 2) * 3]     = cr; colAttr[(i * 2) * 3 + 1]     = cg; colAttr[(i * 2) * 3 + 2]     = cb;
    colAttr[(i * 2 + 1) * 3] = cr; colAttr[(i * 2 + 1) * 3 + 1] = cg; colAttr[(i * 2 + 1) * 3 + 2] = cb;
  }
  _tireTrailMesh.geometry.attributes.position.needsUpdate = true;
  _tireTrailMesh.geometry.attributes.color.needsUpdate = true;
}

function _buildEnvironment(scene) {
  var env = new THREE.Group();
  var bldColors = [0x4a5868, 0x566676, 0x3d4d5d, 0x5a6a7a, 0x6b5a5a];
  var windowMat = new THREE.MeshStandardMaterial({ color: 0x88aacc, metalness: 0.6, roughness: 0.15, emissive: 0x223344, emissiveIntensity: 0.25 });

  // Buildings: 主楼 + 窗户网格 + 屋顶/裙楼，避免纯方块
  for (var b = 0; b < 14; b++) {
    var side = (b % 2 === 0) ? 1 : -1;
    var bx = (b - 7) * 180 + Math.random() * 70;
    var bw = 6 + Math.random() * 8;
    var bz = side * (26 + Math.random() * 28);
    var bh = 12 + Math.random() * 42;
    var depth = 6 + Math.random() * 8;
    var bldColor = bldColors[b % bldColors.length];

    // 主楼
    var bld = _makeBox(bw, bh, depth, bldColor, 0x000000, 0.92);
    bld.position.set(bx, bh / 2, bz);
    bld.castShadow = true; bld.receiveShadow = true;
    env.add(bld);

    // 窗户（面向道路的一面）
    var floors = Math.max(3, Math.floor(bh / 3.2));
    var cols = Math.max(2, Math.floor(bw / 2.5));
    var winW = (bw - 1) / cols * 0.55;
    var winH = 1.2;
    var winGeo = new THREE.PlaneGeometry(winW, winH);
    for (var fi = 0; fi < floors; fi++) {
      for (var cj = 0; cj < cols; cj++) {
        var win = new THREE.Mesh(winGeo, windowMat);
        var wx = bx - (bw - 1) / 2 + (cj + 0.5) * ((bw - 1) / cols);
        var wy = 1.8 + fi * 3.2;
        var faceZ = bz + side * (depth / 2 + 0.02);
        win.position.set(wx, wy, faceZ);
        win.rotation.y = side > 0 ? Math.PI : 0;
        env.add(win);
      }
    }

    // 屋顶女儿墙/设备平台
    var parapet = _makeBox(bw * 0.9, 0.8, depth * 0.9, 0x333333, 0x000000, 0.95);
    parapet.position.set(bx, bh + 0.4, bz);
    parapet.castShadow = true; env.add(parapet);

    // 低层裙楼（50%概率）
    if (Math.random() > 0.5) {
      var podiumH = 3 + Math.random() * 3;
      var podium = _makeBox(bw + 2, podiumH, depth + 2, 0x555555, 0x000000, 0.9);
      podium.position.set(bx, podiumH / 2, bz);
      podium.castShadow = true; podium.receiveShadow = true;
      env.add(podium);
    }
  }

  // Trees: 树干 + 多个不规则球冠，更像真实树木
  for (var t = 0; t < 26; t++) {
    var tx = (t - 13) * 120 + Math.random() * 60;
    var tz = (t % 2 === 0 ? 1 : -1) * (17 + Math.random() * 36);
    var th = 2 + Math.random() * 2.5;
    var trunk = _makeCyl(0.18, 0.28, th, 8, 0x6b4423);
    trunk.position.set(tx, th / 2, tz);
    trunk.castShadow = true; env.add(trunk);
    var canopyColor = 0x2d5a27 + Math.floor(Math.random() * 0x202010);
    var leafMat = new THREE.MeshStandardMaterial({ color: canopyColor, roughness: 0.9 });
    var nBlobs = 3 + Math.floor(Math.random() * 3);
    for (var cb = 0; cb < nBlobs; cb++) {
      var r = 1.2 + Math.random() * 1.6;
      var blob = new THREE.Mesh(new THREE.DodecahedronGeometry(r, 0), leafMat);
      blob.position.set(
        tx + (Math.random() - 0.5) * r * 1.2,
        th + r * 0.4 + Math.random() * 1.2,
        tz + (Math.random() - 0.5) * r * 1.2
      );
      blob.scale.set(1, 0.75 + Math.random() * 0.4, 1);
      blob.castShadow = true;
      env.add(blob);
    }
  }

  // Street props: 路灯杆 + 垃圾桶 + 消防栓
  var propMat = new THREE.MeshStandardMaterial({ color: 0x555555, metalness: 0.5, roughness: 0.5 });
  var trashMat = new THREE.MeshStandardMaterial({ color: 0x446644, roughness: 0.7 });
  var hydrantMat = new THREE.MeshStandardMaterial({ color: 0xaa2222, roughness: 0.5 });
  for (var p = 0; p < 16; p++) {
    var px = (p - 8) * 160 + Math.random() * 60;
    var pside = (p % 2 === 0) ? 1 : -1;
    var pz = pside * (10 + Math.random() * 4);
    var poleH = 5.5 + Math.random() * 1.5;
    var pole = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.1, poleH, 10), propMat);
    pole.position.set(px, poleH / 2, pz);
    pole.castShadow = true; env.add(pole);
    var armLen = 1.2;
    var arm = new THREE.Mesh(new THREE.BoxGeometry(armLen, 0.08, 0.08), propMat);
    arm.position.set(px - pside * armLen / 2, poleH - 0.3, pz);
    env.add(arm);
    var lamp = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.1, 0.18),
      new THREE.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 0.9 }));
    lamp.position.set(px - pside * armLen, poleH - 0.3, pz);
    env.add(lamp);

    if (Math.random() > 0.3) {
      var trash = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.22, 0.7, 10), trashMat);
      trash.position.set(px + pside * 0.6, 0.35, pz + (Math.random() - 0.5) * 0.5);
      trash.castShadow = true; env.add(trash);
    }
    if (Math.random() > 0.8) {
      var hydrant = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.55, 8), hydrantMat);
      hydrant.position.set(px + pside * 0.9, 0.28, pz + (Math.random() - 0.5) * 0.6);
      hydrant.castShadow = true; env.add(hydrant);
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
  /* NOA Phase 6 3D 增强：程序化天空渐变（天顶→地平线），替代纯色背景。
   * 用大半径 SphereGeometry + BackSide + 自定义 ShaderMaterial 实现，
   * 不依赖 Three.js Sky.js 扩展。fog 颜色与地平线色一致保证远处过渡自然。 */
  var skyTop = new THREE.Color(0x2a5fa8);    /* 天顶深蓝 */
  var skyHorizon = new THREE.Color(0xb8d4e8);/* 地平线浅蓝白 */
  scene3d.background = skyHorizon;
  scene3d.fog = new THREE.Fog(0xb8d4e8, 140, 520);
  var skyGeo = new THREE.SphereGeometry(480, 32, 16);
  var skyMat = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    depthWrite: false,
    uniforms: {
      uTop: { value: skyTop },
      uHorizon: { value: skyHorizon },
      uBottom: { value: new THREE.Color(0x6a7a5a) }  /* 地面方向暖灰绿 */
    },
    vertexShader: [
      'varying vec3 vWorldPos;',
      'void main() {',
      '  vWorldPos = (modelMatrix * vec4(position, 1.0)).xyz;',
      '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
      '}'
    ].join('\n'),
    fragmentShader: [
      'uniform vec3 uTop; uniform vec3 uHorizon; uniform vec3 uBottom;',
      'varying vec3 vWorldPos;',
      'void main() {',
      '  float h = normalize(vWorldPos).y;',
      '  vec3 col;',
      '  if (h > 0.0) {',
      '    col = mix(uHorizon, uTop, pow(clamp(h, 0.0, 1.0), 0.6));',
      '  } else {',
      '    col = mix(uHorizon, uBottom, clamp(-h * 2.0, 0.0, 1.0));',
      '  }',
      '  gl_FragColor = vec4(col, 1.0);',
      '}'
    ].join('\n')
  });
  var skyMesh = new THREE.Mesh(skyGeo, skyMat);
  scene3d.add(skyMesh);

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
  scene3d.add(new THREE.AmbientLight(0xaabbdd, 0.32));
  var sun = new THREE.DirectionalLight(0xfff8ee, 1.75);
  sun.position.set(30, 50, 20);
  sun.castShadow = true;
  // 阴影相机参数：更大 frustum 覆盖 chase cam 前方 NPC，4096 贴图减少锯齿
  sun.shadow.mapSize.width = 4096;
  sun.shadow.mapSize.height = 4096;
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 180;
  sun.shadow.camera.left = -60;
  sun.shadow.camera.right = 60;
  sun.shadow.camera.top = 60;
  sun.shadow.camera.bottom = -60;
  sun.shadow.bias = -0.00025;
  sun.shadow.normalBias = 0.025;  // 减少曲面/斜面上的阴影条纹
  scene3d.add(sun);
  _sunLight = sun;  /* 供 _renderFrame 跟随 ego 更新 */
  // sun 目标点跟随 ego（在 _renderFrame 里更新 sun.target.position）
  scene3d.add(sun.target);
  // Hemisphere (sky/ground ambient)
  scene3d.add(new THREE.HemisphereLight(0xaabbdd, 0x554433, 0.4));

  // ── Ground (large flat plane, matches road length) ──
  /* NOA Phase 6 3D 增强：程序化草地纹理（CanvasTexture），替代纯色地面。
   * 256×256 canvas 画基底绿 + 随机深浅斑点模拟草地肌理，repeat 16×8 铺满 800×400。 */
  var gndGeo = new THREE.PlaneGeometry(800, 400);
  var gndCanvas = document.createElement('canvas');
  gndCanvas.width = 256; gndCanvas.height = 256;
  var gctx = gndCanvas.getContext('2d');
  gctx.fillStyle = '#3a5a30'; gctx.fillRect(0, 0, 256, 256);
  for (var gi = 0; gi < 1800; gi++) {
    var gx = Math.random() * 256, gy = Math.random() * 256;
    var shade = 30 + Math.floor(Math.random() * 50);
    gctx.fillStyle = 'rgba(' + (40 + shade) + ',' + (70 + shade) + ',' + (40 + shade * 0.6) + ',0.5)';
    gctx.fillRect(gx, gy, 2, 2);
  }
  var gndTex = new THREE.CanvasTexture(gndCanvas);
  gndTex.wrapS = THREE.RepeatWrapping; gndTex.wrapT = THREE.RepeatWrapping;
  gndTex.repeat.set(16, 8);
  gndTex.anisotropy = 4;
  var gndMat = new THREE.MeshStandardMaterial({ map: gndTex, roughness: 0.95, metalness: 0.0 });
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
  _buildSkyline(scene3d);
  _initTireTrail(scene3d);

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
    // 车轮滚动动画：根据车速绕轮轴旋转
    var egoWheels = ego.userData.wheels;
    if (egoWheels && v.speed > 0.1) {
      var roll = (v.speed || 0) * 0.016 / 0.32;
      for (var wri = 0; wri < egoWheels.length; wri++) {
        egoWheels[wri].rotation.x += roll;
      }
    }
    // 灯光接入感知/规划链路：
    // - 刹车灯：vehicle.brake > 0.1 时点亮（来自 vehicle/state）
    // - 转向灯：metrics.route_lane = -1 左转，+1 右转（来自 planning/trajectory 后缀）
    // - 大灯：常亮
    var _egoMetrics = _topoData.metrics || {};
    var _egoRouteLane = _egoMetrics.route_lane || 0;
    _setVehicleLights(ego, {
      brake: (typeof v.brake === 'number') && v.brake > 0.1,
      turnL: _egoRouteLane < 0,
      turnR: _egoRouteLane > 0,
      head: true
    }, _animT);
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
  // 天际线始终跟随 ego 居中（X+Z），保证远景城市在任意位置都环绕地平线
  if (_skylineGroup) _skylineGroup.position.set(sx, 0, sz);
  // 动态胎痕：采样 ego 后轮世界位置
  _updateTireTrail(sx, sz, _dr.smoothHeading, (v && v.speed) || 0);

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
        // glTF 模型已是真实尺寸（realScale），跳过 (L,H,W) 缩放；
        // 程序化 _buildObstacle 是 unit-normalized，需 scale.set(L,H,W) 映射到真实尺寸
        if (om.userData.realScale) {
          _tmpScale.set(1, 1, 1);
        } else {
          _tmpScale.set(L, H, W);
        }
        om.scale.lerp(_tmpScale, 0.18);

        // ── 朝向：车辆模型前向为 +X，故 rotation.y = -heading。
        // 行人没有 h 字段，用速度向量方向近似；静止行人保持朝前。
        var targetYaw = -ow.heading;
        if ((ow.type === 'pedestrian' || ow.type === 'cyclist') &&
            (Math.abs(ow.vx) > 0.05 || Math.abs(ow.vz) > 0.05)) {
          targetYaw = -Math.atan2(ow.vz, ow.vx);
        }
        var dy = targetYaw - om.rotation.y;
        while (dy > Math.PI) dy -= 2 * Math.PI;
        while (dy < -Math.PI) dy += 2 * Math.PI;
        om.rotation.y += dy * 0.18;

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

        // NPC 车辆车轮滚动动画
        var obsWheels = om.userData.wheels;
        if (obsWheels && (ow.type === 'car' || ow.type === 'suv' || ow.type === 'truck')) {
          var obsSpd = Math.sqrt((ow.vx || 0) * (ow.vx || 0) + (ow.vz || 0) * (ow.vz || 0));
          if (obsSpd > 0.1) {
            // unit-normalized 轮半径 0.17，scale.x = 车长 L，实际轮半径 = 0.17 * L
            var obsRoll = obsSpd * 0.016 / (0.17 * Math.max(1, L));
            for (var owi = 0; owi < obsWheels.length; owi++) {
              obsWheels[owi].rotation.x += obsRoll;
            }
          }
        }

        // 灯光接入感知链路：NPC 障碍物根据 ai 状态点亮刹车/转向灯
        // - 刹车灯：ai ∈ {stop, stop_for_tl, yield}
        // - 转向灯：ai ∈ {branch_sel, merge}，按横向速度方向选边，速度过小双闪
        if (ow.type === 'car' || ow.type === 'suv' || ow.type === 'truck') {
          var _ai = ow.ai || '';
          var _brakeOn = (_ai === 'stop' || _ai === 'stop_for_tl' || _ai === 'yield');
          var _turnActive = (_ai === 'branch_sel' || _ai === 'merge');
          var _turnL = false, _turnR = false;
          if (_turnActive) {
            if (ow.vz > 0.15) _turnR = true;
            else if (ow.vz < -0.15) _turnL = true;
            else { _turnL = true; _turnR = true; }  // 双闪
          }
          _setVehicleLights(om, {
            brake: _brakeOn, turnL: _turnL, turnR: _turnR, head: true
          }, _animT);
        }

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
        // 交通灯 arm 沿模型 +Z，应横跨道路（垂直于道路切线）。
        // 优先使用 flowsim 发布的 heading，无则按道路切线估算。
        if (typeof tlw.h === 'number' && tlw.h !== 0.0) {
          tlm.rotation.y = -(tlw.h + Math.PI / 2);
        } else {
          var tlTan = _getRoadTangentAt(tlw.x, tlw.z);
          if (tlTan.found) {
            tlm.rotation.y = -(tlTan.heading + Math.PI / 2);
          }
        }
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
        // ETC 门架 crossbar 沿模型 +Z，应横跨道路（垂直于道路切线）。
        if (typeof gw.h === 'number' && gw.h !== 0.0) {
          gm.rotation.y = -(gw.h + Math.PI / 2);
        } else {
          var gateTan = _getRoadTangentAt(gw.x, gw.z);
          if (gateTan.found) {
            gm.rotation.y = -(gateTan.heading + Math.PI / 2);
          }
        }
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
    _roadCurves = [];
    if (_roadGroup) _roadGroup.visible = true;
    _applyRoadCurve(scn.road);
  }

  // NOA Phase 6: 规划轨迹渲染 — 从 scn.trajectory_path (Frenet [[s,d,spd],...])
  // 转世界坐标画线。若已有道路网络曲线，则把 ego 当前位置投影到最近 curve，
  // 沿 curve 前进 s 米并横向偏移 d；否则退化为沿 ego heading 直线外推。
  //
  // #3 修复：s 超出当前 edge 剩余长度时，按邻接表 _roadCurveNext 跳到下一条 edge
  // 继续投影，避免弯道末端 clamp 到 edge 尾点导致轨迹堆积。若 path 点含可选
  // edge_id（schema v1.2 预留，第 4 元素），则直接用指定 edge 投影。
  //
  // 3D 增强：用 TubeGeometry 替代 Line（LineBasicMaterial.linewidth 在 WebGL
  // 下被忽略恒为 1px），管半径 0.18m 远距离可见；沿管道 UV 做 dash 流动动画。
  if (scn && scn.trajectory_path && scn.trajectory_path.length > 1) {
    var tpath = scn.trajectory_path;
    var ex = _dr.lastX, ez = _dr.lastZ, eh = _dr.lastHeading || 0;
    var tkey = tpath.length + ':' + Math.round(ex) + ',' + Math.round(ez);
    if (tkey !== _trajLastKey) {
      _trajLastKey = tkey;
      // 构建 world 坐标点数组（Vector3）
      var tvec = [];
      // 把 ego 投影到道路网络曲线，记录起始 edge 索引与 t 参数。
      // trajectory_edge_id（monitor 透传 ego.road_id）优先，避免搜索最近曲线
      // 在多段路网中可能选错边的偏差。无 edge_id 时退化为全局最近搜索。
      var nearCurveIdx = -1, nearT = 0;
      if (_roadCurves && _roadCurves.length) {
        var hintId = (scn.trajectory_edge_id != null) ? scn.trajectory_edge_id : -1;
        if (hintId >= 0 && hintId < _roadCurves.length) {
          // 显式 edge_id：只在该 edge 上搜索 ego 投影点
          nearCurveIdx = hintId;
          var hpts = _roadCurves[hintId].getSpacedPoints(30);
          var bestD2 = Infinity;
          for (var pi = 0; pi < hpts.length; pi++) {
            var dx = hpts[pi].x - ex, dz = hpts[pi].z - ez;
            var d2 = dx * dx + dz * dz;
            if (d2 < bestD2) { bestD2 = d2; nearT = pi / (hpts.length - 1); }
          }
        } else {
          var bestD2 = Infinity;
          for (var ci = 0; ci < _roadCurves.length; ci++) {
            var pts = _roadCurves[ci].getSpacedPoints(30);
            for (var pi = 0; pi < pts.length; pi++) {
              var dx = pts[pi].x - ex, dz = pts[pi].z - ez;
              var d2 = dx * dx + dz * dz;
              if (d2 < bestD2) { bestD2 = d2; nearCurveIdx = ci; nearT = pi / (pts.length - 1); }
            }
          }
        }
      }
      for (var ti = 0; ti < tpath.length; ti++) {
        var pt = tpath[ti];
        if (!pt || pt.length < 2) continue;
        var s = pt[0] || 0, d = pt[1] || 0;
        // 可选 edge_id（schema v1.2 第 4 元素）：直接定位到指定 edge
        var eid = (pt.length >= 4 && typeof pt[3] === 'number') ? pt[3] : -1;
        var wx, wz;
        if (nearCurveIdx >= 0) {
          var curIdx, t0, remain;
          if (eid >= 0 && eid < _roadCurves.length) {
            // planning 显式指定 edge_id：从该 edge 起始处投影
            curIdx = eid; t0 = 0; remain = _roadCurveLens[curIdx] || _roadCurves[curIdx].getLength();
          } else {
            // 链式跨 edge：从 ego 所在 edge 的 nearT 起算，按 s 累计切换 edge
            curIdx = nearCurveIdx; t0 = nearT;
            remain = (_roadCurveLens[curIdx] || _roadCurves[curIdx].getLength()) * (1 - nearT);
          }
          // 按 s 累计找到落点所在 edge（防死循环：最多切 32 条 edge）
          var sLeft = s, guard = 0;
          while (sLeft > remain && _roadCurveNext[curIdx] >= 0 && guard < 32) {
            sLeft -= remain;
            curIdx = _roadCurveNext[curIdx];
            t0 = 0;
            remain = _roadCurveLens[curIdx] || _roadCurves[curIdx].getLength();
            guard++;
          }
          var edgeLen = _roadCurveLens[curIdx] || _roadCurves[curIdx].getLength();
          var t = Math.max(0, Math.min(1, t0 + sLeft / edgeLen));
          var pos = _roadCurves[curIdx].getPointAt(t);
          var tan = _roadCurves[curIdx].getTangentAt(t);
          var nx = -tan.z, nz = tan.x;  // 右侧法线；d>0 在左侧，d<0 在右侧
          wx = pos.x + nx * d;
          wz = pos.z + nz * d;
        } else {
          // 无道路网络时退化为直线外推（旧场景兼容）
          wx = ex + s * Math.cos(eh) - d * Math.sin(eh);
          wz = ez + s * Math.sin(eh) + d * Math.cos(eh);
        }
        tvec.push(new THREE.Vector3(wx, 0.18, wz));
      }
      // 移除旧管
      if (_trajLine) {
        scene3d.remove(_trajLine);
        _trajLine.geometry.dispose();
        _trajLine.material.dispose();
      }
      if (tvec.length >= 2) {
        var tcurve = new THREE.CatmullRomCurve3(tvec);
        var ttubGeo = new THREE.TubeGeometry(tcurve, Math.max(8, tvec.length * 4), 0.18, 8, false);
        /* dash 流动纹理：canvas 画 4 格蓝/透明交替，repeat 沿管道长度铺开，
         * 每帧偏移 offset 产生"规划前进"的流动感。 */
        var dashCv = document.createElement('canvas');
        dashCv.width = 64; dashCv.height = 8;
        var dctx = dashCv.getContext('2d');
        dctx.fillStyle = '#44aaff'; dctx.fillRect(0, 0, 32, 8);
        dctx.fillStyle = 'rgba(68,170,255,0.15)'; dctx.fillRect(32, 0, 32, 8);
        var dashTex = new THREE.CanvasTexture(dashCv);
        dashTex.wrapS = THREE.RepeatWrapping;
        dashTex.wrapT = THREE.RepeatWrapping;
        var tlen = tcurve.getLength();
        dashTex.repeat.set(Math.max(2, tlen / 4), 1);  /* 每 4m 一段 dash */
        var ttubMat = new THREE.MeshStandardMaterial({
          map: dashTex, transparent: true, opacity: 0.9,
          emissive: 0x224488, emissiveIntensity: 0.4, roughness: 0.5
        });
        _trajLine = new THREE.Mesh(ttubGeo, ttubMat);
        _trajLine.userData.dashTex = dashTex;  /* 供 _renderFrame 做流动动画 */
        scene3d.add(_trajLine);
      }
    }
    if (_trajLine) {
      _trajLine.visible = true;
      /* 流动动画：每帧偏移 dash 纹理 offset，模拟规划轨迹向前推进 */
      if (_trajLine.userData.dashTex) {
        _trajLine.userData.dashTex.offset.x = (_animT * 0.08) % 1.0;
      }
    }
  } else if (_trajLine) {
    _trajLine.visible = false;
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
          h: (eg.h || 0),
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

  // Traffic lights: 统一使用 scene.entities 中的 tl（world 坐标 + heading）。
  if (_trafficLightPool && scn && scn.entities) {
    var tlIdx = 0;
    for (var ei3 = 0; ei3 < scn.entities.length && tlIdx < _trafficLightPool.length; ei3++) {
      var ent3 = scn.entities[ei3];
      if (ent3.type !== 'tl') continue;
      _trafficLightWorld[tlIdx] = {
        x: (ent3.x || 0), z: (ent3.y || 0),
        h: (ent3.h || 0),
        state: ent3.state || 'green'
      };
      tlIdx++;
    }
    for (; tlIdx < _trafficLightPool.length; tlIdx++) _trafficLightWorld[tlIdx] = null;
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// Exports
// ══════════════════════════════════════════════════════════════════════════════

export { init3DScene, resize3D, update3D, sceneReady, scene3d, _renderFrame, _applyRoadCurve };
