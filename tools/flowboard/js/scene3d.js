/* global THREE, window, topoData */

// ══════════════════════════════════════════════════════════════════════════════
// scene3d.js — Three.js 3D scene module for ADAS visualization
// ══════════════════════════════════════════════════════════════════════════════

import { safeCall, reportDiag, _makeBox, _makeRect, _buildSedan, _buildObstacle, _buildTrafficLight, _buildETCGate, _auditSceneMaterials } from './utils.js';
import { initDeadReckon, tickDeadReckon, _dr } from './deadreckon.js';
import { init2DFallback } from './scene2d.js';
import { initModelCache, buildEgoCar, buildObstacleGroup, _setVehicleLights } from './models.js';
// 分层架构：utils 层（几何合并、曲线数学、标签 Sprite）已迁移到 scene3d/utils/
import { mergeGeometries as _mergeGeometries, transformGeometry as _transformGeometry } from './scene3d/utils/GeometryMerge.js';
import { curveShiftAt as _curveShiftAt, curveHeadingAt as _curveHeadingAt, getRoadTangentAt as _getRoadTangentAtImpl } from './scene3d/utils/CurveMath.js';
import { makeLabelSprite as _makeLabelSprite, setLabelSprite as _setLabelSprite } from './scene3d/utils/LabelSprite.js';
// 分层架构：model 层（拓扑数据、场景快照、路网模型、性能档位）已迁移到 scene3d/model/
import * as TopologyStore from './scene3d/model/TopologyStore.js';
import { EgoTiltState, TiltPool } from './scene3d/model/VisualPhysics.js';
// 分层架构：view 层（道路、环境、车辆等 Three.js 构建器）
import { buildLegacyRoad, applyRoadCurve } from './scene3d/view/RoadView.js';
// 分层架构：controller 层（相机、用户输入）
import * as CameraController from './scene3d/controller/CameraController.js';

const THREE = window.THREE;

// _getRoadTangentAt 保留无参签名，内部把 _roadCurves 传给 CurveMath 实现。
// 这样 _renderFrame 里的调用点（tlw.x/gw.x 等）无需改动。
function _getRoadTangentAt(x, z) {
  return _getRoadTangentAtImpl(_roadCurves, x, z);
}

// _getRoadElevationAt — 查询世界坐标 (x,z) 处的道路高度（高架 elevation）。
// 在所有 _roadCurves 上找最近采样点，返回其 Y（曲线高度）。
// 用于让 ego/障碍物/红绿灯/ETC 门架 Y 跟随高架 elevation 抬升，避免"悬在路面下方"。
// 平地场景所有曲线 Y=0，返回 0，行为完全向后兼容。
function _getRoadElevationAt(x, z) {
  if (!_roadCurves || !_roadCurves.length) return 0;
  var bestD2 = Infinity, bestY = 0;
  for (var ci = 0; ci < _roadCurves.length; ci++) {
    var curve = _roadCurves[ci];
    if (!curve) continue;
    var pts = curve.getSpacedPoints(24);
    for (var pi = 0; pi < pts.length; pi++) {
      var dx = pts[pi].x - x, dz = pts[pi].z - z;
      var d2 = dx * dx + dz * dz;
      if (d2 < bestD2) { bestD2 = d2; bestY = pts[pi].y; }
    }
  }
  return bestY;
}

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
export function setTopoData(d) {
  _topoData = d || _topoData;
  // 分层架构：同步写入 TopologyStore，供 model 层访问器使用。
  // scene3d.js 内部仍读 _topoData（渐进式迁移，保持逻辑不变）。
  TopologyStore.setTopoData(_topoData);
}

/** Camera chase-cam state vectors */
let _cam = null, _camLook = null, _camTarget = null, _camLookTarget = null;

/** Scene object references — road, ground, environment, car */
let _lidarCloud = null;
let _lidarWorld = [];
let _obsPool = [], _obsWorld = [];
/** MVC Model: 视觉物理状态（roll/pitch） */
let _egoTilt = new EgoTiltState();
let _obsTilt = new TiltPool();
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

/** 远处城市天际线：低多边形建筑剪影环，始终跟随 ego 居中，
 *  配合 fog 产生大气透视效果。 */
let _skylineGroup = null;

/** B.1: 雨粒子系统（LineSegments），提升天气氛围 */
let _rainMesh = null;
let _rainPos = null;              // Float32Array of positions
let _rainVel = null;              // Float32Array of fall speeds
const _RAIN_COUNT = 1200;         // 雨线条数
const _RAIN_BOX = 120;            // 围绕 ego 的 120m × 120m 范围
const _RAIN_HEIGHT = 40;          // 雨滴起始高度

/** B.1: 程序化水面 —— 道路两侧低洼水域，ShaderMaterial 波动 + 菲涅尔反射 */
let _waterMesh = null;
let _waterMat = null;
const _WATER_SIZE = 1200;         // 水面平面尺寸（m）
const _WATER_SEGS = 80;           // 细分段数（high 档位）

/** WebGL context-loss flag — render loop skips while true */
let _glLost = false;

/** 3D 初始化失败标志 — 防止 stale message 覆盖错误提示 */
let _3DInitFailed = false;

/** Animation time counter (incremented per frame) */
let _animT = 0;

/** Performance tier: 'high'|'medium'|'low'. Auto-adjusted by FPS monitor. */
let _perfTier = 'high';
let _perfFrameCount = 0;
let _perfLastTime = 0;
let _perfCheckInterval = 60; // 每 60 帧评估一次

/** scene3d.frame 连续失败计数：FPS 采样对渲染异常无感（异常被 catch 的耗时远低于
 *  真实渲染一帧，_updatePerfTier 会被"虚高的 FPS"骗过），需要单独的异常驱动熔断，
 *  在连续失败达到阈值时强制降级关闭 Bloom，避免画布永久黑屏。 */
let _frameFailCount = 0;
const _FRAME_FAIL_THRESHOLD = 5;

/** Pre-allocated vector/scale objects (avoids per-frame GC pressure) */
let _tmpV3 = null, _tmpScale = null;

/** Obstacle height lookup (defined once, shared across all _renderFrame calls) */
const _OBS_H = { truck: 2.8, pedestrian: 1.8, cyclist: 1.7, cone: 0.8 };
/** Obstacle length/width lookup — without these, types missing len/wid in
 * telemetry (e.g. pedestrians) fall back to vehicle-sized defaults. */
const _OBS_L = { truck: 7.5, pedestrian: 0.5, cyclist: 1.8, cone: 0.5 };
const _OBS_W = { truck: 2.3, pedestrian: 0.5, cyclist: 0.6, cone: 0.5 };

/** MVC Controller: 相机模式与轨道交互状态（scene3d/controller/CameraController.js） */
let _npcPanel = null;     // DOM element for clicked NPC info

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
// 已迁移至 scene3d/utils/LabelSprite.js（makeLabelSprite / setLabelSprite）

// _buildRoad / applyRoadCurve 已迁移至 MVC View 层 scene3d/view/RoadView.js
// 场景保留 _applyRoadCurve 包装函数以管理 _lastCurveKey / _curveActive 状态。

function _applyRoadCurve(roadData) {
  if (!roadData) return;
  var sx = roadData.curve_start_x || 0, len = roadData.curve_length_m || 0;
  var off = roadData.curve_offset_m || 0;
  var key = sx + "," + len + "," + off;
  if (key === _lastCurveKey) return;
  _lastCurveKey = key;
  _curveActive = applyRoadCurve(_roadGroup, roadData);
}

// ══════════════════════════════════════════════════════════════════════════════
// Phase 3: 多段道路网络渲染 — 从 scene/frame 的 road_network.edges 构建
// 使用 CatmullRomCurve3 + 自定义 ribbon mesh 生成沿曲线的平坦路面。
// 每条 edge 的 nodes [[x,y],...] 是道路参考线控制点，x=纵向 y=横向。
// ══════════════════════════════════════════════════════════════════════════════

// 共享沥青纹理缓存：避免每条 edge 都创建 512×512 CanvasTexture。
let _asphaltTex = null;
let _shoulderTex = null;

/** 程序化沥青纹理 v3：深色基底 + 骨料颗粒，与草地/路肩形成强对比。
 *  v2 用 #55585e 中灰底，亮度 ~88，与草地 #5a8a4a 亮度 ~90 几乎相同，
 *  视觉上"路和地一样亮"，被用户吐槽看不出区别。v3 改为 #1f2126 深沥青
 *  （亮度 ~33），亮度差 3 倍，从远景到近景都能清晰分辨路面 vs 草地。 */
function _makeAsphaltTexture() {
  if (_asphaltTex) return _asphaltTex;
  var canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 512;
  var ctx = canvas.getContext('2d');
  // 基底：深沥青（参考真实新铺沥青 #1a1a1a，略带蓝调避免与路肩棕色混淆）
  ctx.fillStyle = '#1f2126'; ctx.fillRect(0, 0, 512, 512);
  // 中等补丁：轮胎磨损/油渍斑（略浅于基底，让路面有层次但不发灰）
  for (var p = 0; p < 35; p++) {
    var px = Math.random() * 512, py = Math.random() * 512;
    var pr = 30 + Math.random() * 80;
    var grd = ctx.createRadialGradient(px, py, 0, px, py, pr);
    var base = 38 + Math.floor(Math.random() * 20);
    var isDark = Math.random() > 0.5;
    var a0 = isDark ? 0.28 : 0.18;
    grd.addColorStop(0, 'rgba(' + base + ',' + (base + 2) + ',' + (base + 6) + ',' + a0 + ')');
    grd.addColorStop(1, 'rgba(' + base + ',' + (base + 2) + ',' + (base + 6) + ',0)');
    ctx.fillStyle = grd;
    ctx.beginPath(); ctx.arc(px, py, pr, 0, Math.PI * 2); ctx.fill();
  }
  // 深色骨料颗粒（沥青质感，与基底同色系加深）
  for (var i = 0; i < 5500; i++) {
    var x = Math.random() * 512, y = Math.random() * 512;
    var shade = Math.floor(Math.random() * 22);
    ctx.fillStyle = 'rgba(' + (25 + shade) + ',' + (27 + shade) + ',' + (32 + shade) + ',0.45)';
    ctx.fillRect(x, y, 2, 2);
  }
  // 浅色颗粒/反光碎石
  for (var j = 0; j < 1800; j++) {
    var x2 = Math.random() * 512, y2 = Math.random() * 512;
    var sh2 = Math.floor(Math.random() * 35);
    ctx.fillStyle = 'rgba(' + (110 + sh2) + ',' + (114 + sh2) + ',' + (120 + sh2) + ',0.22)';
    ctx.fillRect(x2, y2, 2, 2);
  }
  _asphaltTex = new THREE.CanvasTexture(canvas);
  _asphaltTex.wrapS = THREE.RepeatWrapping; _asphaltTex.wrapT = THREE.RepeatWrapping;
  return _asphaltTex;
}

/** 路肩专用纹理：深褐灰土路/碎石带，与沥青有明显分界。 */
function _makeShoulderTexture() {
  if (_shoulderTex) return _shoulderTex;
  var canvas = document.createElement('canvas');
  canvas.width = 256; canvas.height = 256;
  var ctx = canvas.getContext('2d');
  ctx.fillStyle = '#5a4d40'; ctx.fillRect(0, 0, 256, 256);
  for (var p = 0; p < 40; p++) {
    var px = Math.random() * 256, py = Math.random() * 256;
    var pr = 20 + Math.random() * 40;
    var grd = ctx.createRadialGradient(px, py, 0, px, py, pr);
    var base = 80 + Math.floor(Math.random() * 25);
    var a0 = Math.random() > 0.5 ? 0.20 : 0.14;
    grd.addColorStop(0, 'rgba(' + base + ',' + (base - 8) + ',' + (base - 16) + ',' + a0 + ')');
    grd.addColorStop(1, 'rgba(' + base + ',' + (base - 8) + ',' + (base - 16) + ',0)');
    ctx.fillStyle = grd;
    ctx.beginPath(); ctx.arc(px, py, pr, 0, Math.PI * 2); ctx.fill();
  }
  for (var i = 0; i < 1500; i++) {
    var x = Math.random() * 256, y = Math.random() * 256;
    var shade = Math.floor(Math.random() * 25);
    ctx.fillStyle = 'rgba(' + (75 + shade) + ',' + (68 + shade) + ',' + (58 + shade) + ',0.4)';
    ctx.fillRect(x, y, 2, 2);
  }
  _shoulderTex = new THREE.CanvasTexture(canvas);
  _shoulderTex.wrapS = THREE.RepeatWrapping; _shoulderTex.wrapT = THREE.RepeatWrapping;
  return _shoulderTex;
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

  // 道路合并绘制：把所有 edge 的同类型几何体收集后合并，draw call 从 O(N) 降到 O(1)。
  var allRoadPos = [], allRoadIdx = [], allRoadUV = [];
  var allShldPos = [], allShldIdx = [];
  var roadVertOffset = 0, shldVertOffset = 0;
  // 车道线（黄/白，实/虚）全部合并为一个 vertex-colors mesh
  var laneMarkPos = [], laneMarkIdx = [], laneMarkCol = [];
  var laneMarkVertOffset = 0;
  // 护栏/路灯/箭头/停止线模板几何体累积，循环结束后合并
  var guardPostGeos = [], guardCapGeos = [], guardRefGeos = [], guardRailGeos = [];
  var lampPostGeos = [], lampHeadGeos = [], lampGlowGeos = [];
  var arrowGeos = [], stopLineGeos = [];
  // B.1: 路面水坑几何体累积，合并为 1 个 reflective mesh
  var puddleGeos = [];

  // NOA Phase 6: 收集每条 edge 的首尾世界坐标，渲染后扫描分叉/汇入点。
  // 两条 edge 的首/尾节点重合（距离 < 1m）即判定为 junction 连接点。
  var edgeEnds = [];  // [{start:{x,z}, end:{x,z}, lanes, length}, ...]

  for (var ei = 0; ei < edges.length; ei++) {
    var edge = edges[ei];
    var nodes = edge.nodes;
    if (!nodes || nodes.length < 2) continue;

    // 从 nodes 构建 CatmullRomCurve3 控制点
    // nodes[ni] = [x, y, z]：z 是道路 elevation（高架高度），平地场景为 0。
    // 用 z 做 Three.js 的 Y（向上），曲线自然带高度起伏。
    var points = [];
    for (var ni = 0; ni < nodes.length; ni++) {
      var nodeZ = (nodes[ni].length >= 3) ? (nodes[ni][2] || 0) : 0;
      points.push(new THREE.Vector3(nodes[ni][0], nodeZ, nodes[ni][1]));
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
      lanes: lanes, laneWidth: laneWidth, length: length, curve: curve
    });

    // ── 路面 ribbon mesh ──
    // 左边缘 = +halfWidth，右边缘 = -halfWidth（对称）
    // Y 高度 = pos.y（道路 elevation，高架段 > 0）
    var nSeg = Math.max(4, Math.floor(length / SEG_LEN));
    var positions = [];
    var indices = [];
    var uvs = [];
    for (var si = 0; si <= nSeg; si++) {
      var t = si / nSeg;
      var pos = curve.getPointAt(t);
      var tangent = curve.getTangentAt(t);
      // 法线（切线在 XZ 平面内旋转 90°，指向"右"侧，即 d<0）
      var nx = -tangent.z, nz = tangent.x;

      var lx = pos.x + nx * halfWidth, lz = pos.z + nz * halfWidth;  // 左边缘
      var rx = pos.x - nx * halfWidth, rz = pos.z - nz * halfWidth;  // 右边缘
      positions.push(lx, pos.y + 0.01, lz);
      positions.push(rx, pos.y + 0.01, rz);
      // world-space UV：所有 edge 共享同一 repeat（4m 一个纹理周期）
      uvs.push(lx / 4, lz / 4);
      uvs.push(rx / 4, rz / 4);

      if (si < nSeg) {
        var base = si * 2;
        indices.push(base, base + 1, base + 2);
        indices.push(base + 1, base + 3, base + 2);
      }
    }

    // 把当前 edge 路面追加到全局数组
    for (var pi = 0; pi < positions.length; pi++) allRoadPos.push(positions[pi]);
    for (var ui = 0; ui < uvs.length; ui++) allRoadUV.push(uvs[ui]);
    for (var ii = 0; ii < indices.length; ii++) allRoadIdx.push(indices[ii] + roadVertOffset);
    roadVertOffset += positions.length / 3;

    // ── 路肩（路面两侧外扩 1.2m，外侧略低模拟排水横坡）──
    var shldW = 1.2;
    var sPos = [], sIdx = [];
    for (var si2 = 0; si2 <= nSeg; si2++) {
      var t2 = si2 / nSeg;
      var p2 = curve.getPointAt(t2);
      var tan2 = curve.getTangentAt(t2);
      var nx2 = -tan2.z, nz2 = tan2.x;
      // 内侧与路面同高，外侧降低 0.06m
      sPos.push(p2.x + nx2 * halfWidth, p2.y + 0.015, p2.z + nz2 * halfWidth);
      sPos.push(p2.x + nx2 * (halfWidth + shldW), p2.y - 0.045, p2.z + nz2 * (halfWidth + shldW));
      sPos.push(p2.x - nx2 * halfWidth, p2.y + 0.015, p2.z - nz2 * halfWidth);
      sPos.push(p2.x - nx2 * (halfWidth + shldW), p2.y - 0.045, p2.z - nz2 * (halfWidth + shldW));
      if (si2 < nSeg) {
        var b2 = si2 * 4;
        sIdx.push(b2, b2 + 1, b2 + 4,  b2 + 1, b2 + 5, b2 + 4);
        sIdx.push(b2 + 2, b2 + 6, b2 + 3,  b2 + 3, b2 + 6, b2 + 7);
      }
    }
    for (var spi = 0; spi < sPos.length; spi++) allShldPos.push(sPos[spi]);
    for (var sii = 0; sii < sIdx.length; sii++) allShldIdx.push(sIdx[sii] + shldVertOffset);
    shldVertOffset += sPos.length / 3;

    // ── 车道线：中心双黄线 + 多车道时分隔虚线 + 道路边缘白实线 ──
    // 中心双黄线（实线）：只对双向道路（lanes ≥ 2）绘制，单车道为单向路无需中心线
    if (lanes >= 2) {
      laneMarkVertOffset = _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg, -0.12, 0.15, 0xffd633, 0.046);
      laneMarkVertOffset = _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg,  0.12, 0.15, 0xffd633, 0.046);
    }
    // 多车道分隔虚线：每方向 ≥2 车道时才需要内侧分隔线。
    // lanes 是双向总车道数，floor(lanes/2) 是单向车道数，避免 3 车道不对称路误画。
    var perSide = Math.floor(lanes / 2);
    for (var li = 1; li < perSide; li++) {
      var off = li * laneWidth;
      laneMarkVertOffset = _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg,  off, 0.15, 0xffffff, 0.046, true);
      laneMarkVertOffset = _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg, -off, 0.15, 0xffffff, 0.046, true);
    }
    // 道路边缘白实线
    laneMarkVertOffset = _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg,  halfWidth - 0.06, 0.15, 0xffffff, 0.046);
    laneMarkVertOffset = _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg, -halfWidth + 0.06, 0.15, 0xffffff, 0.046);

    // ── 路面导向箭头（每条 edge 中段画一个直行箭头）──
    _addRoadArrow(arrowGeos, curve, length * 0.5, laneWidth, lanes);
    // ── 停止线（起点处，城市/路口道路）──
    if (length < 120) {
      _addStopLine(stopLineGeos, curve, 0, laneWidth, lanes);
    }

    // ── B.1: 路面水坑（雨后积水，低 roughness 模拟反射）──
    // 每条约 0-2 个水坑，沿曲线随机分布，合并后仅 1 个 draw call
    var nPuddle = Math.floor(Math.random() * 2.2);
    for (var pi2 = 0; pi2 < nPuddle; pi2++) {
      var ps = (0.15 + Math.random() * 0.7) * length;
      var pt2 = ps / length;
      var ppos2 = curve.getPointAt(pt2);
      var ptan2 = curve.getTangentAt(pt2);
      var pnx2 = -ptan2.z, pnz2 = ptan2.x;
      var plat = (Math.random() - 0.5) * roadWidth * 0.75;
      var pw2 = 1.0 + Math.random() * 1.2;
      var pl2 = 1.4 + Math.random() * 1.6;
      var puddleGeo = new THREE.PlaneGeometry(pw2, pl2);
      var pm = new THREE.Matrix4();
      pm.makeRotationX(-Math.PI / 2);
      pm.multiply(new THREE.Matrix4().makeRotationZ(-Math.atan2(ptan2.z, ptan2.x)));
      pm.setPosition(ppos2.x + pnx2 * lat, ppos2.y + 0.012, ppos2.z + pnz2 * lat);
      puddleGeos.push(_transformGeometry(puddleGeo, pm));
    }

    // ── 道路护栏（两侧）──
    // 波形护栏：圆形立柱 + 上下双横梁 + 反光片，比单一方块真实。
    // 为了降低 draw call，不直接 group.add，而是把变换后的几何体 push
    // 到数组，等所有 edge 处理完再合并为 2 个 mesh（guardMat + reflectorMat）。
    var guardSpacing = 3.0;
    var guardCount = Math.max(2, Math.floor(length / guardSpacing));
    var guardPostGeoTpl = new THREE.CylinderGeometry(0.055, 0.07, 0.9, 12);
    var guardCapGeoTpl = new THREE.CylinderGeometry(0.08, 0.055, 0.04, 12);
    var guardRailGeoTpl = new THREE.BoxGeometry(1, 0.05, 0.07, 1, 1, 1);
    var guardRefGeoTpl = new THREE.BoxGeometry(0.04, 0.18, 0.12);
    for (var gi2 = 0; gi2 <= guardCount; gi2++) {
      var gt = gi2 / guardCount;
      var gp = curve.getPointAt(gt);
      var gtan = curve.getTangentAt(gt);
      var gnx = -gtan.z, gnz = gtan.x;
      var gLeft = halfWidth + 0.45, gRight = -(halfWidth + 0.45);
      for (var gside = 0; gside < 2; gside++) {
        var lateral = gside === 0 ? gLeft : gRight;
        var gpP = new THREE.Vector3(gp.x + gnx * lateral, gp.y + 0.45, gp.z + gnz * lateral);
        var postMat = new THREE.Matrix4().makeTranslation(gpP.x, gpP.y, gpP.z);
        guardPostGeos.push(_transformGeometry(guardPostGeoTpl, postMat));
        // 立柱顶部盖帽
        var capMat = new THREE.Matrix4().makeTranslation(gpP.x, gpP.y + 0.47, gpP.z);
        guardCapGeos.push(_transformGeometry(guardCapGeoTpl, capMat));
        // 每隔一根立柱加反光片
        if (gi2 % 2 === 0) {
          var refMat = new THREE.Matrix4().makeTranslation(
            gpP.x - gnx * 0.04, gpP.y + 0.12, gpP.z - gnz * 0.04);
          guardRefGeos.push(_transformGeometry(guardRefGeoTpl, refMat));
        }
      }
      // 立柱间横梁（上下各一根）
      if (gi2 < guardCount) {
        var gt2 = (gi2 + 1) / guardCount;
        var gp2 = curve.getPointAt(gt2);
        var gtan2 = curve.getTangentAt(gt2);
        var gnx2 = -gtan2.z, gnz2 = gtan2.x;
        var railLen = gp.distanceTo(gp2);
        var gpMidY = (gp.y + gp2.y) / 2;
        for (var gside = 0; gside < 2; gside++) {
          var lateral = gside === 0 ? gLeft : gRight;
          var gpS = new THREE.Vector3(gp.x + gnx * lateral, gp.y + 0.62, gp.z + gnz * lateral);
          var gpE = new THREE.Vector3(gp2.x + gnx2 * lateral, gp2.y + 0.62, gp2.z + gnz2 * lateral);
          var railM = new THREE.Matrix4();
          // 构造 lookAt 矩阵：Z 轴指向 gpE，Y 轴向上
          var eye = gpS.clone().lerp(gpE, 0.5);
          var target = gpE.clone();
          var up = new THREE.Vector3(0, 1, 0);
          railM.lookAt(eye, target, up);
          // BoxGeometry(1,0.05,0.07) 默认沿 X 轴，需缩放 X=railLen
          railM.multiply(new THREE.Matrix4().makeScale(railLen, 1, 1));
          guardRailGeos.push(_transformGeometry(guardRailGeoTpl, railM));
          // 下横梁
          var railM2 = new THREE.Matrix4();
          eye = gpS.clone().lerp(gpE, 0.5); eye.y = gpMidY + 0.32;
          target = gpE.clone(); target.y = gpMidY + 0.32;
          railM2.lookAt(eye, target, up);
          railM2.multiply(new THREE.Matrix4().makeScale(railLen, 1, 1));
          guardRailGeos.push(_transformGeometry(guardRailGeoTpl, railM2));
        }
      }
    }

    // ── 路灯（两侧交错，每隔 40m）──
    // 静态灯杆 + emissive 灯罩 + 光晕面片，不添加 PointLight，避免动态光源数量爆炸。
    var lampSpacing = 40.0;
    var lampCount = Math.max(1, Math.floor(length / lampSpacing));
    var lampPostGeoTpl = new THREE.CylinderGeometry(0.07, 0.11, 6.0, 10);
    // 灯罩：外壳（扁盒）+ 发光面板（略大扁盒）
    var lampHeadGeoTpl = new THREE.BoxGeometry(0.55, 0.14, 0.22);
    var lampPanelGeoTpl = new THREE.BoxGeometry(0.48, 0.10, 0.30);
    // 光晕面片：面向道路下方，模拟灯罩在地面的光斑
    var lampGlowGeoTpl = new THREE.PlaneGeometry(3.2, 3.2);
    for (var li3 = 1; li3 <= lampCount; li3++) {
      var lt = Math.min(1.0, li3 * lampSpacing / length);
      var lp = curve.getPointAt(lt);
      var ltan = curve.getTangentAt(lt);
      var lnx = -ltan.z, lnz = ltan.x;
      var side = (li3 % 2 === 0) ? 1 : -1;  // 交错
      var lateral = side * (halfWidth + 2.5);
      var poleX = lp.x + lnx * lateral, poleZ = lp.z + lnz * lateral;
      var postMat = new THREE.Matrix4().makeTranslation(poleX, lp.y + 3.0, poleZ);
      lampPostGeos.push(_transformGeometry(lampPostGeoTpl, postMat));
      // 灯罩：lookAt 朝向道路内侧
      var lampEye = new THREE.Vector3(poleX - lnx * side * 0.8, lp.y + 5.9, poleZ - lnz * side * 0.8);
      var lampTarget = new THREE.Vector3(poleX - lnx * side * 3.0, lp.y, poleZ - lnz * side * 3.0);
      var headingL = Math.atan2(-ltan.z * side, -ltan.x * side);
      var lampM = new THREE.Matrix4();
      lampM.makeRotationY(headingL);
      lampM.setPosition(lampEye);
      lampHeadGeos.push(_transformGeometry(lampHeadGeoTpl, lampM));
      // 发光面板：略低于外壳，朝向路面
      var panelM = new THREE.Matrix4();
      panelM.makeRotationY(headingL);
      panelM.setPosition(poleX - lnx * side * 0.8, lp.y + 5.82, poleZ - lnz * side * 0.8);
      lampHeadGeos.push(_transformGeometry(lampPanelGeoTpl, panelM));
      // 地面光晕：贴地半透明面片，位于灯罩正下方道路区域
      var glowX = poleX - lnx * side * 2.0;
      var glowZ = poleZ - lnz * side * 2.0;
      var glowM = new THREE.Matrix4();
      glowM.makeRotationX(-Math.PI / 2);
      glowM.multiply(new THREE.Matrix4().makeRotationZ(-Math.atan2(ltan.z, ltan.x)));
      glowM.setPosition(glowX, lp.y + 0.015, glowZ);
      lampGlowGeos.push(_transformGeometry(lampGlowGeoTpl, glowM));
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

  // ── 合并所有 edge 的路面 + 路肩，创建最终 mesh ─────────────────
  // draw call 从 O(edges) 降到 O(1)，同时共享材质/纹理。
  if (allRoadPos.length >= 9) {
    var roadGeo = new THREE.BufferGeometry();
    roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(allRoadPos, 3));
    roadGeo.setAttribute('uv', new THREE.Float32BufferAttribute(allRoadUV, 2));
    roadGeo.setIndex(allRoadIdx);
    roadGeo.computeVertexNormals();
    var asphaltTex = _makeAsphaltTexture();
    // world-space UV 已经是 x/4,z/4，repeat 保持 1 即为每 4m 一个纹理周期
    asphaltTex.repeat.set(1, 1);
    asphaltTex.wrapS = THREE.RepeatWrapping; asphaltTex.wrapT = THREE.RepeatWrapping;
    asphaltTex.anisotropy = 8;
    var roadMat = new THREE.MeshStandardMaterial({
      map: asphaltTex,
      color: 0xffffff, roughness: 0.62, metalness: 0.0,
      bumpMap: asphaltTex, bumpScale: 0.05,
      envMapIntensity: 0.6  // 略反光，与哑光草地（roughness 0.95）形成质感差异
    });
    var roadMesh = new THREE.Mesh(roadGeo, roadMat);
    roadMesh.receiveShadow = true;
    group.add(roadMesh);
  }
  if (allShldPos.length >= 9) {
    var shldGeo = new THREE.BufferGeometry();
    shldGeo.setAttribute('position', new THREE.Float32BufferAttribute(allShldPos, 3));
    // world-space UV 用于 bumpMap
    var shldUV = [];
    for (var su = 0; su < allShldPos.length; su += 3) {
      shldUV.push(allShldPos[su] / 6, allShldPos[su + 2] / 6);
    }
    shldGeo.setAttribute('uv', new THREE.Float32BufferAttribute(shldUV, 2));
    shldGeo.setIndex(allShldIdx);
    shldGeo.computeVertexNormals();
    var shldTex = _makeShoulderTexture();
    shldTex.repeat.set(1, 1);
    shldTex.wrapS = THREE.RepeatWrapping; shldTex.wrapT = THREE.RepeatWrapping;
    shldTex.anisotropy = 4;
    var shldMesh = new THREE.Mesh(shldGeo,
      new THREE.MeshStandardMaterial({ map: shldTex, color: 0xcccccc, roughness: 0.95, metalness: 0.0, bumpMap: shldTex, bumpScale: 0.01 }));
    shldMesh.receiveShadow = true;
    group.add(shldMesh);
  }

  // 合并护栏：post+cap+rail 同材质合并；reflector 单独合并。
  // 参考图：黑色金属护栏
  var guardMat = new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.45, roughness: 0.45 });
  var reflectorMat = new THREE.MeshStandardMaterial({ color: 0xffeebb, emissive: 0xffeebb, emissiveIntensity: 0.35, roughness: 0.4 });
  var allGuardGeos = guardPostGeos.concat(guardCapGeos).concat(guardRailGeos);
  if (allGuardGeos.length) {
    var guardMesh = new THREE.Mesh(_mergeGeometries(allGuardGeos), guardMat);
    guardMesh.castShadow = true;
    group.add(guardMesh);
  }
  if (guardRefGeos.length) {
    var refMesh = new THREE.Mesh(_mergeGeometries(guardRefGeos), reflectorMat);
    group.add(refMesh);
  }

  // 合并路灯：灯杆 + 灯罩分别合并
  var lampPostMat = new THREE.MeshStandardMaterial({ color: 0x555555, metalness: 0.5, roughness: 0.5 });
  var lampHeadMat = new THREE.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 0.9 });
  var lampGlowMat = new THREE.MeshBasicMaterial({
    color: 0xffffee, transparent: true, opacity: 0.12,
    depthWrite: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending
  });
  if (lampPostGeos.length) {
    var lampPostMesh = new THREE.Mesh(_mergeGeometries(lampPostGeos), lampPostMat);
    lampPostMesh.castShadow = true;
    group.add(lampPostMesh);
  }
  if (lampHeadGeos.length) {
    var lampHeadMesh = new THREE.Mesh(_mergeGeometries(lampHeadGeos), lampHeadMat);
    group.add(lampHeadMesh);
  }
  if (lampGlowGeos.length) {
    var lampGlowMesh = new THREE.Mesh(_mergeGeometries(lampGlowGeos), lampGlowMat);
    lampGlowMesh.renderOrder = 1;  // 确保光晕在路面之上
    group.add(lampGlowMesh);
  }

  // 合并车道线（vertex colors）：所有黄/白、实/虚线合并为 1 个 mesh
  if (laneMarkPos.length >= 6) {
    var laneMarkGeo = new THREE.BufferGeometry();
    laneMarkGeo.setAttribute('position', new THREE.Float32BufferAttribute(laneMarkPos, 3));
    laneMarkGeo.setAttribute('color', new THREE.Float32BufferAttribute(laneMarkCol, 3));
    laneMarkGeo.setIndex(laneMarkIdx);
    laneMarkGeo.computeVertexNormals();
    var laneMarkMat = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.4,
      metalness: 0.0,
      emissive: 0xffffff,
      emissiveIntensity: 0.35  // 提高标线自发光，在深色沥青上更醒目
    });
    var laneMarkMesh = new THREE.Mesh(laneMarkGeo, laneMarkMat);
    laneMarkMesh.receiveShadow = false;
    group.add(laneMarkMesh);
  }

  // 合并箭头 + 停止线为 1 个白色 emissive mesh
  var arrowMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.4, emissive: 0xffffff, emissiveIntensity: 0.15 });
  if (arrowGeos.length) {
    var arrowMesh = new THREE.Mesh(_mergeGeometries(arrowGeos), arrowMat);
    group.add(arrowMesh);
  }
  if (stopLineGeos.length) {
    var stopLineMesh = new THREE.Mesh(_mergeGeometries(stopLineGeos), arrowMat);
    group.add(stopLineMesh);
  }

  // B.1: 合并路面水坑为 1 个 reflective mesh
  if (puddleGeos.length) {
    var puddleMat = new THREE.MeshStandardMaterial({
      color: 0x222228, roughness: 0.12, metalness: 0.55,
      transparent: true, opacity: 0.85
    });
    var puddleMesh = new THREE.Mesh(_mergeGeometries(puddleGeos), puddleMat);
    puddleMesh.receiveShadow = true;
    group.add(puddleMesh);
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
  // fork 拓宽：在分叉点画三角导流岛 + 拓宽渐变段（主干全宽 → 两条匝道起点）。
  // 解决"匝道直接长在道路上"的视觉问题——真实高速分叉点有物理岛和拓宽过渡。
  var _islandMat = new THREE.MeshStandardMaterial({
    color: 0xffcc00, roughness: 0.5, emissive: 0x332200, emissiveIntensity: 0.25,
    side: THREE.DoubleSide
  });
  var _taperMat = new THREE.MeshStandardMaterial({
    color: 0x2a2a2a, roughness: 0.92, metalness: 0.0
  });
  for (var ci = 0; ci < clusters.length; ci++) {
    if (clusters[ci].count < 3) continue;
    var kind = clusters[ci].minLanes <= 1 ? 'fork' : 'merge';
    var color = kind === 'fork' ? 0xff8800 : 0x44cc44;
    var jx = clusters[ci].x, jz = clusters[ci].z;

    // 识别该 cluster 关联的 edge：
    //   incoming = end 在 cluster 处的 edge（主干路，车道数最多）
    //   connecting = start 在 cluster 处的 edge（匝道，车道数少）
    var incomingEdge = null, connectingEdges = [];
    for (var ei = 0; ei < edgeEnds.length; ei++) {
      var ee = edgeEnds[ei];
      var dStart2 = (ee.start.x - jx) * (ee.start.x - jx) + (ee.start.z - jz) * (ee.start.z - jz);
      var dEnd2   = (ee.end.x   - jx) * (ee.end.x   - jx) + (ee.end.z   - jz) * (ee.end.z   - jz);
      if (dEnd2 < JUNCTION_TOL * JUNCTION_TOL) {
        if (!incomingEdge || ee.lanes > incomingEdge.lanes) incomingEdge = ee;
      }
      if (dStart2 < JUNCTION_TOL * JUNCTION_TOL) {
        connectingEdges.push(ee);
      }
    }

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

    // ── fork 拓宽：三角导流岛 + 拓宽渐变段 ──
    // 真实高速分叉点：主干路末尾从全宽渐变收缩，中央有三角物理岛分隔两条匝道入口。
    if (kind === 'fork' && incomingEdge && connectingEdges.length >= 2 && incomingEdge.curve) {
      // incoming 末尾切线方向（curve.getTangentAt(1.0) = 末点切线）
      var inTan;
      try { inTan = incomingEdge.curve.getTangentAt(1.0); }
      catch (eTan) { inTan = new THREE.Vector3(1, 0, 0); }
      var tx = inTan.x, tz = inTan.z;
      var tlen = Math.sqrt(tx * tx + tz * tz);
      if (tlen < 0.001) { tx = 1; tz = 0; } else { tx /= tlen; tz /= tlen; }
      // 法线（切线在 XZ 平面内旋转 90°）
      var tnx = -tz, tnz = tx;
      var inLW = incomingEdge.laneWidth || 3.5;
      var inHalfW = (incomingEdge.lanes * inLW) / 2;

      // (a) 拓宽渐变段：从 cluster 后方 taperLen 处（主干全宽）
      //     线性收缩到 cluster 处（导流岛尖端宽度 0.6m）
      //     覆盖主干末尾断崖式宽度跳变，视觉上"主干拓宽过渡到分叉"。
      var taperLen = 18.0;
      var taperStartHalf = inHalfW;        // 主干全宽的一半
      var taperEndHalf = 0.3;              // cluster 处缩为 0.6m 宽（导流岛尖端）
      var startX = jx - tx * taperLen;
      var startZ = jz - tz * taperLen;
      var sL = { x: startX + tnx * taperStartHalf, z: startZ + tnz * taperStartHalf };
      var sR = { x: startX - tnx * taperStartHalf, z: startZ - tnz * taperStartHalf };
      var eL = { x: jx + tnx * taperEndHalf, z: jz + tnz * taperEndHalf };
      var eR = { x: jx - tnx * taperEndHalf, z: jz - tnz * taperEndHalf };
      var taperGeo = new THREE.BufferGeometry();
      taperGeo.setAttribute('position', new THREE.Float32BufferAttribute([
        sL.x, 0.025, sL.z,
        sR.x, 0.025, sR.z,
        eL.x, 0.025, eL.z,
        eR.x, 0.025, eR.z
      ], 3));
      taperGeo.setIndex([0, 1, 2, 1, 3, 2]);
      taperGeo.computeVertexNormals();
      var taperMesh = new THREE.Mesh(taperGeo, _taperMat);
      taperMesh.receiveShadow = true;
      group.add(taperMesh);

      // (b) 三角导流岛：物理分隔两条匝道入口
      //     底边在 cluster 前方 3m 处，宽 1.6m
      //     顶点在 cluster 处（指向 incoming 方向）
      var islandFrontD = 4.0;
      var islandBackD = 0.5;
      var islandHalfW = 0.8;
      var frontX = jx + tx * islandFrontD;
      var frontZ = jz + tz * islandFrontD;
      var backX = jx - tx * islandBackD;
      var backZ = jz - tz * islandBackD;
      var iFL = { x: frontX + tnx * islandHalfW, z: frontZ + tnz * islandHalfW };
      var iFR = { x: frontX - tnx * islandHalfW, z: frontZ - tnz * islandHalfW };
      var iBk = { x: backX, z: backZ };
      var islandGeo = new THREE.BufferGeometry();
      islandGeo.setAttribute('position', new THREE.Float32BufferAttribute([
        iFL.x, 0.15, iFL.z,
        iFR.x, 0.15, iFR.z,
        iBk.x, 0.15, iBk.z
      ], 3));
      islandGeo.setIndex([0, 1, 2]);
      islandGeo.computeVertexNormals();
      var islandMesh = new THREE.Mesh(islandGeo, _islandMat);
      islandMesh.castShadow = true;
      islandMesh.receiveShadow = true;
      group.add(islandMesh);

      // (c) 两条匝道起点的短拓宽段：从 cluster（窄）到匝道全宽
      //     每条 connecting 在 cluster 处宽度 0.3m，前方 6m 处展开到匝道全宽。
      //     避免匝道"突然出现"在 cluster。
      for (var ci2 = 0; ci2 < connectingEdges.length; ci2++) {
        var ce = connectingEdges[ci2];
        if (!ce.curve) continue;
        var cTan;
        try { cTan = ce.curve.getTangentAt(0.0); }
        catch (eTan2) { cTan = new THREE.Vector3(1, 0, 0); }
        var ctx = cTan.x, ctz = cTan.z;
        var ctlen = Math.sqrt(ctx * ctx + ctz * ctz);
        if (ctlen < 0.001) { ctx = 1; ctz = 0; } else { ctx /= ctlen; ctz /= ctlen; }
        var cnx = -ctz, cnz = ctx;
        var cLW = ce.laneWidth || 3.0;
        var cHalfW = (ce.lanes * cLW) / 2;
        var rampTaperLen = 6.0;
        var cStartX = jx + ctx * 0.5;
        var cStartZ = jz + ctz * 0.5;
        var cEndX = jx + ctx * rampTaperLen;
        var cEndZ = jz + ctz * rampTaperLen;
        var csL = { x: cStartX + cnx * 0.3, z: cStartZ + cnz * 0.3 };
        var csR = { x: cStartX - cnx * 0.3, z: cStartZ - cnz * 0.3 };
        var ceL = { x: cEndX + cnx * cHalfW, z: cEndZ + cnz * cHalfW };
        var ceR = { x: cEndX - cnx * cHalfW, z: cEndZ - cnz * cHalfW };
        var rampTaperGeo = new THREE.BufferGeometry();
        rampTaperGeo.setAttribute('position', new THREE.Float32BufferAttribute([
          csL.x, 0.025, csL.z,
          csR.x, 0.025, csR.z,
          ceL.x, 0.025, ceL.z,
          ceR.x, 0.025, ceR.z
        ], 3));
        rampTaperGeo.setIndex([0, 1, 2, 1, 3, 2]);
        rampTaperGeo.computeVertexNormals();
        var rampTaperMesh = new THREE.Mesh(rampTaperGeo, _taperMat);
        rampTaperMesh.receiveShadow = true;
        group.add(rampTaperMesh);
      }
    }
  }

  // ── 高架桥墩 + 桥面 ──────────────────────────────────────
  // 用户要求："高架桥要有桥墩和桥面"。当某条 edge 的曲线采样点 pos.y > 0.5m
  // （路面高于地面）时，每隔 ~15m 生成一根圆柱桥墩从地面撑到路面底部，
  // 并在路面下方 0.2m 处铺一块略宽于路面的桥面板（连续 strip），形成完整
  // 的高架桥结构。匝道/汇入车道/高速主线高架段都会自动生成。
  var _PIER_SPACING = 15.0;
  var _PIER_MIN_H = 0.5;       // 低于此高度不立桥墩（贴地段）
  var _DECK_THICK = 0.30;      // 桥面厚度
  var _DECK_OVERHANG = 0.4;    // 桥面相对路面两侧外扩（含护栏空间）
  var pierGeos = [];           // 桥墩几何体（柱身 + 墩帽）
  var pierCapGeos = [];
  var deckGeos = [];           // 桥面板（每段一个薄 BoxGeometry）
  var pierShaftTpl = new THREE.CylinderGeometry(0.45, 0.6, 1.0, 12);
  var pierCapTpl = new THREE.BoxGeometry(1.6, 0.35, 1.6);
  for (var bi_e = 0; bi_e < _roadCurves.length; bi_e++) {
    var bCurve = _roadCurves[bi_e];
    var bLen = _roadCurveLens[bi_e] || bCurve.getLength();
    if (bLen < 1) continue;
    // edge lanes/laneWidth 通过 edgeEnds 找回
    var bEdge = edgeEnds[bi_e] || {};
    var bLanes = bEdge.lanes || 2;
    var bLW = bEdge.laneWidth || 3.5;
    var bHalfW = (bLanes * bLW) / 2;

    // ── 桥面板：从曲线 t=0..1 采样，每段生成一个梯形 strip（路面下方 0.2m）。
    //   当某段两端点 pos.y 都 < _PIER_MIN_H 时不生成（贴地段不需要桥面板）。
    var bNSeg = Math.max(4, Math.floor(bLen / SEG_LEN));
    for (var bs0 = 0; bs0 < bNSeg; bs0++) {
      var bt0 = bs0 / bNSeg;
      var bt1 = (bs0 + 1) / bNSeg;
      var bp0 = bCurve.getPointAt(bt0);
      var bp1 = bCurve.getPointAt(bt1);
      // 跳过贴地段（避免地面路上多一层板）
      if (bp0.y < _PIER_MIN_H && bp1.y < _PIER_MIN_H) continue;
      var bTan0 = bCurve.getTangentAt(bt0);
      var bTan1 = bCurve.getTangentAt(bt1);
      var bnx0 = -bTan0.z, bnz0 = bTan0.x;
      var bnx1 = -bTan1.z, bnz1 = bTan1.x;
      var bDeckY = (bp0.y + bp1.y) / 2 - _DECK_THICK / 2 - 0.18;  // 桥面顶距路面底 ~0.18m
      // 4 顶点梯形（0=前左, 1=前右, 2=后左, 3=后右；"前后"指 t0/t1）
      var deckW = bHalfW + _DECK_OVERHANG;
      var d0lx = bp0.x + bnx0 * deckW, d0lz = bp0.z + bnz0 * deckW;
      var d0rx = bp0.x - bnx0 * deckW, d0rz = bp0.z - bnz0 * deckW;
      var d1lx = bp1.x + bnx1 * deckW, d1lz = bp1.z + bnz1 * deckW;
      var d1rx = bp1.x - bnx1 * deckW, d1rz = bp1.z - bnz1 * deckW;
      // 桥面用 8 顶点 Box（顶面 + 底面 + 侧面），简化为只画顶面 + 前后端面（厚度感）
      var deckGeo = new THREE.BufferGeometry();
      // 8 顶点：顶面 4 + 底面 4
      var dYt = bDeckY + _DECK_THICK / 2;
      var dYb = bDeckY - _DECK_THICK / 2;
      deckGeo.setAttribute('position', new THREE.Float32BufferAttribute([
        d0lx, dYt, d0lz,  // 0 顶前左
        d0rx, dYt, d0rz,  // 1 顶前右
        d1lx, dYt, d1lz,  // 2 顶后左
        d1rx, dYt, d1rz,  // 3 顶后右
        d0lx, dYb, d0lz,  // 4 底前左
        d0rx, dYb, d0rz,  // 5 底前右
        d1lx, dYb, d1lz,  // 6 底后左
        d1rx, dYb, d1rz   // 7 底后右
      ], 3));
      // 12 三角形：顶面 + 底面 + 4 侧面
      deckGeo.setIndex([
        0, 2, 1,  1, 2, 3,   // 顶面
        4, 5, 6,  5, 7, 6,   // 底面
        0, 1, 5,  0, 5, 4,   // 前端面
        2, 7, 3,  2, 6, 7,   // 后端面
        0, 6, 2,  0, 4, 6,   // 左侧面
        1, 3, 7,  1, 7, 5    // 右侧面
      ]);
      deckGeo.computeVertexNormals();
      deckGeos.push(deckGeo);

      // ── 桥墩：每段中点立一根（间距约 SEG_LEN × 多段累加），
      //   为控制密度改为按累计弧长每 _PIER_SPACING 米立一根。
    }
    // 按累计弧长均匀布墩（间距 _PIER_SPACING 米），保证弯曲匝道上墩位合理
    var nPiers = Math.max(0, Math.floor(bLen / _PIER_SPACING) - 1);
    if (nPiers > 0) {
      var pierStep = bLen / (nPiers + 1);
      for (var pbi = 1; pbi <= nPiers; pbi++) {
        var pierS = pbi * pierStep;
        var pierT = pierS / bLen;
        if (pierT > 1) pierT = 1;
        var pp = bCurve.getPointAt(pierT);
        if (pp.y < _PIER_MIN_H) continue;  // 贴地无墩
        // 桥墩高度 = 路面高度 - 桥面厚度 - 墩帽高度 - 0.05 余量
        var pierTopY = pp.y - _DECK_THICK - 0.05;
        var pierCapH = 0.35;
        var pierShaftH = pierTopY - pierCapH;
        if (pierShaftH < 0.5) continue;
        // 柱身：CylinderGeometry(rTop=0.45, rBot=0.6, h=1) 居中于原点，
        // 缩放 Y=pierShaftH 后平移 Y=pierShaftH/2（底面贴地）。
        // Three.js 矩阵乘法右乘：M = T·S，故先 makeTranslation 再 multiply(S)。
        var pierMat = new THREE.Matrix4();
        pierMat.makeTranslation(pp.x, pierShaftH / 2, pp.z);
        pierMat.multiply(new THREE.Matrix4().makeScale(1, pierShaftH, 1));
        pierGeos.push(_transformGeometry(pierShaftTpl, pierMat));
        // 墩帽：Box 1.6×0.35×1.6，位于柱身顶部
        var capMat = new THREE.Matrix4().makeTranslation(pp.x, pierShaftH + pierCapH / 2, pp.z);
        pierCapGeos.push(_transformGeometry(pierCapTpl, capMat));
      }
    }
  }
  // 合并桥墩 + 墩帽为 1 个 mesh（混凝土材质）
  var pierMatConcrete = new THREE.MeshStandardMaterial({
    color: 0xb0b0b0, roughness: 0.85, metalness: 0.05
  });
  var allPierGeos = pierGeos.concat(pierCapGeos);
  if (allPierGeos.length) {
    var pierMesh = new THREE.Mesh(_mergeGeometries(allPierGeos), pierMatConcrete);
    pierMesh.castShadow = true;
    pierMesh.receiveShadow = true;
    group.add(pierMesh);
  }
  // 桥面板：暗灰色混凝土材质
  if (deckGeos.length) {
    var deckMesh = new THREE.Mesh(_mergeGeometries(deckGeos), pierMatConcrete);
    deckMesh.castShadow = true;
    deckMesh.receiveShadow = true;
    group.add(deckMesh);
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
 * 性能：不再直接 group.add，而是把顶点/索引/颜色追加到合并数组，
 * 所有 edge 的车道线最终合并为 1 个 vertex-colors mesh。
 *
 * @param {Array} laneMarkPos  位置合并数组
 * @param {Array} laneMarkIdx  索引合并数组
 * @param {Array} laneMarkCol  颜色合并数组
 * @param {number} laneMarkVertOffset  当前顶点偏移
 * @param {THREE.CatmullRomCurve3} curve  道路曲线
 * @param {number} nSeg  采样段数
 * @param {number} lateralOffset  横向偏移（相对道路中心线，Z 方向，标线中心）
 * @param {number} width  标线宽度（m）
 * @param {number} color  颜色
 * @param {number} y  Y 高度
 * @param {boolean} dashed  true=虚线（4.5m dash + 5.5m gap）
 * @return {number} 更新后的顶点偏移
 */
function _addLaneMarkRibbon(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, nSeg, lateralOffset, width, color, y, dashed) {
  var halfW = width / 2;
  var length = curve.getLength();

  if (dashed) {
    // 虚线：每段 dash 独立 ribbon，不连成整体（参考图：短线较密）
    var DASH = 3.0, GAP = 4.5, STEP = DASH + GAP;
    var nDash = Math.floor(length / STEP);
    for (var d = 0; d <= nDash; d++) {
      var s0 = d * STEP;
      var s1 = s0 + DASH;
      if (s1 > length) s1 = length;
      if (s0 >= length) break;
      var dashLen = s1 - s0;
      var dSeg = Math.max(2, Math.ceil(dashLen / 1.5));
      laneMarkVertOffset = _emitRibbonSegment(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, s0, s1, dSeg, length, lateralOffset, halfW, color, y);
    }
  } else {
    laneMarkVertOffset = _emitRibbonSegment(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, 0, length, nSeg, length, lateralOffset, halfW, color, y);
  }
  return laneMarkVertOffset;
}

/**
 * 根据当前道路网络曲线，查询 (x,z) 处最近的道路切线方向。
 * 已迁移至 scene3d/utils/CurveMath.js（getRoadTangentAt）。
 * 文件顶部的 _getRoadTangentAt(x,z) wrapper 保留无参签名，内部转发到 CurveMath。
 */

/** 沿曲线 [s0, s1] 段生成一条 ribbon 并追加到合并数组 */
function _emitRibbonSegment(laneMarkPos, laneMarkIdx, laneMarkCol, laneMarkVertOffset, curve, s0, s1, nSeg, totalLen, lateralOffset, halfW, color, y) {
  var r = ((color >> 16) & 0xff) / 255;
  var g = ((color >> 8) & 0xff) / 255;
  var b = (color & 0xff) / 255;
  var positions = [];
  var indices = [];
  var m = Math.max(2, nSeg);
  for (var si = 0; si <= m; si++) {
    var t = (s0 + (s1 - s0) * si / m) / totalLen;
    if (t > 1) t = 1;
    var pos = curve.getPointAt(t);
    var tan = curve.getTangentAt(t);
    var nx = -tan.z, nz = tan.x;
    // 左右边缘：沿法线偏移；Y 用 pos.y + y（y 是离路面的高度，高架段 pos.y > 0）
    positions.push(pos.x + nx * (lateralOffset - halfW), pos.y + y, pos.z + nz * (lateralOffset - halfW));
    positions.push(pos.x + nx * (lateralOffset + halfW), pos.y + y, pos.z + nz * (lateralOffset + halfW));
    if (si < m) {
      var base = si * 2;
      indices.push(base, base + 1, base + 2);
      indices.push(base + 1, base + 3, base + 2);
    }
  }
  // 追加到全局合并数组（vertex colors），draw call 从每段 1 次降到全部 1 次
  for (var pi = 0; pi < positions.length; pi += 3) {
    laneMarkPos.push(positions[pi], positions[pi + 1], positions[pi + 2]);
    laneMarkCol.push(r, g, b);
  }
  for (var ii = 0; ii < indices.length; ii++) {
    laneMarkIdx.push(indices[ii] + laneMarkVertOffset);
  }
  return laneMarkVertOffset + positions.length / 3;
}

/** 在道路指定 s 位置画一个贴地直行箭头（多个三角形拼接成箭头形状）。
 *  把每个箭头片变换后追加到 arrowGeos，由 _buildRoadNetwork 统一合并。 */
function _addRoadArrow(arrowGeos, curve, s, laneWidth, lanes) {
  var totalLen = curve.getLength();
  if (s >= totalLen) s = totalLen * 0.5;
  var t = s / totalLen;
  var pos = curve.getPointAt(t);
  var tan = curve.getTangentAt(t);
  var nx = -tan.z, nz = tan.x;
  var y = pos.y + 0.046;  // 叠加高架 elevation，避免高架段箭头悬浮在地面
  var heading = Math.atan2(tan.z, tan.x);
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
      var plane = new THREE.PlaneGeometry(sh.w, sh.l);
      var m = new THREE.Matrix4();
      m.makeRotationX(-Math.PI / 2);
      m.multiply(new THREE.Matrix4().makeRotationZ(-heading));
      m.setPosition(cx, y, cz);
      arrowGeos.push(_transformGeometry(plane, m));
    }
  }
}

/** 在道路指定 s 位置画一条横跨所有车道的停止线。
 *  把几何体追加到 stopLineGeos，由 _buildRoadNetwork 统一合并。 */
function _addStopLine(stopLineGeos, curve, s, laneWidth, lanes) {
  var totalLen = curve.getLength();
  if (s >= totalLen) s = 0;
  var t = s / totalLen;
  var pos = curve.getPointAt(t);
  var tan = curve.getTangentAt(t);
  var heading = Math.atan2(tan.z, tan.x);
  var lineGeo = new THREE.PlaneGeometry(0.35, lanes * laneWidth);
  var m = new THREE.Matrix4();
  m.makeRotationX(-Math.PI / 2);
  m.multiply(new THREE.Matrix4().makeRotationZ(-heading));
  m.setPosition(pos.x, pos.y + 0.047, pos.z);  // 叠加高架 elevation
  stopLineGeos.push(_transformGeometry(lineGeo, m));
}

// ── Environment: buildings + trees + street props ──
// ══════════════════════════════════════════════════════════════════════════════
// 远处城市天际线 — 低多边形建筑剪影环，~450m 半径，跟随 ego 居中
// 配合 scene.fog(140, 520) 产生大气透视，营造远景城市感
// ══════════════════════════════════════════════════════════════════════════════
function _buildSkyline(scene) {
  var grp = new THREE.Group();
  var T = THREE;

  // ── 远山（低多边形山脉环）──
  // 参考图：白色/浅蓝低多边形山脉，在 fog 中淡出
  var mountainMat = new T.MeshBasicMaterial({ color: 0xc8d8e8, fog: true, side: T.DoubleSide });
  var coneTpl = new T.ConeGeometry(1, 1, 5);
  var mountainGeos = [];
  var N = 48;
  for (var i = 0; i < N; i++) {
    var ang = (i / N) * Math.PI * 2 + (Math.random() - 0.5) * 0.12;
    var dist = 400 + Math.random() * 120;
    var mx = Math.cos(ang) * dist;
    var mz = Math.sin(ang) * dist;
    var mw = 35 + Math.random() * 55;
    var mh = 30 + Math.random() * 70;
    var matM = new T.Matrix4().makeTranslation(mx, mh / 2 - 8, mz);
    matM.scale(new T.Vector3(mw, mh, mw * (0.6 + Math.random() * 0.6)));
    matM.multiply(new T.Matrix4().makeRotationY(Math.random() * Math.PI));
    mountainGeos.push(_transformGeometry(coneTpl, matM));
  }
  if (mountainGeos.length) {
    var mountainMesh = new T.Mesh(_mergeGeometries(mountainGeos), mountainMat);
    mountainMesh.frustumCulled = true;
    grp.add(mountainMesh);
  }

  // ── 扁平白云（参考图风格）──
  var cloudMat = new T.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.75, fog: true, depthWrite: false });
  var cloudGeos = [];
  var boxTpl = new T.BoxGeometry(1, 1, 1);
  for (var c = 0; c < 18; c++) {
    var cx = (Math.random() - 0.5) * 700;
    var cz = (Math.random() - 0.5) * 500;
    var cy = 55 + Math.random() * 35;
    var cw = 25 + Math.random() * 35;
    var cd = 12 + Math.random() * 18;
    var ch = 3 + Math.random() * 4;
    var matC = new T.Matrix4().makeTranslation(cx, cy, cz);
    matC.scale(new T.Vector3(cw, ch, cd));
    cloudGeos.push(_transformGeometry(boxTpl, matC));
    // 再给每朵云加 1~2 个小块，避免太规则
    var nPuffs = 1 + Math.floor(Math.random() * 2);
    for (var cp = 0; cp < nPuffs; cp++) {
      var pw = cw * (0.4 + Math.random() * 0.4);
      var pd = cd * (0.4 + Math.random() * 0.4);
      var pmat = new T.Matrix4().makeTranslation(
        cx + (Math.random() - 0.5) * cw * 0.7,
        cy + (Math.random() - 0.5) * ch,
        cz + (Math.random() - 0.5) * cd * 0.7
      );
      pmat.scale(new T.Vector3(pw, ch * (0.7 + Math.random() * 0.5), pd));
      cloudGeos.push(_transformGeometry(boxTpl, pmat));
    }
  }
  if (cloudGeos.length) {
    var cloudMesh = new T.Mesh(_mergeGeometries(cloudGeos), cloudMat);
    cloudMesh.frustumCulled = true;
    grp.add(cloudMesh);
  }

  scene.add(grp);
  _skylineGroup = grp;
}

// ══════════════════════════════════════════════════════════════════════════════
// B.1: 雨粒子系统 — 1200 条半透明线段围绕 ego 下落，营造阴雨天气氛围。
// low 性能档位不创建，避免低端设备帧率下降。
// ══════════════════════════════════════════════════════════════════════════════
function _buildRain(scene) {
  if (_perfTier === 'low' || _rainMesh) return;
  var count = _RAIN_COUNT;
  var positions = new Float32Array(count * 6); // 2 vertices × 3 coords per streak
  var velocities = new Float32Array(count);
  for (var i = 0; i < count; i++) {
    var x = (Math.random() - 0.5) * _RAIN_BOX;
    var z = (Math.random() - 0.5) * _RAIN_BOX;
    var y = Math.random() * _RAIN_HEIGHT;
    var len = 0.6 + Math.random() * 0.8;
    positions[i * 6]     = x; positions[i * 6 + 1] = y;       positions[i * 6 + 2] = z;
    positions[i * 6 + 3] = x; positions[i * 6 + 4] = y - len; positions[i * 6 + 5] = z;
    velocities[i] = 12 + Math.random() * 10;
  }
  var geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  var mat = new THREE.LineBasicMaterial({
    color: 0xaaccdd, transparent: true, opacity: 0.35, depthWrite: false
  });
  _rainMesh = new THREE.LineSegments(geo, mat);
  _rainMesh.frustumCulled = false; // 始终跟随 ego 渲染
  scene.add(_rainMesh);
  _rainPos = positions;
  _rainVel = velocities;
}

/** 每帧更新雨粒子位置：下落、触底后重置到 ego 周围空中 */
function _updateRain(sx, sz) {
  if (!_rainMesh || !_rainPos) return;
  // low 档位隐藏雨效；档位切回时恢复显示
  if (_perfTier === 'low') {
    if (_rainMesh.visible) _rainMesh.visible = false;
    return;
  }
  if (!_rainMesh.visible) _rainMesh.visible = true;
  var count = _RAIN_COUNT;
  var dt = 0.016; // ~60fps
  for (var i = 0; i < count; i++) {
    var len = _rainPos[i * 6 + 1] - _rainPos[i * 6 + 4];
    var y = _rainPos[i * 6 + 1] - _rainVel[i] * dt;
    if (y < 0) {
      var nx = sx + (Math.random() - 0.5) * _RAIN_BOX;
      var nz = sz + (Math.random() - 0.5) * _RAIN_BOX;
      var ny = 20 + Math.random() * (_RAIN_HEIGHT - 20);
      len = 0.6 + Math.random() * 0.8;
      _rainPos[i * 6]     = nx; _rainPos[i * 6 + 2] = nz;
      _rainPos[i * 6 + 3] = nx; _rainPos[i * 6 + 5] = nz;
      _rainPos[i * 6 + 1] = ny;
      _rainPos[i * 6 + 4] = ny - len;
    } else {
      _rainPos[i * 6 + 1] = y;
      _rainPos[i * 6 + 4] = y - len;
    }
  }
  _rainMesh.geometry.attributes.position.needsUpdate = true;
}

// ══════════════════════════════════════════════════════════════════════════════
// B.1: 程序化水面 —— 低洼水域，顶点波动 + 菲涅尔反射，跟随 ego 居中渲染
// low 档位跳过以保帧率；medium/high 共用同一 mesh，仅细分不同。
// ══════════════════════════════════════════════════════════════════════════════
function _buildWater(scene) {
  if (_waterMesh || _perfTier === 'low') return;
  var segs = (_perfTier === 'medium') ? 50 : _WATER_SEGS;
  var geo = new THREE.PlaneGeometry(_WATER_SIZE, _WATER_SIZE, segs, segs);
  geo.rotateX(-Math.PI / 2);

  var fogCol = scene.fog ? scene.fog.color : new THREE.Color(0xb8d4e8);
  _waterMat = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    side: THREE.DoubleSide,
    uniforms: {
      uTime:   { value: 0 },
      uColor:  { value: new THREE.Color(0x2a4a5a) },
      uDeep:   { value: new THREE.Color(0x0f2230) },
      uFoam:   { value: new THREE.Color(0xcceeff) },
      uFogCol: { value: fogCol },
      uFogNear:{ value: scene.fog ? scene.fog.near : 140 },
      uFogFar: { value: scene.fog ? scene.fog.far : 520 }
    },
    vertexShader: [
      'uniform float uTime;',
      'varying vec2 vUv;',
      'varying float vElevation;',
      'varying vec3 vWorldPos;',
      'void main() {',
      '  vUv = uv;',
      '  vec3 pos = position;',
      '  float w1 = sin(pos.x * 0.08 + uTime * 1.2) * 0.08;',
      '  float w2 = sin(pos.z * 0.06 + uTime * 0.9) * 0.07;',
      '  float w3 = sin((pos.x + pos.z) * 0.12 + uTime * 1.8) * 0.04;',
      '  pos.y += w1 + w2 + w3;',
      '  vElevation = w1 + w2 + w3;',
      '  vWorldPos = (modelMatrix * vec4(pos, 1.0)).xyz;',
      '  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 1.0);',
      '}'
    ].join('\n'),
    fragmentShader: [
      'uniform vec3 uColor;',
      'uniform vec3 uDeep;',
      'uniform vec3 uFoam;',
      'uniform vec3 uFogCol;',
      'uniform float uFogNear;',
      'uniform float uFogFar;',
      'varying vec2 vUv;',
      'varying float vElevation;',
      'varying vec3 vWorldPos;',
      'void main() {',
      '  // 基础深浅渐变',
      '  vec3 col = mix(uDeep, uColor, 0.55 + 0.45 * sin(vUv.x * 12.0 + vUv.y * 8.0));',
      '  // 浪尖泡沫',
      '  float foam = smoothstep(0.10, 0.18, vElevation);',
      '  col = mix(col, uFoam, foam * 0.35);',
      '  // 简单菲涅尔：视角越平行水面越反光',
      '  vec3 viewDir = normalize(cameraPosition - vWorldPos);',
      '  float fresnel = pow(1.0 - abs(dot(viewDir, vec3(0.0, 1.0, 0.0))), 2.5);',
      '  col = mix(col, uFoam, fresnel * 0.25);',
      '  // 雾融合',
      '  float depth = length(vWorldPos - cameraPosition);',
      '  float fogFactor = clamp((depth - uFogNear) / (uFogFar - uFogNear), 0.0, 1.0);',
      '  col = mix(col, uFogCol, fogFactor);',
      '  gl_FragColor = vec4(col, 0.82 + fresnel * 0.10);',
      '}'
    ].join('\n')
  });

  _waterMesh = new THREE.Mesh(geo, _waterMat);
  _waterMesh.position.set(0, -0.25, 0);     // 略低于路面，模拟路边低洼水域
  _waterMesh.receiveShadow = true;
  _waterMesh.frustumCulled = false;
  scene.add(_waterMesh);
}

/** 每帧更新水面波动时间 uniform；low 档位隐藏水面 */
function _updateWater() {
  if (!_waterMesh || !_waterMat) return;
  if (_perfTier === 'low') {
    if (_waterMesh.visible) _waterMesh.visible = false;
    return;
  }
  if (!_waterMesh.visible) _waterMesh.visible = true;
  _waterMat.uniforms.uTime.value = _animT;
}

/**
 * _mergeGeometries / _transformGeometry 已迁移至 scene3d/utils/GeometryMerge.js
 * （mergeGeometries / transformGeometry），通过顶部 import 别名引入。
 */

function _buildEnvironment(scene) {
  var env = new THREE.Group();
  var T = THREE;

  // 共享模板几何体（只创建一次）
  var boxTpl = new T.BoxGeometry(1, 1, 1);
  var cylTpl = new T.CylinderGeometry(1, 1, 1, 10);
  var coneTpl = new T.ConeGeometry(1, 1, 8);  // 锥形树冠（参考图）
  var dodecTpl = new T.DodecahedronGeometry(1, 0);
  var planeTpl = new T.PlaneGeometry(1, 1);

  // 按材质分组的待合并几何体列表
  var bldGeos = [];      // 建筑主体
  var winGeos = [];      // 窗户
  var trunkGeos = [];    // 树干
  var leafGeos = [];     // 树冠
  var propGeos = [];     // 路灯杆/臂
  var lampGeos = [];     // 路灯灯罩
  var trashGeos = [];    // 垃圾桶
  var hydrantGeos = [];  // 消防栓
  var bushGeos = [];     // B.3: 低矮灌木丛
  var rockGeos = [];     // B.3: 路边石块/土堆

  // Buildings
  for (var b = 0; b < 14; b++) {
    var side = (b % 2 === 0) ? 1 : -1;
    var bx = (b - 7) * 180 + Math.random() * 70;
    var bw = 6 + Math.random() * 8;
    var bz = side * (26 + Math.random() * 28);
    var bh = 12 + Math.random() * 42;
    var depth = 6 + Math.random() * 8;

    var matB = new T.Matrix4().makeTranslation(bx, bh / 2, bz);
    matB.scale(new T.Vector3(bw, bh, depth));
    bldGeos.push(_transformGeometry(boxTpl, matB));

    // 屋顶女儿墙
    var matP = new T.Matrix4().makeTranslation(bx, bh + 0.4, bz);
    matP.scale(new T.Vector3(bw * 0.9, 0.8, depth * 0.9));
    bldGeos.push(_transformGeometry(boxTpl, matP));

    // 低层裙楼（50%概率）
    if (Math.random() > 0.5) {
      var podiumH = 3 + Math.random() * 3;
      var matPod = new T.Matrix4().makeTranslation(bx, podiumH / 2, bz);
      matPod.scale(new T.Vector3(bw + 2, podiumH, depth + 2));
      bldGeos.push(_transformGeometry(boxTpl, matPod));
    }

    // 窗户（面向道路的一面）
    var floors = Math.max(3, Math.floor(bh / 3.2));
    var cols = Math.max(2, Math.floor(bw / 2.5));
    var winW = (bw - 1) / cols * 0.55;
    var winH = 1.2;
    for (var fi = 0; fi < floors; fi++) {
      for (var cj = 0; cj < cols; cj++) {
        var wx = bx - (bw - 1) / 2 + (cj + 0.5) * ((bw - 1) / cols);
        var wy = 1.8 + fi * 3.2;
        var faceZ = bz + side * (depth / 2 + 0.06);
        var matW = new T.Matrix4().makeTranslation(wx, wy, faceZ);
        matW.scale(new T.Vector3(winW, winH, 1));
        if (side > 0) matW.multiply(new T.Matrix4().makeRotationY(Math.PI));
        winGeos.push(_transformGeometry(planeTpl, matW));
      }
    }
  }

  // Trees — 城市绿化锥形树，沿道路两侧排列，参考图风格
  var nTrees = 64;
  for (var t = 0; t < nTrees; t++) {
    var tx = (t - nTrees / 2) * 55 + (Math.random() - 0.5) * 25;
    var tside = (t % 2 === 0 ? 1 : -1);
    // 近路侧 12~20m，远路侧 20~40m，避免遮挡车道
    var tzBase = (t % 4 < 2) ? 12 : 26;
    var tz = tside * (tzBase + Math.random() * 14);
    var th = 3.5 + Math.random() * 2.5;     // 树干高
    var trunkR = 0.16 + Math.random() * 0.06;
    var crownH = 3.5 + Math.random() * 2.0; // 树冠高
    var crownR = 1.4 + Math.random() * 0.8; // 树冠底半径

    // 树干（圆柱，圆锥默认底在 y=0，需平移使底面接地）
    var matTrunk = new T.Matrix4().makeTranslation(tx, th / 2, tz);
    matTrunk.scale(new T.Vector3(trunkR, th, trunkR));
    trunkGeos.push(_transformGeometry(cylTpl, matTrunk));

    // 树冠（圆锥）— 参考图三角形/锥形树
    var matLeaf = new T.Matrix4().makeTranslation(tx, th + crownH / 2 - 0.15, tz);
    matLeaf.scale(new T.Vector3(crownR, crownH, crownR));
    leafGeos.push(_transformGeometry(coneTpl, matLeaf));
  }

  // Street lamps — 城市道路双侧路灯，杆+悬臂+扁圆柱灯罩
  var nLamps = 40;
  for (var p = 0; p < nLamps; p++) {
    var px = (p - nLamps / 2) * 65 + (Math.random() - 0.5) * 15;
    var pside = (p % 2 === 0) ? 1 : -1;
    var pz = pside * (10.5 + Math.random() * 2.5);
    var poleH = 7.5 + Math.random() * 1.0;
    var poleR = 0.10 + Math.random() * 0.03;

    // 灯杆
    var matPole = new T.Matrix4().makeTranslation(px, poleH / 2, pz);
    matPole.scale(new T.Vector3(poleR, poleH, poleR));
    propGeos.push(_transformGeometry(cylTpl, matPole));

    // 悬臂（伸向道路中心）
    var armLen = 1.6 + Math.random() * 0.4;
    var armY = poleH - 0.35;
    var matArm = new T.Matrix4().makeTranslation(px - pside * armLen / 2, armY, pz);
    matArm.scale(new T.Vector3(armLen, 0.07, 0.07));
    propGeos.push(_transformGeometry(boxTpl, matArm));

    // 灯罩：扁圆柱，朝下
    var lampX = px - pside * armLen;
    var matLamp = new T.Matrix4().makeTranslation(lampX, armY - 0.04, pz);
    matLamp.scale(new T.Vector3(0.28, 0.08, 0.18));
    lampGeos.push(_transformGeometry(cylTpl, matLamp));

    // 灯罩下小反光片/LED 面板
    var matLampFace = new T.Matrix4().makeTranslation(lampX, armY - 0.09, pz);
    matLampFace.scale(new T.Vector3(0.22, 0.02, 0.14));
    lampGeos.push(_transformGeometry(boxTpl, matLampFace));

    if (Math.random() > 0.6) {
      var matTrash = new T.Matrix4().makeTranslation(px + pside * 0.9, 0.35, pz + (Math.random() - 0.5) * 0.5);
      matTrash.scale(new T.Vector3(0.25, 0.7, 0.22));
      trashGeos.push(_transformGeometry(cylTpl, matTrash));
    }
  }

  // B.3: 路边灌木丛 —— 树木与建筑之间的中景植被，合并为 1 个 mesh
  for (var bu = 0; bu < 38; bu++) {
    var bxu = (bu - 19) * 90 + Math.random() * 50;
    var bside = (bu % 2 === 0) ? 1 : -1;
    var bzu = bside * (22 + Math.random() * 28);
    var nBush = 2 + Math.floor(Math.random() * 3);
    for (var bb = 0; bb < nBush; bb++) {
      var r = 0.5 + Math.random() * 0.7;
      var matBu = new T.Matrix4().makeTranslation(
        bxu + (Math.random() - 0.5) * 1.6,
        r * 0.55,
        bzu + (Math.random() - 0.5) * 1.6
      );
      matBu.scale(new T.Vector3(r, r * (0.6 + Math.random() * 0.4), r));
      bushGeos.push(_transformGeometry(dodecTpl, matBu));
    }
  }

  // B.3: 零散石块/土堆，丰富地面细节
  for (var rk = 0; rk < 24; rk++) {
    var rx = (rk - 12) * 110 + Math.random() * 60;
    var rside = (rk % 2 === 0) ? 1 : -1;
    var rz = rside * (30 + Math.random() * 40);
    var matR = new T.Matrix4().makeTranslation(rx, 0.15 + Math.random() * 0.15, rz);
    matR.scale(new T.Vector3(0.4 + Math.random() * 0.6, 0.25 + Math.random() * 0.35, 0.4 + Math.random() * 0.6));
    // 随机轻微旋转，避免过于规则
    matR.multiply(new T.Matrix4().makeRotationY(Math.random() * Math.PI));
    rockGeos.push(_transformGeometry(dodecTpl, matR));
  }

  // 合并并创建 mesh
  var bldMat = new T.MeshStandardMaterial({ color: 0x4a5868, roughness: 0.85 });
  var winMat = new T.MeshStandardMaterial({ color: 0x88aacc, metalness: 0.6, roughness: 0.15, emissive: 0x223344, emissiveIntensity: 0.25, side: T.DoubleSide });
  var trunkMat = new T.MeshStandardMaterial({ color: 0x6b4423, roughness: 0.9 });
  var leafMat = new T.MeshStandardMaterial({ color: 0x2d5a27, roughness: 0.9 });
  var propMat = new T.MeshStandardMaterial({ color: 0x444444, metalness: 0.4, roughness: 0.55 });
  var lampMat = new T.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffee, emissiveIntensity: 0.35 });
  var trashMat = new T.MeshStandardMaterial({ color: 0x446644, roughness: 0.7 });
  var hydrantMat = new T.MeshStandardMaterial({ color: 0xaa2222, roughness: 0.5 });
  var bushMat = new T.MeshStandardMaterial({ color: 0x3a6b2e, roughness: 0.95 });
  var rockMat = new T.MeshStandardMaterial({ color: 0x6e6a62, roughness: 0.9 });

  function addMerged(geos, mat, castShadow, receiveShadow) {
    if (!geos.length) return;
    var mesh = new T.Mesh(_mergeGeometries(geos), mat);
    if (castShadow) mesh.castShadow = true;
    if (receiveShadow) mesh.receiveShadow = true;
    env.add(mesh);
  }

  addMerged(bldGeos, bldMat, true, true);
  addMerged(winGeos, winMat, false, false);
  addMerged(trunkGeos, trunkMat, true, false);
  addMerged(leafGeos, leafMat, true, false);
  addMerged(propGeos, propMat, true, false);
  addMerged(lampGeos, lampMat, false, false);
  addMerged(trashGeos, trashMat, true, false);
  addMerged(hydrantGeos, hydrantMat, true, false);
  addMerged(bushGeos, bushMat, true, false);
  addMerged(rockGeos, rockMat, true, false);

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
  var el = document.getElementById("scene3d");
  if (!el) return;
  if (typeof THREE === "undefined") {
    _show3DError("THREE.js not loaded — check /tools/three.min.js");
    return;
  }
  if (!THREE.WebGLRenderer) {
    _show3DError("WebGL not available — your browser or GPU may not support it");
    return;
  }
  // 立即隐藏 placeholder
  var msgEl = document.getElementById("scene3d-msg");
  if (msgEl) msgEl.style.display = "none";

  try {
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
  // 旧 composer 引用了旧 renderer，必须重置否则恢复后仍尝试用 dead context 渲染。
  _composer = null;
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
  var skyTop = new THREE.Color(0x3a8fd8);    /* 天顶明亮蓝 */
  var skyHorizon = new THREE.Color(0xd6eafa);/* 地平线近白 */
  scene3d.background = skyHorizon;
  scene3d.fog = new THREE.Fog(0xd6eafa, 180, 650);
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

  // B.2: 太阳光晕 billboard —— 位于主光源方向，径向渐变模拟大气散射
  var sunMat = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    uniforms: {
      uColor: { value: new THREE.Color(0xfff4e0) },
      uGlow:  { value: new THREE.Color(0xffaa55) }
    },
    vertexShader: [
      'varying vec2 vUv;',
      'void main() {',
      '  vUv = uv;',
      '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
      '}'
    ].join('\n'),
    fragmentShader: [
      'uniform vec3 uColor;',
      'uniform vec3 uGlow;',
      'varying vec2 vUv;',
      'void main() {',
      '  vec2 c = vUv - 0.5;',
      '  float r = length(c);',
      '  float core = smoothstep(0.12, 0.04, r);',
      '  float halo = smoothstep(0.45, 0.15, r) * 0.45;',
      '  vec3 col = mix(uGlow, uColor, core);',
      '  float alpha = core + halo;',
      '  gl_FragColor = vec4(col, alpha);',
      '}'
    ].join('\n')
  });
  var sunSprite = new THREE.Sprite(sunMat);
  sunSprite.scale.set(36, 36, 1);
  sunSprite.position.copy(new THREE.Vector3(30, 50, 20).normalize().multiplyScalar(420));
  scene3d.add(sunSprite);

  // Camera — chase cam. Wider FOV keeps the whole car visible on small screens.
  camera3d = new THREE.PerspectiveCamera(60, w / h, 0.2, 500);

  // Renderer
  // 根据 URL 参数 ?perf=low 或屏幕尺寸选择初始性能档位，避免低端设备开局卡死。
  var urlParams = new URLSearchParams(window.location.search);
  var initPerf = urlParams.get('perf');
  if (initPerf === 'low' || initPerf === 'medium' || initPerf === 'high') {
    _perfTier = initPerf;
  } else if (window.innerWidth < 700) {
    _perfTier = 'medium';
  }
  // WebGL 创建：无 GPU/headless/安全策略导致失败时，优雅降级到 2D canvas
  // 而不是让用户看到空白/等待。
  try {
    renderer3d = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  } catch (glErr) {
    console.warn('[scene3d] WebGL renderer creation failed, falling back to 2D:', glErr);
    reportDiag('scene3d', 'WebGL unavailable — falling back to 2D scene');
    try { init2DFallback(); } catch (_) {}
    _show3DError('WebGL unavailable — switched to 2D fallback. Browser or GPU may not support WebGL.');
    return;
  }
  renderer3d.setSize(w, h);
  renderer3d.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer3d.toneMapping = THREE.ACESFilmicToneMapping;
  renderer3d.toneMappingExposure = 1.1;
  // 开启阴影渲染：物体 castShadow/receiveShadow 才会真正产生阴影，
  // 之前 sun.castShadow=true 但 renderer 未启用 shadowMap，阴影被静默丢弃。
  // low 档位默认关闭阴影以提升帧率。
  renderer3d.shadowMap.enabled = (_perfTier !== 'low');
  renderer3d.shadowMap.type = THREE.PCFSoftShadowMap;
  el.appendChild(renderer3d.domElement);

  // C.1: 绑定轨道/点击交互（renderer 创建后 canvas 才存在）
  CameraController.initCameraControls(renderer3d.domElement, { pickNPC: _pickNPC });

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
  scene3d.add(new THREE.AmbientLight(0xaabbdd, 0.20));
  var sun = new THREE.DirectionalLight(0xfff8ee, 1.20);
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
  scene3d.add(new THREE.HemisphereLight(0xaabbdd, 0x554433, 0.3));

  // ── Ground (large flat plane, matches road length) ──
  /* NOA Phase 6 3D 增强：程序化草地纹理（CanvasTexture），替代纯色地面。
   * 256×256 canvas 画基底绿 + 随机深浅斑点模拟草地肌理，repeat 16×8 铺满 800×400。 */
  var gndGeo = new THREE.PlaneGeometry(800, 400);
  var gndCanvas = document.createElement('canvas');
  gndCanvas.width = 256; gndCanvas.height = 256;
  var gctx = gndCanvas.getContext('2d');
  // 城市绿化草坪：明亮低饱和绿色（参考图风格）
  gctx.fillStyle = '#5a8a4a'; gctx.fillRect(0, 0, 256, 256);
  for (var gi = 0; gi < 2200; gi++) {
    var gx = Math.random() * 256, gy = Math.random() * 256;
    var shade = 20 + Math.floor(Math.random() * 45);
    gctx.fillStyle = 'rgba(' + (65 + shade) + ',' + (110 + shade) + ',' + (55 + shade * 0.5) + ',0.45)';
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

  // ── Road (MVC View: RoadView.js) ──
  _roadGroup = buildLegacyRoad(scene3d);
  _lastCurveKey = '';     // prime curve key; _applyRoadCurve will deform when data arrives
  _curveActive = false;

  // ── Environment ──
  _buildEnvironment(scene3d);
  _buildSkyline(scene3d);
  _buildWater(scene3d);
  _buildRain(scene3d);

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
  _obsTilt.reset();
  _egoTilt.reset();
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
        0.9,   // strength：车漆/车灯/路面反射的光晕强度（极品飞车风格光泽感）
        0.5,   // radius：光晕扩散半径
        0.6    // threshold：降低阈值让更多高光参与 Bloom（车漆反射/玻璃反光都发光）
      );
      _composer.addPass(bloomPass);
      // SMAA 抗锯齿（若 CDN 加载了 SMAAPass）；EffectComposer 链路会绕过 WebGLRenderer
      // 的硬件 MSAA，必须显式加 AA pass 否则画面锯齿严重。
      if (THREE.SMAAPass) {
        try {
          var smaaPass = new THREE.SMAAPass(w, h);
          smaaPass.renderToScreen = true;
          _composer.addPass(smaaPass);
        } catch (smaaErr) {
          console.warn('[scene3d] SMAA setup failed:', smaaErr);
        }
      }
      var copyPass = new THREE.ShaderPass(THREE.CopyShader);
      copyPass.renderToScreen = !THREE.SMAAPass;  // 若有 SMAA 则 SMAA 是最后输出
      _composer.addPass(copyPass);
    } catch (bloomErr) {
      console.warn('[scene3d] Bloom setup failed, falling back to direct render:', bloomErr);
      _composer = null;
    }
  } else if (!window._bloomUnavailable) {
    // 脚本可能异步未加载完，但通常同步 script 标签已执行；兜底降级
    window._bloomUnavailable = true;
  }

  // ── Performance-based adaptive quality ──
  // 每 _perfCheckInterval 帧统计 FPS，低于阈值时自动降级：
  // high   -> 全开（Bloom + 阴影 + 街灯 PointLight）
  // medium -> 关闭 Bloom，保留阴影
  // low    -> 关闭 Bloom + 阴影，减少 uniform 重传
  function _updatePerfTier() {
    _perfFrameCount++;
    var now = performance.now();
    if (_perfFrameCount >= _perfCheckInterval) {
      var elapsed = (now - _perfLastTime) / 1000;
      var fps = elapsed > 0 ? _perfFrameCount / elapsed : 60;
      _perfFrameCount = 0;
      _perfLastTime = now;
      var newTier = _perfTier;
      if (fps < 18) newTier = 'low';
      else if (fps < 28) newTier = 'medium';
      else if (fps > 35 && _perfTier === 'low') newTier = 'medium';
      else if (fps > 45 && _perfTier === 'medium') newTier = 'high';
      if (newTier !== _perfTier) {
        _perfTier = newTier;
        if (renderer3d) {
          renderer3d.shadowMap.enabled = (_perfTier !== 'low');
        }
        console.log('[scene3d] perf tier changed to', _perfTier, '(fps=', fps.toFixed(1), ')');
      }
    }
  }

  // ── Animation loop ──
  function anim3D() {
    requestAnimationFrame(anim3D);
    if (!sceneReady) return;
    if (_glLost) return;               // skip while WebGL context is lost
    if (!renderer3d || !scene3d || !camera3d) return;
    var ok = safeCall('scene3d.frame', function() {
      _renderFrame();
      _updatePerfTier();
      // low/medium 关闭 Bloom；high 且 composer 存在才用 Bloom。
      if (_composer && _perfTier === 'high') {
        _composer.render();
      } else {
        renderer3d.render(scene3d, camera3d);
      }
    });
    if (ok) {
      _frameFailCount = 0;
    } else {
      _frameFailCount++;
      var bad = _auditSceneMaterials(scene3d);
      if (bad.length) {
        console.error('[scene3d.materials] invalid Color-typed properties found:', bad);
        var byProp = {};
        bad.forEach(function(f) { byProp[f.prop] = (byProp[f.prop] || 0) + 1; });
        var summary = Object.keys(byProp).map(function(p) { return p + '×' + byProp[p]; }).join(', ');
        reportDiag('scene3d.materials', 'invalid Color property on ' + bad.length + ' material(s): ' + summary);
      }
      if (_frameFailCount >= _FRAME_FAIL_THRESHOLD && _perfTier === 'high') {
        setPerfTier('medium');
        reportDiag('scene3d.autoDowngrade', 'disabled Bloom after ' + _frameFailCount + ' consecutive frame failures');
      }
    }
  }
  anim3D();
  sceneReady = true;

  } catch (initErr) {
    _show3DError(initErr.message || String(initErr));
  }
  // Phase 4.9: scene3d / camera3d / renderer3d stay module-scoped;
  // debug access flows through explicit window.flowboard debug exports
  // (e.g. _auditMaterials in app.js), not a direct _scene3d reference.
}

/** 在 3D 视图区域显示错误信息，方便诊断 */
function _show3DError(msg) {
  _3DInitFailed = true;
  var el = document.getElementById("scene3d-msg");
  if (!el) return;
  el.style.display = "";
  el.style.color = "#f0a0a0";
  el.setAttribute('data-init-error', '1');
  el.innerHTML = '<div style="font-size:40px;margin-bottom:10px">!</div>' +
    '<div style="color:#f0a0a0;font-size:14px;font-weight:600;margin-bottom:8px">3D Init Failed</div>' +
    '<div style="color:#f08888;font-size:11px;font-family:monospace;line-height:1.5;max-width:380px;word-break:break-all">' +
    (msg || 'unknown error') + '</div>';
  if (window.console && console.error) console.error('[scene3d]', msg);
}

/**
 * 手动设置 3D 性能档位。可用于 UI 开关或测试：
 *   'high'   = Bloom + 阴影全开
 *   'medium' = 无 Bloom，保留阴影
 *   'low'    = 无 Bloom，无阴影
 */
function setPerfTier(tier) {
  if (tier !== 'high' && tier !== 'medium' && tier !== 'low') return;
  _perfTier = tier;
  if (renderer3d) {
    renderer3d.shadowMap.enabled = (_perfTier !== 'low');
  }
  // B.1: 水体/雨效跟随档位显隐；切到 low 时彻底释放水面几何（重建时按档位细分）
  if (_waterMesh) {
    _waterMesh.visible = (_perfTier !== 'low');
  }
  if (_rainMesh) {
    _rainMesh.visible = (_perfTier !== 'low');
  }
}

/** C.1: 切换相机模式；index.html 的视角按钮调用此函数。
 *  实际状态由 MVC Controller 层 CameraController.js 维护。 */
function setCameraMode(mode) {
  CameraController.setCameraMode(mode, {
    onResetSmoothCamera: function() {
      if (!_cam || !_camTarget) return;
      var sx = _dr.smoothX, sz = _dr.smoothZ;
      _cam.set(sx - 15, 5, sz);
      _camLook.set(sx + 8, 0.5, sz);
      _camTarget.set(sx - 15, 5, sz);
      _camLookTarget.set(sx + 8, 0.5, sz);
    }
  });
}

/** C.1: 重置为默认 chase 视角 */
function resetCamera() {
  CameraController.resetCamera({
    onResetSmoothCamera: function() {
      if (!_cam || !_camTarget) return;
      var sx = _dr.smoothX, sz = _dr.smoothZ;
      _cam.set(sx - 15, 5, sz);
      _camLook.set(sx + 8, 0.5, sz);
      _camTarget.set(sx - 15, 5, sz);
      _camLookTarget.set(sx + 8, 0.5, sz);
    }
  });
}

/** C.2: Raycast 检测点击的 NPC，命中后弹出信息面板。
 *  鼠标坐标与 Raycaster 由 CameraController.initCameraControls 在点击时传入。 */
function _pickNPC(mouse, raycaster) {
  if (!raycaster || !camera3d || !_obsPool) return;
  raycaster.setFromCamera(mouse, camera3d);
  // 收集所有可见障碍物的子 mesh
  var targets = [];
  for (var oi = 0; oi < _obsPool.length; oi++) {
    var om = _obsPool[oi];
    if (om && om.visible) targets.push(om);
  }
  if (!targets.length) return;
  var hits = raycaster.intersectObjects(targets, true);
  if (!hits.length) return;
  // 找到命中 mesh 所属的最顶层 obs group（_obsPool 都是 scene3d 的直接子对象）
  var obj = hits[0].object;
  while (obj.parent && obj.parent !== scene3d) {
    obj = obj.parent;
  }
  var idx = _obsPool.indexOf(obj);
  if (idx >= 0) _showNPCDetail(idx);
}

/** C.2: 显示/更新 NPC 信息面板 */
function _showNPCDetail(idx) {
  var ow = _obsWorld[idx];
  if (!ow) return;
  var panel = _getNPCPanel();
  var spd = Math.sqrt((ow.vx || 0) * (ow.vx || 0) + (ow.vz || 0) * (ow.vz || 0));
  var dist = 0;
  if (_carGroup) {
    var dx = ow.x - _carGroup.position.x, dz = ow.z - _carGroup.position.z;
    dist = Math.sqrt(dx * dx + dz * dz);
  }
  var typeName = { car: '轿车', suv: 'SUV', truck: '卡车', pedestrian: '行人', cyclist: '骑行者', cone: '锥桶' }[ow.type] || ow.type;
  var aiName = { cruise: '巡航', follow: '跟车', stop: '停止', stop_for_tl: '等红灯', etc_approach: 'ETC 接近', branch_sel: '选道', merge: '汇入', yield: '让行' }[ow.ai] || (ow.ai || '—');
  panel.innerHTML =
    '<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">' +
      '<h3 style="color:#58a6ff;font-size:14px;margin:0">' + typeName + ' #' + (ow.id || idx) + '</h3>' +
      '<button onclick="flowboard.closeNPCDetail()" class="sm" style="color:#f85149">✕</button>' +
    '</div>' +
    '<div style="color:#484f58;font-size:11px;line-height:1.7">' +
      '<div><span style="color:#8b949e">类型:</span> ' + ow.type + '</div>' +
      '<div><span style="color:#8b949e">速度:</span> <span style="color:#3fb950;font-weight:600">' + spd.toFixed(2) + '</span> m/s</div>' +
      '<div><span style="color:#8b949e">距离:</span> <span style="color:#d29922;font-weight:600">' + dist.toFixed(1) + '</span> m</div>' +
      '<div><span style="color:#8b949e">尺寸:</span> ' + (ow.len || 4).toFixed(1) + ' × ' + (ow.wid || 2).toFixed(1) + ' m</div>' +
      '<div><span style="color:#8b949e">AI 状态:</span> <span style="color:#bc8cff">' + aiName + '</span></div>' +
      '<div><span style="color:#8b949e">航向:</span> ' + ((ow.heading || 0) * 180 / Math.PI).toFixed(1) + '°</div>' +
    '</div>';
  panel.style.display = '';
}

/** C.2: 获取/创建 NPC 详情面板 DOM */
function _getNPCPanel() {
  if (_npcPanel) return _npcPanel;
  var el = document.getElementById('npc-detail-panel');
  if (el) { _npcPanel = el; return el; }
  el = document.createElement('div');
  el.id = 'npc-detail-panel';
  el.style.cssText = 'display:none;position:fixed;right:20px;bottom:20px;width:240px;z-index:9000;padding:12px;border-radius:10px;background:#161b22;border:1px solid #252d3a;box-shadow:0 8px 24px rgba(0,0,0,0.5);';
  document.body.appendChild(el);
  _npcPanel = el;
  return el;
}

/** C.2: 关闭 NPC 信息面板（暴露给 app.js / window.flowboard） */
function closeNPCDetail() {
  if (_npcPanel) _npcPanel.style.display = 'none';
}

// ══════════════════════════════════════════════════════════════════════════════
// Per-frame render: world-space scene, chase camera
// Body tilt (roll/pitch) 视觉物理近似已迁移到 MVC Model 层：
// scene3d/model/VisualPhysics.js
// 车辆模型前向为局部 +X，roll（转弯外倾）用 rotation.x，
// pitch（刹车点头/加速抬头）用 rotation.z，yaw 用 rotation.y。
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
    // 高架 elevation：ego Y 叠加当前道路高度，避免高架段车沉到桥下。
    // ego.position.y 下方会被 Car bounce 覆盖（基于 elevation + sin 抖动）。
    var _egoElev = _getRoadElevationAt(sx, sz);
    ego.position.set(sx, _egoElev, sz);
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
    // MVC Model: 车身侧倾/俯仰由 VisualPhysics 计算，View 层只读结果。
    _egoTilt.updateTargets(now, _dr.smoothHeading, _dr.smoothSpeed);
    var egoTilt = _egoTilt.tick();
    ego.rotation.x = egoTilt.roll;
    ego.rotation.z = egoTilt.pitch;
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
  // B.1: 雨粒子跟随 ego 下落
  _updateRain(sx, sz);
  // B.1: 水面波动动画
  _updateWater();
  // B.1: 水面始终跟随 ego 水平居中，避免边缘穿帮
  if (_waterMesh) { _waterMesh.position.x = sx; _waterMesh.position.z = sz; }

  // ── Camera: chase / top / orbit / driver / front ──
  var narrow = (renderer3d && renderer3d.domElement && renderer3d.domElement.clientWidth < 700);
  var dc = _debugCam;
  if (dc) {
    // debug-cam override (debug3d.html) — keep legacy behaviour
    var back   = dc.back   || (narrow ? 20 : 15);
    var height = dc.height || (narrow ? 6.5 : 5.0);
    var side   = dc.side || 0;
    var camLerp= dc.lerp   || 0.08;
    _camTarget.set(sx - back, height, sz + side);
    _camLookTarget.set(sx + (narrow ? 10 : 8), 0.5, sz);
    _cam.lerp(_camTarget, camLerp);
    _camLook.lerp(_camLookTarget, camLerp);
    camera3d.position.copy(_cam);
    camera3d.lookAt(_camLook);
  } else {
    var _camMode = CameraController.getMode();
    var _orbitState = CameraController.getOrbitState();
    if (_camMode === 'top') {
      camera3d.position.set(sx, 85, sz);
      camera3d.lookAt(sx + 5, 0, sz);
    } else if (_camMode === 'driver') {
      // 驾驶员视角：车顶略后方，看向前方道路
      var dh = -_dr.smoothHeading;
      var dcos = Math.cos(dh), dsin = Math.sin(dh);
      camera3d.position.set(sx - 0.8 * dcos, 1.55, sz - 0.8 * dsin);
      camera3d.lookAt(sx + 20 * dcos, 0.8, sz + 20 * dsin);
    } else if (_camMode === 'front') {
      // 前保险杠视角
      var fh = -_dr.smoothHeading;
      var fcos = Math.cos(fh), fsin = Math.sin(fh);
      camera3d.position.set(sx + 2.0 * fcos, 0.7, sz + 2.0 * fsin);
      camera3d.lookAt(sx + 25 * fcos, 1.0, sz + 25 * fsin);
    } else if (_camMode === 'orbit') {
      // 自由轨道：鼠标拖拽旋转，滚轮缩放
      _orbitState.target.set(sx, 0, sz);
      var ox = _orbitState.distance * Math.sin(_orbitState.polar) * Math.cos(_orbitState.azimuth);
      var oy = _orbitState.distance * Math.cos(_orbitState.polar);
      var oz = _orbitState.distance * Math.sin(_orbitState.polar) * Math.sin(_orbitState.azimuth);
      camera3d.position.set(sx + ox, Math.max(1.5, oy), sz + oz);
      camera3d.lookAt(_orbitState.target);
    } else {
      // chase (default) — 按 ego 航向投影，弯道里相机跟车顺弯
      var backC   = narrow ? 20 : 15;
      var heightC = narrow ? 6.5 : 5.0;
      var lookFwd = narrow ? 10 : 8;
      var chH = -_dr.smoothHeading;
      var ccos = Math.cos(chH), csin = Math.sin(chH);
      _camTarget.set(sx - backC * ccos, heightC, sz - backC * csin);
      _camLookTarget.set(sx + lookFwd * ccos, 0.5, sz + lookFwd * csin);
      _cam.lerp(_camTarget, 0.08);
      _camLook.lerp(_camLookTarget, 0.08);
      camera3d.position.copy(_cam);
      camera3d.lookAt(_camLook);
    }
  }

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
        var L = ow.len || _OBS_L[ow.type] || DEFAULT_OBS_LEN, W = ow.wid || _OBS_W[ow.type] || 2;
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
        // A.3: 距离裁剪 — 按性能档位隐藏远距离 NPC，减少 draw call/顶点/阴影开销。
        // high=250m, medium=180m, low=120m。追逐视角下更远的目标对驾驶决策无意义。
        var MAX_OBS_DIST = (_perfTier === 'low') ? 120 : (_perfTier === 'medium') ? 180 : 250;
        if (distToEgo > MAX_OBS_DIST) {
          om.visible = false;
          if (lbl) lbl.visible = false;
          continue;
        }
        // Unit-normalised model: group origin at road surface (y=0.05).
        // Children sit at fractions 0-1 so scale.set maps directly to metres.
        // 高架 elevation：NPC Y 叠加道路高度，避免高架段 NPC 沉到桥下。
        var _obsElev = _getRoadElevationAt(tx, tz);
        _tmpV3.set(tx, _obsElev + 0.05, tz);
        if (!om.visible || om.position.distanceTo(_tmpV3) > 35) {
          om.position.copy(_tmpV3);
        } else {
          om.position.lerp(_tmpV3, 0.18);
        }
        // 悬挂起伏：复用 ego 的固定频率正弦效果（见下方 "Car bounce"），
        // 按 oi 错开相位避免所有 NPC 同步起伏，之前 NPC 完全没有这个效果。
        var obsBounceSpd = Math.sqrt((ow.vx || 0) * (ow.vx || 0) + (ow.vz || 0) * (ow.vz || 0));
        om.position.y = _obsElev + 0.05 + Math.sin(_animT * 6.5 + oi * 1.7) * 0.008 * Math.min(1, obsBounceSpd * 0.12);
        // realScale 标记：glTF 模型和真实尺寸建模的 _buildObstacle 都跳过 (L,H,W) 缩放；
        // pedestrian/cone 等仍是 unit 模型，需 scale.set(L,H,W) 映射到真实尺寸
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

        // MVC Model: 障碍物 roll/pitch 由 VisualPhysics 计算。
        // 与 ego 一致：喂平滑后的 yaw（-om.rotation.y）而非后端阶跃航向 ow.heading，
        // 否则 yawRate=阶跃差/帧dt 被放大 → roll 尖峰 → 车身猛晃。
        var obsSpdNow = Math.sqrt((ow.vx || 0) * (ow.vx || 0) + (ow.vz || 0) * (ow.vz || 0));
        var ti = _obsTilt.get(oi);
        ti.updateTargets(now, -om.rotation.y, obsSpdNow);
        var obsTilt = ti.tick();
        om.rotation.x = obsTilt.roll;
        om.rotation.z = obsTilt.pitch;

        var c = (_obsColors[ow.type]) || 0xff9944;
        // Defect 5: 障碍物类型变化时重建外形（轿车 ↔ 胶囊行人 ↔ 圆锥路障），
        // 复用同一 group 槽位，只替换 children，保留 position/scale/visible。
        if (om.userData.obsType !== ow.type) {
          var nm = buildObstacleGroup(ow.type, c) || _buildObstacle(ow.type, c);
          // 清理旧 mesh 的 GPU 资源，避免类型频繁切换时内存泄漏。
          while (om.children.length) {
            var oldChild = om.children[0];
            om.remove(oldChild);
            if (oldChild.traverse) {
              oldChild.traverse(function(ch) {
                if (ch.geometry) ch.geometry.dispose();
                if (ch.material) {
                  if (Array.isArray(ch.material)) {
                    ch.material.forEach(function(m) { m.dispose(); });
                  } else {
                    ch.material.dispose();
                  }
                }
              });
            }
          }
          while (nm.children.length) om.add(nm.children[0]);
          om.userData.obsType = ow.type;
          om.userData.obsColor = c;
        }
        // 颜色只在变化时 setHex，避免每帧触发 uniform 重传。
        if (om.userData.obsColor !== c && om.children.length > 0 &&
            om.children[0].material && om.children[0].material.color) {
          om.children[0].material.color.setHex(c);
          om.userData.obsColor = c;
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
            // 标签位置：障碍物位置 + 车顶上方（叠加 elevation 跟随高架段）
            lbl.position.set(om.position.x, om.position.y + H + 1.2, om.position.z);
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
        tlm.position.set(tlw.x, _getRoadElevationAt(tlw.x, tlw.z), tlw.z);
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
        gm.position.set(gw.x, _getRoadElevationAt(gw.x, gw.z), gw.z);
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
        // 多车道收费广场：userData.booms 是 4 个栏杆数组，全部同步抬起
        var boom = gm.userData.boom;
        if (boom) {
          var targetAngle = (gw.progress || 0) * 1.31;
          boom.rotation.y += (targetAngle - boom.rotation.y) * 0.15;
        }
        var booms = gm.userData.booms;
        if (booms && booms.length) {
          var targetAngleArr = (gw.progress || 0) * 1.31;
          for (var bi = 0; bi < booms.length; bi++) {
            if (booms[bi]) {
              booms[bi].rotation.y += (targetAngleArr - booms[bi].rotation.y) * 0.15;
            }
          }
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
  // 悬挂起伏叠加在道路 elevation 之上，避免高架段车抖到桥面下方。
  if (ego && v.speed > 0.5) {
    var _egoElev2 = _getRoadElevationAt(ego.position.x, ego.position.z);
    ego.position.y = _egoElev2 + 0.05 + Math.sin(_animT * 6.5) * 0.008 * Math.min(1, v.speed * 0.12);
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

export { init3DScene, resize3D, update3D, sceneReady, scene3d, _renderFrame, _applyRoadCurve, setCameraMode, resetCamera, closeNPCDetail, setPerfTier };
