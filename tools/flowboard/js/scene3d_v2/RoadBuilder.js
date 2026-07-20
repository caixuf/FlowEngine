/**
 * RoadBuilder.js — v2 道路网络构建
 *
 * 从 scene/frame.road_network.edges 构建：
 *   - 路面 mesh（合并为 1 个 draw call，CanvasTexture 沥青）
 *   - 车道线 mesh（中心黄线 + 车道白线，合并为 1 个 vertex-colors mesh）
 *
 * 砍掉的部件（v2 不做）：
 *   - 护栏 / 路灯 / 路面箭头 / 停止线 / 水坑 / 路口补丁
 *   - 高架桥墩 / 桥栏矮墙
 *   - ETC 门架 / 红绿灯
 *
 * 坐标系：scene/frame 的 nodes[ni] = [x, y, z]
 *   - x = 前向（forward）
 *   - y = lateral（侧向，Three.js 的 z）
 *   - z = elevation（高度，Three.js 的 y 向上）
 * 所以 THREE.Vector3(x, z, y) 把 [x,y,z] 映射到 [forward, up, lateral]。
 */
const THREE = window.THREE;

// ── 沥青纹理（生成一次，所有 edge 共享）──────────────────────
let _asphaltTex = null;
function getAsphaltTexture() {
  if (_asphaltTex) return _asphaltTex;
  var c = document.createElement('canvas');
  c.width = 256; c.height = 256;
  var ctx = c.getContext('2d');
  // 中灰沥青基色（v1 修复后提亮，与绿色草地有明显对比）
  ctx.fillStyle = '#4a4d52';
  ctx.fillRect(0, 0, 256, 256);
  // 骨料颗粒：随机深浅斑点模拟碎石
  for (var i = 0; i < 1800; i++) {
    var shade = Math.floor(Math.random() * 22) - 11;
    var v = 74 + shade;  // 74 = #4a 的十进制
    ctx.fillStyle = 'rgb(' + v + ',' + (v + 3) + ',' + (v + 5) + ')';
    var x = Math.random() * 256, y = Math.random() * 256;
    var r = Math.random() * 1.5 + 0.5;
    ctx.fillRect(x, y, r, r);
  }
  _asphaltTex = new THREE.CanvasTexture(c);
  _asphaltTex.wrapS = THREE.RepeatWrapping;
  _asphaltTex.wrapT = THREE.RepeatWrapping;
  _asphaltTex.repeat.set(2, 2);
  return _asphaltTex;
}

/**
 * 从 edges 构建道路网络 group。
 * @param {Array} edges  scene/frame.road_network.edges
 * @param {THREE.Scene} scene  挂载目标
 * @returns {THREE.Group} 道路 group（已添加到 scene）
 */
export function buildRoadNetwork(edges, scene) {
  if (!edges || !edges.length || !scene) return null;

  // 缓存键：edges id+nodes 数量拼接，未变化跳过重建
  var key = '';
  for (var i = 0; i < edges.length; i++) {
    key += (edges[i].id || i) + ':' + (edges[i].nodes ? edges[i].nodes.length : 0) + ',';
  }

  var store = scene.userData._v2store;
  if (key === store.getLastRoadKey() && store.getRoadGroup()) {
    return store.getRoadGroup();
  }
  store.setLastRoadKey(key);

  // 清除旧 group
  var oldGroup = store.getRoadGroup();
  if (oldGroup) {
    scene.remove(oldGroup);
    oldGroup.traverse(function(child) {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach(function(m) { m.dispose(); });
        else child.material.dispose();
      }
    });
  }

  var group = new THREE.Group();
  group.name = 'roadNetwork';

  // 路面顶点/索引/UV 累积（合并为 1 个 mesh）
  var roadPos = [], roadIdx = [], roadUV = [];
  var roadVertOffset = 0;
  // 车道线顶点/索引/颜色（vertex-colors 合并为 1 个 mesh）
  var lanePos = [], laneIdx = [], laneCol = [];
  var laneVertOffset = 0;

  var curves = [];
  var curveLens = [];

  var SEG_LEN = 3;  // 3m 一段，平衡精度与性能

  for (var ei = 0; ei < edges.length; ei++) {
    var edge = edges[ei];
    var nodes = edge.nodes;
    if (!nodes || nodes.length < 2) continue;

    // nodes[ni] = [x, y, z]：z 是 elevation（高架高度），平地场景为 0。
    // THREE.Vector3(x, z, y) 映射：x=前向, y=向上(elevation), z=侧向(lateral)
    var points = [];
    for (var ni = 0; ni < nodes.length; ni++) {
      var elev = (nodes[ni].length >= 3) ? (nodes[ni][2] || 0) : 0;
      points.push(new THREE.Vector3(nodes[ni][0], elev, nodes[ni][1]));
    }
    var curve = new THREE.CatmullRomCurve3(points);
    curves.push(curve);

    var length = edge.length || curve.getLength();
    curveLens.push(length);

    var lanes = edge.lanes || 2;
    var laneWidth = edge.lane_width || 3.5;
    var isOneway = !!edge.oneway;
    var lanesPerSide = isOneway ? lanes : (lanes / 2);
    var halfWidth = isOneway
      ? (lanes * laneWidth * 0.5 + 0.6)
      : (lanesPerSide * laneWidth + 0.3);

    var nSeg = Math.max(4, Math.ceil(length / SEG_LEN));

    // ── 路面 ribbon（沿 curve 两侧偏移 halfWidth 构成四边形带）──
    for (var s = 0; s < nSeg; s++) {
      var t0 = s / nSeg;
      var t1 = (s + 1) / nSeg;
      var p0 = curve.getPointAt(t0);
      var p1 = curve.getPointAt(t1);
      var tan0 = curve.getTangentAt(t0);
      var tan1 = curve.getTangentAt(t1);
      // 法线 = 切线在 XZ 平面旋转 90°
      var nx0 = -tan0.z, nz0 = tan0.x;
      var nx1 = -tan1.z, nz1 = tan1.x;

      // 四个顶点：左前/右前/左后/右后（"前" = t0，"后" = t1）
      roadPos.push(
        p0.x + nx0 * halfWidth, p0.y, p0.z + nz0 * halfWidth,  // 0: 左前
        p0.x - nx0 * halfWidth, p0.y, p0.z - nz0 * halfWidth,  // 1: 右前
        p1.x + nx1 * halfWidth, p1.y, p1.z + nz1 * halfWidth,  // 2: 左后
        p1.x - nx1 * halfWidth, p1.y, p1.z - nz1 * halfWidth   // 3: 右后
      );
      // UV：沿道路长度方向重复纹理（每 10m 一次），横向 0~1
      var u0 = (s * SEG_LEN) / 10;
      var u1 = ((s + 1) * SEG_LEN) / 10;
      roadUV.push(u0, 0,  u0, 1,  u1, 0,  u1, 1);
      // 两个三角形：(0,2,1) + (1,2,3)
      roadIdx.push(
        roadVertOffset,     roadVertOffset + 2, roadVertOffset + 1,
        roadVertOffset + 1, roadVertOffset + 2, roadVertOffset + 3
      );
      roadVertOffset += 4;
    }

    // ── 车道线 ──
    // 双向道路画中心黄线（实线）+ 车道分界白线（虚线）。
    // 单向道路（oneway）只画车道分界白线，不画中心黄线。
    if (!isOneway) {
      // 中心黄线（实线，双向道路）
      addLaneRibbon(lanePos, laneIdx, laneCol, curve, nSeg, 0, 0.15, [1.0, 0.85, 0.0], 0.02);
    }
    // 车道分界白线（虚线，每个车道边界）
    for (var li = 1; li < lanesPerSide; li++) {
      var offset = li * laneWidth;
      addDashedLaneRibbon(lanePos, laneIdx, laneCol, curve, nSeg, offset, 0.12, [1, 1, 1], 0.02);
      addDashedLaneRibbon(lanePos, laneIdx, laneCol, curve, nSeg, -offset, 0.12, [1, 1, 1], 0.02);
    }
    // 路缘实线（最外侧白线）
    addLaneRibbon(lanePos, laneIdx, laneCol, curve, nSeg, halfWidth - 0.1, 0.10, [1, 1, 1], 0.02);
    addLaneRibbon(lanePos, laneIdx, laneCol, curve, nSeg, -(halfWidth - 0.1), 0.10, [1, 1, 1], 0.02);
  }

  // ── 路面 mesh ──
  if (roadPos.length) {
    var roadGeo = new THREE.BufferGeometry();
    roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(roadPos, 3));
    roadGeo.setAttribute('uv', new THREE.Float32BufferAttribute(roadUV, 2));
    roadGeo.setIndex(roadIdx);
    roadGeo.computeVertexNormals();
    var roadMat = new THREE.MeshStandardMaterial({
      map: getAsphaltTexture(),
      color: 0xffffff,
      roughness: 0.95,
      metalness: 0.0
    });
    var roadMesh = new THREE.Mesh(roadGeo, roadMat);
    roadMesh.name = 'roadSurface';
    group.add(roadMesh);
  }

  // ── 车道线 mesh ──
  if (lanePos.length) {
    var laneGeo = new THREE.BufferGeometry();
    laneGeo.setAttribute('position', new THREE.Float32BufferAttribute(lanePos, 3));
    laneGeo.setAttribute('color', new THREE.Float32BufferAttribute(laneCol, 3));
    laneGeo.setIndex(laneIdx);
    laneGeo.computeVertexNormals();
    var laneMat = new THREE.MeshBasicMaterial({ vertexColors: true });
    var laneMesh = new THREE.Mesh(laneGeo, laneMat);
    laneMesh.name = 'laneMarks';
    group.add(laneMesh);
  }

  scene.add(group);
  store.setRoadGroup(group);
  store.setRoadCurves(curves);
  store.setRoadCurveLens(curveLens);
  // 路网 bbox：Map/Orbit 相机锚定用
  store.setRoadBBox(new THREE.Box3().setFromObject(group));

  return group;
}

/**
 * 添加一条实线车道线 ribbon（沿 curve 横向偏移 lateralOffset）。
 * color 是 [r,g,b]，0~1。yOffset 防止 z-fighting（贴路面上方 0.02m）。
 */
function addLaneRibbon(pos, idx, col, curve, nSeg, lateralOffset, width, color, yOffset) {
  var halfW = width / 2;
  for (var s = 0; s < nSeg; s++) {
    var t0 = s / nSeg;
    var t1 = (s + 1) / nSeg;
    var p0 = curve.getPointAt(t0);
    var p1 = curve.getPointAt(t1);
    var tan0 = curve.getTangentAt(t0);
    var tan1 = curve.getTangentAt(t1);
    var nx0 = -tan0.z, nz0 = tan0.x;
    var nx1 = -tan1.z, nz1 = tan1.x;

    pos.push(
      p0.x + nx0 * (lateralOffset + halfW), p0.y + yOffset, p0.z + nz0 * (lateralOffset + halfW),
      p0.x + nx0 * (lateralOffset - halfW), p0.y + yOffset, p0.z + nz0 * (lateralOffset - halfW),
      p1.x + nx1 * (lateralOffset + halfW), p1.y + yOffset, p1.z + nz1 * (lateralOffset + halfW),
      p1.x + nx1 * (lateralOffset - halfW), p1.y + yOffset, p1.z + nz1 * (lateralOffset - halfW)
    );
    var base = pos.length / 3 - 4;
    idx.push(base, base + 2, base + 1,  base, base + 1, base + 3);
    // 4 顶点同色
    for (var k = 0; k < 4; k++) col.push(color[0], color[1], color[2]);
  }
}

/**
 * 添加一条虚线车道线 ribbon（每 6m 一段 + 3m 间隔）。
 */
function addDashedLaneRibbon(pos, idx, col, curve, nSeg, lateralOffset, width, color, yOffset) {
  var halfW = width / 2;
  // 用 nSeg 段，每 3 段画 2 段空 1 段（dash 周期约 9m @ SEG_LEN=3）
  for (var s = 0; s < nSeg; s++) {
    if (s % 3 === 2) continue;  // 跳过第 3 段，形成虚线
    var t0 = s / nSeg;
    var t1 = (s + 1) / nSeg;
    var p0 = curve.getPointAt(t0);
    var p1 = curve.getPointAt(t1);
    var tan0 = curve.getTangentAt(t0);
    var tan1 = curve.getTangentAt(t1);
    var nx0 = -tan0.z, nz0 = tan0.x;
    var nx1 = -tan1.z, nz1 = tan1.x;

    pos.push(
      p0.x + nx0 * (lateralOffset + halfW), p0.y + yOffset, p0.z + nz0 * (lateralOffset + halfW),
      p0.x + nx0 * (lateralOffset - halfW), p0.y + yOffset, p0.z + nz0 * (lateralOffset - halfW),
      p1.x + nx1 * (lateralOffset + halfW), p1.y + yOffset, p1.z + nz1 * (lateralOffset + halfW),
      p1.x + nx1 * (lateralOffset - halfW), p1.y + yOffset, p1.z + nz1 * (lateralOffset - halfW)
    );
    var base = pos.length / 3 - 4;
    idx.push(base, base + 2, base + 1,  base, base + 1, base + 3);
    for (var k = 0; k < 4; k++) col.push(color[0], color[1], color[2]);
  }
}

/**
 * 查询某 x,z 位置的道路高度（elevation）。
 * v2 简化：找最近 curve，投影取 y。失败返回 0。
 */
export function getRoadElevationAt(x, z, curves) {
  if (!curves || !curves.length) return 0;
  var bestD2 = Infinity;
  var bestY = 0;
  for (var i = 0; i < curves.length; i++) {
    var pts = curves[i].getSpacedPoints(40);
    for (var j = 0; j < pts.length; j++) {
      var dx = pts[j].x - x;
      var dz = pts[j].z - z;
      var d2 = dx * dx + dz * dz;
      if (d2 < bestD2) {
        bestD2 = d2;
        bestY = pts[j].y;
      }
    }
  }
  return bestY;
}
