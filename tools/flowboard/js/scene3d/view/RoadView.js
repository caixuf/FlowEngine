/**
 * RoadView.js — MVC View 层：旧版直路道路构建与曲线变形
 *
 * 负责生成 2 车道 × 3.5m 的静态直路（沥青 + 路肩 + 双黄线），
 * 以及根据 scene.frame.road 的 curve 参数对道路做平滑弯曲变形。
 *
 * 多段道路网络（road_network.edges）的构建保留在 scene3d.js 的
 * _buildRoadNetwork 中，因其与 _roadCurves / _roadCurveLens / _roadCurveNext
 * 等 Model 状态耦合较深，下一轮再拆。
 */

import { curveShiftAt, curveHeadingAt } from '../utils/CurveMath.js';

const THREE = window.THREE;

/**
 * 构建旧版直路：2 车道 × 3.5m，2m 一段分组，便于后续弯曲变形。
 * @param {THREE.Scene} scene
 * @returns {THREE.Group} roadGroup
 */
export function buildLegacyRoad(scene) {
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
  var aMat = new THREE.MeshStandardMaterial({ color: 0x7a7d80, roughness: 0.82 });
  var eGeo = new THREE.BoxGeometry(SEG_LEN, 0.02, 0.28, 1, 1, 1);
  var eMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x333333, roughness: 0.3 });
  var sGeo = new THREE.BoxGeometry(SEG_LEN, 0.04, SHLD_W, 1, 1, 1);
  var sMat = new THREE.MeshStandardMaterial({ color: 0x5a4d40, roughness: 1.0 });

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
    // applyRoadCurve 按 baseZ + curveShift 变形；不存 baseZ 会得到
    // undefined + number = NaN，中心双黄线一弯道就消失。
    cl.userData.baseZ = dy * 0.15;
    roadGroup.add(cl);
  }

  scene.add(roadGroup);
  return roadGroup;
}

/**
 * 对旧版直路应用平滑弯曲。
 * @param {THREE.Group|null} group
 * @param {Object} roadData  { curve_start_x, curve_length_m, curve_offset_m }
 * @returns {boolean} 是否实际发生了弯曲
 */
export function applyRoadCurve(group, roadData) {
  if (!group || !roadData) return false;
  var sx = roadData.curve_start_x || 0, len = roadData.curve_length_m || 0;
  var off = roadData.curve_offset_m || 0;
  if (len <= 0 || Math.abs(off) < 0.01) return false;

  group.traverse(function(child) {
    if (child.userData && child.userData.isRoadSeg) {
      var bx = child.userData.baseX;
      child.position.z = curveShiftAt(bx, sx, len, off);
      // 绕 Y 轴旋转使 segment 跟随道路切线方向，消除折线感
      child.rotation.y = curveHeadingAt(bx, sx, len, off);
      return;
    }
    if (child.userData && child.userData.isLaneMark) {
      child.position.z = child.userData.baseZ + curveShiftAt(child.position.x, sx, len, off);
      return;
    }
  });
  return true;
}
