/**
 * ConstructionView.js — 施工区域障碍物和指示牌
 *
 * 在道路尽头位置设置符合交通规范的施工区域，包含：
 * - 反光材质施工围栏（橙白条纹，横跨整个路面）
 * - 交通锥（缓冲区域，带白色反光带）
 * - "前方施工"指示牌（黄底黑字，放置于围栏前方）
 * - "禁止通行"指示牌（红底白字，放置于围栏上）
 *
 * 布局（ENU 坐标）：
 *   道路末端 → 施工区域（最后30m）→ 围栏（5m缓冲）→ 指示牌（5m前）
 *
 * 路网数据由 sampleEdgeNodes 完成 ENU→THREE 转换，
 * 本视图直接使用 THREE 坐标放置物体。
 *
 * build(roadNetwork) 在路网变化时重建（静态布局，无 update 循环）。
 */

import { LANE_WIDTH, DEFAULT_LANES } from '../core/Constants.js';

// ── 施工区域参数 ──
const ROAD_END_BUFFER = 30;       // 道路末端最后30m为施工区域
const BARRIER_BUFFER = 5;         // 缓冲带距离施工区域起点(m)
const BARRIER_H = 1.8;            // 围栏高度
const BARRIER_T = 0.25;           // 围栏厚度
const BARRIER_SEG_W = 2.0;        // 每段围栏宽度
const CONE_H = 0.7;               // 交通锥高度
const CONE_RADIUS = 0.25;         // 交通锥底半径
const CONE_SPACING = 3.0;         // 交通锥间距
const SIGN_POST_H = 2.8;          // 指示牌立柱高度
const SIGN_W = 1.2;               // 指示牌宽度
const SIGN_H = 0.9;               // 指示牌高度
const SIGN_POST_RADIUS = 0.06;    // 指示牌立柱半径

// ── 颜色 ──
const COLOR_CONE_ORANGE = 0xff5500;
const COLOR_CONE_BAND = 0xffffff;
const COLOR_POST = 0x888888;

// ── 缓存纹理 ──
let _barrierTex = null;
let _signNoEntryTex = null;       // 禁止通行
let _signConstructionTex = null;  // 前方施工

/** 创建围栏橙白条纹纹理 */
function _makeBarrierTexture() {
  if (_barrierTex) return _barrierTex;
  const c = document.createElement('canvas');
  c.width = 64;
  c.height = 256;
  const ctx = c.getContext('2d');
  const h = 32;
  for (let i = 0; i < 8; i++) {
    ctx.fillStyle = i % 2 === 0 ? '#ff6600' : '#f0f0f0';
    ctx.fillRect(0, i * h, c.width, h);
  }
  // 反光渐变覆盖
  const g = ctx.createLinearGradient(0, 0, c.width, 0);
  g.addColorStop(0, 'rgba(255,255,255,0.15)');
  g.addColorStop(0.5, 'rgba(255,255,255,0.0)');
  g.addColorStop(1, 'rgba(255,255,255,0.25)');
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, c.width, c.height);
  _barrierTex = new THREE.CanvasTexture(c);
  _barrierTex.wrapS = _barrierTex.wrapT = THREE.RepeatWrapping;
  return _barrierTex;
}

/** 创建指示牌文字纹理 */
function _makeSignTexture(text, bg, fg, border) {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 192;
  const ctx = c.getContext('2d');

  // 背景色
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, c.width, c.height);

  // 边框
  ctx.strokeStyle = border;
  ctx.lineWidth = 5;
  ctx.strokeRect(3, 3, c.width - 6, c.height - 6);

  // 文字
  ctx.fillStyle = fg;
  ctx.font = `bold 54px 'Microsoft YaHei','SimHei','Noto Sans SC','PingFang SC',sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, c.width / 2, c.height / 2);

  return new THREE.CanvasTexture(c);
}

function _getSignTexture(type) {
  if (type === 'no_entry') {
    if (!_signNoEntryTex) _signNoEntryTex = _makeSignTexture('禁止通行', '#cc0000', '#ffffff', '#ffffff');
    return _signNoEntryTex;
  }
  if (type === 'construction') {
    if (!_signConstructionTex) _signConstructionTex = _makeSignTexture('前方施工', '#ffcc00', '#000000', '#000000');
    return _signConstructionTex;
  }
  return null;
}

export function createConstructionView(scene) {
  const group = new THREE.Group();
  group.name = 'constructionZone';
  scene.add(group);

  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) {
        const tex = c.material.map;
        if (tex && tex !== _barrierTex && tex !== _signNoEntryTex && tex !== _signConstructionTex) {
          tex.dispose();
        }
        c.material.dispose();
      }
    }
  }

  function build(roadNetwork) {
    clear();
    if (!roadNetwork || !roadNetwork.edges || roadNetwork.edges.length === 0) return;

    // 找最长 edge 确定道路终点
    let maxLen = 0, edge = null;
    for (const e of roadNetwork.edges) {
      const len = e.length || e.length_m || 0;
      if (len > maxLen) { maxLen = len; edge = e; }
    }
    if (!edge || maxLen < 100) return;

    const lanes = edge.lanes || DEFAULT_LANES;
    const laneWidth = edge.lane_width || LANE_WIDTH;
    const halfW = (lanes * laneWidth) / 2;

    // 道路终点在 ENU (maxLen, 0, 0) → THREE (maxLen, 0, 0)
    const roadEndX = maxLen;

    // 施工区域从 roadEndX - ROAD_END_BUFFER 开始
    const constructionStartX = roadEndX - ROAD_END_BUFFER;
    const barrierX = constructionStartX - BARRIER_BUFFER;  // 围栏位置
    const signX = barrierX - 5.0;                           // 指示牌位置（围栏前5m）

    // ── 1. 围栏（横跨整个路面，BoxGeometry 分段） ──
    const barrierTex = _makeBarrierTexture();
    const barrierMat = new THREE.MeshStandardMaterial({
      map: barrierTex,
      roughness: 0.35,
      metalness: 0.65,  // exempt: 施工围栏反光材质，非车漆
    });
    const segCount = Math.ceil((halfW * 2) / BARRIER_SEG_W);
    const segGeo = new THREE.BoxGeometry(BARRIER_T, BARRIER_H, BARRIER_SEG_W);
    for (let i = 0; i < segCount; i++) {
      const segZ = -halfW + (i + 0.5) * BARRIER_SEG_W;
      const clampZ = Math.max(-halfW, Math.min(halfW, segZ));
      const mesh = new THREE.Mesh(segGeo, barrierMat);
      mesh.position.set(barrierX, BARRIER_H / 2, clampZ);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
    }

    // ── 2. "禁止通行"指示牌（红底白字，挂在围栏右侧） ──
    const noEntryMat = new THREE.MeshStandardMaterial({
      map: _getSignTexture('no_entry'),
      roughness: 0.25,
      metalness: 0.2,
    });
    const signGeo = new THREE.PlaneGeometry(SIGN_W, SIGN_H);
    const noEntrySign = new THREE.Mesh(signGeo, noEntryMat);
    noEntrySign.position.set(barrierX, BARRIER_H + 0.3, halfW * 0.5);
    noEntrySign.rotation.y = Math.PI;  // 朝向来车方向（-X）
    group.add(noEntrySign);

    // ── 3. "前方施工"指示牌（黄底黑字，围栏前5m，路右侧） ──
    const constMat = new THREE.MeshStandardMaterial({
      map: _getSignTexture('construction'),
      roughness: 0.25,
      metalness: 0.2,
    });
    const constSign = new THREE.Mesh(signGeo, constMat);
    const signZ = halfW + 1.5;
    constSign.position.set(signX, SIGN_POST_H + SIGN_H / 2, signZ);
    constSign.rotation.y = Math.PI;
    group.add(constSign);

    // 指示牌立柱
    const postMat = new THREE.MeshStandardMaterial({ color: COLOR_POST, roughness: 0.5, metalness: 0.4 });
    const postGeo = new THREE.CylinderGeometry(SIGN_POST_RADIUS, SIGN_POST_RADIUS, SIGN_POST_H, 8);
    const post = new THREE.Mesh(postGeo, postMat);
    post.position.set(signX, SIGN_POST_H / 2, signZ);
    post.castShadow = true;
    group.add(post);

    // ── 4. 交通锥（缓冲带，带白色反光带） ──
    const coneMat = new THREE.MeshStandardMaterial({ color: COLOR_CONE_ORANGE, roughness: 0.3, metalness: 0.5 });
    const coneGeo = new THREE.ConeGeometry(CONE_RADIUS, CONE_H, 12);
    const bandMat = new THREE.MeshStandardMaterial({ color: COLOR_CONE_BAND, roughness: 0.2, metalness: 0.8 });  // exempt: 交通锥反光带，非车漆
    const bandGeo = new THREE.CylinderGeometry(CONE_RADIUS * 0.88, CONE_RADIUS * 0.82, 0.08, 12);

    const coneCount = Math.floor(ROAD_END_BUFFER / CONE_SPACING);
    for (let i = 0; i < coneCount; i++) {
      const cx = constructionStartX + (i + 0.5) * CONE_SPACING;
      // 左右交错
      for (const side of [-1, 1]) {
        const cz = side * halfW * 0.6;
        const cone = new THREE.Mesh(coneGeo, coneMat);
        cone.position.set(cx, CONE_H / 2, cz);
        cone.castShadow = true;
        group.add(cone);

        // 反光带
        const band = new THREE.Mesh(bandGeo, bandMat);
        band.position.set(cx, CONE_H * 0.55, cz);
        group.add(band);
      }
    }
  }

  function getGroup() { return group; }

  return { build, clear, getGroup };
}