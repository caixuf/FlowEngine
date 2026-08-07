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
 *
 * 后端单一事实源：build(roadNetwork, zones) 优先消费 scene.construction_zones
 * （flowsim scenario 定义，经 monitor 透传）渲染施工区几何；zones 为空时回退到
 * "道路末端 30m"自算逻辑。施工段占世界坐标 [x-length/2, x+length/2]。
 */

import { LANE_WIDTH, DEFAULT_LANES } from '../core/Constants.js';
import { makeSignTexture, makeStripeTexture } from '../utils/CanvasTextureFactory.js';

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
        // 纹理由 CanvasTextureFactory 缓存，不在这里 dispose
        c.material.dispose();
      }
    }
  }

  function build(roadNetwork, zones) {
    clear();
    if (!roadNetwork || !roadNetwork.edges || roadNetwork.edges.length === 0) return;

    // 找最长 edge 确定道路终点 + 路宽
    let maxLen = 0, edge = null;
    for (const e of roadNetwork.edges) {
      const len = e.length || e.length_m || 0;
      if (len > maxLen) { maxLen = len; edge = e; }
    }
    if (!edge || maxLen < 100) return;

    const lanes = edge.lanes || DEFAULT_LANES;
    const laneWidth = edge.lane_width || LANE_WIDTH;
    const roadHalfW = (lanes * laneWidth) / 2;

    // 道路终点在 ENU (maxLen, 0, 0) → THREE (maxLen, 0, 0)
    const roadEndX = maxLen;

    // 后端单一事实源：优先用 scene.construction_zones 渲染施工区几何。
    // 施工段占世界坐标 [x-length/2, x+length/2]（ENU x 直接映射 THREE x）。
    // 空时回退到"道路末端 30m"旧逻辑，保证无后端数据时仍可渲染。
    if (Array.isArray(zones) && zones.length > 0) {
      for (const z of zones) {
        const zoneLen = (z.length > 0) ? z.length : ROAD_END_BUFFER;
        const startX = z.x - zoneLen / 2;               // 施工区前缘（THREE x）
        const halfW = (z.width > 0) ? z.width / 2 : roadHalfW;
        _renderZone(startX, zoneLen, halfW);
      }
    } else {
      _renderZone(roadEndX - ROAD_END_BUFFER, ROAD_END_BUFFER, roadHalfW);
    }
  }

  // 渲染单个施工区：constructionStartX=前缘 THREE x，bufferLen=纵向长度，halfW=半路宽。
  // 围栏必须落在施工前缘（与 flowsim 注入的 construction 障碍物 / planning 可通行域
  // 一致）。旧实现 barrierX = front - 5m，视觉墙比感知墙靠前 5m → 掉头弧在感知
  // 合法区内仍会"穿围栏"，被误判为进了施工区（可通行域观感故障）。
  function _renderZone(constructionStartX, bufferLen, halfW) {
    const barrierX = constructionStartX;                    // 围栏 = 施工前缘（权威）
    const signX = barrierX - BARRIER_BUFFER - 5.0;           // 指示牌仍在围栏前

    // ── 1. 围栏（横跨整个路面，BoxGeometry 分段） ──
    const barrierTex = makeStripeTexture('#ff6600', '#f0f0f0', { width: 64, height: 256, stripeCount: 8 });
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
      map: makeSignTexture('禁止通行', '#cc0000', '#ffffff', '#ffffff'),
      roughness: 0.25,
      metalness: 0.2,
    });
    const signGeo = new THREE.PlaneGeometry(SIGN_W, SIGN_H);
    const noEntrySign = new THREE.Mesh(signGeo, noEntryMat);
    noEntrySign.position.set(barrierX, BARRIER_H + 0.3, halfW * 0.5);
    noEntrySign.rotation.y = -Math.PI / 2;  /* 正面朝 -X（来车方向），宽度沿 Z（横向） */
    group.add(noEntrySign);

    // ── 3. "前方施工"指示牌（黄底黑字，围栏前5m，路右侧） ──
    const constMat = new THREE.MeshStandardMaterial({
      map: makeSignTexture('前方施工', '#ffcc00', '#000000', '#000000'),
      roughness: 0.25,
      metalness: 0.2,
    });
    const constSign = new THREE.Mesh(signGeo, constMat);
    const signZ = halfW + 1.5;
    constSign.position.set(signX, SIGN_POST_H + SIGN_H / 2, signZ);
    constSign.rotation.y = -Math.PI / 2;  /* 正面朝 -X（来车方向），宽度沿 Z（横向） */
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

    const coneCount = Math.floor(bufferLen / CONE_SPACING);
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