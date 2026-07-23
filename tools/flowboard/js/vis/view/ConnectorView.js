/**
 * ConnectorView.js — 路段拼接连接件
 *
 * 解决多段 edge 直接拼接的视觉断层：
 *   1. LaneTaper    — 车道数/宽度变化的锥形过渡
 *   2. JunctionCap  — 路口两端的路口区域补齐
 *   3. RampMerge    — 匝道汇入主路的喇叭口导流线
 *   4. ViaductPier  — 高架段桥墩支撑
 *   5. BarrierEndCap— 护栏端头防撞桶
 *
 * 全部从 road_network.edges 元数据自动派生，零配置。
 * 连接件 mesh 加到 connectorGroup，road_network 变化时整体重建。
 */

import { getBox, getCylinder, getStdMaterial } from '../core/AssetFactory.js';
import { LANE_WIDTH, DEFAULT_LANES, EDGE_TYPE } from '../core/Constants.js';

const TAPER_COLOR  = 0x2a2a2a;   // 锥形过渡路面（同沥青色）
const JUNCTION_COLOR = 0x353535; // 路口区域稍浅
const MERGE_LINE_COLOR = 0xffffff; // 导流线白
const PIER_COLOR   = 0x6b6b6b;   // 桥墩灰
const BARREL_RED   = 0xd02020;   // 防撞桶红
const BARREL_WHITE = 0xf0f0f0;   // 防撞桶白

export function createConnectorView(scene) {
  const group = new THREE.Group();
  scene.add(group);
  let built = false;

  /** 清空连接件 */
  function clear() {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.geometry) c.geometry.dispose();
    }
    built = false;
  }

  /** 主构建入口：扫描 edges，自动派生连接件 */
  function build(roadNetwork) {
    clear();
    if (!roadNetwork || !roadNetwork.edges) return;

    const edges = roadNetwork.edges;
    const junctions = roadNetwork.junctions || [];

    // ── 1. LaneTaper：相邻 edge 车道数/宽度不同 ──
    for (let i = 0; i < edges.length - 1; i++) {
      const a = edges[i], b = edges[i + 1];
      // 只处理顺序相连的 edge（id 连续），跳过匝道等跳跃 edge
      if (b.id !== a.id + 1) continue;
      const widthA = (a.lanes || DEFAULT_LANES) * (a.lane_width || LANE_WIDTH);
      const widthB = (b.lanes || DEFAULT_LANES) * (b.lane_width || LANE_WIDTH);
      if (Math.abs(widthA - widthB) < 0.1) continue;
      _buildLaneTaper(a, b, widthA, widthB);
    }

    // ── 2. JunctionCap：intersection 类型 edge ──
    for (const edge of edges) {
      if (edge.type === 'intersection') {
        _buildJunctionCap(edge);
      }
    }

    // ── 3. RampMerge：ramp_curve 类型 + junctions 里有 merge ──
    const mergeJunctions = junctions.filter(j => j.type === 'merge');
    for (const edge of edges) {
      if (edge.type !== 'ramp_curve') continue;
      // 查找该 ramp 对应的 merge junction
      const mj = mergeJunctions.find(j => j.incoming_road === edge.id);
      if (mj) {
        const targetEdge = edges.find(e => e.id === mj.target_road);
        if (targetEdge) _buildRampMerge(edge, targetEdge);
      }
    }

    // ── 4. ViaductPier：elevation_profile 中 h > 0.5 的段 ──
    for (const edge of edges) {
      if (!edge.elevation_profile) continue;
      _buildViaductPier(edge);
    }

    // ── 5. BarrierEndCap：每个 edge 起止点放防撞桶 ──
    for (const edge of edges) {
      if (edge.type === 'ramp_curve' || edge.type === 'intersection') continue;
      _buildBarrierEndCap(edge);
    }

    built = true;
  }

  /** 1. LaneTaper：锥形过渡路面
   *  在 edgeA 终点位置生成一个梯形（宽 widthA → widthB），长度 5m */
  function _buildLaneTaper(edgeA, edgeB, widthA, widthB) {
    // 推断接缝位置：用 edgeA 的 nodes 末点，或 length_m 累计
    const pos = _getEdgeEnd(edgeA);
    if (!pos) return;

    const taperLen = 5;
    const hA = widthA / 2, hB = widthB / 2;
    // 梯形顶点（沿 X 轴铺路假设）
    const positions = [
      pos.x - taperLen/2, 0.11, pos.z + hA,   // 左后
      pos.x - taperLen/2, 0.11, pos.z - hA,   // 右后
      pos.x + taperLen/2, 0.11, pos.z + hB,   // 左前
      pos.x + taperLen/2, 0.11, pos.z - hB,   // 右前
    ];
    const indices = [0, 1, 2, 1, 3, 2];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    const mat = getStdMaterial(TAPER_COLOR, 0.92, 0.0);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    group.add(mesh);
  }

  /** 2. JunctionCap：路口区域路面（稍浅色 + 比路宽 1.5 倍） */
  function _buildJunctionCap(edge) {
    const start = _getEdgeStart(edge);
    const end = _getEdgeEnd(edge);
    if (!start || !end) return;

    const width = (edge.lanes || DEFAULT_LANES) * (edge.lane_width || LANE_WIDTH) * 1.5;
    const len = edge.length_m || 60;
    const geo = getBox(len, 0.08, width);
    const mat = getStdMaterial(JUNCTION_COLOR, 0.88, 0.0);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set((start.x + end.x) / 2, 0.12, (start.z + end.z) / 2);
    mesh.receiveShadow = true;
    group.add(mesh);

    // 斑马线（简化：4 条白色薄条）
    const stripeMat = getStdMaterial(0xffffff, 0.6, 0.0);
    for (let i = 0; i < 4; i++) {
      const stripeGeo = getBox(0.3, 0.02, width * 0.8);
      const stripe = new THREE.Mesh(stripeGeo, stripeMat);
      stripe.position.set(start.x + 2 + i * 1.0, 0.17, start.z);
      group.add(stripe);
    }
  }

  /** 3. RampMerge：喇叭口导流线（三角形 + 白色斜线） */
  function _buildRampMerge(rampEdge, mainEdge) {
    const rampEnd = _getEdgeEnd(rampEdge);
    const mainStart = _getEdgeStart(mainEdge);
    if (!rampEnd || !mainStart) return;

    // 三角形导流带（连接 ramp 终点和主路边缘）
    const rampWidth = (rampEdge.lanes || 1) * (rampEdge.lane_width || 3.0);
    const mainWidth = (mainEdge.lanes || DEFAULT_LANES) * (mainEdge.lane_width || LANE_WIDTH);
    const positions = [
      rampEnd.x, 0.11, rampEnd.z + rampWidth/2,
      rampEnd.x, 0.11, rampEnd.z - rampWidth/2,
      mainStart.x + 10, 0.11, mainStart.z - mainWidth/2,
    ];
    const indices = [0, 1, 2];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    const mat = getStdMaterial(TAPER_COLOR, 0.92, 0.0);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    group.add(mesh);

    // 导流斜线（5 条白色短线）
    const lineMat = getStdMaterial(MERGE_LINE_COLOR, 0.5, 0.0);
    for (let i = 0; i < 5; i++) {
      const t = i / 5;
      const lineGeo = getBox(0.15, 0.02, 1.5);
      const line = new THREE.Mesh(lineGeo, lineMat);
      line.position.set(rampEnd.x + t * 10, 0.16, rampEnd.z - rampWidth/2 + t * 1.5);
      line.rotation.y = Math.PI / 6;
      group.add(line);
    }
  }

  /** 4. ViaductPier：高架桥墩（每隔 20m 一根圆柱） */
  function _buildViaductPier(edge) {
    const elev = edge.elevation_profile || [];
    if (elev.length === 0) return;
    const start = _getEdgeStart(edge);
    if (!start) return;

    const pierMat = getStdMaterial(PIER_COLOR, 0.9, 0.0);
    const pierGeo = getCylinder(0.6, 0.8, 1);  // 高度 1，后续按 elevation 缩放
    let prevS = 0, prevH = elev[0].h || 0;

    // 沿 edge 每 20m 放一根桥墩，高度按 elevation_profile 插值
    const totalLen = edge.length_m || 100;
    for (let s = 10; s < totalLen; s += 20) {
      // 找到 s 对应的 elevation
      let h = prevH;
      for (let i = 1; i < elev.length; i++) {
        if (s <= elev[i].s) {
          const t = (s - prevS) / (elev[i].s - prevS || 1);
          h = prevH + (elev[i].h - prevH) * t;
          break;
        }
        prevS = elev[i].s; prevH = elev[i].h;
      }
      if (h < 0.5) continue;  // 低于 0.5m 不需要桥墩

      const pier = new THREE.Mesh(pierGeo, pierMat);
      pier.position.set(start.x + s, h / 2, start.z);
      pier.scale.y = h;
      pier.castShadow = true;
      pier.receiveShadow = true;
      group.add(pier);
    }
  }

  /** 5. BarrierEndCap：防撞桶（红白圆柱）放在 edge 起止点 */
  function _buildBarrierEndCap(edge) {
    const start = _getEdgeStart(edge);
    const end = _getEdgeEnd(edge);
    if (!start || !end) return;

    const width = (edge.lanes || DEFAULT_LANES) * (edge.lane_width || LANE_WIDTH);
    [start, end].forEach(pos => {
      // 红白两段圆柱
      const redGeo = getCylinder(0.3, 0.35, 0.4);
      const redMat = getStdMaterial(BARREL_RED, 0.6, 0.1);
      const red = new THREE.Mesh(redGeo, redMat);
      red.position.set(pos.x, 0.3, pos.z + width / 2 + 0.5);
      red.castShadow = true;
      group.add(red);

      const whiteGeo = getCylinder(0.3, 0.3, 0.3);
      const whiteMat = getStdMaterial(BARREL_WHITE, 0.6, 0.1);
      const white = new THREE.Mesh(whiteGeo, whiteMat);
      white.position.set(pos.x, 0.75, pos.z + width / 2 + 0.5);
      group.add(white);
    });
  }

  /** 工具：获取 edge 起点（优先用 nodes[0]，否则用 length_m 累计）
   *  注意：scene_pub 输出的 nodes 是 [x, y_up, z_north] 数组格式 */
  function _getEdgeStart(edge) {
    if (edge.nodes && edge.nodes.length >= 1) {
      const n = edge.nodes[0];
      if (Array.isArray(n)) return { x: n[0], y: n[1] || 0, z: -(n[2] || 0) };
      if (typeof n === 'object') return { x: n.x || 0, y: n.y || 0, z: -(n.z || 0) };
    }
    if (edge.start_x != null) return { x: edge.start_x, y: 0, z: -(edge.start_z || 0) };
    return null;
  }

  /** 工具：获取 edge 终点 */
  function _getEdgeEnd(edge) {
    if (edge.nodes && edge.nodes.length >= 2) {
      const n = edge.nodes[edge.nodes.length - 1];
      if (Array.isArray(n)) return { x: n[0], y: n[1] || 0, z: -(n[2] || 0) };
      if (typeof n === 'object') return { x: n.x || 0, y: n.y || 0, z: -(n.z || 0) };
    }
    if (edge.start_x != null) {
      const len = edge.length_m || 100;
      const h = edge.heading || 0;
      return {
        x: edge.start_x + Math.cos(h) * len,
        y: 0,
        z: -((edge.start_z || 0) + Math.sin(h) * len),
      };
    }
    return null;
  }

  function isBuilt() { return built; }
  function getGroup() { return group; }

  return { build, clear, isBuilt, getGroup };
}
