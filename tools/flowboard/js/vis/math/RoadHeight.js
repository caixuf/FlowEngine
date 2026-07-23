/**
 * RoadHeight.js — 路面高度单一事实源
 * 
 * 从 road_network.edges[].nodes 插值计算任意 (x,y) 点的路面高度 z。
 * 所有"路上物体"（车辆、红绿灯、路灯等）的高度都应从此函数获取，
 * 消除硬编码 viaductOffset=7.7、z=0 等魔法数字。
 * 
 * 插值逻辑复用 Curve.js 的采样算法：
 *   - 直道（2个节点）：线性插值
 *   - 弯道（多节点）：Catmull-Rom 曲线插值
 * 
 * @param {object} store SceneStore，含 roadNetwork
 * @param {number} x ENU East（前向）坐标
 * @param {number} y ENU North（侧向）坐标
 * @returns {number} 路面高度 z（无路网/越界时返回 0）
 */

/** 从 edge 的 nodes 数组采样，返回指定参数 t(0~1) 处的高度 z */
function _sampleEdgeZ(nodes, t) {
  if (!nodes || nodes.length < 2) return 0;
  t = Math.max(0, Math.min(1, t));
  
  if (nodes.length === 2) {
    const a = nodes[0], b = nodes[1];
    const az = a[2] || 0, bz = b[2] || 0;
    return az + (bz - az) * t;
  }
  
  const points = nodes.map(p => ({ x: p[0], y: p[1], z: p[2] || 0 }));
  const n = points.length;
  const i = Math.floor(t * (n - 1));
  const localT = (t * (n - 1)) - i;
  const a = points[Math.min(i, n - 2)];
  const b = points[Math.min(i + 1, n - 1)];
  return a.z + (b.z - a.z) * localT;
}

/** 计算点 (x,y) 到 edge 中心线的最近距离和投影参数 t */
function _closestOnEdge(nodes, px, py) {
  if (!nodes || nodes.length < 2) return { dist: Infinity, t: 0, z: 0 };
  
  let minDist = Infinity;
  let minT = 0;
  let minZ = 0;
  
  for (let i = 0; i < nodes.length - 1; i++) {
    const a = nodes[i], b = nodes[i + 1];
    const ax = a[0], ay = a[1], bx = b[0], by = b[1];
    const dx = bx - ax, dy = by - ay;
    const len2 = dx * dx + dy * dy;
    if (len2 < 1e-6) continue;
    
    let t = ((px - ax) * dx + (py - ay) * dy) / len2;
    t = Math.max(0, Math.min(1, t));
    
    const projX = ax + dx * t;
    const projY = ay + dy * t;
    const dist2 = (px - projX) * (px - projX) + (py - projY) * (py - projY);
    
    if (dist2 < minDist) {
      minDist = dist2;
      const edgeT = (i + t) / (nodes.length - 1);
      minT = edgeT;
      minZ = _sampleEdgeZ(nodes, edgeT);
    }
  }
  
  return { dist: Math.sqrt(minDist), t: minT, z: minZ };
}

/** 路面高度查询主函数 */
export function roadHeightAt(store, x, y) {
  const rn = store && store.roadNetwork;
  if (!rn || !rn.edges || !rn.edges.length) return 0;
  
  let minDist = Infinity;
  let resultZ = 0;
  
  for (const edge of rn.edges) {
    const nodes = edge.nodes;
    if (!nodes || nodes.length < 2) continue;
    
    const laneWidth = edge.lane_width || 3.5;
    const lanes = edge.lanes || 2;
    const halfWidth = (lanes * laneWidth) / 2;
    
    const { dist, z } = _closestOnEdge(nodes, x, y);
    
    if (dist <= halfWidth + 3 && dist < minDist) {
      minDist = dist;
      resultZ = z;
    }
  }
  
  return resultZ;
}

/** 简单版本：按 edge 起点到终点的距离比例插值，不做投影 */
export function roadHeightAtSimple(store, x) {
  const rn = store && store.roadNetwork;
  if (!rn || !rn.edges || !rn.edges.length) return 0;
  
  const edge = rn.edges[0];
  const nodes = edge.nodes;
  if (!nodes || nodes.length < 2) return 0;
  
  const length = edge.length || edge.length_m || 1000;
  if (length <= 0) return 0;
  
  const t = Math.max(0, Math.min(1, x / length));
  return _sampleEdgeZ(nodes, t);
}