import { LANE_WIDTH } from '../core/Constants.js';

/**
 * FrameValidator.js — SceneDirector frame schema 校验纯函数
 *
 * Step 5 重构：从 SceneDirector.js 抽出，零 THREE 依赖，便于单元测试。
 * 校验逻辑详见各分支注释，与 Step 1 行为一致：
 *   1. topoData 必须是 object（非 object 时整帧丢弃）
 *   2. frame（解包后）必须是 object
 *   3. road_network 必须是 object，edges 必须是 array
 *   4. （空 edges 允许：实际由 update() 的 isViaduct 兜底处理，不算校验失败）
 *   5. ego 必须是 object
 *   6. ego.x 必须是 finite number；heading / lights 若发则必须是 number
 *   7. entities 必须是 array
 *   8. 每个 entity 必须有 type 字段（仅检查非 ego 实体，与 build 过滤一致）
 */

function _typeOf(v) {
  if (v === null) return 'null';
  if (Array.isArray(v)) return 'array';
  return typeof v;
}

/**
 * @param {*} topoData  SceneDirector.update() 收到的原始数据
 * @returns {{
 *   ok: boolean,
 *   frame?: object,
 *   rn?: object,
 *   skipRoad?: boolean,
 *   skipEgo?: boolean,
 *   skipEntities?: boolean,
 *   warnings: Array<{key: string, msg: string}>,
 * }}
 */
export function validateFrame(topoData) {
  const warnings = [];
  if (!topoData) return { ok: false, warnings };
  if (typeof topoData !== 'object') {
    warnings.push({
      key: 'topoData.type',
      msg: 'update() 收到非对象 topoData: ' + _typeOf(topoData) + '，跳过本帧',
    });
    return { ok: false, warnings };
  }

  let frame = topoData;
  if (topoData.metrics && topoData.metrics.scene) frame = topoData.metrics.scene;
  else if (topoData.scene) frame = topoData.scene;

  if (typeof frame !== 'object' || frame === null) {
    warnings.push({
      key: 'frame.type',
      msg: 'frame 解包后非对象: ' + _typeOf(frame) + '，跳过本帧',
    });
    return { ok: false, warnings };
  }

  const rn = frame.road_network || frame.roads;
  let skipRoad = false;
  if (rn) {
    if (typeof rn !== 'object' || rn === null) {
      warnings.push({
        key: 'road_network.type',
        msg: 'road_network 非 object: ' + _typeOf(rn) + '，跳过道路重建',
      });
      skipRoad = true;
    } else if (rn.edges !== undefined && !Array.isArray(rn.edges)) {
      warnings.push({
        key: 'road_network.edges',
        msg: 'road_network.edges 非数组: ' + _typeOf(rn.edges) + '，跳过道路重建',
      });
      skipRoad = true;
    } else if (rn.edges && rn.edges.length > 0) {
      for (let i = 0; i < rn.edges.length; i++) {
        const edge = rn.edges[i];
        if (!edge) continue;
        if (edge.lane_width != null && (edge.lane_width <= 0 || edge.lane_width > 10)) {
          warnings.push({
            key: 'road_network.edges[' + i + '].lane_width',
            msg: 'edge[' + i + '].lane_width=' + edge.lane_width + ' 超出合理范围(0,10]，回退默认 ' + LANE_WIDTH + 'm',
          });
        }
        const len = edge.length || edge.length_m;
        if (len != null && len <= 0) {
          warnings.push({
            key: 'road_network.edges[' + i + '].length',
            msg: 'edge[' + i + '].length=' + len + ' <= 0，道路长度无效',
          });
        }
        if (!edge.nodes || !Array.isArray(edge.nodes) || edge.nodes.length < 2) {
          warnings.push({
            key: 'road_network.edges[' + i + '].nodes',
            msg: 'edge[' + i + '].nodes 无效（需至少 2 个节点），道路可能无法渲染',
          });
        }
      }
    }
  }

  let skipEgo = false;
  if (frame.ego !== undefined) {
    const e = frame.ego;
    if (typeof e !== 'object' || e === null) {
      warnings.push({
        key: 'ego.type',
        msg: 'frame.ego 非 object: ' + _typeOf(e) + '，跳过 ego 更新',
      });
      skipEgo = true;
    } else {
      if (typeof e.x !== 'number' || !isFinite(e.x)) {
        warnings.push({
          key: 'ego.x',
          msg: 'ego.x 非 finite number: ' + _typeOf(e.x) + '，回退 0（ego 会卡在原点，检查 scene_pub ego JSON）',
        });
      }
      if (e.heading !== undefined && typeof e.heading !== 'number') {
        warnings.push({
          key: 'ego.heading',
          msg: 'ego.heading 非 number: ' + _typeOf(e.heading),
        });
      }
      if (e.lights !== undefined && typeof e.lights !== 'number') {
        warnings.push({
          key: 'ego.lights',
          msg: 'ego.lights 非 number: ' + _typeOf(e.lights) + '，回退 0（车灯不亮）',
        });
      }
    }
  }

  let skipEntities = false;
  if (frame.entities !== undefined) {
    if (!Array.isArray(frame.entities)) {
      warnings.push({
        key: 'entities.type',
        msg: 'frame.entities 非数组: ' + _typeOf(frame.entities) + '，跳过实体更新',
      });
      skipEntities = true;
    } else {
      // 与 build 过滤一致：仅检查非 ego 实体
      let nonEgoIdx = 0;
      for (const e of frame.entities) {
        if (!e || e.type === 'ego') continue;
        if (!e.type) {
          warnings.push({
            key: 'entities[' + nonEgoIdx + '].type',
            msg: 'entity 缺 type 字段，会被各 View 静默丢弃',
          });
        } else if (!['car', 'suv', 'truck', 'pedestrian', 'traffic_light', 'tl', 'etc_gate', 'stop_line', 'sign', 'ego'].includes(e.type)) {
          warnings.push({
            key: 'entities[' + nonEgoIdx + '].type',
            msg: 'entity 类型 "' + e.type + '" 不在已知集合 {car, suv, truck, pedestrian, traffic_light, etc_gate, stop_line, sign, ego}，可能被静默兜底为 Car',
          });
        }
        nonEgoIdx++;
      }
    }
  }

  /* ── 不变量：实体/ego 高度 vs 路网高度偏差 >2m → 悬空/入地告警 ──
   * 把"红绿灯在路面下 7m"这类问题从静默变成可见 warn。
   * 只对有路网、有坐标的数据做检查，不因此阻断渲染。 */
  if (rn && rn.edges && rn.edges.length > 0) {
    const toCheck = [];
    if (frame.ego && typeof frame.ego === 'object') {
      toCheck.push({ label: 'ego', x: frame.ego.x, y: frame.ego.y, z: frame.ego.z });
    }
    if (frame.entities && Array.isArray(frame.entities)) {
      for (let i = 0; i < frame.entities.length; i++) {
        const e = frame.entities[i];
        if (!e || e.type === 'ego') continue;
        toCheck.push({ label: 'entity[' + i + '](type=' + (e.type || '?') + ')', x: e.x, y: e.y, z: e.z });
      }
    }
    for (const item of toCheck) {
      const rz = _roadHeightAt(rn.edges, item.x, item.y);
      const dz = Math.abs((item.z || 0) - rz);
      if (dz > 2.0) {
        warnings.push({
          key: 'height.' + item.label,
          msg: item.label + ' z=' + (item.z || 0) + 'm 与路面高度 ' + rz + 'm 偏差 ' + dz.toFixed(1) + 'm，可能悬空或入地',
        });
      }
    }
  }

  return { ok: true, frame, rn, skipRoad, skipEgo, skipEntities, warnings };
}

/** 轻量路面高度查询（FrameValidator 零依赖版）
 *  复用 RoadHeight.js 算法：找最近 edge 投影，线性插值 z。 */
function _roadHeightAt(edges, px, py) {
  let minDist = Infinity, resultZ = 0;
  for (const edge of edges) {
    const nodes = edge.nodes;
    if (!nodes || nodes.length < 2) continue;
    const laneWidth = edge.lane_width || LANE_WIDTH;
    const lanes = edge.lanes || 2;
    const halfWidth = (lanes * laneWidth) / 2 + 3;
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
      const dist = Math.hypot(px - projX, py - projY);
      if (dist < minDist) {
        minDist = dist;
        const edgeT = (i + t) / (nodes.length - 1);
        const az = a[2] || 0, bz = b[2] || 0;
        resultZ = az + (bz - az) * edgeT;
      }
    }
    if (minDist <= halfWidth) break; // 已在道路范围内，不必继续
  }
  return resultZ;
}
