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
        }
        nonEgoIdx++;
      }
    }
  }

  return { ok: true, frame, rn, skipRoad, skipEgo, skipEntities, warnings };
}
