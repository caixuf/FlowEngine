/**
 * sceneAdapter.js — 场景 JSON → vis scene frame 转换器（纯函数，零 THREE 依赖）
 *
 * 背景：3D 渲染（SceneDirector）完全数据驱动，只消费 `metrics.scene` 帧：
 *   { road_network, ego, entities:[...], construction_zones, lighting }
 * 而 `scenarios/*.json` 已经携带 road_network / ego / actors / traffic_lights /
 * construction_zones —— 与后端 flowsim 运行时发布到 topology 的 scene 帧同源。
 *
 * 本模块把「静态场景定义」转换成「可直接喂给 SceneDirector 的一帧快照」，
 * 让综合展示页 showcase.html 无需启动整条 C 管道即可 3D 预览任意场景。
 *
 * 单一事实源：浏览器展示页与门禁测试 (tests/vis_showcase_scenes.test.mjs)
 * 都 import 本模块做转换，避免 Python/JS 两份转换逻辑漂移。
 *
 * 静态快照约定：actor / ego 的速度分量 vx/vy 归零 —— 展示页不跑仿真，
 * 死推算 (DeadReckon) 会用 vx/vy 逐帧外推位置，若不归零车辆会永久漂走。
 * heading 在归零前由 vx/vy 推出并显式写入，保证车头朝向正确。
 */

/** 把 node 归一化为 [x, y, z] 三元组（缺 z 补 0），满足 FrameValidator arity=3。 */
function normalizeNodes(nodes) {
  if (!Array.isArray(nodes)) return nodes;
  return nodes.map((n) => {
    if (Array.isArray(n)) {
      return [n[0] || 0, n[1] || 0, n[2] || 0];
    }
    if (n && typeof n === 'object') {
      return [n.x || 0, n.y || 0, n.z || 0];
    }
    return [0, 0, 0];
  });
}

/** road_network 透传 + 字段补全（length_m→length，nodes 归一化）。 */
function adaptRoadNetwork(rn) {
  if (!rn || !Array.isArray(rn.edges)) {
    return { edges: [] };
  }
  const edges = rn.edges.map((e, i) => ({
    id: e.id != null ? e.id : i,
    name: e.name != null ? e.name : 'edge_' + i,
    type: e.type || 'road',
    lanes: e.lanes || 4,
    lane_width: e.lane_width || 3.5,
    // FrameValidator/roadNetworkHash 读 length；scenario 用 length_m 别名。
    length: e.length != null ? e.length : (e.length_m != null ? e.length_m : 0),
    length_m: e.length_m,
    oneway: e.oneway === true,
    nodes: normalizeNodes(e.nodes),
    speed_limit: e.speed_limit,
  }));
  return { edges };
}

/** ego 定义 → scene.ego（速度归零，静态快照）。 */
function adaptEgo(ego) {
  if (!ego) return { type: 'ego', id: 0, x: 0, y: 0, heading: 0, speed: 0 };
  return {
    type: 'ego',
    id: 0,
    x: ego.x || 0,
    y: ego.y || 0,
    z: ego.z || 0,
    heading: ego.heading || 0,
    // 展示用初速度作为标注速度；vx/vy 归零防死推算漂移。
    speed: ego.init_speed != null ? ego.init_speed : (ego.speed != null ? ego.speed : 0),
    steer: 0,
    throttle: 0,
    brake: 0,
    vx: 0,
    vy: 0,
    length: ego.length != null ? ego.length : 4.6,
    width: ego.width != null ? ego.width : 2.0,
    lights: 0,
  };
}

/** 单个 actor → scene entity（车辆/行人）。 */
function adaptActor(a) {
  const vx = a.vx || 0;
  const vy = a.vy || 0;
  const speed = Math.hypot(vx, vy);
  // heading：优先显式字段，否则由速度向量推（ENU：heading=0 指 +x 东）。
  // 这里算的是 ENU 航向数据（与后端 flowsim 发布的 heading 同一坐标空间），
  // 不是 THREE rotationY，故不适用 Coord.directionToRotationY。 // exempt: ENU heading data
  let heading = 0;
  if (a.heading != null) heading = a.heading;
  else if (speed > 1e-6) heading = Math.atan2(vy, vx); // exempt: ENU heading data, not THREE rotation
  return {
    type: a.type || 'car',
    id: a.id,
    x: a.x || 0,
    y: a.y || 0,
    z: a.z || 0,
    heading,
    speed,
    // 静态快照：vx/vy 归零，避免死推算把车逐帧外推出场。
    vx: 0,
    vy: 0,
    steer: 0,
    throttle: 0,
    brake: 0,
    length: a.len != null ? a.len : (a.length != null ? a.length : undefined),
    width: a.wid != null ? a.wid : (a.width != null ? a.width : undefined),
    ai_state: a.ai_state || '',
    lights: 0,
  };
}

/** 单个红绿灯定义 → scene entity（type='tl'）。 */
function adaptTrafficLight(t) {
  return {
    type: 'tl',
    id: t.id,
    x: t.x || 0,
    // scenario 用 y_lane 表示灯所管辖车道的横向位置。
    y: t.y != null ? t.y : (t.y_lane != null ? t.y_lane : 0),
    z: t.z || 0,
    heading: t.heading != null ? t.heading : 0,
    // 静态展示默认绿灯（无仿真相位推进）。
    state: t.state || 'green',
    remain_s: 0,
  };
}

/**
 * 场景 JSON → vis scene frame。
 * @param {object} raw  scenarios/*.json 解析后的对象
 * @returns {{road_network, ego, entities, construction_zones, lighting, t_us}}
 */
export function scenarioToScene(raw) {
  raw = raw || {};
  const entities = [];

  const actors = Array.isArray(raw.actors) ? raw.actors : [];
  for (const a of actors) entities.push(adaptActor(a));

  const tls = Array.isArray(raw.traffic_lights) ? raw.traffic_lights : [];
  for (const t of tls) entities.push(adaptTrafficLight(t));

  const scene = {
    t_us: 0,
    lighting: raw.lighting || 'day',
    road_network: adaptRoadNetwork(raw.road_network),
    ego: adaptEgo(raw.ego),
    entities,
  };

  if (Array.isArray(raw.construction_zones) && raw.construction_zones.length > 0) {
    scene.construction_zones = raw.construction_zones.map((cz) => ({
      id: cz.id,
      x: cz.x || 0,
      y: cz.y || 0,
      length: cz.length || 0,
      width: cz.width || 0,
    }));
  }

  return scene;
}

/**
 * 包一层 topoData 结构（SceneDirector.update 期望 metrics.scene）。
 * @param {object} raw  scenarios/*.json 解析后的对象
 */
export function scenarioToTopoData(raw) {
  return { metrics: { scene: scenarioToScene(raw) } };
}
