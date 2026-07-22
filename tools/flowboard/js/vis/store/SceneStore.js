/**
 * SceneStore.js — 单一数据源
 * 存储所有场景状态，View 只读。Director 写入。
 * 取代旧架构的 51 个模块级 let 全局变量。
 */

export function createSceneStore() {
  return {
    // ── 道路网络 ──
    roadNetwork: null,        // { edges: [...], hash: string }
    roadHash: '',             // 用于 diff 检测
    isViaduct: false,         // 是否高架模式
    viaductVisLength: 0,      // 高架段实际建造长度（米），用于 wrap 周期；0 = 未建

    // ── ego ──
    ego: null,                // { x, y, heading, speed, steer, brake, throttle, lights, vx, vy, length, width }

    // ── 实体列表 ──
    entities: [],             // [{ id, type, x, y, heading, speed, lights, ai_state, ... }]

    // ── 环境配置 ──
    env: {
      isNight: false,
      lighting: 0,            // 场景配置的 lighting 模式
    },

    // ── 性能档位 ──
    perfTier: 'high',         // high / medium / low

    // ── 调试 ──
    showLabels: true,
    paused: false,
  };
}

/** 计算 road_network 的 hash，用于 diff 检测 */
export function roadNetworkHash(rn) {
  if (!rn || !rn.edges) return '';
  return rn.edges.map(e => `${e.id||0}_${e.lanes||0}_${e.length||0}`).join('|');
}

/**
 * 获取道路组的中心点坐标 {x, z}。
 * 流畅专题：roadGroup 只在 roadHash 变化时重建（SceneDirector.update 里
 * 新建对象），所以中心点可按 roadGroup 引用缓存（WeakMap）。原先每次调用
 * 都 new THREE.Box3().setFromObject() 遍历整棵路树，CameraRig 每帧调
 * 1~3 次 → 60fps 下显著 GC 压力。重建后旧 roadGroup 被 GC，缓存条目自动
 * 回收，无需手动失效。
 */
const _centerCache = new WeakMap();
const _centerBox = new THREE.Box3();
export function getCenter(roadGroup) {
  if (!roadGroup || !roadGroup.children || roadGroup.children.length === 0) {
    return { x: 0, z: 0 };
  }
  const cached = _centerCache.get(roadGroup);
  if (cached) return cached;
  _centerBox.setFromObject(roadGroup);
  if (!isFinite(_centerBox.min.x) || !isFinite(_centerBox.max.x)) {
    return { x: 0, z: 0 };
  }
  const center = {
    x: (_centerBox.min.x + _centerBox.max.x) / 2,
    z: (_centerBox.min.z + _centerBox.max.z) / 2
  };
  _centerCache.set(roadGroup, center);
  return center;
}
