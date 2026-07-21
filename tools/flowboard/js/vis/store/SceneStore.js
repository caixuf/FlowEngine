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
