/**
 * Constants.js — 前端可视化层单一事实源
 * 
 * 所有跨模块共享的常量、魔法数字、枚举值集中定义，消除硬编码漂移。
 */

export const LANE_WIDTH = 3.5;
export const DEFAULT_LANES = 2;

export const EDGE_TYPE = {
  HIGHWAY: 'highway',
  URBAN: 'urban',
  VIADUCT_HIGHWAY: 'viaduct_highway',
  RAMP_CURVE: 'ramp_curve',
  INTERSECTION: 'intersection',
};

export const ENTITY_TYPE = {
  CAR: 'car',
  PEDESTRIAN: 'pedestrian',
  TRAFFIC_LIGHT: 'traffic_light',
  SIGN: 'sign',
};

export const VIADUCT_HEIGHT = 7.0;
export const VIADUCT_RIDE_HEIGHT = 7.7;