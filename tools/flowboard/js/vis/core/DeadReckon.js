/**
 * DeadReckon.js — Single source of truth for dead reckoning in FlowBoard
 *
 * Step 2 重构：从 tools/flowboard/js/deadreckon.js 移到 vis/core/，
 * 算法零改动。原 deadreckon.js 将在 Phase 4 清理时删除。
 *
 * 流畅专题：扩展为多实体。ego 仍走 `_dr` 单例（2D renderer 直接读
 * _dr.smooth*），新增 `_entities` Map 支持 NPC 插值。SSE 5Hz 数据 vs
 * 60fps 渲染：NPC 原先每帧直接 snap 到最新真值，导致 ~200ms 一次的跳变；
 * 现在每帧对平滑状态做指数 lerp + 速度外推，与 ego 一致。
 *
 * Pipeline:
 *   updateDeadReckon(x, z, speed, heading)        ← ego SSE tick (app.js sync2DTarget)
 *   updateEntityDeadReckon(id, x, y, v, h)        ← NPC SSE tick (SceneDirector.update)
 *   tickDeadReckon() / tickEntityDeadReckon(now)  ← 每动画帧 (SceneDirector.tickAnimation)
 *   getDeadReckonState() / getEntitySmooth(id)    ← 3D / 2D renderers 读
 *
 * Coordinate system (matches sim_world_node / monitor_node):
 *   X = forward (m), Z = lateral (m), heading = radians, speed = m/s.
 * 注意：ego 的第 2 参数 z 实际是 sim 的横向 y；entity 同理第 2 参数 = 横向 y。
 *
 * Frame-rate-independent smoothing:
 *   smooth += (target - smooth) * (1 - exp(-lambda * dt))
 * 这把平滑速度与渲染帧率解耦，30 fps 与 144 fps 表现一致。
 */

// ── Smoothing coefficients (higher = snappier) ──
// lambda=8 ≈ 0.125 lerp at 60 fps (matches the previous 0.15 factor)。
// Heading 用更小的 lambda，避免噪声输入导致抖动。
export var LAMBDA_POS = 8.0;
export var LAMBDA_HEADING = 6.0;

// ── Dead-reckoning state ─────────────────────────────────────────
// last*     : updateDeadReckon() 喂入的最新真值
// target*   : 用 last speed 外推出的预测位置
// smooth*   : 指数平滑后的值，渲染层消费
// lastFrameTime : 上一次 tickDeadReckon() 的墙钟
export var _dr = {
  lastX: 0,
  lastZ: 0,
  lastSpeed: 0,
  lastHeading: 0,
  lastTime: 0,
  targetX: 0,
  targetZ: 0,
  targetHeading: 0,
  smoothX: 0,
  smoothZ: 0,
  smoothHeading: 0,
  smoothSpeed: 0,
  lastFrameTime: 0,
  init: false
};

/**
 * initDeadReckon — (re)initialise the dead-reckoning state to zero.
 * Called once during 3D scene init.
 */
export function initDeadReckon() {
  _dr.lastX = 0;
  _dr.lastZ = 0;
  _dr.lastSpeed = 0;
  _dr.lastHeading = 0;
  _dr.lastTime = 0;
  _dr.targetX = 0;
  _dr.targetZ = 0;
  _dr.targetHeading = 0;
  _dr.smoothX = 0;
  _dr.smoothZ = 0;
  _dr.smoothHeading = 0;
  _dr.smoothSpeed = 0;
  _dr.lastFrameTime = 0;
  _dr.init = false;
  resetEntities();
}

/**
 * updateDeadReckon — feed fresh ground-truth into the dead-reckoning
 * state.  Called on every SSE data tick (by app.js sync2DTarget).
 *
 * Position is only accepted when it actually moved; this rejects
 * duplicate / heartbeat frames that would otherwise reset the
 * extrapolation clock and cause a visible stutter.
 *
 * On the very first sample the smoothed state snaps to the ground
 * truth so the car does not lerp in from the world origin.
 *
 * @param {number} x       world X position (forward, m)
 * @param {number} z       world Z position (lateral, m)
 * @param {number} speed   forward speed (m/s)
 * @param {number} heading heading angle (radians)
 */
export function updateDeadReckon(x, z, speed, heading) {
  var now = performance.now() / 1000;
  if (
    !_dr.init ||
    Math.abs(x - _dr.lastX) > 0.01 ||
    Math.abs(z - _dr.lastZ) > 0.01 ||
    Math.abs(speed - _dr.lastSpeed) > 0.1
  ) {
    _dr.lastX = x;
    _dr.lastZ = z;
    _dr.lastSpeed = speed;
    _dr.lastHeading = heading;
    _dr.lastTime = now;
    if (!_dr.init) {
      // First sample: snap smooth to truth so we do not lerp from (0,0).
      _dr.smoothX = x;
      _dr.smoothZ = z;
      _dr.smoothHeading = heading;
      _dr.smoothSpeed = speed;
      _dr.targetX = x;
      _dr.targetZ = z;
      _dr.targetHeading = heading;
      _dr.init = true;
    }
  }
}

/**
 * tickDeadReckon — advance the dead-reckoning prediction by one frame.
 * Must be called from every renderer's animation loop (3D and 2D).
 *
 * Performs:
 *   1. Speed-based position extrapolation (target = last + v * dt)。
 *   2. Frame-rate-independent exponential smoothing toward target。
 *   3. Shortest-path angular lerp for heading (修原先穿越 ±π 时整圈
 *      旋转的 wrap bug)。
 */
export function tickDeadReckon() {
  if (!_dr.init) return;
  var now = performance.now() / 1000;
  if (_dr.lastFrameTime === 0) _dr.lastFrameTime = now;
  // Clamp dt: 后台切走 10s 回来不能让车一下窜出 100m。
  var dt = now - _dr.lastFrameTime;
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0) { _dr.lastFrameTime = now; return; }
  _dr.lastFrameTime = now;
  _advanceState(_dr, dt, now);
}

/**
 * getDeadReckonState — return the current dead-reckoning object.
 * Renderers read smoothX / smoothZ / smoothHeading / smoothSpeed
 * (and lastX / lastZ for world-anchoring obstacles & LiDAR)。
 */
export function getDeadReckonState() {
  return _dr;
}

// ═══════════════════════════════════════════════════════════════════
// 多实体 dead reckoning（流畅专题：NPC 插值）
// ═══════════════════════════════════════════════════════════════════
// SSE 5Hz 真值 vs 60fps 渲染：NPC 原先 zero interpolation，每帧直接 snap
// 真值 → ~200ms 一次跳变。这里给每个 NPC 维护一份与 ego 同构的 drState，
// 每帧做同样的 外推 + 指数 lerp，消除卡顿。
//
// 状态生命周期：updateEntityDeadReckon 喂真值（SSE tick）→
// tickEntityDeadReckon 推进平滑（rAF）→ getEntitySmooth 读（View 层）。
// SceneDirector.update 负责调 pruneEntities 清理消失的实体。

export const _entities = new Map();  // id -> drState
let _entLastFrame = 0;

function _newState(x, y, speed, heading, now) {
  return {
    lastX: x, lastZ: y, lastSpeed: speed, lastHeading: heading, lastTime: now,
    targetX: x, targetZ: y, targetHeading: heading,
    smoothX: x, smoothZ: y, smoothHeading: heading, smoothSpeed: speed,
    init: true
  };
}

/**
 * updateEntityDeadReckon — 喂入某 NPC 的最新真值。SceneDirector.update
 * 构建 store.entities 时对每个 entity 调一次。
 * 与 ego 同样做心跳去重（位移 < 1cm 且速度变化 < 0.1m/s 视为重复帧，
 * 不刷新 lastTime，避免外推时钟被重置）。
 */
export function updateEntityDeadReckon(id, x, y, speed, heading) {
  var now = performance.now() / 1000;
  var s = _entities.get(id);
  if (!s) {
    _entities.set(id, _newState(x, y, speed, heading, now));
    return;
  }
  if (
    Math.abs(x - s.lastX) > 0.01 ||
    Math.abs(y - s.lastZ) > 0.01 ||
    Math.abs(speed - s.lastSpeed) > 0.1
  ) {
    s.lastX = x;
    s.lastZ = y;
    s.lastSpeed = speed;
    s.lastHeading = heading;
    s.lastTime = now;
  }
}

/**
 * tickEntityDeadReckon — 推进所有 NPC 的平滑状态一帧。
 * 所有实体共享同一帧时钟（_entLastFrame），dt 与 ego 路径一致。
 */
export function tickEntityDeadReckon(nowSec) {
  if (_entities.size === 0) return;
  var now = nowSec != null ? nowSec : performance.now() / 1000;
  if (_entLastFrame === 0) _entLastFrame = now;
  var dt = now - _entLastFrame;
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0) { _entLastFrame = now; return; }
  _entLastFrame = now;
  _entities.forEach(function(s) { _advanceState(s, dt, now); });
}

/**
 * _advanceState — ego 与 entity 共用的单步推进：速度外推 + 指数平滑 +
 * 最短路径 heading lerp。state 字段布局与 _dr 一致。
 */
function _advanceState(s, dt, now) {
  // 1. 用 last speed 外推 target（真 dead reckoning）。
  if (s.lastTime > 0) {
    var elapsed = now - s.lastTime;
    if (elapsed > 2.0) elapsed = 2.0; // cap staleness
    s.targetX = s.lastX + s.lastSpeed * elapsed;
    s.targetZ = s.lastZ;
    s.targetHeading = s.lastHeading;
  }

  // 2. 帧率无关的指数平滑。
  var alphaPos = 1 - Math.exp(-LAMBDA_POS * dt);
  var alphaHeading = 1 - Math.exp(-LAMBDA_HEADING * dt);
  s.smoothX += (s.targetX - s.smoothX) * alphaPos;
  s.smoothZ += (s.targetZ - s.smoothZ) * alphaPos;
  s.smoothSpeed += (s.lastSpeed - s.smoothSpeed) * alphaPos;

  // 3. Heading: shortest-path angular lerp (handles ±π wrap)。
  var dh = s.targetHeading - s.smoothHeading;
  while (dh > Math.PI) dh -= 2 * Math.PI;
  while (dh < -Math.PI) dh += 2 * Math.PI;
  s.smoothHeading += dh * alphaHeading;
}

/**
 * getEntitySmooth — 返回某 NPC 的平滑状态 {x, y, heading, speed}。
 * VehicleView 等渲染层通过 SceneDirector 写回 store.entities 后间接消费。
 * 未初始化（id 未知）返回 null。
 */
export function getEntitySmooth(id) {
  var s = _entities.get(id);
  if (!s) return null;
  return {
    x: s.smoothX,
    y: s.smoothZ,
    heading: s.smoothHeading,
    speed: s.smoothSpeed
  };
}

/**
 * pruneEntities — 只保留 aliveIds 中的实体，清理消失的 NPC。
 * SceneDirector.update 构建 store.entities 后调一次，防止 Map 无限增长。
 */
export function pruneEntities(aliveIds) {
  _entities.forEach(function(_, id) {
    if (!aliveIds.has(id)) _entities.delete(id);
  });
}

/** resetEntities — 清空所有实体状态（切场景 / HMR 时调）。 */
export function resetEntities() {
  _entities.clear();
  _entLastFrame = 0;
}
