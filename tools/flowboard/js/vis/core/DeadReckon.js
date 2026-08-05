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
//
// 参数化（替代原硬编码 export var）：原 LAMBDA_POS / LAMBDA_HEADING 是
// 顶层 var，外部只能 import 后改本地绑定，无法运行时调整；现在通过
// setDeadReckonConfig({lambdaPos, lambdaHeading}) 在运行时调整，支持：
//   - 高速场景加大 lambda 让跟踪更紧（避免落后真值过多）
//   - 低速场景减小 lambda 抑制噪声输入导致的抖动
//   - 测试时拉到极大值关闭平滑，断言真值直通
// 默认值与原实现一致，行为保持兼容。
export const _cfg = {
  lambdaPos: 8.0,
  /* λHeading 与 λPos 对称（2026-08 修复）：旧值 6.0 使 heading 平滑比位置
   * 慢 30%（τ=167ms vs 125ms）→ 变道时位置先动、车头朝向后追 → 车身持续
   * 蟹行（0.05-0.2 m/s）→ "车屁股先移动"的观感。同速率后位置与朝向同步
   * 收敛，车头转向与横向位移一致。 */
  lambdaHeading: 8.0,
};

export function setDeadReckonConfig(p) {
  if (!p || typeof p !== 'object') return;
  if (typeof p.lambdaPos === 'number' && p.lambdaPos > 0)      _cfg.lambdaPos = p.lambdaPos;
  if (typeof p.lambdaHeading === 'number' && p.lambdaHeading > 0) _cfg.lambdaHeading = p.lambdaHeading;
}

export function getDeadReckonConfig() {
  return { lambdaPos: _cfg.lambdaPos, lambdaHeading: _cfg.lambdaHeading };
}

// ── 仿真时钟同步（数据时间轴 — NPC 顿挫根治，2026-08）────────────
// 真根因：SSE 交付节拍抖动（实测 100ms+10ms 突发成对），而外推时间轴
// 用的是"帧到达墙钟"（lastTime = performance.now()）→ 交付抖 ±50ms
// 直接变成 ±1m 的目标位置抖动 → NPC 以 ~9Hz 冲-停节拍顿挫。
// 根治：后端在 scene.t_us 导出仿真时间戳（严格均匀 60Hz），前端用
// min-tracking 估计 offset = 墙钟 − 仿真钟（取历史最小=交付延迟最小的
// 那帧），外推 elapsed = (墙钟 − offset) − 帧仿真时间。交付什么时候到
// 不再影响时间轴，只要内容带的 t_us 均匀，外推就均匀。
// offset 缓慢上漂（~6ms/s 上限）防时钟漂移让 min 卡死；跳变 >1s 视为
// 仿真重启，直接重置。
var _simClock = { offset: null };

function _syncSimClock(dataTs, wallNow) {
  var cand = wallNow - dataTs;
  if (_simClock.offset === null || Math.abs(cand - _simClock.offset) > 1.0) {
    _simClock.offset = cand;
  } else if (cand < _simClock.offset) {
    _simClock.offset = cand;
  } else {
    _simClock.offset += Math.min(cand - _simClock.offset, 0.005) * 0.02;
  }
}

/** simNow — 当前仿真时间估计（秒）；无同步数据时返回 null。 */
function _simNow(wallNow) {
  return _simClock.offset === null ? null : wallNow - _simClock.offset;
}

// ── Dead-reckoning state ─────────────────────────────────────────
// last*     : updateDeadReckon() 喂入的最新真值
// target*   : 用 last speed 外推出的预测位置
// smooth*   : 指数平滑后的值，渲染层消费
// lastFrameTime : 上一次 tickDeadReckon() 的墙钟
// lastVx/lastVy : 世界系速度（step_bicycle 的 vx/vy 含绕后轴切向分量
//                 half_wb·yaw_rate）。掉头/急转弯时 speed·(cos,sin) 外推
//                 会丢 ~34% 的横向速度 → 车尾横移（见 _advanceState）。
export var _dr = {
  lastX: 0,
  lastZ: 0,
  lastSpeed: 0,
  lastHeading: 0,
  lastVx: 0,
  lastVy: 0,
  hasVel: false,
  lastYawRate: 0,
  hasYawRate: false,
  lastTime: 0,
  dataTime: null,
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
  _dr.lastVx = 0;
  _dr.lastVy = 0;
  _dr.hasVel = false;
  _dr.lastYawRate = 0;
  _dr.hasYawRate = false;
  _dr.lastTime = 0;
  _dr.dataTime = null;
  _simClock.offset = null;
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
 * @param {number} [vx]    world X velocity (m/s) — step_bicycle 的中心速度，
 *                         含绕后轴切向分量；缺省时回退 speed·(cos,sin) 外推
 * @param {number} [vy]    world Z (lateral) velocity (m/s)
 * @param {number} [yawRate] 横摆角速度 (rad/s) — heading 外推用，与位置外推
 *                         同构（斜坡 vs 阶跃的平滑滞后差异消除，2026-08）
 * @param {number} [dataTs] 帧仿真时间戳 (秒, scene.t_us/1e6) — 提供时外推
 *                         时间轴切到数据时钟，SSE 交付抖动不再影响平滑
 */
export function updateDeadReckon(x, z, speed, heading, vx, vy, yawRate, dataTs) {
  var now = performance.now() / 1000;
  var hasTs = (typeof dataTs === 'number' && isFinite(dataTs) && dataTs > 0);
  if (hasTs) _syncSimClock(dataTs, now);
  var hasVel = (typeof vx === 'number' && isFinite(vx) &&
                typeof vy === 'number' && isFinite(vy));
  var hasYaw = (typeof yawRate === 'number' && isFinite(yawRate));
  if (
    !_dr.init ||
    _dr.hasVel !== hasVel ||
    _dr.hasYawRate !== hasYaw ||
    Math.abs(x - _dr.lastX) > 0.01 ||
    Math.abs(z - _dr.lastZ) > 0.01 ||
    Math.abs(speed - _dr.lastSpeed) > 0.1 ||
    (hasVel && (Math.abs(vx - _dr.lastVx) > 0.1 || Math.abs(vy - _dr.lastVy) > 0.1))
  ) {
    _dr.lastX = x;
    _dr.lastZ = z;
    _dr.lastSpeed = speed;
    _dr.lastHeading = heading;
    _dr.hasVel = hasVel;
    if (hasVel) { _dr.lastVx = vx; _dr.lastVy = vy; }
    _dr.hasYawRate = hasYaw;
    if (hasYaw) { _dr.lastYawRate = yawRate; }
    _dr.lastTime = now;
    _dr.dataTime = hasTs ? dataTs : null;
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

function _newState(x, y, speed, heading, now, vx, vy, dataTs) {
  var hasVel = (typeof vx === 'number' && isFinite(vx) &&
                typeof vy === 'number' && isFinite(vy));
  return {
    lastX: x, lastZ: y, lastSpeed: speed, lastHeading: heading,
    lastVx: hasVel ? vx : 0, lastVy: hasVel ? vy : 0, hasVel: hasVel,
    lastTime: now,
    dataTime: (typeof dataTs === 'number' && isFinite(dataTs) && dataTs > 0) ? dataTs : null,
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
export function updateEntityDeadReckon(id, x, y, speed, heading, vx, vy, dataTs) {
  var now = performance.now() / 1000;
  var hasTs = (typeof dataTs === 'number' && isFinite(dataTs) && dataTs > 0);
  if (hasTs) _syncSimClock(dataTs, now);
  var s = _entities.get(id);
  if (!s) {
    _entities.set(id, _newState(x, y, speed, heading, now, vx, vy, dataTs));
    return;
  }
  var hasVel = (typeof vx === 'number' && isFinite(vx) &&
                typeof vy === 'number' && isFinite(vy));
  if (
    s.hasVel !== hasVel ||
    Math.abs(x - s.lastX) > 0.01 ||
    Math.abs(y - s.lastZ) > 0.01 ||
    Math.abs(speed - s.lastSpeed) > 0.1 ||
    (hasVel && (Math.abs(vx - s.lastVx) > 0.1 || Math.abs(vy - s.lastVy) > 0.1))
  ) {
    s.lastX = x;
    s.lastZ = y;
    s.lastSpeed = speed;
    s.lastHeading = heading;
    s.hasVel = hasVel;
    if (hasVel) { s.lastVx = vx; s.lastVy = vy; }
    s.lastTime = now;
    s.dataTime = hasTs ? dataTs : null;
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
  // 1. 外推 target（真 dead reckoning）。
  if (s.lastTime > 0) {
    // 时间轴选择：帧带仿真时间戳（dataTime）且时钟已同步 → 用数据时间轴
    // （elapsed = 仿真当前时间 − 帧仿真时间，SSE 交付抖动完全不影响）；
    // 否则回退到达墙钟轴（旧行为）。
    var elapsed;
    var simNow = (s.dataTime !== null && s.dataTime !== undefined) ? _simNow(now) : null;
    if (simNow !== null) {
      elapsed = simNow - s.dataTime;
      if (elapsed < 0) elapsed = 0;
    } else {
      elapsed = now - s.lastTime;
    }
    if (elapsed > 2.0) elapsed = 2.0; // cap staleness
    // 位置外推与 heading 外推必须**同构**（否则位置与朝向解耦 → 车身侧滑）。
    // 旧实现：位置用 lastVx/lastVy 沿**切线直线**外推，heading 用 lastYawRate
    // 做**旋转**外推。急弯/掉头（高 yaw_rate）时两块 SSE 包之间（~200ms）中心
    // 沿切线走、车头却转 → 平滑后的位置与朝向收敛到不一致目标 → 车身侧滑
    // （"屁股横移"）。2D 因车钉在屏中央只渲染旋转、看不到平移所以无感；
    // 3D 真实平移到世界坐标才暴露。修复：把速度向量旋转到**弧中点**方向
    // （圆弧/割线近似），让位置与朝向同步收敛。
    var dHeading = s.hasYawRate ? s.lastYawRate * elapsed : 0;
    if (s.hasVel) {
      // 世界系速度外推：step_bicycle 的 vx/vy 是车辆中心速度，含绕后轴的
      // 切向分量 half_wb·yaw_rate·(-sin h, cos h)。只用 speed·(cos,sin)
      // 会在掉头/急转弯时丢掉 ~34% 的横向速度 → 中心横向漂移。vx/vy 由后端
      // 直接发布，无需前端猜半轴距。
      if (dHeading !== 0) {
        var mid = dHeading * 0.5;
        var c = Math.cos(mid), sn = Math.sin(mid);
        var vx2 = s.lastVx * c - s.lastVy * sn;
        var vy2 = s.lastVx * sn + s.lastVy * c;
        s.targetX = s.lastX + vx2 * elapsed;
        s.targetZ = s.lastZ + vy2 * elapsed;
      } else {
        s.targetX = s.lastX + s.lastVx * elapsed;
        s.targetZ = s.lastZ + s.lastVy * elapsed;
      }
    } else {
      // 回退：无 vx/vy（旧 payload / vehicle-only 路径）时沿 heading 圆弧外推
      //（与 heading 旋转一致），无 yaw_rate 才退化为直线。
      if (Math.abs(dHeading) > 1e-6) {
        var th0 = s.lastHeading;
        var R = s.lastSpeed / s.lastYawRate;
        s.targetX = s.lastX + R * (Math.sin(th0 + dHeading) - Math.sin(th0));
        s.targetZ = s.lastZ + R * (Math.cos(th0) - Math.cos(th0 + dHeading));
      } else {
        s.targetX = s.lastX + Math.cos(s.lastHeading) * s.lastSpeed * elapsed;
        s.targetZ = s.lastZ + Math.sin(s.lastHeading) * s.lastSpeed * elapsed;
      }
    }
    // heading 外推（与位置同构）：lastHeading + yawRate·elapsed。
    // 旧实现 targetHeading = lastHeading（每包阶跃）→ 指数平滑对"阶跃"
    // 目标有跟踪滞后、对"斜坡"（位置外推）无滞后 → 位置领先朝向 →
    // 车身横着滑（"车屁股平移"的前端根源，2026-08 修复）。
    if (s.hasYawRate) {
      s.targetHeading = s.lastHeading + s.lastYawRate * elapsed;
    } else {
      s.targetHeading = s.lastHeading;
    }
  }

  // 2. 帧率无关的指数平滑。读 _cfg 而非 LAMBDA_POS/LAMBDA_HEADING 顶层常量，
  // 让 setDeadReckonConfig() 的运行时调整立即生效。
  var alphaPos = 1 - Math.exp(-_cfg.lambdaPos * dt);
  var alphaHeading = 1 - Math.exp(-_cfg.lambdaHeading * dt);
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
