/**
 * DeadReckon.js — Single source of truth for dead reckoning in FlowBoard
 *
 * Step 2 重构：从 tools/flowboard/js/deadreckon.js 移到 vis/core/，
 * 算法零改动。原 deadreckon.js 将在 Phase 4 清理时删除。
 *
 * Pipeline:
 *   updateDeadReckon(x, z, speed, heading)  ← called on every SSE tick
 *   tickDeadReckon()                        ← called every animation frame
 *   getDeadReckonState()                    ← read by 3D / 2D renderers
 *
 * Coordinate system (matches sim_world_node / monitor_node):
 *   X = forward (m), Z = lateral (m), heading = radians, speed = m/s.
 *
 * Frame-rate-independent smoothing:
 *   smooth += (target - smooth) * (1 - exp(-lambda * dt))
 * This decouples smoothing speed from the render rate, so 30 fps and
 * 144 fps displays behave identically.
 */

// ── Smoothing coefficients (higher = snappier) ──
// lambda=8 ≈ 0.125 lerp at 60 fps (matches the previous 0.15 factor).
// Heading uses a smaller lambda so it does not twitch on noisy input.
export var LAMBDA_POS = 8.0;
export var LAMBDA_HEADING = 6.0;

// ── Dead-reckoning state ─────────────────────────────────────────
// last*     : latest ground-truth fed in by updateDeadReckon()
// target*   : extrapolated position using last speed (the prediction)
// smooth*   : exponentially-smoothed value consumed by renderers
// lastFrameTime : wall clock of the previous tickDeadReckon() call
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
 *   1. Speed-based position extrapolation (target = last + v * dt).
 *   2. Frame-rate-independent exponential smoothing toward target.
 *   3. Shortest-path angular lerp for heading (fixes the previous
 *      wrap-around bug where crossing ±π caused a full spin).
 */
export function tickDeadReckon() {
  if (!_dr.init) return;
  var now = performance.now() / 1000;
  if (_dr.lastFrameTime === 0) _dr.lastFrameTime = now;
  // Clamp dt: a tab backgrounded for 10 s must not catapult the car
  // forward by 100 m when it returns.
  var dt = now - _dr.lastFrameTime;
  if (dt > 0.1) dt = 0.1;
  if (dt <= 0) { _dr.lastFrameTime = now; return; }
  _dr.lastFrameTime = now;

  // 1. Extrapolate target using last known speed (true dead reckoning).
  if (_dr.lastTime > 0) {
    var elapsed = now - _dr.lastTime;
    if (elapsed > 2.0) elapsed = 2.0; // cap staleness
    _dr.targetX = _dr.lastX + _dr.lastSpeed * elapsed;
    _dr.targetZ = _dr.lastZ;
    _dr.targetHeading = _dr.lastHeading;
  }

  // 2. Frame-rate-independent exponential smoothing.
  var alphaPos = 1 - Math.exp(-LAMBDA_POS * dt);
  var alphaHeading = 1 - Math.exp(-LAMBDA_HEADING * dt);
  _dr.smoothX += (_dr.targetX - _dr.smoothX) * alphaPos;
  _dr.smoothZ += (_dr.targetZ - _dr.smoothZ) * alphaPos;
  _dr.smoothSpeed += (_dr.lastSpeed - _dr.smoothSpeed) * alphaPos;

  // 3. Heading: shortest-path angular lerp (handles ±π wrap).
  var dh = _dr.targetHeading - _dr.smoothHeading;
  while (dh > Math.PI) dh -= 2 * Math.PI;
  while (dh < -Math.PI) dh += 2 * Math.PI;
  _dr.smoothHeading += dh * alphaHeading;
}

/**
 * getDeadReckonState — return the current dead-reckoning object.
 * Renderers read smoothX / smoothZ / smoothHeading / smoothSpeed
 * (and lastX / lastZ for world-anchoring obstacles & LiDAR).
 */
export function getDeadReckonState() {
  return _dr;
}
