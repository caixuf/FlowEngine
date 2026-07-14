// ═════════════════════════════════════════════════════════════════════
// deadreckon.js — Dead reckoning state and functions for FlowBoard
// ES module.
//
// Predicts position between sparse data updates (~1 Hz SSE) so the
// car moves smoothly at 60 fps instead of freezing between data ticks.
// ═════════════════════════════════════════════════════════════════════

// ── Dead-reckoning state ─────────────────────────────────────────
// Latest ground-truth from topoData, target for smooth interpolation,
// and smoothed (lerp) values consumed by the renderer.
export var _dr = {
  lastX: 0,
  lastZ: 0,
  lastSpeed: 0,
  lastHeading: 0,
  lastTime: 0,
  targetX: 0,
  targetZ: 0,
  smoothX: 0,
  smoothZ: 0,
  smoothHeading: 0,
  init: false
};

// ── 2D canvas state (used by the 2D fallback renderer) ──────────
export var _2d = {
  canvas: null,
  ctx: null,
  active: false,
  animId: 0,
  frame: 0,
  w: 0,
  h: 0,
  ego: { x: 0, y: 0, heading: 0, speed: 0 },
  egoT: { x: 0, y: 0, heading: 0, speed: 0 },
  trail: [],
  scale: 8
};

/**
 * initDeadReckon — (re)initialise the dead-reckoning state to zero.
 */
export function initDeadReckon() {
  _dr.lastX = 0;
  _dr.lastZ = 0;
  _dr.lastSpeed = 0;
  _dr.lastHeading = 0;
  _dr.lastTime = 0;
  _dr.targetX = 0;
  _dr.targetZ = 0;
  _dr.smoothX = 0;
  _dr.smoothZ = 0;
  _dr.smoothHeading = 0;
  _dr.init = false;
}

/**
 * updateDeadReckon — feed fresh ground-truth into the dead-reckoning
 * state.  Called on every SSE data tick.
 *
 * @param {number} x       world X position (forward)
 * @param {number} z       world Z position (lateral)
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
    _dr.init = true;
  }
}

/**
 * getDeadReckonState — return the current dead-reckoning object.
 * Useful for one-shot reads without holding a reference.
 */
export function getDeadReckonState() {
  return _dr;
}

// ═════════════════════════════════════════════════════════════════
// Global references — so the monolithic <script> block can access
// these variables and functions without an import statement.
// ═════════════════════════════════════════════════════════════════
window._dr = _dr;
window._2d = _2d;
window.initDeadReckon = initDeadReckon;
window.updateDeadReckon = updateDeadReckon;
window.getDeadReckonState = getDeadReckonState;
