/**
 * VisualPhysics.js — 纯前端视觉物理模型（MVC Model 层）
 *
 * 不读取 THREE.js，只根据遥测帧间差分计算车身侧倾（roll）和俯仰（pitch）
 * 目标值，供 View 层应用到 mesh.rotation。
 *
 * 后端 physics.cpp 只有 2D 自行车模型（x/y/heading），无 roll/pitch；
 * 这里用航向/速度帧间差分做视觉近似，零后端改动。
 */

const TILT_MAX = 0.06;          // clamp 上限，约 3.4°，避免夸张甩动
const ROLL_GAIN = 0.02;
const PITCH_GAIN = 0.015;
const LERP_FACTOR = 0.15;

function _clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }
function _wrapAngle(d) {
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

/**
 * 由一帧内的航向变化、车速、纵向加速度估算 roll/pitch 目标值。
 * @param {number} dHeadingWrapped  已 wrap 到 [-PI, PI] 的航向变化（rad）
 * @param {number} dt               时间间隔（s），必须 >0 且 <=0.5
 * @param {number} speed            当前速度（m/s）
 * @param {number} prevSpeed        上一帧速度（m/s）
 * @returns {{roll: number, pitch: number}}
 */
export function computeTiltTargets(dHeadingWrapped, dt, speed, prevSpeed) {
  if (dt <= 0 || dt > 0.5) return { roll: 0, pitch: 0 };
  var yawRate = dHeadingWrapped / dt;
  var accel = (speed - prevSpeed) / dt;
  var roll = -yawRate * speed * ROLL_GAIN;   // 转弯时车身向外侧倾斜
  var pitch = accel * PITCH_GAIN;             // 加速抬头/刹车点头
  return {
    roll: _clamp(roll, -TILT_MAX, TILT_MAX),
    pitch: _clamp(pitch, -TILT_MAX, TILT_MAX)
  };
}

/**
 * 单个运动实体的 tilt 状态机。
 * 封装 prev 值缓存、目标值计算和 lerp 平滑。
 */
export class TiltState {
  constructor() {
    this.prevT0 = null;
    this.prevHeading = 0;
    this.prevSpeed = 0;
    this.roll = 0;
    this.pitch = 0;
    this.rollTarget = 0;
    this.pitchTarget = 0;
  }

  /**
   * 输入新遥测，更新目标 roll/pitch。
   * @param {number} t0       当前 tick 时间戳（任意单调单位，用于判断新 tick）
   * @param {number} heading  当前航向（rad）
   * @param {number} speed    当前速度（m/s）
   */
  updateTargets(t0, heading, speed) {
    if (this.prevT0 !== null && t0 !== this.prevT0) {
      var dt = t0 - this.prevT0;
      var dh = _wrapAngle(heading - this.prevHeading);
      var targets = computeTiltTargets(dh, dt, speed, this.prevSpeed);
      this.rollTarget = targets.roll;
      this.pitchTarget = targets.pitch;
      this.prevHeading = heading;
      this.prevSpeed = speed;
    }
    this.prevT0 = t0;
  }

  /**
   * 每帧调用，lerp 当前 roll/pitch 到目标值。
   * @returns {{roll: number, pitch: number}}
   */
  tick() {
    this.roll += (this.rollTarget - this.roll) * LERP_FACTOR;
    this.pitch += (this.pitchTarget - this.pitch) * LERP_FACTOR;
    return { roll: this.roll, pitch: this.pitch };
  }

  reset() {
    this.prevT0 = null;
    this.prevHeading = 0;
    this.prevSpeed = 0;
    this.roll = 0;
    this.pitch = 0;
    this.rollTarget = 0;
    this.pitchTarget = 0;
  }
}

/**
 * Ego 专用 tilt 状态。
 * 与 Obstacle 的区别：ego 每帧都更新，因此用 performance.now() 做 dt。
 */
export class EgoTiltState extends TiltState {
  constructor() {
    super();
    this.prevTime = 0;
  }

  /**
   * @param {number} now      当前时间（s，performance.now()/1000）
   * @param {number} heading  当前航向（rad）
   * @param {number} speed    当前速度（m/s）
   */
  updateTargets(now, heading, speed) {
    if (this.prevTime > 0) {
      var dt = now - this.prevTime;
      if (dt > 0 && dt <= 0.5) {
        var dh = _wrapAngle(heading - this.prevHeading);
        var targets = computeTiltTargets(dh, dt, speed, this.prevSpeed);
        this.rollTarget = targets.roll;
        this.pitchTarget = targets.pitch;
      }
    }
    this.prevTime = now;
    this.prevHeading = heading;
    this.prevSpeed = speed;
  }

  reset() {
    super.reset();
    this.prevTime = 0;
  }
}

/**
 * 障碍物 tilt 状态池，按数组索引复用。
 */
export class TiltPool {
  constructor() {
    this._pool = [];
  }

  get(index) {
    return this._pool[index] || (this._pool[index] = new TiltState());
  }

  reset() {
    this._pool = [];
  }
}
