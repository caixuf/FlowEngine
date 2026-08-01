/**
 * AudioEngine.js — Web Audio API 音效合成
 *
 * 零外部音频文件，全部用 Web Audio API 合成：
 *   - 引擎声：sawtooth 振荡器，freq = f(speed)
 *   - 刹车声：带通滤波白噪声，gain = f(brake)
 *   - 转向灯：方波脉冲，2Hz 打断 800Hz 载波
 *
 * Phase 4 升级：为驾驶场景提供沉浸式音效反馈。
 *
 * 浏览器 autoplay 政策：AudioContext 在用户首次交互后 resume()，
 * 之前静默（不报错，只是没有声音）。
 *
 * 默认静音（_muted=true）：音效属于"用户主动开启"的增强项，
 * 打开页面就自动出声是观感事故。UI 音效按钮调 setMuted(false) 才发声。
 */

/** 创建音效引擎实例
 *  @returns {{ tick: (speed: number, throttle: number, brake: number, lights: number) => void,
 *              resume: () => void, setMuted: (m: boolean) => void,
 *              isMuted: () => boolean, dispose: () => void }} */
export function createAudioEngine() {
  let ctx = null;
  let _resumed = false;
  let _muted = true;

  // 引擎声
  let engineOsc = null;
  let engineGain = null;

  // 刹车声
  let brakeSource = null;
  let brakeGain = null;
  let brakeFilter = null;
  let _brakeActive = false;

  // 转向灯
  let tlOsc = null;
  let tlGain = null;
  let _tlActive = false;
  let _tlLastTick = 0;
  let _tlOn = false;
  let _prevLights = 0;

  // 刹车噪声 buffer
  let _noiseBuffer = null;

  const LIGHT_TURN_LEFT = 1;
  const LIGHT_TURN_RIGHT = 2;
  const LIGHT_HAZARD = 4;

  function _ensureCtx() {
    if (ctx) return true;
    try {
      ctx = new (window.AudioContext || window.webkitAudioContext)();
      if (ctx.state === 'suspended') {
        // 浏览器 autoplay 政策：需要用户交互
        ctx.resume().catch(() => {});
      }
      return true;
    } catch (e) {
      return false;  // 静默降级
    }
  }

  function _createNoiseBuffer() {
    if (!ctx) return null;
    const sr = ctx.sampleRate;
    const len = sr * 0.5;  // 0.5 秒
    const buf = ctx.createBuffer(1, len, sr);
    const data = buf.getChannelData(0);
    for (let i = 0; i < len; i++) {
      data[i] = Math.random() * 2 - 1;
    }
    return buf;
  }

  /** 启动引擎声 */
  function _startEngine() {
    if (!ctx || engineOsc) return;
    engineOsc = ctx.createOscillator();
    engineOsc.type = 'sawtooth';
    engineOsc.frequency.value = 60;

    engineGain = ctx.createGain();
    engineGain.gain.value = 0;

    engineOsc.connect(engineGain);
    engineGain.connect(ctx.destination);
    engineOsc.start();
  }

  /** 更新引擎声 */
  function _updateEngine(speed, throttle) {
    if (!engineOsc || !engineGain) return;
    const freq = 60 + (speed || 0) * 8;
    engineOsc.frequency.linearRampToValueAtTime(
      Math.min(freq, 600),
      ctx.currentTime + 0.1
    );
    const gain = 0.02 + (throttle || 0) * 0.06;
    engineGain.gain.linearRampToValueAtTime(
      Math.min(gain, 0.15),
      ctx.currentTime + 0.1
    );
  }

  /** 更新刹车声 */
  function _updateBrake(brake) {
    if (!ctx) return;
    const shouldBeActive = (brake || 0) > 0.1;
    if (shouldBeActive && !_brakeActive) {
      // 开始刹车声
      if (!_noiseBuffer) _noiseBuffer = _createNoiseBuffer();
      if (!_noiseBuffer) return;

      brakeSource = ctx.createBufferSource();
      brakeSource.buffer = _noiseBuffer;
      brakeSource.loop = true;

      brakeFilter = ctx.createBiquadFilter();
      brakeFilter.type = 'bandpass';
      brakeFilter.frequency.value = 2000;
      brakeFilter.Q.value = 0.5;

      brakeGain = ctx.createGain();
      brakeGain.gain.value = 0;

      brakeSource.connect(brakeFilter);
      brakeFilter.connect(brakeGain);
      brakeGain.connect(ctx.destination);
      brakeSource.start();
      _brakeActive = true;
    } else if (!shouldBeActive && _brakeActive) {
      // 停止刹车声
      if (brakeSource) {
        try { brakeSource.stop(); } catch (e) { /* already stopped */ }
        brakeSource.disconnect();
        brakeSource = null;
      }
      if (brakeFilter) { brakeFilter.disconnect(); brakeFilter = null; }
      if (brakeGain) { brakeGain.disconnect(); brakeGain = null; }
      _brakeActive = false;
    }

    if (_brakeActive && brakeGain) {
      brakeGain.gain.linearRampToValueAtTime(
        Math.min((brake || 0) * 0.08, 0.1),
        ctx.currentTime + 0.05
      );
    }
  }

  /** 更新转向灯声 */
  function _updateTurnLights(lights) {
    if (!ctx) return;
    const hasTurn = (lights & LIGHT_TURN_LEFT) || (lights & LIGHT_TURN_RIGHT) || (lights & LIGHT_HAZARD);
    if (hasTurn && !_tlActive) {
      tlOsc = ctx.createOscillator();
      tlOsc.type = 'square';
      tlOsc.frequency.value = 800;

      tlGain = ctx.createGain();
      tlGain.gain.value = 0;

      tlOsc.connect(tlGain);
      tlGain.connect(ctx.destination);
      tlOsc.start();
      _tlActive = true;
    } else if (!hasTurn && _tlActive) {
      if (tlOsc) { try { tlOsc.stop(); } catch (e) {} tlOsc.disconnect(); tlOsc = null; }
      if (tlGain) { tlGain.disconnect(); tlGain = null; }
      _tlActive = false;
      _tlOn = false;
    }

    if (_tlActive && tlGain && ctx) {
      // 2Hz 脉冲
      const now = ctx.currentTime;
      if (now - _tlLastTick > 0.25) {
        _tlOn = !_tlOn;
        _tlLastTick = now;
      }
      tlGain.gain.value = _tlOn ? 0.06 : 0;
    }
  }

  /** 每帧更新所有音效
   *  @param {number} speed    车速（m/s）
   *  @param {number} throttle 油门（0-1）
   *  @param {number} brake    刹车（0-1）
   *  @param {number} lights   灯光 bitmask */
  function tick(speed, throttle, brake, lights) {
    if (_muted) return;  // 默认静音：不建 ctx、不出声
    if (!_ensureCtx()) return;
    if (!_resumed) {
      if (ctx.state === 'running') _resumed = true;
      else return;  // 等待用户交互
    }

    _startEngine();
    _updateEngine(speed, throttle);
    _updateBrake(brake);
    _updateTurnLights(lights);
    _prevLights = lights;
  }

  /** 用户交互后调用（resume AudioContext） */
  function resume() {
    if (ctx && ctx.state === 'suspended') {
      ctx.resume().then(() => { _resumed = true; }).catch(() => {});
    } else if (ctx) {
      _resumed = true;
    }
  }

  /** 静音开关。静音时立刻停掉所有活跃声源（不等下一帧） */
  function setMuted(m) {
    _muted = !!m;
    if (_muted) _stopAllSources();
    else {
      _ensureCtx();
      resume();
    }
  }

  function isMuted() { return _muted; }

  function _stopAllSources() {
    if (engineOsc) { try { engineOsc.stop(); } catch (e) {} engineOsc.disconnect(); engineOsc = null; }
    if (engineGain) { engineGain.disconnect(); engineGain = null; }
    if (brakeSource) { try { brakeSource.stop(); } catch (e) {} brakeSource.disconnect(); brakeSource = null; }
    if (brakeFilter) { brakeFilter.disconnect(); brakeFilter = null; }
    if (brakeGain) { brakeGain.disconnect(); brakeGain = null; }
    if (tlOsc) { try { tlOsc.stop(); } catch (e) {} tlOsc.disconnect(); tlOsc = null; }
    if (tlGain) { tlGain.disconnect(); tlGain = null; }
    _brakeActive = false;
    _tlActive = false;
    _tlOn = false;
  }

  function dispose() {
    _stopAllSources();
    if (ctx) { ctx.close(); ctx = null; }
    _resumed = false;
  }

  return { tick, resume, setMuted, isMuted, dispose };
}