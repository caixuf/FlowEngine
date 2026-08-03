/**
 * PerfMonitor.js — 可视化模块级 PHM（健康管理）监控
 *
 * 为什么需要独立 watchdog：
 *   自动降级若放在 requestAnimationFrame 循环里，一旦 GPU 被后处理/阴影
 *   卡死，rAF 回调被节流到几乎不触发 → 降级逻辑永远执行不到 → 卡死不可恢复。
 *   本模块用独立的 setInterval 定时器（1s）做降级看门狗，即使 rAF 只剩 2fps
 *   也能采样到真实帧率并主动降档，直到渲染恢复。
 *
 * 降级阶梯（由 main.js 的 onDowngrade 回调执行）：
 *   high → medium → low → ultra（ultra 再压低渲染分辨率，最后兜底）
 *
 * 上报：每窗口把 {fps, tier, drawCalls, jank} 交给 onReport 回调，
 * 由上层 POST 到后端 /api/vis/health，让监控系统能感知可视化健康。
 */
export class PerfMonitor {
  /**
   * @param {object}   opts
   * @param {number}   opts.windowMs          采样窗口（默认 1000ms）
   * @param {number}   opts.lowFps            低帧率阈值（默认 30）
   * @param {number}   opts.downgradeWindows  连续多少窗口低帧才降级（默认 3）
   * @param {Function} opts.onDowngrade       降级回调 (fps, nextTier) => void
   * @param {Function} opts.onReport          上报回调 (stats) => void
   */
  constructor({ windowMs = 1000, lowFps = 30, downgradeWindows = 3,
                onDowngrade = null, onReport = null } = {}) {
    this._windowMs = windowMs;
    this._lowFps = lowFps;
    this._downgradeWindows = downgradeWindows;
    this._onDowngrade = onDowngrade;
    this._onReport = onReport;

    this._frames = 0;          // 当前窗口累计帧数
    this._windowStart = 0;     // 当前窗口起点（performance.now）
    this._consecutive = 0;     // 连续低帧窗口数
    this._timer = null;        // setInterval 句柄
    this._paused = false;      // 手动设档后暂停自动降级
    this._fps = 0;             // 最近窗口 FPS
    this._drawCalls = 0;       // 最近窗口 draw calls
    this._jank = 0;            // 累计卡顿窗口数
    this._lastTier = null;
  }

  /** 渲染循环每帧调用一次（累加帧数） */
  tickFrame() {
    this._frames++;
  }

  /** 读取当前帧数供外部显示（如 statsView） */
  frameCount() { return this._frames; }

  /** 暂停自动降级（手动 setPerfTier 后调用） */
  pause() { this._paused = true; }

  /** 恢复自动降级 */
  resume() { this._paused = false; }

  /** 启动 watchdog（独立 setInterval，不依赖 rAF） */
  start() {
    if (this._timer) return;
    this._windowStart = performance.now();
    this._timer = setInterval(() => this._onWindow(), this._windowMs);
  }

  /** 停止 watchdog */
  dispose() {
    if (this._timer) { clearInterval(this._timer); this._timer = null; }
  }

  /** 最近窗口 FPS */
  getFps() { return this._fps; }

  /** 单个采样窗口结算 */
  _onWindow() {
    const now = performance.now();
    const elapsed = (now - this._windowStart) / 1000;
    const frames = this._frames;
    this._frames = 0;
    this._windowStart = now;

    const fps = elapsed > 0 ? frames / elapsed : 0;
    this._fps = fps;

    // 上报（独立于降级，即使正常也上报健康状态）
    if (this._onReport) {
      this._onReport({ fps, frames, drawCalls: this._drawCalls, jank: this._jank });
    }

    // 自动降级：仅在未手动暂停时生效
    if (this._paused) { this._consecutive = 0; return; }

    if (fps < this._lowFps) {
      this._consecutive++;
      this._jank++;
      if (this._consecutive >= this._downgradeWindows && this._onDowngrade) {
        this._consecutive = 0;
        this._onDowngrade(fps);
      }
    } else {
      this._consecutive = 0;
    }
  }
}