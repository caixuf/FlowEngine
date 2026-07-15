// scene2d.js — 2D Canvas rendering (top-down view, road, cars, obstacles, HUD)
//
// Exports:
//   init2D()         — Initialize 2D canvas & state; fallback if no Three.js
//   init2DFallback() — Direct 2D fallback when WebGL/Three.js unavailable
//   draw2D()         — Main 2D top-down draw (road, cars, obstacles, HUD)
//   switchSceneView(view) — Switch between '3d' and '2d' scene modes
//   _2d              — 2D state object
//
// Ego smoothing is owned by deadreckon.js (_dr.smooth*). The 2D renderer
// reads those values in draw2D() via tickDeadReckon() in the anim loop.
//
// Phase 4.9 cleanup: no `window.X = X` exports — all entry points are
// reached via ES module imports.  app.js re-publishes them under the
// single `window.flowboard` namespace for inline-onclick handlers.

import { safeCall, reportDiag } from './utils.js';
import { _dr, tickDeadReckon } from './deadreckon.js';

// ── 2D state ────────────────────────────────────────────────────────────────
// egoT / gpsHistory were removed: dead-reckoning state (last*/smooth*) is
// now owned by deadreckon.js and fed by app.js sync2DTarget().
var _2d = {
  canvas: null, ctx: null, active: false, animId: 0, frame: 0, w: 0, h: 0,
  trail: [],                                               // last N positions
  scale: 8                                                 // px per meter
};

// Module-internal obstacle target buffers (Phase 4.9: no longer on window)
var _obsTargets2d = [];
var _obsVelBuf = [];

// Live topology data (Phase 4.9: no longer read from window.topoData).
// app.js calls setTopoData() from sync2DTarget()/updateAll() with the
// latest snapshot; draw2D() reads it from this module-scoped variable.
var _topoData = { nodes: [], metrics: {} };
export function setTopoData(d) { _topoData = d || _topoData; }

// Utility: "#rrggbb" → "r,g,b" string for rgba()
function hexToRgb(hex) {
  var r = parseInt(hex.slice(1, 3), 16),
      g = parseInt(hex.slice(3, 5), 16),
      b = parseInt(hex.slice(5, 7), 16);
  return r + ',' + g + ',' + b;
}

// Obstacle color/label maps — defined once, shared across all draw2D calls.
var OBS_COL = { car: '#ff9944', truck: '#ff4422', pedestrian: '#44ee88', cyclist: '#44ccff', vehicle: '#ffaa33', unknown: '#aaaaaa' };
var OBS_LBL = { car: 'CAR', truck: 'TRUCK', pedestrian: 'PED', cyclist: 'CYC', vehicle: 'VEH', unknown: 'OBJ' };

// ── Public API ──────────────────────────────────────────────────────────────

export function init2D() {
  // No monkey-patching of updateAll() — the 2D target (egoT) is now
  // synced by app.js sync2DTarget() inside the normal updateAll()
  // pipeline, and ego smoothing is owned by deadreckon.js.
  // If no Three.js at all, go straight to 2D fallback
  if (typeof THREE === 'undefined') {
    init2DFallback(true);
    return;
  }
  // Otherwise 3D will try to init; 2D may still be activated later
  // by the 3-second fallback timer managed in flowboard.html
}

export function init2DFallback(force) {
  // If 3D scene is already ready, don't override (unless forced)
  if (!force && (_2d.active || (typeof sceneReady !== 'undefined' && sceneReady))) return;
  var el = document.getElementById('scene2d');
  if (!el) return;
  _2d.active = true;
  _2d.canvas = el;
  _2d.ctx = el.getContext('2d');
  // HiDPI
  _2d.w = el.clientWidth || 800;
  _2d.h = el.clientHeight || 400;
  el.width = _2d.w * 2;
  el.height = _2d.h * 2;
  el.style.width = _2d.w + 'px';
  el.style.height = _2d.h + 'px';
  _2d.ctx.scale(2, 2);
  el.style.display = '';
  var msg = document.getElementById('scene3d-msg');
  if (msg) msg.style.display = 'none';
  _2dAnimLoop();
}

// ── Internal: animation loop ──
// Smoothing is centralised in deadreckon.js — this loop just ticks the
// engine and redraws. 3D and 2D now share identical smoothing behaviour.
function _2dAnimLoop() {
  if (!_2d.active) return;
  _2d.animId = requestAnimationFrame(_2dAnimLoop);
  _2d.frame++;
  tickDeadReckon();
  draw2D();
}

export function draw2D() {
  if (!_2d.active || !_2d.ctx) return;
  var ctx = _2d.ctx, W = _2d.w, H = _2d.h;
  var carX = W / 2, carY = H * 0.65, s = _2d.scale;
  // Read smoothed ego state from the central dead-reckoning engine.
  // 2D uses (x=forward, y=lateral) which maps to _dr (smoothX, smoothZ).
  var e = {
    x: _dr.smoothX,
    y: _dr.smoothZ,
    heading: _dr.smoothHeading,
    speed: _dr.smoothSpeed
  };

  // ── Background (dark ADAS display) ──
  ctx.fillStyle = '#060a0f';
  ctx.fillRect(0, 0, W, H);

  // ── Road: NS only — no intersection. Left/right shoulders. ──
  var rw = 3.5 * s;           // one lane width in px
  var rdW = 2 * rw;           // 2 lanes each side = 4-lane road half-width
  // Road drawn in ego-relative frame: world lane-center offset from screen center
  // so the ego marker correctly sits inside its actual lane.
  var cx = carX + (e.y || 0) * s;
  // Longitudinal scroll: dashed markings flow backwards as ego advances
  var scroll = ((e.x || 0) * s) % (s * 3);
  // Shoulder grass
  ctx.fillStyle = '#0b1008';
  ctx.fillRect(0, 0, cx - rdW, H);
  ctx.fillRect(cx + rdW, 0, W - (cx + rdW), H);
  // Kerb strip
  ctx.fillStyle = '#1a2218';
  ctx.fillRect(cx - rdW - 6, 0, 6, H);
  ctx.fillStyle = '#1a2218';
  ctx.fillRect(cx + rdW, 0, 6, H);
  // Asphalt
  ctx.fillStyle = '#111520';
  ctx.fillRect(cx - rdW, 0, rdW * 2, H);

  // ── Range rings (semi-circles ahead of ego) ──
  ctx.save();
  [10, 20, 30, 50, 80].forEach(function (r) {
    var pr = r * s;
    if (pr > carY + 10) return;
    ctx.strokeStyle = 'rgba(88,166,255,0.07)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 6]);
    ctx.beginPath();
    ctx.arc(carX, carY, pr, -Math.PI, 0);
    ctx.stroke();
    ctx.fillStyle = 'rgba(88,166,255,0.18)';
    ctx.font = "8px 'JetBrains Mono',monospace";
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.setLineDash([]);
    ctx.fillText(r + 'm', carX + pr + 4, carY);
    ctx.textAlign = 'right';
    ctx.fillText(r + 'm', carX - pr - 4, carY);
  });
  ctx.setLineDash([]);
  ctx.restore();

  // ── Lane markings ──
  ctx.lineWidth = 2;
  // Solid road edge lines
  ctx.strokeStyle = '#2a3a28';
  ctx.setLineDash([]);
  ctx.beginPath();
  ctx.moveTo(cx - rdW, 0);
  ctx.lineTo(cx - rdW, H);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx + rdW, 0);
  ctx.lineTo(cx + rdW, H);
  ctx.stroke();
  // Inner dashed lane dividers (1 divider each side of center)
  ctx.strokeStyle = '#223322';
  ctx.lineWidth = 2;
  ctx.setLineDash([s * 1.5, s * 1.2]);
  ctx.lineDashOffset = scroll;
  for (var li = -1; li <= 1; li += 2) {
    ctx.beginPath();
    ctx.moveTo(cx + li * rw, 0);
    ctx.lineTo(cx + li * rw, H);
    ctx.stroke();
  }
  // Double center line (yellow)
  ctx.strokeStyle = '#554422';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([s * 4, s * 1.5]);
  ctx.lineDashOffset = scroll;
  ctx.beginPath();
  ctx.moveTo(cx - 3, 0);
  ctx.lineTo(cx - 3, H);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx + 3, 0);
  ctx.lineTo(cx + 3, H);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.lineDashOffset = 0;

  // ── Forward-distance ruler (right side of road) ──
  ctx.font = "8px 'JetBrains Mono',monospace";
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  for (var d = 10; d <= 120; d += 10) {
    var dy = carY - d * s;
    if (dy < 4 || dy > H - 4) continue;
    ctx.strokeStyle = 'rgba(88,166,255,0.06)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(cx - rdW, dy);
    ctx.lineTo(cx + rdW, dy);
    ctx.stroke();
    if (d % 20 === 0) {
      ctx.fillStyle = 'rgba(88,166,255,0.22)';
      ctx.fillText(d + 'm', cx + rdW + 8, dy);
    }
  }

  // ── Ego trail ──
  if (_2d.trail.length > 1) {
    ctx.strokeStyle = 'rgba(88,166,255,0.22)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    for (var ti = 0; ti < _2d.trail.length; ti++) {
      var tp = _2d.trail[ti], tpx = carX - (tp.y - e.y) * s, tpy = carY - (tp.x - e.x) * s;
      if (ti === 0) ctx.moveTo(tpx, tpy);
      else ctx.lineTo(tpx, tpy);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // ── LiDAR point cloud ──
  var scn = (_topoData.metrics || {}).scene;
  if (scn && scn.lidar && scn.lidar.length) {
    scn.lidar.forEach(function (pt) {
      var rx = pt[0] || 0, ry = pt[1] || 0;
      var px = carX - ry * s, py = carY - rx * s;
      var dist = Math.sqrt(rx * rx + ry * ry);
      var t = Math.min(1, dist / 60);
      var lr = Math.floor(255 * (1 - t) + 40 * t),
          lg = Math.floor(100 * (1 - t) + 150 * t),
          lb = Math.floor(40 * (1 - t) + 255 * t);
      ctx.fillStyle = 'rgba(' + lr + ',' + lg + ',' + lb + ',0.75)';
      ctx.beginPath();
      ctx.arc(px, py, 1.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // ── Obstacles (ADAS-HMI bounding-box style) ──
  if (!_obsTargets2d) _obsTargets2d = [];
  if (!_obsVelBuf) _obsVelBuf = [];
  var _dtNow = performance.now() / 1000;
  if (scn && scn.obstacles && scn.obstacles.length) {
    scn.obstacles.forEach(function (o, i) {
      var egVx = e.speed * Math.cos(e.heading || 0),
          egVy = e.speed * Math.sin(e.heading || 0);
      var relVx = (o.vx || 0) - egVx, relVy = (o.vy || 0) - egVy;
      // Data target in screen coords (data updates ~10Hz)
      var dtx = carX - (o.y || 0) * s, dty = carY - (o.x || 0) * s;
      var cur = _obsTargets2d[i];
      // isNaN guard: handles stale entries after obstacle count changes
      if (!cur || isNaN(cur.x)) {
        cur = { x: dtx, y: dty, lastDataX: o.x, lastDataY: o.y, lastDataT: _dtNow };
        _obsTargets2d[i] = cur;
      }
      // Detect data update
      if (o.x !== cur.lastDataX || o.y !== cur.lastDataY) {
        // Large jump in data space → snap instead of lerp
        var jump = Math.abs((o.x || 0) - (cur.lastDataX || 0)) + Math.abs((o.y || 0) - (cur.lastDataY || 0));
        cur.lastDataX = o.x;
        cur.lastDataY = o.y;
        cur.lastDataT = _dtNow;
        if (jump > 20) { cur.x = dtx; cur.y = dty; }
      }
      // Extrapolate screen position using relative velocity since last data tick
      var age = Math.min(_dtNow - cur.lastDataT, 0.5);
      var ex = carX - (((o.y || 0) + relVy * age)) * s,
          ey = carY - (((o.x || 0) + relVx * age)) * s;
      // Smooth toward extrapolated target
      cur.x += (ex - cur.x) * 0.28;
      cur.y += (ey - cur.y) * 0.28;
      var Wd = Math.max(10, (o.wid || 2) * s),
          L = Math.max(14, (o.len || 4) * s);
      var col = OBS_COL[o.type] || OBS_COL.unknown;
      var lbl = OBS_LBL[o.type] || OBS_LBL.unknown;
      var dist = Math.sqrt((o.x || 0) * (o.x || 0) + (o.y || 0) * (o.y || 0));
      // Don't draw if directly on top of ego
      if (dist < 2) return;
      ctx.save();
      ctx.translate(cur.x, cur.y);
      // Drop shadow
      ctx.fillStyle = 'rgba(0,0,0,0.4)';
      ctx.fillRect(-Wd / 2 + 2, -L / 2 + 2, Wd, L);
      // Fill (semi-transparent, type color)
      ctx.fillStyle = 'rgba(' + hexToRgb(col) + ',0.13)';
      ctx.fillRect(-Wd / 2, -L / 2, Wd, L);
      // Bright outline (solid)
      ctx.strokeStyle = col;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([]);
      ctx.strokeRect(-Wd / 2, -L / 2, Wd, L);
      // Front indicator bar (shows heading direction)
      ctx.fillStyle = col;
      ctx.globalAlpha = 0.9;
      ctx.fillRect(-Wd / 2, -L / 2, Wd, 3);
      ctx.globalAlpha = 1;
      // Type label (centered)
      ctx.fillStyle = col;
      ctx.font = "bold 8px 'JetBrains Mono',monospace";
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(lbl, 0, 0);
      // Distance label (below box)
      ctx.fillStyle = 'rgba(200,200,200,0.55)';
      ctx.font = "8px 'JetBrains Mono',monospace";
      ctx.fillText(dist.toFixed(0) + 'm', 0, L / 2 + 8);
      // Velocity arrow (relative motion vs ego)
      var rv = Math.sqrt(relVx * relVx + relVy * relVy);
      if (rv > 0.5) {
        var arA = Math.atan2(-relVy, relVx);
        var arLen = Math.min(rv * s * 0.3, L * 0.7);
        ctx.strokeStyle = col;
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 0.7;
        ctx.beginPath();
        ctx.moveTo(0, -L / 2 - 2);
        ctx.lineTo(Math.cos(arA) * arLen, -L / 2 - 2 - Math.sin(arA) * arLen);
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      ctx.restore();
    });
  } else {
    // Static demo obstacles when no live data
    [{ x: 20, y: -1.75, col: '#ff9944', lbl: 'CAR' }, { x: 40, y: 1.75, col: '#ff4422', lbl: 'CAR' }, { x: 30, y: 8, col: '#44ee88', lbl: 'PED' }].forEach(function (p) {
      var bx = carX - p.y * s, by = carY - p.x * s;
      ctx.fillStyle = 'rgba(0,0,0,0.35)';
      ctx.fillRect(bx - 13 + 2, by - 9 + 2, 26, 18);
      ctx.fillStyle = 'rgba(' + hexToRgb(p.col) + ',0.12)';
      ctx.fillRect(bx - 13, by - 9, 26, 18);
      ctx.strokeStyle = p.col;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(bx - 13, by - 9, 26, 18);
      ctx.fillStyle = p.col;
      ctx.font = "bold 8px 'JetBrains Mono',monospace";
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(p.lbl, bx, by);
      ctx.fillStyle = 'rgba(200,200,200,0.45)';
      ctx.font = '8px monospace';
      ctx.fillText(Math.round(Math.sqrt(p.x * p.x + p.y * p.y)) + 'm', bx, by + 14);
    });
  }

  // ── Ego vehicle (detailed top-down view, with subtle speed animation) ──
  ctx.save();
  ctx.translate(carX, carY);
  ctx.rotate(-e.heading);
  // Speed-based forward tilt (visual acceleration cue)
  if (e.speed > 0.5) ctx.translate(0, -Math.min(3, e.speed * 0.3));
  // Glow (pulses with speed)
  var glowPulse = 0.15 + Math.sin(_2d.frame * 0.1) * 0.05 + e.speed * 0.01;
  var glow = ctx.createRadialGradient(0, 0, 4, 0, 0, 22);
  glow.addColorStop(0, 'rgba(88,166,255,' + glowPulse + ')');
  glow.addColorStop(1, 'rgba(88,166,255,0)');
  ctx.fillStyle = glow;
  ctx.beginPath();
  ctx.arc(0, 0, 22, 0, Math.PI * 2);
  ctx.fill();
  // Shadow
  ctx.fillStyle = 'rgba(0,0,0,0.35)';
  ctx.fillRect(-11, 2, 22, 42);
  // Body
  var bodyGrad = ctx.createLinearGradient(0, -22, 0, 22);
  bodyGrad.addColorStop(0, '#6ab4ff');
  bodyGrad.addColorStop(0.5, '#4488dd');
  bodyGrad.addColorStop(1, '#3377cc');
  ctx.fillStyle = bodyGrad;
  ctx.beginPath();
  // Rounded car body (bw = half-width, bh = half-length; -Y is forward)
  var bw = 9, bh = 20;
  ctx.moveTo(-bw + 4, -bh);
  ctx.lineTo(bw - 4, -bh);
  ctx.quadraticCurveTo(bw, -bh, bw, -bh + 4);
  ctx.lineTo(bw, bh - 6);
  ctx.quadraticCurveTo(bw, bh, bw - 4, bh);
  ctx.lineTo(-bw + 4, bh);
  ctx.quadraticCurveTo(-bw, bh, -bw, bh - 6);
  ctx.lineTo(-bw, -bh + 4);
  ctx.quadraticCurveTo(-bw, -bh, -bw + 4, -bh);
  ctx.closePath();
  ctx.fill();
  // Roof/windshield
  ctx.fillStyle = '#5599cc';
  ctx.fillRect(-5, -7, 10, 14);
  // Windshield (front, lighter)
  ctx.fillStyle = '#99ccff';
  ctx.fillRect(-4, -9, 8, 6);
  // Rear window
  ctx.fillStyle = '#4480b0';
  ctx.fillRect(-4, 6, 8, 5);
  // Front bumper accent
  ctx.fillStyle = '#c9d1d9';
  ctx.fillRect(-4, -bh + 1, 8, 2);
  // Wheels
  ctx.fillStyle = '#111';
  ctx.fillRect(-11, -16, 3.5, 8);
  ctx.fillRect(7.5, -16, 3.5, 8);
  ctx.fillRect(-11, 8, 3.5, 8);
  ctx.fillRect(7.5, 8, 3.5, 8);
  // Front direction arrow
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.moveTo(0, -bh - 4);
  ctx.lineTo(-4, -bh + 4);
  ctx.lineTo(4, -bh + 4);
  ctx.fill();
  ctx.restore();

  // ── Speed indicator bar (top center) ──
  var barW = 120, barH = 6, barX = W / 2 - barW / 2, barY = carY - rdW - 16;
  ctx.fillStyle = '#1a1e28';
  ctx.fillRect(barX, barY, barW, barH);
  var spdFrac = Math.min(1, e.speed / 20);
  var spdGrad = ctx.createLinearGradient(barX, 0, barX + barW, 0);
  spdGrad.addColorStop(0, '#3fb950');
  spdGrad.addColorStop(0.5, '#d29922');
  spdGrad.addColorStop(1, '#f85149');
  ctx.fillStyle = spdGrad;
  ctx.fillRect(barX, barY, barW * spdFrac, barH);

  // ── HUD (bottom bar) ──
  var hudY = H - 52, hudH = 52;
  // Background
  var hudGrad = ctx.createLinearGradient(0, hudY, 0, H);
  hudGrad.addColorStop(0, 'rgba(0,0,0,0.75)');
  hudGrad.addColorStop(1, 'rgba(0,0,0,0.9)');
  ctx.fillStyle = hudGrad;
  ctx.fillRect(0, hudY, W, hudH);
  ctx.strokeStyle = '#1a1f2b';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, hudY);
  ctx.lineTo(W, hudY);
  ctx.stroke();

  // ── HUD content ──
  ctx.textBaseline = 'middle';
  // SPEED — large
  ctx.fillStyle = '#58a6ff';
  ctx.font = "bold 42px 'JetBrains Mono',monospace";
  ctx.textAlign = 'left';
  ctx.fillText((e.speed || 0).toFixed(1), 18, hudY + hudH / 2 - 3);
  ctx.fillStyle = '#8b949e';
  ctx.font = "12px 'Inter',system-ui,sans-serif";
  ctx.fillText('m/s', 86, hudY + hudH / 2 - 3);
  ctx.fillText((e.speed * 3.6 || 0).toFixed(0) + ' km/h', 86, hudY + hudH / 2 + 12);
  // TARGET
  var v = (_topoData.metrics || {}).vehicle || {};
  ctx.fillStyle = '#8b949e';
  ctx.font = "10px 'Inter',system-ui,sans-serif";
  ctx.textAlign = 'left';
  ctx.fillText('TARGET', 150, hudY + 11);
  ctx.fillStyle = '#fff';
  ctx.font = "bold 20px 'JetBrains Mono',monospace";
  ctx.fillText((v.target_speed || 0).toFixed(1), 150, hudY + 30);
  // THROTTLE gauge
  ctx.fillStyle = '#8b949e';
  ctx.font = "10px 'Inter',system-ui,sans-serif";
  ctx.fillText('THR', 240, hudY + 11);
  ctx.fillStyle = '#1a1e28';
  ctx.fillRect(240, hudY + 20, 80, 10);
  var thr = v.throttle || 0;
  ctx.fillStyle = thr > 0.6 ? '#d29922' : thr > 0.3 ? '#58a6ff' : '#3fb950';
  ctx.fillRect(240, hudY + 20, 80 * thr, 10);
  ctx.strokeStyle = '#2a2f3a';
  ctx.lineWidth = 1;
  ctx.strokeRect(240, hudY + 20, 80, 10);
  ctx.fillStyle = '#fff';
  ctx.font = "bold 10px 'JetBrains Mono',monospace";
  ctx.textAlign = 'left';
  ctx.fillText((thr * 100).toFixed(0) + '%', 248, hudY + 31);
  // BRAKE gauge
  ctx.fillStyle = '#8b949e';
  ctx.font = "10px 'Inter',system-ui,sans-serif";
  ctx.fillText('BRK', 340, hudY + 11);
  ctx.fillStyle = '#1a1e28';
  ctx.fillRect(340, hudY + 20, 60, 10);
  var brk = v.brake || 0;
  ctx.fillStyle = brk > 0 ? '#f85149' : '#252d3a';
  ctx.fillRect(340, hudY + 20, 60 * Math.min(1, brk * 3), 10);
  ctx.strokeStyle = '#2a2f3a';
  ctx.lineWidth = 1;
  ctx.strokeRect(340, hudY + 20, 60, 10);
  ctx.fillStyle = '#fff';
  ctx.font = "bold 10px 'JetBrains Mono',monospace";
  ctx.fillText((brk * 100).toFixed(0) + '%', 345, hudY + 31);
  // MODE
  var err = v.error || 0;
  var mode = err < -0.3 ? 'BRAKE' : err > 0.3 ? 'ACCEL' : '⏺ HOLD';
  var modeColor = err < -0.3 ? '#f85149' : err > 0.3 ? '#3fb950' : '#8b949e';
  ctx.fillStyle = modeColor;
  ctx.font = "bold 14px 'Inter',system-ui,sans-serif";
  ctx.textAlign = 'left';
  ctx.fillText(mode, 430, hudY + 20);
  // ERROR
  ctx.fillStyle = '#8b949e';
  ctx.font = "10px 'JetBrains Mono',monospace";
  ctx.fillText('err ' + err.toFixed(1) + ' m/s', 430, hudY + 36);
  // ODO
  ctx.fillStyle = '#484f58';
  ctx.font = "11px 'JetBrains Mono',monospace";
  ctx.textAlign = 'right';
  ctx.fillText('odo ' + (v.x || 0).toFixed(1) + ' m', W - 20, hudY + hudH / 2);
}

export function switchSceneView(mode) {
  var c2d = document.getElementById('scene2d');
  var c3d = document.getElementById('scene3d');
  document.querySelectorAll('#scene-view-btns .toggle-btn').forEach(function (b) {
    b.classList.toggle('active', b.dataset.view === mode);
  });
  if (mode === '2d') {
    if (c3d) c3d.style.display = 'none';
    if (c2d) c2d.style.display = '';
    if (!_2d.active) init2DFallback(true);
  } else {
    if (c2d) c2d.style.display = 'none';
    if (c3d) c3d.style.display = '';
    if (_2d.animId) cancelAnimationFrame(_2d.animId);
    _2d.active = false;
    if (typeof sceneReady !== 'undefined' && !sceneReady && typeof THREE !== 'undefined') {
      try {
        if (typeof init3DScene !== 'undefined') { init3DScene(); sceneReady = true; }
      } catch (e) {}
    }
    if (typeof resize3D !== 'undefined') setTimeout(resize3D, 100);
  }
}

// All public functions are declared with `export function ...` at their
// definition site (Phase 4.9: replaces all window.* assignments).
//
// _2d is the only non-function module export — used by app.js for the
// 2D trail. Exported explicitly because it's a `var` (mutable state).
export { _2d };
