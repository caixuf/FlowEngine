// charts.js — Chart rendering module for FlowBoard
// Handles: speed chart, latency chart, topic frequency bars, status indicators.
// Data is read from window.topoData (global, set by SSE/file updates).

import { safeCall, reportDiag } from './utils.js';

// ════════════════════════════════════════════════════════════════
// State
// ════════════════════════════════════════════════════════════════

var chartHistory = { rate: [], latency: [], frames: [], maxLen: 90 };
var chartPrevPub = -1, chartPrevDel = -1, chartTopic = '';
var chartLastTs = -1, chartLastSampleMs = 0;

// ════════════════════════════════════════════════════════════════
// Initialization
// ════════════════════════════════════════════════════════════════

export function initCharts() {
  // Load saved topic from localStorage
  try {
    var saved = JSON.parse(localStorage.getItem('flowboard') || '{}');
    if (saved.topic) chartTopic = saved.topic;
  } catch (e) {}

  // Set up chart range selector
  var sel = document.getElementById('chart-range');
  if (sel) sel.value = String(chartHistory.maxLen);

  // Set up chart topic selector
  var topicSel = document.getElementById('chart-topic');
  if (topicSel && chartTopic) topicSel.value = chartTopic;
}

// ════════════════════════════════════════════════════════════════
// Update all charts from topology data
// ════════════════════════════════════════════════════════════════

export function updateCharts(data) {
  // data param lets the caller pass the topology payload;
  // fallback to window.topoData for backward compatibility.
  var topoData = data || window.topoData;
  if (!topoData) return;

  populateTopicSelector();

  var m = topoData.metrics || {}, b = m.bus || {}, l = m.latency || {};
  var nowPub, nowDel, nowLat, p99Lat;

  if (chartTopic) {
    var ts = (m.topics || []).filter(function(t) {
      return (t.topic || t.name || '') === chartTopic;
    });
    if (ts.length) {
      var t = ts[0];
      nowPub = t.pub || 0;
      nowDel = t.del || 0;
      nowLat = t.lat_us || 0;
      p99Lat = t.p99_us || 0;
    } else {
      nowPub = 0; nowDel = 0; nowLat = 0; p99Lat = 0;
    }
  } else {
    nowPub = b.published || 0;
    nowDel = b.delivered || 0;
    nowLat = l.avg_us || 0;
    p99Lat = l.p99_us || 0;
  }

  var suf = chartTopic ? ' (' + chartTopic.split('/').pop() + ')' : '';

  // SSE pushes at 10Hz, but the state file refreshes slower — adjacent frames
  // often carry duplicate data. If every frame is recorded, the delta is zero
  // and the chart appears flat. Dedup by server timestamp.
  var dataTs = topoData.timestamp || 0;
  if (chartLastTs >= 0 && dataTs === chartLastTs) {
    drawChart('ch-rate', 'tt-rate', chartHistory.rate, '#58a6ff', 'Pub Rate' + suf, 'msg/s');
    drawChart('ch-lat', 'tt-lat', chartHistory.latency, '#d29922', 'Latency' + suf, 'µs');
    drawChart('ch-frames', 'tt-frames', chartHistory.frames, '#3fb950', 'Deliveries' + suf, 'msg/s');
    return;
  }

  var nowMs = Date.now();
  var elapsed = (chartLastSampleMs > 0) ? Math.max(0.05, (nowMs - chartLastSampleMs) / 1000) : 1;
  chartLastTs = dataTs;
  chartLastSampleMs = nowMs;

  var dp = chartPrevPub >= 0 ? Math.max(0, (nowPub - chartPrevPub) / elapsed) : 0;
  var dd = chartPrevDel >= 0 ? Math.max(0, (nowDel - chartPrevDel) / elapsed) : 0;

  chartHistory.rate.push({ v: dp, raw: nowPub });
  chartHistory.latency.push({ v: nowLat, p99: p99Lat });
  chartHistory.frames.push({ v: dd, raw: nowDel });

  chartPrevPub = nowPub;
  chartPrevDel = nowDel;

  ['rate', 'latency', 'frames'].forEach(function(a) {
    while (chartHistory[a].length > chartHistory.maxLen) chartHistory[a].shift();
  });

  drawChart('ch-rate', 'tt-rate', chartHistory.rate, '#58a6ff', 'Pub Rate' + suf, 'msg/s');
  drawChart('ch-lat', 'tt-lat', chartHistory.latency, '#d29922', 'Latency' + suf, 'µs');
  drawChart('ch-frames', 'tt-frames', chartHistory.frames, '#3fb950', 'Deliveries' + suf, 'msg/s');
}

// ════════════════════════════════════════════════════════════════
// Canvas chart renderer (shared by all three chart panes)
// ════════════════════════════════════════════════════════════════

function drawChart(canvasId, tooltipId, data, color, title, unit) {
  var c = document.getElementById(canvasId);
  if (!c) return;
  var box = c.parentElement, W = box.clientWidth, H = box.clientHeight;
  // Fallback to CSS dimensions if client rect is zero (e.g. element hidden)
  if (W < 10) W = parseInt(box.style.width) || 280;
  if (H < 10) H = parseInt(box.style.height) || 240;
  if (W < 10 || H < 10) return;
  c.width = W * 2; c.height = H * 2; c.style.width = W + 'px'; c.style.height = H + 'px';
  var ctx = c.getContext('2d'); ctx.scale(2, 2);
  var pad = { t: 36, r: 66, b: 34, l: 48 }, w = W - pad.l - pad.r, h = H - pad.t - pad.b;
  ctx.clearRect(0, 0, W, H);
  var bg = ctx.createLinearGradient(0, 0, 0, H);
  bg.addColorStop(0, '#0d1117'); bg.addColorStop(1, '#090c10');
  ctx.fillStyle = bg; ctx.fillRect(0, 0, W, H);
  if (!data || data.length < 2) {
    ctx.fillStyle = '#30363d'; ctx.font = '11px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('Waiting for data...', W / 2, H / 2);
    ctx.fillStyle = '#8b949e'; ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'left';
    ctx.fillText(title, pad.l, pad.t - 10);
    return;
  }
  var vals = data.map(function(d) { return d.v != null ? d.v : d; });
  var maxV = Math.max.apply(null, vals.concat([1])), minV = Math.min.apply(null, vals.concat([0])), range = (maxV - minV) || 1;
  var latest = vals[vals.length - 1], avg = vals.reduce(function(a, b) { return a + b; }, 0) / vals.length;
  var xs = w / Math.max(data.length - 1, 1), ys = h / range;
  // Grid
  var yTicks = 4;
  ctx.strokeStyle = '#161b22'; ctx.lineWidth = 0.5;
  for (var i = 0; i <= yTicks; i++) {
    var y = pad.t + (h * i) / yTicks;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
    ctx.fillStyle = '#484f58'; ctx.font = '9px system-ui'; ctx.textAlign = 'right';
    ctx.fillText(Math.round(maxV - (range * i) / yTicks).toLocaleString(), pad.l - 8, y + 3);
  }
  // Avg line
  var avgY = pad.t + (maxV - avg) * ys;
  ctx.strokeStyle = color + '44'; ctx.lineWidth = 1; ctx.setLineDash([4, 6]);
  ctx.beginPath(); ctx.moveTo(pad.l, avgY); ctx.lineTo(W - pad.r, avgY); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = color + '88'; ctx.font = '9px system-ui'; ctx.textAlign = 'right';
  ctx.fillText('avg ' + Math.round(avg).toLocaleString(), W - pad.r, avgY - 3);
  // Area
  ctx.fillStyle = color + '15'; ctx.beginPath();
  data.forEach(function(d, i) {
    var x = pad.l + i * xs, y = pad.t + (maxV - (d.v != null ? d.v : d)) * ys;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.lineTo(pad.l + (data.length - 1) * xs, pad.t + h);
  ctx.lineTo(pad.l, pad.t + h); ctx.closePath(); ctx.fill();
  // Line
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.beginPath();
  data.forEach(function(d, i) {
    var x = pad.l + i * xs, y = pad.t + (maxV - (d.v != null ? d.v : d)) * ys;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
  // P99
  if (data[0] && data[0].p99 != null) {
    ctx.strokeStyle = '#f85149'; ctx.lineWidth = 1.5; ctx.setLineDash([3, 5]);
    ctx.beginPath();
    data.forEach(function(d, i) {
      var x = pad.l + i * xs, y = pad.t + (maxV - d.p99) * ys;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke(); ctx.setLineDash([]);
    var lastP99 = data[data.length - 1].p99, p99Y = pad.t + (maxV - lastP99) * ys;
    ctx.fillStyle = '#f85149'; ctx.font = 'bold 9px system-ui'; ctx.textAlign = 'left';
    ctx.fillText('p99 ' + Math.round(lastP99), W - pad.r + 4, p99Y + 3);
    ctx.fillStyle = color; ctx.font = '9px system-ui'; ctx.textAlign = 'left';
    ctx.fillText('─ avg', pad.l, pad.t + h + 12);
    ctx.fillStyle = '#f85149';
    ctx.fillText('── p99', pad.l + 50, pad.t + h + 12);
  }
  // Time hints
  ctx.fillStyle = '#30363d'; ctx.font = '8px system-ui'; ctx.textAlign = 'right';
  ctx.fillText('now', W - pad.r, pad.t + h + 12);
  ctx.textAlign = 'left';
  ctx.fillText(data.length + 's ago', pad.l, pad.t + h + 12);
  // Big value
  ctx.fillStyle = color; ctx.font = 'bold 22px system-ui'; ctx.textAlign = 'right';
  ctx.fillText(Math.round(latest).toLocaleString(), W - pad.r, pad.t + 18);
  // Title
  ctx.fillStyle = '#c9d1d9'; ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'left';
  ctx.fillText(title, pad.l, pad.t - 10);
  ctx.fillStyle = '#484f58'; ctx.font = '9px system-ui'; ctx.textAlign = 'right';
  ctx.fillText(unit, W - pad.r - 52, pad.t - 10);
  // Hover
  c.onmousemove = function(e) {
    var rect = c.getBoundingClientRect(), mx = e.clientX - rect.left, my = e.clientY - rect.top;
    if (mx < pad.l || mx > W - pad.r) { document.getElementById(tooltipId).style.display = 'none'; return; }
    var idx = Math.round((mx - pad.l) / xs);
    if (idx < 0 || idx >= data.length) { document.getElementById(tooltipId).style.display = 'none'; return; }
    var d = data[idx], v = d.v != null ? d.v : d, tt = document.getElementById(tooltipId);
    var html = '<b>' + title + '</b> ' + Math.round(v).toLocaleString() + ' ' + unit;
    if (d.p99 != null) html += '<br><span style="color:#f85149">p99 ' + Math.round(d.p99) + ' µs</span>';
    html += '<br><span style="color:#484f58">' + (data.length - idx) + 's ago</span>';
    tt.innerHTML = html; tt.style.display = 'block';
    tt.style.left = Math.min(pad.l + idx * xs + 10, W - 120) + 'px';
    tt.style.top = Math.max(pad.t + (maxV - v) * ys - 30, 4) + 'px';
  };
  c.onmouseleave = function() { document.getElementById(tooltipId).style.display = 'none'; };
}

// ════════════════════════════════════════════════════════════════
// Topic selector population
// ════════════════════════════════════════════════════════════════

function populateTopicSelector() {
  var sel = document.getElementById('chart-topic');
  if (!sel) return;
  var cur = sel.value;
  var topics = ((window.topoData && window.topoData.metrics) || {}).topics || [];
  topics = topics.map(function(t) { return t.topic || t.name || ''; }).filter(Boolean).sort();
  var exist = [];
  for (var i = 1; i < sel.options.length; i++) exist.push(sel.options[i].value);
  if (exist.join(',') === topics.join(',')) return;
  sel.innerHTML = '<option value="">All Topics</option>';
  topics.forEach(function(n) {
    var o = document.createElement('option');
    o.value = n;
    o.textContent = n.split('/').pop();
    if (n === cur) o.selected = true;
    sel.appendChild(o);
  });
}

// ════════════════════════════════════════════════════════════════
// Event handlers (exposed on window for inline onclick)
// ════════════════════════════════════════════════════════════════

window.onChartTopicChange = function onChartTopicChange() {
  chartTopic = document.getElementById('chart-topic').value;
  chartPrevPub = -1;
  chartPrevDel = -1;
  chartLastTs = -1;
  chartLastSampleMs = 0;
  chartHistory = { rate: [], latency: [], frames: [], maxLen: chartHistory.maxLen };
  _saveChartState();
};

window.onChartRangeChange = function onChartRangeChange() {
  chartHistory.maxLen = parseInt(document.getElementById('chart-range').value) || 60;
  chartHistory = { rate: [], latency: [], frames: [], maxLen: chartHistory.maxLen };
};

function _saveChartState() {
  try {
    var existing = JSON.parse(localStorage.getItem('flowboard') || '{}');
    existing.topic = chartTopic;
    localStorage.setItem('flowboard', JSON.stringify(existing));
  } catch (e) {}
}

// ════════════════════════════════════════════════════════════════
// Module-global exports + window compatibility
// ════════════════════════════════════════════════════════════════

window.initCharts = initCharts;
window.updateCharts = updateCharts;
