// ═══════════════════════════════════════════════════════════════
// app.js — FlowBoard 主入口
// ═══════════════════════════════════════════════════════════════
// 架构：
//   3D 场景    → vis/main.js（模块化，替代旧 scene3d_v2.js）
//   2D Canvas  → scene2d.js
//   D3 拓扑图  → charts.js（内联 D3 拓扑，替代旧 d3_topo.js）
//   工具函数   → utils.js
// ═══════════════════════════════════════════════════════════════

import { init3DScene, resize3D, update3D, sceneReady, scene3d, setTopoData as setTopoData3D, setDebugCam, setCameraMode, resetCamera, resetMapView, closeNPCDetail, setPerfTier } from './vis/main.js';
import { init2D, init2DFallback, draw2D, switchSceneView, _2d as _2dState, setTopoData as setTopoData2D } from './scene2d.js';
import { initCharts, updateCharts, onChartTopicChange, onChartRangeChange, setTopoData as setTopoDataChart } from './charts.js';
import { safeCall, reportDiag, clearDiag, _auditSceneMaterials } from './utils.js';
import { updateDeadReckon, _dr, initDeadReckon, tickDeadReckon } from './deadreckon.js';

// ═══════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════
let topoData = null;
let eventSource = null;
let paused = false;
let autoReconnect = true;
let reconnectAttempts = 0;
const MAX_RECONNECT = 10;
const RECONNECT_BASE_DELAY = 1000;

// ═══════════════════════════════════════════════════════════════
// 初始化
// ═══════════════════════════════════════════════════════════════

function initAll() {
  initTopo();           // D3 topology graph
  init3DScene();        // 3D scene
  init2D();             // 2D canvas
  switchSceneView('3d'); // default to 3D
  initCharts();         // charts
  initDeadReckon();     // dead reckoning

  // connect to server
  connectToServer();

  // 后台定时刷新（demo 模式 / 离线）
  startBackgroundUpdate();

  // 窗口大小监听
  window.addEventListener('resize', onResize);
  onResize();

  // 键盘监听
  window.addEventListener('keydown', onKeyDown);

  console.log('[flowboard] init done');
}

// ═══════════════════════════════════════════════════════════════
// 服务器连接
// ═══════════════════════════════════════════════════════════════

function connectToServer() {
  if (eventSource) {
    eventSource.close();
    eventSource = null;
  }

  const host = window.location.hostname || 'localhost';
  const port = window.location.port || 8800;
  const url = `http://${host}:${port}/api/stream`;

  console.log('[flowboard] connecting to', url);

  try {
    eventSource = new EventSource(url);
  } catch (e) {
    console.warn('[flowboard] EventSource not supported, fallback to polling');
    startPolling(url);
    return;
  }

  eventSource.onopen = function() {
    console.log('[flowboard] SSE connected');
    reconnectAttempts = 0;
    setConnStatus('live', '● connected');
    document.getElementById('demo-watermark').style.display = 'none';
  };

  eventSource.onmessage = function(e) {
    try {
      const data = JSON.parse(e.data);
      if (data && data.nodes) {
        topoData = data;
        updateAll();
      }
    } catch (err) {
      console.warn('[flowboard] SSE parse error:', err);
    }
  };

  eventSource.onerror = function() {
    console.warn('[flowboard] SSE error, will retry');
    setConnStatus('dead', '● disconnected');
    eventSource.close();
    eventSource = null;

    if (autoReconnect && reconnectAttempts < MAX_RECONNECT) {
      const delay = RECONNECT_BASE_DELAY * Math.pow(2, reconnectAttempts);
      reconnectAttempts++;
      console.log(`[flowboard] reconnecting in ${delay}ms (attempt ${reconnectAttempts})`);
      setTimeout(connectToServer, delay);
    } else if (reconnectAttempts >= MAX_RECONNECT) {
      console.warn('[flowboard] max reconnect attempts reached, showing demo');
      doSimulate();
    }
  };
}

function startPolling(url) {
  const pollUrl = url.replace('/api/stream', '/api/topology');
  function poll() {
    fetch(pollUrl)
      .then(r => r.json())
      .then(data => {
        if (data && data.nodes) {
          topoData = data;
          updateAll();
        }
      })
      .catch(err => console.warn('[flowboard] poll error:', err));
  }
  poll();
  setInterval(poll, 500);
}

// ═══════════════════════════════════════════════════════════════
// 数据更新
// ═══════════════════════════════════════════════════════════════

function updateAll() {
  if (!topoData) return;

  // 死推算
  updateDeadReckon(topoData);

  // 3D 场景
  safeCall('scene3d', () => update3D(topoData));

  // 2D 画布
  safeCall('scene2d', () => draw2D(topoData));

  // D3 拓扑图
  safeCall('charts', () => updateCharts(topoData));

  // 诊断面板
  reportDiag(topoData);

  // 车辆信息卡
  updateVehicleCard(topoData);

  // 仪表盘
  updateDashboard(topoData);

  // 话题表
  updateTopicTable(topoData);

  // 节点表
  updateNodeTable(topoData);

  // 性能面板
  updatePerfPanel();
}

function setTopoData(d) {
  setTopoData3D(d);
  setTopoData2D(d);
  setTopoDataChart(d);
}

// ═══════════════════════════════════════════════════════════════
// 车辆信息卡
// ═══════════════════════════════════════════════════════════════

function updateVehicleCard(data) {
  const card = document.getElementById('vehicle-card');
  if (!card) return;

  const v = data.metrics && data.metrics.vehicle;
  const mode = data.metrics && data.metrics.driver_mode;

  if (v) {
    document.getElementById('v-speed').textContent = v.speed ? v.speed.toFixed(1) : '--';
    document.getElementById('v-target').textContent = v.target_speed ? v.target_speed.toFixed(1) : '--';
    document.getElementById('v-throttle').textContent = v.throttle ? (v.throttle * 100).toFixed(0) + '%' : '--';
    document.getElementById('v-brake').textContent = v.brake ? (v.brake * 100).toFixed(0) + '%' : '--';
    document.getElementById('v-error').textContent = v.error ? v.error.toFixed(2) : '--';
    document.getElementById('v-mode').textContent = mode || '--';
  }

  card.style.display = '';
}

// ═══════════════════════════════════════════════════════════════
// 仪表盘
// ═══════════════════════════════════════════════════════════════

function updateDashboard(data) {
  const m = data.metrics || {};
  const b = m.bus || {};
  const l = m.latency || {};

  const pubEl = document.getElementById('dash-pub');
  const delEl = document.getElementById('dash-del');
  const dropEl = document.getElementById('dash-drop');
  const latEl = document.getElementById('dash-lat');
  const p99El = document.getElementById('dash-p99');

  if (pubEl) pubEl.textContent = b.published || 0;
  if (delEl) delEl.textContent = b.delivered || 0;
  if (dropEl) dropEl.textContent = b.dropped || 0;
  if (latEl) latEl.textContent = l.avg_us ? l.avg_us.toFixed(0) + ' μs' : '--';
  if (p99El) p99El.textContent = l.p99_us ? l.p99_us.toFixed(0) + ' μs' : '--';
}

// ═══════════════════════════════════════════════════════════════
// 话题表
// ═══════════════════════════════════════════════════════════════

function updateTopicTable(data) {
  const tbody = document.getElementById('topic-tbody');
  if (!tbody) return;

  const topics = (data.metrics && data.metrics.topics) || [];
  if (topics.length === 0) return;

  let html = '';
  topics.forEach(t => {
    const freq = t.freq ? t.freq.toFixed(1) : '--';
    const lat = t.lat_us ? t.lat_us.toFixed(0) + ' μs' : '--';
    const dropRate = t.pub > 0 ? ((t.drop / t.pub) * 100).toFixed(1) + '%' : '0%';
    const rel = t.reliability || '--';
    const transport = t.transport || '--';
    const subs = t.subs || 0;

    html += `<tr>
      <td title="${t.topic}">${t.topic}</td>
      <td>${freq}</td>
      <td>${t.pub}</td>
      <td>${t.del}</td>
      <td class="${t.drop > 0 ? 'warn' : ''}">${t.drop}</td>
      <td>${dropRate}</td>
      <td>${lat}</td>
      <td>${subs}</td>
      <td>${rel}</td>
      <td>${transport}</td>
    </tr>`;
  });

  tbody.innerHTML = html;
}

// ═══════════════════════════════════════════════════════════════
// 节点表
// ═══════════════════════════════════════════════════════════════

function updateNodeTable(data) {
  const tbody = document.getElementById('node-tbody');
  if (!tbody) return;

  const nodes = data.nodes || [];
  if (nodes.length === 0) return;

  let html = '';
  nodes.forEach(n => {
    const caps = n.caps != null ? n.caps : '--';
    const alive = n.alive ? '✓' : '✗';
    const topics = (n.topics || []).map(t => t.topic).join(', ') || '--';

    html += `<tr>
      <td>${n.name}</td>
      <td>${n.pid || '--'}</td>
      <td class="${n.alive ? 'ok' : 'err'}">${alive}</td>
      <td>${caps}</td>
      <td title="${topics}">${topics}</td>
    </tr>`;
  });

  tbody.innerHTML = html;
}

// ═══════════════════════════════════════════════════════════════
// 性能面板
// ═══════════════════════════════════════════════════════════════

function updatePerfPanel() {
  const el = document.getElementById('perf-panel');
  if (!el) return;

  const m = topoData && topoData.metrics;
  if (!m) return;

  const s = m.scheduler || {};
  const t = m.transport || {};

  el.innerHTML = `
    <div class="perf-row"><span>Scheduler</span><span>${s.mode || '--'} | ${s.tasks || 0} tasks</span></div>
    <div class="perf-row"><span>Transport</span><span>local: ${t.local_pub || 0} | remote: ${t.remote_pub || 0}</span></div>
    <div class="perf-row"><span>Driver</span><span>${m.driver_mode || '--'}</span></div>
  `;
}

// ═══════════════════════════════════════════════════════════════
// D3 拓扑图 (内联替代 d3_topo.js)
// ═══════════════════════════════════════════════════════════════

let topoSvg = null, topoSim = null, topoNodes = [], topoLinks = [];

function initTopo() {
  const container = document.getElementById('topo-svg');
  if (!container) return;

  const width = container.clientWidth || 600;
  const height = container.clientHeight || 400;

  topoSvg = d3.select('#topo-svg')
    .append('svg')
    .attr('width', width)
    .attr('height', height);

  topoSim = d3.forceSimulation()
    .force('link', d3.forceLink().id(d => d.id).distance(80))
    .force('charge', d3.forceManyBody().strength(-200))
    .force('center', d3.forceCenter(width / 2, height / 2))
    .force('collision', d3.forceCollide(30));

  topoSim.on('tick', () => {
    topoSvg.selectAll('line')
      .attr('x1', d => d.source.x)
      .attr('y1', d => d.source.y)
      .attr('x2', d => d.target.x)
      .attr('y2', d => d.target.y);

    topoSvg.selectAll('circle')
      .attr('cx', d => d.x)
      .attr('cy', d => d.y);

    topoSvg.selectAll('text')
      .attr('x', d => d.x)
      .attr('y', d => d.y + 3);
  });
}

function updateTopo(data) {
  if (!topoSvg) return;

  const nodes = (data.nodes || []).map(n => ({
    id: n.name,
    alive: n.alive
  }));

  const links = [];
  const seen = new Set();
  (data.endpoints || []).forEach(ep => {
    if (ep.role === 'pub') {
      (data.endpoints || []).forEach(sub => {
        if (sub.role === 'sub' && sub.topic === ep.topic && sub.node !== ep.node) {
          const key = ep.node + '→' + sub.node + '#' + ep.topic;
          if (!seen.has(key)) {
            seen.add(key);
            links.push({ source: ep.node, target: sub.node, topic: ep.topic });
          }
        }
      });
    }
  });

  topoNodes = nodes;
  topoLinks = links;

  // Update
  const link = topoSvg.selectAll('line').data(links, d => d.source + '→' + d.target);
  link.exit().remove();
  link.enter().append('line')
    .attr('stroke', '#555')
    .attr('stroke-width', 1.5)
    .attr('stroke-opacity', 0.6);

  const node = topoSvg.selectAll('circle').data(nodes, d => d.id);
  node.exit().remove();
  node.enter().append('circle')
    .attr('r', 12)
    .attr('fill', d => d.alive ? '#4caf50' : '#f44336')
    .attr('stroke', '#fff')
    .attr('stroke-width', 2)
    .call(d3.drag()
      .on('start', (event, d) => { if (!event.active) topoSim.alphaTarget(0.3).restart(); d.fx = d.x; d.fy = d.y; })
      .on('drag', (event, d) => { d.fx = event.x; d.fy = event.y; })
      .on('end', (event, d) => { if (!event.active) topoSim.alphaTarget(0); d.fx = null; d.fy = null; })
    );

  const label = topoSvg.selectAll('text').data(nodes, d => d.id);
  label.exit().remove();
  label.enter().append('text')
    .attr('text-anchor', 'middle')
    .attr('dy', 3)
    .attr('fill', '#ccc')
    .attr('font-size', 10)
    .text(d => d.id);

  topoSim.nodes(nodes);
  topoSim.force('link').links(links);
  topoSim.alpha(1).restart();
}

// ═══════════════════════════════════════════════════════════════
// 连接状态
// ═══════════════════════════════════════════════════════════════

function setConnStatus(status, text) {
  const el = document.getElementById('conn-status');
  if (!el) return;
  el.textContent = text;
  el.className = 'conn-status ' + status;
}

// ═══════════════════════════════════════════════════════════════
// Demo 模式
// ═══════════════════════════════════════════════════════════════

function doSimulate() {
  topoData = {
    nodes: [
      {name:"perception",pid:1001,alive:true,caps:1,topics:[{topic:"sensor/lidar",freq:20},{topic:"sensor/camera",freq:20},{topic:"sensor/gps",freq:10}]},
      {name:"fusion",pid:1002,alive:true,caps:9,topics:[{topic:"sensor/lidar",freq:0},{topic:"sensor/gps",freq:0},{topic:"fusion/localization",freq:20}]},
      {name:"planning",pid:1004,alive:true,caps:2,topics:[{topic:"fusion/localization",freq:0},{topic:"planning/trajectory",freq:10}]},
      {name:"control",pid:1003,alive:true,caps:2,topics:[{topic:"fusion/localization",freq:0},{topic:"planning/trajectory",freq:0},{topic:"control/cmd",freq:100}]},
      {name:"monitor",pid:1005,alive:true,caps:0,topics:[]}
    ],
    metrics: {
      bus:{published:1250,delivered:2480,dropped:0},
      transport:{local_pub:1250,remote_pub:0},
      scheduler:{tasks:5,mode:"CHOREO"},
      latency:{avg_us:145,p50_us:120,p99_us:450},
      driver_mode:"ACC",
      vehicle:{speed:28.5,target_speed:33.0,throttle:0.92,brake:0,x:65.0,error:4.5},
      scene: {
        road_network: {
          edges: [
            { id: 0, type: "highway", name: "highway", length_m: 2000, lanes: 4, lane_width: 3.5,
              speed_limit: 33.0, nodes: [[0, 0, 0], [2000, 0, 0]] }
          ]
        },
        ego: { x: 65.0, y: 0, heading: 0.0, speed: 28.5, steer: 0.02, brake: 0, throttle: 0.92 },
        entities: [
          { id: 1, type: 'car',   x: 45.0, y:  3.5, heading: 0.0, speed: 25.0, length: 4.6, width: 2.0 },
          { id: 2, type: 'suv',   x: 85.0, y: -3.5, heading: 0.0, speed: 22.0, length: 4.8, width: 2.1 },
          { id: 3, type: 'truck', x: 25.0, y: -3.5, heading: 0.0, speed: 18.0, length: 8.0, width: 2.5 },
          { id: 4, type: 'car',   x: 105.0, y: 3.5, heading: 0.0, speed: 30.0, length: 4.6, width: 2.0 }
        ]
      },
      topics: [
        {topic:"sensor/lidar",pub:500,del:1000,drop:0,lat_us:145,freq:20.0,subs:2,reliability:"best_effort",deadline_ms:0,transport:"shm"},
        {topic:"sensor/camera",pub:500,del:500,drop:0,lat_us:95,freq:20.0,subs:1,reliability:"best_effort",deadline_ms:0,transport:"shm"},
        {topic:"fusion/localization",pub:500,del:1000,drop:0,lat_us:120,freq:20.0,subs:2,reliability:"reliable",deadline_ms:100,transport:"shm"},
        {topic:"planning/trajectory",pub:200,del:200,drop:0,lat_us:85,freq:10.0,subs:1,reliability:"reliable",deadline_ms:20,transport:"dds"}
      ]
    },
    endpoints: [
      {node:"perception",topic:"sensor/lidar",role:"pub",type_id:"0xd712aa51",freq:20.0},
      {node:"perception",topic:"sensor/camera",role:"pub",type_id:"0x4A1B0C2D",freq:20.0},
      {node:"perception",topic:"sensor/gps",role:"pub",type_id:"0x0596b0b7",freq:10.0},
      {node:"fusion",topic:"sensor/lidar",role:"sub",type_id:"0xd712aa51",freq:0},
      {node:"fusion",topic:"sensor/gps",role:"sub",type_id:"0x0596b0b7",freq:0},
      {node:"fusion",topic:"fusion/localization",role:"pub",type_id:"0xf0ed10c0",freq:20.0},
      {node:"planning",topic:"fusion/localization",role:"sub",type_id:"0xf0ed10c0",freq:0},
      {node:"planning",topic:"planning/trajectory",role:"pub",type_id:"0x3A7B1C2D",freq:10.0},
      {node:"control",topic:"fusion/localization",role:"sub",type_id:"0xf0ed10c0",freq:0},
      {node:"control",topic:"planning/trajectory",role:"sub",type_id:"0x3A7B1C2D",freq:0},
      {node:"control",topic:"control/cmd",role:"pub",type_id:"0x2d95c6d2",freq:100.0}
    ]
  };
  updateAll();
  setConnStatus('live','● demo');
  var wm = document.getElementById('demo-watermark');
  if (wm) wm.style.display = '';
  document.getElementById('vehicle-card').style.display = '';
}

// ═══════════════════════════════════════════════════════════════
// 窗口 & 键盘
// ═══════════════════════════════════════════════════════════════

function onResize() {
  resize3D();
  if (_2dState && _2dState.canvas) {
    const container = document.getElementById('scene2d');
    if (container) {
      _2dState.canvas.width = container.clientWidth;
      _2dState.canvas.height = container.clientHeight;
    }
  }
}

function onKeyDown(e) {
  switch (e.key) {
    case '1': switchSceneView('3d'); setCameraMode('chase'); break;
    case '2': switchSceneView('3d'); setCameraMode('top'); break;
    case '3': switchSceneView('3d'); setCameraMode('driver'); break;
    case '4': switchSceneView('3d'); setCameraMode('front'); break;
    case '5': switchSceneView('3d'); setCameraMode('map'); break;
    case '6': switchSceneView('3d'); setCameraMode('orbit'); break;
    case '0': switchSceneView('2d'); break;
    case 'p': paused = !paused; break;
    case 'Escape': closeNPCDetail(); break;
    case 'r': resetCamera(); break;
    case 'd': setDebugCam('debug'); break;
    case 'ArrowUp': setPerfTier('high'); break;
    case 'ArrowDown': setPerfTier('medium'); break;
    case 'ArrowLeft': setPerfTier('low'); break;
    case 'ArrowRight': setPerfTier('high'); break;
  }
}

// ═══════════════════════════════════════════════════════════════
// 后台更新
// ═══════════════════════════════════════════════════════════════

function startBackgroundUpdate() {
  setInterval(function() {
    if (!eventSource && !paused) {
      var m = topoData.metrics||{}, b = m.bus||{}, l = m.latency||{};
      b.published = (b.published||1000) + Math.floor(Math.random()*50 - 25);
      b.delivered = (b.delivered||2000) + Math.floor(Math.random()*100 - 50);
      b.dropped = Math.random() < 0.05 ? (b.dropped||0) + 1 : (b.dropped||0);
      l.avg_us = Math.max(80, (l.avg_us||150) + Math.floor(Math.random()*20 - 10));
      l.p99_us = Math.max(200, (l.p99_us||400) + Math.floor(Math.random()*60 - 30));
      var scn = m.scene; if (scn && scn.ego) {
        scn.ego.x += scn.ego.speed / 3.6;  // km/h → m/s × 1s
        scn.ego.steer = Math.sin(Date.now() / 2000) * 0.03;
        scn.ego.heading = Math.sin(Date.now() / 3000) * 0.05;
      }
      updateAll();
    }
  }, 1000);
}

// ═══════════════════════════════════════════════════════════════
// Single namespace export — only `window.flowboard` is added to the
// global scope; everything else stays module-private.
// ═══════════════════════════════════════════════════════════════

window.flowboard = {
  // Lifecycle
  init: initAll,

  // Data
  setTopoData,
  updateAll,

  // Scene
  switchSceneView,
  setCameraMode,
  resetCamera,
  resetMapView,

  // Debug
  setDebugCam,
  closeNPCDetail,
  setPerfTier,

  // Connection
  connectToServer,
  doSimulate,

  // State
  get sceneReady() { return sceneReady(); },
  get scene3d() { return scene3d; },
  get topoData() { return topoData; },
  get paused() { return paused; },
  set paused(v) { paused = v; },

  // Diagnostics
  reportDiag,
  clearDiag,
  _auditSceneMaterials,

  // Dead Reckoning
  get _dr() { return _dr; },

  // 2D state
  get _2d() { return _2dState; }
};

// ═══════════════════════════════════════════════════════════════
// Auto-init
// ═══════════════════════════════════════════════════════════════

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initAll);
} else {
  initAll();
}