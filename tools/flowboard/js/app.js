// ═══════════════════════════════════════════════════════════════
// FlowBoard — Entry Point ES Module
// ═══════════════════════════════════════════════════════════════
// Imports from sub-modules
import { init3DScene, resize3D, update3D, sceneReady, _renderFrame } from './scene3d.js';
import { init2D, draw2D } from './scene2d.js';
import { initCharts, updateCharts } from './charts.js';
import { safeCall, reportDiag, clearDiag } from './utils.js';

// ═══════════════════════════════════════════════════════════════
// Constants
// ═══════════════════════════════════════════════════════════════
var COLORS = {1:'#3fb950',2:'#58a6ff',4:'#d29922',8:'#bc8cff',0:'#f85149'};

// ═══════════════════════════════════════════════════════════════
// Global State
// ═══════════════════════════════════════════════════════════════
var topoData = {nodes:[], metrics:{}};
window.topoData = topoData;        // global access for other modules

var frames = [];
var paused = false;
var frameCount = 0;
var chartHistory = {rate:[], latency:[], frames:[], maxLen:90};
var serverUrl = 'http://localhost:8800';
var eventSource = null;
var selectedNode = null;
var reconnectTimer = null;
var sseRenewTimer = null;
var trainingPollTimer = null;
var chartTopic = '';
var chartPrevPub = -1, chartPrevDel = -1;
var chartLastTs = -1, chartLastSampleMs = 0;
var connectRetries = 0;
var lastNodeNames = '';

// Load saved state
try {
  var saved = JSON.parse(localStorage.getItem('flowboard')||'{}');
  if (saved.url) serverUrl = saved.url;
  if (saved.topic) chartTopic = saved.topic;
} catch(e) {}

// ═══════════════════════════════════════════════════════════════
// Utility helpers (not in utils.js because they depend on app state)
// ═══════════════════════════════════════════════════════════════

function setConnStatus(cls, text) {
  var el = document.getElementById('conn-dot');
  if (!el) return;
  el.className = 'pill pill-'+cls; el.textContent = text;
}

function applyLiveStatus(d) {
  var wm = document.getElementById('demo-watermark');
  if (!d || typeof d !== 'object') { setConnStatus('live','● live'); if (wm) wm.style.display='none'; return; }
  if (d.source === 'demo') { setConnStatus('live','● demo'); if (wm) wm.style.display=''; return; }
  if (wm) wm.style.display='none';
  if (d.stale === true) {
    var age = (typeof d.age_sec === 'number') ? (' '+Math.round(d.age_sec)+'s') : '';
    setConnStatus('warn','● stale'+age);
  } else {
    setConnStatus('live','● live');
  }
}

function topicFilter() {
  var raw = document.getElementById('topic-filter') ? document.getElementById('topic-filter').value.trim() : '';
  return raw ? raw.split(',').map(function(s){ return s.trim(); }).filter(Boolean) : [];
}

function topicMatches(tn) {
  var f = topicFilter();
  return f.length === 0 || f.some(function(p){ return tn.indexOf(p) >= 0 || p.indexOf(tn) >= 0; });
}

function roleClass(role) { return 'role role-'+(role||'unknown'); }

function endpointRoleFromCaps(t) {
  if (t.role) return t.role;
  var c = Number(t.caps||t.capabilities||0);
  if ((c&1)&&(c&2)) return 'pubsub';
  if (c&1) return 'pub';
  if (c&2) return 'sub';
  if (Number(t.freq||0) > 0) return 'pub';
  return 'unknown';
}

function saveState() {
  try {
    localStorage.setItem('flowboard', JSON.stringify({
      url: serverUrl,
      topic: chartTopic,
      collapsed: Array.from(document.querySelectorAll('.card.collapsed')).map(function(c){
        return c.querySelector('h2').textContent.trim();
      })
    }));
  } catch(e) {}
}

// ═══════════════════════════════════════════════════════════════
// D3 Topology Graph
// ═══════════════════════════════════════════════════════════════
var topoLinks = [], nodePositions = {};
var _topoSim = null;

function initTopo() {
  // D3 topology is fully rebuilt on each data change by updateTopo().
  // initTopo just ensures the container is ready.
  var el = document.getElementById('topo');
  if (!el) return;
  // The SVG is created inside updateTopo() when data arrives.
  // Show the placeholder message until then.
  var msg = document.getElementById('msg');
  if (msg) msg.style.display = 'block';
}

function updateTopo(data) {
  if (_topoSim) { _topoSim.stop(); _topoSim = null; }

  var prevPos = nodePositions;
  nodePositions = {};

  var ns = (data.nodes||[]).map(function(n, i) {
    var nm = n.name || ('_'+i);
    var prev = prevPos[nm];
    var x = prev ? prev.x : 100 + Math.random()*600;
    var y = prev ? prev.y : 80 + Math.random()*220;
    nodePositions[nm] = {x:x, y:y};
    return {
      name: n.name, id: i, x: x, y: y, alive: n.alive, caps: n.caps,
      pid: n.pid, description: n.description, plugin: n.plugin,
      topics: n.topics || []
    };
  });

  var el = document.getElementById('topo');
  var msg = document.getElementById('msg');
  if (!ns.length) { if (msg) msg.style.display = 'block'; return; }
  if (msg) msg.style.display = 'none';

  if (typeof d3 === 'undefined') {
    el.innerHTML = '<div style="padding:20px;color:#f85149">D3.js not loaded</div>';
    return;
  }

  selectedNode = null;

  // Build links from shared topics
  var links = [], topicToPub = {}, topicToSub = {};
  ns.forEach(function(n, i) {
    (n.topics||[]).forEach(function(t) {
      var topic = t.topic || t.name;
      if (!topic) return;
      var role = endpointRoleFromCaps(t);
      if (role === 'pub' || role === 'pubsub') (topicToPub[topic]||(topicToPub[topic]=[])).push(i);
      if (role === 'sub' || role === 'pubsub') (topicToSub[topic]||(topicToSub[topic]=[])).push(i);
    });
  });
  Object.keys(topicToPub).forEach(function(topic) {
    (topicToPub[topic]||[]).forEach(function(p) {
      (topicToSub[topic]||[]).forEach(function(s) {
        if (p !== s) links.push({source:p, target:s, topic:topic});
      });
    });
  });
  // Fallback if no links: infer from topic sharing
  if (!links.length) {
    for (var i=0; i<ns.length; i++) {
      for (var j=i+1; j<ns.length; j++) {
        var shared = (ns[i].topics||[]).filter(function(ti) {
          return (ns[j].topics||[]).some(function(tj) {
            return (tj.topic||tj.name) === (ti.topic||ti.name);
          });
        });
        shared.forEach(function(t) {
          links.push({source:i, target:j, topic:t.topic||t.name});
        });
      }
    }
  }

  // Anchor orphan nodes
  var nodeDeg = new Array(ns.length).fill(0);
  links.forEach(function(l) {
    var s = l.source.id != null ? l.source.id : l.source;
    var t = l.target.id != null ? l.target.id : l.target;
    nodeDeg[s]++; nodeDeg[t]++;
  });
  var hub = 0, hd = 0;
  nodeDeg.forEach(function(d, i) { if (d > hd) { hd = d; hub = i; } });
  ns.forEach(function(n, i) {
    if (nodeDeg[i] === 0 && ns.length > 1) links.push({source:i, target:hub, topic:'__orphan__', _total:1, _idx:0, _orphan:true});
  });

  topoLinks = links;

  // Rebuild SVG
  el.innerHTML = '';
  var svg = d3.select('#topo').append('svg').attr('width','100%').attr('height','100%');
  var g = svg.append('g');
  var defs = svg.append('defs');

  // Arrow markers
  var ARROW_COLORS = ['#58a6ff','#3fb950','#d29922','#bc8cff','#f85149','#8b949e'];
  ARROW_COLORS.forEach(function(color, i) {
    defs.append('marker')
      .attr('id','arrow-'+i)
      .attr('viewBox','0 0 14 14')
      .attr('refX',28).attr('refY',7)
      .attr('markerWidth',7).attr('markerHeight',7)
      .attr('orient','auto')
      .append('path').attr('d','M 0 0 L 14 7 L 0 14 L 3 7 Z')
      .attr('fill',color).attr('opacity',0.85);
  });

  // Multi-link arc parameters
  var pairKey = function(a,b) { return Math.min(a,b)+'--'+Math.max(a,b); };
  var pairCount = {}, pairIdx = {};
  links.forEach(function(l) {
    var k = pairKey(l.source.id!=null ? l.source.id : l.source, l.target.id!=null ? l.target.id : l.target);
    pairCount[k] = (pairCount[k]||0) + 1;
  });
  links.forEach(function(l) {
    var k = pairKey(l.source.id!=null ? l.source.id : l.source, l.target.id!=null ? l.target.id : l.target);
    l._total = pairCount[k];
    l._idx = pairIdx[k] || 0;
    pairIdx[k] = l._idx + 1;
  });

  svg.call(d3.zoom().scaleExtent([0.15,4]).on('zoom', function(e) {
    g.attr('transform', e.transform);
  }));

  // Links
  var linkG = g.append('g').selectAll('path').data(links).join('path')
    .attr('class','link')
    .attr('stroke', function(d,i) { return d._orphan ? 'transparent' : ARROW_COLORS[i%ARROW_COLORS.length]; })
    .attr('stroke-width', function(d) { return d._orphan ? 0 : (/(cmd|plan|traj)/.test(d.topic) ? 2.0 : 1.5); })
    .attr('fill','none')
    .attr('stroke-dasharray', function(d) { return d._orphan ? null : (/(cmd|plan|traj)/.test(d.topic) ? '6,3' : null); })
    .attr('marker-end', function(d,i) { return d._orphan ? null : 'url(#arrow-'+(i%ARROW_COLORS.length)+')'; });

  var linkL = g.append('g').selectAll('text').data(links).join('text')
    .attr('class','link-label')
    .attr('font-size','8px').attr('fill','#8b949e').attr('text-anchor','middle')
    .text(function(d) { return d._orphan ? '' : d.topic.split('/').pop(); });

  // Nodes
  var nodeG = g.append('g').selectAll('g').data(ns).join('g')
    .call(d3.drag()
      .on('start', function(e,d) {
        if (!e.sourceEvent.ctrlKey) { _topoSim.alphaTarget(0.3).restart(); d.fx = d.x; d.fy = d.y; }
      })
      .on('drag', function(e,d) { d.fx = e.x; d.fy = e.y; })
      .on('end', function(e,d) { if (!e.sourceEvent.ctrlKey) { d.fx = null; d.fy = null; } })
    );

  nodeG.append('circle')
    .attr('r', 20)
    .attr('fill', function(d) {
      return d.alive === false ? '#484f58' :
        COLORS[(d.caps||0)&8 ? 8 : (d.caps||0)&4 ? 4 : (d.caps||0)&2 ? 2 : 1];
    })
    .attr('stroke','#21262d').attr('stroke-width',2)
    .style('cursor','pointer')
    .on('click', function(e,d) {
      e.stopPropagation();
      if (selectedNode === d) {
        selectedNode = null;
        nodeG.selectAll('circle').classed('node-selected', false);
      } else {
        selectedNode = d;
        nodeG.selectAll('circle').classed('node-selected', false);
        d3.select(this).classed('node-selected', true);
        showNodeDetail(d.name);
      }
      applyHighlight(nodeG, linkG, linkL);
    })
    .append('title')
    .text(function(d) { return d.name+' (PID '+(d.pid||'—')+')\n'+(d.description||'')+'\n'+(d.plugin||''); });

  nodeG.append('text')
    .text(function(d) { return d.name; })
    .attr('text-anchor','middle').attr('dy',30)
    .attr('fill','#c9d1d9').attr('font-size','9px');

  svg.on('click', function() {
    selectedNode = null;
    nodeG.selectAll('circle').classed('node-selected', false);
    applyHighlight(nodeG, linkG, linkL);
    closeDetail();
  });

  function applyHighlight(ng, lg, ll) {
    if (!selectedNode) {
      ng.selectAll('circle').attr('opacity',1);
      lg.attr('opacity',1);
      ll.attr('opacity',1);
      return;
    }
    var selId = selectedNode.id, connIds = new Set(), connEdges = new Set();
    links.forEach(function(l,i) {
      var s = l.source.id != null ? l.source.id : l.source;
      var t = l.target.id != null ? l.target.id : l.target;
      if (s === selId) { connIds.add(t); connEdges.add(i); }
      if (t === selId) { connIds.add(s); connEdges.add(i); }
    });
    ng.selectAll('circle').attr('opacity', function(d) { return connIds.has(d.id)||d===selectedNode ? 1 : 0.25; });
    lg.attr('opacity', function(d,i) { return connEdges.has(i) ? 1 : 0.12; });
    ll.attr('opacity', function(d,i) { return connEdges.has(i) ? 1 : 0.12; });
  }

  // Force simulation
  _topoSim = d3.forceSimulation(ns)
    .force('link', d3.forceLink(links).distance(160))
    .force('charge', d3.forceManyBody().strength(-200))
    .force('center', d3.forceCenter(400,200).strength(0.15))
    .force('x', d3.forceX(400).strength(0.03))
    .force('y', d3.forceY(200).strength(0.03))
    .force('collision', d3.forceCollide(50))
    .on('tick', function() {
      linkG.attr('d', function(d) {
        var sx = d.source.x, sy = d.source.y, tx = d.target.x, ty = d.target.y;
        var dx = tx - sx, dy = ty - sy, dist = Math.sqrt(dx*dx+dy*dy) || 1;
        var px = -dy/dist, py = dx/dist;
        var total = d._total || 1, idx = d._idx || 0;
        var arcOff = (idx - (total-1)/2) * 28;
        var cx = (sx+tx)/2 + px*arcOff, cy = (sy+ty)/2 + py*arcOff;
        return 'M'+sx+','+sy+' Q'+cx+','+cy+' '+tx+','+ty;
      });
      linkL.attr('x', function(d) {
        var sx = d.source.x, sy = d.source.y, tx = d.target.x, ty = d.target.y;
        var dx = tx - sx, dy = ty - sy, dist = Math.sqrt(dx*dx+dy*dy) || 1;
        var px = -dy/dist;
        var total = d._total || 1, idx = d._idx || 0;
        var arcOff = (idx - (total-1)/2) * 28;
        var cx = (sx+tx)/2 + px*arcOff;
        return 0.25*sx + 0.5*cx + 0.25*tx;
      }).attr('y', function(d) {
        var sx = d.source.x, sy = d.source.y, tx = d.target.x, ty = d.target.y;
        var dy = ty - sy, dist = Math.sqrt((tx-sx)*(tx-sx)+dy*dy) || 1;
        var py = dy/dist;
        var total = d._total || 1, idx = d._idx || 0;
        var arcOff = (idx - (total-1)/2) * 28;
        var cy = (sy+ty)/2 + py*arcOff;
        return 0.25*sy + 0.5*cy + 0.25*ty - 4;
      });
      nodeG.attr('transform', function(d) { return 'translate('+d.x+','+d.y+')'; });
    });

  _topoSim.alpha(1).restart();
  setTimeout(function() {
    svg.transition().duration(500).call(d3.zoom().transform, d3.zoomIdentity);
  }, 100);
}

// ═══════════════════════════════════════════════════════════════
// Demo data
// ═══════════════════════════════════════════════════════════════
function doSimulate() {
  chartHistory = {rate:[], latency:[], frames:[], maxLen: chartHistory.maxLen};
  for (var i=0; i<30; i++) {
    chartHistory.rate.push({v:40+Math.random()*20, raw:1200+i*50});
    chartHistory.latency.push({v:100+Math.random()*200, p99:300+Math.random()*500});
    chartHistory.frames.push({v:80+Math.random()*40, raw:2400+i*100});
  }
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
  window.topoData = topoData;
  updateAll();
  setConnStatus('live','● demo');
  var wm = document.getElementById('demo-watermark');
  if (wm) wm.style.display = '';
  document.getElementById('vehicle-card').style.display = '';
}

// ═══════════════════════════════════════════════════════════════
// SSE Connection + auto-reconnect
// ═══════════════════════════════════════════════════════════════

function startSSE() {
  if (eventSource) { eventSource.close(); eventSource = null; }
  if (sseRenewTimer) { clearTimeout(sseRenewTimer); sseRenewTimer = null; }

  eventSource = new EventSource(serverUrl+'/api/stream');

  // Seamless renewal: server caps each SSE stream at 300s.
  // Reopen at 270s — before server closes — so swap is invisible.
  sseRenewTimer = setTimeout(function() {
    if (!paused) startSSE();
  }, 270000);

  // SSE: only update data model; rendering driven by rAF.
  // Avoid synchronous updateAll() in SSE callback (blocks main thread).
  var _pendingUpdate = false;
  eventSource.onmessage = function(e) {
    if (paused) return;
    try { topoData = JSON.parse(e.data); window.topoData = topoData; }
    catch(err) { return; }
    if (!_pendingUpdate) {
      _pendingUpdate = true;
      requestAnimationFrame(function() {
        _pendingUpdate = false;
        updateAll();
        applyLiveStatus(topoData);
      });
    }
  };

  eventSource.onerror = function() {
    setConnStatus('warn','● retry');
    if (eventSource) { eventSource.close(); eventSource = null; }
    if (sseRenewTimer) { clearTimeout(sseRenewTimer); sseRenewTimer = null; }
    if (reconnectTimer) clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(tryReconnect, 2000);
  };

  eventSource.onopen = function() { setConnStatus('live','● live'); };
}

function tryReconnect() {
  if (paused) { reconnectTimer = setTimeout(tryReconnect, 2000); return; }
  fetch(serverUrl+'/api/topology')
    .then(function(r) { return r.json(); })
    .then(function(d) {
      topoData = d; window.topoData = d;
      updateAll();
      applyLiveStatus(d);
      connectRetries = 0;
      startSSE();
    })
    .catch(function() {
      setConnStatus('warn','● retry');
      reconnectTimer = setTimeout(tryReconnect, 2000);
    });
}

async function doConnect() {
  serverUrl = document.getElementById('url').value.trim();
  saveState();
  if (eventSource) { eventSource.close(); eventSource = null; }
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  try {
    var r = await fetch(serverUrl+'/api/topology');
    topoData = await r.json(); window.topoData = topoData;
    updateAll();
    applyLiveStatus(topoData);
    connectRetries = 0;
    startSSE();
  } catch(err) {
    connectRetries++;
    if (connectRetries <= 3) {
      setConnStatus('warn','● retry ('+connectRetries+'/3)');
      reconnectTimer = setTimeout(doConnect, 1500);
    } else {
      setConnStatus('dead','● offline');
      doSimulate();
    }
  }
}

function doPause() {
  paused = !paused;
  document.getElementById('pause-btn').textContent = paused ? '▶ Resume' : '⏯ Pause';
}

function clearFrames() {
  frames = [];
  frameCount = 0;
  updateFrames();
}

// ═══════════════════════════════════════════════════════════════
// Update pipeline
// ═══════════════════════════════════════════════════════════════

function updateAll() {
  // Each sub-renderer is isolated: a fault must not stop subsequent renders.
  // Failures surface in the diagnostic bar instead of being silently swallowed.
  safeCall('metrics', updateMetrics);
  safeCall('frames', updateFrames);
  safeCall('topicStats', updateTopicStats);
  safeCall('processTopics', updateProcessTopics);
  safeCall('scene3d', update3D);
  safeCall('scene2d', sync2DTarget);    // update 2D target data from topoData
  safeCall('topology', function() {
    var nn = (topoData.nodes||[]).map(function(n){ return n.name; }).sort().join(',');
    if (nn !== lastNodeNames) { lastNodeNames = nn; updateTopo(topoData); }
  });
  safeCall('charts', updateCharts);
  safeCall('counters', function() {
    document.getElementById('node-n').textContent = (topoData.nodes||[]).length;
    document.getElementById('frame-n').textContent = frameCount;
  });
}

function switchSysView(view) {
  document.querySelectorAll('.toggle-btn').forEach(function(b) {
    b.classList.toggle('active', b.dataset.view === view);
  });
  document.getElementById('sys-view-system').style.display = (view === 'system' ? '' : 'none');
  document.getElementById('sys-view-threads').style.display = (view === 'threads' ? '' : 'none');
}

function updateMetrics() {
  var m = topoData.metrics || {}, b = m.bus || {}, l = m.latency || {};
  document.getElementById('m-pub').textContent = (b.published||0).toLocaleString();
  document.getElementById('m-del').textContent = (b.delivered||0).toLocaleString();
  document.getElementById('m-drop').textContent = (b.dropped||0);
  document.getElementById('m-drop').style.color = (b.dropped||0) > 0 ? '#f85149' : '#3fb950';
  document.getElementById('m-lat').textContent = (l.avg_us||0);

  // Alert: drop > 0 -> flash bus card
  var bc = document.getElementById('bus-card');
  if ((b.dropped||0) > 0) {
    bc.classList.add('alert');
    document.getElementById('m-drop').parentElement.querySelector('.lbl').textContent = '⚠ DROPPED';
  } else {
    bc.classList.remove('alert');
    document.getElementById('m-drop').parentElement.querySelector('.lbl').textContent = 'Dropped';
  }

  // Vehicle
  var v = (topoData.metrics||{}).vehicle;
  if (v) {
    document.getElementById('vehicle-card').style.display = '';
    document.getElementById('v-speed').textContent = (v.speed||0).toFixed(1);
    document.getElementById('v-target').textContent = (v.target_speed||0).toFixed(1);
    document.getElementById('v-throttle').textContent = ((v.throttle||0)*100).toFixed(0)+'%';
    document.getElementById('v-throttle').style.color = (v.throttle||0) > 0.5 ? '#d29922' : '#3fb950';
    document.getElementById('v-error').textContent = (v.error||0).toFixed(1);
    var driverMode = (topoData.metrics||{}).driver_mode || 'NA:READY';
    var modeColors = {NA:'#8b949e', ACC:'#58a6ff', CP:'#d29922', NP:'#3fb950', NOA:'#bc8cff'};
    var modeTop = driverMode.split(':')[0];
    var vModeEl = document.getElementById('v-mode');
    vModeEl.textContent = driverMode;
    vModeEl.style.color = modeColors[modeTop] || '#bc8cff';
    var routeLane = (topoData.metrics||{}).route_lane || 0;
    var vRouteEl = document.getElementById('v-route');
    vRouteEl.textContent = routeLane === 0 ? '--' : (routeLane > 0 ? '→ right' : '← left');
    vRouteEl.style.color = routeLane === 0 ? '#484f58' : '#f0883e';
  } else {
    document.getElementById('vehicle-card').style.display = 'none';
  }

  // System resources
  var sm = (topoData.metrics||{}).sysmon;
  if (sm) {
    document.getElementById('sysmon-card').style.display = '';
    var cpu = sm.cpu_total_pct || 0, memp = sm.mem_used_pct || 0;
    var rssMb = Math.round((sm.proc_rss_kb||0)/1024);
    var usedMb = Math.round((sm.mem_used_kb||0)/1024);
    var totMb = Math.round((sm.mem_total_kb||0)/1024);
    document.getElementById('s-cpu').textContent = cpu.toFixed(1);
    document.getElementById('s-cpu').style.color = cpu > 80 ? '#f85149' : (cpu > 50 ? '#d29922' : '#58a6ff');
    document.getElementById('s-mem').textContent = memp.toFixed(1);
    document.getElementById('s-mem').style.color = memp > 85 ? '#f85149' : '#d29922';
    document.getElementById('s-rss').textContent = rssMb;
    document.getElementById('s-load').textContent = (sm.load1||0).toFixed(2);
    document.getElementById('s-detail').textContent =
      'cores '+(sm.cpu_count||0)+' · mem '+usedMb+'/'+totMb+'MB · '+
      'load '+(sm.load1||0).toFixed(2)+'/'+(sm.load5||0).toFixed(2)+'/'+(sm.load15||0).toFixed(2)+' · '+
      'thr '+(sm.thread_count||0)+' · disk R'+((sm.disk_read_bps||0)/1024).toFixed(0)+' W'+((sm.disk_write_bps||0)/1024).toFixed(0)+'KB/s';

    // Threads view
    var thrArr = (sm.threads||[]);
    document.getElementById('thread-cnt').textContent = thrArr.length;
    var nodeNames = (topoData.nodes||[]).map(function(n){ return n.name||''; });
    document.getElementById('threads-list').innerHTML = thrArr.length
      ? thrArr.map(function(th) {
          var cpuColor = th.cpu_pct > 50 ? '#f85149' : (th.cpu_pct > 20 ? '#d29922' : '#3fb950');
          var barColor = th.cpu_pct > 50 ? '#f85149' : (th.cpu_pct > 20 ? '#d29922' : '#3fb950');
          var barPct = Math.min(100, th.cpu_pct * 2);
          var stateIcon = th.state === 'R' ? '🟢' : (th.state === 'S' ? '💤' : '⏸');
          var matchedNode = nodeNames.find(function(nm){ return nm && th.name && th.name.indexOf(nm) >= 0; });
          var nameDisplay = matchedNode ? '<span style="color:#58a6ff">'+matchedNode+'</span>' : '<span>'+th.name+'</span>';
          return '<div class="thread-row" onclick="showNodeDetail(\''+(matchedNode||'')+'\')" style="cursor:'+(matchedNode?'pointer':'default')+'">'+
            '<span class="th-state" title="'+th.state+'">'+stateIcon+'</span>'+
            '<span class="th-name">'+nameDisplay+'</span>'+
            '<span class="th-tid">TID '+th.tid+'</span>'+
            '<span class="th-cpu" style="color:'+cpuColor+'">'+th.cpu_pct.toFixed(1)+'%</span>'+
            '<div class="th-bar"><div class="th-bar-fill" style="width:'+barPct+'%;background:'+barColor+'"></div></div>'+
            '</div>';
        }).join('')
      : '<span style="color:#484f58">No thread data</span>';
  } else {
    document.getElementById('sysmon-card').style.display = 'none';
  }

  // Nodes
  var ns = topoData.nodes || [];
  document.getElementById('node-status').innerHTML = ns.map(function(n) {
    return '<div class="stat-row"><span>'+(n.name||'?')+'</span>'+
      '<span style="color:'+(n.alive===false?'#f85149':'#3fb950')+';cursor:pointer" onclick="showNodeDetail(\''+(n.name||'')+'\')">'+
      (n.alive===false?'💀 Dead':'🟢 PID '+(n.pid||'—'))+'</span></div>';
  }).join('') || '<span style="color:#484f58">No nodes</span>';
}

function updateFrames() {
  var m = topoData.metrics || {}, b = m.bus || {};
  var now = new Date().toISOString().slice(11, 23);
  frames.push({
    ts: now,
    pub: b.published||0,
    del: b.delivered||0,
    drop: b.dropped||0,
    mode: m.driver_mode||'NA',
    lat: (m.latency||{}).avg_us||0
  });
  if (frames.length > 200) frames.shift();
  frameCount++;
  document.getElementById('frames').innerHTML = frames.slice(-60).reverse().map(function(f) {
    return '<div class="frame-line">'+
      '<span style="color:#484f58;min-width:72px">'+f.ts+'</span>'+
      '<span style="color:#58a6ff;min-width:60px">pub:'+f.pub+'</span>'+
      '<span style="min-width:60px">del:'+f.del+'</span>'+
      '<span style="color:'+(f.drop?'#f85149':'#484f58')+';min-width:50px">drop:'+f.drop+'</span>'+
      '<span style="color:#d29922;min-width:60px">lat:'+f.lat+'µs</span>'+
      '<span style="color:#bc8cff;min-width:70px">mode:'+f.mode+'</span></div>';
  }).join('') || '<span style="color:#484f58">Waiting...</span>';
}

// ═══════════════════════════════════════════════════════════════
// Topic QoS / Process Matrix
// ═══════════════════════════════════════════════════════════════

function onFilterChange() { saveState(); updateAll(); }

function updateTopicStats() {
  var ts = (topoData.metrics||{}).topics || [];
  ts = ts.filter(function(t) { return topicMatches(t.topic||t.name||''); });
  if (!ts.length) {
    document.getElementById('topic-stats').innerHTML = '<span style="color:#484f58">Waiting for per-topic data...</span>';
    return;
  }
  document.getElementById('topic-stats').innerHTML =
    '<table style="width:100%;font-size:10px;border-collapse:collapse">'+
    '<tr style="color:#8b949e;border-bottom:1px solid#21262d">'+
    '<th style="text-align:left;padding:2px 3px">Topic</th>'+
    '<th style="text-align:center;padding:2px 3px">QoS</th>'+
    '<th style="text-align:right;padding:2px 3px">Pub</th>'+
    '<th style="text-align:right;padding:2px 3px">Del</th>'+
    '<th style="text-align:right;padding:2px 3px">Drop</th>'+
    '<th style="text-align:right;padding:2px 3px">Lat</th>'+
    '<th style="text-align:right;padding:2px 3px">DL</th>'+
    '<th style="text-align:right;padding:2px 3px">Hz</th></tr>'+
    ts.map(function(t) {
      var dropColor = (t.drop||0) > 0 ? 'color:#f85149' : 'color:#3fb950';
      var dl = (t.deadline_violations||0);
      var dlColor = dl > 0 ? 'color:#f85149;font-weight:bold' : 'color:#3fb950';
      var rel = (t.qos_reliability||t.reliability||'best_effort');
      var relColor = rel === 'reliable' ? 'color:#3fb950' : 'color:#8b949e';
      return '<tr style="border-bottom:1px solid#161b22;cursor:pointer" onclick="var el=document.getElementById(\'topic-filter\');if(el.value===\''+(t.topic||t.name)+'\')el.value=\'\';else el.value=\''+(t.topic||t.name)+'\';onFilterChange()">'+
        '<td style="padding:2px 3px;color:#58a6ff" title="'+(t.topic||t.name)+'">'+((t.topic||t.name||'?').split('/').pop())+'</td>'+
        '<td style="text-align:center;padding:2px 3px;font-size:9px;'+relColor+'" title="depth='+(t.qos_depth||'?')+' '+rel+'">'+rel+'</td>'+
        '<td style="text-align:right;padding:2px 3px">'+(t.pub||0)+'</td>'+
        '<td style="text-align:right;padding:2px 3px">'+(t.del||0)+'</td>'+
        '<td style="text-align:right;padding:2px 3px;'+dropColor+'">'+(t.drop||0)+'</td>'+
        '<td style="text-align:right;padding:2px 3px;color:#d29922" title="p50='+(t.p50_us||'?')+'µs p99='+(t.p99_us||'?')+'µs">'+(t.lat_us||0)+'µs</td>'+
        '<td style="text-align:right;padding:2px 3px;'+dlColor+'" title="deadline violations">'+(dl>999?'999+':dl)+'</td>'+
        '<td style="text-align:right;padding:2px 3px">'+(t.freq||0).toFixed(1)+'</td></tr>';
    }).join('')+'</table>';
}

function updateProcessTopics() {
  var endpoints = (topoData.endpoints||[]).filter(function(e) { return topicMatches(e.topic||''); });
  if (!endpoints.length) {
    (topoData.nodes||[]).forEach(function(n) {
      (n.topics||[]).forEach(function(t) {
        endpoints.push({
          node: n.name,
          topic: t.topic||t.name,
          role: endpointRoleFromCaps(t),
          type_id: t.type_id||'0x00000000',
          freq: Number(t.freq||0)
        });
      });
    });
  }
  if (!endpoints.length) {
    document.getElementById('process-topics').innerHTML = '<span class="muted">Waiting for registry...</span>';
    return;
  }
  document.getElementById('process-topics').innerHTML =
    '<table style="width:100%;font-size:10px;border-collapse:collapse">'+
    '<tr style="color:#8b949e;border-bottom:1px solid#21262d">'+
    '<th style="text-align:left;padding:3px 4px">Process</th>'+
    '<th style="text-align:left;padding:3px 4px">Role</th>'+
    '<th style="text-align:left;padding:3px 4px">Topic</th>'+
    '<th style="text-align:left;padding:3px 4px">Type</th>'+
    '<th style="text-align:right;padding:3px 4px">Freq</th></tr>'+
    endpoints.sort(function(a,b) {
      return (a.node||'').localeCompare(b.node||'') || (a.topic||'').localeCompare(b.topic||'');
    }).map(function(e) {
      return '<tr style="border-bottom:1px solid#161b22;cursor:pointer" onclick="showNodeDetail(\''+(e.node||'')+'\')">'+
        '<td style="padding:3px 4px;color:#c9d1d9">'+(e.node||'?')+'</td>'+
        '<td style="padding:3px 4px"><span class="'+roleClass(e.role||'unknown')+'">'+(e.role||'unknown')+'</span></td>'+
        '<td style="padding:3px 4px" class="topic-full">'+(e.topic||'?')+'</td>'+
        '<td style="padding:3px 4px" class="muted mono">'+(e.type_id||'—')+'</td>'+
        '<td style="text-align:right;padding:3px 4px">'+Number(e.freq||0).toFixed(1)+'Hz</td></tr>';
    }).join('')+'</table>';
}

// ═══════════════════════════════════════════════════════════════
// Node detail panel
// ═══════════════════════════════════════════════════════════════

function showNodeDetail(name) {
  var n = (topoData.nodes||[]).find(function(x) { return x.name === name; });
  if (!n) return;
  document.getElementById('det-name').textContent = n.name;
  var html = '<div class="stat-row"><span class="label">PID</span><span class="value">'+(n.pid||'—')+'</span></div>';
  html += '<div class="stat-row"><span class="label">Status</span><span class="value" style="color:'+(n.alive===false?'#f85149':'#3fb950')+'">'+(n.alive===false?'Dead':'Alive')+'</span></div>';
  if (n.description) html += '<div class="stat-row"><span class="label">Desc</span><span class="value">'+n.description+'</span></div>';
  if (n.plugin) html += '<div class="stat-row"><span class="label">Plugin</span><span class="value mono" style="font-size:10px">'+n.plugin+'</span></div>';
  html += '<div style="margin-top:8px;font-weight:600;color:#8b949e;font-size:10px">Topics</div>';
  (n.topics||[]).forEach(function(t) {
    var role = endpointRoleFromCaps(t);
    html += '<div class="stat-row"><span style="color:#58a6ff;font-size:11px">'+(t.topic||t.name)+'</span><span><span class="'+roleClass(role)+'">'+role+'</span> '+(Number(t.freq||0)>0?Number(t.freq).toFixed(1)+'Hz':'')+'</span></div>';
  });
  // Per-topic live stats for this node
  var ts = (topoData.metrics||{}).topics||[];
  var nodeTopics = (n.topics||[]).map(function(t) { return t.topic||t.name; });
  var matchedTs = ts.filter(function(t) { return nodeTopics.indexOf(t.topic||t.name) >= 0; });
  if (matchedTs.length) {
    html += '<div style="margin-top:8px;font-weight:600;color:#8b949e;font-size:10px">Live Stats</div>';
    matchedTs.forEach(function(t) {
      html += '<div class="stat-row"><span style="font-size:10px">'+((t.topic||t.name).split('/').pop())+'</span><span style="font-size:10px">pub:'+(t.pub||0)+' del:'+(t.del||0)+' lat:'+(t.lat_us||0)+'µs</span></div>';
    });
  }
  // Thread resources for this node
  var smThr = (topoData.metrics||{}).sysmon||{};
  var thrArr = smThr.threads||[];
  var matchThreads = thrArr.filter(function(th) { return th.name && th.name.indexOf(n.name) >= 0; });
  if (matchThreads.length) {
    html += '<div style="margin-top:8px;font-weight:600;color:#8b949e;font-size:10px">Thread Resources</div>';
    matchThreads.forEach(function(th) {
      var cpuColor = th.cpu_pct > 50 ? '#f85149' : (th.cpu_pct > 20 ? '#d29922' : '#3fb950');
      var stateIcon = th.state === 'R' ? '🟢' : (th.state === 'S' ? '💤' : '⏸');
      html += '<div class="stat-row"><span style="font-size:10px">'+stateIcon+' '+th.name+' (TID '+th.tid+')</span><span style="font-size:11px;font-weight:700;color:'+cpuColor+'">'+th.cpu_pct.toFixed(1)+'%</span></div>';
    });
  }
  document.getElementById('det-body').innerHTML = html;
  document.getElementById('detail-panel').classList.add('open');
  document.getElementById('detail-overlay').classList.add('show');
}

function closeDetail() {
  document.getElementById('detail-panel').classList.remove('open');
  document.getElementById('detail-overlay').classList.remove('show');
}

// ═══════════════════════════════════════════════════════════════
// Charts
// ═══════════════════════════════════════════════════════════════

function onChartTopicChange() {
  chartTopic = document.getElementById('chart-topic').value;
  chartPrevPub = -1;
  chartPrevDel = -1;
  chartLastTs = -1;
  chartLastSampleMs = 0;
  chartHistory = {rate:[], latency:[], frames:[], maxLen: chartHistory.maxLen};
  saveState();
}

function onChartRangeChange() {
  chartHistory.maxLen = parseInt(document.getElementById('chart-range').value) || 60;
  chartHistory = {rate:[], latency:[], frames:[], maxLen: chartHistory.maxLen};
}

function populateTopicSelector() {
  var sel = document.getElementById('chart-topic');
  if (!sel) return;
  var cur = sel.value;
  var topics = ((topoData.metrics||{}).topics||[]).map(function(t) { return t.topic||t.name||''; }).filter(Boolean).sort();
  var exist = [];
  for (var i=1; i<sel.options.length; i++) exist.push(sel.options[i].value);
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

function updateCharts() {
  populateTopicSelector();
  var m = topoData.metrics||{}, b = m.bus||{}, l = m.latency||{};
  var nowPub, nowDel, nowLat, p99Lat;

  if (chartTopic) {
    var ts = (m.topics||[]).filter(function(t) { return (t.topic||t.name||'') === chartTopic; });
    if (ts.length) {
      var t = ts[0];
      nowPub = t.pub||0; nowDel = t.del||0; nowLat = t.lat_us||0; p99Lat = t.p99_us||0;
    } else {
      nowPub = 0; nowDel = 0; nowLat = 0; p99Lat = 0;
    }
  } else {
    nowPub = b.published||0; nowDel = b.delivered||0;
    nowLat = l.avg_us||0; p99Lat = l.p99_us||0;
  }

  var suf = chartTopic ? ' ('+chartTopic.split('/').pop()+')' : '';

  // Deduplicate by server timestamp to avoid flattening the curve
  var dataTs = topoData.timestamp || 0;
  if (chartLastTs >= 0 && dataTs === chartLastTs) {
    drawChart('ch-rate','tt-rate', chartHistory.rate, '#58a6ff', 'Pub Rate'+suf, 'msg/s');
    drawChart('ch-lat','tt-lat', chartHistory.latency, '#d29922', 'Latency'+suf, 'µs');
    drawChart('ch-frames','tt-frames', chartHistory.frames, '#3fb950', 'Deliveries'+suf, 'msg/s');
    return;
  }

  var nowMs = Date.now();
  var elapsed = (chartLastSampleMs > 0) ? Math.max(0.05, (nowMs - chartLastSampleMs)/1000) : 1;
  chartLastTs = dataTs;
  chartLastSampleMs = nowMs;

  var dp = chartPrevPub >= 0 ? Math.max(0, (nowPub - chartPrevPub)/elapsed) : 0;
  var dd = chartPrevDel >= 0 ? Math.max(0, (nowDel - chartPrevDel)/elapsed) : 0;

  chartHistory.rate.push({v:dp, raw:nowPub});
  chartHistory.latency.push({v:nowLat, p99:p99Lat});
  chartHistory.frames.push({v:dd, raw:nowDel});

  chartPrevPub = nowPub;
  chartPrevDel = nowDel;

  ['rate','latency','frames'].forEach(function(a) {
    while (chartHistory[a].length > chartHistory.maxLen) chartHistory[a].shift();
  });

  drawChart('ch-rate','tt-rate', chartHistory.rate, '#58a6ff', 'Pub Rate'+suf, 'msg/s');
  drawChart('ch-lat','tt-lat', chartHistory.latency, '#d29922', 'Latency'+suf, 'µs');
  drawChart('ch-frames','tt-frames', chartHistory.frames, '#3fb950', 'Deliveries'+suf, 'msg/s');
}

function drawChart(canvasId, tooltipId, data, color, title, unit) {
  var c = document.getElementById(canvasId);
  if (!c) return;
  var box = c.parentElement, W = box.clientWidth, H = box.clientHeight;

  // Fallback if client rect is zero (e.g. element hidden)
  if (W < 10) W = parseInt(box.style.width) || 280;
  if (H < 10) H = parseInt(box.style.height) || 240;
  if (W < 10 || H < 10) return;

  c.width = W * 2; c.height = H * 2;
  c.style.width = W+'px'; c.style.height = H+'px';
  var ctx = c.getContext('2d'); ctx.scale(2, 2);

  var pad = {t:36, r:66, b:34, l:48};
  var w = W - pad.l - pad.r, h = H - pad.t - pad.b;

  ctx.clearRect(0, 0, W, H);
  var bg = ctx.createLinearGradient(0,0,0,H);
  bg.addColorStop(0, '#0d1117'); bg.addColorStop(1, '#090c10');
  ctx.fillStyle = bg; ctx.fillRect(0,0,W,H);

  if (!data || data.length < 2) {
    ctx.fillStyle = '#30363d'; ctx.font = '11px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('Waiting for data...', W/2, H/2);
    ctx.fillStyle = '#8b949e'; ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'left';
    ctx.fillText(title, pad.l, pad.t-10);
    return;
  }

  var vals = data.map(function(d) { return d.v != null ? d.v : d; });
  var maxV = Math.max.apply(null, vals.concat([1]));
  var minV = Math.min.apply(null, vals.concat([0]));
  var range = (maxV - minV) || 1;
  var latest = vals[vals.length-1];
  var avg = vals.reduce(function(a,b) { return a+b; }, 0) / vals.length;
  var xs = w / Math.max(data.length-1, 1);
  var ys = h / range;

  // Grid
  var yTicks = 4;
  ctx.strokeStyle = '#161b22'; ctx.lineWidth = 0.5;
  for (var i=0; i<=yTicks; i++) {
    var y = pad.t + (h*i)/yTicks;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W-pad.r, y); ctx.stroke();
    ctx.fillStyle = '#484f58'; ctx.font = '9px system-ui'; ctx.textAlign = 'right';
    ctx.fillText(Math.round(maxV - (range*i)/yTicks).toLocaleString(), pad.l-8, y+3);
  }

  // Avg line
  var avgY = pad.t + (maxV - avg) * ys;
  ctx.strokeStyle = color+'44'; ctx.lineWidth = 1; ctx.setLineDash([4,6]);
  ctx.beginPath(); ctx.moveTo(pad.l, avgY); ctx.lineTo(W-pad.r, avgY); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = color+'88'; ctx.font = '9px system-ui'; ctx.textAlign = 'right';
  ctx.fillText('avg '+Math.round(avg).toLocaleString(), W-pad.r, avgY-3);

  // Area fill
  ctx.fillStyle = color+'15'; ctx.beginPath();
  data.forEach(function(d,i) {
    var x = pad.l + i*xs, y = pad.t + (maxV - (d.v!=null ? d.v : d)) * ys;
    i === 0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.lineTo(pad.l + (data.length-1)*xs, pad.t+h);
  ctx.lineTo(pad.l, pad.t+h);
  ctx.closePath(); ctx.fill();

  // Line
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.beginPath();
  data.forEach(function(d,i) {
    var x = pad.l + i*xs, y = pad.t + (maxV - (d.v!=null ? d.v : d)) * ys;
    i === 0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.stroke();

  // P99 line
  if (data[0] && data[0].p99 != null) {
    ctx.strokeStyle = '#f85149'; ctx.lineWidth = 1.5; ctx.setLineDash([3,5]); ctx.beginPath();
    data.forEach(function(d,i) {
      var x = pad.l + i*xs, y = pad.t + (maxV - d.p99) * ys;
      i === 0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.stroke(); ctx.setLineDash([]);
    var lastP99 = data[data.length-1].p99, p99Y = pad.t + (maxV - lastP99) * ys;
    ctx.fillStyle = '#f85149'; ctx.font = 'bold 9px system-ui'; ctx.textAlign = 'left';
    ctx.fillText('p99 '+Math.round(lastP99), W-pad.r+4, p99Y+3);
    ctx.fillStyle = color; ctx.font = '9px system-ui'; ctx.textAlign = 'left';
    ctx.fillText('─ avg', pad.l, pad.t+h+12);
    ctx.fillStyle = '#f85149';
    ctx.fillText('── p99', pad.l+50, pad.t+h+12);
  }

  // Time hints
  ctx.fillStyle = '#30363d'; ctx.font = '8px system-ui'; ctx.textAlign = 'right';
  ctx.fillText('now', W-pad.r, pad.t+h+12);
  ctx.textAlign = 'left';
  ctx.fillText(data.length+'s ago', pad.l, pad.t+h+12);

  // Big value
  ctx.fillStyle = color; ctx.font = 'bold 22px system-ui'; ctx.textAlign = 'right';
  ctx.fillText(Math.round(latest).toLocaleString(), W-pad.r, pad.t+18);

  // Title
  ctx.fillStyle = '#c9d1d9'; ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'left';
  ctx.fillText(title, pad.l, pad.t-10);
  ctx.fillStyle = '#484f58'; ctx.font = '9px system-ui'; ctx.textAlign = 'right';
  ctx.fillText(unit, W-pad.r-52, pad.t-10);

  // Hover tooltip
  c.onmousemove = function(e) {
    var rect = c.getBoundingClientRect();
    var mx = e.clientX - rect.left, my = e.clientY - rect.top;
    if (mx < pad.l || mx > W-pad.r) { document.getElementById(tooltipId).style.display = 'none'; return; }
    var idx = Math.round((mx - pad.l) / xs);
    if (idx < 0 || idx >= data.length) { document.getElementById(tooltipId).style.display = 'none'; return; }
    var d = data[idx], v = d.v != null ? d.v : d;
    var tt = document.getElementById(tooltipId);
    var html = '<b>'+title+'</b> '+Math.round(v).toLocaleString()+' '+unit;
    if (d.p99 != null) html += '<br><span style="color:#f85149">p99 '+Math.round(d.p99)+' µs</span>';
    html += '<br><span style="color:#484f58">'+(data.length-idx)+'s ago</span>';
    tt.innerHTML = html;
    tt.style.display = 'block';
    tt.style.left = Math.min(pad.l+idx*xs+10, W-120)+'px';
    tt.style.top = Math.max(pad.t+(maxV-v)*ys-30, 4)+'px';
  };
  c.onmouseleave = function() { document.getElementById(tooltipId).style.display = 'none'; };
}

// ═══════════════════════════════════════════════════════════════
// Export
// ═══════════════════════════════════════════════════════════════

function toggleExportMenu() {
  document.getElementById('export-menu').classList.toggle('show');
}

function exportPNG() {
  var svg = document.querySelector('#topo svg');
  if (!svg) return;
  var clone = svg.cloneNode(true), w = svg.clientWidth, h = svg.clientHeight;
  clone.setAttribute('width', w); clone.setAttribute('height', h);
  var data = new XMLSerializer().serializeToString(clone);
  var canvas = document.createElement('canvas');
  canvas.width = w*2; canvas.height = h*2;
  var ctx = canvas.getContext('2d');
  var img = new Image();
  img.onload = function() {
    ctx.drawImage(img, 0, 0);
    var a = document.createElement('a');
    a.download = 'topology.png';
    a.href = canvas.toDataURL();
    a.click();
    toast('PNG exported');
  };
  img.src = 'data:image/svg+xml;base64,'+btoa(unescape(encodeURIComponent(data)));
  document.getElementById('export-menu').classList.remove('show');
}

function exportCSV() {
  var ts = (topoData.metrics||{}).topics||[];
  var csv = 'topic,pub,del,drop,lat_us,freq,subs\n';
  ts.forEach(function(t) {
    csv += [t.topic||t.name, t.pub||0, t.del||0, t.drop||0, t.lat_us||0, t.freq||0, t.subs||0].join(',')+'\n';
  });
  var a = document.createElement('a');
  a.download = 'qos.csv';
  a.href = 'data:text/csv;charset=utf-8,'+encodeURIComponent(csv);
  a.click();
  toast('CSV exported');
  document.getElementById('export-menu').classList.remove('show');
}

function toast(msg) {
  var t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(function() { t.classList.remove('show'); }, 1500);
}

// ═══════════════════════════════════════════════════════════════
// Training modal
// ═══════════════════════════════════════════════════════════════

function defaultTrainName() {
  var b = document.getElementById('train-backend').value || 'torch';
  var d = new Date();
  var pad = function(n) { return String(n).padStart(2,'0'); };
  return 'e2e_'+b+'_'+d.getFullYear()+pad(d.getMonth()+1)+pad(d.getDate())+'_'+pad(d.getHours())+pad(d.getMinutes())+pad(d.getSeconds());
}

function openTrainingModal() {
  document.getElementById('training-modal').classList.add('show');
  if (!document.getElementById('train-name').value) document.getElementById('train-name').value = defaultTrainName();
  refreshTrainingStatus();
}

function closeTrainingModal() {
  document.getElementById('training-modal').classList.remove('show');
}

function syncTrainingForm() {
  var backend = document.getElementById('train-backend').value;
  document.getElementById('train-init').disabled = (backend !== 'torch');
  document.getElementById('train-name').value = defaultTrainName();
}

async function refreshTrainingStatus() {
  try {
    var r = await fetch(serverUrl+'/api/training/status');
    var d = await r.json();
    renderTrainingStatus(d);
  } catch(err) {
    document.getElementById('train-status').textContent = 'offline';
    document.getElementById('train-status').style.color = '#f85149';
  }
}

function renderTrainingStatus(d) {
  var job = d.job || {}, models = d.models || [];
  var st = document.getElementById('train-status'), modelEl = document.getElementById('train-model');
  st.textContent = job.running ? 'running' : (job.returncode===0 ? 'done' : (job.error ? 'failed' : 'idle'));
  st.style.color = job.running ? '#d29922' : (job.returncode===0 ? '#3fb950' : (job.error ? '#f85149' : '#58a6ff'));
  modelEl.textContent = job.model_name || '—';
  var log = (job.log_tail||[]).join('\n');
  var logEl = document.getElementById('train-log');
  if (logEl) { logEl.textContent = log || 'No training job yet.'; logEl.scrollTop = logEl.scrollHeight; }
  renderModelList(models);

  var init = document.getElementById('train-init');
  if (init) {
    var cur = init.value;
    init.innerHTML = '<option value="">none</option>'+
      models.filter(function(m) { return m.backend === 'torch'; })
        .map(function(m) { return '<option value="'+m.name+'">'+m.name+'</option>'; }).join('');
    init.value = cur;
  }

  if (job.running && !trainingPollTimer) trainingPollTimer = setInterval(refreshTrainingStatus, 1500);
  if (!job.running && trainingPollTimer) { clearInterval(trainingPollTimer); trainingPollTimer = null; }
}

function renderModelList(models) {
  var el = document.getElementById('train-models');
  if (!el) return;
  if (!models.length) { el.innerHTML = '<span class="muted">No model artifacts yet</span>'; return; }
  el.innerHTML = models.slice().sort(function(a,b) { return (b.mtime||0) - (a.mtime||0); }).map(function(m) {
    var metric = m.metrics||{};
    var loss = metric.mse != null ? (' mse '+Number(metric.mse).toFixed(4)) : (metric.mae != null ? (' mae '+Number(metric.mae).toFixed(4)) : '');
    var promote = m.promotable ? '<button onclick="promoteTrainingModel(\''+m.name+'\')">Promote</button>' : '';
    return '<div class="model-row"><span><b style="color:#58a6ff">'+m.name+'</b><br><span class="muted">'+m.backend+' · samples '+(m.sample_count||'?')+loss+'</span></span><span>'+promote+'</span></div>';
  }).join('');
}

function readOptionalInt(id) {
  var v = document.getElementById(id).value.trim();
  return v ? parseInt(v,10) : null;
}

async function startTraining() {
  var payload = {
    backend: document.getElementById('train-backend').value,
    name: document.getElementById('train-name').value.trim(),
    init_from: document.getElementById('train-init').value,
    run_demo_seconds: readOptionalInt('train-run-demo'),
    epochs: readOptionalInt('train-epochs'),
    hidden: readOptionalInt('train-hidden')
  };
  try {
    var r = await fetch(serverUrl+'/api/training/start', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(payload)
    });
    var d = await r.json();
    if (!d.ok) { toast(d.error||'training failed to start'); return; }
    toast('training started');
    renderTrainingStatus({job:d.job, models:[]});
    refreshTrainingStatus();
  } catch(err) {
    toast('training API offline');
  }
}

async function promoteTrainingModel(name) {
  try {
    var r = await fetch(serverUrl+'/api/training/promote', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({name: name})
    });
    var d = await r.json();
    toast(d.ok ? 'promoted '+name : (d.output||d.error||'promote failed'));
    refreshTrainingStatus();
  } catch(err) {
    toast('promote API offline');
  }
}

// ═══════════════════════════════════════════════════════════════
// 2D target sync — runs inside updateAll() when data arrives
// ═══════════════════════════════════════════════════════════════

var gpsHistory = [];

function sync2DTarget() {
  var m = (topoData.metrics||{}), scn = m.scene, v = m.vehicle||{};
  // Access the 2D state object managed by scene2d.js
  var _2d = window._2d;
  if (!_2d || !_2d.active) return;

  // Update target from scene.ego or vehicle telemetry
  if (scn && scn.ego) {
    _2d.egoT.x = scn.ego.x||0;
    _2d.egoT.y = scn.ego.y||0;
    _2d.egoT.heading = scn.ego.heading||0;
    _2d.egoT.speed = scn.ego.speed||0;
  } else if (v) {
    _2d.egoT.x = v.x||0;
    _2d.egoT.y = v.y||0;
    _2d.egoT.speed = v.speed||0;
    // Derive heading from GPS history
    gpsHistory.push({x:v.x||0, z:(v.y||0)*5});
    if (gpsHistory.length > 60) gpsHistory.shift();
    if (gpsHistory.length > 1) {
      var l = gpsHistory[gpsHistory.length-1];
      var p = gpsHistory[gpsHistory.length-2];
      _2d.egoT.heading = Math.atan2(l.x - p.x, l.z - p.z);
    }
  }

  // Trail
  _2d.trail.push({x:_2d.egoT.x, y:_2d.egoT.y});
  if (_2d.trail.length > 120) _2d.trail.shift();
}

// ═══════════════════════════════════════════════════════════════
// UI helpers
// ═══════════════════════════════════════════════════════════════

function toggleCard(hdr) {
  hdr.parentElement.classList.toggle('collapsed');
  saveState();
  // If the 3D card was just expanded, resize the renderer
  setTimeout(function() {
    var card = hdr.parentElement;
    if (!card.classList.contains('collapsed')) {
      resize3D();
    }
  }, 50);
}

function resetView() {
  var svg = document.querySelector('#topo svg');
  if (!svg) return;
  d3.select('#topo svg').transition().duration(500).call(d3.zoom().transform, d3.zoomIdentity);
}

// ═══════════════════════════════════════════════════════════════
// Keyboard shortcuts
// ═══════════════════════════════════════════════════════════════

document.addEventListener('keydown', function(e) {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.key === 'r' || e.key === 'R') { e.preventDefault(); resetView(); }
  if (e.key === 'f' || e.key === 'F') {
    e.preventDefault();
    var svg = document.querySelector('#topo svg');
    if (svg) d3.select('#topo svg').transition().duration(500).call(d3.zoom().transform, d3.zoomIdentity.translate(400,300).scale(0.8));
  }
  if (e.key === 'Escape') {
    e.preventDefault();
    selectedNode = null;
    closeDetail();
    updateTopo(topoData);
  }
});

// ═══════════════════════════════════════════════════════════════
// Render loop — drives 3D dead-reckoning at 60fps
// ═══════════════════════════════════════════════════════════════

function renderLoop() {
  _renderFrame();
  requestAnimationFrame(renderLoop);
}

// ═══════════════════════════════════════════════════════════════
// Init — called on DOMContentLoaded
// ═══════════════════════════════════════════════════════════════

function initAll() {
  // 1. Initialize D3 topology graph
  initTopo();

  // 2. Initialize 3D scene (Three.js — loads asynchronously, falls back to 2D)
  init3DScene();

  // 3. Initialize 2D canvas (will be shown if 3D fails to load)
  init2D();

  // 4. Initialize charts
  initCharts();

  // 5. Start the render loop (60fps, drives 3D dead-reckoning)
  requestAnimationFrame(renderLoop);

  // 6. Restore saved UI state
  setTimeout(function() {
    var sel = document.getElementById('chart-range');
    if (sel) sel.value = String(chartHistory.maxLen);
    if (chartTopic) document.getElementById('chart-topic').value = chartTopic;

    // Restore collapsed cards — but NEVER collapse the 3D scene card
    try {
      var s = JSON.parse(localStorage.getItem('flowboard')||'{}');
      (s.collapsed||[]).forEach(function(name) {
        document.querySelectorAll('.card-header h2').forEach(function(h) {
          var card = h.parentElement.parentElement;
          if (card.id === 'scene3d-card') return;
          if (h.textContent.trim().startsWith(name.substring(0,8))) card.classList.add('collapsed');
        });
      });
    } catch(e) {}
    // Ensure 3D card is always open on load
    var sc = document.getElementById('scene3d-card');
    if (sc) sc.classList.remove('collapsed');
    // Resize 3D renderer once card is visible
    setTimeout(resize3D, 200);

    // Re-resize on window resize (debounced)
    var _resizeDebounce;
    window.addEventListener('resize', function() {
      clearTimeout(_resizeDebounce);
      _resizeDebounce = setTimeout(resize3D, 100);
    });

    // On mobile: re-resize when 3D card scrolls into view
    try {
      var scene3dObserver = new IntersectionObserver(function(entries) {
        if (entries[0].isIntersecting) { resize3D(); scene3dObserver.disconnect(); }
      }, {threshold: 0.1});
      var scene3dElement = document.getElementById('scene3d');
      if (scene3dElement) scene3dObserver.observe(scene3dElement);
    } catch(e) {}
  }, 100);

  // 7. Connect to server (with fallback to demo)
  setTimeout(function() {
    doConnect().catch(function() { doSimulate(); });
  }, 100);

  // 8. Background data simulation when not connected
  setInterval(function() {
    if (!eventSource && !paused) {
      var m = topoData.metrics||{}, b = m.bus||{}, l = m.latency||{};
      b.published = (b.published||1000) + Math.floor(Math.random()*50 - 25);
      b.delivered = (b.delivered||2000) + Math.floor(Math.random()*100 - 50);
      b.dropped = Math.random() < 0.05 ? (b.dropped||0) + 1 : (b.dropped||0);
      l.avg_us = Math.max(80, (l.avg_us||150) + Math.floor(Math.random()*20 - 10));
      l.p99_us = Math.max(200, (l.p99_us||400) + Math.floor(Math.random()*60 - 30));
      updateAll();
    }
  }, 1000);
}

// ═══════════════════════════════════════════════════════════════
// Global exports for inline onclick handlers in HTML
// ═══════════════════════════════════════════════════════════════

window.topoData = topoData;
window.doToggle = toggleCard;
window.toggleCard = toggleCard;
window.doConnect = doConnect;
window.doSimulate = doSimulate;
window.doPause = doPause;
window.clearFrames = clearFrames;
window.resetView = resetView;
window.onFilterChange = onFilterChange;
window.showNodeDetail = showNodeDetail;
window.closeDetail = closeDetail;
window.switchSysView = switchSysView;
window.onChartTopicChange = onChartTopicChange;
window.onChartRangeChange = onChartRangeChange;
window.toggleExportMenu = toggleExportMenu;
window.exportPNG = exportPNG;
window.exportCSV = exportCSV;
window.openTrainingModal = openTrainingModal;
window.closeTrainingModal = closeTrainingModal;
window.syncTrainingForm = syncTrainingForm;
window.refreshTrainingStatus = refreshTrainingStatus;
window.startTraining = startTraining;
window.promoteTrainingModel = promoteTrainingModel;
window.clearDiag = clearDiag;
window.initTopo = initTopo;
window.initAll = initAll;
window.updateAll = updateAll;
window.drawChart = drawChart;
window.updateCharts = updateCharts;
window.updateFrames = updateFrames;
window.updateMetrics = updateMetrics;
window.updateTopicStats = updateTopicStats;

// ═══════════════════════════════════════════════════════════════
// Boot
// ═══════════════════════════════════════════════════════════════

document.addEventListener('DOMContentLoaded', initAll);
