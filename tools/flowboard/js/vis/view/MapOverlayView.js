/**
 * MapOverlayView.js — 小地图 + HUD 叠加层
 *
 * 纯 Canvas 2D 渲染，零 THREE 依赖。
 * - 右上角：小地图（200×200px，半透明，ego 为中心 100×100m）
 * - 左下角：HUD（speed/heading/steer/ai_state）
 *
 * Phase 3 升级：提供位姿信息叠加。
 */

const MINIMAP_SIZE = 200;
const MINIMAP_RANGE = 100;  // 小地图覆盖范围（米）

let _canvas = null;
let _ctx = null;
let _initialized = false;

function _ensureCanvas() {
  if (_initialized) return true;
  try {
    _canvas = document.createElement('canvas');
    _canvas.id = 'vis-minimap';
    _canvas.width = MINIMAP_SIZE + 200;  // 右侧留白给 HUD 备用
    _canvas.height = MINIMAP_SIZE + 120; // 底部留白给 HUD
    _canvas.style.cssText = [
      'position: fixed',
      'top: 10px',
      'right: 10px',
      'width: ' + (MINIMAP_SIZE + 200) + 'px',
      'height: ' + (MINIMAP_SIZE + 120) + 'px',
      'pointer-events: none',  // 点击穿透到 3D 场景
      'z-index: 100',
      'border-radius: 8px',
    ].join(';');
    document.body.appendChild(_canvas);
    _ctx = _canvas.getContext('2d');
    _initialized = true;
    return true;
  } catch (e) {
    // Node.js 测试环境无 document
    return false;
  }
}

/** 将世界坐标（ENU 米）映射到小地图像素坐标 */
function _worldToMap(wx, wy, egoX, egoY) {
  const scale = MINIMAP_SIZE / MINIMAP_RANGE;
  const dx = wx - egoX;
  const dy = wy - egoY;
  // 小地图：北（+y）向上，东（+x）向右
  const mx = MINIMAP_SIZE / 2 + dx * scale;
  const my = MINIMAP_SIZE / 2 - dy * scale;  // y 翻转（屏幕坐标 y 向下）
  return { mx, my };
}

/** 绘制小地图 */
function _drawMinimap(ctx, ego, entities, roadNetwork) {
  const x0 = 10, y0 = 10;

  // 半透明背景
  ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
  ctx.fillRect(x0, y0, MINIMAP_SIZE, MINIMAP_SIZE);

  // 边框
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
  ctx.lineWidth = 1;
  ctx.strokeRect(x0, y0, MINIMAP_SIZE, MINIMAP_SIZE);

  if (!ego) return;
  const egoX = ego.x || 0;
  const egoY = ego.y || 0;

  // 绘制道路（简化：从 roadNetwork 提取边）
  if (roadNetwork && roadNetwork.edges) {
    ctx.strokeStyle = 'rgba(200, 200, 200, 0.4)';
    ctx.lineWidth = 1;
    for (const edge of roadNetwork.edges) {
      if (!edge || !edge.nodes) continue;
      const nodes = edge.nodes;
      for (let i = 1; i < nodes.length; i++) {
        const p1 = _worldToMap(nodes[i - 1][0], nodes[i - 1][1], egoX, egoY);
        const p2 = _worldToMap(nodes[i][0], nodes[i][1], egoX, egoY);
        ctx.beginPath();
        ctx.moveTo(x0 + p1.mx, y0 + p1.my);
        ctx.lineTo(x0 + p2.mx, y0 + p2.my);
        ctx.stroke();
      }
    }
  }

  // 绘制 NPC
  if (entities) {
    for (const ent of entities) {
      if (!ent || ent.type === 'ego') continue;
      const p = _worldToMap(ent.x || 0, ent.y || 0, egoX, egoY);
      if (p.mx < -10 || p.mx > MINIMAP_SIZE + 10 || p.my < -10 || p.my > MINIMAP_SIZE + 10) continue;
      ctx.fillStyle = 'rgba(255, 100, 100, 0.8)';
      ctx.beginPath();
      ctx.arc(x0 + p.mx, y0 + p.my, 2, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // 绘制 ego（蓝色三角，指向上（北））
  const egoCX = x0 + MINIMAP_SIZE / 2;
  const egoCY = y0 + MINIMAP_SIZE / 2;
  const heading = ego.heading || 0;
  const triSize = 5;
  ctx.fillStyle = '#4488ff';
  ctx.beginPath();
  ctx.moveTo(egoCX + Math.cos(heading) * triSize, egoCY - Math.sin(heading) * triSize);
  ctx.lineTo(egoCX + Math.cos(heading + 2.5) * triSize, egoCY - Math.sin(heading + 2.5) * triSize);
  ctx.lineTo(egoCX + Math.cos(heading - 2.5) * triSize, egoCY - Math.sin(heading - 2.5) * triSize);
  ctx.closePath();
  ctx.fill();
}

/** 绘制 HUD */
function _drawHUD(ctx, ego) {
  if (!ego) return;
  const x0 = 10, y0 = MINIMAP_SIZE + 20;

  const speed = (ego.speed || 0).toFixed(1);
  const heading = ((ego.heading || 0) * 180 / Math.PI % 360).toFixed(0);
  const steer = ((ego.steer || 0) * 180 / Math.PI).toFixed(1);
  const aiState = ego.ai_state || '';

  ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
  ctx.fillRect(x0, y0, 200, 100);

  ctx.font = 'bold 16px monospace';
  ctx.fillStyle = '#ffffff';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText('Speed: ' + speed + ' m/s', x0 + 8, y0 + 8);
  ctx.fillStyle = '#88aaff';
  ctx.fillText('Heading: ' + heading + '°', x0 + 8, y0 + 30);
  ctx.fillStyle = '#88ff88';
  ctx.fillText('Steer: ' + steer + '°', x0 + 8, y0 + 52);
  if (aiState) {
    ctx.fillStyle = '#ffaa44';
    ctx.fillText('State: ' + aiState, x0 + 8, y0 + 74);
  }
}

// ── View 工厂 ──

export function createMapOverlayView(scene) {
  if (!_ensureCanvas()) {
    // Node.js 环境静默降级
    return { update: () => {}, clear: () => {} };
  }

  let _lastDrawHash = '';

  /** 计算一个简单的 hash 来避免不必要的重绘 */
  function _stateHash(store) {
    if (!store) return '';
    const ego = store.ego || {};
    const entCount = (store.entities || []).length;
    return [ego.x, ego.y, ego.heading, ego.speed, ego.steer, ego.ai_state, entCount, store.roadHash].join('|');
  }

  function update(store, now) {
    if (!store || !_ctx) return;

    // 状态未变则跳过重绘（30fps 足够）
    const h = _stateHash(store);
    if (h === _lastDrawHash) return;
    _lastDrawHash = h;

    const ctx = _ctx;
    const w = _canvas.width, hc = _canvas.height;

    // 清空
    ctx.clearRect(0, 0, w, hc);

    _drawMinimap(ctx, store.ego, store.entities, store.roadNetwork);
    _drawHUD(ctx, store.ego);
  }

  function clear() {
    if (_canvas && _canvas.parentNode) {
      _canvas.parentNode.removeChild(_canvas);
    }
    _canvas = null;
    _ctx = null;
    _initialized = false;
  }

  return { update, clear };
}