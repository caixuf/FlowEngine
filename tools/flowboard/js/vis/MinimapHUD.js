/**
 * MinimapHUD.js — 3D 场景叠层 2D 小地图
 *
 * 叠在 #scene3d 右上角的绝对定位 canvas，每帧用 2D Canvas API 绘制：
 *   - 道路中心线（从 roadNetwork.edges）
 *   - NPC 点（entities）
 *   - Ego 箭头（带朝向）
 *   - 视野范围跟随 ego（±range 米，自动平移）
 *
 * 坐标约定：
 *   - roadNetwork.edges 用 THREE 坐标系（start_x, start_z; THREE.z = -ENU.y）
 *   - ego/entities 用 ENU（x, y）
 *   - 小地图统一用 ENU：enu_x = three_x, enu_y = -three_z
 */

import {
  drawRoadNetwork2D,
  strokePolyline,
  worldToEgoCanvas,
} from './math/MapProjection.js';

const W = 220;   // canvas 物理像素宽（px）
const H = 160;   // canvas 物理像素高（px）
const DEFAULT_RANGE = 120;  // 显示范围（ENU 米，半宽）

export function createMinimapHUD(container) {
  // ── DOM ──
  const pip = document.createElement('div');
  pip.id = 'minimap-pip';

  const label = document.createElement('span');
  label.id = 'minimap-label';
  label.textContent = '地图';
  pip.appendChild(label);

  const btn = document.createElement('button');
  btn.id = 'minimap-toggle';
  btn.title = '隐藏小地图（M 键切换）';
  btn.textContent = '×';
  btn.addEventListener('click', () => toggle());
  pip.appendChild(btn);

  const canvas = document.createElement('canvas');
  canvas.id = 'minimap-canvas';
  canvas.width = W;
  canvas.height = H;
  pip.appendChild(canvas);
  container.appendChild(pip);

  const ctx = canvas.getContext('2d');
  let _visible = true;
  let _range = DEFAULT_RANGE;

  // ── 坐标映射：ENU → canvas 相对坐标（相对 ego 中心，未平移/旋转）──
  // 车头朝上模式（2026-08）：draw() 外层 translate(W/2,H/2)+rotate(-h)，
  // 此处只返回相对偏移。ENU y 轴向上 → canvas y 向下取负。
  function toCanvas(ex, ey, cx, cy, heading) {
    const scale = Math.min(W, H) / (2 * _range);
    return worldToEgoCanvas(ex, ey, cx, cy, heading, scale, 0, 0);
  }

  // ── 主绘制函数（每帧调用）──
  function draw(store) {
    if (!_visible || !store) return;

    const ego = store.ego;
    const cx = ego ? ego.x : 0;
    const cy = ego ? ego.y : 0;
    const h = (ego && ego.heading) || 0;

    ctx.clearRect(0, 0, W, H);

    // 背景
    ctx.fillStyle = 'rgba(8, 12, 20, 0.82)';
    ctx.fillRect(0, 0, W, H);

    // 车头朝上（与主场景第三人称一致）：绕 ego 中心旋转 -h。
    // 旧实现固定北向上 → 车向东开时主场景车头朝上、小地图箭头朝右，
    // 观感"小地图与真实场景不同步"（2026-08 修复）。
    // ENU heading 逆时针为正（0=东），canvas 旋转顺时针为正 → 取负。
    ctx.save();
    ctx.translate(W / 2, H / 2);

    // ── 道路 ──
    drawRoadNetwork2D(ctx, store.roadNetwork,
      (x, y) => toCanvas(x, y, cx, cy, h),
      {
        asphalt: '#253246', edgeColor: '#44546d', dividerColor: '#8392aa',
        pxPerMeter: Math.min(W, H) / (2 * _range),
      });

    strokePolyline(ctx, store.trajectoryPath,
      (x, y) => toCanvas(x, y, cx, cy, h), '#58a6ff', 1.5, [4, 3]);

    // ── NPC 实体（车头朝上坐标系：相对 ego 中心）──
    const entities = store.entities || [];
    for (const e of entities) {
      if (!e || e.type === 'pedestrian') continue;
      const [px, py] = toCanvas(e.x, e.y, cx, cy, h);
      if (Math.abs(px) > W / 2 + 8 || Math.abs(py) > H / 2 + 8) continue;
      ctx.fillStyle = e.type === 'vehicle' ? '#f7a825' : '#a0b4c8';
      ctx.beginPath();
      ctx.arc(px, py, 3.5, 0, Math.PI * 2);
      ctx.fill();
    }

    // ── Ego 箭头（车头朝上：旋转已由外层 rotate(-h) 完成，箭头恒朝上）──
    if (ego) {
      const arrowLen = 9, arrowW = 5;

      // 车身矩形（箭头尖端朝 -y = canvas 向上）
      ctx.fillStyle = '#3fb950';
      ctx.fillRect(-arrowW / 2, -arrowLen * 0.3, arrowW, arrowLen * 0.8);

      // 箭头头部三角
      ctx.beginPath();
      ctx.moveTo(0, -arrowLen);
      ctx.lineTo(-arrowW * 0.8, -arrowLen * 0.3);
      ctx.lineTo(arrowW * 0.8, -arrowLen * 0.3);
      ctx.closePath();
      ctx.fillStyle = '#5cff7a';
      ctx.fill();

      // 速度圆圈（外环，绕 ego 中心 = 坐标原点）
      ctx.strokeStyle = '#3fb950';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(0, 0, 7, 0, Math.PI * 2);
      ctx.stroke();
    }

    // 恢复未旋转坐标系
    ctx.restore();

    // 边框（不随车旋转）
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    ctx.strokeRect(0.5, 0.5, W - 1, H - 1);
  }

  function show() { _visible = true; pip.style.display = ''; }
  function hide() { _visible = false; pip.style.display = 'none'; }
  function toggle() { _visible ? hide() : show(); }
  function setRange(r) { _range = r; }
  function isVisible() { return _visible; }

  return { draw, show, hide, toggle, setRange, isVisible };
}
