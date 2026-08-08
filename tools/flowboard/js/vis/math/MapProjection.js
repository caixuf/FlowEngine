import { forwardENU } from './Coord.js';

export function edgeNodesENU(edge) {
  if (!edge || !Array.isArray(edge.nodes)) return [];
  return edge.nodes
    .filter((p) => Array.isArray(p) && p.length >= 2 &&
      Number.isFinite(Number(p[0])) && Number.isFinite(Number(p[1])))
    .map((p) => [Number(p[0]), Number(p[1])]);
}

export function worldToEgoCanvas(x, y, egoX, egoY, heading, scale, originX, originY) {
  const dx = x - egoX;
  const dy = y - egoY;
  const [c, s] = forwardENU(heading);
  const forward = dx * c + dy * s;
  const left = -dx * s + dy * c;
  return [originX - left * scale, originY - forward * scale];
}

export function drawRoadNetwork2D(ctx, roadNetwork, project, options = {}) {
  if (!roadNetwork || !Array.isArray(roadNetwork.edges)) return;
  const asphalt = options.asphalt || '#151b26';
  const edgeColor = options.edgeColor || '#8a94a6';
  const dividerColor = options.dividerColor || '#d6d9df';
  const centerColor = options.centerColor || '#f2c94c';
  const pxPerMeter = Number(options.pxPerMeter) || 1;

  for (const edge of roadNetwork.edges) {
    const points = edgeNodesENU(edge);
    if (points.length < 2) continue;
    const laneWidth = Number(edge.lane_width) || 3.5;
    const lanes = Math.max(1, Number(edge.lanes) || 2);
    const roadWidth = laneWidth * lanes;

    strokePolyline(ctx, points, project, asphalt, roadWidth * pxPerMeter, []);
    strokePolyline(ctx, points, project, edgeColor,
      (roadWidth + 0.4) * pxPerMeter, []);
    strokePolyline(ctx, points, project, asphalt, roadWidth * pxPerMeter, []);

    if (!edge.oneway && lanes >= 2) {
      strokePolyline(ctx, points, project, centerColor,
        Math.max(1, 0.15 * pxPerMeter), [3 * pxPerMeter, 2 * pxPerMeter]);
    }
    for (let lane = 1; lane < lanes; lane++) {
      if (!edge.oneway && lane === lanes / 2) continue;
      drawOffsetPolyline(ctx, points, project,
        (lane - lanes / 2) * laneWidth, dividerColor,
        Math.max(0.75, 0.1 * pxPerMeter), [3 * pxPerMeter, 4 * pxPerMeter]);
    }
  }
}

export function strokePolyline(ctx, points, project, color, width, dash = []) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.setLineDash(dash);
  ctx.beginPath();
  for (let i = 0; i < points.length; i++) {
    const p = project(points[i][0], points[i][1]);
    if (i === 0) ctx.moveTo(p[0], p[1]);
    else ctx.lineTo(p[0], p[1]);
  }
  ctx.stroke();
  ctx.restore();
}

function drawOffsetPolyline(ctx, points, project, offset, color, width, dash) {
  const shifted = [];
  for (let i = 0; i < points.length; i++) {
    const prev = points[Math.max(0, i - 1)];
    const next = points[Math.min(points.length - 1, i + 1)];
    const dx = next[0] - prev[0];
    const dy = next[1] - prev[1];
    const length = Math.hypot(dx, dy) || 1;
    shifted.push([
      points[i][0] - dy / length * offset,
      points[i][1] + dx / length * offset,
    ]);
  }
  strokePolyline(ctx, shifted, project, color, width, dash);
}
