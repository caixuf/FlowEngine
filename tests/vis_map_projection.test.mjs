import assert from 'node:assert/strict';
import {
  drawRoadNetwork2D,
  edgeNodesENU,
  worldToEgoCanvas,
} from '../tools/flowboard/js/vis/math/MapProjection.js';

const edge = {
  nodes: [[0, 0, 0], [50, 10, 0], [100, 30, 0]],
};
assert.deepEqual(edgeNodesENU(edge), [[0, 0], [50, 10], [100, 30]]);

const east = worldToEgoCanvas(10, 0, 0, 0, 0, 2, 100, 100);
assert.deepEqual(east, [100, 80], 'east-facing ego sees east point straight ahead');

const west = worldToEgoCanvas(-10, 0, 0, 0, Math.PI, 2, 100, 100);
assert.ok(Math.abs(west[0] - 100) < 1e-9);
assert.ok(Math.abs(west[1] - 80) < 1e-9,
  'returning ego sees decreasing world x straight ahead');

const north = worldToEgoCanvas(0, 10, 0, 0, Math.PI / 2, 2, 100, 100);
assert.ok(Math.abs(north[0] - 100) < 1e-9);
assert.ok(Math.abs(north[1] - 80) < 1e-9,
  'curved-road heading keeps local forward at screen top');

const widths = [];
const ctx = {
  save() {},
  restore() {},
  beginPath() {},
  moveTo() {},
  lineTo() {},
  stroke() { widths.push(this.lineWidth); },
  setLineDash() {},
  set strokeStyle(_value) {},
  set lineJoin(_value) {},
  set lineCap(_value) {},
  lineWidth: 0,
};
drawRoadNetwork2D(ctx, {
  edges: [{ id: 1, lanes: 4, lane_width: 3.5, oneway: false,
    nodes: [[0, 0], [100, 0]] }],
}, (x, y) => [x, y], { pxPerMeter: 8 });
assert.ok(widths.includes(112), '14m road renders as 112px at 8px/m');

console.log('vis map projection: PASS');
