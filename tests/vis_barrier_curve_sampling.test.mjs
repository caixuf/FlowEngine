import fs from 'node:fs';
import { edgeSampleCount, sampleEdgeNodes } from '../tools/flowboard/js/vis/math/Curve.js';
import { ok, done } from './test-utils.mjs';

console.log('=== 长弯道护栏采样 ===\n');

const scenario = JSON.parse(fs.readFileSync(
  new URL('../scenarios/straight_road.json', import.meta.url), 'utf8'));
const nodes = scenario.road_network.edges[0].nodes;
const count = edgeSampleCount(nodes);
const sampled = sampleEdgeNodes(nodes);

ok('3000m S 弯使用自适应高密度采样', count >= 900);

let maxChord = 0;
for (let i = 3; i < sampled.length; i += 3) {
  maxChord = Math.max(maxChord, Math.hypot(
    sampled[i] - sampled[i - 3],
    sampled[i + 1] - sampled[i - 2],
    sampled[i + 2] - sampled[i - 1]
  ));
}
ok('相邻护栏横梁跨度不超过 5m', maxChord <= 5.0);

done();
