// Quick smoke test: ensure all modules load (parse + import chain).
// This is NOT a full unit test — browser env stubs are minimal.
global.window = global;
global.document = {
  getElementById: () => null,
  querySelector: () => null,
  querySelectorAll: () => [],
  addEventListener: () => {},
  createElement: () => ({ style: {}, classList: { add: () => {} } }),
  getContext: () => ({
    set fillStyle(v) {}, set strokeStyle(v) {}, set lineWidth(v) {},
    set font(v) {}, set textAlign(v) {}, set textBaseline(v) {},
    set globalAlpha(v) {},
    clearRect: () => {}, fillRect: () => {}, fillText: () => {},
    beginPath: () => {}, moveTo: () => {}, lineTo: () => {},
    closePath: () => {}, fill: () => {}, stroke: () => {},
    save: () => {}, restore: () => {}, translate: () => {}, scale: () => {},
    setLineDash: () => {}, arc: () => {}
  })
};
global.localStorage = { getItem: () => null, setItem: () => {} };
global.requestAnimationFrame = (cb) => 0;
global.performance = { now: () => 0 };
global.EventSource = class { close(){} };

import('./app.js').then(
  () => console.log('OK'),
  (e) => console.error('FAIL:', e.message)
);
