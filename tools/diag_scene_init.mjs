import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });
const bad = [];
page.on('response', (res) => {
  if (res.status() >= 400) bad.push(res.status() + ' ' + res.url());
});
const logs = [];
page.on('console', (msg) => logs.push('[' + msg.type() + '] ' + msg.text()));
page.on('pageerror', (err) => logs.push('[pageerror] ' + err.message + '\n' + (err.stack || '')));
await page.goto('http://localhost:8800', { waitUntil: 'domcontentloaded', timeout: 20000 });
await new Promise((r) => setTimeout(r, 15000));
console.log('=== bad responses ===');
console.log(bad.length ? bad.join('\n') : '(none)');
console.log('=== console/page errors (non-warning) ===');
console.log(logs.filter(l => !l.startsWith('[warning]')).join('\n') || '(none)');
const state = await page.evaluate(async () => {
  const mod = await import('/tools/flowboard/js/scene3d.js');
  return { sceneReady: mod.sceneReady, hasScene3d: !!mod.scene3d };
});
console.log('STATE:', JSON.stringify(state));
await browser.close();
