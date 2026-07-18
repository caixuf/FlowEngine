import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

const consoleErrors = [];
page.on('console', (msg) => {
  if (msg.type() === 'error') consoleErrors.push(msg.text());
});
page.on('pageerror', (err) => consoleErrors.push('pageerror: ' + err.message));

await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });
await new Promise((r) => setTimeout(r, 8000));

// 1) canvas non-black check
const canvasResult = await page.evaluate(() => {
  const canvas = document.querySelector('#scene3d canvas, canvas');
  if (!canvas) return { error: 'no canvas found' };
  const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
  const w = canvas.width, h = canvas.height;
  const px = new Uint8Array(w * h * 4);
  gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, px);
  let nonBlack = 0, total = 0;
  for (let i = 0; i < px.length; i += 4) {
    total++;
    if (px[i] || px[i + 1] || px[i + 2]) nonBlack++;
  }
  return { nonBlack, total, w, h };
});
console.log('=== canvas pixel check ===');
console.log(JSON.stringify(canvasResult));

// 2) audit function: should be clean now (bug fixed)
const auditResult = await page.evaluate(() => {
  if (!window.flowboard || !window.flowboard._auditMaterials) return { error: '_auditMaterials not exposed' };
  return window.flowboard._auditMaterials();
});
console.log('=== _auditMaterials() on healthy scene ===');
console.log('findings:', JSON.stringify(auditResult));

// 3) isolated unit test: reproduce the exact historical bug (sheen: 0.4 raw number on
// MeshPhysicalMaterial) against a synthetic scene and confirm _auditSceneMaterials catches it,
// plus confirm a normal Color-valued sheen is NOT flagged (no false positives).
const unitTestResult = await page.evaluate(async () => {
  const mod = await import('/tools/flowboard/js/utils.js');
  const THREE = window.THREE;
  const scene = new THREE.Scene();

  const goodMat = new THREE.MeshPhysicalMaterial({ color: 0xff0000, sheen: new THREE.Color(0.4, 0.4, 0.4) });
  const goodMesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), goodMat);
  goodMesh.name = 'good-fixed-sheen';
  scene.add(goodMesh);

  const badMat = new THREE.MeshPhysicalMaterial({ color: 0x00ff00 });
  badMat.sheen = 0.4; // exact historical bug pattern
  const badMesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), badMat);
  badMesh.name = 'bad-raw-number-sheen';
  scene.add(badMesh);

  return mod._auditSceneMaterials(scene);
});
console.log('=== unit test: _auditSceneMaterials against synthetic good+bad scene ===');
console.log(JSON.stringify(unitTestResult, null, 2));

console.log('=== console errors during load ===');
console.log(consoleErrors.length ? consoleErrors.slice(0, 10).join('\n') : '(none)');

await browser.close();
