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

await page.goto('http://localhost:8800', { waitUntil: 'domcontentloaded', timeout: 20000 });
await new Promise((r) => setTimeout(r, 10000));

// 1) canvas non-black check (black-screen regression)
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
console.log('=== 1) canvas pixel check ===');
console.log(JSON.stringify(canvasResult));

// 2) material audit
const auditResult = await page.evaluate(() => {
  if (!window.flowboard || !window.flowboard._auditMaterials) return { error: '_auditMaterials not exposed' };
  return window.flowboard._auditMaterials();
});
console.log('=== 2) _auditMaterials() ===');
console.log(JSON.stringify(auditResult));

// 3) scene graph sample: wheel pivot + obstacle scale + ego tilt, via the
// live scene3d module singleton (already imported by app.js — dynamic
// import returns the SAME instance, not a fresh reload).
async function sampleScene() {
  return await page.evaluate(async () => {
    const mod = await import('/tools/flowboard/js/scene3d.js');
    const scene = mod.scene3d;
    let ego = null;
    const pedestrians = [], trucks = [], cyclists = [];
    scene.traverse((obj) => {
      if (!ego && obj.userData && obj.userData.frontWheels) ego = obj;
      if (obj.userData && obj.visible) {
        if (obj.userData.obsType === 'pedestrian') pedestrians.push(obj);
        if (obj.userData.obsType === 'truck') trucks.push(obj);
        if (obj.userData.obsType === 'cyclist') cyclists.push(obj);
      }
    });
    const result = {};
    if (ego) {
      const fw = ego.userData.frontWheels;
      result.frontWheelPivotLocal = fw ? { x: fw.position.x, y: fw.position.y, z: fw.position.z } : null;
      result.egoRotationXZ = { x: ego.rotation.x, z: ego.rotation.z };
    } else {
      result.egoNotFound = true;
    }
    result.pedestrianCount = pedestrians.length;
    if (pedestrians.length) {
      const p = pedestrians[0];
      result.pedestrianScale = { x: p.scale.x, y: p.scale.y, z: p.scale.z };
      result.pedestrianRotationXZ = { x: p.rotation.x, z: p.rotation.z };
    }
    result.truckCount = trucks.length;
    if (trucks.length) {
      const t = trucks[0];
      result.truckScale = { x: t.scale.x, y: t.scale.y, z: t.scale.z };
    }
    result.cyclistCount = cyclists.length;
    if (cyclists.length) {
      const c = cyclists[0];
      result.cyclistScale = { x: c.scale.x, y: c.scale.y, z: c.scale.z };
    }
    return result;
  });
}

const sample1 = await sampleScene();
console.log('=== 3) scene sample #1 ===');
console.log(JSON.stringify(sample1, null, 2));

await new Promise((r) => setTimeout(r, 3000));

const sample2 = await sampleScene();
console.log('=== 3) scene sample #2 (+3s) ===');
console.log(JSON.stringify(sample2, null, 2));

console.log('=== console errors during load+13s ===');
console.log(consoleErrors.length ? consoleErrors.slice(0, 15).join('\n') : '(none)');

await browser.close();
