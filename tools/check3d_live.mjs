import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

// Stringify Error objects (including stack) before Puppeteer's console
// bridge collapses them to an opaque JSHandle@error.
await page.evaluateOnNewDocument(() => {
  window.__capturedErrors = [];
  const wrap = (orig) => (...args) => {
    const rendered = args.map((a) => (a && a.stack) ? a.stack : a);
    window.__capturedErrors.push(rendered.join(' '));
    return orig.apply(console, args);
  };
  console.warn = wrap(console.warn);
  console.error = wrap(console.error);
});

const consoleMsgs = [];
page.on('console', (msg) => consoleMsgs.push(`[${msg.type()}] ${msg.text()}`));
page.on('pageerror', (err) => consoleMsgs.push(`[pageerror] ${err.message}`));
page.on('requestfailed', (req) => consoleMsgs.push(`[requestfailed] ${req.url()} ${req.failure()?.errorText}`));

await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });

// give it a few seconds of live data + render frames
await new Promise((r) => setTimeout(r, 6000));

const state = await page.evaluate(() => {
  const overlay = document.querySelector('#loading-3d, .loading-3d, [class*="waiting"], [class*="stale"]');
  const canvas = document.querySelector('canvas');
  return {
    title: document.title,
    hasCanvas: !!canvas,
    canvasSize: canvas ? [canvas.width, canvas.height] : null,
    overlayText: overlay ? overlay.textContent : null,
    bodyText3D: (document.body.innerText.match(/waiting for data|no data for|3d.{0,20}(error|fail)/i) || [])[0] || null,
    threeLoaded: typeof window.THREE !== 'undefined',
  };
});

let pixelReport = 'n/a';
try {
  pixelReport = await page.evaluate(() => {
    const canvas = document.querySelector('canvas');
    if (!canvas) return 'no canvas';
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
    if (!gl) return 'no gl context';
    const w = gl.drawingBufferWidth, h = gl.drawingBufferHeight;
    const pixels = new Uint8Array(w * h * 4);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
    let nonBlack = 0;
    for (let i = 0; i < pixels.length; i += 4) {
      if (pixels[i] || pixels[i + 1] || pixels[i + 2]) nonBlack++;
    }
    const total = pixels.length / 4;
    const cx = Math.floor(w / 2), cy = Math.floor(h / 2);
    const ci = (cy * w + cx) * 4;
    return `size=${w}x${h} nonBlack=${nonBlack}/${total} (${(100*nonBlack/total).toFixed(1)}%) centerPixel=[${pixels[ci]},${pixels[ci+1]},${pixels[ci+2]}]`;
  });
} catch (e) {
  pixelReport = 'error: ' + e.message;
}

await page.screenshot({ path: '/tmp/check3d_live.png' });

console.log('=== STATE ===');
console.log(JSON.stringify(state, null, 2));
console.log('=== PIXELS ===');
console.log(pixelReport);
console.log('=== CONSOLE (last 40) ===');
console.log(consoleMsgs.slice(-40).join('\n'));

const captured = await page.evaluate(() => window.__capturedErrors.slice(-10));
console.log('=== CAPTURED ERROR STACKS (last 10, deduped) ===');
console.log([...new Set(captured)].join('\n---\n'));

await browser.close();
