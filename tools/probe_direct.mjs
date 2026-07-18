import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

await page.evaluateOnNewDocument(() => {
  window.__found = [];

  function isBad(v) {
    return v === undefined || v === null || Number.isNaN(v);
  }

  function checkColor(v) {
    if (v === undefined) return null;
    if (v && typeof v === 'object' && v.isColor === true) {
      if (isBad(v.r) || isBad(v.g) || isBad(v.b)) return 'Color-shaped but bad r/g/b';
      return null;
    }
    return `not a Color (typeof=${typeof v}, value=${typeof v === 'object' ? JSON.stringify(v) : v})`;
  }

  function installRendererHook() {
    if (!window.THREE || !window.THREE.WebGLRenderer) {
      setTimeout(installRendererHook, 10);
      return;
    }
    const OrigRenderer = window.THREE.WebGLRenderer;
    function WrappedRenderer(...args) {
      const instance = new OrigRenderer(...args);
      const origRenderBufferDirect = instance.renderBufferDirect.bind(instance);
      instance.renderBufferDirect = function (camera, scene, geometry, material, object, group) {
        if (window.__found.length < 5) {
          const mats = Array.isArray(material) ? material : [material];
          for (const m of mats) {
            if (!m) continue;
            const problems = [];
            const cProb = checkColor(m.color);
            const eProb = checkColor(m.emissive);
            const sProb = checkColor(m.specular);
            if (cProb) problems.push(`color: ${cProb}`);
            if (eProb) problems.push(`emissive: ${eProb}`);
            if (sProb) problems.push(`specular: ${sProb}`);
            if (problems.length) {
              const chain = [];
              let p = object;
              while (p) { chain.push(p.name || p.type); p = p.parent; }
              window.__found.push({
                problems,
                objectName: object && (object.name || '(unnamed)'),
                objectType: object && object.type,
                objectUuid: object && object.uuid,
                materialType: m.type,
                materialUuid: m.uuid,
                materialName: m.name,
                parentChain: chain.join(' < '),
                colorRaw: typeof m.color === 'object' ? JSON.stringify(m.color) : m.color,
                emissiveRaw: typeof m.emissive === 'object' ? JSON.stringify(m.emissive) : m.emissive,
              });
            }
          }
        }
        return origRenderBufferDirect(camera, scene, geometry, material, object, group);
      };
      return instance;
    }
    WrappedRenderer.prototype = OrigRenderer.prototype;
    window.THREE.WebGLRenderer = WrappedRenderer;
  }

  installRendererHook();
});

page.on('console', () => {});
await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });
await new Promise((r) => setTimeout(r, 15000));

const result = await page.evaluate(() => {
  const canvas = document.querySelector('#scene3d canvas, canvas');
  let nonBlack = 0, total = 0;
  if (canvas) {
    const gl = canvas.getContext('webgl2') || canvas.getContext('webgl');
    try {
      const w = canvas.width, h = canvas.height;
      const px = new Uint8Array(w * h * 4);
      gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, px);
      for (let i = 0; i < px.length; i += 4) {
        total++;
        if (px[i] || px[i + 1] || px[i + 2]) nonBlack++;
      }
    } catch (e) { /* ignore */ }
  }
  return {
    found: window.__found,
    perfTier: window.__scene3dDebug ? window.__scene3dDebug.perfTier : undefined,
    nonBlack, total,
  };
});
console.log('bad materials found at renderBufferDirect:', result.found.length);
for (const f of result.found) {
  console.log('===', JSON.stringify(f, null, 2));
}
console.log('canvas nonBlack:', result.nonBlack, '/', result.total);

await browser.close();
