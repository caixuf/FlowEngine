import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

await page.evaluateOnNewDocument(() => {
  window.__lastScene = null;
  window.__found = [];

  function isBad(v) {
    return v === undefined || v === null || Number.isNaN(v);
  }

  function locate(badValue) {
    const scene = window.__lastScene;
    if (!scene) return { error: 'no scene' };
    let hit = null;
    let objCount = 0;
    const typeCounts = {};
    const nonColorSamples = [];
    scene.traverse((obj) => {
      if (!obj.material) return;
      const kind = obj.isMesh ? 'Mesh' : obj.isLine ? 'Line' : obj.isPoints ? 'Points' : obj.isSprite ? 'Sprite' : obj.type;
      typeCounts[kind] = (typeCounts[kind] || 0) + 1;
      objCount++;
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      for (const m of mats) {
        for (const prop of ['color', 'emissive']) {
          const v = m[prop];
          if (v === undefined) continue;
          const isProperColor = v && typeof v === 'object' && v.isColor === true;
          if (!isProperColor && nonColorSamples.length < 10) {
            const chain = [];
            let p = obj;
            while (p) { chain.push(p.name || p.type); p = p.parent; }
            nonColorSamples.push({
              kind, prop, valueType: typeof v,
              value: typeof v === 'object' ? JSON.stringify(v) : v,
              materialType: m.type, materialUuid: m.uuid,
              parentChain: chain.join(' < '),
            });
          }
          if (!hit && v === badValue) {
            const chain = [];
            let p = obj;
            while (p) { chain.push(p.name || p.type); p = p.parent; }
            hit = {
              prop, kind,
              meshName: obj.name || '(unnamed)',
              meshType: obj.type,
              materialType: m.type,
              materialUuid: m.uuid,
              parentChain: chain.join(' < '),
            };
          }
        }
      }
    });
    return { hit, objCount, typeCounts, nonColorSamples };
  }

  function installRendererHook() {
    if (!window.THREE || !window.THREE.WebGLRenderer) {
      setTimeout(installRendererHook, 10);
      return;
    }
    const OrigRenderer = window.THREE.WebGLRenderer;
    function WrappedRenderer(...args) {
      const instance = new OrigRenderer(...args);
      const origRender = instance.render.bind(instance);
      instance.render = function (scene, camera) {
        window.__lastScene = scene;
        return origRender(scene, camera);
      };
      return instance;
    }
    WrappedRenderer.prototype = OrigRenderer.prototype;
    window.THREE.WebGLRenderer = WrappedRenderer;
  }

  function installColorHook() {
    if (!window.THREE || !window.THREE.Color) {
      setTimeout(installColorHook, 10);
      return;
    }
    const proto = window.THREE.Color.prototype;
    const origCopy = proto.copy;
    proto.copy = function (color) {
      const ret = origCopy.call(this, color);
      if (isBad(this.r) || isBad(this.g) || isBad(this.b)) {
        if (window.__found.length < 2) {
          window.__found.push({
            badValue: color,
            located: locate(color),
            stack: new Error().stack,
          });
        }
      }
      return ret;
    };
  }

  installRendererHook();
  installColorHook();
});

page.on('console', () => {});
await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });
await new Promise((r) => setTimeout(r, 5000));

const found = await page.evaluate(() => window.__found);
console.log('bad writes with location info:', found.length);
for (const f of found) {
  console.log('=== bad write ===');
  console.log('badValue:', f.badValue);
  console.log('located:', JSON.stringify(f.located, null, 2));
  console.log('stack:', f.stack);
}

await browser.close();
