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
    let meshCount = 0;
    const sampleTypes = {};
    scene.traverse((obj) => {
      if (!obj.isMesh || !obj.material) return;
      meshCount++;
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      for (const m of mats) {
        const ct = typeof m.color;
        const et = typeof m.emissive;
        sampleTypes[ct + '/' + et] = (sampleTypes[ct + '/' + et] || 0) + 1;
        if (!hit && (m.color === badValue || m.emissive === badValue)) {
          const chain = [];
          let p = obj;
          while (p) { chain.push(p.name || p.type); p = p.parent; }
          hit = {
            prop: m.color === badValue ? 'color' : 'emissive',
            meshName: obj.name || '(unnamed)',
            meshType: obj.type,
            materialType: m.type,
            materialUuid: m.uuid,
            parentChain: chain.join(' < '),
          };
        }
      }
    });
    return { hit, meshCount, sampleTypes };
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
        if (window.__found.length < 3) {
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
