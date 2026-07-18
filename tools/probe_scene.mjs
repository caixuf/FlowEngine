import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

await page.evaluateOnNewDocument(() => {
  window.__renderCallCount = 0;
  function install() {
    if (!window.THREE || !window.THREE.WebGLRenderer) {
      setTimeout(install, 10);
      return;
    }
    const OrigRenderer = window.THREE.WebGLRenderer;
    function WrappedRenderer(...args) {
      const instance = new OrigRenderer(...args);
      const origRender = instance.render.bind(instance);
      instance.render = function (scene, camera) {
        window.__renderCallCount++;
        window.__lastScene = scene;
        return origRender(scene, camera);
      };
      return instance;
    }
    WrappedRenderer.prototype = OrigRenderer.prototype;
    window.THREE.WebGLRenderer = WrappedRenderer;
    window.__renderHookInstalled = true;
  }
  install();
});

page.on('console', () => {});
await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });
await new Promise((r) => setTimeout(r, 6000));

const result = await page.evaluate(() => {
  const scene = window.__lastScene;
  if (!scene) return { error: 'no scene captured', installed: window.__renderHookInstalled, calls: window.__renderCallCount };
  const fogInfo = scene.fog ? {
    type: scene.fog.type,
    colorType: typeof scene.fog.color,
    colorIsColor: scene.fog.color && scene.fog.color.isColor,
    colorValue: scene.fog.color && typeof scene.fog.color === 'object' ? JSON.stringify(scene.fog.color) : scene.fog.color,
    near: scene.fog.near,
    far: scene.fog.far,
  } : 'no fog';
  const bgInfo = scene.background ? {
    type: typeof scene.background,
    isColor: scene.background.isColor,
    value: typeof scene.background === 'object' ? JSON.stringify(scene.background) : scene.background,
  } : 'no background';
  const fogTrueMats = [];
  scene.traverse((obj) => {
    if (!obj.isMesh || !obj.material) return;
    const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    for (const m of mats) {
      if (m.fog) fogTrueMats.push({ name: obj.name || '(unnamed)', materialType: m.type, uuid: m.uuid });
    }
  });
  const COLOR_PROPS = ['color', 'emissive', 'specular', 'sheenColor', 'attenuationColor'];
  const bad = [];
  let meshCount = 0, materialCount = 0;
  scene.traverse((obj) => {
    if (!obj.isMesh || !obj.material) return;
    meshCount++;
    const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    for (const m of mats) {
      materialCount++;
      for (const prop of COLOR_PROPS) {
        const v = m[prop];
        if (v === undefined) continue;
        const looksLikeColor = v && typeof v === 'object' && v.isColor;
        if (!looksLikeColor) {
          bad.push({
            meshName: obj.name || '(unnamed)',
            meshType: obj.type,
            materialType: m.type,
            materialUuid: m.uuid,
            prop,
            valueType: typeof v,
            value: typeof v === 'object' ? JSON.stringify(v) : v,
            parentChain: (() => {
              const chain = [];
              let p = obj;
              while (p) { chain.push(p.name || p.type); p = p.parent; }
              return chain.join(' < ');
            })(),
          });
        }
      }
    }
  });
  return { meshCount, materialCount, bad, fogInfo, bgInfo, fogTrueMatsCount: fogTrueMats.length, fogTrueMats: fogTrueMats.slice(0, 5) };
});

console.log(JSON.stringify(result, null, 2));

await browser.close();
