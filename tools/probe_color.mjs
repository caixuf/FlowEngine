import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

await page.evaluateOnNewDocument(() => {
  window.__badWrites = [];
  window.__totalWrites = 0;
  window.__idMap = new WeakMap();
  window.__idCounter = 0;
  function idOf(o) {
    if (!window.__idMap.has(o)) window.__idMap.set(o, ++window.__idCounter);
    return window.__idMap.get(o);
  }

  function isBad(v) {
    return v === undefined || v === null || Number.isNaN(v);
  }

  function describe(a) {
    if (a && typeof a === 'object') {
      return `object id=${idOf(a)} ctor=${a.constructor && a.constructor.name} ownProps=${JSON.stringify(Object.getOwnPropertyNames(a))} r=${a.r} g=${a.g} b=${a.b} isColor=${a.isColor} isVector3=${a.isVector3}`;
    }
    return `${typeof a}:${JSON.stringify(a)}`;
  }

  function install() {
    if (!window.THREE || !window.THREE.Color) {
      requestAnimationFrame(install);
      return;
    }
    const proto = window.THREE.Color.prototype;

    const wrap = (name) => {
      const orig = proto[name];
      if (!orig) return;
      proto[name] = function (...args) {
        const ret = orig.apply(this, args);
        window.__totalWrites++;
        if (isBad(this.r) || isBad(this.g) || isBad(this.b)) {
          if (window.__badWrites.length < 8) {
            window.__badWrites.push({
              method: name,
              thisId: idOf(this),
              args: args.map(describe),
              resultRGB: [this.r, this.g, this.b],
              stack: new Error().stack,
            });
          }
        }
        return ret;
      };
    };
    ['copy', 'set', 'setRGB', 'setHex', 'setHSL', 'setStyle', 'lerp', 'lerpColors', 'lerpHSL'].forEach(wrap);
    window.__colorHooksInstalled = true;
  }
  install();
});

page.on('console', () => {});
await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });
await new Promise((r) => setTimeout(r, 3000));

const result = await page.evaluate(() => ({
  installed: window.__colorHooksInstalled,
  total: window.__totalWrites,
  bad: window.__badWrites,
}));

console.log('hooks installed:', result.installed, 'total Color writes observed:', result.total);
console.log('bad writes found:', result.bad.length);
for (const b of result.bad) {
  console.log('=== bad write ===');
  console.log('method:', b.method);
  console.log('args:', JSON.stringify(b.args));
  console.log('resultRGB:', b.resultRGB);
  console.log('stack:', b.stack);
}

await browser.close();
