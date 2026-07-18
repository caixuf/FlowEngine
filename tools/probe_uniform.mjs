import puppeteer from 'puppeteer';

const browser = await puppeteer.launch({
  headless: 'new',
  args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--no-sandbox'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1600, height: 1000 });

await page.evaluateOnNewDocument(() => {
  window.__probe = [];
  window.__idMap = new WeakMap();
  window.__idCounter = 0;
  function idOf(o) {
    if (!window.__idMap.has(o)) window.__idMap.set(o, ++window.__idCounter);
    return window.__idMap.get(o);
  }
  const proto = WebGL2RenderingContext.prototype;
  const orig3fv = proto.uniform3fv;
  let calls = 0;
  proto.uniform3fv = function (loc, v, ...rest) {
    calls++;
    let bad = false;
    let detail = '';
    try {
      const isArr = Array.isArray(v) || ArrayBuffer.isView(v);
      const len = v ? v.length : 'n/a';
      if (!isArr || !v || (len !== 3 && len % 3 !== 0)) {
        bad = true;
      }
      if (!isArr && v && typeof v === 'object') {
        detail = `id=${idOf(v)} ctor=${v.constructor && v.constructor.name} ownProps=${JSON.stringify(Object.getOwnPropertyNames(v))} r=${v.r} g=${v.g} b=${v.b} isColor=${v.isColor} isVector3=${v.isVector3} x=${v.x}`;
      } else {
        detail = `type=${Object.prototype.toString.call(v)} isArrayLike=${isArr} len=${len} vals=${isArr ? Array.from(v).slice(0, 9) : String(v)}`;
      }
    } catch (e) {
      bad = true;
      detail = 'introspect error: ' + e.message;
    }
    if (bad || calls <= 3) {
      window.__probe.push({ call: calls, bad, detail });
    }
    try {
      return orig3fv.call(this, loc, v, ...rest);
    } catch (e) {
      window.__probe.push({ call: calls, threw: e.message, detail });
      throw e;
    }
  };
});

page.on('console', () => {});
await page.goto('http://localhost:8800', { waitUntil: 'networkidle2', timeout: 20000 });
await new Promise((r) => setTimeout(r, 3000));

const probe = await page.evaluate(() => window.__probe);
const bads = probe.filter((p) => p.bad || p.threw);
console.log('total captured entries:', probe.length, 'bad/threw:', bads.length);
console.log('=== first 5 calls (baseline) ===');
for (const p of probe.slice(0, 5)) {
  console.log(`call#${p.call} ${p.detail}`);
}
console.log('=== bad/threw entries (first 3) ===');
for (const p of bads.slice(0, 3)) {
  console.log('---');
  console.log('call#', p.call, 'threw:', p.threw, 'detail:', p.detail);
  console.log(p.stack);
}

await browser.close();
