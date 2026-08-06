/**
 * showcase/main.js — 综合 3D 场景展示控制器
 *
 * 复用现有 vis 3D 渲染架构（tools/flowboard/js/vis/main.js），把
 * scenarios/*.json 逐个转成一帧 scene 快照喂给 SceneDirector，实现
 * 「一个页面浏览全部场景的 3D」。不启动任何 C 管道，纯前端静态预览。
 *
 * 数据链路：
 *   showcase/scenes.json (build_showcase.py 生成)
 *     → sceneAdapter.scenarioToTopoData(raw)
 *       → update3D({metrics:{scene}})
 *         → SceneDirector 渲染
 */
import {
  init3DScene,
  update3D,
  resize3D,
  resetCamera,
  setCameraMode,
} from '/tools/flowboard/js/vis/main.js';
import { scenarioToTopoData } from '/tools/flowboard/js/showcase/sceneAdapter.js';

const SCENES_URL = '/tools/flowboard/showcase/scenes.json';

const els = {
  list: document.getElementById('scene-list'),
  title: document.getElementById('scene-title'),
  desc: document.getElementById('scene-desc'),
  meta: document.getElementById('scene-meta'),
  msg: document.getElementById('showcase-msg'),
  canvas: document.getElementById('scene3d-canvas'),
};

let scenarios = [];
let activeIndex = -1;

function setMsg(text) {
  if (els.msg) els.msg.textContent = text || '';
  if (els.msg) els.msg.style.display = text ? 'block' : 'none';
}

/** 选中并渲染第 i 个场景。 */
function selectScene(i) {
  if (i < 0 || i >= scenarios.length) return;
  activeIndex = i;
  const s = scenarios[i];

  // 高亮列表项
  const items = els.list ? els.list.querySelectorAll('.scene-item') : [];
  items.forEach((el, k) => el.classList.toggle('active', k === i));

  // 顶部信息
  if (els.title) els.title.textContent = s.name || s.file;
  if (els.desc) els.desc.textContent = s.description || '';
  const rn = (s.raw && s.raw.road_network && s.raw.road_network.edges) || [];
  const actors = (s.raw && s.raw.actors) || [];
  const tls = (s.raw && s.raw.traffic_lights) || [];
  if (els.meta) {
    els.meta.textContent =
      `道路段 ${rn.length} · 车辆/行人 ${actors.length} · 红绿灯 ${tls.length}` +
      (s.duration_s ? ` · 时长 ${s.duration_s}s` : '');
  }

  try {
    update3D(scenarioToTopoData(s.raw));
    // 场景切换后把相机重置到新路网中心/ego。
    resetCamera();
    setMsg('');
  } catch (err) {
    console.error('[showcase] 渲染场景失败:', err);
    setMsg('渲染失败: ' + err.message);
  }
}

/** 构建左侧场景列表。 */
function buildList() {
  if (!els.list) return;
  els.list.innerHTML = '';
  scenarios.forEach((s, i) => {
    const item = document.createElement('button');
    item.className = 'scene-item' + (s.enabled === false ? ' disabled-scene' : '');
    item.type = 'button';
    const idx = document.createElement('span');
    idx.className = 'scene-idx';
    idx.textContent = String(i + 1).padStart(2, '0');
    const label = document.createElement('span');
    label.className = 'scene-name';
    label.textContent = s.name || s.file;
    item.appendChild(idx);
    item.appendChild(label);
    item.addEventListener('click', () => selectScene(i));
    els.list.appendChild(item);
  });
}

function wireControls() {
  document.querySelectorAll('[data-cam]').forEach((btn) => {
    btn.addEventListener('click', () => {
      setCameraMode(btn.getAttribute('data-cam'));
      document.querySelectorAll('[data-cam]').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
    });
  });
  window.addEventListener('resize', () => resize3D());
}

async function boot() {
  setMsg('加载场景清单…');
  try {
    init3DScene(els.canvas);
    resize3D();
  } catch (err) {
    console.error('[showcase] 3D 初始化失败:', err);
    setMsg('3D 初始化失败: ' + err.message);
    return;
  }

  wireControls();

  try {
    const resp = await fetch(SCENES_URL, { cache: 'no-cache' });
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const manifest = await resp.json();
    scenarios = Array.isArray(manifest.scenarios) ? manifest.scenarios : [];
  } catch (err) {
    console.error('[showcase] 加载 scenes.json 失败:', err);
    setMsg('加载场景清单失败（请先运行 python3 tools/build_showcase.py）: ' + err.message);
    return;
  }

  if (scenarios.length === 0) {
    setMsg('没有可展示的场景');
    return;
  }

  buildList();
  selectScene(0);
}

boot();
