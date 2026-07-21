/**
 * CameraRig.js — 6 视角相机系统
 * Chase/Map/Orbit/Top/Driver/Front
 * Map/Orbit 用路网 bbox 中心锁定，不跟随 ego（避免跑出场景后白屏）。
 * Orbit 模式：手动鼠标拖拽旋转（不依赖 OrbitControls CDN）。
 */

const MODES = ['chase', 'map', 'orbit', 'top', 'driver', 'front'];

export function createCameraRig(canvas) {
  /* 用 canvas 实际尺寸初始化 aspect，而非 window.innerWidth/innerHeight，
   * 否则 canvas 在卡片内时（如 800x600），aspect 按全屏算会导致画面拉伸。 */
  const cw = canvas ? canvas.clientWidth : window.innerWidth;
  const ch = canvas ? canvas.clientHeight : window.innerHeight;
  const camera = new THREE.PerspectiveCamera(60, cw / Math.max(ch, 1), 0.5, 20000);
  let mode = 'chase';
  let roadBBox = null;

  let orbitYaw = 0, orbitPitch = 0.6, orbitDist = 200;
  let isDragging = false, lastX = 0, lastY = 0;

  if (canvas) {
    canvas.addEventListener('mousedown', (e) => {
      if (mode !== 'orbit') return;
      isDragging = true; lastX = e.clientX; lastY = e.clientY;
    });
    window.addEventListener('mouseup', () => { isDragging = false; });
    window.addEventListener('mousemove', (e) => {
      if (!isDragging) return;
      orbitYaw -= (e.clientX - lastX) * 0.01;
      orbitPitch = Math.max(0.1, Math.min(1.5, orbitPitch - (e.clientY - lastY) * 0.01));
      lastX = e.clientX; lastY = e.clientY;
    });
    canvas.addEventListener('wheel', (e) => {
      if (mode !== 'orbit') return;
      orbitDist = Math.max(50, Math.min(800, orbitDist + e.deltaY * 0.5));
      e.preventDefault();
    }, { passive: false });
  }

  function getCenter(roadGroup) {
    if (roadGroup && roadGroup.children.length > 0) {
      if (!roadBBox) roadBBox = new THREE.Box3().setFromObject(roadGroup);
      if (roadBBox.min.x !== Infinity) {
        return {
          x: (roadBBox.min.x + roadBBox.max.x) / 2,
          z: (roadBBox.min.z + roadBBox.max.z) / 2
        };
      }
    }
    return { x: 500, z: 0 };
  }

  function update(ego, roadGroup, now) {
    let ex = ego ? ego.x : 0;
    const ez = ego ? ego.y : 0;
    const eh = ego ? ego.heading || 0 : 0;
    const eg = ego ? ego.z || 0 : 0;

    const c = getCenter(roadGroup);
    if (roadBBox) {
      const padding = 500;
      const minX = roadBBox.min.x - padding;
      const maxX = roadBBox.max.x + padding;
      if (ex < minX) ex = minX;
      else if (ex > maxX) ex = maxX;
    }

    switch (mode) {
      case 'chase': {
        const behind = 12, height = 6;
        camera.position.set(
          ex - Math.cos(eh) * behind,
          eg + height,
          ez - Math.sin(eh) * behind
        );
        camera.lookAt(ex + Math.cos(eh) * 5, eg + 1, ez + Math.sin(eh) * 5);
        break;
      }
      case 'top': {
        camera.position.set(ex, eg + 150, ez);
        camera.lookAt(ex, eg, ez);
        break;
      }
      case 'driver': {
        camera.position.set(
          ex + Math.cos(eh) * 1.0, eg + 1.5,
          ez + Math.sin(eh) * 1.0
        );
        camera.lookAt(ex + Math.cos(eh) * 20, eg + 1.4, ez + Math.sin(eh) * 20);
        break;
      }
      case 'front': {
        camera.position.set(
          ex + Math.cos(eh) * 8, eg + 2.0,
          ez + Math.sin(eh) * 8
        );
        camera.lookAt(ex, eg + 1.0, ez);
        break;
      }
      case 'map': {
        const c = getCenter(roadGroup);
        camera.position.set(c.x, eg + 80, c.z);
        camera.lookAt(c.x, eg, c.z);
        break;
      }
      case 'orbit': {
        const c = getCenter(roadGroup);
        const cosP = Math.cos(orbitPitch), sinP = Math.sin(orbitPitch);
        camera.position.set(
          c.x + Math.cos(orbitYaw) * orbitDist * cosP,
          eg + orbitDist * sinP,
          c.z + Math.sin(orbitYaw) * orbitDist * cosP
        );
        camera.lookAt(c.x, eg, c.z);
        break;
      }
    }
  }

  function setMode(m) {
    if (!MODES.includes(m)) return;
    mode = m;
  }

  function getMode() { return mode; }

  function reset(roadGroup) {
    roadBBox = null;
    update(null, roadGroup, 0);
  }

  return { camera, update, setMode, getMode, reset, getCenter };
}