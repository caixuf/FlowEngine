/**
 * CameraRig.js — 相机控制器
 * 支持 chase / top / driver / front / map / orbit 六种模式
 */

import { getCenter } from '../store/SceneStore.js';

export function createCameraRig(canvas) {
  const camera = new THREE.PerspectiveCamera(
    55,                                    // FOV
    (canvas.clientWidth || 1) / (canvas.clientHeight || 1),  // aspect
    0.5,                                   // near
    2000                                   // far
  );

  let mode = 'chase';

  // Orbit 状态
  let orbitYaw = 0, orbitPitch = Math.PI / 4, orbitDist = 80;
  let orbitDragging = false, orbitPrevX = 0, orbitPrevY = 0;

  // 拖拽事件
  canvas.addEventListener('mousedown', (e) => {
    if (mode === 'orbit') {
      orbitDragging = true;
      orbitPrevX = e.clientX;
      orbitPrevY = e.clientY;
    }
  });
  window.addEventListener('mouseup', () => { orbitDragging = false; });
  window.addEventListener('mousemove', (e) => {
    if (!orbitDragging) return;
    const dx = e.clientX - orbitPrevX;
    const dy = e.clientY - orbitPrevY;
    orbitYaw -= dx * 0.005;
    orbitPitch = Math.max(0.1, Math.min(Math.PI / 2 - 0.1, orbitPitch - dy * 0.005));
    orbitPrevX = e.clientX;
    orbitPrevY = e.clientY;
  });
  canvas.addEventListener('wheel', (e) => {
    if (mode === 'orbit' || mode === 'map') {
      orbitDist = Math.max(20, Math.min(500, orbitDist + e.deltaY * 0.1));
    }
  });

  function update(ego, roadGroup, now) {
    let ex = ego ? ego.x : 0;
    const ez = ego ? ego.y : 0;
    const eh = ego ? ego.heading || 0 : 0;
    const eg = ego ? ego.z || 0 : 0;

    const c = getCenter(roadGroup);
    let roadBBox = null;
    if (roadGroup && roadGroup.children && roadGroup.children.length > 0) {
      roadBBox = new THREE.Box3().setFromObject(roadGroup);
    }
    if (roadBBox && isFinite(roadBBox.min.x) && isFinite(roadBBox.max.x)) {
      const padding = 500;
      const minX = roadBBox.min.x - padding;
      const maxX = roadBBox.max.x + padding;
      if (ex < minX) ex = minX;
      else if (ex > maxX) ex = maxX;
    }

    switch (mode) {
      case 'chase': {
        const behind = 10, height = 3.5;
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
    if (['chase', 'top', 'driver', 'front', 'map', 'orbit'].includes(m)) {
      mode = m;
    }
  }

  function reset(roadGroup) {
    const c = getCenter(roadGroup);
    orbitYaw = 0;
    orbitPitch = Math.PI / 4;
    orbitDist = 80;

    if (mode === 'chase' || mode === 'top' || mode === 'driver' || mode === 'front' || mode === 'map') {
      camera.position.set(c.x - 10, 10, c.z);
      camera.lookAt(c.x, 0, c.z);
    }
  }

  return { camera, update, setMode, reset };
}
