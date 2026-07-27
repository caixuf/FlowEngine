import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/**
 * CameraRig.js — 相机控制器
 * 支持 chase / top / driver / front / map / orbit 六种模式
 * D-2: orbit 模式改用 OrbitControls，跟车模式保持手动计算
 */

/* 流畅专题：复用单个 Box3，替代每帧 new THREE.Box3().setFromObject()。
 * roadGroup 在 roadHash 变化时才重建，setFromObject 每帧重新算只是为
 * clamp ego 的 x 边界，没必要每帧分配新对象。 */
const _roadBBox = new THREE.Box3();

export function createCameraRig(canvas) {
  const camera = new THREE.PerspectiveCamera(
    55,                                    // FOV
    (canvas.clientWidth || 1) / (canvas.clientHeight || 1),  // aspect
    0.5,                                   // near
    2000                                   // far
  );

  let mode = 'chase';

  // D-2: OrbitControls — 初始 disabled，仅 orbit 模式启用
  const orbitControls = new OrbitControls(camera, canvas);
  orbitControls.enabled = false;
  orbitControls.target.set(0, 0, 0);
  orbitControls.update();

  function update(ego, roadGroup, now) {
    let ex = ego ? ego.x : 0;
    const ez = ego ? -(ego.y) : 0;
    const eh = ego ? ego.heading || 0 : 0;
    const eg = ego ? ego.z || 0 : 0;

    // D-2: 每帧更新 orbitControls.target 跟随 ego
    orbitControls.target.set(ex, eg, ez);

    // 流畅专题：原先这里每帧 const c = getCenter(roadGroup) 但 c 在所有
    // switch 分支里都被各自的 const c 覆盖，属于死代码 + 白算一次 Box3。
    // map/orbit 分支需要时各自调 getCenter（已走 SceneStore WeakMap 缓存）。
    let hasBBox = false;
    if (roadGroup && roadGroup.children && roadGroup.children.length > 0) {
      _roadBBox.setFromObject(roadGroup);
      hasBBox = isFinite(_roadBBox.min.x) && isFinite(_roadBBox.max.x);
    }
    if (hasBBox) {
      const padding = 500;
      const minX = _roadBBox.min.x - padding;
      const maxX = _roadBBox.max.x + padding;
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
        camera.position.set(ex, eg + 80, ez);
        camera.lookAt(ex, eg, ez);
        break;
      }
      case 'orbit': {
        // D-2: OrbitControls 自动处理，无需手动设置
        orbitControls.update();
        break;
      }
    }
  }

  function setMode(m) {
    if (['chase', 'top', 'driver', 'front', 'map', 'orbit'].includes(m)) {
      mode = m;
      // D-2: orbit 模式启用 OrbitControls，其他模式禁用
      orbitControls.enabled = (mode === 'orbit');
    }
  }

  function reset(roadGroup) {
    orbitControls.target.set(0, 0, 0);
    orbitControls.update();

    if (mode === 'chase' || mode === 'top' || mode === 'driver' || mode === 'front' || mode === 'map') {
      // B3 fix: 不再用路包围盒中心（10km 路→x=5000，车在 x≈50 时看空路），
      // 改为对准原点——车初始位置在原点附近，reset 后能看到车。
      camera.position.set(-10, 10, 0);
      camera.lookAt(0, 0, 0);
    }
  }

  return { camera, update, setMode, reset };
}