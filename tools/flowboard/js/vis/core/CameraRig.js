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

  /* 相机跟随（2026-08 顿挫复盘重写）：
   * 位置**刚性锁定** ego 显示位姿，不做二次平滑。
   * 旧实现对"已被 DeadReckon λ=8 平滑过的 ego"再叠一层 λ=12 指数平滑,
   * 车 mesh 与相机变成两个时间常数不同的低通——SSE 到达抖动/外推回拉
   * 先打到车、再打到相机,相位差全部表现为"车相对画面前后蹿"(顿挫感
   * 的直接来源:人眼盯车时以画面为参照,残余高频全集中在车上)。
   * 刚性锁定后车在 chase 画面里像素级固定,顿挫在光学上不可能出现;
   * 抖动转移到均匀纹理的路面滚动上,人眼不敏感。
   * 变道横移由 DeadReckon 本身平滑(旧注释担心的"路跟着晃"针对的是
   * 平滑前的 5Hz 原始跳变,现已不存在)。
   * heading 保留轻量平滑(λ=12):转向瞬态里相机滞后车头一点,能看清
   * 打轮动作,且避免朝向噪声直接晃动整个画面。 */
  let _camSH = 0, _camInit = false;
  let _camLastT = 0;

  function update(ego, roadGroup, now) {
    let ex = ego ? ego.x : 0;
    const ez = ego ? -(ego.y) : 0;
    const ehRaw = ego ? ego.heading || 0 : 0;
    const eg = ego ? ego.z || 0 : 0;

    /* 帧间 dt（now 单位与渲染一致）；首帧 snap 到真值防漂移 */
    const tSec = (now != null && now > 0) ? now : 0;
    const dt = _camLastT > 0 ? Math.min(0.1, Math.max(0.001, tSec - _camLastT)) : 0.016;
    _camLastT = tSec;
    if (!_camInit) { _camSH = ehRaw; _camInit = true; }
    const alpha = 1 - Math.exp(-12 * dt);
    /* heading 最短角插值 */
    let dh = ehRaw - _camSH;
    while (dh > Math.PI) dh -= 2 * Math.PI;
    while (dh < -Math.PI) dh += 2 * Math.PI;
    _camSH += dh * alpha;
    const sEH = _camSH;

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

    const eh = sEH;
    switch (mode) {
      case 'chase': {
        const behind = 10, height = 3.5;
        camera.position.set(
          ex - Math.cos(eh) * behind,
          eg + height,
          ez - Math.sin(eh) * behind
        );
        /* lookAt 前向偏移 5m→2m：原来相机看车前 5m 处，车辆出现在画面
         * 下方偏后；改为 2m 让车辆本身更接近视野中心。
         * 高度 1→0.8：对应 fallback 车身重心（body.y=0.65，整车~0.7m）*/
        camera.lookAt(ex + Math.cos(eh) * 2, eg + 0.8, ez + Math.sin(eh) * 2);
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