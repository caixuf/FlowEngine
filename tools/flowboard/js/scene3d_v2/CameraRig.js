/**
 * CameraRig.js — v2 相机控制
 *
 * 直接复用 v1 的 CameraController（已写得很好，无需重写）：
 *   - Chase: 跟车后方，按 heading 投影
 *   - Map: 鸟瞰正俯视，高度可调
 *   - Orbit: 自由轨道，鼠标拖拽 + 滚轮
 *
 * 砍掉的模式：Driver（驾驶舱内）、Front（车前低视角）— 用户少用，v2 暂不做。
 * 砍掉的功能：NPC 点击 raycast（v2 无详情面板）。
 *
 * 坐标系：相机位置 (x, height, z)，lookAt (x, 0, z)。
 *   - chase: 跟 ego 后方 6m，高 3m
 *   - map: 高空正俯视，默认 120m（v1 修复后的值），不跟 ego 时锁路网中心
 *   - orbit: target 锁路网中心，距离/方位由用户控制
 */
const THREE = window.THREE;
import * as CameraController from '../scene3d/controller/CameraController.js';

export const VALID_MODES = ['chase', 'map', 'orbit'];

export function setMode(mode) {
  if (VALID_MODES.indexOf(mode) < 0) return;
  CameraController.setCameraMode(mode);
}

export function getMode() {
  return CameraController.getMode();
}

export function resetCamera() {
  CameraController.setCameraMode('chase');
}

export function resetMapView() {
  CameraController.resetMapView();
}

/**
 * 初始化鼠标交互（orbit 拖拽 / map 拖拽缩放）。
 * 砍掉 NPC 点击 raycast。
 */
export function initInteraction(canvas) {
  // CameraController.initCameraControls 的 callbacks 参数可选；
  // 不传 pickNPC 即禁用 NPC 点击检测。
  CameraController.initCameraControls(canvas, {});
}

/**
 * 每帧更新相机位置。
 * @param {THREE.Camera} camera
 * @param {number} egoX  平滑后 ego x
 * @param {number} egoZ  平滑后 ego z
 * @param {number} egoHeading  平滑后 ego heading
 * @param {THREE.Box3|null} roadBBox  路网 bbox（map/orbit 锚定用）
 */
export function updateCamera(camera, egoX, egoZ, egoHeading, roadBBox) {
  var mode = CameraController.getMode();

  if (mode === 'chase') {
    // 跟车后方 6m，高 3m
    var chDx = -Math.cos(egoHeading) * 6;
    var chDz = -Math.sin(egoHeading) * 6;
    camera.position.set(egoX + chDx, 3.0, egoZ + chDz);
    camera.up.set(0, 1, 0);
    camera.lookAt(egoX + Math.cos(egoHeading) * 10, 1.0, egoZ + Math.sin(egoHeading) * 10);
    if (camera.fov !== 60) { camera.fov = 60; camera.updateProjectionMatrix(); }
  } else if (mode === 'map') {
    // 鸟瞰正俯视
    var mapState = CameraController.getMapState();
    var mapH = mapState.height;
    var mapCx, mapCz;
    if (mapState.followEgo) {
      mapCx = egoX; mapCz = egoZ;
    } else {
      // 不跟 ego 时锁路网 bbox 中心（fallback 500, 0）
      var cx = 500, cz = 0;
      if (roadBBox && roadBBox.min.x !== Infinity) {
        cx = (roadBBox.min.x + roadBBox.max.x) / 2;
        cz = (roadBBox.min.z + roadBBox.max.z) / 2;
      }
      mapCx = cx + mapState.offset.x;
      mapCz = cz + mapState.offset.z;
    }
    camera.position.set(mapCx, mapH, mapCz + 0.001);
    camera.up.set(0, 1, 0);
    camera.lookAt(mapCx, 0, mapCz);
    if (camera.fov !== 75) { camera.fov = 75; camera.updateProjectionMatrix(); }
  } else if (mode === 'orbit') {
    // 自由轨道：target 锁路网中心（不跟 ego）
    var orbitCx = egoX, orbitCz = egoZ;
    if (roadBBox && roadBBox.min.x !== Infinity) {
      orbitCx = (roadBBox.min.x + roadBBox.max.x) / 2;
      orbitCz = (roadBBox.min.z + roadBBox.max.z) / 2;
    }
    var orbitState = CameraController.getOrbitState();
    if (orbitState) {
      orbitState.target.set(orbitCx, 0, orbitCz);
      var ox = orbitState.distance * Math.sin(orbitState.polar) * Math.cos(orbitState.azimuth);
      var oy = orbitState.distance * Math.cos(orbitState.polar);
      var oz = orbitState.distance * Math.sin(orbitState.polar) * Math.sin(orbitState.azimuth);
      camera.position.set(orbitCx + ox, Math.max(1.5, oy), orbitCz + oz);
      camera.lookAt(orbitState.target);
    } else {
      // orbitState 未初始化（首次切到 orbit 模式时）
      camera.position.set(egoX + 15, 10, egoZ + 15);
      camera.lookAt(egoX, 0, egoZ);
    }
    if (camera.fov !== 60) { camera.fov = 60; camera.updateProjectionMatrix(); }
  } else {
    // 未知模式（含 v1 的 'top' / 'driver' / 'front'，v2 统一退化为 chase）
    camera.position.set(egoX - Math.cos(egoHeading) * 6, 3.0, egoZ - Math.sin(egoHeading) * 6);
    camera.lookAt(egoX + Math.cos(egoHeading) * 10, 1.0, egoZ + Math.sin(egoHeading) * 10);
  }
}
