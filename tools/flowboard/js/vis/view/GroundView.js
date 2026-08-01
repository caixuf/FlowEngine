/**
 * GroundView.js — 程序化草地/地面纹理
 *
 * 使用 Canvas 2D 生成草地纹理：
 *   - 草绿色底 + 随机噪点（深浅变化）
 *   - Sobel 算子生成法线贴图（凹凸感）
 *   - 零外部资产，所有纹理运行时生成
 *
 * Phase 0.3 升级：纯色 → 程序化草地纹理
 */

const TEXTURE_SIZE = 512;       // 纹理分辨率（够用，不浪费 GPU）
const GRASS_COLOR = 0x3e6b34;   // 原纯色版基色，程序纹理围绕它做深浅变化

/** 生成草地颜色纹理（Canvas 2D） */
function _createGrassColorTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = TEXTURE_SIZE;
  canvas.height = TEXTURE_SIZE;
  const ctx = canvas.getContext('2d');

  // 写入像素数据
  const imageData = ctx.createImageData(TEXTURE_SIZE, TEXTURE_SIZE);
  const data = imageData.data;

  for (let y = 0; y < TEXTURE_SIZE; y++) {
    for (let x = 0; x < TEXTURE_SIZE; x++) {
      const i = (y * TEXTURE_SIZE + x) * 4;
      // 基色 #3e6b34（62,107,52）——与旧版纯色一致，只做 ±12 深浅变化。
      // （初版基色 (100,180,60) 亮黄绿整屏刺眼，是"效果差"主因之一）
      const variation = (Math.random() - 0.5) * 24;  // -12 ~ +12
      const r = Math.max(0, Math.min(255, 62 + variation));
      const g = Math.max(0, Math.min(255, 107 + variation));
      const b = Math.max(0, Math.min(255, 52 + variation * 0.5));
      data[i] = r;
      data[i + 1] = g;
      data[i + 2] = b;
      data[i + 3] = 255;
    }
  }
  ctx.putImageData(imageData, 0, 0);

  // 可选：叠加草叶纹理（细短线，颜色贴基色暗侧）
  ctx.strokeStyle = 'rgba(40, 80, 32, 0.15)';
  ctx.lineWidth = 1;
  for (let i = 0; i < 2000; i++) {
    const sx = Math.random() * TEXTURE_SIZE;
    const sy = Math.random() * TEXTURE_SIZE;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + (Math.random() - 0.5) * 4, sy - Math.random() * 6);
    ctx.stroke();
  }

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  // 纹理覆盖范围：512px / 16px_per_m = 32m per tile
  // 地面 20000m → repeat ~625 次（整数，无接缝）
  texture.repeat.set(625, 625);
  texture.anisotropy = 4;
  return texture;
}

/** 生成草地法线贴图（Sobel 梯度，Canvas 2D）
 *  从颜色纹理的亮度变化计算法线，
 *  让草地有凹凸起伏的立体感。 */
function _createGrassNormalMap() {
  const canvas = document.createElement('canvas');
  canvas.width = TEXTURE_SIZE;
  canvas.height = TEXTURE_SIZE;
  const ctx = canvas.getContext('2d');

  // 先画一个随机高度场
  const heightMap = new Float32Array(TEXTURE_SIZE * TEXTURE_SIZE);
  for (let i = 0; i < heightMap.length; i++) {
    heightMap[i] = Math.random() * 0.5 + 0.25;  // 0.25~0.75
  }

  // 简单模糊（3x3 box blur）让法线平滑
  const blurred = new Float32Array(TEXTURE_SIZE * TEXTURE_SIZE);
  for (let y = 1; y < TEXTURE_SIZE - 1; y++) {
    for (let x = 1; x < TEXTURE_SIZE - 1; x++) {
      let sum = 0;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          sum += heightMap[(y + dy) * TEXTURE_SIZE + (x + dx)];
        }
      }
      blurred[y * TEXTURE_SIZE + x] = sum / 9;
    }
  }

  const imageData = ctx.createImageData(TEXTURE_SIZE, TEXTURE_SIZE);
  const data = imageData.data;
  const strength = 2.0;  // 法线强度

  for (let y = 1; y < TEXTURE_SIZE - 1; y++) {
    for (let x = 1; x < TEXTURE_SIZE - 1; x++) {
      const idx = y * TEXTURE_SIZE + x;
      // Sobel X
      const gx = (blurred[(y - 1) * TEXTURE_SIZE + (x + 1)] - blurred[(y - 1) * TEXTURE_SIZE + (x - 1)]
                + blurred[y * TEXTURE_SIZE + (x + 1)] * 2 - blurred[y * TEXTURE_SIZE + (x - 1)] * 2
                + blurred[(y + 1) * TEXTURE_SIZE + (x + 1)] - blurred[(y + 1) * TEXTURE_SIZE + (x - 1)]);
      // Sobel Y
      const gy = (blurred[(y + 1) * TEXTURE_SIZE + (x - 1)] - blurred[(y - 1) * TEXTURE_SIZE + (x - 1)]
                + blurred[(y + 1) * TEXTURE_SIZE + x] * 2 - blurred[(y - 1) * TEXTURE_SIZE + x] * 2
                + blurred[(y + 1) * TEXTURE_SIZE + (x + 1)] - blurred[(y - 1) * TEXTURE_SIZE + (x + 1)]);

      // 法线 = normalize(-gx * strength, -gy * strength, 1.0)
      const nx = -gx * strength;
      const ny = -gy * strength;
      const nz = 1.0;
      const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
      const pi = (idx) * 4;
      data[pi]     = Math.round((nx / len * 0.5 + 0.5) * 255);
      data[pi + 1] = Math.round((ny / len * 0.5 + 0.5) * 255);
      data[pi + 2] = Math.round((nz / len * 0.5 + 0.5) * 255);
      data[pi + 3] = 255;
    }
  }
  ctx.putImageData(imageData, 0, 0);

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(625, 625);
  return texture;
}

export function createGroundView(scene) {
  let mesh = null;
  let _colorTex = null;
  let _normalTex = null;

  function build(size = 20000) {
    if (mesh) {
      scene.remove(mesh);
      mesh.geometry.dispose();
      mesh.material.dispose();
      mesh = null;
    }
    if (size <= 0) return;

    const geo = new THREE.PlaneGeometry(size, size);
    let mat;
    if (typeof document === 'undefined') {
      // Node 测试环境无 document：回退纯色（浏览器走程序纹理）
      mat = new THREE.MeshStandardMaterial({
        color: GRASS_COLOR, roughness: 0.9, metalness: 0.0,
      });
    } else {
      // 生成程序化纹理（只生成一次，后续复用）
      if (!_colorTex) _colorTex = _createGrassColorTexture();
      if (!_normalTex) _normalTex = _createGrassNormalMap();
      mat = new THREE.MeshStandardMaterial({
        map: _colorTex,
        normalMap: _normalTex,
        // 法线强度压低：per-pixel 随机高度场在远处会闪烁（shimmer）
        normalScale: new THREE.Vector2(0.4, 0.4),
        roughness: 0.9,
        metalness: 0.0,
      });
    }
    mesh = new THREE.Mesh(geo, mat);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.y = -0.05;
    mesh.receiveShadow = true;
    mesh.frustumCulled = true;
    scene.add(mesh);
  }

  function getMesh() { return mesh; }

  return { build, getMesh };
}