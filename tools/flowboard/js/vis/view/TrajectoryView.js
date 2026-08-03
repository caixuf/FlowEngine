/**
 * TrajectoryView.js — 规划轨迹渲染
 *
 * 直接渲染 planning_node 输出的真实规划轨迹（全局 ENU 坐标点），
 * 不再用前端自行车模型做"假预测"。
 *
 * 视觉效果（类 Tesla FSD / Apollo 风格）：
 *   - 锥形光带：车头处宽，远处收窄，带渐变透明度
 *   - 辉光底层：柔和蓝光底衬
 *   - 方向箭头：等距间隔的锥形箭头指示行驶方向
 *   - 速度着色：正常蓝色、制动橙红、加速青绿
 */

import { worldToThree, forwardENU } from '../math/Coord.js';

const TRAJ_GROUND_OFFSET = 0.08;  // 轨迹离地面高度 (m)，防 z-fighting
const MAX_POINTS = 64;            // 与后端 Trajectory.points[64] 对齐

/* 光带参数 */
const RIBBON_WIDTH_START = 0.45;  // 车头端宽度 (m)
const RIBBON_WIDTH_END   = 0.06;  // 远端宽度 (m)
const RIBBON_ALPHA_START = 0.85;  // 车头端不透明度
const RIBBON_ALPHA_END   = 0.0;   // 远端不透明度

/* 辉光层参数 */
const GLOW_WIDTH_START = 1.2;
const GLOW_WIDTH_END   = 0.2;
const GLOW_ALPHA       = 0.12;

/* 方向箭头 */
const ARROW_SPACING_M  = 5.0;     // 箭头间距 (m)
const ARROW_LENGTH     = 0.5;     // 箭头长度 (m)
const ARROW_RADIUS     = 0.13;    // 箭头底部半径 (m)
const ARROW_ALPHA      = 0.7;

/* 速度着色 */
const COLOR_NORMAL  = 0x00bbff;   // 正常巡航 - 科技蓝
const COLOR_BRAKE   = 0xff8800;   // 制动/减速 - 橙
const COLOR_ACCEL   = 0x00ffaa;   // 加速 - 青绿
const BRAKE_DECEL   = -1.0;       // m/s² 以下视为制动
const ACCEL_THRESH  =  1.0;       // m/s² 以上视为加速

export function createTrajectoryView(scene) {
  const group = new THREE.Group();
  scene.add(group);

  /* 复用几何体/材质，每帧只更新顶点 */
  let ribbonMesh = null;
  let glowMesh = null;
  let arrowGroup = null;
  let ribbonGeo = null;
  let glowGeo = null;

  function _ensureGeometry(maxSegs) {
    if (ribbonMesh) return;

    /* 主光带：三角形带 (triangle strip)
     * 每点 2 个顶点（左/右），npts 点 → 2*npts 顶点 → 2*(npts-1) 三角形 */
    const nVerts = 2 * maxSegs;

    /* 预构建最大尺寸的索引缓冲 */
    const maxIdx = new Uint32Array(2 * (maxSegs - 1) * 3);
    { let ii = 0;
      for (let i = 0; i < maxSegs - 1; i++) {
        const a = i * 2, b = i * 2 + 1, c = i * 2 + 2, d = i * 2 + 3;
        maxIdx[ii++] = a; maxIdx[ii++] = b; maxIdx[ii++] = c;
        maxIdx[ii++] = b; maxIdx[ii++] = d; maxIdx[ii++] = c;
      }
    }

    ribbonGeo = new THREE.BufferGeometry();
    ribbonGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nVerts * 3), 3));
    ribbonGeo.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(nVerts * 4), 4));
    ribbonGeo.setIndex(new THREE.BufferAttribute(maxIdx, 1));
    ribbonGeo.setDrawRange(0, 0);
    const ribbonMat = new THREE.MeshBasicMaterial({
      vertexColors: true,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    ribbonMesh = new THREE.Mesh(ribbonGeo, ribbonMat);
    ribbonMesh.frustumCulled = false;
    group.add(ribbonMesh);

    /* 辉光层：同样的三角形带，更宽更淡 */
    glowGeo = new THREE.BufferGeometry();
    glowGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nVerts * 3), 3));
    glowGeo.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(nVerts * 4), 4));
    glowGeo.setIndex(new THREE.BufferAttribute(maxIdx.slice(), 1));
    glowGeo.setDrawRange(0, 0);
    const glowMat = new THREE.MeshBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: GLOW_ALPHA,
      depthWrite: false,
      side: THREE.DoubleSide,
      blending: THREE.AdditiveBlending,
    });
    glowMesh = new THREE.Mesh(glowGeo, glowMat);
    glowMesh.frustumCulled = false;
    group.add(glowMesh);

    arrowGroup = new THREE.Group();
    group.add(arrowGroup);
  }

  function clear() {
    if (ribbonGeo) ribbonGeo.setDrawRange(0, 0);
    if (glowGeo)   glowGeo.setDrawRange(0, 0);
    if (arrowGroup) {
      while (arrowGroup.children.length) {
        const c = arrowGroup.children[0];
        arrowGroup.remove(c);
        if (c.geometry) c.geometry.dispose();
        if (c.material) c.material.dispose();
      }
    }
  }

  /**
   * 将轨迹点统一转换为 THREE 空间中的中心曲线点
   * 自动检测坐标系：
   *   - 全局 ENU：首点距 ego 很近（<5m），直接使用
   *   - 车体坐标系：首点是小值（前向/侧向），通过 ego heading 转到全局
   * @param {Array} trajPath  [[x,y,v], ...]
   * @param {Object} ego      ego 位姿
   * @returns {Array<{x,y,z,dx,dz,v}>}
   */
  function _buildCurvePoints(trajPath, ego) {
    if (!trajPath || trajPath.length < 2) return [];

    const egoX = ego.x || 0, egoY = ego.y || 0;
    const egoHeading = ego.heading || 0;
    const egoZ = (ego.z || 0) + TRAJ_GROUND_OFFSET;

    /* 检测首点是否在车体坐标系：距离 ego 全局位置远，但数值本身小 */
    const p0 = trajPath[0];
    const dxEgo = p0[0] - egoX;
    const dyEgo = p0[1] - egoY;
    const distToEgo = Math.sqrt(dxEgo * dxEgo + dyEgo * dyEgo);
    const isVehicleFrame = distToEgo > 10 && Math.abs(p0[0]) < 50 && Math.abs(p0[1]) < 50;

    /* Coord 纯函数：heading → 前向/左向单位向量（ENU 坐标系）
     * forwardENU 返回 [cosH, sinH] = [east, north]，即 ENU 前向 */
    const [fxE, fyE] = forwardENU(egoHeading);
    /* 左向 = 前向逆时针转90°：[-sinH, cosH] = [-fyE, fxE] */
    const lxE = -fyE, lyE = fxE;

    function vehicleToGlobal(vx, vy) {
      // vx=forward, vy=left（waypoint_follower 输出约定）
      return [egoX + vx * fxE + vy * lxE,
              egoY + vx * fyE + vy * lyE];
    }

    /* 全部转 THREE 坐标 */
    const raw = trajPath.map(p => {
      let gx, gy;
      if (isVehicleFrame) {
        [gx, gy] = vehicleToGlobal(p[0], p[1]);
      } else {
        gx = p[0]; gy = p[1];
      }
      const [tx, ty, tz] = worldToThree(gx, gy, egoZ);
      return { x: tx, y: ty, z: tz, v: p[2] || 0 };
    });

    /* 计算累积距离、切线方向，截断异常跳变 */
    const result = [];
    let cumDist = 0;
    let prev = null;

    for (let i = 0; i < raw.length; i++) {
      const pt = raw[i];
      if (prev) {
        const ddx = pt.x - prev.x;
        const ddz = pt.z - prev.z;
        const segLen = Math.sqrt(ddx * ddx + ddz * ddz);
        /* 跳跃检测：单段超过 10m 视为异常（规划不会跳变），截断 */
        if (segLen > 10) break;
        cumDist += segLen;
      }
      /* 计算切线方向（用前后差分） */
      let dx = 0, dz = 0;
      if (i === 0) {
        const nxt = raw[i + 1];
        if (nxt) { dx = nxt.x - pt.x; dz = nxt.z - pt.z; }
      } else if (i === raw.length - 1) {
        dx = pt.x - prev.x; dz = pt.z - prev.z;
      } else {
        const nxt = raw[i + 1];
        dx = nxt.x - prev.x; dz = nxt.z - prev.z;
      }
      const dl = Math.sqrt(dx * dx + dz * dz) || 1;
      result.push({
        x: pt.x, y: pt.y, z: pt.z, v: pt.v,
        dx: dx / dl, dz: dz / dl,  // 单位切线方向
        dist: cumDist,
      });
      prev = pt;
    }
    return result;
  }

  /** 根据轨迹速度分布判断颜色：减速橙、加速青、正常蓝 */
  function _colorBySpeed(points) {
    if (points.length < 2) return COLOR_NORMAL;
    /* 取前几个点和后几个点的速度差来判断加减速趋势 */
    const n = points.length;
    const vStart = points[0].v;
    const vEnd = points[Math.min(n - 1, Math.floor(n * 0.3))].v;
    const dv = vEnd - vStart;
    if (dv < BRAKE_DECEL) return COLOR_BRAKE;   // 速度在下降→制动
    if (dv > ACCEL_THRESH) return COLOR_ACCEL;  // 速度在上升→加速
    return COLOR_NORMAL;
  }

  /** 将 0xRRGGBB 分解为 [r,g,b] (0~1) */
  function _hexToRgb(hex) {
    return [(hex >> 16) & 255, (hex >> 8) & 255, hex & 255].map(c => c / 255);
  }

  /**
   * 构建锥形光带几何体（triangle strip）
   * 宽度从 start 线性递减到 end，颜色从亮到透明
   */
  function _buildRibbon(geo, points, widthStart, widthEnd, alphaStart, alphaEnd, colorHex, additive) {
    const n = points.length;
    if (n < 2) { geo.setDrawRange(0, 0); return; }

    const pos = geo.attributes.position.array;
    const col = geo.attributes.color.array;
    const [r, g, b] = _hexToRgb(colorHex);
    const totalLen = points[n - 1].dist || 1;

    let vi = 0; // vertex index (scalar, into Float32Array)
    let ci = 0; // color index

    for (let i = 0; i < n; i++) {
      const p = points[i];
      const t = totalLen > 0.1 ? p.dist / totalLen : 0;
      const w = (widthStart + (widthEnd - widthStart) * t) * 0.5; // 半宽
      const a = alphaStart + (alphaEnd - alphaStart) * t;

      /* 侧向（左）= 切线方向逆时针转90° (dx,dz) → (-dz, dx) */
      const lx = -p.dz * w;
      const lz =  p.dx * w;

      /* 左顶点 */
      pos[vi++] = p.x + lx;
      pos[vi++] = p.y;
      pos[vi++] = p.z + lz;
      col[ci++] = r; col[ci++] = g; col[ci++] = b; col[ci++] = a;

      /* 右顶点 */
      pos[vi++] = p.x - lx;
      pos[vi++] = p.y;
      pos[vi++] = p.z - lz;
      col[ci++] = r; col[ci++] = g; col[ci++] = b; col[ci++] = a;
    }

    /* 将尾部剩余顶点清零，避免上一帧的残留数据影响显示 */
    const totalVerts = geo.attributes.position.count;
    while (vi < totalVerts * 3) { pos[vi++] = 0; pos[vi++] = 0; pos[vi++] = 0; }
    while (ci < totalVerts * 4) { col[ci++] = 0; col[ci++] = 0; col[ci++] = 0; col[ci++] = 0; }

    geo.attributes.position.needsUpdate = true;
    geo.attributes.color.needsUpdate = true;
    geo.setDrawRange(0, 6 * (n - 1));  // 2*(n-1) tris × 3 idx
    geo.computeVertexNormals();
  }

  /** 沿路径放置方向箭头 */
  function _buildArrows(points, colorHex) {
    while (arrowGroup.children.length) {
      const c = arrowGroup.children[0];
      arrowGroup.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
    if (points.length < 2) return;

    const totalLen = points[points.length - 1].dist || 1;
    if (totalLen < ARROW_LENGTH) return;

    /* 箭头材质（共享） */
    const mat = new THREE.MeshBasicMaterial({
      color: colorHex,
      transparent: true,
      opacity: ARROW_ALPHA,
      depthWrite: false,
    });

    const up = new THREE.Vector3(0, 1, 0);
    let nextArrowDist = ARROW_SPACING_M * 0.3; // 第一个箭头距起点略近

    for (let i = 1; i < points.length; i++) {
      const p = points[i];
      const prev = points[i - 1];
      while (p.dist >= nextArrowDist && i < points.length) {
        /* 在 seg 上插值找到箭头位置 */
        const segLen = p.dist - prev.dist;
        const t = segLen > 0.001 ? (nextArrowDist - prev.dist) / segLen : 0;
        const ax = prev.x + (p.x - prev.x) * t;
        const ay = prev.y;
        const az = prev.z + (p.z - prev.z) * t;
        const adx = p.dx;
        const adz = p.dz;

        const geo = new THREE.ConeGeometry(ARROW_RADIUS, ARROW_LENGTH, 6);
        const mesh = new THREE.Mesh(geo, mat.clone());
        mesh.position.set(ax, ay + 0.20, az);

        /* 箭头指向切线方向 */
        const dir = new THREE.Vector3(adx, 0, adz).normalize();
        const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);
        mesh.quaternion.copy(quat);

        /* 远端箭头渐隐 */
        const fadeT = totalLen > 0.1 ? nextArrowDist / totalLen : 0;
        mesh.material.opacity = ARROW_ALPHA * (1 - fadeT * 0.7);

        arrowGroup.add(mesh);
        nextArrowDist += ARROW_SPACING_M;
      }
    }
  }

  function update(store) {
    const trajPath = store.trajectoryPath;
    const ego = store.ego;

    if (!ego || !trajPath || trajPath.length < 2) {
      clear();
      return;
    }

    _ensureGeometry(MAX_POINTS);

    /* 1. 构建 THREE 空间曲线点（含切线方向、累积距离） */
    const points = _buildCurvePoints(trajPath, ego);
    if (points.length < 2) {
      clear();
      return;
    }

    /* 2. 决定颜色 */
    const color = _colorBySpeed(points);

    /* 3. 构建辉光层 */
    _buildRibbon(glowGeo, points, GLOW_WIDTH_START, GLOW_WIDTH_END, GLOW_ALPHA, 0, color, true);

    /* 4. 构建主光带 */
    _buildRibbon(ribbonGeo, points, RIBBON_WIDTH_START, RIBBON_WIDTH_END, RIBBON_ALPHA_START, RIBBON_ALPHA_END, color, false);

    /* 5. 方向箭头 */
    _buildArrows(points, color);
  }

  function dispose() {
    clear();
    if (ribbonGeo) ribbonGeo.dispose();
    if (glowGeo) glowGeo.dispose();
    if (ribbonMesh && ribbonMesh.material) ribbonMesh.material.dispose();
    if (glowMesh && glowMesh.material) glowMesh.material.dispose();
    scene.remove(group);
  }

  return { update, clear, dispose };
}
