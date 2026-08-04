/**
 * TrajectoryView.js — 规划轨迹渲染
 *
 * 直接渲染 planning_node 输出的真实规划轨迹（全局 ENU 坐标点），
 * 不再用前端自行车模型做"假预测"。
 *
 * 视觉效果（类 Tesla FSD / Apollo 风格）：
 *   - CatmullRom 样条平滑：原始规划点（64点/10Hz）插值到密集顶点，转弯顺滑无折线
 *   - 三层光带：外辉光（加法混合）+ 主体光带（锥形收窄）+ 内亮核心线
 *   - 流动高亮条：沿路径脉冲流动的亮点，强化方向感与科技感
 *   - 方向箭头：等距间隔的锥形箭头
 *   - 按点速度染色：巡航蓝 / 制动橙 / 加速青绿
 */

import { worldToThree, forwardENU } from '../math/Coord.js';

const TRAJ_GROUND_OFFSET = 0.08;
const MAX_PLAN_POINTS   = 64;             // 后端轨迹点上限
const SPLINE_SUBDIV     = 4;              // 每个规划段细分数（64→~250 渲染点）
const MAX_RENDER_POINTS = MAX_PLAN_POINTS * SPLINE_SUBDIV;

/* ── 三层光带参数 ── */
// 外层辉光（加法混合，柔和蓝光）
const OUTER_WIDTH_START = 2.2;
const OUTER_WIDTH_END   = 0.15;
const OUTER_ALPHA       = 0.10;
// 主体光带（2026-08：远端不归零 —— 旧值 END=0.04/0.0 远端完全消失，
// 轨迹只显示车头一段 + 流动亮点 → 观感"点点"。全程可见的连续轨迹线）
const RIBBON_WIDTH_START = 0.55;
const RIBBON_WIDTH_END   = 0.12;
const RIBBON_ALPHA_START = 0.90;
const RIBBON_ALPHA_END   = 0.30;
// 内亮核心线（细线，车头最亮，远端保持可见）
const CORE_WIDTH_START = 0.12;
const CORE_WIDTH_END   = 0.04;
const CORE_ALPHA_START = 1.0;
const CORE_ALPHA_END   = 0.35;

/* 方向箭头 */
const ARROW_SPACING_M  = 4.5;
const ARROW_LENGTH     = 0.55;
const ARROW_RADIUS     = 0.15;
const ARROW_ALPHA      = 0.75;

/* 流动高亮条 */
const FLOW_SPEED       = 12;             // m/s，流动速度
const FLOW_SEG_LEN     = 3.0;            // 高亮段长度
const FLOW_WIDTH       = 0.65;           // 高亮段宽度（相对主光带放大约1.2x）
const FLOW_ALPHA       = 0.45;

/* 速度着色 */
const COLOR_NORMAL  = [0x00, 0xc8, 0xff];  // 科技蓝
const COLOR_BRAKE   = [0xff, 0x88, 0x00];  // 制动橙
const COLOR_ACCEL   = [0x00, 0xff, 0xaa];  // 加速青绿
const BRAKE_DECEL   = -1.0;
const ACCEL_THRESH  =  1.0;

/* 预设颜色数组（避免每帧 hex→rgb 转换） */
const _R = 0, _G = 1, _B = 2;

export function createTrajectoryView(scene) {
  const group = new THREE.Group();
  scene.add(group);

  let outerMesh = null;
  let ribbonMesh = null;
  let coreMesh = null;
  let flowMesh = null;
  let arrowGroup = null;
  let outerGeo = null, ribbonGeo = null, coreGeo = null, flowGeo = null;
  let flowMat = null;

  function _buildIndexBuffer(nSegs) {
    const idx = new Uint32Array(2 * (nSegs - 1) * 3);
    let ii = 0;
    for (let i = 0; i < nSegs - 1; i++) {
      const a = i * 2, b = i * 2 + 1, c = i * 2 + 2, d = i * 2 + 3;
      idx[ii++] = a; idx[ii++] = b; idx[ii++] = c;
      idx[ii++] = b; idx[ii++] = d; idx[ii++] = c;
    }
    return idx;
  }

  function _ensureGeometry() {
    if (ribbonMesh) return;

    const nVerts = 2 * MAX_RENDER_POINTS;
    const maxIdx = _buildIndexBuffer(MAX_RENDER_POINTS);

    /* 外层辉光 */
    outerGeo = new THREE.BufferGeometry();
    outerGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nVerts * 3), 3));
    outerGeo.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(nVerts * 4), 4));
    outerGeo.setIndex(new THREE.BufferAttribute(maxIdx.slice(), 1));
    outerGeo.setDrawRange(0, 0);
    const outerMat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false,
      side: THREE.DoubleSide, blending: THREE.AdditiveBlending,
    });
    outerMesh = new THREE.Mesh(outerGeo, outerMat);
    outerMesh.frustumCulled = false;
    group.add(outerMesh);

    /* 主体光带 */
    ribbonGeo = new THREE.BufferGeometry();
    ribbonGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nVerts * 3), 3));
    ribbonGeo.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(nVerts * 4), 4));
    ribbonGeo.setIndex(new THREE.BufferAttribute(maxIdx.slice(), 1));
    ribbonGeo.setDrawRange(0, 0);
    const ribbonMat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false,
      side: THREE.DoubleSide,
    });
    ribbonMesh = new THREE.Mesh(ribbonGeo, ribbonMat);
    ribbonMesh.frustumCulled = false;
    group.add(ribbonMesh);

    /* 内亮核心线 */
    coreGeo = new THREE.BufferGeometry();
    coreGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nVerts * 3), 3));
    coreGeo.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(nVerts * 4), 4));
    coreGeo.setIndex(new THREE.BufferAttribute(maxIdx.slice(), 1));
    coreGeo.setDrawRange(0, 0);
    const coreMat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false,
      side: THREE.DoubleSide, blending: THREE.AdditiveBlending,
    });
    coreMesh = new THREE.Mesh(coreGeo, coreMat);
    coreMesh.frustumCulled = false;
    group.add(coreMesh);

    /* 流动高亮条（可复用几何，每帧仅更新颜色 alpha 做脉冲） */
    flowGeo = new THREE.BufferGeometry();
    flowGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nVerts * 3), 3));
    flowGeo.setAttribute('color',    new THREE.BufferAttribute(new Float32Array(nVerts * 4), 4));
    flowGeo.setIndex(new THREE.BufferAttribute(maxIdx.slice(), 1));
    flowGeo.setDrawRange(0, 0);
    flowMat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false,
      side: THREE.DoubleSide, blending: THREE.AdditiveBlending,
    });
    flowMesh = new THREE.Mesh(flowGeo, flowMat);
    flowMesh.frustumCulled = false;
    group.add(flowMesh);

    arrowGroup = new THREE.Group();
    group.add(arrowGroup);
  }

  function clear() {
    [outerGeo, ribbonGeo, coreGeo, flowGeo].forEach(g => { if (g) g.setDrawRange(0, 0); });
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
   * 把后端原始轨迹点转为 THREE 坐标并做样条平滑
   * @returns {Array<{x,y,z,dx,dz,dist,v,rgb}>}
   */
  function _buildSmoothPoints(trajPath, ego) {
    if (!trajPath || trajPath.length < 2) return [];

    const egoZ = (ego.z || 0) + TRAJ_GROUND_OFFSET;

    /* 轨迹起点锚定到车头实时位置（ENU）：
     * 规划 10Hz 快照，车每帧（60fps）实时运动。变道时车横向移动快，
     * 轨迹起点停留在上一次规划时的车位置，而车头已前进/横移 → 轨迹与车脱节。
     * 用速度方向把轨迹第0点前移到车头，其余真实规划点保留，CatmullRom
     * 平滑衔接，全程贴合车头。 */
    const [fx, fy] = forwardENU(ego.heading || 0);
    const frontX = ego.x + fx * 1.5;
    const frontY = ego.y + fy * 1.5;

    /* 1. 先把原始点转 THREE 坐标，截断跳变 */
    const raw3d = [];
    for (let i = 0; i < trajPath.length; i++) {
      const p = trajPath[i];
      const [tx, ty, tz] = worldToThree(p[0], p[1], egoZ);
      const v = p[2] || 0;
      if (i > 0) {
        const prev = raw3d[raw3d.length - 1];
        const dx = tx - prev.x, dz = tz - prev.z;
        if (Math.sqrt(dx * dx + dz * dz) > 10) break;
      }
      raw3d.push(new THREE.Vector3(tx, ty, tz));
      raw3d[raw3d.length - 1].v = v;
    }
    /* 用车头实时位置替换轨迹起点（防脱节）。车头不参与上方跳变检查，
     * 避免"车偏离轨迹点>10m"时误截断整条轨迹。 */
    if (raw3d.length >= 1) {
      const [tx0, ty0, tz0] = worldToThree(frontX, frontY, egoZ);
      raw3d[0].set(tx0, ty0, tz0);
      raw3d[0].v = trajPath[0][2] || 0;
    }
    if (raw3d.length < 2) return [];

    /* 2. CatmullRom 样条插值（centripetal 适合非均匀点距） */
    const curve = new THREE.CatmullRomCurve3(raw3d, false, 'centripetal', 0.5);
    const nSmooth = Math.min(raw3d.length * SPLINE_SUBDIV, MAX_RENDER_POINTS);
    const smoothPts = curve.getSpacedPoints(nSmooth - 1);

    /* 3. 在原始 raw3d 上构建速度查找（用于后续颜色插值） */
    const rawDist = [0];
    let rawCum = 0;
    for (let i = 1; i < raw3d.length; i++) {
      rawCum += raw3d[i].distanceTo(raw3d[i - 1]);
      rawDist.push(rawCum);
    }
    const totalRawLen = rawCum;

    /* 4. 为每个平滑点计算切线、累积距离、速度、颜色 */
    const result = [];
    let cumDist = 0;
    for (let i = 0; i < smoothPts.length; i++) {
      const pt = smoothPts[i];
      /* 切线方向 */
      let tx = 0, tz = 0;
      if (i === 0) {
        const n = smoothPts[1];
        if (n) { tx = n.x - pt.x; tz = n.z - pt.z; }
      } else if (i === smoothPts.length - 1) {
        tx = pt.x - smoothPts[i - 1].x;
        tz = pt.z - smoothPts[i - 1].z;
      } else {
        const n = smoothPts[i + 1];
        const p = smoothPts[i - 1];
        tx = n.x - p.x; tz = n.z - p.z;
      }
      const tl = Math.sqrt(tx * tx + tz * tz) || 1;
      tx /= tl; tz /= tl;

      if (i > 0) {
        cumDist += pt.distanceTo(smoothPts[i - 1]);
      }

      /* 速度线性插值：将平滑点的 cumDist 映射回 raw3d 段 */
      const tRaw = totalRawLen > 0.01 ? Math.min(cumDist / totalRawLen, 1) : 0;
      const targetRawDist = tRaw * totalRawLen;
      let ri = 0;
      for (let j = 1; j < rawDist.length; j++) {
        if (rawDist[j] >= targetRawDist || j === rawDist.length - 1) { ri = j - 1; break; }
      }
      const rSeg = rawDist[ri + 1] - rawDist[ri] || 1;
      const rT = Math.min(Math.max((targetRawDist - rawDist[ri]) / rSeg, 0), 1);
      const nextRi = Math.min(ri + 1, raw3d.length - 1);
      const vInterp = raw3d[ri].v + (raw3d[nextRi].v - raw3d[ri].v) * rT;

      result.push({
        x: pt.x, y: pt.y, z: pt.z,
        dx: tx, dz: tz,
        dist: cumDist,
        v: vInterp,
      });
    }

    /* 5. 速度→颜色：先整体判断加减速，再做远端向蓝色渐变 */
    let baseColor;
    if (result.length > 5) {
      const vS = result[0].v;
      const vM = result[Math.min(Math.floor(result.length * 0.4), result.length - 1)].v;
      const dv = vM - vS;
      if (dv < BRAKE_DECEL) baseColor = COLOR_BRAKE;
      else if (dv > ACCEL_THRESH) baseColor = COLOR_ACCEL;
      else baseColor = COLOR_NORMAL;
    } else {
      baseColor = COLOR_NORMAL;
    }

    /* 为每个点附颜色：在基础色上根据局部速度变化微调 */
    for (let i = 0; i < result.length; i++) {
      /* 远端颜色向蓝色过渡（无论什么状态），产生自然渐变 */
      const t = result.length > 1 ? i / (result.length - 1) : 0;
      const r = baseColor[_R] + (COLOR_NORMAL[_R] - baseColor[_R]) * t * 0.3;
      const g = baseColor[_G] + (COLOR_NORMAL[_G] - baseColor[_G]) * t * 0.3;
      const b = baseColor[_B] + (COLOR_NORMAL[_B] - baseColor[_B]) * t * 0.3;
      result[i].rgb = [r / 255, g / 255, b / 255];
    }

    return result;
  }

  /**
   * 写入一条 triangle strip 光带
   */
  function _fillRibbon(geo, points, widthStart, widthEnd, alphaStart, alphaEnd,
                       usePointColor, fixedColor, flowOffset) {
    const n = points.length;
    if (n < 2) { geo.setDrawRange(0, 0); return 0; }

    const pos = geo.attributes.position.array;
    const col = geo.attributes.color.array;
    const totalLen = points[n - 1].dist || 1;

    let vi = 0, ci = 0;

    for (let i = 0; i < n; i++) {
      const p = points[i];
      const t = totalLen > 0.1 ? p.dist / totalLen : 0;
      const w = (widthStart + (widthEnd - widthStart) * t) * 0.5;
      let a = alphaStart + (alphaEnd - alphaStart) * t;

      /* 流动高亮：flowOffset 是时间偏移，以 sin 波控制高亮段 */
      if (flowOffset !== undefined) {
        const phase = ((p.dist - flowOffset) % FLOW_SEG_LEN + FLOW_SEG_LEN) % FLOW_SEG_LEN;
        const glow = Math.max(0, 1 - Math.abs(phase - FLOW_SEG_LEN * 0.5) / (FLOW_SEG_LEN * 0.35));
        a *= glow * glow;  // 平方让亮点更聚焦
        if (a < 0.005) a = 0;
      }

      const lx = -p.dz * w;
      const lz =  p.dx * w;

      let cr, cg, cb;
      if (usePointColor && p.rgb) {
        cr = p.rgb[0]; cg = p.rgb[1]; cb = p.rgb[2];
      } else {
        cr = fixedColor[0]; cg = fixedColor[1]; cb = fixedColor[2];
      }

      /* 左顶点 */
      pos[vi++] = p.x + lx; pos[vi++] = p.y; pos[vi++] = p.z + lz;
      col[ci++] = cr; col[ci++] = cg; col[ci++] = cb; col[ci++] = a;
      /* 右顶点 */
      pos[vi++] = p.x - lx; pos[vi++] = p.y; pos[vi++] = p.z - lz;
      col[ci++] = cr; col[ci++] = cg; col[ci++] = cb; col[ci++] = a;
    }

    const totalVerts = geo.attributes.position.count;
    while (vi < totalVerts * 3) { pos[vi++] = 0; pos[vi++] = 0; pos[vi++] = 0; }
    while (ci < totalVerts * 4) { col[ci++] = 0; col[ci++] = 0; col[ci++] = 0; col[ci++] = 0; }

    geo.attributes.position.needsUpdate = true;
    geo.attributes.color.needsUpdate = true;
    geo.setDrawRange(0, 6 * (n - 1));
    geo.computeVertexNormals();
    return totalLen;
  }

  /** 沿路径放置方向箭头 */
  function _buildArrows(points, colorArr) {
    while (arrowGroup.children.length) {
      const c = arrowGroup.children[0];
      arrowGroup.remove(c);
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    }
    if (points.length < 2) return;

    const totalLen = points[points.length - 1].dist || 1;
    if (totalLen < ARROW_LENGTH) return;

    const hex = (colorArr[_R] << 16) | (colorArr[_G] << 8) | colorArr[_B];
    const mat = new THREE.MeshBasicMaterial({
      color: hex, transparent: true, opacity: ARROW_ALPHA, depthWrite: false,
    });

    const up = new THREE.Vector3(0, 1, 0);
    let nextArrowDist = ARROW_SPACING_M * 0.3;

    for (let i = 1; i < points.length; i++) {
      const p = points[i];
      const prev = points[i - 1];
      const segLen = p.dist - prev.dist;
      while (p.dist >= nextArrowDist) {
        const t = segLen > 0.001 ? (nextArrowDist - prev.dist) / segLen : 0;
        const ax = prev.x + (p.x - prev.x) * t;
        const az = prev.z + (p.z - prev.z) * t;
        const adx = prev.dx + (p.dx - prev.dx) * t;
        const adz = prev.dz + (p.dz - prev.dz) * t;

        const geo = new THREE.ConeGeometry(ARROW_RADIUS, ARROW_LENGTH, 6);
        const mesh = new THREE.Mesh(geo, mat.clone());
        mesh.position.set(ax, prev.y + 0.25, az);

        const dir = new THREE.Vector3(adx, 0, adz).normalize();
        mesh.quaternion.setFromUnitVectors(up, dir);

        const fadeT = totalLen > 0.1 ? nextArrowDist / totalLen : 0;
        mesh.material.opacity = ARROW_ALPHA * (1 - fadeT * 0.7);

        arrowGroup.add(mesh);
        nextArrowDist += ARROW_SPACING_M;
        if (nextArrowDist > totalLen) break;
      }
      if (nextArrowDist > totalLen) break;
    }
  }

  /* 流动动画时间 */
  let _flowTime = 0;
  let _lastFrameT = 0;

  function update(store) {
    const trajPath = store.trajectoryPath;
    const ego = store.ego;
    const now = performance.now() / 1000;

    if (!ego || !trajPath || trajPath.length < 2) {
      clear();
      _lastFrameT = now;
      return;
    }

    _ensureGeometry();

    /* 时间累加（首帧跳过 dt 避免跳变） */
    if (_lastFrameT > 0) {
      _flowTime += (now - _lastFrameT) * FLOW_SPEED;
    }
    _lastFrameT = now;

    /* 1. 构建平滑曲线点 */
    const points = _buildSmoothPoints(trajPath, ego);
    if (points.length < 2) { clear(); return; }

    /* 取起点颜色作为固定色（辉光/箭头用） */
    const firstRgb = points[0].rgb;
    const fixedColor = [firstRgb[0], firstRgb[1], firstRgb[2]];

    /* 2. 外层辉光（固定色，加法混合） */
    _fillRibbon(outerGeo, points, OUTER_WIDTH_START, OUTER_WIDTH_END, OUTER_ALPHA, 0, false, fixedColor);

    /* 3. 主体光带（逐点颜色，平滑色彩过渡） */
    _fillRibbon(ribbonGeo, points, RIBBON_WIDTH_START, RIBBON_WIDTH_END, RIBBON_ALPHA_START, RIBBON_ALPHA_END, true, fixedColor);

    /* 4. 内亮核心（白色偏基础色，加法混合） */
    const coreColor = [
      Math.min(1, fixedColor[0] * 0.4 + 0.6),
      Math.min(1, fixedColor[1] * 0.4 + 0.6),
      Math.min(1, fixedColor[2] * 0.4 + 0.6),
    ];
    _fillRibbon(coreGeo, points, CORE_WIDTH_START, CORE_WIDTH_END, CORE_ALPHA_START, CORE_ALPHA_END, false, coreColor);

    /* 5. 流动高亮条（高亮主光带，略宽一点，用加法混合 + 时间偏移 sin 波） */
    const totalLen = points[points.length - 1].dist || 1;
    const flowOff = _flowTime % (FLOW_SEG_LEN * 2);
    _fillRibbon(flowGeo, points, FLOW_WIDTH, FLOW_WIDTH * 0.3, FLOW_ALPHA, 0, true, fixedColor, flowOff);

    /* 6. 方向箭头 */
    _buildArrows(points, [Math.round(fixedColor[0]*255), Math.round(fixedColor[1]*255), Math.round(fixedColor[2]*255)]);
  }

  function dispose() {
    clear();
    [outerGeo, ribbonGeo, coreGeo, flowGeo].forEach(g => { if (g) g.dispose(); });
    [outerMesh, ribbonMesh, coreMesh, flowMesh].forEach(m => {
      if (m && m.material) m.material.dispose();
    });
    scene.remove(group);
  }

  return { update, clear, dispose };
}
