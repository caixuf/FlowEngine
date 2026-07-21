/**
 * ViaductView.js — 高架 + 平行国道复合场景
 *
 * 移植自 docs/scene.html 的 buildElevatedHighway / buildNationalHighway / makeMarkings /
 * makeGuardrail / buildTallLamp / buildShortLamp / createTrees / createBushes / makeTree。
 * 用于 FlowEngine 基础设施 demo 场景：单 ego 在高架直道上行驶。
 *
 * 接口契约（vis/ 模块标准）：
 *   const v = createViaductView(scene);
 *   v.build({ laneCount: 4 });   // 重建（roadHash 变化时由 SceneDirector 调）
 *   v.getGroup();                // 拿 group
 *
 * 坐标约定：X=路长方向 Y=高度 Z=横向偏移
 *   高架：cx=0, cz=0,   Y=7
 *   国道：cx=0, cz=34,  Y=0.4
 */

const LANE_W = 2;        // 每车道宽（与原模型一致）

// ── 材质表（避免跨实例共享导致颜色互相污染） ──
let _mats = null;
function _ensureMaterials() {
  if (_mats) return _mats;
  _mats = {
    road: new THREE.MeshStandardMaterial({ color: 0x2a2a2a, roughness: 0.85, metalness: 0.02 }),
    roadDark: new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.9, metalness: 0.01 }),
    lineWhite: new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.6 }),
    lineYellow: new THREE.MeshStandardMaterial({ color: 0xffd700, roughness: 0.6 }),
    concrete: new THREE.MeshStandardMaterial({ color: 0x9a9a9a, roughness: 0.75, metalness: 0.1 }),
    concreteDark: new THREE.MeshStandardMaterial({ color: 0x7a7a7a, roughness: 0.8, metalness: 0.05 }),
    metalRail: new THREE.MeshStandardMaterial({ color: 0xa8a8a8, roughness: 0.35, metalness: 0.55 }),
    grass: new THREE.MeshStandardMaterial({ color: 0x4a7a35, roughness: 0.95 }),
    pole: new THREE.MeshStandardMaterial({ color: 0x3a3a3a, roughness: 0.55, metalness: 0.3 }),
    lampGlow: new THREE.MeshStandardMaterial({ color: 0xfff0c0, emissive: 0xffcc66, emissiveIntensity: 0.8 }),
    treeTrunk: new THREE.MeshStandardMaterial({ color: 0x5c4033, roughness: 0.9 }),
    treeLeaf: new THREE.MeshStandardMaterial({ color: 0x3a8032, roughness: 0.85 }),
    treeLeafDark: new THREE.MeshStandardMaterial({ color: 0x2d5e27, roughness: 0.9 }),
    treeLeafLight: new THREE.MeshStandardMaterial({ color: 0x4a9a40, roughness: 0.8 }),
    dirt: new THREE.MeshStandardMaterial({ color: 0x8b7355, roughness: 0.95 }),
    glassBarrier: new THREE.MeshStandardMaterial({
      color: 0x88bbee, transparent: true, opacity: 0.3, roughness: 0.1, metalness: 0.5
    }),
    curb: new THREE.MeshStandardMaterial({ color: 0x888888, roughness: 0.8 }),
  };
  return _mats;
}

// ── 工具：标线 ──
function _makeLineGeo(length, width) {
  const geo = new THREE.PlaneGeometry(length, width);
  geo.rotateX(-Math.PI / 2);
  return geo;
}

function _makeMarkings(length, width, lanes, height) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const laneW = width / lanes;
  const y = height + 0.02;
  const perSide = lanes / 2;

  // 边缘白实线
  const edgeGeo = _makeLineGeo(length, 0.12);
  for (const side of [-1, 1]) {
    const edge = new THREE.Mesh(edgeGeo, M.lineWhite);
    edge.position.set(0, y, side * (width / 2 - 0.3));
    group.add(edge);
  }

  // 双向 2 车道：单黄中心实线
  if (lanes === 2) {
    const centerLine = new THREE.Mesh(_makeLineGeo(length, 0.12), M.lineYellow);
    centerLine.position.set(0, y, 0);
    group.add(centerLine);
  }

  // 双向 4+ 车道：双黄实线
  if (lanes >= 4) {
    const yGeo = _makeLineGeo(length, 0.15);
    for (const side of [-1, 1]) {
      const line = new THREE.Mesh(yGeo, M.lineYellow);
      line.position.set(0, y, side * 0.2);
      group.add(line);
    }
  }

  // 车道分隔虚线
  const dashLen = 4, dashGap = 6;
  const dashCount = Math.floor(length / (dashLen + dashGap));
  const dashGeo = _makeLineGeo(dashLen, 0.1);
  for (let side = -1; side <= 1; side += 2) {
    for (let l = 1; l < perSide; l++) {
      const offset = side * laneW * l;
      for (let i = 0; i < dashCount; i++) {
        const x = -length / 2 + i * (dashLen + dashGap) + dashLen / 2;
        const dash = new THREE.Mesh(dashGeo, M.lineWhite);
        dash.position.set(x, y, offset);
        group.add(dash);
      }
    }
  }
  return group;
}

// ── 工具：波形梁护栏 ──
function _makeGuardrail(length, height, sideOffset, postSpacing = 3) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const railH = 0.75;
  const postCount = Math.floor(length / postSpacing);
  const postGeo = new THREE.BoxGeometry(0.07, railH, 0.07);
  const z = sideOffset;

  for (let i = 0; i < postCount; i++) {
    const x = -length / 2 + i * postSpacing + postSpacing / 2;
    const post = new THREE.Mesh(postGeo, M.metalRail);
    post.position.set(x, height + railH / 2, z);
    post.castShadow = true;
    group.add(post);
  }

  const beamGeo = new THREE.BoxGeometry(length, 0.15, 0.05);
  for (const yOff of [railH - 0.08, railH - 0.4]) {
    const beam = new THREE.Mesh(beamGeo, M.metalRail);
    beam.position.set(0, height + yOff, z);
    beam.castShadow = true;
    group.add(beam);
  }

  for (let i = 0; i < postCount; i++) {
    const x = -length / 2 + i * postSpacing + postSpacing / 2;
    const block = new THREE.Mesh(
      new THREE.BoxGeometry(0.12, 0.25, 0.18), M.metalRail
    );
    const sideSign = sideOffset > 0 ? 1 : -1;
    block.position.set(x, height + railH - 0.2, z - sideSign * 0.1);
    group.add(block);
  }
  return group;
}

// ── 工具：地面高路灯（国道用，9m 高） ──
function _buildTallLamp(x, z, side) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const h = 9;

  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.22, 0.28, 0.3, 8), M.concreteDark
  );
  base.position.set(x, 0.15, z);
  base.castShadow = true; base.receiveShadow = true;
  group.add(base);

  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.07, 0.12, h, 8), M.pole
  );
  pole.position.set(x, 0.3 + h / 2, z);
  pole.castShadow = true;
  group.add(pole);

  const armLen = 2;
  const arm = new THREE.Mesh(
    new THREE.CylinderGeometry(0.04, 0.06, armLen, 8), M.pole
  );
  arm.position.set(x - side * armLen / 2, h - 0.4, z);
  arm.rotation.z = side * 0.15;
  arm.castShadow = true;
  group.add(arm);

  const lampHead = new THREE.Mesh(
    new THREE.BoxGeometry(0.9, 0.18, 0.4), M.pole
  );
  lampHead.position.set(x - side * armLen - side * 0.2, h - 0.6, z);
  lampHead.rotation.z = side * 0.1;
  lampHead.castShadow = true;
  group.add(lampHead);

  const glow = new THREE.Mesh(
    new THREE.PlaneGeometry(0.85, 0.35), M.lampGlow
  );
  glow.rotation.x = Math.PI / 2;
  glow.position.set(x - side * armLen - side * 0.2, h - 0.7, z);
  glow.rotation.z = side * 0.1;
  group.add(glow);

  return group;
}

// ── 工具：高架短路灯（3.5m 高） ──
function _buildShortLamp(x, z, side, baseH) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const h = 3.5;

  const pole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.05, 0.07, h, 8), M.pole
  );
  pole.position.set(x, baseH + h / 2, z);
  pole.castShadow = true;
  group.add(pole);

  const armLen = 1;
  const arm = new THREE.Mesh(
    new THREE.CylinderGeometry(0.03, 0.05, armLen, 6), M.pole
  );
  arm.position.set(x - side * armLen / 2, baseH + h - 0.2, z);
  arm.rotation.z = side * 0.15;
  arm.castShadow = true;
  group.add(arm);

  const lampHead = new THREE.Mesh(
    new THREE.BoxGeometry(0.6, 0.15, 0.25), M.pole
  );
  lampHead.position.set(x - side * armLen - side * 0.1, baseH + h - 0.35, z);
  lampHead.rotation.z = side * 0.1;
  lampHead.castShadow = true;
  group.add(lampHead);

  const glow = new THREE.Mesh(
    new THREE.PlaneGeometry(0.55, 0.22), M.lampGlow
  );
  glow.rotation.x = Math.PI / 2;
  glow.position.set(x - side * armLen - side * 0.1, baseH + h - 0.43, z);
  glow.rotation.z = side * 0.1;
  group.add(glow);

  return group;
}

// ── 高架（沿 X 轴长 200m，Y=7，Z=0） ──
function _buildElevatedHighway(parent, cx, cz, length, width, height, laneCount) {
  const M = _ensureMaterials();
  const group = new THREE.Group();

  // 路面
  const surf = new THREE.Mesh(
    new THREE.BoxGeometry(length, 0.25, width), M.roadDark
  );
  surf.position.set(cx, height + 0.125, cz);
  surf.castShadow = true; surf.receiveShadow = true;
  group.add(surf);

  const top = new THREE.Mesh(
    new THREE.PlaneGeometry(length - 0.2, width - 0.2), M.road
  );
  top.rotation.x = -Math.PI / 2;
  top.position.set(cx, height + 0.26, cz);
  group.add(top);

  const markings = _makeMarkings(length - 4, width - 0.5, laneCount, height + 0.26);
  markings.position.set(cx, 0, cz);
  group.add(markings);

  // 箱梁
  const deck = new THREE.Mesh(
    new THREE.BoxGeometry(length + 1, 0.6, width + 2), M.concreteDark
  );
  deck.position.set(cx, height - 0.1, cz);
  deck.castShadow = true; deck.receiveShadow = true;
  group.add(deck);

  // 双侧护栏
  group.add(_makeGuardrail(length - 4, height + 0.25, cz - (width / 2 + 0.8), 2.5));
  group.add(_makeGuardrail(length - 4, height + 0.25, cz + (width / 2 + 0.8), 2.5));

  // 隔音玻璃屏
  for (const side of [-1, 1]) {
    const barrier = new THREE.Mesh(
      new THREE.BoxGeometry(length - 10, 2, 0.04), M.glassBarrier
    );
    barrier.position.set(cx, height + 0.25 + 1.5, cz + side * (width / 2 + 1.2));
    group.add(barrier);
  }

  // 桥墩
  const pillarSpacing = 16;
  const pillarCount = Math.floor(length / pillarSpacing);
  for (let i = 0; i <= pillarCount; i++) {
    const px = -length / 2 + i * pillarSpacing;

    const cap = new THREE.Mesh(
      new THREE.BoxGeometry(3, 0.5, width + 4), M.concrete
    );
    cap.position.set(cx + px, height - 0.35, cz);
    cap.castShadow = true; cap.receiveShadow = true;
    group.add(cap);

    for (const side of [-1, 1]) {
      const pillar = new THREE.Mesh(
        new THREE.BoxGeometry(1.2, height - 0.9, 1.2), M.concrete
      );
      pillar.position.set(cx + px, (height - 0.9) / 2, cz + side * (width / 2 + 0.3));
      pillar.castShadow = true; pillar.receiveShadow = true;
      group.add(pillar);

      const line = new THREE.Mesh(
        new THREE.BoxGeometry(1.25, 0.05, 1.25), M.concreteDark
      );
      line.position.set(cx + px, 1.5, cz + side * (width / 2 + 0.3));
      group.add(line);

      const footing = new THREE.Mesh(
        new THREE.BoxGeometry(2, 0.3, 2), M.concreteDark
      );
      footing.position.set(cx + px, 0.15, cz + side * (width / 2 + 0.3));
      footing.receiveShadow = true;
      group.add(footing);
    }
  }

  // 端部挡墙
  for (const side of [-1, 1]) {
    const endX = cx + side * length / 2;
    const wall = new THREE.Mesh(
      new THREE.BoxGeometry(0.6, 1.2, width + 1.5), M.concreteDark
    );
    wall.position.set(endX - side * 0.3, height + 0.85, cz);
    wall.castShadow = true;
    group.add(wall);
  }

  // 路灯（间距 20m）
  const lampSpacing = 20;
  const lampCount = Math.floor((length - 10) / lampSpacing);
  for (let i = 0; i <= lampCount; i++) {
    const lx = -length / 2 + 5 + i * lampSpacing;
    for (const side of [-1, 1]) {
      group.add(_buildShortLamp(cx + lx, cz + side * (width / 2 + 1), side, height + 0.25));
    }
  }

  parent.add(group);
}

// ── 国道（地面 Y=0.4，Z=34） ──
function _buildNationalHighway(parent, cx, cz, length, width, laneCount) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const ROAD_Y = 0.40;
  const SUBBASE_H = 0.40;
  const subBaseW = width + 8;

  // 路基主体
  const subBase = new THREE.Mesh(
    new THREE.BoxGeometry(length, SUBBASE_H, subBaseW),
    new THREE.MeshStandardMaterial({ color: 0x8B7355, roughness: 1 })
  );
  subBase.position.set(cx, SUBBASE_H / 2, cz);
  subBase.receiveShadow = true;
  group.add(subBase);

  // 路肩
  const shoulderW = 2.0;
  for (const side of [-1, 1]) {
    const sh = new THREE.Mesh(
      new THREE.PlaneGeometry(length, shoulderW), M.dirt
    );
    sh.rotation.x = -Math.PI / 2;
    sh.position.set(cx, ROAD_Y + 0.02, cz + side * (width / 2 + shoulderW / 2));
    sh.receiveShadow = true;
    group.add(sh);
  }

  // 路面
  const road = new THREE.Mesh(
    new THREE.PlaneGeometry(length, width), M.road
  );
  road.rotation.x = -Math.PI / 2;
  road.position.set(cx, ROAD_Y + 0.04, cz);
  road.receiveShadow = true;
  group.add(road);

  // 路缘石
  const curbH = 0.15;
  const curbGeo = new THREE.BoxGeometry(length, curbH, 0.2);
  for (const side of [-1, 1]) {
    const curb = new THREE.Mesh(curbGeo, M.curb);
    curb.position.set(cx, ROAD_Y + curbH / 2, cz + side * (width / 2 + 0.1));
    curb.castShadow = true; curb.receiveShadow = true;
    group.add(curb);
  }

  // 标线
  const markings = _makeMarkings(length - 2, width - 0.3, laneCount, ROAD_Y + 0.04);
  markings.position.set(cx, 0, cz);
  group.add(markings);

  // 排水沟
  for (const side of [-1, 1]) {
    const ditch = new THREE.Mesh(
      new THREE.BoxGeometry(length, 0.2, 0.6),
      new THREE.MeshStandardMaterial({ color: 0x5A5A5A, roughness: 0.9 })
    );
    ditch.position.set(cx, -0.05, cz + side * (subBaseW / 2 + 2.0));
    ditch.receiveShadow = true;
    group.add(ditch);
  }

  // 波形梁护栏
  group.add(_makeGuardrail(length - 4, ROAD_Y, cz - (width / 2 + shoulderW + 0.4), 3));
  group.add(_makeGuardrail(length - 4, ROAD_Y, cz + (width / 2 + shoulderW + 0.4), 3));

  // 路灯（间距 25m）
  const lampSpacing = 25;
  const lampCount = Math.floor(length / lampSpacing);
  for (let i = 0; i <= lampCount; i++) {
    const x = -length / 2 + i * lampSpacing;
    for (const side of [-1, 1]) {
      group.add(_buildTallLamp(cx + x, cz + side * (width / 2 + shoulderW + 2.5), side));
    }
  }

  parent.add(group);
}

// ── 树木（简版） ──
function _makeTreeSimple(x, z, scale, group) {
  const M = _ensureMaterials();
  const tree = new THREE.Group();
  const trunkH = 2 * scale;
  const trunk = new THREE.Mesh(
    new THREE.CylinderGeometry(0.1 * scale, 0.18 * scale, trunkH, 5), M.treeTrunk
  );
  trunk.position.y = trunkH / 2;
  trunk.castShadow = true;
  tree.add(trunk);
  const leaf = new THREE.Mesh(
    new THREE.ConeGeometry(1.4 * scale, 2 * scale, 5), M.treeLeafDark
  );
  leaf.position.y = trunkH + scale;
  leaf.castShadow = true;
  tree.add(leaf);
  tree.position.set(x, 0, z);
  group.add(tree);
}

function _makeTree(x, z, scale, group) {
  const M = _ensureMaterials();
  const tree = new THREE.Group();
  const trunkH = 2 * scale;
  const trunk = new THREE.Mesh(
    new THREE.CylinderGeometry(0.12 * scale, 0.2 * scale, trunkH, 6), M.treeTrunk
  );
  trunk.position.y = trunkH / 2;
  trunk.castShadow = true;
  tree.add(trunk);
  const colors = [M.treeLeafDark, M.treeLeaf, M.treeLeafLight];
  const sizes = [1.5 * scale, 1.2 * scale, 0.85 * scale];
  const heights = [trunkH + 0.8 * scale, trunkH + 1.8 * scale, trunkH + 2.5 * scale];
  for (let i = 0; i < 3; i++) {
    const leaf = new THREE.Mesh(
      new THREE.ConeGeometry(sizes[i], 1.6 * scale, 7), colors[i]
    );
    leaf.position.y = heights[i];
    leaf.castShadow = true;
    tree.add(leaf);
  }
  tree.position.set(x, 0, z);
  tree.rotation.y = Math.random() * Math.PI * 2;
  group.add(tree);
}

// ── 灌木 ──
function _createBushes(parent, laneCount) {
  const M = _ensureMaterials();
  const w = laneCount * LANE_W;
  const elevatedSafe = w / 2 + 10;
  const nationalSafe = 34 + (w / 2 + 14);
  for (let i = 0; i < 30; i++) {
    const x = -95 + Math.random() * 190;
    const side = Math.random() < 0.5 ? -1 : 1;
    const z = side < 0 ? -(elevatedSafe + Math.random() * 8) : nationalSafe + Math.random() * 8;
    const s = 0.2 + Math.random() * 0.3;
    const bush = new THREE.Mesh(
      new THREE.SphereGeometry(s, 7, 5), M.treeLeaf
    );
    bush.position.set(x, s * 0.5, z);
    bush.scale.y = 0.6;
    bush.castShadow = true;
    parent.add(bush);
  }
}

// ── 树木：国道南侧 / 高架北侧 / 隔离带 / 远景 ──
function _createTrees(parent, laneCount) {
  const w = laneCount * LANE_W;
  const elevatedEdge = w / 2 + 8;
  const nationalOuter = 34 + (w / 2 + 12);
  const nationalInner = 34 - (w / 2 + 11);
  const elevatedInner = w / 2 + 8;

  // 国道南侧
  for (let i = 0; i < 35; i++) {
    const x = -95 + i * 5.5 + (Math.random() - 0.5) * 2;
    const z = nationalOuter + 5 + Math.random() * 10;
    const s = 0.6 + Math.random() * 0.5;
    _makeTree(x, z, s, parent);
  }
  // 高架北侧
  for (let i = 0; i < 35; i++) {
    const x = -95 + i * 5.5 + (Math.random() - 0.5) * 2;
    const z = -(elevatedEdge + 5) - Math.random() * 10;
    const s = 0.6 + Math.random() * 0.5;
    _makeTree(x, z, s, parent);
  }
  // 高架/国道之间隔离带
  const gapZ1 = elevatedInner;
  const gapZ2 = nationalInner;
  if (gapZ2 > gapZ1 + 4) {
    for (let i = 0; i < 50; i++) {
      const x = -95 + Math.random() * 190;
      const z = gapZ1 + Math.random() * (gapZ2 - gapZ1);
      if (Math.random() < 0.7) {
        const s = 0.15 + Math.random() * 0.2;
        const M = _ensureMaterials();
        const bush = new THREE.Mesh(
          new THREE.SphereGeometry(s, 6, 4), M.treeLeaf
        );
        bush.position.set(x, s * 0.4, z);
        bush.scale.y = 0.4;
        bush.castShadow = true;
        parent.add(bush);
      } else {
        const s = 0.2 + Math.random() * 0.2;
        _makeTree(x, z, s, parent);
      }
    }
  }
  // 远景树林
  for (let i = 0; i < 500; i++) {
    const x = -250 + Math.random() * 500;
    const r = Math.random();
    let z;
    if (r < 0.3) z = -(30 + Math.random() * 120);
    else if (r < 0.6) z = 55 + Math.random() * 130;
    else {
      z = -160 + Math.random() * 320;
      if (z > -15 && z < 50) {
        z = (Math.random() < 0.5) ? -15 - Math.random() * 15 : 50 + Math.random() * 15;
      }
    }
    const distFromCenter = Math.sqrt(x * x + (z - 17) * (z - 17));
    if (distFromCenter < 55) continue;
    const s = 0.7 + Math.random() * 0.7;
    _makeTreeSimple(x, z, s, parent);
  }
}

// ── 模块入口 ──
export function createViaductView(scene) {
  const group = new THREE.Group();
  scene.add(group);
  let built = false;

  /**
   * 重建场景
   * @param {object} opts
   * @param {number} opts.laneCount - 车道数（2 或 4，默认 4）
   * @param {number} opts.length    - 道路长度（默认 200m）
   * @param {boolean} opts.withEnvironment - 是否建树/灌木（默认 true）
   */
  function build(opts = {}) {
    // 清理
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.traverse) {
        c.traverse(o => {
          if (o.geometry) o.geometry.dispose();
        });
      }
    }

    const laneCount = opts.laneCount || 4;
    const length = opts.length || 200;
    const width = laneCount * LANE_W;
    const withEnv = opts.withEnvironment !== false;

    // 高架
    _buildElevatedHighway(group, 0, 0, length, width, 7, laneCount);
    // 国道
    _buildNationalHighway(group, 0, 34, length, width, laneCount);

    if (withEnv) {
      _createTrees(group, laneCount);
      _createBushes(group, laneCount);
    }

    built = true;
  }

  function getGroup() { return group; }
  function isBuilt() { return built; }

  /** 让 ViaductView 跟随 ego 视觉位置。ego 视觉 = simX % 200，wrap 时同步跳。
   *  group 整体平移到 (egoVisualX - 100, egoVisualX + 100) → ego 永远在 group 中点。
   *  副作用：整组跟着走，路上物体相对 ego 静止（车在拖世界），
   *  但能确保 wrap 那一帧不会闪烁。 */
  function followEgo(egoVisualX) {
    if (!built || egoVisualX == null) return;
    group.position.x = egoVisualX - 100;
  }

  /** 暴露 roadGroup 概念（用于 CameraRig clamp） */
  function getBBox() {
    if (!built) return null;
    const box = new THREE.Box3().setFromObject(group);
    return box;
  }

  return { build, getGroup, isBuilt, getBBox, followEgo };
}
