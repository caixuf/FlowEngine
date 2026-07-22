/**
 * ViaductView.js — 高架 + 平行国道复合场景（优化版）
 *
 * 性能优化要点：
 *   1. InstancedMesh — 树木、路灯、护栏立柱
 *   2. mergeGeometries — 标线（虚线大量小 mesh）
 *   3. MeshLambertMaterial — 静态物体（草地、护栏、桥墩等）
 *   4. 减少 castShadow — 仅路面/车辆投阴影，树木不投
 *   5. 材质缓存 + 统一 dispose
 */

import { mergeGeometries } from '../math/GeometryMerge.js';

let _mats = null;
let _geos = null;

function _ensureMaterials() {
  if (_mats) return _mats;
  _mats = {
    road: new THREE.MeshStandardMaterial({ color: 0x2a2a2a, roughness: 0.85, metalness: 0.02 }),
    roadDark: new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.9, metalness: 0.01 }),
    lineWhite: new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.6 }),
    lineYellow: new THREE.MeshStandardMaterial({ color: 0xffd700, roughness: 0.6 }),
    concrete: new THREE.MeshLambertMaterial({ color: 0x9a9a9a }),
    concreteDark: new THREE.MeshLambertMaterial({ color: 0x7a7a7a }),
    metalRail: new THREE.MeshLambertMaterial({ color: 0xa8a8a8 }),
    grass: new THREE.MeshLambertMaterial({ color: 0x4a7a35 }),
    pole: new THREE.MeshLambertMaterial({ color: 0x3a3a3a }),
    lampGlow: new THREE.MeshStandardMaterial({ color: 0xfff0c0, emissive: 0xffcc66, emissiveIntensity: 0.8 }),
    treeTrunk: new THREE.MeshLambertMaterial({ color: 0x5c4033 }),
    treeLeaf: new THREE.MeshLambertMaterial({ color: 0x3a8032 }),
    treeLeafDark: new THREE.MeshLambertMaterial({ color: 0x2d5e27 }),
    treeLeafLight: new THREE.MeshLambertMaterial({ color: 0x4a9a40 }),
    dirt: new THREE.MeshLambertMaterial({ color: 0x8b7355 }),
    glassBarrier: new THREE.MeshStandardMaterial({
      color: 0x88bbee, transparent: true, opacity: 0.3, roughness: 0.1, metalness: 0.5
    }),
    curb: new THREE.MeshLambertMaterial({ color: 0x888888 }),
  };
  return _mats;
}

function _ensureGeos() {
  if (_geos) return _geos;
  _geos = {
    shortLamp: _buildShortLampGeo(),
    tallLamp: _buildTallLampGeo(),
    treeSimple: _buildSimpleTreeGeo(),
    treeDetail: _buildDetailTreeGeo(),
    bush: new THREE.SphereGeometry(0.25, 6, 4),
    guardrailPost: new THREE.BoxGeometry(0.07, 0.75, 0.07),
  };
  return _geos;
}

function _buildShortLampGeo() {
  const group = new THREE.Group();
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.07, 3.5, 6));
  pole.position.y = 1.75;
  group.add(pole);
  const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.05, 1, 6));
  arm.position.set(-0.5, 3.3, 0);
  arm.rotation.z = 0.15;
  group.add(arm);
  const lampHead = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.15, 0.25));
  lampHead.position.set(-1.1, 3.15, 0);
  lampHead.rotation.z = 0.1;
  group.add(lampHead);
  const glow = new THREE.Mesh(new THREE.PlaneGeometry(0.55, 0.22));
  glow.rotation.x = Math.PI / 2;
  glow.position.set(-1.1, 3.07, 0);
  glow.rotation.z = 0.1;
  group.add(glow);
  return group;
}

function _buildTallLampGeo() {
  const group = new THREE.Group();
  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.28, 0.3, 8));
  base.position.y = 0.15;
  group.add(base);
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.12, 9, 8));
  pole.position.y = 4.8;
  group.add(pole);
  const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.06, 2, 8));
  arm.position.set(-1, 8.6, 0);
  arm.rotation.z = 0.15;
  group.add(arm);
  const lampHead = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.18, 0.4));
  lampHead.position.set(-2.2, 8.4, 0);
  lampHead.rotation.z = 0.1;
  group.add(lampHead);
  const glow = new THREE.Mesh(new THREE.PlaneGeometry(0.85, 0.35));
  glow.rotation.x = Math.PI / 2;
  glow.position.set(-2.2, 8.3, 0);
  glow.rotation.z = 0.1;
  group.add(glow);
  return group;
}

function _buildSimpleTreeGeo() {
  const group = new THREE.Group();
  const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.18, 2, 5));
  trunk.position.y = 1;
  group.add(trunk);
  const leaf = new THREE.Mesh(new THREE.ConeGeometry(1.4, 2, 5));
  leaf.position.y = 2;
  group.add(leaf);
  return group;
}

function _buildDetailTreeGeo() {
  const group = new THREE.Group();
  const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.2, 2, 6));
  trunk.position.y = 1;
  group.add(trunk);
  const sizes = [1.5, 1.2, 0.85];
  const heights = [1.8, 2.8, 3.5];
  for (let i = 0; i < 3; i++) {
    const leaf = new THREE.Mesh(new THREE.ConeGeometry(sizes[i], 1.6, 7));
    leaf.position.y = heights[i];
    group.add(leaf);
  }
  return group;
}

function _makeLineGeo(length, width) {
  const geo = new THREE.PlaneGeometry(length, width);
  geo.rotateX(-Math.PI / 2);
  return geo;
}

function _makeMarkings(length, width, lanes, height) {
  const M = _ensureMaterials();
  const whiteGeos = [];
  const yellowGeos = [];
  const laneW = width / lanes;
  const perSide = lanes / 2;

  const edgeGeo = _makeLineGeo(length, 0.12);
  whiteGeos.push(edgeGeo.clone());
  whiteGeos.push(edgeGeo.clone());

  if (lanes === 2) {
    yellowGeos.push(_makeLineGeo(length, 0.12));
  }

  if (lanes >= 4) {
    const yGeo = _makeLineGeo(length, 0.15);
    yellowGeos.push(yGeo.clone());
    yellowGeos.push(yGeo.clone());
  }

  const dashLen = 4, dashGap = 6;
  const dashCount = Math.floor(length / (dashLen + dashGap));
  const dashGeo = _makeLineGeo(dashLen, 0.1);
  for (let side = -1; side <= 1; side += 2) {
    for (let l = 1; l < perSide; l++) {
      const offset = side * laneW * l;
      for (let i = 0; i < dashCount; i++) {
        const x = -length / 2 + i * (dashLen + dashGap) + dashLen / 2;
        const g = dashGeo.clone();
        g.translate(x, height + 0.02, offset);
        whiteGeos.push(g);
      }
    }
  }

  const group = new THREE.Group();
  const y = height + 0.02;

  const whiteMesh = new THREE.Mesh(mergeGeometries(whiteGeos), M.lineWhite);
  whiteMesh.position.y = y;
  group.add(whiteMesh);

  if (yellowGeos.length) {
    const yellowMesh = new THREE.Mesh(mergeGeometries(yellowGeos), M.lineYellow);
    yellowMesh.position.y = y;
    yellowMesh.position.z = lanes >= 4 ? 0.2 : 0;
    group.add(yellowMesh);
    if (lanes >= 4) {
      const yellowMesh2 = new THREE.Mesh(mergeGeometries(yellowGeos.map(g => g.clone())), M.lineYellow);
      yellowMesh2.position.y = y;
      yellowMesh2.position.z = -0.2;
      group.add(yellowMesh2);
    }
  }

  return group;
}

function _makeGuardrail(length, height, sideOffset, postSpacing = 3) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const railH = 0.75;
  const z = sideOffset;

  const beamGeo = new THREE.BoxGeometry(length, 0.15, 0.05);
  for (const yOff of [railH - 0.08, railH - 0.4]) {
    const beam = new THREE.Mesh(beamGeo, M.metalRail);
    beam.position.set(0, height + yOff, z);
    beam.castShadow = true;
    group.add(beam);
  }

  const postCount = Math.floor(length / postSpacing);
  const postGeo = new THREE.BoxGeometry(0.07, railH, 0.07);
  const postMat = M.metalRail;
  const postPositions = [];
  for (let i = 0; i < postCount; i++) {
    const x = -length / 2 + i * postSpacing + postSpacing / 2;
    postPositions.push({ x, y: height + railH / 2, z });
  }

  if (postPositions.length > 0) {
    const instanced = new THREE.InstancedMesh(postGeo, postMat, postPositions.length);
    instanced.castShadow = true;
    const dummy = new THREE.Object3D();
    for (let i = 0; i < postPositions.length; i++) {
      dummy.position.set(postPositions[i].x, postPositions[i].y, postPositions[i].z);
      dummy.updateMatrix();
      instanced.setMatrixAt(i, dummy.matrix);
    }
    group.add(instanced);
  }

  return group;
}

function _buildElevatedHighway(parent, cx, cz, length, width, height, laneCount) {
  const M = _ensureMaterials();
  const group = new THREE.Group();

  const surf = new THREE.Mesh(new THREE.BoxGeometry(length, 0.25, width), M.roadDark);
  surf.position.set(cx, height + 0.125, cz);
  surf.castShadow = true; surf.receiveShadow = true;
  group.add(surf);

  const top = new THREE.Mesh(new THREE.PlaneGeometry(length - 0.2, width - 0.2), M.road);
  top.rotation.x = -Math.PI / 2;
  top.position.set(cx, height + 0.26, cz);
  group.add(top);

  const markings = _makeMarkings(length - 4, width - 0.5, laneCount, height + 0.26);
  markings.position.set(cx, 0, cz);
  group.add(markings);

  const deck = new THREE.Mesh(new THREE.BoxGeometry(length + 1, 0.6, width + 2), M.concreteDark);
  deck.position.set(cx, height - 0.1, cz);
  deck.castShadow = true; deck.receiveShadow = true;
  group.add(deck);

  group.add(_makeGuardrail(length - 4, height + 0.25, cz - (width / 2 + 0.8), 2.5));
  group.add(_makeGuardrail(length - 4, height + 0.25, cz + (width / 2 + 0.8), 2.5));

  for (const side of [-1, 1]) {
    const barrier = new THREE.Mesh(new THREE.BoxGeometry(length - 10, 2, 0.04), M.glassBarrier);
    barrier.position.set(cx, height + 0.25 + 1.5, cz + side * (width / 2 + 1.2));
    group.add(barrier);
  }

  const pillarSpacing = 16;
  const pillarCount = Math.floor(length / pillarSpacing);
  for (let i = 0; i <= pillarCount; i++) {
    const px = -length / 2 + i * pillarSpacing;

    const cap = new THREE.Mesh(new THREE.BoxGeometry(3, 0.5, width + 4), M.concrete);
    cap.position.set(cx + px, height - 0.35, cz);
    cap.castShadow = true; cap.receiveShadow = true;
    group.add(cap);

    for (const side of [-1, 1]) {
      const pillar = new THREE.Mesh(new THREE.BoxGeometry(1.2, height - 0.9, 1.2), M.concrete);
      pillar.position.set(cx + px, (height - 0.9) / 2, cz + side * (width / 2 + 0.3));
      pillar.castShadow = true; pillar.receiveShadow = true;
      group.add(pillar);

      const line = new THREE.Mesh(new THREE.BoxGeometry(1.25, 0.05, 1.25), M.concreteDark);
      line.position.set(cx + px, 1.5, cz + side * (width / 2 + 0.3));
      group.add(line);

      const footing = new THREE.Mesh(new THREE.BoxGeometry(2, 0.3, 2), M.concreteDark);
      footing.position.set(cx + px, 0.15, cz + side * (width / 2 + 0.3));
      footing.receiveShadow = true;
      group.add(footing);
    }
  }

  for (const side of [-1, 1]) {
    const endX = cx + side * length / 2;
    const wall = new THREE.Mesh(new THREE.BoxGeometry(0.6, 1.2, width + 1.5), M.concreteDark);
    wall.position.set(endX - side * 0.3, height + 0.85, cz);
    wall.castShadow = true;
    group.add(wall);
  }

  const geo = _ensureGeos();
  const lampSpacing = 20;
  const lampCount = Math.floor((length - 10) / lampSpacing);
  const lampPositions = [];
  for (let i = 0; i <= lampCount; i++) {
    const lx = -length / 2 + 5 + i * lampSpacing;
    for (const side of [-1, 1]) {
      lampPositions.push({
        x: cx + lx, y: height + 0.25, z: cz + side * (width / 2 + 1),
        side
      });
    }
  }

  if (lampPositions.length > 0) {
    const instanced = new THREE.InstancedMesh(geo.shortLamp.children[0].geometry, M.pole, lampPositions.length);
    instanced.castShadow = true;
    const dummy = new THREE.Object3D();
    for (let i = 0; i < lampPositions.length; i++) {
      const p = lampPositions[i];
      dummy.position.set(p.x, p.y, p.z);
      dummy.scale.x = p.side;
      dummy.updateMatrix();
      instanced.setMatrixAt(i, dummy.matrix);
    }
    group.add(instanced);
  }

  parent.add(group);
}

function _buildNationalHighway(parent, cx, cz, length, width, laneCount) {
  const M = _ensureMaterials();
  const group = new THREE.Group();
  const ROAD_Y = 0.40;
  const SUBBASE_H = 0.40;
  const subBaseW = width + 8;

  const subBase = new THREE.Mesh(new THREE.BoxGeometry(length, SUBBASE_H, subBaseW), M.dirt);
  subBase.position.set(cx, SUBBASE_H / 2, cz);
  subBase.receiveShadow = true;
  group.add(subBase);

  const shoulderW = 2.0;
  for (const side of [-1, 1]) {
    const sh = new THREE.Mesh(new THREE.PlaneGeometry(length, shoulderW), M.dirt);
    sh.rotation.x = -Math.PI / 2;
    sh.position.set(cx, ROAD_Y + 0.02, cz + side * (width / 2 + shoulderW / 2));
    sh.receiveShadow = true;
    group.add(sh);
  }

  const road = new THREE.Mesh(new THREE.PlaneGeometry(length, width), M.road);
  road.rotation.x = -Math.PI / 2;
  road.position.set(cx, ROAD_Y + 0.04, cz);
  road.receiveShadow = true;
  group.add(road);

  const curbH = 0.15;
  const curbGeo = new THREE.BoxGeometry(length, curbH, 0.2);
  for (const side of [-1, 1]) {
    const curb = new THREE.Mesh(curbGeo, M.curb);
    curb.position.set(cx, ROAD_Y + curbH / 2, cz + side * (width / 2 + 0.1));
    curb.castShadow = true; curb.receiveShadow = true;
    group.add(curb);
  }

  const markings = _makeMarkings(length - 2, width - 0.3, laneCount, ROAD_Y + 0.04);
  markings.position.set(cx, 0, cz);
  group.add(markings);

  for (const side of [-1, 1]) {
    const ditch = new THREE.Mesh(new THREE.BoxGeometry(length, 0.2, 0.6), M.concreteDark);
    ditch.position.set(cx, -0.05, cz + side * (subBaseW / 2 + 2.0));
    ditch.receiveShadow = true;
    group.add(ditch);
  }

  group.add(_makeGuardrail(length - 4, ROAD_Y, cz - (width / 2 + shoulderW + 0.4), 3));
  group.add(_makeGuardrail(length - 4, ROAD_Y, cz + (width / 2 + shoulderW + 0.4), 3));

  const geo = _ensureGeos();
  const lampSpacing = 25;
  const lampCount = Math.floor(length / lampSpacing);
  const lampPositions = [];
  for (let i = 0; i <= lampCount; i++) {
    const x = -length / 2 + i * lampSpacing;
    for (const side of [-1, 1]) {
      lampPositions.push({
        x: cx + x, y: 0, z: cz + side * (width / 2 + shoulderW + 2.5),
        side
      });
    }
  }

  if (lampPositions.length > 0) {
    const instanced = new THREE.InstancedMesh(geo.tallLamp.children[0].geometry, M.pole, lampPositions.length);
    instanced.castShadow = true;
    const dummy = new THREE.Object3D();
    for (let i = 0; i < lampPositions.length; i++) {
      const p = lampPositions[i];
      dummy.position.set(p.x, p.y, p.z);
      dummy.scale.x = p.side;
      dummy.updateMatrix();
      instanced.setMatrixAt(i, dummy.matrix);
    }
    group.add(instanced);
  }

  parent.add(group);
}

function _createTrees(parent, laneCount, laneWidth) {
  const M = _ensureMaterials();
  const geo = _ensureGeos();
  const w = laneCount * laneWidth;
  const elevatedEdge = w / 2 + 8;
  const nationalOuter = -34 - (w / 2 + 12);
  const nationalInner = -34 + (w / 2 + 11);
  const elevatedInner = w / 2 + 8;

  const detailTreePositions = [];
  const bushPositions = [];
  const simpleTreePositions = [];

  for (let i = 0; i < 35; i++) {
    const x = -95 + i * 5.5 + (Math.random() - 0.5) * 2;
    const z = nationalOuter + 5 + Math.random() * 10;
    detailTreePositions.push({ x, z, scale: 0.6 + Math.random() * 0.5 });
  }

  for (let i = 0; i < 35; i++) {
    const x = -95 + i * 5.5 + (Math.random() - 0.5) * 2;
    const z = -(elevatedEdge + 5) - Math.random() * 10;
    detailTreePositions.push({ x, z, scale: 0.6 + Math.random() * 0.5 });
  }

  const gapZ1 = elevatedInner;
  const gapZ2 = nationalInner;
  const gapStart = Math.min(gapZ1, gapZ2);
  const gapEnd   = Math.max(gapZ1, gapZ2);
  if (gapEnd > gapStart + 4) {
    for (let i = 0; i < 50; i++) {
      const x = -95 + Math.random() * 190;
      const z = gapStart + Math.random() * (gapEnd - gapStart);
      if (Math.random() < 0.7) {
        bushPositions.push({ x, z, scale: 0.15 + Math.random() * 0.2 });
      } else {
        detailTreePositions.push({ x, z, scale: 0.2 + Math.random() * 0.2 });
      }
    }
  }

  for (let i = 0; i < 100; i++) {
    const x = -250 + Math.random() * 500;
    const r = Math.random();
    let z;
    if (r < 0.3) z = -(30 + Math.random() * 120);
    else if (r < 0.6) z = 55 + Math.random() * 130;
    else {
      z = -160 + Math.random() * 320;
      if (z > -50 && z < 15) {
        z = (Math.random() < 0.5) ? -50 - Math.random() * 15 : 15 + Math.random() * 15;
      }
    }
    const distFromCenter = Math.sqrt(x * x + (z + 17) * (z + 17));
    if (distFromCenter < 55) continue;
    simpleTreePositions.push({ x, z, scale: 0.7 + Math.random() * 0.7 });
  }

  if (detailTreePositions.length > 0) {
    const trunkGeo = new THREE.CylinderGeometry(0.12, 0.2, 2, 6);
    const leafGeo = new THREE.ConeGeometry(1.5, 1.6, 7);
    const leafGeo2 = new THREE.ConeGeometry(1.2, 1.6, 7);
    const leafGeo3 = new THREE.ConeGeometry(0.85, 1.6, 7);

    const trunkInst = new THREE.InstancedMesh(trunkGeo, M.treeTrunk, detailTreePositions.length);
    const leafInst = new THREE.InstancedMesh(leafGeo, M.treeLeafDark, detailTreePositions.length);
    const leafInst2 = new THREE.InstancedMesh(leafGeo2, M.treeLeaf, detailTreePositions.length);
    const leafInst3 = new THREE.InstancedMesh(leafGeo3, M.treeLeafLight, detailTreePositions.length);

    const dummy = new THREE.Object3D();
    for (let i = 0; i < detailTreePositions.length; i++) {
      const p = detailTreePositions[i];
      const s = p.scale;

      dummy.position.set(p.x, s, p.z);
      dummy.scale.set(s, s, s);
      dummy.updateMatrix();
      trunkInst.setMatrixAt(i, dummy.matrix);

      dummy.position.y = s + 0.8 * s;
      dummy.scale.set(1.5 * s, s, 1.5 * s);
      dummy.updateMatrix();
      leafInst.setMatrixAt(i, dummy.matrix);

      dummy.position.y = s + 1.8 * s;
      dummy.scale.set(1.2 * s, s, 1.2 * s);
      dummy.updateMatrix();
      leafInst2.setMatrixAt(i, dummy.matrix);

      dummy.position.y = s + 2.5 * s;
      dummy.scale.set(0.85 * s, s, 0.85 * s);
      dummy.updateMatrix();
      leafInst3.setMatrixAt(i, dummy.matrix);
    }

    parent.add(trunkInst);
    parent.add(leafInst);
    parent.add(leafInst2);
    parent.add(leafInst3);
  }

  if (bushPositions.length > 0) {
    const instanced = new THREE.InstancedMesh(geo.bush, M.treeLeaf, bushPositions.length);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < bushPositions.length; i++) {
      const p = bushPositions[i];
      dummy.position.set(p.x, p.scale * 0.4, p.z);
      dummy.scale.set(p.scale, p.scale * 0.4, p.scale);
      dummy.updateMatrix();
      instanced.setMatrixAt(i, dummy.matrix);
    }
    parent.add(instanced);
  }

  if (simpleTreePositions.length > 0) {
    const trunkGeo = new THREE.CylinderGeometry(0.1, 0.18, 2, 5);
    const leafGeo = new THREE.ConeGeometry(1.4, 2, 5);

    const trunkInst = new THREE.InstancedMesh(trunkGeo, M.treeTrunk, simpleTreePositions.length);
    const leafInst = new THREE.InstancedMesh(leafGeo, M.treeLeafDark, simpleTreePositions.length);

    const dummy = new THREE.Object3D();
    for (let i = 0; i < simpleTreePositions.length; i++) {
      const p = simpleTreePositions[i];
      const s = p.scale;

      dummy.position.set(p.x, s, p.z);
      dummy.scale.set(s, s, s);
      dummy.updateMatrix();
      trunkInst.setMatrixAt(i, dummy.matrix);

      dummy.position.y = s + s;
      dummy.scale.set(1.4 * s, s, 1.4 * s);
      dummy.updateMatrix();
      leafInst.setMatrixAt(i, dummy.matrix);
    }

    parent.add(trunkInst);
    parent.add(leafInst);
  }
}

function _createBushes(parent, laneCount, laneWidth) {
  const M = _ensureMaterials();
  const w = laneCount * laneWidth;
  const elevatedSafe = w / 2 + 10;
  const nationalSafe = -34 - (w / 2 + 14);

  const bushPositions = [];
  for (let i = 0; i < 20; i++) {
    const x = -95 + Math.random() * 190;
    const side = Math.random() < 0.5 ? -1 : 1;
    const z = side < 0 ? -(elevatedSafe + Math.random() * 8) : nationalSafe - Math.random() * 8;
    bushPositions.push({ x, z, scale: 0.2 + Math.random() * 0.3 });
  }

  if (bushPositions.length > 0) {
    const geo = new THREE.SphereGeometry(0.25, 7, 5);
    const instanced = new THREE.InstancedMesh(geo, M.treeLeaf, bushPositions.length);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < bushPositions.length; i++) {
      const p = bushPositions[i];
      dummy.position.set(p.x, p.scale * 0.5, p.z);
      dummy.scale.set(p.scale, p.scale * 0.6, p.scale);
      dummy.updateMatrix();
      instanced.setMatrixAt(i, dummy.matrix);
    }
    parent.add(instanced);
  }
}

export function createViaductView(scene) {
  const group = new THREE.Group();
  scene.add(group);
  let built = false;

  function build(opts = {}) {
    while (group.children.length) {
      const c = group.children[0];
      group.remove(c);
      if (c.traverse) {
        c.traverse(o => {
          if (o.geometry) o.geometry.dispose();
          if (o.material) {
            if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
            else o.material.dispose();
          }
        });
      }
    }

    const laneCount = opts.laneCount || 4;
    const laneWidth = opts.laneWidth || 3.5;
    const length = opts.length || 200;
    const width = laneCount * laneWidth;
    const withEnv = opts.withEnvironment !== false;

    _buildElevatedHighway(group, 0, 0, length, width, 7, laneCount);
    _buildNationalHighway(group, 0, -34, length, width, laneCount);

    if (withEnv) {
      _createTrees(group, laneCount, laneWidth);
      _createBushes(group, laneCount, laneWidth);
    }

    built = true;
  }

  function getGroup() { return group; }
  function isBuilt() { return built; }

  /* followEgo(centerX) — 把整段高架组的中心对齐到 centerX。
   * SceneDirector 在高架模式每帧调用，参数为
   *   wrapOffset + visLength / 2
   * 即"当前 wrap 周期的中点"。高架组以 cx=0 为中心建造，
   * 故 group.position.x = centerX 时高架段恰好覆盖
   *   [wrapOffset, wrapOffset + visLength]
   * ego 的 simX 总落在该区间内（视觉位置 = simX % visLength
   * 落在 [0, visLength)，物理位置仍在 [wrapOffset, wrapOffset+visLength)），
   * 因此 ego 永远在高架段上，wrap 边界不再出现"驶出高架 100m"的接缝。
   *
   * 历史 -100 常数是个谜之偏移，会把高架组往后挪 100m，导致
   * ego 接近 wrap 周期末尾（simX 接近 wrapOffset+visLength）时
   * 已驶出高架末段约 100m，每 500m 跳一次可见接缝。 */
  function followEgo(centerX) {
    if (!built || centerX == null) return;
    group.position.x = centerX;
  }

  function getBBox() {
    if (!built) return null;
    const box = new THREE.Box3().setFromObject(group);
    return box;
  }

  return { build, getGroup, isBuilt, getBBox, followEgo };
}