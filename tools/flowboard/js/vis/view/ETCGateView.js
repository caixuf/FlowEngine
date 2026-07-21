/**
 * ETCGateView.js — ETC 收费广场门架
 *
 * 从 utils.js _buildETCGate 搬迁：4 车道收费广场
 *   - 广场地面（深沥青）
 *   - 入口大型标志门架（跨全部车道，"ETC"发光字）
 *   - 4 个收费亭（黄黑警示岛 + 小屋 + 屋顶 + 窗 + 显示屏）
 *   - 4 套 ETC 门架（2 立柱 + 横梁 + 标志板 + 栏杆）
 *   - 4 个栏杆 boom：根据 entity.state 控制抬起角度
 *
 * update(store) 扫描 entities 里 type='etc_gate' 的实体，
 *   state='open'/'closed' 或 1/0 → boom.rotation.y 抬起/落下
 *
 * 单个 ETC 广场 mesh 内含全部 4 车道，position 取 entity.x/y。
 */

import { getStdMaterial, createEmissiveMaterial } from '../core/AssetFactory.js';
import { worldToThree } from '../math/Coord.js';

const N_BOOTHS = 4;
const LANE_W = 3.5;

export function createETCGateView(scene) {
  // entity id → { group, booms: [boom×4] }
  const pool = new Map();

  function _createETCGate() {
    const g = new THREE.Group();
    const totalWidth = N_BOOTHS * LANE_W;
    const halfW = totalWidth / 2;

    // ── 共享材质 ──
    const plazaMat    = getStdMaterial(0x1a1a1a, 0.92, 0.0);
    const islandMat   = createEmissiveMaterial(0xffcc00, 0.18);
    islandMat.color.setHex(0xffcc00); islandMat.emissive.setHex(0x332200);
    const boothMat    = getStdMaterial(0xe8e8ec, 0.55, 0.1);
    const boothRoofMat= createEmissiveMaterial(0x2255aa, 0.18);
    boothRoofMat.color.setHex(0x2255aa); boothRoofMat.emissive.setHex(0x113355);
    const glassMat    = createEmissiveMaterial(0x0a1a2a, 0.12);
    glassMat.color.setHex(0x0a1a2a); glassMat.emissive.setHex(0x081020);
    const poleMat     = getStdMaterial(0x445566, 0.35, 0.6);
    const signMat     = createEmissiveMaterial(0x2266aa, 0.55);
    signMat.color.setHex(0x2266aa); signMat.emissive.setHex(0x1166aa);
    const boomMat     = createEmissiveMaterial(0xffcc00, 0.35);
    boomMat.color.setHex(0xffcc00); boomMat.emissive.setHex(0x442200);
    const screenMat   = createEmissiveMaterial(0x00ff66, 0.9);
    const tipMat      = createEmissiveMaterial(0xff2222, 0.9);

    // ── 广场地面 ──
    const plaza = new THREE.Mesh(
      new THREE.PlaneGeometry(10.0, totalWidth + 2.5),
      plazaMat
    );
    plaza.rotation.x = -Math.PI / 2;
    plaza.position.y = 0.02;
    plaza.receiveShadow = true;
    g.add(plaza);

    // ── 入口大型标志门架 ──
    const megaPoleGeo = new THREE.CylinderGeometry(0.16, 0.20, 7.0, 12);
    const megaPole1 = new THREE.Mesh(megaPoleGeo, poleMat);
    megaPole1.position.set(-3.5, 3.5, -halfW - 0.7);
    megaPole1.castShadow = true;
    g.add(megaPole1);
    const megaPole2 = new THREE.Mesh(megaPoleGeo, poleMat);
    megaPole2.position.set(-3.5, 3.5, halfW + 0.7);
    megaPole2.castShadow = true;
    g.add(megaPole2);
    const megaBeam = new THREE.Mesh(
      new THREE.BoxGeometry(0.22, 0.22, totalWidth + 1.6),
      poleMat
    );
    megaBeam.position.set(-3.5, 6.85, 0);
    megaBeam.castShadow = true;
    g.add(megaBeam);
    const megaSign = new THREE.Mesh(
      new THREE.BoxGeometry(0.10, 0.85, totalWidth * 0.7),
      signMat
    );
    megaSign.position.set(-3.38, 6.85, 0);
    g.add(megaSign);
    const etcText = new THREE.Mesh(
      new THREE.BoxGeometry(0.04, 0.18, totalWidth * 0.45),
      screenMat
    );
    etcText.position.set(-3.30, 6.40, 0);
    g.add(etcText);

    // ── 共享几何体 ──
    const poleGeo   = new THREE.CylinderGeometry(0.08, 0.10, 5.0, 10);
    const beamGeo   = new THREE.BoxGeometry(0.12, 0.12, LANE_W - 0.15);
    const signGeo   = new THREE.BoxGeometry(0.06, 0.32, (LANE_W - 0.15) * 0.75);
    const islandGeo = new THREE.BoxGeometry(2.0, 0.20, 0.45);
    const boothGeo  = new THREE.BoxGeometry(1.7, 2.3, 1.05);
    const roofGeo   = new THREE.BoxGeometry(1.85, 0.12, 1.2);
    const winGeo    = new THREE.BoxGeometry(0.04, 0.7, 0.85);
    const screenGeo = new THREE.BoxGeometry(0.04, 0.18, 0.45);

    const booms = [];

    // ── 4 个并列收费口 ──
    for (let i = 0; i < N_BOOTHS; i++) {
      const zCenter = (i - (N_BOOTHS - 1) / 2) * LANE_W;
      const poleZ1 = zCenter - LANE_W / 2 + 0.15;
      const poleZ2 = zCenter + LANE_W / 2 - 0.15;
      const islandZ = zCenter + LANE_W / 2 - 0.25;

      // 收费岛
      const island = new THREE.Mesh(islandGeo, islandMat);
      island.position.set(0, 0.10, islandZ);
      island.castShadow = true; island.receiveShadow = true;
      g.add(island);

      // 收费亭
      const booth = new THREE.Mesh(boothGeo, boothMat);
      booth.position.set(0, 1.35, islandZ);
      booth.castShadow = true;
      g.add(booth);
      const roof = new THREE.Mesh(roofGeo, boothRoofMat);
      roof.position.set(0, 2.56, islandZ);
      g.add(roof);
      const win = new THREE.Mesh(winGeo, glassMat);
      win.position.set(-0.86, 1.5, islandZ);
      g.add(win);
      const screen = new THREE.Mesh(screenGeo, screenMat);
      screen.position.set(-0.88, 1.05, islandZ);
      g.add(screen);

      // ETC 门架
      const pole1 = new THREE.Mesh(poleGeo, poleMat);
      pole1.position.set(0, 2.5, poleZ1);
      pole1.castShadow = true;
      g.add(pole1);
      const pole2 = new THREE.Mesh(poleGeo, poleMat);
      pole2.position.set(0, 2.5, poleZ2);
      pole2.castShadow = true;
      g.add(pole2);
      const beam = new THREE.Mesh(beamGeo, poleMat);
      beam.position.set(0, 4.85, zCenter);
      beam.castShadow = true;
      g.add(beam);
      const laneSign = new THREE.Mesh(signGeo, signMat);
      laneSign.position.set(0.09, 4.85, zCenter);
      g.add(laneSign);

      // 栏杆 boom（pivot 在 pole1 端，绕 Y 抬起）
      const boomLen = LANE_W - 0.35;
      const boomGeo = new THREE.BoxGeometry(0.07, 0.07, boomLen);
      boomGeo.translate(0, 0, boomLen / 2);
      const boom = new THREE.Mesh(boomGeo, boomMat);
      boom.position.set(0, 4.45, poleZ1);
      boom.castShadow = true;
      g.add(boom);
      booms.push(boom);

      // 末端红色警示灯
      const tipLight = new THREE.Mesh(
        new THREE.SphereGeometry(0.06, 8, 6),
        tipMat
      );
      tipLight.position.set(0, 4.45, poleZ1 + boomLen);
      g.add(tipLight);
    }

    g.userData.booms = booms;
    scene.add(g);
    return { group: g, booms };
  }

  /** boom 抬起角度：state='open'/1 → 60° 抬起；'closed'/0 → 0° 落下 */
  function _setBoomState(entry, state) {
    const open = state === 'open' || state === 1 || state === true;
    const targetAngle = open ? -Math.PI / 3 : 0;  // 抬起 60°（绕 X 轴）
    for (const boom of entry.booms) {
      boom.rotation.x = targetAngle;
    }
  }

  /** 主更新入口：扫描 entities 找 type='etc_gate' */
  function update(store) {
    const all = (store.entities || []).filter(e => e.type === 'etc_gate');

    const aliveIds = new Set(all.map(e => e.id));
    for (const [id, entry] of pool.entries()) {
      if (!aliveIds.has(id)) {
        scene.remove(entry.group);
        pool.delete(id);
      }
    }

    for (const ent of all) {
      let entry = pool.get(ent.id);
      if (!entry) {
        entry = _createETCGate();
        pool.set(ent.id, entry);
      }
      entry.group.position.set(...worldToThree(ent.x || 0, ent.y || 0, 0));
      _setBoomState(entry, ent.state);
    }
  }

  function clear() {
    for (const [, entry] of pool) scene.remove(entry.group);
    pool.clear();
  }

  return { update, clear };
}
