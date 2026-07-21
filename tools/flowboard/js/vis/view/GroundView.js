/**
 * GroundView.js — 草地/地面
 * 纯色 #3e6b34（深饱和草绿），无 canvas 纹理。
 * 位置 y=-0.05（路面上方 0.10m，防 z-fight）。
 */

const GRASS_COLOR = 0x3e6b34;

export function createGroundView(scene) {
  let mesh = null;

  function build(size = 50000) {
      if (mesh) {
        scene.remove(mesh);
        mesh.geometry.dispose();
        mesh.material.dispose();
        mesh = null;
      }
      if (size <= 0) return;  // size=0 = 不建草地（场景自带）
      const geo = new THREE.PlaneGeometry(size, size);
      const mat = new THREE.MeshStandardMaterial({
        color: GRASS_COLOR, roughness: 0.95, metalness: 0.0
      });
      mesh = new THREE.Mesh(geo, mat);
      mesh.rotation.x = -Math.PI / 2;
      mesh.position.y = -0.05;
      mesh.receiveShadow = true;
      scene.add(mesh);
    }

  function getMesh() { return mesh; }

  return { build, getMesh };
}
