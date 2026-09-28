// Relative-wind arrows drawn in the aircraft (CAD) frame: the analysis flies along CAD +X,
// so air arrives from +X toward −X. The label follows the arrows on screen.
import * as THREE from 'three';
import { disposeTree } from './scene.js';

export function createFlowIndicator(scene, viewport) {
  const group = new THREE.Group();
  group.visible = false;
  scene.add(group);
  const label = document.createElement('div');
  label.id = 'flow-label';
  label.hidden = true;
  viewport.append(label);
  const labelLocal = new THREE.Vector3();
  let asset = null;

  function build(a) {
    while (group.children.length) {
      const o = group.children[0];
      group.remove(o);
      disposeTree(o);
    }
    const box = new THREE.Box3(),
      v = new THREE.Vector3();
    for (const p of a.parts)
      for (let i = 0; i < p.positions.length; i += 3)
        box.expandByPoint(v.set(p.positions[i], p.positions[i + 1], p.positions[i + 2]));
    const size = box.getSize(new THREE.Vector3()),
      c = box.getCenter(new THREE.Vector3()),
      L = Math.max(size.x, size.y, size.z, 0.05),
      h = Math.max(size.z, L * 0.15),
      len = L * 0.28,
      start = box.max.x + L * 0.45;
    for (const y of [-0.32, 0, 0.32])
      for (const z of [-0.2, 0.2])
        group.add(
          new THREE.ArrowHelper(
            new THREE.Vector3(-1, 0, 0),
            new THREE.Vector3(start, c.y + y * size.y, c.z + z * h),
            len,
            0x3cc9d8,
            len * 0.26,
            len * 0.13
          )
        );
    labelLocal.set(start - len / 2, c.y, c.z - h * 0.45);
  }

  return {
    group,
    // Rebuild the arrows when the aircraft asset changes; refresh the label text every time.
    update(currentAsset, on, speed) {
      group.visible = on;
      if (!on) {
        label.hidden = true;
        return;
      }
      if (asset !== currentAsset) {
        asset = currentAsset;
        build(currentAsset);
      }
      label.textContent = `공기 흐름 · 기체 +X → −X${Number.isFinite(speed) ? ` · ${speed} m/s` : ''}`;
    },
    // Follow the aircraft pose and place the label; called from the render loop.
    sync(aircraftGroup, camera) {
      if (!group.visible || !aircraftGroup) return;
      group.position.copy(aircraftGroup.position);
      group.quaternion.copy(aircraftGroup.quaternion);
      group.updateMatrixWorld(true);
      const p = group.localToWorld(labelLocal.clone()).project(camera),
        inside = p.z < 1 && Math.abs(p.x) < 1.1 && Math.abs(p.y) < 1.1;
      label.hidden = !inside;
      if (inside) {
        label.style.left = `${((p.x + 1) * viewport.clientWidth) / 2}px`;
        label.style.top = `${((1 - p.y) * viewport.clientHeight) / 2}px`;
      }
    }
  };
}
