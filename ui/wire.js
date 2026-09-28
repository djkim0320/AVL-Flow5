// Straight stowed-cable preview between the winch feed point and the payload, with marks
// where that line crosses the aircraft. Only geometry lives here; app.js writes the status text.
import * as THREE from 'three';
import { disposeTree } from './scene.js';

const wireColor = '#4c9aff',
  wireWarning = '#ff5a52';

export function createWire(scene) {
  const group = new THREE.Group();
  scene.add(group);
  const startMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.004, 16, 10),
    new THREE.MeshBasicMaterial({ color: wireColor, depthTest: false })
  );
  const endMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.0035, 16, 10),
    new THREE.MeshBasicMaterial({ color: '#2fcf7c', depthTest: false })
  );
  startMarker.renderOrder = endMarker.renderOrder = 10;
  startMarker.visible = endMarker.visible = false;
  scene.add(startMarker, endMarker);
  let signature = '';

  function clear() {
    while (group.children.length) {
      const child = group.children[0];
      group.remove(child);
      disposeTree(child);
    }
  }

  return {
    // Force the next update to rebuild even if the endpoints did not move (e.g. the aircraft mesh changed).
    invalidate() {
      signature = '';
    },
    // Returns null when there is no cable, {changed:false} when nothing moved, else the measured result.
    update({ start, end, cable, aircraft, xray }) {
      startMarker.visible = !!start;
      if (start) startMarker.position.copy(start);
      endMarker.visible = !!end;
      if (end) endMarker.position.copy(end);
      if (!start || !end) {
        clear();
        signature = '';
        return null;
      }
      const sig = JSON.stringify([
        start.toArray(),
        end.toArray(),
        cable.diameter_m,
        cable.length_m,
        xray,
        aircraft?.matrixWorld.elements
      ]);
      if (sig === signature) return { changed: false };
      signature = sig;
      clear();
      const distance = start.distanceTo(end),
        direction = end.clone().sub(start).normalize();
      let crossings = [];
      if (distance > 1e-8 && aircraft) {
        aircraft.updateMatrixWorld(true);
        const probe = new THREE.Raycaster(start, direction, 1e-5, Math.max(distance - 1e-5, 1e-5));
        crossings = probe
          .intersectObject(aircraft, true)
          .filter((h, i, arr) => !i || Math.abs(h.distance - arr[i - 1].distance) > 1e-5);
      }
      const short = cable.length_m + 1e-9 < distance;
      const warning = crossings.length > 0 || short;
      if (distance > 1e-8) {
        const tube = new THREE.Mesh(
          new THREE.CylinderGeometry(cable.diameter_m / 2, cable.diameter_m / 2, distance, 12),
          new THREE.MeshStandardMaterial({ color: warning ? wireWarning : wireColor, roughness: 0.55 })
        );
        tube.position.copy(start).add(end).multiplyScalar(0.5);
        tube.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
        group.add(tube);
        const line = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([start, end]),
          new THREE.LineBasicMaterial({ color: warning ? wireWarning : wireColor })
        );
        group.add(line);
      }
      for (const h of crossings) {
        const mark = new THREE.Mesh(
          new THREE.SphereGeometry(0.003, 10, 8),
          new THREE.MeshBasicMaterial({ color: wireWarning, depthTest: false })
        );
        mark.position.copy(h.point);
        mark.renderOrder = 8;
        group.add(mark);
      }
      return { changed: true, distance, crossings: crossings.length, short };
    }
  };
}
