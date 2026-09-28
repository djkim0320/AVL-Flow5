// Three.js scene, camera, renderer and gizmos for the assembly editor.
// Nothing here knows about the placed models; app.js owns those.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

export function disposeTree(root) {
  root.traverse(o => {
    o.geometry?.dispose();
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    mats.forEach(m => m?.dispose());
  });
}

export function createScene(viewport, { onRender }) {
  // The canvas is transparent; the viewport's CSS gradient (theme.css) is the scene background.
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, 0.001, 10000);
  camera.up.set(0, 0, -1);
  camera.position.set(-2.1, -2, -1.1);
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setClearColor(0x000000, 0);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.setAttribute('aria-hidden', 'true');
  viewport.prepend(renderer.domElement);
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.target.set(-0.2, 0, 0);
  orbit.enableDamping = false;
  // Right drag orbits even over a model (left drag on a model moves it); Shift/Ctrl + right drag and the middle button pan.
  orbit.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE };
  orbit.screenSpacePanning = true;
  // Middle-button dragging moves the camera target, not browser auto-scroll.
  renderer.domElement.addEventListener('mousedown', event => {
    if (event.button === 1) event.preventDefault();
  });
  const transform = new TransformControls(camera, renderer.domElement);
  transform.setSize(0.82);
  scene.add(transform.getHelper());
  scene.add(new THREE.HemisphereLight(0xffffff, 0x60788e, 2.3));
  for (const [position, intensity] of [
    [[2, -3, -5], 3],
    [[-3, 2, -1], 1.4]
  ]) {
    const l = new THREE.DirectionalLight(0xffffff, intensity);
    l.position.fromArray(position);
    scene.add(l);
  }
  // 10 cm minor / 1 m major floor grid, recoloured from CSS tokens when the theme changes.
  const grid = new THREE.Group();
  grid.rotation.x = Math.PI / 2;
  grid.position.z = 0.35;
  scene.add(grid);
  function applyTheme() {
    const css = getComputedStyle(document.documentElement),
      color = name => new THREE.Color(css.getPropertyValue(name).trim());
    while (grid.children.length) {
      const child = grid.children[0];
      grid.remove(child);
      disposeTree(child);
    }
    for (const [divisions, token, opacity] of [
      [100, '--grid-minor', 0.7],
      [10, '--grid-major', 0.9]
    ]) {
      const g = new THREE.GridHelper(10, divisions, color(token), color(token));
      g.material.transparent = true;
      g.material.opacity = opacity;
      g.material.depthWrite = false;
      grid.add(g);
    }
  }
  applyTheme();
  scene.add(new THREE.AxesHelper(0.18));
  const observer = new ResizeObserver(() => {
    const { width, height } = viewport.getBoundingClientRect();
    renderer.setSize(width, height);
    camera.aspect = width / Math.max(height, 1);
    camera.updateProjectionMatrix();
    onRender();
  });
  observer.observe(viewport);
  orbit.addEventListener('change', onRender);
  transform.addEventListener('change', onRender);

  // Frame `bounds` from a preset direction: all (isometric), side, top, rear or focus (keeps the isometric direction).
  function fitView(bounds, kind = 'all') {
    const center = bounds.getCenter(new THREE.Vector3()),
      size = bounds.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() / 2, 0.015),
      distance = ((radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2))) * 1.2) / Math.min(camera.aspect, 1);
    const direction =
      kind === 'side'
        ? new THREE.Vector3(0, -1, 0)
        : kind === 'top'
          ? new THREE.Vector3(0, 0, -1)
          : kind === 'rear'
            ? new THREE.Vector3(-1, 0, 0)
            : new THREE.Vector3(-1.4, -1.2, -0.8).normalize();
    camera.up.set(0, 0, -1);
    if (kind === 'top') camera.up.set(1, 0, 0);
    camera.position.copy(center).addScaledVector(direction, distance);
    orbit.target.copy(center);
    camera.near = Math.max(0.0001, distance / 10000);
    camera.far = Math.max(100, distance * 100);
    camera.updateProjectionMatrix();
    orbit.update();
  }

  return { scene, camera, renderer, orbit, transform, applyTheme, fitView, draw: () => renderer.render(scene, camera) };
}
