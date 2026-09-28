import * as THREE from 'three';
import { createScene, disposeTree } from './scene.js';
import { createFlowIndicator } from './flow-indicator.js';
import { createWire } from './wire.js';
import { geometryPart, validateAsset, readModel, MAX_FILE_BYTES } from './model-io.js';
import { openDraftDB, loadDraft, saveDraft, postProject, downloadJSON } from './project-store.js';
import { finiteInput, editPoseComponent } from './coordinates.js';
import { pointAsset, previewPosition, migrateProject } from './point-mass.js';
import { initAnalysis } from './analysis-ui.js';
import { initAircraftDefinition } from './aircraft-definition-ui.js';
import {
  localFromWorld,
  worldFromLocal,
  onCenterline,
  quarterTurn,
  applyMatrix,
  reorientAsset,
  isBlankDefinition
} from './aircraft-model.js';
import {
  appendComponents,
  moveComponent,
  removeComponent,
  reviseDefinition,
  importBatch
} from './aircraft-assembly.js';

const $ = id => document.getElementById(id);
const names = { aircraft: '기체', sensor: '질점', winch: '윈치' };
const colors = { aircraft: '#b3c9d2', sensor: '#d69c43', winch: '#577b8c' };
const objects = { aircraft: null, sensor: null, winch: null };
let selected = null,
  selectedComponent = null,
  pendingRole = null,
  drag = null,
  lastPointer = null;
let cable = { diameter_m: 0.001, length_m: 2.7, attachment: null };
let undoStack = [],
  redoStack = [],
  busy = false,
  savingTimer = null,
  draftDB = null,
  dirty = false;
let analysisUI = null;
let aircraftDefinition = null,
  definitionUI = null,
  cgCallback = null;
let recentModelFiles = [];
const viewport = $('viewport'),
  ray = new THREE.Raycaster(),
  mouse = new THREE.Vector2();
const {
  scene,
  camera,
  renderer,
  orbit,
  transform,
  applyTheme: applySceneTheme,
  fitView,
  draw
} = createScene(viewport, { onRender: () => render() });
const wire = createWire(scene);
const cgHandle = new THREE.Group();
cgHandle.visible = false;
scene.add(cgHandle);
const cgLabel = document.createElement('div');
cgLabel.id = 'cg-label';
cgLabel.textContent = 'CG · 편집 중';
cgLabel.hidden = true;
viewport.append(cgLabel);
const cgBall = new THREE.Mesh(
  new THREE.SphereGeometry(0.018, 20, 12),
  new THREE.MeshBasicMaterial({ color: '#ff64d5', depthTest: false })
);
cgBall.renderOrder = 20;
cgHandle.add(cgBall);
const cgAxes = new THREE.AxesHelper(0.09);
cgAxes.material.depthTest = false;
cgAxes.renderOrder = 20;
cgHandle.add(cgAxes);
const aeroOverlay = new THREE.Group();
scene.add(aeroOverlay);
const flow = createFlowIndicator(scene, viewport);
function updateFlow() {
  let speed = null;
  try {
    speed = analysisUI?.snapshot()?.speed;
  } catch {}
  flow.update(objects.aircraft?.asset || null, !!objects.aircraft && $('show-flow').checked, speed);
}
function highlightParts(indices) {
  objects.aircraft?.group.traverse(m => {
    if (m.material?.emissive) m.material.emissive.set(indices.includes(m.userData.part) ? '#285faf' : '#000000');
  });
  render();
}
// Role colours while the definition panel shows the parts step; null restores each part's own colour.
function colorParts(colors) {
  objects.aircraft?.group.traverse(m => {
    if (!m.isMesh || m.userData.part == null) return;
    m.userData.baseColor ??= m.material.color.getHex();
    m.material.color.set(colors?.[m.userData.part] ?? m.userData.baseColor);
  });
  render();
}
function componentRecord() {
  return selected === 'aircraft' ? objects.aircraft?.asset.components?.find(c => c.id === selectedComponent) : null;
}
function selectionGroup() {
  return componentRecord() ? objects.aircraft.componentGroups.get(selectedComponent) : objects[selected]?.group;
}
function commitComponent() {
  const c = componentRecord(),
    g = selectionGroup();
  if (!c || !g) return;
  if (
    c.position.every((v, i) => Math.abs(v - g.position.toArray()[i]) < 1e-12) &&
    c.quaternion.every((v, i) => Math.abs(v - g.quaternion.toArray()[i]) < 1e-12)
  )
    return;
  const a = objects.aircraft,
    result = moveComponent(a.asset, c.id, g.position.toArray(), g.quaternion.toArray());
  aircraftDefinition = reviseDefinition(aircraftDefinition, result.asset, {
    affected: result.affected,
    delta: result.delta
  });
  install('aircraft', result.asset, a.group.position.toArray(), a.group.quaternion.toArray());
  wire.invalidate();
  select('aircraft', c.id);
  definitionUI?.refresh();
  status('파일 배치를 변경했습니다. 영향받은 공력 단면과 전체 CG·관성을 다시 입력한 뒤 새 버전으로 등록하세요.');
}
function editCG(point, callback) {
  transform.detach();
  cgCallback = callback || null;
  cgHandle.visible = !!point;
  transform.setSpace(point ? 'local' : 'world');
  if (point) {
    $('move').setAttribute('aria-pressed', 'true');
    $('rotate').setAttribute('aria-pressed', 'false');
  }
  if (point && objects.aircraft) {
    objects.aircraft.group.updateMatrixWorld(true);
    cgHandle.position.copy(objects.aircraft.group.localToWorld(new THREE.Vector3().fromArray(point)));
    cgHandle.quaternion.copy(objects.aircraft.group.quaternion);
    transform.setMode('translate');
    transform.setSpace('local');
    transform.attach(cgHandle);
    status('분홍색 CG의 축을 드래그하세요. 좌표는 CAD 원점 기준으로 저장합니다.');
  }
  render();
}
function showAeroSurfaces(surfaces) {
  while (aeroOverlay.children.length) {
    const o = aeroOverlay.children[0];
    aeroOverlay.remove(o);
    disposeTree(o);
  }
  const g = objects.aircraft?.group;
  if (!g) return;
  aeroOverlay.position.copy(g.position);
  aeroOverlay.quaternion.copy(g.quaternion);
  for (const surface of surfaces) {
    const sections = surface.sections.filter(s => s.le_m.every(Number.isFinite) && Number.isFinite(s.chord_m));
    for (const side of surface.mirror ? [1, -1] : [1]) {
      const points = [];
      for (const sec of sections) {
        const le = new THREE.Vector3(...sec.le_m),
          te = le.clone();
        te.x -= sec.chord_m;
        le.y *= side;
        te.y *= side;
        points.push(le, te);
      }
      for (let i = 0; i < sections.length - 1; i++)
        points.push(points[i * 2], points[(i + 1) * 2], points[i * 2 + 1], points[(i + 1) * 2 + 1]);
      if (points.length) {
        const line = new THREE.LineSegments(
          new THREE.BufferGeometry().setFromPoints(points),
          new THREE.LineBasicMaterial({ color: '#f6c453', depthTest: false })
        );
        line.renderOrder = 18;
        aeroOverlay.add(line);
      }
    }
  }
  render();
}

let selectionBox = null;

function status(message) {
  $('status').textContent = message;
}
function error(message) {
  $('error').textContent = message;
  $('error').hidden = !message;
  if (document.body.dataset.view === 'analysis') analysisUI?.showError(message);
}
function setBusy(value, message = '모델을 불러오는 중…') {
  busy = value;
  $('busy').hidden = !value;
  $('busy-text').textContent = message;
  viewport.setAttribute('aria-busy', String(value));
  document.querySelectorAll('button,input,select').forEach(b => {
    if (value) {
      b.dataset.wasDisabled = String(b.disabled);
      b.disabled = true;
    } else if (b.dataset.wasDisabled) {
      b.disabled = b.dataset.wasDisabled === 'true';
      delete b.dataset.wasDisabled;
    }
  });
  if (!value) {
    refresh();
    analysisUI?.refreshState();
  }
}
async function task(fn, message) {
  if (busy) return;
  error('');
  setBusy(true, message);
  try {
    await fn();
  } catch (e) {
    error(e.message || '처리하지 못했습니다. 다시 시도해 주세요.');
  } finally {
    setBusy(false);
  }
}
function render() {
  if (selectionBox && selectionGroup()) selectionBox.setFromObject(selectionGroup());
  if (typeof cgHandle !== 'undefined') {
    cgLabel.hidden = !cgHandle.visible;
    if (cgHandle.visible) {
      const p = cgHandle.position.clone().project(camera);
      cgLabel.style.left = `${((p.x + 1) * viewport.clientWidth) / 2 + 12}px`;
      cgLabel.style.top = `${((1 - p.y) * viewport.clientHeight) / 2 + 12}px`;
    }
  }
  flow.sync(objects.aircraft?.group, camera);
  draw();
}
transform.addEventListener('dragging-changed', event => {
  orbit.enabled = !event.value;
  if (!event.value) {
    if (transform.object === cgHandle && cgCallback) {
      cgCallback(
        localFromWorld(
          cgHandle.position.toArray(),
          objects.aircraft.group.position.toArray(),
          objects.aircraft.group.quaternion.toArray()
        )
      );
      status('CG 위치를 변경했습니다. 새 버전 등록 후 해석에 적용됩니다.');
    } else {
      commitComponent();
      remember();
      if (!selectedComponent) status('위치를 변경했습니다.');
    }
  }
});
transform.addEventListener('objectChange', () => {
  refreshInspector();
  updateWire();
  render();
});

function install(role, asset, position, quaternion = [0, 0, 0, 1]) {
  validateAsset(asset);
  if (selected === role) transform.detach();
  if (objects[role]) {
    scene.remove(objects[role].group);
    disposeTree(objects[role].group);
  }
  const group = new THREE.Group();
  group.name = role;
  group.userData.role = role;
  const componentGroups = new Map(),
    partComponents = new Map();
  for (const c of asset.components || []) {
    const child = new THREE.Group();
    child.name = c.name;
    child.position.fromArray(c.position);
    child.quaternion.fromArray(c.quaternion);
    child.userData = { role, component: c.id };
    group.add(child);
    componentGroups.set(c.id, child);
    for (const index of c.parts) partComponents.set(index, c.id);
  }
  asset.id ||= crypto.randomUUID();
  if (asset.kind === 'point_mass') {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.025, 16, 12),
      new THREE.MeshStandardMaterial({ color: colors.sensor, roughness: 0.65 })
    );
    marker.userData = { role };
    group.add(marker);
  }
  for (const [i, part] of (asset.parts || []).entries()) {
    const component = partComponents.get(i),
      parent = componentGroups.get(component) || group;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(part.positions, 3));
    geometry.setIndex(part.indices);
    if (component) {
      parent.updateMatrix();
      geometry.applyMatrix4(parent.matrix.clone().invert());
    }
    geometry.computeVertexNormals();
    geometry.computeBoundingSphere();
    const material = new THREE.MeshStandardMaterial({
      color: part.color || colors[role],
      roughness: 0.65,
      metalness: 0.12,
      side: THREE.DoubleSide
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = part.name;
    mesh.userData = { role, part: i, component };
    parent.add(mesh);
  }
  group.position.fromArray(position);
  group.quaternion.fromArray(quaternion).normalize();
  scene.add(group);
  group.updateMatrixWorld(true);
  objects[role] = { asset, group, componentGroups };
  applyXray();
}
function remove(role) {
  if (!objects[role]) return;
  if (selected === role) {
    selected = null;
    selectedComponent = null;
    transform.detach();
  }
  scene.remove(objects[role].group);
  disposeTree(objects[role].group);
  objects[role] = null;
}
function select(role, component = null) {
  selected = objects[role] ? role : null;
  selectedComponent = role === 'aircraft' && objects.aircraft?.componentGroups.has(component) ? component : null;
  transform.detach();
  if (selectionBox) {
    scene.remove(selectionBox);
    selectionBox.dispose();
    selectionBox = null;
  }
  if (selected) {
    selectionBox = new THREE.BoxHelper(selectionGroup(), 0x4c9aff);
    selectionBox.material.depthTest = false;
    selectionBox.material.transparent = true;
    selectionBox.material.opacity = 0.65;
    scene.add(selectionBox);
    // Aircraft orientation is changed only by the 90° CAD turns (the analysis frame), never by a display rotation.
    if (componentRecord()) {
      if (!componentRecord().locked) transform.attach(selectionGroup());
    } else if (
      selected !== 'sensor' &&
      (selected !== 'aircraft' || (!$('aircraft-lock').checked && transform.getMode() !== 'rotate'))
    )
      transform.attach(objects[selected].group);
  }
  refresh();
  render();
}
function refresh() {
  const count = Object.values(objects).filter(Boolean).length;
  $('object-count').textContent = `${count}개`;
  $('object-empty').hidden = count > 0;
  $('empty-scene').hidden = count > 0;
  const list = $('object-list');
  list.replaceChildren();
  for (const [role, entry] of Object.entries(objects))
    if (entry) {
      const b = document.createElement('button');
      b.className = 'object-item';
      b.setAttribute('aria-pressed', String(selected === role && !selectedComponent));
      b.dataset.role = role;
      b.disabled = busy;
      const swatch = document.createElement('span');
      swatch.className = 'swatch';
      swatch.style.background = colors[role];
      const text = document.createElement('span');
      text.textContent = names[role];
      const small = document.createElement('small');
      small.textContent = entry.asset.source?.name || names[role];
      text.append(small);
      b.append(swatch, text);
      b.onclick = () => select(role);
      list.append(b);
      if (role === 'aircraft')
        for (const c of entry.asset.components || []) {
          const child = document.createElement('button');
          child.className = 'object-item component-item';
          child.setAttribute('aria-pressed', String(selectedComponent === c.id));
          child.disabled = busy;
          const label = document.createElement('span');
          label.textContent = c.name;
          const count = document.createElement('small');
          count.textContent = `${c.parts.length}개 부품${c.locked ? ' · 고정' : ''}`;
          label.append(count);
          child.append(label);
          child.onclick = () => select('aircraft', c.id);
          list.append(child);
        }
    }
  $('add-winch').disabled = busy || !!objects.winch;
  $('add-point').disabled = busy || !objects.winch || !!objects.sensor;
  $('point-mass').disabled = busy;
  $('connection-state').textContent = objects.sensor && objects.winch ? '자동 연결됨' : '질점 대기';
  $('connection-state').dataset.state = objects.sensor && objects.winch ? 'connected' : 'unconnected';
  const step = !objects.aircraft ? 1 : !objects.winch ? 2 : !objects.sensor ? 3 : 4;
  document.querySelectorAll('[data-step]').forEach(item => {
    const n = Number(item.dataset.step);
    item.classList.toggle('done', n < step);
    if (n === step) item.setAttribute('aria-current', 'step');
    else item.removeAttribute('aria-current');
  });
  $('undo').disabled = busy || undoStack.length < 2;
  $('redo').disabled = busy || !redoStack.length;
  refreshInspector();
  updateWire();
  updateFlow();
}
function refreshInspector() {
  const component = componentRecord();
  $('selection-kind').dataset.role = component ? 'component' : selected || '';
  const entry = objects[selected];
  $('selection-kind').textContent = component ? '조립 부품' : entry ? names[selected] : '선택 없음';
  // Role first; the source name is secondary.
  $('selection-name').textContent = component ? component.name : entry ? names[selected] : '모델을 선택하세요.';
  if (entry?.asset.source?.name) {
    const from = document.createElement('small');
    from.textContent = component
      ? '기체에 속한 파일 · 위치는 파일 중심 / 기체 원점 기준'
      : (selected === 'winch' ? '형상 · ' : selected === 'aircraft' ? '파일 · ' : '') + entry.asset.source.name;
    $('selection-name').append(from);
  }
  $('component-actions').hidden = !component;
  if (component) $('component-lock').checked = component.locked;
  if (selected === 'aircraft' && entry && !component) {
    const turns = entry.asset.source?.orientation_turns || [];
    $('orient-state').textContent = turns.length ? `보정됨 · ${turns.join(' → ')}` : '보정 없음 · 파일 방향 그대로';
    $('orient-state').dataset.corrected = String(!!turns.length);
    const q = entry.group.quaternion;
    $('reset-aircraft-pose').hidden = Math.abs(q.w) > 1 - 1e-9;
  }
  $('transform-fields').disabled =
    busy ||
    !entry ||
    selected === 'sensor' ||
    (component ? component.locked : selected === 'aircraft' && $('aircraft-lock').checked);
  if (!entry) {
    for (const id of ['px', 'py', 'pz', 'rx', 'ry', 'rz']) $(id).value = '';
    $('selection-size').textContent = '모델을 선택하면 외곽 크기가 표시됩니다.';
    return;
  }
  const g = selectionGroup(),
    e = new THREE.Euler().setFromQuaternion(g.quaternion, 'XYZ');
  ['px', 'py', 'pz'].forEach((id, i) => {
    if (document.activeElement !== $(id)) $(id).value = (g.position.getComponent(i) * 1000).toFixed(2);
  });
  ['rx', 'ry', 'rz'].forEach((id, i) => {
    if (document.activeElement !== $(id)) $(id).value = THREE.MathUtils.radToDeg([e.x, e.y, e.z][i]).toFixed(2);
  });
  if (selected === 'sensor') {
    $('selection-size').textContent = `질량 ${entry.asset.mass_kg} kg · 점의 크기는 표시용입니다.`;
    return;
  }
  const bounds = new THREE.Box3().setFromObject(g).getSize(new THREE.Vector3());
  $('selection-size').textContent = `외곽 크기 ${bounds
    .toArray()
    .map(v => (v * 1000).toFixed(1))
    .join(' × ')} mm${selected === 'winch' ? ' · 파란 점이 줄 출구입니다.' : ''}`;
}
function applyXray() {
  objects.aircraft?.group.traverse(o => {
    if (o.isMesh) {
      o.material.transparent = $('xray').checked;
      o.material.opacity = $('xray').checked ? 0.17 : 1;
      o.material.depthWrite = !$('xray').checked;
    }
  });
  render();
}
function snapshot() {
  syncPoint();
  return {
    aircraft_definition: structuredClone(aircraftDefinition),
    objects: Object.fromEntries(
      Object.entries(objects).map(([k, v]) => [
        k,
        v
          ? {
              asset: v.asset.kind === 'point_mass' ? { ...v.asset } : v.asset,
              position: v.group.position.toArray(),
              quaternion: v.group.quaternion.toArray()
            }
          : null
      ])
    ),
    cable: structuredClone(cable),
    selected,
    selectedComponent,
    locked: $('aircraft-lock').checked
  };
}
function signature(s) {
  return JSON.stringify({
    ...s,
    objects: Object.fromEntries(
      Object.entries(s.objects).map(([k, v]) => [
        k,
        v ? { asset: v.asset.id, mass_kg: v.asset.mass_kg, position: v.position, quaternion: v.quaternion } : null
      ])
    )
  });
}
function remember() {
  const s = snapshot();
  if (undoStack.length && signature(undoStack.at(-1)) === signature(s)) return;
  undoStack.push(s);
  if (undoStack.length > 40) undoStack.shift();
  redoStack = [];
  dirty = true;
  queueDraft();
  refresh();
  render();
}
function restore(s, replaceAssets = false) {
  aircraftDefinition = structuredClone(s.aircraft_definition || null);
  editCG(null);
  transform.detach();
  for (const role of Object.keys(objects)) {
    const v = s.objects[role];
    if (!v) remove(role);
    else if (!replaceAssets && objects[role]?.asset.id === v.asset.id) {
      objects[role].group.position.fromArray(v.position);
      objects[role].group.quaternion.fromArray(v.quaternion);
      if (v.asset.kind === 'point_mass') objects[role].asset = { ...v.asset };
    } else install(role, v.asset, v.position, v.quaternion);
  }
  cable = structuredClone(s.cable);
  $('aircraft-lock').checked = s.locked ?? true;
  $('wire-diameter').value = cable.diameter_m * 1000;
  $('wire-length').value = cable.length_m;
  if (objects.sensor) $('point-mass').value = objects.sensor.asset.mass_kg;
  select(s.selected, s.selectedComponent);
  refresh();
  definitionUI?.refresh();
  render();
}
function undo() {
  if (undoStack.length < 2) return;
  redoStack.push(undoStack.pop());
  restore(undoStack.at(-1));
  queueDraft();
  status('이전 배치로 되돌렸습니다.');
}
function redo() {
  if (!redoStack.length) return;
  const s = redoStack.pop();
  undoStack.push(s);
  restore(s);
  queueDraft();
  status('배치를 다시 적용했습니다.');
}
function worldAttachment() {
  if (!cable.attachment || !objects.sensor) return null;
  objects.sensor.group.updateMatrixWorld(true);
  return objects.sensor.group.localToWorld(new THREE.Vector3().fromArray(cable.attachment.local_point_m));
}
function winchPoint() {
  objects.winch?.group.updateMatrixWorld(true);
  return objects.winch?.group.localToWorld(new THREE.Vector3());
}
function updateWire() {
  syncPoint();
  const result = wire.update({
    start: winchPoint(),
    end: worldAttachment(),
    cable,
    aircraft: objects.aircraft?.group || null,
    xray: $('xray').checked
  });
  if (!result) {
    $('wire-measure').textContent = '연결된 줄 없음';
    $('path-status').className = 'path-status';
    $('path-status').textContent = '윈치와 질점을 추가하면 줄을 자동 연결합니다.';
    return;
  }
  if (!result.changed) return;
  const { distance, crossings, short } = result;
  $('wire-measure').textContent = `시작 줄 ${(distance * 1000).toFixed(1)} mm · 전개 목표 ${cable.length_m} m`;
  $('path-status').className = `path-status ${crossings > 0 || short ? 'warning' : 'clear'}`;
  $('path-status').textContent = short
    ? `줄 길이가 직선 거리 ${distance.toFixed(3)} m보다 짧습니다. 길이나 배치를 수정하세요.`
    : crossings
      ? `수납 위치의 짧은 줄이 기체와 ${crossings}곳에서 겹칩니다. 질점 해석에는 형상 접촉이 포함되지 않습니다.`
      : '줄을 감은 시작 상태입니다. 해석에서 목표 길이까지 전개한 뒤 유지 비행·회수를 계산합니다. 질점 해석에는 형상 접촉이 포함되지 않습니다.';
}
function pointerRay(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.set(((event.clientX - rect.left) / rect.width) * 2 - 1, (-(event.clientY - rect.top) / rect.height) * 2 + 1);
  ray.setFromCamera(mouse, camera);
}
function pick(event, role = null) {
  pointerRay(event);
  scene.updateMatrixWorld(true);
  let targets = Object.values(objects)
    .filter(Boolean)
    .map(v => v.group);
  if (role) targets = objects[role] ? [objects[role].group] : [];
  const hits = ray.intersectObjects(targets, true);
  return (
    hits.find(h => h.object.isMesh && (!$('xray').checked || role || h.object.userData.role !== 'aircraft')) || null
  );
}
function syncPoint() {
  cable.attachment = objects.sensor && objects.winch ? { kind: 'point_mass', local_point_m: [0, 0, 0] } : null;
  if (!cable.attachment) return;
  objects.sensor.group.position.fromArray(
    previewPosition(
      objects.winch.group.position.toArray(),
      objects.aircraft?.group.quaternion.toArray() || [0, 0, 0, 1],
      cable.length_m
    )
  );
  objects.sensor.group.quaternion.identity();
  objects.sensor.group.updateMatrixWorld(true);
}
function addPoint() {
  if (!objects.winch) throw new Error('윈치를 먼저 추가하세요.');
  if (objects.sensor) return;
  readBasicInputs();
  install('sensor', pointAsset(finiteInput($('point-mass').value)), [0, 0, 0]);
  syncPoint();
  select('sensor');
  remember();
  view();
  status('질점을 윈치 가까이 수납하고 줄을 연결했습니다. 입력한 길이는 전개 목표입니다.');
}

renderer.domElement.addEventListener(
  'pointerdown',
  event => {
    lastPointer = { x: event.clientX, y: event.clientY };
    if (event.button !== 0 || busy || transform.axis) return;
    if (definitionUI?.isActive()) {
      const h = pick(event, 'aircraft');
      if (h) {
        definitionUI.pick(h.object.userData.part, event.shiftKey);
        event.stopImmediatePropagation();
      }
      return;
    }
    const hit = pick(event);
    if (!hit) return;
    const role = hit.object.userData.role;
    select(role, hit.object.userData.component);
    if (
      role === 'sensor' ||
      (componentRecord() ? componentRecord().locked : role === 'aircraft' && $('aircraft-lock').checked)
    )
      return;
    if (transform.getMode() === 'rotate') return;
    const target = selectionGroup(),
      world = target.getWorldPosition(new THREE.Vector3());
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(camera.getWorldDirection(new THREE.Vector3()), world);
    pointerRay(event);
    const point = ray.ray.intersectPlane(plane, new THREE.Vector3());
    if (!point) return;
    drag = {
      role,
      target,
      plane,
      offset: world.sub(point),
      start: target.position.clone(),
      x: event.clientX,
      y: event.clientY,
      moved: false,
      id: event.pointerId
    };
    event.stopImmediatePropagation();
    orbit.enabled = false;
    renderer.domElement.setPointerCapture(event.pointerId);
  },
  true
);
renderer.domElement.addEventListener(
  'pointermove',
  event => {
    if (drag) {
      pointerRay(event);
      const point = ray.ray.intersectPlane(drag.plane, new THREE.Vector3());
      if (point && Math.hypot(event.clientX - drag.x, event.clientY - drag.y) > 2) {
        drag.moved = true;
        drag.target.position.copy(drag.target.parent.worldToLocal(point.add(drag.offset)));
        refreshInspector();
        updateWire();
        render();
      }
      event.stopImmediatePropagation();
      return;
    }
  },
  true
);
function finishDrag(event, cancel = false) {
  if (!drag) return;
  const d = drag;
  drag = null;
  if (cancel) d.target.position.copy(d.start);
  orbit.enabled = true;
  if (renderer.domElement.hasPointerCapture(d.id)) renderer.domElement.releasePointerCapture(d.id);
  event?.stopImmediatePropagation();
  if (d.moved && !cancel) {
    commitComponent();
    remember();
  } else {
    refresh();
    render();
  }
}
renderer.domElement.addEventListener('pointerup', e => finishDrag(e), true);
renderer.domElement.addEventListener('pointercancel', e => finishDrag(e, true), true);
renderer.domElement.addEventListener('contextmenu', event => event.preventDefault());

function view(kind = 'all') {
  const entries = kind === 'focus' && selected ? [{ group: selectionGroup() }] : Object.values(objects).filter(Boolean);
  if (!entries.length) return;
  const bounds = new THREE.Box3();
  entries.forEach(o => bounds.expandByObject(o.group));
  if (kind !== 'focus' && flow.group.visible) bounds.expandByObject(flow.group);
  fitView(bounds, kind);
  render();
}
async function addWinch() {
  if (objects.winch) return;
  if (!objects.aircraft) throw new Error('기체를 먼저 불러오세요.');
  const drum = new THREE.CylinderGeometry(0.025, 0.025, 0.05, 24).translate(0, 0, 0.025),
    base = new THREE.BoxGeometry(0.075, 0.065, 0.008).translate(0, 0, 0.054);
  const asset = {
    parts: [geometryPart(drum, 'winch_drum'), geometryPart(base, 'winch_base')],
    source: { name: '윈치 표시 · 치수는 해석에 사용하지 않음', units: 'm', axes: 'FRD', origin: 'feed_point' }
  };
  drum.dispose();
  base.dispose();
  const group = objects.aircraft.group;
  group.updateMatrixWorld(true);
  const cgY =
    aircraftDefinition?.mass?.mode === 'total' && Number.isFinite(aircraftDefinition.mass.cg_m?.[1])
      ? aircraftDefinition.mass.cg_m[1]
      : 0;
  const position = onCenterline(
    new THREE.Box3().setFromObject(group).getCenter(new THREE.Vector3()).toArray(),
    group.position.toArray(),
    group.quaternion.toArray(),
    cgY
  );
  install('winch', asset, position, group.quaternion.toArray());
  select('winch');
  remember();
  status('윈치를 기체 중심선에 추가했습니다. 파란 점이 줄 출구입니다. 윈치 모양과 치수는 표시용입니다.');
}

const turnQuaternion = m =>
  new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().set(
      m[0][0],
      m[0][1],
      m[0][2],
      0,
      m[1][0],
      m[1][1],
      m[1][2],
      0,
      m[2][0],
      m[2][1],
      m[2][2],
      0,
      0,
      0,
      0,
      1
    )
  );
// Keep the winch fixed to the airframe while the aircraft frame changes (CAD turn or pose reset).
function carryWinch(fromQ, toQ, localTurn = null) {
  const a = objects.aircraft.group,
    w = objects.winch?.group;
  if (!w) return;
  const o = a.position.toArray();
  let local = localFromWorld(w.position.toArray(), o, fromQ.toArray());
  let ql = fromQ.clone().invert().multiply(w.quaternion);
  if (localTurn) {
    local = applyMatrix(localTurn, local);
    ql = turnQuaternion(localTurn).multiply(ql);
  }
  w.position.fromArray(worldFromLocal(local, o, toQ.toArray()));
  w.quaternion.copy(toQ.clone().multiply(ql).normalize());
  w.updateMatrixWorld(true);
}
function reorientAircraft(axis, sign) {
  const a = objects.aircraft;
  if (!a) return;
  if (aircraftDefinition && !isBlankDefinition(aircraftDefinition))
    throw new Error(
      '기체 정의에 입력한 좌표·단면은 현재 방향 기준입니다. 방향 보정은 기체 정의를 입력하기 전에 하세요. 되돌리기(Ctrl+Z)로 입력 전 상태로 돌아갈 수 있습니다.'
    );
  const q = a.group.quaternion.clone(),
    m = quarterTurn(axis, sign);
  carryWinch(q, q, m);
  aircraftDefinition = null;
  editCG(null);
  showAeroSurfaces([]);
  install('aircraft', reorientAsset(a.asset, axis, sign), a.group.position.toArray(), q.toArray());
  wire.invalidate();
  select('aircraft');
  remember();
  definitionUI?.refresh();
  status(
    `기체 방향을 ${axis.toUpperCase()}축 ${sign < 0 ? '−' : '+'}90° 보정했습니다. 청록색 공기 흐름 화살표가 기체 코 쪽에서 오는지 확인하세요.`
  );
}
function resetAircraftPose() {
  const a = objects.aircraft;
  if (!a) return;
  const from = a.group.quaternion.clone(),
    to = new THREE.Quaternion();
  carryWinch(from, to);
  a.group.quaternion.copy(to);
  a.group.updateMatrixWorld(true);
  wire.invalidate();
  select('aircraft');
  remember();
  status('기체의 화면 표시 회전을 0으로 되돌렸습니다. 윈치는 기체와 함께 움직였습니다.');
}

async function loadModels(files, replace = false) {
  if (!files.length) return;
  if (files.length > 100) throw new Error('한 번에 100개 이하 파일을 선택하세요.');
  const options = {
    unit: $('import-unit').value,
    axes: $('import-axes').value,
    onStage: message => ($('busy-text').textContent = message)
  };
  const assets = await importBatch(files, file => readModel(file, options), 2),
    old = objects.aircraft,
    base = replace ? null : old?.asset;
  const asset = validateAsset(appendComponents(base, assets));
  // Commit once after every conversion succeeds; no partial assembly on error.
  definitionUI?.close();
  if (replace) {
    aircraftDefinition = null;
    remove('sensor');
    remove('winch');
    cable.attachment = null;
  } else aircraftDefinition = reviseDefinition(aircraftDefinition, asset, { topology: true });
  install(
    'aircraft',
    asset,
    base ? old.group.position.toArray() : [0, 0, 0],
    base ? old.group.quaternion.toArray() : [0, 0, 0, 1]
  );
  select('aircraft', asset.components.at(-1).id);
  wire.invalidate();
  remember();
  view();
  status(
    `${files.length}개 파일을 ${replace ? '새 기체로 불러왔습니다' : '조립체에 추가했습니다'}. CAD 원점은 유지합니다. 오른쪽 파일을 선택해 이동·회전하거나 기체 정의에서 역할을 지정하세요.`
  );
  $('scene-badge').textContent = '사용자 조립체 · 배치 편집';
  recentModelFiles = [...files];
  $('recent-model-file').hidden = false;
  $('recent-model-file').textContent =
    `${files[0].name}${files.length > 1 ? ` 외 ${files.length - 1}개` : ''} · 다시 추가`;
}

function project({ includeAnalysis = true } = {}) {
  readBasicInputs();
  syncPoint();
  return {
    schema: 'dbf-assembly/2',
    units: 'm',
    axes: 'FRD',
    aircraft_definition: structuredClone(aircraftDefinition),
    saved_at: new Date().toISOString(),
    objects: Object.fromEntries(
      Object.entries(objects).map(([role, v]) => [
        role,
        v ? { ...v.asset, position: v.group.position.toArray(), quaternion: v.group.quaternion.toArray() } : null
      ])
    ),
    cable: structuredClone(cable),
    view: { selected, selectedComponent, aircraft_locked: $('aircraft-lock').checked },
    ...(includeAnalysis ? { analysis: analysisUI?.snapshot() } : {})
  };
}
async function applyProject(value) {
  const legacy = value.schema === 'dbf-assembly/1';
  value = migrateProject(value);
  if (value.schema !== 'dbf-assembly/2' || value.units !== 'm' || value.axes !== 'FRD')
    throw new Error('이 편집기에서 저장한 DBF 배치 JSON을 선택해 주세요.');
  const staged = {};
  for (const role of Object.keys(objects)) {
    const o = value.objects[role];
    if (o) {
      validateAsset(o);
      for (const key of ['position', 'quaternion'])
        if (!Array.isArray(o[key]) || !o[key].every(Number.isFinite))
          throw new Error('저장된 위치 값이 올바르지 않습니다.');
      staged[role] = { asset: { ...o, id: crypto.randomUUID() }, position: o.position, quaternion: o.quaternion };
    } else staged[role] = null;
  }
  restore(
    {
      objects: staged,
      aircraft_definition: value.aircraft_definition,
      cable: value.cable,
      selected: value.view?.selected,
      selectedComponent: value.view?.selectedComponent,
      locked: value.view?.aircraft_locked
    },
    true
  );
  await analysisUI?.restore(value.analysis);
  remember();
  view();
  if (legacy) status(value.migration_note);
}
const PREVIOUS_SESSION = 'previous-session';
function queueDraft() {
  clearTimeout(savingTimer);
  $('save-state').textContent = '배치 변경됨';
  $('save-state').title = '';
  savingTimer = setTimeout(async () => {
    let value;
    try {
      value = project();
    } catch (e) {
      $('save-state').textContent = '입력 확인 필요 · 저장되지 않음';
      $('save-state').title = e.message;
      return;
    }
    try {
      draftDB ||= await openDraftDB();
    } catch (e) {
      $('save-state').textContent = '파일 저장을 사용하세요';
      return;
    }
    try {
      await saveDraft(draftDB, value);
      $('save-state').textContent = '이 브라우저에 자동 저장됨';
      dirty = false;
    } catch (e) {
      $('save-state').textContent = '자동 저장 실패 · 파일 저장을 사용하세요';
    }
  }, 400);
}

$('empty-load').onclick = () => $('load-aircraft').click();
$('empty-restore').onclick = () =>
  task(async () => {
    const previous = await loadDraft(draftDB, PREVIOUS_SESSION);
    if (!previous) throw new Error('이전 작업이 없습니다.');
    if (previous.schema === 'dbf-assembly/1') await saveDraft(draftDB, previous, 'legacy-draft-before-point-mass');
    await applyProject(previous);
    $('empty-restore').hidden = true;
    if (previous.schema !== 'dbf-assembly/1') status('이 브라우저에 저장한 이전 작업을 열었습니다.');
  }, '이전 작업을 여는 중…');
$('add-winch').onclick = () => task(addWinch, '윈치를 추가하는 중…');
$('load-aircraft').onclick = () => {
  pendingRole = 'append';
  $('model-file').click();
};
$('replace-aircraft').onclick = () => {
  pendingRole = 'replace';
  $('model-file').click();
};
$('model-file').onchange = e => {
  const files = [...e.target.files],
    replace = pendingRole === 'replace';
  e.target.value = '';
  if (files.length) task(() => loadModels(files, replace));
};
$('recent-model-file').addEventListener('dragstart', e => {
  if (busy || !recentModelFiles.length) {
    e.preventDefault();
    return;
  }
  e.dataTransfer.setData('application/x-dbf-recent-model', 'aircraft');
  e.dataTransfer.effectAllowed = 'copy';
});
$('recent-model-file').onclick = () => {
  if (!busy && recentModelFiles.length) task(() => loadModels(recentModelFiles));
};
const isModelDrag = e =>
  Array.from(e.dataTransfer?.types || []).some(t => t === 'Files' || t === 'application/x-dbf-recent-model');
viewport.addEventListener('dragover', e => {
  if (!isModelDrag(e)) return;
  e.preventDefault();
  e.dataTransfer.dropEffect = busy ? 'none' : 'copy';
  viewport.classList.toggle('file-drag-over', !busy);
});
viewport.addEventListener('dragleave', e => {
  if (!viewport.contains(e.relatedTarget)) viewport.classList.remove('file-drag-over');
});
viewport.addEventListener('drop', e => {
  if (!isModelDrag(e)) return;
  e.preventDefault();
  viewport.classList.remove('file-drag-over');
  if (busy) {
    error('파일을 불러오는 중입니다. 완료 후 다시 놓아 주세요.');
    return;
  }
  const files = Array.from(e.dataTransfer.files || []);
  if (!files.length && e.dataTransfer.getData('application/x-dbf-recent-model') === 'aircraft')
    files.push(...recentModelFiles);
  if (files.length) task(() => loadModels(files));
});
// Keep files dropped outside the canvas from replacing the editor page.
window.addEventListener('dragover', e => {
  if (isModelDrag(e)) e.preventDefault();
});
window.addEventListener('drop', e => {
  if (isModelDrag(e)) e.preventDefault();
});
$('add-point').onclick = () => task(async () => addPoint(), '질점을 추가하는 중…');
function readBasicInputs() {
  const mass = pointAsset(finiteInput($('point-mass').value)).mass_kg;
  const length = finiteInput($('wire-length').value),
    diameter = finiteInput($('wire-diameter').value) / 1000;
  if (length <= 0 || length > 100 || diameter < 0.00005 || diameter > 0.02)
    throw new Error('줄 길이는 0 초과~100 m, 직경은 0.05~20 mm로 입력하세요.');
  cable.length_m = length;
  cable.diameter_m = diameter;
  if (objects.sensor) objects.sensor.asset = { ...objects.sensor.asset, mass_kg: mass };
}
function editBasics() {
  try {
    readBasicInputs();
    error('');
    remember();
  } catch (e) {
    error(e.message);
  }
}
$('point-mass').oninput = $('point-mass').onchange = editBasics;
$('undo').onclick = undo;
$('redo').onclick = redo;
$('aircraft-lock').onchange = () => {
  select(selected, selectedComponent);
  remember();
};
$('xray').onchange = () => {
  applyXray();
  wire.invalidate();
  updateWire();
};
$('component-lock').onchange = () => {
  const c = componentRecord();
  if (!c) return;
  const a = objects.aircraft;
  a.asset = {
    ...a.asset,
    id: crypto.randomUUID(),
    components: a.asset.components.map(v => (v.id === c.id ? { ...v, locked: $('component-lock').checked } : v))
  };
  select(selected, selectedComponent);
  remember();
};
$('component-define').onclick = () => {
  const c = componentRecord();
  if (c) {
    transform.detach();
    definitionUI.open(c.parts);
  }
};
$('component-remove').onclick = () => {
  const c = componentRecord();
  if (!c || busy) return;
  const a = objects.aircraft,
    result = removeComponent(a.asset, c.id);
  definitionUI.close();
  aircraftDefinition = reviseDefinition(aircraftDefinition, result.asset, {
    map: result.map,
    affected: result.affected,
    topology: true
  });
  if (result.asset) {
    install('aircraft', result.asset, a.group.position.toArray(), a.group.quaternion.toArray());
    select('aircraft');
  } else {
    remove('aircraft');
    remove('winch');
    remove('sensor');
    cable.attachment = null;
    select(null);
  }
  wire.invalidate();
  remember();
  status('선택한 파일을 조립체에서 제거했습니다. 되돌리기로 복원할 수 있습니다. 물성과 공력 면을 다시 확인하세요.');
};
$('show-flow').onchange = () => {
  updateFlow();
  render();
};
document.querySelectorAll('[data-turn]').forEach(
  b =>
    (b.onclick = () => {
      if (busy) return;
      error('');
      try {
        reorientAircraft(b.dataset.turn, Number(b.dataset.sign));
      } catch (e) {
        error(e.message);
      }
    })
);
$('reset-aircraft-pose').onclick = () => {
  if (!busy) resetAircraftPose();
};
for (const id of ['px', 'py', 'pz', 'rx', 'ry', 'rz'])
  $(id).oninput = $(id).onchange = () => {
    if (!selected || !objects[selected] || selected === 'sensor' || componentRecord()?.locked) return;
    error('');
    try {
      editPoseComponent(selectionGroup(), id, $(id).value);
      commitComponent();
      remember();
    } catch (e) {
      error(e.message);
    }
  };
for (const id of ['wire-diameter', 'wire-length']) $(id).oninput = $(id).onchange = editBasics;
for (const [id, kind] of [
  ['view-all', 'all'],
  ['view-side', 'side'],
  ['view-top', 'top'],
  ['view-rear', 'rear'],
  ['focus', 'focus']
])
  $(id).onclick = () => view(kind);
function setMode(mode) {
  if (cgHandle.visible && cgCallback) {
    if (mode === 'rotate') {
      status('CG는 위치만 설정합니다. 분홍색 축을 드래그하세요.');
      return;
    }
    transform.setMode('translate');
    transform.attach(cgHandle);
    render();
    return;
  }
  transform.setMode(mode);
  $('move').setAttribute('aria-pressed', String(mode === 'translate'));
  $('rotate').setAttribute('aria-pressed', String(mode === 'rotate'));
  select(selected, selectedComponent);
  if (mode === 'rotate' && selected === 'aircraft' && !selectedComponent)
    status('기체 방향은 오른쪽 선택 속성의 해석 기준 방향에서 90° 단위로 바꿉니다.');
}
$('move').onclick = () => setMode('translate');
$('rotate').onclick = () => setMode('rotate');
$('save-project').onclick = () =>
  task(async () => {
    const value = project();
    await postProject(value);
    downloadJSON(value, 'DBF_센서배치.dbf-scene.json');
    dirty = false;
    $('save-state').textContent = '프로젝트 파일 저장됨';
    status('모델과 배치를 프로젝트 파일로 저장했습니다.');
  }, '배치를 저장하는 중…');
$('save-project-step').onclick = () => $('save-project').click();
$('open-project').onclick = () => $('project-file').click();
$('project-file').onchange = e => {
  const file = e.target.files[0];
  e.target.value = '';
  if (file)
    task(async () => {
      if (file.size > MAX_FILE_BYTES) throw new Error('64 MB 이하 프로젝트를 선택하세요.');
      const value = migrateProject(JSON.parse(await file.text()));
      await postProject(value);
      await applyProject(value);
      status(value.migration_note || '저장한 기체·윈치·질점을 다시 불러왔습니다.');
    }, '프로젝트를 여는 중…');
};
$('export-setup').onclick = () => {
  error('');
  if (!objects.aircraft || !objects.sensor || !objects.winch || !cable.attachment) {
    error('기체와 윈치를 배치하고 질점을 추가하세요.');
    return;
  }
  scene.updateMatrixWorld(true);
  const aircraftInverse = objects.aircraft.group.matrixWorld.clone().invert(),
    sensorMatrix = aircraftInverse.clone().multiply(objects.sensor.group.matrixWorld),
    p = new THREE.Vector3(),
    q = new THREE.Quaternion(),
    scale = new THREE.Vector3();
  sensorMatrix.decompose(p, q, scale);
  const start = winchPoint().applyMatrix4(aircraftInverse),
    end = worldAttachment().applyMatrix4(aircraftInverse);
  downloadJSON(
    {
      schema: 'dbf-assembly-setup/2',
      units: 'SI',
      axes: 'FRD',
      aircraft_definition: structuredClone(aircraftDefinition),
      analysis_ready: false,
      aircraft_origin: 'imported model origin; CG must be specified separately',
      sensor: { kind: 'point_mass', mass_kg: objects.sensor.asset.mass_kg, preview_position_aircraft_m: p.toArray() },
      winch: { feed_point_aircraft_m: start.toArray() },
      cable: { ...cable, straight_distance_m: start.distanceTo(end) },
      sources: Object.fromEntries(Object.entries(objects).map(([k, v]) => [k, v.asset.source])),
      required_analysis_inputs: [
        'aircraft mass, CG and inertia',
        'aerodynamic database matching the aircraft',
        'cable mass, stiffness, damping and drag',
        'deployment/recovery commands; point-mass model excludes CAD contact'
      ]
    },
    'DBF_해석용배치.setup.json'
  );
  status('좌표·연결점·줄 설정을 내보냈습니다. 해석 물성은 별도로 지정해야 합니다.');
};
window.addEventListener('keydown', e => {
  if ($('analysis-dialog')?.open) return;
  if (busy || /INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName)) return;
  if (e.key === 'Escape') {
    if (drag) finishDrag(null, true);
    transform.reset();
    select(selected, selectedComponent);
    status('현재 조작을 취소했습니다.');
  }
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
    e.preventDefault();
    e.shiftKey ? redo() : undo();
  } else if (!e.ctrlKey && !e.metaKey) {
    if (e.key.toLowerCase() === 'g') setMode('translate');
    if (e.key.toLowerCase() === 'r') setMode('rotate');
    if (e.key.toLowerCase() === 'f') view('focus');
  }
});
window.addEventListener('beforeunload', e => {
  if (dirty) {
    e.preventDefault();
    e.returnValue = '';
  }
});
// Read-only diagnostic surface for reproducible local UI checks.
window.dbfEditor = {
  snapshot: () => project(),
  getScreenPoint: (role, local = [0, 0, 0]) => {
    const v = objects[role]?.group.localToWorld(new THREE.Vector3().fromArray(local));
    if (!v) return null;
    v.project(camera);
    const r = renderer.domElement.getBoundingClientRect();
    return { x: r.left + ((v.x + 1) * r.width) / 2, y: r.top + ((1 - v.y) * r.height) / 2 };
  }
};
definitionUI = initAircraftDefinition({
  getAircraft: () => objects.aircraft?.asset,
  getDefinition: () => aircraftDefinition,
  setDefinition: v => {
    aircraftDefinition = v;
  },
  getProject: () => project({ includeAnalysis: false }),
  highlight: highlightParts,
  colorParts,
  editCG,
  showSurfaces: showAeroSurfaces,
  onSave: remember,
  onError: error,
  onRegistered: async id => {
    await analysisUI.useModel(id);
    queueDraft();
    await analysisUI.open();
  }
});
function openDefinition() {
  transform.detach();
  definitionUI.open();
}
$('open-aircraft-definition').onclick = openDefinition;
analysisUI = initAnalysis({
  getProject: () => project({ includeAnalysis: false }),
  openDefinition,
  onChange: () => {
    dirty = true;
    queueDraft();
    updateFlow();
  }
});
$('open-analysis').onclick = $('open-analysis-panel').onclick = () => analysisUI.open();
function syncThemeButton() {
  const light = document.documentElement.dataset.theme === 'light',
    label = light ? '어두운 화면으로 전환' : '밝은 화면으로 전환';
  $('theme-toggle').setAttribute('aria-label', label);
  $('theme-toggle').title = label;
}
$('theme-toggle').onclick = () => {
  const next = document.documentElement.dataset.theme === 'light' ? 'dark' : 'light';
  document.documentElement.dataset.theme = next;
  try {
    localStorage.setItem('dbf-theme', next);
  } catch {}
  syncThemeButton();
  applySceneTheme();
  render();
};
syncThemeButton();
undoStack = [snapshot()];
refresh();
render();
// The editor opens empty. The last non-empty autosave is kept under its own key before this session's
// autosaves overwrite 'draft', and the empty screen offers it back.
try {
  draftDB = await openDraftDB();
  const last = await loadDraft(draftDB);
  if (last && Object.values(last.objects || {}).some(Boolean)) await saveDraft(draftDB, last, PREVIOUS_SESSION);
  const previous = await loadDraft(draftDB, PREVIOUS_SESSION);
  if (previous) {
    const name = previous.objects?.aircraft?.source?.name,
      saved = Date.parse(previous.saved_at);
    $('empty-restore').hidden = false;
    $('empty-restore').title = [name, Number.isFinite(saved) ? new Date(saved).toLocaleString() : '']
      .filter(Boolean)
      .join(' · ');
  }
} catch (e) {
  $('save-state').textContent = '파일 저장 사용';
}
