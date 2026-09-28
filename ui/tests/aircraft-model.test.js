import test from 'node:test';
import assert from 'node:assert/strict';
import {
  newDefinition,
  suggestRole,
  surfaceDefaults,
  localFromWorld,
  worldFromLocal,
  onCenterline,
  quarterTurn,
  applyMatrix,
  reorientAsset,
  isBlankDefinition,
  defaultStations,
  parseFoil
} from '../aircraft-model.js';
test('new aircraft keeps unknown physical values empty', () => {
  const d = newDefinition({ source: { name: 'any.step' }, parts: [{ name: 'panel' }] });
  assert.equal(d.parts[0].role, 'unassigned');
  assert.equal(d.mass.mass_kg, null);
  assert.deepEqual(d.mass.cg_m, [null, null, null]);
  assert.equal(d.surfaces.length, 0);
});
test('CG drag is converted from rotated translated world to CAD frame', () => {
  const s = Math.sqrt(0.5),
    r = localFromWorld([5, 8, 9], [5, 6, 9], [0, 0, s, s]);
  assert.ok(Math.abs(r[0] - 2) < 1e-12);
  assert.ok(Math.abs(r[1]) < 1e-12);
  assert.ok(Math.abs(r[2]) < 1e-12);
});
test('world and CAD frames round-trip for a rotated translated aircraft', () => {
  const s = Math.sqrt(0.5),
    o = [5, 6, 9],
    q = [0, 0, s, s],
    w = worldFromLocal([0.3, -0.2, 0.7], o, q),
    l = localFromWorld(w, o, q);
  [0.3, -0.2, 0.7].forEach((v, i) => assert.ok(Math.abs(l[i] - v) < 1e-12));
});
test('default winch lands on the CG plane even when the aircraft is rotated', () => {
  const q = [0, 0, Math.sin(0.3), Math.cos(0.3)],
    o = [1, 2, 3],
    w = onCenterline([1.4, 2.5, 3.2], o, q, 0.002),
    l = localFromWorld(w, o, q);
  assert.ok(Math.abs(l[1] - 0.002) < 1e-12);
  const before = localFromWorld([1.4, 2.5, 3.2], o, q);
  assert.ok(Math.abs(l[0] - before[0]) < 1e-12 && Math.abs(l[2] - before[2]) < 1e-12);
});
test('quarter turns are exact right-handed rotations', () => {
  assert.deepEqual(applyMatrix(quarterTurn('z', 1), [1, 0, 0]), [0, 1, 0]);
  assert.deepEqual(applyMatrix(quarterTurn('y', 1), [0, 0, 1]), [1, 0, 0]);
  assert.deepEqual(applyMatrix(quarterTurn('x', 1), [0, 1, 0]), [0, 0, 1]);
  for (const a of ['x', 'y', 'z'])
    assert.deepEqual(
      applyMatrix(quarterTurn(a, -1), applyMatrix(quarterTurn(a, 1), [0.3, -0.2, 0.7])),
      [0.3, -0.2, 0.7]
    );
});
test('reorienting bakes the turn into CAD vertices and records provenance', () => {
  const asset = {
    id: 'old',
    parts: [{ name: 'nose', positions: [1, 0, 0, 0, 2, 0, 0, 0, 3], indices: [0, 1, 2] }],
    source: { name: 'plane.step', origin: 'reference' }
  };
  const r = reorientAsset(asset, 'z', 1);
  assert.deepEqual(r.parts[0].positions, [0, 1, 0, -2, 0, 0, 0, 0, 3]);
  assert.equal(r.id, undefined);
  assert.equal(r.source.origin, 'reference');
  assert.deepEqual(r.source.orientation_turns, ['Z+90°']);
  assert.deepEqual(asset.parts[0].positions, [1, 0, 0, 0, 2, 0, 0, 0, 3]);
  let back = r;
  for (let i = 0; i < 3; i++) back = reorientAsset(back, 'z', 1);
  assert.deepEqual(back.parts[0].positions, asset.parts[0].positions);
  assert.deepEqual(back.source.orientation_matrix, [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
  ]);
});
test('blank definition template matches the server rule', () => {
  const d = newDefinition({ source: { name: 'a.step' }, parts: [{ name: 'wing' }, { name: 'tail' }] });
  assert.ok(isBlankDefinition(d));
  for (const edit of [
    x => (x.parts[0].role = 'main_wing'),
    x => (x.mass.mass_kg = 2),
    x => (x.mass.inertia_kgm2[1][1] = 0.2),
    x => (x.references.span_m = 1.5),
    x => (x.reviewed = true),
    x => x.surfaces.push({})
  ]) {
    const e = structuredClone(d);
    edit(e);
    assert.equal(isBlankDefinition(e), false);
  }
});
test('vertical section proposal runs root to tip with descending FRD Z', () => {
  const a = { parts: [{ positions: [0, 0, -1, 0, 0, 0] }] };
  assert.deepEqual(defaultStations(a, [0], 'z', false), [-0.020000000000000018, -0.5, -0.98]);
});
test('invalid DAT never becomes a substitute foil', () => {
  assert.throws(() => parseFoil('name\n1 0\n0 0\n1 0'));
});

test('part names suggest roles; structure words win over fin, wing and hatch', () => {
  const cases = {
    'S27.step / Aileron_R': 'aileron',
    Elevator_L: 'elevator',
    Rudder_R: 'rudder',
    Horizontal_stabilizer: 'horizontal_tail',
    Vertical_fin_L: 'vertical_tail',
    Wing_L_outer: 'main_wing',
    Pod_shell_L: 'fuselage',
    Nose_gear: 'landing_gear',
    Main_hub_1: 'landing_gear',
    Cargo_hatch_R: 'door',
    Sensor_fin_2: 'equipment',
    Central_wing_saddle: 'equipment',
    Hatch_latch_L_422: 'equipment',
    Tail_boom_L: 'equipment',
    '보조익 오른쪽': 'aileron',
    Vertical_stabilizer: 'vertical_tail',
    'asm.step / Vertical_stabiliser': 'vertical_tail',
    VerticalStabilizer: 'vertical_tail',
    main_wing_final: 'main_wing',
    Wing_trailing_edge: 'main_wing',
    Tripod_mount: 'equipment',
    Spare_wing: 'main_wing',
    '부품 7': null
  };
  for (const [name, role] of Object.entries(cases)) assert.equal(suggestRole(name), role, name);
});

test('a new surface takes its axis, control and name from the selected roles', () => {
  const d = {
    parts: ['main_wing', 'aileron', 'vertical_tail', 'rudder', 'elevator'].map((role, index) => ({ index, role }))
  };
  assert.deepEqual(surfaceDefaults(d, [0, 1]), {
    axis: 'y',
    control: 'aileron',
    name: '주익 + 에일러론',
    controlOnly: false
  });
  assert.deepEqual(surfaceDefaults(d, [2, 3]), {
    axis: 'z',
    control: 'rudder',
    name: '수직 꼬리날개 + 방향타',
    controlOnly: false
  });
  assert.equal(surfaceDefaults(d, [4]).controlOnly, true);
  assert.equal(surfaceDefaults(d, [1, 4]).control, 'none');
});
