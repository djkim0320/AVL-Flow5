// Persisted definitions use CAD-local SI/FRD coordinates, never scene coordinates.
import { Matrix4, Quaternion } from 'three';
export const roleLabels = {
  unassigned: '미지정',
  main_wing: '주익',
  horizontal_tail: '수평 꼬리날개',
  vertical_tail: '수직 꼬리날개',
  aileron: '에일러론',
  elevator: '승강타',
  rudder: '방향타',
  fuselage: '동체',
  landing_gear: '착륙장치',
  door: '문',
  equipment: '장비·구조물',
  excluded: '공력 제외',
  // Older definitions: one generic control role. Kept readable, no longer offered.
  control: '조종면 · 종류 미지정'
};
// Roles offered in the picker, in the order they are shown. Lifting and control roles must be in a surface.
export const roleGroups = [
  ['양력면', ['main_wing', 'horizontal_tail', 'vertical_tail']],
  ['조종면', ['aileron', 'elevator', 'rudder']],
  ['기타', ['fuselage', 'landing_gear', 'door', 'equipment', 'excluded']]
];
export const controlRoles = ['aileron', 'elevator', 'rudder'];
export const roleColors = {
  main_wing: '#4c8dff',
  horizontal_tail: '#2fb5a3',
  vertical_tail: '#8b7cf6',
  aileron: '#ff8a3d',
  elevator: '#f2b13b',
  rudder: '#ef5f7a',
  fuselage: '#a7b1bc',
  landing_gear: '#5d6975',
  door: '#c28e5c',
  equipment: '#7f9468',
  excluded: '#39414b',
  control: '#ff8a3d'
};
// First match wins. Names are split into words (on _, -, spaces, digits and camelCase) and English keywords match
// only at a word start, so "final" is not a fin and "trailing" is not a rail. Structure/equipment words come first
// so Sensor_fin, Hatch_latch or Central_wing_saddle are not read as a fin, door or wing; control surfaces precede
// their parent surfaces, and an explicit vertical word precedes the generic stabilizer rule.
const ROLE_PATTERNS = [
  [
    'equipment',
    /\b(latch|hinge|frame|rail|mount|plug|switch|saddle|guide|restraint|divider|floor|envelope|firewall|sensor|winch|batter|motor|servo|receiver|container|prop|boom|strut|spar\b|spars\b)|배터리|모터|윈치|서보|센서|프로펠러/
  ],
  ['aileron', /\b(aileron|flaperon)|에일러론|보조익/],
  ['elevator', /\belevator|승강타/],
  ['rudder', /\brudder|방향타/],
  ['door', /\b(door|hatch)|문|해치/],
  ['landing_gear', /\b(gear|wheel|hub|tire|tyre|skid)|바퀴|착륙/],
  ['vertical_tail', /\b(vertical|v ?tail|fins?\b)|수직/],
  ['horizontal_tail', /\b(horizontal|h ?tail|tailplane|stabili[sz]er|stabilator)|수평/],
  ['main_wing', /wing|주익|날개/],
  ['fuselage', /\b(fuselage|pod\b|pods\b|body|nose|spinner|shell|cowl|canopy)|동체/]
];
export function suggestRole(name) {
  const text = String(name || '')
    .split(' / ')
    .at(-1)
    .replace(/([a-z])([A-Z])/g, '$1 $2')
    .toLowerCase()
    .replace(/[\s_\-.0-9]+/g, ' ');
  return ROLE_PATTERNS.find(([, pattern]) => pattern.test(text))?.[0] ?? null;
}
// Defaults for a new aerodynamic surface built from the selected parts.
export function surfaceDefaults(definition, indices) {
  const roles = new Set(indices.map(i => definition.parts[i].role));
  const controls = controlRoles.filter(r => roles.has(r));
  const lifting = ['main_wing', 'horizontal_tail', 'vertical_tail'].find(r => roles.has(r));
  const control = controls.length === 1 ? controls[0] : 'none';
  return {
    axis: roles.has('vertical_tail') || roles.has('rudder') ? 'z' : 'y',
    control,
    name: [lifting, control === 'none' ? null : control]
      .filter(Boolean)
      .map(r => roleLabels[r])
      .join(' + '),
    controlOnly: controls.length > 0 && !lifting
  };
}
export const blankTensor = () => [
  [null, 0, 0],
  [0, null, 0],
  [0, 0, null]
];
export function newDefinition(aircraft) {
  return {
    schema: 'dbf-aircraft/1',
    name: (aircraft.source?.name || '새 기체').replace(/\.[^.]+$/, ''),
    parts: aircraft.parts.map((p, index) => ({
      index,
      name: p.name || `부품 ${index + 1}`,
      role: 'unassigned',
      mass_kg: null,
      cg_m: [null, null, null],
      inertia_kgm2: blankTensor(),
      mass_excluded: false
    })),
    mass: { mode: 'total', mass_kg: null, cg_m: [null, null, null], inertia_kgm2: blankTensor() },
    physical: { profile_cd: null, max_thrust_N: null, thrust_point_m: [null, null, null] },
    references: { area_m2: null, chord_m: null, span_m: null },
    surfaces: [],
    source: '',
    source_kind: 'assumption',
    reviewed: false
  };
}
export function partBounds(aircraft, indices) {
  const lo = [Infinity, Infinity, Infinity],
    hi = [-Infinity, -Infinity, -Infinity];
  for (const index of indices)
    for (let j = 0; j < aircraft.parts[index].positions.length; j++) {
      const axis = j % 3,
        v = aircraft.parts[index].positions[j];
      lo[axis] = Math.min(lo[axis], v);
      hi[axis] = Math.max(hi[axis], v);
    }
  return { lo, hi, center: lo.map((v, i) => (v + hi[i]) / 2) };
}
export function defaultStations(aircraft, indices, axis, mirror) {
  const b = partBounds(aircraft, indices),
    k = axis === 'y' ? 1 : 2;
  let a = b.lo[k],
    z = b.hi[k];
  if (mirror) a = Math.max(0, a);
  if (z - a < 1e-5) throw new Error('이 방향으로 자를 수 있는 폭이 없습니다. 부품과 절단 축을 확인하세요.');
  const values = [0.02, 0.5, 0.98].map(u => a + (z - a) * u);
  return axis === 'z' ? values.reverse() : values;
}
export function localFromWorld(world, origin, quaternion) {
  // q^-1 (world - origin) q; no Three dependency for persistence/coordinate tests.
  const v = world.map((x, i) => x - origin[i]),
    [x, y, z, w] = quaternion;
  const qv = [-x, -y, -z],
    cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const uv = cross(qv, v),
    uuv = cross(qv, uv);
  return v.map((n, i) => n + 2 * (w * uv[i] + uuv[i]));
}
export function worldFromLocal(local, origin, quaternion) {
  const [x, y, z, w] = quaternion,
    qv = [x, y, z],
    cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const uv = cross(qv, local),
    uuv = cross(qv, uv);
  return local.map((n, i) => n + 2 * (w * uv[i] + uuv[i]) + origin[i]);
}
// The analysis always reads CAD +X as forward (air flows +X → −X). Reorienting therefore
// bakes an exact quarter turn into the vertices instead of changing the display pose.
export function quarterTurn(axis, sign) {
  const s = sign < 0 ? -1 : 1;
  return axis === 'x'
    ? [
        [1, 0, 0],
        [0, 0, -s],
        [0, s, 0]
      ]
    : axis === 'y'
      ? [
          [0, 0, s],
          [0, 1, 0],
          [-s, 0, 0]
        ]
      : [
          [0, -s, 0],
          [s, 0, 0],
          [0, 0, 1]
        ];
}
export const applyMatrix = (m, v) => [0, 1, 2].map(r => m[r][0] * v[0] + m[r][1] * v[1] + m[r][2] * v[2] + 0);
const multiply = (a, b) => a.map(r => [0, 1, 2].map(j => r[0] * b[0][j] + r[1] * b[1][j] + r[2] * b[2][j] + 0));
export function reorientAsset(asset, axis, sign) {
  const m = quarterTurn(axis, sign),
    identity = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ];
  const parts = asset.parts.map(p => {
    const q = new Array(p.positions.length);
    for (let i = 0; i < q.length; i += 3) {
      const v = applyMatrix(m, [p.positions[i], p.positions[i + 1], p.positions[i + 2]]);
      q[i] = v[0];
      q[i + 1] = v[1];
      q[i + 2] = v[2];
    }
    return { ...p, positions: q };
  });
  const source = {
    ...asset.source,
    orientation_matrix: multiply(m, asset.source?.orientation_matrix || identity),
    orientation_turns: [...(asset.source?.orientation_turns || []), `${axis.toUpperCase()}${sign < 0 ? '−' : '+'}90°`]
  };
  const { id, ...rest } = asset;
  const result = { ...rest, parts, source };
  if (asset.components) {
    const turn = new Quaternion().setFromRotationMatrix(
      new Matrix4().set(
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
    result.components = asset.components.map(c => ({
      ...c,
      position: applyMatrix(m, c.position),
      quaternion: turn
        .clone()
        .multiply(new Quaternion(...c.quaternion))
        .normalize()
        .toArray()
    }));
  }
  return result;
}
// Mirrors aircraft_definition.is_blank: a template with no role, surface, review or physical value.
export function isBlankDefinition(d) {
  if (!d || typeof d !== 'object' || d.reviewed || d.surfaces?.length) return false;
  const unset = v => v == null || (Array.isArray(v) && v.every(unset));
  const blankTensor = t =>
    t == null ||
    (Array.isArray(t) &&
      t.length === 3 &&
      t.every(r => Array.isArray(r) && r.length === 3) &&
      [0, 1, 2].every(i => t[i][i] == null) &&
      [0, 1, 2].every(i => [0, 1, 2].every(j => i === j || t[i][j] === 0 || t[i][j] == null)));
  for (const p of d.parts || [])
    if (
      (p.role ?? 'unassigned') !== 'unassigned' ||
      p.mass_excluded ||
      !unset(p.mass_kg) ||
      !unset(p.cg_m) ||
      !blankTensor(p.inertia_kgm2)
    )
      return false;
  const m = d.mass || {};
  if (!unset(m.mass_kg) || !unset(m.cg_m) || !blankTensor(m.inertia_kgm2)) return false;
  return ['physical', 'references'].every(g => Object.values(d[g] || {}).every(unset));
}
// Symmetric tow trim needs the winch feed on the aircraft's CG plane (CAD-local Y).
export function onCenterline(world, origin, quaternion, y = 0) {
  const local = localFromWorld(world, origin, quaternion);
  local[1] = y;
  return worldFromLocal(local, origin, quaternion);
}
export function parseFoil(text) {
  const rows = text
    .trim()
    .split(/\r?\n/)
    .map(x => x.trim())
    .filter(Boolean);
  if (!/^[-+\d.]/.test(rows[0])) rows.shift();
  const points = rows.map(r => r.split(/\s+/).map(Number));
  if (points.length < 8 || points.some(r => r.length !== 2 || !r.every(Number.isFinite)))
    throw new Error('에어포일 DAT에는 제목 다음에 x/c y/c 좌표를 입력하세요.');
  return { kind: 'coordinates', points, source: '사용자 DAT' };
}
