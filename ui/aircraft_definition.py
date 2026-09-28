"""Aircraft-independent UI definitions, section extraction and real solver inputs.

All geometry is in the imported CAD's SI/FRD frame, independent of scene pose.
No airframe name, mesh index or mass property is inferred from an example.
"""
import copy
import hashlib
import json
import re
from pathlib import Path
import numpy as np

ROLES = {'unassigned', 'main_wing', 'horizontal_tail', 'vertical_tail', 'fuselage',
         'control', 'door', 'equipment', 'excluded'}
LIFTING = {'main_wing', 'horizontal_tail', 'vertical_tail'}


def digest(value):
    # Roundtrip through JSON normalizes integer/float spelling used by browsers.
    def normalize(x):
        if isinstance(x, bool) or x is None: return x
        if isinstance(x, (int, float)): return float(x)
        if isinstance(x, list): return [normalize(v) for v in x]
        if isinstance(x, dict): return {k: normalize(v) for k, v in x.items()}
        return x
    return hashlib.sha256(json.dumps(normalize(value), sort_keys=True, separators=(',', ':'),
                                   ensure_ascii=False, allow_nan=False).encode('utf8')).hexdigest()


def is_blank(value):
    """True for an untouched template: no role, surface, review or physical value.

    The UI creates this template when the definition panel opens. It carries no
    input, so it must not change which registered model the CAD matches.
    Names and source text are ignored because they hold no physics.
    """
    if not isinstance(value, dict) or value.get('reviewed') or value.get('surfaces'):
        return False
    def unset(v): return v is None or isinstance(v, list) and all(unset(x) for x in v)
    def blank_tensor(t):
        return (t is None or isinstance(t, list) and len(t) == 3 and all(isinstance(r, list) and len(r) == 3 for r in t)
                and all(t[i][i] is None for i in range(3)) and all(t[i][j] in (0, None) for i in range(3) for j in range(3) if i != j))
    for part in value.get('parts', []):
        if (part.get('role', 'unassigned') != 'unassigned' or part.get('mass_excluded') or not unset(part.get('mass_kg'))
                or not unset(part.get('cg_m')) or not blank_tensor(part.get('inertia_kgm2'))):
            return False
    mass = value.get('mass', {})
    if not unset(mass.get('mass_kg')) or not unset(mass.get('cg_m')) or not blank_tensor(mass.get('inertia_kgm2')):
        return False
    return all(unset(v) for group in ('physical', 'references') for v in value.get(group, {}).values())


def effective(value):
    """The definition that identifies the aircraft, or None when it is absent or blank."""
    return None if value is None or is_blank(value) else value


def scalar(value, label, lo=None, hi=None):
    if isinstance(value, bool) or value is None:
        raise ValueError(label + ': 숫자를 입력하세요.')
    try: value = float(value)
    except (TypeError, ValueError): raise ValueError(label + ': 숫자를 입력하세요.')
    if not np.isfinite(value) or lo is not None and value < lo or hi is not None and value > hi:
        raise ValueError(label + ': 허용 범위를 확인하세요.')
    return value


def vector(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(label + ': 좌표 세 개를 입력하세요.')
    return np.array([scalar(v, label) for v in value])


def inertia(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(label + ': CG 기준 관성 행렬을 입력하세요.')
    a = np.array([vector(row, label) for row in value])
    if not np.allclose(a, a.T, atol=1e-10, rtol=0):
        raise ValueError(label + ': 관성 행렬은 대칭이어야 합니다.')
    e = np.linalg.eigvalsh(a)
    if e[0] <= 0 or e[2] > e[0] + e[1] + 1e-9:
        raise ValueError(label + ': 양의 주관성과 관성 삼각부등식을 확인하세요.')
    return a


def validate_draft(definition, aircraft):
    """Incomplete drafts are valid to save; physical readiness is checked separately."""
    if definition is None: return
    if not aircraft or not isinstance(definition, dict) or definition.get('schema') != 'dbf-aircraft/1':
        raise ValueError('기체 정의 형식을 확인하세요.')
    parts = definition.get('parts')
    if not isinstance(parts, list) or len(parts) != len(aircraft['parts']):
        raise ValueError('기체 정의의 부품 수가 CAD와 다릅니다. 새 형상에 다시 지정하세요.')
    for i, part in enumerate(parts):
        if not isinstance(part, dict) or part.get('index') != i or part.get('role') not in ROLES:
            raise ValueError('부품 번호 또는 역할이 올바르지 않습니다.')
    if not isinstance(definition.get('surfaces', []), list) or len(definition.get('surfaces', [])) > 100:
        raise ValueError('공력 면은 100개 이하여야 합니다.')
    # Avoid arbitrary/nonfinite structures entering project snapshots.
    if len(json.dumps(definition, allow_nan=False)) > 4_000_000:
        raise ValueError('기체 정의가 너무 큽니다.')


def mass_properties(definition):
    value = definition['mass']
    if value.get('mode') == 'total':
        return dict(mass_kg=scalar(value.get('mass_kg'), '기체 질량', 1e-6),
                    cg_m=vector(value.get('cg_m'), '기체 CG').tolist(),
                    inertia_kgm2=inertia(value.get('inertia_kgm2'), '기체 관성').tolist())
    if value.get('mode') != 'components': raise ValueError('질량 입력 방식을 선택하세요.')
    rows = []
    for p in definition['parts']:
        # Aerodynamic exclusions do not exclude mass: every physical CAD part is accounted for.
        if p.get('mass_excluded') is True: continue
        m = scalar(p.get('mass_kg'), p['name'] + ' 질량', 1e-9)
        cg = vector(p.get('cg_m'), p['name'] + ' CG')
        j = inertia(p.get('inertia_kgm2'), p['name'] + ' 관성')
        rows.append((m, cg, j))
    if not rows: raise ValueError('질량을 합산할 부품이 없습니다.')
    mass = sum(r[0] for r in rows)
    cg = sum(m*p for m, p, _ in rows) / mass
    j = sum(j + m*((p-cg)@(p-cg)*np.eye(3) - np.outer(p-cg, p-cg)) for m, p, j in rows)
    return dict(mass_kg=mass, cg_m=cg.tolist(), inertia_kgm2=j.tolist())


def selected_mesh(aircraft, indices):
    if not isinstance(indices, list) or not indices or len(set(indices)) != len(indices):
        raise ValueError('공력 면에 사용할 부품을 선택하세요.')
    triangles = []
    for i in indices:
        if type(i) is not int or not 0 <= i < len(aircraft['parts']):
            raise ValueError('선택한 부품이 없습니다.')
        p = aircraft['parts'][i]
        v = np.asarray(p['positions'], float).reshape(-1, 3)
        f = np.asarray(p['indices'], int).reshape(-1, 3)
        triangles.append(v[f])
    return np.concatenate(triangles)


def extract_sections(aircraft, request):
    """Intersect the selected tessellated surface, preserving measured camber.

    This is a proposed lifting surface, never an automatic semantic assignment.
    Disconnected sections/gaps are rejected rather than bridged with a fake foil.
    """
    tri = selected_mesh(aircraft, request['parts'])
    axis = {'y': 1, 'z': 2}.get(request.get('axis'))
    if axis is None: raise ValueError('단면을 자를 축 Y 또는 Z를 선택하세요.')
    other = 2 if axis == 1 else 1
    stations = request.get('stations_m')
    if not isinstance(stations, list) or not 2 <= len(stations) <= 40:
        raise ValueError('단면 위치를 2~40개 지정하세요.')
    result = []
    for station in stations:
        station = scalar(station, '단면 위치')
        coords = tri[:, :, axis]
        intersects = tri[(coords.min(1) < station) & (coords.max(1) > station)]
        segments = []
        for t in intersects:
            hits = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                d = t[b, axis] - t[a, axis]
                if abs(d) > 1e-14:
                    u = (station-t[a, axis])/d
                    if -1e-10 <= u <= 1+1e-10:
                        hits.append(t[a]+u*(t[b]-t[a]))
            hits = np.unique(np.round(hits, 11), axis=0)
            if len(hits) == 2: segments.append(hits[:, [0, other]])
        if len(segments) < 4:
            raise ValueError(f'{station:g} m: 닫힌 날개 단면을 찾지 못했습니다. 끝면을 피해서 위치를 지정하세요.')
        seg = np.asarray(segments)
        front, rear = seg[:, :, 0].max(), seg[:, :, 0].min()
        chord = front-rear
        if chord <= 1e-5: raise ValueError('단면 시위가 너무 작습니다.')
        x = (1-np.cos(np.linspace(0, np.pi, 81)))/2
        upper, lower = [], []
        for u in x:
            cut = front-np.clip(u, 1e-6, 1-1e-6)*chord
            hits = []
            for a, b in seg:
                if abs(b[0]-a[0]) < 1e-14: continue
                w = (cut-a[0])/(b[0]-a[0])
                if -1e-9 <= w <= 1+1e-9: hits.append(a[1]+w*(b[1]-a[1]))
            hits = np.unique(np.round(hits, 9))
            if len(hits) != 2:
                raise ValueError(f'{station:g} m: 단면이 여러 겹이거나 끊겨 있습니다. 주익과 조종면 등 연결된 외피만 선택하세요.')
            # Horizontal sections: FRD -Z is airfoil up; vertical: +Y.
            upper.append(-min(hits) if axis == 1 else max(hits))
            lower.append(-max(hits) if axis == 1 else min(hits))
        center_le = (upper[0]+lower[0])/2
        center_te = (upper[-1]+lower[-1])/2
        incidence = np.degrees(np.arctan2(center_le-center_te, chord))
        baseline = center_le + x*(center_te-center_le)
        foil = np.vstack((np.c_[x[::-1], (np.array(upper)-baseline)[::-1]/chord],
                          np.c_[x[1:], (np.array(lower)-baseline)[1:]/chord]))
        le = [front, 0., 0.];le[axis] = station;le[other] = -center_le if axis == 1 else center_le
        result.append(dict(le_m=le, chord_m=chord, incidence_deg=incidence,
                           foil={'kind': 'coordinates', 'points': foil.tolist(), 'source': 'CAD mesh intersection'},
                           span_panels=8))
    return dict(sections=result, note='선택한 CAD 메시를 절단한 단면입니다. 단면 사이를 선형 보간합니다. 루트·팁 위치와 빠진 면적을 확인하고 수정하세요.')


def foil_points(value):
    if value.get('kind') == 'naca4':
        code = value.get('code', '')
        if not re.fullmatch(r'\d{4}', code): raise ValueError('NACA 4자리 번호를 입력하세요.')
        m, p, t = int(code[0])/100, int(code[1])/10, int(code[2:])/100
        if t <= 0 or (m > 0 and p == 0): raise ValueError('NACA 두께와 캠버 위치를 확인하세요.')
        x = (1-np.cos(np.linspace(0, np.pi, 81)))/2
        yt = 5*t*(.2969*np.sqrt(x)-.126*x-.3516*x*x+.2843*x**3-.1036*x**4)
        yc = np.zeros_like(x);slope = np.zeros_like(x)
        if m:
            k = x < p
            yc[k] = m/p**2*(2*p*x[k]-x[k]**2);slope[k] = 2*m/p**2*(p-x[k])
            yc[~k] = m/(1-p)**2*((1-2*p)+2*p*x[~k]-x[~k]**2);slope[~k] = 2*m/(1-p)**2*(p-x[~k])
        theta = np.arctan(slope)
        return np.vstack((np.c_[x-yt*np.sin(theta), yc+yt*np.cos(theta)][::-1],
                          np.c_[x+yt*np.sin(theta), yc-yt*np.cos(theta)][1:]))
    if value.get('kind') != 'coordinates': raise ValueError('단면 에어포일을 지정하세요.')
    a = np.asarray(value.get('points'), float)
    if a.ndim != 2 or a.shape[1] != 2 or not 8 <= len(a) <= 2000 or not np.isfinite(a).all():
        raise ValueError('에어포일 좌표가 올바르지 않습니다.')
    le = int(np.argmin(a[:, 0]))
    if not 1 < le < len(a)-2 or abs(a[:, 0].min()) > .05 or abs(a[:, 0].max()-1) > .05:
        raise ValueError('에어포일은 시위 1 기준으로 뒷전→앞전→뒷전 순서여야 합니다.')
    if np.any(np.diff(a[:le+1, 0]) > .005) or np.any(np.diff(a[le:, 0]) < -.005):
        raise ValueError('에어포일 좌표 순서를 확인하세요.')
    return a


def export_aero(definition, aircraft):
    validate_draft(definition, aircraft)
    surfaces = definition.get('surfaces', [])
    if not surfaces: raise ValueError('공력 면을 하나 이상 정의하세요.')
    refs = definition.get('references', {})
    sr, cr, br = [scalar(refs.get(k), label, 1e-6) for k, label in
                  [('area_m2', '기준 날개 면적'), ('chord_m', '기준 평균 시위'), ('span_m', '기준 날개폭')]]
    # Both solvers rotate about the explicitly supplied aircraft CG.
    reference=np.asarray(mass_properties(definition)['cg_m'])*[-1.,1.,-1.]
    lines = ['UI aircraft lifting surfaces', '0.0', '0 0 0', f'{sr} {cr} {br}', ' '.join(f'{x:.17g}' for x in reference), '0']
    files = [];control_order = [];coverage = {};described = set()
    for n, surface in enumerate(surfaces):
        indices = surface.get('parts', []);selected_mesh(aircraft, indices)
        axis = surface.get('axis');mirror = surface.get('mirror') is True
        if axis not in ('y', 'z') or mirror and axis != 'y': raise ValueError('대칭 복제는 Y 방향 면에만 적용하세요.')
        sections = surface.get('sections', [])
        if not 2 <= len(sections) <= 40: raise ValueError('공력 면마다 단면을 2~40개 입력하세요.')
        lines += ['SURFACE', f'Surface_{n}', '8 1.0']
        if mirror: lines += ['YDUPLICATE', '0.0']
        control = surface.get('control', 'none')
        if control not in ('none', 'aileron', 'elevator', 'rudder'): raise ValueError('조종면 종류를 확인하세요.')
        if control != 'none':
            if control not in control_order: control_order.append(control)
            hinge = scalar(surface.get('hinge_fraction'), '조종면 힌지 위치', .05, .98)
        stations = []
        for j, section in enumerate(sections):
            le = vector(section.get('le_m'), '단면 앞전 위치');stations.append(le[1 if axis == 'y' else 2])
            chord = scalar(section.get('chord_m'), '시위', 1e-5)
            angle = scalar(section.get('incidence_deg'), '단면 설치각', -45, 45)
            ny = scalar(section.get('span_panels', 8), '분할 수', 2, 80)
            if ny != int(ny): raise ValueError('분할 수는 정수여야 합니다.')
            foil = foil_points(section['foil']);filename = f'foil_{n}_{j}.dat'
            text = filename+'\n'+'\n'.join(f'{x:.10g} {y:.10g}' for x, y in foil)+'\n'
            files.append(dict(name=filename, text=text))
            avl = le*np.array([-1, 1, -1])
            lines += ['SECTION', ' '.join(map(str, [*avl, chord, angle, int(ny), 1])), 'AFILE', filename]
            if control != 'none':
                lines += ['CONTROL', f'{control} 1 {hinge} 0 0 0 '+('-1' if control == 'aileron' else '1')]
        if mirror and min(stations) < -1e-9: raise ValueError('대칭 복제 면은 Y ≥ 0인 반쪽만 정의하세요.')
        if axis == 'y' and np.any(np.diff(stations) <= 0) or axis == 'z' and np.any(np.diff(stations) >= 0):
            raise ValueError('Y 면은 Y 증가, 수직면은 Z 감소 순서로 단면을 입력하세요.')
        intervals = [(min(stations), max(stations))]
        if mirror: intervals.append((-max(stations), -min(stations)))
        for index in indices:
            for old_axis, old_intervals in coverage.get(index, []):
                if old_axis != axis or any(min(b, d)-max(a, c) > 1e-8
                    for a, b in intervals for c, d in old_intervals):
                    raise ValueError('같은 CAD 부품의 공력 구간이 겹칩니다. 부분 조종면은 span 구간을 나누어 정의하세요.')
            coverage.setdefault(index, []).append((axis, intervals))
        described.update(indices)
    for p in definition['parts']:
        if p['role'] == 'unassigned': raise ValueError(p['name'] + ': 부품 역할을 지정하세요.')
        if p['role'] in LIFTING | {'control'} and p['index'] not in described:
            raise ValueError(p['name'] + ': 공력 면에 연결하세요.')
    files.insert(0, dict(name='aircraft.avl', text='\n'.join(lines)+'\n'))
    controls = {k: control_order.index(k)+1 if k in control_order else None for k in ('elevator', 'aileron', 'rudder')}
    return dict(files=files, controls=controls, refs=[sr, cr, br],
                limitations=['양력면 모델입니다. 동체·문·장비의 압력 분포는 포함하지 않으며 별도 입력한 추가 항력을 사용합니다.',
                             '조종면은 지정한 면의 전체 span에 적용됩니다. 일부 구간 조종면은 별도 공력 면으로 나누세요.'])


def generic_case(definition, project, aero):
    """Explicit point-mass defaults, independent of any named example aircraft."""
    mass = mass_properties(definition);physical = definition.get('physical', {})
    cd = scalar(physical.get('profile_cd'), '기체 추가 항력계수', 0, 3)
    thrust = scalar(physical.get('max_thrust_N'), '최대 추력', .001)
    thrust_point = vector(physical.get('thrust_point_m'), '추력 작용점') - mass['cg_m']
    if not definition.get('source', '').strip(): raise ValueError('물성·형상 출처 또는 가정을 적어 주세요.')
    length = project['cable']['length_m'];stored = min(.02, length*.05)
    defaults = definition.get('operating', {})
    speed = scalar(defaults.get('speed_m_s', 20), '속도', 1.01, 100)
    rho = scalar(defaults.get('rho_kg_m3', 1.225), '밀도', .05, 2)
    c = dict(name=definition['name'],
        aircraft={**mass, 'tow_point_m': [0., 0., 0.], 'thrust_point_m': thrust_point.tolist(),
                  'profile_cd': cd, 'max_thrust_N': thrust},
        sensor=dict(model='point_mass', mass_kg=project['objects']['sensor']['mass_kg'], tow_point_m=[0., 0., 0.]),
        cable=dict(length_m=length, segments=10, density_kg_m=.0015, diameter_m=project['cable']['diameter_m'],
                   EA_N=60., damping_Ns_m=.03, damping_engagement_strain=.01, cd_normal=1.2, cd_tangent=.02,
                   spool_k_N_m=800., spool_c_Ns_m=3., limit_N=None, guide_points_body_m=[]),
        # Inert legacy contact fields retained only for the shared case schema; contact is disabled.
        bay=dict(stowed_center_m=[0., 0., stored], exit_x_m=-.1, front_x_m=.1, half_width_m=.1,
                 floor_z_m=.1, ceiling_z_m=-.1, ramp_slope=0., lip_depth_m=.01, door_length_m=.1,
                 door_thickness_m=.001, wall_thickness_m=.001, contact_radius_m=.001, release_push_N=0.,
                 contact_k_N_m=0., contact_c_Ns_m=0., friction=0., latch_k_N_m=0., latch_c_Ns_m=0.,
                 latch_kr_Nm_rad=0., latch_cr_Nms_rad=0., capture_radius_m=.001, capture_speed_m_s=.001,
                 capture_angle_deg=1., limit_contact_N=None, limit_penetration_m=None),
        winch=dict(stowed_length_m=stored, radius_m=.015, limit_torque_Nm=None, release_s=0., recovery_start_s=0.,
                   door_schedule=[[0., 0.], [2., 0.]], length_schedule=[[0., length], [2., length]]),
        flight=dict(speed_m_s=speed, altitude_m=100., rho_kg_m3=rho, g_m_s2=9.80665,
                    wind_ned_m_s=[0., 0., 0.], local_flow_factor=1., controls={}, gust=None,
                    controller=dict(enabled=False)),
        simulation=dict(duration_s=2., sample_dt_s=.02, max_step_s=.025, rtol=1e-5, atol=1e-7,
                        maximum_runtime_s=1800.),
        collision=dict(enabled=False),
        aero=dict(executable='vendor/avl/avl.exe', flow5_executable='vendor/flow5/flow5.exe', geometry='',
                  backend='avl', elevator_index=aero['controls']['elevator'],
                  lateral_control_indices=[aero['controls']['aileron'], aero['controls']['rudder']],
                  moment_reference_frd_m=list(mass['cg_m']), alpha_deg=[-4., 0., 4., 8., 12.],
                  beta_deg=[-8., 0., 8.], elevator_deg=[-8., 0., 8.]), provenance={})
    if physical.get('flow5_closure_confirmed') is True:
        c['aero']['rate_derivative_closure']='classical_longitudinal_lateral'
        c['aero']['flow5_method']='VLM2'
    for name in ('aircraft', 'sensor', 'cable', 'bay', 'winch', 'flight', 'aero'):
        c['provenance'][name] = dict(kind='assumption', source='UI generic point-mass defaults; review physical inputs before analysis.')
    c['provenance']['aircraft'] = dict(kind=definition.get('source_kind', 'assumption'), source=definition['source'])
    c['provenance']['aero'] = dict(kind='calculated', source='AVL inputs generated from user-confirmed UI sections; '+ '; '.join(aero['limitations']))
    c['provenance']['bay']['source'] = 'No CAD contact or capture in deployed flight/recovery. Inactive shared-schema fields only.'
    return c


def preview(project):
    definition = project.get('aircraft_definition')
    if not definition: raise ValueError('기체 정의를 먼저 입력하세요.')
    aero = export_aero(definition, project['objects']['aircraft'])
    mass = mass_properties(definition)
    return dict(**aero, mass=mass, definition_sha256=digest(definition))


def register_definition(project):
    import model_registry
    d = project.get('aircraft_definition');result = preview(project)
    if d.get('reviewed') is not True: raise ValueError('공력 단면과 물성 확인을 체크하세요.')
    if not result['controls']['elevator']:
        raise ValueError('현재 전개 비행·회수 트림에는 elevator로 지정한 피치 조종면이 필요합니다.')
    if not project['objects'].get('sensor') or not project['objects'].get('winch'):
        raise ValueError('윈치와 질점을 배치하세요.')
    case = generic_case(d, project, result)
    value = dict(name=d['name'], case_text=json.dumps(case), files=result['files'],
                 aircraft_cg_local_m=result['mass']['cg_m'], sensor_cg_local_m=[0., 0., 0.],
                 door_parts=[], door_reference_deg=0., source_confirmed=True)
    return model_registry.register(project, value, definition=d)
