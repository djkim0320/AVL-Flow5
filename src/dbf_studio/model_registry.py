"""Immutable user model registrations, tied to CAD signatures and supplied physics."""

from pathlib import PurePosixPath
import copy
import hashlib
import json
import re
import uuid
import numpy as np
import yaml
from .paths import ROOT, DATA
from scipy.spatial import ConvexHull
from dbf_stability.config import validate
from dbf_stability.point_mass import configure
from .aircraft_definition import digest as definition_digest, effective, generic_case, preview

DIRECTORY = DATA / 'models'


def signature(item):
    h = hashlib.sha256()
    for part in item['parts']:
        h.update(part['name'].encode('utf8'))
        p = np.asarray(part['positions'], dtype='<f8').copy()
        p[p == 0] = 0.0
        h.update(p.tobytes())
        h.update(np.asarray(part['indices'], dtype='<i8').tobytes())
    return h.hexdigest()


def vector(value, name):
    a = np.asarray(value, float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError(name + ': 유한한 좌표 3개가 필요합니다.')
    return a.tolist()


def load(model_id):
    if not isinstance(model_id, str) or not re.fullmatch('[0-9a-f]{32}', model_id):
        raise ValueError('등록 모델 ID가 올바르지 않습니다.')
    p = DIRECTORY / model_id / 'model.json'
    if not p.is_file():
        raise ValueError('등록 모델이 없습니다. 해당 PC에 모델을 먼저 등록하세요.')
    model = json.loads(p.read_text('utf8'))
    for name, expected in model['aero_file_sha256'].items():
        if hashlib.sha256((p.parent / 'aero' / name).read_bytes()).hexdigest() != expected:
            raise ValueError('등록 후 공력 입력 파일이 변경됐습니다. 새 모델로 등록하세요: ' + name)
    return model


def list_models():
    rows = []
    for p in sorted(DIRECTORY.glob('*/model.json'), key=lambda p: p.stat().st_mtime, reverse=True):
        value = json.loads(p.read_text('utf8'))
        rows.append(dict(id=value['id'], name=value['name']))
    return rows


def match_aircraft(project, selected_id):
    """Identify matching geometry before showing any model as analysis-ready.

    This does not infer aerodynamics or confirm all solver inputs. The existing
    prepare/catalog checks still validate the selected registration and files.
    """
    aircraft = project['objects'].get('aircraft')
    rows = list_models()
    selected = next((row for row in rows if row['id'] == selected_id), None)
    result = dict(
        aircraft_name=(aircraft or {}).get('source', {}).get('name', '기체 없음'),
        selected=selected,
        matches=[],
        matches_selected=False,
        status='no_aircraft' if not aircraft else 'unregistered',
    )
    if not aircraft:
        return result
    digest = signature(aircraft)
    for row in rows:
        metadata = json.loads((DIRECTORY / row['id'] / 'model.json').read_text('utf8'))
        expected = metadata.get('signatures', {}).get('aircraft')
        if digest == expected:
            current = effective(project.get('aircraft_definition'))
            saved = metadata.get('definition_sha256')
            if current is not None and definition_digest(current) != saved:
                continue
            if saved is not None and current is None:
                continue
            result['matches'].append(row)
    result['matches_selected'] = any(row['id'] == selected_id for row in result['matches'])
    result['status'] = (
        'matched' if result['matches_selected'] else 'wrong_selection' if result['matches'] else 'unregistered'
    )
    return result


def register(project, value, definition=None):
    """Register only supplied, validated geometry and physics."""
    name = value.get('name', '').strip()
    if not name or len(name) > 100:
        raise ValueError('모델 이름은 1~100자로 입력하세요.')
    try:
        config = json.loads(value['case_text'])
    except json.JSONDecodeError:
        config = yaml.safe_load(value['case_text'])
    if not isinstance(config, dict):
        raise ValueError('해석 설정 YAML/JSON이 필요합니다.')
    config = copy.deepcopy(config)
    if config.pop('template_unfilled', False):
        raise ValueError('빈 설정 템플릿의 값을 채운 뒤 template_unfilled를 false로 바꾸세요.')
    for path in config.get('field_help', {}):
        value_at_path = config
        try:
            for key in re.findall(r'[^.\[\]]+', path):
                value_at_path = value_at_path[int(key)] if isinstance(value_at_path, list) else value_at_path[key]
        except (KeyError, IndexError, TypeError) as exc:
            raise ValueError('빈 설정 템플릿의 항목을 확인하세요: ' + path) from exc
        if value_at_path is None:
            raise ValueError('빈 설정 템플릿의 값을 채우세요: ' + path)
    point_mass = project.get('schema') == 'dbf-assembly/2'
    if point_mass:
        sensor = project['objects'].get('sensor')
        if not sensor or sensor.get('kind') != 'point_mass':
            raise ValueError('질점을 먼저 추가하세요.')
        configure(config, sensor['mass_kg'])
    if value.get('source_confirmed') is not True:
        raise ValueError('공력 형상·좌표계·물성 출처를 확인해 주세요.')
    for role in ('aircraft', 'sensor', 'winch'):
        if not project['objects'].get(role):
            raise ValueError('등록하려면 기체·센서·윈치를 먼저 배치하세요.')
        if point_mass and role == 'sensor':
            continue
        names = [p['name'] for p in project['objects'][role]['parts']]
        if len(set(names)) != len(names) and definition is None:
            raise ValueError(role + ': 부품 이름이 중복됩니다. 구분되는 이름을 사용하세요.')
    cg_a = vector(value['aircraft_cg_local_m'], '기체 CAD 기준 CG')
    cg_s = [0.0, 0.0, 0.0] if point_mass else vector(value['sensor_cg_local_m'], '센서 CAD 기준 CG')
    door = value.get('door_parts', [])
    names = [p['name'] for p in project['objects']['aircraft']['parts']]
    if not isinstance(door, list) or any(x not in names for x in door):
        raise ValueError('문으로 지정한 부품 이름이 기체 CAD에 없습니다.')
    angle = float(value['door_reference_deg'])
    if not np.isfinite(angle) or not 0 <= angle <= 360:
        raise ValueError('CAD에 저장된 문 각도를 확인하세요.')
    # Bound the accepted tessellation approximation relative to the explicitly
    # supplied contact skin. Keep fins as separate solids and reject larger gaps.
    geometry_checks = []
    for part in (
        []
        if point_mass
        else project['objects']['sensor']['parts']
        + [p for p in project['objects']['aircraft']['parts'] if p['name'] in door]
    ):
        p = np.asarray(part['positions']).reshape(-1, 3)
        f = np.asarray(part['indices']).reshape(-1, 3)
        p, unique = np.unique(p, axis=0, return_inverse=True)
        f = unique[f]
        p = p - p.mean(0)
        edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        if np.any(counts != 2) or np.any(
            np.linalg.norm(np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]]), axis=1) < 1e-14
        ):
            raise ValueError(part['name'] + ': 닫히지 않거나 겹친 면이 있습니다. 솔리드로 다시 내보내세요.')
        hull = ConvexHull(p)
        volume = abs(np.einsum('ij,ij->i', p[f[:, 0]], np.cross(p[f[:, 1]], p[f[:, 2]])).sum() / 6)
        if part in project['objects']['sensor']['parts']:
            convex = hull.volume > 0 and abs(volume / hull.volume - 1) < 0.001
            # A small notch may have negligible volume but still be important
            # to contact. Require each triangle to lie on the hull boundary.
            tolerance = float(config['collision']['skin_m']) * 0.1
            if not np.isfinite(tolerance) or tolerance <= 0:
                raise ValueError('접촉 간격은 유한한 양수여야 합니다.')
            centroids = p[f].mean(1)
            maximum_gap = 0.0
            for start in range(0, len(centroids), 256):
                distances = centroids[start : start + 256] @ hull.equations[:, :3].T + hull.equations[:, 3]
                maximum_gap = max(maximum_gap, float(-distances.max(1).min()))
                if maximum_gap > tolerance:
                    convex = False
                    break
            if not convex:
                raise ValueError(
                    part['name'] + ': 센서를 닫힌 볼록 부품으로 나눠 주세요. 허용한 메시 근사 범위를 벗어났습니다.'
                )
            geometry_checks.append(
                dict(
                    part=part['name'],
                    centroid_hull_gap_m=maximum_gap,
                    relative_volume_error=abs(volume / hull.volume - 1),
                    tolerance_m=tolerance,
                )
            )
    files = value.get('files', [])
    if not files or len(files) > 100:
        raise ValueError('AVL 형상과 참조 에어포일 파일을 함께 선택하세요.')
    contents = {}
    for file in files:
        path = PurePosixPath(file['name'].replace('\\', '/'))
        if (
            path.is_absolute()
            or '..' in path.parts
            or ':' in str(path)
            or path.suffix.lower() not in ('.avl', '.dat', '.txt', '.mass')
        ):
            raise ValueError('허용하지 않는 공력 파일 경로입니다.')
        if str(path) in contents:
            raise ValueError('공력 파일 이름이 중복됩니다.')
        contents[str(path)] = file['text'].encode('ascii')
    avls = [n for n in contents if n.lower().endswith('.avl')]
    if len(avls) != 1:
        raise ValueError('AVL 형상 파일 하나를 선택하세요.')
    geometry = avls[0]
    lines = contents[geometry].decode('ascii').splitlines()
    for i, line in enumerate(lines):
        if line.strip().split()[:1] in (['AFILE'], ['BFILE']):
            ref = PurePosixPath(geometry).parent / PurePosixPath(lines[i + 1].strip())
            if '..' in ref.parts or str(ref) not in contents:
                raise ValueError('참조 파일 누락: ' + str(ref))
    model_id = uuid.uuid4().hex
    directory = DIRECTORY / model_id
    config['_root'] = str(ROOT)
    config['_source'] = 'UI registration ' + model_id
    installed = {'executable': 'vendor/avl/avl.exe', 'flow5_executable': 'vendor/flow5/bin/flow5_v7.57_win64/flow5.exe'}
    config['aero']['geometry'] = str(directory / 'aero' / geometry)
    for key in ('executable', 'flow5_executable'):
        config['aero'][key] = installed[key]
    # User CAD is the only collision geometry. Example-specific analytic contacts and
    # decomposition rules are not applicable to a new model.
    collision = config.get('collision', {})
    if not point_mass and not collision.get('enabled'):
        raise ValueError('새 모델은 CAD 접촉을 켜고 접촉 물성을 입력해야 합니다.')
    if collision.get('analytic_conical_stop') or collision.get('part_materials'):
        raise ValueError(
            '새 CAD에는 다른 모델의 전용 접촉 설정을 적용할 수 없습니다. analytic_conical_stop과 part_materials를 비운 설정을 사용하세요.'
        )
    collision['mesh_directory'] = str(directory / 'meshes')
    collision['parts_manifest'] = {}
    validate(config)
    model = dict(
        id=model_id,
        name=name,
        config=config,
        aircraft_cg_local_m=cg_a,
        sensor_cg_local_m=cg_s,
        door_parts=door,
        door_reference_deg=angle,
        signatures={k: signature(v) for k, v in project['objects'].items() if v and v.get('kind') != 'point_mass'},
        aero_file_sha256={n: hashlib.sha256(data).hexdigest() for n, data in contents.items()},
        sensor_convex_checks=geometry_checks,
        sources={
            k: (dict(kind='point_mass', mass_kg=v['mass_kg']) if v.get('kind') == 'point_mass' else v['source'])
            for k, v in project['objects'].items()
            if v
        },
    )
    if definition is not None:
        model.update(aircraft_definition=copy.deepcopy(definition), definition_sha256=definition_digest(definition))
    directory.mkdir(parents=True)
    for name, data in contents.items():
        p = directory / 'aero' / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    (directory / 'model.json').write_text(
        json.dumps(model, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8'
    )
    return dict(id=model_id, name=model['name'])


def register_definition(project):
    d = project.get('aircraft_definition')
    result = preview(project)
    if d.get('reviewed') is not True:
        raise ValueError('공력 단면과 물성 확인을 체크하세요.')
    if not result['controls']['elevator']:
        raise ValueError('현재 전개 비행·회수 트림에는 elevator로 지정한 피치 조종면이 필요합니다.')
    if not project['objects'].get('sensor') or not project['objects'].get('winch'):
        raise ValueError('윈치와 질점을 배치하세요.')
    case = generic_case(d, project, result)
    value = dict(
        name=d['name'],
        case_text=json.dumps(case),
        files=result['files'],
        aircraft_cg_local_m=result['mass']['cg_m'],
        sensor_cg_local_m=[0.0, 0.0, 0.0],
        door_parts=[],
        door_reference_deg=0.0,
        source_confirmed=True,
    )
    return register(project, value, definition=d)
