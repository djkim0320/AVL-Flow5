"""Local assembly editor. Uploaded STEP conversion uses isolated worker processes."""

import os

for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
from pathlib import Path
import argparse, hashlib, json, multiprocessing, uuid
import traceback
from concurrent.futures import ProcessPoolExecutor
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs, unquote
import numpy as np
from .analysis_jobs import JobManager
from .analysis_bridge import catalog, prepare
from . import model_registry
from .paths import UI, DATA, TEMPLATES
import yaml
from scipy.spatial.transform import Rotation
from dbf_stability.compute_resources import apply_affinity
from .aircraft_definition import validate_draft, extract_sections, mass_properties, preview
from .assembly import validate_components

MAX_BYTES = 64 * 1024**2


def worker_init():
    apply_affinity()


def step_part_names(path, solids):
    """Names of the STEP assembly nodes, one per solid in `solids` order, or None when they cannot be matched."""
    import cadquery as cq

    try:
        assembly = cq.Assembly.importStep(str(path))
    except Exception:
        return None
    named = []

    def walk(node, location):
        location = location * node.loc
        if node.obj is not None:
            shape = node.obj if isinstance(node.obj, cq.Shape) else node.obj.val()
            for solid in shape.moved(location).Solids():
                named.append((node.name, solid.Center()))
        for child in node.children:
            walk(child, location)

    walk(assembly, cq.Location())
    if len(named) != len(solids):
        return None
    names = []
    for (name, center), solid in zip(named, solids):
        if (center - solid.Center()).Length > 1e-3 * max(solid.BoundingBox().DiagonalLength, 1):
            return None
        names.append(str(name)[:120])
    return names


def convert_step(path):
    import cadquery as cq

    solids = cq.importers.importStep(str(path)).solids().vals()
    names = step_part_names(path, solids) or [f'부품 {i + 1}' for i in range(len(solids))]
    parts = []
    for name, solid in zip(names, solids):
        vertices, faces = solid.tessellate(0.2, 0.15)
        parts.append(
            dict(
                name=name,
                positions=np.array([v.toTuple() for v in vertices]).ravel().tolist(),
                indices=np.asarray(faces, dtype=int).ravel().tolist(),
            )
        )
    if not parts:
        raise ValueError('STEP 파일에 닫힌 솔리드가 없습니다. 솔리드 STEP으로 다시 내보내 주세요.')
    return dict(parts=parts, source_units='mm', sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


def validate_project(value):
    if not isinstance(value, dict) or value.get('schema') not in ('dbf-assembly/1', 'dbf-assembly/2'):
        raise ValueError('DBF 배치 파일 형식이 아닙니다. 이 편집기에서 저장한 JSON을 선택해 주세요.')
    if value.get('units') != 'm' or value.get('axes') != 'FRD':
        raise ValueError('저장 좌표는 SI / FRD여야 합니다.')
    objects = value.get('objects')
    if not isinstance(objects, dict) or set(objects) - {'aircraft', 'sensor', 'winch'}:
        raise ValueError('모델 목록을 확인해 주세요.')
    for role, item in objects.items():
        if item is None:
            continue
        if value['schema'] == 'dbf-assembly/2' and role == 'sensor':
            if not isinstance(item, dict) or item.get('kind') != 'point_mass' or 'parts' in item:
                raise ValueError('센서는 형상 없는 질점이어야 합니다.')
            mass = item.get('mass_kg')
            if (
                isinstance(mass, bool)
                or not isinstance(mass, (int, float))
                or not np.isfinite(mass)
                or not 0.0001 <= mass <= 10
            ):
                raise ValueError('질점 질량은 0.0001~10 kg이어야 합니다.')
            p = np.asarray(item.get('position'), float)
            if p.shape != (3,) or not np.isfinite(p).all():
                raise ValueError('질점 위치가 올바르지 않습니다.')
            if item.get('quaternion') != [0, 0, 0, 1]:
                raise ValueError('질점에는 회전 자세가 없습니다.')
            continue
        if not isinstance(item, dict) or not isinstance(item.get('parts'), list) or not item['parts']:
            raise ValueError('모델에 표시할 표면이 없습니다.')
        for key, count in [('position', 3), ('quaternion', 4)]:
            a = np.asarray(item[key], float)
            if a.shape != (count,) or not np.isfinite(a).all():
                raise ValueError('위치 또는 회전 값이 올바르지 않습니다.')
        if abs(np.linalg.norm(item['quaternion']) - 1) > 1e-5:
            raise ValueError('회전 값이 정규화되지 않았습니다.')
        for part in item['parts']:
            p = np.asarray(part['positions'], float)
            f = np.asarray(part['indices'])
            if p.ndim != 1 or p.size == 0 or p.size % 3 or not np.isfinite(p).all():
                raise ValueError('모델 꼭짓점이 올바르지 않습니다.')
            if (
                f.ndim != 1
                or f.size == 0
                or f.size % 3
                or not np.isfinite(f).all()
                or np.any(f != np.floor(f))
                or f.min() < 0
                or f.max() >= p.size // 3
            ):
                raise ValueError('모델 표면 인덱스가 올바르지 않습니다.')
        if sum(len(p['indices']) // 3 for p in item['parts']) > 1000000:
            raise ValueError('모델 삼각형은 100만 개 이하여야 합니다.')
        if not isinstance(item.get('source'), dict) or not isinstance(item['source'].get('origin'), str):
            raise ValueError('모델 원점 정보가 없습니다.')

        validate_components(item)
    cable = value.get('cable')

    validate_draft(value.get('aircraft_definition'), objects.get('aircraft'))
    if not isinstance(cable, dict):
        raise ValueError('줄 설정이 없습니다.')
    d = float(cable.get('diameter_m', 0))
    length = float(cable.get('length_m', 0))
    if not np.isfinite([d, length]).all() or not 0.00005 <= d <= 0.02 or length <= 0:
        raise ValueError('줄 직경은 0.05~20 mm, 길이는 0보다 커야 합니다.')
    if value['schema'] == 'dbf-assembly/2':
        attached = bool(objects.get('sensor') and objects.get('winch'))
        expected = {'kind': 'point_mass', 'local_point_m': [0, 0, 0]} if attached else None
        if cable.get('attachment') != expected:
            raise ValueError('질점과 윈치의 자동 연결 정보가 올바르지 않습니다.')
        if attached:
            q = (objects.get('aircraft') or {}).get('quaternion', [0, 0, 0, 1])
            # Read old full-length previews as well as the new stowed preview.
            expected_positions = [
                np.asarray(objects['winch']['position']) + Rotation.from_quat(q).apply([0, 0, distance])
                for distance in (min(0.02, length * 0.05), length)
            ]
            if not any(
                np.allclose(objects['sensor']['position'], position, rtol=0, atol=1e-7)
                for position in expected_positions
            ):
                raise ValueError('질점 미리보기 위치가 윈치 위치·줄 길이와 일치하지 않습니다.')
        return value
    if cable.get('attachment') is not None:
        if not isinstance(cable['attachment'], dict):
            raise ValueError('줄 연결점 형식이 올바르지 않습니다.')
        if not objects.get('sensor') or not objects.get('winch'):
            raise ValueError('줄 연결에는 센서와 윈치가 필요합니다.')
        a = cable['attachment']
        parts = objects['sensor']['parts']
        i = a.get('part')
        if type(i) is not int or not 0 <= i < len(parts):
            raise ValueError('연결한 센서 표면이 없습니다.')
        face = a.get('face')
        f = np.asarray(parts[i]['indices']).reshape(-1, 3)
        if type(face) is not int or not 0 <= face < len(f):
            raise ValueError('연결한 삼각형이 없습니다.')
        bary = np.asarray(a['barycentric'], float)
        local = np.asarray(a['local_point_m'], float)
        if bary.shape != (3,) or not np.isfinite(bary).all() or bary.min() < -1e-6 or abs(bary.sum() - 1) > 1e-6:
            raise ValueError('표면 연결 좌표가 올바르지 않습니다.')
        p = np.asarray(parts[i]['positions'], float).reshape(-1, 3)
        if local.shape != (3,) or not np.allclose(bary @ p[f[face]], local, rtol=0, atol=1e-6):
            raise ValueError('연결점이 저장된 센서 표면과 일치하지 않습니다.')
    return value


STATIC_SUFFIXES = {'.html', '.js', '.css', '.json', '.map', '.wasm', '.png', '.svg', '.woff2'}
RESULT_SUFFIXES = {'.html', '.json', '.csv', '.npz', '.npy', '.log'}
RESULT_MIME = {
    '.html': 'text/html; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.csv': 'text/csv; charset=utf-8',
    '.log': 'text/plain; charset=utf-8',
}
# Bad input from the page: reported with the message, status 400. Anything else is a server fault (500).
CLIENT_ERRORS = (ValueError, FileNotFoundError, KeyError, TypeError, json.JSONDecodeError)


class Handler(SimpleHTTPRequestHandler):
    """Routes are looked up in GET_ROUTES / POST_ROUTES: exact paths first, then prefixes."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(UI), **kwargs)

    def end_headers(self):
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def respond(self, value, status=200):
        data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_bytes(self, data, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def fail(self, exc):
        if isinstance(exc, CLIENT_ERRORS):
            message = f'입력 항목이 없습니다: {exc}' if isinstance(exc, KeyError) else str(exc)
            return self.respond(dict(error=message), 400)
        traceback.print_exc()
        return self.respond(
            dict(error=f'서버 내부 오류입니다. 서버 로그를 확인하세요. {type(exc).__name__}: {exc}'), 500
        )

    @staticmethod
    def route(table, path):
        if path in table['exact']:
            return table['exact'][path], None
        for prefix, handler in table['prefix']:
            if path.startswith(prefix):
                return handler, path[len(prefix) :]
        return None, None

    # ------------------------------------------------------------------ GET
    def do_GET(self):
        url = urlparse(self.path)
        handler, rest = self.route(GET_ROUTES, url.path)
        try:
            if handler:
                return handler(self, url, rest)
            if url.path.startswith('/api/'):
                return self.respond(dict(error='없는 요청입니다.'), 404)
            return self.serve_static(url.path)
        except Exception as exc:
            return self.fail(exc)

    def serve_static(self, path):
        # Serve only editor resources, dependencies, and saved project JSON.
        relative = unquote(path).lstrip('/') or 'index.html'
        target = (UI / relative).resolve()
        if not target.is_relative_to(UI) or target.suffix.lower() not in STATIC_SUFFIXES:
            return self.respond(dict(error='열 수 없는 파일입니다.'), 404)
        if path == '/':
            self.path = '/index.html'
        return super().do_GET()

    def get_health(self, url, rest):
        return self.respond(dict(status='ready', step_workers=self.server.pool._max_workers))

    def get_models(self, url, rest):
        return self.respond(model_registry.list_models())

    def get_model_template(self, url, rest):
        return self.respond(yaml.safe_load((TEMPLATES / 'model_physics_template.yaml').read_text('utf8')))

    def get_catalog(self, url, rest):
        model = parse_qs(url.query).get('model', [None])[0]
        if not model:
            raise ValueError('등록 모델을 선택하세요.')
        return self.respond(catalog(model))

    def get_jobs(self, url, rest):
        return self.respond(self.server.jobs.list())

    def get_job(self, url, job_id):
        return self.respond(self.server.jobs.status(job_id))

    def get_analysis_file(self, url, rest):
        parts = unquote(rest).split('/')
        folder = self.server.jobs.folder(parts[0])
        target = (folder / ('/'.join(parts[1:]))).resolve()
        if not target.is_relative_to(folder) or not target.is_file() or target.suffix.lower() not in RESULT_SUFFIXES:
            return self.respond(dict(error='결과 파일이 없습니다.'), 404)
        return self.send_bytes(target.read_bytes(), RESULT_MIME.get(target.suffix, 'application/octet-stream'))

    # ------------------------------------------------------------------ POST
    def do_POST(self):
        url = urlparse(self.path)
        try:
            origin = self.headers.get('Origin')
            port = self.server.server_port
            if origin and origin not in {f'http://127.0.0.1:{port}', f'http://localhost:{port}'}:
                return self.respond(dict(error='로컬 편집기에서 다시 시도해 주세요.'), 403)
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= MAX_BYTES:
                raise ValueError('파일 크기는 64 MB 이하여야 합니다.')
            data = self.rfile.read(size)
            handler, rest = self.route(POST_ROUTES, url.path)
            if not handler:
                return self.respond(dict(error='없는 요청입니다.'), 404)
            return handler(self, url, rest, data)
        except Exception as exc:
            return self.fail(exc)

    @staticmethod
    def project_request(data):
        """Parse a JSON body carrying `project` and validate that project."""
        value = json.loads(data)
        return value, validate_project(value['project'])

    def post_aircraft(self, url, action, data):
        value, project = self.project_request(data)
        if action == 'sections':
            job = self.server.pool.submit(extract_sections, project['objects']['aircraft'], value['request'])
            return self.respond(job.result(timeout=120))
        if action == 'mass':
            return self.respond(mass_properties(project['aircraft_definition']))
        if action == 'preview':
            return self.respond(preview(project))
        if action == 'register':
            return self.respond(model_registry.register_definition(project), 201)
        return self.respond(dict(error='없는 기체 정의 요청입니다.'), 404)

    def post_models_match(self, url, rest, data):
        value, project = self.project_request(data)
        return self.respond(model_registry.match_aircraft(project, value.get('model_id')))

    def post_models_register(self, url, rest, data):
        value, project = self.project_request(data)
        return self.respond(model_registry.register(project, value['registration']), 201)

    def post_analysis_validate(self, url, rest, data):
        value, project = self.project_request(data)
        prepared = prepare(project, value['settings'])
        return self.respond(
            dict(
                ready=True,
                warnings=prepared['warnings'],
                mapping=prepared['mapping'],
                schedule=prepared['schedule'],
                duration_s=prepared['duration_s'],
            )
        )

    def post_analysis_start(self, url, rest, data):
        value, project = self.project_request(data)
        prepared = prepare(project, value['settings'])
        return self.respond(self.server.jobs.start(project, prepared), 202)

    def post_analysis_cancel(self, url, job_id, data):
        return self.respond(self.server.jobs.cancel(job_id))

    def post_import_step(self, url, rest, data):
        name = Path(parse_qs(url.query).get('name', ['model.step'])[0]).name
        if Path(name).suffix.lower() not in ('.step', '.stp'):
            raise ValueError('STEP 또는 STP 파일을 선택해 주세요.')
        directory = DATA / 'uploads'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f'{uuid.uuid4().hex}.step'
        path.write_bytes(data)
        return self.respond(self.server.pool.submit(convert_step, path).result(timeout=180))

    def post_project(self, url, rest, data):
        value = validate_project(json.loads(data))
        directory = DATA / 'projects'
        directory.mkdir(parents=True, exist_ok=True)
        name = uuid.uuid4().hex + '.json'
        (directory / name).write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf8')
        return self.respond(dict(url='/data/projects/' + name, id=name[:-5]))


GET_ROUTES = dict(
    exact={
        '/api/health': Handler.get_health,
        '/api/models': Handler.get_models,
        '/api/models/template': Handler.get_model_template,
        '/api/analysis/catalog': Handler.get_catalog,
        '/api/analysis/jobs': Handler.get_jobs,
    },
    prefix=[('/api/analysis/jobs/', Handler.get_job), ('/analysis-files/', Handler.get_analysis_file)],
)
POST_ROUTES = dict(
    exact={
        '/api/models/match': Handler.post_models_match,
        '/api/models/register': Handler.post_models_register,
        '/api/analysis/validate': Handler.post_analysis_validate,
        '/api/analysis/jobs': Handler.post_analysis_start,
        '/api/import-step': Handler.post_import_step,
        '/api/projects': Handler.post_project,
    },
    prefix=[('/api/aircraft/', Handler.post_aircraft), ('/api/analysis/cancel/', Handler.post_analysis_cancel)],
)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', type=int, default=8767)
    p.add_argument('--workers', type=int, default=2)
    args = p.parse_args()
    if not 1 <= args.workers <= 11:
        raise ValueError('STEP workers must be between 1 and 11')
    worker_init()
    with ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context('spawn'), initializer=worker_init
    ) as pool:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
        server.pool = pool
        server.jobs = JobManager()
        print(f'DBF assembly editor: http://127.0.0.1:{args.port}', flush=True)
        try:
            server.serve_forever()
        finally:
            server.jobs.close()
            server.server_close()


if __name__ == '__main__':
    main()
