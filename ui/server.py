"""Local assembly editor. Uploaded STEP conversion uses isolated worker processes."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,hashlib,json,multiprocessing,re,sys,uuid
from concurrent.futures import ProcessPoolExecutor
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse,parse_qs,unquote
import numpy as np
from analysis_jobs import JobManager
from analysis_bridge import catalog,prepare
import model_registry

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
MAX_BYTES=64*1024**2


def worker_init():
    from dbf_stability.compute_resources import apply_affinity
    apply_affinity()


def convert_step(path):
    import cadquery as cq
    work=cq.importers.importStep(str(path))
    parts=[]
    for i,solid in enumerate(work.solids().vals()):
        vertices,faces=solid.tessellate(.2,.15)
        parts.append(dict(name=f'부품 {i+1}',positions=np.array([v.toTuple() for v in vertices]).ravel().tolist(),
                          indices=np.asarray(faces,dtype=int).ravel().tolist()))
    if not parts:raise ValueError('STEP 파일에 닫힌 솔리드가 없습니다. 솔리드 STEP으로 다시 내보내 주세요.')
    return dict(parts=parts,source_units='mm',sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())


def validate_project(value):
    if not isinstance(value,dict) or value.get('schema') not in ('dbf-assembly/1','dbf-assembly/2'):
        raise ValueError('DBF 배치 파일 형식이 아닙니다. 이 편집기에서 저장한 JSON을 선택해 주세요.')
    if value.get('units')!='m' or value.get('axes')!='FRD':raise ValueError('저장 좌표는 SI / FRD여야 합니다.')
    objects=value.get('objects')
    if not isinstance(objects,dict) or set(objects)-{'aircraft','sensor','winch'}:raise ValueError('모델 목록을 확인해 주세요.')
    for role,item in objects.items():
        if item is None:continue
        if value['schema']=='dbf-assembly/2' and role=='sensor':
            if not isinstance(item,dict) or item.get('kind')!='point_mass' or 'parts' in item:
                raise ValueError('센서는 형상 없는 질점이어야 합니다.')
            mass=item.get('mass_kg')
            if isinstance(mass,bool) or not isinstance(mass,(int,float)) or not np.isfinite(mass) or not .0001<=mass<=10:
                raise ValueError('질점 질량은 0.0001~10 kg이어야 합니다.')
            p=np.asarray(item.get('position'),float)
            if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('질점 위치가 올바르지 않습니다.')
            if item.get('quaternion')!=[0,0,0,1]:raise ValueError('질점에는 회전 자세가 없습니다.')
            continue
        if not isinstance(item,dict) or not isinstance(item.get('parts'),list) or not item['parts']:
            raise ValueError('모델에 표시할 표면이 없습니다.')
        for key,count in [('position',3),('quaternion',4)]:
            a=np.asarray(item[key],float)
            if a.shape!=(count,) or not np.isfinite(a).all():raise ValueError('위치 또는 회전 값이 올바르지 않습니다.')
        if abs(np.linalg.norm(item['quaternion'])-1)>1e-5:raise ValueError('회전 값이 정규화되지 않았습니다.')
        for part in item['parts']:
            p=np.asarray(part['positions'],float);f=np.asarray(part['indices'])
            if p.ndim!=1 or p.size==0 or p.size%3 or not np.isfinite(p).all():raise ValueError('모델 꼭짓점이 올바르지 않습니다.')
            if f.ndim!=1 or f.size==0 or f.size%3 or not np.isfinite(f).all() or np.any(f!=np.floor(f)) or f.min()<0 or f.max()>=p.size//3:
                raise ValueError('모델 표면 인덱스가 올바르지 않습니다.')
        if sum(len(p['indices'])//3 for p in item['parts'])>1000000:
            raise ValueError('모델 삼각형은 100만 개 이하여야 합니다.')
        if not isinstance(item.get('source'),dict) or not isinstance(item['source'].get('origin'),str):
            raise ValueError('모델 원점 정보가 없습니다.')
    cable=value.get('cable')
    from aircraft_definition import validate_draft
    validate_draft(value.get('aircraft_definition'),objects.get('aircraft'))
    if not isinstance(cable,dict):raise ValueError('줄 설정이 없습니다.')
    d=float(cable.get('diameter_m',0));length=float(cable.get('length_m',0))
    if not np.isfinite([d,length]).all() or not .00005<=d<=.02 or length<=0:
        raise ValueError('줄 직경은 0.05~20 mm, 길이는 0보다 커야 합니다.')
    if value['schema']=='dbf-assembly/2':
        attached=bool(objects.get('sensor') and objects.get('winch'))
        expected={'kind':'point_mass','local_point_m':[0,0,0]} if attached else None
        if cable.get('attachment')!=expected:raise ValueError('질점과 윈치의 자동 연결 정보가 올바르지 않습니다.')
        if attached:
            from scipy.spatial.transform import Rotation
            q=(objects.get('aircraft') or {}).get('quaternion',[0,0,0,1])
            # Read old full-length previews as well as the new stowed preview.
            expected_positions=[np.asarray(objects['winch']['position'])+Rotation.from_quat(q).apply([0,0,distance]) for distance in (min(.02,length*.05),length)]
            if not any(np.allclose(objects['sensor']['position'],position,rtol=0,atol=1e-7) for position in expected_positions):
                raise ValueError('질점 미리보기 위치가 윈치 위치·줄 길이와 일치하지 않습니다.')
        return value
    if cable.get('attachment') is not None:
        if not isinstance(cable['attachment'],dict):raise ValueError('줄 연결점 형식이 올바르지 않습니다.')
        if not objects.get('sensor') or not objects.get('winch'):raise ValueError('줄 연결에는 센서와 윈치가 필요합니다.')
        a=cable['attachment'];parts=objects['sensor']['parts'];i=a.get('part')
        if type(i) is not int or not 0<=i<len(parts):raise ValueError('연결한 센서 표면이 없습니다.')
        face=a.get('face');f=np.asarray(parts[i]['indices']).reshape(-1,3)
        if type(face) is not int or not 0<=face<len(f):raise ValueError('연결한 삼각형이 없습니다.')
        bary=np.asarray(a['barycentric'],float);local=np.asarray(a['local_point_m'],float)
        if bary.shape!=(3,) or not np.isfinite(bary).all() or bary.min() < -1e-6 or abs(bary.sum()-1)>1e-6:raise ValueError('표면 연결 좌표가 올바르지 않습니다.')
        p=np.asarray(parts[i]['positions'],float).reshape(-1,3)
        if local.shape!=(3,) or not np.allclose(bary@p[f[face]],local,rtol=0,atol=1e-6):raise ValueError('연결점이 저장된 센서 표면과 일치하지 않습니다.')
    return value


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(HERE),**kwargs)

    def end_headers(self):
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Cache-Control','no-store')
        super().end_headers()

    def respond(self,value,status=200):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf8')
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)

    def do_GET(self):
        path=urlparse(self.path).path
        if path=='/api/health':return self.respond(dict(status='ready',step_workers=self.server.pool._max_workers))
        try:
            if path=='/api/models':return self.respond(model_registry.list_models())
            if path=='/api/models/template':
                import yaml
                return self.respond(yaml.safe_load((HERE/'templates/model_physics_template.yaml').read_text('utf8')))
            if path=='/api/analysis/catalog':
                model=parse_qs(urlparse(self.path).query).get('model',[None])[0]
                if not model:raise ValueError('등록 모델을 선택하세요.')
                return self.respond(catalog(model))
            if path=='/api/analysis/jobs':return self.respond(self.server.jobs.list())
            if path.startswith('/api/analysis/jobs/'):
                return self.respond(self.server.jobs.status(path.rsplit('/',1)[-1]))
            if path.startswith('/analysis-files/'):
                parts=unquote(path).split('/');folder=self.server.jobs.folder(parts[2]);target=(folder/('/'.join(parts[3:]))).resolve()
                if not target.is_relative_to(folder) or not target.is_file() or target.suffix.lower() not in {'.html','.json','.csv','.npz','.npy','.log'}:
                    return self.respond(dict(error='결과 파일이 없습니다.'),404)
                data=target.read_bytes();self.send_response(200)
                mime={'.html':'text/html; charset=utf-8','.json':'application/json; charset=utf-8','.csv':'text/csv; charset=utf-8','.log':'text/plain; charset=utf-8'}
                self.send_header('Content-Type',mime.get(target.suffix,'application/octet-stream'))
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
        except (ValueError,FileNotFoundError) as exc:return self.respond(dict(error=str(exc)),400)
        if path.startswith('/api/'):
            return self.respond(dict(error='없는 요청입니다.'),404)
        # Serve only editor resources, dependencies, and saved project JSON.
        relative=unquote(path).lstrip('/') or 'index.html'
        target=(HERE/relative).resolve()
        if not target.is_relative_to(HERE) or target.suffix.lower() not in {'.html','.js','.css','.json','.map','.wasm','.png','.svg','.woff2'}:
            return self.respond(dict(error='열 수 없는 파일입니다.'),404)
        if path=='/':self.path='/index.html'
        return super().do_GET()

    def do_POST(self):
        try:
            origin=self.headers.get('Origin')
            if origin and origin not in {f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'}:
                return self.respond(dict(error='로컬 편집기에서 다시 시도해 주세요.'),403)
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=MAX_BYTES:raise ValueError('파일 크기는 64 MB 이하여야 합니다.')
            data=self.rfile.read(size);url=urlparse(self.path)
            if url.path.startswith('/api/aircraft/'):
                from aircraft_definition import extract_sections,mass_properties,preview,register_definition
                value=json.loads(data);project=validate_project(value['project'])
                if url.path=='/api/aircraft/sections':
                    return self.respond(self.server.pool.submit(extract_sections,project['objects']['aircraft'],value['request']).result(timeout=120))
                if url.path=='/api/aircraft/mass':
                    return self.respond(mass_properties(project['aircraft_definition']))
                if url.path=='/api/aircraft/preview':return self.respond(preview(project))
                if url.path=='/api/aircraft/register':return self.respond(register_definition(project),201)
                return self.respond(dict(error='없는 기체 정의 요청입니다.'),404)
            if url.path=='/api/models/match':
                value=json.loads(data);project=validate_project(value['project'])
                return self.respond(model_registry.match_aircraft(project,value.get('model_id')))
            if url.path=='/api/models/register':
                value=json.loads(data);project=validate_project(value['project'])
                return self.respond(model_registry.register(project,value['registration']),201)
            if url.path in ('/api/analysis/validate','/api/analysis/jobs'):
                value=json.loads(data);project=validate_project(value['project'])
                prepared=prepare(project,value['settings'])
                if url.path.endswith('/validate'):
                    return self.respond(dict(ready=True,warnings=prepared['warnings'],mapping=prepared['mapping'],schedule=prepared['schedule'],duration_s=prepared['duration_s']))
                return self.respond(self.server.jobs.start(project,prepared),202)
            if url.path.startswith('/api/analysis/cancel/'):
                return self.respond(self.server.jobs.cancel(url.path.rsplit('/',1)[-1]))
            if url.path=='/api/import-step':
                name=Path(parse_qs(url.query).get('name',['model.step'])[0]).name
                if Path(name).suffix.lower() not in ('.step','.stp'):raise ValueError('STEP 또는 STP 파일을 선택해 주세요.')
                directory=HERE/'data/uploads';directory.mkdir(parents=True,exist_ok=True)
                path=directory/f'{uuid.uuid4().hex}.step';path.write_bytes(data)
                result=self.server.pool.submit(convert_step,path).result(timeout=180)
                return self.respond(result)
            if url.path=='/api/projects':
                value=validate_project(json.loads(data));directory=HERE/'data/projects';directory.mkdir(parents=True,exist_ok=True)
                name=uuid.uuid4().hex+'.json';(directory/name).write_text(json.dumps(value,ensure_ascii=False,allow_nan=False),encoding='utf8')
                return self.respond(dict(url='/data/projects/'+name,id=name[:-5]))
            return self.respond(dict(error='없는 요청입니다.'),404)
        except Exception as exc:
            self.respond(dict(error=f'처리하지 못했습니다. 파일과 입력값을 확인해 주세요. {exc}'),400)


def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8767);p.add_argument('--workers',type=int,default=2)
    args=p.parse_args()
    if not 1<=args.workers<=11:raise ValueError('STEP workers must be between 1 and 11')
    worker_init()
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn'),initializer=worker_init) as pool:
        server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
        server.pool=pool;server.jobs=JobManager()
        print(f'DBF assembly editor: http://127.0.0.1:{args.port}',flush=True)
        try:server.serve_forever()
        finally:server.jobs.close();server.server_close()

if __name__=='__main__':main()
