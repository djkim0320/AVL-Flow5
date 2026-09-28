"""Register the new N3 CAD and run the installed solvers through DBF Studio."""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, json, struct, uuid, sys
from urllib.request import Request, urlopen
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
URL='http://127.0.0.1:8767'

def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')

def api(path,value=None):
    req=Request(URL+path,data=None if value is None else json.dumps(value,ensure_ascii=False).encode('utf8'),headers={'Content-Type':'application/json'})
    try:
        with urlopen(req,timeout=120) as r:return json.load(r)
    except Exception as e:
        if hasattr(e,'read'):print(e.read().decode('utf8'),flush=True)
        raise

def glb_parts():
    # Read the exported file, rather than creating a second tessellation.
    raw=(HERE/'N3_aircraft_open.glb').read_bytes();n=struct.unpack_from('<I',raw,12)[0]
    doc=json.loads(raw[20:20+n]);blob=raw[28+n:]
    def array(i):
        a=doc['accessors'][i];b=doc['bufferViews'][a['bufferView']]
        width=3 if a['type']=='VEC3' else 1
        return np.frombuffer(blob,dtype='<f4' if a['componentType']==5126 else '<u4',
            offset=b.get('byteOffset',0)+a.get('byteOffset',0),count=a['count']*width).reshape(-1,width)
    parts=[]
    for mesh in doc['meshes']:
        p=mesh['primitives'][0];v=array(p['attributes']['POSITION']).astype(float)
        v=v[:,[0,2,1]]*[1,1,-1]
        parts.append(dict(name=mesh['name'],positions=v.ravel().tolist(),indices=array(p['indices']).ravel().tolist(),
            color=doc['materials'][p['material']]['pbrMetallicRoughness']['baseColorFactor']))
    return parts

def register(mesh):
    if (HERE/'ui_registration.json').exists():raise FileExistsError('Already registered; use start/update actions.')
    dims=json.loads((HERE/'dimensions.json').read_text('utf8'));feed=np.array(dims['feed_local_frd_m'])
    def obj(source,position,parts=None):
        d=dict(source=source,id=str(uuid.uuid4()),position=list(position),quaternion=[0,0,0,1])
        if parts is not None:d['parts']=parts
        return d
    aircraft=obj(dict(name='N3_aircraft_open.glb',units='m',axes='yup',origin='file_origin',source_origin_shift_m=[0,0,0],
        sha256=hashlib.sha256((HERE/'N3_aircraft_open.glb').read_bytes()).hexdigest()),[0,0,0],glb_parts())
    # Small octahedron denotes the cable's exit force point; the CAD contains
    # the physical internal drum separately. This marker has no assigned mass.
    marker=dict(name='N3_winch_exit_marker',positions=(np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]])*.009).ravel().tolist(),
        indices=[0,2,4,2,1,4,1,3,4,3,0,4,2,0,5,1,2,5,3,1,5,0,3,5],color=[.98,.48,.1,1])
    winch=obj(dict(name='N3 내부 윈치 · 줄 출구',origin='design_exit',units='m',axes='FRD'),feed,[marker])
    sensor=obj(dict(name='센서 역할 질점',origin='point_mass'),feed+[0,0,3.])
    sensor.update(kind='point_mass',mass_kg=.12)
    project=dict(schema='dbf-assembly/2',units='m',axes='FRD',saved_at=datetime.now(timezone.utc).isoformat(),
        objects=dict(aircraft=aircraft,winch=winch,sensor=sensor),cable=dict(length_m=3.,diameter_m=.001,
        attachment=dict(kind='point_mass',local_point_m=[0,0,0])),view=dict(selected='aircraft',aircraft_locked=True))
    directory=HERE/'aero'/mesh
    registration=dict(name='N3 DBF 2026–27 · 신규 1.80 m 화물기',case_text=(HERE/f'N3_{mesh}.yaml').read_text('utf8'),
        files=[dict(name=p.name,text=p.read_text('ascii')) for p in directory.iterdir() if p.suffix in ('.avl','.dat','.mass')],
        aircraft_cg_local_m=[0,0,0],sensor_cg_local_m=[0,0,0],door_parts=dims['door_parts'],door_reference_deg=270,source_confirmed=True)
    result=api('/api/models/register',dict(project=project,registration=registration));save(HERE/'ui_registration.json',result)
    settings=api('/api/analysis/catalog?model='+result['id'])['defaults']
    settings.update(task='flight',phase='deployed',start='equilibrium',rebuild=True,aero_job='new',workers=12,controller=False)
    project['analysis']=settings;save(HERE/'N3_ready.dbf.json',project)
    save(HERE/'ui_project_location.json',api('/api/projects',project));print(result,flush=True)

def start(backend,task,grid=None):
    project=json.loads((HERE/'N3_ready.dbf.json').read_text('utf8'));s=project['analysis'].copy()
    s.update(backend=backend,task=task,phase='deployed',workers=12,controller=False,pitch_delta=1.,duration=2.)
    if task=='recovery':
        previous=json.loads((HERE/f'ui_{backend}_flight_job.json').read_text('utf8'))
        s.update(aero_job=previous['id'],rebuild=False)
    else:s.update(aero_job='new',rebuild=True)
    if grid is not None:s['aero_grid']=grid
    validation=api('/api/analysis/validate',dict(project=project,settings=s));save(HERE/f'ui_{backend}_{task}_validation.json',validation)
    job=api('/api/analysis/jobs',dict(project=project,settings=s));save(HERE/f'ui_{backend}_{task}_job.json',job)
    print(json.dumps(job,ensure_ascii=False),flush=True)

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf8')
    p=argparse.ArgumentParser();p.add_argument('action',choices=['register','start','update']);p.add_argument('--mesh',default='extra')
    p.add_argument('--backend',default='avl');p.add_argument('--task',default='flight');a=p.parse_args()
    if a.action=='register':register(a.mesh)
    elif a.action=='start':start(a.backend,a.task)
    else:
        project=json.loads((HERE/'N3_ready.dbf.json').read_text('utf8'));job=json.loads((HERE/f'ui_{a.backend}_flight_job.json').read_text('utf8'))
        project['analysis'].update(backend=a.backend,aero_job=job['id'],rebuild=False)
        request=json.loads((ROOT/'ui/data/analysis'/job['id']/'request.json').read_text('utf8'))
        project['analysis']['aero_grid']={k:request['config']['aero'][k] for k in ('alpha_deg','beta_deg','elevator_deg')}
        save(HERE/'N3_ready.dbf.json',project);save(HERE/'ui_project_location.json',api('/api/projects',project))
