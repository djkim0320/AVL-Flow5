"""Audit the real UI import/registration/AVL runs delivered with N2."""
from pathlib import Path
import hashlib,json,shutil
import numpy as np
import pandas as pd
from dbf_stability.math3d import rotation

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
JOBS={'aircraft':'3455fc99097c43948feb97b5676759c0','deployed_8':'80854e86339749949fb3ddcb885e3252',
      'deployed_16':'82ac8697ead4428fa904cfabde06ed06','deployed_32':'9998b390b6ee460b9f6b8ce2d00caa05',
      'response':'b18828e3a958454c96d3274a943eb3cd'}
MODEL='8afe804476b5470bb480b996d87756e0'
def read(p):return json.loads(p.read_text(encoding='utf8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    dims=read(OUT/'dimensions.json');records={}
    source_sha=sha(OUT/'N2_aircraft_open.glb')
    for name,id in JOBS.items():
        p=ROOT/'ui/data/analysis'/id;j=read(p/'job.json');req=read(p/'request.json');project=read(p/'project.json')
        assert j['state']=='completed',(name,j['state'])
        assert req['settings']['model_id']==MODEL
        assert project['objects']['aircraft']['source']['sha256']==source_sha
        assert project['objects']['sensor']['kind']=='point_mass'
        assert 'parts' not in project['objects']['sensor']
        assert req['settings']['workers']==12
        a=project['objects']['aircraft'];verts=np.concatenate([np.array(p['positions']).reshape(-1,3) for p in a['parts']])
        assert abs(np.ptp(verts[:,1])-dims['wingspan_m'])<1e-6
        assert np.ptp(verts[:,1])<=dims['wingspan_limit_m']
        r={'id':id,'result':j['result'],'elapsed_s':j['finished']-j['created']}
        if (p/'modes.csv').exists():
            modes=pd.read_csv(p/'modes.csv');r['max_real_eigenvalue_1_s']=float(modes.real_1_s.max())
        if req['phase']=='deployed':
            y=np.load(p/'trim_state.npy');delta=rotation(y[6:10]).T@(y[13:16]-y[:3])-req['config']['aircraft']['tow_point_m']
            r['equilibrium_exit_to_point_m']=float(np.linalg.norm(delta))
            assert r['equilibrium_exit_to_point_m']>=dims['minimum_exit_tip_distance_m']
        records[name]=r
    p=ROOT/'ui/data/analysis'/JOBS['deployed_8'];meta=read(p/'aero_metadata.json')
    assert meta['solver']=='AVL 3.52'
    assert meta['geometry_sha256']==sha(OUT/'aero/N2.avl')
    raw=Path(meta['raw_directory']);outputs=list(raw.glob('case_*/derivatives.txt'))
    assert len(outputs)==45
    for f in outputs:assert 'Standard axis orientation' in f.read_text()
    with np.load(p/'aero_database.npz') as db:
        assert np.isfinite(db['coeff']).all();assert abs(db['refs'][2]-1.7)<1e-7
    response=ROOT/'ui/data/analysis'/JOBS['response']
    df=pd.read_csv(response/'simulation/timeseries.csv')
    assert df.time_s.iloc[-1]==2.0
    assert np.isfinite(df[['tension_N','pitch_deg','roll_deg','yaw_deg','altitude_m']]).all().all()
    expected=dims['aircraft_mass_kg']+dims['payload_kg']+.0015*dims['line_m']
    assert np.max(np.abs(df.total_mass_kg-expected))<1e-12
    convergence={}
    for n,a,b in [('8_to_16','deployed_8','deployed_16'),('16_to_32','deployed_16','deployed_32')]:
        convergence[n]=abs(records[b]['max_real_eigenvalue_1_s']/records[a]['max_real_eigenvalue_1_s']-1)*100
    audit={'model_id':MODEL,'solver':meta,'jobs':records,'leading_mode_change_percent':convergence,
      'mass_total_kg':expected,'mass_range_kg':float(np.ptp(df.total_mass_kg)),
      'actual_response_samples':len(df),'GLB_sha256':source_sha,'CAD_AVL_span_agree':True,
      'UI_observed':['GLB file chooser import with m/Y-up','Undo then actual mouse drag of recent file card into canvas',
        'Winch gizmo drag changed X/Z; exact final feed coordinates entered','Point0.1kg / cable2.8m',
        'New model registration through file inputs','All five analysis jobs submitted through UI',
        'Playback advanced through actual response samples'],
      'untested':['Operating-system file-manager external file drag gesture','Full mission','Sensor aerodynamics/shape/contact',
        'Full aero mesh/table convergence','Measured mass, inertia and cable properties','Competition flight/structure/electrical acceptance']}
    (OUT/'verification.json').write_text(json.dumps(audit,indent=2,ensure_ascii=False),encoding='utf8')
    shutil.copy2(response/'project.json',OUT/'N2_ready.dbf.json')
    print(json.dumps({'jobs':{k:v['result'].get('unstable',v['result']['status']) for k,v in records.items()},
       'leading_mode_change_percent':convergence,'mass_kg':expected,'response_samples':len(df)},indent=2))

if __name__=='__main__':main()
