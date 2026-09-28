"""Fresh AVL database, symmetric trims, independent short integrations."""
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse, json, traceback, time, hashlib
import numpy as np
from dbf_stability import load_case, build_aero_database, AeroDatabase, solve_trim, simulate
from dbf_stability.plots import history_figure

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

def save_json(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')

def job(args):
    mode,db,out,duration,case_path=args
    case=load_case(case_path);aero=AeroDatabase(db)
    folder=Path(out)/mode;folder.mkdir(parents=True,exist_ok=True)
    try:
        trim_mode='stowed' if mode=='mission' else mode
        trim=solve_trim(case,aero,mode=trim_mode)
        save_json(folder/'trim.json',{k:v for k,v in trim.items() if k!='state'})
        np.save(folder/'trim_state.npy',trim['state'])
        # A small actual perturbation for the deployed cable/sensor response.
        if mode=='deployed':case['flight']['initial_sensor_velocity_delta_m_s']=[0,.02,0]
        result=simulate(case,aero,trim=trim,phase=mode,duration=duration,output=folder)
        history_figure(result).update_layout(title=f'H1-A {mode}: temporary assumed mass / sensor').write_html(folder/'history.html',include_plotlyjs=True)
        answer={k:result.summary[k] for k in ['status','duration_s','max_tension_N','max_contact_N','max_penetration_m','max_pitch_change_deg','captured','violations','runtime_s','numerically_converged']}
        answer['trim']={k:v for k,v in trim.items() if k!='state'}
        return mode,answer
    except Exception as exc:
        (folder/'failure.log').write_text(traceback.format_exc(),encoding='utf8')
        answer={'status':'failed','message':str(exc)};save_json(folder/'summary.json',answer)
        return mode,answer

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workers',type=int,default=6)
    parser.add_argument('--reuse-aero',action='store_true');parser.add_argument('--mission-duration',type=float,default=.85)
    parser.add_argument('--output',default=None)
    parser.add_argument('--case',type=Path,default=ROOT/'examples/h1_opening_70.yaml',help='Case YAML; default is the 56 x 70 mm opening with a 140-degree door')
    parser.add_argument('--modes',nargs='+',default=['aircraft_only','stowed','deployed','mission'],choices=['aircraft_only','stowed','deployed','mission'])
    a=parser.parse_args();out=Path(a.output) if a.output else HERE/'runs'/time.strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    db=HERE/'aerodynamics/aero_database.npz';db.parent.mkdir(exist_ok=True)
    case=load_case(a.case)
    input_hashes={str(f.relative_to(HERE)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((HERE/'geometry').glob('*')) if f.is_file()}
    hashfile=db.parent/'input_hashes.json'
    if not a.reuse_aero:
        print('Building 105 fresh H1 AVL cases',flush=True)
        build_aero_database(case,output=db,workers=a.workers)
        save_json(hashfile,input_hashes)
    else:
        ad=AeroDatabase(db)
        assert ad.metadata['geometry_sha256']==hashlib.sha256((ROOT/case['aero']['geometry']).read_bytes()).hexdigest()
        assert json.loads(hashfile.read_text(encoding='utf8'))==input_hashes,'Geometry/airfoil/mass files changed; regenerate actual AVL data.'
    ad=AeroDatabase(db);save_json(out/'aero_metadata.json',ad.metadata)
    print('AVL database ready: '+str(db),flush=True)
    results={}
    jobs=[(mode,str(db),str(out),a.mission_duration if mode=='mission' else .25 if mode=='stowed' else .5,str(a.case.resolve())) for mode in a.modes]
    with ProcessPoolExecutor(max_workers=min(a.workers,4)) as pool:
        futures=[pool.submit(job,j) for j in jobs]
        for future in as_completed(futures):
            name,result=future.result();results[name]=result
            save_json(out/'checks.json',results);print(name,json.dumps(result),flush=True)
    save_json(HERE/'latest_run.json',{'path':str(out),'checks':results})
    print('Results: '+str(out),flush=True)

if __name__=='__main__':main()
