"""Compare enlarged openings from t=0 using the recorded actual AVL database."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import hashlib
import json
from dbf_stability import load_case, AeroDatabase, solve_trim, simulate
from dbf_stability.plots import history_figure
from geometry_paths import cad_directory, mesh_directory

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def run_job(height,out):
    c=load_case(ROOT/f'examples/h1_opening_{height:g}.yaml')
    aero=AeroDatabase(HERE/'aerodynamics/aero_database.npz')
    trim=solve_trim(c,aero,mode='stowed')
    directory=Path(out)/f'opening_{height:g}'/'mission'
    directory.mkdir(parents=True)
    r=simulate(c,aero,trim,phase='mission',duration=7.,output=directory)
    history_figure(r).write_html(directory/'history.html',include_plotlyjs=True)
    return str(height),{k:r.summary[k] for k in ('status','duration_s','captured','max_tension_N','max_contact_N','max_pitch_change_deg','runtime_s')}


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--heights',nargs='+',type=float,default=[70,80]);p.add_argument('--workers',type=int,default=2);args=p.parse_args()
    inputs=[];geometry={}
    for h in args.heights:
        c=load_case(ROOT/f'examples/h1_opening_{h:g}.yaml')
        audit=json.loads((mesh_directory(c).parent/'door_swing.json').read_text(encoding='utf8'))
        hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in cad_directory(c).glob('*.step')}
        if audit['intersecting_angles'] or audit['error_angles'] or hashes!=audit['cad_sha256']:
            raise ValueError(f'{h}: missing, failed or stale door CAD sweep')
        if audit['door_hinge_offset_frd_m']!=c['bay']['door_hinge_offset_m'] or audit['door_closed_offset_frd_m']!=c['bay']['door_closed_offset_m']:
            raise ValueError('Door sweep transform differs from simulation')
        if not all(0<=angle<=180 for _,angle in c['winch']['door_schedule']):raise ValueError('Unaudited door range')
        inputs.append({k:c[k] for k in ('aircraft','sensor','cable','winch','flight','simulation','aero')})
        geometry[str(h)]={'cad':hashes,'meshes':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in mesh_directory(c).glob('*')}}
    if not all(c==inputs[0] for c in inputs):raise ValueError('Only the opening geometry may differ')
    args.output.mkdir(parents=True,exist_ok=False)
    sources=list((ROOT/'src/dbf_stability').glob('*.py'))+[Path(__file__).resolve()]
    (args.output/'source_versions.json').write_text(json.dumps({str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},indent=2),encoding='utf8')
    (args.output/'study.json').write_text(json.dumps({'heights_mm':args.heights,'width_mm':56,'door_deg':140,
        'starts_at_s':0.,'requested_end_s':7.,'identical_aircraft_sensor_winch_inputs':True,
        'aero':'Actual AVL 3.52 H1 lifting-surface table. Opening/door/wake aero changes unmodelled; mass/inertia held fixed.',
        'aero_sha256':hashlib.sha256((HERE/'aerodynamics/aero_database.npz').read_bytes()).hexdigest(),'geometry':geometry},indent=2),encoding='utf8')
    results={}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_job,h,str(args.output)) for h in args.heights]):
            h,result=future.result();results[h]=result;print(h,json.dumps(result),flush=True)
            (args.output/'checks.json').write_text(json.dumps(results,indent=2),encoding='utf8')


if __name__=='__main__':main()
