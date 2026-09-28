"""Full-from-zero mechanical angle comparison using the actual H1 AVL table."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
import argparse
import json
import hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import yaml
from dbf_stability import load_case, changed, AeroDatabase, solve_trim, simulate
from dbf_stability.plots import history_figure

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def run_job(angle,out):
    case=changed(load_case(ROOT/'examples/h1_reference.yaml'), **{
        'bay.door_hinge_offset_m':[-.003,0.,.003],
        'bay.door_closed_offset_m':[-.003,0.,0.],
        'winch.door_schedule':[[0.,0.],[.4,float(angle)],[6.5,float(angle)],[7.,float(angle)]]})
    case['name']=f'H1_offset_hinge_door_{angle}_assumption_case'
    case['provenance']['bay']['source']+=' Door hinge moved aft/down 3 mm, closed leaf moved aft 3 mm onto the outer frame face; ideal offset hinge, no bracket or actuator packaging validation.'
    case['provenance']['winch']['source']+=f' Door opens to {angle} degrees in 0.4 s and stays open through 7 s for comparison.'
    directory=Path(out)/f'angle_{angle}'/'mission'
    directory.mkdir(parents=True)
    case_path=ROOT/f'examples/h1_door_{angle}.yaml'
    case_path.write_text(yaml.safe_dump({k:v for k,v in case.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
    case=load_case(case_path)
    aero=AeroDatabase(HERE/'aerodynamics/aero_database.npz')
    trim=solve_trim(case,aero,mode='stowed')
    result=simulate(case,aero,trim,phase='mission',duration=7.,output=directory)
    history_figure(result).write_html(directory/'history.html',include_plotlyjs=True)
    keys=('status','duration_s','captured','max_tension_N','max_contact_N','max_pitch_change_deg',
          'minimum_checked_mesh_gap_m','clearance_checks','runtime_s')
    return str(angle),{k:result.summary[k] for k in keys}


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--angles',nargs='+',type=int,default=[100,140,160])
    p.add_argument('--workers',type=int,default=3);args=p.parse_args()
    audit=json.loads((HERE/'diagnostics/door_angles/offset_swing.json').read_text(encoding='utf8'))
    if audit['intersecting_angles'] or audit['error_angles'] or audit['door_hinge_offset_frd_m']!=[-.003,0.,.003] or audit.get('door_closed_offset_frd_m')!=[-.003,0.,0.]:
        raise ValueError('The selected hinge requires a successful door/fixed-aircraft CAD sweep before simulation')
    if any(angle<0 or angle>180 for angle in args.angles):
        raise ValueError('Requested angle is outside the audited 0-180 degree range')
    current_cad={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in (HERE/'cad').glob('*.step')}
    if current_cad!=audit['cad_sha256']:
        raise ValueError('CAD changed after the door sweep; rerun audit_door_swing.py')
    args.output.mkdir(parents=True,exist_ok=False)
    sources=list((ROOT/'src/dbf_stability').glob('*.py'))+[Path(__file__).resolve()]
    (args.output/'source_versions.json').write_text(json.dumps({str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},indent=2),encoding='utf8')
    (args.output/'study.json').write_text(json.dumps({'angles_deg':args.angles,'hinge_offset_frd_m':[-.003,0,.003],'closed_leaf_offset_frd_m':[-.003,0,0],
        'starts_at_s':0.,'requested_end_s':7.,'same_winch_sensor_aircraft_inputs':True,
        'aero':'Actual AVL 3.52 H1 lifting-surface database reused; changed door/wake aerodynamics unmodelled.',
        'aero_sha256':hashlib.sha256((HERE/'aerodynamics/aero_database.npz').read_bytes()).hexdigest()},indent=2),encoding='utf8')
    results={}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_job,angle,str(args.output)) for angle in args.angles]):
            label,result=future.result();results[label]=result
            print(label,json.dumps(result),flush=True)
            (args.output/'checks.json').write_text(json.dumps(results,indent=2),encoding='utf8')


if __name__=='__main__':main()
