"""Independent process runs for the corrected H1 collision model."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
import argparse
import json
import hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
from dbf_stability import load_case,AeroDatabase,solve_trim,simulate,changed
from dbf_stability.plots import history_figure

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def run_job(args):
    label,changes,duration,out=args
    cfg=changed(load_case(ROOT/'examples/h1_reference.yaml'),**changes)
    aero=AeroDatabase(HERE/'aerodynamics/aero_database.npz')
    trim=solve_trim(cfg,aero,mode='stowed')
    directory=Path(out)/label/'mission'
    result=simulate(cfg,aero,trim,phase='mission',duration=duration,output=directory)
    history_figure(result).write_html(directory/'history.html',include_plotlyjs=True)
    return label,{k:result.summary[k] for k in ('status','duration_s','captured','max_tension_N','max_contact_N',
        'max_penetration_m','max_pitch_change_deg','minimum_checked_mesh_gap_m','clearance_checks','runtime_s')}


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=7.);p.add_argument('--workers',type=int,default=2)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    sources=[ROOT/'src/dbf_stability'/name for name in ('model.py','collision.py','contact_integration.py','analysis.py','config.py','math3d.py')]
    (args.output/'source_versions.json').write_text(json.dumps({str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},indent=2),encoding='utf8')
    cases=[('baseline',{}),('time_refined',{'collision.maximum_surface_travel_m':.0001,
        'collision.maximum_free_surface_travel_m':.001,'simulation.max_step_s':.0015})]
    jobs=[(label,changes,args.duration,str(args.output)) for label,changes in cases]
    results={}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for future in as_completed([pool.submit(run_job,job) for job in jobs]):
            label,result=future.result();results[label]=result
            print(label,json.dumps(result),flush=True)
            (args.output/'checks.json').write_text(json.dumps(results,indent=2),encoding='utf8')


if __name__=='__main__':main()
