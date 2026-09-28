"""Parallel actual-flow5 trim comparison for the unchanged lifting surfaces."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse,json
from dbf_stability import load_case,AeroDatabase,solve_trim
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def calculate(task):
    name,aero_path,phases=task
    c=load_case(ROOT/'examples'/name)
    a=AeroDatabase(aero_path)
    answer={}
    for phase in phases:
        try:
            trim=solve_trim(c,a,mode=phase)
            answer[phase]={'status':'converged',**{k:v for k,v in trim.items() if k!='state'}}
        except RuntimeError as exc:
            answer[phase]={'status':'not_converged','error':str(exc),
                'interpretation':'Equilibrium solver did not converge; not proof that a physical equilibrium is impossible.'}
    return name,answer


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--aero',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--include-deployed',action='store_true')
    args=p.parse_args()
    if args.out.exists():raise FileExistsError(args.out)
    phases=('stowed','deployed') if args.include_deployed else ('stowed',)
    tasks=[(name,str(args.aero.resolve()),phases) for name in ['h1_round_x_capture_stop_v3_shielded.yaml','h1_underbody_270_v2.yaml']]
    with ProcessPoolExecutor(max_workers=2) as pool:
        result=dict(pool.map(calculate,tasks))
    result['scope']='Same actual flow5 wing/tail database. Door hardware mass/CG/inertia and mechanical geometry changed. Door-local aerodynamics not recalculated; this is not a drag benefit estimate.'
    result['aero_path']=str(args.aero.resolve())
    path=args.out;path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2),encoding='utf8');print(json.dumps(result),flush=True)
