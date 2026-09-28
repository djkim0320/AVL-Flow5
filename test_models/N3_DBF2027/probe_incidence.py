from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import json,sys
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'test_models/H1_reference')]
from dbf_stability.avl import run_avl,parse_output
from compute_resources import apply_affinity

def run(inc):
    out=HERE/'runs/incidence_design_01'/f'output_{inc}'
    d=run_avl(ROOT/'vendor/avl/avl.exe',out.parent/'inputs'/f'inc_{inc}.avl',out,0,0,0,2)
    f=parse_output((out/'forces.txt').read_text());s=parse_output((out/'stability.txt').read_text())
    return dict(incidence=inc,CL=f['CLtot'],CDi=f['CDind'],Cm=d['Cmtot'],Cma=s['Cma'],CLa=s['CLa'],Xnp=s['Xnp'])

if __name__=='__main__':
    apply_affinity()
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,[0.,.5,1.,1.5]))
    (HERE/'runs/incidence_design_01/results.json').write_text(json.dumps(rows,indent=2))
    print(json.dumps(rows,indent=2))
