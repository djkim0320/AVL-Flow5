"""Independent real AVL calls compared to the flow5 lifting-surface database."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
import pandas as pd
from dbf_stability import load_case,AeroDatabase
from dbf_stability.avl import run_avl,COEFF

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'test_models/H1_reference/runs/flow5_connection_01'


def job(item):
    i,point=item;c=load_case(ROOT/'examples/h1_normal_r3_flow5.yaml')
    avl=run_avl(ROOT/'vendor/avl/avl.exe',(ROOT/c['aero']['geometry']).with_stem('normal_r3_no_body'),OUT/'avl_comparison'/f'case_{i:03d}',*point,elevator_index=2)
    flow=AeroDatabase(OUT/'aerodynamics/aero_database.npz')
    f=flow.evaluate(np.deg2rad(point[0]),np.deg2rad(point[1]),point[2],np.zeros(3),20)
    a=np.array([avl[k+'tot'] for k in COEFF])
    return dict(alpha_deg=point[0],beta_deg=point[1],elevator_deg=point[2],**{f'flow5_{k}':float(v) for k,v in zip(COEFF,f)},**{f'avl_{k}':float(v) for k,v in zip(COEFF,a)},**{f'difference_{k}':float(v) for k,v in zip(COEFF,f-a)})


if __name__=='__main__':
    points=[(a,b,0) for a in [-2,0,2] for b in [-4,0,4]]+[(0,0,-8),(0,0,8)]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(job,enumerate(points)))
    p=OUT/'avl_comparison';pd.DataFrame(rows).to_csv(p/'comparison.csv',index=False)
    report=dict(scope='Both solvers use lifting surfaces without fuselage. Native meshes and geometric versus linearized controls differ. Differences are not accuracy or convergence proof.',
                rows=rows,largest_absolute_difference={k:max(abs(r[f'difference_{k}']) for r in rows) for k in COEFF})
    (p/'comparison.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report['largest_absolute_difference']))
