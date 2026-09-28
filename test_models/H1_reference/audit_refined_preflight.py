"""Geometric corridor check, not a prescribed motion or a dynamics result."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import argparse
import numpy as np
from dbf_stability import load_case
from dbf_stability.collision import MeshContacts

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def initialize(case_path):
    global C,W
    C=load_case(case_path);W=MeshContacts(C)


def sample(x):
    center=np.array(C['bay']['stowed_center_m']);center[0]=x
    nose=center+C['sensor']['tow_point_m']
    chain=np.array([nose,C['aircraft']['tow_point_m']])
    hits=W.contacts(center,np.eye(3),chain,140,distance=.02)
    nearest=min(hits,key=lambda h:h.get('geometry_gap',h['gap'])) if hits else None
    return dict(center_frd_m=center.tolist(),gap_m=nearest.get('geometry_gap',nearest['gap']) if nearest else .02,
                nearest_pair=[nearest['moving'],nearest['fixed']] if nearest else None,
                penetrating_pairs=[[h['moving'],h['fixed']] for h in hits if h.get('geometry_gap',h['gap'])<0])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,default=ROOT/'examples/h1_refined_r2.yaml');args=p.parse_args()
    c=load_case(args.case)
    xs=np.linspace(c['bay']['stowed_center_m'][0],c['bay']['exit_x_m']-.2,41)
    with ProcessPoolExecutor(max_workers=4,initializer=initialize,initargs=(str(args.case.resolve()),)) as pool: rows=list(pool.map(sample,xs))
    report=dict(scope='Aligned sensor and straight wire corridor at 41 static poses; not predicted motion, no dynamic capture claim.',
                minimum_gap_m=min(r['gap_m'] for r in rows),intersecting_poses=sum(bool(r['penetrating_pairs']) for r in rows),rows=rows)
    out=(ROOT/c['collision']['mesh_directory']).parent/'preflight.json'
    out.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2),flush=True)
    if report['intersecting_poses']:raise SystemExit(1)
