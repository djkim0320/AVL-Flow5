"""Inspect force continuity about the finalized stopped state."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import sys,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from dbf_stability import AeroDatabase
from dbf_stability.model import CoupledModel

if __name__=='__main__':
    run=ROOT/'test_models/H1_reference/runs/funnel_mission_01'
    c=json.loads((run/'mission/inputs.json').read_text(encoding='utf8'))
    with np.load(run/'mission/accepted_checkpoint.npz',allow_pickle=False) as d:
        y=d['state'].copy();t=float(d['time']);meta=json.loads(str(d['metadata']))
    m=CoupledModel(c,AeroDatabase(json.loads((run/'source.json').read_text())['aero_path']),meta['trim'])
    m.active_override=meta['active_nodes'];f,diag=m.rhs(t,y,True)
    def hits(z):
        return [{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in h.items()}
                for h in m.contact_geometry(t,z) if h['gap']<c['collision']['skin_m']]
    rows=[];direction=f.copy()
    if '--column' in sys.argv:
        index=int(sys.argv[sys.argv.index('--column')+1]);direction=np.eye(len(y))[index]
    for epsilon in [0.,1e-10,-1e-10,1e-9,-1e-9,1e-8,-1e-8,1e-7,-1e-7,1e-6,-1e-6]:
        z=y+epsilon*direction;g=m.rhs(t+epsilon,z)
        rows.append(dict(dt=epsilon,max_derivative_change=float(abs(g-f).max()),
            index=int(abs(g-f).argmax()),norm=float(np.linalg.norm(g-f)),hits=hits(z)))
    out=run.parent/'stagnation_diagnosis_01';out.mkdir(exist_ok=True)
    data=dict(time=t,diagnostics=diag,rows=rows)
    (out/('continuity_column.json' if '--column' in sys.argv else 'continuity_trajectory.json')).write_text(json.dumps(data,indent=2),encoding='utf8')
    print(json.dumps(data),flush=True);m.close()
