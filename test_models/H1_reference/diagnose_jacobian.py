"""Locate derivative errors without altering forces or trajectory."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'src'))
from dbf_stability import AeroDatabase
from dbf_stability.model import CoupledModel

if __name__=='__main__':
    r=ROOT/'test_models/H1_reference/runs/funnel_mission_01'
    c=json.loads((r/'mission/inputs.json').read_text(encoding='utf8'))
    with np.load(r/'mission/accepted_checkpoint.npz') as d:
        t=float(d['time']);y=d['state'].copy();meta=json.loads(str(d['metadata']))
    m=CoupledModel(c,AeroDatabase(json.loads((r/'source.json').read_text())['aero_path']),meta['trim'])
    m.active_override=meta['active_nodes'];j=m.jacobian()(t,y).toarray();f=m.rhs(t,y)
    central=np.empty_like(j);forward=np.empty_like(j);backward=np.empty_like(j)
    for i in range(len(y)):
        h=1e-8;yp=y.copy();ym=y.copy();yp[i]+=h;ym[i]-=h
        fp,fm=m.rhs(t,yp),m.rhs(t,ym)
        forward[:,i]=(fp-f)/(yp[i]-y[i]);backward[:,i]=(f-fm)/(y[i]-ym[i])
        central[:,i]=(fp-fm)/(yp[i]-ym[i])
    errors=np.linalg.norm(j-central,axis=0);ii=np.argsort(errors)[-12:]
    data=[dict(column=int(i),mismatch=float(errors[i]),old_norm=float(np.linalg.norm(j[:,i])),
        central_norm=float(np.linalg.norm(central[:,i])),
        asymmetry=float(np.linalg.norm(forward[:,i]-backward[:,i]))) for i in ii]
    print(json.dumps(data,indent=2),flush=True)
    out=r.parent/'stagnation_diagnosis_01'
    np.savez_compressed(out/'jacobians.npz',old=j,central=central,forward=forward,backward=backward)
    (out/'jacobian_errors.json').write_text(json.dumps(data,indent=2))
    m.close()
