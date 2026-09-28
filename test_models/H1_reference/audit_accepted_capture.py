"""Inspect immutable accepted capture states while the dynamics continue.

This creates a geometry-only snapshot, not a completed simulation or load report.
The live checkpoint file is never opened; no active solver files are changed.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
import numpy as np
import pandas as pd
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel


def main():
    p=argparse.ArgumentParser();p.add_argument('parent',type=Path);args=p.parse_args();parent=args.parent.resolve()
    here=Path(__file__).resolve().parent
    rows=[];sources=[]
    for part in sorted((parent/'mission/accepted_history').glob('part_*.npz')):
        with np.load(part,allow_pickle=False) as data:
            rows.extend((float(t),y.copy(),bool(cap)) for t,y,cap in zip(data['time'],data['states'],data['captured']))
            meta=json.loads(str(data['metadata']))
        sources.append(dict(file=str(part),sha256=hashlib.sha256(part.read_bytes()).hexdigest()))
    first=next((i for i,r in enumerate(rows) if r[2]),None)
    if first is None:raise ValueError('No accepted captured state yet')
    chosen=sorted({0,max(0,first-1),first,min(first+1,len(rows)-1),len(rows)-1})
    out=parent/'capture_snapshot';out.mkdir(exist_ok=False)
    c=load_case(parent/'case.yaml');a=AeroDatabase(here/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    model=CoupledModel(c,a,meta['trim']);model.capture_time=meta['capture_time_s']
    table=[]
    for i in chosen:
        t,y,captured=rows[i];model.captured=captured
        table.append(dict(time_s=t,door_deg=model.door_state(t)[0],active_nodes=model.active_count(model.length(t)[0]),captured=captured))
    np.savez_compressed(out/'states.npz',time=[rows[i][0] for i in chosen],states=[rows[i][1] for i in chosen])
    pd.DataFrame(table).to_csv(out/'timeseries.csv',index=False)
    (out/'inputs.json').write_text(json.dumps(c,ensure_ascii=False,indent=2),encoding='utf8')
    (out/'events.json').write_text(json.dumps([dict(event='capture',time_s=meta['capture_time_s'])]),encoding='utf8')
    (out/'summary.json').write_text(json.dumps(dict(status='geometry_snapshot',captured=True,
        scope='Selected unmodified accepted states around capture. Live run is unfinished; no load or completion claim.',
        source=str(parent/'mission'),source_chunks=sources),indent=2),encoding='utf8')
    subprocess.run([sys.executable,str(here/'audit_trajectory_collisions.py'),str(out),'--workers','1','--stride','1'],check=True)


if __name__=='__main__':main()
