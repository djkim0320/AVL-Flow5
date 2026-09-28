"""Diagnostic ablations only; never used as a mission result."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json,sys,time
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE/'solver_variants/analytic_stop'))
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.contact_integration import checked_radau

parent=HERE/'runs/recovery_repair_11/analytic_80';c=load_case(parent/'case.yaml')
source=json.loads((ROOT/'outputs/validation/captured_cost.json').read_text())['source']
with np.load(source,allow_pickle=False) as d:t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz');rows=[]
for exclude in ('cable','sensor'):
 m=CoupledModel(c,a,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes']
 geometry=m.contact_geometry
 def filtered(t,y,distance=None):
  hits=geometry(t,y,distance)
  return hits if distance is not None else [h for h in hits if h['kind']!=exclude]
 m.contact_geometry=filtered;m.rhs(t,y);start=time.perf_counter()
 r=checked_radau(m.rhs,(t,t+.003),y,m,[],.003,1e-5,1e-7,m.jacobian())
 row=dict(excluded_force_only=exclude,elapsed_s=time.perf_counter()-start,steps=len(r.t)-1,end_s=float(r.t[-1]),success=r.success)
 print(row,flush=True);rows.append(row);m._mesh_contacts.close()
(ROOT/'outputs/validation/contact_ablation_diagnostic.json').write_text(json.dumps(dict(scope='diagnostic ablation, not a mission or repaired result',source=source,rows=rows),indent=2))
