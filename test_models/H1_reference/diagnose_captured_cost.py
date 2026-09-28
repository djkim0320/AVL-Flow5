"""Inspect actual accepted-state forces and implicit integration work."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json,sys,time,cProfile,pstats,io
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE/'solver_variants/analytic_stop'))
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import rotation
import dbf_stability.contact_integration as ci

parent=HERE/'runs/recovery_repair_11/analytic_80';c=load_case(parent/'case.yaml')
file=sorted((parent/'mission/accepted_history').glob('part_*.npz'))[-1]
with np.load(file,allow_pickle=False) as d:t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
m=CoupledModel(c,a,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes']
f,diag=m.rhs(t,y,True);gap=m.clearance_metric(t,y)+c['collision']['minimum_gap_m'];hits=m.contact_geometry(t,y)
groups={}
for h in hits:
 key=h['moving']+'/'+h['fixed'];q=groups.setdefault(key,dict(count=0,gap_m=1.,weight=0.));q['count']+=1;q['gap_m']=min(q['gap_m'],h['gap']);q['weight']+=h.get('weight',1.)
result=dict(time_s=t,source=str(file),active_nodes=m.active_override,minimum_gap_m=gap,surface_speed=ci.surface_speed(m,t,y),
 sensor_relative_acceleration=float(np.linalg.norm(f[16:19]-f[3:6])),max_node_relative_acceleration=float(np.linalg.norm(f[26:].reshape(m.n,6)[:,3:]-f[3:6],axis=1).max()),
 groups=groups,diagnostics={k:v for k,v in diag.items() if np.isscalar(v)})
prof=cProfile.Profile();prof.enable()
for j in range(10):m.rhs(t,y)
prof.disable();out=io.StringIO();pstats.Stats(prof,stream=out).sort_stats('cumtime').print_stats(22);result['profile']=out.getvalue()
steps=[];Original=ci.Radau
class RecordingRadau(Original):
 def step(self):
  before=self.t;ev=self.nfev;ja=self.njev;lu=self.nlu;h=self.h_abs
  value=super().step()
  steps.append(dict(t=before,dt=self.t-before,proposed=h,max_step=self.max_step,rhs=self.nfev-ev,jac=self.njev-ja,lu=self.nlu-lu))
  return value
ci.Radau=RecordingRadau
start=time.perf_counter();r=ci.checked_radau(m.rhs,(t,t+.003),y,m,[],.003,1e-5,1e-7,m.jacobian())
result.update(steps=steps,elapsed_s=time.perf_counter()-start,success=r.success)
(ROOT/'outputs/validation/captured_cost.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2),flush=True);m._mesh_contacts.close()
