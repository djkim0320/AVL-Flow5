"""Compare conservative distance certificates against every original query."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import copy,json,sys,time
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE/'solver_variants/separation_cache'))
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.contact_integration import checked_radau

c=load_case(HERE/'runs/recovery_repair_11/analytic_80/case.yaml')
source=json.loads((ROOT/'outputs/validation/captured_cost.json').read_text())['source']
with np.load(source,allow_pickle=False) as d:t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
models=[]
for enabled in (False,True):
 config=copy.deepcopy(c);config['collision']['separation_cache']=enabled
 m=CoupledModel(config,a,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes'];models.append(m)
rng=np.random.default_rng(435);maximum=0.;gap_error=0.;elapsed=[0.,0.]
for index in range(60):
 z=y.copy();z[13:16]+=rng.uniform(-.0003,.0003,3);z[26:29]+=rng.uniform(-.0003,.0003,3)
 values=[];gaps=[]
 for j,m in enumerate(models):
  start=time.perf_counter();values.append(m.rhs(t,z));gaps.append(m.clearance_metric(t,z));elapsed[j]+=time.perf_counter()-start
 maximum=max(maximum,float(abs(values[0]-values[1]).max()));gap_error=max(gap_error,abs(gaps[0]-gaps[1]))
np.testing.assert_allclose(maximum,0,atol=1e-10);np.testing.assert_allclose(gap_error,0,atol=1e-12)
m=models[1];start=time.perf_counter();r=checked_radau(m.rhs,(t,t+.003),y,m,[],.003,1e-5,1e-7,m.jacobian())
answer=dict(poses=60,max_rhs_difference=maximum,max_clearance_difference_m=gap_error,elapsed_without_and_with_cache_s=elapsed,
 integration_elapsed_s=time.perf_counter()-start,steps=len(r.t)-1,success=r.success,
 scope='Distance-query acceleration only; same force and clearance results at sampled perturbed poses')
(ROOT/'outputs/validation/separation_cache.json').write_text(json.dumps(answer,indent=2));print(json.dumps(answer),flush=True)
for m in models:m._mesh_contacts.close()
