"""Compare isolated reference and accelerated solvers at recorded poses."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import importlib,importlib.util,json,sys,time
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
def package(name,folder):
    spec=importlib.util.spec_from_file_location(name,folder/'__init__.py',submodule_search_locations=[str(folder)])
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module,importlib.import_module(name+'.model'),importlib.import_module(name+'.contact_integration')
first=package('reference_solver',HERE/'solver_variants/internal_cable_flow/dbf_stability')
second=package('accelerated_solver',HERE/'solver_variants/fast_analytic_flow/dbf_stability')
c=first[0].load_case(HERE/'runs/recovery_repair_11/analytic_80/case.yaml');c['flight']['cable_bay_shielding']=True;c['flight']['internal_cable_flow_factor']=0.
source=json.loads((ROOT/'outputs/validation/captured_cost.json').read_text())['source']
with np.load(source,allow_pickle=False) as d:t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
with np.load(ROOT/'tests/data/capture_stop_contact.npz',allow_pickle=False) as d:poses=list(d['states'].copy())
rng=np.random.default_rng(905)
for i in range(36):
    z=y.copy();z[13:16]+=rng.uniform(-.0003,.0003,3);z[26:29]+=rng.uniform(-.0003,.0003,3);poses.append(z)
models=[]
for pkg,mod,_ in (first,second):
    aero=pkg.AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    m=mod.CoupledModel(c,aero,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes'];models.append(m)
elapsed=[0.,0.];max_diff=0.;gap_diff=0.
for z in poses:
    values=[];gaps=[]
    for i,m in enumerate(models):
        start=time.perf_counter();values.append(m.rhs(t,z));elapsed[i]+=time.perf_counter()-start;gaps.append(m.clearance_metric(t,z))
    np.testing.assert_allclose(values[0],values[1],rtol=1e-7,atol=1e-6)
    max_diff=max(max_diff,float(abs(values[0]-values[1]).max()));gap_diff=max(gap_diff,abs(gaps[0]-gaps[1]))
assert gap_diff<1e-12
m=models[1];start=time.perf_counter();r=second[2].checked_radau(m.rhs,(t,t+.003),y,m,[],.003,1e-5,1e-7,m.jacobian())
answer=dict(poses=len(poses),max_rhs_difference=max_diff,max_clearance_difference_m=gap_diff,rhs_elapsed_reference_fast_s=elapsed,
    integration_elapsed_s=time.perf_counter()-start,steps=len(r.t)-1,success=r.success,
    scope='Recorded and perturbed-state force equivalence and unchanged full-mesh clearance; not trajectory convergence')
(ROOT/'outputs/validation/fast_contact.json').write_text(json.dumps(answer,indent=2));print(json.dumps(answer),flush=True)
for m in models:m._mesh_contacts.close()
