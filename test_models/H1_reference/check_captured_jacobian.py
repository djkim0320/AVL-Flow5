"""Read-only short integration probe at an accepted captured pose."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import inspect,json,sys,textwrap,time,types
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE/'solver_variants/analytic_stop'))
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.contact_integration import checked_radau
from dbf_stability.checkpoint import IntegrationStop


def main():
    parent=HERE/'runs/recovery_repair_11/analytic_80';c=load_case(parent/'case.yaml')
    file=sorted((parent/'mission/accepted_history').glob('part_*.npz'))[-1]
    with np.load(file,allow_pickle=False) as d:start=float(d['time'][-1]);state=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
    assert meta['captured'];end=min(start+.005,min(t for t,_ in c['winch']['length_schedule'] if t>start+1e-8))
    a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    source=textwrap.dedent(inspect.getsource(CoupledModel.jacobian))
    old='step=np.sqrt(np.finfo(float).eps)*np.maximum(1.,np.abs(y))'
    new=old+'\n        for offset in [0,13]+list(range(26,self.size,6)):\n            step[offset:offset+3]=np.sqrt(np.finfo(float).eps)*np.maximum(1.,np.abs(y[offset:offset+3]-y[:3]))'
    assert old in source;namespace=dict(vars(sys.modules[CoupledModel.__module__]));exec(source.replace(old,new),namespace)
    records=[];states=[]
    for local in (False,True):
        m=CoupledModel(c,a,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes']
        m.rhs(start,state);count=[0];real_rhs=m.rhs;started=time.perf_counter()
        def rhs(t,y,diagnostics=False):
            count[0]+=1
            if time.perf_counter()-started>120:raise IntegrationStop('probe_budget','120 s diagnostic probe limit')
            return real_rhs(t,y,diagnostics)
        m.rhs=rhs
        jac=namespace['jacobian'](m) if local else m.jacobian()
        result=checked_radau(rhs,(start,end),state,m,[],c['simulation'].get('contact_max_step_s',.003),c['simulation']['rtol'],c['simulation']['atol'],jac)
        records.append(dict(local_position_step=local,elapsed_s=time.perf_counter()-started,rhs_evaluations=count[0],accepted_steps=len(result.t)-1,end_s=float(result.t[-1]),success=result.success))
        states.append(result.y[:,-1]);m._mesh_contacts.close();print(records[-1],flush=True)
    answer=dict(start_s=start,requested_end_s=end,source=str(file),records=records,max_state_difference=float(np.max(abs(states[0]-states[1]))))
    (ROOT/'outputs/validation/captured_jacobian_probe.json').write_text(json.dumps(answer,indent=2),encoding='utf8');print(json.dumps(answer),flush=True)


if __name__=='__main__':main()
