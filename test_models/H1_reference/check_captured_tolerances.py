"""Compare integration accuracy/cost at the same recorded captured state."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import copy,json,sys,time
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE/'solver_variants/analytic_stop'))
from dbf_stability import load_case,AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.contact_integration import checked_radau
from dbf_stability.checkpoint import IntegrationStop


def main():
    parent=HERE/'runs/recovery_repair_11/analytic_80';base=load_case(parent/'case.yaml')
    file=sorted((parent/'mission/accepted_history').glob('part_*.npz'))[-1]
    with np.load(file,allow_pickle=False) as d:start=float(d['time'][-1]);state=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
    assert meta['captured'];end=min(start+.005,min(t for t,_ in base['winch']['length_schedule'] if t>start+1e-8))
    a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz');records=[];end_states=[]
    for factor in (1.,10.,100.):
        c=copy.deepcopy(base);c['simulation']['rtol']*=factor;c['simulation']['atol']*=factor
        m=CoupledModel(c,a,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s'];m.active_override=meta['active_nodes']
        m.rhs(start,state);count=[0];real_rhs=m.rhs;started=time.perf_counter()
        def rhs(t,y,diagnostics=False):
            count[0]+=1
            if time.perf_counter()-started>120:raise IntegrationStop('probe_budget','120 s probe limit')
            return real_rhs(t,y,diagnostics)
        m.rhs=rhs
        r=checked_radau(rhs,(start,end),state,m,[],c['simulation'].get('contact_max_step_s',.003),c['simulation']['rtol'],c['simulation']['atol'],m.jacobian())
        elapsed=time.perf_counter()-started
        _,d=real_rhs(r.t[-1],r.y[:,-1],True)
        records.append(dict(factor=factor,rtol=c['simulation']['rtol'],atol=c['simulation']['atol'],elapsed_s=elapsed,
            rhs_evaluations=count[0],accepted_steps=len(r.t)-1,end_s=float(r.t[-1]),success=r.success,
            final_contact_N=float(d['contact_N']),final_tension_N=float(d['tension_N']),final_capture_N=float(d['capture_N'])))
        end_states.append(r.y[:,-1]);m._mesh_contacts.close();print(records[-1],flush=True)
    errors=[dict(factor=records[i]['factor'],max_state_difference=float(abs(y-end_states[0]).max()),
        aircraft_position_difference_m=float(np.linalg.norm(y[:3]-end_states[0][:3])),
        sensor_position_difference_m=float(np.linalg.norm(y[13:16]-end_states[0][13:16])),
        sensor_velocity_difference_m_s=float(np.linalg.norm(y[16:19]-end_states[0][16:19]))) for i,y in enumerate(end_states)]
    result=dict(start_s=start,end_s=end,source=str(file),records=records,errors=errors,
        scope='5 ms post-capture tolerance probe only; not full-trajectory convergence')
    (ROOT/'outputs/validation/captured_tolerance_probe.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    np.savez_compressed(ROOT/'outputs/validation/captured_tolerance_probe.npz',initial=state,end_states=end_states,time=[start,end])
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
