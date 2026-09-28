"""Bounded experiments on a finalized real accepted state, never a live file."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,json,sys,time
import numpy as np
from compute_resources import apply_affinity

def main():
    apply_affinity()
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--method',choices=['grouped','adaptive'],required=True)
    p.add_argument('--workers',type=int,default=1)
    p.add_argument('--seconds',type=float,default=45.)
    p.add_argument('--span',type=float,default=.001)
    p.add_argument('--step-factor',type=float,default=1.)
    p.add_argument('--tolerance-factor',type=float,default=1.)
    args=p.parse_args();root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/'src'))
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    from dbf_stability.checkpoint import IntegrationStop
    from dbf_stability.contact_integration import checked_radau
    run=args.run.resolve();args.out.mkdir(parents=True,exist_ok=False)
    status=json.loads((run/'mission/summary.json').read_text(encoding='utf8'))
    if status['status']=='running':raise ValueError('Finalized source required')
    c=json.loads((run/'mission/inputs.json').read_text(encoding='utf8'))
    with np.load(run/'mission/accepted_checkpoint.npz',allow_pickle=False) as d:
        t=float(d['time']);y=d['state'].copy();meta=json.loads(str(d['metadata']))
    c['simulation']['jacobian_method']=args.method
    c['simulation']['jacobian_workers']=args.workers
    c['simulation']['jacobian_step_factor']=args.step_factor
    c['simulation']['rtol']*=args.tolerance_factor
    c['simulation']['atol']*=args.tolerance_factor
    a=AeroDatabase(json.loads((run/'source.json').read_text())['aero_path'])
    m=CoupledModel(c,a,meta['trim']);m.active_override=meta['active_nodes']
    m.captured=meta['captured'];m.capture_time=meta['capture_time_s']
    # Load CAD/JIT before measuring steady-state integration.
    m.rhs(t,y);started=time.perf_counter();count=0;accepted=[]
    def rhs(tt,z):
        nonlocal count
        count+=1
        if time.perf_counter()-started>args.seconds:raise IntegrationStop('benchmark_budget','Bounded experiment')
        return m.rhs(tt,z)
    def accept(tt,z,*rest):accepted.append(tt)
    jac=m.jacobian() if args.method=='grouped' else None
    end=t+args.span
    for key in ('length_schedule','door_schedule'):
        end=min([end]+[float(corner) for corner,_ in c['winch'][key] if corner>t+1e-10])
    result=checked_radau(rhs,(t,end),y,m,[],c['simulation']['max_step_s'],
        c['simulation']['rtol'],c['simulation']['atol'],jac,on_accepted=accept)
    elapsed=time.perf_counter()-started
    data=dict(method=args.method,workers=args.workers,start_s=t,end_s=float(result.t[-1]),
        advance_s=float(result.t[-1]-t),target_end_s=end,tolerance_factor=args.tolerance_factor,
        step_factor=args.step_factor,success=result.success,message=result.message,
        wall_s=elapsed,steps=len(result.t)-1,rhs_calls=count,
        minimum_gap_m=result.minimum_checked_gap_m)
    np.savez_compressed(args.out/'trajectory.npz',time=result.t,states=result.y.T)
    (args.out/'result.json').write_text(json.dumps(data,indent=2),encoding='utf8')
    print(json.dumps(data),flush=True)
    if hasattr(m,'close'):m.close()
    elif m._mesh_contacts:m._mesh_contacts.close()

if __name__=='__main__':main()
