"""Read an immutable accepted-history chunk and profile its archived force model."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,cProfile,json,pstats,sys,time
import numpy as np

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--integration',action='store_true')
    args=p.parse_args();run=args.run.resolve();args.out.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(run/'solver'))
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    c=json.loads((run/'mission/inputs.json').read_text(encoding='utf8'))
    chunk=sorted((run/'mission/accepted_history').glob('part_*.npz'))[-1]
    with np.load(chunk,allow_pickle=False) as d:
        t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
    source=json.loads((run/'source.json').read_text(encoding='utf8'))
    m=CoupledModel(c,AeroDatabase(source['aero_path']),meta['trim'])
    m.active_override=meta['active_nodes'];m.captured=meta['captured'];m.capture_time=meta['capture_time_s']
    _,diagnostics=m.rhs(t,y,True)
    original=m._mesh_contacts.mesh_mesh_features;calls=[]
    def observed(*args):
        index=len(calls);start=time.perf_counter();result=original(*args)
        calls.append(dict(pair=index,shapes=[getattr(v,'shape',None) for v in args],seconds=time.perf_counter()-start,contacts=len(result)))
        np.savez_compressed(args_out/f'pair_{index}.npz',**{f'a{i}':v for i,v in enumerate(args)})
        return result
    args_out=args.out;m._mesh_contacts.mesh_mesh_features=observed
    m.rhs(t,y);m._mesh_contacts.mesh_mesh_features=original
    (args.out/'pairs.json').write_text(json.dumps(calls,indent=2),encoding='utf8')
    profiler=cProfile.Profile();profiler.enable()
    for _ in range(60):m.rhs(t,y)
    profiler.disable();profiler.dump_stats(str(args.out/'rhs.prof'))
    with (args.out/'rhs.txt').open('w',encoding='utf8') as stream:pstats.Stats(profiler,stream=stream).sort_stats('cumtime').print_stats(35)
    from dbf_stability.contact_integration import surface_speed
    result=dict(chunk=str(chunk),time_s=t,diagnostics=diagnostics,surface_speed=surface_speed(m,t,y),gap=m.clearance_metric(t,y))
    (args.out/'state.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result),flush=True)
    if args.integration:
        from dbf_stability.contact_integration import checked_radau
        length,payout=m.length(t);delta=.002
        for key in ('length_schedule','door_schedule'):
            for corner,_ in c['winch'][key]:
                if corner>t+1e-10:delta=min(delta,(corner-t)/2)
        if payout:
            for j in range(m.n):
                distance=((j+.5)*m.h-length)/payout
                if distance>1e-10:delta=min(delta,distance/2)
        profile=cProfile.Profile();profile.enable()
        integration=checked_radau(m.rhs,(t,t+delta),y,m,[],c['simulation']['max_step_s'],c['simulation']['rtol'],c['simulation']['atol'],m.jacobian())
        profile.disable()
        with (args.out/'integration.txt').open('w',encoding='utf8') as stream:pstats.Stats(profile,stream=stream).sort_stats('cumtime').print_stats(35)
        print(json.dumps(dict(integrated_s=delta,steps=len(integration.t)-1,success=integration.success)),flush=True)
    m._mesh_contacts.close()
