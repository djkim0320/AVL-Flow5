"""Actual-state continuation with an explicit faster post-latch reel command."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,copy,hashlib,json,sys
import numpy as np
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
if __name__=='__main__':
    sys.path.insert(0,str(HERE/'solver_variants/fast_analytic_flow'))
from dbf_stability import load_case,AeroDatabase,simulate
from dbf_stability.model import schedule,CoupledModel

def cleanup_schedule(config,start,speed=.25,ramp=.2):
    w=config['winch'];old=copy.deepcopy(w)
    length,rate=map(float,schedule(w['length_schedule'],start));target=float(w['length_schedule'][-1][1])
    initial=-rate
    if not np.isfinite([start,speed,ramp,length,rate,target]).all() or speed<=0 or ramp<=0:
        raise ValueError('Finite positive cleanup speed and ramp required')
    if not 0<=initial<speed or length<target-1e-12:raise ValueError('Requires recovery or fully reeled line')
    ramp_distance=(initial+speed)*ramp/2;stop_distance=speed*ramp/2
    coast=(length-target-ramp_distance-stop_distance)/speed
    knots=[(start,length)];t=start;l=length
    mode='accelerated_cleanup';peak=speed
    if length-target<=1e-12:
        if abs(rate)>1e-12:raise ValueError('Line at target but command still moving')
        mode='already_reeled';peak=0.;l=target
    elif coast<=0:
        # Capture can occur during the last braking ramp, with less than a
        # millimetre left. Preserve that already prescribed motion exactly;
        # accelerating it again is unnecessary and may not fit the distance.
        future=[(float(tt),float(ll)) for tt,ll in old['length_schedule'] if tt>start]
        reached=next((i for i,(_,ll) in enumerate(future) if abs(ll-target)<=1e-12),None)
        if reached is None:raise ValueError('Original recovery never reaches the stowed line length')
        knots+=future[:reached+1]
        rows=np.asarray(knots);rates=np.diff(rows[:,1])/np.diff(rows[:,0])
        if np.any(rates>1e-12) or np.max(abs(rates))>speed+1e-12:
            raise ValueError('Original final recovery violates cleanup direction or speed')
        t,l=knots[-1];peak=float(np.max(abs(rates)));mode='retained_final_recovery'
    else:
        for duration,v0,v1 in [(ramp,initial,speed),(coast,speed,speed),(ramp,speed,0.)]:
            for tau in np.linspace(0,duration,11)[1:]:
                knots.append((t+float(tau),float(l-v0*tau-(v1-v0)*tau*tau/(2*duration))))
            t+=duration;l=knots[-1][1]
    assert abs(l-target)<1e-12
    knots[-1]=(t,target);close_start=t+.2;close_end=close_start+.8;end=close_end+.3
    w['length_schedule']=[p for p in old['length_schedule'] if p[0]<start]+knots+[(end,target)]
    door=schedule(old['door_schedule'],start)[0]
    w['door_schedule']=[p for p in old['door_schedule'] if p[0]<start]+[(start,door),(close_start,door),(close_end,0.),(end,0.)]
    assert w.get('door_capture_interlock')
    config['mission_profile'].update(retrieval_complete_s=t,end_s=end,post_capture_cleanup_start_s=start,
        post_capture_reel_speed_m_s=peak,post_capture_reel_ramp_s=ramp,post_capture_cleanup_mode=mode)
    config['simulation']['duration_s']=end;config['simulation']['maximum_runtime_s']=3600.
    action=(f'accelerate the remaining internal line to {speed:g} m/s over {ramp:g} s and brake over the same duration' if mode=='accelerated_cleanup' else
            'retain the original remaining final-recovery knots without re-acceleration' if mode=='retained_final_recovery' else 'hold the already fully reeled line')
    config['provenance']['winch']['source']+=f' After confirmed capture at the saved branch state {start:.9f} s, {action}, then wait 0.2 s and close the door over 0.8 s. Prescribed command, not a torque-limited motor model. The existing capture interlock and torque/power limits remain.'
    return end,old

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--speed',type=float,default=.25)
    args=p.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    def record(stage,**kw):
        (out/'pipeline.json').write_text(json.dumps(dict(stage=stage,**kw),ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(dict(stage=stage,**kw)),flush=True)
    source=sorted((args.source/'mission/accepted_history').glob('part_*.npz'))[-1]
    with np.load(source,allow_pickle=False) as d:t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
    if not meta['captured']:raise ValueError('Faster cleanup is permitted only after actual capture')
    c=load_case(args.source/'case.yaml');end,old=cleanup_schedule(c,t,args.speed)
    case=out/'case.yaml';case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8');c=load_case(case)
    branch=dict(source_parent=str(args.source.resolve()),history_file=str(source.resolve()),history_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        start_s=t,capture_time_s=meta['capture_time_s'],initial_state_sha256=hashlib.sha256(y.tobytes()).hexdigest(),
        old_winch=old,new_winch=c['winch'],scope='Unmodified captured mechanical state; only subsequent reel and door commands changed. Acceleration solver matched reference RHS and full mesh clearance at 40 recorded/perturbed poses.')
    (out/'branch_source.json').write_text(json.dumps(branch,ensure_ascii=False,indent=2),encoding='utf8')
    a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz');m=CoupledModel(c,a,meta['trim']);m.captured=True;m.capture_time=meta['capture_time_s']
    if m.clearance_metric(t,y)<=0:raise ValueError('Initial actual geometry is invalid')
    m._mesh_contacts.close();record('simulation',start_time_s=t,end_time_s=end)
    r=simulate(c,a,meta['trim'],phase='mission',initial_state=y,start_time=t,duration=end-t,output=out/'mission',initial_capture_time=meta['capture_time_s'])
    np.testing.assert_array_equal(r.states[0],y)
    r.summary.update(branch_source=branch,scope='포획된 실제 저장 상태에서 이어 계산한 내부 줄 정리·문 닫힘 시험. 위치·속도·자세를 바꾸지 않았으며 이후 윈치·문 명령을 변경했습니다.')
    r.save(out/'mission');record('calculated',status=r.summary['status'],captured=r.summary['captured'],end_time_s=float(r.time[-1]),final_door_deg=r.summary['final_door_deg'])

if __name__=='__main__':main()
