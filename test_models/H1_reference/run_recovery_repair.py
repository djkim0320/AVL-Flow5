"""Independent recovery experiments from a saved pre-contact state.

No prescribed sensor motion, force removal, capture-tolerance widening or
collision bypass. The real 25 m/s flow5 aircraft database is reused unchanged.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import argparse,copy,hashlib,json,subprocess,sys
import numpy as np
import yaml
from scipy.interpolate import CubicHermiteSpline
from dbf_stability import load_case,AeroDatabase,simulate
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation,cross
from dbf_stability.model import schedule,CoupledModel

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
BASE=HERE/'runs/control_100m_25ms_02'
AERO=HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz'


def slowed_retrieval(c,start):
    original=copy.deepcopy(c['winch']['length_schedule'])
    length,v0=schedule(original,start)
    low=c['winch']['stowed_length_m'];v1=-.04;deceleration=.6
    end_ramp=start+deceleration;l1=length+(v0+v1)*deceleration/2
    brake=1.;l2=low-v1*brake/2
    cruise_end=end_ramp+(l1-l2)/abs(v1);end=cruise_end+brake
    first=CubicHermiteSpline([start,end_ramp],[length,l1],[v0,v1])
    last=CubicHermiteSpline([cruise_end,end],[l2,low],[v1,0.])
    times=np.unique(np.r_[np.arange(start,end,.05),start,end_ramp,cruise_end,end,end+3.])
    rows=[r for r in original if r[0]<start]
    for t in times:
        value=float(first(t)) if t<=end_ramp else l1+v1*(t-end_ramp) if t<cruise_end else float(last(t)) if t<=end else low
        rows.append([float(t),float(max(low,value))])
    c['winch']['length_schedule']=rows
    opening=[r for r in c['winch']['door_schedule'] if r[0]<start]
    c['winch']['door_schedule']=opening+[[end+.8,140.],[end+1.6,0.],[end+3.,0.]]
    c['mission_profile'].update(slow_approach_start_s=start,retrieval_complete_s=end,end_s=end+3.,
        speed_policy='Smooth 0.6 s approach deceleration to 0.04 m/s, 1 s terminal braking; same 60 s tow.')
    c['simulation']['duration_s']=end+3.
    c['provenance']['winch']['source']+=' Repair trial: final retrieval slowed to 0.04 m/s after a 0.6 s deceleration; 1 s braking; same capture and physical contact limits. Prescribed speed, not torque-controlled motor dynamics.'
    for t in np.linspace(0,start,201):
        assert abs(schedule(original,t)[0]-schedule(rows,t)[0])<1e-12
    return c


def execute(spec):
    name,parent,kind=spec
    out=Path(parent)/name;out.mkdir(parents=True,exist_ok=False)
    def record(stage,**data):
        value=dict(stage=stage,**data);p=out/'pipeline.tmp'
        p.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');p.replace(out/'pipeline.json')
        print(name,json.dumps(value,ensure_ascii=False),flush=True)
    try:
        baseline=load_result(BASE/'mission')
        index=int(np.argmin(abs(baseline.time-69.4)));start=float(baseline.time[index]);old=baseline.states[index].copy()
        c=load_case(ROOT/'examples/h1_normal_r3_round_x_sensor.yaml') if kind.startswith('round') else copy.deepcopy(baseline.config)
        if kind.endswith('slow'):slowed_retrieval(c,start)
        c['collision']['feature_engine']='numba'
        c['name']='R3_recovery_repair_'+name
        c['simulation']['maximum_runtime_s']=1800.
        case=ROOT/'examples'/f'h1_recovery_repair_{name}.yaml'
        case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
        c=load_case(case);y=old.copy();shift=np.asarray(c['sensor'].get('cg_shift_from_r3_body_m',[0.,0.,0.]))
        rs=rotation(old[19:23]);offset=rs@shift
        y[13:16]+=offset;y[16:19]+=cross(rs@old[23:26],offset)
        # Change only the mass-reference coordinate for a redesigned sensor.
        # Its external placement and the cable endpoint are not moved into the bay.
        old_point=old[13:16]+rs@np.asarray(baseline.config['sensor']['tow_point_m'])
        new_point=y[13:16]+rs@np.asarray(c['sensor']['tow_point_m'])
        assert np.allclose(old_point,new_point,atol=1e-12,rtol=0.)
        assert c['bay']['capture_radius_m']==baseline.config['bay']['capture_radius_m']
        assert c['bay']['capture_speed_m_s']==baseline.config['bay']['capture_speed_m_s']
        assert c['bay']['capture_angle_deg']==baseline.config['bay']['capture_angle_deg']
        assert c['collision']['minimum_gap_m']==baseline.config['collision']['minimum_gap_m']
        a=AeroDatabase(AERO);trim=json.loads((BASE/'trim.json').read_text(encoding='utf8'))
        m=CoupledModel(c,a,trim,phase='mission');gap=m.clearance_metric(start,y)+c['collision']['minimum_gap_m']
        if gap<=c['collision']['minimum_gap_m']:raise ValueError('Initial candidate pose intersects geometry')
        source=dict(baseline=str(BASE/'mission'),time_s=start,row=index,initial_minimum_gap_m=gap,
            state_sha256=hashlib.sha256(old.tobytes()).hexdigest(),candidate_state_sha256=hashlib.sha256(y.tobytes()).hexdigest(),
            cg_reference_shift_body_m=shift.tolist(),external_towpoint_preserved=True,
            aero_database=str(AERO),aero_sha256=hashlib.sha256(AERO.read_bytes()).hexdigest(),
            scope='Recovery-only new design/control experiment at a saved pre-contact pose. Not a newly simulated deployment or strict unchanged-input resume.')
        (out/'branch_source.json').write_text(json.dumps(source,ensure_ascii=False,indent=2),encoding='utf8')
        (out/'trim.json').write_text(json.dumps(trim,indent=2),encoding='utf8')
        record('simulation',start_time_s=start,end_time_s=c['mission_profile']['end_s'],initial_gap_m=gap)
        r=simulate(c,a,trim,phase='mission',initial_state=y,start_time=start,
            duration=c['mission_profile']['end_s']-start,output=out/'mission')
        r.summary.update(branch_source=source,
            scope=f'회수 수정안 시험: 기존 {start:.3f}초 자세에서 시작. 센서 CG 좌표 변경은 외부 견인점 위치·속도를 보존하며 변환했습니다. 수정안의 처음부터 전개 과정은 이 결과에 포함하지 않습니다.')
        r.save(out/'mission')
        record('calculated',status=r.summary['status'],captured=r.summary['captured'],end_time_s=float(r.time[-1]),
            max_tension_N=r.summary['max_tension_N'],max_contact_N=r.summary['max_contact_N'],final_door_deg=r.summary['final_door_deg'])
        return dict(name=name,path=str(out),status=r.summary['status'],captured=r.summary['captured'],end_time_s=float(r.time[-1]))
    except Exception as exc:
        record('failed',message=str(exc));raise


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=2)
    p.add_argument('--variants',nargs='+',default=['original_slow','round_nominal'])
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    results=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(execute,(name,str(args.out.resolve()),name)) for name in args.variants]
        for future in as_completed(futures):
            try:results.append(future.result())
            except Exception as exc:results.append(dict(status='failed',message=str(exc)))
            (args.out/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)
