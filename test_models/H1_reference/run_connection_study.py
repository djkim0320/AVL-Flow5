"""Real solver/mission connection checks; preserved R3 results are read-only."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import hashlib
import json
import numpy as np
from dbf_stability import load_case,AeroDatabase,solve_trim,analyze_stability,simulate,resume_simulation
from dbf_stability.plots import history_figure
from dbf_stability.math3d import quaternion

ROOT=Path(__file__).resolve().parents[2]
HERE=ROOT/'test_models/H1_reference'
OUT=HERE/'runs/flow5_connection_01'
FLOW=OUT/'aerodynamics/aero_database.npz'
AVL=HERE/'runs/normal_r3_01/aerodynamics/aero_database.npz'


def dump(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')


def job(kind):
    if kind=='door_interlock':
        source=HERE/'runs/normal_r3_01/time_refined/mission'
        c=json.loads((source/'inputs.json').read_text(encoding='utf8'))
        c['winch'].update(door_capture_interlock=True,door_capture_hold_s=.2)
        c['simulation'].update(maximum_runtime_s=600,stagnation_window_s=120,stagnation_min_advance_s=.001,checkpoint_interval_s=10)
        aero=AeroDatabase(AVL);trim=solve_trim(c,aero,mode='stowed')
        with np.load(source/'states.npz') as data:
            index=int(np.argmin(abs(data['time']-9.7)));t=float(data['time'][index]);y=data['states'][index].copy()
        target=OUT/kind/'mission'
        r=simulate(c,aero,trim,initial_state=y,start_time=t,duration=11-t,output=target)
        r.summary['scope']='기존 AVL 궤적의 9.70초 상태에서 시작한 도어 연동 재계산입니다. 전 과정 재실행 결과가 아닙니다.'
        r.summary['initial_state_source']=dict(path=str(source/'states.npz'),sha256=hashlib.sha256((source/'states.npz').read_bytes()).hexdigest(),row=index,time_s=t)
        r.save(target)
        # Check the strict restart path with the actual contact/sequence state.
        if r.summary['status']=='completed':
            resumed=resume_simulation(c,aero,target/'accepted_checkpoint.npz',duration=.02,output=OUT/kind/'restart')
            dump(OUT/kind/'restart_check.json',dict(status=resumed.summary['status'],state_continuous=bool(np.array_equal(r.states[-1],resumed.states[0])),capture_preserved=r.summary['captured']==resumed.summary['captured'],door_continuous_deg=float(resumed.table.door_deg.iloc[0]-r.table.door_deg.iloc[-1])))
    else:
        c=load_case(ROOT/'examples/h1_normal_r3_flow5.yaml');aero=AeroDatabase(FLOW)
        c['simulation'].update(maximum_runtime_s=600,stagnation_window_s=120,checkpoint_interval_s=10)
        mode='stowed' if kind=='flow5_deployment' else 'aircraft_only'
        trim=solve_trim(c,aero,mode=mode);target=OUT/kind/'mission'
        target.mkdir(parents=True,exist_ok=False)
        dump(target/'trim.json',{k:v for k,v in trim.items() if k!='state'})
        if mode=='aircraft_only':
            linear=analyze_stability(c,aero,trim);linear['modes'].to_csv(target/'modes.csv',index=False)
            np.savez_compressed(target/'linearization.npz',matrix=linear['matrix'],eigenvalues=linear['eigenvalues'])
            initial=trim['state'].copy();initial[6:10]=quaternion(pitch=trim['alpha_rad']+np.deg2rad(1))
            r=simulate(c,aero,trim,phase=mode,initial_state=initial,duration=20,output=target)
            r.summary['scope']='flow5 기체 단독 1도 피치 교란 응답입니다. 센서 결합 안정성 판정은 포함하지 않습니다.'
            r.summary['linear_unstable']=linear['unstable'];r.save(target)
        else:
            r=simulate(c,aero,trim,phase='mission',duration=3,output=target)
            r.summary['scope']='flow5 공력을 사용한 0–3초 문 개방·센서 전개 계산입니다. 회수·포획은 이 구간에 포함하지 않았습니다.'
            r.save(target)
    history_figure(r).write_html(target/'history.html',include_plotlyjs=True)
    return kind,{k:v for k,v in r.summary.items() if k not in ('assumptions','collision_metadata','aero_metadata')}


if __name__=='__main__':
    dump(OUT/'source_versions.json',{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/dbf_stability').glob('*.py')})
    with ProcessPoolExecutor(max_workers=3) as pool:
        rows={}
        for future in as_completed([pool.submit(job,k) for k in ['door_interlock','flow5_deployment','flow5_airframe_response']]):
            kind,result=future.result();rows[kind]=result;dump(OUT/'connection_results.json',rows);print(json.dumps({kind:result}),flush=True)
