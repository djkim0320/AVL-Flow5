"""Strictly continue a runtime-limited repair case and join its saved results."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,copy,hashlib,json
import numpy as np
import pandas as pd
from dbf_stability import AeroDatabase,resume_simulation
from dbf_stability.analysis import SimulationResult,load_result

HERE=Path(__file__).resolve().parent


def join(first,second,sources):
    if first.time[-1]!=second.time[0] or not np.array_equal(first.states[-1],second.states[0]):
        raise ValueError('Continuation state is discontinuous; refusing to join')
    tail=second.table.iloc[1:].copy()
    for key in ('contact_impulse_Ns','capture_impulse_Ns','winch_work_J'):
        tail[key]+=float(first.table[key].iloc[-1])
    table=pd.concat([first.table,tail],ignore_index=True)
    times=np.r_[first.time,second.time[1:]];states=np.vstack([first.states,second.states[1:]])
    delta=np.diff(times)
    if np.any(delta<0):raise ValueError('Decreasing combined timestamps')
    # Capture records the left/right force limits at the identical mechanical
    # state and time. Preserve both; only differing states at equal times fail.
    for index in np.flatnonzero(delta==0):
        if not np.array_equal(states[index],states[index+1]):
            raise ValueError('Different mechanical states at the same timestamp')
    summary=copy.deepcopy(second.summary)
    summary.update(start_time_s=float(times[0]),end_time_s=float(times[-1]),duration_s=float(times[-1]-times[0]),
        requested_duration_s=first.summary['requested_duration_s'],runtime_s=first.summary['runtime_s']+second.summary['runtime_s'],
        max_tension_N=float(table.tension_N.max()),max_contact_N=float(table.contact_N.max()),
        max_capture_N=float(table.capture_N.max()),max_penetration_m=float(table.penetration_m.max()),
        contact_impulse_Ns=float(table.contact_impulse_Ns.iloc[-1]),capture_impulse_Ns=float(table.capture_impulse_Ns.iloc[-1]),
        max_pitch_change_deg=float(abs(table.pitch_deg-table.pitch_deg.iloc[0]).max()),
        clearance_checks=first.summary['clearance_checks']+second.summary['clearance_checks'],
        minimum_checked_mesh_gap_m=min(first.summary['minimum_checked_mesh_gap_m'],second.summary['minimum_checked_mesh_gap_m']),
        geometry_validity='not_audited',branch_source=first.summary.get('branch_source'),
        scope=f'{times[0]:.3f}초 저장 자세에서 시작한 회수 수정안 시험. 계산 시간 한도에서 저장된 상태를 그대로 이어 계산했으며, 접합 지점의 모든 위치·속도·자세·와이어 상태가 동일합니다. 새 형상의 전개 과정은 포함하지 않습니다.',
        continuation_parts=[dict(path=str(p),states_sha256=hashlib.sha256((p/'states.npz').read_bytes()).hexdigest()) for p in sources])
    violations=[]
    for key,section,limit_key in [('tension_N','cable','limit_N'),('torque_Nm','winch','limit_torque_Nm'),('winch_power_W','winch','limit_power_W'),('contact_N','bay','limit_contact_N'),('penetration_m','bay','limit_penetration_m')]:
        limit=second.config[section].get(limit_key);peak=float(table[key].abs().max()) if key=='winch_power_W' else float(table[key].max())
        if limit is not None and peak>limit:violations.append(dict(quantity=key,limit=limit,peak=peak))
    summary['violations']=violations
    return SimulationResult(times,states,table,summary,first.events+second.events,second.config)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('parent',type=Path);parser.add_argument('--budget',type=float,default=3600.)
    args=parser.parse_args();parent=args.parent.resolve();source=parent/'mission';first=load_result(source)
    if first.summary['status']!='runtime_limit':raise ValueError('Only runtime-limited trajectories may be continued by this command')
    c=copy.deepcopy(first.config);c['simulation']['maximum_runtime_s']=args.budget
    aero=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    target=first.summary['start_time_s']+first.summary['requested_duration_s'];start=float(first.time[-1]);suffix=parent/'continuation'
    def record(stage,**data):
        p=parent/'pipeline.tmp';p.write_text(json.dumps(dict(stage=stage,**data),ensure_ascii=False,indent=2),encoding='utf8');p.replace(parent/'pipeline.json')
    record('simulation',live_directory='continuation',start_time_s=start,end_time_s=target,strict_restart=True)
    second=resume_simulation(c,aero,source/'accepted_checkpoint.npz',duration=target-start,output=suffix)
    combined=join(first,second,[source,suffix]);destination=parent/'complete';combined.save(destination)
    record('calculated',live_directory='continuation',result_directory='complete',status=combined.summary['status'],
        captured=combined.summary['captured'],end_time_s=float(combined.time[-1]),
        max_tension_N=combined.summary['max_tension_N'],max_contact_N=combined.summary['max_contact_N'],final_door_deg=combined.summary['final_door_deg'])
    print(json.dumps(dict(path=str(destination),status=combined.summary['status'],captured=combined.summary['captured'],state_continuous=True)),flush=True)


if __name__=='__main__':main()
