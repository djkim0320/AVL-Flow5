"""Join the recovered accepted prefix and strict checkpoint continuation."""
from pathlib import Path
import json
import copy
import hashlib
import numpy as np
import pandas as pd
from dbf_stability.analysis import load_result, SimulationResult
from dbf_stability.plots import history_figure
from recover_span_hold_journal import finish_summary

OUT=Path(__file__).resolve().parent/'runs/span_hold_01'

if __name__=='__main__':
    paths=[OUT/'recovered',OUT/'continuation']
    first,second=[load_result(p) for p in paths]
    assert first.time[-1]==second.time[0]
    assert np.array_equal(first.states[-1],second.states[0])
    checkpoint=OUT/'mission/accepted_checkpoint.tmp'
    assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()==second.summary['restart_source_sha256']
    with np.load(checkpoint) as d:identity=json.loads(str(d['metadata']))['fingerprints']
    assert identity==first.summary['fingerprints']
    with np.load(paths[1]/'accepted_checkpoint.npz') as d:
        assert identity==json.loads(str(d['metadata']))['fingerprints']
    time=np.r_[first.time,second.time[1:]];states=np.vstack([first.states,second.states[1:]])
    frame=pd.concat([first.table,second.table.iloc[1:]],ignore_index=True)
    assert np.all(np.diff(time)>=0)
    base=copy.deepcopy(second.summary)
    for key in ('restart_source','restart_source_sha256'):base.pop(key,None)
    base.update(requested_duration_s=first.config['simulation']['duration_s'],runtime_s=None,
        clearance_checks=None,minimum_checked_mesh_gap_m=None,
        scope='2.7m 전개 후 60초 유지·회수 명령을 적용한 실제 계산. 저장 파일 잠금 전의 정상 저장 궤적과 1.56009초부터 재개한 궤적을 연결했으며 위치·속도·자세를 수정하지 않았습니다.',
        restart_state_continuous=True,
        source_segments=[dict(path=str(p),status=r.summary['status'],start_time_s=float(r.time[0]),end_time_s=float(r.time[-1]),
            sha256={name:hashlib.sha256((p/name).read_bytes()).hexdigest() for name in ('states.npz','inputs.json','summary.json')}) for p,r in zip(paths,(first,second))])
    summary=finish_summary(time,frame,base,first.config)
    profile=first.config['mission_profile']
    held=max(0.,min(time[-1],profile['recovery_start_s'])-profile['deployment_complete_s'])
    summary['observed_command_hold_s']=held
    summary['hold_60s_completed']=bool(held>=60.-1e-8)
    summary['recovery_started']=bool(time[-1]>profile['recovery_start_s'])
    summary['retrieval_command_completed']=bool(time[-1]>=profile['retrieval_complete_s'])
    expected_mass=first.config['aircraft']['mass_kg']+first.config['sensor']['mass_kg']+first.config['cable']['length_m']*first.config['cable']['density_kg_m']
    assert np.allclose(frame.total_mass_kg,expected_mass,rtol=0,atol=1e-12)
    summary['total_mass_kg']=expected_mass
    events=first.events+[dict(time_s=float(second.time[0]),event='strict_checkpoint_resume')]+second.events
    result=SimulationResult(time,states,frame,summary,events,first.config)
    target=OUT/'combined';target.mkdir(exist_ok=False)
    result.save(target)
    history_figure(result).write_html(target/'history.html',include_plotlyjs=True)
    print(json.dumps({k:summary[k] for k in ('status','end_time_s','max_tension_N','max_pitch_change_deg','observed_command_hold_s','recovery_started','captured')},ensure_ascii=False))
