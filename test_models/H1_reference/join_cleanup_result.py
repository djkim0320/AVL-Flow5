"""Join the recorded recovery prefix to the real post-capture continuation."""
from pathlib import Path
import argparse,copy,hashlib,json
import numpy as np
from dbf_stability.analysis import load_result
from resume_recovery_case import join

def main():
    p=argparse.ArgumentParser();p.add_argument('parent',type=Path);args=p.parse_args();parent=args.parent.resolve()
    first=load_result(parent/'prefix');second=load_result(parent/'mission')
    if second.summary['status']!='completed':raise ValueError('Continuation did not reach its requested end')
    # Only commands after the exact common state may differ.
    for key in ('aircraft','sensor','cable','bay','flight','collision','aero'):
        if first.config[key]!=second.config[key]:raise ValueError('Physical input changed across prefix/continuation: '+key)
    for key in ('length_schedule','door_schedule'):
        old=np.array(first.config['winch'][key]);new=np.array(second.config['winch'][key])
        np.testing.assert_allclose(np.interp(first.time,old[:,0],old[:,1]),np.interp(first.time,new[:,0],new[:,1]),atol=1e-12,rtol=0)
    combined=join(first,second,[parent/'prefix',parent/'mission'])
    evidence=json.loads((parent/'branch_source.json').read_text(encoding='utf8'))
    source=Path(evidence['source_parent'])
    combined.summary.update(scope='70.812초 실제 저장 자세부터 회수·포획을 계산한 기록과, 71.348초 포획 상태에서 내부 줄 정리 명령을 바꾼 계산을 연결했습니다. 접합 시각의 134개 상태값이 모두 동일합니다. 새 형상의 전개부터 계산한 결과는 아닙니다.',
        runtime_scope='Diagnostic reconstruction time plus continuation runtime; does not include the original prefix integration runtime.',
        accepted_history_sources=first.summary['accepted_history_sources'],
        recovery_initial_state_source=json.loads((source/'branch_source.json').read_text(encoding='utf8')),
        command_change_source=evidence,
        junction=dict(time_s=float(first.time[-1]),state_components=first.states.shape[1],all_components_exactly_equal=True,
            state_sha256=hashlib.sha256(first.states[-1].tobytes()).hexdigest()),
        clearance_checks_scope='Recomputed full-mesh checks at prefix accepted endpoints/midpoints plus continuation integrator checks; independent STEP audit is separate.')
    combined.save(parent/'complete')
    (parent/'pipeline.json').write_text(json.dumps(dict(stage='calculated',result_directory='complete',live_directory='mission',
        status=combined.summary['status'],captured=combined.summary['captured'],end_time_s=float(combined.time[-1]),
        final_door_deg=combined.summary['final_door_deg'],state_continuous=True),indent=2),encoding='utf8')
    print(json.dumps(dict(path=str(parent/'complete'),states=len(combined.time),summary=combined.summary['status'],
        captured=combined.summary['captured'],door_deg=combined.summary['final_door_deg'],violations=combined.summary['violations'])),flush=True)

if __name__=='__main__':main()
