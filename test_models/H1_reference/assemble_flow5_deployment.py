"""Join two verified continuous saved segments; no new or adjusted poses."""
from pathlib import Path
import copy
import hashlib
import json
import numpy as np
import pandas as pd
from dbf_stability.analysis import load_result, SimulationResult
from dbf_stability.plots import history_figure

RUN = Path(__file__).resolve().parent/'runs/flow5_connection_01'


def main():
    paths = [RUN/'flow5_deployment'/name for name in ('mission', 'continuation')]
    first, second = [load_result(p) for p in paths]
    assert first.time[-1] == second.time[0]
    assert np.array_equal(first.states[-1], second.states[0])
    assert second.summary['status'] == 'completed' and second.time[-1] == 3.
    assert second.summary['restart_source_sha256'] == hashlib.sha256((paths[0]/'accepted_checkpoint.npz').read_bytes()).hexdigest()
    for p in paths:
        with np.load(p/'accepted_checkpoint.npz') as d:
            identity = json.loads(str(d['metadata']))['fingerprints']
        if p == paths[0]: baseline = identity
        else: assert identity == baseline
    for key in ['captured', 'door_deg', 'length_m', 'tension_N', 'contact_N']:
        assert np.isclose(first.table[key].iloc[-1], second.table[key].iloc[0], rtol=1e-8, atol=1e-10)
    time = np.r_[first.time, second.time[1:]]
    states = np.vstack([first.states, second.states[1:]])
    frame = pd.concat([first.table, second.table.iloc[1:]], ignore_index=True)
    assert np.all(np.diff(time) >= 0)
    for name, force in [('contact_impulse_Ns', 'contact_N'), ('capture_impulse_Ns', 'capture_N'), ('winch_work_J', 'winch_power_W')]:
        values = frame[force].to_numpy()
        frame[name] = np.r_[0., np.cumsum(np.diff(time)*(values[1:]+values[:-1])/2)]
    summary = copy.deepcopy(second.summary)
    for key in ('restart_source', 'restart_source_sha256'): summary.pop(key)
    summary.update(start_time_s=0., duration_s=3., requested_duration_s=3.,
        scope='flow5 공력으로 계산한 0–3초 전개 구간. 2.21494초의 저장 상태에서 재개한 두 계산을 상태 수정 없이 연결했습니다. 회수·포획 검증은 포함하지 않습니다.',
        runtime_s=sum(r.summary['runtime_s'] for r in (first, second)),
        clearance_checks=sum(r.summary['clearance_checks'] for r in (first, second)),
        minimum_checked_mesh_gap_m=min(r.summary['minimum_checked_mesh_gap_m'] for r in (first, second)),
        max_tension_N=float(frame.tension_N.max()), max_contact_N=float(frame.contact_N.max()),
        max_penetration_m=float(frame.penetration_m.max()), max_capture_N=float(frame.capture_N.max()),
        contact_impulse_Ns=float(frame.contact_impulse_Ns.iloc[-1]), capture_impulse_Ns=float(frame.capture_impulse_Ns.iloc[-1]),
        max_pitch_change_deg=float(abs(frame.pitch_deg-frame.pitch_deg.iloc[0]).max()),
        source_segments=[dict(path=str(p), status=r.summary['status'],
            start_time_s=float(r.time[0]), end_time_s=float(r.time[-1]),
            sha256={name:hashlib.sha256((p/name).read_bytes()).hexdigest() for name in ('states.npz','inputs.json','summary.json')})
            for p,r in zip(paths,(first,second))],
        restart_state_continuous=True, segment_fingerprints=baseline)
    summary['violations'] = []
    for column,section,key in [('tension_N','cable','limit_N'),('torque_Nm','winch','limit_torque_Nm'),('winch_power_W','winch','limit_power_W'),('contact_N','bay','limit_contact_N'),('penetration_m','bay','limit_penetration_m')]:
        limit = first.config[section].get(key)
        peak = float(frame[column].abs().max() if column == 'winch_power_W' else frame[column].max())
        if limit is not None and peak > limit:
            summary['violations'].append(dict(quantity=column, limit=limit, peak=peak))
    events = first.events + [dict(time_s=float(second.time[0]), event='checkpoint_resume')] + second.events
    result = SimulationResult(time, states, frame, summary, events, first.config)
    output = RUN/'flow5_deployment/combined'
    output.mkdir(exist_ok=False)
    result.save(output)
    history_figure(result).write_html(output/'history.html', include_plotlyjs=True)
    results = json.loads((RUN/'connection_results.json').read_text(encoding='utf8'))
    results['flow5_deployment_combined'] = summary
    (RUN/'connection_results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps({k:summary[k] for k in ('status','end_time_s','max_tension_N','max_contact_N','max_pitch_change_deg','restart_state_continuous')}))


if __name__ == '__main__': main()
