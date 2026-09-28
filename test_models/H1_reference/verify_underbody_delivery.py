"""Delivery evidence from completed states and hash-matched actual CAD audits."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
from dbf_stability.analysis import load_result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
    folder=args.run/'complete';r=load_result(folder)
    audit=json.loads((folder/'cad_collision_audit.json').read_text(encoding='utf8'))
    digest=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    assert audit['source_states_sha256']==digest(folder/'states.npz')
    assert audit['source_inputs_sha256']==digest(folder/'inputs.json')
    assert audit['cad_sha256']=={f.name:digest(f) for f in Path(audit['cad_directory']).glob('*.step')}
    assert r.summary['status']=='completed' and r.summary['captured']
    assert abs(r.summary['final_door_deg'])<1e-6
    assert audit['intersecting_samples']==0 and audit['cad_query_error_samples']==0
    assert np.isfinite(r.states).all()
    assert np.ptp(r.table.total_mass_kg)<1e-10
    assert np.all(np.diff(r.time)>=0)
    assert -r.states[:,2].max()>0 and -r.states[:,15].max()>0
    for first,second in zip(r.summary['continuation_parts'][:-1],r.summary['continuation_parts'][1:]):
        a,b=load_result(first['path']),load_result(second['path'])
        assert a.time[-1]==b.time[0]
        np.testing.assert_array_equal(a.states[-1],b.states[0])
    initial_source=r.summary['source']
    assert initial_source['aero_sha256']==digest(initial_source['aero_path'])
    if 'initial_state_sha256' in initial_source:
        assert hashlib.sha256(r.states[0].tobytes()).hexdigest()==initial_source['initial_state_sha256']
    full_mission=float(r.time[0])==0.
    mission_checks={}
    if full_mission:
        profile=r.config['mission_profile'];deployed=profile['deployment_complete_s'];recovery=profile['recovery_start_s']
        length=profile['deployed_length_m']
        assert np.isclose(length,1.5*profile['wingspan_m'],atol=1e-12,rtol=0)
        assert np.isclose(recovery-deployed,60.,atol=1e-12,rtol=0)
        assert np.any(np.isclose(r.time,deployed,atol=1e-10,rtol=0))
        assert np.any(np.isclose(r.time,recovery,atol=1e-10,rtol=0))
        hold=r.table[(r.table.time_s>=deployed)&(r.table.time_s<=recovery)]
        assert len(hold)>2 and np.allclose(hold.length_m,length,atol=1e-12,rtol=0)
        assert not hold.captured.any() and r.summary['capture_time_s']>recovery
        assert abs(r.table.door_deg.iloc[0])<1e-12 and np.isclose(r.table.door_deg.max(),270.)
        assert int(r.table.active_nodes.max())==r.config['cable']['segments']
        mission_checks=dict(fresh_time_zero=True,deployment_complete_s=deployed,
            full_payout_m=length,wire_span_ratio=1.5,full_payout_hold_s=recovery-deployed,
            recovery_start_s=recovery,hold_samples=len(hold),
            hold_altitude_range_m=[float(hold.altitude_m.min()),float(hold.altitude_m.max())],
            hold_speed_range_m_s=[float(hold.airspeed_m_s.min()),float(hold.airspeed_m_s.max())])
    result=dict(status='verified_completed_full_mission' if full_mission else 'verified_completed_window',scope=r.summary['scope'],start_s=float(r.time[0]),end_s=float(r.time[-1]),
        captured=True,capture_time_s=r.summary['capture_time_s'],final_door_deg=float(r.table.door_deg.iloc[-1]),
        final_altitude_m=float(r.table.altitude_m.iloc[-1]),final_airspeed_m_s=float(r.table.airspeed_m_s.iloc[-1]),
        max_tension_N=float(r.table.tension_N.max()),max_contact_N=float(r.table.contact_N.max()),
        max_winch_torque_Nm=float(r.table.torque_Nm.abs().max()),max_winch_power_W=float(r.table.winch_power_W.abs().max()),
        configured_limit_violations=r.summary['violations'],total_mass_kg=float(r.table.total_mass_kg.iloc[0]),
        saved_states=len(r.time),CAD_checked_poses=audit['samples'],CAD_overlap_poses=0,CAD_query_error_poses=0,
        exact_state_continuity=True,mission_checks=mission_checks,
        altitude_range_m=[float(r.table.altitude_m.min()),float(r.table.altitude_m.max())],
        airspeed_range_m_s=[float(r.table.airspeed_m_s.min()),float(r.table.airspeed_m_s.max())],
        numerically_converged=r.summary['numerically_converged'],physical_validation=r.summary['physical_validation'],
        sources={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in ('states.npz','inputs.json','cad_collision_audit.json')})
    (folder/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    (folder/'summary.json').write_text(json.dumps(r.summary,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps(result,ensure_ascii=True,indent=2),flush=True)
