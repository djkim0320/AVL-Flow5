"""Compare actual H1 baseline/refined trajectories and their independent audits."""
import argparse
import json
from pathlib import Path
import numpy as np
from dbf_stability.analysis import load_result


def compare(root):
    results={name:load_result(root/name/'mission') for name in ('baseline','time_refined')}
    metrics={}
    for key in ('max_tension_N','max_pitch_change_deg','max_contact_N','contact_impulse_Ns'):
        a,b=(r.summary[key] for r in results.values())
        change=abs(a-b)/max(abs(a),abs(b),1e-12)*100
        metrics[key]=dict(baseline=a,time_refined=b,change_percent=change,within_5_percent=bool(change<=5))
    audits={}
    for name,result in results.items():
        p=root/name/'mission/cad_collision_audit.json'
        if not p.exists():
            raise ValueError(f'Independent CAD audit is required: {p}')
        audit=json.loads(p.read_text(encoding='utf8'))
        audits[name]={k:audit[k] for k in ('samples','intersecting_samples','cad_query_error_samples','geometry_validity')}
        audits[name]['covers_all_saved_states']=audit['samples']==len(result.time) and audit['stride']==1
    a,b=results.values()
    times=np.linspace(max(a.time[0],b.time[0]),min(a.time[-1],b.time[-1]),1401)
    differences={}
    for key in ('tension_N','pitch_deg','sensor_local_x_m','sensor_local_z_m'):
        differences[key]=float(np.max(np.abs(np.interp(times,a.time,a.table[key])-np.interp(times,b.time,b.table[key]))))
    report=dict(
        requested_duration_s=7.,
        runs={name:{k:r.summary[k] for k in ('status','duration_s','captured','minimum_checked_mesh_gap_m','runtime_s','clearance_checks')} for name,r in results.items()},
        all_requested_time_completed=all(r.summary['status']=='completed' and r.time[-1]>=7-1e-9 for r in results.values()),
        matching_termination_status=a.summary['status']==b.summary['status'],
        termination_time_difference_s=float(abs(a.time[-1]-b.time[-1])),
        geometry_audits=audits,
        all_saved_states_without_overlap=all(x['geometry_validity']=='no_overlap_at_checked_samples' and x['covers_all_saved_states'] for x in audits.values()),
        sampled_load_and_attitude_comparison=metrics,
        all_comparison_metrics_within_5_percent=all(x['within_5_percent'] for x in metrics.values()),
        maximum_interpolated_history_difference=differences,
        numerically_converged=False,
        limitations=['Time-step comparison only; cable 10/20/40 mesh convergence not established.',
                     'Peak loads and impulse use saved time samples; narrower impact peaks require finer output sampling.',
                     'Independent CAD audit covers saved times, not continuous motion.',
                     'Temporary geometry and uncalibrated near-rigid contact; no experimental validation.'])
    (root/'verification.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    print(json.dumps(compare(parser.parse_args().root),indent=2))
