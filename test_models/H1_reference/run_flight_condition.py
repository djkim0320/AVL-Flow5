"""Rebuild real flow5 aerodynamics and run a separate flight-condition experiment."""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys
import numpy as np
from dbf_stability import load_case, build_aero_database, solve_trim, simulate
from dbf_stability.analysis import load_result
from dbf_stability.plots import history_figure
from render_flight_condition import render

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def record(out, stage, **details):
    path = out/'pipeline.json'
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(dict(stage=stage, **details), ensure_ascii=False, indent=2), encoding='utf8')
    temp.replace(path)
    print(json.dumps(dict(stage=stage, **details), ensure_ascii=False), flush=True)


def execute(script, *args):
    subprocess.run([sys.executable, str(HERE/script), *map(str, args)], cwd=ROOT, check=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--case', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--workers', type=int, default=4)
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    c = load_case(args.case)
    (out/'inputs.json').write_text(json.dumps(c, ensure_ascii=False, indent=2), encoding='utf8')
    (out/'source_versions.json').write_text(json.dumps({str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
        for f in (ROOT/'src/dbf_stability').glob('*.py')}, indent=2), encoding='utf8')
    try:
        record(out, 'aerodynamics', speed_m_s=c['flight']['speed_m_s'], altitude_m=c['flight']['altitude_m'])
        render(out)
        a = build_aero_database(c, out/'aerodynamics/aero_database.npz', workers=args.workers)
        record(out, 'trim', solver=a.metadata['solver'], actual_solver_points=a.metadata['actual_solver_points'])
        trim = solve_trim(c, a, mode='stowed')
        if not np.isfinite(trim['residual_norm']) or trim['residual_norm'] > 1e-5:
            raise RuntimeError('New-speed trim did not converge')
        (out/'trim.json').write_text(json.dumps({k:v for k,v in trim.items() if k!='state'}, indent=2), encoding='utf8')
        record(out, 'simulation', trim={k:trim[k] for k in ('alpha_deg','elevator_deg','thrust_N','residual_norm')})
        render(out)
        r = simulate(c, a, trim, phase='mission', output=out/'mission')
        profile = c['mission_profile']
        held = max(0., min(r.time[-1], profile['recovery_start_s'])-profile['deployment_complete_s'])
        r.summary.update(observed_command_hold_s=float(held), hold_60s_completed=bool(held>=profile['hold_duration_s']-1e-8),
            recovery_started=bool(r.time[-1]>profile['recovery_start_s']),
            retrieval_command_completed=bool(r.time[-1]>=profile['retrieval_complete_s']),
            scope=f"시작 고도 {c['flight']['altitude_m']:g}m, 속도 {c['flight']['speed_m_s']:g}m/s. 새 속도의 수납 트림에서 시작해 조종면·추력을 유지한 계산입니다.")
        r.save(out/'mission')
        history_figure(r).write_html(out/'mission/history.html', include_plotlyjs=True)
        record(out, 'cad_audit', calculation_status=r.summary['status'], end_time_s=float(r.time[-1]))
        render(out)
        execute('audit_trajectory_collisions.py', out/'mission', '--workers', max(args.workers, 6), '--time-step', '.1')
        record(out, 'replay', calculation_status=r.summary['status'])
        execute('visualize_run.py', '--run', out/'mission')
        checked = load_result(out/'mission')
        baseline = load_result(HERE/'runs/span_hold_01/combined')
        rows = []
        for label, result in [('기존 조건', baseline), ('고도·속도 증가', checked)]:
            rows.append(dict(label=label, start_altitude_m=result.config['flight']['altitude_m'],
                start_speed_m_s=result.config['flight']['speed_m_s'], status=result.summary['status'],
                end_time_s=float(result.time[-1]), final_altitude_m=float(result.table.altitude_m.iloc[-1]),
                final_sensor_altitude_m=float(result.table.sensor_altitude_m.iloc[-1]),
                min_altitude_m=float(result.table.altitude_m.min()),
                max_tension_N=result.summary['max_tension_N'], max_pitch_change_deg=result.summary['max_pitch_change_deg'],
                observed_command_hold_s=result.summary['observed_command_hold_s'],
                recovery_started=result.summary['recovery_started'], captured=result.summary['captured'],
                geometry_validity=result.summary['geometry_validity']))
        (out/'comparison.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf8')
        record(out, 'completed', calculation_status=checked.summary['status'], end_time_s=float(checked.time[-1]),
            captured=checked.summary['captured'], geometry_validity=checked.summary['geometry_validity'])
        render(out)
        latest = dict(path=str(out), mission_path=str(out/'mission'), case=str(args.case.resolve()), report=str(out/'report.html'),
            normal_mission_complete=bool(checked.summary['status']=='completed' and checked.summary['captured']), numerically_converged=False)
        (HERE/'latest_run.json').write_text(json.dumps(latest, ensure_ascii=False, indent=2), encoding='utf8')
    except Exception as exc:
        record(out, 'failed', message=str(exc))
        render(out)
        raise


if __name__ == '__main__':
    main()
