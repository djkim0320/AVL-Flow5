"""Repair a command-builder failure after a completed, captured settling phase.

Not a retry of a physical failure. The finalized checkpoint, archived solver,
geometry, aero and complete mechanical state are verified before any command is
changed. The original approach and settling results are retained verbatim.
"""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
from pathlib import Path
import argparse
import copy
import hashlib
import json
import shutil
import subprocess
import sys
import numpy as np
import yaml
from compute_resources import apply_affinity

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    apply_affinity()
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    source, out = args.source.resolve(), args.out.resolve()
    pipeline = json.loads((source/'pipeline.json').read_text(encoding='utf8'))
    if pipeline != dict(stage='failed', message='Not enough line for the requested acceleration and braking'):
        raise ValueError('Only the diagnosed short-line command-builder failure is accepted')
    out.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source/'solver', out/'solver', ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    sys.path.insert(0, str(out/'solver'))
    from dbf_stability import AeroDatabase, simulate
    from dbf_stability.analysis import load_result
    from dbf_stability.plots import history_figure
    from continue_underbody_mission import read_checkpoint
    from resume_recovery_case import join
    from run_captured_cleanup import cleanup_schedule
    digest = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    original = json.loads((source/'source.json').read_text(encoding='utf8'))
    if original['solver_sha256'] != {p.name:digest(p) for p in (out/'solver/dbf_stability').glob('*.py')}:
        raise ValueError('Archived force model changed')
    aero = AeroDatabase(original['aero_path'])
    if digest(aero.path) != original['aero_sha256']:
        raise ValueError('Aerodynamic database changed')
    approach, settled = load_result(source/'mission'), load_result(source/'settling')
    if approach.summary['status'] != 'capture_event' or settled.summary['status'] != 'completed' or not settled.summary['captured']:
        raise ValueError('A successful capture and completed settling interval are required')
    config = copy.deepcopy(settled.config)
    checkpoint = source/'settling/accepted_checkpoint.npz'  # finalized phase only
    start, state, meta = read_checkpoint(checkpoint, config, aero, settled)
    combined = join(approach, settled, [source/'mission', source/'settling'])
    end, old = cleanup_schedule(config, start)
    if config['mission_profile']['post_capture_cleanup_mode'] != 'retained_final_recovery':
        raise ValueError('Expected the verified original final braking history')
    command = dict(parent=str(source), checkpoint_sha256=digest(checkpoint), start_s=start,
                   initial_state_sha256=hashlib.sha256(state.tobytes()).hexdigest(),
                   old_winch=old, new_winch=config['winch'],
                   builder_sha256=digest(HERE/'run_captured_cleanup.py'),
                   explanation='Original final line recovery retained exactly; door closes after full reeling and hold. No mechanical state or force model changed.')
    (out/'command_change_source.json').write_text(json.dumps(command, indent=2), encoding='utf8')
    (out/'source.json').write_text(json.dumps(original, indent=2), encoding='utf8')
    shutil.copy2(source/'case.yaml', out/'case.yaml')
    (out/'cleanup_case.yaml').write_text(yaml.safe_dump({k:v for k,v in config.items() if not k.startswith('_')}, sort_keys=False, allow_unicode=True), encoding='utf8')
    shutil.copy2(source/'progress.html', out/'progress.html')
    for filename, value in [('continuation.json', {'progress_href':None}), ('completion_report.json', {'report_href':None})]:
        (out/filename).write_text(json.dumps(value), encoding='utf8')
    def record(stage, **values):
        row = dict(stage=stage, **values)
        temporary=out/'pipeline.tmp'
        temporary.write_text(json.dumps(row, ensure_ascii=False, indent=2), encoding='utf8')
        temporary.replace(out/'pipeline.json')
        print(json.dumps(row), flush=True)
    try:
        record('cleanup', live_directory='cleanup', capture_time_s=meta['capture_time_s'])
        (source/'continuation.json').write_text(json.dumps({'progress_href':Path(os.path.relpath(out/'progress.html', source)).as_posix()}), encoding='utf8')
        result = simulate(config, aero, meta['trim'], phase='mission', initial_state=state,
                          start_time=start, duration=end-start, output=out/'cleanup', initial_capture_time=meta['capture_time_s'])
        np.testing.assert_array_equal(result.states[0], state)
        combined = join(combined, result, [source/'mission', source/'settling', out/'cleanup'])
        scope=('새 수납 트림의 0초부터 계산한 전체 임무.' if combined.time[0]==0 else
               f'{combined.time[0]:.3f}초의 실제 저장 회수 자세에 수정 형상을 적용한 회수 구간 시험. 새 형상의 전개 과정은 포함하지 않습니다.')
        combined.summary.update(source=original, command_change_source=command,
            requested_duration_s=end-float(combined.time[0]),
            scope=scope+' 실제 포획과 안정화 후 남은 원래 감속 명령을 유지하고 문 닫힘까지 이어 계산했습니다. 모든 연결 지점의 기계 상태는 정확히 같습니다.',
            door_kinematics='Prescribed hinge; closed-pose mass properties. Door inertia variation, actuator dynamics and local door drag unresolved.')
        combined.save(out/'complete')
        history_figure(combined).write_html(out/'complete/history.html', include_plotlyjs=True)
        record('audit', result_directory='complete', status=combined.summary['status'])
        subprocess.run([sys.executable, str(HERE/'audit_trajectory_collisions.py'), str(out/'complete'), '--time-step', '.2'], cwd=ROOT, check=True)
        subprocess.run([sys.executable, str(HERE/'visualize_run.py'), '--run', str(out/'complete')], cwd=ROOT, check=True)
        audit=json.loads((out/'complete/cad_collision_audit.json').read_text(encoding='utf8'))
        passed=(combined.summary['status']=='completed' and combined.summary['captured']
                and abs(combined.summary['final_door_deg'])<1e-6
                and not audit['intersecting_samples'] and not audit['cad_query_error_samples'])
        if passed:
            subprocess.run([sys.executable, str(HERE/'verify_underbody_delivery.py'), str(out)], cwd=ROOT, check=True)
        record('completed' if passed else 'partial', result_directory='complete', status=combined.summary['status'],
               captured=combined.summary['captured'], final_door_deg=combined.summary['final_door_deg'],
               end_time_s=float(combined.time[-1]), message=combined.summary['message'])
    except Exception as exc:
        record('failed', message=str(exc))
        raise


if __name__ == '__main__':
    main()
