"""Force a genuine time refinement in the contact window from the same saved state."""

from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
from threadpoolctl import threadpool_limits
from dbf_stability import AeroDatabase, changed, simulate
from dbf_stability.analysis import load_result


def job(args):
    c, aero_path, state, t0, step, out = args
    c = changed(c, **{'simulation.max_step_s': step, 'simulation.sample_dt_s': 0.001})
    with threadpool_limits(limits=1):
        r = simulate(
            c,
            AeroDatabase(aero_path),
            phase='mission',
            initial_state=state,
            start_time=t0,
            duration=11 - t0,
            output=out,
        )
    return r.summary


def main():
    root = Path(__file__).resolve().parents[1]
    base = load_result(root / 'outputs/mission_40')
    i = int(np.argmin(abs(base.time - 9)))
    t0 = float(base.time[i])
    out = root / 'outputs/recovery_time_convergence'
    jobs = [
        (base.config, str(root / 'outputs/aero_database.npz'), base.states[i], t0, step, str(out / f'case_{j}'))
        for j, step in enumerate([0.003, 0.0015])
    ]
    with ProcessPoolExecutor(max_workers=2) as pool:
        rows = list(pool.map(job, jobs))
    checks = []
    for metric in ['max_tension_N', 'max_pitch_change_deg', 'contact_impulse_Ns', 'max_contact_N', 'max_capture_N']:
        v = [r[metric] for r in rows]
        change = abs(v[1] - v[0]) / max(abs(v[1]), 1e-6)
        checks.append(
            {'metric': metric, 'values': v, 'relative_change': change, 'status': 'pass' if change <= 0.05 else 'fail'}
        )
    report = {
        'scope': '40-cell recovery branch from identical saved state at 9 s; not a full-cycle mesh check',
        'max_step_s': [0.003, 0.0015],
        'output_dt_s': 0.001,
        'checks': checks,
        'statuses': [r['status'] for r in rows],
        'captured': [r['captured'] for r in rows],
    }
    (out / 'convergence.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
