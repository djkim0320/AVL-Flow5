"""Branch from recorded physical states; no synthetic replacement solver outputs."""

from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import copy
import json
import numpy as np
from threadpoolctl import threadpool_limits
from dbf_stability import *
from dbf_stability.analysis import load_result
from dbf_stability.model import schedule
from dbf_stability.math3d import rotation


def run_one(args):
    name, c, aero_path, state, t0, duration, out = args
    with threadpool_limits(limits=1):
        r = simulate(
            c,
            AeroDatabase(aero_path),
            phase='mission',
            initial_state=state,
            duration=duration,
            start_time=t0,
            output=out,
        )
    return {'scenario': name, **r.summary}


def main():
    root = Path(__file__).resolve().parents[1]
    baseline = load_result(root / 'outputs/mission_convergence/case_000')
    jobs = []
    for name, t0, duration in [
        ('abrupt_payout_stop', 1.8, 0.5),
        ('faster_recovery', 8, 2.8),
        ('exit_misalignment', 8, 2.8),
        ('low_contact_damping', 8, 2.8),
        ('winch_limit', 8, 0.5),
        ('gentle_capture', 8, 4.5),
    ]:
        idx = int(np.argmin(abs(baseline.time - t0)))
        t0 = float(baseline.time[idx])
        state = baseline.states[idx].copy()
        c = copy.deepcopy(baseline.config)
        length = schedule(c['winch']['length_schedule'], t0)[0]
        if name == 'abrupt_payout_stop':
            c['winch']['length_schedule'] = [[0, 0.03], [t0, length], [11, length]]
        elif name == 'faster_recovery':
            c['winch']['length_schedule'] = [
                [0, 0.03],
                [0.6, 0.03],
                [4, 2.6],
                [5, 2.6],
                [t0, length],
                [9, 0.03],
                [11, 0.03],
            ]
        elif name == 'exit_misalignment':
            state[13:16] += rotation(state[6:10]) @ np.array([0, 0.13, 0])
            c['bay']['capture_radius_m'] = 0.02
        elif name == 'low_contact_damping':
            c['bay']['contact_c_Ns_m'] = 0.05
        elif name == 'winch_limit':
            c['winch']['limit_torque_Nm'] = 0.001
        elif name == 'gentle_capture':
            c['winch']['length_schedule'] = [
                [0, 0.03],
                [0.6, 0.03],
                [4, 2.6],
                [5, 2.6],
                [t0, length],
                [9.6, 0.15],
                [10.8, 0.03],
                [12.5, 0.03],
            ]
            c['winch']['door_schedule'] = [[0, 0], [0.5, 100], [11.5, 100], [12, 0], [12.5, 0]]
            c['bay']['capture_radius_m'] = 0.085
            c['provenance']['bay']['source'] += ' Assumed 85 mm capture envelope for gentler recovery variant.'
        c['provenance']['flight']['source'] += f' Branch scenario {name}, initial state from full cycle at {t0}s.'
        jobs.append(
            (
                name,
                c,
                str(root / 'outputs/aero_database.npz'),
                state,
                t0,
                duration,
                str(root / 'outputs/edge_cases' / name),
            )
        )
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(run_one, jobs))
    (root / 'outputs/edge_cases/summary.json').write_text(json.dumps(rows, indent=2), encoding='utf8')
    for r in rows:
        print(r['scenario'], r['status'], r['capture_status'], r['violations'], flush=True)


if __name__ == '__main__':
    main()
