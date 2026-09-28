"""Reproducible full-cycle mesh/time sensitivity, not a physical accuracy claim."""

from pathlib import Path
import json
import numpy as np
from dbf_stability import *


def summarize(table, out):
    checks = []
    for metric in ['max_tension_N', 'max_pitch_change_deg', 'contact_impulse_Ns', 'max_contact_N', 'max_capture_N']:
        if metric not in table or table['status'].ne('completed').any():
            checks.append({'metric': metric, 'status': 'incomplete'})
            continue
        v = table[metric].to_numpy(float)
        if np.max(abs(v)) < 1e-7:
            checks.append({'metric': metric, 'status': 'not_excited', 'values': v.tolist()})
            continue
        delta = abs(np.diff(v)) / np.maximum(abs(v[1:]), 1e-6)
        checks.append(
            {
                'metric': metric,
                'status': 'pass' if np.max(delta) <= 0.05 else 'fail',
                'values': v.tolist(),
                'relative_changes': delta.tolist(),
            }
        )
    report = {
        'case': 'complete scheduled-control mission',
        'target': 0.05,
        'checks': checks,
        'same_capture_outcome': len(set(table.get('capture_status', []))) == 1,
        'all_numerical_checks_pass': bool(checks and all(x['status'] == 'pass' for x in checks)),
        'physical_validation': False,
    }
    (out / 'convergence.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps(report, indent=2), flush=True)


def main():
    root = Path(__file__).resolve().parents[1]
    c = load_case(root / 'examples/mission_scheduled.yaml')
    a = AeroDatabase(root / 'outputs/aero_database.npz')
    c = changed(c, **{'simulation.sample_dt_s': 0.01})
    variants = [
        {'cable.segments': 10, 'simulation.max_step_s': 0.025},
        {'cable.segments': 20, 'simulation.max_step_s': 0.025},
        {'cable.segments': 40, 'simulation.max_step_s': 0.025},
        {'cable.segments': 40, 'simulation.max_step_s': 0.0125},
    ]
    out = root / 'outputs/mission_convergence'
    table = run_sweep(c, a, variants, out, workers=4, phase='mission')
    summarize(table, out)


if __name__ == '__main__':
    main()
