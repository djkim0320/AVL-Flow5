"""Recompute the right-hand force limit at saved capture events, without changing trajectories."""

from pathlib import Path
import numpy as np
import pandas as pd
from dbf_stability import AeroDatabase, solve_trim
from dbf_stability.analysis import load_result
from dbf_stability.model import CoupledModel


def refresh(directory, aero):
    result = load_result(directory)
    if result.summary.get('event_sample_format', 1) >= 2:
        return result
    trim = solve_trim(
        result.config, aero, mode='stowed' if result.summary['phase'] == 'mission' else result.summary['phase']
    )
    model = CoupledModel(result.config, aero, trim, phase=result.summary['phase'], fixed_length=trim.get('length_m'))
    for event in result.events:
        if event['event'] != 'capture_engaged':
            continue
        i = int(np.argmin(abs(result.time - event['time_s'])))
        if abs(result.time[i] - event['time_s']) > 1e-8:
            raise ValueError('Missing saved capture event state')
        model.captured = True
        model.active_override = int(result.table.active_nodes.iloc[i])
        _, d = model.rhs(result.time[i], result.states[i], True)
        row = result.table.iloc[i].to_dict()
        row.update(d)
        result.table = pd.concat(
            [result.table.iloc[: i + 1], pd.DataFrame([row]), result.table.iloc[i + 1 :]], ignore_index=True
        )
        result.time = np.insert(result.time, i + 1, result.time[i])
        result.states = np.insert(result.states, i + 1, result.states[i], axis=0)
    for name, force in [('contact', 'contact_N'), ('capture', 'capture_N')]:
        values = result.table[force].values
        result.table[name + '_impulse_Ns'] = np.r_[0, np.cumsum(np.diff(result.time) * (values[1:] + values[:-1]) / 2)]
        result.summary[name + '_impulse_Ns'] = float(result.table[name + '_impulse_Ns'].iloc[-1])
    result.summary['max_capture_N'] = float(result.table.capture_N.max())
    result.summary['event_sample_format'] = 2
    result.summary['event_postprocessing'] = (
        'Force jump at capture recomputed from saved continuous state; trajectory unchanged.'
    )
    result.save(directory)
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    aero = AeroDatabase(root / 'outputs/aero_database.npz')
    directories = [root / 'outputs/mission_40', root / 'outputs/mission_scheduled']
    for parent in ['mission_convergence', 'recovery_time_convergence', 'edge_cases']:
        directories.extend(p.parent for p in (root / 'outputs' / parent).glob('*/summary.json'))
    for directory in directories:
        refresh(directory, aero)
    print('Capture right-limit force samples updated; all integrated trajectories preserved.')


if __name__ == '__main__':
    main()
