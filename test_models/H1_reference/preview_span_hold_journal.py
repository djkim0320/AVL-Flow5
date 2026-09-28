"""Freeze a preview from immutable accepted-state chunks while integration runs."""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import hashlib
import json
import numpy as np
import pandas as pd
import recover_span_hold_journal as recovery
from dbf_stability.analysis import SimulationResult
from dbf_stability.math3d import euler


def diagnostic(sample):
    t, y, captured, meta = sample
    model = recovery.MODEL
    model.active_override = meta['active_nodes']
    model.captured = bool(captured)
    model.capture_time = meta['capture_time_s'] if captured else None
    _, row = model.rhs(t, y, True)
    for i, name in enumerate(('roll_deg', 'pitch_deg', 'yaw_deg')):
        row[name] = float(np.rad2deg(euler(y[6:10])[i]))
        row['sensor_' + name] = float(np.rad2deg(euler(y[19:23])[i]))
    row.update(time_s=float(t), altitude_m=float(-y[2]), sensor_altitude_m=float(-y[15]))
    return row


def main():
    out = recovery.OUT
    # Freeze filenames first; never open the checkpoint the running solver replaces.
    paths = sorted((out/'mission/accepted_history').glob('part_*.npz'))
    paths += sorted((out/'continuation/accepted_history').glob('part_*.npz'))
    times, states, flags, origins, metadata, sources = [], [], [], [], [], []
    for i, path in enumerate(paths):
        with np.load(path, allow_pickle=False) as data:
            t, y, f = data['time'].copy(), data['states'].copy(), data['captured'].copy()
            meta = json.loads(str(data['metadata']))
        times.append(t); states.append(y); flags.append(f)
        origins.append(np.full(len(t), i)); metadata.append(meta)
        sources.append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    assert all(m['fingerprints'] == metadata[0]['fingerprints'] for m in metadata)
    t, y, f, source = np.concatenate(times), np.vstack(states), np.concatenate(flags), np.concatenate(origins)
    assert np.all(np.diff(t) >= 0)
    duplicate = np.flatnonzero(np.diff(t) == 0)
    assert all(np.array_equal(y[i], y[i+1]) for i in duplicate)
    keep = np.r_[np.diff(t) > 0, True]
    t, y, f, source = t[keep], y[keep], f[keep], source[keep]
    targets = np.r_[np.arange(t[0], t[-1], .025), t[-1]]
    right = np.minimum(np.searchsorted(t, targets), len(t)-1)
    left = np.maximum(right-1, 0)
    selected = np.unique(np.where(abs(t[right]-targets) < abs(t[left]-targets), right, left))
    samples = [(float(t[i]), y[i], bool(f[i]), metadata[source[i]]) for i in selected]
    with ProcessPoolExecutor(max_workers=4, initializer=recovery.init_worker) as pool:
        rows = list(pool.map(diagnostic, samples))
    case = json.loads((out/'mission/inputs.json').read_text(encoding='utf8'))
    base = json.loads((out/'recovered/summary.json').read_text(encoding='utf8'))
    base.update(status='partial_snapshot', message='Saved-state preview; full integration continues separately.',
        journal_source=sources, snapshot_source_rows=len(t), preview_rows=len(selected),
        captured=bool(f[-1]), capture_time_s=metadata[-1]['capture_time_s'],
        scope='계산이 끝난 구간만 재생합니다. 60초 유지와 회수 구간은 아직 포함되지 않았습니다. 아래 수치는 재생용으로 선택한 시각의 값입니다.',
        geometry_validity='not_audited', limitations=['Preview samples only; extrema and impulses are not full-resolution load results.',
            'Full mission remains in progress. No interpolated or prescribed replacement motion.'],
        numerically_converged=False)
    frame = pd.DataFrame(rows)
    summary = recovery.finish_summary(t[selected], frame, base, case)
    events = [dict(time_s=float(t[selected[i]]), event=str(frame.phase.iloc[i]))
              for i in range(len(frame)) if i == 0 or frame.phase.iloc[i] != frame.phase.iloc[i-1]]
    target = out/'preview'
    target.mkdir(exist_ok=False)
    SimulationResult(t[selected], y[selected], frame, summary, events, case).save(target)
    print(json.dumps(dict(end_time_s=float(t[-1]), preview_rows=len(selected), accepted_rows=len(t))), flush=True)


if __name__ == '__main__':
    main()
