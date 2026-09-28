"""Accepted-state checkpoints, strict restarts and integration budgets."""

from pathlib import Path
import copy
import hashlib
import json
import time
import numpy as np


class IntegrationStop(RuntimeError):
    def __init__(self, status, message):
        self.status = status
        super().__init__(message)


def fingerprints(config, aero):
    c = copy.deepcopy(config)
    for key in ('_root', '_source'):
        c.pop(key, None)
    # Runtime budgets and output frequency may be adjusted at restart.
    c.pop('simulation', None)
    digest = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
    meshes = {}
    if config.get('collision', {}).get('enabled'):
        meshes = {
            p.name: digest(p)
            for p in sorted((Path(config['_root']) / config['collision']['mesh_directory']).glob('*.stl'))
        }
    return dict(
        config=hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest(),
        aero=digest(aero.path),
        meshes=meshes,
        source={p.name: digest(p) for p in sorted(Path(__file__).parent.glob('*.py'))},
    )


def atomic_npz(path, **values):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    with temporary.open('wb') as f:
        np.savez_compressed(f, **values)
    temporary.replace(path)


class AcceptedProgress:
    def __init__(self, model, output, start, end):
        self.model = model
        self.output = Path(output) if output else None
        self.started = self.window = time.perf_counter()
        self.last_write = self.started
        self.time = self.window_time = float(start)
        self.end = end
        self.evaluations = 0
        self.config = model.c['simulation']
        self.rows = []
        self.chunk = 0
        self.identity = fingerprints(model.c, model.aero) if output else None

    def check(self):
        self.evaluations += 1
        now = time.perf_counter()
        limit = self.config.get('maximum_runtime_s')
        if limit and now - self.started >= limit:
            raise IntegrationStop('runtime_limit', f'Runtime budget {limit:g} s reached; accepted state retained.')
        limit = self.config.get('maximum_rhs_evaluations')
        if limit and self.evaluations > limit:
            raise IntegrationStop('evaluation_limit', f'RHS budget {limit:g} reached; accepted state retained.')
        window = self.config.get('stagnation_window_s')
        if window and now - self.window >= window:
            advance = self.time - self.window_time
            if advance < self.config.get('stagnation_min_advance_s', 0.001):
                raise IntegrationStop(
                    'numerical_stagnation',
                    f'Accepted time advanced {advance:g} s in {now - self.window:g} wall seconds.',
                )
            self.window = now
            self.window_time = self.time

    def accept(self, t, state, dense=None, left=None):
        self.time = float(t)
        if not self.output:
            return
        # Preserve the actual accepted interpolant midpoint, never Newton trials.
        if dense is not None and left is not None and t > left:
            mid = (left + t) / 2
            self.rows.append((mid, dense(mid).copy(), self.model.captured))
        self.rows.append((float(t), state.copy(), self.model.captured))
        if time.perf_counter() - self.last_write >= self.config.get('checkpoint_interval_s', 10.0):
            self.flush()

    def flush(self):
        if not self.output or not self.rows:
            return
        directory = self.output / 'accepted_history'
        directory.mkdir(exist_ok=True)
        metadata = dict(
            format=1,
            phase=self.model.phase,
            captured=self.model.captured,
            capture_time_s=self.model.capture_time,
            active_nodes=self.model.active_override,
            fixed_length_m=self.model.fixed_length,
            trim={k: v for k, v in self.model.trim.items() if k != 'state'},
            fingerprints=self.identity,
        )
        t, y, _ = self.rows[-1]
        # A chunk is durable before the checkpoint references its end.
        atomic_npz(
            directory / f'part_{self.chunk:06d}.npz',
            time=np.array([r[0] for r in self.rows]),
            states=np.array([r[1] for r in self.rows]),
            captured=np.array([r[2] for r in self.rows]),
            metadata=json.dumps(metadata),
        )
        atomic_npz(self.output / 'accepted_checkpoint.npz', time=t, state=y, metadata=json.dumps(metadata))
        (self.output / 'accepted_progress.json').write_text(
            json.dumps(
                dict(
                    time_s=t,
                    requested_end_s=self.end,
                    runtime_s=time.perf_counter() - self.started,
                    rhs_evaluations=self.evaluations,
                    captured=self.model.captured,
                    capture_time_s=self.model.capture_time,
                    chunks=self.chunk + 1,
                )
            ),
            encoding='utf8',
        )
        self.chunk += 1
        self.rows = []
        self.last_write = time.perf_counter()
