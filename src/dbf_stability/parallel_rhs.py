"""Independent force evaluations for implicit-solver finite differences.

Every spawned process owns its model and Bullet connection. Only numerical
derivative columns run concurrently; integration and events remain ordered.
"""

import atexit
import ctypes
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from threadpoolctl import threadpool_limits

_model = None
_threads = None


def affinity_mask():
    if os.name != 'nt':
        return sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else None
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = ctypes.c_void_p
    kernel.GetProcessAffinityMask.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_size_t),
    ]
    process, system = ctypes.c_size_t(), ctypes.c_size_t()
    if not kernel.GetProcessAffinityMask(kernel.GetCurrentProcess(), ctypes.byref(process), ctypes.byref(system)):
        raise ctypes.WinError(ctypes.get_last_error())
    return process.value


def _initialize(config, aero_path, trim, phase, fixed_length, affinity):
    global _model, _threads
    for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ[key] = '1'

    _threads = threadpool_limits(limits=1)
    if os.name == 'nt':
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.GetCurrentProcess.restype = ctypes.c_void_p
        kernel.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        if not kernel.SetProcessAffinityMask(kernel.GetCurrentProcess(), affinity):
            raise ctypes.WinError(ctypes.get_last_error())
    elif affinity is not None:
        os.sched_setaffinity(0, affinity)
    # Imported here because this runs in the spawned worker process; model.py imports this module.
    from .aero_database import AeroDatabase
    from .model import CoupledModel

    _model = CoupledModel(config, AeroDatabase(aero_path), trim, phase, fixed_length)
    atexit.register(_model.close)


def _evaluate(task):
    t, states, active, captured, capture_time = task
    _model.active_override = active
    _model.captured = captured
    _model.capture_time = capture_time
    return np.column_stack([_model.rhs(t, state) for state in states.T])


class ParallelRHS:
    def __init__(self, model, workers):
        if not isinstance(workers, int) or workers < 2:
            raise ValueError('Parallel RHS needs at least two workers')
        self.model, self.workers = model, workers
        self.pool = ProcessPoolExecutor(
            max_workers=workers,
            mp_context=multiprocessing.get_context('spawn'),
            initializer=_initialize,
            initargs=(model.c, str(model.aero.path), model.trim, model.phase, model.fixed_length, affinity_mask()),
        )

    def __call__(self, t, states):
        model = self.model
        chunks = np.array_split(states, min(states.shape[1], self.workers * 3), axis=1)
        tasks = [(t, block, model.active_override, model.captured, model.capture_time) for block in chunks]
        return np.column_stack(list(self.pool.map(_evaluate, tasks)))

    def close(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
