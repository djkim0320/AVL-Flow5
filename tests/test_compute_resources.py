"""Exercise real Windows process affinity and the shared CAD worker ceiling."""

import ctypes
import multiprocessing as mp
import os
from pathlib import Path
import queue
import sys
import time

import pytest

HERE = Path(__file__).resolve().parents[1] / 'test_models' / 'H1_reference'


def _worker_probe(messages, release):
    sys.path.insert(0, str(HERE))
    from compute_resources import acquire_worker

    acquire_worker()
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = ctypes.c_void_p
    kernel.GetProcessAffinityMask.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_size_t),
    ]
    process_mask, system_mask = ctypes.c_size_t(), ctypes.c_size_t()
    if not kernel.GetProcessAffinityMask(
        kernel.GetCurrentProcess(), ctypes.byref(process_mask), ctypes.byref(system_mask)
    ):
        raise ctypes.WinError(ctypes.get_last_error())
    messages.put(('start', time.perf_counter(), os.getpid(), process_mask.value))
    if not release.wait(40):
        raise TimeoutError('Resource test release was not received')
    messages.put(('end', time.perf_counter(), os.getpid(), process_mask.value))


@pytest.mark.skipif(os.name != 'nt', reason='Windows H1 resource policy')
def test_shared_worker_ceiling_and_reserved_processors():
    sys.path.insert(0, str(HERE))
    from compute_resources import profile

    policy = profile()
    limit = policy['cad_audit_workers']
    expected_mask = sum(1 << i for i in policy['logical_processors'])
    context = mp.get_context('spawn')
    messages, release = context.Queue(), context.Event()
    processes = [context.Process(target=_worker_probe, args=(messages, release)) for _ in range(limit + 2)]
    rows = []
    try:
        for process in processes:
            process.start()
        for _ in range(limit):
            rows.append(messages.get(timeout=30))
        assert all(row[0] == 'start' and row[3] == expected_mask for row in rows)
        # Two extra processes have started but may not enter the CAD work slot.
        with pytest.raises(queue.Empty):
            messages.get(timeout=0.4)
        release.set()
        while len(rows) < 2 * len(processes):
            rows.append(messages.get(timeout=15))
        for process in processes:
            process.join(timeout=10)
            assert process.exitcode == 0
        active = maximum = 0
        for kind, _, _, mask in sorted(rows, key=lambda row: row[1]):
            assert mask == expected_mask
            active += 1 if kind == 'start' else -1
            maximum = max(maximum, active)
        assert maximum == limit and active == 0
    finally:
        release.set()
        for process in processes:
            if process.pid is not None:
                process.join(timeout=2)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2)
        messages.close()
