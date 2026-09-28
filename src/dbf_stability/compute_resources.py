"""Model-independent CPU budget; solver processes inherit the parent affinity."""

import ctypes
import os


def worker_limit(cpu_count=None):
    return min(12, max(1, int(cpu_count or os.cpu_count() or 1) - 4))


def apply_affinity():
    available = os.cpu_count() or 1
    used = list(range(worker_limit(available)))
    if os.name == 'nt':
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.GetCurrentProcess.restype = ctypes.c_void_p
        kernel.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        if not kernel.SetProcessAffinityMask(kernel.GetCurrentProcess(), sum(1 << i for i in used)):
            raise ctypes.WinError(ctypes.get_last_error())
    elif hasattr(os, 'sched_getaffinity'):
        allowed = sorted(os.sched_getaffinity(0))
        used = allowed[: min(len(allowed), len(used))]
        os.sched_setaffinity(0, used)
    return dict(logical_processors=used, reserved_logical_processors=[i for i in range(available) if i not in used])
