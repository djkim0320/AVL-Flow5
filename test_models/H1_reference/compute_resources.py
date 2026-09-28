"""Project CPU affinity and process-wide CAD worker permits on Windows.

File byte locks are released by Windows even if a worker crashes. Pools share
the same permits, so overlapping audits cannot each consume the entire budget.
No solver, case, checkpoint or integration tolerance is modified here.
"""
import ctypes
import json
import os
from pathlib import Path
import time

HERE = Path(__file__).resolve().parent
PROFILE = HERE / 'compute_resources.json'
_permit = None


def profile():
    value = json.loads(PROFILE.read_text(encoding='utf8'))
    available = os.cpu_count() or 1
    used = value['logical_processors']
    reserved = value['reserved_logical_processors']
    if (not used or len(set(used)) != len(used) or set(used) & set(reserved)
            or any(not isinstance(i, int) or i < 0 or i >= available for i in used + reserved)):
        raise ValueError('Invalid CPU resource profile for this machine')
    if not 1 <= value['cad_audit_workers'] <= len(used) - value['integration_processes']:
        raise ValueError('CAD workers exceed the shared resource budget')
    return value


def apply_affinity():
    value = profile()
    if os.name != 'nt':
        raise RuntimeError('This resource profile is for the Windows H1 workstation')
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = ctypes.c_void_p
    kernel.SetProcessAffinityMask.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    mask = sum(1 << i for i in value['logical_processors'])
    if not kernel.SetProcessAffinityMask(kernel.GetCurrentProcess(), mask):
        raise ctypes.WinError(ctypes.get_last_error())
    return value


def available_memory():
    class MemoryStatus(ctypes.Structure):
        _fields_ = [('length', ctypes.c_ulong), ('load', ctypes.c_ulong)] + [
            (name, ctypes.c_ulonglong) for name in
            ('total', 'available', 'total_page', 'available_page', 'total_virtual', 'available_virtual', 'extended')]
    status = MemoryStatus()
    status.length = ctypes.sizeof(status)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    if not kernel.GlobalMemoryStatusEx(ctypes.byref(status)):
        raise ctypes.WinError(ctypes.get_last_error())
    return status.available


def audit_budget(state_bytes, requested_workers):
    value = apply_affinity()
    free = available_memory()
    # Each worker loads the trajectory, transformed arrays and OpenCASCADE.
    # Leave memory for the running integrators; do not force paging for 10 jobs.
    per_worker = 768 * 1024**2 + 4 * state_bytes
    capacity = int((free - value['minimum_free_memory_gib'] * 1024**3) / per_worker)
    if capacity < 1:
        raise MemoryError('Insufficient free RAM for a CAD worker and the integration reserve')
    workers = min(value['cad_audit_workers'], capacity)
    return dict(workers=workers, requested_workers=requested_workers,
                policy='Active project resource profile overrides the legacy --workers setting',
                profile=value, available_memory_bytes=free,
                estimated_bytes_per_worker=per_worker,
                global_cad_worker_limit=value['cad_audit_workers'])


def acquire_worker():
    global _permit
    if _permit is not None:
        return
    import msvcrt
    value = apply_affinity()
    directory = HERE / 'runs' / '_compute_slots'
    directory.mkdir(parents=True, exist_ok=True)
    while True:
        for index in range(value['cad_audit_workers']):
            handle = (directory / f'worker_{index:02d}.lock').open('a+b')
            handle.seek(0, 2)
            if not handle.tell():
                handle.write(b'0'); handle.flush()
            handle.seek(0)
            try:
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError:
                handle.close()
                continue
            _permit = handle
            return
        time.sleep(.1)
