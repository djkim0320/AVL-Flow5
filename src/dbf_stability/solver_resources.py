"""Bound concurrent solver processes without altering numerical inputs."""
import ctypes
import os


def available_memory_bytes():
    if os.name == 'nt':
        class MemoryStatus(ctypes.Structure):
            _fields_ = [('length', ctypes.c_ulong), ('load', ctypes.c_ulong)] + [
                (name, ctypes.c_ulonglong) for name in ('total_phys', 'avail_phys',
                'total_commit', 'avail_commit', 'total_virtual', 'avail_virtual', 'extended')]
        status = MemoryStatus(); status.length = ctypes.sizeof(status)
        if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            raise OSError('Cannot read available solver memory')
        return min(status.avail_phys, status.avail_commit)
    if hasattr(os, 'sysconf'):
        return os.sysconf('SC_AVPHYS_PAGES') * os.sysconf('SC_PAGE_SIZE')
    raise OSError('Available solver memory is unavailable on this platform')


def avl_worker_budget(requested, jobs, available):
    if isinstance(requested, bool) or not isinstance(requested, int) or requested < 1:
        raise ValueError('AVL workers must be a positive integer')
    # Bundled AVL allocates 1.4 GB in its three NVX=5000 double arrays alone.
    # 2 GiB/process and 1 GiB reserve are conservative scheduling estimates,
    # not measured peaks or an assurance against competing memory allocations.
    per_worker, reserve = 2 * 1024**3, 1024**3
    capacity = int(max(0, available - reserve) // per_worker)
    if capacity < 1:
        raise MemoryError('AVL requires at least 3 GiB available memory including reserve; close unused applications and retry')
    effective = min(requested, jobs, capacity)
    return dict(requested_workers=requested, effective_workers=effective,
                available_memory_bytes=int(available), estimated_bytes_per_worker=per_worker,
                reserve_bytes=reserve, policy='AVL memory and condition-count cap')
