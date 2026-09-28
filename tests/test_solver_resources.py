import pytest
from dbf_stability.solver_resources import avl_worker_budget


def test_avl_memory_cap_leaves_reserve_and_preserves_requested_ceiling():
    budget = avl_worker_budget(12, 27, 7 * 1024**3)
    assert budget['effective_workers'] == 3
    assert avl_worker_budget(2, 27, 32 * 1024**3)['effective_workers'] == 2
    assert avl_worker_budget(12, 1, 32 * 1024**3)['effective_workers'] == 1


def test_insufficient_memory_blocks_instead_of_starting_doomed_solvers():
    with pytest.raises(MemoryError):
        avl_worker_budget(12, 27, 2 * 1024**3)
    with pytest.raises(ValueError):
        avl_worker_budget(0, 27, 32 * 1024**3)
