"""Contact continuity and process isolation regressions."""

import json
from pathlib import Path
import numpy as np
import pytest
from dbf_stability.collision import capsule_mesh, barrier_force
from dbf_stability.model import CoupledModel
from dbf_stability import AeroDatabase


def wedge_load(offset):
    # Two real plane patches meeting in an inward corner. The old nearest-only
    # force switches abruptly when a cable crosses their distance bisector.
    vertices = np.array([[-1, -1, 0.1], [-1, 0, 0], [-1, 1, 0.1], [1, -1, 0.1], [1, 0, 0], [1, 1, 0.1]])
    faces = np.array([[0, 1, 3], [1, 4, 3], [1, 2, 4], [2, 5, 4]])
    a = np.array([-0.2, offset, 0.001])
    b = np.array([0.2, offset, 0.001])
    hits = capsule_mesh(a, b, 0.0005, vertices, faces, 0.00016, skin=0.0008)
    force = np.zeros(3)
    for _, normal, gap, gradient, share in hits:
        magnitude, _ = barrier_force(gap, 0.0, [0, 0, 0], 0.0008, 20000.0, 8.0, 0.1)
        force += normal * magnitude * gradient * share
    return force


def test_cable_facet_bisector_force_is_continuous():
    left, right = wedge_load(-1e-10), wedge_load(1e-10)
    assert np.linalg.norm(left - right) < 1e-5
    assert np.linalg.norm(wedge_load(0.0)) > 1.0
    assert abs(wedge_load(0.0)[1]) < 1e-10


def test_planar_contact_and_skin_boundary():
    vertices = np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0.0]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    a = np.array([-0.1, 0, 0.001])
    b = np.array([0.1, 0, 0.001])
    hits = capsule_mesh(a, b, 0.0005, vertices, faces, 0.00016, skin=0.0008)
    assert sum(h[4] for h in hits) == pytest.approx(1.0)
    assert all(h[2] == pytest.approx(0.0005) for h in hits)
    for h in hits:
        np.testing.assert_allclose(h[1], [0, 0, 1], atol=1e-10)
    assert capsule_mesh(a + [0, 0, 0.001], b + [0, 0, 0.001], 0.0005, vertices, faces, 0.00016, skin=0.0008) == []


def test_real_cad_parallel_rhs_and_event_state_match():
    root = Path(__file__).resolve().parents[1]
    run = root / 'test_models/H1_reference/runs/funnel_mission_01'
    if not (run / 'mission/accepted_checkpoint.npz').exists():
        pytest.skip('Actual H1 stopped-state fixture not present')
    config = json.loads((run / 'mission/inputs.json').read_text(encoding='utf8'))
    explicit_modeling_manifest(config, 'h1_recovery_funnel_v2.yaml')
    with np.load(run / 'mission/accepted_checkpoint.npz', allow_pickle=False) as d:
        t = float(d['time'])
        state = d['state'].copy()
        meta = json.loads(str(d['metadata']))
    aero = AeroDatabase(json.loads((run / 'source.json').read_text())['aero_path'])
    config['simulation']['jacobian_workers'] = 2
    m = CoupledModel(config, aero, meta['trim'])
    try:
        for active, captured in [(18, False), (17, False), (18, True)]:
            m.active_override = active
            m.captured = captured
            m.capture_time = t - 0.1 if captured else None
            states = np.column_stack([state, state + np.eye(len(state))[129] * 1e-8])
            serial = np.column_stack([m.rhs(t, z) for z in states.T])
            np.testing.assert_allclose(m.rhs_batch(t, states), serial, rtol=1e-12, atol=1e-10)
        m.active_override = 18
        m.captured = False
        m.capture_time = None
        parallel = m.jacobian()(t, state).toarray()
        config['simulation']['jacobian_workers'] = 1
        serial = m.jacobian()(t, state).toarray()
        np.testing.assert_allclose(parallel, serial, rtol=1e-10, atol=1e-6)
    finally:
        m.close()
    assert m._parallel_rhs is None and m._mesh_contacts is None


from tests.conftest import explicit_modeling_manifest
