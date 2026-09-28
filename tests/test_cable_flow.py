import numpy as np
from dbf_stability.cable_flow import nodal_exposure, outside_fraction, drag
from dbf_stability.math3d import rotation, quaternion


def test_straight_span_exposure_is_exact_and_direction_independent():
    a = np.array([-2.0, -2.0, 1.0, -1.0, 0.0])
    b = np.array([-1.0, 2.0, 2.0, 1.0, 0.0])
    expected = [1.0, 0.5, 0.0, 0.5, 0.0]
    np.testing.assert_allclose(outside_fraction(a, b, 0.0), expected)
    np.testing.assert_allclose(outside_fraction(b, a, 0.0), expected)


def test_nodal_exposure_is_invariant_under_rigid_world_transform():
    chain = np.array([[-2.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    expected = np.array([1.0, 0.5, 0.0])
    shift = np.array([1770.0, 24.0, -100.0])
    r = rotation(quaternion(0.2, -0.1, 0.4))
    np.testing.assert_allclose(nodal_exposure(chain, np.zeros(3), np.eye(3), 0.0), expected)
    np.testing.assert_allclose(nodal_exposure(chain @ r.T + shift, shift, r, 0.0), expected, atol=3e-13)


def test_air_comoving_with_bay_has_no_drag_on_a_comoving_cable():
    tangent = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    np.testing.assert_array_equal(drag(np.zeros((2, 3)), tangent, 1.225, 0.001, 0.15, 1.2, 0.02), 0.0)
    v = np.array([[25.0, 0.0, 0.0], [0.0, 25.0, 0.0]])
    force = drag(v, tangent, 1.225, 0.001, 0.15, 1.2, 0.02)
    np.testing.assert_allclose(force[0], [-0.5 * 1.225 * 0.001 * 0.15 * 0.02 * 625, 0.0, 0.0])
    np.testing.assert_allclose(force[1], [0.0, -0.5 * 1.225 * 0.001 * 0.15 * 1.2 * 625, 0.0])
    assert np.all((v * force).sum(axis=1) < 0)


def test_internal_air_drag_reaction_preserves_total_force_and_moment():
    from pathlib import Path
    import copy, json
    from dbf_stability import load_case, AeroDatabase
    from dbf_stability.model import CoupledModel
    from dbf_stability.math3d import point_state

    root = Path(__file__).resolve().parents[1]
    here = root / 'test_models/H1_reference'
    c = load_case(here / 'runs/recovery_repair_11/analytic_80/case.yaml')
    explicit_modeling_manifest(c, 'h1_round_x_capture_stop_v2.yaml')
    source = json.loads((root / 'outputs/validation/captured_cost.json').read_text())['source']
    with np.load(source, allow_pickle=False) as d:
        t = float(d['time'][-1])
        y = d['states'][-1].copy()
        meta = json.loads(str(d['metadata']))
    aero = AeroDatabase(here / 'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    models = []
    derivatives = []
    for enabled in (False, True):
        cfg = copy.deepcopy(c)
        cfg['flight']['cable_bay_shielding'] = enabled
        cfg['flight']['internal_cable_flow_factor'] = 0.0
        m = CoupledModel(cfg, aero, meta['trim'])
        m.captured = True
        m.capture_time = meta['capture_time_s']
        m.active_override = meta['active_nodes']
        models.append(m)
        derivatives.append(m.rhs(t, y))
    try:
        m = models[1]
        a, s = y[:13], y[13:26]
        r = rotation(a[6:10])
        rs = rotation(s[6:10])
        k = m.active_override
        nodes = y[26:].reshape(m.n, 6)
        nose, _ = point_state(s, m.sensor_attach)
        anchor, _ = point_state(a, m.attach)
        chain = np.vstack([nose, nodes[:k, :3], anchor])
        exposure = nodal_exposure(chain, a[:3], r, c['bay']['exit_x_m'])
        np.testing.assert_array_equal(exposure, 0.0)
        tangent = chain[2:] - chain[:-2]
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        cable = c['cable']
        old = drag(
            nodes[:k, 3:] - m.wind(t),
            tangent,
            c['flight']['rho_kg_m3'],
            cable['diameter_m'],
            m.h,
            cable['cd_normal'],
            cable['cd_tangent'],
        )
        df = derivatives[1] - derivatives[0]
        nodal_force = df[26:].reshape(m.n, 6)[:, 3:] * m.mn
        total = m.ma * df[3:6] + m.ms * df[16:19] + nodal_force.sum(axis=0)
        np.testing.assert_allclose(total, -old.sum(axis=0), atol=1e-10)
        moment = (
            r @ m.Ia @ df[10:13]
            + rs @ m.Is @ df[23:26]
            + np.cross(s[:3] - a[:3], m.ms * df[16:19])
            + np.cross(nodes[:, :3] - a[:3], nodal_force).sum(axis=0)
        )
        np.testing.assert_allclose(moment, -np.cross(nodes[:k, :3] - a[:3], old).sum(axis=0), atol=1e-10)
    finally:
        for m in models:
            m._mesh_contacts.close()


from tests.conftest import explicit_modeling_manifest
