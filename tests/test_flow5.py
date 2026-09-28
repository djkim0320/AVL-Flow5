from pathlib import Path
import numpy as np
import pytest
from dbf_stability import AeroDatabase
from dbf_stability.flow5 import parse_operating_point, to_body_coefficients

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'test_models/H1_reference/runs/flow5_connection_01'


def test_registered_geometry_export_rejects_unhandled_hinge_axes(tmp_path):
    from dbf_stability import load_case
    from dbf_stability.flow5 import read_lifting_surfaces

    cfg = load_case(ROOT / 'examples/h1_recovery_funnel_v2.yaml')
    path = ROOT / cfg['aero']['geometry']
    refs, surfaces = read_lifting_surfaces(path)
    assert len(surfaces) == 3 and refs[0] > 0
    text = path.read_text().replace('aileron 1 .75 0 1 0 -1', 'aileron 1 .75 1 0 0 -1')
    changed = tmp_path / 'changed.avl'
    changed.write_text(text)
    with pytest.raises(ValueError, match='span-axis'):
        read_lifting_surfaces(changed)


def test_native_output_is_required_and_never_filled_with_zero(tmp_path):
    path = tmp_path / 'incomplete.csv'
    path.write_text('CL CX CY\n.2 .01 0\n')
    with pytest.raises(ValueError, match='Incomplete'):
        parse_operating_point(path)


def test_flow5_true_wind_moments_and_body_side_force_conversion():
    # Prescribe a known body-axis force/moment, project into native conventions,
    # and check an oblique wind condition including unequal b and c scales.
    a, b = np.deg2rad([20, 15])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    wd = np.array([ca * cb, -sb, sa * cb])
    ws = np.array([ca * sb, cb, sa * sb])
    wn = np.cross(wd, ws)
    force = np.array([0.03, -0.04, 1.2])
    moment = np.array([0.2, -0.1, -0.3])
    refs = [2, 0.4, 3]
    d = {
        'α': 20,
        'β': 15,
        'CL': force @ wn,
        'CY': force[1],
        'CD_inviscid': force @ wd,
        'CD_viscous': 0,
        'Cl': moment @ wd / 3,
        'Cm_inviscid': moment @ ws / 0.4,
        'Cm_viscous': 0,
        'Cn_inviscid': moment @ wn / 3,
        'Cn_viscous': 0,
    }
    assert to_body_coefficients(d, refs) == pytest.approx(
        np.r_[force * [-1, 1, -1], moment * [-1, 1, -1] / [3, 0.4, 3]]
    )


def test_real_flow5_database_controls_and_rate_signs():
    path = RUN / 'aerodynamics/aero_database.npz'
    if not path.exists():
        pytest.skip('Build with actual flow5 first')
    db = AeroDatabase(path)
    assert 'flow5' in db.metadata['solver']
    assert db.metadata['actual_solver_points'] == 270
    for alpha in [-2, 0, 2]:
        neutral = db.evaluate(np.deg2rad(alpha), 0, 0, np.zeros(3), 20)
        nose_down = db.evaluate(np.deg2rad(alpha), 0, 8, np.zeros(3), 20)
        assert nose_down[4] < neutral[4]
        assert nose_down[2] < neutral[2]
    controls = db.controls([[0, 0, 0]])[0]
    assert controls[3, 0] < 0 and controls[1, 1] < 0 and controls[5, 1] > 0
    rates = db.rates([[0, 0, 0]])[0]
    assert rates[3, 0] < 0 and rates[4, 1] < 0 and rates[5, 2] < 0
    raw = Path(db.metadata['raw_directory']) / 'job_005/output/dbf/DBF_R3_lifting_surfaces/polar'
    rows = [parse_operating_point(p) for p in raw.glob('*.csv')]
    level = next(d for d in rows if d['α'] == 0 and d['β'] == 0)
    assert rates[4, 1] == pytest.approx(level['Cmq'])
    assert db.coeff([[0, 0, 0]])[0] == pytest.approx(to_body_coefficients(level, db.refs))
