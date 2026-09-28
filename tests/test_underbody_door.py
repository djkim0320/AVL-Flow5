"""270 degree door reference, mass mapping, and real capture event restart."""

from pathlib import Path
import json, sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from dbf_stability import load_case, AeroDatabase, simulate
from dbf_stability.door import door_box, door_hinge, door_reference_shift, door_rotation_radius
from dbf_stability.collision import read_binary_stl
from dbf_stability.analysis import load_result
from dbf_stability.model import CoupledModel
from dbf_stability.contact_integration import separated_from_entire_door_sweep

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / 'test_models/H1_reference'
sys.path.insert(0, str(HERE))
from run_capture_stop import map_reference


@pytest.mark.parametrize('angle', [0, 45, 90, 140, 180, 225, 270])
def test_extended_leaf_mesh_matches_contact_envelope(angle):
    c = load_case(ROOT / 'examples/h1_underbody_270_v2.yaml')
    b = c['bay']
    v, _ = read_binary_stl(ROOT / c['collision']['mesh_directory'] / 'rear_door_100deg_FRD_m.stl')
    h = door_hinge(b)
    posed = Rotation.from_euler('y', angle - 100, degrees=True).apply(v + door_reference_shift(b) - h) + h
    center, axes, half = door_box(b, angle)
    local = (posed - center) @ axes
    np.testing.assert_allclose(local.min(0), -half, atol=1.1e-7)
    np.testing.assert_allclose(local.max(0), half, atol=1.1e-7)
    assert np.linalg.norm((posed - h)[:, [0, 2]], axis=1).max() < door_rotation_radius(b) + 1.1e-7
    if angle == 0:
        assert abs(posed[:, 2].min() - b['ceiling_z_m']) < 1.1e-7
    if angle == 270:
        assert np.ptp(posed[:, 2]) == pytest.approx(0.002, abs=1.1e-7)


def test_added_hardware_and_cg_preserve_all_unchanged_physical_shapes():
    old = load_case(ROOT / 'examples/h1_round_x_capture_stop_v3_shielded.yaml')
    new = load_case(ROOT / 'examples/h1_underbody_270_v2.yaml')
    folder = ROOT / new['collision']['mesh_directory']
    info = json.loads((folder.parent / 'geometry.json').read_text(encoding='utf8'))
    shift = np.asarray(info['aircraft_cg_shift_m'])
    assert new['aircraft']['mass_kg'] - old['aircraft']['mass_kg'] == pytest.approx(
        info['new_door_mass_kg'] - info['old_door_mass_kg'] + 0.01
    )
    assert np.linalg.eigvalsh(new['aircraft']['inertia_kgm2']).min() > 0
    for path in (ROOT / old['collision']['mesh_directory']).glob('*_FRD_m.stl'):
        if path.stem.startswith('rear_door'):
            continue
        v, f = read_binary_stl(path)
        v2, f2 = read_binary_stl(folder / path.name)
        np.testing.assert_allclose(v[f] - shift, v2[f2], atol=1.3e-7, rtol=0)
    assert new['mission_profile']['deployed_length_m'] == 2.7
    assert new['mission_profile']['hold_duration_s'] == 60


def test_capture_event_preserves_real_state_and_radial_sweep_certificate(tmp_path):
    source = load_result(HERE / 'runs/recovery_repair_16/captured_cleanup/complete')
    c = load_case(ROOT / 'examples/h1_underbody_270_v2.yaml')
    y, _ = map_reference(source.states[-1], source.config, c)
    a = AeroDatabase(HERE / 'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    trim = json.loads((HERE / 'runs/recovery_repair_13/shielded_22/trim.json').read_text(encoding='utf8'))
    t = float(source.time[-1])
    model = CoupledModel(c, a, trim)
    try:
        assert model.capture_metric(t, y) < 0
        assert model.clearance_metric(t, y) > 0
        assert separated_from_entire_door_sweep(model, t, y, 0.002)
        # Moving a sensor into the hinge cylinder must revoke the certificate.
        bad = y.copy()
        ra = Rotation.from_quat(y[[7, 8, 9, 6]]).as_matrix()
        bad[13:16] = y[:3] + ra @ door_hinge(c['bay'])
        assert not separated_from_entire_door_sweep(model, t, bad, 0.002)
    finally:
        if model._mesh_contacts is not None:
            model._mesh_contacts.close()
    result = simulate(
        c, a, trim, initial_state=y, start_time=t, duration=0.001, output=tmp_path / 'event', stop_on_capture=True
    )
    assert result.summary['status'] == 'capture_event'
    assert result.summary['capture_time_s'] == t
    assert result.summary['captured']
    np.testing.assert_array_equal(result.states[0], y)
    np.testing.assert_array_equal(result.states[-1], y)
    # A capture-ready pose below ground must not be reported as successful capture.
    underground = y.copy()
    underground[2] += 101.0
    underground[15] += 101.0
    underground[26:].reshape(-1, 6)[:, 2] += 101.0
    failed = simulate(
        c,
        a,
        trim,
        initial_state=underground,
        start_time=t,
        duration=0.001,
        output=tmp_path / 'invalid_event',
        stop_on_capture=True,
    )
    assert failed.summary['status'] == 'ground_contact'
    assert not failed.summary['captured']
