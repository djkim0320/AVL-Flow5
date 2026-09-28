"""Added hardware mass and geometry preserve the physical reference mapping."""

from pathlib import Path
import json, sys
import numpy as np
import pytest
from dbf_stability import load_case, AeroDatabase
from dbf_stability.collision import read_binary_stl
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import rotation, quaternion

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / 'test_models/H1_reference'


@pytest.mark.parametrize('version', ['v2', 'v3'])
def test_added_stop_mass_inertia_and_geometry_reference(version):
    old = load_case(ROOT / 'examples/h1_normal_r3_round_x_sensor.yaml')
    new = load_case(ROOT / f'examples/h1_round_x_capture_stop_{version}.yaml')
    folder = (ROOT / new['collision']['mesh_directory']).parent
    record = json.loads((folder / 'geometry.json').read_text(encoding='utf8'))
    assert record['roundtrip_valid'] and record['solids'] == 41
    assert record['stowed_overlap_mm3'] == 0 and record['stowed_CAD_gap_m'] > 0.0008
    delta = np.array(record['aircraft_cg_shift_m'])

    def pa(r):
        return np.eye(3) * (r @ r) - np.outer(r, r)

    m = record['mass_kg']
    old_mass = old['aircraft']['mass_kg']
    p = np.array(record['center_in_old_aircraft_frd_m'])
    assert abs(new['aircraft']['mass_kg'] - old_mass - m) < 1e-12
    np.testing.assert_allclose(delta, m * p / (old_mass + m), atol=1e-15)
    expected = (
        np.array(old['aircraft']['inertia_kgm2'])
        + old_mass * pa(delta)
        + np.array(record['stop_inertia_at_center_kgm2'])
        + m * pa(p - delta)
    )
    np.testing.assert_allclose(new['aircraft']['inertia_kgm2'], expected, atol=1e-15)
    np.testing.assert_allclose(new['aircraft']['cg_m'], np.array(old['aircraft']['cg_m']) + delta, atol=1e-15)
    for path in (ROOT / old['collision']['mesh_directory']).glob('*_FRD_m.stl'):
        v, f = read_binary_stl(path)
        v2, f2 = read_binary_stl(folder / 'meshes' / path.name)
        np.testing.assert_allclose(v[f] - delta, v2[f2], atol=1.3e-7, rtol=0)
    assert 'capture_front_stop' in new['collision']['concave_parts']


@pytest.mark.parametrize('version', ['v2', 'v3'])
def test_mass_reference_mapping_preserves_towpoints_and_aero_moment(version):
    sys.path.insert(0, str(HERE))
    from run_capture_stop import map_reference
    from dbf_stability.analysis import load_result

    source = load_result(HERE / 'runs/recovery_repair_04/round_nominal/mission')
    c = load_case(ROOT / f'examples/h1_round_x_capture_stop_{version}.yaml')
    shifted, changes = map_reference(source.states[-1], source.config, c)
    assert changes['sensor'] == [0.0, 0.0, 0.0]
    np.testing.assert_array_equal(shifted[13:], source.states[-1, 13:])
    a = AeroDatabase(HERE / 'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    old = source.config
    for cfg in (old, c):
        cfg['flight']['controller']['enabled'] = False
    trim = json.loads((HERE / 'runs/recovery_repair_04/round_nominal/trim.json').read_text(encoding='utf8'))
    first, second = CoupledModel(old, a, trim), CoupledModel(c, a, trim)
    y = source.states[-1, :13].copy()
    y[10:13] = 0
    y[6:10] = quaternion(pitch=trim['alpha_rad'])
    y[3:6] = [25, 0, 0]
    f0, m0, _ = first.aircraft_loads(source.time[-1], y)
    y2 = y.copy()
    delta = np.array(changes['aircraft'])
    y2[:3] += rotation(y[6:10]) @ delta
    f1, m1, _ = second.aircraft_loads(source.time[-1], y2)
    np.testing.assert_allclose(f1, f0, atol=1e-12)
    np.testing.assert_allclose(m1, m0 - np.cross(delta, rotation(y[6:10]).T @ f0), atol=1e-10)
