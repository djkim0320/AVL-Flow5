"""Feedback sign, bounds and opt-in behavior against the real 25 m/s flow5 run."""

import copy
import json
from pathlib import Path
import numpy as np
import pytest
from dbf_stability import load_case, AeroDatabase
from dbf_stability.model import CoupledModel
from dbf_stability.control import validate_controller, longitudinal_commands

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'test_models/H1_reference/runs/span_hold_100m_25ms_01'


@pytest.fixture(scope='module')
def experiment():
    path = BASE / 'aerodynamics/aero_database.npz'
    if not path.exists():
        pytest.skip('Requires saved actual flow5 25 m/s aerodynamic database')
    cfg = load_case(ROOT / 'examples/h1_normal_r3_flow5_controlled.yaml')
    aero = AeroDatabase(path)
    trim = json.loads((BASE / 'trim.json').read_text())
    return cfg, aero, trim


def models(experiment):
    cfg, aero, trim = experiment
    controlled = CoupledModel(copy.deepcopy(cfg), aero, trim, phase='aircraft_only')
    disabled = copy.deepcopy(cfg)
    disabled['flight'].pop('controller')
    return controlled, CoupledModel(disabled, aero, trim, phase='aircraft_only')


def test_below_altitude_commands_actual_nose_up_moment(experiment):
    controlled, baseline = models(experiment)
    y = controlled.initial()[:13]
    y[2] = -95.0  # five metres below target, otherwise level trim
    _, old_m, old = baseline.aircraft_loads(10.0, y)
    _, new_m, new = controlled.aircraft_loads(10.0, y)
    assert new['elevator_deg'] < old['elevator_deg']
    assert new_m[1] > old_m[1]  # FRD positive My raises nose


def test_pitch_rate_damped_and_slow_speed_increases_thrust(experiment):
    controlled, _ = models(experiment)
    y = controlled.initial()[:13]
    _, m0, d0 = controlled.aircraft_loads(10.0, y)
    y[11] = np.deg2rad(2.0)
    _, m1, d1 = controlled.aircraft_loads(10.0, y)
    assert d1['elevator_deg'] > d0['elevator_deg']
    assert m1[1] < m0[1]
    y[3] -= 2.0
    _, _, slow = controlled.aircraft_loads(10.0, y)
    assert slow['thrust_N'] > d0['thrust_N']


def test_activation_continuous_and_rejected_trials_do_not_change_state(experiment):
    controlled, baseline = models(experiment)
    y = controlled.initial()[:13]
    y[2] = -98.5
    original = y.copy()
    activation = controlled.controller['enable_from_s']
    for t in (0.0, activation):
        f, m, _ = controlled.aircraft_loads(t, y)
        f0, m0, _ = baseline.aircraft_loads(t, y)
        np.testing.assert_array_equal(f, f0)
        np.testing.assert_array_equal(m, m0)
    first = controlled.aircraft_loads(10.0, y)
    controlled.aircraft_loads(50.0, y)
    controlled.aircraft_loads(2.0, y)
    repeat = controlled.aircraft_loads(10.0, y)
    for i in (0, 1):
        np.testing.assert_array_equal(first[i], repeat[i])
    assert first[2] == repeat[2]
    np.testing.assert_array_equal(y, original)


@pytest.mark.parametrize('sign', [-1, 1])
def test_commands_stay_within_hardware_and_aerodynamic_limits(experiment, sign):
    controlled, _ = models(experiment)
    y = controlled.initial()[:13]
    y[2] += sign * 1000
    y[11] = sign * 5.0  # severe rate error forces elevator saturation
    y[3] = 2.0 if sign > 0 else 50.0
    el, thrust, d = longitudinal_commands(
        controlled.controller,
        10.0,
        y,
        controlled.trim,
        controlled.trim['alpha_rad'],
        y[3],
        controlled.trim['elevator_deg'],
        controlled.trim['thrust_N'],
    )
    assert d['elevator_saturated'] and d['thrust_saturated']
    assert -7.5 <= el <= 7.5
    assert 0.0 <= thrust <= 15.0
    # Extreme rate is a controller stress input, outside the small-rate plant.
    # Verify the resulting elevator remains admissible to the actual aero grid.
    assert np.isfinite(controlled.aero.evaluate(controlled.trim['alpha_rad'], 0.0, el, np.zeros(3), 25.0)).all()


@pytest.mark.parametrize(
    'key,value',
    [
        ('engage_ramp_s', 0.0),
        ('pitch_kp', float('nan')),
        ('elevator_limits_deg', [-9.0, 9.0]),
        ('thrust_limits_N', [-1.0, 15.0]),
        ('measurement_model', 'unspecified'),
    ],
)
def test_invalid_control_assumptions_rejected(experiment, key, value):
    cfg = copy.deepcopy(experiment[0])
    cfg['flight']['controller'][key] = value
    with pytest.raises(ValueError):
        validate_controller(cfg['flight'], cfg['aircraft'], cfg['aero'])


def test_disabled_controller_preserves_aircraft_loads(experiment):
    controlled, baseline = models(experiment)
    controlled.controller['enabled'] = False
    y = controlled.initial()[:13]
    y[2] -= 20
    f, m, d = controlled.aircraft_loads(50.0, y)
    f0, m0, d0 = baseline.aircraft_loads(50.0, y)
    np.testing.assert_array_equal(f, f0)
    np.testing.assert_array_equal(m, m0)
    assert d == d0
