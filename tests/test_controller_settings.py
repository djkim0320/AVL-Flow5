"""Controller commands and input contract, independent of saved solver runs."""

import numpy as np
import pytest
from dbf_stability.control import controller_defaults, validate_controller, longitudinal_commands


def settings():
    cfg = dict(
        flight=dict(altitude_m=100.0, speed_m_s=20.0),
        aircraft=dict(max_thrust_N=30.0),
        aero=dict(elevator_deg=[-8.0, 0.0, 8.0]),
    )
    c = controller_defaults(cfg)
    c['enabled'] = True
    cfg['flight']['controller'] = c
    return cfg, c


def test_complete_opt_in_defaults_fit_model_limits():
    cfg, c = settings()
    assert not controller_defaults(cfg)['enabled']
    validate_controller(cfg['flight'], cfg['aircraft'], cfg['aero'])
    assert c['elevator_limits_deg'] == [-8.0, 8.0]
    assert c['thrust_limits_N'] == [0.0, 30.0]


def test_feedback_commands_not_position_overwrite_and_activation_is_continuous():
    _, c = settings()
    c['elevator_pitch_sign'] = -1
    state = np.zeros(13)
    state[2] = -98.0
    state[3] = 19.0
    before = state.copy()
    trim = dict(alpha_rad=0.0, elevator_deg=1.0, thrust_N=4.0)

    def commands(t):
        return longitudinal_commands(c, t, state, trim, 0.0, 19.0, 1.0, 4.0)

    assert commands(0)[:2] == (1.0, 4.0)
    full = commands(2.0)
    assert full[0] < 1.0 and full[1] > 4.0
    assert abs(commands(1e-8)[0] - 1.0) < 1e-7
    commands(100.0)
    assert commands(2.0) == full
    np.testing.assert_array_equal(state, before)


def test_rate_damping_and_hardware_saturation():
    _, c = settings()
    c['elevator_pitch_sign'] = -1
    state = np.zeros(13)
    state[2] = -100.0
    state[11] = 20.0
    el, thrust, d = longitudinal_commands(c, 2.0, state, dict(alpha_rad=0.0), 0.0, 2.0, 1.0, 4.0)
    assert el == 8.0 and thrust == 30.0
    assert d['elevator_saturated'] and d['thrust_saturated']


@pytest.mark.parametrize(
    'key,value',
    [
        ('pitch_kp', 0.0),
        ('pitch_kp', np.nan),
        ('elevator_pitch_sign', 'guess'),
        ('elevator_limits_deg', [-9.0, 8.0]),
        ('thrust_limits_N', [0.0, 31.0]),
    ],
)
def test_invalid_tuning_rejected(key, value):
    cfg, c = settings()
    c[key] = value
    with pytest.raises(ValueError):
        validate_controller(cfg['flight'], cfg['aircraft'], cfg['aero'])
