"""Explicit, opt-in longitudinal feedback for simulation experiments.

Outer altitude/climb-rate feedback commands pitch; pitch/rate feedback commands
elevator, and airspeed feedback commands thrust. Memoryless PD/P, ideal state
measurement and instantaneous actuators: not a TECS or flight-certified autopilot.
"""

import numpy as np


def controller_defaults(config):
    """Visible experimental gains, never enabled implicitly or claimed tuned."""
    return dict(
        enabled=False,
        type='longitudinal_pd',
        enable_from_s=0.0,
        engage_ramp_s=1.0,
        altitude_m=config['flight']['altitude_m'],
        airspeed_m_s=config['flight']['speed_m_s'],
        altitude_kp_deg_m=0.8,
        climb_kd_deg_per_m_s=2.0,
        pitch_kp=0.8,
        pitch_rate_kd_s=0.15,
        airspeed_kp_N_per_m_s=4.0,
        elevator_pitch_sign='from_aero',
        pitch_limits_deg=[-6.0, 6.0],
        elevator_limits_deg=[min(config['aero']['elevator_deg']), max(config['aero']['elevator_deg'])],
        thrust_limits_N=[0.0, config['aircraft']['max_thrust_N']],
        measurement_model='perfect_state',
        actuator_model='instantaneous_bounded',
        gain_source='experimental initial gains; not aircraft-specific tuning',
    )


def resolve_controller(config, aero, trim):
    """Determine/check elevator direction from the actual CG moment derivative."""
    c = dict(config['flight'].get('controller') or {})
    if not c.get('enabled'):
        return c
    limits = c['elevator_limits_deg']
    if limits[0] < aero.axes[2][0] or limits[1] > aero.axes[2][-1]:
        raise ValueError('Controller elevator limits exceed loaded aerodynamic database')
    if (
        not limits[0] <= trim['elevator_deg'] <= limits[1]
        or not c['thrust_limits_N'][0] <= trim['thrust_N'] <= c['thrust_limits_N'][1]
    ):
        raise ValueError('Controller limits exclude the initial trim commands')
    lo = max(aero.axes[2][0], trim['elevator_deg'] - 0.1)
    hi = min(aero.axes[2][-1], trim['elevator_deg'] + 0.1)
    if hi <= lo:
        raise ValueError('Cannot determine elevator authority at trim')
    speed = config['flight']['speed_m_s']
    dc = (
        aero.evaluate(trim['alpha_rad'], 0.0, hi, np.zeros(3), speed)
        - aero.evaluate(trim['alpha_rad'], 0.0, lo, np.zeros(3), speed)
    ) / (hi - lo)
    area, chord, _ = aero.refs
    offset = np.array(aero.metadata['moment_reference_frd_m']) - config['aircraft']['cg_m']
    derivative = 0.5 * config['flight']['rho_kg_m3'] * speed**2 * area * (chord * dc[4] + np.cross(offset, dc[:3])[1])
    if not np.isfinite(derivative) or abs(derivative) < 1e-8:
        raise ValueError('No usable elevator pitch authority in the actual aerodynamic table')
    sign = int(np.sign(derivative))
    if c['elevator_pitch_sign'] != 'from_aero' and c['elevator_pitch_sign'] != sign:
        raise ValueError('Controller elevator direction opposes the actual CG pitch moment derivative')
    c.update(elevator_pitch_sign=sign, elevator_moment_derivative_Nm_deg=float(derivative))
    return c


def validate_controller(flight, aircraft, aero):
    c = flight.get('controller')
    if c is None:
        return
    if not isinstance(c, dict) or not isinstance(c.get('enabled'), bool):
        raise ValueError('flight.controller requires a boolean enabled field')
    if not c['enabled']:
        return
    if c.get('type') != 'longitudinal_pd':
        raise ValueError('Unsupported flight.controller.type')
    if c.get('measurement_model') != 'perfect_state' or c.get('actuator_model') != 'instantaneous_bounded':
        raise ValueError('Controller requires explicit perfect_state and instantaneous_bounded assumptions')
    scalar = (
        'enable_from_s',
        'engage_ramp_s',
        'altitude_m',
        'airspeed_m_s',
        'altitude_kp_deg_m',
        'climb_kd_deg_per_m_s',
        'pitch_kp',
        'pitch_rate_kd_s',
        'airspeed_kp_N_per_m_s',
    )
    if any(key not in c or not np.isfinite(c[key]) for key in scalar):
        raise ValueError('Missing/nonfinite controller parameter')
    if c['enable_from_s'] < 0 or c['engage_ramp_s'] <= 0 or c['altitude_m'] <= 0 or c['airspeed_m_s'] <= 1:
        raise ValueError('Invalid controller timing or targets')
    if any(c[key] <= 0 for key in scalar[4:]):
        raise ValueError('Controller gains must be positive')
    if c.get('elevator_pitch_sign') not in (-1, 1, 'from_aero'):
        raise ValueError('elevator_pitch_sign must be -1, 1 or from_aero')
    for name in ('pitch_limits_deg', 'elevator_limits_deg', 'thrust_limits_N'):
        limits = np.asarray(c.get(name, []), float)
        if limits.shape != (2,) or not np.isfinite(limits).all() or limits[0] >= limits[1]:
            raise ValueError(f'Invalid controller {name}')
    if not -85 < c['pitch_limits_deg'][0] < c['pitch_limits_deg'][1] < 85:
        raise ValueError('Controller pitch limits must remain within +/-85 degrees')
    if c['elevator_limits_deg'][0] < min(aero['elevator_deg']) or c['elevator_limits_deg'][1] > max(
        aero['elevator_deg']
    ):
        raise ValueError('Controller elevator limits exceed aerodynamic grid')
    if c['thrust_limits_N'][0] < 0 or c['thrust_limits_N'][1] > aircraft['max_thrust_N']:
        raise ValueError('Controller thrust limits exceed aircraft limits')


def longitudinal_commands(c, t, aircraft_state, trim, pitch_rad, speed, elevator, thrust):
    """Pure function of current state/time; safe for rejected implicit RHS trials."""
    blend = float(np.clip((t - c['enable_from_s']) / c['engage_ramp_s'], 0.0, 1.0))
    altitude_error = c['altitude_m'] + aircraft_state[2]
    climb_rate = -aircraft_state[5]
    pitch_command = float(
        np.clip(
            np.rad2deg(trim['alpha_rad'])
            + c['altitude_kp_deg_m'] * altitude_error
            - c['climb_kd_deg_per_m_s'] * climb_rate,
            *c['pitch_limits_deg'],
        )
    )
    pitch_error = pitch_command - np.rad2deg(pitch_rad)
    elevator_raw = elevator + blend * c['elevator_pitch_sign'] * (
        c['pitch_kp'] * pitch_error - c['pitch_rate_kd_s'] * np.rad2deg(aircraft_state[11])
    )
    thrust_raw = thrust + blend * c['airspeed_kp_N_per_m_s'] * (c['airspeed_m_s'] - speed)
    # Before activation, preserve the original open-loop path exactly.
    el = float(np.clip(elevator_raw, *c['elevator_limits_deg'])) if blend > 0 else elevator
    power = float(np.clip(thrust_raw, *c['thrust_limits_N'])) if blend > 0 else thrust
    diagnostics = dict(
        controller_active=blend > 0,
        controller_blend=blend,
        altitude_setpoint_m=c['altitude_m'],
        airspeed_setpoint_m_s=c['airspeed_m_s'],
        altitude_error_m=float(altitude_error),
        climb_rate_m_s=float(climb_rate),
        pitch_setpoint_deg=pitch_command,
        controller_elevator_unlimited_deg=float(elevator_raw),
        controller_thrust_unlimited_N=float(thrust_raw),
        elevator_saturated=bool(el != elevator_raw),
        thrust_saturated=bool(power != thrust_raw),
    )
    return el, power, diagnostics
