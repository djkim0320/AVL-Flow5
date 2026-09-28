"""Explicit geometry-free payload model; no inferred sensor aerodynamics."""
import numpy as np


def configure(config, mass_kg):
    if isinstance(mass_kg, bool) or not np.isfinite(mass_kg) or mass_kg <= 0:
        raise ValueError('Point mass must be finite and positive')
    config['sensor'] = dict(model='point_mass', mass_kg=float(mass_kg), tow_point_m=[0., 0., 0.])
    # The simplified model has no CAD contact or payload attitude. Keep the
    # translational latch and cable material model for payout/recovery histories.
    config['collision'] = {'enabled': False}
    config['cable']['guide_points_body_m'] = []
    config['bay']['release_push_N'] = 0.
    config['bay']['stowed_quaternion_wxyz'] = [1., 0., 0., 0.]
    config['flight'].pop('initial_sensor_angles_delta_deg', None)
    config['provenance']['sensor'] = dict(kind='design', source=(
        'UI point mass: gravity and cable tension only; no sensor shape, drag, '
        'rotational inertia or surface contact. Marker radius is visual only.'))


def enabled(config):
    return config['sensor'].get('model') == 'point_mass'
