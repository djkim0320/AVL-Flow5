"""Initial material-cell placement through physical, aircraft-fixed guide rings.

Guide coordinates initialise the line only. Subsequent motion and all reactions
are solved by the existing cable/contact equations, not constrained to this path.
"""

import numpy as np
from .math3d import rotation


def path_points(config, nose, feed):
    guides = config['cable'].get('guide_points_body_m', [])
    return np.array([nose, *reversed(guides), feed], float)


def sample_path(points, distances):
    spans = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if len(spans) == 1 and spans[0] < 1e-9:
        return np.tile(points[0], (len(distances), 1))
    if np.any(spans < 1e-9):
        raise ValueError('Cable route has duplicate adjacent points')
    arcs = np.r_[0, np.cumsum(spans)]
    return np.column_stack([np.interp(distances, arcs, points[:, i]) for i in range(3)])


def cable_initial_points(config, aircraft, sensor, active, length):
    r = rotation(aircraft[6:10])
    rs = rotation(sensor[6:10])
    nose = r.T @ (sensor[:3] - aircraft[:3]) + r.T @ rs @ np.asarray(config['sensor']['tow_point_m'])
    points = path_points(config, nose, config['aircraft']['tow_point_m'])
    route_length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
    # Keep legacy straight slack-line initialisation: material centres share the
    # specified initial path proportionally; mass and rest lengths never change.
    fraction = (np.arange(active) + 0.5) * (config['cable']['length_m'] / config['cable']['segments']) / length
    return aircraft[:3] + sample_path(points, fraction * route_length) @ r.T
