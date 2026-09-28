"""Vectorized evaluation of the unchanged per-witness sensor barrier law."""

import numpy as np


def sensor_contact_loads(hits, a, s, ra, rs, skin, materials, default, friction, door_rate, hinge):
    active = [h for h in hits if h['kind'] == 'sensor' and h['gap'] < skin]
    if not active:
        return None
    p = np.array([h['point'] for h in active])
    normal = np.array([h['normal'] for h in active])
    gap = np.array([h['gap'] for h in active])
    weight = np.array([h.get('weight', 1.0) for h in active])
    scale = np.array([h.get('normal_velocity_scale', 1.0) for h in active])
    k = np.array([materials.get(h['fixed'], default)['stiffness_N_m'] for h in active])
    c = np.array([materials.get(h['fixed'], default)['damping_Ns_m'] for h in active])
    world = a[:3] + p @ ra.T
    velocity = s[3:6] + np.cross(rs @ s[10:13], world - s[:3])
    surface = (
        np.cross(np.array([0.0, np.deg2rad(door_rate), 0.0]), p - hinge)
        * np.array([h['door'] for h in active])[:, None]
    )
    relative = (velocity - a[3:6]) @ ra - np.cross(a[10:13], p) - surface
    speed = (relative * normal).sum(axis=1)
    tangent = relative - speed[:, None] * normal
    compression = skin - gap
    magnitude = (
        np.maximum(
            0.0,
            k * compression * skin / np.maximum(gap, skin * 0.02)
            - c * np.minimum(1.0, compression / skin) * speed * scale,
        )
        * weight
    )
    fb = magnitude[:, None] * (normal - friction * tangent / np.sqrt((tangent * tangent).sum(axis=1) + 1e-6)[:, None])
    force = fb @ ra.T
    return dict(
        sensor_force=force.sum(axis=0),
        sensor_torque=rs.T @ np.cross(world - s[:3], force).sum(axis=0),
        aircraft_force=-force.sum(axis=0),
        aircraft_torque=-ra.T @ np.cross(world - a[:3], force).sum(axis=0),
        contact_N=float(magnitude.sum()),
        door_power_W=float((fb * surface).sum()),
        penetration_m=float(np.maximum(0.0, -gap).max()),
        count=len(active),
    )
