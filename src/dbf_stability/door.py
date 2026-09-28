"""Shared FRD geometry for a rear door with an optional offset hinge.

Closed-leaf and hinge offsets are relative to the original aperture. Existing
CAD assets describe the leaf at 100 degrees about its original lower edge.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def hinge_offset(bay):
    return np.asarray(bay.get('door_hinge_offset_m', [0., 0., 0.]), float)


def closed_offset(bay):
    return np.asarray(bay.get('door_closed_offset_m', [0., 0., 0.]), float)


def door_hinge(bay):
    return np.array([bay['exit_x_m'], 0., bay['floor_z_m']]) + hinge_offset(bay)


def door_reference_shift(bay):
    """Translation of the original 100-degree asset for the offset hinge."""
    r=Rotation.from_euler('y', 100, degrees=True).as_matrix()
    return (np.eye(3)-r) @ hinge_offset(bay) + r @ closed_offset(bay)


def door_box(bay, angle_deg):
    axes = Rotation.from_euler('y', angle_deg, degrees=True).as_matrix()
    half = np.array([bay['door_thickness_m']/2, bay['half_width_m'], bay['door_length_m']/2])
    center = door_hinge(bay) + axes @ (np.array([0., 0., -half[2]])+closed_offset(bay)-hinge_offset(bay))
    return center, axes, half


def door_rotation_radius(bay):
    """Largest distance from the hinge axis to any closed-leaf corner."""
    if 'door_mesh_radius_m' in bay:return bay['door_mesh_radius_m']
    o = hinge_offset(bay)-closed_offset(bay)
    return float(np.hypot(bay['door_thickness_m']/2+abs(o[0]),
                         max(abs(o[2]), abs(bay['door_length_m']+o[2]))))
