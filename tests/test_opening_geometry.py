"""Check the enlarged physical aperture and door using exported solid meshes."""

from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from dbf_stability import load_case
from dbf_stability.collision import read_binary_stl, MeshContacts
from dbf_stability.door import door_hinge, door_reference_shift, door_box

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('height', [70, 80])
def test_enlarged_aperture_has_matching_meshes(height):
    c = load_case(ROOT / f'examples/h1_opening_{height}.yaml')
    b = c['bay']
    meshes = ROOT / c['collision']['mesh_directory']
    ceiling, _ = read_binary_stl(meshes / 'guide_ceiling_FRD_m.stl')
    assert np.isclose(ceiling[:, 2].max(), b['ceiling_z_m'], atol=6e-8)
    assert np.isclose(b['floor_z_m'] - b['ceiling_z_m'], height * 0.001)
    for sign in (-1, 1):
        v, _ = read_binary_stl(meshes / f'guide_side_{sign:+d}_FRD_m.stl')
        assert np.allclose([v[:, 2].min(), v[:, 2].max()], [b['ceiling_z_m'], b['floor_z_m']], atol=6e-8)
    v, _ = read_binary_stl(meshes / 'rear_exit_frame_FRD_m.stl')
    assert np.allclose([v[:, 2].min(), v[:, 2].max()], [b['ceiling_z_m'] - 0.004, b['floor_z_m'] + 0.004], atol=6e-8)
    door, _ = read_binary_stl(meshes / 'rear_door_100deg_FRD_m.stl')
    h = door_hinge(b)
    for angle in (0, 45, 100, 140, 180):
        posed = (door + door_reference_shift(b) - h) @ Rotation.from_euler(
            'y', angle - 100, degrees=True
        ).as_matrix().T + h
        center, axes, half = door_box(b, angle)
        local = (posed - center) @ axes
        assert np.allclose(local.min(0), -half, atol=6e-8)
        assert np.allclose(local.max(0), half, atol=6e-8)
    world = MeshContacts(c)
    try:
        assert len(world.sensor) == 5
        assert any(o['name'] == 'rear_exit_frame_ceiling' for o in world.obstacles)
    finally:
        world.close()
