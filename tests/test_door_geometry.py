from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from dbf_stability import load_case, changed
from dbf_stability.door import door_hinge, door_reference_shift, door_box, door_rotation_radius
from dbf_stability.collision import read_binary_stl, BOX_SIGNS, MeshContacts

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('offset,closed', [([0,0,0],[0,0,0]),([-.003,0,.003],[0,0,0]),([-.003,0,.003],[-.003,0,0])])
@pytest.mark.parametrize('angle', [0,45,100,140,160,180])
def test_door_mesh_and_analytic_box_agree(offset,closed,angle):
    c=changed(load_case(ROOT/'examples/h1_reference.yaml'),**{'bay.door_hinge_offset_m':offset,'bay.door_closed_offset_m':closed})
    b=c['bay'];h=door_hinge(b)
    path=ROOT/c['collision']['mesh_directory']/'rear_door_100deg_FRD_m.stl'
    vertices,_=read_binary_stl(path)
    posed=(vertices+door_reference_shift(b)-h)@Rotation.from_euler('y',angle-100,degrees=True).as_matrix().T+h
    center,axes,half=door_box(b,angle)
    local=(posed-center)@axes
    assert np.allclose(local.min(0),-half,atol=6e-8)
    assert np.allclose(local.max(0),half,atol=6e-8)
    assert np.max(np.linalg.norm((posed-h)[:,[0,2]],axis=1))<=door_rotation_radius(b)+6e-8
    if angle==0:
        assert np.allclose(center,np.array([b['exit_x_m'],0,b['floor_z_m']-b['door_length_m']/2])+closed)
    world=MeshContacts(c)
    try:
        obs=next(o for o in world.obstacles if o['door'])
        transformed=Rotation.from_euler('y',angle-100,degrees=True).apply(obs['vertices'])+world.hinge
        assert np.allclose(transformed,posed,atol=1e-12)
    finally:world.close()


@pytest.mark.parametrize('bad', [[0,1],[0,np.nan,0],['bad',0,0]])
def test_invalid_hinge_is_rejected(bad):
    with pytest.raises(ValueError):
        changed(load_case(ROOT/'examples/h1_reference.yaml'),**{'bay.door_hinge_offset_m':bad})
