"""Geometry consistency and the saved first-snag regression; no imposed motion."""
from pathlib import Path
import json,sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from dbf_stability import load_case,AeroDatabase
from dbf_stability.analysis import load_result
from dbf_stability.model import CoupledModel
from dbf_stability.door import door_box,door_hinge,door_reference_shift
from dbf_stability.collision import read_binary_stl
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'test_models/H1_reference'
sys.path.insert(0,str(HERE))
from run_capture_stop import map_reference

@pytest.mark.parametrize('angle',[0,45,90,140,180,225,270])
def test_wider_door_uses_same_cad_and_collision_envelope(angle):
    c=load_case(ROOT/'examples/h1_recovery_funnel_v2.yaml');b=c['bay']
    v,_=read_binary_stl(ROOT/c['collision']['mesh_directory']/'rear_door_100deg_FRD_m.stl')
    h=door_hinge(b);posed=Rotation.from_euler('y',angle-100,degrees=True).apply(v+door_reference_shift(b)-h)+h
    center,axes,half=door_box(b,angle);local=(posed-center)@axes
    np.testing.assert_allclose(local.min(0),-half,atol=1.1e-7)
    np.testing.assert_allclose(local.max(0),half,atol=1.1e-7)

def test_mass_changes_and_unchanged_lifting_surfaces():
    old=load_case(ROOT/'examples/h1_underbody_270_v2.yaml');new=load_case(ROOT/'examples/h1_recovery_funnel_v2.yaml')
    folder=ROOT/new['collision']['mesh_directory'];record=json.loads((folder.parent/'geometry.json').read_text())
    delta=sum(r['signed_mass_kg'] for r in record['mass_changes'])
    assert new['aircraft']['mass_kg']==pytest.approx(old['aircraft']['mass_kg']+delta)
    assert record['shell_obstruction_mm3']==0
    assert max(record['mount_to_duct_gap_mm'].values())<1e-5
    assert new['sensor']==old['sensor']
    assert new['collision']['minimum_gap_m']==old['collision']['minimum_gap_m']
    assert new['aero']['geometry']==old['aero']['geometry']
    for key in ('capture_radius_m','capture_speed_m_s','capture_angle_deg'):
        assert new['bay'][key]==old['bay'][key]
    for name in ('main_wing_CAD_approx','horizontal_tail_CAD_approx','vertical_fin_CAD_approx','sensor_body'):
        v,f=read_binary_stl(ROOT/old['collision']['mesh_directory']/f'{name}_FRD_m.stl')
        w,g=read_binary_stl(folder/f'{name}_FRD_m.stl')
        np.testing.assert_allclose(v[f]-record['cg_shift_m'],w[g],atol=1.3e-7,rtol=0)

def test_original_first_nose_snag_has_no_obstacle_in_new_geometry():
    r=load_result(HERE/'runs/underbody_mission_02/complete');i=np.argmin(abs(r.time-70.2));t=float(r.time[i]);old=r.states[i]
    c=load_case(ROOT/'examples/h1_recovery_funnel_v2.yaml');new,_=map_reference(old,r.config,c)
    aero=AeroDatabase(HERE/'runs/underbody_aero_refined_01/aero_database.npz')
    explicit_modeling_manifest(r.config,'h1_underbody_270_v2.yaml')
    models=[CoupledModel(r.config,aero),CoupledModel(c,aero)]
    try:
        before=models[0].contact_geometry(t,old)
        assert any(h['moving']=='sensor_body' and h['fixed'].startswith('fuselage') and h['gap']<c['collision']['skin_m'] for h in before)
        after=models[1].contact_geometry(t,new,distance=.002)
        assert not [h for h in after if h['kind']=='sensor' and h.get('geometry_gap',h['gap'])<.002]
        # Mapping changed a reference point, not either body's orientation/rates.
        np.testing.assert_array_equal(old[6:13],new[6:13]);np.testing.assert_array_equal(old[19:26],new[19:26])
    finally:
        for m in models:
            if m._mesh_contacts:m._mesh_contacts.close()

from conftest import explicit_modeling_manifest
