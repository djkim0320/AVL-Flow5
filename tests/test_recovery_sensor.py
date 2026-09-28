"""Real sensor CAD, body-reference mapping and rotated-fin contact geometry."""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.spatial.transform import Rotation
from dbf_stability import load_case
from dbf_stability.collision import MeshContacts

ROOT=Path(__file__).resolve().parents[1]


def test_redesigned_sensor_mass_reference_and_airframe_preservation():
    old=load_case(ROOT/'examples/h1_normal_r3_flow5_controlled.yaml')
    new=load_case(ROOT/'examples/h1_normal_r3_round_x_sensor.yaml')
    variant=(ROOT/new['collision']['mesh_directory']).parent
    records=json.loads((variant/'geometry.json').read_text(encoding='utf8'))
    assert records['roundtrip_valid']
    assert abs(sum(r['mass_kg'] for r in records['mass_records'])-.04)<1e-12
    cg=sum(r['mass_kg']*np.array(r['center_aircraft_frd_m']) for r in records['mass_records'])/.04
    np.testing.assert_allclose(cg,new['bay']['stowed_center_m'],atol=1e-12)
    np.testing.assert_allclose(cg+new['sensor']['tow_point_m'],np.array(old['bay']['stowed_center_m'])+old['sensor']['tow_point_m'],atol=1e-12)
    assert np.linalg.eigvalsh(new['sensor']['inertia_kgm2']).min()>0
    for name,expected in records['unchanged_airframe_sha256'].items():
        assert hashlib.sha256((variant/'cad'/name).read_bytes()).hexdigest()==expected


def test_rotated_fin_mesh_matches_same_physical_box_rotated_as_a_body():
    old=load_case(ROOT/'examples/h1_normal_r3_flow5_controlled.yaml')
    new=load_case(ROOT/'examples/h1_normal_r3_round_x_sensor.yaml')
    first,second=MeshContacts(old),MeshContacts(new)
    rx=Rotation.from_euler('x',45,degrees=True).as_matrix()
    shift=np.array(new['sensor']['cg_shift_from_r3_body_m'])
    checked=0
    try:
        for x,z,pitch in [(-.57,.053,0),(-.6,.055,3),(-.6,.056,-3),(-.55,.052,5)]:
            r=Rotation.from_euler('y',pitch,degrees=True).as_matrix();center=np.array([x,0.,z])
            a=first.contacts(center,r@rx,np.empty((0,3)),140.,distance=.01)
            b=second.contacts(center+r@shift,r,np.empty((0,3)),140.,distance=.01)
            def reduce(hits):
                out={}
                for h in hits:
                    if h['moving'].startswith('sensor_fin_'):
                        key=(h['moving'],h['fixed']);out[key]=min(out.get(key,np.inf),h.get('geometry_gap',h['gap']))
                return out
            aa,bb=reduce(a),reduce(b)
            assert aa.keys()==bb.keys()
            for key in aa:
                # Guide boxes use analytical distances. Curved/sloped mesh
                # queries use Bullet's iterative GJK and differ at micrometres
                # after the same geometry is encoded in a different frame.
                tolerance=2e-7 if key[1].startswith(('guide_','rear_exit_frame')) else 2e-6
                assert abs(aa[key]-bb[key])<tolerance, key
                checked+=1
        assert checked>=8
    finally:first.close();second.close()


def test_force_does_not_disappear_at_gjk_search_cutoff():
    """The real stalled pose used to lose half its force after a 10 nm shift."""
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    runs=ROOT/'test_models/H1_reference/runs'
    aero=AeroDatabase(runs/'span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    trim=json.loads((runs/'control_100m_25ms_02/trim.json').read_text(encoding='utf8'))
    config=load_case(ROOT/'examples/h1_recovery_repair_round_nominal.yaml')
    with np.load(ROOT/'tests/data/recovery_contact_onset.npz',allow_pickle=False) as data:
        time=float(data['round_nominal_time']);state=data['round_nominal_state'].copy()
        active=int(data['round_nominal_active_nodes'])
    model=CoupledModel(config,aero,trim,phase='mission');model.active_override=active
    try:
        _,base=model.rhs(time,state,True)
        assert base['contact_N']>0
        for column in (0,1,2,13,14,15):
            for shift in (-1e-8,1e-8):
                nearby=state.copy();nearby[column]+=shift
                _,values=model.rhs(time,nearby,True)
                assert abs(values['contact_N']-base['contact_N'])<1e-4
        # A wider SEARCH band must not widen the actual force skin.
        outside=state.copy();outside[0]+=1e-5
        _,values=model.rhs(time,outside,True)
        assert values['contact_N']==0.
    finally:
        if model._mesh_contacts:model._mesh_contacts.close()


def test_compiled_features_preserve_coupled_forces_and_moments():
    import copy
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    runs=ROOT/'test_models/H1_reference/runs'
    aero=AeroDatabase(runs/'span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    trim=json.loads((runs/'control_100m_25ms_02/trim.json').read_text(encoding='utf8'))
    with np.load(ROOT/'tests/data/recovery_contact_onset.npz',allow_pickle=False) as data:
        for name in ('round_nominal','original_slow'):
            config=load_case(ROOT/f'examples/h1_recovery_repair_{name}.yaml')
            config['collision']['feature_engine']='numpy'
            compiled=copy.deepcopy(config);compiled['collision']['feature_engine']='numba'
            first=CoupledModel(config,aero,trim,phase='mission');second=CoupledModel(compiled,aero,trim,phase='mission')
            first.active_override=second.active_override=int(data[name+'_active_nodes'])
            t=float(data[name+'_time']);state=data[name+'_state'].copy()
            try:
                for axis in (13,14,15):
                    for delta in (-1e-4,0.,1e-4):
                        y=state.copy();y[axis]+=delta
                        a,da=first.rhs(t,y,True);b,db=second.rhs(t,y,True)
                        np.testing.assert_allclose(a,b,atol=5e-7,rtol=2e-9)
                        np.testing.assert_allclose(da['contact_N'],db['contact_N'],atol=1e-9,rtol=2e-9)
            finally:
                for model in (first,second):
                    if model._mesh_contacts:model._mesh_contacts.close()
