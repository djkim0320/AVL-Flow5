"""R2 physical invariants: actual meshes, mass bookkeeping, released actuator."""
from pathlib import Path
import json
import copy
import numpy as np
import pytest
from dbf_stability import load_case, AeroDatabase
from dbf_stability.collision import MeshContacts, read_binary_stl
from dbf_stability.model import CoupledModel
from dbf_stability.math3d import rotation

ROOT=Path(__file__).resolve().parents[1]
VARIANT=ROOT/'test_models/H1_reference/geometry_variants/refined_r2'


@pytest.fixture(params=['refined_r2','normal_r3'])
def refined(request): return load_case(ROOT/f'examples/h1_{request.param}.yaml')


@pytest.fixture
def refined_aero(refined):
    path=(ROOT/'outputs/refined_r2_aero_database.npz' if 'refined_r2' in refined['aero']['geometry']
          else ROOT/'test_models/H1_reference/runs/normal_r3_01/aerodynamics/aero_database.npz')
    if not path.exists():pytest.skip('Build the actual matching refined-model AVL database first')
    return AeroDatabase(path)


def test_mass_tensor_and_all_coordinate_references_shift_together(refined):
    variant=(ROOT/refined['collision']['mesh_directory']).parent
    data=json.loads((variant/'mass_properties.json').read_text(encoding='utf8'))
    rows=data['records'];mass=sum(r['mass_kg'] for r in rows)
    shift=np.array(data['old_to_new_cg_frd_m'])
    assert mass==pytest.approx(refined['aircraft']['mass_kg'])
    assert np.linalg.norm(sum(r['mass_kg']*(np.array(r['center_old_frd_m'])-shift) for r in rows))<1e-12
    assert not any(r['part'].startswith('sensor') or r['part']=='cable' for r in rows)
    assert np.min(np.linalg.eigvalsh(refined['aircraft']['inertia_kgm2']))>0
    old=load_case(ROOT/'examples/h1_opening_70.yaml')
    assert np.allclose(refined['bay']['stowed_center_m'],np.array(old['bay']['stowed_center_m'])-shift)
    mesh=ROOT/refined['collision']['mesh_directory']
    ceiling,_=read_binary_stl(mesh/'guide_ceiling_FRD_m.stl')
    assert ceiling[:,2].max()==pytest.approx(refined['bay']['ceiling_z_m'],abs=8e-8)
    assert refined['bay']['floor_z_m']-refined['bay']['ceiling_z_m']==pytest.approx(.070)


def test_fairlead_has_a_real_hole_and_drum_is_checked(refined):
    world=MeshContacts(refined)
    try:
        feed=np.array(refined['aircraft']['tow_point_m'])
        far=np.array([-3.,0.,0.])
        hits=world.contacts(far,np.eye(3),np.array([feed-[.015,0,0],feed]),140)
        assert not any(h['fixed']=='fairlead_ring' and h['gap']<0 for h in hits)
        drum=next(p for p in world.obstacles if p['name']=='winch_drum')
        center=(drum['lo']+drum['hi'])/2
        hits=world.contacts(far,np.eye(3),np.array([center-[.04,0,0],center+[.04,0,0]]),140)
        assert any(h['fixed']=='winch_drum' and h['kind']=='cable' and h['gap']<0 for h in hits)
        assert next(p for p in world.obstacles if p['name']=='fairlead_ring')['concave']
    finally: world.close()


def test_recovery_does_not_continue_release_push(refined,refined_aero):
    # Same real aero/state on both sides of a force comparison; no mocked loads.
    aero=refined_aero
    disabled=copy.deepcopy(refined);disabled['bay']['release_push_N']=0.
    first=CoupledModel(refined,aero);second=CoupledModel(disabled,aero)
    state=first.initial()
    at_release=first.rhs(1.,state)-second.rhs(1.,state)
    force=first.ms*at_release[16:19]
    assert force==pytest.approx(rotation(state[6:10])@np.array([-.12,0,0]),abs=1e-9)
    assert np.allclose(first.rhs(3.1,state),second.rhs(3.1,state),atol=1e-12)


def test_reel_schedule_is_bounded_and_stops_before_door_closing(refined):
    rows=np.array(refined['winch']['length_schedule']);start=refined['winch']['stowed_length_m']
    assert rows[:,1].min()>=start-1e-12
    assert rows[:,1].max()<=refined['cable']['length_m']+1e-12
    assert np.allclose(rows[rows[:,0]>=9.,1],start)
    nose=np.array(refined['bay']['stowed_center_m'])+refined['sensor']['tow_point_m']
    assert np.linalg.norm(nose-refined['aircraft']['tow_point_m'])==pytest.approx(start)


@pytest.mark.parametrize('segments',[10,20,40])
def test_initial_exposed_line_has_no_artificial_elastic_preload(refined,segments,refined_aero):
    refined['cable']['segments']=segments
    aero=refined_aero
    model=CoupledModel(refined,aero)
    state=model.initial();r=rotation(state[6:10]);s=rotation(state[19:23])
    nose=state[13:16]+s@model.sensor_attach;feed=state[:3]+r@model.attach
    k=model.active_count(model.length(0)[0]);nodes=state[26:].reshape(segments,6)
    chain=np.vstack([nose,nodes[:k,:3],feed])
    rests=np.r_[model.h/2,np.full(k-1,model.h),model.length(0)[0]-(k-.5)*model.h]
    assert np.allclose(np.linalg.norm(np.diff(chain,axis=0),axis=1),rests,atol=1e-12)
    assert np.allclose(nodes[k:,:3],feed)


def test_stationary_trim_phases_use_stationary_door(refined,refined_aero):
    aero=refined_aero
    assert CoupledModel(refined,aero,phase='deployed').door_state(0)==(140,0)
    assert CoupledModel(refined,aero,phase='stowed').door_state(.2)==(0,0)
    assert CoupledModel(refined,aero,phase='mission').door_state(.2)==pytest.approx((70,350))


@pytest.mark.parametrize('fixture',['normal_r3_ramp_trial.npz','normal_r3_shell_trial.npz','normal_r3_wire_trial.npz'])
def test_ramp_contact_is_continuous_and_conserves_reaction(fixture):
    # A recorded UNACCEPTED Newton trial from the numerical stall, used only as
    # a difficult force-evaluation test input, never as a motion result.
    c=load_case(ROOT/'examples/h1_normal_r3.yaml')
    a=AeroDatabase(ROOT/'test_models/H1_reference/runs/normal_r3_01/aerodynamics/aero_database.npz')
    from dbf_stability import solve_trim
    trim=solve_trim(c,a,mode='stowed')
    model=CoupledModel(c,a,trim)
    d=np.load(ROOT/'tests/data'/fixture)
    state=d['state'];time=float(d['time']);model.active_override=int(d['active_nodes'])
    slopes=[]
    for h in (1e-6,5e-7):
        plus=state.copy();minus=state.copy();plus[14]+=h;minus[14]-=h
        slopes.append((model.rhs(time,plus)-model.rhs(time,minus))/(2*h))
    assert np.linalg.norm(slopes[0]-slopes[1])<1e-3
    free=copy.deepcopy(c);free['collision']['enabled']=False
    baseline=CoupledModel(free,a,trim);baseline.active_override=model.active_override
    delta=model.rhs(time,state)-baseline.rhs(time,state)
    nodes=state[26:].reshape(model.n,6);dn=delta[26:].reshape(model.n,6)
    force=model.ma*delta[3:6]+model.ms*delta[16:19]+model.mn*dn[:,3:].sum(0)
    moment=np.cross(state[:3],model.ma*delta[3:6])+rotation(state[6:10])@model.Ia@delta[10:13]
    moment+=np.cross(state[13:16],model.ms*delta[16:19])+rotation(state[19:23])@model.Is@delta[23:26]
    moment+=np.cross(nodes[:,:3],model.mn*dn[:,3:]).sum(0)
    assert np.linalg.norm(force)<1e-8
    assert np.linalg.norm(moment)<1e-8


def test_short_contact_is_retained_between_regular_output_samples():
    # Real positive-gap contact state, shorter than the 10 ms output period.
    # It is an initial-condition regression test, not a resumed mission result.
    from dbf_stability import solve_trim,simulate
    c=load_case(ROOT/'examples/h1_normal_r3.yaml')
    a=AeroDatabase(ROOT/'test_models/H1_reference/runs/normal_r3_01/aerodynamics/aero_database.npz')
    trim=solve_trim(c,a,mode='stowed')
    trial=np.load(ROOT/'tests/data/normal_r3_shell_trial.npz')
    t=float(trial['time'])
    r=simulate(c,a,trim,initial_state=trial['state'],start_time=t,duration=.0002)
    assert r.summary['status']=='completed'
    assert len(r.time)>2
    assert r.table.contact_N.max()>0
    assert r.summary['contact_impulse_Ns']>0


def test_clearance_only_reduction_preserves_complete_mesh_minimum():
    c=load_case(ROOT/'examples/h1_normal_r3.yaml')
    a=AeroDatabase(ROOT/'test_models/H1_reference/runs/normal_r3_01/aerodynamics/aero_database.npz')
    m=CoupledModel(c,a)
    trial=np.load(ROOT/'tests/data/normal_r3_shell_trial.npz')
    m.active_override=int(trial['active_nodes'])
    m.contact_geometry(float(trial['time']),trial['state'],distance=.01)
    world=m._mesh_contacts;checked=0
    for part in world.sensor:
        for obstacle in world.obstacles:
            points=world.p.getClosestPoints(part['body'],obstacle['body'],.01,physicsClientId=world.client)
            if not points:continue
            full=world._reduce(points,'sensor',-1,part['name'],obstacle)
            nearest=world._reduce(points,'sensor',-1,part['name'],obstacle,nearest_only=True)
            assert len(nearest)==1
            assert nearest[0]['gap']==min(hit['gap'] for hit in full)
            assert nearest[0]['gap']==min(p[8] for p in points)
            checked+=1
    assert checked>0
    world.close()


def test_regularised_wire_triangle_contact_matches_exact_plane_solution():
    from dbf_stability.collision import capsule_mesh
    vertices=np.array([[-1.,-1,0],[1,-1,0],[1,1,0],[-1,1,0]])
    faces=np.array([[0,1,2],[0,2,3]])
    a=np.array([-.2,0,.0012]);b=np.array([.2,0,.0012])
    point,normal,gap,weight=capsule_mesh(a,b,.0005,vertices,faces,.00016)
    assert point==pytest.approx([0,0,.0012],abs=1e-12)
    assert normal==pytest.approx([0,0,1],abs=1e-12)
    assert gap==pytest.approx(.0007,abs=1e-12)
    assert weight==pytest.approx(1,abs=1e-12)
    # Reverse the segment: the physical closest point and force stay unchanged.
    reversed_result=capsule_mesh(b,a,.0005,vertices,faces,.00016)
    for first,second in zip((point,normal,gap,weight),reversed_result):
        assert first==pytest.approx(second,abs=1e-12)


def test_wire_shell_contact_force_has_no_jumping_witness():
    c=load_case(ROOT/'examples/h1_normal_r3.yaml')
    a=AeroDatabase(ROOT/'test_models/H1_reference/runs/normal_r3_01/aerodynamics/aero_database.npz')
    m=CoupledModel(c,a)
    d=np.load(ROOT/'tests/data/normal_r3_wire_trial.npz')
    y=d['state'];t=float(d['time']);m.active_override=int(d['active_nodes'])
    slopes=[]
    direction=np.zeros(len(y));direction[26:]=np.random.default_rng(943).normal(size=len(y)-26)
    for h in (1e-7,5e-8):
        slopes.append((m.rhs(t,y+h*direction)-m.rhs(t,y-h*direction))/(2*h))
    assert np.linalg.norm(slopes[0]-slopes[1])/max(1.,np.linalg.norm(slopes[1]))<.01
