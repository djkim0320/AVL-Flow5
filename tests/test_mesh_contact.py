from pathlib import Path
import numpy as np
import pytest
from dbf_stability import load_case, AeroDatabase, solve_trim, simulate, changed
from dbf_stability.model import CoupledModel
from dbf_stability.collision import MeshContacts, barrier_force, box_box_manifold, capsule_box
from dbf_stability.math3d import rotation

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def h1():
    return load_case(ROOT/'examples/h1_reference.yaml')


@pytest.fixture
def h1aero():
    return AeroDatabase(ROOT/'test_models/H1_reference/aerodynamics/aero_database.npz')


def test_whole_sensor_and_segment_interior_detect_container(h1):
    world=MeshContacts(h1)
    center=np.asarray(h1['bay']['stowed_center_m'])
    floor=h1['bay']['floor_z_m']
    # A wire crosses a plate while both endpoints remain outside its material.
    chain=np.array([[center[0],0,floor-.02],[center[0],0,floor+.02]])
    hits=world.contacts(center,np.eye(3),chain,100)
    assert any(h['kind']=='cable' and h['fixed']=='guide_floor' and h['gap']<0 for h in hits)
    center[2]+= .025
    hits=world.contacts(center,np.eye(3),np.empty((0,3)),100)
    assert any(h['kind']=='sensor' and h['fixed']=='guide_floor' and h['gap']<0 for h in hits)
    world.close()


def test_contact_force_reacts_on_aircraft_and_preserves_momentum(h1,h1aero):
    c=changed(h1,**{'flight.g_m_s2':0.,'cable.cd_normal':0.,'cable.cd_tangent':0.,'winch.release_s':0.,'bay.release_push_N':0.})
    m=CoupledModel(c,h1aero)
    m.aircraft_loads=lambda t,a:(np.zeros(3),np.zeros(3),{})
    m.sensor_loads=lambda t,a,s:(np.zeros(3),np.zeros(3),0.)
    y=m.initial();y[15]+=.0046
    derivative,diagnosis=m.rhs(0,y,True)
    assert diagnosis['contact_N']>0
    nodes=y[26:].reshape(-1,6);dn=derivative[26:].reshape(-1,6)
    linear=m.ma*derivative[3:6]+m.ms*derivative[16:19]+m.mn*dn[:,3:].sum(0)
    angular=np.cross(y[:3],m.ma*derivative[3:6])+rotation(y[6:10])@m.Ia@derivative[10:13]
    angular+=np.cross(y[13:16],m.ms*derivative[16:19])+rotation(y[19:23])@m.Is@derivative[23:26]
    angular+=np.cross(nodes[:,:3],m.mn*dn[:,3:]).sum(0)
    assert np.linalg.norm(linear)<1e-9
    assert np.linalg.norm(angular)<1e-8


def test_barrier_and_friction(h1):
    c=h1['collision'];s=c['skin_m']
    values=[barrier_force(g,0,[0,0,0],s,800,2,.1)[0] for g in [s,s/2,s/10,s/100]]
    assert values[0]==0 and np.all(np.diff(values)>0)
    n,f=barrier_force(s/2,0,[1,0,0],s,800,2,.1)
    assert n>0 and f[0]<0


def test_initial_solid_intersection_is_rejected(h1,h1aero):
    trim=solve_trim(h1,h1aero,mode='stowed');y=trim['state'].copy()
    y[13:16]+=rotation(y[6:10])@np.array([0,0,.025])
    r=simulate(h1,h1aero,trim,phase='mission',initial_state=y,duration=.01)
    assert r.summary['status']=='contact_domain_exceeded'
    assert r.time[-1]==0


def test_box_corner_distance_and_span_crossing():
    rows=box_box_manifold(np.array([2.5,2.5,0]),np.eye(3),np.ones(3),np.zeros(3),np.eye(3),np.ones(3),1.)
    assert rows and min(row[3] for row in rows)==pytest.approx(np.sqrt(.5),abs=1e-12)
    point,normal,gap=capsule_box(np.array([0,0,-1.]),np.array([0,0,1.]),.001,np.zeros(3),np.eye(3),np.array([.1,.1,.001]))
    assert gap<0
    point,normal,gap=capsule_box(np.array([-.05,0,.003]),np.array([.05,0,.003]),.001,np.zeros(3),np.eye(3),np.array([.1,.1,.001]))
    assert gap==pytest.approx(.001,abs=1e-12)
    assert np.allclose(normal,[0,0,1])


def test_door_edge_does_not_switch_entire_force_between_fin_corners(h1,h1aero):
    from scipy.spatial.transform import Rotation
    recorded=np.load(ROOT/'tests/data/h1_door_edge_trial.npz')
    m=CoupledModel(h1,h1aero);m.active_override=int(recorded['active_nodes'])
    moments=[]
    for angle in [-1e-7,0,1e-7]:
        y=recorded['state'].copy()
        mat=rotation(y[19:23])@Rotation.from_rotvec([angle,0,0]).as_matrix()
        xyzw=Rotation.from_matrix(mat).as_quat();y[19:23]=np.r_[xyzw[3],xyzw[:3]]
        dy=m.rhs(float(recorded['time']),y)
        moments.append(m.Is@dy[23:26]+np.cross(y[23:26],m.Is@y[23:26]))
    # Previous single-witness contact generated about 0.003 N m of torque jump.
    assert np.max(np.abs(np.diff(moments,axis=0)))<1e-6


def test_dense_bound_catches_motion_between_zero_speed_samples(h1,h1aero):
    from dbf_stability.contact_integration import dense_surface_travel
    model=CoupledModel(h1,h1aero);model.active_override=0
    initial=model.initial()
    def curve(t):
        t=np.atleast_1d(t);states=np.repeat(initial[:,None],len(t),axis=1)
        states[13]+=t*(t-.5)*(t-1)
        return states
    bound=dense_surface_travel(model,curve,0,1)
    assert bound>.04


def test_fast_thin_wall_approach_stops_before_penetration(h1,h1aero):
    cfg=changed(h1,**{'winch.release_s':0.,'winch.door_schedule':[[0,100],[1,100]],'bay.release_push_N':0.})
    trim=solve_trim(cfg,h1aero,mode='stowed');y=trim['state'].copy();ra=rotation(y[6:10])
    y[13:16]+=ra@np.array([0,0,.004]);y[16:19]+=ra@np.array([0,0,4.])
    result=simulate(cfg,h1aero,trim,phase='mission',initial_state=y,duration=.006)
    assert result.summary['status']=='contact_clearance_limit'
    assert result.summary['max_penetration_m']==0
    assert result.time[-1]<.006


def test_recovery_wire_door_edge_force_is_locally_continuous(h1,h1aero):
    recorded=np.load(ROOT/'tests/data/h1_recovery_contact_trial.npz')
    model=CoupledModel(h1,h1aero,solve_trim(h1,h1aero,mode='stowed'))
    model.active_override=int(recorded['active_nodes'])
    t=float(recorded['time']);y=recorded['state'];base=model.rhs(t,y)
    for j in (2,8,58,64):
        plus=y.copy();minus=y.copy();plus[j]+=1e-8;minus[j]-=1e-8
        right=model.rhs(t,plus)-base;left=base-model.rhs(t,minus)
        assert np.linalg.norm(right-left)/np.linalg.norm(right+left)<1e-3


def test_endpoint_entering_skin_cannot_remove_interior_span_contact(h1):
    world=MeshContacts(h1);skin=h1['collision']['skin_m'];radius=h1['cable']['diameter_m']/2
    obstacle=dict(name='guide_floor',box_kind='aabb',lo=np.array([-.1,-.1,-.001]),hi=np.array([.1,.1,.001]),door=False)
    forces=[]
    for delta in (-1e-8,1e-8):
        a=np.array([-.05,0,.001+radius+skin+delta]);b=np.array([.15,0,.001+radius+skin-.0003])
        point,normal,gap=capsule_box(a,b,radius,np.zeros(3),np.eye(3),np.array([.1,.1,.001]))
        hit=dict(kind='cable',span=0,moving='cable_span_0',fixed='guide_floor',point=point,normal=normal,gap=gap,door=False)
        contacts=world._cable_manifold(hit,a,b,obstacle,100,radius)
        forces.append(sum(barrier_force(h['gap'],0,[0,0,0],skin,20000,8,.1)[0]*h['weight'] for h in contacts))
    # Previous endpoint selection jumped from 0.0002 to 6.26 N here.
    assert min(forces)>1.
    assert abs(forces[1]-forces[0])<.001
    world.close()


def test_regularised_wire_distance_gradient_and_force_balance(h1,h1aero):
    center=np.zeros(3);axes=np.eye(3);half=np.array([.1,.1,.001])
    radius=h1['cable']['diameter_m']/2;epsilon=h1['collision']['cable_contact_regularization_m']
    a=np.array([-.05,0,.0022]);b=np.array([.15,0,.0019])
    point,normal,gap=capsule_box(a,b,radius,center,axes,half,epsilon)
    fraction=np.dot(point-a,b-a)/np.dot(b-a,b-a)
    distance=np.linalg.norm(point-np.clip(point,-half,half))
    gradient=(1-fraction)*normal*distance/(gap+radius)
    numeric=[]
    for j in range(3):
        plus=a.copy();minus=a.copy();plus[j]+=1e-8;minus[j]-=1e-8
        numeric.append((capsule_box(plus,b,radius,center,axes,half,epsilon)[2]-capsule_box(minus,b,radius,center,axes,half,epsilon)[2])/2e-8)
    assert np.allclose(gradient,numeric,atol=1e-7)
    cfg=changed(h1,**{'flight.g_m_s2':0.,'cable.cd_normal':0.,'cable.cd_tangent':0.,'bay.release_push_N':0.})
    model=CoupledModel(cfg,h1aero,phase='deployed',fixed_length=.15);model.active_override=1
    model.aircraft_loads=lambda t,a:(np.zeros(3),np.zeros(3),{})
    model.sensor_loads=lambda t,a,s:(np.zeros(3),np.zeros(3),1.)
    y=model.initial();y[26:29]=y[:3]+[-.55,0,cfg['bay']['floor_z_m']-.0011]
    dy,diag=model.rhs(0,y,True)
    assert diag['cable_contact_N']>0
    nodes=y[26:].reshape(-1,6);dn=dy[26:].reshape(-1,6)
    linear=model.ma*dy[3:6]+model.ms*dy[16:19]+model.mn*dn[:,3:].sum(0)
    angular=np.cross(y[:3],model.ma*dy[3:6])+rotation(y[6:10])@model.Ia@dy[10:13]
    angular+=np.cross(y[13:16],model.ms*dy[16:19])+rotation(y[19:23])@model.Is@dy[23:26]
    angular+=np.cross(nodes[:,:3],model.mn*dn[:,3:]).sum(0)
    assert np.linalg.norm(linear)<1e-9
    assert np.linalg.norm(angular)<1e-8


def test_full_body_contact_does_not_jump_between_gjk_witnesses(h1,h1aero):
    data=np.load(ROOT/'test_models/H1_reference/diagnostics/recovery_final_trial.npz')
    m=CoupledModel(h1,h1aero,solve_trim(h1,h1aero,mode='stowed'))
    m.active_override=int(data['active_nodes']);t=float(data['time']);y=data['state'];base=m.rhs(t,y)
    for j in (0,7,8,9,13,15,20,22):
        plus=y.copy();minus=y.copy();plus[j]+=1e-8;minus[j]-=1e-8
        right=m.rhs(t,plus)-base;left=base-m.rhs(t,minus)
        # Former single-witness body contact produced >60 rad/s² jumps.
        assert np.max(abs(right-left))<1e-4


def test_mesh_feature_distance_matches_known_box_corner(h1):
    from dbf_stability.collision import mesh_box_manifold,polyhedron_edges,BOX_SIGNS
    world=MeshContacts(h1);part=next(p for p in world.sensor if p['name']=='sensor_fin_0')
    vertices=BOX_SIGNS.astype(float)+[2.5,2.5,0]
    rows=mesh_box_manifold(vertices,part['faces'],polyhedron_edges(vertices,part['faces']),np.zeros(3),np.eye(3),np.ones(3),1.)
    assert min(row[3] for row in rows)==pytest.approx(np.sqrt(.5),abs=1e-12)
    world.close()
