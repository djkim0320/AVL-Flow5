"""Run explicitly with the isolated analytic solver package on sys.path."""
from pathlib import Path
import json
import numpy as np
import cadquery as cq
from scipy.spatial.transform import Rotation
from dbf_stability import load_case
from dbf_stability.analytic_contact import sphere_cone_contacts
from dbf_stability.collision import barrier_force
from dbf_stability.math3d import rotation
from analytic_stop_probe import cone_spec

ROOT=Path(__file__).resolve().parents[2]


def fixtures():
    c=load_case(ROOT/'examples/h1_round_x_capture_stop_v2.yaml')
    with np.load(ROOT/'tests/data/capture_stop_contact.npz',allow_pickle=False) as d:ys=d['states'].copy()
    return c,ys


def test_exact_gap_matches_independent_cad_at_perturbed_recorded_poses():
    c,states=fixtures();cad=(ROOT/c['collision']['mesh_directory']).parent/'cad'
    body=cq.importers.importStep(str(cad/'sensor_body.step')).val().translate(tuple(-1000*np.asarray(c['bay']['stowed_center_m'])))
    stop=cq.importers.importStep(str(cad/'capture_front_stop.step')).val();checked=0
    for y in states:
        ra,rs=rotation(y[6:10]),rotation(y[19:23]);r=ra.T@rs;center=ra.T@(y[13:16]-y[:3])
        rv=Rotation.from_matrix(r).as_rotvec();angle=np.linalg.norm(rv);axis=rv/angle
        rotated=body.rotate((0,0,0),tuple(axis),float(np.rad2deg(angle)))
        for offset in [np.zeros(3),np.array([0,.0001,0]),np.array([0,0,-.0001])]:
            position=center+offset;manifold=sphere_cone_contacts(position,r,cone_spec(c),c['collision']['skin_m'])
            if manifold is None:continue
            cad_gap=rotated.translate(tuple(position*1000)).distance(stop)*.001
            assert abs(cad_gap-manifold['minimum_gap'])<1e-8
            checked+=1
    assert checked>=10


def contact_load(c,y,order):
    ra,rs=rotation(y[6:10]),rotation(y[19:23]);r=ra.T@rs;center=ra.T@(y[13:16]-y[:3])
    manifold=sphere_cone_contacts(center,r,cone_spec(c,order),c['collision']['skin_m']);assert manifold is not None
    force=np.zeros(3);moment=np.zeros(3)
    for h in manifold['contacts']:
        v=ra.T@(y[16:19]-y[3:6])+np.cross(ra.T@rs@y[23:26],h['point']-center)-np.cross(y[10:13],h['point'])
        vn=v@h['normal'];fn,ft=barrier_force(h['gap'],vn,v-vn*h['normal'],c['collision']['skin_m'],20000.,80.,.1)
        f=h['weight']*(fn*h['normal']+ft);force+=f;moment+=np.cross(h['point']-center,f)
    return force,moment


def test_angular_quadrature_256_to_512_changes_loads_under_one_percent():
    c,states=fixtures()
    for y in states:
        coarse=contact_load(c,y,256);fine=contact_load(c,y,512)
        for a,b in zip(coarse,fine):assert np.linalg.norm(a-b)<.01*max(np.linalg.norm(b),1e-6)


def test_axisymmetric_contact_has_no_artificial_side_force_or_moment():
    c,_=fixtures();center=np.array(c['bay']['stowed_center_m'])+[.0003,0,0]
    spec=cone_spec(c,256);manifold=sphere_cone_contacts(center,np.eye(3),spec,c['collision']['skin_m'])
    assert manifold is not None and len(manifold['contacts'])==256
    force=np.zeros(3);moment=np.zeros(3)
    for h in manifold['contacts']:
        fn,_=barrier_force(h['gap'],0.,np.zeros(3),c['collision']['skin_m'],20000.,80.,.1)
        f=fn*h['normal']*h['weight'];force+=f;moment+=np.cross(h['point']-center,f)
    assert force[0]<0
    np.testing.assert_allclose(force[1:],0,atol=1e-12);np.testing.assert_allclose(moment,0,atol=1e-12)
    assert sphere_cone_contacts(center,Rotation.from_euler('y',45,degrees=True).as_matrix(),spec,c['collision']['skin_m']) is None


def test_door_sweep_separation_includes_sensor_extent_and_every_cable_span():
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    from dbf_stability.contact_integration import separated_from_entire_door_sweep
    c,states=fixtures();y=states[-1].copy();time=71.03697797541403
    aero=AeroDatabase(ROOT/'test_models/H1_reference/runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    model=CoupledModel(c,aero);model.contact_geometry(time,y,distance=.01)
    try:
        assert separated_from_entire_door_sweep(model,time,y,.0002)
        geometry=model._mesh_contacts;plate=next(p for p in geometry.obstacles if p['door'])
        radius=np.linalg.norm(plate['vertices'][:,[0,2]],axis=1).max();door_x=geometry.hinge[0]+radius
        ra,rs=rotation(y[6:10]),rotation(y[19:23]);relative=ra.T@rs;center=ra.T@(y[13:16]-y[:3])
        sensor_min=min((p['vertices']@relative[0]).min()+center[0] for p in geometry.sensor)
        assert sensor_min-door_x>.0002
        nearby=y.copy();nearby[13:16]+=ra@np.array([door_x+.0001-sensor_min,0.,0.])
        assert not separated_from_entire_door_sweep(model,time,nearby,.0002)
        cable=y.copy();assert model.active_count(model.length(time)[0])>0
        cable[26:29]=y[:3]+ra@np.array([door_x-.01,0.,geometry.hinge[2]])
        assert not separated_from_entire_door_sweep(model,time,cable,.0002)
    finally:model._mesh_contacts.close()


def test_widened_bore_curved_clearance_matches_independent_step():
    c=load_case(ROOT/'examples/h1_round_x_capture_stop_v3.yaml')
    cad=(ROOT/c['collision']['mesh_directory']).parent/'cad'
    body=cq.importers.importStep(str(cad/'sensor_body.step')).val().translate(tuple(-1000*np.asarray(c['bay']['stowed_center_m'])))
    stop=cq.importers.importStep(str(cad/'capture_front_stop.step')).val()
    checked=0
    for pitch,yaw in [(0.,0.),(.5,0.),(-.5,.5)]:
        orientation=Rotation.from_euler('zy',[yaw,pitch],degrees=True)
        rv=orientation.as_rotvec();angle=np.linalg.norm(rv)
        rotated=body if angle==0 else body.rotate((0,0,0),tuple(rv/angle),float(np.rad2deg(angle)))
        for offset in [[0,0,0],[.0002,0,0],[0,0,.0001]]:
            center=np.array(c['bay']['stowed_center_m'])+offset
            manifold=sphere_cone_contacts(center,orientation.as_matrix(),cone_spec(c),c['collision']['skin_m'])
            assert manifold is not None
            actual=rotated.translate(tuple(1000*center)).distance(stop)*.001
            assert actual>0 and abs(actual-manifold['minimum_gap'])<1e-8
            checked+=1
    assert checked==9
