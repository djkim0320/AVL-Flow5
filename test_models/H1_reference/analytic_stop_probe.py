"""Exact spherical-cap/cone support planes, with angular force quadrature.

The finite cone and hemispherical support domains are explicitly checked.
Other finite edges, rear faces and cable contacts require the generic mesh.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import json
from pathlib import Path
import numpy as np


def cone_spec(c,order=128):
    record=json.loads(((Path(c['_root'])/c['collision']['mesh_directory']).parent/'geometry.json').read_text(encoding='utf8'))
    tip=np.asarray(c['bay']['stowed_center_m'])+np.asarray(c['sensor']['tow_point_m'])
    front=float(tip[0]+.0015)
    return dict(part='capture_front_stop',nose_radius_m=.014,
        nose_center_sensor_m=(np.asarray(c['sensor']['tow_point_m'])-[.014,0,0]).tolist(),
        back_x_m=front-record['cone_length_mm']*.001,front_x_m=front,
        center_yz_m=tip[1:].tolist(),mouth_radius_m=record['mouth_diameter_mm']*.0005,
        bore_radius_m=record['cable_bore_diameter_mm']*.0005,quadrature_order=order)


def sphere_cone_contacts(center,relative_rotation,spec,skin):
    """Return None outside the analytic domain, otherwise all active witnesses."""
    radius=spec['nose_radius_m'];sphere=center+relative_rotation@np.asarray(spec['nose_center_sensor_m'])
    back,front=spec['back_x_m'],spec['front_x_m'];mouth,bore=spec['mouth_radius_m'],spec['bore_radius_m']
    slope=(mouth-bore)/(front-back);scale=np.sqrt(1+slope*slope)
    radial=sphere[1:]-np.asarray(spec['center_yz_m']);angle=np.arctan2(radial[1],radial[0])
    angles=angle+np.arange(spec['quadrature_order'])*2*np.pi/spec['quadrature_order']
    normals=np.column_stack([np.full(len(angles),-slope),-np.cos(angles),-np.sin(angles)])/scale
    # The sphere must be the support surface for every conical tangent plane.
    # Otherwise part of the cylinder/rear face may be closer.
    if np.min((-normals)@relative_rotation[:,0])<=0:return None
    gaps=(mouth-slope*(sphere[0]-back)-radial[0]*np.cos(angles)-radial[1]*np.sin(angles))/scale-radius
    points=sphere-radius*normals
    others=sphere-(radius+gaps[:,None])*normals
    near=gaps<2*skin
    if not np.any(near):return None
    if np.any(others[near,0]<back) or np.any(others[near,0]>front):return None
    minimum=float(gaps.min())
    weights=np.exp(-np.clip((gaps-minimum)/(.2*skin),0.,700.));weights/=weights.sum()
    return [dict(point=p,obstacle_point=q,normal=n,gap=float(g),weight=float(w),analytic_gap=minimum)
            for p,q,n,g,w in zip(points,others,normals,gaps,weights) if g<skin and w>1e-12]


def main():
    import cadquery as cq
    from dbf_stability import load_case
    from dbf_stability.math3d import rotation
    from dbf_stability.collision import barrier_force
    from scipy.spatial.transform import Rotation
    root=Path(__file__).resolve().parents[2];c=load_case(root/'examples/h1_round_x_capture_stop_v2.yaml')
    cad=(root/c['collision']['mesh_directory']).parent/'cad'
    body=cq.importers.importStep(str(cad/'sensor_body.step')).val().translate(tuple(-1000*np.asarray(c['bay']['stowed_center_m'])))
    stop=cq.importers.importStep(str(cad/'capture_front_stop.step')).val()
    with np.load(root/'tests/data/capture_stop_contact.npz',allow_pickle=False) as d:times,states=d['time'].copy(),d['states'].copy()
    records=[]
    for t,y in zip(times,states):
        ra,rs=rotation(y[6:10]),rotation(y[19:23]);r=ra.T@rs;center=ra.T@(y[13:16]-y[:3])
        rv=Rotation.from_matrix(r).as_rotvec();angle=np.linalg.norm(rv);axis=rv/angle if angle>1e-12 else np.array([1.,0,0])
        moved=body.rotate((0,0,0),tuple(axis),float(np.rad2deg(angle))).translate(tuple(center*1000))
        gap=moved.distance(stop)*.001
        spec=cone_spec(c);sphere=center+r@np.asarray(spec['nose_center_sensor_m']);radial=np.linalg.norm(sphere[1:]-spec['center_yz_m'])
        slope=(spec['mouth_radius_m']-spec['bore_radius_m'])/(spec['front_x_m']-spec['back_x_m'])
        exact=(spec['mouth_radius_m']-slope*(sphere[0]-spec['back_x_m'])-radial)/np.sqrt(1+slope*slope)-spec['nose_radius_m']
        assert abs(exact-gap)<1e-8,(exact,gap)
        forces=[]
        for order in (32,64,128,256):
            witnesses=sphere_cone_contacts(center,r,cone_spec(c,order),c['collision']['skin_m']);assert witnesses is not None
            force=np.zeros(3);moment=np.zeros(3)
            for h in witnesses:
                v=ra.T@(y[16:19]-y[3:6])+np.cross(ra.T@rs@y[23:26],h['point']-center)-np.cross(y[10:13],h['point'])
                vn=v@h['normal'];fn,ft=barrier_force(h['gap'],vn,v-vn*h['normal'],c['collision']['skin_m'],20000.,240.,.1)
                f=h['weight']*(fn*h['normal']+ft);force+=f;moment+=np.cross(h['point']-center,f)
            forces.append(dict(order=order,force_N=force.tolist(),moment_Nm=moment.tolist()))
        records.append(dict(time_s=float(t),CAD_gap_m=gap,analytic_gap_m=exact,forces=forces))
    result=dict(records=records,scope='CAD distance equality and angular quadrature convergence at actual saved poses; not a full trajectory or material validation')
    (root/'outputs/validation/analytic_stop_probe.json').write_text(json.dumps(result,indent=2),encoding='utf8');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
