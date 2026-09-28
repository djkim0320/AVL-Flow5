"""Actual R3 feature-force equivalence and timing; no surrogate collision data."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json,time
import numpy as np
from dbf_stability import contact_features as fast
from dbf_stability import load_case
from dbf_stability import collision
from dbf_stability.math3d import rotation

ROOT=Path(__file__).resolve().parents[2]


def main():
    c=load_case(ROOT/'examples/h1_recovery_repair_round_nominal.yaml');m=collision.MeshContacts(c)
    with np.load(ROOT/'tests/data/recovery_contact_onset.npz') as data:y=data['round_nominal_state'].copy()
    ra=rotation(y[6:10]);rel=ra.T@rotation(y[19:23]);center=ra.T@(y[13:16]-y[:3])
    part=next(p for p in m.sensor if p['name']=='sensor_body');vertices=part['vertices']@rel.T+center
    jobs=[]
    for name in ['rear_exit_frame_floor','rear_door_100deg']:
        obs=next(p for p in m.obstacles if p['name']==name)
        args=(vertices,part['faces'],part['edges'],*m._box_geometry(obs,140.),m.skin)
        jobs.append((name,collision.mesh_box_manifold,fast.mesh_box_manifold,args))
    obs=next(p for p in m.obstacles if p['name'].startswith('fuselage'))
    select=np.all(obs['triangle_hi']>=vertices.min(0)-.001,axis=1)&np.all(obs['triangle_lo']<=vertices.max(0)+.001,axis=1)
    faces=obs['faces'][select];indices,inverse=np.unique(faces,return_inverse=True)
    vb=obs['vertices'][indices];fb=inverse.reshape(-1,3);eb=collision.polyhedron_edges(vb,fb)
    jobs.append(('shell',collision.mesh_mesh_manifold,fast.mesh_mesh_manifold,(vertices,part['faces'],part['edges'],vb,fb,eb,m.skin)))
    def load(rows):
        f=np.zeros(3);torque=np.zeros(3)
        for x,other,n,g,w in rows:
            normal,friction=collision.barrier_force(g,-.1,np.array([.01,.02,.03]),m.skin,20000.,8.,.2)
            force=w*(normal*n+friction);f+=force;torque+=np.cross(x-center,force)
        return np.r_[f,torque]
    report=[]
    try:
        for name,original,compiled,args in jobs:
            difference=0.;checked=0
            for axis in range(3):
                for delta in [-1e-4,-1e-8,0.,1e-8,1e-4,.003]:
                    va=args[0].copy();va[:,axis]+=delta;data=(va,)+args[1:]
                    before=original(*data);after=compiled(*data)
                    error=float(np.max(abs(load(before)-load(after))))
                    if not np.allclose(load(before),load(after),atol=2e-8,rtol=2e-7):raise AssertionError((name,axis,delta,error,load(before),load(after)))
                    difference=max(difference,error);checked+=1
            seconds=[]
            for function in (original,compiled):
                start=time.perf_counter()
                for _ in range(10):function(*args)
                seconds.append((time.perf_counter()-start)/10)
            report.append(dict(pair=name,cases=checked,max_force_moment_absolute_difference=difference,numpy_seconds=seconds[0],compiled_seconds=seconds[1],speedup=seconds[0]/seconds[1]))
            print(json.dumps(report[-1]),flush=True)
    finally:m.close()
    output=ROOT/'outputs/validation/contact_feature_acceleration.json'
    output.write_text(json.dumps(dict(no_fastmath=True,feature_sampling=False,rows=report),indent=2),encoding='utf8')


if __name__=='__main__':main()
