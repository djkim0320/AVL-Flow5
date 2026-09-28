"""Bound intersample rigid-door travel using exact CAD distance lower bounds.

The existing audit's shell crop removes material at least 1 mm beyond the entire
rotation envelope. Capping every reported gap at 1 mm retains a conservative
distance lower bound even for the cropped-away shell. No pin is excluded.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import audit_door_swing as audit
from assess_underbody_door import initialize
from dbf_stability import load_case
from dbf_stability.door import door_rotation_radius


def distance_at(angle):
    plate=audit.DOOR.rotate(tuple(audit.HINGE),tuple(audit.HINGE+[0,1,0]),float(angle)-100)
    bb=plate.BoundingBox();best=1.;nearest='conservative_shell_crop_boundary';errors=[]
    def lower(other):
        ob=other.BoundingBox()
        return np.linalg.norm([max(0.,getattr(bb,k+'min')-getattr(ob,k+'max'),getattr(ob,k+'min')-getattr(bb,k+'max')) for k in 'xyz'])
    for name,shape in sorted(audit.FIXED.items(),key=lambda item:lower(item[1])):
        if lower(shape)>best+1e-7:continue
        try:
            gap=float(plate.distance(shape))
            if not np.isfinite(gap) or gap<0:raise ValueError('Invalid CAD distance')
            if gap<best:best=gap;nearest=name
        except (ValueError,RuntimeError) as exc:errors.append({'part':name,'error':str(exc)})
    return dict(angle_deg=float(angle),gap_lower_bound_mm=best,nearest=nearest,errors=errors)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--workers',type=int,default=2);p.add_argument('--step',type=float,default=.2)
    args=p.parse_args();c=load_case(args.case);step=args.step
    if step<=0 or not np.isclose(270/step,round(270/step)):raise ValueError('Step must divide 270 degrees')
    angles=np.linspace(0,270,round(270/step)+1)
    with ProcessPoolExecutor(max_workers=args.workers,initializer=initialize,initargs=(str(args.case.resolve()),)) as pool:
        rows=list(pool.map(distance_at,angles,chunksize=5))
    radius_mm=1000*door_rotation_radius(c['bay'])
    # Every intermediate pose is at most half an angular interval from a sample.
    # Rigid rotation moves any leaf point by at most this chord length.
    bound_mm=2*radius_mm*np.sin(np.deg2rad(step)/4)
    tolerance_mm=1e-4
    minimum=min(r['gap_lower_bound_mm'] for r in rows)
    errors=sum(bool(r['errors']) for r in rows)
    suffix=np.minimum.accumulate(np.array([r['gap_lower_bound_mm'] for r in rows])[::-1])[::-1]
    eligible=np.flatnonzero(suffix>bound_mm+tolerance_mm)
    first=int(eligible[0]) if len(eligible) else None
    result=dict(scope='Rigid door vs exported fixed CAD and stowed sensor over the entire 0..270 degree path. Geometric distance plus rigid-motion bound; not actuator, deformation or manufacturing validation.',
        source_case=str(args.case.resolve()),source_case_sha256=hashlib.sha256(args.case.read_bytes()).hexdigest(),
        cad_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(c['_root'])/c['collision']['mesh_directory']).parent.joinpath('cad').glob('*.step')},
        samples=len(rows),step_deg=step,maximum_rotation_radius_mm=radius_mm,
        maximum_intersample_distance_to_nearest_pose_mm=float(bound_mm),cad_numerical_allowance_mm=tolerance_mm,
        minimum_sampled_gap_lower_bound_mm=minimum,certified_path_gap_lower_bound_mm=float(minimum-bound_mm-tolerance_mm),
        passed=bool(not errors and minimum>bound_mm+tolerance_mm),query_error_samples=errors,
        positive_clearance_certified_interval_deg=[rows[first]['angle_deg'],270.] if first is not None and not errors else None,
        interval_explanation='The listed interval has a positive CAD distance minus rigid-motion bound; outside it no positive-clearance certificate is asserted. Manufacturing tolerances and deformation are not included.',rows=rows)
    args.out.write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','cad_sha256')},indent=2),flush=True)
