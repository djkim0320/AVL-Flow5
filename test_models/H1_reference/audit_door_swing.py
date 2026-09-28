"""Sampled actual-CAD door/fixed-aircraft intersections, separate from dynamics."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse
import json
import hashlib
import numpy as np
from dbf_stability import load_case
from dbf_stability.door import door_hinge, door_reference_shift
from geometry_paths import cad_directory

HERE=Path(__file__).resolve().parent


def init_worker(offset,case_path):
    global DOOR, FIXED, HINGE
    import cadquery as cq
    c=load_case(case_path);b=c['bay']
    b['door_hinge_offset_m']=offset
    b['door_closed_offset_m']=b.get('door_closed_offset_m',[-.003,0.,0.])
    DOOR=cq.importers.importStep(str(cad_directory(c)/'rear_door_100deg.step')).val().translate(tuple(1000*door_reference_shift(b)))
    HINGE=1000*door_hinge(b)
    FIXED={p.stem:cq.importers.importStep(str(p)).val() for p in cad_directory(c).glob('*.step')
           if not p.stem.startswith('sensor') and p.stem not in ('rear_door_100deg','H1_A_temporary_assembly','aircraft_tow_point')}
    # Exact broad phase: trim only shell material outside a box enclosing every
    # possible door rotation. No potential overlapping material is removed.
    bb=DOOR.BoundingBox()
    corners=np.array([[x,y,z] for x in (bb.xmin,bb.xmax) for y in (bb.ymin,bb.ymax) for z in (bb.zmin,bb.zmax)])
    radius=float(np.linalg.norm((corners-HINGE)[:,[0,2]],axis=1).max())+1.
    low=HINGE+[-radius,-1000*b['half_width_m']-2.,-radius]
    extent=[2*radius,2000*b['half_width_m']+4.,2*radius]
    region=cq.Solid.makeBox(*extent,cq.Vector(*low))
    for name,shape in list(FIXED.items()):
        if name.startswith('fuselage'):
            cropped=shape.intersect(region)
            if cropped.Volume()>1e-8: FIXED[name]=cropped
            else: del FIXED[name]


def inspect(angle):
    plate=DOOR.rotate(tuple(HINGE),tuple(HINGE+[0,1,0]),float(angle)-100)
    bb=plate.BoundingBox();hits=[];errors=[]
    for name,other in FIXED.items():
        ob=other.BoundingBox()
        if any(getattr(bb,a+'max')<getattr(ob,a+'min')-1e-6 or getattr(bb,a+'min')>getattr(ob,a+'max')+1e-6 for a in 'xyz'):continue
        try:
            if plate.distance(other)>1e-5:continue
            volume=plate.intersect(other).Volume()
            if volume>1e-5:hits.append({'part':name,'overlap_mm3':float(volume)})
        except (ValueError,RuntimeError) as e:errors.append({'part':name,'error':str(e)})
    return {'angle_deg':float(angle),'hits':hits,'errors':errors}


def main():
    p=argparse.ArgumentParser();p.add_argument('--offset',nargs=3,type=float,default=[-.003,0,.003]);p.add_argument('--step',type=float,default=1.)
    p.add_argument('--case',type=Path,default=HERE.parents[1]/'examples/h1_reference.yaml')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=4);args=p.parse_args()
    angles=np.arange(0,180+args.step/2,args.step)
    c=load_case(args.case)
    with ProcessPoolExecutor(max_workers=args.workers,initializer=init_worker,initargs=(args.offset,str(args.case))) as pool:
        rows=list(pool.map(inspect,angles))
    result={'door_hinge_offset_frd_m':args.offset,'door_closed_offset_frd_m':[-.003,0.,0.],'angle_step_deg':args.step,'sampled_angles':len(rows),
            'intersecting_angles':sum(bool(r['hits']) for r in rows),'error_angles':sum(bool(r['errors']) for r in rows),
            'scope':'Sampled door leaf vs fixed aircraft CAD; shell broad phase is an exact CAD crop enclosing the full door sweep. No hinge bracket, actuator, or continuous sweep proof.',
            'audit_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'door_transform_sha256':hashlib.sha256((HERE.parents[1]/'src/dbf_stability/door.py').read_bytes()).hexdigest(),
            'case':str(args.case.resolve()),'cad_directory':str(cad_directory(c)),
            'cad_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in cad_directory(c).glob('*.step')},'rows':rows}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','cad_sha256')},indent=2))
    for row in rows:
        if row['hits'] or row['errors']:print(row)


if __name__=='__main__':main()
