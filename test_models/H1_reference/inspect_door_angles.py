"""Read-only door clearance checks; saved sensor poses are not new trajectories."""
import os
for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
from dbf_stability.analysis import load_result
from dbf_stability.collision import MeshContacts
from dbf_stability.math3d import rotation

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'runs/contact_corrected_06/baseline/mission'
OUT=HERE/'diagnostics/door_angles'


def inspect_angle(angle):
    r=load_result(SOURCE);world=MeshContacts(r.config)
    world.obstacles=[o for o in world.obstacles if o['door']]
    rows=[]
    for i in np.flatnonzero((r.time<=1.3)|(r.time>=5.3)):
        y=r.states[i];ra=rotation(y[6:10]);rs=rotation(y[19:23])
        center=ra.T@(y[13:16]-y[:3])
        # Query only the door/body pair, retaining all sensor parts.
        hits=world.contacts(center,ra.T@rs,np.empty((0,3)),angle,distance=.15)
        closest=min(hits,key=lambda h:h.get('geometry_gap',h['gap'])) if hits else None
        rows.append({'time_s':float(r.time[i]),'sensor_door_gap_m':float(closest.get('geometry_gap',closest['gap'])) if closest else None,
                     'part':closest['moving'] if closest else None})
    world.close()
    import cadquery as cq
    door=cq.importers.importStep(str(HERE/'cad/rear_door_100deg.step')).val()
    b=r.config['bay'];hinge=np.array([b['exit_x_m'],0,b['floor_z_m']])*1000
    plate=door.rotate(tuple(hinge),tuple(hinge+[0,1,0]),angle-100)
    overlaps=[];errors=[]
    bounds=plate.BoundingBox()
    for p in (HERE/'cad').glob('*.step'):
        if p.stem.startswith('sensor') or p.stem in ('rear_door_100deg','H1_A_temporary_assembly','aircraft_tow_point'):
            continue
        other=cq.importers.importStep(str(p)).val();bb=other.BoundingBox()
        if any(getattr(bounds,axis+'max')<getattr(bb,axis+'min') or getattr(bounds,axis+'min')>getattr(bb,axis+'max') for axis in 'xyz'):
            continue
        try:
            if plate.distance(other)>1e-5:continue
            overlap=plate.intersect(other);volume=overlap.Volume()
            if volume>1e-5:
                q=overlap.Center();overlaps.append({'part':p.stem,'volume_mm3':volume,'center_frd_m':[q.x/1000,q.y/1000,q.z/1000]})
        except (ValueError,RuntimeError) as e:errors.append({'part':p.stem,'error':str(e)})
    recovery=[x for x in rows if x['time_s']>=5.3 and x['sensor_door_gap_m'] is not None]
    engaged=[x for x in recovery if x['sensor_door_gap_m']<r.config['collision']['skin_m']]
    return str(angle),{'angle_deg':angle,'saved_pose_comparison_only':True,
        'minimum_recovery_sensor_door_gap_m':min(x['sensor_door_gap_m'] for x in recovery),
        'first_recovery_contact_skin_entry':engaged[0] if engaged else None,
        'fixed_aircraft_overlaps':overlaps,'cad_errors':errors,'rows':rows}


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(max_workers=4) as pool:
        results=dict(pool.map(inspect_angle,[100,140,160,180]))
    (OUT/'geometry_comparison.json').write_text(json.dumps(results,indent=2),encoding='utf8')
    print(json.dumps({k:{a:b for a,b in v.items() if a!='rows'} for k,v in results.items()},indent=2))
