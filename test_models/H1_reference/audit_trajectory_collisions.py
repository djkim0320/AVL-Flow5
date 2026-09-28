"""Independent OpenCASCADE solid intersections of SAVED states, no motion edits.

Checks the delivered CAD, including the shell, against the complete sensor solids
and cable capsules (cylinders plus endpoint spheres). Only legacy drum-centre
feeds exclude the drum.
This is a sampled geometric audit, not continuous collision detection or a force
model. Run in separate processes because OpenCASCADE operations are CPU bound.
"""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
import json
import hashlib
import time
from pathlib import Path
import numpy as np
from dbf_stability.analysis import load_result
from dbf_stability.math3d import rotation
from dbf_stability.door import door_hinge, door_reference_shift
from geometry_paths import cad_directory
from compute_resources import acquire_worker, audit_budget

HERE = Path(__file__).resolve().parent


def init_worker(run):
    global CQ, RESULT, SENSOR, OBSTACLES, OBSTACLE_BOUNDS, HINGE, SWEPT_BOUNDS
    acquire_worker()
    import cadquery as cq
    CQ = cq
    RESULT = load_result(run, apply_geometry_audit=False)
    shapes = {f.stem: cq.importers.importStep(str(f)).val()
              for f in cad_directory(RESULT.config).glob('*.step') if f.stem != 'H1_A_temporary_assembly'}
    sensor_names = [n for n in shapes if n == 'sensor_body' or n.startswith('sensor_fin_')]
    SENSOR = cq.Compound.makeCompound([shapes[n] for n in sensor_names]).translate(
        tuple(-1000 * np.array(RESULT.config['bay']['stowed_center_m'])))
    OBSTACLES = {n: s for n, s in shapes.items()
                 if n not in sensor_names and n not in ('sensor_tow_point', 'aircraft_tow_point')}
    # Exact broad-phase crop: retain every bit of shell material inside a box
    # enclosing ALL saved sensor/cable poses, with a 2 mm margin. This removes
    # distant B-spline surfaces from costly CAD distance queries. Each queried
    # solid is checked against that bound below; no nearby wall is discarded.
    from scipy.spatial.transform import Rotation
    states=RESULT.states
    ra=Rotation.from_quat(states[:,[7,8,9,6]]).as_matrix()
    rs=Rotation.from_quat(states[:,[20,21,22,19]]).as_matrix()
    relative=np.einsum('nji,njk->nik',ra,rs)
    centers=np.einsum('nji,nj->ni',ra,states[:,13:16]-states[:,:3])*1000
    slo,shi=bounds(SENSOR)
    corners=np.array([[x,y,z] for x in (slo[0],shi[0]) for y in (slo[1],shi[1]) for z in (slo[2],shi[2])])
    lo=np.full(3,np.inf);hi=np.full(3,-np.inf)
    for start in range(0,len(states),1000):
        stop=min(start+1000,len(states))
        points=np.einsum('nij,vj->nvi',relative[start:stop],corners)+centers[start:stop,None,:]
        lo=np.minimum(lo,points.min(axis=(0,1)));hi=np.maximum(hi,points.max(axis=(0,1)))
        nose=centers[start:stop]+np.einsum('nij,j->ni',relative[start:stop],1000*np.asarray(RESULT.config['sensor']['tow_point_m']))
        nodes=states[start:stop,26:].reshape(stop-start,-1,6)[:,:,:3]-states[start:stop,None,:3]
        nodes=np.einsum('nji,nkj->nki',ra[start:stop],nodes)*1000
        radius=RESULT.config['cable']['diameter_m']*500
        lo=np.minimum(lo,nose.min(0)-radius);hi=np.maximum(hi,nose.max(0)+radius)
        lo=np.minimum(lo,nodes.min(axis=(0,1))-radius);hi=np.maximum(hi,nodes.max(axis=(0,1))+radius)
    feed=1000*np.asarray(RESULT.config['aircraft']['tow_point_m'])
    lo=np.minimum(lo,feed-radius)-2;hi=np.maximum(hi,feed+radius)+2
    SWEPT_BOUNDS=(lo,hi)
    crop=cq.Solid.makeBox(*map(float,hi-lo),cq.Vector(*lo))
    for name in list(OBSTACLES):
        if name.startswith('fuselage'):
            OBSTACLES[name]=OBSTACLES[name].intersect(crop)
    b = RESULT.config['bay']
    HINGE = door_hinge(b) * 1000
    OBSTACLES['rear_door_100deg'] = OBSTACLES['rear_door_100deg'].translate(tuple(1000*door_reference_shift(b)))
    OBSTACLE_BOUNDS = {n: bounds(s) for n, s in OBSTACLES.items()}
    door_geometry.cache_clear()


def bounds(shape):
    b = shape.BoundingBox()
    return np.array([b.xmin, b.ymin, b.zmin]), np.array([b.xmax, b.ymax, b.zmax])


@lru_cache(maxsize=64)
def door_geometry(angle):
    shape=OBSTACLES['rear_door_100deg'].rotate(
        tuple(HINGE), tuple(HINGE+[0,1,0]), angle)
    return shape,bounds(shape)


def audit_sample(i):
    r = RESULT
    y = r.states[i]
    ra, rs = rotation(y[6:10]), rotation(y[19:23])
    center = ra.T @ (y[13:16] - y[:3])
    from scipy.spatial.transform import Rotation
    rotvec = Rotation.from_matrix(ra.T @ rs).as_rotvec()
    angle = np.linalg.norm(rotvec)
    sensor = SENSOR.rotate((0, 0, 0), tuple(rotvec / angle), np.rad2deg(angle)) if angle > 1e-12 else SENSOR
    sensor = sensor.translate(tuple(center * 1000))
    door_angle = float(r.table.door_deg.iloc[i]) - 100
    obstacles = dict(OBSTACLES)
    door,door_bounds=door_geometry(door_angle)
    obstacles['rear_door_100deg'] = door
    obstacle_bounds = dict(OBSTACLE_BOUNDS)
    obstacle_bounds['rear_door_100deg'] = door_bounds
    nose = center + ra.T @ rs @ np.array(r.config['sensor']['tow_point_m'])
    k = int(r.table.active_nodes.iloc[i])
    nodes = (y[26:].reshape(-1, 6)[:k, :3] - y[:3]) @ ra
    chain = np.vstack([nose, nodes, r.config['aircraft']['tow_point_m']]) * 1000
    bodies = [('sensor', sensor)]
    radius = r.config['cable']['diameter_m'] * 500
    for j, (p, q) in enumerate(zip(chain[:-1], chain[1:])):
        d = q - p
        length = np.linalg.norm(d)
        if length > 1e-6:
            bodies.append((f'cable_span_{j}', CQ.Solid.makeCylinder(
                radius, length, CQ.Vector(*p), CQ.Vector(*(d / length)))))
    # Match the capsule envelope used by the dynamics, including bends and
    # span endpoints. Flat-ended cylinders alone can miss a node-wall overlap.
    for j,p in enumerate(chain):
        bodies.append((f'cable_node_{j}',CQ.Solid.makeSphere(radius,CQ.Vector(*p),angleDegrees1=-90,angleDegrees2=90)))
    hits, errors = [], []
    for moving, shape in bodies:
        lo, hi = bounds(shape)
        if np.any(lo<SWEPT_BOUNDS[0]-1e-6) or np.any(hi>SWEPT_BOUNDS[1]+1e-6):
            raise ValueError('Moving CAD outside conservative shell crop; refusing incomplete audit')
        for fixed, obstacle in obstacles.items():
            # Cable terminates at the drum centre in this legacy configuration.
            if (moving.startswith('cable') and fixed == 'winch_drum'
                    and r.config['winch'].get('line_attaches_to_drum_center', True)):
                continue
            olo, ohi = obstacle_bounds[fixed]
            if np.any(hi < olo - 1e-6) or np.any(ohi < lo - 1e-6):
                continue
            # The overlap volume is the actual criterion. An extra global
            # NURBS minimum-distance solve is costly and cannot improve it.
            try:
                overlap = shape.intersect(obstacle)
                volume = overlap.Volume()
            except (ValueError, RuntimeError) as exc:
                # CAD boolean failures are unknown, never a collision-free result.
                errors.append({'moving': moving, 'fixed': fixed, 'error': str(exc)})
                continue
            if volume > 1e-5:  # mm^3; exclude mathematical touching.
                p = overlap.Center()
                hits.append({'moving': moving, 'fixed': fixed,
                             'overlap_mm3': float(volume),
                             'point_frd_m': [p.x / 1000, p.y / 1000, p.z / 1000]})
    return {'index': int(i), 'time_s': float(r.time[i]), 'hits': hits, 'errors': errors}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--stride', type=int, default=10)
    p.add_argument('--time-step', type=float, help='Check regular time bins plus their force/clearance extrema and every replay frame')
    p.add_argument('--workers', type=int, default=6, help='Legacy request; active compute_resources.json controls the shared budget')
    p.add_argument('--verbose', action='store_true')
    args = p.parse_args()
    result = load_result(args.run, apply_geometry_audit=False)
    resources = audit_budget(result.states.nbytes, args.workers)
    (args.run / 'compute_resources.json').write_text(json.dumps(resources, indent=2), encoding='utf8')
    print(json.dumps({'compute_resources': resources}), flush=True)
    if args.time_step is not None:
        if args.time_step<=0:raise ValueError('time-step must be positive')
        chosen={0,len(result.time)-1}
        def nearest(targets):
            right=np.minimum(np.searchsorted(result.time,targets),len(result.time)-1)
            left=np.maximum(right-1,0)
            return np.where(abs(result.time[right]-targets)<abs(result.time[left]-targets),right,left)
        chosen.update(map(int,nearest(np.arange(result.time[0],result.time[-1],args.time_step))))
        from replay_sampling import replay_indices
        chosen.update(map(int,replay_indices(result)))
        bins=np.floor((result.time-result.time[0])/args.time_step).astype(int)
        for _,frame in result.table.groupby(bins):
            chosen.update([int(frame.contact_N.idxmax()),int(frame.tension_N.idxmax()),int(frame.minimum_mesh_gap_m.idxmin())])
        for event in result.events:
            if 'time_s' in event:
                at=int(nearest([event['time_s']])[0]);chosen.update([max(0,at-1),at,min(len(result.time)-1,at+1)])
        from capture_check import capture_readiness
        chosen.update(capture_readiness(result)['indices'])
        indices=sorted(chosen)
        selection='Regular time bins, each bin maximum force/tension and minimum mesh gap, event neighbours, capture readiness minima, every rendered replay frame'
    else:
        indices = sorted(set(range(0, len(result.time), args.stride)) | {len(result.time)-1})
        selection='Every saved state' if args.stride==1 else 'Fixed saved-row stride and final state'
    rows = []
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=resources['workers'], initializer=init_worker,
                             initargs=(str(args.run),)) as pool:
        for row in pool.map(audit_sample, indices):
            rows.append(row)
            if len(rows)==1 or len(rows)%50==0 or len(rows)==len(indices):
                progress=dict(completed_samples=len(rows),total_samples=len(indices),
                              elapsed_s=time.perf_counter()-started,last_time_s=row['time_s'],workers=resources['workers'])
                (args.run/'cad_audit_progress.json').write_text(json.dumps(progress),encoding='utf8')
            if row['hits'] and args.verbose:
                print(json.dumps(row), flush=True)
            elif len(rows) % 50 == 0:
                print(f'Audited {len(rows)}/{len(indices)} saved states', flush=True)
    hits = [r for r in rows if r['hits']]
    errors = [r for r in rows if r['errors']]
    report = {'method': 'OpenCASCADE intersections of delivered STEP solids; cable capsules as cylinders and endpoint spheres',
              'compute_resources': resources, 'elapsed_s': time.perf_counter()-started,
              'source_run': str(args.run.resolve()), 'samples': len(rows), 'stride': args.stride if args.time_step is None else None,
              'saved_states':len(result.time),'time_step_s':args.time_step,'sample_selection':selection,
              'every_saved_state_checked':len(rows)==len(result.time),
              'source_states_sha256': hashlib.sha256((args.run / 'states.npz').read_bytes()).hexdigest(),
              'source_inputs_sha256': hashlib.sha256((args.run / 'inputs.json').read_bytes()).hexdigest(),
              'audit_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'door_transform_sha256': hashlib.sha256((HERE.parents[1]/'src/dbf_stability/door.py').read_bytes()).hexdigest(),
              'cad_sha256': {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                             for f in sorted(cad_directory(result.config).glob('*.step'))},
              'cad_directory': str(cad_directory(result.config)),
              'continuous_collision_detection': False,
              'shell_query_optimization':'Exact STEP intersection with all-saved-pose swept bounding box plus 2 mm; each moving solid checked inside that bound',
              'intentional_exclusion': ('Cable inside winch drum; visual tow-point markers.'
                  if result.config['winch'].get('line_attaches_to_drum_center', True)
                  else 'Visual tow-point markers only; drum and fairlead included.'),
              'intersecting_samples': len(hits), 'first_sampled_intersection': hits[0] if hits else None,
              'geometry_validity': 'invalid' if hits else 'audit_incomplete' if errors else 'no_overlap_at_checked_samples',
              'cad_query_error_samples': len(errors),
              'rows': rows}
    (args.run / 'cad_collision_audit.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}), flush=True)


if __name__ == '__main__':
    main()
