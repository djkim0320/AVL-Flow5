"""Independent STEP audit of selected immutable accepted deployment states."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
import numpy as np
import pandas as pd
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--interval',type=float,default=.05);p.add_argument('--workers',type=int,default=2)
    p.add_argument('--replay',action='store_true',help='Render selected real states; no full-resolution load/impulse claim')
    args=p.parse_args();run=args.run.resolve();out=args.out.resolve()
    if args.interval<=0:raise ValueError('Positive sampling interval required')
    sys.path.insert(0,str(run/'solver'))
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    c=json.loads((run/'mission/inputs.json').read_text(encoding='utf8'))
    source=json.loads((run/'source.json').read_text(encoding='utf8'))
    chunks=sorted((run/'mission/accepted_history').glob('part_*.npz'))
    rows=[];sources=[]
    for chunk in chunks:
        with np.load(chunk,allow_pickle=False) as d:
            meta=json.loads(str(d['metadata']))
            rows.extend((float(t),y.copy(),bool(cap),meta['active_nodes'],meta['capture_time_s'])
                for t,y,cap in zip(d['time'],d['states'],d['captured']))
        sources.append(dict(file=str(chunk),sha256=hashlib.sha256(chunk.read_bytes()).hexdigest()))
    times=np.array([row[0] for row in rows]);chosen={0,len(rows)-1}
    for target in np.r_[np.arange(times[0],times[-1],args.interval),c['winch']['release_s'],.4]:
        if times[0]<=target<=times[-1]:chosen.add(int(np.argmin(abs(times-target))))
    indices=sorted(chosen)
    model=CoupledModel(c,AeroDatabase(source['aero_path']),meta['trim']);table=[]
    for i in indices:
        t,y,captured,k,capture=rows[i];model.captured=captured;model.capture_time=capture
        model.active_override=k
        if args.replay:
            from dbf_stability.math3d import euler
            _,row=model.rhs(t,y,True)
            for j,name in enumerate(('roll_deg','pitch_deg','yaw_deg')):
                row[name]=float(np.rad2deg(euler(y[6:10])[j]));row['sensor_'+name]=float(np.rad2deg(euler(y[19:23])[j]))
            row.update(time_s=t,altitude_m=float(-y[2]),sensor_altitude_m=float(-y[15]))
        else:
            row=dict(time_s=t,door_deg=model.door_state(t)[0],active_nodes=model.active_count(model.length(t)[0]) if k is None else k,captured=captured)
        table.append(row)
    out.mkdir(parents=True,exist_ok=False)
    np.savez_compressed(out/'states.npz',time=times[indices],states=np.array([rows[i][1] for i in indices]))
    pd.DataFrame(table).to_csv(out/'timeseries.csv',index=False)
    (out/'inputs.json').write_text(json.dumps(c,ensure_ascii=False,indent=2),encoding='utf8')
    (out/'events.json').write_text('[]',encoding='utf8')
    (out/'summary.json').write_text(json.dumps(dict(status='partial_snapshot' if args.replay else 'geometry_snapshot',captured=bool(table[-1]['captured']),
        scope='저장된 자세 중 선택한 구간만 재생합니다. 포획·문 닫힘과 전체 임무 완료를 뜻하지 않습니다. 표본별 하중은 전체 구간 최댓값이나 한도 판정의 근거로 쓰지 않습니다.',
        start_time_s=float(times[0]),end_time_s=float(times[-1]),original_saved_states=len(rows),
        requested_duration_s=c['mission_profile']['end_s']-float(times[0]),final_door_deg=float(table[-1]['door_deg']),
        aero_metadata=model.aero.metadata,mesh_contact_enabled=True,numerically_converged=False,
        physical_validation='unvalidated_assumption_case',selected_states=len(indices),nominal_sample_interval_s=args.interval,source_chunks=sources),indent=2),encoding='utf8')
    subprocess.run([sys.executable,str(HERE/'audit_trajectory_collisions.py'),str(out),'--stride','1','--workers',str(args.workers)],cwd=ROOT,check=True)
    audit=json.loads((out/'cad_collision_audit.json').read_text(encoding='utf8'))
    if args.replay:
        subprocess.run([sys.executable,str(HERE/'visualize_run.py'),'--run',str(out)],cwd=ROOT,check=True)
    if model._mesh_contacts:model._mesh_contacts.close()
    print(json.dumps(dict(start_s=float(times[0]),end_s=float(times[-1]),samples=audit['samples'],
        overlap_samples=audit['intersecting_samples'],query_error_samples=audit['cad_query_error_samples'])),flush=True)


if __name__=='__main__':main()
