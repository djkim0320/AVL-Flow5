"""Publish accepted-state diagnostics without opening a live checkpoint."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,json,sys,time
import numpy as np
from compute_resources import apply_affinity


def main():
    apply_affinity()
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--once',action='store_true')
    args=p.parse_args();run=args.run.resolve();sys.path.insert(0,str(run/'solver'))
    from dbf_stability import AeroDatabase
    from dbf_stability.model import CoupledModel
    from dbf_stability.math3d import euler
    source=json.loads((run/'source.json').read_text(encoding='utf8'))
    aero=AeroDatabase(source['aero_path']);model=None;previous=None;directory=None
    try:
        while True:
            pipeline=json.loads((run/'pipeline.json').read_text(encoding='utf8'))
            live=pipeline.get('live_directory')
            if not live:
                if pipeline['stage'] in ('completed','partial','failed'):break
                time.sleep(5);continue
            folder=run/live;chunks=sorted((folder/'accepted_history').glob('part_*.npz'))
            if chunks and chunks[-1]!=previous:
                chunk=chunks[-1]
                with np.load(chunk,allow_pickle=False) as d:
                    t=float(d['time'][-1]);y=d['states'][-1].copy();meta=json.loads(str(d['metadata']))
                if directory!=folder:
                    if model is not None and model._mesh_contacts is not None:model._mesh_contacts.close()
                    c=json.loads((folder/'inputs.json').read_text(encoding='utf8'))
                    model=CoupledModel(c,aero,meta['trim']);directory=folder
                model.active_override=meta['active_nodes'];model.captured=meta['captured'];model.capture_time=meta['capture_time_s']
                _,d=model.rhs(t,y,True)
                result={key:d[key] for key in ('phase','length_m','tension_N','torque_Nm','contact_N','airspeed_m_s','alpha_deg','beta_deg','elevator_deg','thrust_N','door_deg','sensor_local_x_m','sensor_local_y_m','sensor_local_z_m','minimum_mesh_gap_m','captured')}
                result.update(time_s=t,altitude_m=-float(y[2]),pitch_deg=float(np.rad2deg(euler(y[6:10])[1])),
                              source=str(chunk.relative_to(run)),sample_kind='latest accepted state; these are not trajectory maxima')
                tmp=run/'live_metrics.tmp';tmp.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8');tmp.replace(run/'live_metrics.json')
                print(json.dumps(result),flush=True);previous=chunk
            if args.once:break
            time.sleep(15)
    finally:
        if model is not None and model._mesh_contacts is not None:model._mesh_contacts.close()


if __name__=='__main__':main()
