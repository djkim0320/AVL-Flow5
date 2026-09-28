"""Continue a durable checkpoint from the interrupted span/hold mission."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json
import numpy as np
from dbf_stability import AeroDatabase, resume_simulation
from dbf_stability.plots import history_figure

HERE=Path(__file__).resolve().parent
OUT=HERE/'runs/span_hold_01'

if __name__=='__main__':
    p=OUT/'mission'
    checkpoint=p/'accepted_checkpoint.tmp'
    with np.load(checkpoint) as d:
        start=float(d['time']);state=d['state'].copy()
    # The checkpoint file was complete before Windows rejected its replacement.
    latest=sorted((p/'accepted_history').glob('*.npz'))[-1]
    with np.load(latest) as d:
        assert start==float(d['time'][-1]) and np.array_equal(state,d['states'][-1])
    c=json.loads((p/'inputs.json').read_text(encoding='utf8'))
    aero=AeroDatabase(HERE/'runs/flow5_connection_01/aerodynamics/aero_database.npz')
    (p/'summary.json').write_text(json.dumps(dict(status='interrupted_io',message='Windows rejected atomic checkpoint replacement while a reader held the old file. Complete temporary checkpoint matches the last durable journal row.',start_time_s=0.,end_time_s=start,requested_end_s=c['simulation']['duration_s']),indent=2),encoding='utf8')
    result=resume_simulation(c,aero,checkpoint,duration=c['simulation']['duration_s']-start,output=OUT/'continuation')
    assert np.array_equal(state,result.states[0])
    history_figure(result).write_html(OUT/'continuation/history.html',include_plotlyjs=True)
    print(json.dumps({k:v for k,v in result.summary.items() if k not in ('assumptions','aero_metadata','collision_metadata')},ensure_ascii=False),flush=True)
