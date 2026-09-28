"""Reintegrate the repaired recovery window in one run from its real first state."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,json,sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'solver_variants/fast_analytic_flow'))
from dbf_stability import AeroDatabase,simulate
from dbf_stability.analysis import load_result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--reference',type=Path,default=HERE/'runs/recovery_repair_16/captured_cleanup/complete');p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();reference=load_result(args.reference)
    if bool(reference.table.captured.iloc[0]):raise ValueError('Reference must begin before capture')
    branch=reference.summary['command_change_source']
    import numpy as np
    with np.load(branch['history_file'],allow_pickle=False) as data:meta=json.loads(str(data['metadata']))
    c=reference.config;c['simulation']['maximum_runtime_s']=7200.
    result=simulate(c,AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz'),meta['trim'],
        phase='mission',initial_state=reference.states[0],start_time=float(reference.time[0]),
        duration=float(reference.time[-1]-reference.time[0]),output=args.out)
    print(json.dumps(dict(status=result.summary['status'],captured=result.summary['captured'],final_door_deg=result.summary['final_door_deg'])),flush=True)
