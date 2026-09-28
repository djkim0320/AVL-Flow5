"""Refine actual solver samples; retain the previous database unchanged."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
import argparse,hashlib,json
import numpy as np
from dbf_stability import load_case,AeroDatabase,solve_trim
from dbf_stability.flow5 import build_flow5_database,_database_job,to_body_coefficients
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    c=load_case(ROOT/'examples/h1_underbody_270_v2.yaml')
    c['aero']['alpha_deg']=[-4.,-3.,-2.5,-2.25,-2.,-1.75,-1.5,-1.,0.,2.,4.,8.]
    c['aero']['beta_deg']=[-4.,0.,4.]
    c['aero']['elevator_deg']=[-8.,-6.,-4.,-2.,0.,1.,2.,2.5,3.,3.25,3.5,3.75,4.,4.5,5.,6.,8.]
    (out/'input.json').write_text(json.dumps(c,indent=2,default=str),encoding='utf8')
    a=build_flow5_database(c,out/'aero_database.npz',workers=2)
    trim=solve_trim(c,a,mode='stowed')
    (out/'trim.json').write_text(json.dumps({k:v for k,v in trim.items() if k!='state'},indent=2),encoding='utf8')
    point=(round(trim['alpha_deg'],3),0.,25.)
    job=_database_job((c,str(out/'trim_holdout'),trim['elevator_deg'],None,0.,[point]))
    direct=to_body_coefficients(job['rows'][point],job['refs'])
    interpolated=a.evaluate(np.deg2rad(point[0]),0.,trim['elevator_deg'],np.zeros(3),25.)
    scale=.5*c['flight']['rho_kg_m3']*25**2*a.refs[0]
    delta=(direct-interpolated)*scale*np.r_[np.ones(3),a.refs[[2,1,2]]]
    validation=dict(aero_sha256=hashlib.sha256((out/'aero_database.npz').read_bytes()).hexdigest(),
        point=point,elevator_deg=trim['elevator_deg'],direct=direct.tolist(),
        interpolated=interpolated.tolist(),force_difference_N=delta[:3].tolist(),
        moment_difference_Nm=delta[3:].tolist(),
        force_tolerance_N=.05,moment_tolerance_Nm=.02,
        passed=bool(np.max(np.abs(delta[:3]))<.05 and np.max(np.abs(delta[3:]))<.02),
        scope='Independent actual flow5 spot check at the rounded new trim; not validation of all off-grid conditions or door-local flow.')
    (out/'trim_validation.json').write_text(json.dumps(validation,indent=2),encoding='utf8')
    print(json.dumps(dict(trim={k:v for k,v in trim.items() if k!='state'},validation=validation),indent=2),flush=True)
    if not validation['passed']:raise RuntimeError('Refined trim fails independent solver check')
