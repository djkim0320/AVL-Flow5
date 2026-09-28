"""Independent real-flow5 holdout after a mechanical geometry/mass revision."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import argparse,copy,hashlib,json
import numpy as np
from dbf_stability import load_case,AeroDatabase,solve_trim
from dbf_stability.flow5 import _database_job,to_body_coefficients

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--aero',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False);c=load_case(a.case);db=AeroDatabase(a.aero)
    for k,axis in zip(('alpha_deg','beta_deg','elevator_deg'),db.axes):c['aero'][k]=axis.tolist()
    trim=solve_trim(c,db,mode='stowed');trim.pop('state')
    (a.out/'trim.json').write_text(json.dumps(trim,indent=2))
    solver=copy.deepcopy(c);solver['aircraft']['cg_m']=db.metadata['moment_reference_frd_m']
    point=(round(trim['alpha_deg'],3),0.,c['flight']['speed_m_s'])
    job=_database_job((solver,str(a.out/'actual_flow5'),trim['elevator_deg'],None,0.,[point]))
    direct=to_body_coefficients(job['rows'][point],job['refs'])
    sampled=db.evaluate(np.deg2rad(point[0]),0.,trim['elevator_deg'],np.zeros(3),point[2])
    scale=.5*c['flight']['rho_kg_m3']*point[2]**2*db.refs[0]
    delta=(direct-sampled)*scale*np.r_[np.ones(3),db.refs[[2,1,2]]]
    result=dict(case_sha256=hashlib.sha256(a.case.read_bytes()).hexdigest(),aero_sha256=hashlib.sha256(a.aero.read_bytes()).hexdigest(),point=point,elevator_deg=trim['elevator_deg'],direct=direct.tolist(),interpolated=sampled.tolist(),force_difference_N=delta[:3].tolist(),moment_difference_Nm=delta[3:].tolist(),force_tolerance_N=.05,moment_tolerance_Nm=.02,passed=bool(max(abs(delta[:3]))<.05 and max(abs(delta[3:]))<.02),execution=job['manifest'],scope='Actual flow5 at the changed-mass trim. Database physical moment reference retained, CG translation in coupled model. New duct local flow is not resolved.')
    (a.out/'trim_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    if not result['passed']:raise RuntimeError('Actual-flow5 interpolation check failed')
