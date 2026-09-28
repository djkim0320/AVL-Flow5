"""Checks against the newly computed N3 results, without surrogate fixtures."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import copy,json
import numpy as np
import pandas as pd
from postprocess import HERE,ROOT,folder,save,AeroDatabase,solve_trim,analyze_stability
from dbf_stability.model import CoupledModel
from compute_resources import apply_affinity

def eigen_check(args):
    backend,step=args;p=folder(backend);c=json.loads((p/'request.json').read_text('utf8'))['config'];c['simulation']['jacobian_workers']=1
    db=AeroDatabase(p/'aero_database.npz');t=json.loads((p/'trim.json').read_text('utf8'));t['state']=np.load(p/'trim_state.npy')
    r=analyze_stability(c,db,t,perturbation=step)
    return dict(backend=backend,finite_difference_step=step,max_real_1_s=float(r['modes'].real_1_s.max()),
        positive_above_1e_minus3=int((r['modes'].real_1_s>1e-3).sum()))

if __name__=='__main__':
    apply_affinity();p=folder('flow5');c=json.loads((p/'request.json').read_text('utf8'))['config'];db=AeroDatabase(p/'aero_database.npz')
    original=db.coeff.values.copy();strict=copy.deepcopy(c);strict['simulation']['trim_angular_tolerance_rad_s2']=1e-5
    rejected=False
    try:solve_trim(strict,db)
    except RuntimeError as e:
        if 'TRIM_INFEASIBLE' not in str(e):raise
        rejected=True
    assert rejected,'Strict failure must remain reproducible'
    trim=solve_trim(c,db)
    assert trim['max_linear_acceleration_m_s2']<=1e-5
    assert 1e-5<trim['max_angular_acceleration_rad_s2']<=5e-5
    assert np.array_equal(original,db.coeff.values),'Trim must not zero/symmetrize solver coefficients'
    invalid=copy.deepcopy(c);invalid['simulation']['trim_angular_tolerance_rad_s2']=0
    try:solve_trim(invalid,db)
    except ValueError as e:assert 'tolerances' in str(e)
    else:raise AssertionError('Invalid angular tolerance accepted')
    mass_checks={}
    for b in ('avl','flow5'):
        d=pd.read_csv(folder(b)/'simulation/timeseries.csv');mass_checks[b]=float(np.ptp(d.total_mass_kg))
        assert mass_checks[b]<1e-12
    with ProcessPoolExecutor(max_workers=6) as pool:
        modes=list(pool.map(eigen_check,[(b,h) for b in ('avl','flow5') for h in (1e-4,1e-5,1e-6)]))
    save(HERE/'runs/final/quality_checks.json',dict(strict_tolerance_failure_reproduced=rejected,
        explicit_tolerance_pass=True,raw_coefficients_unchanged=True,invalid_tolerance_rejected=True,
        mass_variation_kg=mass_checks,eigenvalue_step_sensitivity=modes))
    print(json.dumps(modes),flush=True)
