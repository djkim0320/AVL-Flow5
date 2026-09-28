"""Compare old/new stowed trim and execute the actual solver at both trim points."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'): os.environ[k]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
from dbf_stability import load_case,AeroDatabase,solve_trim
from dbf_stability.avl import run_avl,COEFF,parse_output

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'runs/normal_r3_01'


def job(name):
    c=load_case(ROOT/('examples/h1_opening_70.yaml' if name=='previous' else 'examples/h1_normal_r3.yaml'))
    a=AeroDatabase(HERE/'aerodynamics/aero_database.npz' if name=='previous' else OUT/'aerodynamics/aero_database.npz')
    trim=solve_trim(c,a,mode='stowed');directory=OUT/'aerodynamics/direct_trim'/name
    direct=run_avl(ROOT/c['aero']['executable'],ROOT/c['aero']['geometry'],directory,trim['alpha_deg'],0.,trim['elevator_deg'],2)
    table=a.evaluate(trim['alpha_rad'],0.,trim['elevator_deg'],np.zeros(3),c['flight']['speed_m_s'])
    raw=np.array([direct[k+'tot'] for k in COEFF]);q=.5*c['flight']['rho_kg_m3']*c['flight']['speed_m_s']**2
    dimensional=(table-raw)*q*a.refs[0]*np.r_[np.ones(3),[a.refs[2],a.refs[1],a.refs[2]]]
    # The final spiral-stability ratio also ends with 'Cnb ='; it is not Cn_beta.
    stability=parse_output((directory/'stability.txt').read_text(encoding='utf8').split('Clb Cnr / Clr Cnb')[0])
    return name,dict(trim={k:v for k,v in trim.items() if k!='state'},mass_kg=c['aircraft']['mass_kg'],cg_frd_m=c['aircraft']['cg_m'],
        interpolation_minus_direct=dict(zip(['Fx_N','Fy_N','Fz_N','Mx_Nm','My_Nm','Mz_Nm'],map(float,dimensional))),
        actual_AVL=dict(solver_version=direct['solver_version'],raw_directory=str(directory),CLa=stability['CLa'],Cma=stability['Cma'],Clb=stability['Clb'],Cnb=stability['Cnb'],
                        static_margin_fraction=-stability['Cma']/stability['CLa']),
        limitations='Static margin is a local AVL derivative ratio; drag and mass are assumptions; no flight-test validation.')


if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool: result=dict(pool.map(job,['previous','normal_r3']))
    (OUT/'aerodynamics/trim_comparison.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2),flush=True)
