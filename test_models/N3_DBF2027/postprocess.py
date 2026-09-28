"""Mission mass cases, measured solver-table checks and reviewable comparison data."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse,copy,json,sys
import numpy as np
import pandas as pd
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'test_models/H1_reference')]
from dbf_stability import AeroDatabase,solve_trim,analyze_stability
from dbf_stability.math3d import rotation
from dbf_stability.avl import run_avl,parse_output
from dbf_stability.flow5 import _database_job,to_body_coefficients,to_body_rate_derivatives
from compute_resources import apply_affinity

def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf8')

def folder(backend):
    job=json.loads((HERE/f'ui_{backend}_flight_job.json').read_text('utf8'))
    return ROOT/'ui/data/analysis'/job['id']

def add_rigid_mass(c,components):
    """Shift CG, inertia and lever arms consistently; points are absolute ARU."""
    c=copy.deepcopy(c);a=c['aircraft'];old=np.array(a['cg_m']);m=a['mass_kg']
    comp=[(mass,np.array(pos)*[-1,1,-1],np.asarray(size)) for mass,pos,size in components]
    total=m+sum(x[0] for x in comp);cg=(m*old+sum(mass*pos for mass,pos,_ in comp))/total
    d=old-cg;I=np.asarray(a['inertia_kgm2'])+m*(d@d*np.eye(3)-np.outer(d,d))
    for mass,pos,(x,y,z) in comp:
        d=pos-cg;I+=mass/12*np.diag([y*y+z*z,x*x+z*z,x*x+y*y])+mass*(d@d*np.eye(3)-np.outer(d,d))
    for name in ('tow_point_m','thrust_point_m'):a[name]=(np.array(a[name])+old-cg).tolist()
    a.update(mass_kg=total,cg_m=cg.tolist(),inertia_kgm2=I.tolist())
    return c

def mass_case(c,mission):
    cable=(c['cable']['length_m']*c['cable']['density_kg_m'],[.785,0,.055],[0,0,0])
    if mission=='M1':return add_rigid_mass(c,[cable]),'aircraft_only'
    if mission=='M2':return add_rigid_mass(c,[cable,(.12,[.54,0,-.015],[0,0,0]),(.07,[.54,0,-.015],[.28,.13,.13])]),'aircraft_only'
    return copy.deepcopy(c),'deployed'

def trim_job(arg):
    backend,mission,speed,cd=arg;out=folder(backend)
    c=json.loads((out/'request.json').read_text('utf8'))['config'];c,mode=mass_case(c,mission)
    c['flight']['speed_m_s']=speed;c['aircraft']['profile_cd']=cd;c['simulation']['jacobian_workers']=1
    db=AeroDatabase(out/'aero_database.npz');tr=solve_trim(c,db,mode)
    rec={k:v for k,v in tr.items() if k!='state'}
    mass=c['aircraft']['mass_kg']+(c['sensor']['mass_kg']+c['cable']['density_kg_m']*c['cable']['length_m'] if mode=='deployed' else 0)
    rec.update(backend=backend,mission=mission,speed_m_s=speed,profile_cd=cd,total_mass_kg=mass,
        CG_ARU_m=(np.array(c['aircraft']['cg_m'])*[-1,1,-1]).tolist(),propulsive_power_W=tr['thrust_N']*speed)
    point=np.array([[tr['alpha_deg'],0,tr['elevator_deg']]])
    coeff=db.coeff(point)[0];a=tr['alpha_rad'];cl=coeff[0]*np.sin(a)-coeff[2]*np.cos(a);cdi=-coeff[0]*np.cos(a)-coeff[2]*np.sin(a)
    rec.update(CL=float(cl),CDi=float(cdi),CD_total_assumed=float(cdi+cd),L_D_assumed=float(cl/(cdi+cd)))
    if cd==.03:
        modes=analyze_stability(c,db,tr);rec.update(unstable=modes['unstable'],max_real_1_s=float(modes['modes'].real_1_s.max()))
        modes['modes'].to_csv(HERE/f'runs/final/{backend}_{mission}_{speed:g}ms_modes.csv',index=False)
    if mode=='deployed':
        y=tr['state'];r=rotation(y[6:10]);sensor=r.T@(y[13:16]-y[:3]);feed=np.array(c['aircraft']['tow_point_m'])
        distance=float(np.linalg.norm(sensor-feed));rec.update(exit_to_point_m=distance,
            conservative_exit_to_front_tip_m=distance-.11,minimum_required_m=2.7,
            sensor_position_body_m=sensor.tolist(),tip_distance_scope='220 mm reserved sensor centered at point; no attitude/aerodynamic sensor model')
    stem=HERE/f'runs/final/{backend}_{mission}_{speed:g}ms_cd{cd:.3f}'
    save(Path(str(stem)+'.json'),rec)
    save(Path(str(stem)+'_inputs.json'),c);np.save(Path(str(stem)+'_state.npy'),tr['state'])
    return rec

def holdout(backend):
    out=folder(backend);db=AeroDatabase(out/'aero_database.npz');c=json.loads((out/'request.json').read_text('utf8'))['config']
    t=json.loads((out/'trim.json').read_text('utf8'));a=t['alpha_deg'];el=t['elevator_deg'];directory=HERE/f'runs/final/{backend}_direct_trim_{out.name}'
    if backend=='avl':
        d=run_avl(ROOT/c['aero']['executable'],c['aero']['geometry'],directory,a,0.,el,2,timeout_s=600.)
        direct=np.array([d[k+'tot'] for k in ('CX','CY','CZ','Cl','Cm','Cn')]);der=parse_output((directory/'stability.txt').read_text('utf8'))
        stability={k:der[k] for k in ('CLa','Cma','Cnb','Clb','Cmq','Clp','Cnr','Xnp')}
        direct_rates=np.array([[d[k+r] for r in 'pqr'] for k in ('CX','CY','CZ','Cl','Cm','Cn')])
        stability['static_margin_percent_MAC']=100*(der['Xnp']+c['aircraft']['cg_m'][0])/db.refs[1]
    else:
        # Native CSV prints six fractional digits for alpha; avoid a false
        # missing-point mismatch caused by asking for unprintable precision.
        a=round(a,6)
        r=_database_job((c,str(directory),el,None,0.,[(a,0.,20.)]));d=r['rows'][(a,0.,20.)]
        direct=to_body_coefficients(d,r['refs']);direct_rates=to_body_rate_derivatives(d,r['refs'])
        stability={k:float(v) for k,v in d.items() if k in ('CZa','Cma','Cyb','Clb','Cnb','Cmq','Clp','Cnr')}
    predicted=db.coeff([[a,0.,el]])[0]
    rec=dict(backend=backend,alpha_deg=a,elevator_deg=el,direct_coeff=direct.tolist(),table_coeff=predicted.tolist(),
        absolute_error=(predicted-direct).tolist(),stability=stability,direct_rates=direct_rates.tolist(),
        rate_absolute_error=(db.rates([[a,0.,el]])[0]-direct_rates).tolist(),
        force_error_N=(.5*c['flight']['rho_kg_m3']*20**2*db.refs[0]*(predicted[:3]-direct[:3])).tolist(),
        moment_error_Nm=(.5*c['flight']['rho_kg_m3']*20**2*db.refs[0]*(predicted[3:]-direct[3:])*db.refs[[2,1,2]]).tolist())
    save(HERE/f'runs/final/{backend}_holdout.json',rec);return rec

def comparison():
    rows=[]
    for backend in ('avl','flow5'):
        out=folder(backend);db=AeroDatabase(out/'aero_database.npz')
        for alpha in db.axes[0]:
            p=[[alpha,0.,0.]];c=db.coeff(p)[0];r=db.rates(p)[0];a=np.deg2rad(alpha)
            rows.append(dict(backend=backend,alpha_deg=float(alpha),CL=float(c[0]*np.sin(a)-c[2]*np.cos(a)),
                CDi=float(-c[0]*np.cos(a)-c[2]*np.sin(a)),Cm=float(c[4]),Cmq=float(r[4,1]),Clp=float(r[3,0]),Cnr=float(r[5,2])))
    pd.DataFrame(rows).to_csv(HERE/'runs/final/polars.csv',index=False)
    d=pd.read_csv(HERE/'runs/mesh_02/mesh_results.csv');a=d[d.mesh=='fine'].set_index(['backend','alpha','beta','elevator']);b=d[d.mesh=='extra'].set_index(['backend','alpha','beta','elevator'])
    cols=['CL','CDi','Cm','Cmq','Clp','Cnr'];absdiff=(a[cols]-b[cols]).abs();percent=100*absdiff/b[cols].abs()
    absdiff.to_csv(HERE/'runs/final/mesh_absolute_changes.csv');percent.to_csv(HERE/'runs/final/mesh_percent_changes.csv')
    finer=json.loads((HERE/'runs/flow5_refinement_01/results.json').read_text('utf8'))
    finer={r['mesh']:r['rows'][0] for r in finer};u=finer['ultra'];f=finer['finest']
    save(HERE/'runs/final/mesh_assessment.json',dict(relative_target_percent=5.,
        max_relative_percent=percent.groupby('backend').max().to_dict('index'),
        max_absolute_change=absdiff.groupby('backend').max().to_dict('index'),
        flow5_6144_to_9600_change_percent={'CL':100*abs(u['CL']-f['CL'])/abs(f['CL']),
            'CDi':100*abs(u['CDi']-f['CDi'])/abs(f['CDi']),
            'Cnr':100*abs(u['rates'][5][2]-f['rates'][5][2])/abs(f['rates'][5][2])},
        all_metrics_converged=False,notes=['Cm near zero needs the absolute difference to interpret relative percentages.',
        'flow5 yaw damping Cnr has not converged; stability is provisional. No all-metrics pass is asserted.']))

def main():
    sys.stdout.reconfigure(encoding='utf8');apply_affinity();p=argparse.ArgumentParser();p.add_argument('action',choices=['cases','holdout','compare']);p.add_argument('--backend',default='avl');a=p.parse_args()
    (HERE/'runs/final').mkdir(parents=True,exist_ok=True)
    if a.action=='holdout':print(json.dumps(holdout(a.backend),ensure_ascii=False),flush=True)
    elif a.action=='compare':comparison()
    else:
        jobs=[(b,m,v,.03) for b in ('avl','flow5') for m in ('M1','M2','M3') for v in ([15.,20.,25.] if b=='avl' else [20.])]
        jobs += [(b,'M3',20.,cd) for b in ('avl','flow5') for cd in (.02,.04)]
        with ProcessPoolExecutor(max_workers=12) as pool:records=list(pool.map(trim_job,jobs))
        save(HERE/'runs/final/mission_cases.json',records);pd.DataFrame(records).drop(columns=['CG_ARU_m','sensor_position_body_m']).to_csv(HERE/'runs/final/mission_cases.csv',index=False)
        print(pd.DataFrame(records)[['backend','mission','speed_m_s','profile_cd','alpha_deg','elevator_deg','thrust_N']].to_string(index=False),flush=True)

if __name__=='__main__':main()
