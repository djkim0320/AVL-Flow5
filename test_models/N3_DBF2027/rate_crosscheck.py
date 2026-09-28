"""Lateral rate-derivative cross-check of the saved actual AVL and flow5 tables.

Reads the UI flight-job databases and trims only. No solver is run, nothing is
substituted between solvers and existing results are not modified. The spiral
root is the classical approximation -(g/V)(Clb Cnr-Cnb Clr)/(Clb Cnp-Cnb Clp),
reported only to relate derivative differences to analyze_stability eigenvalues.
"""
import json
import numpy as np
from postprocess import HERE,folder,save,AeroDatabase


def lateral(backend):
    out=folder(backend);db=AeroDatabase(out/'aero_database.npz')
    flight=json.loads((out/'request.json').read_text('utf8'))['config']['flight']
    trim=json.loads((out/'trim.json').read_text('utf8'));alpha,elevator=trim['alpha_deg'],trim['elevator_deg']
    beta=float(db.axes[1][db.axes[1]>0].min())
    plus,minus=db.coeff([[alpha,beta,elevator],[alpha,-beta,elevator]])
    slope=(plus-minus)/(2*np.deg2rad(beta));rates=db.rates([[alpha,0.,elevator]])[0]
    clb,cnb=slope[3],slope[5];clp,cnp,clr,cnr=rates[3,0],rates[5,0],rates[3,2],rates[5,2]
    criterion=clb*cnr-cnb*clr;sweep=[]
    for a in db.axes[0]:
        c=db.coeff([[a,0.,elevator]])[0];r=np.deg2rad(a)
        sweep.append(dict(alpha_deg=float(a),CL=float(c[0]*np.sin(r)-c[2]*np.cos(r)),Clr=float(db.rates([[a,0.,elevator]])[0][3,2])))
    fit=np.polyfit([s['CL'] for s in sweep],[s['Clr'] for s in sweep],1)
    return dict(backend=backend,job=out.name,solver=db.metadata['solver'],alpha_deg=alpha,elevator_deg=elevator,beta_secant_deg=beta,
        axes='FRD body; rates per pb/2V and rb/2V; beta slopes per rad',
        Clb=float(clb),Cnb=float(cnb),Clp=float(clp),Cnp=float(cnp),Clr=float(clr),Cnr=float(cnr),
        spiral_criterion=float(criterion),Clr_for_neutral_spiral=float(clb*cnr/cnb),
        spiral_root_approximation_1_s=float(-flight['g_m_s2']/flight['speed_m_s']*criterion/(clb*cnp-cnb*clp)),
        dClr_dCL=float(fit[0]),Clr_at_zero_CL=float(fit[1]),alpha_sweep=sweep)


if __name__=='__main__':
    path=HERE/'runs/rate_crosscheck_01.json'
    if path.exists():raise FileExistsError(path)
    records=[lateral(b) for b in ('avl','flow5')]
    save(path,dict(records=records,scope='Saved actual solver tables only; spiral criterion > 0 means spirally stable in this approximation.'))
    for r in records:
        print(r['backend'],'Clr',round(r['Clr'],5),'dClr/dCL',round(r['dClr_dCL'],4),'criterion',round(r['spiral_criterion'],6),
              'root',round(r['spiral_root_approximation_1_s'],5),flush=True)
