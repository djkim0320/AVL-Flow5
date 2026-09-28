"""Direct real flow5 spot check near the updated stowed trim, no door-flow claim."""
from pathlib import Path
import json,numpy as np
from dbf_stability import load_case,AeroDatabase
from dbf_stability.flow5 import _database_job,to_body_coefficients
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


if __name__=='__main__':
    c=load_case(ROOT/'examples/h1_underbody_270_v2.yaml')
    a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    trim=json.loads((HERE/'runs/underbody_mission_01/trim.json').read_text(encoding='utf8'))
    # Compare at exactly the database's physical moment reference; CG translation
    # and extra profile drag are applied later by the coupled model.
    c['aircraft']['cg_m']=a.metadata['moment_reference_frd_m']
    point=(round(trim['alpha_deg'],3),0.,25.)
    elevator=trim['elevator_deg']
    output=HERE/'runs/door_underbody_03/flow5_trim_spot_check'
    job=_database_job((c,str(output),elevator,None,0.,[point]))
    direct=to_body_coefficients(job['rows'][point],job['refs'])
    database=a.evaluate(np.deg2rad(point[0]),0.,elevator,np.zeros(3),25.)
    scale=.5*c['flight']['rho_kg_m3']*25**2*a.refs[0]
    delta=(direct-database)*scale*np.r_[np.ones(3),a.refs[[2,1,2]]]
    result=dict(solver=job['manifest']['solver'],angle_deg=point[0],elevator_deg=elevator,
        speed_m_s=25.,direct_body_coefficients=direct.tolist(),database_body_coefficients=database.tolist(),
        direct_minus_database_force_N=delta[:3].tolist(),direct_minus_database_moment_Nm=delta[3:].tolist(),
        moment_reference_frd_m=c['aircraft']['cg_m'],execution=job['manifest'],
        scope='Actual flow5 VLM2 run on unchanged wing/tail near updated trim. A spot check of interpolation, not new door/bracket/fuselage CFD or a certification of the full database.')
    (output/'comparison.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2),flush=True)
