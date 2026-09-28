"""Compare direct flow5 grid nodes and intermediate elevator conditions."""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json
import numpy as np
from dbf_stability import load_case, AeroDatabase
from dbf_stability.flow5 import _database_job, to_body_coefficients

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

if __name__=='__main__':
    c=load_case(ROOT/'examples/h1_underbody_270_v2.yaml')
    a=AeroDatabase(HERE/'runs/span_hold_100m_25ms_01/aerodynamics/aero_database.npz')
    c['aircraft']['cg_m']=a.metadata['moment_reference_frd_m']
    out=HERE/'runs/door_underbody_03/flow5_grid_diagnosis'
    out.mkdir(exist_ok=False)
    points=[(-2.,0.,25.),(0.,0.,25.)]
    jobs=[(c,str(out/f'e_{int(e):+03d}'),e,None,0.,points) for e in [-8.,0.,4.,8.]]
    with ProcessPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(_database_job,jobs))
    comparison=[]
    for job in results:
        for point,row in job['rows'].items():
            direct=to_body_coefficients(row,job['refs'])
            stored=a.evaluate(np.deg2rad(point[0]),0.,job['elevator'],np.zeros(3),25.)
            comparison.append(dict(alpha=point[0],elevator=job['elevator'],direct=direct.tolist(),
                database=stored.tolist(),difference=(direct-stored).tolist(),directory=job['directory']))
    (out/'comparison.json').write_text(json.dumps(comparison,indent=2),encoding='utf8')
    print(json.dumps(comparison,indent=2),flush=True)
