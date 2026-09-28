from pathlib import Path
import json
import numpy as np
from dbf_stability import *
from dbf_stability.verification import convergence_study


def main():
    root=Path(__file__).resolve().parents[1]
    c=load_case(root/'examples/reference.yaml');a=AeroDatabase(root/'outputs/aero_database.npz')
    out=root/'outputs/validation';out.mkdir(parents=True,exist_ok=True)
    for mode in ['aircraft_only','stowed','deployed']:
        t=solve_trim(c,a,mode=mode)
        (out/f'trim_{mode}.json').write_text(json.dumps({k:v for k,v in t.items() if k!='state'},indent=2))
    result=analyze_stability(c,a)
    result['modes'].to_csv(out/'modes.csv',index=False)
    np.save(out/'state_matrix.npy',result['matrix'])
    print(json.dumps(convergence_study(c,a,out/'convergence',duration=.2),indent=2),flush=True)


if __name__=='__main__':main()
