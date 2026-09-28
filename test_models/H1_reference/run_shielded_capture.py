"""Explicit cable-in-bay airflow assumption; isolated reproducible solver."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import sys,yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
variant='fast_analytic_flow' if '--fast' in sys.argv else 'internal_cable_flow'
if '--fast' in sys.argv:sys.argv.remove('--fast')
sys.path.insert(0,str(HERE/'solver_variants'/variant))
from dbf_stability import load_case
from run_capture_stop import main

if __name__=='__main__':
    case=ROOT/'examples/h1_round_x_capture_stop_v3_shielded.yaml'
    c=load_case(ROOT/'examples/h1_round_x_capture_stop_v3.yaml')
    c['flight']['cable_bay_shielding']=True
    c['flight']['internal_cable_flow_factor']=0.
    c['provenance']['cable']['source']+=' Cable aerodynamic exposure is the length-weighted rear-opening fraction of the two half-spans around each node. Outside uses the declared local-flow factor; inside air is assumed to co-move with the aircraft, so only cable motion relative to that internal air creates drag. This is an unmeasured shielding assumption, not a resolved cabin/wake flow.'
    case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
    sys.argv+=['--case',str(case)]
    main()
