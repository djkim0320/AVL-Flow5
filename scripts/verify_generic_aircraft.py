"""Reproducible generated-aircraft integration test; not an S27 prediction."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import sys,json,argparse
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ui'),str(ROOT/'ui/tests'),str(ROOT/'src')]

def main():
    from test_aircraft_definition import definition_fixture
    from aircraft_definition import register_definition
    import model_registry
    from server import build_reference,worker_init
    from analysis_bridge import catalog,prepare
    from dbf_stability import build_aero_database,solve_trim,analyze_stability,simulate
    import numpy as np
    worker_init()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/generic_aircraft_ui_20260928')
    out=parser.parse_args().output.resolve();out.mkdir(parents=True,exist_ok=True)
    if (out/'generated_avl_database.npz').exists():
        raise FileExistsError('Use --output with a new folder; previous results are preserved')
    model_registry.DIRECTORY=out/'validation_models'
    p=definition_fixture();registration=register_definition(p)
    (out/'analytic_validation_project.json').write_text(json.dumps(p,ensure_ascii=False,indent=2),encoding='utf8')
    s=catalog(registration['id'])['defaults'];s.update(task='flight',phase='deployed',duration=.2,workers=12,pitch_delta=.1)
    s['aero_grid']={k:[-4.,0.,8.] if k=='alpha_deg' else [-4.,0.,4.] for k in ['alpha_deg','beta_deg','elevator_deg']}
    prepared=prepare(p,s,build_reference());c=prepared['config']
    print('Building 27 real AVL operating points; requested ceiling 12 workers',flush=True)
    db=build_aero_database(c,out/'generated_avl_database.npz',workers=12)
    print('Solving coupled point-mass trim and modes',flush=True)
    trim=solve_trim(c,db,mode='deployed')
    stability=analyze_stability(c,db,trim)
    result=simulate(c,db,trim=trim,phase='deployed',duration=.2,output=out/'flight')
    summary=dict(scope='Analytic validation fixture; not S27 or team flight prediction',solver=db.metadata['solver'],
        actual_avl_conditions=27,execution=db.metadata['execution'],trim={k:v for k,v in trim.items() if k!='state'},
        mode_count=len(stability['modes']),status=result.summary['status'],time_s=float(result.table.time_s.iloc[-1]),
        definition_sha256=prepared['profile']['definition_sha256'])
    (out/'integration_result.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
