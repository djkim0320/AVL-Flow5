"""Actual AVL R2 screening, database, trim/modes and coupled mission runs."""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'): os.environ[k]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import copy
import hashlib
import json
import traceback
import numpy as np
from dbf_stability import load_case, AeroDatabase, build_aero_database, solve_trim, analyze_stability, simulate
from dbf_stability.avl import run_avl, COEFF
from dbf_stability.plots import history_figure
from geometry_paths import cad_directory

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
VARIANT=HERE/'geometry_variants/refined_r2'
CASE=ROOT/'examples/h1_refined_r2.yaml'
OUT=HERE/'runs/refined_r2_01'
DB=OUT/'aerodynamics/aero_database.npz'


def initialize(case_path,output_path):
    global CASE,OUT,VARIANT,DB
    CASE=Path(case_path);OUT=Path(output_path)
    c=load_case(CASE);VARIANT=(ROOT/c['aero']['geometry']).parent.parent
    DB=OUT/'aerodynamics/aero_database.npz'


def dump(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf8')


def aero_job(args):
    variant,point=args
    c=load_case(CASE)
    suffix='' if variant=='nominal' else '_'+variant
    out=OUT/'aerodynamics/screen'/variant/('_'.join(f'{x:g}' for x in point))
    geometry=ROOT/c['aero']['geometry']
    d=run_avl(ROOT/c['aero']['executable'], geometry.with_stem(geometry.stem+suffix), out,*point,elevator_index=2)
    return dict(variant=variant,point=list(point),raw_directory=d['raw_dir'],solver_version=d['solver_version'],
                coefficients={k:d[k+'tot'] for k in COEFF},
                derivatives={k:d[k] for k in ('CLa','Cma','CYb','Clb','Cnb','CLq','Cmq','Cnr') if k in d},
                stability_text=(out/'stability.txt').read_text(encoding='utf8'))


def screen(workers):
    jobs=[(variant,point) for variant in ('coarse','nominal','fine','no_body') for point in [(-2.,0.,3.),(3.,4.,-5.)]]
    with ProcessPoolExecutor(max_workers=workers,initializer=initialize,initargs=(str(CASE),str(OUT))) as pool: rows=list(pool.map(aero_job,jobs))
    comparison=[]
    for point in [(-2.,0.,3.),(3.,4.,-5.)]:
        group={r['variant']:r for r in rows if r['point']==list(point)}
        comparison.append(dict(point=list(point),nominal_minus_fine={k:group['nominal']['coefficients'][k]-group['fine']['coefficients'][k] for k in COEFF},
                               body_minus_no_body={k:group['nominal']['coefficients'][k]-group['no_body']['coefficients'][k] for k in COEFF}))
    dump(OUT/'aerodynamics/screen.json',dict(rows=rows,comparisons=comparison,scope='Two actual AVL sample conditions; report coefficients and absolute differences, not an automatic convergence pass.'))
    for r in comparison: print(json.dumps(r),flush=True)


def trim_job(args):
    mode,n=args;c=load_case(CASE);c['cable']['segments']=n;a=AeroDatabase(DB)
    target=OUT/'equilibrium'/f'{mode}_{n}'
    try:
        trim=solve_trim(c,a,mode=mode)
    except (ValueError,RuntimeError) as exc:
        answer=dict(status='equilibrium_not_found',mode=mode,segments=n,error=str(exc),note='No linear stability claim is made without a valid equilibrium. Contact with the internal guide can invalidate a free-cable equilibrium.')
        dump(target/'summary.json',answer)
        return f'{mode}_{n}',answer
    dump(target/'trim.json',{k:v for k,v in trim.items() if k!='state'})
    np.save(target/'trim_state.npy',trim['state'])
    answer={k:v for k,v in trim.items() if k!='state'}
    if mode!='stowed':
        modes=analyze_stability(c,a,trim)
        modes['modes'].to_csv(target/'modes.csv',index=False)
        np.savez_compressed(target/'linearization.npz',matrix=modes['matrix'],eigenvalues=modes['eigenvalues'])
        answer.update(unstable=modes['unstable'],largest_real_eigenvalue=float(max(modes['eigenvalues'].real)))
    dump(target/'summary.json',answer)
    return f'{mode}_{n}',answer


def mission_job(refined):
    c=load_case(CASE)
    if refined:
        c['simulation']['max_step_s']*=.5
        c['simulation']['contact_max_step_s']=c['simulation'].get('contact_max_step_s',.003)*.5
        # The contact travel bound is often more restrictive than max_step.
        # Refine it too, or both nominal time settings give identical steps.
        for key in ('maximum_surface_travel_m','maximum_free_surface_travel_m'):
            if key in c['collision']:c['collision'][key]*=.5
    target=OUT/('time_refined' if refined else 'nominal')/'mission'
    target.mkdir(parents=True,exist_ok=False)
    try:
        a=AeroDatabase(DB);trim=solve_trim(c,a,mode='stowed')
        dump(target/'trim.json',{k:v for k,v in trim.items() if k!='state'})
        result=simulate(c,a,trim,phase='mission',duration=c['simulation']['duration_s'],output=target)
        history_figure(result).write_html(target/'history.html',include_plotlyjs=True)
        return str(refined),{k:result.summary[k] for k in ('status','duration_s','captured','max_tension_N','max_contact_N','max_pitch_change_deg','runtime_s','violations')}
    except Exception as e:
        (target/'failure.log').write_text(traceback.format_exc(),encoding='utf8')
        raise


def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['screen','database','equilibrium','mission']);p.add_argument('--workers',type=int,default=6);p.add_argument('--time-refined',action='store_true')
    p.add_argument('--case',type=Path,default=CASE);p.add_argument('--output',type=Path,default=OUT);args=p.parse_args()
    initialize(str(args.case.resolve()),str(args.output.resolve()))
    c=load_case(CASE);OUT.mkdir(parents=True,exist_ok=True)
    if args.stage=='screen': screen(args.workers)
    elif args.stage=='database':
        if DB.exists(): raise FileExistsError(DB)
        build_aero_database(c,output=DB,workers=args.workers)
        dump(DB.parent/'input_hashes.json',{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((VARIANT/'aero').glob('*')) if f.is_file()})
        print('Actual AVL database completed: '+str(DB),flush=True)
    elif args.stage=='equilibrium':
        with ProcessPoolExecutor(max_workers=args.workers,initializer=initialize,initargs=(str(CASE),str(OUT))) as pool:
            rows=dict(pool.map(trim_job,[('aircraft_only',10),('stowed',10)]+[('deployed',n) for n in (10,20,40)]))
        dump(OUT/'equilibrium/comparison.json',rows);print(json.dumps(rows,indent=2),flush=True)
    else:
        audit=json.loads((VARIANT/'door_swing.json').read_text(encoding='utf8'))
        hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in cad_directory(c).glob('*.step')}
        if audit['intersecting_angles'] or audit['error_angles'] or hashes!=audit['cad_sha256']: raise ValueError('Missing, failed or stale door CAD sweep')
        a=AeroDatabase(DB)
        if a.metadata['geometry_sha256']!=hashlib.sha256((ROOT/c['aero']['geometry']).read_bytes()).hexdigest(): raise ValueError('Stale AVL geometry')
        dump(OUT/'source_versions.json',{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in list((ROOT/'src/dbf_stability').glob('*.py'))+[Path(__file__)]})
        modes=[False,True] if args.time_refined else [False]
        with ProcessPoolExecutor(max_workers=min(args.workers,len(modes)),initializer=initialize,initargs=(str(CASE),str(OUT))) as pool:
            rows={}
            for future in as_completed([pool.submit(mission_job,v) for v in modes]):
                key,value=future.result();rows[key]=value;dump(OUT/'mission_comparison.json',rows);print(json.dumps({key:value}),flush=True)


if __name__=='__main__':main()
