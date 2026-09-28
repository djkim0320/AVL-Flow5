"""Fresh solver campaigns; no mock aerodynamics or reused aircraft tables."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,copy,json,sys,time,hashlib
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
import pandas as pd
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'test_models/H1_reference')]
from dbf_stability import load_case,build_aero_database,AeroDatabase,solve_trim,analyze_stability
from dbf_stability.avl import run_avl,parse_output
from dbf_stability.flow5 import _database_job,to_body_coefficients,to_body_rate_derivatives
from compute_resources import apply_affinity


def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf8')


def probe(job):
    backend,mesh,el,conditions,out=job;out=Path(out)
    c=load_case(HERE/f'N3_{mesh}.yaml');rows=[]
    if backend=='avl':
        for i,(alpha,beta) in enumerate(conditions):
            d=run_avl(ROOT/c['aero']['executable'],c['aero']['geometry'],out/f'point_{i:02d}',alpha,beta,el,2)
            ft=parse_output((out/f'point_{i:02d}/forces.txt').read_text())
            st=parse_output((out/f'point_{i:02d}/stability.txt').read_text())
            rows.append(dict(alpha=alpha,beta=beta,elevator=el,CL=ft['CLtot'],CDi=ft['CDind'],
                coeff=[d[k+'tot'] for k in ('CX','CY','CZ','Cl','Cm','Cn')],
                rates=[[d[k+r] for r in 'pqr'] for k in ('CX','CY','CZ','Cl','Cm','Cn')],
                raw=str(out/f'point_{i:02d}'),stability={k:st.get(k) for k in ('CLa','Cma','Cnb','Clb','Xnp')}))
    else:
        c['aero']['backend']='flow5'
        result=_database_job((c,str(out),el,None,0.,[(float(a),float(b),20.) for a,b in conditions]))
        for (alpha,beta,_),d in result['rows'].items():
            rows.append(dict(alpha=alpha,beta=beta,elevator=el,CL=d['CL'],CDi=d['CD_inviscid'],
                coeff=to_body_coefficients(d,result['refs']).tolist(),rates=to_body_rate_derivatives(d,result['refs']).tolist(),raw=str(out)))
    save(out/'probe.json',dict(backend=backend,mesh=mesh,rows=rows))
    return dict(backend=backend,mesh=mesh,rows=rows)


def mesh_campaign(out,workers,meshes):
    jobs=[]
    for backend in ('avl','flow5'):
        for mesh in meshes:
            for el,conditions in [(0.,[(-2.,0.),(0.,0.),(4.,0.),(0.,3.)]),(-4.,[(0.,0.)]),(4.,[(0.,0.)])]:
                jobs.append((backend,mesh,el,conditions,str(out/f'{backend}_{mesh}_e{el:+g}')))
    records=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures={pool.submit(probe,j):j for j in jobs}
        for f in as_completed(futures):
            r=f.result();records.append(r);print('completed',r['backend'],r['mesh'],r['rows'][0]['elevator'],flush=True)
            save(out/'mesh_progress.json',{'completed':len(records),'total':len(jobs)})
    save(out/'mesh_results.json',records)
    table=[]
    for r in records:
        for row in r['rows']:
            d={k:v for k,v in row.items() if k not in ('coeff','rates','stability')};d.update(backend=r['backend'],mesh=r['mesh'])
            d.update(dict(zip(('CX','CY','CZ','Cl','Cm','Cn'),row['coeff'])))
            d.update(Cmq=row['rates'][4][1],Clp=row['rates'][3][0],Cnr=row['rates'][5][2]);table.append(d)
    df=pd.DataFrame(table);df.to_csv(out/'mesh_results.csv',index=False)
    print(df[['backend','mesh','alpha','beta','elevator','CL','CDi','Cm','Cmq']].to_string(index=False),flush=True)


def database(out,workers,mesh,backend):
    c=load_case(HERE/f'N3_{mesh}.yaml');c['aero']['backend']=backend
    db=build_aero_database(c,out/'aero_database.npz',workers=workers)
    save(out/'metadata.json',db.metadata)
    import yaml
    (out/'case.yaml').write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True),encoding='utf8')
    records={}
    for mode in ('aircraft_only','deployed'):
        trim=solve_trim(c,db,mode=mode);np.save(out/f'{mode}_state.npy',trim['state'])
        records[mode]={k:v for k,v in trim.items() if k!='state'}
        modes=analyze_stability(c,db,trim);modes['modes'].to_csv(out/f'{mode}_modes.csv',index=False)
        records[mode].update(unstable=modes['unstable'],max_real_eigenvalue_1_s=float(modes['modes'].real_1_s.max()))
        print(backend,mode,records[mode],flush=True)
    save(out/'trims.json',records)


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['mesh','database']);p.add_argument('--out',required=True)
    p.add_argument('--workers',type=int,default=12);p.add_argument('--meshes',nargs='+',default=['coarse','medium','fine'])
    p.add_argument('--mesh',default='medium');p.add_argument('--backend',default='avl',choices=['avl','flow5']);a=p.parse_args()
    apply_affinity();out=HERE/a.out
    if out.exists():raise FileExistsError(out)
    out.mkdir(parents=True);started=time.time()
    save(out/'campaign.json',dict(action=a.action,mesh=a.mesh,backend=a.backend,workers=a.workers,started=started,
        geometry_definition=json.loads((HERE/'geometry_definition.json').read_text('utf8')),
        geometry_sha256={mesh:hashlib.sha256((HERE/'aero'/mesh/'N3.avl').read_bytes()).hexdigest() for mesh in a.meshes}))
    if a.action=='mesh':mesh_campaign(out,a.workers,a.meshes)
    else:database(out,a.workers,a.mesh,a.backend)
    save(out/'complete.json',dict(runtime_s=time.time()-started,completed=True))


if __name__=='__main__':main()
