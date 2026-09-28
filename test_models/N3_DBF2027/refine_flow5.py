"""Additional flow5 yaw-damping check at neutral controls, actual solver only."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from concurrent.futures import ProcessPoolExecutor
import json
from design import HERE,MESHES,make_case
from analysis_campaign import probe,save
from compute_resources import apply_affinity

if __name__=='__main__':
    apply_affinity();out=HERE/'runs/flow5_refinement_01'
    if out.exists():raise FileExistsError(out)
    out.mkdir()
    meshes={'ultra':(32,16),'finest':(40,20)}
    for mesh,counts in meshes.items():MESHES[mesh]=counts;make_case(mesh)
    save(out/'inputs.json',dict(meshes=meshes,condition=dict(alpha=0,beta=0,elevator=0),purpose='Further Cnr convergence check; not a replacement full database'))
    jobs=[('flow5',mesh,0.,[(0.,0.)],str(out/mesh)) for mesh in meshes]
    with ProcessPoolExecutor(max_workers=2) as pool:records=list(pool.map(probe,jobs))
    save(out/'results.json',records)
    for r in records:
        v=r['rows'][0];print(r['mesh'],v['CL'],v['CDi'],v['coeff'][4],v['rates'][4][1],v['rates'][5][2],flush=True)
