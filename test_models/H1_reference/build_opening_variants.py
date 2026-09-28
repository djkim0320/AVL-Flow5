"""Enlarge the copied H1 rear opening; never read the source CAD project."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key]='1'
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import argparse
import json
import hashlib
import numpy as np
import yaml
import cadquery as cq
from dbf_stability import load_case
from build_cad import box, stl, glb

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def build(height_mm):
    c=load_case(ROOT/'examples/h1_door_140.yaml');b=c['bay']
    old_ceiling=b['ceiling_z_m'];height=height_mm*.001
    if height<=b['floor_z_m']-old_ceiling:
        raise ValueError('This builder only enlarges the original opening')
    out=HERE/'geometry_variants'/f'opening_{height_mm:g}'
    cad=out/'cad';meshes=out/'meshes'
    cad.mkdir(parents=True,exist_ok=False);meshes.mkdir()
    b['ceiling_z_m']=b['floor_z_m']-height;b['door_length_m']=height
    x,floor,ceil,hw,t=b['exit_x_m'],b['floor_z_m'],b['ceiling_z_m'],b['half_width_m'],b['wall_thickness_m']
    front=b['front_x_m']
    originals=sorted((HERE/'cad').glob('*.step'))
    shapes={f.stem:cq.importers.importStep(str(f)).val() for f in originals if f.stem!='H1_A_temporary_assembly'}
    shapes['guide_ceiling']=box([x,-hw,ceil-t],[front,hw,ceil])
    for sign in (-1,1):
        y1,y2=sorted([sign*hw,sign*(hw+t)])
        shapes[f'guide_side_{sign:+d}']=box([x,y1,ceil],[front,y2,floor])
    shapes['rear_exit_frame']=box([x-.002,-hw-.004,ceil-.004],[x+.002,hw+.004,floor+.004]).cut(
        box([x-.003,-hw,ceil],[x+.003,hw,floor]))
    raw=box([x-b['door_thickness_m']/2,-hw,ceil],[x+b['door_thickness_m']/2,hw,floor])
    h=np.array([x,0,floor])*1000
    shapes['rear_door_100deg']=raw.rotate(tuple(h),tuple(h+[0,1,0]),100)
    p=json.loads((HERE/'parameters.json').read_text(encoding='utf8'))
    cg=np.array(p['aircraft_cg_aru_m']);tunnel=np.array(p['tunnel_box_aru_m'])
    # Raise the entire original cutout ceiling by the same amount, including its margin.
    tunnel[1,2]+=old_ceiling-ceil
    transformed=(tunnel-cg)*[-1,1,-1]
    cutter=box(transformed.min(0),transformed.max(0))
    fuselage='fuselage_H1_approx_with_rear_cutout'
    shapes[fuselage]=shapes[fuselage].cut(cutter)
    assembly=cq.Assembly(name=f'H1_opening_{height_mm:g}_raw100_FRD_mm');parts=[];rows=[]
    for name,shape in shapes.items():
        if not shape.isValid() or shape.Volume()<=0:raise ValueError(f'Invalid solid: {name}')
        color=[.91,.63,.17,1] if name.startswith('sensor') else [.18,.53,.73,1] if name.startswith('rear_door') else [.35,.55,.58,.3] if name.startswith('fuselage') else [.55,.60,.65,1]
        cq.exporters.export(shape,str(cad/f'{name}.step'))
        vertices,faces=shape.tessellate(.25,.15)
        v=np.array([a.toTuple() for a in vertices])*.001;f=np.array(faces,dtype=int)
        stl(meshes/f'{name}_FRD_m.stl',v,f);parts.append((name,v,f,color))
        assembly.add(shape,name=name,color=cq.Color(*color))
        rows.append({'part':name,'valid':shape.isValid(),'solids':len(shape.Solids()),'volume_mm3':shape.Volume()})
    assembly.export(str(cad/'H1_A_temporary_assembly.step'))
    restored=cq.importers.importStep(str(cad/'H1_A_temporary_assembly.step')).val()
    assert restored.isValid() and len(restored.Solids())==sum(r['solids'] for r in rows)
    glb(meshes/'H1_A_temporary_assembly.glb',parts)
    c['name']=f'H1_56x{height_mm:g}_opening_assumption_case'
    c['collision']['mesh_directory']=meshes.relative_to(ROOT).as_posix()
    c['provenance']['bay']['source']=(f'Opening width 56 mm, height {height_mm:g} mm; floor and offset hinge retained; ceiling, side walls, frame, door and fuselage cutout enlarged together. Ideal prescribed door, no detailed hinge or structural verification.')
    c['provenance']['aircraft']['source']+=' Opening geometry comparison holds assumed mass, CG and inertia fixed; changed material mass not estimated.'
    case=ROOT/f'examples/h1_opening_{height_mm:g}.yaml'
    case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
    report={'height_mm':height_mm,'width_mm':56,'ceiling_raised_mm':(old_ceiling-ceil)*1000,
        'case':str(case),'roundtrip_valid':restored.isValid(),'solids':len(restored.Solids()),'parts':rows,
        'raw_door_reference_deg':100,'operating_door_deg':140,'tunnel_box_aru_m':tunnel.tolist(),
        'original_cad_sha256':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in originals},
        'builder_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'scope':'Geometry variant only. Assumed mass/inertia and AVL lifting surfaces retained; hole flow and structure unvalidated.'}
    (out/'geometry.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    return {k:v for k,v in report.items() if k not in ('parts','original_cad_sha256')}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--heights',nargs='+',type=float,default=[70,80]);p.add_argument('--workers',type=int,default=2);args=p.parse_args()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for result in pool.map(build,args.heights):print(json.dumps(result),flush=True)
