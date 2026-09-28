"""Rounded sensor nose and X fins, from this project's R3 STEP solids only."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import hashlib,json,shutil
import numpy as np
import cadquery as cq
import yaml
from dbf_stability import load_case
from build_cad import stl,glb

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def main():
    c=load_case(ROOT/'examples/h1_normal_r3_flow5_controlled.yaml')
    original=HERE/'geometry_variants/normal_r3'
    out=HERE/'geometry_variants/normal_r3_round_x_sensor_v2'
    out.mkdir(exist_ok=False)
    cad=out/'cad';mesh=out/'meshes';cad.mkdir();mesh.mkdir()
    files=sorted((original/'cad').glob('*.step'))
    shapes={p.stem:cq.importers.importStep(str(p)).val() for p in files if p.stem!='H1_A_temporary_assembly'}
    body=shapes['sensor_body'];bounds=body.BoundingBox()
    axis=np.array([(bounds.xmin+bounds.xmax)/2,(bounds.ymin+bounds.ymax)/2,(bounds.zmin+bounds.zmax)/2])
    radius=(bounds.ymax-bounds.ymin)/2
    center=np.array([bounds.xmax-radius,axis[1],axis[2]])
    cylinder=cq.Solid.makeCylinder(radius,center[0]-bounds.xmin,cq.Vector(bounds.xmin,*axis[1:]),cq.Vector(1,0,0))
    shapes['sensor_body']=cylinder.fuse(cq.Solid.makeSphere(radius,cq.Vector(*center),angleDegrees1=-90,angleDegrees2=90)).clean()
    for name in shapes:
        if name.startswith('sensor_fin_'):
            shapes[name]=shapes[name].rotate(tuple(axis),tuple(axis+[1,0,0]),45.)
    names=[n for n in shapes if n=='sensor_body' or n.startswith('sensor_fin_')]
    volume=sum(shapes[n].Volume() for n in names)
    mass=c['sensor']['mass_kg']
    cg=sum(shapes[n].Volume()*np.array(shapes[n].Center().toTuple()) for n in names)/volume*.001
    inertia=np.zeros((3,3));records=[]
    for n in names:
        s=shapes[n];m=mass*s.Volume()/volume
        center=np.array(s.Center().toTuple())*.001;delta=center-cg
        tensor=np.array(cq.Shape.matrixOfInertia(s))*m/s.Volume()*1e-6
        inertia+=tensor+m*((delta@delta)*np.eye(3)-np.outer(delta,delta))
        records.append(dict(part=n,mass_kg=m,center_aircraft_frd_m=center.tolist(),inertia_at_part_kgm2=tensor.tolist()))
    old_cg=np.asarray(c['bay']['stowed_center_m']);shift=cg-old_cg
    c['bay']['stowed_center_m']=cg.tolist()
    c['sensor']['tow_point_m']=(np.asarray(c['sensor']['tow_point_m'])-shift).tolist()
    c['sensor']['inertia_kgm2']=inertia.tolist()
    c['sensor']['cg_shift_from_r3_body_m']=shift.tolist()
    assert abs(np.linalg.norm(cg+c['sensor']['tow_point_m']-np.asarray(c['aircraft']['tow_point_m']))-c['winch']['stowed_length_m'])<1e-10
    assembly=cq.Assembly(name='R3_rounded_sensor_X_fins_FRD_mm');parts=[];samples=[]
    for name,shape in shapes.items():
        if not shape.isValid() or shape.Volume()<=0:raise ValueError(f'Invalid solid {name}')
        sensor=name in names
        color=[.91,.63,.17,1] if name.startswith('sensor') else [.18,.53,.73,1] if name.startswith('rear_door') else [.35,.55,.58,.3] if name.startswith('fuselage') else [.55,.60,.65,1]
        if sensor:
            cq.exporters.export(shape,str(cad/f'{name}.step'))
            vertices,faces=shape.tessellate(.25,.15)
            v=np.array([a.toTuple() for a in vertices])*.001;f=np.asarray(faces,int)
            # Spherical poles can tessellate to zero-area triangles. Their
            # removal changes no surface and keeps edge normals well defined.
            tri=v[f];area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
            f=f[area>1e-14]
            stl(mesh/f'{name}_FRD_m.stl',v,f)
            samples.extend((v-cg).tolist())
        else:
            shutil.copy2(original/'cad'/f'{name}.step',cad/f'{name}.step')
            shutil.copy2(original/'meshes'/f'{name}_FRD_m.stl',mesh/f'{name}_FRD_m.stl')
            from dbf_stability.collision import read_binary_stl
            v,f=read_binary_stl(mesh/f'{name}_FRD_m.stl')
        parts.append((name,v,f,color));assembly.add(shape,name=name,color=cq.Color(*color))
    c['sensor']['contact_points_m']=np.unique(np.round(samples,9),axis=0).tolist()
    assembly.export(str(cad/'H1_A_temporary_assembly.step'))
    restored=cq.importers.importStep(str(cad/'H1_A_temporary_assembly.step')).val()
    assert restored.isValid() and len(restored.Solids())==sum(len(s.Solids()) for s in shapes.values())
    glb(mesh/'H1_A_temporary_assembly.glb',parts)
    c['collision']['mesh_directory']=mesh.relative_to(ROOT).as_posix()
    c['name']='R3_round_nose_X_fin_recovery_trial'
    c['provenance']['sensor']['source']='Candidate: unchanged 120 mm outer length and 28 mm body diameter, 14 mm radius hemispherical nose; four existing fins clocked 45 degrees. Assumed total 40 g redistributed uniformly over sensor CAD solids; CG/inertia recomputed. Existing lumped sensor aero coefficients retained as unvalidated assumptions, not newly resolved sensor CFD.'
    c['provenance']['bay']['source']+=' Capture CG coordinate updated for the new sensor mass centre; same physical stowed body pose, same guide/door/fairlead geometry and capture tolerances.'
    c['provenance']['aero']['source']='Reuses actual flow5 7.57 25 m/s R3 lifting-surface database. Aircraft geometry/mass are unchanged. Sensor aerodynamic coefficients remain explicitly assumed.'
    case=ROOT/'examples/h1_normal_r3_round_x_sensor.yaml'
    case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
    load_case(case)
    result=dict(case=str(case),source=str(original),roundtrip_valid=restored.isValid(),solids=len(restored.Solids()),
        sensor_mass_kg=mass,cg_shift_from_r3_body_m=shift.tolist(),inertia_kgm2=inertia.tolist(),mass_records=records,
        unchanged_airframe_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.stem not in names and p.stem!='H1_A_temporary_assembly'},
        builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'geometry.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('mass_records','unchanged_airframe_sha256')},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
