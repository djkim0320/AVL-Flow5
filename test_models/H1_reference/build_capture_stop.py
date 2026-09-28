"""Physical conical nose stop with a cable bore; include its mass and CG shift."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import json,hashlib,argparse
import numpy as np
import cadquery as cq
import yaml
from dbf_stability import load_case
from dbf_stability.collision import read_binary_stl
from build_cad import stl,glb

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def parallel_axis(r):return (r@r)*np.eye(3)-np.outer(r,r)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bore-mm',type=float,default=16.)
    parser.add_argument('--version',default='v2')
    args=parser.parse_args()
    if not 6<=args.bore_mm<28:raise ValueError('Bore must pass the cable while retaining the 28 mm sensor nose')
    if not args.version.isidentifier():raise ValueError('Invalid geometry version')
    c=load_case(ROOT/'examples/h1_normal_r3_round_x_sensor.yaml')
    source=(ROOT/c['collision']['mesh_directory']).parent
    out=HERE/f'geometry_variants/normal_r3_round_x_capture_stop_{args.version}';out.mkdir(exist_ok=False)
    cad=out/'cad';meshes=out/'meshes';cad.mkdir();meshes.mkdir()
    shapes={p.stem:cq.importers.importStep(str(p)).val() for p in (source/'cad').glob('*.step') if p.stem!='H1_A_temporary_assembly'}
    b=c['bay'];nose=np.asarray(b['stowed_center_m'])+np.asarray(c['sensor']['tow_point_m'])
    radius=14.;bore=args.bore_mm/2;mouth=22.;tip_gap=1.5;normal_clearance=.9
    # Choose a cone tangent to the hemispherical nose at a 0.9 mm clearance.
    # r_bore + slope*(r_nose+tip_gap) = (r_nose+clearance)*sqrt(1+slope**2).
    distance=radius+tip_gap;effective=radius+normal_clearance
    roots=np.roots([distance**2-effective**2,2*bore*distance,bore**2-effective**2])
    slope=float(max(roots));length=(mouth-bore)/slope
    front=1000*nose[0]+tip_gap;back=front-length;thickness=4.
    body=cq.Solid.makeBox(length+thickness,2*b['half_width_m']*1000,(b['floor_z_m']-b['ceiling_z_m'])*1000,
        cq.Vector(back,-b['half_width_m']*1000,b['ceiling_z_m']*1000))
    funnel=cq.Solid.makeCone(mouth,bore,length,cq.Vector(back,nose[1]*1000,nose[2]*1000),cq.Vector(1,0,0))
    hole=cq.Solid.makeCylinder(bore,thickness+1,cq.Vector(front-.01,nose[1]*1000,nose[2]*1000),cq.Vector(1,0,0))
    stop=body.cut(funnel.fuse(hole)).clean()
    assert stop.isValid() and len(stop.Solids())==1
    sensor=cq.Compound.makeCompound([s for n,s in shapes.items() if n=='sensor_body' or n.startswith('sensor_fin_')])
    stowed_gap=stop.distance(sensor)*.001;overlap=stop.intersect(sensor).Volume()
    assert stowed_gap>.0008 and overlap<1e-5,(stowed_gap,overlap)
    mass=.020  # Explicit module mass budget, not an asserted material density.
    center=np.asarray(stop.Center().toTuple())*.001
    inertia=np.asarray(cq.Shape.matrixOfInertia(stop))*mass/stop.Volume()*1e-6
    previous_mass=c['aircraft']['mass_kg'];total=previous_mass+mass;shift=mass*center/total
    c['aircraft']['inertia_kgm2']=(np.asarray(c['aircraft']['inertia_kgm2'])+previous_mass*parallel_axis(shift)+inertia+mass*parallel_axis(center-shift)).tolist()
    c['aircraft']['mass_kg']=total
    c['aircraft']['cg_m']=(np.asarray(c['aircraft']['cg_m'])+shift).tolist()
    c['aircraft']['cg_shift_from_r3_body_m']=shift.tolist()
    for key in ('tow_point_m','thrust_point_m'):c['aircraft'][key]=(np.asarray(c['aircraft'][key])-shift).tolist()
    b['stowed_center_m']=(np.asarray(b['stowed_center_m'])-shift).tolist()
    for key in ('exit_x_m','front_x_m'):b[key]-=float(shift[0])
    for key in ('floor_z_m','ceiling_z_m'):b[key]-=float(shift[2])
    shapes['capture_front_stop']=stop
    assembly=cq.Assembly(name='R3_conical_capture_stop_FRD_mm');parts=[]
    for name,shape in shapes.items():
        shape=shape.translate(tuple(-1000*shift));cq.exporters.export(shape,str(cad/f'{name}.step'))
        if name=='capture_front_stop':
            vertices,faces=shape.tessellate(.15,.15);v=np.array([p.toTuple() for p in vertices])*.001;f=np.asarray(faces,int)
            triangles=v[f];area=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1);f=f[area>1e-14]
        else:v,f=read_binary_stl(source/'meshes'/f'{name}_FRD_m.stl');v-=shift
        stl(meshes/f'{name}_FRD_m.stl',v,f)
        color=[.88,.28,.32,1] if name=='capture_front_stop' else [.91,.63,.17,1] if name.startswith('sensor') else [.18,.53,.73,1] if name.startswith('rear_door') else [.35,.55,.58,.3] if name.startswith('fuselage') else [.55,.60,.65,1]
        assembly.add(shape,name=name,color=cq.Color(*color));parts.append((name,v,f,color))
    assembly.export(str(cad/'H1_A_temporary_assembly.step'));restored=cq.importers.importStep(str(cad/'H1_A_temporary_assembly.step')).val()
    assert restored.isValid() and len(restored.Solids())==len(shapes)
    glb(meshes/'H1_A_temporary_assembly.glb',parts)
    c['collision']['mesh_directory']=meshes.relative_to(ROOT).as_posix()
    c['collision']['concave_parts'].append('capture_front_stop');c['collision']['feature_engine']='numba'
    c['name']='R3_conical_nose_stop_recovery_test'
    c['provenance']['aircraft']['source']+=' Added 20 g assumed capture-stop mass, with its CAD-derived CG and inertia. Aircraft CG, inertia and all aircraft-relative geometry/tow/thrust coordinates updated; aerodynamic moment coefficients retained at their original physical reference point.'
    c['provenance']['bay']['source']+=f' Physical conical nose stop with 44 mm mouth, {args.bore_mm:g} mm through-bore for the deflecting cable and 0.9 mm stowed normal clearance. Same 6 mm / 0.15 m/s / 8 degree capture thresholds. Stop mass is an assumed 20 g module budget; strength and impact response are unvalidated.'
    case=ROOT/f'examples/h1_round_x_capture_stop_{args.version}.yaml';case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8');load_case(case)
    record=dict(case=str(case),source=str(source),mass_kg=mass,center_in_old_aircraft_frd_m=center.tolist(),aircraft_cg_shift_m=shift.tolist(),
        aircraft_mass_kg=total,stop_inertia_at_center_kgm2=inertia.tolist(),stowed_CAD_gap_m=stowed_gap,stowed_overlap_mm3=overlap,
        cone_length_mm=length,cone_slope=slope,mouth_diameter_mm=2*mouth,cable_bore_diameter_mm=2*bore,
        roundtrip_valid=restored.isValid(),solids=len(restored.Solids()),builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'geometry.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(record,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
