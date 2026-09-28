"""Replace the stepped recovery box with a continuous flared recovery duct.

Real STEP booleans and mass changes; source CAD is preserved. All material
densities/budgets are explicit assumptions, not measured construction data.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse,copy,hashlib,json
import cadquery as cq
import numpy as np
import yaml
from dbf_stability import load_case
from dbf_stability.door import door_hinge,door_reference_shift
from dbf_stability.collision import read_binary_stl
from build_cad import wire,stl,glb
from build_underbody_door import properties,pa
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--version',default='v1');args=p.parse_args()
    if not args.version.isidentifier():raise ValueError('Invalid version')
    c=load_case(ROOT/'examples/h1_underbody_270_v2.yaml');old=copy.deepcopy(c)
    source=(ROOT/c['collision']['mesh_directory']).parent
    out=HERE/f'geometry_variants/normal_r3_recovery_funnel_{args.version}'
    out.mkdir(exist_ok=False);cad=out/'cad';meshes=out/'meshes';cad.mkdir();meshes.mkdir()
    shapes={f.stem:cq.importers.importStep(str(f)).val() for f in (source/'cad').glob('*.step') if f.stem!='H1_A_temporary_assembly'}
    originals=dict(shapes);b=c['bay'];z0=b['stowed_center_m'][2];xe=b['exit_x_m'];xf=b['front_x_m'];xt=b['stowed_center_m'][0]-.025
    # 40 mm lower floor, 10 mm higher roof; no vertical intermediate ledges.
    b['half_width_m']=.040;b['floor_z_m']+=.040;b['ceiling_z_m']-=.010
    stations=[(xe,.040,b['ceiling_z_m'],b['floor_z_m']),
              (xt,.025,z0-.025,z0+.025),(xf+.005,.025,z0-.025,z0+.025)]
    def section(x,w,top,bottom,expand=0.):
        return wire([[x,-w-expand,top-expand],[x,w+expand,top-expand],
                     [x,w+expand,bottom+expand],[x,-w-expand,bottom+expand]])
    outer=cq.Solid.makeLoft([section(*q,.002) for q in stations],ruled=True)
    # Use the exact same entry and throat planes for walls and clearance tool.
    inner_exact=cq.Solid.makeLoft([section(*q) for q in stations],ruled=True)
    duct=outer.cut(inner_exact).clean()
    if not duct.isValid() or len(duct.Solids())!=1:raise ValueError('Invalid duct')
    removed=[n for n in shapes if n.startswith(('guide_','capture_ramp_')) or n in ('rear_exit_frame','capture_lower_pad')]
    for n in removed:shapes.pop(n)
    shapes['recovery_funnel']=duct
    # Remove old bulkhead/floor material along the entire duct, including the
    # upstream entry plane. Exterior shell outside this opening stays present.
    cut_rows=[(xe-.025,*stations[0][1:]),*stations,(xf+.010,*stations[-1][1:])]
    cutter=cq.Solid.makeLoft([section(*q,.0021) for q in cut_rows],ruled=True)
    shell='fuselage_H1_approx_with_rear_cutout'
    shapes[shell]=shapes[shell].cut(cutter).clean()
    # Wider, taller door and laterally relocated hinge supports.
    old_h=1000*door_hinge(old['bay'])
    old_closed=originals['rear_door_100deg'].translate(tuple(1000*door_reference_shift(old['bay']))).rotate(tuple(old_h),tuple(old_h+[0,1,0]),-100)
    b['door_length_m']=b['floor_z_m']-b['ceiling_z_m']+.032
    hinge=1000*door_hinge(b)
    closed=cq.Solid.makeBox(2.,80.,b['door_length_m']*1000,cq.Vector(hinge[0]-1.,-40.,b['ceiling_z_m']*1000))
    closed=closed.cut(cq.Solid.makeCylinder(.7,84.,cq.Vector(hinge[0],-42.,hinge[2]),cq.Vector(0,1,0))).clean()
    shapes['rear_door_100deg']=closed.rotate(tuple(hinge),tuple(hinge+[0,1,0]),100).translate(tuple(-1000*door_reference_shift(b)))
    hardware={'door_hinge_pin':cq.Solid.makeCylinder(.5,88.,cq.Vector(hinge[0],-44.,hinge[2]),cq.Vector(0,1,0))}
    for sign in (-1,1):
        yy=41. if sign>0 else -43.
        cheek=cq.Solid.makeBox(22.,2.,44.,cq.Vector(hinge[0]-4.,yy,b['floor_z_m']*1000-10.))
        hole=cq.Solid.makeCylinder(.7,4.,cq.Vector(hinge[0],yy-1.,hinge[2]),cq.Vector(0,1,0))
        hardware[f'door_hinge_cheek_{sign:+d}']=cheek.cut(hole).clean()
        # Brackets connect to the new duct; upper stop remains 0.2 mm above leaf.
        yy=22. if sign>0 else -26.
        stop_x=(hinge[0]+120.)*.001
        start_z=1000*np.interp(stop_x,[q[0] for q in stations],[q[3] for q in stations])+1.
        end_z=b['floor_z_m']*1000+28.8
        hardware[f'door_fold_stop_{sign:+d}']=cq.Solid.makeBox(6.,4.,end_z-start_z,cq.Vector(hinge[0]+120.,yy,start_z))
    shapes.update(hardware)
    # No residual shell may protrude into the duct's designed free passage.
    obstruction=shapes[shell].intersect(inner_exact).Volume()
    if obstruction>1e-5:raise ValueError(f'Shell obstructs new passage: {obstruction} mm3')
    attachment_gaps={n:s.distance(duct) for n,s in hardware.items() if n!='door_hinge_pin'}
    if max(attachment_gaps.values())>1e-5:raise ValueError(f'Disconnected mounting members: {attachment_gaps}')
    budgets={r['part']:r['mass_kg'] for r in json.loads((HERE/'geometry_variants/normal_r3/mass_properties.json').read_text())['records']}
    source_record=json.loads((source/'geometry.json').read_text());changes=[];mass_records=[]
    def change(name,shape,mass):
        center,I=properties(shape,abs(mass));I*=np.sign(mass)
        changes.append((mass,center,I));mass_records.append(dict(part=name,signed_mass_kg=mass,center_old_frd_m=center.tolist(),inertia_kgm2=I.tolist()))
    for n in removed:change(n,originals[n],-budgets[n])
    change(shell,originals[shell],-.4)
    change(shell,shapes[shell],.4*shapes[shell].Volume()/originals[shell].Volume())
    change('recovery_funnel',duct,duct.Volume()*1e-9*1200.)
    change('rear_door_100deg',old_closed,-source_record['new_door_mass_kg'])
    change('rear_door_100deg',closed,source_record['new_door_mass_kg']*closed.Volume()/old_closed.Volume())
    old_hardware={r['part']:r['mass_kg'] for r in source_record['hardware_records']}
    total_volume=sum(s.Volume() for s in hardware.values())
    # Larger mounting members: preserve the source hardware's effective density.
    old_volume=sum(originals[n].Volume() for n in hardware)
    density=source_record['hardware_mass_kg']/old_volume
    for n,s in hardware.items():change(n,originals[n],-old_hardware[n]);change(n,s,density*s.Volume())
    mass=old['aircraft']['mass_kg']+sum(m for m,_,_ in changes)
    shift=sum(m*r for m,r,_ in changes)/mass
    I=np.asarray(old['aircraft']['inertia_kgm2'])+sum(J+m*pa(r) for m,r,J in changes)-mass*pa(shift)
    if np.min(np.linalg.eigvalsh(I))<=0:raise ValueError('Invalid aircraft inertia')
    a=c['aircraft'];a['mass_kg']=float(mass);a['inertia_kgm2']=I.tolist()
    for key in ('cg_m','cg_shift_from_r3_body_m'):a[key]=(np.asarray(a[key])+shift).tolist()
    for key in ('tow_point_m','thrust_point_m'):a[key]=(np.asarray(a[key])-shift).tolist()
    b['stowed_center_m']=(np.asarray(b['stowed_center_m'])-shift).tolist()
    for key in ('exit_x_m','front_x_m'):b[key]-=float(shift[0])
    for key in ('floor_z_m','ceiling_z_m'):b[key]-=float(shift[2])
    modified=set(removed)|set(hardware)|{shell,'recovery_funnel','rear_door_100deg'}
    assembly=cq.Assembly(name='R3_flared_recovery_duct_FRD_mm');opened=cq.Assembly(name='R3_flared_recovery_270_FRD_mm');parts=[]
    for n,s in shapes.items():
        if not s.isValid() or len(s.Solids())!=1:raise ValueError(f'Invalid solid {n}')
        s=s.translate(tuple(-1000*shift));cq.exporters.export(s,str(cad/f'{n}.step'))
        if n in modified:
            vv,ff=s.tessellate(.08,.12);v=np.array([p.toTuple() for p in vv])*.001;f=np.asarray(ff,int)
            tri=v[f];f=f[np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)>1e-14]
        else:v,f=read_binary_stl(source/'meshes'/f'{n}_FRD_m.stl');v-=shift
        stl(meshes/f'{n}_FRD_m.stl',v,f)
        color=[.10,.48,.72,1] if n.startswith(('rear_door','door_')) else [.14,.63,.47,.65] if n=='recovery_funnel' else [.88,.28,.32,1] if n=='capture_front_stop' else [.91,.63,.17,1] if n.startswith('sensor') else [.35,.55,.58,.3] if n.startswith('fuselage') else [.55,.60,.65,1]
        assembly.add(s,name=n,color=cq.Color(*color));parts.append((n,v,f,color))
        if n=='rear_door_100deg':
            h=1000*door_hinge(b);s=s.translate(tuple(1000*door_reference_shift(b))).rotate(tuple(h),tuple(h+[0,1,0]),170)
        opened.add(s,name=n,color=cq.Color(*color))
    assembly.export(str(cad/'H1_A_temporary_assembly.step'));opened.export(str(out/'H1_recovery_funnel_open.step'))
    restored=cq.importers.importStep(str(cad/'H1_A_temporary_assembly.step')).val()
    if not restored.isValid() or len(restored.Solids())!=len(shapes):raise ValueError('STEP roundtrip')
    glb(meshes/'H1_A_temporary_assembly.glb',parts)
    c['collision']['mesh_directory']=meshes.relative_to(ROOT).as_posix();c['collision']['concave_parts'].append('recovery_funnel')
    spec=c['collision']['analytic_conical_stop'];spec['back_x_m']-=float(shift[0]);spec['front_x_m']-=float(shift[0]);spec['center_yz_m']=(np.asarray(spec['center_yz_m'])-shift[1:]).tolist()
    spec['cad_sha256']={n:hashlib.sha256((cad/n).read_bytes()).hexdigest() for n in ('sensor_body.step','capture_front_stop.step')}
    c['name']='R3_flared_recovery_duct_'+args.version
    c['provenance']['bay']['source']='Actual CAD flared recovery duct: 80x120 mm mouth, continuous taper to 50x50 mm throat, 2 mm walls. Old rectangular frame, box guides and separate ramps removed. Fuselage cutout follows duct; 152x80 mm door, 270 degree fold. Capture thresholds and contact laws unchanged. Structural strength and local airflow unverified.'
    c['provenance']['aircraft']['source']+=' Flared duct assumed density 1200 kg/m3; removed original component mass budgets, shell mass scaled by removed volume, door/support source effective densities retained. Recomputed CAD mass, CG and inertia at closed door pose.'
    case=ROOT/f'examples/h1_recovery_funnel_{args.version}.yaml';case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8');load_case(case)
    record=dict(case=str(case),source=str(source),mouth_mm=[80.,120.],throat_mm=[50.,50.],floor_drop_mm=40.,roof_raise_mm=10.,wall_mm=2.,door_mm=[80.,152.,2.],duct_mass_kg=duct.Volume()*1e-9*1200.,aircraft_mass_kg=mass,mass_increment_kg=mass-old['aircraft']['mass_kg'],cg_shift_m=shift.tolist(),mass_changes=mass_records,removed_parts=removed,stations_old_frd_m=stations,shell_obstruction_mm3=obstruction,mount_to_duct_gap_mm=attachment_gaps,roundtrip_valid=True,solids=len(shapes),builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    for k in ('cone_length_mm','cone_slope','mouth_diameter_mm','cable_bore_diameter_mm'):record[k]=source_record[k]
    (out/'geometry.json').write_text(json.dumps(record,indent=2));print(json.dumps({k:v for k,v in record.items() if k!='mass_changes'}),flush=True)

if __name__=='__main__':main()
