"""H1 R2: local copied geometry, explicit mass assumptions, reproducible CAD/AVL.

Never reads the original modelling project. SI in inputs; CAD STEP uses mm.
"""
import os
for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
from pathlib import Path
import json
import hashlib
import csv
import shutil
import numpy as np
import cadquery as cq
from scipy.interpolate import PchipInterpolator
import yaml
from dbf_stability import load_case
from dbf_stability.avl import write_mass_file
from build_cad import box, wire, stl, glb

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / 'geometry_variants/refined_r2'


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding='utf8')


def body_outline(body, x, inset=0.):
    vals = {k: float(PchipInterpolator(body['x'], body[k])(x))
            for k in ('top', 'shoulder', 'bottom')}
    w = float(PchipInterpolator(body['halfwidth_x'], body['halfwidth'])(x)) - inset
    top, shoulder, bottom = vals['top']-inset, vals['shoulder'], vals['bottom']+inset
    radius = min(.016, .3*w, .3*(shoulder-bottom))
    yz = [(w*np.cos(a), shoulder+(top-shoulder)*np.sin(a)) for a in np.linspace(0, np.pi, 25)]
    yz.append((-w, bottom+radius))
    yz.extend([(-w+radius+radius*np.cos(a), bottom+radius+radius*np.sin(a))
               for a in np.linspace(np.pi, 1.5*np.pi, 9)[1:]])
    yz.append((w-radius, bottom))
    yz.extend([(w-radius+radius*np.cos(a), bottom+radius+radius*np.sin(a))
               for a in np.linspace(-np.pi/2, 0, 9)[1:]])
    return np.array(yz)


def foil_wire(foil, x, y, z, chord, incidence):
    angle = np.deg2rad(incidence)
    q, n = foil.T*chord
    return wire(np.c_[x+q*np.cos(angle)+n*np.sin(angle), np.full(len(q), y),
                      z-q*np.sin(angle)+n*np.cos(angle)])


def section(x, y, z, chord, incidence, foil, panels, control=None):
    s = f'SECTION\n{x:.9f} {y:.9f} {z:.9f} {chord:.9f} {incidence:.6f} {panels} 1\nAFILE\n{foil}\n'
    if control:
        s += 'CONTROL\n'+control+'\n'
    return s


def write_avl(p, cg_aru, path, scale=1., body=True):
    w, h = p['wing_section'], p['tail_section']
    text = ('H1 R2 section-based lifting surfaces and equivalent-area body\n0.0\n0 0 0\n'
            f'0.55 {w["chord_m"]:.9f} 1.8\n'+' '.join(f'{v:.9f}' for v in cg_aru)+'\n0.0\n')
    # Preserve sourced planform/incidence, resolve control boundaries and use finer panels.
    def n(value): return max(1, int(round(value*scale)))
    text += f'SURFACE\nH1_Wing\n{n(18)} 1\nYDUPLICATE\n0\n'
    for y, count, control in [(0,8,None),(.25,8,None),(.5316,12,'aileron 1 .75 0 1 0 -1'),
                              (.8794,3,'aileron 1 .75 0 1 0 -1'),(.9,1,None)]:
        text += section(w['leading_edge_m'][0],y,w['leading_edge_m'][2],w['chord_m'],2,'h1_wing.dat',n(count),control)
    text += f'SURFACE\nH1_Horizontal_tail\n{n(14)} 1\nYDUPLICATE\n0\n'
    for y,count,control in [(0,3,None),(.0365,10,'elevator 1 .75 0 1 0 1'),
                            (.18,10,'elevator 1 .75 0 1 0 1'),(.3145,4,'elevator 1 .75 0 1 0 1'),(.35,1,None)]:
        text += section(h['leading_edge_m'][0],y,h['leading_edge_m'][2],h['chord_m'],0,'h1_tail.dat',n(count),control)
    text += f'SURFACE\nH1_Vertical_fin\n{n(14)} 1\n'
    # Root-to-tip ordering and hinge convention retained from the checked H1 model.
    for z in (.03,.13,.23,.33):
        f=(z-.03)/.30
        text += section(1.1525+.035*f,0,z,.27-.14*f,0,'h1_fin.dat',n(8),'rudder 1 .75 0 0 1 1')
    if body:
        text += f'BODY\nH1_equivalent_area_fuselage\n{n(40)} 1\nBFILE\nfuselage_equivalent.dat\n'
    path.write_text(text, encoding='ascii')


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    cad, meshes, aero = [OUT/name for name in ('cad','meshes','aero')]
    for folder in (cad, meshes, aero): folder.mkdir()
    c = load_case(ROOT/'examples/h1_opening_70.yaml')
    p = json.loads((HERE/'parameters.json').read_text(encoding='utf8'))
    cg_old = np.array(p['aircraft_cg_aru_m'])
    transform = np.diag([-1.,1.,-1.])
    def to_old(shape):
        return shape.rotate((0,0,0),(0,1,0),180).translate(tuple(cg_old*1000))
    originals = sorted((HERE/'geometry_variants/opening_70/cad').glob('*.step'))
    shapes = {f.stem:cq.importers.importStep(str(f)).val() for f in originals if f.stem!='H1_A_temporary_assembly'}

    # Interpolate the copied shape stations without overshoot, retaining a real hollow shell.
    body = p['source_body_sections']
    xs = np.unique(np.r_[np.linspace(0,1.5,51), body['x']])
    def outline(x, inset=0): return wire([[x,y,z] for y,z in body_outline(body,x,inset)])
    outer = cq.Solid.makeLoft([outline(x) for x in xs], ruled=True)
    inner = cq.Solid.makeLoft([outline(x,.0018) for x in xs], ruled=True)
    tunnel = [[.76,-.031,-.062],[1.6,.031,.036]]
    # Forward extension houses the relocated reel and fairlead; no empty solid hull.
    shapes['fuselage_H1_approx_with_rear_cutout'] = to_old(outer.cut(inner).cut(box(*tunnel)))

    # Rebuild lifting CAD with chord-normal hinge cuts and a 0.6 mm control gap.
    for typ, halfspan, edges, angle, prefix in [('wing',.9,(.5316,.8794),2,'aileron'),
                                             ('tail',.35,(.0365,.3145),0,'elevator')]:
        sec=p[typ+'_section'];x,_,z=sec['leading_edge_m'];ch=sec['chord_m']
        foil=np.loadtxt(HERE/f'geometry/h1_{typ}.dat',skiprows=1)
        whole=cq.Solid.makeLoft([foil_wire(foil,x,y,z,ch,angle) for y in [-halfspan,0,halfspan]],ruled=True)
        fixed=whole
        for sign in (-1,1):
            lo,hi=sorted([sign*edges[0],sign*edges[1]])
            cut=box([x+.75*ch,lo,z-.1],[x+ch+.025,hi,z+.1]).rotate((x*1000,0,z*1000),(x*1000,1000,z*1000),angle)
            leaf=whole.intersect(cut)
            expanded=box([x+.75*ch-.0003,lo-.0003,z-.1],[x+ch+.03,hi+.0003,z+.1]).rotate((x*1000,0,z*1000),(x*1000,1000,z*1000),angle)
            fixed=fixed.cut(expanded)
            shapes[f'{prefix}_{sign:+d}_CAD_approx']=to_old(leaf)
        shapes['main_wing_CAD_approx' if typ=='wing' else 'horizontal_tail_CAD_approx']=to_old(fixed)

    # Feed point is now outside the drum. The stored portion of line is implicit in the reel.
    feed_aru=np.array([.82,0.,-.02]); drum_aru=np.array([.79,0.,-.005])
    feed=(feed_aru-cg_old)@transform
    shapes['winch_drum']=to_old(cq.Solid.makeCylinder(15,24,cq.Vector(*(drum_aru*1000+[0,-12,0])),cq.Vector(0,1,0)))
    # Centred longitudinal motor housing; an ideal right-angle drive is assumed.
    shapes['winch_motor']=to_old(cq.Solid.makeCylinder(10,22,cq.Vector(744,0,-5),cq.Vector(1,0,0)))
    shapes['aircraft_tow_point']=cq.Solid.makeSphere(1.5,cq.Vector(*(feed*1000)))
    ring=cq.Solid.makeCylinder(7,4,cq.Vector(*(feed*1000-[2,0,0])),cq.Vector(1,0,0)).cut(
        cq.Solid.makeCylinder(3,6,cq.Vector(*(feed*1000-[3,0,0])),cq.Vector(1,0,0)))
    shapes['fairlead_ring']=ring

    # Passive four-sided lead-in: 56x70 at entry to a 50x50 mm throat before stow.
    b=c['bay']; z0=b['stowed_center_m'][2]; xe=b['exit_x_m']+.004
    xt=b['stowed_center_m'][0]-.065; xf=b['front_x_m']
    hw=b['half_width_m']; ztop=b['ceiling_z_m']; zbottom=b['floor_z_m']
    def prism(points):
        return cq.Solid.makeLoft([wire(a) for a in points],ruled=True)
    for side in (-1,1):
        def wall(x, inner):
            low,high=sorted([side*inner, side*hw])
            return [[x,low,ztop],[x,high,ztop],[x,high,zbottom],[x,low,zbottom]]
        shapes[f'capture_ramp_side_{side:+d}']=prism([wall(xe,hw-.0001),wall(xt,.025),wall(xf,.025)])
    def horizontal(x, low, high): return [[x,-.025,low],[x,.025,low],[x,.025,high],[x,-.025,high]]
    shapes['capture_ramp_top']=prism([horizontal(xe,ztop,ztop+.0001),horizontal(xt,ztop,z0-.025),horizontal(xf,ztop,z0-.025)])
    shapes['capture_ramp_bottom']=prism([horizontal(xe,zbottom-.0001,zbottom),horizontal(xt,z0+.025,zbottom),horizontal(xf,z0+.025,zbottom)])

    # Component masses remain explicit assumptions. Use actual CAD centres and tensors
    # inside each structural group rather than full-span bounding-box inertias.
    groups={'wing':(.50, [n for n in shapes if n.startswith(('main_wing','aileron_'))]),
            'fuselage':(.40, ['fuselage_H1_approx_with_rear_cutout']),
            'tail_group':(.14,[n for n in shapes if n.startswith(('horizontal_tail','elevator_','vertical_fin','rudder_'))]),
            'bay_door_guides_capture':(.090,[n for n in shapes if n.startswith(('guide_','capture_','rear_exit','rear_door','fairlead'))]),
            'winch_motor_drum':(.10,['winch_drum','winch_motor'])}
    # Mass properties for the door are computed closed at the actual hinge offsets.
    from dbf_stability.door import door_hinge, door_reference_shift
    mass_shapes=dict(shapes); hinge=1000*door_hinge(b)
    mass_shapes['rear_door_100deg']=shapes['rear_door_100deg'].translate(tuple(1000*door_reference_shift(b))).rotate(tuple(hinge),tuple(hinge+[0,1,0]),-100)
    records=[]
    for group,(mass,names) in groups.items():
        total_volume=sum(mass_shapes[n].Volume() for n in names)
        for name in names:
            shape=mass_shapes[name];m=mass*shape.Volume()/total_volume
            center=np.array(shape.Center().toTuple())*.001
            tensor=np.asarray(cq.Shape.matrixOfInertia(shape))*m/shape.Volume()*1e-6
            records.append(dict(part=name,group=group,mass_kg=m,center_old_frd_m=center.tolist(),inertia_at_part_kgm2=tensor.tolist(),kind='assumption',source='Assumed group mass distributed uniformly over CAD volumes'))
    with (HERE/'mass_budget.csv').open(encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            if row['part'] in groups: continue
            mass=float(row['mass_kg']);aru=np.array([float(row[k+'_aru_m']) for k in 'xyz'])
            dim=np.array([float(row[k+'_m']) for k in ('length','width','height')])
            center=(aru-cg_old)@transform
            tensor=np.diag(mass/12*(sum(dim*dim)-dim*dim))
            records.append(dict(part=row['part'],group=row['part'],mass_kg=mass,center_old_frd_m=center.tolist(),inertia_at_part_kgm2=tensor.tolist(),kind='assumption',source='Retained explicit component box mass budget'))
    mass=sum(r['mass_kg'] for r in records)
    cg=sum(r['mass_kg']*np.array(r['center_old_frd_m']) for r in records)/mass
    inertia=np.zeros((3,3))
    for row in records:
        delta=np.array(row['center_old_frd_m'])-cg
        inertia+=np.array(row['inertia_at_part_kgm2'])+row['mass_kg']*((delta@delta)*np.eye(3)-np.outer(delta,delta))
    cg_aru=cg_old+transform@cg
    # Shift all CAD and all aircraft-relative references together to the new CG.
    shapes={n:s.translate(tuple(-1000*cg)) for n,s in shapes.items()}
    c['aircraft'].update(mass_kg=mass,inertia_kgm2=inertia.tolist(),cg_m=(transform@cg_aru).tolist(),tow_point_m=(feed-cg).tolist())
    c['aircraft']['thrust_point_m']=(np.array(c['aircraft']['thrust_point_m'])-cg).tolist()
    b['stowed_center_m']=(np.array(b['stowed_center_m'])-cg).tolist()
    for key in ('front_x_m','exit_x_m'): b[key]=float(b[key]-cg[0])
    for key in ('floor_z_m','ceiling_z_m'): b[key]=float(b[key]-cg[2])
    b['release_push_until_s']=3.
    c['collision'].update(mesh_directory=meshes.relative_to(ROOT).as_posix(),concave_parts=['fairlead_ring'])
    c['winch']['line_attaches_to_drum_center']=False
    natural=float(np.linalg.norm(np.array(b['stowed_center_m'])+c['sensor']['tow_point_m']-c['aircraft']['tow_point_m']))
    c['winch']['stowed_length_m']=natural
    # Open-loop reel command with a slow approach, zero terminal speed; no aircraft controller.
    knots=[(0,natural),(.6,natural),(2.5,1.5),(3.,1.5),(5.5,.42),(9.,natural),(10.,natural),(11.,natural)]
    times=np.unique(np.r_[np.arange(0,11.0001,.1),[q[0] for q in knots]])
    values=PchipInterpolator(*np.array(knots).T)(times)
    c['winch']['length_schedule']=[[float(t),float(np.clip(v,natural,1.5))] for t,v in zip(times,values)]
    c['winch']['door_schedule']=[[0,0],[.4,140],[9.8,140],[10.6,0],[11,0]]
    c['simulation']['duration_s']=11.
    c['name']='H1_R2_refined_geometry_assumed_properties'
    c['aero']['geometry']=(aero/'h1_r2.avl').relative_to(ROOT).as_posix()
    c['aero']['moment_reference_frd_m']=(transform@cg_aru).tolist()
    c['aero']['alpha_deg']=[-6.,-4.,-2.,0.,2.,4.,6.,9.,12.]
    c['aero']['beta_deg']=[-8.,-4.,0.,4.,8.]
    c['aero']['elevator_deg']=[-20.,-10.,-5.,0.,5.,10.,20.]
    c['provenance']['aircraft']['source']='R2 CAD-weighted assumed component masses and inertias; 90 g bay group, 100 g relocated reel group. Door inertia frozen at closed pose; real mass and materials unmeasured.'
    c['provenance']['bay']['source']='56x70 mm rear opening, 140-degree offset door, passive 50x50 mm capture throat, forward fairlead; geometric design assumptions, no structure/material qualification.'
    c['provenance']['winch']['source']='Feed point outside drum, forward relocated winch; prescribed decelerating length schedule with slow final approach. No torque-limited motor dynamics. Release push stops before retrieval.'
    c['provenance']['aero']['source']='Actual AVL 3.52; refined sourced wing/tail panels and equivalent-area slender-body fuselage. Body/no-body sensitivity required. Profile drag and open-door wake remain unmeasured. https://web.mit.edu/drela/Public/web/avl/avl_doc.txt'
    for name in ('h1_wing.dat','h1_tail.dat','h1_fin.dat'): shutil.copy2(HERE/'geometry'/name,aero/name)
    # Equivalent-area circular body required by AVL; external envelope, not hollow material area.
    areas=[]
    for x in xs:
        yz=body_outline(body,x); area=.5*abs(np.sum(yz[:,0]*np.roll(yz[:,1],-1)-yz[:,1]*np.roll(yz[:,0],-1)))
        areas.append(area)
    radii=np.sqrt(np.array(areas)/np.pi)
    body_points=np.vstack([np.c_[xs[::-1],radii[::-1]],np.c_[xs,-radii]])
    np.savetxt(aero/'fuselage_equivalent.dat',body_points,header='H1 R2 equivalent-area body envelope',comments='',fmt='%.9f')
    write_avl(p,cg_aru,aero/'h1_r2.avl')
    write_avl(p,cg_aru,aero/'h1_r2_no_body.avl',body=False)
    write_avl(p,cg_aru,aero/'h1_r2_coarse.avl',scale=.7)
    write_avl(p,cg_aru,aero/'h1_r2_fine.avl',scale=1.4)
    case=ROOT/'examples/h1_refined_r2.yaml'
    case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
    c=load_case(case)
    for name in ('h1_r2','h1_r2_no_body','h1_r2_coarse','h1_r2_fine'): write_mass_file(c,aero/(name+'.mass'))
    write_mass_file(c,aero/'h1_r2_stowed.mass',stowed=True)
    assembly=cq.Assembly(name='H1_R2_refined_FRD_mm');parts=[];validation=[]
    for name,shape in shapes.items():
        if not shape.isValid() or shape.Volume()<=0: raise ValueError(f'Invalid R2 CAD: {name}')
        color=[.92,.63,.15,1] if name.startswith('sensor') else [.18,.53,.73,1] if name.startswith('rear_door') else [.26,.55,.58,.3] if name.startswith('fuselage') else [.7,.75,.72,1] if 'wing' in name or 'tail' in name else [.5,.55,.6,1]
        cq.exporters.export(shape,str(cad/(name+'.step')))
        vertices,faces=shape.tessellate(.18,.12)
        v=np.array([a.toTuple() for a in vertices])*.001;f=np.array(faces,int)
        stl(meshes/(name+'_FRD_m.stl'),v,f);parts.append((name,v,f,color));assembly.add(shape,name=name,color=cq.Color(*color))
        validation.append(dict(part=name,valid=True,solids=len(shape.Solids()),volume_mm3=shape.Volume()))
    assembly.export(str(cad/'H1_A_temporary_assembly.step'))
    restored=cq.importers.importStep(str(cad/'H1_A_temporary_assembly.step')).val()
    if not restored.isValid() or len(restored.Solids())!=sum(r['solids'] for r in validation): raise ValueError('STEP roundtrip failed')
    glb(meshes/'H1_A_temporary_assembly.glb',parts)
    dump(OUT/'mass_properties.json',dict(records=records,aircraft_mass_kg=mass,cg_aru_m=cg_aru.tolist(),inertia_frd_kgm2=inertia.tolist(),old_to_new_cg_frd_m=cg.tolist(),sensor_and_cable_in_aircraft_mass=False,door_mass_pose='closed; prescribed door does not update inertia'))
    dump(OUT/'geometry.json',dict(case=str(case),roundtrip_valid=True,solids=len(restored.Solids()),parts=validation,
        body_stations=len(xs),body_area_table=list(zip(xs.tolist(),areas)),guide_throat_mm=[50,50],opening_mm=[56,70],
        feed_aru_m=feed_aru.tolist(),drum_aru_m=drum_aru.tolist(),tunnel_box_aru_m=tunnel,
        raw_door_reference_deg=100,operating_door_deg=140,source_project_accessed=False,
        builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_cad_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in originals},
        limitations=['Assumed component masses and effective CAD volume distributions','Equivalent-area body omits bay/door wake and section asymmetry','Compliant abstract latch; no jaw actuator dynamics','Prescribed door and reel; no structural stress or material calibration']))
    print(json.dumps(dict(case=str(case),parts=len(shapes),mass_kg=mass,cg_aru_m=cg_aru.tolist(),stowed_line_m=natural,body_stations=len(xs)),indent=2),flush=True)


if __name__=='__main__': main()
