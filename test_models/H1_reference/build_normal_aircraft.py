"""Conventional high-wing R3 aircraft, explicit test design, reproducible CAD/AVL.

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
OUT = HERE / 'geometry_variants/normal_r3'

# Original-nose aft/right/up coordinates. A new assumed airframe, not team CAD.
AIRFRAME = {
    'x':[0.,.045,.14,.30,.50,.72,.88,1.02,1.142],
    'halfwidth_x':[0.,.045,.14,.30,.50,.72,.88,1.02,1.142],
    'halfwidth':[.009,.035,.065,.082,.086,.080,.066,.050,.037],
    'top':[.014,.046,.071,.092,.105,.100,.095,.086,.080],
    'shoulder':[.003,.009,.022,.033,.036,.030,.025,.020,.012],
    'bottom':[-.014,-.044,-.075,-.098,-.100,-.095,-.078,-.066,-.058],
}


def wing_station(y, tail=False):
    y=abs(y)
    if tail:
        f=y/.35
        return 1.22+.03*f, .075+.010*f, .23-.06*f, 0.
    f=max(0.,(y-.1)/.8)
    return .40+.045*f, .105+.8*f*np.tan(np.deg2rad(4)), .335-.060*f, 2.-f


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
    area=.2*.335+1.6*(.335+.275)/2
    mac=(.2*.335**2+1.6*(.335**2+.335*.275+.275**2)/3)/area
    text = ('Normal R3 conventional high-wing test aircraft\n0.0\n0 0 0\n'
            f'{area:.9f} {mac:.9f} 1.8\n'+' '.join(f'{v:.9f}' for v in cg_aru)+'\n0.0\n')
    def n(value): return max(1, int(round(value*scale)))
    for tail,name,nch,stations in [
        (False,'R3_Main_wing',18,[(0,4,False),(.1,12,False),(.52,12,True),(.86,3,True),(.9,1,False)]),
        (True,'R3_Horizontal_tail',14,[(0,4,False),(.05,16,True),(.325,3,True),(.35,1,False)])]:
        text+=f'SURFACE\n{name}\n{n(nch)} 1\nYDUPLICATE\n0\n'
        for y,count,controlled in stations:
            x,z,ch,inc=wing_station(y,tail)
            control=('elevator 1 .75 0 1 0 1' if tail else 'aileron 1 .75 0 1 0 -1') if controlled else None
            text+=section(x,y,z,ch,inc,'h1_tail.dat' if tail else 'h1_wing.dat',n(count),control)
    text += f'SURFACE\nR3_Vertical_fin\n{n(14)} 1\n'
    for z in (.080,.180,.280,.370):
        f=(z-.080)/.290
        text += section(1.230+.110*f,0,z,.280-.150*f,0,'h1_fin.dat',n(8),'rudder 1 .75 0 0 1 1')
    if body:
        text += f'BODY\nR3_equivalent_area_fuselage\n{n(40)} 1\nBFILE\nfuselage_equivalent.dat\n'
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

    # Closed cabin and rear bulkhead. Only the door aperture is open; the tail
    # boom joins above it, so no long open slot is cut through the underside.
    body = AIRFRAME
    xs = np.array(body['x'])
    def outline(x, inset=0): return wire([[x,y,z] for y,z in body_outline(body,x,inset)])
    outer = cq.Solid.makeLoft([outline(x) for x in xs], ruled=False)
    inner_x=np.r_[.025,xs[1:-1],1.136]
    inner = cq.Solid.makeLoft([outline(x,.0018) for x in inner_x], ruled=False)
    boom_stations=[(.94,.059,.033),(1.14,.066,.027),(1.32,.072,.021),(1.48,.075,.014),(1.56,.075,.008)]
    def boom(inset):
        return cq.Solid.makeLoft([cq.Wire.makeCircle((r-inset)*1000,cq.Vector(x*1000,0,z*1000),cq.Vector(1,0,0)) for x,z,r in boom_stations],ruled=False)
    envelope=outer.fuse(boom(0))
    cavity=inner.fuse(boom(.0015))
    tunnel=[[1.135,-.028,-.048],[1.155,.028,.022]]
    shapes['fuselage_H1_approx_with_rear_cutout']=to_old(envelope.cut(cavity).cut(box(*tunnel)))

    # Rebuild lifting CAD with chord-normal hinge cuts and a 0.6 mm control gap.
    for typ, halfspan, edges, prefix in [('wing',.9,(.52,.86),'aileron'),
                                       ('tail',.35,(.05,.325),'elevator')]:
        foil=np.loadtxt(HERE/f'geometry/h1_{typ}.dat',skiprows=1)
        def station_wire(y):
            x,z,ch,inc=wing_station(y,typ=='tail')
            return foil_wire(foil,x,y,z,ch,inc)
        ys=sorted(set([-halfspan,-edges[1],-edges[0],-.1,0,.1,edges[0],edges[1],halfspan]))
        whole=cq.Solid.makeLoft([station_wire(y) for y in ys],ruled=True)
        fixed=whole
        for sign in (-1,1):
            lo,hi=sorted([sign*edges[0],sign*edges[1]])
            def cutter_wire(y,margin):
                x,z,ch,inc=wing_station(y,typ=='tail');a=np.deg2rad(inc)
                coords=[]
                for q,t in [(.75*ch-margin,-.1),(ch+.03,-.1),(ch+.03,.1),(.75*ch-margin,.1)]:
                    coords.append([x+q*np.cos(a)+t*np.sin(a),y,z-q*np.sin(a)+t*np.cos(a)])
                return wire(coords)
            cut=cq.Solid.makeLoft([cutter_wire(lo,0),cutter_wire(hi,0)],ruled=True)
            leaf=whole.intersect(cut)
            expanded=cq.Solid.makeLoft([cutter_wire(lo-.0003,.0005),cutter_wire(hi+.0003,.0005)],ruled=True)
            fixed=fixed.cut(expanded)
            shapes[f'{prefix}_{sign:+d}_CAD_approx']=to_old(leaf)
        shapes['main_wing_CAD_approx' if typ=='wing' else 'horizontal_tail_CAD_approx']=to_old(fixed)

    foil=np.loadtxt(HERE/'geometry/h1_fin.dat',skiprows=1)
    def fin_wire(x,z,ch):return wire([[x+q*ch,n*ch,z] for q,n in foil])
    fin=cq.Solid.makeLoft([fin_wire(1.23,.08,.28),fin_wire(1.34,.37,.13)],ruled=True)
    rudder_cut=cq.Solid.makeLoft([wire([[1.44,-.08,.079],[1.54,-.08,.079],[1.54,.08,.079],[1.44,.08,.079]]),
                                 wire([[1.4375,-.08,.371],[1.54,-.08,.371],[1.54,.08,.371],[1.4375,.08,.371]])],ruled=True)
    shapes['vertical_fin_CAD_approx']=to_old(fin.cut(rudder_cut))
    shapes['rudder_CAD_approx']=to_old(fin.intersect(rudder_cut))
    # Landing gear and nose propulsion make the assembly a recognisable complete
    # airframe. Their lumped masses stay within the declared component budgets.
    for i,(x,y) in enumerate([(.19,0.),(.53,-.23),(.53,.23)]):
        wheel=cq.Solid.makeCylinder(50,24,cq.Vector(x*1000,y*1000-12,-205),cq.Vector(0,1,0))
        shapes[f'landing_wheel_{i}']=to_old(wheel)
        start=np.array([x,0 if i==0 else np.sign(y)*.055,-.07]);end=np.array([x,y,-.205]);d=end-start
        strut=cq.Solid.makeCylinder(4,np.linalg.norm(d)*1000,cq.Vector(*(start*1000)),cq.Vector(*d))
        shapes[f'landing_strut_{i}']=to_old(strut)
    shapes['propulsion_motor']=to_old(cq.Solid.makeCylinder(22,50,cq.Vector(-25,0,0),cq.Vector(1,0,0)))
    shapes['propulsion_spinner']=to_old(cq.Solid.makeCone(2,24,40,cq.Vector(-68,0,0),cq.Vector(1,0,0)))
    for sign in (-1,1):
        blade=cq.Solid.makeLoft([wire([[-.031,sign*.021,-.012],[-.025,sign*.021,-.012],[-.025,sign*.021,.012],[-.031,sign*.021,.012]]),
                               wire([[-.030,sign*.185,-.006],[-.026,sign*.185,-.006],[-.026,sign*.185,.006],[-.030,sign*.185,.006]])],ruled=True)
        shapes[f'propulsion_blade_{sign:+d}']=to_old(blade)

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
            'winch_motor_drum':(.10,['winch_drum','winch_motor']),
            'landing_gear':(.12,[n for n in shapes if n.startswith('landing_')]),
            'propulsion':(.25,[n for n in shapes if n.startswith('propulsion_')])}
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
    c['name']='R3_conventional_high_wing_rear_sensor_test_aircraft'
    c['aero']['geometry']=(aero/'normal_r3.avl').relative_to(ROOT).as_posix()
    c['aero']['moment_reference_frd_m']=(transform@cg_aru).tolist()
    c['aero']['alpha_deg']=[-6.,-4.,-2.,0.,2.,4.,6.,9.,12.]
    c['aero']['beta_deg']=[-8.,-4.,0.,4.,8.]
    c['aero']['elevator_deg']=[-20.,-10.,-5.,0.,5.,10.,20.]
    c['provenance']['aircraft']['source']='New conventional high-wing TEST DESIGN: smooth closed fuselage, raised tail boom, tapered 1.8 m wing with 4 deg dihedral and 1 deg washout, tapered conventional tail, tricycle gear and nose propeller. CAD-weighted assumed masses, not team aircraft data. Door inertia frozen closed.'
    c['provenance']['bay']['source']='56x70 mm rear opening, 140-degree offset door, passive 50x50 mm capture throat, forward fairlead; geometric design assumptions, no structure/material qualification.'
    c['provenance']['winch']['source']='Feed point outside drum, forward relocated winch; prescribed decelerating length schedule with slow final approach. No torque-limited motor dynamics. Release push stops before retrieval.'
    c['provenance']['aero']['source']='Actual AVL 3.52 on the new R3 planform, twist and dihedral; copied H1 airfoil coordinates retained. Equivalent-area body from exact outer-CAD cross sections; body/no-body sensitivity required. Gear/propeller/profile drag and open-door wake remain unmeasured. https://web.mit.edu/drela/Public/web/avl/avl_doc.txt'
    for name in ('h1_wing.dat','h1_tail.dat','h1_fin.dat'): shutil.copy2(HERE/'geometry'/name,aero/name)
    # Equivalent-area circular body required by AVL; external envelope, not hollow material area.
    area_x=np.unique(np.r_[np.linspace(.00001,1.55999,65),body['x'][1:-1], [v[0] for v in boom_stations[:-1]]])
    areas=[]
    for x in area_x:
        faces=cq.Workplane('YZ').newObject([envelope]).section(float(x)*1000).vals()
        area=sum(face.Area() for face in faces)*1e-6
        if area<=0:raise ValueError(f'Missing body section at {x}')
        areas.append(area)
    radii=np.sqrt(np.array(areas)/np.pi)
    body_points=np.vstack([np.c_[area_x[::-1],radii[::-1]],np.c_[area_x,-radii]])
    np.savetxt(aero/'fuselage_equivalent.dat',body_points,header='R3 equivalent area from actual outer CAD sections',comments='',fmt='%.9f')
    write_avl(p,cg_aru,aero/'normal_r3.avl')
    write_avl(p,cg_aru,aero/'normal_r3_no_body.avl',body=False)
    write_avl(p,cg_aru,aero/'normal_r3_coarse.avl',scale=.7)
    write_avl(p,cg_aru,aero/'normal_r3_fine.avl',scale=1.4)
    case=ROOT/'examples/h1_normal_r3.yaml'
    case.write_text(yaml.safe_dump({k:v for k,v in c.items() if not k.startswith('_')},sort_keys=False,allow_unicode=True),encoding='utf8')
    c=load_case(case)
    for name in ('normal_r3','normal_r3_no_body','normal_r3_coarse','normal_r3_fine'): write_mass_file(c,aero/(name+'.mass'))
    write_mass_file(c,aero/'normal_r3_stowed.mass',stowed=True)
    assembly=cq.Assembly(name='Normal_R3_FRD_mm');parts=[];validation=[]
    for name,shape in shapes.items():
        if not shape.isValid() or shape.Volume()<=0: raise ValueError(f'Invalid R3 CAD: {name}')
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
        body_stations=len(xs)+len(boom_stations),body_area_table=list(zip(area_x.tolist(),areas)),guide_throat_mm=[50,50],opening_mm=[56,70],
        aircraft_style='Conventional high-wing rear deployment test design',wing_span_m=1.8,wing_area_m2=.555,wing_root_chord_m=.335,wing_tip_chord_m=.275,dihedral_deg=4,washout_deg=1,
        fuselage_length_m=1.56,propeller_diameter_m=.37,tail_span_m=.7,
        feed_aru_m=feed_aru.tolist(),drum_aru_m=drum_aru.tolist(),tunnel_box_aru_m=tunnel,
        raw_door_reference_deg=100,operating_door_deg=140,source_project_accessed=False,
        builder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_cad_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in originals},
        limitations=['Assumed component masses and effective CAD volume distributions','Equivalent-area body omits bay/door wake and section asymmetry','Compliant abstract latch; no jaw actuator dynamics','Prescribed door and reel; no structural stress or material calibration']))
    print(json.dumps(dict(case=str(case),parts=len(shapes),mass_kg=mass,cg_aru_m=cg_aru.tolist(),stowed_line_m=natural,body_stations=len(xs)),indent=2),flush=True)


if __name__=='__main__': main()
