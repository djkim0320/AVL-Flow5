"""N3: new parametric DBF cargo/sensor aircraft, SI design axes aft/right/up.

Geometry and AVL share these stations. CAD is CG-relative FRD; GLB is Y-up.
Component mass budgets are assumptions, not densities inferred from CAD shells.
"""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
from pathlib import Path
import argparse, csv, json, struct, sys
import numpy as np
import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'src'))
T=np.diag([-1.,1.,-1.])
SPAN=1.8
# span, leading-edge x/z, chord and geometric incidence; all metres/degrees.
WING=[(0.,.335,.130,.350,1.5),(.14,.335,.130,.350,1.5),(.50,.351,.149,.320,.8),(.90,.393,.175,.220,-.5)]
TAIL=[(0.,1.245,.205,.210,1.0),(.08,1.245,.205,.210,1.0),(.36,1.285,.220,.145,1.0)]
FIN=[(.115,1.165,.300),(.255,1.212,.230),(.430,1.302,.135)]
FEED=np.array([.910,0.,-.040])
LINE=3.0
PAYLOAD=.12
MASS_ROWS=[
 ('fuselage_shell_frames',.48,[.485,0.,0.],[.88,.22,.22]),
 ('main_wing_spar_skin',.56,[.450,0.,.142],[.30,1.80,.040]),
 ('horizontal_tail',.105,[1.310,0.,.210],[.18,.72,.023]),
 ('vertical_tail',.050,[1.315,0.,.265],[.22,.025,.315]),
 ('raised_tail_boom',.085,[1.090,0.,.140],[.52,.038,.038]),
 ('battery_6S_3000mAh_envelope',.48,[.170,0.,.012],[.135,.048,.049]),
 ('motor_mount',.205,[.040,0.,.020],[.085,.06,.06]),
 ('commercial_propeller_envelope',.035,[-.035,0.,.020],[.015,.3302,.025]),
 ('landing_gear_wheels',.180,[.425,0.,-.190],[.55,.39,.06]),
 ('internal_winch',.180,[.785,0.,.055],[.065,.070,.055]),
 ('servos_pushrods',.120,[.700,0.,.085],[.85,.08,.09]),
 ('receiver_and_separate_Rx_servo_battery',.080,[.260,0.,.020],[.075,.055,.025]),
 ('rear_door_hinge_latch',.055,[.810,0.,-.116],[.20,.195,.005]),
 ('ESC_fuse_arming_wires',.080,[.210,0.,-.035],[.12,.08,.035]),
]
MESHES={'coarse':(8,4),'medium':(12,6),'fine':(18,9),'extra':(24,12)}


def mass_properties():
    mass=sum(r[1] for r in MASS_ROWS)
    cg=sum(m*np.asarray(p) for _,m,p,_ in MASS_ROWS)/mass
    tensor=np.zeros((3,3))
    for _,m,p,(a,b,c) in MASS_ROWS:
        d=np.asarray(p)-cg
        tensor+=m/12*np.diag([b*b+c*c,a*a+c*c,a*a+b*b])+m*(d@d*np.eye(3)-np.outer(d,d))
    return mass,cg,T@tensor@T


def references():
    area=2*sum((b[0]-a[0])*(a[3]+b[3])/2 for a,b in zip(WING,WING[1:]))
    mac=2*sum((b[0]-a[0])*(a[3]**2+a[3]*b[3]+b[3]**2)/3 for a,b in zip(WING,WING[1:]))/area
    return area,mac,SPAN


def foil(m=.02,p=.4,t=.12):
    x=(1-np.cos(np.linspace(0,np.pi,65)))/2
    yt=5*t*(.2969*np.sqrt(x)-.1260*x-.3516*x*x+.2843*x**3-.1015*x**4)
    yc=np.where(x<p,m/p**2*(2*p*x-x*x),m/(1-p)**2*((1-2*p)+2*p*x-x*x))
    dy=np.where(x<p,2*m/p**2*(p-x),2*m/(1-p)**2*(p-x));a=np.arctan(dy)
    return np.r_[np.c_[x-yt*np.sin(a),yc+yt*np.cos(a)][::-1],np.c_[x+yt*np.sin(a),yc-yt*np.cos(a)][1:]]


def write_avl(mesh='fine'):
    directory=HERE/'aero'/mesh;directory.mkdir(parents=True,exist_ok=True)
    nx,ny=MESHES[mesh];mass,cg,I=mass_properties();area,mac,span=references()
    for name,coords in [('n3_2412.dat',foil()),('n3_0012.dat',foil(0.))]:
        np.savetxt(directory/name,coords,header=name,comments='',fmt='%.12f')
    lines=['N3 DBF2027 new cargo aircraft '+mesh,'0.0','0 0 0',f'{area:.12f} {mac:.12f} {span}',
           ' '.join(map(str,cg)),'0.0']
    for name,stations,airfoil,control,hinge,start in [('N3_MainWing',WING,'n3_2412.dat','aileron',.75,.50),('N3_HorizontalTail',TAIL,'n3_0012.dat','elevator',.70,.08)]:
        lines+=['SURFACE',name,f'{nx} 1','YDUPLICATE','0']
        for y,x,z,c,inc in stations:
            lines+=['SECTION',f'{x} {y} {z} {c} {inc} {ny} 1','AFILE',airfoil]
            if y>=start:lines+=['CONTROL',f'{control} 1 {hinge} 0 1 0 '+('-1' if control=='aileron' else '1')]
    lines+=['SURFACE','N3_VerticalTail',f'{nx} 1']
    for z,x,c in FIN:
        lines+=['SECTION',f'{x} 0 {z} {c} 0 {ny} 1','AFILE','n3_0012.dat','CONTROL','rudder 1 .70 0 0 1 1']
    path=directory/'N3.avl';path.write_text('\n'.join(lines)+'\n',encoding='ascii')
    return path


def make_case(mesh='fine'):
    from dbf_stability.point_mass import configure
    from dbf_stability.config import validate
    from dbf_stability.avl import write_mass_file
    mass,cg,I=mass_properties();feed=T@(FEED-cg)
    # This file supplies only the existing solver schema; every physical section
    # below is declared for N3, with no previous aircraft aerodynamics reused.
    c=yaml.safe_load((ROOT/'examples/h1_recovery_funnel_v2.yaml').read_text('utf8'))
    c['name']='N3_DBF2027_new_cargo_aircraft';configure(c,PAYLOAD)
    c['aircraft']=dict(mass_kg=mass,inertia_kgm2=I.tolist(),cg_m=(T@cg).tolist(),tow_point_m=feed.tolist(),
        thrust_point_m=(T@(np.array([.04,0.,.020])-cg)).tolist(),profile_cd=.030,max_thrust_N=30.)
    c['cable']=dict(length_m=LINE,segments=16,density_kg_m=.0015,diameter_m=.001,EA_N=60.,damping_Ns_m=.03,
        damping_engagement_strain=.01,cd_normal=1.2,cd_tangent=.02,spool_k_N_m=800.,spool_c_Ns_m=3.,limit_N=10.,guide_points_body_m=[])
    c['bay']=dict(exit_x_m=float(feed[0]),front_x_m=float(feed[0]+.30),half_width_m=.102,
        floor_z_m=float(cg[2]+.110),ceiling_z_m=float(cg[2]-.105),ramp_slope=0.,lip_depth_m=0.,door_length_m=.20,
        door_thickness_m=.003,wall_thickness_m=.003,contact_radius_m=.0015,release_push_N=0.,contact_k_N_m=800.,
        contact_c_Ns_m=2.,friction=.1,latch_k_N_m=500.,latch_c_Ns_m=5.,latch_kr_Nm_rad=0.,latch_cr_Nms_rad=0.,
        capture_radius_m=.006,capture_speed_m_s=.15,capture_angle_deg=8.,stowed_center_m=(feed+[0,0,.02]).tolist(),
        stowed_quaternion_wxyz=[1,0,0,0])
    c['winch']=dict(stowed_length_m=.02,radius_m=.016,limit_torque_Nm=.20,limit_power_W=8.,release_s=.6,
        recovery_start_s=65.,door_schedule=[[0,270],[80,270]],length_schedule=[[0,.02],[.6,.02],[5,LINE],[65,LINE],[80,.02]],door_capture_interlock=False)
    c['flight']=dict(speed_m_s=20.,altitude_m=100.,rho_kg_m3=1.2133,g_m_s2=9.80665,wind_ned_m_s=[0,0,0],
        local_flow_factor=1.,gust=None,controls={},controller={'enabled':False})
    c['simulation']=dict(duration_s=2.,sample_dt_s=.02,max_step_s=.01,rtol=1e-5,atol=1e-7,jacobian_workers=11,
        maximum_runtime_s=1800,checkpoint_interval_s=2,stagnation_window_s=120,stagnation_min_advance_s=.001,
        trim_linear_tolerance_m_s2=1e-5,trim_angular_tolerance_rad_s2=5e-5)
    geometry=write_avl(mesh)
    c['aero']=dict(executable='vendor/avl/avl.exe',geometry=str(geometry),elevator_index=2,lateral_control_indices=[1,3],
        moment_reference_frd_m=(T@cg).tolist(),alpha_deg=[-4,-2,0,2,4,6,8],beta_deg=[-4,0,4],elevator_deg=[-8,-4,0,4,8],
        backend='avl',flow5_executable='vendor/flow5/bin/flow5_v7.57_win64/flow5.exe',flow5_method='VLM2',
        rate_derivative_closure='classical_longitudinal_lateral',threads_per_worker=1,lateral_difference_deg=.5,timeout_s=1800)
    c['provenance']={k:{'kind':'assumed','source':v} for k,v in dict(
        aircraft='N3 new parametric CAD/AVL; component mass budget and box-envelope inertias, unmeasured. Receiver plus separate Rx/servo COTS battery allocated 80 g (assumed 6.6 V/700 mAh envelope, exact product unselected); ESC BEC disabled in intended wiring. CD0=.030 design assumption; sensitivity .020/.040.',
        sensor='0.12 kg point payload only. Sensor 220x90x100 mm reserved envelope is a packaging assumption, not an aerodynamic sensor model.',
        cable='3.0 m to provide margin over 2.70 m exit-to-tip distance; 1 mm, 1.5 g/m, EA60 N, CDn1.2/CDt.02 assumed.',
        bay='N3 aft compartment, 270 degree underbody door; CAD visualization only; no payload contact in point mode.',
        winch='Internal winch at ARU(.785,0,.055); force reference at exit(.910,0,-.040); 16 mm radius/.20 Nm/8 W assumed limits.',
        flight='20 m/s nominal at100 m, rho1.2133. Manual/open-loop model. No automatic flight controller.',
        aero='Fresh actual AVL3.52 and flow5 7.57 lifting-surface solves; inviscid; open fuselage/propwash/viscous separation excluded; explicit extra drag assumption.'
    ).items()}
    validate(c);write_mass_file(c,geometry.with_suffix('.mass'))
    (HERE/f'N3_{mesh}.yaml').write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True),encoding='utf8')
    return c


def glb(parts,path):
    doc=dict(asset={'version':'2.0','generator':'N3 common CAD/AVL design'},scene=0,scenes=[{'nodes':[]}],nodes=[],meshes=[],materials=[],accessors=[],bufferViews=[])
    blob=bytearray()
    def put(a,kind,ctype):
        while len(blob)%4:blob.append(0)
        view=len(doc['bufferViews']);doc['bufferViews'].append(dict(buffer=0,byteOffset=len(blob),byteLength=a.nbytes));blob.extend(a.tobytes())
        item=dict(bufferView=view,componentType=ctype,count=len(a),type=kind)
        if kind=='VEC3':item.update(min=a.min(0).tolist(),max=a.max(0).tolist())
        doc['accessors'].append(item);return len(doc['accessors'])-1
    for name,v,f,color in parts:
        vertex=put(np.asarray(v[:,[0,2,1]]*[1,-1,1],dtype='<f4'),'VEC3',5126)
        indices=put(np.asarray(f.ravel(),dtype='<u4'),'SCALAR',5125);i=len(doc['meshes'])
        doc['materials'].append(dict(name=name,pbrMetallicRoughness={'baseColorFactor':color,'metallicFactor':.05,'roughnessFactor':.6},doubleSided=True))
        doc['meshes'].append(dict(name=name,primitives=[{'attributes':{'POSITION':vertex},'indices':indices,'material':i}]))
        doc['nodes'].append(dict(name=name,mesh=i));doc['scenes'][0]['nodes'].append(i)
    while len(blob)%4:blob.append(0)
    doc['buffers']=[{'byteLength':len(blob)}];js=json.dumps(doc,separators=(',',':')).encode();js+=b' '*((-len(js))%4)
    path.write_bytes(struct.pack('<4sII',b'glTF',2,28+len(js)+len(blob))+struct.pack('<I4s',len(js),b'JSON')+js+struct.pack('<I4s',len(blob),b'BIN\0')+blob)


def build_cad():
    import cadquery as cq
    mass,cg,I=mass_properties();raw={};white=[.88,.91,.94,1];blue=[.075,.28,.45,1];orange=[.98,.48,.10,1];gray=[.15,.18,.22,1]
    def wire(points):return cq.Wire.makePolygon([cq.Vector(*(np.asarray(p)*1000)) for p in points],close=True)
    def box(center,size):return cq.Solid.makeBox(*(np.asarray(size)*1000),cq.Vector(*((np.asarray(center)-np.asarray(size)/2)*1000)))
    def add(name,shape,color=white):
        if not shape.isValid() or shape.Volume()<=0:raise ValueError('Invalid CAD solid: '+name)
        raw[name]=(shape,color)
    def ring(x,ry,rz,zc=0.):
        a=np.linspace(0,2*np.pi,65)[:-1];u=np.cos(a);v=np.sin(a)
        return wire(np.c_[np.full(len(a),x),ry*np.sign(u)*abs(u)**.65,zc+rz*np.sign(v)*abs(v)**.65])
    outer=cq.Solid.makeLoft([ring(x,ry,rz,zc) for x,ry,rz,zc in [(.005,.023,.025,.02),(.085,.056,.062,.016),(.18,.087,.095,.006),(.28,.110,.113,0),(.91,.110,.113,0)]],ruled=True)
    cavity=cq.Solid.makeLoft([ring(.28,.106,.109),ring(.93,.106,.109)],ruled=True)
    add('N3_cargo_fuselage',outer.cut(cavity))
    add('N3_cargo_floor',box([.59,0,-.090],[.57,.178,.003]),gray)
    for y in [-.072,.072]:add('N3_retention_rail_'+str(y),box([.53,y,-.080],[.37,.008,.014]),blue)
    add('N3_rear_door_270',box([.81,0,-.119],[.20,.198,.003]),orange)
    add('N3_door_hinge',cq.Solid.makeCylinder(3,190,cq.Vector(910,-95,-116),cq.Vector(0,1,0)),gray)
    def strut(name,a,b,r,color=gray):
        d=np.asarray(b)-a;add(name,cq.Solid.makeCylinder(r*1000,float(np.linalg.norm(d)*1000),cq.Vector(*(np.asarray(a)*1000)),cq.Vector(*d)),color)
    strut('N3_raised_tail_boom',[.80,0,.097],[1.445,0,.166],.018)
    def split_foil(coords,hinge,rear):
        le=np.argmin(coords[:,0]);upper=coords[:le+1][::-1];lower=coords[le:]
        u=[hinge,np.interp(hinge,upper[:,0],upper[:,1])];l=[hinge,np.interp(hinge,lower[:,0],lower[:,1])]
        if rear:return np.vstack([upper[upper[:,0]>hinge][::-1],u,l,lower[lower[:,0]>hinge]])
        return np.vstack([u,upper[(upper[:,0]<hinge)&(upper[:,0]>0)][::-1],coords[le],lower[(lower[:,0]>0)&(lower[:,0]<hinge)],l])
    def section(st,coords):
        y,x,z,chord,inc=st;a=np.deg2rad(inc);u,v=coords.T*chord
        return wire(np.c_[x+u*np.cos(a)+v*np.sin(a),np.full(len(u),y),z-u*np.sin(a)+v*np.cos(a)])
    for name,stations,coords,start,hinge in [('wing',WING,foil(),.50,.75),('tail',TAIL,foil(0.),.08,.70)]:
        for side in [-1,1]:
            for j,(a,b) in enumerate(zip(stations,stations[1:])):
                sections=[(side*s[0],*s[1:]) for s in (a,b)]
                moving=a[0]>=start
                add(f'N3_{name}_{side}_{j}',cq.Solid.makeLoft([section(s,split_foil(coords,hinge,False) if moving else coords) for s in sections],ruled=True),white if name=='wing' else blue)
                if moving:add(f'N3_{name}_control_{side}_{j}',cq.Solid.makeLoft([section(s,split_foil(coords,hinge,True)) for s in sections],ruled=True),orange)
    for rear in [False,True]:
        coords=split_foil(foil(0.),.70,rear)
        add('N3_rudder' if rear else 'N3_fin',cq.Solid.makeLoft([wire([[x+u*c,v*c,z] for u,v in coords]) for z,x,c in FIN],ruled=True),orange if rear else blue)
    for x,y in [(.61,-.195),(.61,.195),(.10,0)]:
        strut('N3_gear_'+str((x,y)),[x,np.sign(y)*.075,-.085],[x,y,-.215],.0035)
        add('N3_wheel_'+str((x,y)),cq.Solid.makeCylinder(38,18,cq.Vector(x*1000,y*1000-9,-215),cq.Vector(0,1,0)),gray)
        add('N3_wheel_hub_'+str((x,y)),cq.Solid.makeCylinder(12,19,cq.Vector(x*1000,y*1000-9.5,-215),cq.Vector(0,1,0)),orange)
    add('N3_spinner',cq.Solid.makeCone(1,22,40,cq.Vector(-55,0,20),cq.Vector(1,0,0)),orange)
    for side in [-1,1]:
        blade=cq.Workplane('YZ').center(side*84,20).ellipse(81.1,12).extrude(3).val().translate((-35,0,0))
        add('N3_commercial_propeller_envelope_'+str(side),blade,gray)
    add('N3_winch_drum_internal',cq.Solid.makeCylinder(16,48,cq.Vector(785,-24,55),cq.Vector(0,1,0)),orange)
    for y in [-.028,.028]:add('N3_winch_mount_'+str(y),box([.785,y,.055],[.044,.004,.055]),gray)
    # Accessible switch and arming-plug envelopes, not a certified wiring design.
    add('N3_RC_switch',box([.245,-.102,.035],[.018,.009,.009]),orange)
    add('N3_arming_plug',box([.205,.030,.110],[.018,.010,.018]),orange)
    parts=[];assembly=cq.Assembly(name='N3_DBF2027')
    for name,(shape,color) in raw.items():
        shape=shape.translate(tuple(-cg*1000)).rotate((0,0,0),(0,1,0),180)
        assembly.add(shape,name=name.replace('(','').replace(')','').replace(',','_'),color=cq.Color(*color))
        vs,fs=shape.tessellate(.45,.12);parts.append((name,np.array([v.toTuple() for v in vs])*.001,np.array(fs),color))
    assembly.save(str(HERE/'N3_aircraft_open.step'));glb(parts,HERE/'N3_aircraft_open.glb')
    np.savez_compressed(HERE/'cad_meshes.npz',**{f'v{i}':p[1] for i,p in enumerate(parts)},**{f'f{i}':p[2] for i,p in enumerate(parts)})
    (HERE/'cad_parts.json').write_text(json.dumps([{'name':n,'color':c} for n,_,_,c in parts],indent=2),encoding='utf8')
    return parts


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--aero-only',action='store_true');args=parser.parse_args()
    for mesh in MESHES:make_case(mesh)
    mass,cg,I=mass_properties();area,mac,span=references()
    with (HERE/'mass_budget.csv').open('w',newline='',encoding='utf8') as f:
        w=csv.writer(f);w.writerow(['component','mass_kg','CG_ARU_m','envelope_m','source']);w.writerows([*row,'assumed design mass/envelope'] for row in MASS_ROWS)
    dims=dict(name='N3 DBF2027 new cargo aircraft',span_m=span,area_m2=area,mac_m=mac,aspect_ratio=span**2/area,
        aircraft_mass_kg=mass,cg_aru_m=cg.tolist(),inertia_frd_kgm2=I.tolist(),feed_local_frd_m=(T@(FEED-cg)).tolist(),line_m=LINE,payload_kg=PAYLOAD,
        sensor_reservation_m=[.220,.090,.100],container_reservation_m=[.280,.130,.130],battery_design_Wh=22.2*3.,
        Rx_servo_battery_design_Wh=6.6*.7,arming_plug_position_aru_m=[.205,.030,.110],
        minimum_arming_plug_to_propeller_plane_m=.205-.018/2-(-.032),
        door_parts=['N3_rear_door_270'],door_reference_deg=270,import_units='m',import_axes='yup',aircraft_cg_local_m=[0,0,0],
        rules_url='https://aiaa.org/wp-content/uploads/2026/09/DBF-2027-Rules-Draft.pdf',rules_status='current official link uses Draft filename; accessed 2026-09-26',
        rule_span_limit_m=1.8288,minimum_exit_sensor_tip_distance_m=1.5*span,
        limits=['Packaging envelopes are not sensor design/certification','Mass/inertia and CD0 assumed','Point model cannot certify sensor attitude, capture or geometric clearance','Electrical/structure/drop/flight tests pending'])
    if not args.aero_only:
        parts=build_cad();vertices=np.concatenate([p[1] for p in parts]);dims.update(CAD_bounds_frd_m=[vertices.min(0).tolist(),vertices.max(0).tolist()],triangles=sum(len(p[2]) for p in parts),parts=len(parts))
        assert abs(np.ptp(vertices[:,1])-span)<1e-6
    (HERE/'dimensions.json').write_text(json.dumps(dims,indent=2),encoding='utf8');print(json.dumps(dims,indent=2),flush=True)
    (HERE/'geometry_definition.json').write_text(json.dumps(dict(wing=WING,tail=TAIL,fin=FIN,masses=MASS_ROWS,meshes=MESHES),indent=2),encoding='utf8')


if __name__=='__main__':main()
