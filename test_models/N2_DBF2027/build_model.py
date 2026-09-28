"""New N2 test aircraft. One geometry definition feeds real CAD and AVL.

ARU design coordinates: nose aft/right/up, metres. Exports are CG-relative
FRD (STEP mm; GLB metres/Y-up). Masses are explicit design assumptions.
"""
import os
for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[k]='1'
from pathlib import Path
import csv, json, struct
import numpy as np
import cadquery as cq
import yaml
from dbf_stability.config import validate
from dbf_stability.point_mass import configure

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
SPAN=1.70
WING=[(0.,.32,.09,.30,2.),(.12,.32,.09,.30,2.),(.48,.337,.115,.26,1.5),(.85,.355,.145,.22,1.)]
TAIL=[(0.,1.035,.10,.185,0.),(.10,1.035,.10,.185,0.),(.31,1.07,.11,.135,0.)]
FIN=[(.09,1.01,.24),(.20,1.05,.18),(.34,1.13,.10)]

def foil(m=0.,p=.4,t=.12):
    x=(1-np.cos(np.linspace(0,np.pi,61)))/2
    yt=5*t*(.2969*np.sqrt(x)-.1260*x-.3516*x*x+.2843*x**3-.1015*x**4)
    yc=np.where(x<p,m/p**2*(2*p*x-x*x),m/(1-p)**2*((1-2*p)+2*p*x-x*x))
    dy=np.where(x<p,2*m/p**2*(p-x),2*m/(1-p)**2*(p-x));a=np.arctan(dy)
    upper=np.c_[x-yt*np.sin(a),yc+yt*np.cos(a)]
    lower=np.c_[x+yt*np.sin(a),yc-yt*np.cos(a)]
    return np.r_[upper[::-1],lower[1:]]

def wire(p):return cq.Wire.makePolygon([cq.Vector(*(np.asarray(q)*1000)) for q in p],close=True)
def box(center,size):
    return cq.Solid.makeBox(*(np.asarray(size)*1000),cq.Vector(*((np.asarray(center)-np.asarray(size)/2)*1000)))
def wing_wire(st,coords):
    y,x,z,c,inc=st;a=np.deg2rad(inc);u,v=coords.T*c
    return wire(np.c_[x+u*np.cos(a)+v*np.sin(a),np.full(len(u),y),z-u*np.sin(a)+v*np.cos(a)])
def write_glb(parts):
    doc={'asset':{'version':'2.0','generator':'N2 CadQuery test design'},'scene':0,'scenes':[{'nodes':[]}],
         'nodes':[],'meshes':[],'materials':[],'accessors':[],'bufferViews':[]}
    blob=bytearray()
    def put(a,kind,ctype):
        while len(blob)%4:blob.append(0)
        view=len(doc['bufferViews']);doc['bufferViews'].append({'buffer':0,'byteOffset':len(blob),'byteLength':a.nbytes});blob.extend(a.tobytes())
        access={'bufferView':view,'componentType':ctype,'count':len(a),'type':kind}
        if kind=='VEC3':access.update(min=a.min(0).tolist(),max=a.max(0).tolist())
        doc['accessors'].append(access);return len(doc['accessors'])-1
    for name,v,f,color in parts:
        p=put(np.asarray(v[:,[0,2,1]]*[1,-1,1],dtype='<f4'),'VEC3',5126)
        idx=put(np.asarray(f.ravel(),dtype='<u4'),'SCALAR',5125);i=len(doc['meshes'])
        doc['materials'].append({'pbrMetallicRoughness':{'baseColorFactor':color,'metallicFactor':0,'roughnessFactor':.65},'doubleSided':True})
        doc['meshes'].append({'name':name,'primitives':[{'attributes':{'POSITION':p},'indices':idx,'material':i}]})
        doc['nodes'].append({'name':name,'mesh':i});doc['scenes'][0]['nodes'].append(i)
    while len(blob)%4:blob.append(0)
    doc['buffers']=[{'byteLength':len(blob)}];js=json.dumps(doc,separators=(',',':')).encode();js+=b' '*((-len(js))%4)
    (OUT/'N2_aircraft_open.glb').write_bytes(struct.pack('<4sII',b'glTF',2,28+len(js)+len(blob))+struct.pack('<I4s',len(js),b'JSON')+js+struct.pack('<I4s',len(blob),b'BIN\0')+blob)

def main():
    # Distributed rectangular component envelopes; no solid-volume density fiction.
    masses=[['fuselage',.40,[.48,0,0],[.80,.17,.18]],['wing',.44,[.43,0,.105],[.27,1.70,.036]],
      ['horizontal_tail',.075,[1.12,0,.10],[.16,.62,.02]],['fin',.035,[1.15,0,.20],[.18,.015,.25]],
      ['tail_boom',.065,[1.00,0,.065],[.50,.025,.025]],['battery',.52,[.19,0,-.02],[.12,.05,.04]],
      ['motor_propeller',.20,[.045,0,0],[.07,.06,.06]],['gear',.14,[.39,0,-.17],[.50,.35,.05]],
      ['winch',.14,[.70,0,-.025],[.07,.06,.055]],['servos_wiring',.09,[.59,0,0],[.45,.12,.10]],
      ['electronics',.075,[.30,0,.02],[.10,.06,.03]]]
    mass=sum(r[1] for r in masses);cg=sum(r[1]*np.array(r[2]) for r in masses)/mass
    I=np.zeros((3,3))
    for name,m,p,size in masses:
        d=np.asarray(p)-cg;a,b,c=size
        I+=m/12*np.diag([b*b+c*c,a*a+c*c,a*a+b*b])+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    T=np.diag([-1.,1.,-1.]);I=T@I@T
    with (OUT/'mass_budget.csv').open('w',newline='',encoding='utf8') as h:
        w=csv.writer(h);w.writerow(['component','mass_kg','CG_ARU_m','envelope_m','source']);w.writerows([*r,'assumed design budget'] for r in masses)
    raw={};white=[.83,.9,.95,1];blue=[.10,.36,.65,1];gray=[.20,.24,.29,1]
    def add(n,s,c=white):
        if not s.isValid():raise ValueError('Invalid CAD: '+n)
        raw[n]=(s,c)
    # Hollow cargo pod, gently tapered nose, raised tail boom and fully open exit.
    outer=box([.50,0,0],[.64,.18,.19]);inner=box([.51,0,0],[.65,.174,.184])
    add('cargo_pod',outer.cut(inner))
    rings=[]
    for x,wy,hz in [(.02,.023,.024),(.09,.052,.057),(.18,.09,.095)]:
        rings.append(wire([[x,wy*np.cos(a),hz*np.sin(a)] for a in np.linspace(0,2*np.pi,49)[:-1]]))
    add('nose_cowling',cq.Solid.makeLoft(rings,ruled=False),blue)
    for label,sts,coords in [('wing',WING,foil(.02,.4,.12)),('tail',TAIL,foil(0,.4,.10))]:
        for sign in [-1,1]:
            section=[(sign*y,x,z,c,i) for y,x,z,c,i in sts]
            add(label+('_left' if sign<0 else '_right'),cq.Solid.makeLoft([wing_wire(s,coords) for s in section],ruled=True),white if label=='wing' else blue)
    coords=foil(0,.4,.10);finw=[]
    for z,x,c in FIN:finw.append(wire([[x+u*c,v*c,z] for u,v in coords]))
    add('vertical_fin',cq.Solid.makeLoft(finw,ruled=True),blue)
    add('raised_tail_boom',cq.Solid.makeCylinder(13,470,cq.Vector(770,0,65),cq.Vector(1,0,0)),gray)
    add('door_folded_270',box([.74,0,-.100],[.16,.178,.003]),blue)
    # Bay accommodates a declared 210 x 100 x 110 mm container envelope.
    for y in [-.065,.065]:add('cargo_rail_'+str(y),box([.50,y,-.080],[.36,.009,.015]),gray)
    for x,y in [(.62,-.17),(.62,.17),(.13,0)]:
        start=np.array([x,np.sign(y)*.065,-.080]);end=np.array([x,y,-.209]);d=end-start
        add('gear_strut_'+str((x,y)),cq.Solid.makeCylinder(3,float(np.linalg.norm(d)*1000),cq.Vector(*(start*1000)),cq.Vector(*d)),gray)
        add('wheel_'+str((x,y)),cq.Solid.makeCylinder(34,16,cq.Vector(x*1000,y*1000-8,-209),cq.Vector(0,1,0)),gray)
    add('propeller_hub',cq.Solid.makeCylinder(12,25,cq.Vector(-10,0,0),cq.Vector(1,0,0)),gray)
    for sign in [-1,1]:add('propeller_blade_'+str(sign),box([-.009,sign*.076,0],[.006,.140,.019]),gray)
    parts=[];assembly=cq.Assembly(name='N2_DBF2027')
    for name,(s,color) in raw.items():
        s=s.translate(tuple(-cg*1000)).rotate((0,0,0),(0,1,0),180)
        assembly.add(s,name=name.replace('(','').replace(')','').replace(',','_'),color=cq.Color(*color))
        vs,fs=s.tessellate(.6,.15);parts.append((name,np.array([v.toTuple() for v in vs])*.001,np.array(fs),color))
    assembly.save(str(OUT/'N2_aircraft_open.step'));write_glb(parts)
    aero=OUT/'aero';aero.mkdir(exist_ok=True)
    for name,coords in [('n2_2412.dat',foil(.02,.4,.12)),('n2_0010.dat',foil(0,.4,.10))]:
        np.savetxt(aero/name,coords,header=name,comments='')
    S=2*sum((b[0]-a[0])*(a[3]+b[3])/2 for a,b in zip(WING,WING[1:]))
    mac=2*sum((b[0]-a[0])*(a[3]**2+a[3]*b[3]+b[3]**2)/3 for a,b in zip(WING,WING[1:]))/S
    txt=f'N2 DBF2027 assumed high-wing test aircraft\n0.0\n0 0 0\n{S:.10f} {mac:.10f} {SPAN}\n'+ ' '.join(map(str,cg))+'\n0.0\n'
    for name,sts,file,ctrl in [('N2_MainWing',WING,'n2_2412.dat','aileron 1 .75 0 1 0 -1'),('N2_HorizontalTail',TAIL,'n2_0010.dat','elevator 1 .70 0 1 0 1')]:
        txt+=f'SURFACE\n{name}\n12 1\nYDUPLICATE\n0\n'
        for y,x,z,c,inc in sts:
            txt+=f'SECTION\n{x} {y} {z} {c} {inc} 8 1\nAFILE\n{file}\nCONTROL\n{ctrl}\n'
    txt+='SURFACE\nN2_Fin\n10 1\n'
    for z,x,c in FIN:txt+=f'SECTION\n{x} 0 {z} {c} 0 8 1\nAFILE\nn2_0010.dat\nCONTROL\nrudder 1 .70 0 0 1 1\n'
    (aero/'N2.avl').write_text(txt,encoding='ascii')
    c=yaml.safe_load((ROOT/'examples/h1_recovery_funnel_v2.yaml').read_text(encoding='utf8'));configure(c,.10)
    feed=T@(np.array([.82,0,-.065])-cg)
    c['name']='N2_DBF2027_highwing_point_mass'
    c['aircraft']={'mass_kg':mass,'inertia_kgm2':I.tolist(),'cg_m':(T@cg).tolist(),'tow_point_m':feed.tolist(),'thrust_point_m':(T@(np.array([.015,0,0])-cg)).tolist(),'profile_cd':.035,'max_thrust_N':25.}
    c['cable'].update(length_m=2.80,segments=8,EA_N=60.,limit_N=8.)
    c['bay']={'exit_x_m':float(feed[0]),'front_x_m':float(feed[0]+.22),'half_width_m':.087,'floor_z_m':float(cg[2]+.095),'ceiling_z_m':float(cg[2]-.092),'ramp_slope':0.,'lip_depth_m':0.,'door_length_m':.16,'door_thickness_m':.003,'wall_thickness_m':.003,'contact_radius_m':.0015,'release_push_N':0.,'contact_k_N_m':800.,'contact_c_Ns_m':2.,'friction':.1,'latch_k_N_m':500.,'latch_c_Ns_m':5.,'latch_kr_Nm_rad':0.,'latch_cr_Nms_rad':0.,'capture_radius_m':.006,'capture_speed_m_s':.15,'capture_angle_deg':8.,'stowed_center_m':(feed+[0,0,.02]).tolist(),'stowed_quaternion_wxyz':[1,0,0,0]}
    c['winch']={'stowed_length_m':.02,'radius_m':.015,'limit_torque_Nm':.15,'limit_power_W':5.,'release_s':.6,'recovery_start_s':65.,'door_schedule':[[0,270],[75,270]],'length_schedule':[[0,.02],[.6,.02],[5,2.8],[65,2.8],[75,.02]],'door_capture_interlock':False}
    c['flight']={'speed_m_s':25.,'altitude_m':100.,'rho_kg_m3':1.2133,'g_m_s2':9.80665,'wind_ned_m_s':[0,0,0],'local_flow_factor':1.,'gust':None,'controls':{},'controller':{'enabled':False}}
    c['simulation']={'duration_s':1.,'sample_dt_s':.02,'max_step_s':.01,'rtol':1e-5,'atol':1e-7,'jacobian_workers':12,'maximum_runtime_s':1800,'checkpoint_interval_s':10,'stagnation_window_s':120,'stagnation_min_advance_s':.001}
    c['aero'].update(geometry=str(aero/'N2.avl'),backend='avl',moment_reference_frd_m=(T@cg).tolist(),alpha_deg=[-4.,0.,4.,8.,12.],beta_deg=[-5.,0.,5.],elevator_deg=[-15.,0.,15.],threads_per_worker=1)
    c['provenance']={k:{'type':'assumed','source':v} for k,v in {
      'aircraft':'N2 new CAD/AVL common parametric geometry; mass_budget.csv box-envelope inertia; CD0=.035 unmeasured includes fuselage/gear/door.',
      'sensor':'0.10 kg point mass; no sensor CAD/aerodynamics/attitude/contact.',
      'cable':'Assumed 1 mm, 1.5 g/m, EA60 N, normal CD1.2; no material test.',
      'bay':'N2 open cargo pod with folded door, CAD visualization only in point mode.',
      'winch':'Assumed 15 mm drum, 0.15 Nm and 5 W limits; CG-relative feed recorded in dimensions.json.',
      'flight':'25 m/s at 100 m, density1.2133 kg/m3; no automatic controller.',
      'aero':'Actual AVL3.52 calculation for N2 wing/tail NACA2412/0010 geometry; fuselage/propwash excluded from VLM, CD0 explicit.'}.items()}
    validate(c);(OUT/'N2_case.yaml').write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True),encoding='utf8')
    info={'name':c['name'],'wingspan_m':SPAN,'area_m2':S,'mac_m':mac,'aircraft_mass_kg':mass,'cg_aru_m':cg.tolist(),'aircraft_cg_local_m':[0,0,0],'inertia_frd_kgm2':I.tolist(),'feed_local_frd_m':feed.tolist(),'line_m':2.8,'payload_kg':.1,'import_units':'m','import_axes':'yup','door_parts':['door_folded_270'],'door_reference_deg':270,'CAD_bounds_frd_m':[np.concatenate([p[1] for p in parts]).min(0).tolist(),np.concatenate([p[1] for p in parts]).max(0).tolist()],'triangles':sum(len(p[2]) for p in parts),'rule_source':'https://aiaa.org/wp-content/uploads/2026/09/DBF-2027-Rules-Draft.pdf','rule_status':'draft PDF linked by current official rules page, accessed2026-09-26','wingspan_limit_m':1.8288,'minimum_exit_tip_distance_m':SPAN*1.5,'scope':'Test geometry, not certified competition aircraft; sensor rule compliance and flight/structure/electrical checks not established.'}
    (OUT/'dimensions.json').write_text(json.dumps(info,indent=2),encoding='utf8');print(json.dumps(info,indent=2),flush=True)

if __name__=='__main__':main()
