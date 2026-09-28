"""Actual CadQuery/OpenCASCADE solids and mesh exports in aircraft-CG FRD.

STEP: millimetres; STL vertex values: metres; GLB: metres with FRD->Y-up map.
Preserved source meshes and reconstructed CAD surfaces are exported separately.
"""
from pathlib import Path
import csv, json, struct, math
import numpy as np
import cadquery as cq
from extract_h1 import HERE, SOURCE, read_glb
from concurrent.futures import ProcessPoolExecutor

def clearance_init(cad_directory):
    global SWEEP_SENSOR, SWEEP_OBSTACLES
    shapes={f.stem:cq.importers.importStep(str(f)).val() for f in Path(cad_directory).glob('*.step') if f.stem!='H1_A_temporary_assembly'}
    names=[n for n in shapes if n=='sensor_body' or n.startswith('sensor_fin_')]
    SWEEP_SENSOR=cq.Compound.makeCompound([shapes[n] for n in names])
    SWEEP_OBSTACLES={n:s for n,s in shapes.items() if n not in names and n not in ['sensor_tow_point','aircraft_tow_point']}

def clearance_position(x):
    shape=SWEEP_SENSOR.translate((-(x-.99)*1000,0,0));a=shape.BoundingBox()
    def bound(s):
        b=s.BoundingBox()
        gaps=[max(0,getattr(a,k+'min')-getattr(b,k+'max'),getattr(b,k+'min')-getattr(a,k+'max')) for k in 'xyz']
        return float(np.linalg.norm(gaps))
    best=float('inf');nearest=''
    for name,obs in sorted(SWEEP_OBSTACLES.items(),key=lambda item:bound(item[1])):
        if bound(obs)>best+1e-8:continue
        d=shape.distance(obs)
        if d<best:best=d;nearest=name
    return {'sensor_cg_aru_x_m':x,'sensor_cg_aru_z_m':-.02,'minimum_clearance_m':best*.001,
            'nearest_part':nearest,'free_of_collision':best>1e-3}

def stl(path,v,f):
    tri=v[f];norm=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);length=np.linalg.norm(norm,axis=1)
    norm/=np.maximum(length[:,None],1e-30)
    rec=np.zeros(len(f),dtype=[('n','<f4',3),('v','<f4',(3,3)),('attr','<u2')]);rec['n']=norm;rec['v']=tri
    path.write_bytes(b'H1 aircraft CG FRD, vertex unit metre'.ljust(80,b' ')+struct.pack('<I',len(f))+rec.tobytes())

def glb(path,parts):
    doc={'asset':{'version':'2.0','generator':'H1_reference source meshes / real CadQuery tessellation'},
         'scene':0,'scenes':[{'nodes':[]}],'nodes':[],'meshes':[],'materials':[],'accessors':[],'bufferViews':[],
         'extras':{'coordinates':'Y-up display: glTF(x,y,z)=(FRD x,-FRD z,FRD y); SI metres'}}
    blob=bytearray()
    def buffer(arr,kind,ctype):
        while len(blob)%4:blob.append(0)
        index=len(doc['bufferViews']);doc['bufferViews'].append({'buffer':0,'byteOffset':len(blob),'byteLength':arr.nbytes});blob.extend(arr.tobytes())
        a={'bufferView':index,'componentType':ctype,'count':len(arr),'type':kind}
        if kind=='VEC3':a.update(min=arr.min(0).tolist(),max=arr.max(0).tolist())
        doc['accessors'].append(a);return len(doc['accessors'])-1
    for name,v,f,color in parts:
        pos=np.asarray(v[:,[0,2,1]]*[1,-1,1],dtype='<f4');idx=np.asarray(f.ravel(),dtype='<u4')
        pi=buffer(pos,'VEC3',5126);fi=buffer(idx,'SCALAR',5125)
        mid=len(doc['materials']);doc['materials'].append({'name':name,'pbrMetallicRoughness':{'baseColorFactor':color,'metallicFactor':0,'roughnessFactor':.65},'doubleSided':True,**({'alphaMode':'BLEND'} if color[3]<1 else {})})
        mi=len(doc['meshes']);doc['meshes'].append({'name':name,'primitives':[{'attributes':{'POSITION':pi},'indices':fi,'material':mid}]})
        ni=len(doc['nodes']);doc['nodes'].append({'mesh':mi,'name':name});doc['scenes'][0]['nodes'].append(ni)
    while len(blob)%4:blob.append(0)
    doc['buffers']=[{'byteLength':len(blob)}]
    js=json.dumps(doc,separators=(',',':')).encode();js+=b' '*((-len(js))%4)
    total=12+8+len(js)+8+len(blob)
    path.write_bytes(struct.pack('<4sII',b'glTF',2,total)+struct.pack('<I4s',len(js),b'JSON')+js+struct.pack('<I4s',len(blob),b'BIN\0')+blob)

def wire(points):return cq.Wire.makePolygon([cq.Vector(*(np.array(p)*1000)) for p in points],close=True)
def box(lo,hi):
    lo=np.array(lo);hi=np.array(hi)
    return cq.Solid.makeBox(*((hi-lo)*1000),cq.Vector(*(lo*1000)))

def main():
    p=json.loads((HERE/'parameters.json').read_text(encoding='utf8'));cg=np.array(p['aircraft_cg_aru_m']);scg=np.array(p['sensor_geometric_cg_offset_frd_m'])
    out=HERE/'cad';out.mkdir(exist_ok=True);meshdir=HERE/'meshes';meshdir.mkdir(exist_ok=True)
    shapes={};colors={}
    def add(name,shape,color):shapes[name]=shape;colors[name]=color
    b=p['source_body_sections']
    def outline(x,inset=0):
        w=float(np.interp(x,b['halfwidth_x'],b['halfwidth']))-inset
        top=float(np.interp(x,b['x'],b['top']))-inset;shoulder=float(np.interp(x,b['x'],b['shoulder']))
        bot=float(np.interp(x,b['x'],b['bottom']))+inset;r=min(.016,w*.3,(shoulder-bot)*.3)
        yz=[(w*np.cos(a),shoulder+(top-shoulder)*np.sin(a)) for a in np.linspace(0,np.pi,25)]
        yz.append((-w,bot+r))
        yz.extend([(-w+r+r*np.cos(a),bot+r+r*np.sin(a)) for a in np.linspace(np.pi,1.5*np.pi,9)[1:]])
        yz.append((w-r,bot))
        yz.extend([(w-r+r*np.cos(a),bot+r+r*np.sin(a)) for a in np.linspace(-np.pi/2,0,9)[1:]])
        return wire([[x,y,z] for y,z in yz])
    outer=cq.Solid.makeLoft([outline(x) for x in b['x']],ruled=True)
    inner=cq.Solid.makeLoft([outline(x,.0018) for x in b['x']],ruled=True)
    shell=outer.cut(inner);tunnel=box(*p['tunnel_box_aru_m'])
    add('fuselage_H1_approx_with_rear_cutout',shell.cut(tunnel),[.18,.57,.60,.30])
    # CAD planform approximation; separately export exact original lifting meshes.
    foil=np.loadtxt(HERE/'geometry/h1_wing.dat',skiprows=1)
    tailfoil=np.loadtxt(HERE/'geometry/h1_tail.dat',skiprows=1)
    def wingwire(foil,x,y,z,chord,angle):
        a=np.deg2rad(angle);q=foil[:,0]*chord;n=foil[:,1]*chord
        return wire(np.c_[x+q*np.cos(a)+n*np.sin(a),np.full(len(q),y),z-q*np.sin(a)+n*np.cos(a)])
    w=p['wing_section'];wx,_,wz=w['leading_edge_m'];wc=w['chord_m']
    wholewing=cq.Solid.makeLoft([wingwire(foil,wx,y,wz,wc,2) for y in [-.9,.9]],ruled=True)
    # Separate ailerons with rectangular cutters; exact hinge geometry kept in source mesh exports.
    wing=wholewing
    for sign in [-1,1]:
        y1,y2=sorted([sign*.5316,sign*.8794]);cut=box([wx+.75*wc,y1,.05],[wx+wc+.02,y2,.18])
        add(f'aileron_{sign:+d}_CAD_approx',wholewing.intersect(cut),[.93,.43,.12,1]);wing=wing.cut(cut)
    add('main_wing_CAD_approx',wing,[.83,.85,.80,1])
    w=p['tail_section'];tx,_,tz=w['leading_edge_m'];tc=w['chord_m']
    tail=cq.Solid.makeLoft([wingwire(tailfoil,tx,y,tz,tc,0) for y in [-.35,.35]],ruled=True)
    for sign in [-1,1]:
        y1,y2=sorted([sign*.0365,sign*.3145]);cut=box([tx+.75*tc,y1,0],[tx+tc+.01,y2,.06])
        add(f'elevator_{sign:+d}_CAD_approx',tail.intersect(cut),[.93,.43,.12,1]);tail=tail.cut(cut)
    add('horizontal_tail_CAD_approx',tail,[.83,.85,.80,1])
    def finwire(x,z,chord):return wire([[x+q*chord,n*chord,z] for q,n in tailfoil])
    fin=cq.Solid.makeLoft([finwire(1.1525,.03,.27),finwire(1.1875,.33,.13)],ruled=True)
    ruddercut=cq.Solid.makeLoft([wire([[1.355,-.05,.029],[1.45,-.05,.029],[1.45,.05,.029],[1.355,.05,.029]]),
                                wire([[1.285,-.05,.331],[1.45,-.05,.331],[1.45,.05,.331],[1.285,.05,.331]])],ruled=True)
    add('vertical_fin_CAD_approx',fin.cut(ruddercut),[.18,.57,.60,1]);add('rudder_CAD_approx',fin.intersect(ruddercut),[.93,.43,.12,1])
    # New rear bay exactly matches the solver's zero-slope finite slabs.
    front,exit=.91,1.14;hw=.028;floor=-.048;ceil=.008;t=.002
    add('guide_floor',box([front,-hw,floor-t],[exit,hw,floor]),[.35,.38,.42,1])
    add('guide_ceiling',box([front,-hw,ceil],[exit,hw,ceil+t]),[.55,.6,.65,.35])
    for sign in [-1,1]:
        y1,y2=sorted([sign*hw,sign*(hw+t)])
        add(f'guide_side_{sign:+d}',box([front,y1,floor],[exit,y2,ceil]),[.55,.6,.65,.35])
    door=box([exit-t/2,-hw,floor],[exit+t/2,hw,ceil])
    hinge=(exit*1000,0,floor*1000)
    door_open=door.rotate(hinge,(exit*1000,1000,floor*1000),100)
    add('rear_door_100deg',door_open,[.98,.57,.18,1])
    # Sensor CG and geometric centre differ due to aft fins; convert local FRD to ARU.
    center=np.array(p['sensor_stowed_cg_aru_m']);T=np.diag([-1.,1.,-1.])
    def sensor_aru(local):return center+T@(np.array(local)-scg)
    nose=sensor_aru([.06,0,0])
    body=cq.Solid.makeCylinder(14,120,cq.Vector(*(nose*1000)),cq.Vector(1,0,0))
    add('sensor_body',body,[.98,.64,.1,1])
    sensor_names=['sensor_body']
    for k,(y,z) in enumerate([(.0185,0),(-.0185,0),(0,.0185),(0,-.0185)]):
        size=np.array([.03,.009,.002] if y else [.03,.002,.009]);c=sensor_aru([-.038,y,z]);name=f'sensor_fin_{k}'
        add(name,box(c-size/2,c+size/2),[.95,.31,.11,1]);sensor_names.append(name)
    # Contact tow marker corresponds to actual sensor nose (no new dynamics mass).
    add('sensor_tow_point',cq.Solid.makeSphere(1.5,cq.Vector(*(nose*1000))),[.8,.13,.12,1])
    # Capture lower pad and side pads outside the prescribed free corridor.
    add('capture_lower_pad',box([.975,-.028,-.055],[1.005,.028,-.050]),[.40,.23,.65,1])
    add('winch_drum',cq.Solid.makeCylinder(15,24,cq.Vector(900,-12,-20),cq.Vector(0,1,0)),[.2,.23,.27,1])
    add('aircraft_tow_point',cq.Solid.makeSphere(1.5,cq.Vector(900,0,-20)),[.8,.13,.12,1])
    # Exit frame occupies material around the solver's aperture, not its interior.
    frame=box([1.138,-.032,-.052],[1.142,.032,.012]).cut(box([1.137,-hw,floor],[1.143,hw,ceil]))
    add('rear_exit_frame',frame,[.2,.4,.7,1])
    # Transform every engineering output to aircraft CG FRD.
    transformed={n:s.rotate((0,0,0),(0,1,0),180).translate((cg[0]*1000,-cg[1]*1000,cg[2]*1000)) for n,s in shapes.items()}
    assembly=cq.Assembly(name='H1_A_temporary_reference_FRD_mm');meshparts=[];audit=[]
    for name,shape in transformed.items():
        if not shape.isValid() or shape.Volume()<=0:raise ValueError(f'Invalid CAD solid {name}')
        assembly.add(shape,name=name,color=cq.Color(*colors[name]))
        cq.exporters.export(shape,str(out/f'{name}.step'))
        verts,faces=shape.tessellate(.25,.15);v=np.array([x.toTuple() for x in verts])*.001;f=np.array(faces,dtype=int)
        stl(meshdir/f'{name}_FRD_m.stl',v,f);meshparts.append((name,v,f,colors[name]))
        audit.append({'part':name,'valid':shape.isValid(),'solids':len(shape.Solids()),'volume_mm3':shape.Volume(),
                      'bbox_frd_m':[v.min(0).tolist(),v.max(0).tolist()]})
    assembly.export(str(out/'H1_A_temporary_assembly.step'))
    glb(meshdir/'H1_A_temporary_assembly.glb',meshparts)
    # Full STEP roundtrip on lightweight real CAD assembly.
    imported=cq.importers.importStep(str(out/'H1_A_temporary_assembly.step')).val()
    validation={'cadquery_version':cq.__version__,'native_cad_export':True,'parts':audit,'roundtrip_valid':imported.isValid(),
                'roundtrip_solids':len(imported.Solids()),'export_solids':sum(r['solids'] for r in audit),
                'units':{'STEP':'mm, CG FRD','STL':'m, CG FRD','GLB':'m, x_forward, y_up, z_right'}}
    assert validation['roundtrip_valid'] and validation['roundtrip_solids']==validation['export_solids']
    # Exact source wing/tail/control meshes, separately inspectable from CAD approximations.
    sourceparts=read_glb(SOURCE/'GLB/DBF_A_H1.glb','DBF A | H1');exact=[]
    prefixes=('03_Main wing','Main wing outer','Wing tip','Aileron ','04_Tail','Tail tip','Horizontal fixed stabilizer','Elevator','05_Vertical','Rudder')
    selected=[]
    for name,(v,f) in sourceparts.items():
        if name.startswith(prefixes) and 'cover' not in name.lower():
            v=(v-cg)*[-1,1,-1];safe='source_'+''.join(c if c.isalnum() else '_' for c in name)
            stl(meshdir/f'{safe}_FRD_m.stl',v,f);exact.append((name,v,f,[.48,.65,.78,1]));selected.append(name)
    glb(meshdir/'H1_A_original_lifting_surfaces.glb',exact);validation['preserved_source_mesh_parts']=selected
    (HERE/'cad_validation.json').write_text(json.dumps(validation,indent=2),encoding='utf8')
    # Deterministic geometric sweep, not a claimed dynamics trajectory.
    sensor=cq.Compound.makeCompound([shapes[n] for n in sensor_names])
    checks=[]
    print('CAD roundtrip passed; checking 74 path stations on 6 workers',flush=True)
    with ProcessPoolExecutor(max_workers=6,initializer=clearance_init,initargs=(str(out),)) as pool:
        for result in pool.map(clearance_position,np.linspace(.99,1.72,74)):
            checks.append(result)
            if len(checks)%10==0:print(f'Clearance stations {len(checks)}/74',flush=True)
    # Door opening/closing sweep with sensor at stowed location.
    doors=[]
    for a in np.linspace(0,100,51):
        s=door.rotate(hinge,(exit*1000,1000,floor*1000),float(a))
        doors.append({'angle_deg':float(a),'sensor_clearance_m':sensor.distance(s)*.001})
    for filename,rows in [('clearance_path.csv',checks),('door_sweep.csv',doors)]:
        with (HERE/filename).open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    # Compare original closed H1 outline at the tail station against sensor envelope.
    cg_path=1.40;orig_bottom=float(np.interp(cg_path,b['x'],b['bottom']))
    clearance={'path_sweep_samples':len(checks),'minimum_CAD_clearance_m':min(x['minimum_clearance_m'] for x in checks),
       'all_swept_positions_collision_free':all(x['free_of_collision'] for x in checks),
       'door_minimum_sensor_clearance_m':min(x['sensor_clearance_m'] for x in doors),
       'analytic_fin_to_guide_m':.028-.023,'contact_sphere_envelope_clearance_m':.028-.023-.0015,
       'original_H1_bottom_z_at_1p4_m':orig_bottom,'sensor_path_center_z_m':-.02,
       'requires_original_aft_lower_shell_cutout':True,
       'scope':'Level straight prescribed 74-position CAD sweep; valid in reverse with open door. Not an integrated deployment/recovery proof. No cable-mesh collision or structural approval.'}
    (HERE/'clearance_summary.json').write_text(json.dumps(clearance,indent=2),encoding='utf8')
    # Technical visual with physical geometry; no image synthesis.
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(15,8));ax=fig.add_subplot(121,projection='3d');side=fig.add_subplot(222);top=fig.add_subplot(224)
    for name,v,f,c in meshparts:
        if name.startswith(('sensor_tow','aircraft_tow')):continue
        q=v*np.array([1,1,-1]);tri=q[f];stride=max(1,len(tri)//1600)
        ax.add_collection3d(Poly3DCollection(tri[::stride],facecolors=c,edgecolors='none',alpha=min(c[3],.90)))
    ax.set(xlim=(-1.05,.65),ylim=(-.95,.95),zlim=(-.22,.35),xlabel='Forward x / m',ylabel='Right y / m',zlabel='Up / m')
    ax.set_box_aspect((1.7,1.9,.57));ax.view_init(24,-55);ax.set_title('H1-A temporary CAD / CG origin')
    xx=np.linspace(.86,1.55,250);topz=np.interp(xx,b['x'],b['top']);bot=np.interp(xx,b['x'],b['bottom'])
    side.plot(xx,topz,color='#287778');side.plot(xx,bot,'--',color='#287778',label='Original H1 lower envelope')
    side.plot([.91,1.14],[-.048,-.048],color='#555555',lw=4);side.plot([.91,1.14],[.008,.008],color='#555555',lw=3)
    side.plot([.99,1.55],[-.02,-.02],color='#df8c17',label='Prescribed sensor CG path')
    side.axvline(1.14,color='#4575b4',label='Rear outlet')
    side.fill_between([.885,1.55],-.062,.022,alpha=.08,color='red',label='Required lower cutout corridor')
    side.set(xlim=(.86,1.55),ylim=(-.075,.085),xlabel='Original aft x / m',ylabel='Original up z / m',title='Rear bay and required lower-shell cutout');side.legend(fontsize=7)
    ww=np.interp(xx,b['halfwidth_x'],b['halfwidth']);top.plot(xx,ww,color='#287778');top.plot(xx,-ww,color='#287778')
    for y in [-.028,.028]:top.plot([.91,1.14],[y,y],color='#555555',lw=3)
    for y in [-.023,.023]:top.plot([.99,1.55],[y,y],'--',color='#df8c17')
    top.set(xlim=(.86,1.55),ylim=(-.09,.09),xlabel='Original aft x / m',ylabel='Right y / m',title='56 mm guide / 46 mm fin span')
    fig.suptitle('TEMPORARY ASSUMPTION MODEL | A wing 0.55 m2 | not a manufacturing release',fontsize=13)
    fig.tight_layout();fig.savefig(HERE/'overview.png',dpi=150);plt.close(fig)
    print(json.dumps({'parts':len(shapes),'roundtrip_valid':validation['roundtrip_valid'],'clearance':clearance},indent=2),flush=True)

if __name__=='__main__':main()
