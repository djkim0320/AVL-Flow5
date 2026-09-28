"""Read only the copied H1 GLB scenes; export auditable geometry measurements."""
from pathlib import Path
import json, struct, hashlib
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[2] / '모델링' / 'H1_비행기_모델'

def read_glb(path, scene_name):
    data = path.read_bytes()
    assert data[:4] == b'glTF'
    size = struct.unpack_from('<I', data, 12)[0]
    doc = json.loads(data[20:20+size])
    offset = 20+size
    binary = data[offset+8:offset+8+struct.unpack_from('<I', data, offset)[0]]
    def accessor(i):
        a = doc['accessors'][i]; v = doc['bufferViews'][a['bufferView']]
        dtype = {5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']]
        cols = {'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4}[a['type']]
        off = v.get('byteOffset',0)+a.get('byteOffset',0)
        item = np.dtype(dtype).itemsize
        return np.ndarray((a['count'],cols),dtype=dtype,buffer=binary,offset=off,
                          strides=(v.get('byteStride',cols*item),item)).copy()
    parts = {}
    def visit(i, parent):
        node = doc['nodes'][i]
        if 'matrix' in node:
            local = np.array(node['matrix']).reshape(4,4).T
        else:
            local = np.eye(4)
            local[:3,:3] = Rotation.from_quat(node.get('rotation',[0,0,0,1])).as_matrix() @ np.diag(node.get('scale',[1,1,1]))
            local[:3,3] = node.get('translation',[0,0,0])
        world = parent@local
        if 'mesh' in node:
            vertices, faces = [], []
            for p in doc['meshes'][node['mesh']]['primitives']:
                assert p.get('mode',4)==4
                xyz = accessor(p['attributes']['POSITION'])
                v = xyz@world[:3,:3].T+world[:3,3]
                # glTF Y-up -> original Blender aft/right/up.
                v = v[:,[0,2,1]]*np.array([1,-1,1])
                f = accessor(p['indices']).reshape(-1,3)
                faces.append(f+sum(len(x) for x in vertices)); vertices.append(v)
            parts[node['name']] = (np.vstack(vertices), np.vstack(faces))
        for c in node.get('children',[]): visit(c,world)
    scene = doc['scenes'][doc.get('scene',0)] if scene_name is None else next(s for s in doc['scenes'] if s.get('name')==scene_name)
    for i in scene['nodes']: visit(i,np.eye(4))
    return parts

def section_profile(vertices, span_axis, incidence_deg=0):
    v = vertices[np.isclose(vertices[:,span_axis],vertices[:,span_axis].max(),atol=2e-6)]
    normal_axis = 2 if span_axis==1 else 1
    a = np.deg2rad(incidence_deg); c,s=np.cos(a),np.sin(a)
    q = v[:,0]*c-v[:,normal_axis]*s
    n = v[:,0]*s+v[:,normal_axis]*c
    i = np.argmin(q); q0,n0=q[i],n[i]; chord=q.max()-q0
    xy=np.unique(np.round(np.c_[(q-q0)/chord,(n-n0)/chord],9),axis=0)
    # Traverse the simple closed section. Do not split at y=0: cambered upper
    # and lower skins can both lie on one side of that line near the trailing edge.
    ang = np.arctan2(xy[:,1]-float(np.mean(xy[:,1])),xy[:,0]-.5)
    xy = xy[np.argsort(ang)]
    xy = np.roll(xy,-int(np.argmax(xy[:,0])),axis=0)
    le=np.zeros(3);le[0]=q0*c+n0*s;le[normal_axis]=-q0*s+n0*c;le[span_axis]=v[0,span_axis]
    return {'chord_m':float(chord),'leading_edge_m':le.tolist(),'incidence_deg':incidence_deg},xy

def main():
    out=HERE/'source_audit';out.mkdir(parents=True,exist_ok=True)
    report={'source_directory':str(SOURCE),'coordinate_system':'original aft/right/up, metres','variants':{}}
    for variant in ['A','B']:
        path=SOURCE/'GLB'/f'DBF_{variant}_H1.glb'
        parts=read_glb(path,f'DBF {variant} | H1')
        name=next(k for k in parts if k.startswith('03_Main wing centre'))
        sec,foil=section_profile(parts[name][0],1,2)
        tail=next(k for k in parts if k.startswith('04_Tail centre'))
        tailsec,tailfoil=section_profile(parts[tail][0],1)
        bounds={k:[v.min(0).tolist(),v.max(0).tolist()] for k,(v,f) in parts.items()}
        report['variants'][variant]={'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'scene':f'DBF {variant} | H1','parts':len(parts),'wing_section':sec,'tail_section':tailsec,'bounds_m':bounds}
        np.savetxt(out/f'{variant}_wing_section.dat',foil,header=f'H1 {variant} extracted wing section',comments='',fmt='%.9f')
        np.savetxt(out/f'{variant}_tail_section.dat',tailfoil,header=f'H1 {variant} extracted tail section',comments='',fmt='%.9f')
        print(variant,sec,tailsec,flush=True)
        for k,(v,f) in parts.items():
            if k.startswith(('05_Vertical','Rudder')):
                print(k, 'root/tip', flush=True)
                for z in [v[:,2].min(),v[:,2].max()]:
                    vs=v[np.isclose(v[:,2],z,atol=2e-6)]
                    print(z,vs.min(0),vs.max(0),flush=True)
    (out/'geometry_measurements.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    files=['DBF_He51_Inspired.blend','body_sections.json','STEP/source_geometry.json']
    (out/'source_hashes.json').write_text(json.dumps({f:hashlib.sha256((SOURCE/f).read_bytes()).hexdigest() for f in files},indent=2),encoding='utf-8')

if __name__=='__main__':main()
