"""Physical guide meshes and explicit UI mechanism overrides."""
import numpy as np
from scipy.spatial.transform import Rotation


def defaults(c):
    from dbf_stability.door import door_hinge
    b=c['bay'];w=c['winch']
    return dict(hinge_m=door_hinge(b).tolist(),open_deg=max(v for _,v in w['door_schedule']),
                initial_deg=0.,open_time_s=.4,release_s=w['release_s'],close_time_s=.8,
                guide_points_m=c['cable'].get('guide_points_body_m',[]),guide_inner_radius_m=.005,guide_tube_radius_m=.001,
                capture_radius_m=b['capture_radius_m'],capture_speed_m_s=b['capture_speed_m_s'],
                capture_angle_deg=b['capture_angle_deg'],friction=b['friction'],
                latch_k_N_m=b['latch_k_N_m'],latch_c_Ns_m=b['latch_c_Ns_m'])


def apply(c,settings):
    from model_registry import vector
    m=settings['mechanism'];b=c['bay']
    hinge=np.asarray(vector(m['hinge_m'],'문 힌지'))
    b['door_hinge_offset_m']=(hinge-[b['exit_x_m'],0,b['floor_z_m']]).tolist()
    for key in ('capture_radius_m','capture_speed_m_s','capture_angle_deg','friction','latch_k_N_m','latch_c_Ns_m'):
        x=float(m[key])
        if not np.isfinite(x) or x<0 or (key!='friction' and x==0):raise ValueError(key+': 양의 물성을 입력하세요.')
        b[key]=x
    for key,lo,hi in [('open_deg',0,360),('initial_deg',0,360),('open_time_s',.001,60),
                      ('release_s',.001,120),('close_time_s',.001,60),('guide_inner_radius_m',.0001,.5),('guide_tube_radius_m',.0001,.1)]:
        x=float(m[key])
        if not np.isfinite(x) or not lo<=x<=hi:raise ValueError(key+': 범위를 확인하세요.')
        m[key]=x
    if m['release_s']<m['open_time_s']:raise ValueError('고정 해제 시각은 문이 열린 이후여야 합니다.')
    if m['open_deg']<m['initial_deg']:raise ValueError('열림 각도는 시작 각도 이상이어야 합니다.')
    points=m['guide_points_m']
    if not isinstance(points,list) or len(points)>16:raise ValueError('가이드 좌표는 최대 16개입니다.')
    c['cable']['guide_points_body_m']=[vector(p,'가이드') for p in points]
    if points and not c.get('collision',{}).get('enabled'):raise ValueError('가이드를 사용하려면 CAD 접촉을 켜세요.')
    if points and m['guide_inner_radius_m']<=c['cable']['diameter_m']/2+c['collision']['skin_m']:
        raise ValueError('가이드 안쪽 반경은 줄 반경과 접촉 간격보다 커야 합니다.')
    c['winch']['release_s']=m['release_s']
    return m


def ring_mesh(center,normal,inner,tube):
    normal=np.asarray(normal,float);normal/=np.linalg.norm(normal)
    reference=np.eye(3)[np.argmin(abs(normal))];u=np.cross(normal,reference);u/=np.linalg.norm(u);v=np.cross(normal,u)
    vertices=[];faces=[];major=inner+tube;n=48;k=12
    for i in range(n):
        radial=np.cos(2*np.pi*i/n)*u+np.sin(2*np.pi*i/n)*v
        for j in range(k):vertices.append(center+(major+tube*np.cos(2*np.pi*j/k))*radial+tube*np.sin(2*np.pi*j/k)*normal)
    for i in range(n):
        for j in range(k):
            a=i*k+j;b=((i+1)%n)*k+j;d=i*k+(j+1)%k;e=((i+1)%n)*k+(j+1)%k
            faces.extend([[a,b,e],[a,e,d]])
    return np.asarray(vertices),np.asarray(faces)


def guide_meshes(c,m):
    guides=c['cable'].get('guide_points_body_m',[])
    points=np.array([c['aircraft']['tow_point_m'],*guides,
                     np.asarray(c['bay']['stowed_center_m'])+Rotation.from_quat(np.roll(c['bay'].get('stowed_quaternion_wxyz',[1,0,0,0]),-1)).apply(c['sensor']['tow_point_m'])])
    for i,center in enumerate(guides,1):
        direction=points[i+1]-points[i-1]
        if np.linalg.norm(direction)<1e-8:raise ValueError('가이드 방향을 정할 수 없습니다. 왕복하거나 중복된 좌표를 확인하세요.')
        yield 'route_ring_'+str(i),ring_mesh(np.array(center),direction,m['guide_inner_radius_m'],m['guide_tube_radius_m'])


def write_stl(path,v,f):
    import struct
    with path.open('wb') as out:
        out.write(b'DBF registered geometry SI FRD'.ljust(80,b' '));out.write(struct.pack('<I',len(f)))
        for triangle in np.asarray(v)[f]:out.write(struct.pack('<12fH',*([0.]*3),*triangle.ravel(),0))
