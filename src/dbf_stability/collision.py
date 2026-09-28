"""CAD mesh contact queries for the coupled solver (Bullet, DIRECT connection).

Bullet supplies geometry only. Radau still integrates the existing aircraft,
sensor and material cable states; no second dynamics solver or hidden controller.
Every cable span is a capsule, not just a set of colliding end points.
"""
from pathlib import Path
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from .door import door_hinge, door_reference_shift, door_box

BOX_SIGNS=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
BOX_EDGES=np.array([(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(BOX_SIGNS[i]!=BOX_SIGNS[j])==1])


def box_box_manifold(ca,aa,ha,cb,ab,hb,skin):
    """Exact box feature distances with continuous load sharing at parallel edges."""
    va=(BOX_SIGNS*ha)@aa.T+ca;vb=(BOX_SIGNS*hb)@ab.T+cb
    axes=np.vstack([aa.T,ab.T,np.cross(aa.T[:,None,:],ab.T[None,:,:]).reshape(-1,3)])
    axes=axes[np.linalg.norm(axes,axis=1)>1e-10];axes/=np.linalg.norm(axes,axis=1)[:,None]
    projection=axes@(ca-cb)
    sep=abs(projection)-abs(axes@aa)@ha-abs(axes@ab)@hb
    if np.max(sep)>=skin:return []
    if np.max(sep)<=0:
        k=int(np.argmax(sep));normal=axes[k]*(1 if projection[k]>=0 else -1)
        return [(ca,cb,normal,float(sep[k]),1.)]
    pa=[va, np.clip((vb-ca)@aa,-ha,ha)@aa.T+ca]
    pb=[np.clip((va-cb)@ab,-hb,hb)@ab.T+cb,vb]
    factors=[np.ones(8),np.ones(8)]
    a0=va[BOX_EDGES[:,0]][:,None,:];b0=vb[BOX_EDGES[:,0]][None,:,:]
    u=(va[BOX_EDGES[:,1]]-va[BOX_EDGES[:,0]])[:,None,:]
    v=(vb[BOX_EDGES[:,1]]-vb[BOX_EDGES[:,0]])[None,:,:]
    w=a0-b0
    a=(u*u).sum(2);b=(u*v).sum(2);c=(v*v).sum(2);d=(u*w).sum(2);e=(v*w).sum(2)
    den=np.maximum(a*c-b*b,0);relative=den/(a*c)
    t=np.divide(b*e-c*d,den,out=np.full_like(den,.5),where=relative>1e-14)
    s=np.divide(a*e-b*d,den,out=np.clip(e/c,0,1),where=relative>1e-14)
    candidates=[(np.clip(t,0,1),np.clip(s,0,1),relative/(relative+1e-8)),
                (np.zeros_like(b),np.clip(e/c,0,1),np.ones_like(b)),
                (np.ones_like(b),np.clip((e+b)/c,0,1),np.ones_like(b)),
                (np.clip(-d/a,0,1),np.zeros_like(b),np.ones_like(b)),
                (np.clip((b-d)/a,0,1),np.ones_like(b),np.ones_like(b))]
    for t,s,factor in candidates:
        pa.append((a0+t[:,:,None]*u).reshape(-1,3));pb.append((b0+s[:,:,None]*v).reshape(-1,3));factors.append(factor.ravel())
    pa,pb=np.vstack(pa),np.vstack(pb);factors=np.concatenate(factors)
    delta=pa-pb;gap=np.linalg.norm(delta,axis=1);minimum=float(gap.min())
    if minimum>=skin:return []
    weights=factors*np.exp(-np.clip((gap-minimum)/(skin*.2),0,700));weights/=weights.sum()
    mask=(gap<skin)&(weights>1e-10)
    # Merge coincident witnesses only after weighting the fixed feature set.
    rows={}
    for x,y,g,w in zip(pa[mask],pb[mask],gap[mask],weights[mask]):
        key=tuple(np.round(np.r_[x,y],11))
        if key in rows:rows[key][-1]+=w
        else:rows[key]=[x,y,(x-y)/max(g,1e-15),float(g),float(w)]
    return [(*row,minimum) for row in rows.values()]


def read_binary_stl(path):
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    record = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    if len(data) != 84 + count * record.itemsize:
        raise ValueError(f'Expected binary STL in metres: {path}')
    triangles = np.frombuffer(data, record, count=count, offset=84)['vertices'].astype(float)
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return vertices, inverse.reshape(-1, 3)


def polyhedron_edges(vertices,faces):
    """Exclude triangulation diagonals within a planar polyhedron face."""
    normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    adjacent={}
    for i,face in enumerate(faces):
        for a,b in zip(face,np.roll(face,-1)):
            adjacent.setdefault(tuple(sorted((int(a),int(b)))),[]).append(i)
    return np.array([edge for edge,indices in adjacent.items()
        if len(indices)!=2 or abs(normals[indices[0]]@normals[indices[1]])<.999999],int)


def mesh_box_manifold(va,faces,edges,cb,ab,hb,skin):
    """Complete convex-polyhedron features, with continuous load sharing.

    Cylinder mesh contacts must not inherit a single jumping GJK witness.
    Vertex/face and edge/edge pairs include contacts inside a long body edge.
    """
    vb=(BOX_SIGNS*hb)@ab.T+cb
    pa=[va];pb=[np.clip((va-cb)@ab,-hb,hb)@ab.T+cb];factors=[np.ones(len(va))]
    triangles=va[faces];p=vb[:,None,:];origin=triangles[None,:,0,:]
    u=triangles[:,1]-triangles[:,0];v=triangles[:,2]-triangles[:,0]
    normal=np.cross(u,v);normal/=np.linalg.norm(normal,axis=1)[:,None]
    projected=p-((p-origin)*normal).sum(2)[:,:,None]*normal
    delta=projected-origin;uu=(u*u).sum(1);uv=(u*v).sum(1);vv=(v*v).sum(1)
    du=(delta*u).sum(2);dv=(delta*v).sum(2);den=uu*vv-uv*uv
    s=(vv*du-uv*dv)/den;t=(uu*dv-uv*du)/den
    points=[projected];distances=[np.where((s>=0)&(t>=0)&(s+t<=1),((p-projected)**2).sum(2),np.inf)]
    for j in range(3):
        start=triangles[None,:,j,:];direction=triangles[None,:,(j+1)%3,:]-start
        fraction=np.clip(((p-start)*direction).sum(2)/(direction*direction).sum(2),0,1)
        q=start+fraction[:,:,None]*direction
        points.append(q);distances.append(((p-q)**2).sum(2))
    distances=np.stack(distances);points=np.stack(points)
    choose=np.argmin(distances,axis=0)
    closest=points[choose,np.arange(8)[:,None],np.arange(len(faces))[None,:]]
    choose_triangle=np.argmin(((closest-p)**2).sum(2),axis=1)
    pa.append(closest[np.arange(8),choose_triangle]);pb.append(vb);factors.append(np.ones(8))
    a0=va[edges[:,0]][:,None,:];b0=vb[BOX_EDGES[:,0]][None,:,:]
    u=(va[edges[:,1]]-va[edges[:,0]])[:,None,:];v=(vb[BOX_EDGES[:,1]]-vb[BOX_EDGES[:,0]])[None,:,:]
    w=a0-b0;a=(u*u).sum(2);b=(u*v).sum(2);c=(v*v).sum(2);d=(u*w).sum(2);e=(v*w).sum(2)
    den=np.maximum(a*c-b*b,0);relative=den/(a*c)
    t=np.divide(b*e-c*d,den,out=np.full_like(den,.5),where=relative>1e-14)
    s=np.divide(a*e-b*d,den,out=np.clip(e/c,0,1),where=relative>1e-14)
    candidates=[(np.clip(t,0,1),np.clip(s,0,1),relative/(relative+1e-8)),
        (np.zeros_like(b),np.clip(e/c,0,1),np.ones_like(b)),(np.ones_like(b),np.clip((e+b)/c,0,1),np.ones_like(b)),
        (np.clip(-d/a,0,1),np.zeros_like(b),np.ones_like(b)),(np.clip((b-d)/a,0,1),np.ones_like(b),np.ones_like(b))]
    for t,s,factor in candidates:
        pa.append((a0+t[:,:,None]*u).reshape(-1,3));pb.append((b0+s[:,:,None]*v).reshape(-1,3));factors.append(factor.ravel())
    pa,pb=np.vstack(pa),np.vstack(pb);factors=np.concatenate(factors)
    delta=pa-pb;gaps=np.linalg.norm(delta,axis=1);minimum=float(gaps.min())
    if minimum>=skin:return []
    weights=factors*np.exp(-np.clip((gaps-minimum)/(skin*.2),0,700));weights/=weights.sum()
    rows={}
    mask=(gaps<skin)&(weights>1e-10)
    for x,y,g,weight in zip(pa[mask],pb[mask],gaps[mask],weights[mask]):
        key=tuple(np.round(np.r_[x,y],11))
        if key in rows:rows[key][-1]+=weight
        else:rows[key]=[x,y,(x-y)/max(g,1e-15),float(g),float(weight)]
    return list(rows.values())


def closest_on_triangles(points, vertices, faces):
    """Closest point on a closed triangulated surface, for each query vertex."""
    triangles=vertices[faces];p=points[:,None,:];origin=triangles[None,:,0,:]
    u=triangles[:,1]-triangles[:,0];v=triangles[:,2]-triangles[:,0]
    normal=np.cross(u,v);normal/=np.linalg.norm(normal,axis=1)[:,None]
    projected=p-((p-origin)*normal).sum(2)[:,:,None]*normal
    delta=projected-origin;uu=(u*u).sum(1);uv=(u*v).sum(1);vv=(v*v).sum(1)
    du=(delta*u).sum(2);dv=(delta*v).sum(2);den=uu*vv-uv*uv
    s=(vv*du-uv*dv)/den;t=(uu*dv-uv*du)/den
    candidates=[projected]
    distances=[np.where((s>=0)&(t>=0)&(s+t<=1),((p-projected)**2).sum(2),np.inf)]
    for j in range(3):
        start=triangles[None,:,j,:];direction=triangles[None,:,(j+1)%3,:]-start
        fraction=np.clip(((p-start)*direction).sum(2)/(direction*direction).sum(2),0,1)
        q=start+fraction[:,:,None]*direction
        candidates.append(q);distances.append(((p-q)**2).sum(2))
    choose=np.argmin(np.stack(distances),axis=0)
    closest=np.stack(candidates)[choose,np.arange(len(points))[:,None],np.arange(len(faces))[None,:]]
    choose_triangle=np.argmin(((closest-p)**2).sum(2),axis=1)
    return closest[np.arange(len(points)),choose_triangle]


def mesh_mesh_manifold(va,fa,ea,vb,fb,eb,skin):
    """Smooth feature load sharing for separated polyhedral surfaces.

    GJK remains the signed-gap test. Complete vertex/face and edge/edge
    witnesses replace its discontinuous single contact point on guide ramps
    or a local patch of the nonconvex fuselage shell. Intersection checks are
    still performed on the complete original solids by the caller.
    """
    pa=[va,closest_on_triangles(vb,va,fa)]
    pb=[closest_on_triangles(va,vb,fb),vb]
    factors=[np.ones(len(va)),np.ones(len(vb))]
    a0=va[ea[:,0]][:,None,:];b0=vb[eb[:,0]][None,:,:]
    u=(va[ea[:,1]]-va[ea[:,0]])[:,None,:];v=(vb[eb[:,1]]-vb[eb[:,0]])[None,:,:]
    w=a0-b0;a=(u*u).sum(2);b=(u*v).sum(2);c=(v*v).sum(2);d=(u*w).sum(2);e=(v*w).sum(2)
    den=np.maximum(a*c-b*b,0);relative=den/(a*c)
    t=np.divide(b*e-c*d,den,out=np.full_like(den,.5),where=relative>1e-14)
    s=np.divide(a*e-b*d,den,out=np.clip(e/c,0,1),where=relative>1e-14)
    candidates=[(np.clip(t,0,1),np.clip(s,0,1),relative/(relative+1e-8)),
        (np.zeros_like(b),np.clip(e/c,0,1),np.ones_like(b)),(np.ones_like(b),np.clip((e+b)/c,0,1),np.ones_like(b)),
        (np.clip(-d/a,0,1),np.zeros_like(b),np.ones_like(b)),(np.clip((b-d)/a,0,1),np.ones_like(b),np.ones_like(b))]
    for t,s,factor in candidates:
        pa.append((a0+t[:,:,None]*u).reshape(-1,3));pb.append((b0+s[:,:,None]*v).reshape(-1,3));factors.append(factor.ravel())
    pa,pb=np.vstack(pa),np.vstack(pb);factors=np.concatenate(factors)
    delta=pa-pb;gaps=np.linalg.norm(delta,axis=1);minimum=float(gaps.min())
    if minimum>=skin:return []
    weights=factors*np.exp(-np.clip((gaps-minimum)/(skin*.2),0,700));weights/=weights.sum()
    rows={};mask=(gaps<skin)&(weights>1e-10)
    for x,y,g,weight in zip(pa[mask],pb[mask],gaps[mask],weights[mask]):
        key=tuple(np.round(np.r_[x,y],11))
        if key in rows:rows[key][-1]+=weight
        else:rows[key]=[x,y,(x-y)/max(g,1e-15),float(g),float(weight)]
    return list(rows.values())


def barrier_force(gap, normal_speed, tangent_velocity, skin, stiffness, damping, friction):
    """Repulsive barrier within a declared numerical contact skin.

    The barrier tends to infinity at the surface. Penetrating trial states are
    finite for Newton iteration, but accepted states are separately rejected.
    Damping/friction are dissipative and vanish at the skin boundary.
    This enforces near-rigid separation; it is NOT a calibrated impact material.
    """
    if gap >= skin:
        return 0., np.zeros(3)
    compression = skin - gap
    engagement = min(1., compression / skin)
    normal = max(0., stiffness * compression * skin / max(gap, skin * .02)
                 - damping * engagement * normal_speed)
    tang = np.asarray(tangent_velocity)
    return normal, -friction * normal * tang / np.sqrt(np.dot(tang, tang) + 1e-6)


def capsule_box(a,b,radius,center,axes,half,regularization=0.):
    """Exact closest segment/AABB distance, including interior span crossings.

    Squared distance is piecewise quadratic in the segment parameter. Minimise
    each interval between slab crossings; no point sampling or GJK tolerance.
    """
    origin=(a-center)@axes;delta=(b-a)@axes
    crossings=[]
    for i in range(3):
        if abs(delta[i])>1e-15:
            crossings.extend([(-half[i]-origin[i])/delta[i],(half[i]-origin[i])/delta[i]])
    edges=np.unique(np.r_[0.,[v for v in crossings if 0<v<1],1.])
    mids=(edges[:-1]+edges[1:])/2
    probes=origin+mids[:,None]*delta
    active=abs(probes)>half
    slopes=active*delta
    offsets=active*(origin-np.sign(probes)*half)
    regularizer=regularization**2
    denom=(slopes*slopes).sum(1)+regularizer
    stationary=np.divide(-(offsets*slopes).sum(1)+.5*regularizer,denom,out=mids.copy(),where=denom>1e-25)
    fractions=np.r_[edges,np.clip(stationary,edges[:-1],edges[1:])]
    points=origin+fractions[:,None]*delta
    closest=np.clip(points,-half,half);differences=points-closest
    squares=(differences*differences).sum(1)
    objective=squares+regularizer*(fractions-.5)**2;minimum=objective.min()
    tied=np.flatnonzero(objective<=minimum+1e-22)
    idx=tied[np.argmin(abs(fractions[tied]-.5))]
    point=points[idx];distance=np.sqrt(squares[idx])
    if distance>1e-12:
        normal=axes@(differences[idx]/distance);gap=np.sqrt(objective[idx])-radius
    else:
        clear=half-abs(point);axis=int(np.argmin(clear));direction=np.zeros(3)
        direction[axis]=1 if point[axis]>=0 else -1
        normal=axes@direction;gap=-radius-clear[axis]
    return center+axes@point,normal,float(gap)


def capsule_mesh(a,b,radius,vertices,faces,regularization,skin=None):
    """Exact regularised segment/triangle feature minimum on a surface patch.

    The small axial regulariser chooses a continuous cable coordinate along
    parallel surfaces, just as capsule_box does. Triangle faces and every edge
    are included; the original full mesh still supplies the signed-gap guard.
    """
    triangles=vertices[faces];u=b-a;uu=float(u@u);eps2=regularization**2
    origin=triangles[:,0];e1=triangles[:,1]-origin;e2=triangles[:,2]-origin
    normals=np.cross(e1,e2);normals/=np.linalg.norm(normals,axis=1)[:,None]
    d0=((a-origin)*normals).sum(1);dn=normals@u
    projection=a-d0[:,None]*normals;projected_u=u-dn[:,None]*normals
    w=projection-origin
    aa=(e1*e1).sum(1);bb=(e1*e2).sum(1);cc=(e2*e2).sum(1);det=aa*cc-bb*bb
    def bary(delta):
        de=(delta*e1).sum(1);df=(delta*e2).sum(1)
        return (cc*de-bb*df)/det,(aa*df-bb*de)/det
    v0,w0=bary(w);dv,dw=bary(projected_u)
    # Projected segment lies inside a triangle over an interval of t.
    low=np.zeros(len(faces));high=np.ones(len(faces));valid=np.ones(len(faces),bool)
    for base,slope in ((v0,dv),(w0,dw),(1-v0-w0,-dv-dw)):
        moving=abs(slope)>1e-15
        crossing=np.divide(-base,slope,out=np.zeros_like(base),where=moving)
        low=np.where(slope>1e-15,np.maximum(low,crossing),low)
        high=np.where(slope< -1e-15,np.minimum(high,crossing),high)
        valid&=moving|(base>=-1e-12)
    valid&=low<=high
    tf=np.clip((.5*eps2-d0*dn)/(dn*dn+eps2),low,high)
    cp=a+tf[:,None]*u;op=cp-(d0+tf*dn)[:,None]*normals
    points=[cp[valid]];others=[op[valid]];fractions=[tf[valid]]
    owners=[np.flatnonzero(valid)]
    # Box-constrained closest coordinates for each segment/triangle edge pair.
    edge0=triangles.reshape(-1,3)
    edge1=np.roll(triangles,-1,axis=1).reshape(-1,3)
    v=edge1-edge0;w=a-edge0
    vv=(v*v).sum(1);uv=v@u;uw=w@u;vw=(v*w).sum(1)
    den=(uu+eps2)*vv-uv*uv
    t=(uv*vw-vv*(uw-.5*eps2))/den
    s=(uv*t+vw)/vv
    cases=[(t,s,(t>=0)&(t<=1)&(s>=0)&(s<=1)),
        (np.zeros_like(t),np.clip(vw/vv,0,1),np.ones_like(t,bool)),
        (np.ones_like(t),np.clip((vw+uv)/vv,0,1),np.ones_like(t,bool)),
        (np.clip((.5*eps2-uw)/(uu+eps2),0,1),np.zeros_like(t),np.ones_like(t,bool)),
        (np.clip((.5*eps2-uw+uv)/(uu+eps2),0,1),np.ones_like(t),np.ones_like(t,bool))]
    for t,s,mask in cases:
        points.append(a+t[mask,None]*u);others.append(edge0[mask]+s[mask,None]*v[mask]);fractions.append(t[mask])
        owners.append(np.repeat(np.arange(len(faces)),3)[mask])
    points=np.vstack(points);others=np.vstack(others);fractions=np.concatenate(fractions)
    delta=points-others;distance2=(delta*delta).sum(1)
    objective=distance2+eps2*(fractions-.5)**2
    if skin is not None:
        # A nonconvex inner wall has simultaneous nearest faces. Selecting just
        # argmin makes its normal jump at every polygonal bore facet bisector.
        # The strictly convex segment/triangle problem has one continuous
        # witness per triangle. Compact weights vanish at the search boundary,
        # so adding/removing a broad-phase candidate cannot jump the force.
        owners=np.concatenate(owners)
        order=np.lexsort((objective,owners))
        first=np.r_[True,np.diff(owners[order])!=0]
        indices=order[first]
        regularized=np.sqrt(objective[indices]);gaps=regularized-radius
        inside=gaps<skin
        indices=indices[inside];gaps=gaps[inside];regularized=regularized[inside]
        if not len(indices):return []
        distances=np.sqrt(distance2[indices])
        weights=(skin-gaps)**2;weights/=weights.sum()
        return [(points[i],delta[i]/max(d,1e-15),float(g),float(d/max(r,1e-15)),float(w))
                for i,d,g,r,w in zip(indices,distances,gaps,regularized,weights)]
    idx=int(np.argmin(objective))
    distance=np.sqrt(distance2[idx]);regularized=np.sqrt(objective[idx])
    normal=delta[idx]/max(distance,1e-15)
    return points[idx],normal,float(regularized-radius),float(distance/max(regularized,1e-15))


class MeshContacts:
    def __init__(self, cfg):
        manifest=cfg['collision'].get('parts_manifest')
        if not isinstance(manifest,dict) or not manifest:
            raise ValueError('접촉 계산에는 부품 역할을 지정한 collision.parts_manifest가 필요합니다.')
        import pybullet as p
        self.p = p
        self.client = p.connect(p.DIRECT)
        self.cfg = cfg
        c = cfg['collision']
        self.skin = c['skin_m']
        engine=c.get('feature_engine','numpy')
        if engine in ('numba','numba_bounded'):
            from . import contact_features
            self.mesh_box_features=contact_features.mesh_box_manifold
            self.mesh_mesh_features=contact_features.mesh_mesh_manifold
            if engine=='numba_bounded':
                from .contact_bounded import bounded_features
                self.mesh_mesh_features=bounded_features
        elif engine=='numpy':
            self.mesh_box_features=mesh_box_manifold
            self.mesh_mesh_features=mesh_mesh_manifold
        else:
            self.close()
            raise ValueError(f'Unknown contact feature engine: {engine}')
        self.cable_regularization = c['cable_contact_regularization_m']
        root = Path(cfg['_root']) / c['mesh_directory']
        self.obstacles, self.sensor = [], []
        self.capsules = {}
        self.hashes = {}
        self.hinge = door_hinge(cfg['bay'])
        stowed = np.asarray(cfg['bay']['stowed_center_m'])
        seen=set()
        for path in sorted(root.glob('*_FRD_m.stl')):
            name = path.name.removesuffix('_FRD_m.stl')
            if name not in manifest:continue
            seen.add(name)
            self.hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
            v, f = read_binary_stl(path)
            description=manifest[name]
            if description.get('role') not in ('sensor','door','fixed'):raise ValueError('접촉 부품 역할을 확인하세요: '+name)
            is_sensor = description['role']=='sensor'
            is_door = description['role']=='door'
            if is_sensor:
                if description.get('coordinates')=='aircraft_stowed':v -= stowed
            elif is_door:
                v += (door_reference_shift(cfg['bay']) if description.get('bay_reference_shift') else np.zeros(3))-self.hinge
            if description.get('decomposition')=='bay_aperture':
                # Exact four-box decomposition of the delivered rectangular
                # aperture ring. Its empty opening is never convex-filled.
                lo,hi=v.min(0),v.max(0);bay=cfg['bay'];hw=bay['half_width_m']
                bars=[('floor',[lo[0],lo[1],bay['floor_z_m']],hi),
                      ('ceiling',lo,[hi[0],hi[1],bay['ceiling_z_m']]),
                      ('left',[lo[0],lo[1],bay['ceiling_z_m']],[hi[0],-hw,bay['floor_z_m']]),
                      ('right',[lo[0],hw,bay['ceiling_z_m']],[hi[0],hi[1],bay['floor_z_m']])]
                for suffix,low,high in bars:
                    low,high=np.asarray(low),np.asarray(high)
                    vertices=(BOX_SIGNS*(high-low)+(high+low))/2
                    shape=p.createCollisionShape(p.GEOM_MESH,vertices=vertices.tolist(),physicsClientId=self.client)
                    body=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,physicsClientId=self.client)
                    p.changeDynamics(body,-1,collisionMargin=0.,physicsClientId=self.client)
                    self.obstacles.append(dict(name=name+'_'+suffix,body=body,vertices=vertices,faces=np.empty((0,3),int),
                        concave=False,lo=low,hi=high,door=False,spool=False,box_kind='aabb'))
                continue
            # Only the shell and the aperture ring are concave. A convex hull of
            # either would fill the internal void and block the intended passage.
            concave = bool(description.get('concave'))
            kwargs = dict(shapeType=p.GEOM_MESH, vertices=v.tolist(), physicsClientId=self.client)
            if concave:
                kwargs.update(indices=f.ravel().tolist(), flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
            shape = p.createCollisionShape(**kwargs)
            if shape < 0:
                raise ValueError(f'Cannot create collision shape: {path}')
            body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, physicsClientId=self.client)
            p.changeDynamics(body, -1, collisionMargin=0., physicsClientId=self.client)
            info = dict(name=name, body=body, vertices=v, faces=f, concave=concave,lo=v.min(0), hi=v.max(0), door=is_door)
            info['reference_deg']=float(description['reference_deg']) if is_door else 0.
            info['spool']=bool(description.get('spool'))
            info['box_kind']=description.get('box_kind')
            info['feature_edges']=bool(description.get('edges'))
            # The box fast path is exact only for mesh-aligned cuboids. Rotated
            # fins must use their actual polyhedron, not their enlarged AABB.
            info['axis_aligned_box'] = len(v) == 8 and all(len(np.unique(v[:,j])) == 2 for j in range(3))
            info['box_geometry'] = None
            if is_sensor and len(v) == 8:
                center=v.mean(axis=0)
                if info['axis_aligned_box']:
                    axes=np.eye(3)
                else:
                    _,axes=np.linalg.eigh((v-center).T@(v-center))
                    if np.linalg.det(axes)<0:axes[:,0]*=-1
                local=(v-center)@axes;half=np.max(abs(local),axis=0)
                if np.max(abs(abs(local)-half))<5e-8:
                    info['box_geometry']=(center,axes,half)
            if concave:
                info['triangle_lo']=v[f].min(1)
                info['triangle_hi']=v[f].max(1)
            if is_sensor or description.get('edges'):info['edges']=polyhedron_edges(v,f)
            (self.sensor if is_sensor else self.obstacles).append(info)
        if not self.sensor or not self.obstacles or set(manifest)!=seen:
            self.close()
            raise ValueError('Collision meshes must include every registered sensor and fixed part')
        unknown=set(c.get('part_materials',{}))-{part['name'] for part in self.obstacles}
        if unknown:
            self.close()
            raise ValueError(f'Contact materials refer to unknown fixed parts: {sorted(unknown)}')
        analytic=c.get('analytic_conical_stop')
        if analytic:
            if set(analytic.get('cad_sha256',{}))!={'sensor_body.step',analytic['part']+'.step'}:
                self.close();raise ValueError('Analytic conical stop requires both CAD reference hashes')
            for name,expected in analytic['cad_sha256'].items():
                if hashlib.sha256((root.parent/'cad'/name).read_bytes()).hexdigest()!=expected:
                    self.close();raise ValueError('Analytic conical stop CAD reference changed')

    def close(self):
        if getattr(self, 'client', -1) >= 0:
            self.p.disconnect(physicsClientId=self.client)
            self.client = -1

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def contacts(self, center, relative_rotation, chain, door_deg, distance=None):
        """Aircraft-frame closest points/normals; signed gap is positive outside."""
        p, client = self.p, self.client
        if len(self.capsules)>2048:
            # Bullet 3.2.7's removeCollisionShape leaves query shapes allocated.
            # A geometry-only client has no dynamic state; rebuilding this cache
            # bounds memory without changing any mechanical state or geometry.
            p.resetSimulation(physicsClientId=client)
            self.capsules.clear()
            for part in self.obstacles+self.sensor:
                args=dict(shapeType=p.GEOM_MESH,vertices=part['vertices'].tolist(),physicsClientId=client)
                if part['concave']:args.update(indices=part['faces'].ravel().tolist(),flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
                shape=p.createCollisionShape(**args)
                part['body']=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=shape,physicsClientId=client)
                p.changeDynamics(part['body'],-1,collisionMargin=0.,physicsClientId=client)
        margin = self.skin if distance is None else distance
        # GJK's reported distance is approximate. Using the force cutoff itself
        # as its search radius can drop a real mesh feature just inside the skin
        # and make its force jump on/off under nanometre pose changes. Search a
        # wider band, then let the actual feature distances and barrier cutoff
        # determine the force. Clearance queries keep their original radius.
        query_margin = margin + self.skin if distance is None else margin
        for obs in self.obstacles:
            if obs['door']:
                q = Rotation.from_euler('y', door_deg - obs['reference_deg'], degrees=True)
                p.resetBasePositionAndOrientation(obs['body'], self.hinge, q.as_quat(), physicsClientId=client)
                vertices = q.apply(obs['vertices']) + self.hinge
                obs['lo'], obs['hi'] = vertices.min(0), vertices.max(0)
                obs['query_vertices']=vertices
                if obs['concave']:
                    obs['triangle_lo']=vertices[obs['faces']].min(1)
                    obs['triangle_hi']=vertices[obs['faces']].max(1)
        lows, highs = np.array([o['lo'] for o in self.obstacles]), np.array([o['hi'] for o in self.obstacles])

        def candidates(lo, hi):
            return np.flatnonzero(np.all(hi + margin >= lows, axis=1) & np.all(lo - margin <= highs, axis=1))

        hits = []
        q = Rotation.from_matrix(relative_rotation).as_quat()
        for part in self.sensor:
            v = part['vertices'] @ relative_rotation.T + center
            selected = candidates(v.min(0), v.max(0))
            if not len(selected):
                continue
            p.resetBasePositionAndOrientation(part['body'], center, q, physicsClientId=client)
            for j in selected:
                obs = self.obstacles[j]
                box=self._box_geometry(obs,door_deg)
                if part['box_geometry'] is not None and box is not None:
                    local_center,local_axes,ha=part['box_geometry']
                    ca=center+relative_rotation@local_center
                    for row in box_box_manifold(ca,relative_rotation@local_axes,ha,*box,margin):
                        x,other,normal,gap,weight=row[:5]
                        hits.append(dict(kind='sensor',span=-1,moving=part['name'],fixed=obs['name'],point=x,
                           obstacle_point=other,normal=normal,gap=gap,geometry_gap=row[5] if len(row)>5 else gap,weight=weight,door=obs['door']))
                    continue
                analytic=self.cfg['collision'].get('analytic_conical_stop')
                if distance is None and analytic and part['name']=='sensor_body' and obs['name']==analytic['part']:
                    from .analytic_contact import sphere_cone_contacts
                    sphere=center+relative_rotation@np.asarray(analytic['nose_center_sensor_m'])
                    # The entire sphere is behind the end plane; all remaining
                    # body points are behind its supporting hemispherical cap.
                    # The finite-cone support checks then certify this force
                    # manifold without enumerating tessellation witnesses.
                    # Every separate clearance query still uses the full mesh.
                    if sphere[0]+analytic['nose_radius_m']<analytic['front_x_m']:
                        manifold=sphere_cone_contacts(center,relative_rotation,analytic,self.skin)
                        if manifold is not None and manifold['minimum_gap']>0:
                            for row in manifold['contacts']:
                                hits.append(dict(**row,kind='sensor',span=-1,moving=part['name'],fixed=obs['name'],
                                    geometry_gap=manifold['minimum_gap'],door=False,geometry_method='sphere_cone_support_planes'))
                            continue
                points = p.getClosestPoints(part['body'], obs['body'], query_margin, physicsClientId=client)
                if (analytic and part['name']=='sensor_body'
                        and obs['name']==analytic['part'] and points and min(q[8] for q in points)>0):
                    from .analytic_contact import sphere_cone_contacts
                    manifold=sphere_cone_contacts(center,relative_rotation,analytic,self.skin)
                    if distance is not None and manifold is not None:
                        # Keep all original mesh distances, and add the exact
                        # curved-surface gap where its finite domain is valid.
                        hits.append(dict(kind='sensor',span=-1,moving=part['name'],fixed=obs['name'],
                            point=manifold['point'],obstacle_point=manifold['obstacle_point'],normal=manifold['normal'],
                            gap=manifold['minimum_gap'],geometry_gap=manifold['minimum_gap'],door=False,
                            geometry_method='exact_sphere_cone_clearance'))
                    # The support and finite-cone/bore domain checks establish
                    # that the spherical cap is the active surface. Individual
                    # tessellation-edge normals are not analytic CAD normals.
                    if distance is None:
                        if manifold is not None:
                            geometric_gap=float(min(q[8] for q in points))
                            for row in manifold['contacts']:
                                hits.append(dict(**row,kind='sensor',span=-1,moving=part['name'],fixed=obs['name'],
                                    geometry_gap=min(geometric_gap,manifold['minimum_gap']),door=False,geometry_method='sphere_cone_support_planes'))
                            continue
                if distance is None and obs['concave'] and points and min(q[8] for q in points)>0:
                    # A shell lip also has parallel edge witnesses. Use all real
                    # nearby triangle features once, not a jumping deepest GJK
                    # witness or one duplicate force per tessellation normal.
                    select=np.all(obs['triangle_hi']>=v.min(0)-margin,axis=1)&np.all(obs['triangle_lo']<=v.max(0)+margin,axis=1)
                    faces=obs['faces'][select]
                    if len(faces):
                        indices,inverse=np.unique(faces,return_inverse=True)
                        vertices=obs.get('query_vertices',obs['vertices'])[indices];faces=inverse.reshape(-1,3)
                        edges=polyhedron_edges(vertices,faces)
                        geometric_gap=float(min(q[8] for q in points))
                        for x,other,normal,gap,weight in self.mesh_mesh_features(v,part['faces'],part['edges'],vertices,faces,edges,margin):
                            hits.append(dict(kind='sensor',span=-1,moving=part['name'],fixed=obs['name'],point=x,
                                obstacle_point=other,normal=normal,gap=gap,geometry_gap=geometric_gap,weight=weight,door=obs['door']))
                        continue
                for hit in self._reduce(points, 'sensor', -1, part['name'], obs, nearest_only=distance is not None):
                    if distance is None and hit['gap']>0 and obs.get('feature_edges',False):
                        for x,other,normal,gap,weight in self.mesh_mesh_features(v,part['faces'],part['edges'],obs['vertices'],obs['faces'],obs['edges'],margin):
                            hits.append({**hit,'point':x,'obstacle_point':other,'normal':normal,'gap':gap,
                                'geometry_gap':hit['gap'],'weight':weight})
                    elif distance is not None or box is None or hit['gap']<=0:
                        hits.append(hit)
                    else:
                        for x,other,normal,gap,weight in self.mesh_box_features(v,part['faces'],part['edges'],*box,margin):
                            hits.append({**hit,'point':x,'obstacle_point':other,'normal':normal,'gap':gap,
                                'geometry_gap':hit['gap'],'weight':weight})
        radius = self.cfg['cable']['diameter_m'] / 2
        for i, (a, b) in enumerate(zip(chain[:-1], chain[1:])):
            selected = [j for j in candidates(np.minimum(a, b)-radius, np.maximum(a, b)+radius)
                        if not (self.obstacles[j]['spool']
                                and self.cfg['winch'].get('line_attaches_to_drum_center', True))]
            if not selected:
                continue
            meshes=[]
            for j in selected:
                obs=self.obstacles[j];box=self._box_geometry(obs,door_deg)
                if box is None:
                    meshes.append(j);continue
                center_box,axes_box,half_box=box
                point,normal,gap=capsule_box(a,b,radius,center_box,axes_box,half_box)
                if gap<=margin:
                    hit=dict(kind='cable',span=i,moving=f'cable_span_{i}',fixed=obs['name'],point=point,normal=normal,gap=gap,door=obs['door'])
                    hits.extend(self._cable_manifold(hit,a,b,obs,door_deg,radius))
            selected=meshes
            if not selected:continue
            delta = b-a
            length = np.linalg.norm(delta)
            if length < 1e-10:
                q = [0., 0., 0., 1.]
            else:
                direction = delta / length
                q = np.r_[-direction[1], direction[0], 0., 1+direction[2]]
                q = q / np.linalg.norm(q) if direction[2] > -.999999999 else np.array([1., 0., 0., 0.])
            # Exact current length, with no quantisation or end-point-only tests.
            key=float(length)
            if key not in self.capsules:
                self.capsules[key] = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=length,
                                                           physicsClientId=client)
            shape=self.capsules[key]
            for j in selected:
                obs = self.obstacles[j]
                points = p.getClosestPoints(-1, obs['body'], query_margin, collisionShapeA=shape,
                            collisionShapePositionA=(a+b)/2, collisionShapeOrientationA=q,
                            physicsClientId=client)
                if (distance is None and points and min(q[8] for q in points)>0
                        and (obs['concave'] or obs.get('feature_edges',False) or obs['spool'])):
                    lo=np.minimum(a,b)-radius-margin;hi=np.maximum(a,b)+radius+margin
                    if 'triangle_lo' not in obs:
                        triangles=obs['vertices'][obs['faces']]
                        obs['triangle_lo']=triangles.min(1);obs['triangle_hi']=triangles.max(1)
                    select=np.all(obs['triangle_hi']>=lo,axis=1)&np.all(obs['triangle_lo']<=hi,axis=1)
                    faces=obs['faces'][select]
                    if len(faces):
                        manifold=capsule_mesh(a,b,radius,obs.get('query_vertices',obs['vertices']),faces,self.cable_regularization,skin=self.skin)
                        for point,normal,gap,gradient,share in manifold:
                            hits.append(dict(kind='cable',span=i,moving=f'cable_span_{i}',fixed=obs['name'],point=point,
                                normal=normal,gap=gap,geometry_gap=float(min(q[8] for q in points)),weight=gradient*share,
                                normal_velocity_scale=gradient,door=obs['door']))
                        continue
                for hit in self._reduce(points, 'cable', i, f'cable_span_{i}', obs, nearest_only=distance is not None):
                    hits.extend(self._cable_manifold(hit,a,b,obs,door_deg,radius))
        return hits

    def _box_geometry(self,obs,door_deg):
        kind=obs.get('box_kind')
        if kind not in ('aabb','bay_door'):return None
        if kind=='bay_door':
            center,axes,half=door_box(self.cfg['bay'],door_deg)
        else:
            axes=np.eye(3);center=(obs['lo']+obs['hi'])/2;half=(obs['hi']-obs['lo'])/2
        return center,axes,half

    def _box_face(self,hit,obs,door_deg):
        box=self._box_geometry(obs,door_deg)
        if box is None:return None
        center,axes,half=box
        components=axes.T@hit['normal'];axis=int(np.argmax(abs(components)))
        if abs(components[axis])<.999:
            # Supporting plane at an edge/corner. It can also have a whole
            # contact line (e.g. a fin across the door hinge), not one vertex.
            normal=hit['normal'];plane=hit.get('obstacle_point',hit['point']-hit['gap']*normal)
        else:
            normal=axes[:,axis]*np.sign(components[axis]);plane=center+normal*half[axis]
        return normal,plane,center,axes,half

    def _cable_manifold(self,hit,a,b,obs,door_deg,radius):
        box=self._box_geometry(obs,door_deg)
        if box is None:return [hit]
        center,axes,half=box
        # A unique contact coordinate replaces the discontinuous switch from
        # an interior span contact to a just-entering endpoint. The regularised
        # distance is sqrt(distance_to_box^2 + eps^2*(fraction-.5)^2).
        # Its gradient gives the same endpoint shape functions, scaled below.
        # The exact unregularised gap remains the separation guard.
        point,normal,gap=capsule_box(a,b,radius,center,axes,half,self.cable_regularization)
        local=(point-center)@axes
        distance=np.linalg.norm(local-np.clip(local,-half,half))
        weight=distance/max(gap+radius,1e-15) if gap+radius>0 else 1.
        return [{**hit,'point':point,'normal':normal,'gap':gap,'geometry_gap':hit['gap'],
                 'weight':weight,'normal_velocity_scale':weight}]

    def _sensor_manifold(self, hit, vertices, obs, door_deg):
        """Smooth load sharing on planar CAD faces; full-solid gap is retained.

        A single GJK witness jumps between equally close corners on parallel
        faces. Such a jump must not relocate the entire normal force/torque.
        The supporting face's vertex forces smoothly share the load instead.
        """
        face=self._box_face(hit,obs,door_deg)
        if face is None:return [hit]
        normal,plane,center,axes,half=face
        gaps=(vertices-plane)@normal
        projected=vertices-gaps[:,None]*normal
        within=np.all(abs((projected-center)@axes)<=half+1e-7,axis=1)
        if not np.any(within):
            return [hit]  # Edge of the sensor crossing the interior of a plate.
        v=vertices[within];g=gaps[within]
        weights=np.exp(-np.clip((g-g.min())/(self.skin*.2),0,700));weights/=weights.sum()
        manifold=[{**hit,'point':point,'normal':normal,'gap':float(gap),'geometry_gap':hit['gap'],'weight':float(weight)}
                  for point,gap,weight in zip(v,g,weights) if gap<self.skin and weight>1e-12]
        return manifold or [hit]

    @staticmethod
    def _reduce(points, kind, span, moving, obs, nearest_only=False):
        # Coplanar tessellation triangles must not multiply the penalty stiffness.
        # Keep one deepest point for each normal direction on each part pair.
        # Clearance queries consume only the minimum signed gap. Normal
        # clustering of hundreds of nearby shell triangles cannot change it.
        # Keep every relevant normal for FORCE queries; this fast path changes
        # neither the geometry minimum nor the equations of motion.
        if nearest_only and points:
            points=[min(points,key=lambda x:x[8])]
        groups = []
        for point in sorted(points, key=lambda x: x[8]):
            normal = np.asarray(point[7])
            if any(np.dot(normal, h['normal']) > .999 for h in groups):
                continue
            groups.append(dict(kind=kind, span=span, moving=moving, fixed=obs['name'],
                               point=np.asarray(point[5]), obstacle_point=np.asarray(point[6]), normal=normal, gap=float(point[8]),
                               door=obs['door']))
        return groups
