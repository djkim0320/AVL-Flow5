"""Radau integration with bounded relative surface travel and clearance checks.

No velocity reset, positional projection, or interpolated replacement trajectory.
Accepted motion stays in the force model. A geometry violation stops the run.
"""
from types import SimpleNamespace
import numpy as np
from scipy.integrate import Radau, OdeSolution
from scipy.optimize import brentq
from .math3d import rotation, cross
from .door import door_rotation_radius
from .checkpoint import IntegrationStop

_BEZIER_INV=np.linalg.inv(np.array([[1,0,0,0],[8,12,6,1],[1,6,12,8],[0,0,0,1]],float)/np.array([1,27,27,1])[:,None])


def dense_surface_travel(model,dense,start,end,include_door=True):
    """Conservative bound on the actual cubic Radau interpolant, not three speeds.

    Bezier derivative curves lie in the convex hull of their derivative controls.
    Quaternion normalisation is bounded explicitly. This bounds every sensor
    surface point and every point on every cable span in aircraft coordinates.
    """
    dt=end-start
    control=_BEZIER_INV@dense(np.linspace(start,end,4)).T
    def speed(p):return np.linalg.norm(3*np.diff(p,axis=0)/dt,axis=1).max()
    def omega(q):
        minimum=np.linalg.norm(q[0])-np.linalg.norm(q-q[0],axis=1).max()
        return np.inf if minimum<=.5 else 2*speed(q)/minimum
    wa,ws=omega(control[:,6:10]),omega(control[:,19:23])
    relative=control[:,13:16]-control[:,:3]
    bound=speed(relative)+wa*np.linalg.norm(relative,axis=1).max()+(wa+ws)*model.c['sensor'].get('mesh_radius_m',model.c['sensor']['length_m'])
    k=model.active_count(model.length(start)[0]) if model.active_override is None else model.active_override
    for j in range(k):
        relative=control[:,26+6*j:29+6*j]-control[:,:3]
        bound=max(bound,speed(relative)+wa*np.linalg.norm(relative,axis=1).max())
    from .model import schedule
    door_speed=abs(np.deg2rad(model.door_state(start)[1]))*door_rotation_radius(model.c['bay']) if include_door else 0.
    return dt*(bound+door_speed)


def surface_speed(model, t, y,include_door=True):
    a,s=y[:13],y[13:26];ra=rotation(a[6:10])
    local=ra.T@(s[:3]-a[:3])
    wa=np.linalg.norm(a[10:13])
    speed=np.linalg.norm(s[3:6]-a[3:6])+wa*np.linalg.norm(local)+model.c['sensor'].get('mesh_radius_m',model.c['sensor']['length_m'])*(np.linalg.norm(s[10:13])+wa)
    k=model.active_count(model.length(t)[0]) if model.active_override is None else model.active_override
    if k:
        nodes=y[26:].reshape(model.n,6)[:k]
        relative_speed=np.linalg.norm(nodes[:,3:]-a[3:6],axis=1)+wa*np.linalg.norm(nodes[:,:3]-a[:3],axis=1)
        speed=max(speed,relative_speed.max())
    from .model import schedule
    if include_door:speed+=abs(np.deg2rad(model.door_state(t)[1]))*door_rotation_radius(model.c['bay'])
    return float(speed)


def separated_from_entire_door_sweep(model,t,y,travel):
    """Certify the entire moving mesh stays forward of every door angle.

    A cylinder about the hinge contains the complete door sweep. The proposed
    body/cable movement bound must still fit in the axial separation. If this
    sufficient condition fails, the original full door motion bound is used.
    """
    geometry=model._mesh_contacts
    if geometry is None:return False
    plates=[part for part in geometry.obstacles if part['door']]
    if not plates:return True
    radius=max(np.linalg.norm(plate['vertices'][:,[0,2]],axis=1).max() for plate in plates)
    door_maximum_x=geometry.hinge[0]+radius
    ra,rs=rotation(y[6:10]),rotation(y[19:23]);relative=ra.T@rs
    center=ra.T@(y[13:16]-y[:3])
    posed=[part['vertices']@relative.T+center for part in geometry.sensor]
    minimum=min(float(part[:,0].min()) for part in posed)
    k=model.active_count(model.length(t)[0]) if model.active_override is None else model.active_override
    nodes=(y[26:].reshape(model.n,6)[:k,:3]-y[:3])@ra
    chain=np.vstack([center+relative@model.sensor_attach,nodes,model.attach])
    minimum=min(minimum,float(chain[:,0].min()-model.c['cable']['diameter_m']/2))
    if minimum-travel>door_maximum_x:
        return True
    # A longer belly-folding leaf reaches forward of the stowed sensor's rear
    # x coordinate but still sweeps entirely below it. Distance from each
    # conservative x-z AABB to the hinge axis certifies radial separation too.
    # No CAD clearance query or force evaluation is omitted by this speed bound.
    def radial_bound(points, inflation=0.):
        lo=points[:,[0,2]].min(0)-inflation-geometry.hinge[[0,2]]
        hi=points[:,[0,2]].max(0)+inflation-geometry.hinge[[0,2]]
        return np.linalg.norm(np.maximum(np.maximum(lo,-hi),0.))
    if any(radial_bound(part)<=radius+travel for part in posed):
        return False
    cable_radius=model.c['cable']['diameter_m']/2
    return all(radial_bound(chain[i:i+2],cable_radius)>radius+travel for i in range(len(chain)-1))


def checked_radau(fun, interval, y, model, events, max_step, rtol, atol, jac, on_accepted=None):
    start,end=interval
    geometry=model.mesh_contact_enabled
    events=list(events)+([model.clearance_metric] if geometry else [])
    values=[event(start,y) for event in events]
    if geometry and values[-1] <= 0:
        raise ValueError('CONTACT_DOMAIN: initial geometry violates minimum clearance')
    times=[start];states=[y.copy()];interpolants=[]
    contact_sample_times=[]
    t_events=[[] for _ in events]
    travel=model.c.get('collision',{}).get('maximum_surface_travel_m',np.inf)
    checks=0;minimum_gap=model.c.get('collision',{}).get('skin_m',np.inf)
    success=True;message='';retries=0;failure_status=None
    method=model.c.get('simulation',{}).get('jacobian_method','grouped')
    if method not in ('grouped','adaptive'):
        raise ValueError('Unknown Jacobian method: '+str(method))
    options={}
    if method=='adaptive':
        # SciPy selects/retries perturbations against roundoff; its batched
        # derivative calls can use independent processes without changing RHS.
        original_fun=fun
        def vectorized_fun(t,states):
            if states.shape[1]==1:
                return original_fun(t,states[:,0])[:,None]
            return model.rhs_batch(t,states)
        fun=vectorized_fun;jac=None
        options=dict(vectorized=True,jac_sparsity=model.jac_sparsity())
    try:
        solver=Radau(fun,start,y,end,max_step=max_step,rtol=rtol,atol=atol,jac=jac,**options)
    except IntegrationStop as exc:
        solver=None;success=False;message=str(exc);failure_status=exc.status
    while solver is not None and solver.status=='running':
        before=solver.t;state=solver.y.copy()
        permitted=np.inf;gap=np.inf
        include_door=True
        if geometry:
            gap=values[-1]+model.c['collision']['minimum_gap_m']
            cap=travel if gap<=model.c['collision']['skin_m'] else model.c['collision'].get('maximum_free_surface_travel_m',travel)
            permitted=min(cap,.4*gap)
            include_door=not separated_from_entire_door_sweep(model,before,state,permitted)
            speed=surface_speed(model,before,state,include_door=include_door)
            accel=max(np.linalg.norm(solver.f[16:19]-solver.f[3:6]),
                      np.linalg.norm(solver.f[26:].reshape(model.n,6)[:,3:]-solver.f[3:6],axis=1).max())
        # Leave room for curvature/acceleration. A proposal exactly on the
        # bound repeatedly rejected otherwise, forcing expensive Radau restarts.
            solver.max_step=min(max_step,.8*permitted/max(speed,1e-8),np.sqrt(2*permitted/max(accel,1e-8)))
        try:
            reason=solver.step()
        except (IntegrationStop,ValueError) as exc:
            success=False;message=str(exc)
            failure_status=getattr(exc,'status','aero_domain_exceeded' if 'bounds' in str(exc) or 'AERO_DOMAIN' in str(exc) else 'numerical_failure')
            break
        if solver.status=='failed':
            success=False;message=reason;break
        dense=solver.dense_output();dt=solver.t-before
        bound=dense_surface_travel(model,dense,before,solver.t,include_door=include_door) if geometry else 0.
        # Unexpected acceleration cannot turn one accepted step into a thin-wall
        # jump: repeat from its untouched left state with a smaller step.
        if bound>permitted:
            retries+=1
            if retries>20:
                success=False;message='CONTACT_DOMAIN: could not bound surface travel';break
            try:
                solver=Radau(fun,before,state,end,max_step=min(max_step,dt/3),first_step=dt/3,rtol=rtol,atol=atol,jac=jac,**options)
            except IntegrationStop as exc:
                success=False;message=str(exc);failure_status=exc.status;break
            continue
        retries=0
        first=None
        old_t=before;old_values=values
        contact_step=geometry and gap<=model.c['collision']['skin_m']
        # Check the cubic dense curve as well as each accepted endpoint. Total
        # surface travel is bounded by 40% of the preceding actual clearance.
        for tt in [(before+solver.t)/2,solver.t]:
            zz=dense(tt)
            current=[event(tt,zz) for event in events]
            if geometry:
                contact_step=contact_step or current[-1]+model.c['collision']['minimum_gap_m']<=model.c['collision']['skin_m']
                checks+=1
                minimum_gap=min(minimum_gap,current[-1]+model.c['collision']['minimum_gap_m'])
            roots=[]
            for j,(left,right) in enumerate(zip(old_values,current)):
                if left>0 and right<=0:
                    root=brentq(lambda v:events[j](v,dense(v)),old_t,tt,xtol=1e-12)
                    roots.append((root,j))
            if roots:
                first=min(roots);break
            old_t,old_values=tt,current
        stop=first[0] if first else solver.t
        if contact_step:
            contact_sample_times.extend([before,(before+stop)/2,stop])
        interpolants.append(dense);times.append(stop);states.append(dense(stop));values=old_values
        if on_accepted:on_accepted(stop,states[-1],dense,before)
        if first:
            t_events[first[1]].append(stop)
            break
    if not interpolants:
        # Match solve_ivp failure semantics; callers will not request dense output.
        solution=lambda t:np.repeat(y[:,None],np.asarray(t).size,axis=1)
    else:
        solution=OdeSolution(np.asarray(times),interpolants)
    return SimpleNamespace(t=np.asarray(times),y=np.asarray(states).T,sol=solution,
        t_events=[np.asarray(v) for v in t_events],success=success,message=message,
        failure_status=failure_status,
        clearance_checks=checks,minimum_checked_gap_m=minimum_gap,
        contact_sample_times=np.asarray(contact_sample_times))
