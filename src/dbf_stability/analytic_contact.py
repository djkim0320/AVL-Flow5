"""Finite spherical-cap / conical-seat support-plane force quadrature.

All CAD mesh clearance queries remain active.
"""
import numpy as np

def sphere_cone_contacts(center,relative_rotation,spec,skin):
    """Return None outside the analytic domain, otherwise all active witnesses."""
    radius=spec['nose_radius_m'];sphere=center+relative_rotation@np.asarray(spec['nose_center_sensor_m'])
    back,front=spec['back_x_m'],spec['front_x_m'];mouth,bore=spec['mouth_radius_m'],spec['bore_radius_m']
    slope=(mouth-bore)/(front-back);scale=np.sqrt(1+slope*slope)
    radial=sphere[1:]-np.asarray(spec['center_yz_m']);angle=np.arctan2(radial[1],radial[0])
    angles=angle+np.arange(spec['quadrature_order'])*2*np.pi/spec['quadrature_order']
    normals=np.column_stack([np.full(len(angles),-slope),-np.cos(angles),-np.sin(angles)])/scale
    # The sphere must be the support surface for every conical tangent plane.
    # Otherwise part of the cylinder/rear face may be closer.
    axis=relative_rotation[:,0]
    if slope*axis[0]-np.linalg.norm(axis[1:])<=0:return None
    # A nose protruding through the end plane must fit inside the bore.
    # Otherwise its finite circular edge requires the complete mesh model.
    if sphere[0]+radius>front:
        if sphere[0]>=front:return None
        section=np.sqrt(max(0.,radius*radius-(front-sphere[0])**2))
        if np.linalg.norm(radial)+section>=bore-2*skin:return None
    gaps=(mouth-slope*(sphere[0]-back)-radial[0]*np.cos(angles)-radial[1]*np.sin(angles))/scale-radius
    points=sphere-radius*normals
    others=sphere-(radius+gaps[:,None])*normals
    near=gaps<2*skin
    if not np.any(near):return None
    if np.any(others[near,0]<back) or np.any(others[near,0]>front):return None
    minimum=float(gaps.min())
    weights=np.exp(-np.clip((gaps-minimum)/(.2*skin),0.,700.));weights/=weights.sum()
    rows=[dict(point=p,obstacle_point=q,normal=n,gap=float(g),weight=float(w),analytic_gap=minimum)
            for p,q,n,g,w in zip(points,others,normals,gaps,weights) if g<skin and w>1e-12]
    nearest=int(np.argmin(gaps))
    return dict(contacts=rows,minimum_gap=minimum,point=points[nearest],
        obstacle_point=others[nearest],normal=normals[nearest])

