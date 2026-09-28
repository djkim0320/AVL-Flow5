"""SI; inertial NED; body FRD; scalar-first body-to-inertial quaternions."""
import numpy as np


def cross(a,b):
    """Low-overhead three-vector product; batch inputs use NumPy broadcasting."""
    if np.ndim(a)>1 or np.ndim(b)>1:
        return np.cross(a,b)
    return np.array([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]])


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(q):
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def qdot(q, w):
    s, v = q[0], q[1:]
    return .5 * np.r_[-np.dot(v, w), s*np.asarray(w)+cross(v, w)]


def quaternion(roll=0., pitch=0., yaw=0.):
    cr, cp, cy = np.cos(np.array([roll, pitch, yaw])/2)
    sr, sp, sy = np.sin(np.array([roll, pitch, yaw])/2)
    return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def euler(q):
    r = rotation(q)
    return np.array([np.arctan2(r[2, 1], r[2, 2]),
                     np.arcsin(np.clip(-r[2, 0], -1, 1)), np.arctan2(r[1, 0], r[0, 0])])


def attitude_error(target, actual):
    """Rotation vector taking actual attitude to target, in actual body axes."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(rotation(actual).T @ rotation(target)).as_rotvec()


def point_state(body, local):
    r = rotation(body[6:10])
    return body[:3]+r@local, body[3:6]+r@cross(body[10:13], local)


def spring_force(delta, relative_velocity, rest, ea, damping, tension_only=True, regularization=1e-6, damping_strain=0.):
    """Force on first end toward second end; damping never creates compressive cable load."""
    length = np.linalg.norm(delta)
    if length < 1e-12:
        return np.zeros(3), 0.
    direction = delta / length
    extension = length-rest
    if tension_only and extension <= 0:
        return np.zeros(3), 0.
    engagement = min(1., max(0., extension)/(max(rest, regularization)*damping_strain)) if tension_only and damping_strain>0 else 1.
    tension = ea/max(rest, regularization)*extension+engagement*damping*np.dot(relative_velocity, direction)
    if tension_only:
        tension = max(0., tension)
    return tension*direction, tension


def contact_force(penetration, normal_velocity, tangent_velocity, stiffness, damping, friction):
    """Repulsive linear penalty contact with dissipative regularized Coulomb friction."""
    if penetration <= 0:
        return 0., np.zeros(3)
    normal = max(0., stiffness*penetration-damping*normal_velocity)
    tang = np.asarray(tangent_velocity)
    return normal, -friction*normal*tang/np.sqrt(np.dot(tang, tang)+1e-6)


def sphere_box_contact(point, center, axes, half_extents, radius):
    """Finite two-sided box contact; avoids infinite half-space loads behind a door."""
    axes=np.asarray(axes);half=np.asarray(half_extents)
    local=axes.T@(np.asarray(point)-center)
    closest=np.clip(local,-half,half);delta=local-closest
    distance=np.linalg.norm(delta)
    if distance>1e-12:
        return axes@(delta/distance), radius-distance
    clearance=half-abs(local);idx=int(np.argmin(clearance));normal=np.zeros(3)
    normal[idx]=1 if local[idx]>=0 else -1
    return axes@normal, radius+clearance[idx]
