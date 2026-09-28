"""Explain the three actual capture conditions on saved states."""
import numpy as np
from scipy.spatial.transform import Rotation


def capture_readiness(result):
    eligible=np.flatnonzero(result.time>=result.config['winch']['recovery_start_s'])
    if not len(eligible):return dict(status='recovery_not_reached',indices=[])
    y=result.states[eligible];b=result.config['bay']
    ra=Rotation.from_quat(y[:,[7,8,9,6]])
    rs=Rotation.from_quat(y[:,[20,21,22,19]])
    local=ra.inv().apply(y[:,13:16]-y[:,:3])
    distance=np.linalg.norm(local-np.array(b['stowed_center_m']),axis=1)
    relative=ra.inv().apply(y[:,16:19]-y[:,3:6])-np.cross(y[:,10:13],local)
    speed=np.linalg.norm(relative,axis=1)
    relative_rotation=ra.inv()*rs
    angle=np.rad2deg(relative_rotation.magnitude())
    ratio=np.maximum.reduce([distance/b['capture_radius_m'],speed/b['capture_speed_m_s'],angle/b['capture_angle_deg']])
    def record(index):
        row=int(eligible[index])
        return dict(index=row,time_s=float(result.time[row]),position_error_mm=float(distance[index]*1000),
            relative_speed_m_s=float(speed[index]),relative_angle_deg=float(angle[index]),
            relative_euler_xyz_deg=relative_rotation[index].as_euler('xyz',degrees=True).tolist(),
            largest_condition_ratio=float(ratio[index]))
    best=record(int(np.argmin(ratio)));closest=record(int(np.argmin(distance)))
    return dict(status='evaluated',indices=sorted({best['index'],closest['index']}),
        closest_combined_conditions=best,closest_position=closest,
        final_state=record(len(eligible)-1),
        limits=dict(position_error_mm=b['capture_radius_m']*1000,relative_speed_m_s=b['capture_speed_m_s'],relative_angle_deg=b['capture_angle_deg']),
        note='All three conditions must hold together. Saved-state diagnostic, not a replacement for the continuous capture event.')
